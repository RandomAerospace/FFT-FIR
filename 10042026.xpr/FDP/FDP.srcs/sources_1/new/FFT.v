`timescale 1ns / 1ps


//Interace modules


//this module acts as an interface between the FIR and FFT modules
//50ksps->decimate by 2-> 25ksps, BW=(25/2)=12.5kHz-> +/-6.25kHz
module visual_fft_bridge(
    input clk,          // 100MHz
    input reset,        // Asserted HIGH for SPIRAL
    input signed [31:0] fir_data, 
    input fir_valid,    // 50kHz
    
    output reg next,    // Trigger to SPIRAL
    output [11:0] X0, X1, X2, X3 // To SPIRAL
);

    //Decimation & Collection Buffer
    reg [1:0] deci_cnt; //FYI we need to decimate by a factor of 2
    reg [9:0] write_ptr;
    reg signed [11:0] buffer [0:1023];
    
    //Extract the saturated 12-bit portion from the 32-bit FIR output
    wire signed [11:0] fft_input_scaled = fir_data[26:15];

    //HANN WINDOWING LOGIC
    //ROM to store 1024 points of the Hann window (Q15 format: 0 to 32767)
    reg [15:0] hann_rom [0:1023];
    initial begin
        $readmemh("1024_Hann_Window.mem", hann_rom);
    end

    //The write_ptr (0 to 1023) matches the position in the 1024-point window
    wire [15:0] current_coeff = hann_rom[write_ptr];
    
    //Multiply audio by window coefficient.
    //We append 1'b0 to current_coeff to force it to be treated as a positive signed number
    wire signed [27:0] mult_result = fft_input_scaled * $signed({1'b0, current_coeff});
    
    //Shift right by 15 to compensate for Q15 multiplication
    wire signed [11:0] windowed_sample = mult_result[26:15];
    // ----------------------------

    //urst Control State Machine (so that we collect the 1024 samples, then burst into SPIRAL FFT core
    reg [1:0] state; // 0: Collect, 1: Pulse Next, 2: Bursting
    reg [8:0] read_ptr; // 0 to 511 (for 512 pairs)

    always @(posedge clk) begin
        if (reset) begin
            state <= 0; write_ptr <= 0; deci_cnt <= 0; next <= 0;
        end else begin
            case (state)
                0: begin //COLLECT SAMPLES
                    next <= 0;
                    if (fir_valid) begin
                        if (deci_cnt == 2'b10) begin 
                            deci_cnt <= 0;
                            
                            //Write the WINDOWED sample to the buffer, not the raw one
                            buffer[write_ptr] <= windowed_sample; 
                            
                            if (write_ptr == 1023) begin
                                write_ptr <= 0;
                                state <= 1; // Buffer full, start FFT
                            end else begin
                                write_ptr <= write_ptr + 1;
                            end
                        end else begin
                            deci_cnt <= deci_cnt + 1;
                        end
                    end
                end

                1: begin //PULSE NEXT
                    next <= 1;
                    read_ptr <= 0;
                    state <= 2;
                end

                2: begin //BURST DATA (100MHz)
                    next <= 0;
                    if (read_ptr == 511) state <= 0;
                    else read_ptr <= read_ptr + 1;
                end
            endcase
        end
    end

    //Interleave from Buffer
    //In state 2, we read two consecutive addresses every clock cycle
    assign X0 = (state == 2) ? buffer[{read_ptr, 1'b0}] : 12'h000; // Real N
    assign X1 = 12'h000;                                          // Imag N
    assign X2 = (state == 2) ? buffer[{read_ptr, 1'b1}] : 12'h000; // Real N+1
    assign X3 = 12'h000;                                          // Imag N+1

endmodule


//display driver
module fft_processor (
    input clk,
    input reset,
    input next_out,                     
    input signed [11:0] Y0, Y1, Y2, Y3, 
    
    // Scroll Control
    input [8:0] scroll_offset, // 0 to 416 (512 - 96)
    
    input [6:0] oled_x,       
    output [5:0] db_height    
);

    //Magnitude Approximation (Alpha Max + Beta Min) 
    function [11:0] get_mag;
        input signed [11:0] re;
        input signed [11:0] im;
        reg [11:0] abs_re, abs_im;
        begin
            abs_re = (re[11]) ? -re : re;
            abs_im = (im[11]) ? -im : im;
            if (abs_re > abs_im) 
                get_mag = abs_re + (abs_im >> 1);
            else 
                get_mag = abs_im + (abs_re >> 1);
        end
    endfunction

    //Capture Control (512 bins = 256 cycles) 
    reg [7:0] write_ptr = 0; 
    reg processing = 0;
    reg data_active = 0;

    always @(posedge clk) begin
        if (reset) begin
            write_ptr <= 0;
            processing <= 0;
            data_active <= 0;
        end else begin
            // SPIRAL: Data starts cycle AFTER next_out
            data_active <= next_out; 

            if (data_active) begin
                processing <= 1;
                write_ptr <= 0;
            end else if (write_ptr == 255 && processing) begin 
                //Capture 256 pairs (512 bins total)
                processing <= 0;
            end else if (processing) begin
                write_ptr <= write_ptr + 1;
            end
        end
    end

    //Spectrum Storage (Bins 0-511) ---
    //Two RAMs to handle the interleaved 2-words-per-cycle output
    reg [11:0] ram_even [0:255];
    reg [11:0] ram_odd  [0:255];

    always @(posedge clk) begin
        if (processing) begin
            ram_even[write_ptr] <= get_mag(Y0, Y1); // Bins 0, 2, 4...
            ram_odd[write_ptr]  <= get_mag(Y2, Y3); // Bins 1, 3, 5...
        end
    end


    //CROLLING MAPPING LOGIC ---
    // Instead of (oled_x * 4),use a sliding window
    //oled_x is 0-95. scroll_offset moves the window.
    wire [8:0] bin_index = oled_x + scroll_offset;
    
    //RAM bank selection (Even/Odd)
    wire [7:0] ram_addr = bin_index[8:1]; 
    wire is_odd = bin_index[0];

    reg [11:0] magnitude_to_db;
    always @(*) begin
        // Ensure we don't read past the 512-bin limit
        if (bin_index < 512)
            magnitude_to_db = (is_odd) ? ram_odd[ram_addr] : ram_even[ram_addr];
        else
            magnitude_to_db = 0;
    end

    
        //dB Look-Up & Noise Gate ---
        reg [5:0] db_lut [0:2047]; 
        initial $readmemh("2048_dBLUT.mem", db_lut);
        
        //logarithmic math means that the silence and thermal noise is amplified
        
        localparam NOISE_THRESHOLD = 12'd45; 
    
        //Cap at 2047 to prevent memory overflow
        wire [11:0] safe_magnitude = (magnitude_to_db > 2047) ? 12'd2047 : magnitude_to_db;
        
        //THE NOISE GATE: If it's below the threshold, crush it to 0 (Silence)
        wire [11:0] gated_magnitude = (safe_magnitude < NOISE_THRESHOLD) ? 12'd0 : safe_magnitude;
        
        assign db_height = db_lut[gated_magnitude];
        
        
endmodule


module seven_seg_driver (
    input clk,                   // 100MHz system clock
    input [8:0] scroll_offset,   // From the scroll_controller
    output reg [3:0] an,         // Anodes (Digit selectors, Active LOW)
    output reg [6:0] seg         // Cathodes (Segment selectors, Active LOW)
);

    // --- 1. Translate Offset to Page Number ---
    // Avoids hardware division by using simple thresholds
    reg [3:0] page;
    always @(*) begin
        if (scroll_offset < 96)       page = 4'd1;
        else if (scroll_offset < 192) page = 4'd2;
        else if (scroll_offset < 288) page = 4'd3;
        else if (scroll_offset < 384) page = 4'd4;
        else                          page = 4'd5;
    end

    // Refresh Counter for Multiplexing ---
    // A 20-bit counter at 100MHz overflows roughly every 10ms (100Hz refresh rate)
    reg [19:0] refresh_counter = 0;
    always @(posedge clk) begin
        refresh_counter <= refresh_counter + 1;
    end
    
    //Use the top 2 bits to cycle through the 4 digits (00, 01, 10, 11)
    wire [1:0] digit_select = refresh_counter[19:18];

    //Digit Decoding and Display Logic ---
    always @(*) begin
        case (digit_select)
            2'b00: begin
                an = 4'b1110; // Activate Digit 0 (Rightmost)
                // Decode the Page Number
                case (page)
                    1: seg = 7'b1111001; // '1'
                    2: seg = 7'b0100100; // '2'
                    3: seg = 7'b0110000; // '3'
                    4: seg = 7'b0011001; // '4'
                    5: seg = 7'b0010010; // '5'
                    default: seg = 7'b1000000; // '0' (Fallback)
                endcase
            end
            
            2'b01: begin
                an = 4'b1101; // Activate Digit 1
                seg = 7'b1111111; // Blank
            end
            
            2'b10: begin
                an = 4'b1011; // Activate Digit 2
                seg = 7'b0111111; // '-' (Dash)
            end
            
            2'b11: begin
                an = 4'b0111; // Activate Digit 3 (Leftmost)
                seg = 7'b0001100; // 'P'
            end
        endcase
    end
endmodule



module scroll_controller(
    input clk,          // 100MHz
    input reset,
    input btnL,         // Scroll Left
    input btnR,         // Scroll Right
    output reg [8:0] scroll_offset = 0
);

    reg [19:0] debounce_count = 0;
    reg btnL_reg = 0;
    reg btnR_reg = 0;

    always @(posedge clk) begin
        if (reset) begin
            scroll_offset <= 0;
            debounce_count <= 0;
            btnL_reg <= 0;
            btnR_reg <= 0;
        end else begin
            // Let the timer run continuously
            debounce_count <= debounce_count + 1;
            
            // Execute button logic ONLY once every ~10.5ms (when the counter rolls over)
            if (debounce_count == 20'd0) begin
                
                // Store the current button states for the NEXT 10ms check
                btnL_reg <= btnL;
                btnR_reg <= btnR;
                
                // Edge Detection: Is the button currently pressed, but wasn't 10ms ago?
                if (btnR && !btnR_reg) begin
                    if (scroll_offset <= 320) 
                        scroll_offset <= scroll_offset + 96;
                    else 
                        scroll_offset <= 416; // Max out at Page 5
                end 
                else if (btnL && !btnL_reg) begin
                    if (scroll_offset >= 96) 
                        scroll_offset <= scroll_offset - 96;
                    else 
                        scroll_offset <= 0;   // Bottom out at Page 1
                end
            end
        end
    end
endmodule

/*
 * This source file contains a Verilog description of an IP core
 * automatically generated by the SPIRAL HDL Generator.
 *
 * This product includes a hardware design developed by Carnegie Mellon University.
 *
 * Copyright (c) 2005-2011 by Peter A. Milder for the SPIRAL Project,
 * Carnegie Mellon University
 *
 * For more information, see the SPIRAL project website at:
 *   http://www.spiral.net
 *
 * This design is provided for internal, non-commercial research use only
 * and is not for redistribution, with or without modifications.
 * 
 * You may not use the name "Carnegie Mellon University" or derivations
 * thereof to endorse or promote products derived from this software.
 *
 * THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT ANY WARRANTY OF ANY KIND, EITHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO ANY WARRANTY
 * THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS OR BE ERROR-FREE AND ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
 * TITLE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY
 * BE LIABLE FOR ANY DAMAGES, INCLUDING BUT NOT LIMITED TO DIRECT, INDIRECT,
 * SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR IN
 * ANY WAY CONNECTED WITH THIS SOFTWARE (WHETHER OR NOT BASED UPON WARRANTY,
 * CONTRACT, TORT OR OTHERWISE).
 *
 */

//   Input/output stream: 2 complex words per cycle
//   Throughput: one transform every 512 cycles
//   Latency: 1373 cycles

//   Resources required:
//     36 multipliers (12 x 12 bit)
//     58 adders (12 x 12 bit)
//     2 RAMs (256 words, 24 bits per word)
//     2 RAMs (512 words, 24 bits per word)
//     2 RAMs (16 words, 24 bits per word)
//     2 RAMs (128 words, 24 bits per word)
//     6 RAMs (1024 words, 24 bits per word)
//     2 RAMs (8 words, 24 bits per word)
//     2 RAMs (64 words, 24 bits per word)
//     2 RAMs (32 words, 24 bits per word)
//     2 ROMs (16 words, 12 bits per word)
//     2 ROMs (128 words, 12 bits per word)
//     2 ROMs (64 words, 12 bits per word)
//     2 ROMs (32 words, 12 bits per word)
//     2 ROMs (512 words, 12 bits per word)
//     2 ROMs (256 words, 12 bits per word)
//     2 ROMs (8 words, 12 bits per word)

// Generated on Wed Apr 08 07:23:45 UTC 2026

// Latency: 1373 clock cycles
// Throughput: 1 transform every 512 cycles


// We use an interleaved complex data format.  X0 represents the
// real portion of the first input, and X1 represents the imaginary
// portion.  The X variables are system inputs and the Y variables
// are system outputs.

// The design uses a system of flag signals to indicate the
// beginning of the input and output data streams.  The 'next'
// input (asserted high), is used to instruct the system that the
// input stream will begin on the following cycle.

// This system has a 'gap' of 512 cycles.  This means that
// 512 cycles must elapse between the beginning of the input
// vectors.

// The output signal 'next_out' (also asserted high) indicates
// that the output vector will begin streaming out of the system
 // on the following cycle.

// The system has a latency of 1373 cycles.  This means that
// the 'next_out' will be asserted 1373 cycles after the user
// asserts 'next'.

// The simple testbench below will demonstrate the timing for loading
// and unloading data vectors.
// The system reset signal is asserted high.

// Please note: when simulating floating point code, you must include
// Xilinx's DSP slice simulation module.


module dft_testbench();
   reg clk, reset, next;
   wire next_out;
   integer i, j, k, l, m;
   reg [15:0] counter;
   reg [11:0] in [3:0];
   wire [11:0] X0;
   wire [11:0] Y0;
   wire [11:0] X1;
   wire [11:0] Y1;
   wire [11:0] X2;
   wire [11:0] Y2;
   wire [11:0] X3;
   wire [11:0] Y3;
   reg clrCnt;
   assign X0 = in[0];
   assign X1 = in[1];
   assign X2 = in[2];
   assign X3 = in[3];

   initial clk = 0;

   always #10000 clk = ~clk;


   // Instantiate top-level module of core 'X' signals are system inputs
   // and 'Y' signals are system outputs
   dft_top dft_top_instance (.clk(clk), .reset(reset), .next(next), .next_out(next_out),
    .X0(X0), .Y0(Y0),
    .X1(X1), .Y1(Y1),
    .X2(X2), .Y2(Y2),
    .X3(X3), .Y3(Y3));

   // You can use this counter to verify that the gap and latency are as expected.
   always @(posedge clk) begin
      if (clrCnt) counter <= 0;
      else counter <= counter+1;
   end


   initial begin
      @(posedge clk);
      @(posedge clk);

      // On the next cycle, begin loading input vector.
      next <= 1;
      clrCnt <= 1;
      @(posedge clk);
      clrCnt <= 0;
      next <= 0;

      // The 1024 complex data points enter the system over 512 cycles
      for (j=0; j < 511; j = j+1) begin
          // Input: 2 complex words per cycle
         for (k=0; k < 4; k = k+1) begin
            in[k] <= j*4 + k;
         end
         @(posedge clk);
      end
      j = 511;
      for (k=0; k < 4; k = k+1) begin
         in[k] <= j*4 + k;
      end


      @(posedge clk);
      // Wait until the next data vector can be entered
      while (counter < 510)
        @(posedge clk);

      // On the next cycle, we will start the next data vector
      next <= 1;
      clrCnt <= 1;
      @(posedge clk);
      clrCnt <= 0;
      next <= 0;

      // Start entering next input vector
      for (j=0; j < 511; j = j+1) begin
         // Input 4 words per cycle
         for (k=0; k < 4; k = k+1) begin
            in[k] <= 2048 + j*4 + k;
          end
          @(posedge clk);
       end
       j = 511;
       for (k=0; k < 4; k = k+1) begin
          in[k] <= 2048 + j*4 + k;
       end
   end


   initial begin
      // set initial values
      in[0] <= 0;
      in[1] <= 0;
      in[2] <= 0;
      in[3] <= 0;
      next <= 0;
      reset <= 0;

      @(posedge clk);
      reset <= 1;
      @(posedge clk);
      reset <= 0;
      @(posedge clk);
      @(posedge clk);
      // Wait until next_out goes high, then wait one clock cycle and begin receiving data
      @(posedge next_out);
      @(posedge clk); #1;
      $display("--- begin output 1---");

      for (m=0; m < 511; m=m+1) begin
         $display("%x", Y0);
         $display("%x", Y1);
         $display("%x", Y2);
         $display("%x", Y3);
         @(posedge clk); #1;
      end
      $display("%x", Y0);
      $display("%x", Y1);
      $display("%x", Y2);
      $display("%x", Y3);
      // Wait until next_out goes high, then wait one clock cycle and begin receiving data
      @(posedge next_out);
      @(posedge clk); #1;
      $display("--- begin output 2---");

      for (m=0; m < 511; m=m+1) begin
         $display("%x", Y0);
         $display("%x", Y1);
         $display("%x", Y2);
         $display("%x", Y3);
         @(posedge clk); #1;
      end
      $display("%x", Y0);
      $display("%x", Y1);
      $display("%x", Y2);
      $display("%x", Y3);
      $finish;
   end
endmodule

// Latency: 1373
// Gap: 512
// module_name_is:dft_top
module dft_top(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [11:0] t0_0;
   wire [11:0] t0_1;
   wire [11:0] t0_2;
   wire [11:0] t0_3;
   wire next_0;
   wire [11:0] t1_0;
   wire [11:0] t1_1;
   wire [11:0] t1_2;
   wire [11:0] t1_3;
   wire next_1;
   wire [11:0] t2_0;
   wire [11:0] t2_1;
   wire [11:0] t2_2;
   wire [11:0] t2_3;
   wire next_2;
   wire [11:0] t3_0;
   wire [11:0] t3_1;
   wire [11:0] t3_2;
   wire [11:0] t3_3;
   wire next_3;
   wire [11:0] t4_0;
   wire [11:0] t4_1;
   wire [11:0] t4_2;
   wire [11:0] t4_3;
   wire next_4;
   wire [11:0] t5_0;
   wire [11:0] t5_1;
   wire [11:0] t5_2;
   wire [11:0] t5_3;
   wire next_5;
   wire [11:0] t6_0;
   wire [11:0] t6_1;
   wire [11:0] t6_2;
   wire [11:0] t6_3;
   wire next_6;
   wire [11:0] t7_0;
   wire [11:0] t7_1;
   wire [11:0] t7_2;
   wire [11:0] t7_3;
   wire next_7;
   wire [11:0] t8_0;
   wire [11:0] t8_1;
   wire [11:0] t8_2;
   wire [11:0] t8_3;
   wire next_8;
   wire [11:0] t9_0;
   wire [11:0] t9_1;
   wire [11:0] t9_2;
   wire [11:0] t9_3;
   wire next_9;
   wire [11:0] t10_0;
   wire [11:0] t10_1;
   wire [11:0] t10_2;
   wire [11:0] t10_3;
   wire next_10;
   wire [11:0] t11_0;
   wire [11:0] t11_1;
   wire [11:0] t11_2;
   wire [11:0] t11_3;
   wire next_11;
   wire [11:0] t12_0;
   wire [11:0] t12_1;
   wire [11:0] t12_2;
   wire [11:0] t12_3;
   wire next_12;
   wire [11:0] t13_0;
   wire [11:0] t13_1;
   wire [11:0] t13_2;
   wire [11:0] t13_3;
   wire next_13;
   wire [11:0] t14_0;
   wire [11:0] t14_1;
   wire [11:0] t14_2;
   wire [11:0] t14_3;
   wire next_14;
   wire [11:0] t15_0;
   wire [11:0] t15_1;
   wire [11:0] t15_2;
   wire [11:0] t15_3;
   wire next_15;
   wire [11:0] t16_0;
   wire [11:0] t16_1;
   wire [11:0] t16_2;
   wire [11:0] t16_3;
   wire next_16;
   wire [11:0] t17_0;
   wire [11:0] t17_1;
   wire [11:0] t17_2;
   wire [11:0] t17_3;
   wire next_17;
   wire [11:0] t18_0;
   wire [11:0] t18_1;
   wire [11:0] t18_2;
   wire [11:0] t18_3;
   wire next_18;
   wire [11:0] t19_0;
   wire [11:0] t19_1;
   wire [11:0] t19_2;
   wire [11:0] t19_3;
   wire next_19;
   wire [11:0] t20_0;
   wire [11:0] t20_1;
   wire [11:0] t20_2;
   wire [11:0] t20_3;
   wire next_20;
   wire [11:0] t21_0;
   wire [11:0] t21_1;
   wire [11:0] t21_2;
   wire [11:0] t21_3;
   wire next_21;
   wire [11:0] t22_0;
   wire [11:0] t22_1;
   wire [11:0] t22_2;
   wire [11:0] t22_3;
   wire next_22;
   wire [11:0] t23_0;
   wire [11:0] t23_1;
   wire [11:0] t23_2;
   wire [11:0] t23_3;
   wire next_23;
   wire [11:0] t24_0;
   wire [11:0] t24_1;
   wire [11:0] t24_2;
   wire [11:0] t24_3;
   wire next_24;
   wire [11:0] t25_0;
   wire [11:0] t25_1;
   wire [11:0] t25_2;
   wire [11:0] t25_3;
   wire next_25;
   wire [11:0] t26_0;
   wire [11:0] t26_1;
   wire [11:0] t26_2;
   wire [11:0] t26_3;
   wire next_26;
   wire [11:0] t27_0;
   wire [11:0] t27_1;
   wire [11:0] t27_2;
   wire [11:0] t27_3;
   wire next_27;
   wire [11:0] t28_0;
   wire [11:0] t28_1;
   wire [11:0] t28_2;
   wire [11:0] t28_3;
   wire next_28;
   wire [11:0] t29_0;
   wire [11:0] t29_1;
   wire [11:0] t29_2;
   wire [11:0] t29_3;
   wire next_29;
   wire [11:0] t30_0;
   wire [11:0] t30_1;
   wire [11:0] t30_2;
   wire [11:0] t30_3;
   wire next_30;
   assign t0_0 = X0;
   assign Y0 = t30_0;
   assign t0_1 = X1;
   assign Y1 = t30_1;
   assign t0_2 = X2;
   assign Y2 = t30_2;
   assign t0_3 = X3;
   assign Y3 = t30_3;
   assign next_0 = next;
   assign next_out = next_30;

// latency=484, gap=512
   rc23404 stage0(.clk(clk), .reset(reset), .next(next_0), .next_out(next_1),
    .X0(t0_0), .Y0(t1_0),
    .X1(t0_1), .Y1(t1_1),
    .X2(t0_2), .Y2(t1_2),
    .X3(t0_3), .Y3(t1_3));


// latency=2, gap=512
   codeBlock23406 stage1(.clk(clk), .reset(reset), .next_in(next_1), .next_out(next_2),
       .X0_in(t1_0), .Y0(t2_0),
       .X1_in(t1_1), .Y1(t2_1),
       .X2_in(t1_2), .Y2(t2_2),
       .X3_in(t1_3), .Y3(t2_3));


// latency=4, gap=512
   rc23487 stage2(.clk(clk), .reset(reset), .next(next_2), .next_out(next_3),
    .X0(t2_0), .Y0(t3_0),
    .X1(t2_1), .Y1(t3_1),
    .X2(t2_2), .Y2(t3_2),
    .X3(t2_3), .Y3(t3_3));


// latency=8, gap=512
   DirSum_23668 stage3(.next(next_3), .clk(clk), .reset(reset), .next_out(next_4),
       .X0(t3_0), .Y0(t4_0),
       .X1(t3_1), .Y1(t4_1),
       .X2(t3_2), .Y2(t4_2),
       .X3(t3_3), .Y3(t4_3));


// latency=2, gap=512
   codeBlock23671 stage4(.clk(clk), .reset(reset), .next_in(next_4), .next_out(next_5),
       .X0_in(t4_0), .Y0(t5_0),
       .X1_in(t4_1), .Y1(t5_1),
       .X2_in(t4_2), .Y2(t5_2),
       .X3_in(t4_3), .Y3(t5_3));


// latency=5, gap=512
   rc23752 stage5(.clk(clk), .reset(reset), .next(next_5), .next_out(next_6),
    .X0(t5_0), .Y0(t6_0),
    .X1(t5_1), .Y1(t6_1),
    .X2(t5_2), .Y2(t6_2),
    .X3(t5_3), .Y3(t6_3));


// latency=8, gap=512
   DirSum_23941 stage6(.next(next_6), .clk(clk), .reset(reset), .next_out(next_7),
       .X0(t6_0), .Y0(t7_0),
       .X1(t6_1), .Y1(t7_1),
       .X2(t6_2), .Y2(t7_2),
       .X3(t6_3), .Y3(t7_3));


// latency=2, gap=512
   codeBlock23944 stage7(.clk(clk), .reset(reset), .next_in(next_7), .next_out(next_8),
       .X0_in(t7_0), .Y0(t8_0),
       .X1_in(t7_1), .Y1(t8_1),
       .X2_in(t7_2), .Y2(t8_2),
       .X3_in(t7_3), .Y3(t8_3));


// latency=7, gap=512
   rc24025 stage8(.clk(clk), .reset(reset), .next(next_8), .next_out(next_9),
    .X0(t8_0), .Y0(t9_0),
    .X1(t8_1), .Y1(t9_1),
    .X2(t8_2), .Y2(t9_2),
    .X3(t8_3), .Y3(t9_3));


// latency=8, gap=512
   DirSum_24230 stage9(.next(next_9), .clk(clk), .reset(reset), .next_out(next_10),
       .X0(t9_0), .Y0(t10_0),
       .X1(t9_1), .Y1(t10_1),
       .X2(t9_2), .Y2(t10_2),
       .X3(t9_3), .Y3(t10_3));


// latency=2, gap=512
   codeBlock24233 stage10(.clk(clk), .reset(reset), .next_in(next_10), .next_out(next_11),
       .X0_in(t10_0), .Y0(t11_0),
       .X1_in(t10_1), .Y1(t11_1),
       .X2_in(t10_2), .Y2(t11_2),
       .X3_in(t10_3), .Y3(t11_3));


// latency=11, gap=512
   rc24314 stage11(.clk(clk), .reset(reset), .next(next_11), .next_out(next_12),
    .X0(t11_0), .Y0(t12_0),
    .X1(t11_1), .Y1(t12_1),
    .X2(t11_2), .Y2(t12_2),
    .X3(t11_3), .Y3(t12_3));


// latency=8, gap=512
   DirSum_24551 stage12(.next(next_12), .clk(clk), .reset(reset), .next_out(next_13),
       .X0(t12_0), .Y0(t13_0),
       .X1(t12_1), .Y1(t13_1),
       .X2(t12_2), .Y2(t13_2),
       .X3(t12_3), .Y3(t13_3));


// latency=2, gap=512
   codeBlock24554 stage13(.clk(clk), .reset(reset), .next_in(next_13), .next_out(next_14),
       .X0_in(t13_0), .Y0(t14_0),
       .X1_in(t13_1), .Y1(t14_1),
       .X2_in(t13_2), .Y2(t14_2),
       .X3_in(t13_3), .Y3(t14_3));


// latency=19, gap=512
   rc24635 stage14(.clk(clk), .reset(reset), .next(next_14), .next_out(next_15),
    .X0(t14_0), .Y0(t15_0),
    .X1(t14_1), .Y1(t15_1),
    .X2(t14_2), .Y2(t15_2),
    .X3(t14_3), .Y3(t15_3));


// latency=8, gap=512
   DirSum_24936 stage15(.next(next_15), .clk(clk), .reset(reset), .next_out(next_16),
       .X0(t15_0), .Y0(t16_0),
       .X1(t15_1), .Y1(t16_1),
       .X2(t15_2), .Y2(t16_2),
       .X3(t15_3), .Y3(t16_3));


// latency=2, gap=512
   codeBlock24939 stage16(.clk(clk), .reset(reset), .next_in(next_16), .next_out(next_17),
       .X0_in(t16_0), .Y0(t17_0),
       .X1_in(t16_1), .Y1(t17_1),
       .X2_in(t16_2), .Y2(t17_2),
       .X3_in(t16_3), .Y3(t17_3));


// latency=35, gap=512
   rc25020 stage17(.clk(clk), .reset(reset), .next(next_17), .next_out(next_18),
    .X0(t17_0), .Y0(t18_0),
    .X1(t17_1), .Y1(t18_1),
    .X2(t17_2), .Y2(t18_2),
    .X3(t17_3), .Y3(t18_3));


// latency=8, gap=512
   DirSum_25449 stage18(.next(next_18), .clk(clk), .reset(reset), .next_out(next_19),
       .X0(t18_0), .Y0(t19_0),
       .X1(t18_1), .Y1(t19_1),
       .X2(t18_2), .Y2(t19_2),
       .X3(t18_3), .Y3(t19_3));


// latency=2, gap=512
   codeBlock25452 stage19(.clk(clk), .reset(reset), .next_in(next_19), .next_out(next_20),
       .X0_in(t19_0), .Y0(t20_0),
       .X1_in(t19_1), .Y1(t20_1),
       .X2_in(t19_2), .Y2(t20_2),
       .X3_in(t19_3), .Y3(t20_3));


// latency=67, gap=512
   rc25533 stage20(.clk(clk), .reset(reset), .next(next_20), .next_out(next_21),
    .X0(t20_0), .Y0(t21_0),
    .X1(t20_1), .Y1(t21_1),
    .X2(t20_2), .Y2(t21_2),
    .X3(t20_3), .Y3(t21_3));


// latency=8, gap=512
   DirSum_26218 stage21(.next(next_21), .clk(clk), .reset(reset), .next_out(next_22),
       .X0(t21_0), .Y0(t22_0),
       .X1(t21_1), .Y1(t22_1),
       .X2(t21_2), .Y2(t22_2),
       .X3(t21_3), .Y3(t22_3));


// latency=2, gap=512
   codeBlock26221 stage22(.clk(clk), .reset(reset), .next_in(next_22), .next_out(next_23),
       .X0_in(t22_0), .Y0(t23_0),
       .X1_in(t22_1), .Y1(t23_1),
       .X2_in(t22_2), .Y2(t23_2),
       .X3_in(t22_3), .Y3(t23_3));


// latency=131, gap=512
   rc26302 stage23(.clk(clk), .reset(reset), .next(next_23), .next_out(next_24),
    .X0(t23_0), .Y0(t24_0),
    .X1(t23_1), .Y1(t24_1),
    .X2(t23_2), .Y2(t24_2),
    .X3(t23_3), .Y3(t24_3));


// latency=8, gap=512
   DirSum_27499 stage24(.next(next_24), .clk(clk), .reset(reset), .next_out(next_25),
       .X0(t24_0), .Y0(t25_0),
       .X1(t24_1), .Y1(t25_1),
       .X2(t24_2), .Y2(t25_2),
       .X3(t24_3), .Y3(t25_3));


// latency=2, gap=512
   codeBlock27502 stage25(.clk(clk), .reset(reset), .next_in(next_25), .next_out(next_26),
       .X0_in(t25_0), .Y0(t26_0),
       .X1_in(t25_1), .Y1(t26_1),
       .X2_in(t25_2), .Y2(t26_2),
       .X3_in(t25_3), .Y3(t26_3));


// latency=259, gap=512
   rc27583 stage26(.clk(clk), .reset(reset), .next(next_26), .next_out(next_27),
    .X0(t26_0), .Y0(t27_0),
    .X1(t26_1), .Y1(t27_1),
    .X2(t26_2), .Y2(t27_2),
    .X3(t26_3), .Y3(t27_3));


// latency=8, gap=512
   DirSum_29803 stage27(.next(next_27), .clk(clk), .reset(reset), .next_out(next_28),
       .X0(t27_0), .Y0(t28_0),
       .X1(t27_1), .Y1(t28_1),
       .X2(t27_2), .Y2(t28_2),
       .X3(t27_3), .Y3(t28_3));


// latency=2, gap=512
   codeBlock29806 stage28(.clk(clk), .reset(reset), .next_in(next_28), .next_out(next_29),
       .X0_in(t28_0), .Y0(t29_0),
       .X1_in(t28_1), .Y1(t29_1),
       .X2_in(t28_2), .Y2(t29_2),
       .X3_in(t28_3), .Y3(t29_3));


// latency=259, gap=512
   rc29887 stage29(.clk(clk), .reset(reset), .next(next_29), .next_out(next_30),
    .X0(t29_0), .Y0(t30_0),
    .X1(t29_1), .Y1(t30_1),
    .X2(t29_2), .Y2(t30_2),
    .X3(t29_3), .Y3(t30_3));


endmodule

// Latency: 484
// Gap: 512
module rc23404(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm23402 instPerm31544(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 484
// Gap: 512
module perm23402(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 512;
   parameter logDepth = 9;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[9] ^ addr0[0];
   assign inAddr0[0] = addr0[8];
   assign inAddr0[1] = addr0[7];
   assign inAddr0[2] = addr0[6];
   assign inAddr0[3] = addr0[5];
   assign inAddr0[4] = addr0[4];
   assign inAddr0[5] = addr0[3];
   assign inAddr0[6] = addr0[2];
   assign inAddr0[7] = addr0[1];
   assign inAddr0[8] = addr0[0];
   assign outBank0[0] = addr0b[9] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outAddr0[1] = addr0b[2];
   assign outAddr0[2] = addr0b[3];
   assign outAddr0[3] = addr0b[4];
   assign outAddr0[4] = addr0b[5];
   assign outAddr0[5] = addr0b[6];
   assign outAddr0[6] = addr0b[7];
   assign outAddr0[7] = addr0b[8];
   assign outAddr0[8] = addr0b[9];
   assign outBank_a0[0] = addr0c[9] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];
   assign outAddr_a0[1] = addr0c[2];
   assign outAddr_a0[2] = addr0c[3];
   assign outAddr_a0[3] = addr0c[4];
   assign outAddr_a0[4] = addr0c[5];
   assign outAddr_a0[5] = addr0c[6];
   assign outAddr_a0[6] = addr0c[7];
   assign outAddr_a0[7] = addr0c[8];
   assign outAddr_a0[8] = addr0c[9];

   assign inBank1[0] = addr1[9] ^ addr1[0];
   assign inAddr1[0] = addr1[8];
   assign inAddr1[1] = addr1[7];
   assign inAddr1[2] = addr1[6];
   assign inAddr1[3] = addr1[5];
   assign inAddr1[4] = addr1[4];
   assign inAddr1[5] = addr1[3];
   assign inAddr1[6] = addr1[2];
   assign inAddr1[7] = addr1[1];
   assign inAddr1[8] = addr1[0];
   assign outBank1[0] = addr1b[9] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outAddr1[1] = addr1b[2];
   assign outAddr1[2] = addr1b[3];
   assign outAddr1[3] = addr1b[4];
   assign outAddr1[4] = addr1b[5];
   assign outAddr1[5] = addr1b[6];
   assign outAddr1[6] = addr1b[7];
   assign outAddr1[7] = addr1b[8];
   assign outAddr1[8] = addr1b[9];
   assign outBank_a1[0] = addr1c[9] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];
   assign outAddr_a1[1] = addr1c[2];
   assign outAddr_a1[2] = addr1c[3];
   assign outAddr_a1[3] = addr1c[4];
   assign outAddr_a1[4] = addr1c[5];
   assign outAddr_a1[5] = addr1c[6];
   assign outAddr_a1[6] = addr1c[7];
   assign outAddr_a1[7] = addr1c[8];
   assign outAddr_a1[8] = addr1c[9];

   nextReg #(482, 9) nextReg_31549(.X(next), .Y(next0), .reset(reset), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31552(.X(next0), .Y(next_out), .clk(clk));


   memArray1024_23402 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
        outCount_for_rd_data <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 481)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
         if (inCount == 483)
            outCount_for_rd_data <= 0;
         else
            outCount_for_rd_data <= outCount_for_rd_data+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 481) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 511) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 481)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[8];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[8];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[8];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray1024_23402(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 512;
   parameter logDepth = 9;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   nextReg #(512, 9) nextReg_31557(.X(next), .Y(next0), .reset(reset), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule

module nextReg(X, Y, reset, clk);
   parameter depth=2, logDepth=1;

   output Y;
   input X;
   input              clk, reset;
   reg [logDepth:0] count;
   reg                active;

   assign Y = (count == depth) ? 1 : 0;

   always @ (posedge clk) begin
      if (reset == 1) begin
         count <= 0;
         active <= 0;
      end
      else if (X == 1) begin
         active <= 1;
         count <= 1;
      end
      else if (count == depth) begin
         count <= 0;
         active <= 0;
      end
      else if (active)
         count <= count+1;
   end
endmodule


module memMod(in, out, inAddr, outAddr, writeSel, clk);
   
   parameter depth=1024, width=16, logDepth=10;
   
   input [width-1:0]    in;
   input [logDepth-1:0] inAddr, outAddr;
   input 	        writeSel, clk;
   output [width-1:0] 	out;
   reg [width-1:0] 	out;
   
   // synthesis attribute ram_style of mem is block

   reg [width-1:0] 	mem[depth-1:0]; 
   
   always @(posedge clk) begin
      out <= mem[outAddr];
      
      if (writeSel)
        mem[inAddr] <= in;
   end
endmodule 



module memMod_dist(in, out, inAddr, outAddr, writeSel, clk);
   
   parameter depth=1024, width=16, logDepth=10;
   
   input [width-1:0]    in;
   input [logDepth-1:0] inAddr, outAddr;
   input 	        writeSel, clk;
   output [width-1:0] 	out;
   reg [width-1:0] 	out;
   
   // synthesis attribute ram_style of mem is distributed

   reg [width-1:0] 	mem[depth-1:0]; 
   
   always @(posedge clk) begin
      out <= mem[outAddr];
      
      if (writeSel)
        mem[inAddr] <= in;
   end
endmodule 

module switch(ctrl, x0, x1, y0, y1);
    parameter width = 16;
    input [width-1:0] x0, x1;
    output [width-1:0] y0, y1;
    input ctrl;
    assign y0 = (ctrl == 0) ? x0 : x1;
    assign y1 = (ctrl == 0) ? x1 : x0;
endmodule

module shiftRegFIFO(X, Y, clk);
   parameter depth=1, width=1;

   output [width-1:0] Y;
   input  [width-1:0] X;
   input              clk;

   reg [width-1:0]    mem [depth-1:0];
   integer            index;

   assign Y = mem[depth-1];

   always @ (posedge clk) begin
      for(index=1;index<depth;index=index+1) begin
         mem[index] <= mem[index-1];
      end
      mem[0]<=X;
   end
endmodule

// Latency: 2
// Gap: 1
module codeBlock23406(clk, reset, next_in, next_out,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(1, 1) shiftFIFO_31564(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a550;
   wire signed [11:0] a551;
   wire signed [11:0] a552;
   wire signed [11:0] a553;
   wire signed [12:0] tm234;
   wire signed [12:0] tm235;
   wire signed [12:0] tm236;
   wire signed [12:0] tm237;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   wire signed [11:0] t237;
   wire signed [11:0] t238;
   wire signed [11:0] t239;
   wire signed [11:0] t240;


   assign a550 = X0;
   assign a551 = X2;
   assign a552 = X1;
   assign a553 = X3;
   assign Y0 = t237;
   assign Y1 = t238;
   assign Y2 = t239;
   assign Y3 = t240;
   assign t237 = tm234[12:1];
   assign t238 = tm235[12:1];
   assign t239 = tm236[12:1];
   assign t240 = tm237[12:1];

    addfxp #(13, 1) add23418(.a({{1{a550[11]}}, a550}), .b({{1{a551[11]}}, a551}), .clk(clk), .q(tm234));    // 0
    addfxp #(13, 1) add23433(.a({{1{a552[11]}}, a552}), .b({{1{a553[11]}}, a553}), .clk(clk), .q(tm235));    // 0
    subfxp #(13, 1) sub23448(.a({{1{a550[11]}}, a550}), .b({{1{a551[11]}}, a551}), .clk(clk), .q(tm236));    // 0
    subfxp #(13, 1) sub23463(.a({{1{a552[11]}}, a552}), .b({{1{a553[11]}}, a553}), .clk(clk), .q(tm237));    // 0


   always @(posedge clk) begin
      if (reset == 1) begin
      end
      else begin
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
      end
   end
endmodule

// Latency: 4
// Gap: 2
module rc23487(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm23485 instPerm31565(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 4
// Gap: 2
module perm23485(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 2;
   parameter logDepth = 1;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[1] ^ addr0[0];
   assign inAddr0[0] = addr0[0];
   assign outBank0[0] = addr0b[1] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outBank_a0[0] = addr0c[1] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];

   assign inBank1[0] = addr1[1] ^ addr1[0];
   assign inAddr1[0] = addr1[0];
   assign outBank1[0] = addr1b[1] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outBank_a1[0] = addr1c[1] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];

   shiftRegFIFO #(2, 1) shiftFIFO_31568(.X(next), .Y(next0), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31571(.X(next0), .Y(next_out), .clk(clk));


   memArray4_23485 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

    reg resetOutCountRd2_2;
    reg resetOutCountRd2_3;

    always @(posedge clk) begin
        if (reset == 1) begin
            resetOutCountRd2_2 <= 0;
            resetOutCountRd2_3 <= 0;
        end
        else begin
            resetOutCountRd2_2 <= (inCount == 1) ? 1'b1 : 1'b0;
            resetOutCountRd2_3 <= resetOutCountRd2_2;
            if (resetOutCountRd2_3 == 1'b1)
                outCount_for_rd_data <= 0;
            else
                outCount_for_rd_data <= outCount_for_rd_data+1;
        end
    end
   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 1)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 1) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 1) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 1)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[0];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[0];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[0];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray4_23485(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 2;
   parameter logDepth = 1;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   shiftRegFIFO #(2, 1) shiftFIFO_31574(.X(next), .Y(next0), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule

// Latency: 8
// Gap: 2
module DirSum_23668(clk, reset, next, next_out,
      X0, Y0,
      X1, Y1,
      X2, Y2,
      X3, Y3);

   output next_out;
   input clk, reset, next;

   reg [0:0] i9;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   always @(posedge clk) begin
      if (reset == 1) begin
         i9 <= 0;
      end
      else begin
         if (next == 1)
            i9 <= 0;
         else if (i9 == 1)
            i9 <= 0;
         else
            i9 <= i9 + 1;
      end
   end

   codeBlock23490 codeBlockIsnt31575(.clk(clk), .reset(reset), .next_in(next), .next_out(next_out),
.i9_in(i9),
       .X0_in(X0), .Y0(Y0),
       .X1_in(X1), .Y1(Y1),
       .X2_in(X2), .Y2(Y2),
       .X3_in(X3), .Y3(Y3));

endmodule

module D36_23654(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [0:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h0;
      1: out3 <= 12'hc00;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



module D34_23662(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [0:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h400;
      1: out3 <= 12'h0;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



// Latency: 8
// Gap: 1
module codeBlock23490(clk, reset, next_in, next_out,
   i9_in,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;
   input [0:0] i9_in;
   reg [0:0] i9;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(7, 1) shiftFIFO_31578(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a534;
   wire signed [11:0] a523;
   wire signed [11:0] a537;
   wire signed [11:0] a527;
   wire signed [11:0] a538;
   wire signed [11:0] a539;
   reg signed [11:0] tm274;
   reg signed [11:0] tm278;
   reg signed [11:0] tm290;
   reg signed [11:0] tm297;
   reg signed [11:0] tm275;
   reg signed [11:0] tm279;
   reg signed [11:0] tm291;
   reg signed [11:0] tm298;
   wire signed [11:0] tm2;
   wire signed [11:0] a528;
   wire signed [11:0] tm3;
   wire signed [11:0] a530;
   reg signed [11:0] tm276;
   reg signed [11:0] tm280;
   reg signed [11:0] tm292;
   reg signed [11:0] tm299;
   reg signed [11:0] tm40;
   reg signed [11:0] tm41;
   reg signed [11:0] tm277;
   reg signed [11:0] tm281;
   reg signed [11:0] tm293;
   reg signed [11:0] tm300;
   reg signed [11:0] tm294;
   reg signed [11:0] tm301;
   wire signed [11:0] a529;
   wire signed [11:0] a531;
   wire signed [11:0] a532;
   wire signed [11:0] a533;
   reg signed [11:0] tm295;
   reg signed [11:0] tm302;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   reg signed [11:0] tm296;
   reg signed [11:0] tm303;


   assign a534 = X0;
   assign a523 = a534;
   assign a537 = X1;
   assign a527 = a537;
   assign a538 = X2;
   assign a539 = X3;
   assign a528 = tm2;
   assign a530 = tm3;
   assign Y0 = tm296;
   assign Y1 = tm303;

   D36_23654 instD36inst0_23654(.addr(i9[0:0]), .out(tm3), .clk(clk));

   D34_23662 instD34inst0_23662(.addr(i9[0:0]), .out(tm2), .clk(clk));

    multfix #(12, 2) m23589(.a(tm40), .b(tm277), .clk(clk), .q_sc(a529), .q_unsc(), .rst(reset));
    multfix #(12, 2) m23611(.a(tm41), .b(tm281), .clk(clk), .q_sc(a531), .q_unsc(), .rst(reset));
    multfix #(12, 2) m23629(.a(tm41), .b(tm277), .clk(clk), .q_sc(a532), .q_unsc(), .rst(reset));
    multfix #(12, 2) m23640(.a(tm40), .b(tm281), .clk(clk), .q_sc(a533), .q_unsc(), .rst(reset));
    subfxp #(12, 1) sub23618(.a(a529), .b(a531), .clk(clk), .q(Y2));    // 6
    addfxp #(12, 1) add23647(.a(a532), .b(a533), .clk(clk), .q(Y3));    // 6


   always @(posedge clk) begin
      if (reset == 1) begin
         tm40 <= 0;
         tm277 <= 0;
         tm41 <= 0;
         tm281 <= 0;
         tm41 <= 0;
         tm277 <= 0;
         tm40 <= 0;
         tm281 <= 0;
      end
      else begin
         i9 <= i9_in;
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
         tm274 <= a538;
         tm278 <= a539;
         tm290 <= a523;
         tm297 <= a527;
         tm275 <= tm274;
         tm279 <= tm278;
         tm291 <= tm290;
         tm298 <= tm297;
         tm276 <= tm275;
         tm280 <= tm279;
         tm292 <= tm291;
         tm299 <= tm298;
         tm40 <= a528;
         tm41 <= a530;
         tm277 <= tm276;
         tm281 <= tm280;
         tm293 <= tm292;
         tm300 <= tm299;
         tm294 <= tm293;
         tm301 <= tm300;
         tm295 <= tm294;
         tm302 <= tm301;
         tm296 <= tm295;
         tm303 <= tm302;
      end
   end
endmodule

// Latency: 2
// Gap: 1
module codeBlock23671(clk, reset, next_in, next_out,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(1, 1) shiftFIFO_31581(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a490;
   wire signed [11:0] a491;
   wire signed [11:0] a492;
   wire signed [11:0] a493;
   wire signed [12:0] tm238;
   wire signed [12:0] tm239;
   wire signed [12:0] tm240;
   wire signed [12:0] tm241;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   wire signed [11:0] t213;
   wire signed [11:0] t214;
   wire signed [11:0] t215;
   wire signed [11:0] t216;


   assign a490 = X0;
   assign a491 = X2;
   assign a492 = X1;
   assign a493 = X3;
   assign Y0 = t213;
   assign Y1 = t214;
   assign Y2 = t215;
   assign Y3 = t216;
   assign t213 = tm238[12:1];
   assign t214 = tm239[12:1];
   assign t215 = tm240[12:1];
   assign t216 = tm241[12:1];

    addfxp #(13, 1) add23683(.a({{1{a490[11]}}, a490}), .b({{1{a491[11]}}, a491}), .clk(clk), .q(tm238));    // 0
    addfxp #(13, 1) add23698(.a({{1{a492[11]}}, a492}), .b({{1{a493[11]}}, a493}), .clk(clk), .q(tm239));    // 0
    subfxp #(13, 1) sub23713(.a({{1{a490[11]}}, a490}), .b({{1{a491[11]}}, a491}), .clk(clk), .q(tm240));    // 0
    subfxp #(13, 1) sub23728(.a({{1{a492[11]}}, a492}), .b({{1{a493[11]}}, a493}), .clk(clk), .q(tm241));    // 0


   always @(posedge clk) begin
      if (reset == 1) begin
      end
      else begin
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
      end
   end
endmodule

// Latency: 5
// Gap: 4
module rc23752(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm23750 instPerm31582(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 5
// Gap: 4
module perm23750(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 4;
   parameter logDepth = 2;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[2] ^ addr0[0];
   assign inAddr0[0] = addr0[1];
   assign inAddr0[1] = addr0[0];
   assign outBank0[0] = addr0b[2] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outAddr0[1] = addr0b[2];
   assign outBank_a0[0] = addr0c[2] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];
   assign outAddr_a0[1] = addr0c[2];

   assign inBank1[0] = addr1[2] ^ addr1[0];
   assign inAddr1[0] = addr1[1];
   assign inAddr1[1] = addr1[0];
   assign outBank1[0] = addr1b[2] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outAddr1[1] = addr1b[2];
   assign outBank_a1[0] = addr1c[2] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];
   assign outAddr_a1[1] = addr1c[2];

   shiftRegFIFO #(3, 1) shiftFIFO_31585(.X(next), .Y(next0), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31588(.X(next0), .Y(next_out), .clk(clk));


   memArray8_23750 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

    reg resetOutCountRd2_4;

    always @(posedge clk) begin
        if (reset == 1) begin
            resetOutCountRd2_4 <= 0;
        end
        else begin
            resetOutCountRd2_4 <= (inCount == 3) ? 1'b1 : 1'b0;
            if (resetOutCountRd2_4 == 1'b1)
                outCount_for_rd_data <= 0;
            else
                outCount_for_rd_data <= outCount_for_rd_data+1;
        end
    end
   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 2)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 2) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 3) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 2)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[1];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[1];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[1];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray8_23750(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 4;
   parameter logDepth = 2;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   shiftRegFIFO #(4, 1) shiftFIFO_31591(.X(next), .Y(next0), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule

// Latency: 8
// Gap: 4
module DirSum_23941(clk, reset, next, next_out,
      X0, Y0,
      X1, Y1,
      X2, Y2,
      X3, Y3);

   output next_out;
   input clk, reset, next;

   reg [1:0] i8;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   always @(posedge clk) begin
      if (reset == 1) begin
         i8 <= 0;
      end
      else begin
         if (next == 1)
            i8 <= 0;
         else if (i8 == 3)
            i8 <= 0;
         else
            i8 <= i8 + 1;
      end
   end

   codeBlock23755 codeBlockIsnt31592(.clk(clk), .reset(reset), .next_in(next), .next_out(next_out),
.i8_in(i8),
       .X0_in(X0), .Y0(Y0),
       .X1_in(X1), .Y1(Y1),
       .X2_in(X2), .Y2(Y2),
       .X3_in(X3), .Y3(Y3));

endmodule

module D32_23921(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [1:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h0;
      1: out3 <= 12'hd2c;
      2: out3 <= 12'hc00;
      3: out3 <= 12'hd2c;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



module D30_23933(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [1:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h400;
      1: out3 <= 12'h2d4;
      2: out3 <= 12'h0;
      3: out3 <= 12'hd2c;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



// Latency: 8
// Gap: 1
module codeBlock23755(clk, reset, next_in, next_out,
   i8_in,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;
   input [1:0] i8_in;
   reg [1:0] i8;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(7, 1) shiftFIFO_31595(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a474;
   wire signed [11:0] a463;
   wire signed [11:0] a477;
   wire signed [11:0] a467;
   wire signed [11:0] a478;
   wire signed [11:0] a479;
   reg signed [11:0] tm304;
   reg signed [11:0] tm308;
   reg signed [11:0] tm320;
   reg signed [11:0] tm327;
   reg signed [11:0] tm305;
   reg signed [11:0] tm309;
   reg signed [11:0] tm321;
   reg signed [11:0] tm328;
   wire signed [11:0] tm6;
   wire signed [11:0] a468;
   wire signed [11:0] tm7;
   wire signed [11:0] a470;
   reg signed [11:0] tm306;
   reg signed [11:0] tm310;
   reg signed [11:0] tm322;
   reg signed [11:0] tm329;
   reg signed [11:0] tm48;
   reg signed [11:0] tm49;
   reg signed [11:0] tm307;
   reg signed [11:0] tm311;
   reg signed [11:0] tm323;
   reg signed [11:0] tm330;
   reg signed [11:0] tm324;
   reg signed [11:0] tm331;
   wire signed [11:0] a469;
   wire signed [11:0] a471;
   wire signed [11:0] a472;
   wire signed [11:0] a473;
   reg signed [11:0] tm325;
   reg signed [11:0] tm332;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   reg signed [11:0] tm326;
   reg signed [11:0] tm333;


   assign a474 = X0;
   assign a463 = a474;
   assign a477 = X1;
   assign a467 = a477;
   assign a478 = X2;
   assign a479 = X3;
   assign a468 = tm6;
   assign a470 = tm7;
   assign Y0 = tm326;
   assign Y1 = tm333;

   D32_23921 instD32inst0_23921(.addr(i8[1:0]), .out(tm7), .clk(clk));

   D30_23933 instD30inst0_23933(.addr(i8[1:0]), .out(tm6), .clk(clk));

    multfix #(12, 2) m23854(.a(tm48), .b(tm307), .clk(clk), .q_sc(a469), .q_unsc(), .rst(reset));
    multfix #(12, 2) m23876(.a(tm49), .b(tm311), .clk(clk), .q_sc(a471), .q_unsc(), .rst(reset));
    multfix #(12, 2) m23894(.a(tm49), .b(tm307), .clk(clk), .q_sc(a472), .q_unsc(), .rst(reset));
    multfix #(12, 2) m23905(.a(tm48), .b(tm311), .clk(clk), .q_sc(a473), .q_unsc(), .rst(reset));
    subfxp #(12, 1) sub23883(.a(a469), .b(a471), .clk(clk), .q(Y2));    // 6
    addfxp #(12, 1) add23912(.a(a472), .b(a473), .clk(clk), .q(Y3));    // 6


   always @(posedge clk) begin
      if (reset == 1) begin
         tm48 <= 0;
         tm307 <= 0;
         tm49 <= 0;
         tm311 <= 0;
         tm49 <= 0;
         tm307 <= 0;
         tm48 <= 0;
         tm311 <= 0;
      end
      else begin
         i8 <= i8_in;
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
         tm304 <= a478;
         tm308 <= a479;
         tm320 <= a463;
         tm327 <= a467;
         tm305 <= tm304;
         tm309 <= tm308;
         tm321 <= tm320;
         tm328 <= tm327;
         tm306 <= tm305;
         tm310 <= tm309;
         tm322 <= tm321;
         tm329 <= tm328;
         tm48 <= a468;
         tm49 <= a470;
         tm307 <= tm306;
         tm311 <= tm310;
         tm323 <= tm322;
         tm330 <= tm329;
         tm324 <= tm323;
         tm331 <= tm330;
         tm325 <= tm324;
         tm332 <= tm331;
         tm326 <= tm325;
         tm333 <= tm332;
      end
   end
endmodule

// Latency: 2
// Gap: 1
module codeBlock23944(clk, reset, next_in, next_out,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(1, 1) shiftFIFO_31598(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a430;
   wire signed [11:0] a431;
   wire signed [11:0] a432;
   wire signed [11:0] a433;
   wire signed [12:0] tm242;
   wire signed [12:0] tm243;
   wire signed [12:0] tm244;
   wire signed [12:0] tm245;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   wire signed [11:0] t189;
   wire signed [11:0] t190;
   wire signed [11:0] t191;
   wire signed [11:0] t192;


   assign a430 = X0;
   assign a431 = X2;
   assign a432 = X1;
   assign a433 = X3;
   assign Y0 = t189;
   assign Y1 = t190;
   assign Y2 = t191;
   assign Y3 = t192;
   assign t189 = tm242[12:1];
   assign t190 = tm243[12:1];
   assign t191 = tm244[12:1];
   assign t192 = tm245[12:1];

    addfxp #(13, 1) add23956(.a({{1{a430[11]}}, a430}), .b({{1{a431[11]}}, a431}), .clk(clk), .q(tm242));    // 0
    addfxp #(13, 1) add23971(.a({{1{a432[11]}}, a432}), .b({{1{a433[11]}}, a433}), .clk(clk), .q(tm243));    // 0
    subfxp #(13, 1) sub23986(.a({{1{a430[11]}}, a430}), .b({{1{a431[11]}}, a431}), .clk(clk), .q(tm244));    // 0
    subfxp #(13, 1) sub24001(.a({{1{a432[11]}}, a432}), .b({{1{a433[11]}}, a433}), .clk(clk), .q(tm245));    // 0


   always @(posedge clk) begin
      if (reset == 1) begin
      end
      else begin
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
      end
   end
endmodule

// Latency: 7
// Gap: 8
module rc24025(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm24023 instPerm31599(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 7
// Gap: 8
module perm24023(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 8;
   parameter logDepth = 3;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[3] ^ addr0[0];
   assign inAddr0[0] = addr0[1];
   assign inAddr0[1] = addr0[2];
   assign inAddr0[2] = addr0[0];
   assign outBank0[0] = addr0b[3] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outAddr0[1] = addr0b[2];
   assign outAddr0[2] = addr0b[3];
   assign outBank_a0[0] = addr0c[3] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];
   assign outAddr_a0[1] = addr0c[2];
   assign outAddr_a0[2] = addr0c[3];

   assign inBank1[0] = addr1[3] ^ addr1[0];
   assign inAddr1[0] = addr1[1];
   assign inAddr1[1] = addr1[2];
   assign inAddr1[2] = addr1[0];
   assign outBank1[0] = addr1b[3] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outAddr1[1] = addr1b[2];
   assign outAddr1[2] = addr1b[3];
   assign outBank_a1[0] = addr1c[3] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];
   assign outAddr_a1[1] = addr1c[2];
   assign outAddr_a1[2] = addr1c[3];

   shiftRegFIFO #(5, 1) shiftFIFO_31602(.X(next), .Y(next0), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31605(.X(next0), .Y(next_out), .clk(clk));


   memArray16_24023 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
        outCount_for_rd_data <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 4)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
         if (inCount == 6)
            outCount_for_rd_data <= 0;
         else
            outCount_for_rd_data <= outCount_for_rd_data+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 4) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 7) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 4)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[2];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[2];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[2];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray16_24023(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 8;
   parameter logDepth = 3;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   shiftRegFIFO #(8, 1) shiftFIFO_31608(.X(next), .Y(next0), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule

// Latency: 8
// Gap: 8
module DirSum_24230(clk, reset, next, next_out,
      X0, Y0,
      X1, Y1,
      X2, Y2,
      X3, Y3);

   output next_out;
   input clk, reset, next;

   reg [2:0] i7;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   always @(posedge clk) begin
      if (reset == 1) begin
         i7 <= 0;
      end
      else begin
         if (next == 1)
            i7 <= 0;
         else if (i7 == 7)
            i7 <= 0;
         else
            i7 <= i7 + 1;
      end
   end

   codeBlock24028 codeBlockIsnt31609(.clk(clk), .reset(reset), .next_in(next), .next_out(next_out),
.i7_in(i7),
       .X0_in(X0), .Y0(Y0),
       .X1_in(X1), .Y1(Y1),
       .X2_in(X2), .Y2(Y2),
       .X3_in(X3), .Y3(Y3));

endmodule

module D28_24198(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [2:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h0;
      1: out3 <= 12'he78;
      2: out3 <= 12'hd2c;
      3: out3 <= 12'hc4e;
      4: out3 <= 12'hc00;
      5: out3 <= 12'hc4e;
      6: out3 <= 12'hd2c;
      7: out3 <= 12'he78;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



module D26_24218(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [2:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h400;
      1: out3 <= 12'h3b2;
      2: out3 <= 12'h2d4;
      3: out3 <= 12'h188;
      4: out3 <= 12'h0;
      5: out3 <= 12'he78;
      6: out3 <= 12'hd2c;
      7: out3 <= 12'hc4e;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



// Latency: 8
// Gap: 1
module codeBlock24028(clk, reset, next_in, next_out,
   i7_in,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;
   input [2:0] i7_in;
   reg [2:0] i7;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(7, 1) shiftFIFO_31612(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a414;
   wire signed [11:0] a403;
   wire signed [11:0] a417;
   wire signed [11:0] a407;
   wire signed [11:0] a418;
   wire signed [11:0] a419;
   reg signed [11:0] tm334;
   reg signed [11:0] tm338;
   reg signed [11:0] tm350;
   reg signed [11:0] tm357;
   reg signed [11:0] tm335;
   reg signed [11:0] tm339;
   reg signed [11:0] tm351;
   reg signed [11:0] tm358;
   wire signed [11:0] tm10;
   wire signed [11:0] a408;
   wire signed [11:0] tm11;
   wire signed [11:0] a410;
   reg signed [11:0] tm336;
   reg signed [11:0] tm340;
   reg signed [11:0] tm352;
   reg signed [11:0] tm359;
   reg signed [11:0] tm56;
   reg signed [11:0] tm57;
   reg signed [11:0] tm337;
   reg signed [11:0] tm341;
   reg signed [11:0] tm353;
   reg signed [11:0] tm360;
   reg signed [11:0] tm354;
   reg signed [11:0] tm361;
   wire signed [11:0] a409;
   wire signed [11:0] a411;
   wire signed [11:0] a412;
   wire signed [11:0] a413;
   reg signed [11:0] tm355;
   reg signed [11:0] tm362;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   reg signed [11:0] tm356;
   reg signed [11:0] tm363;


   assign a414 = X0;
   assign a403 = a414;
   assign a417 = X1;
   assign a407 = a417;
   assign a418 = X2;
   assign a419 = X3;
   assign a408 = tm10;
   assign a410 = tm11;
   assign Y0 = tm356;
   assign Y1 = tm363;

   D28_24198 instD28inst0_24198(.addr(i7[2:0]), .out(tm11), .clk(clk));

   D26_24218 instD26inst0_24218(.addr(i7[2:0]), .out(tm10), .clk(clk));

    multfix #(12, 2) m24127(.a(tm56), .b(tm337), .clk(clk), .q_sc(a409), .q_unsc(), .rst(reset));
    multfix #(12, 2) m24149(.a(tm57), .b(tm341), .clk(clk), .q_sc(a411), .q_unsc(), .rst(reset));
    multfix #(12, 2) m24167(.a(tm57), .b(tm337), .clk(clk), .q_sc(a412), .q_unsc(), .rst(reset));
    multfix #(12, 2) m24178(.a(tm56), .b(tm341), .clk(clk), .q_sc(a413), .q_unsc(), .rst(reset));
    subfxp #(12, 1) sub24156(.a(a409), .b(a411), .clk(clk), .q(Y2));    // 6
    addfxp #(12, 1) add24185(.a(a412), .b(a413), .clk(clk), .q(Y3));    // 6


   always @(posedge clk) begin
      if (reset == 1) begin
         tm56 <= 0;
         tm337 <= 0;
         tm57 <= 0;
         tm341 <= 0;
         tm57 <= 0;
         tm337 <= 0;
         tm56 <= 0;
         tm341 <= 0;
      end
      else begin
         i7 <= i7_in;
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
         tm334 <= a418;
         tm338 <= a419;
         tm350 <= a403;
         tm357 <= a407;
         tm335 <= tm334;
         tm339 <= tm338;
         tm351 <= tm350;
         tm358 <= tm357;
         tm336 <= tm335;
         tm340 <= tm339;
         tm352 <= tm351;
         tm359 <= tm358;
         tm56 <= a408;
         tm57 <= a410;
         tm337 <= tm336;
         tm341 <= tm340;
         tm353 <= tm352;
         tm360 <= tm359;
         tm354 <= tm353;
         tm361 <= tm360;
         tm355 <= tm354;
         tm362 <= tm361;
         tm356 <= tm355;
         tm363 <= tm362;
      end
   end
endmodule

// Latency: 2
// Gap: 1
module codeBlock24233(clk, reset, next_in, next_out,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(1, 1) shiftFIFO_31615(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a369;
   wire signed [11:0] a370;
   wire signed [11:0] a371;
   wire signed [11:0] a372;
   wire signed [12:0] tm246;
   wire signed [12:0] tm247;
   wire signed [12:0] tm248;
   wire signed [12:0] tm249;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   wire signed [11:0] t165;
   wire signed [11:0] t166;
   wire signed [11:0] t167;
   wire signed [11:0] t168;


   assign a369 = X0;
   assign a370 = X2;
   assign a371 = X1;
   assign a372 = X3;
   assign Y0 = t165;
   assign Y1 = t166;
   assign Y2 = t167;
   assign Y3 = t168;
   assign t165 = tm246[12:1];
   assign t166 = tm247[12:1];
   assign t167 = tm248[12:1];
   assign t168 = tm249[12:1];

    addfxp #(13, 1) add24245(.a({{1{a369[11]}}, a369}), .b({{1{a370[11]}}, a370}), .clk(clk), .q(tm246));    // 0
    addfxp #(13, 1) add24260(.a({{1{a371[11]}}, a371}), .b({{1{a372[11]}}, a372}), .clk(clk), .q(tm247));    // 0
    subfxp #(13, 1) sub24275(.a({{1{a369[11]}}, a369}), .b({{1{a370[11]}}, a370}), .clk(clk), .q(tm248));    // 0
    subfxp #(13, 1) sub24290(.a({{1{a371[11]}}, a371}), .b({{1{a372[11]}}, a372}), .clk(clk), .q(tm249));    // 0


   always @(posedge clk) begin
      if (reset == 1) begin
      end
      else begin
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
      end
   end
endmodule

// Latency: 11
// Gap: 16
module rc24314(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm24312 instPerm31616(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 11
// Gap: 16
module perm24312(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 16;
   parameter logDepth = 4;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[4] ^ addr0[0];
   assign inAddr0[0] = addr0[1];
   assign inAddr0[1] = addr0[2];
   assign inAddr0[2] = addr0[3];
   assign inAddr0[3] = addr0[0];
   assign outBank0[0] = addr0b[4] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outAddr0[1] = addr0b[2];
   assign outAddr0[2] = addr0b[3];
   assign outAddr0[3] = addr0b[4];
   assign outBank_a0[0] = addr0c[4] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];
   assign outAddr_a0[1] = addr0c[2];
   assign outAddr_a0[2] = addr0c[3];
   assign outAddr_a0[3] = addr0c[4];

   assign inBank1[0] = addr1[4] ^ addr1[0];
   assign inAddr1[0] = addr1[1];
   assign inAddr1[1] = addr1[2];
   assign inAddr1[2] = addr1[3];
   assign inAddr1[3] = addr1[0];
   assign outBank1[0] = addr1b[4] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outAddr1[1] = addr1b[2];
   assign outAddr1[2] = addr1b[3];
   assign outAddr1[3] = addr1b[4];
   assign outBank_a1[0] = addr1c[4] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];
   assign outAddr_a1[1] = addr1c[2];
   assign outAddr_a1[2] = addr1c[3];
   assign outAddr_a1[3] = addr1c[4];

   nextReg #(9, 4) nextReg_31621(.X(next), .Y(next0), .reset(reset), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31624(.X(next0), .Y(next_out), .clk(clk));


   memArray32_24312 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
        outCount_for_rd_data <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 8)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
         if (inCount == 10)
            outCount_for_rd_data <= 0;
         else
            outCount_for_rd_data <= outCount_for_rd_data+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 8) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 15) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 8)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[3];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[3];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[3];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray32_24312(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 16;
   parameter logDepth = 4;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   nextReg #(16, 4) nextReg_31629(.X(next), .Y(next0), .reset(reset), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule

// Latency: 8
// Gap: 16
module DirSum_24551(clk, reset, next, next_out,
      X0, Y0,
      X1, Y1,
      X2, Y2,
      X3, Y3);

   output next_out;
   input clk, reset, next;

   reg [3:0] i6;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   always @(posedge clk) begin
      if (reset == 1) begin
         i6 <= 0;
      end
      else begin
         if (next == 1)
            i6 <= 0;
         else if (i6 == 15)
            i6 <= 0;
         else
            i6 <= i6 + 1;
      end
   end

   codeBlock24317 codeBlockIsnt31634(.clk(clk), .reset(reset), .next_in(next), .next_out(next_out),
.i6_in(i6),
       .X0_in(X0), .Y0(Y0),
       .X1_in(X1), .Y1(Y1),
       .X2_in(X2), .Y2(Y2),
       .X3_in(X3), .Y3(Y3));

endmodule

module D24_24495(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [3:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h0;
      1: out3 <= 12'hf38;
      2: out3 <= 12'he78;
      3: out3 <= 12'hdc7;
      4: out3 <= 12'hd2c;
      5: out3 <= 12'hcad;
      6: out3 <= 12'hc4e;
      7: out3 <= 12'hc14;
      8: out3 <= 12'hc00;
      9: out3 <= 12'hc14;
      10: out3 <= 12'hc4e;
      11: out3 <= 12'hcad;
      12: out3 <= 12'hd2c;
      13: out3 <= 12'hdc7;
      14: out3 <= 12'he78;
      15: out3 <= 12'hf38;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



module D22_24531(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [3:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h400;
      1: out3 <= 12'h3ec;
      2: out3 <= 12'h3b2;
      3: out3 <= 12'h353;
      4: out3 <= 12'h2d4;
      5: out3 <= 12'h239;
      6: out3 <= 12'h188;
      7: out3 <= 12'hc8;
      8: out3 <= 12'h0;
      9: out3 <= 12'hf38;
      10: out3 <= 12'he78;
      11: out3 <= 12'hdc7;
      12: out3 <= 12'hd2c;
      13: out3 <= 12'hcad;
      14: out3 <= 12'hc4e;
      15: out3 <= 12'hc14;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



// Latency: 8
// Gap: 1
module codeBlock24317(clk, reset, next_in, next_out,
   i6_in,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;
   input [3:0] i6_in;
   reg [3:0] i6;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(7, 1) shiftFIFO_31637(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a353;
   wire signed [11:0] a342;
   wire signed [11:0] a356;
   wire signed [11:0] a346;
   wire signed [11:0] a357;
   wire signed [11:0] a358;
   reg signed [11:0] tm364;
   reg signed [11:0] tm368;
   reg signed [11:0] tm380;
   reg signed [11:0] tm387;
   reg signed [11:0] tm365;
   reg signed [11:0] tm369;
   reg signed [11:0] tm381;
   reg signed [11:0] tm388;
   wire signed [11:0] tm14;
   wire signed [11:0] a347;
   wire signed [11:0] tm15;
   wire signed [11:0] a349;
   reg signed [11:0] tm366;
   reg signed [11:0] tm370;
   reg signed [11:0] tm382;
   reg signed [11:0] tm389;
   reg signed [11:0] tm64;
   reg signed [11:0] tm65;
   reg signed [11:0] tm367;
   reg signed [11:0] tm371;
   reg signed [11:0] tm383;
   reg signed [11:0] tm390;
   reg signed [11:0] tm384;
   reg signed [11:0] tm391;
   wire signed [11:0] a348;
   wire signed [11:0] a350;
   wire signed [11:0] a351;
   wire signed [11:0] a352;
   reg signed [11:0] tm385;
   reg signed [11:0] tm392;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   reg signed [11:0] tm386;
   reg signed [11:0] tm393;


   assign a353 = X0;
   assign a342 = a353;
   assign a356 = X1;
   assign a346 = a356;
   assign a357 = X2;
   assign a358 = X3;
   assign a347 = tm14;
   assign a349 = tm15;
   assign Y0 = tm386;
   assign Y1 = tm393;

   D24_24495 instD24inst0_24495(.addr(i6[3:0]), .out(tm15), .clk(clk));

   D22_24531 instD22inst0_24531(.addr(i6[3:0]), .out(tm14), .clk(clk));

    multfix #(12, 2) m24416(.a(tm64), .b(tm367), .clk(clk), .q_sc(a348), .q_unsc(), .rst(reset));
    multfix #(12, 2) m24438(.a(tm65), .b(tm371), .clk(clk), .q_sc(a350), .q_unsc(), .rst(reset));
    multfix #(12, 2) m24456(.a(tm65), .b(tm367), .clk(clk), .q_sc(a351), .q_unsc(), .rst(reset));
    multfix #(12, 2) m24467(.a(tm64), .b(tm371), .clk(clk), .q_sc(a352), .q_unsc(), .rst(reset));
    subfxp #(12, 1) sub24445(.a(a348), .b(a350), .clk(clk), .q(Y2));    // 6
    addfxp #(12, 1) add24474(.a(a351), .b(a352), .clk(clk), .q(Y3));    // 6


   always @(posedge clk) begin
      if (reset == 1) begin
         tm64 <= 0;
         tm367 <= 0;
         tm65 <= 0;
         tm371 <= 0;
         tm65 <= 0;
         tm367 <= 0;
         tm64 <= 0;
         tm371 <= 0;
      end
      else begin
         i6 <= i6_in;
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
         tm364 <= a357;
         tm368 <= a358;
         tm380 <= a342;
         tm387 <= a346;
         tm365 <= tm364;
         tm369 <= tm368;
         tm381 <= tm380;
         tm388 <= tm387;
         tm366 <= tm365;
         tm370 <= tm369;
         tm382 <= tm381;
         tm389 <= tm388;
         tm64 <= a347;
         tm65 <= a349;
         tm367 <= tm366;
         tm371 <= tm370;
         tm383 <= tm382;
         tm390 <= tm389;
         tm384 <= tm383;
         tm391 <= tm390;
         tm385 <= tm384;
         tm392 <= tm391;
         tm386 <= tm385;
         tm393 <= tm392;
      end
   end
endmodule

// Latency: 2
// Gap: 1
module codeBlock24554(clk, reset, next_in, next_out,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(1, 1) shiftFIFO_31640(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a309;
   wire signed [11:0] a310;
   wire signed [11:0] a311;
   wire signed [11:0] a312;
   wire signed [12:0] tm250;
   wire signed [12:0] tm251;
   wire signed [12:0] tm252;
   wire signed [12:0] tm253;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   wire signed [11:0] t141;
   wire signed [11:0] t142;
   wire signed [11:0] t143;
   wire signed [11:0] t144;


   assign a309 = X0;
   assign a310 = X2;
   assign a311 = X1;
   assign a312 = X3;
   assign Y0 = t141;
   assign Y1 = t142;
   assign Y2 = t143;
   assign Y3 = t144;
   assign t141 = tm250[12:1];
   assign t142 = tm251[12:1];
   assign t143 = tm252[12:1];
   assign t144 = tm253[12:1];

    addfxp #(13, 1) add24566(.a({{1{a309[11]}}, a309}), .b({{1{a310[11]}}, a310}), .clk(clk), .q(tm250));    // 0
    addfxp #(13, 1) add24581(.a({{1{a311[11]}}, a311}), .b({{1{a312[11]}}, a312}), .clk(clk), .q(tm251));    // 0
    subfxp #(13, 1) sub24596(.a({{1{a309[11]}}, a309}), .b({{1{a310[11]}}, a310}), .clk(clk), .q(tm252));    // 0
    subfxp #(13, 1) sub24611(.a({{1{a311[11]}}, a311}), .b({{1{a312[11]}}, a312}), .clk(clk), .q(tm253));    // 0


   always @(posedge clk) begin
      if (reset == 1) begin
      end
      else begin
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
      end
   end
endmodule

// Latency: 19
// Gap: 32
module rc24635(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm24633 instPerm31641(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 19
// Gap: 32
module perm24633(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 32;
   parameter logDepth = 5;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[5] ^ addr0[0];
   assign inAddr0[0] = addr0[1];
   assign inAddr0[1] = addr0[2];
   assign inAddr0[2] = addr0[3];
   assign inAddr0[3] = addr0[4];
   assign inAddr0[4] = addr0[0];
   assign outBank0[0] = addr0b[5] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outAddr0[1] = addr0b[2];
   assign outAddr0[2] = addr0b[3];
   assign outAddr0[3] = addr0b[4];
   assign outAddr0[4] = addr0b[5];
   assign outBank_a0[0] = addr0c[5] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];
   assign outAddr_a0[1] = addr0c[2];
   assign outAddr_a0[2] = addr0c[3];
   assign outAddr_a0[3] = addr0c[4];
   assign outAddr_a0[4] = addr0c[5];

   assign inBank1[0] = addr1[5] ^ addr1[0];
   assign inAddr1[0] = addr1[1];
   assign inAddr1[1] = addr1[2];
   assign inAddr1[2] = addr1[3];
   assign inAddr1[3] = addr1[4];
   assign inAddr1[4] = addr1[0];
   assign outBank1[0] = addr1b[5] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outAddr1[1] = addr1b[2];
   assign outAddr1[2] = addr1b[3];
   assign outAddr1[3] = addr1b[4];
   assign outAddr1[4] = addr1b[5];
   assign outBank_a1[0] = addr1c[5] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];
   assign outAddr_a1[1] = addr1c[2];
   assign outAddr_a1[2] = addr1c[3];
   assign outAddr_a1[3] = addr1c[4];
   assign outAddr_a1[4] = addr1c[5];

   nextReg #(17, 5) nextReg_31646(.X(next), .Y(next0), .reset(reset), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31649(.X(next0), .Y(next_out), .clk(clk));


   memArray64_24633 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
        outCount_for_rd_data <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 16)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
         if (inCount == 18)
            outCount_for_rd_data <= 0;
         else
            outCount_for_rd_data <= outCount_for_rd_data+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 16) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 31) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 16)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[4];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[4];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[4];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray64_24633(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 32;
   parameter logDepth = 5;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   nextReg #(32, 5) nextReg_31654(.X(next), .Y(next0), .reset(reset), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule

// Latency: 8
// Gap: 32
module DirSum_24936(clk, reset, next, next_out,
      X0, Y0,
      X1, Y1,
      X2, Y2,
      X3, Y3);

   output next_out;
   input clk, reset, next;

   reg [4:0] i5;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   always @(posedge clk) begin
      if (reset == 1) begin
         i5 <= 0;
      end
      else begin
         if (next == 1)
            i5 <= 0;
         else if (i5 == 31)
            i5 <= 0;
         else
            i5 <= i5 + 1;
      end
   end

   codeBlock24638 codeBlockIsnt31659(.clk(clk), .reset(reset), .next_in(next), .next_out(next_out),
.i5_in(i5),
       .X0_in(X0), .Y0(Y0),
       .X1_in(X1), .Y1(Y1),
       .X2_in(X2), .Y2(Y2),
       .X3_in(X3), .Y3(Y3));

endmodule

module D20_24832(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [4:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h0;
      1: out3 <= 12'hf9c;
      2: out3 <= 12'hf38;
      3: out3 <= 12'hed7;
      4: out3 <= 12'he78;
      5: out3 <= 12'he1d;
      6: out3 <= 12'hdc7;
      7: out3 <= 12'hd76;
      8: out3 <= 12'hd2c;
      9: out3 <= 12'hce8;
      10: out3 <= 12'hcad;
      11: out3 <= 12'hc79;
      12: out3 <= 12'hc4e;
      13: out3 <= 12'hc2c;
      14: out3 <= 12'hc14;
      15: out3 <= 12'hc05;
      16: out3 <= 12'hc00;
      17: out3 <= 12'hc05;
      18: out3 <= 12'hc14;
      19: out3 <= 12'hc2c;
      20: out3 <= 12'hc4e;
      21: out3 <= 12'hc79;
      22: out3 <= 12'hcad;
      23: out3 <= 12'hce8;
      24: out3 <= 12'hd2c;
      25: out3 <= 12'hd76;
      26: out3 <= 12'hdc7;
      27: out3 <= 12'he1d;
      28: out3 <= 12'he78;
      29: out3 <= 12'hed7;
      30: out3 <= 12'hf38;
      31: out3 <= 12'hf9c;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



module D18_24900(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [4:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h400;
      1: out3 <= 12'h3fb;
      2: out3 <= 12'h3ec;
      3: out3 <= 12'h3d4;
      4: out3 <= 12'h3b2;
      5: out3 <= 12'h387;
      6: out3 <= 12'h353;
      7: out3 <= 12'h318;
      8: out3 <= 12'h2d4;
      9: out3 <= 12'h28a;
      10: out3 <= 12'h239;
      11: out3 <= 12'h1e3;
      12: out3 <= 12'h188;
      13: out3 <= 12'h129;
      14: out3 <= 12'hc8;
      15: out3 <= 12'h64;
      16: out3 <= 12'h0;
      17: out3 <= 12'hf9c;
      18: out3 <= 12'hf38;
      19: out3 <= 12'hed7;
      20: out3 <= 12'he78;
      21: out3 <= 12'he1d;
      22: out3 <= 12'hdc7;
      23: out3 <= 12'hd76;
      24: out3 <= 12'hd2c;
      25: out3 <= 12'hce8;
      26: out3 <= 12'hcad;
      27: out3 <= 12'hc79;
      28: out3 <= 12'hc4e;
      29: out3 <= 12'hc2c;
      30: out3 <= 12'hc14;
      31: out3 <= 12'hc05;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



// Latency: 8
// Gap: 1
module codeBlock24638(clk, reset, next_in, next_out,
   i5_in,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;
   input [4:0] i5_in;
   reg [4:0] i5;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(7, 1) shiftFIFO_31662(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a293;
   wire signed [11:0] a282;
   wire signed [11:0] a296;
   wire signed [11:0] a286;
   wire signed [11:0] a297;
   wire signed [11:0] a298;
   reg signed [11:0] tm394;
   reg signed [11:0] tm398;
   reg signed [11:0] tm410;
   reg signed [11:0] tm417;
   reg signed [11:0] tm395;
   reg signed [11:0] tm399;
   reg signed [11:0] tm411;
   reg signed [11:0] tm418;
   wire signed [11:0] tm18;
   wire signed [11:0] a287;
   wire signed [11:0] tm19;
   wire signed [11:0] a289;
   reg signed [11:0] tm396;
   reg signed [11:0] tm400;
   reg signed [11:0] tm412;
   reg signed [11:0] tm419;
   reg signed [11:0] tm72;
   reg signed [11:0] tm73;
   reg signed [11:0] tm397;
   reg signed [11:0] tm401;
   reg signed [11:0] tm413;
   reg signed [11:0] tm420;
   reg signed [11:0] tm414;
   reg signed [11:0] tm421;
   wire signed [11:0] a288;
   wire signed [11:0] a290;
   wire signed [11:0] a291;
   wire signed [11:0] a292;
   reg signed [11:0] tm415;
   reg signed [11:0] tm422;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   reg signed [11:0] tm416;
   reg signed [11:0] tm423;


   assign a293 = X0;
   assign a282 = a293;
   assign a296 = X1;
   assign a286 = a296;
   assign a297 = X2;
   assign a298 = X3;
   assign a287 = tm18;
   assign a289 = tm19;
   assign Y0 = tm416;
   assign Y1 = tm423;

   D20_24832 instD20inst0_24832(.addr(i5[4:0]), .out(tm19), .clk(clk));

   D18_24900 instD18inst0_24900(.addr(i5[4:0]), .out(tm18), .clk(clk));

    multfix #(12, 2) m24737(.a(tm72), .b(tm397), .clk(clk), .q_sc(a288), .q_unsc(), .rst(reset));
    multfix #(12, 2) m24759(.a(tm73), .b(tm401), .clk(clk), .q_sc(a290), .q_unsc(), .rst(reset));
    multfix #(12, 2) m24777(.a(tm73), .b(tm397), .clk(clk), .q_sc(a291), .q_unsc(), .rst(reset));
    multfix #(12, 2) m24788(.a(tm72), .b(tm401), .clk(clk), .q_sc(a292), .q_unsc(), .rst(reset));
    subfxp #(12, 1) sub24766(.a(a288), .b(a290), .clk(clk), .q(Y2));    // 6
    addfxp #(12, 1) add24795(.a(a291), .b(a292), .clk(clk), .q(Y3));    // 6


   always @(posedge clk) begin
      if (reset == 1) begin
         tm72 <= 0;
         tm397 <= 0;
         tm73 <= 0;
         tm401 <= 0;
         tm73 <= 0;
         tm397 <= 0;
         tm72 <= 0;
         tm401 <= 0;
      end
      else begin
         i5 <= i5_in;
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
         tm394 <= a297;
         tm398 <= a298;
         tm410 <= a282;
         tm417 <= a286;
         tm395 <= tm394;
         tm399 <= tm398;
         tm411 <= tm410;
         tm418 <= tm417;
         tm396 <= tm395;
         tm400 <= tm399;
         tm412 <= tm411;
         tm419 <= tm418;
         tm72 <= a287;
         tm73 <= a289;
         tm397 <= tm396;
         tm401 <= tm400;
         tm413 <= tm412;
         tm420 <= tm419;
         tm414 <= tm413;
         tm421 <= tm420;
         tm415 <= tm414;
         tm422 <= tm421;
         tm416 <= tm415;
         tm423 <= tm422;
      end
   end
endmodule

// Latency: 2
// Gap: 1
module codeBlock24939(clk, reset, next_in, next_out,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(1, 1) shiftFIFO_31665(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a249;
   wire signed [11:0] a250;
   wire signed [11:0] a251;
   wire signed [11:0] a252;
   wire signed [12:0] tm254;
   wire signed [12:0] tm255;
   wire signed [12:0] tm256;
   wire signed [12:0] tm257;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   wire signed [11:0] t117;
   wire signed [11:0] t118;
   wire signed [11:0] t119;
   wire signed [11:0] t120;


   assign a249 = X0;
   assign a250 = X2;
   assign a251 = X1;
   assign a252 = X3;
   assign Y0 = t117;
   assign Y1 = t118;
   assign Y2 = t119;
   assign Y3 = t120;
   assign t117 = tm254[12:1];
   assign t118 = tm255[12:1];
   assign t119 = tm256[12:1];
   assign t120 = tm257[12:1];

    addfxp #(13, 1) add24951(.a({{1{a249[11]}}, a249}), .b({{1{a250[11]}}, a250}), .clk(clk), .q(tm254));    // 0
    addfxp #(13, 1) add24966(.a({{1{a251[11]}}, a251}), .b({{1{a252[11]}}, a252}), .clk(clk), .q(tm255));    // 0
    subfxp #(13, 1) sub24981(.a({{1{a249[11]}}, a249}), .b({{1{a250[11]}}, a250}), .clk(clk), .q(tm256));    // 0
    subfxp #(13, 1) sub24996(.a({{1{a251[11]}}, a251}), .b({{1{a252[11]}}, a252}), .clk(clk), .q(tm257));    // 0


   always @(posedge clk) begin
      if (reset == 1) begin
      end
      else begin
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
      end
   end
endmodule

// Latency: 35
// Gap: 64
module rc25020(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm25018 instPerm31666(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 35
// Gap: 64
module perm25018(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 64;
   parameter logDepth = 6;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[6] ^ addr0[0];
   assign inAddr0[0] = addr0[1];
   assign inAddr0[1] = addr0[2];
   assign inAddr0[2] = addr0[3];
   assign inAddr0[3] = addr0[4];
   assign inAddr0[4] = addr0[5];
   assign inAddr0[5] = addr0[0];
   assign outBank0[0] = addr0b[6] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outAddr0[1] = addr0b[2];
   assign outAddr0[2] = addr0b[3];
   assign outAddr0[3] = addr0b[4];
   assign outAddr0[4] = addr0b[5];
   assign outAddr0[5] = addr0b[6];
   assign outBank_a0[0] = addr0c[6] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];
   assign outAddr_a0[1] = addr0c[2];
   assign outAddr_a0[2] = addr0c[3];
   assign outAddr_a0[3] = addr0c[4];
   assign outAddr_a0[4] = addr0c[5];
   assign outAddr_a0[5] = addr0c[6];

   assign inBank1[0] = addr1[6] ^ addr1[0];
   assign inAddr1[0] = addr1[1];
   assign inAddr1[1] = addr1[2];
   assign inAddr1[2] = addr1[3];
   assign inAddr1[3] = addr1[4];
   assign inAddr1[4] = addr1[5];
   assign inAddr1[5] = addr1[0];
   assign outBank1[0] = addr1b[6] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outAddr1[1] = addr1b[2];
   assign outAddr1[2] = addr1b[3];
   assign outAddr1[3] = addr1b[4];
   assign outAddr1[4] = addr1b[5];
   assign outAddr1[5] = addr1b[6];
   assign outBank_a1[0] = addr1c[6] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];
   assign outAddr_a1[1] = addr1c[2];
   assign outAddr_a1[2] = addr1c[3];
   assign outAddr_a1[3] = addr1c[4];
   assign outAddr_a1[4] = addr1c[5];
   assign outAddr_a1[5] = addr1c[6];

   nextReg #(33, 6) nextReg_31671(.X(next), .Y(next0), .reset(reset), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31674(.X(next0), .Y(next_out), .clk(clk));


   memArray128_25018 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
        outCount_for_rd_data <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 32)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
         if (inCount == 34)
            outCount_for_rd_data <= 0;
         else
            outCount_for_rd_data <= outCount_for_rd_data+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 32) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 63) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 32)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[5];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[5];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[5];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray128_25018(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 64;
   parameter logDepth = 6;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   nextReg #(64, 6) nextReg_31679(.X(next), .Y(next0), .reset(reset), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule

// Latency: 8
// Gap: 64
module DirSum_25449(clk, reset, next, next_out,
      X0, Y0,
      X1, Y1,
      X2, Y2,
      X3, Y3);

   output next_out;
   input clk, reset, next;

   reg [5:0] i4;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   always @(posedge clk) begin
      if (reset == 1) begin
         i4 <= 0;
      end
      else begin
         if (next == 1)
            i4 <= 0;
         else if (i4 == 63)
            i4 <= 0;
         else
            i4 <= i4 + 1;
      end
   end

   codeBlock25023 codeBlockIsnt31684(.clk(clk), .reset(reset), .next_in(next), .next_out(next_out),
.i4_in(i4),
       .X0_in(X0), .Y0(Y0),
       .X1_in(X1), .Y1(Y1),
       .X2_in(X2), .Y2(Y2),
       .X3_in(X3), .Y3(Y3));

endmodule

module D16_25249(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [5:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h0;
      1: out3 <= 12'hfce;
      2: out3 <= 12'hf9c;
      3: out3 <= 12'hf6a;
      4: out3 <= 12'hf38;
      5: out3 <= 12'hf07;
      6: out3 <= 12'hed7;
      7: out3 <= 12'hea7;
      8: out3 <= 12'he78;
      9: out3 <= 12'he4a;
      10: out3 <= 12'he1d;
      11: out3 <= 12'hdf2;
      12: out3 <= 12'hdc7;
      13: out3 <= 12'hd9e;
      14: out3 <= 12'hd76;
      15: out3 <= 12'hd50;
      16: out3 <= 12'hd2c;
      17: out3 <= 12'hd09;
      18: out3 <= 12'hce8;
      19: out3 <= 12'hcca;
      20: out3 <= 12'hcad;
      21: out3 <= 12'hc92;
      22: out3 <= 12'hc79;
      23: out3 <= 12'hc62;
      24: out3 <= 12'hc4e;
      25: out3 <= 12'hc3c;
      26: out3 <= 12'hc2c;
      27: out3 <= 12'hc1f;
      28: out3 <= 12'hc14;
      29: out3 <= 12'hc0b;
      30: out3 <= 12'hc05;
      31: out3 <= 12'hc01;
      32: out3 <= 12'hc00;
      33: out3 <= 12'hc01;
      34: out3 <= 12'hc05;
      35: out3 <= 12'hc0b;
      36: out3 <= 12'hc14;
      37: out3 <= 12'hc1f;
      38: out3 <= 12'hc2c;
      39: out3 <= 12'hc3c;
      40: out3 <= 12'hc4e;
      41: out3 <= 12'hc62;
      42: out3 <= 12'hc79;
      43: out3 <= 12'hc92;
      44: out3 <= 12'hcad;
      45: out3 <= 12'hcca;
      46: out3 <= 12'hce8;
      47: out3 <= 12'hd09;
      48: out3 <= 12'hd2c;
      49: out3 <= 12'hd50;
      50: out3 <= 12'hd76;
      51: out3 <= 12'hd9e;
      52: out3 <= 12'hdc7;
      53: out3 <= 12'hdf2;
      54: out3 <= 12'he1d;
      55: out3 <= 12'he4a;
      56: out3 <= 12'he78;
      57: out3 <= 12'hea7;
      58: out3 <= 12'hed7;
      59: out3 <= 12'hf07;
      60: out3 <= 12'hf38;
      61: out3 <= 12'hf6a;
      62: out3 <= 12'hf9c;
      63: out3 <= 12'hfce;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



module D14_25447(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [5:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h400;
      1: out3 <= 12'h3ff;
      2: out3 <= 12'h3fb;
      3: out3 <= 12'h3f5;
      4: out3 <= 12'h3ec;
      5: out3 <= 12'h3e1;
      6: out3 <= 12'h3d4;
      7: out3 <= 12'h3c4;
      8: out3 <= 12'h3b2;
      9: out3 <= 12'h39e;
      10: out3 <= 12'h387;
      11: out3 <= 12'h36e;
      12: out3 <= 12'h353;
      13: out3 <= 12'h336;
      14: out3 <= 12'h318;
      15: out3 <= 12'h2f7;
      16: out3 <= 12'h2d4;
      17: out3 <= 12'h2b0;
      18: out3 <= 12'h28a;
      19: out3 <= 12'h262;
      20: out3 <= 12'h239;
      21: out3 <= 12'h20e;
      22: out3 <= 12'h1e3;
      23: out3 <= 12'h1b6;
      24: out3 <= 12'h188;
      25: out3 <= 12'h159;
      26: out3 <= 12'h129;
      27: out3 <= 12'hf9;
      28: out3 <= 12'hc8;
      29: out3 <= 12'h96;
      30: out3 <= 12'h64;
      31: out3 <= 12'h32;
      32: out3 <= 12'h0;
      33: out3 <= 12'hfce;
      34: out3 <= 12'hf9c;
      35: out3 <= 12'hf6a;
      36: out3 <= 12'hf38;
      37: out3 <= 12'hf07;
      38: out3 <= 12'hed7;
      39: out3 <= 12'hea7;
      40: out3 <= 12'he78;
      41: out3 <= 12'he4a;
      42: out3 <= 12'he1d;
      43: out3 <= 12'hdf2;
      44: out3 <= 12'hdc7;
      45: out3 <= 12'hd9e;
      46: out3 <= 12'hd76;
      47: out3 <= 12'hd50;
      48: out3 <= 12'hd2c;
      49: out3 <= 12'hd09;
      50: out3 <= 12'hce8;
      51: out3 <= 12'hcca;
      52: out3 <= 12'hcad;
      53: out3 <= 12'hc92;
      54: out3 <= 12'hc79;
      55: out3 <= 12'hc62;
      56: out3 <= 12'hc4e;
      57: out3 <= 12'hc3c;
      58: out3 <= 12'hc2c;
      59: out3 <= 12'hc1f;
      60: out3 <= 12'hc14;
      61: out3 <= 12'hc0b;
      62: out3 <= 12'hc05;
      63: out3 <= 12'hc01;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



// Latency: 8
// Gap: 1
module codeBlock25023(clk, reset, next_in, next_out,
   i4_in,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;
   input [5:0] i4_in;
   reg [5:0] i4;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(7, 1) shiftFIFO_31687(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a233;
   wire signed [11:0] a222;
   wire signed [11:0] a236;
   wire signed [11:0] a226;
   wire signed [11:0] a237;
   wire signed [11:0] a238;
   reg signed [11:0] tm424;
   reg signed [11:0] tm428;
   reg signed [11:0] tm440;
   reg signed [11:0] tm447;
   reg signed [11:0] tm425;
   reg signed [11:0] tm429;
   reg signed [11:0] tm441;
   reg signed [11:0] tm448;
   wire signed [11:0] tm22;
   wire signed [11:0] a227;
   wire signed [11:0] tm23;
   wire signed [11:0] a229;
   reg signed [11:0] tm426;
   reg signed [11:0] tm430;
   reg signed [11:0] tm442;
   reg signed [11:0] tm449;
   reg signed [11:0] tm80;
   reg signed [11:0] tm81;
   reg signed [11:0] tm427;
   reg signed [11:0] tm431;
   reg signed [11:0] tm443;
   reg signed [11:0] tm450;
   reg signed [11:0] tm444;
   reg signed [11:0] tm451;
   wire signed [11:0] a228;
   wire signed [11:0] a230;
   wire signed [11:0] a231;
   wire signed [11:0] a232;
   reg signed [11:0] tm445;
   reg signed [11:0] tm452;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   reg signed [11:0] tm446;
   reg signed [11:0] tm453;


   assign a233 = X0;
   assign a222 = a233;
   assign a236 = X1;
   assign a226 = a236;
   assign a237 = X2;
   assign a238 = X3;
   assign a227 = tm22;
   assign a229 = tm23;
   assign Y0 = tm446;
   assign Y1 = tm453;

   D16_25249 instD16inst0_25249(.addr(i4[5:0]), .out(tm23), .clk(clk));

   D14_25447 instD14inst0_25447(.addr(i4[5:0]), .out(tm22), .clk(clk));

    multfix #(12, 2) m25122(.a(tm80), .b(tm427), .clk(clk), .q_sc(a228), .q_unsc(), .rst(reset));
    multfix #(12, 2) m25144(.a(tm81), .b(tm431), .clk(clk), .q_sc(a230), .q_unsc(), .rst(reset));
    multfix #(12, 2) m25162(.a(tm81), .b(tm427), .clk(clk), .q_sc(a231), .q_unsc(), .rst(reset));
    multfix #(12, 2) m25173(.a(tm80), .b(tm431), .clk(clk), .q_sc(a232), .q_unsc(), .rst(reset));
    subfxp #(12, 1) sub25151(.a(a228), .b(a230), .clk(clk), .q(Y2));    // 6
    addfxp #(12, 1) add25180(.a(a231), .b(a232), .clk(clk), .q(Y3));    // 6


   always @(posedge clk) begin
      if (reset == 1) begin
         tm80 <= 0;
         tm427 <= 0;
         tm81 <= 0;
         tm431 <= 0;
         tm81 <= 0;
         tm427 <= 0;
         tm80 <= 0;
         tm431 <= 0;
      end
      else begin
         i4 <= i4_in;
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
         tm424 <= a237;
         tm428 <= a238;
         tm440 <= a222;
         tm447 <= a226;
         tm425 <= tm424;
         tm429 <= tm428;
         tm441 <= tm440;
         tm448 <= tm447;
         tm426 <= tm425;
         tm430 <= tm429;
         tm442 <= tm441;
         tm449 <= tm448;
         tm80 <= a227;
         tm81 <= a229;
         tm427 <= tm426;
         tm431 <= tm430;
         tm443 <= tm442;
         tm450 <= tm449;
         tm444 <= tm443;
         tm451 <= tm450;
         tm445 <= tm444;
         tm452 <= tm451;
         tm446 <= tm445;
         tm453 <= tm452;
      end
   end
endmodule

// Latency: 2
// Gap: 1
module codeBlock25452(clk, reset, next_in, next_out,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(1, 1) shiftFIFO_31690(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a189;
   wire signed [11:0] a190;
   wire signed [11:0] a191;
   wire signed [11:0] a192;
   wire signed [12:0] tm258;
   wire signed [12:0] tm259;
   wire signed [12:0] tm260;
   wire signed [12:0] tm261;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   wire signed [11:0] t93;
   wire signed [11:0] t94;
   wire signed [11:0] t95;
   wire signed [11:0] t96;


   assign a189 = X0;
   assign a190 = X2;
   assign a191 = X1;
   assign a192 = X3;
   assign Y0 = t93;
   assign Y1 = t94;
   assign Y2 = t95;
   assign Y3 = t96;
   assign t93 = tm258[12:1];
   assign t94 = tm259[12:1];
   assign t95 = tm260[12:1];
   assign t96 = tm261[12:1];

    addfxp #(13, 1) add25464(.a({{1{a189[11]}}, a189}), .b({{1{a190[11]}}, a190}), .clk(clk), .q(tm258));    // 0
    addfxp #(13, 1) add25479(.a({{1{a191[11]}}, a191}), .b({{1{a192[11]}}, a192}), .clk(clk), .q(tm259));    // 0
    subfxp #(13, 1) sub25494(.a({{1{a189[11]}}, a189}), .b({{1{a190[11]}}, a190}), .clk(clk), .q(tm260));    // 0
    subfxp #(13, 1) sub25509(.a({{1{a191[11]}}, a191}), .b({{1{a192[11]}}, a192}), .clk(clk), .q(tm261));    // 0


   always @(posedge clk) begin
      if (reset == 1) begin
      end
      else begin
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
      end
   end
endmodule

// Latency: 67
// Gap: 128
module rc25533(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm25531 instPerm31691(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 67
// Gap: 128
module perm25531(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 128;
   parameter logDepth = 7;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[7] ^ addr0[0];
   assign inAddr0[0] = addr0[1];
   assign inAddr0[1] = addr0[2];
   assign inAddr0[2] = addr0[3];
   assign inAddr0[3] = addr0[4];
   assign inAddr0[4] = addr0[5];
   assign inAddr0[5] = addr0[6];
   assign inAddr0[6] = addr0[0];
   assign outBank0[0] = addr0b[7] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outAddr0[1] = addr0b[2];
   assign outAddr0[2] = addr0b[3];
   assign outAddr0[3] = addr0b[4];
   assign outAddr0[4] = addr0b[5];
   assign outAddr0[5] = addr0b[6];
   assign outAddr0[6] = addr0b[7];
   assign outBank_a0[0] = addr0c[7] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];
   assign outAddr_a0[1] = addr0c[2];
   assign outAddr_a0[2] = addr0c[3];
   assign outAddr_a0[3] = addr0c[4];
   assign outAddr_a0[4] = addr0c[5];
   assign outAddr_a0[5] = addr0c[6];
   assign outAddr_a0[6] = addr0c[7];

   assign inBank1[0] = addr1[7] ^ addr1[0];
   assign inAddr1[0] = addr1[1];
   assign inAddr1[1] = addr1[2];
   assign inAddr1[2] = addr1[3];
   assign inAddr1[3] = addr1[4];
   assign inAddr1[4] = addr1[5];
   assign inAddr1[5] = addr1[6];
   assign inAddr1[6] = addr1[0];
   assign outBank1[0] = addr1b[7] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outAddr1[1] = addr1b[2];
   assign outAddr1[2] = addr1b[3];
   assign outAddr1[3] = addr1b[4];
   assign outAddr1[4] = addr1b[5];
   assign outAddr1[5] = addr1b[6];
   assign outAddr1[6] = addr1b[7];
   assign outBank_a1[0] = addr1c[7] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];
   assign outAddr_a1[1] = addr1c[2];
   assign outAddr_a1[2] = addr1c[3];
   assign outAddr_a1[3] = addr1c[4];
   assign outAddr_a1[4] = addr1c[5];
   assign outAddr_a1[5] = addr1c[6];
   assign outAddr_a1[6] = addr1c[7];

   nextReg #(65, 7) nextReg_31696(.X(next), .Y(next0), .reset(reset), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31699(.X(next0), .Y(next_out), .clk(clk));


   memArray256_25531 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
        outCount_for_rd_data <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 64)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
         if (inCount == 66)
            outCount_for_rd_data <= 0;
         else
            outCount_for_rd_data <= outCount_for_rd_data+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 64) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 127) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 64)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[6];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[6];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[6];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray256_25531(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 128;
   parameter logDepth = 7;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   nextReg #(128, 7) nextReg_31704(.X(next), .Y(next0), .reset(reset), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule

// Latency: 8
// Gap: 128
module DirSum_26218(clk, reset, next, next_out,
      X0, Y0,
      X1, Y1,
      X2, Y2,
      X3, Y3);

   output next_out;
   input clk, reset, next;

   reg [6:0] i3;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   always @(posedge clk) begin
      if (reset == 1) begin
         i3 <= 0;
      end
      else begin
         if (next == 1)
            i3 <= 0;
         else if (i3 == 127)
            i3 <= 0;
         else
            i3 <= i3 + 1;
      end
   end

   codeBlock25536 codeBlockIsnt31709(.clk(clk), .reset(reset), .next_in(next), .next_out(next_out),
.i3_in(i3),
       .X0_in(X0), .Y0(Y0),
       .X1_in(X1), .Y1(Y1),
       .X2_in(X2), .Y2(Y2),
       .X3_in(X3), .Y3(Y3));

endmodule

module D10_25956(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [6:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h400;
      1: out3 <= 12'h400;
      2: out3 <= 12'h3ff;
      3: out3 <= 12'h3fd;
      4: out3 <= 12'h3fb;
      5: out3 <= 12'h3f8;
      6: out3 <= 12'h3f5;
      7: out3 <= 12'h3f1;
      8: out3 <= 12'h3ec;
      9: out3 <= 12'h3e7;
      10: out3 <= 12'h3e1;
      11: out3 <= 12'h3db;
      12: out3 <= 12'h3d4;
      13: out3 <= 12'h3cc;
      14: out3 <= 12'h3c4;
      15: out3 <= 12'h3bb;
      16: out3 <= 12'h3b2;
      17: out3 <= 12'h3a8;
      18: out3 <= 12'h39e;
      19: out3 <= 12'h393;
      20: out3 <= 12'h387;
      21: out3 <= 12'h37b;
      22: out3 <= 12'h36e;
      23: out3 <= 12'h361;
      24: out3 <= 12'h353;
      25: out3 <= 12'h345;
      26: out3 <= 12'h336;
      27: out3 <= 12'h327;
      28: out3 <= 12'h318;
      29: out3 <= 12'h307;
      30: out3 <= 12'h2f7;
      31: out3 <= 12'h2e6;
      32: out3 <= 12'h2d4;
      33: out3 <= 12'h2c2;
      34: out3 <= 12'h2b0;
      35: out3 <= 12'h29d;
      36: out3 <= 12'h28a;
      37: out3 <= 12'h276;
      38: out3 <= 12'h262;
      39: out3 <= 12'h24e;
      40: out3 <= 12'h239;
      41: out3 <= 12'h224;
      42: out3 <= 12'h20e;
      43: out3 <= 12'h1f9;
      44: out3 <= 12'h1e3;
      45: out3 <= 12'h1cc;
      46: out3 <= 12'h1b6;
      47: out3 <= 12'h19f;
      48: out3 <= 12'h188;
      49: out3 <= 12'h171;
      50: out3 <= 12'h159;
      51: out3 <= 12'h141;
      52: out3 <= 12'h129;
      53: out3 <= 12'h111;
      54: out3 <= 12'hf9;
      55: out3 <= 12'he0;
      56: out3 <= 12'hc8;
      57: out3 <= 12'haf;
      58: out3 <= 12'h96;
      59: out3 <= 12'h7d;
      60: out3 <= 12'h64;
      61: out3 <= 12'h4b;
      62: out3 <= 12'h32;
      63: out3 <= 12'h19;
      64: out3 <= 12'h0;
      65: out3 <= 12'hfe7;
      66: out3 <= 12'hfce;
      67: out3 <= 12'hfb5;
      68: out3 <= 12'hf9c;
      69: out3 <= 12'hf83;
      70: out3 <= 12'hf6a;
      71: out3 <= 12'hf51;
      72: out3 <= 12'hf38;
      73: out3 <= 12'hf20;
      74: out3 <= 12'hf07;
      75: out3 <= 12'heef;
      76: out3 <= 12'hed7;
      77: out3 <= 12'hebf;
      78: out3 <= 12'hea7;
      79: out3 <= 12'he8f;
      80: out3 <= 12'he78;
      81: out3 <= 12'he61;
      82: out3 <= 12'he4a;
      83: out3 <= 12'he34;
      84: out3 <= 12'he1d;
      85: out3 <= 12'he07;
      86: out3 <= 12'hdf2;
      87: out3 <= 12'hddc;
      88: out3 <= 12'hdc7;
      89: out3 <= 12'hdb2;
      90: out3 <= 12'hd9e;
      91: out3 <= 12'hd8a;
      92: out3 <= 12'hd76;
      93: out3 <= 12'hd63;
      94: out3 <= 12'hd50;
      95: out3 <= 12'hd3e;
      96: out3 <= 12'hd2c;
      97: out3 <= 12'hd1a;
      98: out3 <= 12'hd09;
      99: out3 <= 12'hcf9;
      100: out3 <= 12'hce8;
      101: out3 <= 12'hcd9;
      102: out3 <= 12'hcca;
      103: out3 <= 12'hcbb;
      104: out3 <= 12'hcad;
      105: out3 <= 12'hc9f;
      106: out3 <= 12'hc92;
      107: out3 <= 12'hc85;
      108: out3 <= 12'hc79;
      109: out3 <= 12'hc6d;
      110: out3 <= 12'hc62;
      111: out3 <= 12'hc58;
      112: out3 <= 12'hc4e;
      113: out3 <= 12'hc45;
      114: out3 <= 12'hc3c;
      115: out3 <= 12'hc34;
      116: out3 <= 12'hc2c;
      117: out3 <= 12'hc25;
      118: out3 <= 12'hc1f;
      119: out3 <= 12'hc19;
      120: out3 <= 12'hc14;
      121: out3 <= 12'hc0f;
      122: out3 <= 12'hc0b;
      123: out3 <= 12'hc08;
      124: out3 <= 12'hc05;
      125: out3 <= 12'hc03;
      126: out3 <= 12'hc01;
      127: out3 <= 12'hc00;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



module D12_26216(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [6:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h0;
      1: out3 <= 12'hfe7;
      2: out3 <= 12'hfce;
      3: out3 <= 12'hfb5;
      4: out3 <= 12'hf9c;
      5: out3 <= 12'hf83;
      6: out3 <= 12'hf6a;
      7: out3 <= 12'hf51;
      8: out3 <= 12'hf38;
      9: out3 <= 12'hf20;
      10: out3 <= 12'hf07;
      11: out3 <= 12'heef;
      12: out3 <= 12'hed7;
      13: out3 <= 12'hebf;
      14: out3 <= 12'hea7;
      15: out3 <= 12'he8f;
      16: out3 <= 12'he78;
      17: out3 <= 12'he61;
      18: out3 <= 12'he4a;
      19: out3 <= 12'he34;
      20: out3 <= 12'he1d;
      21: out3 <= 12'he07;
      22: out3 <= 12'hdf2;
      23: out3 <= 12'hddc;
      24: out3 <= 12'hdc7;
      25: out3 <= 12'hdb2;
      26: out3 <= 12'hd9e;
      27: out3 <= 12'hd8a;
      28: out3 <= 12'hd76;
      29: out3 <= 12'hd63;
      30: out3 <= 12'hd50;
      31: out3 <= 12'hd3e;
      32: out3 <= 12'hd2c;
      33: out3 <= 12'hd1a;
      34: out3 <= 12'hd09;
      35: out3 <= 12'hcf9;
      36: out3 <= 12'hce8;
      37: out3 <= 12'hcd9;
      38: out3 <= 12'hcca;
      39: out3 <= 12'hcbb;
      40: out3 <= 12'hcad;
      41: out3 <= 12'hc9f;
      42: out3 <= 12'hc92;
      43: out3 <= 12'hc85;
      44: out3 <= 12'hc79;
      45: out3 <= 12'hc6d;
      46: out3 <= 12'hc62;
      47: out3 <= 12'hc58;
      48: out3 <= 12'hc4e;
      49: out3 <= 12'hc45;
      50: out3 <= 12'hc3c;
      51: out3 <= 12'hc34;
      52: out3 <= 12'hc2c;
      53: out3 <= 12'hc25;
      54: out3 <= 12'hc1f;
      55: out3 <= 12'hc19;
      56: out3 <= 12'hc14;
      57: out3 <= 12'hc0f;
      58: out3 <= 12'hc0b;
      59: out3 <= 12'hc08;
      60: out3 <= 12'hc05;
      61: out3 <= 12'hc03;
      62: out3 <= 12'hc01;
      63: out3 <= 12'hc00;
      64: out3 <= 12'hc00;
      65: out3 <= 12'hc00;
      66: out3 <= 12'hc01;
      67: out3 <= 12'hc03;
      68: out3 <= 12'hc05;
      69: out3 <= 12'hc08;
      70: out3 <= 12'hc0b;
      71: out3 <= 12'hc0f;
      72: out3 <= 12'hc14;
      73: out3 <= 12'hc19;
      74: out3 <= 12'hc1f;
      75: out3 <= 12'hc25;
      76: out3 <= 12'hc2c;
      77: out3 <= 12'hc34;
      78: out3 <= 12'hc3c;
      79: out3 <= 12'hc45;
      80: out3 <= 12'hc4e;
      81: out3 <= 12'hc58;
      82: out3 <= 12'hc62;
      83: out3 <= 12'hc6d;
      84: out3 <= 12'hc79;
      85: out3 <= 12'hc85;
      86: out3 <= 12'hc92;
      87: out3 <= 12'hc9f;
      88: out3 <= 12'hcad;
      89: out3 <= 12'hcbb;
      90: out3 <= 12'hcca;
      91: out3 <= 12'hcd9;
      92: out3 <= 12'hce8;
      93: out3 <= 12'hcf9;
      94: out3 <= 12'hd09;
      95: out3 <= 12'hd1a;
      96: out3 <= 12'hd2c;
      97: out3 <= 12'hd3e;
      98: out3 <= 12'hd50;
      99: out3 <= 12'hd63;
      100: out3 <= 12'hd76;
      101: out3 <= 12'hd8a;
      102: out3 <= 12'hd9e;
      103: out3 <= 12'hdb2;
      104: out3 <= 12'hdc7;
      105: out3 <= 12'hddc;
      106: out3 <= 12'hdf2;
      107: out3 <= 12'he07;
      108: out3 <= 12'he1d;
      109: out3 <= 12'he34;
      110: out3 <= 12'he4a;
      111: out3 <= 12'he61;
      112: out3 <= 12'he78;
      113: out3 <= 12'he8f;
      114: out3 <= 12'hea7;
      115: out3 <= 12'hebf;
      116: out3 <= 12'hed7;
      117: out3 <= 12'heef;
      118: out3 <= 12'hf07;
      119: out3 <= 12'hf20;
      120: out3 <= 12'hf38;
      121: out3 <= 12'hf51;
      122: out3 <= 12'hf6a;
      123: out3 <= 12'hf83;
      124: out3 <= 12'hf9c;
      125: out3 <= 12'hfb5;
      126: out3 <= 12'hfce;
      127: out3 <= 12'hfe7;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



// Latency: 8
// Gap: 1
module codeBlock25536(clk, reset, next_in, next_out,
   i3_in,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;
   input [6:0] i3_in;
   reg [6:0] i3;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(7, 1) shiftFIFO_31712(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a173;
   wire signed [11:0] a162;
   wire signed [11:0] a176;
   wire signed [11:0] a166;
   wire signed [11:0] a177;
   wire signed [11:0] a178;
   reg signed [11:0] tm454;
   reg signed [11:0] tm458;
   reg signed [11:0] tm470;
   reg signed [11:0] tm477;
   reg signed [11:0] tm455;
   reg signed [11:0] tm459;
   reg signed [11:0] tm471;
   reg signed [11:0] tm478;
   wire signed [11:0] tm26;
   wire signed [11:0] a167;
   wire signed [11:0] tm27;
   wire signed [11:0] a169;
   reg signed [11:0] tm456;
   reg signed [11:0] tm460;
   reg signed [11:0] tm472;
   reg signed [11:0] tm479;
   reg signed [11:0] tm88;
   reg signed [11:0] tm89;
   reg signed [11:0] tm457;
   reg signed [11:0] tm461;
   reg signed [11:0] tm473;
   reg signed [11:0] tm480;
   reg signed [11:0] tm474;
   reg signed [11:0] tm481;
   wire signed [11:0] a168;
   wire signed [11:0] a170;
   wire signed [11:0] a171;
   wire signed [11:0] a172;
   reg signed [11:0] tm475;
   reg signed [11:0] tm482;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   reg signed [11:0] tm476;
   reg signed [11:0] tm483;


   assign a173 = X0;
   assign a162 = a173;
   assign a176 = X1;
   assign a166 = a176;
   assign a177 = X2;
   assign a178 = X3;
   assign a167 = tm26;
   assign a169 = tm27;
   assign Y0 = tm476;
   assign Y1 = tm483;

   D10_25956 instD10inst0_25956(.addr(i3[6:0]), .out(tm26), .clk(clk));

   D12_26216 instD12inst0_26216(.addr(i3[6:0]), .out(tm27), .clk(clk));

    multfix #(12, 2) m25635(.a(tm88), .b(tm457), .clk(clk), .q_sc(a168), .q_unsc(), .rst(reset));
    multfix #(12, 2) m25657(.a(tm89), .b(tm461), .clk(clk), .q_sc(a170), .q_unsc(), .rst(reset));
    multfix #(12, 2) m25675(.a(tm89), .b(tm457), .clk(clk), .q_sc(a171), .q_unsc(), .rst(reset));
    multfix #(12, 2) m25686(.a(tm88), .b(tm461), .clk(clk), .q_sc(a172), .q_unsc(), .rst(reset));
    subfxp #(12, 1) sub25664(.a(a168), .b(a170), .clk(clk), .q(Y2));    // 6
    addfxp #(12, 1) add25693(.a(a171), .b(a172), .clk(clk), .q(Y3));    // 6


   always @(posedge clk) begin
      if (reset == 1) begin
         tm88 <= 0;
         tm457 <= 0;
         tm89 <= 0;
         tm461 <= 0;
         tm89 <= 0;
         tm457 <= 0;
         tm88 <= 0;
         tm461 <= 0;
      end
      else begin
         i3 <= i3_in;
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
         tm454 <= a177;
         tm458 <= a178;
         tm470 <= a162;
         tm477 <= a166;
         tm455 <= tm454;
         tm459 <= tm458;
         tm471 <= tm470;
         tm478 <= tm477;
         tm456 <= tm455;
         tm460 <= tm459;
         tm472 <= tm471;
         tm479 <= tm478;
         tm88 <= a167;
         tm89 <= a169;
         tm457 <= tm456;
         tm461 <= tm460;
         tm473 <= tm472;
         tm480 <= tm479;
         tm474 <= tm473;
         tm481 <= tm480;
         tm475 <= tm474;
         tm482 <= tm481;
         tm476 <= tm475;
         tm483 <= tm482;
      end
   end
endmodule

// Latency: 2
// Gap: 1
module codeBlock26221(clk, reset, next_in, next_out,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(1, 1) shiftFIFO_31715(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a129;
   wire signed [11:0] a130;
   wire signed [11:0] a131;
   wire signed [11:0] a132;
   wire signed [12:0] tm262;
   wire signed [12:0] tm263;
   wire signed [12:0] tm264;
   wire signed [12:0] tm265;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   wire signed [11:0] t69;
   wire signed [11:0] t70;
   wire signed [11:0] t71;
   wire signed [11:0] t72;


   assign a129 = X0;
   assign a130 = X2;
   assign a131 = X1;
   assign a132 = X3;
   assign Y0 = t69;
   assign Y1 = t70;
   assign Y2 = t71;
   assign Y3 = t72;
   assign t69 = tm262[12:1];
   assign t70 = tm263[12:1];
   assign t71 = tm264[12:1];
   assign t72 = tm265[12:1];

    addfxp #(13, 1) add26233(.a({{1{a129[11]}}, a129}), .b({{1{a130[11]}}, a130}), .clk(clk), .q(tm262));    // 0
    addfxp #(13, 1) add26248(.a({{1{a131[11]}}, a131}), .b({{1{a132[11]}}, a132}), .clk(clk), .q(tm263));    // 0
    subfxp #(13, 1) sub26263(.a({{1{a129[11]}}, a129}), .b({{1{a130[11]}}, a130}), .clk(clk), .q(tm264));    // 0
    subfxp #(13, 1) sub26278(.a({{1{a131[11]}}, a131}), .b({{1{a132[11]}}, a132}), .clk(clk), .q(tm265));    // 0


   always @(posedge clk) begin
      if (reset == 1) begin
      end
      else begin
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
      end
   end
endmodule

// Latency: 131
// Gap: 256
module rc26302(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm26300 instPerm31716(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 131
// Gap: 256
module perm26300(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 256;
   parameter logDepth = 8;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[8] ^ addr0[0];
   assign inAddr0[0] = addr0[1];
   assign inAddr0[1] = addr0[2];
   assign inAddr0[2] = addr0[3];
   assign inAddr0[3] = addr0[4];
   assign inAddr0[4] = addr0[5];
   assign inAddr0[5] = addr0[6];
   assign inAddr0[6] = addr0[7];
   assign inAddr0[7] = addr0[0];
   assign outBank0[0] = addr0b[8] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outAddr0[1] = addr0b[2];
   assign outAddr0[2] = addr0b[3];
   assign outAddr0[3] = addr0b[4];
   assign outAddr0[4] = addr0b[5];
   assign outAddr0[5] = addr0b[6];
   assign outAddr0[6] = addr0b[7];
   assign outAddr0[7] = addr0b[8];
   assign outBank_a0[0] = addr0c[8] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];
   assign outAddr_a0[1] = addr0c[2];
   assign outAddr_a0[2] = addr0c[3];
   assign outAddr_a0[3] = addr0c[4];
   assign outAddr_a0[4] = addr0c[5];
   assign outAddr_a0[5] = addr0c[6];
   assign outAddr_a0[6] = addr0c[7];
   assign outAddr_a0[7] = addr0c[8];

   assign inBank1[0] = addr1[8] ^ addr1[0];
   assign inAddr1[0] = addr1[1];
   assign inAddr1[1] = addr1[2];
   assign inAddr1[2] = addr1[3];
   assign inAddr1[3] = addr1[4];
   assign inAddr1[4] = addr1[5];
   assign inAddr1[5] = addr1[6];
   assign inAddr1[6] = addr1[7];
   assign inAddr1[7] = addr1[0];
   assign outBank1[0] = addr1b[8] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outAddr1[1] = addr1b[2];
   assign outAddr1[2] = addr1b[3];
   assign outAddr1[3] = addr1b[4];
   assign outAddr1[4] = addr1b[5];
   assign outAddr1[5] = addr1b[6];
   assign outAddr1[6] = addr1b[7];
   assign outAddr1[7] = addr1b[8];
   assign outBank_a1[0] = addr1c[8] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];
   assign outAddr_a1[1] = addr1c[2];
   assign outAddr_a1[2] = addr1c[3];
   assign outAddr_a1[3] = addr1c[4];
   assign outAddr_a1[4] = addr1c[5];
   assign outAddr_a1[5] = addr1c[6];
   assign outAddr_a1[6] = addr1c[7];
   assign outAddr_a1[7] = addr1c[8];

   nextReg #(129, 8) nextReg_31721(.X(next), .Y(next0), .reset(reset), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31724(.X(next0), .Y(next_out), .clk(clk));


   memArray512_26300 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
        outCount_for_rd_data <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 128)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
         if (inCount == 130)
            outCount_for_rd_data <= 0;
         else
            outCount_for_rd_data <= outCount_for_rd_data+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 128) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 255) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 128)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[7];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[7];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[7];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray512_26300(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 256;
   parameter logDepth = 8;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   nextReg #(256, 8) nextReg_31729(.X(next), .Y(next0), .reset(reset), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule

// Latency: 8
// Gap: 256
module DirSum_27499(clk, reset, next, next_out,
      X0, Y0,
      X1, Y1,
      X2, Y2,
      X3, Y3);

   output next_out;
   input clk, reset, next;

   reg [7:0] i2;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   always @(posedge clk) begin
      if (reset == 1) begin
         i2 <= 0;
      end
      else begin
         if (next == 1)
            i2 <= 0;
         else if (i2 == 255)
            i2 <= 0;
         else
            i2 <= i2 + 1;
      end
   end

   codeBlock26305 codeBlockIsnt31734(.clk(clk), .reset(reset), .next_in(next), .next_out(next_out),
.i2_in(i2),
       .X0_in(X0), .Y0(Y0),
       .X1_in(X1), .Y1(Y1),
       .X2_in(X2), .Y2(Y2),
       .X3_in(X3), .Y3(Y3));

endmodule

module D8_26723(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [7:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h0;
      1: out3 <= 12'hff3;
      2: out3 <= 12'hfe7;
      3: out3 <= 12'hfda;
      4: out3 <= 12'hfce;
      5: out3 <= 12'hfc1;
      6: out3 <= 12'hfb5;
      7: out3 <= 12'hfa8;
      8: out3 <= 12'hf9c;
      9: out3 <= 12'hf8f;
      10: out3 <= 12'hf83;
      11: out3 <= 12'hf76;
      12: out3 <= 12'hf6a;
      13: out3 <= 12'hf5d;
      14: out3 <= 12'hf51;
      15: out3 <= 12'hf45;
      16: out3 <= 12'hf38;
      17: out3 <= 12'hf2c;
      18: out3 <= 12'hf20;
      19: out3 <= 12'hf13;
      20: out3 <= 12'hf07;
      21: out3 <= 12'hefb;
      22: out3 <= 12'heef;
      23: out3 <= 12'hee3;
      24: out3 <= 12'hed7;
      25: out3 <= 12'hecb;
      26: out3 <= 12'hebf;
      27: out3 <= 12'heb3;
      28: out3 <= 12'hea7;
      29: out3 <= 12'he9b;
      30: out3 <= 12'he8f;
      31: out3 <= 12'he84;
      32: out3 <= 12'he78;
      33: out3 <= 12'he6d;
      34: out3 <= 12'he61;
      35: out3 <= 12'he56;
      36: out3 <= 12'he4a;
      37: out3 <= 12'he3f;
      38: out3 <= 12'he34;
      39: out3 <= 12'he28;
      40: out3 <= 12'he1d;
      41: out3 <= 12'he12;
      42: out3 <= 12'he07;
      43: out3 <= 12'hdfc;
      44: out3 <= 12'hdf2;
      45: out3 <= 12'hde7;
      46: out3 <= 12'hddc;
      47: out3 <= 12'hdd2;
      48: out3 <= 12'hdc7;
      49: out3 <= 12'hdbd;
      50: out3 <= 12'hdb2;
      51: out3 <= 12'hda8;
      52: out3 <= 12'hd9e;
      53: out3 <= 12'hd94;
      54: out3 <= 12'hd8a;
      55: out3 <= 12'hd80;
      56: out3 <= 12'hd76;
      57: out3 <= 12'hd6d;
      58: out3 <= 12'hd63;
      59: out3 <= 12'hd5a;
      60: out3 <= 12'hd50;
      61: out3 <= 12'hd47;
      62: out3 <= 12'hd3e;
      63: out3 <= 12'hd35;
      64: out3 <= 12'hd2c;
      65: out3 <= 12'hd23;
      66: out3 <= 12'hd1a;
      67: out3 <= 12'hd12;
      68: out3 <= 12'hd09;
      69: out3 <= 12'hd01;
      70: out3 <= 12'hcf9;
      71: out3 <= 12'hcf0;
      72: out3 <= 12'hce8;
      73: out3 <= 12'hce1;
      74: out3 <= 12'hcd9;
      75: out3 <= 12'hcd1;
      76: out3 <= 12'hcca;
      77: out3 <= 12'hcc2;
      78: out3 <= 12'hcbb;
      79: out3 <= 12'hcb4;
      80: out3 <= 12'hcad;
      81: out3 <= 12'hca6;
      82: out3 <= 12'hc9f;
      83: out3 <= 12'hc98;
      84: out3 <= 12'hc92;
      85: out3 <= 12'hc8b;
      86: out3 <= 12'hc85;
      87: out3 <= 12'hc7f;
      88: out3 <= 12'hc79;
      89: out3 <= 12'hc73;
      90: out3 <= 12'hc6d;
      91: out3 <= 12'hc68;
      92: out3 <= 12'hc62;
      93: out3 <= 12'hc5d;
      94: out3 <= 12'hc58;
      95: out3 <= 12'hc53;
      96: out3 <= 12'hc4e;
      97: out3 <= 12'hc49;
      98: out3 <= 12'hc45;
      99: out3 <= 12'hc40;
      100: out3 <= 12'hc3c;
      101: out3 <= 12'hc38;
      102: out3 <= 12'hc34;
      103: out3 <= 12'hc30;
      104: out3 <= 12'hc2c;
      105: out3 <= 12'hc29;
      106: out3 <= 12'hc25;
      107: out3 <= 12'hc22;
      108: out3 <= 12'hc1f;
      109: out3 <= 12'hc1c;
      110: out3 <= 12'hc19;
      111: out3 <= 12'hc16;
      112: out3 <= 12'hc14;
      113: out3 <= 12'hc11;
      114: out3 <= 12'hc0f;
      115: out3 <= 12'hc0d;
      116: out3 <= 12'hc0b;
      117: out3 <= 12'hc09;
      118: out3 <= 12'hc08;
      119: out3 <= 12'hc06;
      120: out3 <= 12'hc05;
      121: out3 <= 12'hc04;
      122: out3 <= 12'hc03;
      123: out3 <= 12'hc02;
      124: out3 <= 12'hc01;
      125: out3 <= 12'hc01;
      126: out3 <= 12'hc00;
      127: out3 <= 12'hc00;
      128: out3 <= 12'hc00;
      129: out3 <= 12'hc00;
      130: out3 <= 12'hc00;
      131: out3 <= 12'hc01;
      132: out3 <= 12'hc01;
      133: out3 <= 12'hc02;
      134: out3 <= 12'hc03;
      135: out3 <= 12'hc04;
      136: out3 <= 12'hc05;
      137: out3 <= 12'hc06;
      138: out3 <= 12'hc08;
      139: out3 <= 12'hc09;
      140: out3 <= 12'hc0b;
      141: out3 <= 12'hc0d;
      142: out3 <= 12'hc0f;
      143: out3 <= 12'hc11;
      144: out3 <= 12'hc14;
      145: out3 <= 12'hc16;
      146: out3 <= 12'hc19;
      147: out3 <= 12'hc1c;
      148: out3 <= 12'hc1f;
      149: out3 <= 12'hc22;
      150: out3 <= 12'hc25;
      151: out3 <= 12'hc29;
      152: out3 <= 12'hc2c;
      153: out3 <= 12'hc30;
      154: out3 <= 12'hc34;
      155: out3 <= 12'hc38;
      156: out3 <= 12'hc3c;
      157: out3 <= 12'hc40;
      158: out3 <= 12'hc45;
      159: out3 <= 12'hc49;
      160: out3 <= 12'hc4e;
      161: out3 <= 12'hc53;
      162: out3 <= 12'hc58;
      163: out3 <= 12'hc5d;
      164: out3 <= 12'hc62;
      165: out3 <= 12'hc68;
      166: out3 <= 12'hc6d;
      167: out3 <= 12'hc73;
      168: out3 <= 12'hc79;
      169: out3 <= 12'hc7f;
      170: out3 <= 12'hc85;
      171: out3 <= 12'hc8b;
      172: out3 <= 12'hc92;
      173: out3 <= 12'hc98;
      174: out3 <= 12'hc9f;
      175: out3 <= 12'hca6;
      176: out3 <= 12'hcad;
      177: out3 <= 12'hcb4;
      178: out3 <= 12'hcbb;
      179: out3 <= 12'hcc2;
      180: out3 <= 12'hcca;
      181: out3 <= 12'hcd1;
      182: out3 <= 12'hcd9;
      183: out3 <= 12'hce1;
      184: out3 <= 12'hce8;
      185: out3 <= 12'hcf0;
      186: out3 <= 12'hcf9;
      187: out3 <= 12'hd01;
      188: out3 <= 12'hd09;
      189: out3 <= 12'hd12;
      190: out3 <= 12'hd1a;
      191: out3 <= 12'hd23;
      192: out3 <= 12'hd2c;
      193: out3 <= 12'hd35;
      194: out3 <= 12'hd3e;
      195: out3 <= 12'hd47;
      196: out3 <= 12'hd50;
      197: out3 <= 12'hd5a;
      198: out3 <= 12'hd63;
      199: out3 <= 12'hd6d;
      200: out3 <= 12'hd76;
      201: out3 <= 12'hd80;
      202: out3 <= 12'hd8a;
      203: out3 <= 12'hd94;
      204: out3 <= 12'hd9e;
      205: out3 <= 12'hda8;
      206: out3 <= 12'hdb2;
      207: out3 <= 12'hdbd;
      208: out3 <= 12'hdc7;
      209: out3 <= 12'hdd2;
      210: out3 <= 12'hddc;
      211: out3 <= 12'hde7;
      212: out3 <= 12'hdf2;
      213: out3 <= 12'hdfc;
      214: out3 <= 12'he07;
      215: out3 <= 12'he12;
      216: out3 <= 12'he1d;
      217: out3 <= 12'he28;
      218: out3 <= 12'he34;
      219: out3 <= 12'he3f;
      220: out3 <= 12'he4a;
      221: out3 <= 12'he56;
      222: out3 <= 12'he61;
      223: out3 <= 12'he6d;
      224: out3 <= 12'he78;
      225: out3 <= 12'he84;
      226: out3 <= 12'he8f;
      227: out3 <= 12'he9b;
      228: out3 <= 12'hea7;
      229: out3 <= 12'heb3;
      230: out3 <= 12'hebf;
      231: out3 <= 12'hecb;
      232: out3 <= 12'hed7;
      233: out3 <= 12'hee3;
      234: out3 <= 12'heef;
      235: out3 <= 12'hefb;
      236: out3 <= 12'hf07;
      237: out3 <= 12'hf13;
      238: out3 <= 12'hf20;
      239: out3 <= 12'hf2c;
      240: out3 <= 12'hf38;
      241: out3 <= 12'hf45;
      242: out3 <= 12'hf51;
      243: out3 <= 12'hf5d;
      244: out3 <= 12'hf6a;
      245: out3 <= 12'hf76;
      246: out3 <= 12'hf83;
      247: out3 <= 12'hf8f;
      248: out3 <= 12'hf9c;
      249: out3 <= 12'hfa8;
      250: out3 <= 12'hfb5;
      251: out3 <= 12'hfc1;
      252: out3 <= 12'hfce;
      253: out3 <= 12'hfda;
      254: out3 <= 12'hfe7;
      255: out3 <= 12'hff3;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



module D6_27497(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [7:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h400;
      1: out3 <= 12'h400;
      2: out3 <= 12'h400;
      3: out3 <= 12'h3ff;
      4: out3 <= 12'h3ff;
      5: out3 <= 12'h3fe;
      6: out3 <= 12'h3fd;
      7: out3 <= 12'h3fc;
      8: out3 <= 12'h3fb;
      9: out3 <= 12'h3fa;
      10: out3 <= 12'h3f8;
      11: out3 <= 12'h3f7;
      12: out3 <= 12'h3f5;
      13: out3 <= 12'h3f3;
      14: out3 <= 12'h3f1;
      15: out3 <= 12'h3ef;
      16: out3 <= 12'h3ec;
      17: out3 <= 12'h3ea;
      18: out3 <= 12'h3e7;
      19: out3 <= 12'h3e4;
      20: out3 <= 12'h3e1;
      21: out3 <= 12'h3de;
      22: out3 <= 12'h3db;
      23: out3 <= 12'h3d7;
      24: out3 <= 12'h3d4;
      25: out3 <= 12'h3d0;
      26: out3 <= 12'h3cc;
      27: out3 <= 12'h3c8;
      28: out3 <= 12'h3c4;
      29: out3 <= 12'h3c0;
      30: out3 <= 12'h3bb;
      31: out3 <= 12'h3b7;
      32: out3 <= 12'h3b2;
      33: out3 <= 12'h3ad;
      34: out3 <= 12'h3a8;
      35: out3 <= 12'h3a3;
      36: out3 <= 12'h39e;
      37: out3 <= 12'h398;
      38: out3 <= 12'h393;
      39: out3 <= 12'h38d;
      40: out3 <= 12'h387;
      41: out3 <= 12'h381;
      42: out3 <= 12'h37b;
      43: out3 <= 12'h375;
      44: out3 <= 12'h36e;
      45: out3 <= 12'h368;
      46: out3 <= 12'h361;
      47: out3 <= 12'h35a;
      48: out3 <= 12'h353;
      49: out3 <= 12'h34c;
      50: out3 <= 12'h345;
      51: out3 <= 12'h33e;
      52: out3 <= 12'h336;
      53: out3 <= 12'h32f;
      54: out3 <= 12'h327;
      55: out3 <= 12'h31f;
      56: out3 <= 12'h318;
      57: out3 <= 12'h310;
      58: out3 <= 12'h307;
      59: out3 <= 12'h2ff;
      60: out3 <= 12'h2f7;
      61: out3 <= 12'h2ee;
      62: out3 <= 12'h2e6;
      63: out3 <= 12'h2dd;
      64: out3 <= 12'h2d4;
      65: out3 <= 12'h2cb;
      66: out3 <= 12'h2c2;
      67: out3 <= 12'h2b9;
      68: out3 <= 12'h2b0;
      69: out3 <= 12'h2a6;
      70: out3 <= 12'h29d;
      71: out3 <= 12'h293;
      72: out3 <= 12'h28a;
      73: out3 <= 12'h280;
      74: out3 <= 12'h276;
      75: out3 <= 12'h26c;
      76: out3 <= 12'h262;
      77: out3 <= 12'h258;
      78: out3 <= 12'h24e;
      79: out3 <= 12'h243;
      80: out3 <= 12'h239;
      81: out3 <= 12'h22e;
      82: out3 <= 12'h224;
      83: out3 <= 12'h219;
      84: out3 <= 12'h20e;
      85: out3 <= 12'h204;
      86: out3 <= 12'h1f9;
      87: out3 <= 12'h1ee;
      88: out3 <= 12'h1e3;
      89: out3 <= 12'h1d8;
      90: out3 <= 12'h1cc;
      91: out3 <= 12'h1c1;
      92: out3 <= 12'h1b6;
      93: out3 <= 12'h1aa;
      94: out3 <= 12'h19f;
      95: out3 <= 12'h193;
      96: out3 <= 12'h188;
      97: out3 <= 12'h17c;
      98: out3 <= 12'h171;
      99: out3 <= 12'h165;
      100: out3 <= 12'h159;
      101: out3 <= 12'h14d;
      102: out3 <= 12'h141;
      103: out3 <= 12'h135;
      104: out3 <= 12'h129;
      105: out3 <= 12'h11d;
      106: out3 <= 12'h111;
      107: out3 <= 12'h105;
      108: out3 <= 12'hf9;
      109: out3 <= 12'hed;
      110: out3 <= 12'he0;
      111: out3 <= 12'hd4;
      112: out3 <= 12'hc8;
      113: out3 <= 12'hbb;
      114: out3 <= 12'haf;
      115: out3 <= 12'ha3;
      116: out3 <= 12'h96;
      117: out3 <= 12'h8a;
      118: out3 <= 12'h7d;
      119: out3 <= 12'h71;
      120: out3 <= 12'h64;
      121: out3 <= 12'h58;
      122: out3 <= 12'h4b;
      123: out3 <= 12'h3f;
      124: out3 <= 12'h32;
      125: out3 <= 12'h26;
      126: out3 <= 12'h19;
      127: out3 <= 12'hd;
      128: out3 <= 12'h0;
      129: out3 <= 12'hff3;
      130: out3 <= 12'hfe7;
      131: out3 <= 12'hfda;
      132: out3 <= 12'hfce;
      133: out3 <= 12'hfc1;
      134: out3 <= 12'hfb5;
      135: out3 <= 12'hfa8;
      136: out3 <= 12'hf9c;
      137: out3 <= 12'hf8f;
      138: out3 <= 12'hf83;
      139: out3 <= 12'hf76;
      140: out3 <= 12'hf6a;
      141: out3 <= 12'hf5d;
      142: out3 <= 12'hf51;
      143: out3 <= 12'hf45;
      144: out3 <= 12'hf38;
      145: out3 <= 12'hf2c;
      146: out3 <= 12'hf20;
      147: out3 <= 12'hf13;
      148: out3 <= 12'hf07;
      149: out3 <= 12'hefb;
      150: out3 <= 12'heef;
      151: out3 <= 12'hee3;
      152: out3 <= 12'hed7;
      153: out3 <= 12'hecb;
      154: out3 <= 12'hebf;
      155: out3 <= 12'heb3;
      156: out3 <= 12'hea7;
      157: out3 <= 12'he9b;
      158: out3 <= 12'he8f;
      159: out3 <= 12'he84;
      160: out3 <= 12'he78;
      161: out3 <= 12'he6d;
      162: out3 <= 12'he61;
      163: out3 <= 12'he56;
      164: out3 <= 12'he4a;
      165: out3 <= 12'he3f;
      166: out3 <= 12'he34;
      167: out3 <= 12'he28;
      168: out3 <= 12'he1d;
      169: out3 <= 12'he12;
      170: out3 <= 12'he07;
      171: out3 <= 12'hdfc;
      172: out3 <= 12'hdf2;
      173: out3 <= 12'hde7;
      174: out3 <= 12'hddc;
      175: out3 <= 12'hdd2;
      176: out3 <= 12'hdc7;
      177: out3 <= 12'hdbd;
      178: out3 <= 12'hdb2;
      179: out3 <= 12'hda8;
      180: out3 <= 12'hd9e;
      181: out3 <= 12'hd94;
      182: out3 <= 12'hd8a;
      183: out3 <= 12'hd80;
      184: out3 <= 12'hd76;
      185: out3 <= 12'hd6d;
      186: out3 <= 12'hd63;
      187: out3 <= 12'hd5a;
      188: out3 <= 12'hd50;
      189: out3 <= 12'hd47;
      190: out3 <= 12'hd3e;
      191: out3 <= 12'hd35;
      192: out3 <= 12'hd2c;
      193: out3 <= 12'hd23;
      194: out3 <= 12'hd1a;
      195: out3 <= 12'hd12;
      196: out3 <= 12'hd09;
      197: out3 <= 12'hd01;
      198: out3 <= 12'hcf9;
      199: out3 <= 12'hcf0;
      200: out3 <= 12'hce8;
      201: out3 <= 12'hce1;
      202: out3 <= 12'hcd9;
      203: out3 <= 12'hcd1;
      204: out3 <= 12'hcca;
      205: out3 <= 12'hcc2;
      206: out3 <= 12'hcbb;
      207: out3 <= 12'hcb4;
      208: out3 <= 12'hcad;
      209: out3 <= 12'hca6;
      210: out3 <= 12'hc9f;
      211: out3 <= 12'hc98;
      212: out3 <= 12'hc92;
      213: out3 <= 12'hc8b;
      214: out3 <= 12'hc85;
      215: out3 <= 12'hc7f;
      216: out3 <= 12'hc79;
      217: out3 <= 12'hc73;
      218: out3 <= 12'hc6d;
      219: out3 <= 12'hc68;
      220: out3 <= 12'hc62;
      221: out3 <= 12'hc5d;
      222: out3 <= 12'hc58;
      223: out3 <= 12'hc53;
      224: out3 <= 12'hc4e;
      225: out3 <= 12'hc49;
      226: out3 <= 12'hc45;
      227: out3 <= 12'hc40;
      228: out3 <= 12'hc3c;
      229: out3 <= 12'hc38;
      230: out3 <= 12'hc34;
      231: out3 <= 12'hc30;
      232: out3 <= 12'hc2c;
      233: out3 <= 12'hc29;
      234: out3 <= 12'hc25;
      235: out3 <= 12'hc22;
      236: out3 <= 12'hc1f;
      237: out3 <= 12'hc1c;
      238: out3 <= 12'hc19;
      239: out3 <= 12'hc16;
      240: out3 <= 12'hc14;
      241: out3 <= 12'hc11;
      242: out3 <= 12'hc0f;
      243: out3 <= 12'hc0d;
      244: out3 <= 12'hc0b;
      245: out3 <= 12'hc09;
      246: out3 <= 12'hc08;
      247: out3 <= 12'hc06;
      248: out3 <= 12'hc05;
      249: out3 <= 12'hc04;
      250: out3 <= 12'hc03;
      251: out3 <= 12'hc02;
      252: out3 <= 12'hc01;
      253: out3 <= 12'hc01;
      254: out3 <= 12'hc00;
      255: out3 <= 12'hc00;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



// Latency: 8
// Gap: 1
module codeBlock26305(clk, reset, next_in, next_out,
   i2_in,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;
   input [7:0] i2_in;
   reg [7:0] i2;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(7, 1) shiftFIFO_31737(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a113;
   wire signed [11:0] a102;
   wire signed [11:0] a116;
   wire signed [11:0] a106;
   wire signed [11:0] a117;
   wire signed [11:0] a118;
   reg signed [11:0] tm484;
   reg signed [11:0] tm488;
   reg signed [11:0] tm500;
   reg signed [11:0] tm507;
   reg signed [11:0] tm485;
   reg signed [11:0] tm489;
   reg signed [11:0] tm501;
   reg signed [11:0] tm508;
   wire signed [11:0] tm30;
   wire signed [11:0] a107;
   wire signed [11:0] tm31;
   wire signed [11:0] a109;
   reg signed [11:0] tm486;
   reg signed [11:0] tm490;
   reg signed [11:0] tm502;
   reg signed [11:0] tm509;
   reg signed [11:0] tm96;
   reg signed [11:0] tm97;
   reg signed [11:0] tm487;
   reg signed [11:0] tm491;
   reg signed [11:0] tm503;
   reg signed [11:0] tm510;
   reg signed [11:0] tm504;
   reg signed [11:0] tm511;
   wire signed [11:0] a108;
   wire signed [11:0] a110;
   wire signed [11:0] a111;
   wire signed [11:0] a112;
   reg signed [11:0] tm505;
   reg signed [11:0] tm512;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   reg signed [11:0] tm506;
   reg signed [11:0] tm513;


   assign a113 = X0;
   assign a102 = a113;
   assign a116 = X1;
   assign a106 = a116;
   assign a117 = X2;
   assign a118 = X3;
   assign a107 = tm30;
   assign a109 = tm31;
   assign Y0 = tm506;
   assign Y1 = tm513;

   D8_26723 instD8inst0_26723(.addr(i2[7:0]), .out(tm31), .clk(clk));

   D6_27497 instD6inst0_27497(.addr(i2[7:0]), .out(tm30), .clk(clk));

    multfix #(12, 2) m26404(.a(tm96), .b(tm487), .clk(clk), .q_sc(a108), .q_unsc(), .rst(reset));
    multfix #(12, 2) m26426(.a(tm97), .b(tm491), .clk(clk), .q_sc(a110), .q_unsc(), .rst(reset));
    multfix #(12, 2) m26444(.a(tm97), .b(tm487), .clk(clk), .q_sc(a111), .q_unsc(), .rst(reset));
    multfix #(12, 2) m26455(.a(tm96), .b(tm491), .clk(clk), .q_sc(a112), .q_unsc(), .rst(reset));
    subfxp #(12, 1) sub26433(.a(a108), .b(a110), .clk(clk), .q(Y2));    // 6
    addfxp #(12, 1) add26462(.a(a111), .b(a112), .clk(clk), .q(Y3));    // 6


   always @(posedge clk) begin
      if (reset == 1) begin
         tm96 <= 0;
         tm487 <= 0;
         tm97 <= 0;
         tm491 <= 0;
         tm97 <= 0;
         tm487 <= 0;
         tm96 <= 0;
         tm491 <= 0;
      end
      else begin
         i2 <= i2_in;
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
         tm484 <= a117;
         tm488 <= a118;
         tm500 <= a102;
         tm507 <= a106;
         tm485 <= tm484;
         tm489 <= tm488;
         tm501 <= tm500;
         tm508 <= tm507;
         tm486 <= tm485;
         tm490 <= tm489;
         tm502 <= tm501;
         tm509 <= tm508;
         tm96 <= a107;
         tm97 <= a109;
         tm487 <= tm486;
         tm491 <= tm490;
         tm503 <= tm502;
         tm510 <= tm509;
         tm504 <= tm503;
         tm511 <= tm510;
         tm505 <= tm504;
         tm512 <= tm511;
         tm506 <= tm505;
         tm513 <= tm512;
      end
   end
endmodule

// Latency: 2
// Gap: 1
module codeBlock27502(clk, reset, next_in, next_out,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(1, 1) shiftFIFO_31740(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a69;
   wire signed [11:0] a70;
   wire signed [11:0] a71;
   wire signed [11:0] a72;
   wire signed [12:0] tm266;
   wire signed [12:0] tm267;
   wire signed [12:0] tm268;
   wire signed [12:0] tm269;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   wire signed [11:0] t45;
   wire signed [11:0] t46;
   wire signed [11:0] t47;
   wire signed [11:0] t48;


   assign a69 = X0;
   assign a70 = X2;
   assign a71 = X1;
   assign a72 = X3;
   assign Y0 = t45;
   assign Y1 = t46;
   assign Y2 = t47;
   assign Y3 = t48;
   assign t45 = tm266[12:1];
   assign t46 = tm267[12:1];
   assign t47 = tm268[12:1];
   assign t48 = tm269[12:1];

    addfxp #(13, 1) add27514(.a({{1{a69[11]}}, a69}), .b({{1{a70[11]}}, a70}), .clk(clk), .q(tm266));    // 0
    addfxp #(13, 1) add27529(.a({{1{a71[11]}}, a71}), .b({{1{a72[11]}}, a72}), .clk(clk), .q(tm267));    // 0
    subfxp #(13, 1) sub27544(.a({{1{a69[11]}}, a69}), .b({{1{a70[11]}}, a70}), .clk(clk), .q(tm268));    // 0
    subfxp #(13, 1) sub27559(.a({{1{a71[11]}}, a71}), .b({{1{a72[11]}}, a72}), .clk(clk), .q(tm269));    // 0


   always @(posedge clk) begin
      if (reset == 1) begin
      end
      else begin
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
      end
   end
endmodule

// Latency: 259
// Gap: 512
module rc27583(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm27581 instPerm31741(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 259
// Gap: 512
module perm27581(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 512;
   parameter logDepth = 9;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[9] ^ addr0[0];
   assign inAddr0[0] = addr0[1];
   assign inAddr0[1] = addr0[2];
   assign inAddr0[2] = addr0[3];
   assign inAddr0[3] = addr0[4];
   assign inAddr0[4] = addr0[5];
   assign inAddr0[5] = addr0[6];
   assign inAddr0[6] = addr0[7];
   assign inAddr0[7] = addr0[8];
   assign inAddr0[8] = addr0[0];
   assign outBank0[0] = addr0b[9] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outAddr0[1] = addr0b[2];
   assign outAddr0[2] = addr0b[3];
   assign outAddr0[3] = addr0b[4];
   assign outAddr0[4] = addr0b[5];
   assign outAddr0[5] = addr0b[6];
   assign outAddr0[6] = addr0b[7];
   assign outAddr0[7] = addr0b[8];
   assign outAddr0[8] = addr0b[9];
   assign outBank_a0[0] = addr0c[9] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];
   assign outAddr_a0[1] = addr0c[2];
   assign outAddr_a0[2] = addr0c[3];
   assign outAddr_a0[3] = addr0c[4];
   assign outAddr_a0[4] = addr0c[5];
   assign outAddr_a0[5] = addr0c[6];
   assign outAddr_a0[6] = addr0c[7];
   assign outAddr_a0[7] = addr0c[8];
   assign outAddr_a0[8] = addr0c[9];

   assign inBank1[0] = addr1[9] ^ addr1[0];
   assign inAddr1[0] = addr1[1];
   assign inAddr1[1] = addr1[2];
   assign inAddr1[2] = addr1[3];
   assign inAddr1[3] = addr1[4];
   assign inAddr1[4] = addr1[5];
   assign inAddr1[5] = addr1[6];
   assign inAddr1[6] = addr1[7];
   assign inAddr1[7] = addr1[8];
   assign inAddr1[8] = addr1[0];
   assign outBank1[0] = addr1b[9] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outAddr1[1] = addr1b[2];
   assign outAddr1[2] = addr1b[3];
   assign outAddr1[3] = addr1b[4];
   assign outAddr1[4] = addr1b[5];
   assign outAddr1[5] = addr1b[6];
   assign outAddr1[6] = addr1b[7];
   assign outAddr1[7] = addr1b[8];
   assign outAddr1[8] = addr1b[9];
   assign outBank_a1[0] = addr1c[9] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];
   assign outAddr_a1[1] = addr1c[2];
   assign outAddr_a1[2] = addr1c[3];
   assign outAddr_a1[3] = addr1c[4];
   assign outAddr_a1[4] = addr1c[5];
   assign outAddr_a1[5] = addr1c[6];
   assign outAddr_a1[6] = addr1c[7];
   assign outAddr_a1[7] = addr1c[8];
   assign outAddr_a1[8] = addr1c[9];

   nextReg #(257, 9) nextReg_31746(.X(next), .Y(next0), .reset(reset), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31749(.X(next0), .Y(next_out), .clk(clk));


   memArray1024_27581 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
        outCount_for_rd_data <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 256)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
         if (inCount == 258)
            outCount_for_rd_data <= 0;
         else
            outCount_for_rd_data <= outCount_for_rd_data+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 256) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 511) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 256)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[8];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[8];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[8];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray1024_27581(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 512;
   parameter logDepth = 9;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   nextReg #(512, 9) nextReg_31754(.X(next), .Y(next0), .reset(reset), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule

// Latency: 8
// Gap: 512
module DirSum_29803(clk, reset, next, next_out,
      X0, Y0,
      X1, Y1,
      X2, Y2,
      X3, Y3);

   output next_out;
   input clk, reset, next;

   reg [8:0] i1;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   always @(posedge clk) begin
      if (reset == 1) begin
         i1 <= 0;
      end
      else begin
         if (next == 1)
            i1 <= 0;
         else if (i1 == 511)
            i1 <= 0;
         else
            i1 <= i1 + 1;
      end
   end

   codeBlock27585 codeBlockIsnt31759(.clk(clk), .reset(reset), .next_in(next), .next_out(next_out),
.i1_in(i1),
       .X0_in(X0), .Y0(Y0),
       .X1_in(X1), .Y1(Y1),
       .X2_in(X2), .Y2(Y2),
       .X3_in(X3), .Y3(Y3));

endmodule

module D4_28773(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [8:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h0;
      1: out3 <= 12'hffa;
      2: out3 <= 12'hff3;
      3: out3 <= 12'hfed;
      4: out3 <= 12'hfe7;
      5: out3 <= 12'hfe1;
      6: out3 <= 12'hfda;
      7: out3 <= 12'hfd4;
      8: out3 <= 12'hfce;
      9: out3 <= 12'hfc7;
      10: out3 <= 12'hfc1;
      11: out3 <= 12'hfbb;
      12: out3 <= 12'hfb5;
      13: out3 <= 12'hfae;
      14: out3 <= 12'hfa8;
      15: out3 <= 12'hfa2;
      16: out3 <= 12'hf9c;
      17: out3 <= 12'hf95;
      18: out3 <= 12'hf8f;
      19: out3 <= 12'hf89;
      20: out3 <= 12'hf83;
      21: out3 <= 12'hf7c;
      22: out3 <= 12'hf76;
      23: out3 <= 12'hf70;
      24: out3 <= 12'hf6a;
      25: out3 <= 12'hf64;
      26: out3 <= 12'hf5d;
      27: out3 <= 12'hf57;
      28: out3 <= 12'hf51;
      29: out3 <= 12'hf4b;
      30: out3 <= 12'hf45;
      31: out3 <= 12'hf3e;
      32: out3 <= 12'hf38;
      33: out3 <= 12'hf32;
      34: out3 <= 12'hf2c;
      35: out3 <= 12'hf26;
      36: out3 <= 12'hf20;
      37: out3 <= 12'hf1a;
      38: out3 <= 12'hf13;
      39: out3 <= 12'hf0d;
      40: out3 <= 12'hf07;
      41: out3 <= 12'hf01;
      42: out3 <= 12'hefb;
      43: out3 <= 12'hef5;
      44: out3 <= 12'heef;
      45: out3 <= 12'hee9;
      46: out3 <= 12'hee3;
      47: out3 <= 12'hedd;
      48: out3 <= 12'hed7;
      49: out3 <= 12'hed1;
      50: out3 <= 12'hecb;
      51: out3 <= 12'hec5;
      52: out3 <= 12'hebf;
      53: out3 <= 12'heb9;
      54: out3 <= 12'heb3;
      55: out3 <= 12'head;
      56: out3 <= 12'hea7;
      57: out3 <= 12'hea1;
      58: out3 <= 12'he9b;
      59: out3 <= 12'he95;
      60: out3 <= 12'he8f;
      61: out3 <= 12'he8a;
      62: out3 <= 12'he84;
      63: out3 <= 12'he7e;
      64: out3 <= 12'he78;
      65: out3 <= 12'he72;
      66: out3 <= 12'he6d;
      67: out3 <= 12'he67;
      68: out3 <= 12'he61;
      69: out3 <= 12'he5b;
      70: out3 <= 12'he56;
      71: out3 <= 12'he50;
      72: out3 <= 12'he4a;
      73: out3 <= 12'he45;
      74: out3 <= 12'he3f;
      75: out3 <= 12'he39;
      76: out3 <= 12'he34;
      77: out3 <= 12'he2e;
      78: out3 <= 12'he28;
      79: out3 <= 12'he23;
      80: out3 <= 12'he1d;
      81: out3 <= 12'he18;
      82: out3 <= 12'he12;
      83: out3 <= 12'he0d;
      84: out3 <= 12'he07;
      85: out3 <= 12'he02;
      86: out3 <= 12'hdfc;
      87: out3 <= 12'hdf7;
      88: out3 <= 12'hdf2;
      89: out3 <= 12'hdec;
      90: out3 <= 12'hde7;
      91: out3 <= 12'hde1;
      92: out3 <= 12'hddc;
      93: out3 <= 12'hdd7;
      94: out3 <= 12'hdd2;
      95: out3 <= 12'hdcc;
      96: out3 <= 12'hdc7;
      97: out3 <= 12'hdc2;
      98: out3 <= 12'hdbd;
      99: out3 <= 12'hdb8;
      100: out3 <= 12'hdb2;
      101: out3 <= 12'hdad;
      102: out3 <= 12'hda8;
      103: out3 <= 12'hda3;
      104: out3 <= 12'hd9e;
      105: out3 <= 12'hd99;
      106: out3 <= 12'hd94;
      107: out3 <= 12'hd8f;
      108: out3 <= 12'hd8a;
      109: out3 <= 12'hd85;
      110: out3 <= 12'hd80;
      111: out3 <= 12'hd7b;
      112: out3 <= 12'hd76;
      113: out3 <= 12'hd72;
      114: out3 <= 12'hd6d;
      115: out3 <= 12'hd68;
      116: out3 <= 12'hd63;
      117: out3 <= 12'hd5e;
      118: out3 <= 12'hd5a;
      119: out3 <= 12'hd55;
      120: out3 <= 12'hd50;
      121: out3 <= 12'hd4c;
      122: out3 <= 12'hd47;
      123: out3 <= 12'hd42;
      124: out3 <= 12'hd3e;
      125: out3 <= 12'hd39;
      126: out3 <= 12'hd35;
      127: out3 <= 12'hd30;
      128: out3 <= 12'hd2c;
      129: out3 <= 12'hd27;
      130: out3 <= 12'hd23;
      131: out3 <= 12'hd1f;
      132: out3 <= 12'hd1a;
      133: out3 <= 12'hd16;
      134: out3 <= 12'hd12;
      135: out3 <= 12'hd0d;
      136: out3 <= 12'hd09;
      137: out3 <= 12'hd05;
      138: out3 <= 12'hd01;
      139: out3 <= 12'hcfd;
      140: out3 <= 12'hcf9;
      141: out3 <= 12'hcf5;
      142: out3 <= 12'hcf0;
      143: out3 <= 12'hcec;
      144: out3 <= 12'hce8;
      145: out3 <= 12'hce4;
      146: out3 <= 12'hce1;
      147: out3 <= 12'hcdd;
      148: out3 <= 12'hcd9;
      149: out3 <= 12'hcd5;
      150: out3 <= 12'hcd1;
      151: out3 <= 12'hccd;
      152: out3 <= 12'hcca;
      153: out3 <= 12'hcc6;
      154: out3 <= 12'hcc2;
      155: out3 <= 12'hcbe;
      156: out3 <= 12'hcbb;
      157: out3 <= 12'hcb7;
      158: out3 <= 12'hcb4;
      159: out3 <= 12'hcb0;
      160: out3 <= 12'hcad;
      161: out3 <= 12'hca9;
      162: out3 <= 12'hca6;
      163: out3 <= 12'hca2;
      164: out3 <= 12'hc9f;
      165: out3 <= 12'hc9c;
      166: out3 <= 12'hc98;
      167: out3 <= 12'hc95;
      168: out3 <= 12'hc92;
      169: out3 <= 12'hc8e;
      170: out3 <= 12'hc8b;
      171: out3 <= 12'hc88;
      172: out3 <= 12'hc85;
      173: out3 <= 12'hc82;
      174: out3 <= 12'hc7f;
      175: out3 <= 12'hc7c;
      176: out3 <= 12'hc79;
      177: out3 <= 12'hc76;
      178: out3 <= 12'hc73;
      179: out3 <= 12'hc70;
      180: out3 <= 12'hc6d;
      181: out3 <= 12'hc6b;
      182: out3 <= 12'hc68;
      183: out3 <= 12'hc65;
      184: out3 <= 12'hc62;
      185: out3 <= 12'hc60;
      186: out3 <= 12'hc5d;
      187: out3 <= 12'hc5a;
      188: out3 <= 12'hc58;
      189: out3 <= 12'hc55;
      190: out3 <= 12'hc53;
      191: out3 <= 12'hc50;
      192: out3 <= 12'hc4e;
      193: out3 <= 12'hc4c;
      194: out3 <= 12'hc49;
      195: out3 <= 12'hc47;
      196: out3 <= 12'hc45;
      197: out3 <= 12'hc42;
      198: out3 <= 12'hc40;
      199: out3 <= 12'hc3e;
      200: out3 <= 12'hc3c;
      201: out3 <= 12'hc3a;
      202: out3 <= 12'hc38;
      203: out3 <= 12'hc36;
      204: out3 <= 12'hc34;
      205: out3 <= 12'hc32;
      206: out3 <= 12'hc30;
      207: out3 <= 12'hc2e;
      208: out3 <= 12'hc2c;
      209: out3 <= 12'hc2a;
      210: out3 <= 12'hc29;
      211: out3 <= 12'hc27;
      212: out3 <= 12'hc25;
      213: out3 <= 12'hc23;
      214: out3 <= 12'hc22;
      215: out3 <= 12'hc20;
      216: out3 <= 12'hc1f;
      217: out3 <= 12'hc1d;
      218: out3 <= 12'hc1c;
      219: out3 <= 12'hc1a;
      220: out3 <= 12'hc19;
      221: out3 <= 12'hc18;
      222: out3 <= 12'hc16;
      223: out3 <= 12'hc15;
      224: out3 <= 12'hc14;
      225: out3 <= 12'hc12;
      226: out3 <= 12'hc11;
      227: out3 <= 12'hc10;
      228: out3 <= 12'hc0f;
      229: out3 <= 12'hc0e;
      230: out3 <= 12'hc0d;
      231: out3 <= 12'hc0c;
      232: out3 <= 12'hc0b;
      233: out3 <= 12'hc0a;
      234: out3 <= 12'hc09;
      235: out3 <= 12'hc08;
      236: out3 <= 12'hc08;
      237: out3 <= 12'hc07;
      238: out3 <= 12'hc06;
      239: out3 <= 12'hc06;
      240: out3 <= 12'hc05;
      241: out3 <= 12'hc04;
      242: out3 <= 12'hc04;
      243: out3 <= 12'hc03;
      244: out3 <= 12'hc03;
      245: out3 <= 12'hc02;
      246: out3 <= 12'hc02;
      247: out3 <= 12'hc02;
      248: out3 <= 12'hc01;
      249: out3 <= 12'hc01;
      250: out3 <= 12'hc01;
      251: out3 <= 12'hc00;
      252: out3 <= 12'hc00;
      253: out3 <= 12'hc00;
      254: out3 <= 12'hc00;
      255: out3 <= 12'hc00;
      256: out3 <= 12'hc00;
      257: out3 <= 12'hc00;
      258: out3 <= 12'hc00;
      259: out3 <= 12'hc00;
      260: out3 <= 12'hc00;
      261: out3 <= 12'hc00;
      262: out3 <= 12'hc01;
      263: out3 <= 12'hc01;
      264: out3 <= 12'hc01;
      265: out3 <= 12'hc02;
      266: out3 <= 12'hc02;
      267: out3 <= 12'hc02;
      268: out3 <= 12'hc03;
      269: out3 <= 12'hc03;
      270: out3 <= 12'hc04;
      271: out3 <= 12'hc04;
      272: out3 <= 12'hc05;
      273: out3 <= 12'hc06;
      274: out3 <= 12'hc06;
      275: out3 <= 12'hc07;
      276: out3 <= 12'hc08;
      277: out3 <= 12'hc08;
      278: out3 <= 12'hc09;
      279: out3 <= 12'hc0a;
      280: out3 <= 12'hc0b;
      281: out3 <= 12'hc0c;
      282: out3 <= 12'hc0d;
      283: out3 <= 12'hc0e;
      284: out3 <= 12'hc0f;
      285: out3 <= 12'hc10;
      286: out3 <= 12'hc11;
      287: out3 <= 12'hc12;
      288: out3 <= 12'hc14;
      289: out3 <= 12'hc15;
      290: out3 <= 12'hc16;
      291: out3 <= 12'hc18;
      292: out3 <= 12'hc19;
      293: out3 <= 12'hc1a;
      294: out3 <= 12'hc1c;
      295: out3 <= 12'hc1d;
      296: out3 <= 12'hc1f;
      297: out3 <= 12'hc20;
      298: out3 <= 12'hc22;
      299: out3 <= 12'hc23;
      300: out3 <= 12'hc25;
      301: out3 <= 12'hc27;
      302: out3 <= 12'hc29;
      303: out3 <= 12'hc2a;
      304: out3 <= 12'hc2c;
      305: out3 <= 12'hc2e;
      306: out3 <= 12'hc30;
      307: out3 <= 12'hc32;
      308: out3 <= 12'hc34;
      309: out3 <= 12'hc36;
      310: out3 <= 12'hc38;
      311: out3 <= 12'hc3a;
      312: out3 <= 12'hc3c;
      313: out3 <= 12'hc3e;
      314: out3 <= 12'hc40;
      315: out3 <= 12'hc42;
      316: out3 <= 12'hc45;
      317: out3 <= 12'hc47;
      318: out3 <= 12'hc49;
      319: out3 <= 12'hc4c;
      320: out3 <= 12'hc4e;
      321: out3 <= 12'hc50;
      322: out3 <= 12'hc53;
      323: out3 <= 12'hc55;
      324: out3 <= 12'hc58;
      325: out3 <= 12'hc5a;
      326: out3 <= 12'hc5d;
      327: out3 <= 12'hc60;
      328: out3 <= 12'hc62;
      329: out3 <= 12'hc65;
      330: out3 <= 12'hc68;
      331: out3 <= 12'hc6b;
      332: out3 <= 12'hc6d;
      333: out3 <= 12'hc70;
      334: out3 <= 12'hc73;
      335: out3 <= 12'hc76;
      336: out3 <= 12'hc79;
      337: out3 <= 12'hc7c;
      338: out3 <= 12'hc7f;
      339: out3 <= 12'hc82;
      340: out3 <= 12'hc85;
      341: out3 <= 12'hc88;
      342: out3 <= 12'hc8b;
      343: out3 <= 12'hc8e;
      344: out3 <= 12'hc92;
      345: out3 <= 12'hc95;
      346: out3 <= 12'hc98;
      347: out3 <= 12'hc9c;
      348: out3 <= 12'hc9f;
      349: out3 <= 12'hca2;
      350: out3 <= 12'hca6;
      351: out3 <= 12'hca9;
      352: out3 <= 12'hcad;
      353: out3 <= 12'hcb0;
      354: out3 <= 12'hcb4;
      355: out3 <= 12'hcb7;
      356: out3 <= 12'hcbb;
      357: out3 <= 12'hcbe;
      358: out3 <= 12'hcc2;
      359: out3 <= 12'hcc6;
      360: out3 <= 12'hcca;
      361: out3 <= 12'hccd;
      362: out3 <= 12'hcd1;
      363: out3 <= 12'hcd5;
      364: out3 <= 12'hcd9;
      365: out3 <= 12'hcdd;
      366: out3 <= 12'hce1;
      367: out3 <= 12'hce4;
      368: out3 <= 12'hce8;
      369: out3 <= 12'hcec;
      370: out3 <= 12'hcf0;
      371: out3 <= 12'hcf5;
      372: out3 <= 12'hcf9;
      373: out3 <= 12'hcfd;
      374: out3 <= 12'hd01;
      375: out3 <= 12'hd05;
      376: out3 <= 12'hd09;
      377: out3 <= 12'hd0d;
      378: out3 <= 12'hd12;
      379: out3 <= 12'hd16;
      380: out3 <= 12'hd1a;
      381: out3 <= 12'hd1f;
      382: out3 <= 12'hd23;
      383: out3 <= 12'hd27;
      384: out3 <= 12'hd2c;
      385: out3 <= 12'hd30;
      386: out3 <= 12'hd35;
      387: out3 <= 12'hd39;
      388: out3 <= 12'hd3e;
      389: out3 <= 12'hd42;
      390: out3 <= 12'hd47;
      391: out3 <= 12'hd4c;
      392: out3 <= 12'hd50;
      393: out3 <= 12'hd55;
      394: out3 <= 12'hd5a;
      395: out3 <= 12'hd5e;
      396: out3 <= 12'hd63;
      397: out3 <= 12'hd68;
      398: out3 <= 12'hd6d;
      399: out3 <= 12'hd72;
      400: out3 <= 12'hd76;
      401: out3 <= 12'hd7b;
      402: out3 <= 12'hd80;
      403: out3 <= 12'hd85;
      404: out3 <= 12'hd8a;
      405: out3 <= 12'hd8f;
      406: out3 <= 12'hd94;
      407: out3 <= 12'hd99;
      408: out3 <= 12'hd9e;
      409: out3 <= 12'hda3;
      410: out3 <= 12'hda8;
      411: out3 <= 12'hdad;
      412: out3 <= 12'hdb2;
      413: out3 <= 12'hdb8;
      414: out3 <= 12'hdbd;
      415: out3 <= 12'hdc2;
      416: out3 <= 12'hdc7;
      417: out3 <= 12'hdcc;
      418: out3 <= 12'hdd2;
      419: out3 <= 12'hdd7;
      420: out3 <= 12'hddc;
      421: out3 <= 12'hde1;
      422: out3 <= 12'hde7;
      423: out3 <= 12'hdec;
      424: out3 <= 12'hdf2;
      425: out3 <= 12'hdf7;
      426: out3 <= 12'hdfc;
      427: out3 <= 12'he02;
      428: out3 <= 12'he07;
      429: out3 <= 12'he0d;
      430: out3 <= 12'he12;
      431: out3 <= 12'he18;
      432: out3 <= 12'he1d;
      433: out3 <= 12'he23;
      434: out3 <= 12'he28;
      435: out3 <= 12'he2e;
      436: out3 <= 12'he34;
      437: out3 <= 12'he39;
      438: out3 <= 12'he3f;
      439: out3 <= 12'he45;
      440: out3 <= 12'he4a;
      441: out3 <= 12'he50;
      442: out3 <= 12'he56;
      443: out3 <= 12'he5b;
      444: out3 <= 12'he61;
      445: out3 <= 12'he67;
      446: out3 <= 12'he6d;
      447: out3 <= 12'he72;
      448: out3 <= 12'he78;
      449: out3 <= 12'he7e;
      450: out3 <= 12'he84;
      451: out3 <= 12'he8a;
      452: out3 <= 12'he8f;
      453: out3 <= 12'he95;
      454: out3 <= 12'he9b;
      455: out3 <= 12'hea1;
      456: out3 <= 12'hea7;
      457: out3 <= 12'head;
      458: out3 <= 12'heb3;
      459: out3 <= 12'heb9;
      460: out3 <= 12'hebf;
      461: out3 <= 12'hec5;
      462: out3 <= 12'hecb;
      463: out3 <= 12'hed1;
      464: out3 <= 12'hed7;
      465: out3 <= 12'hedd;
      466: out3 <= 12'hee3;
      467: out3 <= 12'hee9;
      468: out3 <= 12'heef;
      469: out3 <= 12'hef5;
      470: out3 <= 12'hefb;
      471: out3 <= 12'hf01;
      472: out3 <= 12'hf07;
      473: out3 <= 12'hf0d;
      474: out3 <= 12'hf13;
      475: out3 <= 12'hf1a;
      476: out3 <= 12'hf20;
      477: out3 <= 12'hf26;
      478: out3 <= 12'hf2c;
      479: out3 <= 12'hf32;
      480: out3 <= 12'hf38;
      481: out3 <= 12'hf3e;
      482: out3 <= 12'hf45;
      483: out3 <= 12'hf4b;
      484: out3 <= 12'hf51;
      485: out3 <= 12'hf57;
      486: out3 <= 12'hf5d;
      487: out3 <= 12'hf64;
      488: out3 <= 12'hf6a;
      489: out3 <= 12'hf70;
      490: out3 <= 12'hf76;
      491: out3 <= 12'hf7c;
      492: out3 <= 12'hf83;
      493: out3 <= 12'hf89;
      494: out3 <= 12'hf8f;
      495: out3 <= 12'hf95;
      496: out3 <= 12'hf9c;
      497: out3 <= 12'hfa2;
      498: out3 <= 12'hfa8;
      499: out3 <= 12'hfae;
      500: out3 <= 12'hfb5;
      501: out3 <= 12'hfbb;
      502: out3 <= 12'hfc1;
      503: out3 <= 12'hfc7;
      504: out3 <= 12'hfce;
      505: out3 <= 12'hfd4;
      506: out3 <= 12'hfda;
      507: out3 <= 12'hfe1;
      508: out3 <= 12'hfe7;
      509: out3 <= 12'hfed;
      510: out3 <= 12'hff3;
      511: out3 <= 12'hffa;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



module D2_29801(addr, out, clk);
   input clk;
   output [11:0] out;
   reg [11:0] out, out2, out3;
   input [8:0] addr;

   always @(posedge clk) begin
      out2 <= out3;
      out <= out2;
   case(addr)
      0: out3 <= 12'h400;
      1: out3 <= 12'h400;
      2: out3 <= 12'h400;
      3: out3 <= 12'h400;
      4: out3 <= 12'h400;
      5: out3 <= 12'h400;
      6: out3 <= 12'h3ff;
      7: out3 <= 12'h3ff;
      8: out3 <= 12'h3ff;
      9: out3 <= 12'h3fe;
      10: out3 <= 12'h3fe;
      11: out3 <= 12'h3fe;
      12: out3 <= 12'h3fd;
      13: out3 <= 12'h3fd;
      14: out3 <= 12'h3fc;
      15: out3 <= 12'h3fc;
      16: out3 <= 12'h3fb;
      17: out3 <= 12'h3fa;
      18: out3 <= 12'h3fa;
      19: out3 <= 12'h3f9;
      20: out3 <= 12'h3f8;
      21: out3 <= 12'h3f8;
      22: out3 <= 12'h3f7;
      23: out3 <= 12'h3f6;
      24: out3 <= 12'h3f5;
      25: out3 <= 12'h3f4;
      26: out3 <= 12'h3f3;
      27: out3 <= 12'h3f2;
      28: out3 <= 12'h3f1;
      29: out3 <= 12'h3f0;
      30: out3 <= 12'h3ef;
      31: out3 <= 12'h3ee;
      32: out3 <= 12'h3ec;
      33: out3 <= 12'h3eb;
      34: out3 <= 12'h3ea;
      35: out3 <= 12'h3e8;
      36: out3 <= 12'h3e7;
      37: out3 <= 12'h3e6;
      38: out3 <= 12'h3e4;
      39: out3 <= 12'h3e3;
      40: out3 <= 12'h3e1;
      41: out3 <= 12'h3e0;
      42: out3 <= 12'h3de;
      43: out3 <= 12'h3dd;
      44: out3 <= 12'h3db;
      45: out3 <= 12'h3d9;
      46: out3 <= 12'h3d7;
      47: out3 <= 12'h3d6;
      48: out3 <= 12'h3d4;
      49: out3 <= 12'h3d2;
      50: out3 <= 12'h3d0;
      51: out3 <= 12'h3ce;
      52: out3 <= 12'h3cc;
      53: out3 <= 12'h3ca;
      54: out3 <= 12'h3c8;
      55: out3 <= 12'h3c6;
      56: out3 <= 12'h3c4;
      57: out3 <= 12'h3c2;
      58: out3 <= 12'h3c0;
      59: out3 <= 12'h3be;
      60: out3 <= 12'h3bb;
      61: out3 <= 12'h3b9;
      62: out3 <= 12'h3b7;
      63: out3 <= 12'h3b4;
      64: out3 <= 12'h3b2;
      65: out3 <= 12'h3b0;
      66: out3 <= 12'h3ad;
      67: out3 <= 12'h3ab;
      68: out3 <= 12'h3a8;
      69: out3 <= 12'h3a6;
      70: out3 <= 12'h3a3;
      71: out3 <= 12'h3a0;
      72: out3 <= 12'h39e;
      73: out3 <= 12'h39b;
      74: out3 <= 12'h398;
      75: out3 <= 12'h395;
      76: out3 <= 12'h393;
      77: out3 <= 12'h390;
      78: out3 <= 12'h38d;
      79: out3 <= 12'h38a;
      80: out3 <= 12'h387;
      81: out3 <= 12'h384;
      82: out3 <= 12'h381;
      83: out3 <= 12'h37e;
      84: out3 <= 12'h37b;
      85: out3 <= 12'h378;
      86: out3 <= 12'h375;
      87: out3 <= 12'h372;
      88: out3 <= 12'h36e;
      89: out3 <= 12'h36b;
      90: out3 <= 12'h368;
      91: out3 <= 12'h364;
      92: out3 <= 12'h361;
      93: out3 <= 12'h35e;
      94: out3 <= 12'h35a;
      95: out3 <= 12'h357;
      96: out3 <= 12'h353;
      97: out3 <= 12'h350;
      98: out3 <= 12'h34c;
      99: out3 <= 12'h349;
      100: out3 <= 12'h345;
      101: out3 <= 12'h342;
      102: out3 <= 12'h33e;
      103: out3 <= 12'h33a;
      104: out3 <= 12'h336;
      105: out3 <= 12'h333;
      106: out3 <= 12'h32f;
      107: out3 <= 12'h32b;
      108: out3 <= 12'h327;
      109: out3 <= 12'h323;
      110: out3 <= 12'h31f;
      111: out3 <= 12'h31c;
      112: out3 <= 12'h318;
      113: out3 <= 12'h314;
      114: out3 <= 12'h310;
      115: out3 <= 12'h30b;
      116: out3 <= 12'h307;
      117: out3 <= 12'h303;
      118: out3 <= 12'h2ff;
      119: out3 <= 12'h2fb;
      120: out3 <= 12'h2f7;
      121: out3 <= 12'h2f3;
      122: out3 <= 12'h2ee;
      123: out3 <= 12'h2ea;
      124: out3 <= 12'h2e6;
      125: out3 <= 12'h2e1;
      126: out3 <= 12'h2dd;
      127: out3 <= 12'h2d9;
      128: out3 <= 12'h2d4;
      129: out3 <= 12'h2d0;
      130: out3 <= 12'h2cb;
      131: out3 <= 12'h2c7;
      132: out3 <= 12'h2c2;
      133: out3 <= 12'h2be;
      134: out3 <= 12'h2b9;
      135: out3 <= 12'h2b4;
      136: out3 <= 12'h2b0;
      137: out3 <= 12'h2ab;
      138: out3 <= 12'h2a6;
      139: out3 <= 12'h2a2;
      140: out3 <= 12'h29d;
      141: out3 <= 12'h298;
      142: out3 <= 12'h293;
      143: out3 <= 12'h28e;
      144: out3 <= 12'h28a;
      145: out3 <= 12'h285;
      146: out3 <= 12'h280;
      147: out3 <= 12'h27b;
      148: out3 <= 12'h276;
      149: out3 <= 12'h271;
      150: out3 <= 12'h26c;
      151: out3 <= 12'h267;
      152: out3 <= 12'h262;
      153: out3 <= 12'h25d;
      154: out3 <= 12'h258;
      155: out3 <= 12'h253;
      156: out3 <= 12'h24e;
      157: out3 <= 12'h248;
      158: out3 <= 12'h243;
      159: out3 <= 12'h23e;
      160: out3 <= 12'h239;
      161: out3 <= 12'h234;
      162: out3 <= 12'h22e;
      163: out3 <= 12'h229;
      164: out3 <= 12'h224;
      165: out3 <= 12'h21f;
      166: out3 <= 12'h219;
      167: out3 <= 12'h214;
      168: out3 <= 12'h20e;
      169: out3 <= 12'h209;
      170: out3 <= 12'h204;
      171: out3 <= 12'h1fe;
      172: out3 <= 12'h1f9;
      173: out3 <= 12'h1f3;
      174: out3 <= 12'h1ee;
      175: out3 <= 12'h1e8;
      176: out3 <= 12'h1e3;
      177: out3 <= 12'h1dd;
      178: out3 <= 12'h1d8;
      179: out3 <= 12'h1d2;
      180: out3 <= 12'h1cc;
      181: out3 <= 12'h1c7;
      182: out3 <= 12'h1c1;
      183: out3 <= 12'h1bb;
      184: out3 <= 12'h1b6;
      185: out3 <= 12'h1b0;
      186: out3 <= 12'h1aa;
      187: out3 <= 12'h1a5;
      188: out3 <= 12'h19f;
      189: out3 <= 12'h199;
      190: out3 <= 12'h193;
      191: out3 <= 12'h18e;
      192: out3 <= 12'h188;
      193: out3 <= 12'h182;
      194: out3 <= 12'h17c;
      195: out3 <= 12'h176;
      196: out3 <= 12'h171;
      197: out3 <= 12'h16b;
      198: out3 <= 12'h165;
      199: out3 <= 12'h15f;
      200: out3 <= 12'h159;
      201: out3 <= 12'h153;
      202: out3 <= 12'h14d;
      203: out3 <= 12'h147;
      204: out3 <= 12'h141;
      205: out3 <= 12'h13b;
      206: out3 <= 12'h135;
      207: out3 <= 12'h12f;
      208: out3 <= 12'h129;
      209: out3 <= 12'h123;
      210: out3 <= 12'h11d;
      211: out3 <= 12'h117;
      212: out3 <= 12'h111;
      213: out3 <= 12'h10b;
      214: out3 <= 12'h105;
      215: out3 <= 12'hff;
      216: out3 <= 12'hf9;
      217: out3 <= 12'hf3;
      218: out3 <= 12'hed;
      219: out3 <= 12'he6;
      220: out3 <= 12'he0;
      221: out3 <= 12'hda;
      222: out3 <= 12'hd4;
      223: out3 <= 12'hce;
      224: out3 <= 12'hc8;
      225: out3 <= 12'hc2;
      226: out3 <= 12'hbb;
      227: out3 <= 12'hb5;
      228: out3 <= 12'haf;
      229: out3 <= 12'ha9;
      230: out3 <= 12'ha3;
      231: out3 <= 12'h9c;
      232: out3 <= 12'h96;
      233: out3 <= 12'h90;
      234: out3 <= 12'h8a;
      235: out3 <= 12'h84;
      236: out3 <= 12'h7d;
      237: out3 <= 12'h77;
      238: out3 <= 12'h71;
      239: out3 <= 12'h6b;
      240: out3 <= 12'h64;
      241: out3 <= 12'h5e;
      242: out3 <= 12'h58;
      243: out3 <= 12'h52;
      244: out3 <= 12'h4b;
      245: out3 <= 12'h45;
      246: out3 <= 12'h3f;
      247: out3 <= 12'h39;
      248: out3 <= 12'h32;
      249: out3 <= 12'h2c;
      250: out3 <= 12'h26;
      251: out3 <= 12'h1f;
      252: out3 <= 12'h19;
      253: out3 <= 12'h13;
      254: out3 <= 12'hd;
      255: out3 <= 12'h6;
      256: out3 <= 12'h0;
      257: out3 <= 12'hffa;
      258: out3 <= 12'hff3;
      259: out3 <= 12'hfed;
      260: out3 <= 12'hfe7;
      261: out3 <= 12'hfe1;
      262: out3 <= 12'hfda;
      263: out3 <= 12'hfd4;
      264: out3 <= 12'hfce;
      265: out3 <= 12'hfc7;
      266: out3 <= 12'hfc1;
      267: out3 <= 12'hfbb;
      268: out3 <= 12'hfb5;
      269: out3 <= 12'hfae;
      270: out3 <= 12'hfa8;
      271: out3 <= 12'hfa2;
      272: out3 <= 12'hf9c;
      273: out3 <= 12'hf95;
      274: out3 <= 12'hf8f;
      275: out3 <= 12'hf89;
      276: out3 <= 12'hf83;
      277: out3 <= 12'hf7c;
      278: out3 <= 12'hf76;
      279: out3 <= 12'hf70;
      280: out3 <= 12'hf6a;
      281: out3 <= 12'hf64;
      282: out3 <= 12'hf5d;
      283: out3 <= 12'hf57;
      284: out3 <= 12'hf51;
      285: out3 <= 12'hf4b;
      286: out3 <= 12'hf45;
      287: out3 <= 12'hf3e;
      288: out3 <= 12'hf38;
      289: out3 <= 12'hf32;
      290: out3 <= 12'hf2c;
      291: out3 <= 12'hf26;
      292: out3 <= 12'hf20;
      293: out3 <= 12'hf1a;
      294: out3 <= 12'hf13;
      295: out3 <= 12'hf0d;
      296: out3 <= 12'hf07;
      297: out3 <= 12'hf01;
      298: out3 <= 12'hefb;
      299: out3 <= 12'hef5;
      300: out3 <= 12'heef;
      301: out3 <= 12'hee9;
      302: out3 <= 12'hee3;
      303: out3 <= 12'hedd;
      304: out3 <= 12'hed7;
      305: out3 <= 12'hed1;
      306: out3 <= 12'hecb;
      307: out3 <= 12'hec5;
      308: out3 <= 12'hebf;
      309: out3 <= 12'heb9;
      310: out3 <= 12'heb3;
      311: out3 <= 12'head;
      312: out3 <= 12'hea7;
      313: out3 <= 12'hea1;
      314: out3 <= 12'he9b;
      315: out3 <= 12'he95;
      316: out3 <= 12'he8f;
      317: out3 <= 12'he8a;
      318: out3 <= 12'he84;
      319: out3 <= 12'he7e;
      320: out3 <= 12'he78;
      321: out3 <= 12'he72;
      322: out3 <= 12'he6d;
      323: out3 <= 12'he67;
      324: out3 <= 12'he61;
      325: out3 <= 12'he5b;
      326: out3 <= 12'he56;
      327: out3 <= 12'he50;
      328: out3 <= 12'he4a;
      329: out3 <= 12'he45;
      330: out3 <= 12'he3f;
      331: out3 <= 12'he39;
      332: out3 <= 12'he34;
      333: out3 <= 12'he2e;
      334: out3 <= 12'he28;
      335: out3 <= 12'he23;
      336: out3 <= 12'he1d;
      337: out3 <= 12'he18;
      338: out3 <= 12'he12;
      339: out3 <= 12'he0d;
      340: out3 <= 12'he07;
      341: out3 <= 12'he02;
      342: out3 <= 12'hdfc;
      343: out3 <= 12'hdf7;
      344: out3 <= 12'hdf2;
      345: out3 <= 12'hdec;
      346: out3 <= 12'hde7;
      347: out3 <= 12'hde1;
      348: out3 <= 12'hddc;
      349: out3 <= 12'hdd7;
      350: out3 <= 12'hdd2;
      351: out3 <= 12'hdcc;
      352: out3 <= 12'hdc7;
      353: out3 <= 12'hdc2;
      354: out3 <= 12'hdbd;
      355: out3 <= 12'hdb8;
      356: out3 <= 12'hdb2;
      357: out3 <= 12'hdad;
      358: out3 <= 12'hda8;
      359: out3 <= 12'hda3;
      360: out3 <= 12'hd9e;
      361: out3 <= 12'hd99;
      362: out3 <= 12'hd94;
      363: out3 <= 12'hd8f;
      364: out3 <= 12'hd8a;
      365: out3 <= 12'hd85;
      366: out3 <= 12'hd80;
      367: out3 <= 12'hd7b;
      368: out3 <= 12'hd76;
      369: out3 <= 12'hd72;
      370: out3 <= 12'hd6d;
      371: out3 <= 12'hd68;
      372: out3 <= 12'hd63;
      373: out3 <= 12'hd5e;
      374: out3 <= 12'hd5a;
      375: out3 <= 12'hd55;
      376: out3 <= 12'hd50;
      377: out3 <= 12'hd4c;
      378: out3 <= 12'hd47;
      379: out3 <= 12'hd42;
      380: out3 <= 12'hd3e;
      381: out3 <= 12'hd39;
      382: out3 <= 12'hd35;
      383: out3 <= 12'hd30;
      384: out3 <= 12'hd2c;
      385: out3 <= 12'hd27;
      386: out3 <= 12'hd23;
      387: out3 <= 12'hd1f;
      388: out3 <= 12'hd1a;
      389: out3 <= 12'hd16;
      390: out3 <= 12'hd12;
      391: out3 <= 12'hd0d;
      392: out3 <= 12'hd09;
      393: out3 <= 12'hd05;
      394: out3 <= 12'hd01;
      395: out3 <= 12'hcfd;
      396: out3 <= 12'hcf9;
      397: out3 <= 12'hcf5;
      398: out3 <= 12'hcf0;
      399: out3 <= 12'hcec;
      400: out3 <= 12'hce8;
      401: out3 <= 12'hce4;
      402: out3 <= 12'hce1;
      403: out3 <= 12'hcdd;
      404: out3 <= 12'hcd9;
      405: out3 <= 12'hcd5;
      406: out3 <= 12'hcd1;
      407: out3 <= 12'hccd;
      408: out3 <= 12'hcca;
      409: out3 <= 12'hcc6;
      410: out3 <= 12'hcc2;
      411: out3 <= 12'hcbe;
      412: out3 <= 12'hcbb;
      413: out3 <= 12'hcb7;
      414: out3 <= 12'hcb4;
      415: out3 <= 12'hcb0;
      416: out3 <= 12'hcad;
      417: out3 <= 12'hca9;
      418: out3 <= 12'hca6;
      419: out3 <= 12'hca2;
      420: out3 <= 12'hc9f;
      421: out3 <= 12'hc9c;
      422: out3 <= 12'hc98;
      423: out3 <= 12'hc95;
      424: out3 <= 12'hc92;
      425: out3 <= 12'hc8e;
      426: out3 <= 12'hc8b;
      427: out3 <= 12'hc88;
      428: out3 <= 12'hc85;
      429: out3 <= 12'hc82;
      430: out3 <= 12'hc7f;
      431: out3 <= 12'hc7c;
      432: out3 <= 12'hc79;
      433: out3 <= 12'hc76;
      434: out3 <= 12'hc73;
      435: out3 <= 12'hc70;
      436: out3 <= 12'hc6d;
      437: out3 <= 12'hc6b;
      438: out3 <= 12'hc68;
      439: out3 <= 12'hc65;
      440: out3 <= 12'hc62;
      441: out3 <= 12'hc60;
      442: out3 <= 12'hc5d;
      443: out3 <= 12'hc5a;
      444: out3 <= 12'hc58;
      445: out3 <= 12'hc55;
      446: out3 <= 12'hc53;
      447: out3 <= 12'hc50;
      448: out3 <= 12'hc4e;
      449: out3 <= 12'hc4c;
      450: out3 <= 12'hc49;
      451: out3 <= 12'hc47;
      452: out3 <= 12'hc45;
      453: out3 <= 12'hc42;
      454: out3 <= 12'hc40;
      455: out3 <= 12'hc3e;
      456: out3 <= 12'hc3c;
      457: out3 <= 12'hc3a;
      458: out3 <= 12'hc38;
      459: out3 <= 12'hc36;
      460: out3 <= 12'hc34;
      461: out3 <= 12'hc32;
      462: out3 <= 12'hc30;
      463: out3 <= 12'hc2e;
      464: out3 <= 12'hc2c;
      465: out3 <= 12'hc2a;
      466: out3 <= 12'hc29;
      467: out3 <= 12'hc27;
      468: out3 <= 12'hc25;
      469: out3 <= 12'hc23;
      470: out3 <= 12'hc22;
      471: out3 <= 12'hc20;
      472: out3 <= 12'hc1f;
      473: out3 <= 12'hc1d;
      474: out3 <= 12'hc1c;
      475: out3 <= 12'hc1a;
      476: out3 <= 12'hc19;
      477: out3 <= 12'hc18;
      478: out3 <= 12'hc16;
      479: out3 <= 12'hc15;
      480: out3 <= 12'hc14;
      481: out3 <= 12'hc12;
      482: out3 <= 12'hc11;
      483: out3 <= 12'hc10;
      484: out3 <= 12'hc0f;
      485: out3 <= 12'hc0e;
      486: out3 <= 12'hc0d;
      487: out3 <= 12'hc0c;
      488: out3 <= 12'hc0b;
      489: out3 <= 12'hc0a;
      490: out3 <= 12'hc09;
      491: out3 <= 12'hc08;
      492: out3 <= 12'hc08;
      493: out3 <= 12'hc07;
      494: out3 <= 12'hc06;
      495: out3 <= 12'hc06;
      496: out3 <= 12'hc05;
      497: out3 <= 12'hc04;
      498: out3 <= 12'hc04;
      499: out3 <= 12'hc03;
      500: out3 <= 12'hc03;
      501: out3 <= 12'hc02;
      502: out3 <= 12'hc02;
      503: out3 <= 12'hc02;
      504: out3 <= 12'hc01;
      505: out3 <= 12'hc01;
      506: out3 <= 12'hc01;
      507: out3 <= 12'hc00;
      508: out3 <= 12'hc00;
      509: out3 <= 12'hc00;
      510: out3 <= 12'hc00;
      511: out3 <= 12'hc00;
      default: out3 <= 0;
   endcase
   end
// synthesis attribute rom_style of out3 is "block"
endmodule



// Latency: 8
// Gap: 1
module codeBlock27585(clk, reset, next_in, next_out,
   i1_in,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;
   input [8:0] i1_in;
   reg [8:0] i1;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(7, 1) shiftFIFO_31762(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a53;
   wire signed [11:0] a42;
   wire signed [11:0] a56;
   wire signed [11:0] a46;
   wire signed [11:0] a57;
   wire signed [11:0] a58;
   reg signed [11:0] tm514;
   reg signed [11:0] tm518;
   reg signed [11:0] tm530;
   reg signed [11:0] tm537;
   reg signed [11:0] tm515;
   reg signed [11:0] tm519;
   reg signed [11:0] tm531;
   reg signed [11:0] tm538;
   wire signed [11:0] tm34;
   wire signed [11:0] a47;
   wire signed [11:0] tm35;
   wire signed [11:0] a49;
   reg signed [11:0] tm516;
   reg signed [11:0] tm520;
   reg signed [11:0] tm532;
   reg signed [11:0] tm539;
   reg signed [11:0] tm104;
   reg signed [11:0] tm105;
   reg signed [11:0] tm517;
   reg signed [11:0] tm521;
   reg signed [11:0] tm533;
   reg signed [11:0] tm540;
   reg signed [11:0] tm534;
   reg signed [11:0] tm541;
   wire signed [11:0] a48;
   wire signed [11:0] a50;
   wire signed [11:0] a51;
   wire signed [11:0] a52;
   reg signed [11:0] tm535;
   reg signed [11:0] tm542;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   reg signed [11:0] tm536;
   reg signed [11:0] tm543;


   assign a53 = X0;
   assign a42 = a53;
   assign a56 = X1;
   assign a46 = a56;
   assign a57 = X2;
   assign a58 = X3;
   assign a47 = tm34;
   assign a49 = tm35;
   assign Y0 = tm536;
   assign Y1 = tm543;

   D4_28773 instD4inst0_28773(.addr(i1[8:0]), .out(tm35), .clk(clk));

   D2_29801 instD2inst0_29801(.addr(i1[8:0]), .out(tm34), .clk(clk));

    multfix #(12, 2) m27684(.a(tm104), .b(tm517), .clk(clk), .q_sc(a48), .q_unsc(), .rst(reset));
    multfix #(12, 2) m27706(.a(tm105), .b(tm521), .clk(clk), .q_sc(a50), .q_unsc(), .rst(reset));
    multfix #(12, 2) m27724(.a(tm105), .b(tm517), .clk(clk), .q_sc(a51), .q_unsc(), .rst(reset));
    multfix #(12, 2) m27735(.a(tm104), .b(tm521), .clk(clk), .q_sc(a52), .q_unsc(), .rst(reset));
    subfxp #(12, 1) sub27713(.a(a48), .b(a50), .clk(clk), .q(Y2));    // 6
    addfxp #(12, 1) add27742(.a(a51), .b(a52), .clk(clk), .q(Y3));    // 6


   always @(posedge clk) begin
      if (reset == 1) begin
         tm104 <= 0;
         tm517 <= 0;
         tm105 <= 0;
         tm521 <= 0;
         tm105 <= 0;
         tm517 <= 0;
         tm104 <= 0;
         tm521 <= 0;
      end
      else begin
         i1 <= i1_in;
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
         tm514 <= a57;
         tm518 <= a58;
         tm530 <= a42;
         tm537 <= a46;
         tm515 <= tm514;
         tm519 <= tm518;
         tm531 <= tm530;
         tm538 <= tm537;
         tm516 <= tm515;
         tm520 <= tm519;
         tm532 <= tm531;
         tm539 <= tm538;
         tm104 <= a47;
         tm105 <= a49;
         tm517 <= tm516;
         tm521 <= tm520;
         tm533 <= tm532;
         tm540 <= tm539;
         tm534 <= tm533;
         tm541 <= tm540;
         tm535 <= tm534;
         tm542 <= tm541;
         tm536 <= tm535;
         tm543 <= tm542;
      end
   end
endmodule

// Latency: 2
// Gap: 1
module codeBlock29806(clk, reset, next_in, next_out,
   X0_in, Y0,
   X1_in, Y1,
   X2_in, Y2,
   X3_in, Y3);

   output next_out;
   input clk, reset, next_in;

   reg next;

   input [11:0] X0_in,
      X1_in,
      X2_in,
      X3_in;

   reg   [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   shiftRegFIFO #(1, 1) shiftFIFO_31765(.X(next), .Y(next_out), .clk(clk));


   wire signed [11:0] a9;
   wire signed [11:0] a10;
   wire signed [11:0] a11;
   wire signed [11:0] a12;
   wire signed [12:0] tm270;
   wire signed [12:0] tm271;
   wire signed [12:0] tm272;
   wire signed [12:0] tm273;
   wire signed [11:0] Y0;
   wire signed [11:0] Y1;
   wire signed [11:0] Y2;
   wire signed [11:0] Y3;
   wire signed [11:0] t21;
   wire signed [11:0] t22;
   wire signed [11:0] t23;
   wire signed [11:0] t24;


   assign a9 = X0;
   assign a10 = X2;
   assign a11 = X1;
   assign a12 = X3;
   assign Y0 = t21;
   assign Y1 = t22;
   assign Y2 = t23;
   assign Y3 = t24;
   assign t21 = tm270[12:1];
   assign t22 = tm271[12:1];
   assign t23 = tm272[12:1];
   assign t24 = tm273[12:1];

    addfxp #(13, 1) add29818(.a({{1{a9[11]}}, a9}), .b({{1{a10[11]}}, a10}), .clk(clk), .q(tm270));    // 0
    addfxp #(13, 1) add29833(.a({{1{a11[11]}}, a11}), .b({{1{a12[11]}}, a12}), .clk(clk), .q(tm271));    // 0
    subfxp #(13, 1) sub29848(.a({{1{a9[11]}}, a9}), .b({{1{a10[11]}}, a10}), .clk(clk), .q(tm272));    // 0
    subfxp #(13, 1) sub29863(.a({{1{a11[11]}}, a11}), .b({{1{a12[11]}}, a12}), .clk(clk), .q(tm273));    // 0


   always @(posedge clk) begin
      if (reset == 1) begin
      end
      else begin
         X0 <= X0_in;
         X1 <= X1_in;
         X2 <= X2_in;
         X3 <= X3_in;
         next <= next_in;
      end
   end
endmodule

// Latency: 259
// Gap: 512
module rc29887(clk, reset, next, next_out,
   X0, Y0,
   X1, Y1,
   X2, Y2,
   X3, Y3);

   output next_out;
   input clk, reset, next;

   input [11:0] X0,
      X1,
      X2,
      X3;

   output [11:0] Y0,
      Y1,
      Y2,
      Y3;

   wire [23:0] t0;
   wire [23:0] s0;
   assign t0 = {X0, X1};
   wire [23:0] t1;
   wire [23:0] s1;
   assign t1 = {X2, X3};
   assign Y0 = s0[23:12];
   assign Y1 = s0[11:0];
   assign Y2 = s1[23:12];
   assign Y3 = s1[11:0];

   perm29885 instPerm31766(.x0(t0), .y0(s0),
    .x1(t1), .y1(s1),
   .clk(clk), .next(next), .next_out(next_out), .reset(reset)
);



endmodule

// Latency: 259
// Gap: 512
module perm29885(clk, next, reset, next_out,
   x0, y0,
   x1, y1);
   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 512;
   parameter logDepth = 9;
   parameter width = 24;

   input [width-1:0]  x0;
   output [width-1:0]  y0;
   wire [width-1:0]  ybuff0;
   input [width-1:0]  x1;
   output [width-1:0]  y1;
   wire [width-1:0]  ybuff1;
   input 	      clk, next, reset;
   output 	     next_out;

   wire    	     next0;

   reg              inFlip0, outFlip0;
   reg              inActive, outActive;

   wire [logBanks-1:0] inBank0, outBank0;
   wire [logDepth-1:0] inAddr0, outAddr0;
   wire [logBanks-1:0] outBank_a0;
   wire [logDepth-1:0] outAddr_a0;
   wire [logDepth+logBanks-1:0] addr0, addr0b, addr0c;
   wire [logBanks-1:0] inBank1, outBank1;
   wire [logDepth-1:0] inAddr1, outAddr1;
   wire [logBanks-1:0] outBank_a1;
   wire [logDepth-1:0] outAddr_a1;
   wire [logDepth+logBanks-1:0] addr1, addr1b, addr1c;


   reg [logDepth-1:0]  inCount, outCount, outCount_d, outCount_dd, outCount_for_rd_addr, outCount_for_rd_data;  

   assign    addr0 = {inCount, 1'd0};
   assign    addr0b = {outCount, 1'd0};
   assign    addr0c = {outCount_for_rd_addr, 1'd0};
   assign    addr1 = {inCount, 1'd1};
   assign    addr1b = {outCount, 1'd1};
   assign    addr1c = {outCount_for_rd_addr, 1'd1};
    wire [width+logDepth-1:0] w_0_0, w_0_1, w_1_0, w_1_1;

    reg [width-1:0] z_0_0;
    reg [width-1:0] z_0_1;
    wire [width-1:0] z_1_0, z_1_1;

    wire [logDepth-1:0] u_0_0, u_0_1, u_1_0, u_1_1;

    always @(posedge clk) begin
    end

   assign inBank0[0] = addr0[1] ^ addr0[0];
   assign inAddr0[0] = addr0[2];
   assign inAddr0[1] = addr0[3];
   assign inAddr0[2] = addr0[4];
   assign inAddr0[3] = addr0[5];
   assign inAddr0[4] = addr0[6];
   assign inAddr0[5] = addr0[7];
   assign inAddr0[6] = addr0[8];
   assign inAddr0[7] = addr0[9];
   assign inAddr0[8] = addr0[0];
   assign outBank0[0] = addr0b[9] ^ addr0b[0];
   assign outAddr0[0] = addr0b[1];
   assign outAddr0[1] = addr0b[2];
   assign outAddr0[2] = addr0b[3];
   assign outAddr0[3] = addr0b[4];
   assign outAddr0[4] = addr0b[5];
   assign outAddr0[5] = addr0b[6];
   assign outAddr0[6] = addr0b[7];
   assign outAddr0[7] = addr0b[8];
   assign outAddr0[8] = addr0b[9];
   assign outBank_a0[0] = addr0c[9] ^ addr0c[0];
   assign outAddr_a0[0] = addr0c[1];
   assign outAddr_a0[1] = addr0c[2];
   assign outAddr_a0[2] = addr0c[3];
   assign outAddr_a0[3] = addr0c[4];
   assign outAddr_a0[4] = addr0c[5];
   assign outAddr_a0[5] = addr0c[6];
   assign outAddr_a0[6] = addr0c[7];
   assign outAddr_a0[7] = addr0c[8];
   assign outAddr_a0[8] = addr0c[9];

   assign inBank1[0] = addr1[1] ^ addr1[0];
   assign inAddr1[0] = addr1[2];
   assign inAddr1[1] = addr1[3];
   assign inAddr1[2] = addr1[4];
   assign inAddr1[3] = addr1[5];
   assign inAddr1[4] = addr1[6];
   assign inAddr1[5] = addr1[7];
   assign inAddr1[6] = addr1[8];
   assign inAddr1[7] = addr1[9];
   assign inAddr1[8] = addr1[0];
   assign outBank1[0] = addr1b[9] ^ addr1b[0];
   assign outAddr1[0] = addr1b[1];
   assign outAddr1[1] = addr1b[2];
   assign outAddr1[2] = addr1b[3];
   assign outAddr1[3] = addr1b[4];
   assign outAddr1[4] = addr1b[5];
   assign outAddr1[5] = addr1b[6];
   assign outAddr1[6] = addr1b[7];
   assign outAddr1[7] = addr1b[8];
   assign outAddr1[8] = addr1b[9];
   assign outBank_a1[0] = addr1c[9] ^ addr1c[0];
   assign outAddr_a1[0] = addr1c[1];
   assign outAddr_a1[1] = addr1c[2];
   assign outAddr_a1[2] = addr1c[3];
   assign outAddr_a1[3] = addr1c[4];
   assign outAddr_a1[4] = addr1c[5];
   assign outAddr_a1[5] = addr1c[6];
   assign outAddr_a1[6] = addr1c[7];
   assign outAddr_a1[7] = addr1c[8];
   assign outAddr_a1[8] = addr1c[9];

   nextReg #(257, 9) nextReg_31771(.X(next), .Y(next0), .reset(reset), .clk(clk));


   shiftRegFIFO #(2, 1) shiftFIFO_31774(.X(next0), .Y(next_out), .clk(clk));


   memArray1024_29885 #(numBanks, logBanks, depth, logDepth, width)
     memSys(.inFlip(inFlip0), .outFlip(outFlip0), .next(next), .reset(reset),
        .x0(w_1_0[width+logDepth-1:logDepth]), .y0(ybuff0),
        .inAddr0(w_1_0[logDepth-1:0]),
        .outAddr0(u_1_0), 
        .x1(w_1_1[width+logDepth-1:logDepth]), .y1(ybuff1),
        .inAddr1(w_1_1[logDepth-1:0]),
        .outAddr1(u_1_1), 
        .clk(clk));

   always @(posedge clk) begin
      if (reset == 1) begin
      z_0_0 <= 0;
      z_0_1 <= 0;
         inFlip0 <= 0; outFlip0 <= 1; outCount <= 0; inCount <= 0;
        outCount_for_rd_addr <= 0;
        outCount_for_rd_data <= 0;
      end
      else begin
          outCount_d <= outCount;
          outCount_dd <= outCount_d;
         if (inCount == 256)
            outCount_for_rd_addr <= 0;
         else
            outCount_for_rd_addr <= outCount_for_rd_addr+1;
         if (inCount == 258)
            outCount_for_rd_data <= 0;
         else
            outCount_for_rd_data <= outCount_for_rd_data+1;
      z_0_0 <= ybuff0;
      z_0_1 <= ybuff1;
         if (inCount == 256) begin
            outFlip0 <= ~outFlip0;
            outCount <= 0;
         end
         else
            outCount <= outCount+1;
         if (inCount == 511) begin
            inFlip0 <= ~inFlip0;
         end
         if (next == 1) begin
            if (inCount >= 256)
               inFlip0 <= ~inFlip0;
            inCount <= 0;
         end
         else
            inCount <= inCount + 1;
      end
   end
    assign w_0_0 = {x0, inAddr0};
    assign w_0_1 = {x1, inAddr1};
    assign y0 = z_1_0;
    assign y1 = z_1_1;
    assign u_0_0 = outAddr_a0;
    assign u_0_1 = outAddr_a1;
    wire wr_ctrl_st_0;
    assign wr_ctrl_st_0 = inCount[0];

    switch #(logDepth+width) in_sw_0_0(.x0(w_0_0), .x1(w_0_1), .y0(w_1_0), .y1(w_1_1), .ctrl(wr_ctrl_st_0));
    wire rdd_ctrl_st_0;
    assign rdd_ctrl_st_0 = outCount_for_rd_data[8];

    switch #(width) out_sw_0_0(.x0(z_0_0), .x1(z_0_1), .y0(z_1_0), .y1(z_1_1), .ctrl(rdd_ctrl_st_0));
    wire rda_ctrl_st_0;
    assign rda_ctrl_st_0 = outCount_for_rd_addr[8];

    switch #(logDepth) rdaddr_sw_0_0(.x0(u_0_0), .x1(u_0_1), .y0(u_1_0), .y1(u_1_1), .ctrl(rda_ctrl_st_0));
endmodule

module memArray1024_29885(next, reset,
                x0, y0,
                inAddr0,
                outAddr0,
                x1, y1,
                inAddr1,
                outAddr1,
                clk, inFlip, outFlip);

   parameter numBanks = 2;
   parameter logBanks = 1;
   parameter depth = 512;
   parameter logDepth = 9;
   parameter width = 24;
         
   input     clk, next, reset;
   input    inFlip, outFlip;
   wire    next0;
   
   input [width-1:0]   x0;
   output [width-1:0]  y0;
   input [logDepth-1:0] inAddr0, outAddr0;
   input [width-1:0]   x1;
   output [width-1:0]  y1;
   input [logDepth-1:0] inAddr1, outAddr1;
   nextReg #(512, 9) nextReg_31779(.X(next), .Y(next0), .reset(reset), .clk(clk));


   memMod #(depth*2, width, logDepth+1) 
     memMod0(.in(x0), .out(y0), .inAddr({inFlip, inAddr0}),
	   .outAddr({outFlip, outAddr0}), .writeSel(1'b1), .clk(clk));   
   memMod #(depth*2, width, logDepth+1) 
     memMod1(.in(x1), .out(y1), .inAddr({inFlip, inAddr1}),
	   .outAddr({outFlip, outAddr1}), .writeSel(1'b1), .clk(clk));   
endmodule


						module multfix(clk, rst, a, b, q_sc, q_unsc);
						   parameter WIDTH=35, CYCLES=6;

						   input signed [WIDTH-1:0]    a,b;
						   output [WIDTH-1:0]          q_sc;
						   output [WIDTH-1:0]              q_unsc;

						   input                       clk, rst;
						   
						   reg signed [2*WIDTH-1:0]    q[CYCLES-1:0];
						   wire signed [2*WIDTH-1:0]   res;   
						   integer                     i;

						   assign                      res = q[CYCLES-1];   
						   
						   assign                      q_unsc = res[WIDTH-1:0];
						   assign                      q_sc = {res[2*WIDTH-1], res[2*WIDTH-4:WIDTH-2]};
						      
						   always @(posedge clk) begin
						      q[0] <= a * b;
						      for (i = 1; i < CYCLES; i=i+1) begin
						         q[i] <= q[i-1];
						      end
						   end
						                  
						endmodule 
module addfxp(a, b, q, clk);

   parameter width = 16, cycles=1;
   
   input signed [width-1:0]  a, b;
   input                     clk;   
   output signed [width-1:0] q;
   reg signed [width-1:0]    res[cycles-1:0];

   assign                    q = res[cycles-1];
   
   integer                   i;   
   
   always @(posedge clk) begin
     res[0] <= a+b;
      for (i=1; i < cycles; i = i+1)
        res[i] <= res[i-1];
      
   end
   
endmodule

module subfxp(a, b, q, clk);

   parameter width = 16, cycles=1;
   
   input signed [width-1:0]  a, b;
   input                     clk;   
   output signed [width-1:0] q;
   reg signed [width-1:0]    res[cycles-1:0];

   assign                    q = res[cycles-1];
   
   integer                   i;   
   
   always @(posedge clk) begin
     res[0] <= a-b;
      for (i=1; i < cycles; i = i+1)
        res[i] <= res[i-1];
      
   end
  
endmodule