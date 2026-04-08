`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.03.2026 12:02:01
// Design Name: 
// Module Name: ADC
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

// --- CLOCK DIVIDER MODULES ---
// These scale down the 100MHz system clock to speeds peripherals can handle.

module sixp25MHz(input CLOCK, output reg sixp25out = 0);
    reg [2:0] counter = 0;
    always @ (posedge CLOCK) begin
        counter <= (counter == 7) ? 0 : counter +1;
        sixp25out <= (counter == 7) ? ~sixp25out : sixp25out;
    end
endmodule

module t1Mhz (input CLOCK, output reg t1out = 0); 
    reg [5:0] counter = 0;
    always @ (posedge CLOCK) begin
        counter <= (counter == 49) ? 0 : counter +1;
        t1out <= (counter == 49) ? ~t1out : t1out;
    end
endmodule

// --- SPI MASTER MODULE ---
// Acts as the "translator" between the FPGA and the Pmod MIC3 microphone.
module spi_master (
    input CLOCK,
    input MISO,             // Serial data input from the microphone
    output SCK,            // Serial clock output to the microphone (1MHz)
    output reg CS = 1,     // Chip Select: Active LOW to start communication
    output reg [11:0] audio_data = 0, // Final 12-bit digital audio value
    output reg data_ready = 0         // Pulses HIGH when a full sample is ready
);

    wire t1out;
    t1Mhz onemeg (CLOCK, t1out); // Generate the 1MHz SPI clock
    assign SCK = t1out;
       
    // 20-tick loop: At 1MHz, this creates a 50kHz sampling rate
    reg [4:0] tick_counter = 0; 
    reg [15:0] shift_reg = 0;   // Buffer to hold bits as they arrive

    always @(posedge t1out) begin
        tick_counter <= (tick_counter == 19) ? 0 : tick_counter + 1; //20 counts to match 50K sampling rate

        case (tick_counter)
            0: begin
                CS <= 1'b0;         // Wake up the mic
                data_ready <= 1'b0;
            end

            // Ticks 1-16: Shift in 16 bits of data from the MISO line
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16: begin
                shift_reg <= {shift_reg[14:0], MISO}; 
            end

            17: begin
                CS <= 1'b1;         // Put mic to sleep
                // Extract bits [12:1] as per AD7476 datasheet to get 12-bit audio
                audio_data <= shift_reg[12:1]; 
                data_ready <= 1'b1; // Tell the system a new sample is ready
            end

            default: data_ready <= 1'b0;
        endcase
    end
endmodule



// --- SPI SLAVE (Speaker DAC) ---
module spi_dac (
    input clk_1MHz,          
    input data_ready,        // Trigger to send data
    input [11:0] audio_in,   
    output DAC_SCLK,         
    output reg DAC_SYNC = 1, 
    output reg DAC_DINA = 0  
);
    assign DAC_SCLK = clk_1MHz;
    reg [4:0] tick_counter = 0;
    reg [15:0] shift_reg = 0;
    reg active = 0;

    always @(posedge clk_1MHz) begin
        if (data_ready == 1'b1 && active == 0) begin
            active <= 1;
            tick_counter <= 16;
            shift_reg <= {4'b0000, audio_in}; // Pad with 4 leading zeros
            DAC_SYNC <= 0; 
        end 
        else if (active == 1) begin
            if (tick_counter > 0) begin
                DAC_DINA <= shift_reg[15];            
                shift_reg <= {shift_reg[14:0], 1'b0}; 
                tick_counter <= tick_counter - 1;
            end else begin
                DAC_SYNC <= 1; 
                active <= 0;
            end
        end
    end
endmodule

// --- CHIPMUNK TRIGGER (76.9 kHz) ---
module trigger_chipmunk (
    input clk_1MHz,
    output reg trigger_out = 0
);
    reg [3:0] count = 0;
    always @(posedge clk_1MHz) begin
        if (count == 13) begin 
            count <= 0;
            trigger_out <= 1'b1;
        end else begin
            count <= count + 1;
            trigger_out <= 1'b0;
        end
    end
endmodule

// --- VADER TRIGGER (25 kHz) ---
module trigger_vader (
    input clk_1MHz,
    output reg trigger_out = 0
);
    reg [5:0] count = 0;
    always @(posedge clk_1MHz) begin
        if (count == 39) begin 
            count <= 0;
            trigger_out <= 1'b1;
        end else begin
            count <= count + 1;
            trigger_out <= 1'b0;
        end
    end
endmodule

// --- TRUE BYPASS TRIGGER (50 kHz) ---
module trigger_bypass (
    input clk_1MHz,
    output reg trigger_out = 0
);
    reg [4:0] count = 0;
    always @(posedge clk_1MHz) begin
        if (count == 19) begin 
            count <= 0;
            trigger_out <= 1'b1;
        end else begin
            count <= count + 1;
            trigger_out <= 1'b0;
        end
    end
endmodule

// --- AUDIO RING BUFFER (FIFO) ---
module audio_fifo (
    input CLOCK,              
    input write_trigger,      
    input [11:0] write_data,  
    input read_trigger,       
    output reg [11:0] read_data = 0 
);
    (* ram_style = "block" *) reg [11:0] memory [0:4095];
    reg [11:0] write_ptr = 0;
    reg [11:0] read_ptr = 0;
    reg prev_write = 0, prev_read = 0;
    
    always @(posedge CLOCK) begin
        // Write Port
        prev_write <= write_trigger;
        if (write_trigger == 1'b1 && prev_write == 1'b0) begin
            memory[write_ptr] <= write_data;
            write_ptr <= write_ptr + 1; 
        end
        // Read Port
        prev_read <= read_trigger;
        if (read_trigger == 1'b1 && prev_read == 1'b0) begin
            read_data <= memory[read_ptr];
            read_ptr <= read_ptr + 1;   
        end
    end
endmodule



// --- AUDIO BUFFER MODULE ---
// Logic to store audio samples and decide which pixels to light up.
module audio_buffer (
    input CLOCK,
    input wire [11:0] audio_in,      // 12-bit sample from SPI master
    input wire data_ready,           // 50kHz trigger signal
    input wire [12:0] pixel_index,   // Current pixel (0-6143) being drawn
    output reg [15:0] oled_color_out // Color to be displayed (White or Black)
);

    // Memory to store 96 samples (one for each column of the OLED)
    reg [11:0] wave_memory [0:95];
    reg [6:0] write_x = 0; // Pointer for current column being updated
    reg prev_data_ready = 0; 

// 1. Generate a 60 Hz Refresh Timer (100MHz / 60 = ~1,666,666 cycles)
    reg [21:0] refresh_timer = 0;
    reg capturing = 1;

    always @(posedge CLOCK) begin
        // Keep the timer spinning
        refresh_timer <= (refresh_timer == 4_166_666) ? 0 : refresh_timer + 1;
        
        // Every 1/60th of a second,
        if (refresh_timer == 0) begin
            capturing <= 1;
        end

        // EDGE DETECTION: Update memory only when data_ready goes from 0 to 1
        prev_data_ready <= data_ready;
        if (data_ready == 1'b1 && prev_data_ready == 1'b0) begin
            
            // Only overwrite the memory if the camera shutter is open
            if (capturing == 1'b1) begin
                wave_memory[write_x] <= audio_in;
                
                // If we just finished drawing the very last column (95)
                if (write_x == 95) begin
                    write_x <= 0;
                    capturing <= 0; // Picture taken! Freeze the memory until the next 60Hz tick
                end else begin
                    write_x <= write_x + 1;
                end
            end
        end
    end


    
// --- MEMORY AND COORDINATE SETUP ---
            wire [6:0] read_x = pixel_index % 96; 
            wire [5:0] read_y = pixel_index / 96; 
            
            // Treat the raw memory as a proper signed 2's complement number
            wire signed [11:0] signed_sample = wave_memory[read_x];
        
            // --- 1 & 2. LINEAR TIME-DOMAIN MATH ---
            // (We bypass the dB LUT entirely for the waveform view!)
            
            // Divide the -2048 to +2047 audio by 64 so it fits a 64-pixel tall screen
            wire signed [5:0] scaled_sample = signed_sample >>> 6; 
        
            // Center it at Y=32. (We subtract because Y=0 is the top of the OLED screen)
            wire [5:0] target_y = 6'd32 - scaled_sample;
        
            // --- 3. DRAWING LOGIC ---
            always @(*) begin
                // Draw the target pixel, plus thickness, with boundary safety
                if (read_y == target_y || 
                   (target_y < 63 && read_y == target_y + 1) || 
                   (target_y > 0  && read_y == target_y - 1)) begin
                    
                    oled_color_out = 16'hFFFF; // White Waveform Line
                    
                // Draw the faint Zero-Crossing Axis Line (Y = 32)
                end else if (read_y == 32 && read_x[0] == 1'b0) begin
                    oled_color_out = 16'h8410; // Faint Gray Color
                    
                end else begin
                    oled_color_out = 16'h0000; // Black Background
                end
            end
endmodule

module top (
    input CLOCK,            // 100MHz Basys 3 System Clock
    input btnC,             // Reset (Active High for SPIRAL)
    input btnU, btnD,       // Chipmunk / Vader
    input btnL, btnR,       // Scrolling Left/Right
    input sw0,              // 0=Waveform, 1=Spectrum
    input sw1,              // FFT Pipeline Reset
    
    // 7-Segment Display
    output [6:0] seg,
    output [3:0] an,
    
    // MIC (ADC)
    input MISO,             
    output CS, SCK,         
    
    // DAC (Pmod DA2)
    output DAC_SYNC, DAC_DINA, DAC_SCLK,
    
    // OLED
    output [7:0] JC,        
    
    // VGA
    output [3:0] vgaRed, vgaGreen, vgaBlue, 
    output Hsync, Vsync
);

    // --- 1. CLOCK GENERATION ---
    reg [3:0] clk_divider = 0;
    always @(posedge CLOCK) clk_divider <= clk_divider + 1;
    wire clk_25M   = clk_divider[1]; // VGA
    wire w_6p25MHz = clk_divider[3]; // OLED
    
    reg [5:0] count_1mhz = 0;
    reg w_1MHz_clock = 0;
    always @(posedge CLOCK) begin
        count_1mhz <= (count_1mhz == 49) ? 0 : count_1mhz + 1;
        w_1MHz_clock <= (count_1mhz == 49) ? ~w_1MHz_clock : w_1MHz_clock;
    end

    // --- 2. AUDIO INPUT (MIC) ---
    wire [11:0] w_adc_raw;
    wire w_adc_ready;
    
    spi_master mic_driver (
        .CLOCK(CLOCK), .MISO(MISO), .SCK(SCK), .CS(CS),
        .audio_data(w_adc_raw), .data_ready(w_adc_ready)   
    );

    // --- 3. FIR FILTER PIPELINE ---
    wire [15:0] fir_in_data;
    wire        fir_in_valid;
    wire [31:0] fir_out_high_res;
    wire        fir_out_valid;
    wire [11:0] fir_audio_clean; // Filtered & Saturated 12-bit
    wire        fir_audio_ready;

    audio_pipe_bridge fir_brdg (
        .clk(CLOCK), .reset(sw1),
        .adc_raw(w_adc_raw), .adc_ready(w_adc_ready),
        .fir_in_data(fir_in_data), .fir_in_valid(fir_in_valid),
        .fir_out_data(fir_out_high_res), .fir_out_valid(fir_out_valid),
        .audio_out_12bit(fir_audio_clean), .audio_out_ready(fir_audio_ready)
    );

    FIR_69 lpf_filter (
        .clk(CLOCK), .reset(sw1),
        .s_axis_fir_tdata(fir_in_data), .s_axis_fir_tvalid(fir_in_valid),
        .m_axis_fir_tdata(fir_out_high_res), .m_axis_fir_tvalid(fir_out_valid),
        .m_axis_fir_tready(1'b1)
    );

    // --- 4. FFT & VISUAL PIPELINE ---
    wire f_next, f_next_out;
    wire [11:0] X0, X1, X2, X3, Y0, Y1, Y2, Y3;
    wire [5:0] fft_height;
    wire [12:0] w_pixel_index;
    wire [6:0] cur_x = w_pixel_index % 96;
    wire [8:0] w_scroll_offset;

    // Decimate, Window, and Burst into SPIRAL
    visual_fft_bridge fft_vis_brdg (
        .clk(CLOCK), .reset(sw1),
        .fir_data(fir_out_high_res), .fir_valid(fir_out_valid),
        .next(f_next), .X0(X0), .X1(X1), .X2(X2), .X3(X3)
    );

    dft_top dft_inst (
        .clk(CLOCK), .reset(sw1), 
        .next(f_next), .next_out(f_next_out), 
        .X0(X0), .X1(X1), .X2(X2), .X3(X3), 
        .Y0(Y0), .Y1(Y1), .Y2(Y2), .Y3(Y3)
    );
    
    
    seven_seg_driver seg_display (
            .clk(CLOCK),                   
            .scroll_offset(w_scroll_offset),   
            .an(an),         
            .seg(seg)         
        );
        
    scroll_controller scroller (
        .clk(CLOCK), .reset(btnC),
        .btnL(btnL), .btnR(btnR),
        .scroll_offset(w_scroll_offset)
    );

    fft_processor proc (
        .clk(CLOCK), .reset(sw1), .next_out(f_next_out),
        .Y0(Y0), .Y1(Y1), .Y2(Y2), .Y3(Y3),
        .scroll_offset(w_scroll_offset),
        .oled_x(cur_x), .db_height(fft_height)
    );

    // --- 5. VISUALIZATION MUX ---
    wire [15:0] waveform_color, fft_color, final_oled_color;
    wire [12:0] vga_read_addr;
    wire [15:0] vga_pixel_data;
        
    audio_buffer wave_buf (
        .CLOCK(CLOCK), .audio_in(fir_audio_clean), .data_ready(fir_audio_ready), 
        .pixel_index(w_pixel_index), .oled_color_out(waveform_color)
    );
    
    wire [5:0] cur_y = w_pixel_index / 96;
    assign fft_color = (cur_y >= (63 - fft_height)) ? 16'h0000 : 16'hFFFF;
    assign final_oled_color = (sw0) ? fft_color : waveform_color;
    
    // --- 6. OUTPUT DRIVERS (OLED / VGA / DAC) ---
    Oled_Display oled_unit (
        .clk(w_6p25MHz), .reset(btnC),
        .pixel_index(w_pixel_index), .pixel_data(final_oled_color),   
        .cs(JC[0]), .sdin(JC[1]), .sclk(JC[3]), .d_cn(JC[4]),
        .resn(JC[5]), .vccen(JC[6]), .pmoden(JC[7]) 
    );

    frame_buffer mirroring_unit (
        .clk(CLOCK), .we(1'b1), .write_addr(w_pixel_index), .din(final_oled_color),      
        .read_addr(vga_read_addr), .dout(vga_pixel_data)    
    );

    vga_controller vga_unit (
        .clk_25M(clk_25M), .vgaRed(vgaRed), .vgaGreen(vgaGreen), .vgaBlue(vgaBlue),
        .hsync(Hsync), .vsync(Vsync), .fb_addr(vga_read_addr), .fb_data(vga_pixel_data)
    );

 // Pitch Shifter logic
    wire w_chipmunk_trigger, w_vader_trigger, w_bypass_trigger;
    wire [11:0] w_fifo_data;
    
    trigger_chipmunk fast_clk (.clk_1MHz(w_1MHz_clock), .trigger_out(w_chipmunk_trigger));
    trigger_vader    slow_clk (.clk_1MHz(w_1MHz_clock), .trigger_out(w_vader_trigger));
    trigger_bypass   norm_clk (.clk_1MHz(w_1MHz_clock), .trigger_out(w_bypass_trigger)); // <-- NEW
    
    // THE 3-WAY MUX: Now entirely running on safe 1MHz clock signals
    wire w_play_trig = (btnU) ? w_chipmunk_trigger : 
                       (btnD) ? w_vader_trigger : 
                                w_bypass_trigger; 
    
    audio_fifo ring_buffer (
        .CLOCK(CLOCK), 
        .write_trigger(fir_audio_ready), // 100MHz domain writes
        .write_data(fir_audio_clean),    
        .read_trigger(w_play_trig),      // 1MHz domain reads
        .read_data(w_fifo_data)          
    );

    // Convert the signed FIFO data back to unsigned (0-4095) for the hardware DAC
    wire [11:0] unsigned_dac_audio = {~w_fifo_data[11], w_fifo_data[10:0]};

    spi_dac speaker_driver (
        .clk_1MHz(w_1MHz_clock), 
        .data_ready(w_play_trig), 
        .audio_in(unsigned_dac_audio), 
        .DAC_SYNC(DAC_SYNC), .DAC_DINA(DAC_DINA), .DAC_SCLK(DAC_SCLK)
    );
    
endmodule