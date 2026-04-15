`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.04.2026 13:43:18
// Design Name: 
// Module Name: FIR
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
//This FIR module is originally meant to be a AXI stream but due to skill issue,most of the AXI
//features are ignored!
//source: https://www.hackster.io/whitney-knitter/dsp-for-fpga-simple-fir-filter-in-verilog-91208d

module audio_pipe_bridge(
    input clk,
    input reset,
    input [11:0] adc_raw,      
    input adc_ready,           
    
    output [15:0] fir_in_data,
    output fir_in_valid,
    input signed [31:0] fir_out_data, 
    input fir_out_valid, 
    
    output reg [11:0] audio_out_12bit,
    output reg audio_out_ready 
);

    // --- THE EDGE DETECTOR FIX ---
    reg prev_adc_ready = 0;
    always @(posedge clk) begin
        prev_adc_ready <= adc_ready;
    end
    // fir_in_valid is now strictly a 10-nanosecond pulse! No more double-triggering.
    assign fir_in_valid = (adc_ready && !prev_adc_ready);

    wire signed [11:0] adc_signed = {~adc_raw[11], adc_raw[10:0]};  
    assign fir_in_data = { {4{adc_signed[11]}}, adc_signed };

    wire signed [31:0] scaled_fir = fir_out_data >>> 15;
    
    always @(posedge clk) begin
        if (reset) begin 
            audio_out_12bit <= 0;
            audio_out_ready <= 0;
        end else if (fir_out_valid) begin
            if (scaled_fir > 2047) 
                audio_out_12bit <= 12'h7FF; 
            else if (scaled_fir < -2048) 
                audio_out_12bit <= 12'h800;
            else 
                audio_out_12bit <= scaled_fir[11:0];
            
            audio_out_ready <= 1'b1;
        end else begin
            audio_out_ready <= 1'b0;
        end
    end
endmodule



//this FIR module expects 16 bit audio input
//this FIR module then outputs 32 bits

module FIR_69 (
    input clk,                       // 100MHz System Clock
    input reset,                     // Active High Reset (sw1)
    
    // AXI-Stream Input (From Audio Bridge)
    input signed [15:0] s_axis_fir_tdata,   
    input s_axis_fir_tvalid,         // 50kHz strobe
    output s_axis_fir_tready,        // High when waiting for data
    
    // AXI-Stream Output (To FFT/Audio Bridge)
    output reg signed [31:0] m_axis_fir_tdata,
    output reg m_axis_fir_tvalid,
    input m_axis_fir_tready          // Assumed always 1'b1 from your top module
);

    // --- 1. Shift Register & Coefficient ROM ---
    // 69 taps require 69 memory slots
    reg signed [15:0] shift_reg [0:68];
    reg signed [15:0] coeffs [0:68];
    
    // Load your Q15 coefficients generated from the Python script
    initial begin
        $readmemh("69_Taps.mem", coeffs);
    end

    // --- 2. State Machine & Accumulator ---
    localparam IDLE = 2'b00;
    localparam CALC = 2'b01;
    localparam DONE = 2'b10;

    reg [1:0] state = IDLE;
    reg [6:0] count = 0;             // 0 to 68
    reg signed [39:0] acc = 0;       // 40-bit accumulator to prevent internal overflow
    integer i;
    // We are ready to accept new data only when idling
    assign s_axis_fir_tready = (state == IDLE);

    always @(posedge clk) begin
        if (reset) begin
            state <= IDLE;
            m_axis_fir_tvalid <= 0;
            m_axis_fir_tdata <= 0;
            acc <= 0;
            count <= 0;
            // Note: We don't strictly need to clear the shift register, 
            // but it helps keep simulation waveforms clean.
        end else begin
            case (state)
                IDLE: begin
                    m_axis_fir_tvalid <= 0; // Clear output valid flag
                    
                    if (s_axis_fir_tvalid) begin
                        // 1. Shift the entire register down by one
                       
                        for (i = 68; i > 0; i = i - 1) begin
                            shift_reg[i] <= shift_reg[i-1];
                        end
                        // 2. Load the newest audio sample into index 0
                        shift_reg[0] <= s_axis_fir_tdata;
                        
                        // 3. Prepare for MAC loop
                        acc <= 0;
                        count <= 0;
                        state <= CALC;
                    end
                end

                CALC: begin
                    // The Multiply-Accumulate (MAC) operation
                    // This is the ONLY math happening, easily passing 100MHz timing
                    acc <= acc + (shift_reg[count] * coeffs[count]);
                    
                    // Check if we reached the 69th tap
                    if (count == 68) begin
                        state <= DONE;
                    end else begin
                        count <= count + 1;
                    end
                end

                DONE: begin
                    // Wait if the downstream module isn't ready (AXI handshake)
                    if (m_axis_fir_tready) begin
                        // Truncate the 40-bit accumulator back to 32-bit for the pipeline
                        // The Q15 scaling compensation happens later in your visual bridge
                        m_axis_fir_tdata <= acc[31:0]; 
                        m_axis_fir_tvalid <= 1'b1;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule

//Highpass filter
module FIR_2khpf (
    input clk,                       // 100MHz System Clock
    input reset,                     // Active High Reset (sw1)
    
    // AXI-Stream Input (From Audio Bridge)
    input signed [15:0] s_axis_fir_tdata,   
    input s_axis_fir_tvalid,         // 50kHz strobe
    output s_axis_fir_tready,        // High when waiting for data
    
    // AXI-Stream Output (To FFT/Audio Bridge)
    output reg signed [31:0] m_axis_fir_tdata,
    output reg m_axis_fir_tvalid,
    input m_axis_fir_tready          // Assumed always 1'b1 from your top module
);

    // --- 1. Shift Register & Coefficient ROM ---
    // 69 taps require 69 memory slots
    reg signed [15:0] shift_reg [0:136];
    reg signed [15:0] coeffs [0:136];
    
    // Load your Q15 coefficients generated from the Python script
    initial begin
        $readmemh("2khpf.mem", coeffs);
    end

    // --- 2. State Machine & Accumulator ---
    localparam IDLE = 2'b00;
    localparam CALC = 2'b01;
    localparam DONE = 2'b10;

    reg [1:0] state = IDLE;
    reg [8:0] count = 0;             // 0 to 68
    reg signed [39:0] acc = 0;       // 40-bit accumulator to prevent internal overflow
    integer i;
    // We are ready to accept new data only when idling
    assign s_axis_fir_tready = (state == IDLE);

    always @(posedge clk) begin
        if (reset) begin
            state <= IDLE;
            m_axis_fir_tvalid <= 0;
            m_axis_fir_tdata <= 0;
            acc <= 0;
            count <= 0;
            //Note:  don't strictly need to clear the shift register, 
            //but it helps keep simulation waveforms clean.
        end else begin
            case (state)
                IDLE: begin
                    m_axis_fir_tvalid <= 0; // Clear output valid flag
                    
                    if (s_axis_fir_tvalid) begin
                        // 1. Shift the entire register down by one
                       
                        for (i = 137; i > 0; i = i - 1) begin
                            shift_reg[i] <= shift_reg[i-1];
                        end
                        // 2. Load the newest audio sample into index 0
                        shift_reg[0] <= s_axis_fir_tdata;
                        
                        // 3. Prepare for MAC loop
                        acc <= 0;
                        count <= 0;
                        state <= CALC;
                    end
                end

                CALC: begin
                    //The Multiply-Accumulate (MAC) operation
                    //This is the ONLY math happening, easily passing 100MHz timing
                    acc <= acc + (shift_reg[count] * coeffs[count]);
                    
                    // Check if we reached the 137th tap
                    if (count == 137) begin
                        state <= DONE;
                    end else begin
                        count <= count + 1;
                    end
                end

                DONE: begin
                    // Wait if the downstream module isn't ready (AXI handshake)
                    if (m_axis_fir_tready) begin
                        // Truncate the 40-bit accumulator back to 32-bit for the pipeline
                        // The Q15 scaling compensation happens later in your visual bridge
                        m_axis_fir_tdata <= acc[31:0]; 
                        m_axis_fir_tvalid <= 1'b1;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule


//bandpass filter
//Highpass filter
//181 tap!
module FIR_2k4kbpf (
    input clk,// 100MHz System Clock
    input reset,// Active High Reset (sw1)
    
    //AXI-Stream Input (From Audio Bridge)
    input signed [15:0] s_axis_fir_tdata,   
    input s_axis_fir_tvalid,//50kHz strobe
    output s_axis_fir_tready,// High when waiting for data
    
    //AXI-Stream Output (To FFT/Audio Bridge)
    output reg signed [31:0] m_axis_fir_tdata,
    output reg m_axis_fir_tvalid,
    input m_axis_fir_tready          // Assumed always 1'b1 from top module
);

    // --- 1. Shift Register & Coefficient ROM ---
    // 69 taps require 69 memory slots
    reg signed [15:0] shift_reg [0:180];
    reg signed [15:0] coeffs [0:180];
    
    // Load your Q15 coefficients generated from the Python script
    initial begin
        $readmemh("2k4kBPF.mem", coeffs);
    end

    // --- 2. State Machine & Accumulator ---
    localparam IDLE = 2'b00;
    localparam CALC = 2'b01;
    localparam DONE = 2'b10;

    reg [1:0] state = IDLE;
    reg [8:0] count = 0;             // 0 to 138 or 8 bits
    reg signed [39:0] acc = 0;       // 40-bit accumulator to prevent internal overflow
    integer i;
    // We are ready to accept new data only when idling
    assign s_axis_fir_tready = (state == IDLE);

    always @(posedge clk) begin
        if (reset) begin
            state <= IDLE;
            m_axis_fir_tvalid <= 0;
            m_axis_fir_tdata <= 0;
            acc <= 0;
            count <= 0;
            //Note:  don't strictly need to clear the shift register, 
            //but it helps keep simulation waveforms clean.
        end else begin
            case (state)
                IDLE: begin
                    m_axis_fir_tvalid <= 0; // Clear output valid flag
                    
                    if (s_axis_fir_tvalid) begin
                        // 1. Shift the entire register down by one
                       
                        for (i = 181; i > 0; i = i - 1) begin
                            shift_reg[i] <= shift_reg[i-1];
                        end
                        // 2. Load the newest audio sample into index 0
                        shift_reg[0] <= s_axis_fir_tdata;
                        
                        // 3. Prepare for MAC loop
                        acc <= 0;
                        count <= 0;
                        state <= CALC;
                    end
                end

                CALC: begin
                    //The Multiply-Accumulate (MAC) operation
                    //This is the ONLY math happening, easily passing 100MHz timing
                    acc <= acc + (shift_reg[count] * coeffs[count]);
                    
                    // Check if we reached the 137th tap
                    if (count == 181) begin
                        state <= DONE;
                    end else begin
                        count <= count + 1;
                    end
                end

                DONE: begin
                    // Wait if the downstream module isn't ready (AXI handshake)
                    if (m_axis_fir_tready) begin
                        // Truncate the 40-bit accumulator back to 32-bit for the pipeline
                        // The Q15 scaling compensation happens later in your visual bridge
                        m_axis_fir_tdata <= acc[31:0]; 
                        m_axis_fir_tvalid <= 1'b1;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule