`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/01/2026 03:00:02 PM
// Design Name: 
// Module Name: vga_controller
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


module vga_controller(
    input clk_25M,           // 25MHz VGA standard clock
    output hsync, vsync,     // Synchronization pulses for the monitor
    output [3:0] vgaRed, vgaGreen, vgaBlue, // 4-bit Analog color signals
    output [12:0] fb_addr,   // Address sent to the Frame Buffer to get pixel data
    input [15:0] fb_data     // The 16-bit color coming back from memory
);
    //10-bit counters to track the beam position on the screen
    reg [9:0] h_cnt = 0; // Horizontal: 0 to 799 (includes "black" porch areas)
    reg [9:0] v_cnt = 0; // Vertical: 0 to 524

    //VGA SCANNING LOGIC ---
    //This creates the standard 640x480 @ 60Hz timing.
    always @(posedge clk_25M) begin
        if (h_cnt == 799) begin
            h_cnt <= 0;
            if (v_cnt == 524) v_cnt <= 0;
            else v_cnt <= v_cnt + 1;
        end else h_cnt <= h_cnt + 1;
    end

    //SYNC SIGNAL GENERATION ---
    //Industry standard timings for when the monitor should return to start of line/page.
    assign hsync = (h_cnt >= 656 && h_cnt < 752) ? 0 : 1; // Active LOW pulse
    assign vsync = (v_cnt >= 490 && v_cnt < 492) ? 0 : 1; // Active LOW pulse

    //WINDOW LOGIC (Scaling & Centering) ---
    //The original image is 96x64. We scale it by 4x to 384x256.
    //We center it: (640-384)/2 = 128 (Start X), (480-256)/2 = 112 (Start Y).
    //Note: We use '127' to compensate for the 1-clock delay of the BRAM.
    wire video_on = (h_cnt >= 127 && h_cnt < 511) && (v_cnt >= 112 && v_cnt < 368);
    
    //ADDRESS CALCULATION (The Scaling Math) ---
    //This converts the big VGA screen coordinates back into small OLED coordinates.
    //'>> 2' is a bit-shift that divides by 4. This effectively repeats each 
    //OLED pixel 4 times horizontally and 4 times vertically.
    //Formula: Local_X + (Local_Y * Width_of_OLED)
    assign fb_addr = video_on ? (((h_cnt - 127) >> 2) + (((v_cnt - 112) >> 2) * 96)) : 0;

    //COLOR MAPPING ---
    //The Frame Buffer stores RGB565 (16-bit). We extract the most significant 4 bits 
    //for each color to send to the Basys 3's VGA resistor ladder.
    assign vgaRed   = video_on ? fb_data[15:12] : 4'h0; // Red bits
    assign vgaGreen = video_on ? fb_data[10:7]  : 4'h0; // Green bits
    assign vgaBlue  = video_on ? fb_data[4:1]   : 4'h0; // Blue bits

endmodule