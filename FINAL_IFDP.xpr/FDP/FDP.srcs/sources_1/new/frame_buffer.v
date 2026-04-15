`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/01/2026 02:59:15 PM
// Design Name: 
// Module Name: frame_buffer
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

module frame_buffer(
    input clk,               // 100MHz System Clock
    input we,                // Write Enable: When HIGH, we save new data to memory
    input [12:0] write_addr, // The OLED pixel index (0-6143) where we want to write
    input [15:0] din,        // The 16-bit color (White/Black) we want to save
    input [12:0] read_addr,  // The address requested by the VGA controller
    output reg [15:0] dout   // The color data sent back to the VGA
);
    //MEMORY ALLOCATION ---
    //define an array of 6144 registers, each 16-bits wide.
    //Logic: 96 pixels (width) * 64 pixels (height) = 6144 total pixels.
    reg [15:0] mem [6143:0];

    //SYNCHRONOUS MEMORY OPERATION
    // This block handles both Writing and Reading on every clock cycle.
    always @(posedge clk) begin
        // WRITE OPERATION (The OLED Side)
        // If Write Enable is active, we store the incoming color (din)
        // into the memory slot specified by write_addr.
        if (we) begin
            mem[write_addr] <= din;
        end
        
        // 2. READ OPERATION (The VGA Side)
        // On every clock cycle, look at the read_addr requested by the VGA
        // and fetch that color to output (dout).
        // Note: Because this is inside an 'always @(posedge clk)', there is 
        // a 1-clock cycle delay between asking for an address and getting the data.
        dout <= mem[read_addr];
    end
endmodule
