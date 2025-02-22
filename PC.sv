`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/30/2024 03:01:29 PM
// Design Name: 
// Module Name: PC
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

// 32 bit register
module PC (
    input[31:0] PC_DIN,     // 32-bit input data (DIN)
    input clk,           // Clock input signal (clk)
    input PC_WE,           // write enable signal
    input PC_RST,           // reset signal
    output logic[31:0] PC_COUNT  // 32-bit output data (DOUT)
    );
    
    // Always block that triggers on the rising edge of the clock signal (posedge clk)
    always_ff @(posedge clk)
    begin
        if (PC_RST == 1'b1)
            PC_COUNT <=32'b0;
        else if (PC_WE == 1'b1)
                PC_COUNT <= PC_DIN;
    end
endmodule
