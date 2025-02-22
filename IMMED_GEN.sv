`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/14/2024 03:26:02 PM
// Design Name: 
// Module Name: IMMED_GEN
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
module IMMED_GEN(
    input [31:7] IR, //25-bits
    output logic [31:0] U,
    output logic [31:0] I,
    output logic [31:0] S,
    output logic [31:0] J,
    output logic [31:0] B
    );
    
    always_comb
    begin
        U = {IR[31:12], 12'b0};
        I = {{21{IR[31]}}, IR[30:20]};
        S = {{21{IR[31]}}, IR[30:25], IR[11:7]};
        J = {{12{IR[31]}}, IR[19:12], IR[20], IR [30:21], 1'b0};
        B = {{20{IR[31]}}, IR[7], IR[30:25], IR[11:8], 1'b0};
    end
endmodule