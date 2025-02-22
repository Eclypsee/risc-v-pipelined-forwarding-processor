`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/23/2024 02:39:44 PM
// Design Name: 
// Module Name: BAG
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


module BAG(
    input [31:0] PC,
    input [31:0] J_type,
    input [31:0] B_type,
    input [31:0] I_type,
    input [31:0] rs1,

    output [31:0] jal,
    output [31:0] branch,
    output [31:0] jalr
);
    //branch formed by assigning B type immediate to current PC value
    assign branch = B_type + PC;
    //jal formed by assigning J type immediate to current PC value
    assign jal = J_type + PC;
    //jalr formed by assigning I type immediate to rs1
    assign jalr = I_type + rs1;


endmodule
