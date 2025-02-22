`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/28/2024 04:33:14 PM
// Design Name: 
// Module Name: ALU_SRCA_MUX
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


module ALU_SRCA_MUX(
    //inputs
    input [31:0] rs1,
    input [31:0] Utype,
    input [31:0] NOT_rs1,
    input [1:0] srcA_SEL,

    //outputs
    output logic [31:0] ALU_srcA
    );

    always_comb begin
        case(srcA_SEL)
            0: ALU_srcA = rs1;
            1: ALU_srcA = Utype;
            2: ALU_srcA = NOT_rs1;
            default: ALU_srcA = 32'hDEADBEEF;
        endcase
    end
endmodule
