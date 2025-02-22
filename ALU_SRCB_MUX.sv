`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/28/2024 04:34:02 PM
// Design Name: 
// Module Name: ALU_SRCB_MUX
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


module ALU_SRCB_MUX(
    //inputs
    input [31:0] rs2,
    input [31:0] Itype,
    input [31:0] Stype,
    input [31:0] PC,
    input [31:0] csr_RD,
    input [2:0] srcB_SEL,

    //outputs
    output logic [31:0] ALU_srcB
    );

    always_comb begin
        case(srcB_SEL)
            0: ALU_srcB = rs2;
            1: ALU_srcB = Itype;
            2: ALU_srcB = Stype;
            3: ALU_srcB = PC;
            4: ALU_srcB = csr_RD;
            default: ALU_srcB = 32'hDEADBEEF;
        endcase
    end
endmodule
