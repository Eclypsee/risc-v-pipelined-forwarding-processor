`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/14/2024 03:26:02 PM
// Design Name: 
// Module Name: ALU
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


module ALU(
    input [31:0] srcA,
    input [31:0] srcB,
    input [3:0] alu_fun,
    output logic [31:0] alu_result
    );
    
    always_comb begin
    case(alu_fun) 
        4'b0000: alu_result = srcA+srcB; //add
        4'b1000: alu_result = srcA-srcB; //sub
        4'b0110: alu_result = srcA | srcB; //or
        4'b0111: alu_result = srcA & srcB; //and
        4'b0100: alu_result = srcA ^ srcB; //xor
        4'b0101: alu_result = srcA >> srcB[4:0]; //srl shifts right srcA by lower 5 bits of srcB

        4'b0001: alu_result = srcA << (srcB[4:0]); // sll shifts srcA by the lower 5 bits of srcB
        4'b1101: alu_result = $signed(srcA) >>> srcB[4:0]; // SRA: shift right using the lower 5 bits of srcB
        4'b0010: alu_result = ($signed(srcA) < $signed(srcB)) ? 32'b1 : 32'b0; //slt set 1 if srcA < srcB, but both are signed 2s complement
        4'b0011: alu_result = (srcA < srcB) ? 32'b1 : 32'b0; //sltu set 1 if srcA < srcB
        4'b1001: alu_result = srcA; //lui copy only. srcA is the input for imm U type
        default: alu_result = 32'hDEADBEEF;
    endcase
    end
endmodule
