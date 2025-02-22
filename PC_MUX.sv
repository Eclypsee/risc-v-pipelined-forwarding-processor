`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/23/2024 01:11:29 PM
// Design Name: 
// Module Name: PC_MUX
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


module PC_MUX(
    input[31:0] add_four,    // 32-bit input representing the address for PC + 4 (used in normal execution)
    input[31:0] jalr,        // 32-bit input representing the address for jump-and-link-register (jalr) instructions
    input[31:0] branch,      // 32-bit input representing the branch target address
    input[31:0] jal,         // 32-bit input representing the jump-and-link (jal) target address
    input[31:0] mtvec,       // 32-bit input representing the machine trap vector base address (used for interrupts/exceptions)
    input[31:0] mepc,       // 32-bit input representing the machine exception program counter (return address from exception)
    input[2:0] PC_SEL,       // 3-bit control signal for selecting which input to output as the next PC value
    output logic[31:0] OUT   // 32-bit output representing the selected PC value
    );
    
    always_comb 
    begin
        case(PC_SEL)
            0: OUT = add_four;
            1: OUT = jalr;
            2: OUT = branch;
            3: OUT = jal;
            4: OUT = mtvec;
            5: OUT = mepc;
            default: OUT = 32'hDEADBEEF;
        endcase
    end
endmodule
