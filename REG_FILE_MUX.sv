`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/09/2024 03:00:46 PM
// Design Name: 
// Module Name: REG_FILE_MUX
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


module REG_FILE_MUX(
    input[31:0] pc_plus_four,
    input[31:0] csr_rd,        
    input[31:0] dout2,      
    input[31:0] alu_result,    
    input[1:0] RF_SEL,       
    output logic[31:0] OUT   
    );
    always_comb 
    begin
        case(RF_SEL)
            0: OUT = pc_plus_four;
            1: OUT = csr_rd;
            2: OUT = dout2;
            3: OUT = alu_result;
            default: OUT = 32'hDEADBEEF;
        endcase
    end
endmodule
