`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/07/2024 02:16:41 PM
// Design Name: 
// Module Name: REG_FILE
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


module REG_FILE(
    input en,
    input [4:0] adr1,
    input [4:0] adr2,
    input [4:0] w_adr,
    input [31:0] w_data,
    input clk,
    
    output [31:0] rs1,
    output [31:0] rs2
    );
    // Create a memory module with 32-bit width and 32 addresses
    logic [31:0] register [0:31];
    // Initialize the memory to be all 0s
    initial begin
        int i;
        for (i=0; i<32; i=i+1) begin
            register[i] = 0;
        end
    end
    
    //async read rs1 and rs2 and
    assign rs1 = register[adr1];
    assign rs2 = register[adr2];
    
    //each clock cycle, check if enable write, then check if we are trying to save to reg 0
    always_ff @(negedge clk) begin //write negative edge 
        if (en) begin
            if (w_adr != 0) begin
                register[w_adr] <= w_data;  
            end
        end
    end
endmodule
