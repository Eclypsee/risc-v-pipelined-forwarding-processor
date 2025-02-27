`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  :(((((
// 
// Create Date: 02/16/2025 05:47:03 PM
// Design Name: 
// Module Name: DataHazardHandler
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


module DataHazardHandler(
input de_ex_rs1_used, de_ex_rs2_used, if_de_rs1_used, if_de_rs2_used,
input [4:0] if_de_rs1addr, if_de_rs2addr, de_ex_rs1addr, de_ex_rs2addr, 
input [4:0] ex_mem_addr, mem_wb_addr, de_ex_addr,
input [6:0] de_ex_opcode, mem_wb_opcode,//ex_mem_opcode //checking if lw instruction
input ex_mem_RegWrite, mem_wb_RegWrite, //Register WE is high for these instructions
output logic lw_stall, //if we need to forward for RAW 1 inst above from a lw instruction we need to stall
output logic [1:0] forwardA, forwardB,//going to the SEL input of the two Muxes going into the ALU
output logic [1:0]forwardtoStore
//for forward, 0 means no forwarding, 1 means forward from ALU, 2 means forward from Memory
);


always_comb
begin

    forwardA = 2'b00;
    forwardB = 2'b00;
    forwardtoStore = 2'b00;
    lw_stall = 1'b0;

    //execution hazards, we forward from ex_mem_struct
    if((ex_mem_RegWrite && (ex_mem_addr != 'd0)) && (ex_mem_addr == de_ex_rs1addr) && de_ex_rs1_used)
        forwardA = 2'b01;
    if((ex_mem_RegWrite && (ex_mem_addr != 'd0)) && (ex_mem_addr == de_ex_rs2addr) && de_ex_rs2_used)
        forwardB = 2'b01;

    //memory hazards, we forward from mem_wb_struct
    if(((mem_wb_RegWrite && (mem_wb_addr != 0)) && (mem_wb_addr == de_ex_rs1addr))&& (!((ex_mem_RegWrite && (ex_mem_addr != 'd0)) & (ex_mem_addr == de_ex_rs1addr))) && de_ex_rs1_used)begin
        forwardA = 2'b10;
        if(mem_wb_opcode==7'b0000011)
            forwardA = 2'b11;
    end
    if(((mem_wb_RegWrite && (mem_wb_addr != 0)) && (mem_wb_addr == de_ex_rs2addr))&& (!((ex_mem_RegWrite && (ex_mem_addr != 'd0)) & (ex_mem_addr == de_ex_rs2addr))) && de_ex_rs2_used)begin
        forwardB = 2'b10;
        if(mem_wb_opcode==7'b0000011)
            forwardB = 2'b11;
    end

    //lw stall
    if((de_ex_addr!=0&&de_ex_opcode == 7'b0000011)&&((if_de_rs1addr==de_ex_addr&&if_de_rs1_used)||(if_de_rs2addr==de_ex_addr&&if_de_rs2_used)))
        lw_stall=1'b1;


    //RaW for store word hazards
    if(de_ex_opcode == 7'b0100011 && (ex_mem_RegWrite && (ex_mem_addr != 'd0)) && (ex_mem_addr == de_ex_rs2addr))
        forwardtoStore = 2'b01;
    if(de_ex_opcode == 7'b0100011 && ((mem_wb_RegWrite && (mem_wb_addr != 0)) && (mem_wb_addr == de_ex_rs2addr))&& (!((ex_mem_RegWrite && (ex_mem_addr != 'd0)) && (ex_mem_addr == de_ex_rs2addr))))
        forwardtoStore = 2'b10;

end
endmodule
//////////////////////////