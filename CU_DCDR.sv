`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 10/28/2024 03:16:24 PM
// Design Name:
// Module Name: CU_DCDR
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
module CU_DCDR(
	//inputs
	input [6:0] opcode, //this is called IR[6:0]
	input [2:0] funct3, //this is called IR[14:12], the exact instruction from the opcode series
	input ir_30, // for instructions with the same opcode AND function, we use this bit to find the instruction
	input int_taken,
	//input br_eq, These go to the pipeline register, logic can be done at high level
	//input br_lt,
	//input br_ltu,
	//outputs
	output logic [3:0] ALU_FUN,
	output logic [1:0] srcA_SEL,
	output logic [2:0] srcB_SEL,
	//output logic [2:0] PC_SEL,
	output logic [1:0] RF_SEL,
    
	output logic RF_WE,
	output logic MEM_WE2,
	output logic MEM_RDEN2
	);
	always_comb begin
    	ALU_FUN = 4'b0;
    	srcA_SEL = 2'b0;
    	srcB_SEL = 3'b0;
    	//PC_SEL = 3'b0;
    	RF_SEL = 2'b0;
    	RF_WE = 1'b0;
    	MEM_WE2 = 1'b0;
    	MEM_RDEN2 = 1'b0;

    	//interrupt check
    	//if(int_taken)
    	//PC_SEL = 4;//mtvec

    	//decoding instructions
    	//else begin
        	case(opcode)
            	// CSR type ///////////////////////////////////////////////////////////////////////////////
            	7'b1110011: begin
                	case(funct3)
                    	//"mret"
                    	3'b000: begin
                        	//PC_SEL = 5;//mepc
                    	end
                    	//"csrrw"
                    	3'b001: begin
                        	RF_SEL = 1;
                        	ALU_FUN = 4'b1001;//copy rs1
                    	end
                    	//"csrrs"
                    	3'b010: begin
                         	RF_SEL = 1;
                         	ALU_FUN = 4'b0110;//or
                         	srcB_SEL = 4;//csr rd
                    	end
                    	//"csrrc"
                    	3'b011: begin
                        	RF_SEL = 1;
                        	ALU_FUN = 4'b0111;//and
                        	srcA_SEL = 2; //not rs1
                        	srcB_SEL = 4;//csr rd
                    	end
                	endcase
            	end
            	// R type /////////////////////////////////////////////////////////////////////////////////
            	7'b0110011: begin
                	///////these hold true for all R types ///////
                	RF_SEL = 2'b11; //alu result->regfile
                	RF_WE = 1'b1;
                	//src_a and src_b select are 0(0 means taken from rs1,rs2)
                	// PC sel is 0 (0 means increment pc)
                	case(funct3)
                    	3'b000: begin
                        	case(ir_30)
                        	//"sub"
                        	1'b1: ALU_FUN = 4'b1000;
                        	//"add"
                        	1'b0: ALU_FUN = 4'b0000;
                        	endcase
                    	end
                    	//"and"
                    	3'b111: ALU_FUN = 4'b0111;
                    	//"or"
                    	3'b110: ALU_FUN = 4'b0110;
                    	//"sll"
                    	3'b001: ALU_FUN = 4'b0001;
                    	//"slt"
                    	3'b010: ALU_FUN = 4'b0010;
                    	//"sltu"
                    	3'b011: ALU_FUN = 4'b0011;
                    	3'b101: begin
                        	case(ir_30)
                        	//"sra"
                        	1'b1: ALU_FUN = 4'b1101;
                        	//"srl"
                        	1'b0: ALU_FUN = 4'b0101;
                        	endcase
                    	end
                    	//"xor"
                    	3'b100: ALU_FUN = 4'b0100;
                	endcase
            	end
            	// I type /////////////////////////////////////////////////////////////////////////////////
            	7'b0010011: begin
                	//for all I type
                	srcA_SEL = 2'b0; //taken directly from reg file, rs1
                	srcB_SEL = 3'b001; // immediate,  I type
                	RF_SEL = 2'b11; //alu result->regfile
                	RF_WE = 1'b1;
                	case(funct3)
                    	//PC_SEL defaults to 0
                    	//"addi"
                    	3'b000: ALU_FUN = 4'b0000;
                    	//"andi"
                    	3'b111: ALU_FUN = 4'b0111;
                    	//"ori"
                    	3'b110: ALU_FUN = 4'b0110;
                    	//"slli"
                    	3'b001: ALU_FUN = 4'b0001;
                    	//"slti"
                    	3'b010: ALU_FUN = 4'b0010;
                    	//"sltiu"
                    	3'b011: ALU_FUN = 4'b0011;
                    	3'b101: begin
                        	case(ir_30)
                        	//"srai"
                        	1'b1: ALU_FUN = 4'b1101;
                        	//"srli"
                        	1'b0: ALU_FUN = 4'b0101;
                        	endcase
                    	end
                    	//"xori"
                    	3'b100: ALU_FUN = 4'b0100;
                	endcase
            	end
            	//"jalr"
            	7'b1100111: begin
                	//src a and b dont matter
                	RF_SEL = 2'b0; // rf sel = PC+4, next instruction saved to reg file so we can jump back to here
                	//PC_SEL = 3'b001; //jalr input
                	RF_WE = 1'b1;
                	//no alu function
            	end
            	//loads
            	7'b0000011: begin
                	RF_WE = 1'b1;
                	srcA_SEL = 2'b0; //rs1
                	srcB_SEL = 3'b001; //I type imm
                	RF_SEL = 2'b10; //2 since we want dout2
                	//PC_SEL defaults to 0;
                	ALU_FUN = 4'b0000; //add
                	MEM_RDEN2 = 1'b1;
                	//"lb", "lbu", "lh", "lhu", "lw" use same controls
            	end
            	// S type /////////////////////////////////////////////////////////////////////////////////
            	7'b0100011: begin
                	//srca select defaults to rs1
                	srcB_SEL = 3'b010;//s type
                	MEM_WE2 = 1'b1;
                	//alu fun defaults to add
                	//PC sel defaults to 0
                	//"sb","sh","sw" all use the same controls
            	end
            	//BRANCH STUFF MOVED TO STAGE 4/5 of pipeline
            	// B type /////////////////////////////////////////////////////////////////////////////////
  /*          	7'b1100011: begin
                	case(funct3)
                    	//"beq"
                    	3'b000: if(br_eq) PC_SEL = 3'b010;// pc sel is 2 since we branch
                    	//"bge"
                    	3'b101: if(!br_lt) PC_SEL = 3'b010;// pc sel is 2 since we branch
                    	//"bgeu"
                    	3'b111: if(!br_ltu) PC_SEL = 3'b010;// pc sel is 2 since we branch
                    	//"blt"
                    	3'b100: if(br_lt) PC_SEL = 3'b010;// pc sel is 2 since we branch
                    	//"bltu"
                    	3'b110: if(br_ltu) PC_SEL = 3'b010;// pc sel is 2 since we branch
                    	//"bne"
                    	3'b001: if(!br_eq) PC_SEL = 3'b010;// pc sel is 2 since we branch
                	endcase
            	end */

            	// U type /////////////////////////////////////////////////////////////////////////////////
            	7'b0110111: begin
                	//"lui"
                	ALU_FUN = 4'b1001;
                	srcA_SEL = 2'b01; // U type imm
                	RF_SEL = 2'b11; // rf sel = 3, which is the alu result, which we want to store to reg file
                	RF_WE = 1'b1;
            	end
            	7'b0010111: begin
                	//"auipc"
                	//alu fun defaults to add(we add U type with pc count)
                	srcA_SEL = 2'b01; //U type imm
                	srcB_SEL = 3'b011; //PC count
                	RF_SEL = 2'b11; //store alu result
                	RF_WE = 1'b1;
            	end

            	// J type /////////////////////////////////////////////////////////////////////////////////
            	7'b1101111: begin
                	//"jal"
                	//alu fun dont matter
                	//srca and b dont matter
                	//PC_SEL = 3'b011; //jal input
                	//rf sel is pc+4
                	RF_WE = 1'b1;
            	end

            	default:; //blank default
        	endcase
    	end
	//end
endmodule
