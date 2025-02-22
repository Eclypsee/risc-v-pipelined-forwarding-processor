`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Five Knights at Freddy's
// Engineer:  J. Calleness
// edited by John Fortnite
// Create Date: 01/04/2019 04:32:12 PM
// Design Name:
// Module Name: OTTER_PROCESSOUIE
// Project Name:
// Target Devices:its so over
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// :(



  typedef enum logic [6:0] {
       	LUI  	= 7'b0110111,
       	AUIPC	= 7'b0010111,
       	JAL  	= 7'b1101111,
       	JALR 	= 7'b1100111,
       	BRANCH   = 7'b1100011,
       	LOAD 	= 7'b0000011,
       	STORE	= 7'b0100011,
       	OP_IMM   = 7'b0010011,
       	OP   	= 7'b0110011,
       	SYSTEM   = 7'b1110011
 } opcode_t;
   	 
typedef struct packed{
	opcode_t opcode;
	logic [4:0] rs1_addr;
	logic [4:0] rs2_addr;
	logic [4:0] rd_addr;
	logic rs1_used;
	logic rs2_used;
	logic rd_used;
	logic [3:0] alu_fun;
	logic memWrite;
	logic memRead2;
	logic regWrite;
	logic [1:0] rf_wr_sel;
	logic [2:0] mem_type;
	logic [31:0] pc;
	logic [31:0] J;
	logic [31:0] B;
	logic [31:0] I;
	logic [31:0] rs1;
	logic [31:0] rs2;
	logic [31:0] ALU_src_A;
	logic [31:0] ALU_src_B;
	logic [31:0] ALU_Result;
	logic [31:0] pcPLUS4;
	logic [31:0] ir;
} instr_t;

	instr_t if_de_inst, de_ex_inst, ex_mem_inst, mem_wb_inst;

module OTTER_PROCESSOR(input CLK,
            	input INTR,
            	input RST,
            	input [31:0] IOBUS_IN,
            	output [31:0] IOBUS_OUT,
            	output [31:0] IOBUS_ADDR,
            	output logic IOBUS_WR
);      	 
wire [31:0] PC_MUX_OUT, jalr, branch, jal,S,U;
wire [31:0]pc;
wire SIGN;

wire pcWrite;
wire memWrite;
wire [1:0]SIZE;
wire [2:0]PC_SEL;
wire MEM_RDEN1, MEM_RDEN2, MEM_WE2;
//==== Instruction Fetch ==========================================
// logic pcWrite; 	//Hardwired high, assuming no hazards
// logic MEM_RDEN1; 	//Fetch new instruction every cycle
logic lw_stall;
PC_MUX PC_MUX(
	.add_four(pc + 32'h4),
	.jalr(jalr),
	.branch(branch),
	.jal(jal),
	.mtvec('b0), //for when we implement interrupts maybe
	.mepc('b0), //above
	.PC_SEL(PC_SEL),
	.OUT(PC_MUX_OUT)
);
	
PC PC(
	.PC_DIN(PC_MUX_OUT),
	.clk(CLK),
	.PC_WE(pcWrite),
	.PC_RST(RST),
	.PC_COUNT(pc)
);
wire [31:0]MEM_ADDR2, MEM_DIN2, MEM_IO, DOUT1, DOUT2;
	
OTTER_mem_byte Memory(
	.MEM_CLK(CLK),
	.MEM_READ1(MEM_RDEN1),
	.MEM_READ2(MEM_RDEN2),
	.MEM_WRITE2(MEM_WE2),
	.MEM_ADDR1(pc),
	.MEM_ADDR2(MEM_ADDR2),
	.MEM_DIN2(MEM_DIN2),
	.MEM_SIZE(SIZE),
	.MEM_SIGN(SIGN),
	.IO_IN(MEM_IO),
	.IO_WR(IOBUS_WR),
	.MEM_DOUT1(DOUT1),
	.MEM_DOUT2(DOUT2)
);

////////BRANCH_HAZARD_HANDLING/////////////////////////////
logic flush;
logic flush_hold;

assign flush = (PC_SEL != 0) ? 1'b1: 1'b0;
/////////////////////////////////////////////////


logic [31:0] decode_pc;
	always_ff @(posedge CLK) begin
			if(!lw_stall)
				decode_pc <= pc;
		flush_hold <= flush;
			      	 
end


assign pcWrite = !lw_stall;
assign MEM_RDEN1 = !lw_stall;

 
//==== Instruction Decode =============================================  
//
//alu
logic [31:0] ALU_RESULT;
logic [3:0] struct_alu_fun;
logic [1:0] struct_rf_wr_sel;
logic struct_regWrite, struct_memWrite, struct_memRead2;
//reg file
logic [31:0] struct_rs1, struct_rs2;
//immed gen
logic [31:0] struct_I, struct_J, struct_B;
//alu muxes
logic [31:0] struct_ALU_src_A, struct_ALU_src_B; 

//DECODER MAKING
wire [1:0]srcA_SEL;
wire [2:0]srcB_SEL;

	assign if_de_inst.pc = decode_pc;
	assign if_de_inst.ir = DOUT1;
	assign if_de_inst.pcPLUS4 = decode_pc + 4;
	assign if_de_inst.rs1_addr = DOUT1[19:15];
	assign if_de_inst.rs2_addr = DOUT1[24:20];
	assign if_de_inst.rd_addr = DOUT1[11:7];  
	assign if_de_inst.mem_type = DOUT1[14:12];
	assign if_de_inst.opcode = opcode_t'(DOUT1[6:0]);
	assign if_de_inst.rs1_used=	if_de_inst.rs1_addr != 0
								&& if_de_inst.opcode != LUI
								&& if_de_inst.opcode != AUIPC
								&& if_de_inst.opcode != JAL;
	assign if_de_inst.rs2_used= if_de_inst.rs2_addr != 0
								&& if_de_inst.opcode != LUI
								&& if_de_inst.opcode != AUIPC
								&& if_de_inst.opcode != JAL
								&& if_de_inst.opcode != JALR
								&& if_de_inst.opcode != LOAD
								&& if_de_inst.opcode != OP_IMM
								&& if_de_inst.opcode != STORE;

CU_DCDR Decoder (
	.funct3(if_de_inst.ir[14:12]),
	.ir_30(if_de_inst.ir[30]),
	.opcode(if_de_inst.ir[6:0]),
	.int_taken(1'b0), //hardwired to low bc no interrupts implemented in pipelined design
	.ALU_FUN(struct_alu_fun),
	.srcA_SEL(srcA_SEL),
	.srcB_SEL(srcB_SEL),
	.RF_SEL(struct_rf_wr_sel),
	.RF_WE(struct_regWrite),
	.MEM_WE2(struct_memWrite),
	.MEM_RDEN2(struct_memRead2)
);

wire [31:0]w_data;

//Register File making with muxes and stuff
REG_FILE_MUX RegFileMux(
	.pc_plus_four(mem_wb_inst.pcPLUS4),
	.csr_rd(), //not used is interrupt stuff
	.dout2(DOUT2),
	.alu_result(mem_wb_inst.ALU_Result),
	.RF_SEL(mem_wb_inst.rf_wr_sel),
	.OUT(w_data)
);

REG_FILE Registers(
	.en(mem_wb_inst.regWrite),
	.adr1(if_de_inst.rs1_addr),
	.adr2(if_de_inst.rs2_addr),
	.w_adr(mem_wb_inst.rd_addr),
	.w_data(w_data),
	.clk(CLK),
	.rs1(struct_rs1),
	.rs2(struct_rs2)
);

//Immediate Generator
IMMED_GEN ImmGen(
	.IR(if_de_inst.ir[31:7]),
	.U(U),
	.I(struct_I),
	.S(S),
	.J(struct_J),
	.B(struct_B)
);

ALU_SRCA_MUX ALU_A_MUX(
	.rs1(struct_rs1),
	.Utype(U),
	.NOT_rs1(!struct_rs1),
	.srcA_SEL(srcA_SEL),
	.ALU_srcA(struct_ALU_src_A)
);

ALU_SRCB_MUX ALU_B_MUX
(
	.rs2(struct_rs2),
	.Itype(struct_I),
	.Stype(S),
	.PC(if_de_inst.pc),
	.csr_RD(),
	.srcB_SEL(srcB_SEL),
	.ALU_srcB(struct_ALU_src_B)
);

	always_ff@(posedge CLK)//sequentially run line by line
	begin

		de_ex_inst <= if_de_inst; //putting the info from reg inbetween IF and DEC into the reg inbetween DE and EX
		
		//decode out
		de_ex_inst.alu_fun <= struct_alu_fun;
		de_ex_inst.rf_wr_sel <= struct_rf_wr_sel;
		de_ex_inst.regWrite <= struct_regWrite;
		de_ex_inst.memWrite <= struct_memWrite;
		de_ex_inst.memRead2 <= struct_memRead2;

		//reg file out
		de_ex_inst.rs1 <= struct_rs1;
		de_ex_inst.rs2 <= struct_rs2;

		//immed gen
		de_ex_inst.I <= struct_I;
		de_ex_inst.J <= struct_J;
		de_ex_inst.B <= struct_B;

		//alu A mux
		de_ex_inst.ALU_src_A <= struct_ALU_src_A;

		//alu B mux
		de_ex_inst.ALU_src_B <= struct_ALU_src_B;
		
		if(lw_stall || flush || flush_hold) begin
			de_ex_inst.regWrite <= 1'b0;
			de_ex_inst.memWrite <= 1'b0;
			de_ex_inst.ir <= 32'b0;
			de_ex_inst.rs1 <= 32'b0;
			de_ex_inst.rs2 <= 32'b0;
			de_ex_inst.opcode <= 7'b0;

		end

	end
    
//==== Execute ============================================================
logic [1:0] forwardA;
logic [1:0] forwardB;
logic [1:0] forwardtoStore;
logic [31:0] ALU_fwd_muxA, ALU_fwd_muxB,Branch_rs1_fwd, Branch_rs2_fwd;

always_comb begin
	case(forwardA)//rs1 forwarding mux for BAG
		0: Branch_rs1_fwd = de_ex_inst.rs1;
		1: Branch_rs1_fwd = ex_mem_inst.ALU_Result;
		2: Branch_rs1_fwd = mem_wb_inst.ALU_Result;
		3: Branch_rs1_fwd = DOUT2;
		default: Branch_rs1_fwd = 32'hDEADBEEF;
	endcase
end

always_comb begin
	case(forwardB)//rs1 forwarding mux for BAG
		0: Branch_rs2_fwd = de_ex_inst.rs2;
		1: Branch_rs2_fwd = ex_mem_inst.ALU_Result;
		2: Branch_rs2_fwd = mem_wb_inst.ALU_Result;
		3: Branch_rs2_fwd = DOUT2;
		default: Branch_rs2_fwd = 32'hDEADBEEF;
	endcase
end

BAG BAG(
	.PC(de_ex_inst.pc),
	.J_type(de_ex_inst.J),
	.B_type(de_ex_inst.B),
	.I_type(de_ex_inst.I),
	.rs1(Branch_rs1_fwd),
	.jal(jal),
	.branch(branch),
	.jalr(jalr)
);

BCG BCG(
	.rs1(Branch_rs1_fwd),
	.rs2(Branch_rs2_fwd),
	.opcode(de_ex_inst.opcode),
	.funct3(de_ex_inst.ir[14:12]),
	.PC_SEL(PC_SEL)
);


always_comb begin
	case(forwardA)
		0: ALU_fwd_muxA = de_ex_inst.ALU_src_A;
		1: ALU_fwd_muxA = ex_mem_inst.ALU_Result;
		2: ALU_fwd_muxA = mem_wb_inst.ALU_Result;
		3: ALU_fwd_muxA = DOUT2;
		default: ALU_fwd_muxA = 32'hDEADBEEF;
	endcase
end

always_comb begin
	case(forwardB)
		0: ALU_fwd_muxB = de_ex_inst.ALU_src_B;
		1: ALU_fwd_muxB = ex_mem_inst.ALU_Result;
		2: ALU_fwd_muxB = mem_wb_inst.ALU_Result;
		3: ALU_fwd_muxB = DOUT2;
		default: ALU_fwd_muxB = 32'hDEADBEEF;
	endcase
end

ALU ALU(
	.srcA(ALU_fwd_muxA),
	.srcB(ALU_fwd_muxB),
	.alu_fun(de_ex_inst.alu_fun),
	.alu_result(ALU_RESULT) //system verilog will make a temporary ALU_RESULT logic for us. 
);

 	always_ff@(posedge CLK)
 	begin
      	ex_mem_inst <= de_ex_inst;
      	ex_mem_inst.ALU_Result <= ALU_RESULT;
		if(forwardtoStore==2'b01) ex_mem_inst.rs2 <= ex_mem_inst.ALU_Result;
		if(forwardtoStore==2'b10) ex_mem_inst.rs2 <= mem_wb_inst.ALU_Result;
 	end

//==== Memory =======================================================
    
	assign MEM_WE2 = ex_mem_inst.memWrite;
	assign MEM_RDEN2 = ex_mem_inst.memRead2;
	assign SIZE =  ex_mem_inst.mem_type[1:0];
	assign SIGN = ex_mem_inst.mem_type[2];
	assign MEM_DIN2 = ex_mem_inst.rs2;
	assign MEM_ADDR2 = ex_mem_inst.ALU_Result;
	assign IOBUS_ADDR = ex_mem_inst.ALU_Result;
	assign IOBUS_OUT = ex_mem_inst.rs2;
    
	always_ff@(posedge CLK)
	begin
     	mem_wb_inst <= ex_mem_inst;
	end
	 
//==== Write Back ==================================================

//No need to write anything here as the mem_wb_inst stuff that goes to the reg file is already
//hardwired


///////////////////Hazard Handler/////////////////////////////////////////////////
DataHazardHandler DataHazardHandler(
	.de_ex_rs1_used(de_ex_inst.rs1_used),
	.de_ex_rs2_used(de_ex_inst.rs2_used),
	.if_de_rs1_used(if_de_inst.rs1_used),
	.if_de_rs2_used(if_de_inst.rs2_used),
	.if_de_rs1addr(if_de_inst.rs1_addr),
	.if_de_rs2addr(if_de_inst.rs2_addr),
	.de_ex_rs1addr(de_ex_inst.rs1_addr),
	.de_ex_rs2addr(de_ex_inst.rs2_addr),
	.ex_mem_addr(ex_mem_inst.rd_addr),
	.mem_wb_addr(mem_wb_inst.rd_addr),
	.de_ex_addr(de_ex_inst.rd_addr),
	.de_ex_opcode(de_ex_inst.opcode),
	.ex_mem_opcode(ex_mem_inst.opcode),
	.mem_wb_opcode(mem_wb_inst.opcode),
	.ex_mem_RegWrite(ex_mem_inst.regWrite),
	.mem_wb_RegWrite(mem_wb_inst.regWrite),
	//outputs
	.forwardtoStore(forwardtoStore),
	.lw_stall(lw_stall),
	.forwardA(forwardA),
	.forwardB(forwardB)

);
//////////////////////////////////////////////////////////////////////////////////

endmodule
