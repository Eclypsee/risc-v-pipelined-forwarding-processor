`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 10/23/2024 02:39:55 PM
// Design Name:
// Module Name: BCG
// Project Name:
// Target Devices:
// Tool Versions:
// Description: Branch Control Generator
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module BCG(
	input logic [31:0] rs1,
	input logic [31:0] rs2,
	input logic [6:0] opcode, // Brought from the Decoder
	input logic [2:0] funct3, // Brought from the Decoder

	output logic [2:0] PC_SEL // Brought from the Decoder
);

	// Branch condition calculations
	assign br_eq = (rs1 == rs2);        	// Equal comparison
	assign br_lt = ($signed(rs1) < $signed(rs2)); // Signed less than
	assign br_ltu = (rs1 < rs2);        	// Unsigned less than

	always_comb begin
    	PC_SEL = 3'b000; // Default: No branch

    	case (opcode)
        	7'b1100011: begin // Branch instructions
            	case (funct3)
                	3'b000: if (br_eq)  PC_SEL = 3'b010; // BEQ
                	3'b001: if (!br_eq) PC_SEL = 3'b010; // BNE
                	3'b100: if (br_lt)  PC_SEL = 3'b010; // BLT
                	3'b101: if (!br_lt) PC_SEL = 3'b010; // BGE
                	3'b110: if (br_ltu) PC_SEL = 3'b010; // BLTU
                	3'b111: if (!br_ltu) PC_SEL = 3'b010; // BGEU
            	endcase
        	end
			7'b1101111: begin//jal
				PC_SEL = 3'b011;//select jal in pc mux
			end
			7'b1100111: begin//jalr
				PC_SEL = 3'b001;
			end
    	endcase
	end

endmodule
