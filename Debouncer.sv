`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: John Elden Ring
// 
// Create Date: 03/10/2022 11:34:01 AM
// Module Name: Debouncer
// Project Name: Button Debouncer for OTTER 
// Target Devices: Basys3
// Description: Debounces a button input. Input is initially debounced with double
//                registers at slower (800 Hz) clock to remove high freq bounce. 
//                A one_shot pulse at the slower clock is created for triggering
//                a shorter one_shot output using the higher freq clock (50 MHz)
//              DB_BTN is the debounced button with a 1.25 ms pulse (800 Hz)
//              OneShot is an adjustable length pulse that is triggered  
//                  once for every button press. Initially 80 ns for OTTER
// Revision:
// Revision 0.01 - File Created
//          0.02 - Update simulation to wait for slow clock to become valid
//               - Change 400 Hz clock to 800 Hz for quicker response
//               - Change one shot from 40ns to 80ns with shift reg
//          0.03 - Change from 100 MHz input clock to 50 MHz input clock
//               - DB_BTN is left as a debugging output only 
//////////////////////////////////////////////////////////////////////////////////


module Debouncer(
    input CLK_50,
    input RST,
    input BTN,
    //output DB_BTN,
    output OneShot
    );
    
    // clock divider signals
    logic clk_800z;
    logic [31:0] clock_div_counter;
    
    // debouncer register signals
    logic db_dff_1, db_dff_2;
    logic db_onepulse;
    
    // oneshot register signals
    logic [4:0] oneshot_reg;
    
    // create a 800 Hz clock from 50 MHz input clock
    always_ff @(posedge CLK_50) begin
        
        if (RST == 1'b1) begin
            clock_div_counter <= 0;
            clk_800z <= 0;
        end
        else
            clock_div_counter <= clock_div_counter + 1;
        
        // 31250 @ 50 MHz is half a period of 800 Hz.
        if (clock_div_counter >= 31249) begin    
            clock_div_counter <= 0;
            clk_800z <= ~clk_800z;
        end
        
    end
    
    // Debounce input signal with double buffer registers at slow clock
    always_ff @(posedge clk_800z) begin
       db_dff_1 <= BTN;
       db_dff_2 <= db_dff_1;
    end
    
    // create oneshot trigger from debounce registers at 800 Hz clock (1.25 ms)
    assign db_onepulse = db_dff_1 & ~db_dff_2;
    //assign DB_BTN = db_onepulse;
    
    // create an 80ns OneShot with 4 high speed registers in series
    always_ff @(posedge CLK_50) begin
        oneshot_reg <= {oneshot_reg[3:0],db_onepulse};
    end

    // create short pulse one shot from high speed registers
    assign OneShot = oneshot_reg[0] & ~oneshot_reg[4];     
    
endmodule