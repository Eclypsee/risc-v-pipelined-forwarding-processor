`timescale 1ns / 1ps

module OTTER_PROCESSOR_TB;
    // Testbench Signals
    logic CLK;
    logic RST;


    // Instantiate the Processor
    OTTER_PROCESSOR uut ( 
        .CLK(CLK),
        .INTR(),
        .RST(RST),
        .IOBUS_IN(32'b0),
        .IOBUS_OUT(),
        .IOBUS_ADDR(),
        .IOBUS_WR()
    );

    // Clock Generation (50 MHz -> 10 ns period)
    always #10 CLK = ~CLK;

    initial begin
        // Initialize signals
        CLK = 0;
        RST = 1;
        #20;//double clock cycle
        RST = 0;
        // Let the processor run for a while
        #1000;

    end
endmodule