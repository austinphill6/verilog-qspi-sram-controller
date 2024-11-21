`timescale 1ps / 1ps

`define STRLEN 32

module testbench;
// This tasks are used to check if a given test has passed and if failed it will // return a designated text less than 32 characters; if passed it will display // "passed" and will increase the passed counter 

    task passTest;
        input actualOut, expectedOut;
        input [`STRLEN*8:0] testType;
        inout [7:0] passed;

        if(actualOut == expectedOut) begin $display ("%s passed", testType); passed = passed + 1; end
        else $display ("%s failed: 0x%x should be 0x%x", testType, actualOut, expectedOut);
    endtask

// confirm that all tests passed if the number of passed test is equal to the // total number of tests

    task allPassed;
        input [7:0] passed;
        input [7:0] numTests;

        if(passed == numTests) $display ("All tests passed");
        else $display("Some tests failed: %d of %d passed", passed, numTests);
    endtask

// Testbench regs
    reg [7:0] passed;
    reg [63:0] counter;

// Inputs
    reg clk, we, re, reset;
    reg [31:0] address, data_in;

// Outputs
    wire cs_n, working_n;
    wire [31:0] data_out;

// Inouts
    wire [3:0] sio;

// Instantiate the Unit Under Test (UUT)
    QSPIShifter uut
    (
        .clk(clk),
        .we(we),
        .re(re),
        .reset(reset),
        .address(address),
        .data_in(data_in),
        .cs_n(cs_n),
        .data_out(data_out),
        .sio(sio)
	);
   
    always begin
        #10; clk = 1; #10; clk = 0;
    end

    always @(posedge clk) begin
        counter <= counter + 1;
        if (counter >= 64'd100) $finish;
    end

    always @(posedge cs_n) begin
        re = 0;
        we = 0;
    end

    initial
    begin
        // these are for the waveform generator
        $dumpfile("test.vcd");
        $dumpvars(0, testbench);

        // Initialize Inputs
        counter = 0;
        we = 0;
        re = 0;
        address = 0;
        data_in = 0;
        reset = 1;
        passed = 0;
        
        #20
        reset = 0;

        #500

        re = 1;
        address = 32'd111;
        #700

        we = 1;
        data_in = 32'd100;
        address = 32'd333;
        // sio[0] = 1;
        // sio[1] = 0;
        // sio[2] = 1;
        // sio[3] = 1;
        #300

        // Add stimulus here
        // Test case 1
        passTest(clk, 1'b0, "Test compilation", passed);

        allPassed(passed, 1);
    end

endmodule