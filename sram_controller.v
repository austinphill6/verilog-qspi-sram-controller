module sram_controller
    (
        // these are the controls from the cpu
        input [31:0] write_data,
        input [31:0] address,
        input we,
        input access_n,
        input clk,
        output busy_n,
        output [31:0] read_data,
        // this interfaces with the sram chip
        inout sio0,
        inout sio1,
        inout sio2,
        inout sio3,
        output cs_n
    );

    reg [1:0] state, next_state

    // this is going to be our state switching
    always @(posedge clk) begin
        if
    end
endmodule