module qspi_shifter 
    (
        // this interfaces with the sram_controller
        input clk,
        input reset,
        input write_enable,
        input enable,
        input [31:0] address,
        input [31:0] data_in,
        output reg [31:0] data_out,
        // this interfaces with the sram chip
        inout sio0,
        inout sio1,
        inout sio2,
        inout sio3,
        output cs_n // TODO: I need to implement the use of the cs_n pin
    );

    reg [2:0] state, sub_state;

    wire [7:0] command;

    always @(*) begin
        if (reset) command = 0'h38;
        else if (write_enable) command = 0'h02;
        else command = 0'h0b; // if we can only run at 33 Hz we will want to transition from HSR to READ and change this command
    end

    always @(posedge clk) begin
        if (enable): begin
            case (state):
                // TODO: this should be the wait command
                3'b000: begin
                end
                // send the command state
                3'b001: begin
                    case (sub_state):
                        // nibble #1
                        3'b000: begin
                            sio0 = command[4];
                            sio1 = command[5];
                            sio2 = command[6];
                            sio3 = command[7];
                            sub_state = 3'b001;
                        end
                        // nibble #2
                        3'b001: begin
                            sio0 = command[0];
                            sio1 = command[1];
                            sio2 = command[2];
                            sio3 = command[3];
                            state = 3'b010;
                            sub_state = 3'b000;
                        end
                        // catching errors
                        default: begin
                            sio0 = 1'b0;
                            sio1 = 1'b0;
                            sio2 = 1'b0;
                            sio3 = 1'b0;
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                    endcase
                end
                // this should be the address state
                3'b010: begin
                    case (sub_state):
                        // nibble #1
                        3'b000: begin
                            sio0 = address[20];
                            sio1 = address[21];
                            sio2 = address[22];
                            sio3 = address[23];
                            sub_state = 3'b001;
                        end
                        // nibble #2
                        3'b001: begin
                            sio0 = address[16];
                            sio1 = address[17];
                            sio2 = address[18];
                            sio3 = address[19];
                            sub_state = 3'b010;
                        end
                        // nibble #3
                        3'b010: begin
                            sio0 = address[12];
                            sio1 = address[13];
                            sio2 = address[14];
                            sio3 = address[15];
                            sub_state = 3'b011;
                        end
                        // nibble #4
                        3'b011: begin
                            sio0 = address[8];
                            sio1 = address[9];
                            sio2 = address[10];
                            sio3 = address[11];
                            sub_state = 3'b100;
                        end
                        // nibble #5
                        3'b100: begin
                            sio0 = address[4];
                            sio1 = address[5];
                            sio2 = address[6];
                            sio3 = address[7];
                            sub_state = 3'b101;
                        end
                        // nibble #6
                        3'b101: begin
                            sio0 = address[0];
                            sio1 = address[1];
                            sio2 = address[2];
                            sio3 = address[3];
                            if (write_enable) state = 3'b101; // if we are writing go to the write state
                            else state = 3'b010; // if we are reading go to the dummy space state
                            sub_state = 3'b000;
                        end
                        // catching errors
                        default: begin
                            sio0 = 1'b0;
                            sio1 = 1'b0;
                            sio2 = 1'b0;
                            sio3 = 1'b0;
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                    endcase
                end
                // this should be the dummy space state
                // basically we want to wait 3 bytes or 6 clock cycles
                // if we can only run at 33 Hz we will want to transition from HSR to READ and only need one byte (or 2 clock cycles)
                3'b011: begin
                    sio0 = 1'bZ;
                    sio1 = 1'bZ;
                    sio2 = 1'bZ;
                    sio3 = 1'bZ;
                    case (sub_state):
                        // nibble #1
                        3'b000: begin
                            sub_state = 3'b001;
                        end
                        // nibble #2
                        3'b001: begin
                            sub_state = 3'b010;
                        end
                        // nibble #3
                        3'b010: begin
                            sub_state = 3'b011;
                        end
                        // nibble #4
                        3'b011: begin
                            sub_state = 3'b100;
                        end
                        // nibble #5
                        3'b100: begin
                            sub_state = 3'b101;
                        end
                        // nibble #6
                        3'b101: begin
                            state = 3'b100;
                            sub_state = 3'b000;
                        end
                        // catching errors
                        default: begin
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                    endcase
                end
                // TODO: this should be the read state
                // since we want to read a word (32 bits, 4 bytes) we will need 8 cycles
                3'b100: begin
                    sio0 = 1'bZ;
                    sio1 = 1'bZ;
                    sio2 = 1'bZ;
                    sio3 = 1'bZ;
                    case (sub_state):
                        // nibble #1
                        3'b000: begin
                            data_out[4] = sio0;
                            data_out[5] = sio1;
                            data_out[6] = sio2;
                            data_out[7] = sio3;
                            sub_state = 3'b001;
                        end
                        // nibble #2
                        3'b001: begin
                            data_out[0] = sio0;
                            data_out[1] = sio1;
                            data_out[2] = sio2;
                            data_out[3] = sio3;
                            sub_state = 3'b010;
                        end
                        // nibble #3
                        3'b010: begin
                            data_out[12] = sio0;
                            data_out[13] = sio1;
                            data_out[14] = sio2;
                            data_out[15] = sio3;
                            sub_state = 3'b011;
                        end
                        // nibble #4
                        3'b011: begin
                            data_out[8] = sio0;
                            data_out[9] = sio1;
                            data_out[10] = sio2;
                            data_out[11] = sio3;
                            sub_state = 3'b100;
                        end
                        // nibble #5
                        3'b100: begin
                            data_out[20] = sio0;
                            data_out[21] = sio1;
                            data_out[22] = sio2;
                            data_out[23] = sio3;
                            sub_state = 3'b101;
                        end
                        // nibble #6
                        3'b101: begin
                            data_out[16] = sio0;
                            data_out[17] = sio1;
                            data_out[18] = sio2;
                            data_out[19] = sio3;
                            sub_state = 3'b110;
                        end
                        // nibble #7
                        3'b110: begin
                            data_out[28] = sio0;
                            data_out[29] = sio1;
                            data_out[30] = sio2;
                            data_out[31] = sio3;
                            sub_state = 3'b111;
                        end
                        // nibble #8
                        3'b111: begin
                            data_out[24] = sio0;
                            data_out[25] = sio1;
                            data_out[26] = sio2;
                            data_out[27] = sio3;
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                        // catching errors
                        default: begin
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                    endcase
                end
                // TODO: this should be the write state
                3'b101: begin
                    case (sub_state):
                        // nibble #1
                        3'b000: begin
                            sio0 = data_in[4];
                            sio1 = data_in[5];
                            sio2 = data_in[6];
                            sio3 = data_in[7];
                            sub_state = 3'b001;
                        end
                        // nibble #2
                        3'b001: begin
                            sio0 = data_in[0];
                            sio1 = data_in[1];
                            sio2 = data_in[2];
                            sio3 = data_in[3];
                            sub_state = 3'b010;
                        end
                        // nibble #3
                        3'b010: begin
                            sio0 = data_in[12];
                            sio1 = data_in[13];
                            sio2 = data_in[14];
                            sio3 = data_in[15];
                            sub_state = 3'b011;
                        end
                        // nibble #4
                        3'b011: begin
                            sio0 = data_in[8];
                            sio1 = data_in[9];
                            sio2 = data_in[10];
                            sio3 = data_in[11];
                            sub_state = 3'b100;
                        end
                        // nibble #5
                        3'b100: begin
                            sio0 = data_in[20];
                            sio1 = data_in[21];
                            sio2 = data_in[22];
                            sio3 = data_in[23];
                            sub_state = 3'b101;
                        end
                        // nibble #6
                        3'b101: begin
                            sio0 = data_in[16];
                            sio1 = data_in[17];
                            sio2 = data_in[18];
                            sio3 = data_in[19];
                            sub_state = 3'b110;
                        end
                        // nibble #7
                        3'b110: begin
                            sio0 = data_in[28];
                            sio1 = data_in[29];
                            sio2 = data_in[30];
                            sio3 = data_in[31];
                            sub_state = 3'b111;
                        end
                        // nibble #8
                        3'b111: begin
                            sio0 = data_in[24];
                            sio1 = data_in[25];
                            sio2 = data_in[26];
                            sio3 = data_in[27];
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                        // catching errors
                        default: begin
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                    endcase
                end
                // this should be the state to set it into quad spi mode
                // we dont need to adjust the mode because we want in sequential mode
                3'b110: begin
                    sio1 = 1'bZ;
                    sio2 = 1'bZ;
                    sio3 = 1'b1;
                    case (sub_state):
                        // bit #1
                        3'b000: begin
                            sio0 = command[7];
                            sub_state = 3'b001;
                        end
                        // bit #2
                        3'b001: begin
                            sio0 = command[6];
                            sub_state = 3'b010;
                        end
                        // bit #3
                        3'b010: begin
                            sio0 = command[5];
                            sub_state = 3'b011;
                        end
                        // bit #4
                        3'b011: begin
                            sio0 = command[4];
                            sub_state = 3'b100;
                        end
                        // bit #5
                        3'b100: begin
                            sio0 = command[3];
                            sub_state = 3'b101;
                        end
                        // bit #6
                        3'b101: begin
                            sio0 = command[2];
                            sub_state = 3'b110;
                        end
                        // bit #7
                        3'b110: begin
                            sio0 = command[1];
                            sub_state = 3'b111;
                        end
                        // bit #8
                        3'b111: begin
                            sio0 = command[0];
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                        default: begin
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                    endcase
                end
                // set the default state
                default: state = 3'b000;
            endcase
        end
        else cs_n = 1'b1;
    end

endmodule