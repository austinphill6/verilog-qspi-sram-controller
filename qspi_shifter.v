module QSPIShifter 
    (
        // this interfaces with the sram_controller
        input clk,
        input reset,
        input we,
        input re,
        input [31:0] address,
        input [31:0] data_in,
        output reg [31:0] data_out,
        // this interfaces with the sram chip
        inout [3:0] sio,
        output reg cs_n
    );

    reg [2:0] state, sub_state;

    reg [7:0] command;

    // this is some tristate buffer shenanigans
    reg write_to_chip, only_sio0;
    reg sio0_buf, sio1_buf, sio2_buf, sio3_buf;
    assign sio[0] = (write_to_chip | only_sio0) ? sio0_buf : 1'bz;
    assign sio[1] = (write_to_chip & !only_sio0) ? sio1_buf : 1'bz;
    assign sio[2] = (write_to_chip & !only_sio0) ? sio2_buf : 1'bZ;
    assign sio[3] = (write_to_chip & !only_sio0) ? sio3_buf : 1'bZ;

    always @(posedge clk) begin
        if(reset) begin
            cs_n = 1'b1;
            write_to_chip = 0;
            only_sio0 = 0;
            command = 8'h38;
            state = 3'b110;
            sub_state = 3'b000;
        end
        else begin
            case (state)
                // this should be the wait command
                3'b000: begin
                    cs_n = 1'b1;
                    write_to_chip = 0;
                    only_sio0 = 0;
                    case (sub_state)
                        3'b000: begin
                            state = 3'b000;
                            sub_state = 3'b001;
                        end
                        3'b001: begin
                            if (re) begin
                                state = 3'b001;
                                command = 8'h0b;
                            end
                            else if (we) begin
                                state = 3'b001;
                                command = 8'h02;
                            end
                            else state = 3'b000;
                            sub_state = 3'b000;
                        end
                    endcase
                end
                // send the command state
                3'b001: begin
                    cs_n = 1'b0;
                    write_to_chip = 1;
                    //if(!(re|we)) state = 3'b000;
                    case (sub_state)
                        // nibble #1
                        3'b000: begin
                            sio0_buf = command[4];
                            sio1_buf = command[5];
                            sio2_buf = command[6];
                            sio3_buf = command[7];
                            sub_state = 3'b001;
                        end
                        // nibble #2
                        3'b001: begin
                            sio0_buf = command[0];
                            sio1_buf = command[1];
                            sio2_buf = command[2];
                            sio3_buf = command[3];
                            state = 3'b010;
                            sub_state = 3'b000;
                        end
                        // catching errors
                        default: begin
                            sio0_buf = 1'b0;
                            sio1_buf = 1'b0;
                            sio2_buf = 1'b0;
                            sio3_buf = 1'b0;
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                    endcase
                end
                // this should be the address state
                3'b010: begin
                    case (sub_state)
                        // nibble #1
                        3'b000: begin
                            sio0_buf = address[20];
                            sio1_buf = address[21];
                            sio2_buf = address[22];
                            sio3_buf = address[23];
                            sub_state = 3'b001;
                        end
                        // nibble #2
                        3'b001: begin
                            sio0_buf = address[16];
                            sio1_buf = address[17];
                            sio2_buf = address[18];
                            sio3_buf = address[19];
                            sub_state = 3'b010;
                        end
                        // nibble #3
                        3'b010: begin
                            sio0_buf = address[12];
                            sio1_buf = address[13];
                            sio2_buf = address[14];
                            sio3_buf = address[15];
                            sub_state = 3'b011;
                        end
                        // nibble #4
                        3'b011: begin
                            sio0_buf = address[8];
                            sio1_buf = address[9];
                            sio2_buf = address[10];
                            sio3_buf = address[11];
                            sub_state = 3'b100;
                        end
                        // nibble #5
                        3'b100: begin
                            sio0_buf = address[4];
                            sio1_buf = address[5];
                            sio2_buf = address[6];
                            sio3_buf = address[7];
                            sub_state = 3'b101;
                        end
                        // nibble #6
                        3'b101: begin
                            sio0_buf = address[0];
                            sio1_buf = address[1];
                            sio2_buf = address[2];
                            sio3_buf = address[3];
                            if (we) state = 3'b101; // if we are writing go to the write state
                            else if (re) state = 3'b011; // if we are reading go to the dummy space state
                            else state = 3'b000;
                            sub_state = 3'b000;
                        end
                        // catching errors
                        default: begin
                            sio0_buf = 1'b0;
                            sio1_buf = 1'b0;
                            sio2_buf = 1'b0;
                            sio3_buf = 1'b0;
                            state = 3'b000;
                            sub_state = 3'b000;
                        end
                    endcase
                end
                // this should be the dummy space state
                // basically we want to wait 3 bytes or 6 clock cycles
                // if we can only run at 33 Hz we will want to transition from HSR to READ and only need one byte (or 2 clock cycles)
                3'b011: begin
                    write_to_chip = 0;
                    case (sub_state)
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
                // this should be the read state
                // since we want to read a word (32 bits, 4 bytes) we will need 8 cycles
                3'b100: begin
                    case (sub_state)
                        // nibble #1
                        3'b000: begin
                            data_out[4] = sio[0];
                            data_out[5] = sio[1];
                            data_out[6] = sio[2];
                            data_out[7] = sio[3];
                            sub_state = 3'b001;
                        end
                        // nibble #2
                        3'b001: begin
                            data_out[0] = sio[0];
                            data_out[1] = sio[1];
                            data_out[2] = sio[2];
                            data_out[3] = sio[3];
                            sub_state = 3'b010;
                        end
                        // nibble #3
                        3'b010: begin
                            data_out[12] = sio[0];
                            data_out[13] = sio[1];
                            data_out[14] = sio[2];
                            data_out[15] = sio[3];
                            sub_state = 3'b011;
                        end
                        // nibble #4
                        3'b011: begin
                            data_out[8] = sio[0];
                            data_out[9] = sio[1];
                            data_out[10] = sio[2];
                            data_out[11] = sio[3];
                            sub_state = 3'b100;
                        end
                        // nibble #5
                        3'b100: begin
                            data_out[20] = sio[0];
                            data_out[21] = sio[1];
                            data_out[22] = sio[2];
                            data_out[23] = sio[3];
                            sub_state = 3'b101;
                        end
                        // nibble #6
                        3'b101: begin
                            data_out[16] = sio[0];
                            data_out[17] = sio[1];
                            data_out[18] = sio[2];
                            data_out[19] = sio[3];
                            sub_state = 3'b110;
                        end
                        // nibble #7
                        3'b110: begin
                            data_out[28] = sio[0];
                            data_out[29] = sio[1];
                            data_out[30] = sio[2];
                            data_out[31] = sio[3];
                            sub_state = 3'b111;
                        end
                        // nibble #8
                        3'b111: begin
                            data_out[24] = sio[0];
                            data_out[25] = sio[1];
                            data_out[26] = sio[2];
                            data_out[27] = sio[3];
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
                // this should be the write state
                3'b101: begin
                    case (sub_state)
                        // nibble #1
                        3'b000: begin
                            sio0_buf = data_in[4];
                            sio1_buf = data_in[5];
                            sio2_buf = data_in[6];
                            sio3_buf = data_in[7];
                            sub_state = 3'b001;
                        end
                        // nibble #2
                        3'b001: begin
                            sio0_buf = data_in[0];
                            sio1_buf = data_in[1];
                            sio2_buf = data_in[2];
                            sio3_buf = data_in[3];
                            sub_state = 3'b010;
                        end
                        // nibble #3
                        3'b010: begin
                            sio0_buf = data_in[12];
                            sio1_buf = data_in[13];
                            sio2_buf = data_in[14];
                            sio3_buf = data_in[15];
                            sub_state = 3'b011;
                        end
                        // nibble #4
                        3'b011: begin
                            sio0_buf = data_in[8];
                            sio1_buf = data_in[9];
                            sio2_buf = data_in[10];
                            sio3_buf = data_in[11];
                            sub_state = 3'b100;
                        end
                        // nibble #5
                        3'b100: begin
                            sio0_buf = data_in[20];
                            sio1_buf = data_in[21];
                            sio2_buf = data_in[22];
                            sio3_buf = data_in[23];
                            sub_state = 3'b101;
                        end
                        // nibble #6
                        3'b101: begin
                            sio0_buf = data_in[16];
                            sio1_buf = data_in[17];
                            sio2_buf = data_in[18];
                            sio3_buf = data_in[19];
                            sub_state = 3'b110;
                        end
                        // nibble #7
                        3'b110: begin
                            sio0_buf = data_in[28];
                            sio1_buf = data_in[29];
                            sio2_buf = data_in[30];
                            sio3_buf = data_in[31];
                            sub_state = 3'b111;
                        end
                        // nibble #8
                        3'b111: begin
                            sio0_buf = data_in[24];
                            sio1_buf = data_in[25];
                            sio2_buf = data_in[26];
                            sio3_buf = data_in[27];
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
                    cs_n = 1'b0;
                    write_to_chip = 1;
                    only_sio0 = 1;
                    command = 8'h38;
                    case (sub_state)
                        // bit #1
                        3'b000: begin
                            sio0_buf = command[7];
                            sub_state = 3'b001;
                        end
                        // bit #2
                        3'b001: begin
                            sio0_buf = command[6];
                            sub_state = 3'b010;
                        end
                        // bit #3
                        3'b010: begin
                            sio0_buf = command[5];
                            sub_state = 3'b011;
                        end
                        // bit #4
                        3'b011: begin
                            sio0_buf = command[4];
                            sub_state = 3'b100;
                        end
                        // bit #5
                        3'b100: begin
                            sio0_buf = command[3];
                            sub_state = 3'b101;
                        end
                        // bit #6
                        3'b101: begin
                            sio0_buf = command[2];
                            sub_state = 3'b110;
                        end
                        // bit #7
                        3'b110: begin
                            sio0_buf = command[1];
                            sub_state = 3'b111;
                        end
                        // bit #8
                        3'b111: begin
                            sio0_buf = command[0];
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
                default: begin
                    write_to_chip = 0;
                    only_sio0 = 0;
                    state = 3'b000;
                    sub_state = 3'b000;
                end
            endcase
        end
    end

endmodule