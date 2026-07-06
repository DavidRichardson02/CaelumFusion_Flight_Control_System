`timescale 1ns / 1ps
`default_nettype none


//==============================================================================
// i2c_simple_sensor_model
//------------------------------------------------------------------------------
// Lightweight open-drain I2C responder.
//
// Supported behavior:
//   - START/STOP recognition
//   - 7-bit address match + R/W decoding
//   - ACK for matching address and for subsequent write bytes
//   - streaming deterministic read bytes
//   - repeated START recovery
//
// Not modeled:
//   - exact register map semantics
//   - clock stretching
//   - bus timing limits
//==============================================================================
module i2c_simple_sensor_model #(
    parameter [6:0] I2C_ADDR   = 7'h47,
    parameter [7:0] READ_BYTE0 = 8'hAA,
    parameter [7:0] READ_BYTE1 = 8'h55,
    parameter [7:0] READ_BYTE2 = 8'h12,
    parameter [7:0] READ_BYTE3 = 8'h34,
    parameter [7:0] READ_BYTE4 = 8'h56,
    parameter [7:0] READ_BYTE5 = 8'h78,
    parameter       ONE_WRITE_THEN_ADDR = 1'b0
)(
    inout wire scl,
    inout wire sda
);
    localparam [2:0] ST_IDLE      = 3'd0;
    localparam [2:0] ST_ADDR      = 3'd1;
    localparam [2:0] ST_ADDR_ACK  = 3'd2;
    localparam [2:0] ST_WRITE     = 3'd3;
    localparam [2:0] ST_WRITE_ACK = 3'd4;
    localparam [2:0] ST_READ      = 3'd5;
    localparam [2:0] ST_READ_ACK  = 3'd6;

    reg       sda_drive_low;
    reg [2:0] state;
    reg [7:0] shift_reg;
    reg [2:0] bit_count;
    reg       selected;
    reg       rw_latched;
    reg [2:0] rd_index;
    reg [2:0] tx_bit_index;
    reg       ignore_ack_release_stop;

    integer start_count;
    integer stop_count;
    integer addr_match_count;
    integer write_byte_count;
    integer read_data_count;

    assign sda = sda_drive_low ? 1'b0 : 1'bz;

    function [7:0] rd_mux;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: rd_mux = READ_BYTE0;
                3'd1: rd_mux = READ_BYTE1;
                3'd2: rd_mux = READ_BYTE2;
                3'd3: rd_mux = READ_BYTE3;
                3'd4: rd_mux = READ_BYTE4;
                default: rd_mux = READ_BYTE5;
            endcase
        end
    endfunction

    wire [7:0] current_rd_byte;
    wire [7:0] next_shift_byte;
    assign current_rd_byte = rd_mux(rd_index);
    assign next_shift_byte = {shift_reg[6:0], sda};

    task reset_fsm;
        begin
            sda_drive_low   = 1'b0;
            state           = ST_IDLE;
            shift_reg       = 8'h00;
            bit_count       = 3'd0;
            selected        = 1'b0;
            rw_latched      = 1'b0;
            rd_index        = 3'd0;
            tx_bit_index    = 3'd7;
            ignore_ack_release_stop = 1'b0;
            start_count     = 0;
            stop_count      = 0;
            addr_match_count = 0;
            write_byte_count = 0;
            read_data_count  = 0;
        end
    endtask

    initial begin
        reset_fsm;
    end

    // START condition: SDA falling while SCL high.
    always @(negedge sda) begin
        if (scl === 1'b1) begin
            start_count    = start_count + 1;
            sda_drive_low <= 1'b0;
            state         <= ST_ADDR;
            shift_reg     <= 8'h00;
            bit_count     <= 3'd0;
            selected      <= 1'b0;
            rw_latched    <= 1'b0;
            rd_index      <= 3'd0;
            tx_bit_index  <= 3'd7;
        end
    end

    // STOP condition: SDA rising while SCL high.
    always @(posedge sda) begin
        if (scl === 1'b1) begin
            if (ignore_ack_release_stop) begin
                ignore_ack_release_stop = 1'b0;
            end else begin
                stop_count     = stop_count + 1;
                sda_drive_low <= 1'b0;
                state         <= ST_IDLE;
                bit_count     <= 3'd0;
                tx_bit_index  <= 3'd7;
            end
        end
    end

    // Prepare ACK/data while SCL low.
    always @(negedge scl) begin
        case (state)
            ST_ADDR_ACK,
            ST_WRITE_ACK: begin
                if (selected) begin
                    sda_drive_low <= 1'b1;
                    ignore_ack_release_stop = 1'b1;
                end else begin
                    sda_drive_low <= 1'b0;
                end
            end

            ST_READ: begin
                if (current_rd_byte[tx_bit_index] == 1'b0)
                    sda_drive_low <= 1'b1;
                else
                    sda_drive_low <= 1'b0;
            end

            default: begin
                sda_drive_low <= sda_drive_low;
            end
        endcase
    end

    // Sample/control on SCL rising edge.
    always @(posedge scl) begin
        case (state)
            ST_ADDR: begin
                shift_reg <= next_shift_byte;
                if (bit_count == 3'd7) begin
                    selected   <= (next_shift_byte[7:1] == I2C_ADDR);
                    rw_latched <= next_shift_byte[0];
                    if (next_shift_byte[7:1] == I2C_ADDR)
                        addr_match_count = addr_match_count + 1;
                    state      <= ST_ADDR_ACK;
                    bit_count  <= 3'd0;
                end else begin
                    bit_count <= bit_count + 3'd1;
                end
            end

            ST_ADDR_ACK: begin
                sda_drive_low <= 1'b0;
                if (selected) begin
                    if (rw_latched) begin
                        tx_bit_index <= 3'd7;
                        state        <= ST_READ;
                    end else begin
                        state        <= ST_WRITE;
                    end
                end else begin
                    state <= ST_IDLE;
                end
            end

            ST_WRITE: begin
                shift_reg <= next_shift_byte;
                if (bit_count == 3'd7) begin
                    if (selected)
                        write_byte_count = write_byte_count + 1;
                    state     <= ST_WRITE_ACK;
                    bit_count <= 3'd0;
                end else begin
                    bit_count <= bit_count + 3'd1;
                end
            end

            ST_WRITE_ACK: begin
                sda_drive_low <= 1'b0;
                if (ONE_WRITE_THEN_ADDR && selected) begin
                    state        <= ST_ADDR;
                    shift_reg    <= 8'h00;
                    bit_count    <= 3'd0;
                    selected     <= 1'b0;
                    rw_latched   <= 1'b0;
                    rd_index     <= 3'd0;
                    tx_bit_index <= 3'd7;
                end else begin
                    state        <= ST_WRITE;
                end
            end

            ST_READ: begin
                if (tx_bit_index == 3'd0) begin
                    read_data_count = read_data_count + 1;
                    sda_drive_low <= 1'b0;
                    state         <= ST_READ_ACK;
                end else begin
                    tx_bit_index <= tx_bit_index - 3'd1;
                end
            end

            ST_READ_ACK: begin
                // Master ACK=0 continues, NACK=1 ends read.
                if (sda === 1'b0) begin
                    if (rd_index == 3'd5)
                        rd_index <= 3'd0;
                    else
                        rd_index <= rd_index + 3'd1;
                    tx_bit_index <= 3'd7;
                    state        <= ST_READ;
                end else begin
                    state <= ST_IDLE;
                end
            end

            default: begin
                state <= ST_IDLE;
            end
        endcase
    end

endmodule

`default_nettype wire
