`timescale 1ns / 1ps

module cache_controller(
    input             clk,
    input             reset,
    input             read,
    input             write,
    input      [31:0] addr,
    input      [31:0] wdata,
    output reg [31:0] rdata,
    output reg        ready
);

parameter ADDR_WIDTH  = 32;
parameter DATA_WIDTH  = 32;
parameter SETS        = 128;
parameter WAYS        = 4;
parameter BLOCK_SIZE  = 64;
parameter WORD_BYTES  = 4;
parameter WORDS       = BLOCK_SIZE/WORD_BYTES; // 16 words per block
parameter INDEX_BITS  = 7;
parameter OFFSET_BITS = 6;
parameter TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;
parameter WORD_OFF    = OFFSET_BITS - 2;

localparam IDLE        = 3'd0,
           READ_HIT    = 3'd1,
           READ_MISS   = 3'd2,
           WRITE_HIT   = 3'd3,
           WRITE_MISS  = 3'd4,
           EVICT       = 3'd5;

reg [2:0] state, next_state;

reg [TAG_BITS-1:0]   tags    [0:SETS-1][0:WAYS-1];
reg                  valid   [0:SETS-1][0:WAYS-1];
reg                  dirty   [0:SETS-1][0:WAYS-1];
reg [DATA_WIDTH-1:0] data    [0:SETS-1][0:WAYS-1][0:WORDS-1];
reg [1:0]            lru     [0:SETS-1][0:WAYS-1];

wire [INDEX_BITS-1:0] set_idx    = addr[OFFSET_BITS +: INDEX_BITS];
wire [TAG_BITS-1:0]   tag_in     = addr[OFFSET_BITS+INDEX_BITS +: TAG_BITS];
wire [WORD_OFF-1:0]   word_off   = addr[OFFSET_BITS-1:2];

reg hit;
reg [1:0] hit_way;
reg free_found;
reg [1:0] free_way;

reg pending_rw;   // 0 = read, 1 = write
reg [1:0] alloc_way;

integer i, j, k;

always @(*) begin
    hit        = 1'b0;
    hit_way    = 2'b00;
    free_found = 1'b0;
    free_way   = 2'b00;
    for (i = 0; i < WAYS; i = i + 1) begin
        if (valid[set_idx][i] && tags[set_idx][i] == tag_in) begin
            hit     = 1'b1;
            hit_way = i[1:0];
        end
        if (!valid[set_idx][i] && !free_found) begin
            free_found = 1'b1;
            free_way   = i[1:0];
        end
    end
end

task update_lru(
    input [INDEX_BITS-1:0] idx,
    input [1:0]           way
);
    reg [1:0] old_val;
    integer   m;
begin
    old_val = lru[idx][way];
    for (m = 0; m < WAYS; m = m + 1) begin
        if (lru[idx][m] < old_val) begin
            lru[idx][m] = lru[idx][m] + 1;
        end
    end
    lru[idx][way] = 0;
end
endtask

function [1:0] find_lru;
    input [INDEX_BITS-1:0] idx;
    reg   [1:0]            maxv;
    integer                n;
begin
    maxv     = 2'b00;
    find_lru = 2'b00;
    for (n = 0; n < WAYS; n = n + 1) begin
        if (lru[idx][n] > maxv) begin
            maxv     = lru[idx][n];
            find_lru = n[1:0];
        end
    end
end
endfunction

always @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= IDLE;
        ready <= 1'b0;
        for (i = 0; i < SETS; i = i + 1) begin
            for (j = 0; j < WAYS; j = j + 1) begin
                valid[i][j] = 1'b0;
                dirty[i][j] = 1'b0;
                lru[i][j]   = j[1:0];
            end
        end
    end else begin
        state <= next_state;
        if (state == IDLE && (read || write)) begin
            pending_rw  <= write;
            if (hit)
                alloc_way = hit_way;
            else if (free_found)
                alloc_way = free_way;
            else
                alloc_way = find_lru(set_idx);
        end
    end
end

always @(*) begin
    next_state = state;
    ready      = 1'b0;
    rdata      = 32'b0;

    case (state)
        IDLE: begin
            if (read) begin
                if (hit) begin
                    next_state = READ_HIT;
                end else if (free_found) begin
                    next_state = READ_MISS;
                end else begin
                    next_state = EVICT;
                end
            end else if (write) begin
                if (hit) begin
                    next_state = WRITE_HIT;
                end else if (free_found) begin
                    next_state = WRITE_MISS;
                end else begin
                    next_state = EVICT;
                end
            end
        end

        READ_HIT: begin
            rdata = data[set_idx][alloc_way][word_off];
            update_lru(set_idx, alloc_way);
            ready      = 1'b1;
            next_state = IDLE;
        end

        WRITE_HIT: begin
            data[set_idx][alloc_way][word_off] = wdata;
            dirty[set_idx][alloc_way]          = 1'b1;
            update_lru(set_idx, alloc_way);
            ready      = 1'b1;
            next_state = IDLE;
        end

        READ_MISS: begin
            tags[set_idx][alloc_way]    = tag_in;
            valid[set_idx][alloc_way]   = 1'b1;
            dirty[set_idx][alloc_way]   = 1'b0;
            for (i = 0; i < WORDS; i = i + 1) begin
                data[set_idx][alloc_way][i] = 32'b0;
            end
            rdata      = 32'b0;
            update_lru(set_idx, alloc_way);
            ready      = 1'b1;
            next_state = IDLE;
        end

        WRITE_MISS: begin
            tags[set_idx][alloc_way]    = tag_in;
            valid[set_idx][alloc_way]   = 1'b1;
            dirty[set_idx][alloc_way]   = 1'b1;
            for (i = 0; i < WORDS; i = i + 1) begin
                data[set_idx][alloc_way][i] = 32'b0;
            end
            data[set_idx][alloc_way][word_off] = wdata;
            update_lru(set_idx, alloc_way);
            ready      = 1'b1;
            next_state = IDLE;
        end

        EVICT: begin
            valid[set_idx][alloc_way] = 1'b0;
            dirty[set_idx][alloc_way] = 1'b0;
            if (pending_rw)
                next_state = WRITE_MISS;
            else
                next_state = READ_MISS;
        end

        default: begin
            next_state = IDLE;
        end
    endcase
end

endmodule