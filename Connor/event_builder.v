///////////////////////////////////////////////////////////////////////////////////////////////////
// Company: SwRI/VT
//
// File: eventbuilder.v
// Version: 1
// Date: 02/02/2026
// Description:
//
//  This module is part of the event decoder pipeline and its purpose is to construct a final data
//  value to send to the ethernet packetizer. It receives inputs from both the charge integrator
//  and position calculator. It uses the time stamps from each of these two inputs to line up
//  event data based on time.
//
// Targeted device: <Family::ProASIC3E> <Die::A3PE1500> <Package::208 PQFP>
// Author: VT MDE S26-23
// Contributors: Anthony Marcic
//
/////////////////////////////////////////////////////////////////////////////////////////////////// 

`timescale 1ns/1ps

module eventbuilder #(
    parameter TS_WIDTH      = 52,
    parameter POS_WIDTH     = 32, // Assuming {x[15:0], y[15:0]}
    parameter MAG_WIDTH     = 26,
    parameter DEPTH_LOG2    = 3
)(
    input wire                  clk,
    input wire                  rst_n,

    // Position Calculator Module Signals
    input wire                  pos_valid,
    input wire [TS_WIDTH-1:0]   pos_time,
    input wire [POS_WIDTH-1:0]  pos_data,

    // Charge Integrator Signals
    input wire                  chg_valid,
    input wire [TS_WIDTH-1:0]   chg_time,
    input wire [MAG_WIDTH-1:0]  chg_mag,
    input wire                  chg_overflow,

    // Ethernet Packetizer Signals
    output reg                  out_valid,
    output reg [TS_WIDTH-1:0]   out_time,
    output reg [POS_WIDTH-1:0]  out_pos,
    output reg [MAG_WIDTH-1:0]  out_mag,

    // Status Signals
    output reg                  drop_pos,
    output reg                  drop_chg,
    output reg                  match_overflow
);

    localparam integer DEPTH = (1 << DEPTH_LOG2);

    // FIFO Storage
    reg [TS_WIDTH-1:0]          pos_mem_time [0:DEPTH-1];
    reg [POS_WIDTH-1:0]         pos_mem_data [0:DEPTH-1];

    reg [TS_WIDTH-1:0] chg_mem_time [0:DEPTH-1];
    reg [MAG_WIDTH-1:0] chg_mem_mag  [0:DEPTH-1];
    reg                 chg_mem_of   [0:DEPTH-1];

    // Pointers and counts
    reg [DEPTH_LOG2:0] pos_wptr, pos_rptr, pos_count;
    reg [DEPTH_LOG2:0] chg_wptr, chg_rptr, chg_count;

    wire pos_full  = (pos_count == DEPTH);
    wire pos_empty = (pos_count == 0);
    wire chg_full  = (chg_count == DEPTH);
    wire chg_empty = (chg_count == 0);

    // FIFO head
    wire [TS_WIDTH-1:0]  pos_head_time = pos_mem_time[pos_rptr[DEPTH_LOG2-1:0]];
    wire [POS_WIDTH-1:0] pos_head_data = pos_mem_data[pos_rptr[DEPTH_LOG2-1:0]];

    wire [TS_WIDTH-1:0]  chg_head_time = chg_mem_time[chg_rptr[DEPTH_LOG2-1:0]];
    wire [MAG_WIDTH-1:0] chg_head_mag  = chg_mem_mag [chg_rptr[DEPTH_LOG2-1:0]];
    wire                 chg_head_of   = chg_mem_of  [chg_rptr[DEPTH_LOG2-1:0]];

    // Pop signals from matcher
    reg pop_pos;
    reg pop_chg;

    // Push into FIFOs
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_wptr  <= { (DEPTH_LOG2+1){1'b0} };
            pos_count <= { (DEPTH_LOG2+1){1'b0} };
            chg_wptr  <= { (DEPTH_LOG2+1){1'b0} };
            chg_count <= { (DEPTH_LOG2+1){1'b0} };
        end else begin
            // Position push
            if (pos_valid && !pos_full) begin
                pos_mem_time[pos_wptr[DEPTH_LOG2-1:0]] <= pos_time;
                pos_mem_data[pos_wptr[DEPTH_LOG2-1:0]] <= pos_data;
                pos_wptr  <= pos_wptr + 1'b1;
                pos_count <= pos_count + 1'b1;
            end
            // Charge push
            if (chg_valid && !chg_full) begin
                chg_mem_time[chg_wptr[DEPTH_LOG2-1:0]] <= chg_time;
                chg_mem_mag [chg_wptr[DEPTH_LOG2-1:0]] <= chg_mag;
                chg_mem_of  [chg_wptr[DEPTH_LOG2-1:0]] <= chg_overflow;
                chg_wptr  <= chg_wptr + 1'b1;
                chg_count <= chg_count + 1'b1;
            end

            // Note: If full, new inputs are dropped
            // Potentially add explicit overflow flags/counters here
        end
    end

    // Pop from FIFOs (advance read pointers and decrement counts)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_rptr <= { (DEPTH_LOG2+1){1'b0} };
            chg_rptr <= { (DEPTH_LOG2+1){1'b0} };
        end else begin
            if (pop_pos && !pos_empty) begin
                pos_rptr  <= pos_rptr + 1'b1;
                pos_count <= pos_count - 1'b1;
            end
            if (pop_chg && !chg_empty) begin
                chg_rptr  <= chg_rptr + 1'b1;
                chg_count <= chg_count - 1'b1;
            end
        end
    end

    // Matcher: Determine what's being popped off the FIFO
    always @(*) begin
        pop_pos = 1'b0;
        pop_chg = 1'b0;

        if (!pos_empty && !chg_empty) begin
            if (pos_head_time == chg_head_time) begin
                pop_pos = 1'b1;
                pop_chg = 1'b1;
            end else if (pos_head_time < chg_head_time) begin
                pop_pos = 1'b1; // drop older position
            end else begin
                pop_chg = 1'b1; // drop older charge
            end
        end
    end

    // Output registers (1-cycle pulses)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid      <= 1'b0;
            out_time       <= {TS_WIDTH{1'b0}};
            out_pos        <= {POS_WIDTH{1'b0}};
            out_mag        <= {MAG_WIDTH{1'b0}};
            drop_pos       <= 1'b0;
            drop_chg       <= 1'b0;
            match_overflow <= 1'b0;
        end else begin
            // defaults each cycle
            out_valid      <= 1'b0;
            drop_pos       <= 1'b0;
            drop_chg       <= 1'b0;
            match_overflow <= 1'b0;

            if (!pos_empty && !chg_empty) begin
                if (pos_head_time == chg_head_time) begin
                    out_valid      <= 1'b1;
                    out_time       <= pos_head_time;
                    out_pos        <= pos_head_data;
                    out_mag        <= chg_head_mag;
                    match_overflow <= chg_head_of;
                end else if (pos_head_time < chg_head_time) begin
                    drop_pos <= 1'b1;
                end else begin
                    drop_chg <= 1'b1;
                end
            end
        end
    end

endmodule



