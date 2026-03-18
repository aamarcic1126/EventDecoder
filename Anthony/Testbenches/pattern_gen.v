///////////////////////////////////////////////////////////////////////////////////////////////////
// Company: <Name>
//
// File: pattern_gen.v
// File history:
//      <Revision number>: <Date>: <Comments>
//      <Revision number>: <Date>: <Comments>
//      <Revision number>: <Date>: <Comments>
//
// Description: 
//
// <Description here>
//
// Targeted device: <Family::ProASIC3E> <Die::A3PE1500> <Package::208 PQFP>
// Author: <Name>
//
/////////////////////////////////////////////////////////////////////////////////////////////////// 

//`timescale <time_units> / <precision>

module pattern_gen #(
    parameter integer       DIV        = 2000,        // emit every DIV clocks
    parameter signed [15:0] POINT_STEP = 16'sd100,     // approx spacing between points
    parameter integer       LEN_INC    = 30,           // segment length growth per ring
    parameter [15:0]        BOUND_ABS  = 16'd16384     // stop when abs(next x/y) >= this
)(
    input  wire clk,
    input  wire rst_n,
    input  wire enable,

    output reg  signed [15:0] x,
    output reg  signed [15:0] y,
    output reg                valid,

    output reg  [31:0] point_count_out,
    output reg  [19:0] rot20_out
);

    // -------------------------
    // Utility: clog2 for counter width (Verilog)
    // -------------------------
    function integer clog2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            clog2 = 0;
            while (v > 0) begin
                v = v >> 1;
                clog2 = clog2 + 1;
            end
        end
    endfunction

    localparam integer DIV_W = (clog2(DIV) < 1) ? 1 : clog2(DIV);

    // -------------------------
    // Utility: absolute value for signed 18-bit
    // -------------------------
    function [17:0] abs18;
        input signed [17:0] v;
        begin
            if (v[17]) abs18 = -v;
            else       abs18 =  v;
        end
    endfunction

    // -------------------------
    // enable rising-edge detect
    // -------------------------
    reg enable_d;
    wire enable_rise;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) enable_d <= 1'b0;
        else       enable_d <= enable;
    end

    assign enable_rise = enable & ~enable_d;

    // -------------------------
    // FSM states (no SystemVerilog typedef/enum)
    // -------------------------
    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_RUN  = 2'd1;
    localparam [1:0] S_DONE = 2'd2;

    reg [1:0] st;

    // -------------------------
    // Emit divider
    // -------------------------
    reg  [DIV_W-1:0] div_cnt;
    wire emit_pulse;

    assign emit_pulse = (enable && (st == S_RUN) && (div_cnt == (DIV-1)));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= {DIV_W{1'b0}};
        end else if (!enable) begin
            div_cnt <= {DIV_W{1'b0}};
        end else if (enable_rise) begin
            div_cnt <= {DIV_W{1'b0}};
        end else if (st != S_RUN) begin
            div_cnt <= {DIV_W{1'b0}};
        end else if (emit_pulse) begin
            div_cnt <= {DIV_W{1'b0}};
        end else begin
            div_cnt <= div_cnt + {{(DIV_W-1){1'b0}},1'b1};
        end
    end

    // -------------------------
    // Spiral internal state
    // -------------------------
    // Internal position (NOT the outputs)
    reg signed [15:0] x_state, y_state;

    // dir: 0=R,1=U,2=L,3=D
    reg [1:0]  dir;

    // segment length and countdown (in number of emitted points)
    reg [15:0] step_len;
    reg [15:0] step_left;

    // toggles each segment; after 2 segments, grow step_len
    reg seg_in_pair;

    // deterministic dither for "not exact" step: alternate +/-2 around POINT_STEP
    reg step_dither;
    wire signed [15:0] step_amt;

    assign step_amt = step_dither ? (POINT_STEP - 16'sd2) : (POINT_STEP + 16'sd2);

    // Next coordinate candidate in wider math
    reg signed [17:0] x_next_w, y_next_w;
    wire [17:0] x_abs_next, y_abs_next;
    wire hit_bound_next;
    wire [17:0] bound_ext;

    assign bound_ext = {2'b00, BOUND_ABS};

    always @* begin
        x_next_w = { {2{x_state[15]}}, x_state };
        y_next_w = { {2{y_state[15]}}, y_state };

        case (dir)
            2'd0: x_next_w = { {2{x_state[15]}}, x_state } + { {2{step_amt[15]}}, step_amt }; // R
            2'd1: y_next_w = { {2{y_state[15]}}, y_state } + { {2{step_amt[15]}}, step_amt }; // U
            2'd2: x_next_w = { {2{x_state[15]}}, x_state } - { {2{step_amt[15]}}, step_amt }; // L
            2'd3: y_next_w = { {2{y_state[15]}}, y_state } - { {2{step_amt[15]}}, step_amt }; // D
            default: begin end
        endcase
    end

    assign x_abs_next = abs18(x_next_w);
    assign y_abs_next = abs18(y_next_w);

    assign hit_bound_next = (x_abs_next >= bound_ext) || (y_abs_next >= bound_ext);

    // -------------------------
    // Extra info internal state
    // -------------------------
    reg [31:0] point_count; // internal counter (first output is 1)
    reg [19:0] rot20_reg;   // internal rotating one-hot

    // -------------------------
    // Main sequential logic
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE;

            // outputs
            x <= 16'sd0;
            y <= 16'sd0;
            valid <= 1'b0;
            point_count_out <= 32'd0;
            rot20_out <= 20'd0;

            // internal state
            x_state <= 16'sd0;
            y_state <= 16'sd0;

            dir <= 2'd0;
            step_len <= 16'd1;
            step_left <= 16'd1;
            seg_in_pair <= 1'b0;
            step_dither <= 1'b0;

            point_count <= 32'd0;
            rot20_reg <= 20'd1;

        end else begin
            // Default: when valid is low, ALL outputs are low
            valid <= 1'b0;
            x <= 16'sd0;
            y <= 16'sd0;
            point_count_out <= 32'd0;
            rot20_out <= 20'd0;

            // If enable drops, go idle/testing and reset internal pattern state
            if (!enable) begin
                st <= S_IDLE;

                x_state <= 16'sd0;
                y_state <= 16'sd0;

                dir <= 2'd0;
                step_len <= 16'd1;
                step_left <= 16'd1;
                seg_in_pair <= 1'b0;
                step_dither <= 1'b0;

                point_count <= 32'd0;
                rot20_reg <= 20'd1;

            end else if (enable_rise) begin
                // Restart from beginning on enable rising edge
                st <= S_RUN;

                x_state <= 16'sd0;
                y_state <= 16'sd0;

                dir <= 2'd0;
                step_len <= 16'd1;
                step_left <= 16'd1;
                seg_in_pair <= 1'b0;
                step_dither <= 1'b0;

                point_count <= 32'd0;
                rot20_reg <= 20'd1;

            end else begin
                case (st)
                    S_IDLE: begin
                        // stay quiet; outputs already forced low
                    end

                    S_RUN: begin
                        if (emit_pulse) begin
                            if (hit_bound_next) begin
                                // Stop and return to "testing" behavior (quiet)
                                st <= S_DONE;

                                // Reset internal state so next enable_rise starts cleanly
                                x_state <= 16'sd0;
                                y_state <= 16'sd0;

                                dir <= 2'd0;
                                step_len <= 16'd1;
                                step_left <= 16'd1;
                                seg_in_pair <= 1'b0;
                                step_dither <= 1'b0;

                                point_count <= 32'd0;
                                rot20_reg <= 20'd1;

                            end else begin
                                // Commit internal position
                                x_state <= x_next_w[15:0];
                                y_state <= y_next_w[15:0];

                                // Assert outputs ONLY on this cycle
                                valid <= 1'b1;
                                x <= x_next_w[15:0];
                                y <= y_next_w[15:0];

                                point_count_out <= point_count + 32'd1;
                                rot20_out <= rot20_reg;

                                // Advance metadata state
                                point_count <= point_count + 32'd1;
                                rot20_reg <= {rot20_reg[18:0], rot20_reg[19]}; // rotate-left

                                // dither the step
                                step_dither <= ~step_dither;

                                // segment bookkeeping
                                if (step_left == 16'd1) begin
                                    dir <= dir + 2'd1;

                                    if (seg_in_pair) begin
                                        // grow length after two segments
                                        step_len <= step_len + LEN_INC[15:0];
                                        step_left <= step_len + LEN_INC[15:0];
                                        seg_in_pair <= 1'b0;
                                    end else begin
                                        step_left <= step_len;
                                        seg_in_pair <= 1'b1;
                                    end
                                end else begin
                                    step_left <= step_left - 16'd1;
                                end
                            end
                        end
                    end

                    S_DONE: begin
                        // stay quiet until enable goes low (then enable_rise restarts later)
                    end

                    default: begin
                        st <= S_IDLE;
                    end
                endcase
            end
        end
    end

endmodule


