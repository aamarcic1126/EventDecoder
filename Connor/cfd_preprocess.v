// ===========================================================================
// File: cfd_preprocess.v
//
// Baseline subtraction preprocessing for 4-channel CFD.
//
// Takes 12-bit unsigned ADC samples on four channels, computes the
// sum of the first 8 samples per event as the baseline (which equals
// the average in UQ12.3 fixed-point format), and subtracts it from
// every sample including the first 8.
//
// Architecture: an 8-deep shift register per channel acts as a delay
// line.  During the first 8 input samples, the baseline sum is
// accumulated while the shift register fills.  From sample 9 onward,
// the shift register outputs the sample from 8 cycles ago, corrected
// by baseline subtraction.  After in_valid drops, the remaining 8
// samples are flushed from the shift register.
//
// Overlap support: if a new event begins (in_valid rises) before the
// flush completes, the flush continues using a saved copy of the old
// baseline while the new event's samples enter the shift register and
// naturally push the old samples out.  The new event's accumulation
// phase (8 cycles) provides exactly enough time to drain the old data.
// Minimum gap between events: 1 idle cycle.
//
// Fixed-point format:
//   Baseline = sum of 8 samples (15-bit unsigned = UQ12.3).
//   Each sample padded: {sample[11:0], 3'b000} = UQ12.3 (15-bit).
//   Output = padded_sample - baseline = signed 16-bit (SQ12.3).
//
// Latency: 8 sample clocks at the start of each event.
// Output samples per event: same as input samples.
// If the input event has fewer than 8 samples, no output is produced.
// ===========================================================================

module cfd_preprocess (
    input  wire        clk,
    input  wire        rst_n,

    // Raw ADC inputs (active when in_valid is high)
    input  wire        in_valid,
    input  wire [11:0] x1_raw,
    input  wire [11:0] x2_raw,
    input  wire [11:0] y1_raw,
    input  wire [11:0] y2_raw,

    // Event tag (latched on rising edge of in_valid)
    input  wire [63:0] tag_in,
    output wire [63:0] tag_out,

    // Corrected outputs (active when out_valid is high)
    output wire        out_valid,
    output wire signed [15:0] x1_out,
    output wire signed [15:0] x2_out,
    output wire signed [15:0] y1_out,
    output wire signed [15:0] y2_out
);

    // -----------------------------------------------------------------
    // Delayed in_valid for edge detection
    // -----------------------------------------------------------------
    reg in_valid_d;

    always @(posedge clk) begin
        if (!rst_n)
            in_valid_d <= 1'b0;
        else
            in_valid_d <= in_valid;
    end

    wire event_start = in_valid & ~in_valid_d;

    // -----------------------------------------------------------------
    // Tag register ? latched on rising edge of in_valid
    //
    // tag_out is muxed: during flush_active, the saved flush_tag is
    // output so that tag_out always corresponds to the data being
    // output, even when a new event overlaps the flush.
    // On start_flush_comb, flush_tag hasn't been saved yet, but
    // tag_reg still holds the current event's tag, so the mux
    // selects tag_reg (correct).
    // -----------------------------------------------------------------
    reg [63:0] tag_reg;

    always @(posedge clk) begin
        if (!rst_n)
            tag_reg <= 64'd0;
        else if (event_start)
            tag_reg <= tag_in;
    end

    // -----------------------------------------------------------------
    // Baseline done flag (forward declaration for use below)
    // -----------------------------------------------------------------
    reg baseline_done;

    // -----------------------------------------------------------------
    // Flush control
    //
    // When in_valid drops after streaming, flush_active goes high for
    // 8 cycles to drain the shift register.  If a new event starts
    // before flush completes, flush continues ? the new event's
    // incoming samples push old samples through the shift register.
    //
    // start_flush_comb is a combinational bridge that fires for the
    // single cycle between in_valid dropping and flush_active going
    // high (non-blocking assignment delay).
    //
    // The saved baseline (flush_bl_*) is used for all flush outputs
    // so that a new event's accumulation doesn't corrupt the old
    // event's baseline subtraction.
    // -----------------------------------------------------------------
    reg        flush_active;
    reg  [2:0] flush_cnt;       // counts 6 down to 0 (7 cycles after bridge)
    reg [14:0] flush_bl_x1, flush_bl_x2, flush_bl_y1, flush_bl_y2;
    reg [63:0] flush_tag;
    reg [14:0] bl_x1, bl_x2, bl_y1, bl_y2;

    wire start_flush_comb = baseline_done & in_valid_d & ~in_valid & ~flush_active;

    always @(posedge clk) begin
        if (!rst_n) begin
            flush_active <= 1'b0;
            flush_cnt    <= 3'd0;
            flush_bl_x1  <= 15'd0;
            flush_bl_x2  <= 15'd0;
            flush_bl_y1  <= 15'd0;
            flush_bl_y2  <= 15'd0;
            flush_tag    <= 64'd0;
        end else if (start_flush_comb) begin
            flush_active <= 1'b1;
            flush_cnt    <= 3'd6;
            flush_bl_x1  <= bl_x1;
            flush_bl_x2  <= bl_x2;
            flush_bl_y1  <= bl_y1;
            flush_bl_y2  <= bl_y2;
            flush_tag    <= tag_reg;
        end else if (flush_active) begin
            if (flush_cnt == 3'd0)
                flush_active <= 1'b0;
            else
                flush_cnt <= flush_cnt - 3'd1;
        end
    end

    // -----------------------------------------------------------------
    // Master shift enable
    //
    // Active whenever valid input data is present, flushing, or on
    // the bridge cycle.  During flush overlap with a new event,
    // both in_valid and flush_active are high ? shift_en is still 1.
    // -----------------------------------------------------------------
    wire do_shift = in_valid | flush_active | start_flush_comb;

    // -----------------------------------------------------------------
    // Shift registers (8-deep delay line per channel)
    //
    // New samples enter at index 0; the oldest exits at index 7.
    // During flush without a new event, zeros enter.
    // During flush overlapping with a new event, the new event's
    // samples enter ? pushing old samples out naturally.
    // -----------------------------------------------------------------
    reg [11:0] sr_x1 [0:7];
    reg [11:0] sr_x2 [0:7];
    reg [11:0] sr_y1 [0:7];
    reg [11:0] sr_y2 [0:7];

    wire [11:0] sr_in_x1 = in_valid ? x1_raw : 12'd0;
    wire [11:0] sr_in_x2 = in_valid ? x2_raw : 12'd0;
    wire [11:0] sr_in_y1 = in_valid ? y1_raw : 12'd0;
    wire [11:0] sr_in_y2 = in_valid ? y2_raw : 12'd0;

    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 8; i = i + 1) begin
                sr_x1[i] <= 12'd0;
                sr_x2[i] <= 12'd0;
                sr_y1[i] <= 12'd0;
                sr_y2[i] <= 12'd0;
            end
        end else if (do_shift) begin
            sr_x1[0] <= sr_in_x1;
            sr_x2[0] <= sr_in_x2;
            sr_y1[0] <= sr_in_y1;
            sr_y2[0] <= sr_in_y2;
            for (i = 1; i < 8; i = i + 1) begin
                sr_x1[i] <= sr_x1[i-1];
                sr_x2[i] <= sr_x2[i-1];
                sr_y1[i] <= sr_y1[i-1];
                sr_y2[i] <= sr_y2[i-1];
            end
        end
    end

    // -----------------------------------------------------------------
    // Baseline accumulators
    //
    // Sum of first 8 samples per channel (15-bit unsigned).
    // Max value: 8 x 4095 = 32760, fits in 15 bits.
    // This sum IS the average in UQ12.3 format (no division needed).
    //
    // On event_start: initialize with the first sample.
    // On subsequent accumulate cycles: add the new sample.
    // -----------------------------------------------------------------
    reg [3:0]  accum_cnt;

    wire accumulating = in_valid & ~baseline_done;

    always @(posedge clk) begin
        if (!rst_n) begin
            bl_x1         <= 15'd0;
            bl_x2         <= 15'd0;
            bl_y1         <= 15'd0;
            bl_y2         <= 15'd0;
            accum_cnt     <= 4'd0;
            baseline_done <= 1'b0;
        end else if (event_start) begin
            bl_x1         <= {3'd0, x1_raw};
            bl_x2         <= {3'd0, x2_raw};
            bl_y1         <= {3'd0, y1_raw};
            bl_y2         <= {3'd0, y2_raw};
            accum_cnt     <= 4'd1;
            baseline_done <= 1'b0;
        end else if (accumulating) begin
            bl_x1 <= bl_x1 + {3'd0, x1_raw};
            bl_x2 <= bl_x2 + {3'd0, x2_raw};
            bl_y1 <= bl_y1 + {3'd0, y1_raw};
            bl_y2 <= bl_y2 + {3'd0, y2_raw};
            accum_cnt <= accum_cnt + 4'd1;
            if (accum_cnt == 4'd7)
                baseline_done <= 1'b1;
        end
    end

    // -----------------------------------------------------------------
    // Output baseline mux
    //
    // During flush (flush_active or start_flush_comb), use the saved
    // baseline from the previous event.  During normal streaming, use
    // the current baseline.
    // -----------------------------------------------------------------
    wire flush_outputting = flush_active | start_flush_comb;

    // Baseline mux: use saved flush baseline only during flush_active
    // (NOT on start_flush_comb, because flush_bl hasn't been saved yet ?
    // the non-blocking assignment takes effect next posedge).
    // On start_flush_comb, flush_active is 0 so bl_* is selected,
    // which is correct since it still holds the current event's baseline.
    wire [14:0] out_bl_x1 = flush_active ? flush_bl_x1 : bl_x1;
    wire [14:0] out_bl_x2 = flush_active ? flush_bl_x2 : bl_x2;
    wire [14:0] out_bl_y1 = flush_active ? flush_bl_y1 : bl_y1;
    wire [14:0] out_bl_y2 = flush_active ? flush_bl_y2 : bl_y2;

    // Tag mux: same logic ? flush_active selects saved tag,
    // otherwise tag_reg (which is still correct on start_flush_comb).
    assign tag_out = flush_active ? flush_tag : tag_reg;

    // -----------------------------------------------------------------
    // Output computation (combinational)
    //
    // Pad the delayed sample to UQ12.3: {sample[11:0], 3'b000}
    // Subtract the selected baseline: signed 16-bit result (SQ12.3)
    // -----------------------------------------------------------------
    wire [14:0] padded_x1 = {sr_x1[7], 3'b000};
    wire [14:0] padded_x2 = {sr_x2[7], 3'b000};
    wire [14:0] padded_y1 = {sr_y1[7], 3'b000};
    wire [14:0] padded_y2 = {sr_y2[7], 3'b000};

    assign x1_out = $signed({1'b0, padded_x1}) - $signed({1'b0, out_bl_x1});
    assign x2_out = $signed({1'b0, padded_x2}) - $signed({1'b0, out_bl_x2});
    assign y1_out = $signed({1'b0, padded_y1}) - $signed({1'b0, out_bl_y1});
    assign y2_out = $signed({1'b0, padded_y2}) - $signed({1'b0, out_bl_y2});

    // -----------------------------------------------------------------
    // Output valid (combinational)
    //
    // Two sources of valid output:
    //   1. Normal streaming: baseline_done & in_valid & ~event_start
    //      (event_start suppressed because baseline_done is stale
    //       from the previous event on that cycle)
    //   2. Flush: flush_active | start_flush_comb
    //
    // During flush overlap with a new event's accumulation, only
    // flush_outputting drives out_valid ? the stream condition is
    // false because baseline_done was cleared by event_start.
    // -----------------------------------------------------------------
    wire stream_out = baseline_done & in_valid & ~event_start;

    assign out_valid = stream_out | flush_outputting;

endmodule