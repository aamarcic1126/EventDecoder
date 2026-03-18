// =======================
// File: CDF_top.v
// =======================

// Shared delay line RAM (simple circular buffer) with write-first bypass
module cfd_delay_line_ram #(
    parameter integer DATA_W        = 16,
    parameter integer DELAY_RAM_AW  = 7,
    parameter integer DEPTH         = (1 << DELAY_RAM_AW)
)(
    input  wire                     clk,
    input  wire                     we,
    input  wire [DELAY_RAM_AW-1:0]  wr_addr,
    input  wire [DELAY_RAM_AW-1:0]  rd_addr,
    input  wire [DATA_W-1:0]        din,
    output reg  [DATA_W-1:0]        dout
);

    (* syn_ramstyle = "rw_check" *) reg [DATA_W-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we)
            mem[wr_addr] <= din;

        // Write-first bypass: if reading and writing the same address
        // on the same cycle, forward the new write data rather than
        // reading stale memory contents.
        if (we && (rd_addr == wr_addr))
            dout <= din;
        else
            dout <= mem[rd_addr];
    end

endmodule


// ============================================================================
// Lane frontend
//
// Signal path:
//   sample_in ??? delay RAM (1 clk) ??? 3 pipeline regs ??? delayed_d3 ???
//                                                                         ??? bip = del - att
//   sample_in ??? signed multiplier IP (3 clk) ??? 1 reg ??? att_samp_q ?
//
// Both paths have 4 clock cycles of latency so that the delayed and
// attenuated versions of the *same* original sample arrive at the
// subtraction simultaneously.  The extra pipeline stage (vs. a minimum
// of 3) breaks the critical timing path between the multiplier's
// combinational output carry chain and the 17-bit subtractor.
//
// Pipeline note: the first ~6 bipolar samples after event_start are
// computed from stale pipeline data.  The pipeline fill blanking counter
// prevents any zero crossing from being accepted during this period.
// ============================================================================
module cfd_lane_frontend #(
    parameter integer DELAY_RAM_AW      = 7,
    parameter integer MAX_EVENT_SAMPLES = 1024,
    parameter integer IDX_W             = 10,
    parameter integer FRAC_BITS         = 10
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         sample_valid,
    input  wire                         event_start,
    input  wire signed [15:0]           sample_in,

    // Runtime configuration (latched at event_start)
    input  wire [13:0]                  att_q0_13,
    input  wire [DELAY_RAM_AW-1:0]      delay_val,
    input  wire signed [15:0]           threshold,
    input  wire [7:0]                   zc_neg_samples,  // 0-255

    // Per-event latched outputs (stable from first ZC until next event start)
    output reg                          hit_found,
    output reg [15:0]                   ca_out,      // always >= 0 at ZC
    output reg signed [16:0]            cb_out,
    output reg [IDX_W-1:0]             idx_cb_out,

    // Debug / optional
    output reg  signed [16:0]           bip,
    output wire                         zc
);

    // -----------------------------------------------------------------
    // Config registration
    //
    // Latch all runtime configuration on event_start so values cannot
    // change mid-event.  Delay is floored from 0 to 1 to avoid a
    // simultaneous read/write on the delay RAM.
    // -----------------------------------------------------------------
    reg [DELAY_RAM_AW-1:0]  delay_reg;
    reg [13:0]              att_reg;
    reg signed [15:0]       thresh_reg;
    reg [7:0]               zc_neg_reg;

    always @(posedge clk) begin
        if (!rst_n) begin
            delay_reg  <= {{(DELAY_RAM_AW-1){1'b0}}, 1'b1};  // default 1
            att_reg    <= 14'd0;
            thresh_reg <= 16'sd0;
            zc_neg_reg <= 8'd2;
        end else if (event_start) begin
            delay_reg <= (delay_val == {DELAY_RAM_AW{1'b0}})
                         ? {{(DELAY_RAM_AW-1){1'b0}}, 1'b1}
                         : delay_val;
            att_reg    <= att_q0_13;
            thresh_reg <= threshold;
            zc_neg_reg <= zc_neg_samples;
        end
    end

    // -----------------------------------------------------------------
    // Delay line (1 clock read latency)
    // -----------------------------------------------------------------
    reg  [DELAY_RAM_AW-1:0] wr_ptr;
    wire [DELAY_RAM_AW-1:0] rd_ptr;
    wire [15:0] delayed_u;

    assign rd_ptr = wr_ptr - delay_reg;

    cfd_delay_line_ram #(
        .DATA_W     (16),
        .DELAY_RAM_AW(DELAY_RAM_AW),
        .DEPTH      (1 << DELAY_RAM_AW)
    ) u_delay (
        .clk    (clk),
        .we     (sample_valid),
        .wr_addr(wr_ptr),
        .rd_addr(rd_ptr),
        .din    (sample_in),
        .dout   (delayed_u)
    );

    // Running sample index ? resets at event start
    reg [IDX_W-1:0] idx_curr;

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr   <= {DELAY_RAM_AW{1'b0}};
            idx_curr <= {IDX_W{1'b0}};
        end else if (sample_valid) begin
            wr_ptr <= wr_ptr + {{(DELAY_RAM_AW-1){1'b0}}, 1'b1};

            if (event_start)
                idx_curr <= {IDX_W{1'b0}};
            else if (idx_curr == (MAX_EVENT_SAMPLES - 1))
                idx_curr <= {IDX_W{1'b0}};
            else
                idx_curr <= idx_curr + {{(IDX_W-1){1'b0}}, 1'b1};
        end
    end

    // -----------------------------------------------------------------
    // Three pipeline registers on delayed path  (1 RAM + 3 regs = 4 clk)
    //
    // An extra stage (vs. the minimum 2) is added to match the
    // pipeline register inserted after the multiplier output to
    // break the critical timing path.
    // -----------------------------------------------------------------
    reg [15:0] delayed_d1;
    reg [15:0] delayed_d2;
    reg [15:0] delayed_d3;

    always @(posedge clk) begin
        if (!rst_n) begin
            delayed_d1 <= 16'd0;
            delayed_d2 <= 16'd0;
            delayed_d3 <= 16'd0;
        end else if (sample_valid) begin
            delayed_d1 <= delayed_u;
            delayed_d2 <= delayed_d1;
            delayed_d3 <= delayed_d2;
        end
    end

    // -----------------------------------------------------------------
    // Multiply: sample_in x att_q0_13  (signed, 3-stage pipelined IP)
    //
    // The multiplier IP has 3 internal pipeline stages.  An additional
    // output register (att_samp_q) is inserted to break the critical
    // path between the multiplier's combinational output carry chain
    // and the 17-bit bipolar subtractor.
    //
    // Total attenuated-path latency: 3 (IP) + 1 (att_samp_q) = 4 clk,
    // matching the delayed path (1 RAM + 3 regs = 4 clk).
    // -----------------------------------------------------------------
    wire [15:0] mult_dataA = sample_in[15:0];
    wire [13:0] mult_dataB = att_reg[13:0];
    wire [29:0] prod_w;

    mult_14_16_3 mult_att (
        .Clock(clk),
        .DataA(mult_dataA),
        .DataB(mult_dataB),
        .Mult (prod_w)
    );

    // Attenuated sample: signed product >> 13   (Q0.13 scaling)
    // prod_w is a SIGNED 30-bit result; bit 29 is the sign bit.
    // Register to break timing path.
    reg signed [16:0] att_samp_q;

    always @(posedge clk) begin
        if (!rst_n)
            att_samp_q <= 17'sd0;
        else if (sample_valid)
            att_samp_q <= $signed(prod_w[29:13]);
    end

    // Sign-extend delayed sample to 17 bits
    wire signed [16:0] delayed_signed = $signed({delayed_d3[15], delayed_d3});

    // -----------------------------------------------------------------
    // Index pipeline  (3 stages, aligned with bipolar output)
    //
    // The data path from sample_in to the bip register has 5 register
    // stages: 3 (multiplier IP) + 1 (att_samp_q) + 1 (bip reg).
    // The index path must match: idx_curr (1 reg) + pipe0/1/2 (3 regs)
    // + idx_bip (1 reg) = 5 total.  idx_bip then aligns with bip.
    // -----------------------------------------------------------------
    reg [IDX_W-1:0] idx_pipe0, idx_pipe1, idx_pipe2;

    always @(posedge clk) begin
        if (!rst_n) begin
            idx_pipe0 <= {IDX_W{1'b0}};
            idx_pipe1 <= {IDX_W{1'b0}};
            idx_pipe2 <= {IDX_W{1'b0}};
        end else if (sample_valid) begin
            idx_pipe0 <= idx_curr;
            idx_pipe1 <= idx_pipe0;
            idx_pipe2 <= idx_pipe1;
        end
    end

    // -----------------------------------------------------------------
    // Bipolar signal + one-sample history
    // -----------------------------------------------------------------
    reg signed [16:0] bip_d;
    reg [IDX_W-1:0]   idx_bip;
    reg [IDX_W-1:0]   idx_bip_d;

    // Consecutive-negative counter (8 bits, saturating, max 255)
    reg [7:0] neg_cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            bip       <= 17'sd0;
            bip_d     <= 17'sd0;
            idx_bip   <= {IDX_W{1'b0}};
            idx_bip_d <= {IDX_W{1'b0}};
            neg_cnt   <= 8'd0;
        end else if (sample_valid) begin
            // bipolar = delayed[n-D] - fraction * current[n]
            bip   <= delayed_signed - att_samp_q;
            bip_d <= bip;

            idx_bip   <= idx_pipe2;
            idx_bip_d <= idx_bip;

            // Track consecutive negative bipolar samples.
            // On event start, reset the counter.
            if (event_start) begin
                neg_cnt <= 8'd0;
            end else if (zc_neg_reg == 8'd0) begin
                // No consecutive-negative requirement
                neg_cnt <= 8'd1;
            end else if (bip[16]) begin
                // bip is negative ? increment (saturating at 255)
                if (neg_cnt < 8'd255)
                    neg_cnt <= neg_cnt + 8'd1;
            end else begin
                neg_cnt <= 8'd0;
            end
        end
    end

    // -----------------------------------------------------------------
    // Zero-crossing detect
    //   bip_d < 0  AND  bip >= 0  AND  bip_d != 0
    //   plus N-consecutive-negative qualification
    // -----------------------------------------------------------------
    wire zc_raw = (bip_d[16] && !bip[16] && (bip_d != 17'sd0));
    assign zc   = zc_raw && ((zc_neg_reg == 8'd0) ? 1'b1
                           : (neg_cnt >= zc_neg_reg));

    // -----------------------------------------------------------------
    // Threshold arming gate
    //
    // The ZC detector is deaf until the raw input exceeds the
    // registered threshold.  The comparison is on undelayed sample_in,
    // which sees the pulse peak several samples before the bipolar
    // zero crossing, so the gate opens in time.  Once armed, it stays
    // armed until the next event_start.
    //
    // If threshold == 0, the gate is effectively disabled (any
    // positive sample arms it immediately).
    // -----------------------------------------------------------------
    reg thresh_armed;

    always @(posedge clk) begin
        if (!rst_n) begin
            thresh_armed <= 1'b0;
        end else begin
            if (event_start)
                thresh_armed <= 1'b0;
            else if (sample_valid && !thresh_armed && (sample_in > thresh_reg))
                thresh_armed <= 1'b1;
        end
    end

    // -----------------------------------------------------------------
    // Pipeline fill blanking
    //
    // The first several bipolar samples after event_start are computed
    // from stale pipeline data.  The data path from sample_in to the
    // bip_d register (which participates in the ZC comparison) has
    // 6 register stages: 4 (delay/mult path) + 1 (bip) + 1 (bip_d).
    // We blank the hit latch for 6 valid sample cycles after
    // event_start so that no garbage can be mistaken for a crossing.
    // -----------------------------------------------------------------
    localparam [2:0] FILL_CYCLES = 3'd6;

    reg [2:0] fill_cnt;

    always @(posedge clk) begin
        if (!rst_n)
            fill_cnt <= 3'd0;
        else if (event_start)
            fill_cnt <= 3'd0;
        else if (sample_valid && (fill_cnt < FILL_CYCLES))
            fill_cnt <= fill_cnt + 3'd1;
    end

    wire pipeline_valid = (fill_cnt == FILL_CYCLES);

    // -----------------------------------------------------------------
    // First-hit latch
    //
    // hit_armed is set on event_start and cleared on the first valid ZC.
    // hit_found stays high from the first ZC until the next event_start.
    // Ca / Cb / idx_cb_out are latched and remain stable for the
    // top-level state machine to read after the event ends.
    //
    // A zero crossing is accepted only when ALL of the following hold:
    //   1. hit_armed       ? no previous ZC accepted this event
    //   2. thresh_armed    ? raw input exceeded threshold at least once
    //   3. pipeline_valid  ? pipeline has filled with real data
    //   4. sample_valid    ? a valid sample is present
    //   5. zc              ? bipolar zero crossing detected (incl. neg_cnt)
    //
    // ca_out is [15:0] (unsigned) because bip >= 0 at the zero crossing
    // by definition (the !bip[16] condition guarantees this).
    // -----------------------------------------------------------------
    reg hit_armed;

    always @(posedge clk) begin
        if (!rst_n) begin
            hit_found  <= 1'b0;
            hit_armed  <= 1'b0;
            ca_out     <= 16'd0;
            cb_out     <= 17'sd0;
            idx_cb_out <= {IDX_W{1'b0}};
        end else begin
            // Arm on event start, clear previous hit
            if (event_start) begin
                hit_found <= 1'b0;
                hit_armed <= 1'b1;
            end

            // Latch first zero crossing during this event
            if (hit_armed && thresh_armed && pipeline_valid && sample_valid && zc) begin
                hit_found  <= 1'b1;
                hit_armed  <= 1'b0;
                ca_out     <= bip[15:0];  // known non-negative, drop sign bit
                cb_out     <= bip_d;      // previous bipolar sample (< 0)
                idx_cb_out <= idx_bip_d;  // sample index of Cb
            end
        end
    end

endmodule