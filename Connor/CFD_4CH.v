// =======================
// File: CFD_4CH.v
// =======================
//
// Four-channel Constant Fraction Discriminator ? top level
//
// Event protocol:
//   1. sample_valid goes HIGH  -> new event begins; frontends arm & idx resets.
//   2. sample_valid stays HIGH -> ADC samples stream in every clock.
//   3. sample_valid goes LOW   -> event ends.
//   4. If the state machine is idle (S_WAIT), the four frontends' results
//      are captured into holding registers and processed:
//        * All four hit   -> sequentially divide each channel's Ca/Cb,
//                            then pulse event_valid with all timestamps.
//        * Any missing    -> pulse event_fail.
//      If the state machine is still busy from a prior event, the new
//      event's results are discarded and event_dropped pulses.
//   5. Return to waiting for the next event.
//
// The frontends run freely ? they re-arm on every rising edge of
// sample_valid regardless of the state machine.  The holding registers
// decouple the frontends from the divider, so a new event can begin
// processing in the frontends while the divider finishes the previous
// event without any data corruption.
//
// A 64-bit tag is captured on each event_start and output alongside
// the corresponding event's result (event_valid, event_fail, or
// event_dropped).

module CFD_4CH #(
    parameter integer DELAY_RAM_AW      = 7,
    parameter integer MAX_EVENT_SAMPLES = 1024,
    parameter integer IDX_W             = 10,
    parameter integer FRAC_BITS         = 10
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  sample_valid,
    input  wire signed [15:0]    x1_in,
    input  wire signed [15:0]    x2_in,
    input  wire signed [15:0]    y1_in,
    input  wire signed [15:0]    y2_in,

    // Runtime configuration (must be stable before sample_valid rises)
    input  wire [13:0]                  att_q0_13,
    input  wire [DELAY_RAM_AW-1:0]      delay_val,
    input  wire signed [15:0]           threshold,
    input  wire [7:0]                   zc_neg_samples,

    // Event tag (latched on event_start, output with result)
    input  wire [63:0]           tag_in,

    // Timestamps ? all four valid on the event_valid pulse
    output reg  [IDX_W-1:0]      tpx1_int,
    output reg  [FRAC_BITS-1:0]  tpx1_frac,
    output reg  [IDX_W-1:0]      tpx2_int,
    output reg  [FRAC_BITS-1:0]  tpx2_frac,
    output reg  [IDX_W-1:0]      tpy1_int,
    output reg  [FRAC_BITS-1:0]  tpy1_frac,
    output reg  [IDX_W-1:0]      tpy2_int,
    output reg  [FRAC_BITS-1:0]  tpy2_frac,

    output reg                   event_valid,
    output reg                   event_fail,
    output reg                   event_dropped,
    output reg  [63:0]           tag_out
);

    // =================================================================
    // State encoding
    // =================================================================
    localparam [2:0] S_WAIT      = 3'd0,
                     S_CHECK     = 3'd1,
                     S_DIV_START = 3'd2,
                     S_DIV_WAIT  = 3'd3,
                     S_PUBLISH   = 3'd4;

    reg [2:0]  state;
    reg [1:0]  ch_cnt;

    // =================================================================
    // Event start / end detection ? computed once, shared by all lanes
    //
    // Frontends receive the raw (ungated) event_start so they re-arm
    // on every new event regardless of the state machine.  The holding
    // register capture is guarded separately.
    // =================================================================
    reg sv_d;

    always @(posedge clk) begin
        if (!rst_n)
            sv_d <= 1'b0;
        else
            sv_d <= sample_valid;
    end

    wire event_start = sample_valid & ~sv_d;
    wire event_end   = sv_d & ~sample_valid;

    // =================================================================
    // Live tag register
    //
    // Captured on every event_start (ungated), so it always holds the
    // tag of the most recent event ? even if the state machine is busy.
    // =================================================================
    reg [63:0] tag_live;

    always @(posedge clk) begin
        if (!rst_n)
            tag_live <= 64'd0;
        else if (event_start)
            tag_live <= tag_in;
    end

    // =================================================================
    // Four frontends
    // =================================================================
    wire               fe_hit [0:3];
    wire [15:0]        fe_ca  [0:3];
    wire signed [16:0] fe_cb  [0:3];
    wire [IDX_W-1:0]   fe_idx [0:3];

    wire signed [16:0] fe_bip [0:3];
    wire               fe_zc  [0:3];

    cfd_lane_frontend #(
        .DELAY_RAM_AW     (DELAY_RAM_AW),
        .MAX_EVENT_SAMPLES(MAX_EVENT_SAMPLES),
        .IDX_W            (IDX_W),
        .FRAC_BITS        (FRAC_BITS)
    ) fe_x1 (
        .clk           (clk),
        .rst_n         (rst_n),
        .sample_valid  (sample_valid),
        .event_start   (event_start),
        .sample_in     (x1_in),
        .att_q0_13     (att_q0_13),
        .delay_val     (delay_val),
        .threshold     (threshold),
        .zc_neg_samples(zc_neg_samples),
        .hit_found     (fe_hit[0]),
        .ca_out        (fe_ca[0]),
        .cb_out        (fe_cb[0]),
        .idx_cb_out    (fe_idx[0]),
        .bip           (fe_bip[0]),
        .zc            (fe_zc[0])
    );

    cfd_lane_frontend #(
        .DELAY_RAM_AW     (DELAY_RAM_AW),
        .MAX_EVENT_SAMPLES(MAX_EVENT_SAMPLES),
        .IDX_W            (IDX_W),
        .FRAC_BITS        (FRAC_BITS)
    ) fe_x2 (
        .clk           (clk),
        .rst_n         (rst_n),
        .sample_valid  (sample_valid),
        .event_start   (event_start),
        .sample_in     (x2_in),
        .att_q0_13     (att_q0_13),
        .delay_val     (delay_val),
        .threshold     (threshold),
        .zc_neg_samples(zc_neg_samples),
        .hit_found     (fe_hit[1]),
        .ca_out        (fe_ca[1]),
        .cb_out        (fe_cb[1]),
        .idx_cb_out    (fe_idx[1]),
        .bip           (fe_bip[1]),
        .zc            (fe_zc[1])
    );

    cfd_lane_frontend #(
        .DELAY_RAM_AW     (DELAY_RAM_AW),
        .MAX_EVENT_SAMPLES(MAX_EVENT_SAMPLES),
        .IDX_W            (IDX_W),
        .FRAC_BITS        (FRAC_BITS)
    ) fe_y1 (
        .clk           (clk),
        .rst_n         (rst_n),
        .sample_valid  (sample_valid),
        .event_start   (event_start),
        .sample_in     (y1_in),
        .att_q0_13     (att_q0_13),
        .delay_val     (delay_val),
        .threshold     (threshold),
        .zc_neg_samples(zc_neg_samples),
        .hit_found     (fe_hit[2]),
        .ca_out        (fe_ca[2]),
        .cb_out        (fe_cb[2]),
        .idx_cb_out    (fe_idx[2]),
        .bip           (fe_bip[2]),
        .zc            (fe_zc[2])
    );

    cfd_lane_frontend #(
        .DELAY_RAM_AW     (DELAY_RAM_AW),
        .MAX_EVENT_SAMPLES(MAX_EVENT_SAMPLES),
        .IDX_W            (IDX_W),
        .FRAC_BITS        (FRAC_BITS)
    ) fe_y2 (
        .clk           (clk),
        .rst_n         (rst_n),
        .sample_valid  (sample_valid),
        .event_start   (event_start),
        .sample_in     (y2_in),
        .att_q0_13     (att_q0_13),
        .delay_val     (delay_val),
        .threshold     (threshold),
        .zc_neg_samples(zc_neg_samples),
        .hit_found     (fe_hit[3]),
        .ca_out        (fe_ca[3]),
        .cb_out        (fe_cb[3]),
        .idx_cb_out    (fe_idx[3]),
        .bip           (fe_bip[3]),
        .zc            (fe_zc[3])
    );

    // =================================================================
    // Holding registers
    //
    // Snapshot the frontend results and the live tag on event_end, but
    // ONLY when the state machine is idle (S_WAIT).  This decouples
    // the frontends from the divider.
    //
    // If event_end arrives while the state machine is busy, the
    // holding registers are NOT overwritten and event_dropped pulses
    // with tag_out set to tag_live (the dropped event's tag).
    // =================================================================
    reg [15:0]        hold_ca  [0:3];
    reg signed [16:0] hold_cb  [0:3];
    reg [IDX_W-1:0]   hold_idx [0:3];
    reg               hold_all_hit;
    reg [63:0]        hold_tag;

    integer ch;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (ch = 0; ch < 4; ch = ch + 1) begin
                hold_ca[ch]  <= 16'd0;
                hold_cb[ch]  <= 17'sd0;
                hold_idx[ch] <= {IDX_W{1'b0}};
            end
            hold_all_hit <= 1'b0;
            hold_tag     <= 64'd0;
        end else if (event_end && (state == S_WAIT)) begin
            for (ch = 0; ch < 4; ch = ch + 1) begin
                hold_ca[ch]  <= fe_ca[ch];
                hold_cb[ch]  <= fe_cb[ch];
                hold_idx[ch] <= fe_idx[ch];
            end
            hold_all_hit <= fe_hit[0] & fe_hit[1] & fe_hit[2] & fe_hit[3];
            hold_tag     <= tag_live;
        end
    end

    // =================================================================
    // Channel mux ? reads from holding registers
    // =================================================================
    reg [15:0]        sel_ca;
    reg signed [16:0] sel_cb;
    reg [IDX_W-1:0]   sel_idx;

    always @(*) begin
        case (ch_cnt)
            2'd0: begin sel_ca = hold_ca[0]; sel_cb = hold_cb[0]; sel_idx = hold_idx[0]; end
            2'd1: begin sel_ca = hold_ca[1]; sel_cb = hold_cb[1]; sel_idx = hold_idx[1]; end
            2'd2: begin sel_ca = hold_ca[2]; sel_cb = hold_cb[2]; sel_idx = hold_idx[2]; end
            default: begin sel_ca = hold_ca[3]; sel_cb = hold_cb[3]; sel_idx = hold_idx[3]; end
        endcase
    end

    // Divider operands:  frac = (-Cb) / (Ca - Cb)
    wire signed [16:0] ca_wide = {1'b0, sel_ca};
    wire [16:0] div_num_w = sel_cb[16] ? $unsigned(-sel_cb) : 17'd0;
    wire [16:0] div_den_w = $unsigned(ca_wide - sel_cb);

    // =================================================================
    // Divider instance
    // =================================================================
    reg                  div_start;
    wire                 div_busy;
    wire                 div_done;
    wire [FRAC_BITS-1:0] div_q;

    cfd_div_u_frac_serial #(
        .W        (17),
        .FRAC_BITS(FRAC_BITS)
    ) u_div_ser (
        .clk  (clk),
        .rst_n(rst_n),
        .start(div_start),
        .num  (div_num_w),
        .den  (div_den_w),
        .busy (div_busy),
        .done (div_done),
        .q    (div_q)
    );

    // =================================================================
    // Main state machine
    // =================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state         <= S_WAIT;
            ch_cnt        <= 2'd0;
            div_start     <= 1'b0;
            event_valid   <= 1'b0;
            event_fail    <= 1'b0;
            event_dropped <= 1'b0;
            tag_out       <= 64'd0;

            tpx1_int  <= {IDX_W{1'b0}};
            tpx1_frac <= {FRAC_BITS{1'b0}};
            tpx2_int  <= {IDX_W{1'b0}};
            tpx2_frac <= {FRAC_BITS{1'b0}};
            tpy1_int  <= {IDX_W{1'b0}};
            tpy1_frac <= {FRAC_BITS{1'b0}};
            tpy2_int  <= {IDX_W{1'b0}};
            tpy2_frac <= {FRAC_BITS{1'b0}};
        end else begin
            // Default single-cycle pulses
            event_valid   <= 1'b0;
            event_fail    <= 1'b0;
            event_dropped <= 1'b0;
            div_start     <= 1'b0;

            // ---------------------------------------------------------
            // Event drop detection: event_end while busy.
            // Output the dropped event's tag (from tag_live, since
            // the holding registers were not updated).
            // ---------------------------------------------------------
            if (event_end && (state != S_WAIT)) begin
                event_dropped <= 1'b1;
                tag_out       <= tag_live;
            end

            case (state)
                // -------------------------------------------------
                // Wait for event_end.  The holding registers are
                // captured in the separate always block above on
                // the same cycle, so they are ready for S_CHECK.
                // -------------------------------------------------
                S_WAIT: begin
                    if (event_end)
                        state <= S_CHECK;
                end

                // -------------------------------------------------
                // Evaluate whether all four channels found a crossing
                // -------------------------------------------------
                S_CHECK: begin
                    if (hold_all_hit) begin
                        state  <= S_DIV_START;
                        ch_cnt <= 2'd0;
                    end else begin
                        state <= S_PUBLISH;
                    end
                end

                // -------------------------------------------------
                // Kick off the divider for channel ch_cnt
                // -------------------------------------------------
                S_DIV_START: begin
                    div_start <= 1'b1;
                    state     <= S_DIV_WAIT;
                end

                // -------------------------------------------------
                // Wait for divider to finish, capture result
                // -------------------------------------------------
                S_DIV_WAIT: begin
                    if (div_done) begin
                        case (ch_cnt)
                            2'd0: begin tpx1_int <= sel_idx; tpx1_frac <= div_q; end
                            2'd1: begin tpx2_int <= sel_idx; tpx2_frac <= div_q; end
                            2'd2: begin tpy1_int <= sel_idx; tpy1_frac <= div_q; end
                            2'd3: begin tpy2_int <= sel_idx; tpy2_frac <= div_q; end
                        endcase

                        if (ch_cnt == 2'd3) begin
                            state <= S_PUBLISH;
                        end else begin
                            ch_cnt <= ch_cnt + 2'd1;
                            state  <= S_DIV_START;
                        end
                    end
                end

                // -------------------------------------------------
                // Publish result for one clock cycle
                // -------------------------------------------------
                S_PUBLISH: begin
                    tag_out <= hold_tag;

                    if (hold_all_hit)
                        event_valid <= 1'b1;
                    else
                        event_fail <= 1'b1;

                    state <= S_WAIT;
                end

                default: state <= S_WAIT;
            endcase
        end
    end

endmodule