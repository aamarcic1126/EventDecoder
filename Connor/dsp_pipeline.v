// ===========================================================================
// File: cfd_system_top.v
//
// System top level: ADC ? Preprocessor ? CFD + Charge Integrator
//                                         ?
//                                   Position Calculator
//
// Signal chain:
//   1. Raw 12-bit unsigned ADC samples (4 channels) enter the
//      preprocessor, which computes and subtracts the baseline
//      (average of first 8 samples).  Output: signed 16-bit SQ12.3.
//
//   2. The corrected samples feed both (in parallel):
//      a. CFD_4CH ? extracts zero-crossing timestamps for all 4 channels
//      b. Charge_Integrator_V1 ? accumulates total charge across the event
//
//   3. The CFD's timestamps feed the position calculator, which computes
//      X/Y position in microns using the propagation constants Kx and Ky.
//
// Tag propagation:
//   The 64-bit tag_in must be stable when adc_valid rises.
//
//   tag_in ? preprocessor (latched on adc_valid rising edge)
//          ? pp_tag (muxed: flush_tag during flush, tag_reg otherwise)
//          ? CFD tag_in  (latched on pp_valid rising edge)
//          ? cfd_tag     (output on event_valid / event_fail / event_dropped)
//          ? position calculator tag_in (latched on event_valid)
//          ? pos_tag_out (output on pos_valid / pos_rejected)
//
//          ? charge integrator in_timestamp (latched on first pp_valid)
//          ? charge_tag_out (output on charge_valid)
//
//   Each output carries the tag of the event that produced it.
//   During flush overlap (1-cycle gap between events), the preprocessor
//   uses a saved tag so that flush outputs carry the old event's tag
//   while the new event's tag is being latched.
//
// Timing:
//   For the same event:
//     charge_valid fires shortly after preprocessor output ends.
//     pos_valid fires ~90 cycles later (46 CFD division + 45 position mult).
//   Use tags to correlate results across outputs.
// ===========================================================================

module dsp_pipeline #(
    // Preprocessor / CFD shared
    parameter integer DELAY_RAM_AW      = 7,
    parameter integer MAX_EVENT_SAMPLES = 1024,
    parameter integer IDX_W             = 10,
    parameter integer FRAC_BITS         = 10,

    // Position calculator
    parameter integer K_WIDTH   = 20,
    parameter integer K_FRAC    = 19,
    parameter integer POS_WIDTH = 32,
    parameter integer WINDOW_X  = 51000,
    parameter integer WINDOW_Y  = 51000,

    // Charge integrator
    parameter integer SUM_WIDTH = 26
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- Raw ADC inputs ----
    input  wire        adc_valid,
    input  wire [11:0] adc_x1,
    input  wire [11:0] adc_x2,
    input  wire [11:0] adc_y1,
    input  wire [11:0] adc_y2,

    // ---- Runtime CFD configuration ----
    // (must be stable before adc_valid rises)
    input  wire [13:0]                 att_q0_13,
    input  wire [DELAY_RAM_AW-1:0]     delay_val,
    input  wire signed [15:0]          threshold,
    input  wire [7:0]                  zc_neg_samples,

    // ---- Runtime position calculator constants (UQ1.19) ----
    input  wire [K_WIDTH-1:0]          kx,
    input  wire [K_WIDTH-1:0]          ky,

    // ---- Event tag (must be stable when adc_valid rises) ----
    input  wire [63:0]                 tag_in,

    // ---- Position calculator outputs ----
    output wire signed [POS_WIDTH-1:0] x_pos,
    output wire signed [POS_WIDTH-1:0] y_pos,
    output wire                        pos_valid,
    output wire                        pos_rejected,
    output wire [63:0]                 pos_tag_out,

    // ---- CFD event status outputs ----
    output wire                        event_valid,
    output wire                        event_fail,
    output wire                        event_dropped,
    output wire [63:0]                 cfd_tag_out,

    // ---- Charge integrator outputs ----
    output wire                        charge_valid,
    output wire [SUM_WIDTH-1:0]        charge_sum,
    output wire [63:0]                 charge_tag_out,
    output wire                        charge_overflow
);

    // =================================================================
    // Internal wires: preprocessor outputs
    // =================================================================
    wire                pp_valid;
    wire signed [15:0]  pp_x1, pp_x2, pp_y1, pp_y2;
    wire [63:0]         pp_tag;

    // =================================================================
    // Internal wires: CFD outputs
    // =================================================================
    wire                cfd_ev_valid;
    wire                cfd_ev_fail;
    wire                cfd_ev_dropped;
    wire [IDX_W-1:0]    cfd_tpx1_int,  cfd_tpx2_int,  cfd_tpy1_int,  cfd_tpy2_int;
    wire [FRAC_BITS-1:0] cfd_tpx1_frac, cfd_tpx2_frac, cfd_tpy1_frac, cfd_tpy2_frac;
    wire [63:0]         cfd_tag;

    // =================================================================
    // Preprocessor
    //
    // 12-bit unsigned ADC ? baseline subtraction ? signed 16-bit SQ12.3
    // Tag latched on adc_valid rising edge, muxed during flush.
    // Latency: 8 sample clocks at event start.
    // =================================================================
    cfd_preprocess u_preprocess (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (adc_valid),
        .x1_raw   (adc_x1),
        .x2_raw   (adc_x2),
        .y1_raw   (adc_y1),
        .y2_raw   (adc_y2),
        .tag_in   (tag_in),
        .tag_out  (pp_tag),
        .out_valid(pp_valid),
        .x1_out   (pp_x1),
        .x2_out   (pp_x2),
        .y1_out   (pp_y1),
        .y2_out   (pp_y2)
    );

    // =================================================================
    // CFD ? 4-channel constant fraction discriminator
    //
    // Tag latched on pp_valid rising edge (preprocessor's event_start).
    // Output tag on event_valid, event_fail, and event_dropped.
    // =================================================================
    CFD_4CH #(
        .DELAY_RAM_AW     (DELAY_RAM_AW),
        .MAX_EVENT_SAMPLES(MAX_EVENT_SAMPLES),
        .IDX_W            (IDX_W),
        .FRAC_BITS        (FRAC_BITS)
    ) u_cfd (
        .clk           (clk),
        .rst_n         (rst_n),
        .sample_valid  (pp_valid),
        .x1_in         (pp_x1),
        .x2_in         (pp_x2),
        .y1_in         (pp_y1),
        .y2_in         (pp_y2),
        .att_q0_13     (att_q0_13),
        .delay_val     (delay_val),
        .threshold     (threshold),
        .zc_neg_samples(zc_neg_samples),
        .tag_in        (pp_tag),
        .tpx1_int      (cfd_tpx1_int),
        .tpx1_frac     (cfd_tpx1_frac),
        .tpx2_int      (cfd_tpx2_int),
        .tpx2_frac     (cfd_tpx2_frac),
        .tpy1_int      (cfd_tpy1_int),
        .tpy1_frac     (cfd_tpy1_frac),
        .tpy2_int      (cfd_tpy2_int),
        .tpy2_frac     (cfd_tpy2_frac),
        .event_valid   (cfd_ev_valid),
        .event_fail    (cfd_ev_fail),
        .event_dropped (cfd_ev_dropped),
        .tag_out       (cfd_tag)
    );

    // CFD status ? forwarded to top-level ports
    assign event_valid   = cfd_ev_valid;
    assign event_fail    = cfd_ev_fail;
    assign event_dropped = cfd_ev_dropped;
    assign cfd_tag_out   = cfd_tag;

    // =================================================================
    // Position calculator
    //
    // Converts timestamp differences to X/Y position in microns.
    // Triggered only on event_valid (not fail/dropped).
    // Tag received from CFD, passed through to pos_tag_out.
    // =================================================================
    cfd_position_calc #(
        .IDX_W     (IDX_W),
        .FRAC_BITS (FRAC_BITS),
        .K_WIDTH   (K_WIDTH),
        .K_FRAC    (K_FRAC),
        .POS_WIDTH (POS_WIDTH),
        .WINDOW_X  (WINDOW_X),
        .WINDOW_Y  (WINDOW_Y)
    ) u_pos_calc (
        .clk         (clk),
        .rst_n       (rst_n),
        .event_valid (cfd_ev_valid),
        .tx1_int     (cfd_tpx1_int),
        .tx1_frac    (cfd_tpx1_frac),
        .tx2_int     (cfd_tpx2_int),
        .tx2_frac    (cfd_tpx2_frac),
        .ty1_int     (cfd_tpy1_int),
        .ty1_frac    (cfd_tpy1_frac),
        .ty2_int     (cfd_tpy2_int),
        .ty2_frac    (cfd_tpy2_frac),
        .kx          (kx),
        .ky          (ky),
        .tag_in      (cfd_tag),
        .x_pos       (x_pos),
        .y_pos       (y_pos),
        .pos_valid   (pos_valid),
        .pos_rejected(pos_rejected),
        .tag_out     (pos_tag_out)
    );

    // =================================================================
    // Charge integrator
    //
    // Accumulates total charge (sum of all 4 channels × all samples).
    // Runs in parallel with the CFD from the same preprocessor outputs.
    // Tag latched on first pp_valid sample of each burst.
    // =================================================================
    Charge_Integrator_V1 #(
        .ADC_WIDTH (16),
        .TS_WIDTH  (64),
        .SUM_WIDTH (SUM_WIDTH)
    ) u_charge (
        .clk          (clk),
        .rst_n        (rst_n),
        .in_valid     (pp_valid),
        .adc0         (pp_x1),
        .adc1         (pp_x2),
        .adc2         (pp_y1),
        .adc3         (pp_y2),
        .in_timestamp (pp_tag),
        .out_valid    (charge_valid),
        .out_sum      (charge_sum),
        .out_timestamp(charge_tag_out),
        .out_overflow (charge_overflow)
    );

endmodule
