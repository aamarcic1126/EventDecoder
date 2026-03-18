// ===========================================================================
// File: cfd_position_calc.v
//
// Position calculator for CFD_4CH.
//
// Computes event position from zero-crossing timestamps:
//   x_pos = (tx2 - tx1) * Kx    [signed, in microns]
//   y_pos = (ty2 - ty1) * Ky    [signed, in microns]
//
// where Kx, Ky are UQ1.19 propagation constants encoding:
//   K = v_prop_mm_per_ns * 1000 / (2 * 1024 * fs_GHz)
//
// To compute K_encoded for a given system:
//   K_encoded = round(v_prop_mm_per_ns * 1000 / (2 * 1024 * fs_GHz) * 2^19)
//
// Example: v = 1.33 mm/ns, fs = 3.6 GHz:
//   K_real = 1.33 * 1000 / (2 * 1024 * 3.6) = 0.18042 um/count
//   K_encoded = round(0.18042 * 524288) = 94603
//
// Sequential signed x unsigned multiply, 20 cycles per axis, 2 axes.
// Total latency: 45 cycles from event_valid to pos_valid.
//
// A 64-bit tag is registered on event_valid and passed through
// to the output on pos_valid.
// ===========================================================================

module cfd_position_calc #(
    parameter integer IDX_W      = 10,
    parameter integer FRAC_BITS  = 10,
    parameter integer K_WIDTH    = 20,    // UQ1.19 total bits
    parameter integer K_FRAC     = 19,    // fractional bits in K
    parameter integer POS_WIDTH  = 32,
    // Position window: events with |x_pos| > WINDOW_X or
    // |y_pos| > WINDOW_Y are rejected.  Units are microns.
    // Set to maximum positive value to effectively disable.
    parameter integer WINDOW_X   = 51000,  // ±51 mm default
    parameter integer WINDOW_Y   = 51000   // ±51 mm default
)(
    input  wire                           clk,
    input  wire                           rst_n,

    // From CFD_4CH ? active on event_valid pulse
    input  wire                           event_valid,
    input  wire [IDX_W-1:0]              tx1_int,
    input  wire [FRAC_BITS-1:0]          tx1_frac,
    input  wire [IDX_W-1:0]              tx2_int,
    input  wire [FRAC_BITS-1:0]          tx2_frac,
    input  wire [IDX_W-1:0]              ty1_int,
    input  wire [FRAC_BITS-1:0]          ty1_frac,
    input  wire [IDX_W-1:0]              ty2_int,
    input  wire [FRAC_BITS-1:0]          ty2_frac,

    // Propagation constants (UQ1.19, unsigned 20-bit)
    input  wire [K_WIDTH-1:0]            kx,
    input  wire [K_WIDTH-1:0]            ky,

    // Tag passthrough
    input  wire [63:0]                   tag_in,

    // Outputs ? valid on pos_valid pulse
    output reg signed [POS_WIDTH-1:0]    x_pos,
    output reg signed [POS_WIDTH-1:0]    y_pos,
    output reg                           pos_valid,
    output reg                           pos_rejected,
    output reg [63:0]                    tag_out
);

    // -----------------------------------------------------------------
    // Derived widths
    // -----------------------------------------------------------------
    localparam integer TS_W   = IDX_W + FRAC_BITS;      // 20: full timestamp
    localparam integer DT_W   = TS_W + 1;               // 21: signed difference
    localparam integer PROD_W = DT_W + K_WIDTH;          // 41: full product

    // -----------------------------------------------------------------
    // State encoding
    // -----------------------------------------------------------------
    localparam [2:0] S_IDLE    = 3'd0,
                     S_CALC_DT = 3'd1,
                     S_MULT_X  = 3'd2,
                     S_STORE_X = 3'd3,
                     S_MULT_Y  = 3'd4,
                     S_STORE_Y = 3'd5,
                     S_PUBLISH = 3'd6;

    reg [2:0]  state;
    reg [4:0]  bit_cnt;          // counts 0 to K_WIDTH-1

    // -----------------------------------------------------------------
    // Registered inputs
    // -----------------------------------------------------------------
    reg [63:0]           tag_reg;
    reg [K_WIDTH-1:0]    kx_reg;
    reg [K_WIDTH-1:0]    ky_reg;

    // Full timestamps (concatenated int + frac)
    wire [TS_W-1:0] t_x1 = {tx1_int, tx1_frac};
    wire [TS_W-1:0] t_x2 = {tx2_int, tx2_frac};
    wire [TS_W-1:0] t_y1 = {ty1_int, ty1_frac};
    wire [TS_W-1:0] t_y2 = {ty2_int, ty2_frac};

    // Signed timestamp difference for Y axis (X is computed directly
    // into dt_active, so no separate register is needed)
    reg signed [DT_W-1:0] dt_y;

    // -----------------------------------------------------------------
    // Sequential multiplier registers
    //
    // MSB-first shift-and-add:  signed dt × unsigned K
    //   accum starts at 0
    //   each cycle: accum = (accum << 1) + (k_shift[MSB] ? dt : 0)
    //   k_shift shifts left each cycle
    //
    // After K_WIDTH iterations, accum holds the full signed product.
    // -----------------------------------------------------------------
    reg signed [PROD_W-1:0]   accum;
    reg [K_WIDTH-1:0]         k_shift;

    // Sign-extended dt for addition into the accumulator
    reg signed [DT_W-1:0]     dt_active;  // holds dt for current axis

    wire signed [PROD_W-1:0]  dt_ext = {{(PROD_W-DT_W){dt_active[DT_W-1]}}, dt_active};

    // -----------------------------------------------------------------
    // Stored axis results
    // -----------------------------------------------------------------
    reg signed [POS_WIDTH-1:0] x_result;
    reg signed [POS_WIDTH-1:0] y_result;

    // -----------------------------------------------------------------
    // Product to position conversion
    //
    // product has K_FRAC fractional bits.  Arithmetic right shift by
    // K_FRAC gives integer microns.  Add rounding bias (1 << (K_FRAC-1))
    // before the shift for proper rounding.
    // -----------------------------------------------------------------
    wire signed [PROD_W-1:0] accum_rounded = accum + (1 <<< (K_FRAC - 1));
    wire signed [PROD_W-1:0] pos_shifted   = accum_rounded >>> K_FRAC;

    // -----------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            bit_cnt      <= 5'd0;
            pos_valid    <= 1'b0;
            pos_rejected <= 1'b0;
            x_pos        <= {POS_WIDTH{1'b0}};
            y_pos     <= {POS_WIDTH{1'b0}};
            tag_out   <= 64'd0;
            tag_reg   <= 64'd0;
            kx_reg    <= {K_WIDTH{1'b0}};
            ky_reg    <= {K_WIDTH{1'b0}};
            dt_y      <= {DT_W{1'b0}};
            accum     <= {PROD_W{1'b0}};
            k_shift   <= {K_WIDTH{1'b0}};
            dt_active <= {DT_W{1'b0}};
            x_result  <= {POS_WIDTH{1'b0}};
            y_result  <= {POS_WIDTH{1'b0}};
        end else begin
            pos_valid    <= 1'b0;
            pos_rejected <= 1'b0;

            case (state)
                // -------------------------------------------------
                // Wait for event_valid, latch inputs
                // -------------------------------------------------
                S_IDLE: begin
                    if (event_valid) begin
                        tag_reg <= tag_in;
                        kx_reg  <= kx;
                        ky_reg  <= ky;
                        state   <= S_CALC_DT;
                    end
                end

                // -------------------------------------------------
                // Compute signed timestamp differences and
                // prepare X-axis multiply
                // -------------------------------------------------
                S_CALC_DT: begin
                    dt_y <= $signed({1'b0, t_y2}) - $signed({1'b0, t_y1});

                    // Prepare X multiply (X difference loaded into dt_active)
                    dt_active <= $signed({1'b0, t_x2}) - $signed({1'b0, t_x1});
                    k_shift   <= kx_reg;
                    accum     <= {PROD_W{1'b0}};
                    bit_cnt   <= K_WIDTH[4:0];

                    state <= S_MULT_X;
                end

                // -------------------------------------------------
                // Sequential multiply: (tx2-tx1) * Kx  (K_WIDTH cycles)
                //
                // MSB-first: shift accum left, conditionally add dt
                // -------------------------------------------------
                S_MULT_X: begin
                    // Shift and conditionally add
                    if (k_shift[K_WIDTH-1])
                        accum <= ({accum[PROD_W-2:0], 1'b0}) + dt_ext;
                    else
                        accum <= {accum[PROD_W-2:0], 1'b0};

                    k_shift <= {k_shift[K_WIDTH-2:0], 1'b0};
                    bit_cnt <= bit_cnt - 5'd1;

                    if (bit_cnt == 5'd1)
                        state <= S_STORE_X;
                end

                // -------------------------------------------------
                // Store X result, prepare Y-axis multiply
                // -------------------------------------------------
                S_STORE_X: begin
                    x_result  <= pos_shifted[POS_WIDTH-1:0];

                    // Prepare Y multiply
                    dt_active <= dt_y;
                    k_shift   <= ky_reg;
                    accum     <= {PROD_W{1'b0}};
                    bit_cnt   <= K_WIDTH[4:0];

                    state <= S_MULT_Y;
                end

                // -------------------------------------------------
                // Sequential multiply: dt_y * Ky  (K_WIDTH cycles)
                // -------------------------------------------------
                S_MULT_Y: begin
                    if (k_shift[K_WIDTH-1])
                        accum <= ({accum[PROD_W-2:0], 1'b0}) + dt_ext;
                    else
                        accum <= {accum[PROD_W-2:0], 1'b0};

                    k_shift <= {k_shift[K_WIDTH-2:0], 1'b0};
                    bit_cnt <= bit_cnt - 5'd1;

                    if (bit_cnt == 5'd1)
                        state <= S_STORE_Y;
                end

                // -------------------------------------------------
                // Store Y result
                // -------------------------------------------------
                S_STORE_Y: begin
                    y_result <= pos_shifted[POS_WIDTH-1:0];
                    state    <= S_PUBLISH;
                end

                // -------------------------------------------------
                // Window check and output
                //
                // Compare |x_result| and |y_result| against the
                // compile-time window parameters.  If either axis
                // exceeds its window, assert pos_rejected instead
                // of pos_valid.
                // -------------------------------------------------
                S_PUBLISH: begin
                    x_pos   <= x_result;
                    y_pos   <= y_result;
                    tag_out <= tag_reg;

                    if ((x_result > $signed(WINDOW_X)) ||
                        (x_result < -$signed(WINDOW_X)) ||
                        (y_result > $signed(WINDOW_Y)) ||
                        (y_result < -$signed(WINDOW_Y)))
                        pos_rejected <= 1'b1;
                    else
                        pos_valid <= 1'b1;

                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule