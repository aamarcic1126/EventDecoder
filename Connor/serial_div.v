// =======================
// File: serial_div.v
// =======================
//
// Unsigned fractional serial divider.
//
// Computes  Q = num / den  as a FRAC_BITS-bit fixed-point fraction
// (i.e. the result is in the range [0, 1) represented as an integer
// with an implicit 2^{-FRAC_BITS} scaling).
//
// Restoring division: each iteration shifts the remainder left by one
// bit, compares with the denominator, and subtracts if greater.
//
// Latency: FRAC_BITS clock cycles after start is asserted.

module cfd_div_u_frac_serial #(
    parameter integer W         = 17,
    parameter integer FRAC_BITS = 10
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 start,
    input  wire [W-1:0]         num,
    input  wire [W-1:0]         den,
    output reg                  busy,
    output reg                  done,
    output reg  [FRAC_BITS-1:0] q
);

    reg [W-1:0] rem;
    reg [W-1:0] denom;
    reg [$clog2(FRAC_BITS+1)-1:0] bit_cnt;

    // q_r only needs FRAC_BITS-1 bits: on each iteration we read
    // q_r[FRAC_BITS-3:0] for the shift, and on the final iteration
    // we read all of q_r[FRAC_BITS-2:0] to construct the full output.
    reg [FRAC_BITS-2:0] q_r;

    // Shifted remainder (W+1 bits) for comparison; only the lower W
    // bits are stored back into rem after each iteration.
    wire [W:0]   rem_shifted = {rem, 1'b0};
    wire         cmp         = (rem_shifted >= {1'b0, denom});
    wire [W-1:0] rem_sub     = rem_shifted[W-1:0] - denom;

    always @(posedge clk) begin
        if (!rst_n) begin
            busy    <= 1'b0;
            done    <= 1'b0;
            q       <= {FRAC_BITS{1'b0}};
            q_r     <= {(FRAC_BITS-1){1'b0}};
            rem     <= {W{1'b0}};
            denom   <= {W{1'b0}};
            bit_cnt <= {($clog2(FRAC_BITS+1)){1'b0}};
        end else begin
            done <= 1'b0;

            if (start && !busy) begin
                // Latch operands and begin
                busy    <= 1'b1;
                q_r     <= {(FRAC_BITS-1){1'b0}};
                rem     <= num;
                denom   <= den;
                bit_cnt <= FRAC_BITS[$clog2(FRAC_BITS+1)-1:0];
            end else if (busy) begin
                // --- Shift & subtract ---
                if (cmp) begin
                    rem <= rem_sub;
                    q_r <= {q_r[FRAC_BITS-3:0], 1'b1};
                end else begin
                    rem <= rem_shifted[W-1:0];
                    q_r <= {q_r[FRAC_BITS-3:0], 1'b0};
                end

                bit_cnt <= bit_cnt - 1'b1;

                // --- Final iteration ---
                // Capture the completed quotient *including* the current
                // bit.  (The q_r non-blocking assignment above has not
                // taken effect yet, so we construct q explicitly from
                // the pre-update q_r plus the current comparison result.)
                if (bit_cnt == 1) begin
                    busy <= 1'b0;
                    done <= 1'b1;

                    if (cmp)
                        q <= {q_r, 1'b1};
                    else
                        q <= {q_r, 1'b0};
                end
            end
        end
    end

endmodule