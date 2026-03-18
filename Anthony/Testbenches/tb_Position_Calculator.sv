`timescale 1ns/1ps
`default_nettype none

module tb_position_calculator;

  // ----------------------------
  // Match DUT parameterization here
  // ----------------------------
  localparam int MULT_LATENCY = 3;
  localparam int TOTAL_LAT    = MULT_LATENCY + 2; // valid_in -> valid_out latency in this RTL

  localparam int EDGE_CASE_TRIALS = 20000;
  localparam int TRIALS           = 200000;

  // ----------------------------
  // Clock / Reset
  // ----------------------------
  logic clk;
  logic rst_n;

  initial begin
    clk = 1'b0;
    forever #5ns clk = ~clk; // 100 MHz
  end

  initial begin
    rst_n = 1'b1;
    #2ns;
    rst_n = 1'b0;
    #20ns;
    rst_n = 1'b1;
  end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  logic                valid_in;
  logic [15:0]         tpx1_in, tpx2_in, tpy1_in, tpy2_in;
  logic signed [15:0]  posconst_x_in, posconst_y_in;
  logic [51:0]         timestamp_in;

  logic signed [31:0]  position_x_out, position_y_out;
  logic [51:0]         timestamp_out;
  logic                valid_out;
  logic                overflow_out;

  Position_Calculator #(.MULT_LATENCY(MULT_LATENCY)) dut (
    .clk            (clk),
    .rst_n          (rst_n),
    .valid_in       (valid_in),

    .tpx1_in        (tpx1_in),
    .tpx2_in        (tpx2_in),
    .tpy1_in        (tpy1_in),
    .tpy2_in        (tpy2_in),

    .posconst_x_in  (posconst_x_in),
    .posconst_y_in  (posconst_y_in),

    .timestamp_in   (timestamp_in),

    .position_x_out (position_x_out),
    .position_y_out (position_y_out),
    .timestamp_out  (timestamp_out),
    .valid_out      (valid_out),
    .overflow_out   (overflow_out)
  );

  // ----------------------------
  // Scoreboard pipeline model
  // ----------------------------
  typedef struct packed {
    logic                v;
    logic [51:0]         ts;
    logic signed [31:0]  px;
    logic signed [31:0]  py;
    logic                ovf;
  } exp_t;

  exp_t exp_pipe [0:TOTAL_LAT];

  function automatic exp_t compute_expected(
    input logic               v_in,
    input logic [51:0]        ts_in,
    input logic [15:0]        ax1, ax2, ay1, ay2,
    input logic signed [15:0] pcx, pcy
  );
    exp_t e;
    logic signed [16:0] dx, dy;
    logic signed [32:0] px33, py33;
    logic               ovx, ovy;
    begin
      // Match RTL exactly: unsigned 16-bit extended with 0, then subtract => signed 17-bit
      dx   = $signed({1'b0, ax1}) - $signed({1'b0, ax2});
      dy   = $signed({1'b0, ay1}) - $signed({1'b0, ay2});

      // Signed 17x16 => signed 33
      px33 = dx * pcx;
      py33 = dy * pcy;

      // Match RTL overflow detect: prod[32] ^ prod[31]
      ovx = (px33[32] ^ px33[31]);
      ovy = (py33[32] ^ py33[31]);

      e.v   = v_in;
      e.ts  = ts_in;
      e.px  = px33[31:0];          // truncation matches RTL
      e.py  = py33[31:0];
      e.ovf = v_in & (ovx | ovy);  // match ovf_aligned = v_aligned & (ovf_x|ovf_y)
      return e;
    end
  endfunction

  // ----------------------------
  // Init inputs
  // ----------------------------
  initial begin
    valid_in       = 1'b0;
    tpx1_in        = 16'd0;
    tpx2_in        = 16'd0;
    tpy1_in        = 16'd0;
    tpy2_in        = 16'd0;
    posconst_x_in  = 16'sd0;
    posconst_y_in  = 16'sd0;
    timestamp_in   = 52'd0;
  end

  int test_count = 0;

  // Pipeline update + checking
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i <= TOTAL_LAT; i++) exp_pipe[i] <= '0;
    end else begin
      // shift expected pipeline
      for (int i = TOTAL_LAT; i > 0; i--) exp_pipe[i] <= exp_pipe[i-1];

      // NOTE: TOTAL_LAT = MULT_LATENCY + 2 matches updated RTL:
      // v0 (cycle 0) -> v1 (cycle 1) -> v_stage[MULT_LATENCY] -> output reg
      exp_pipe[0] <= compute_expected(valid_in, timestamp_in,
                                      tpx1_in, tpx2_in, tpy1_in, tpy2_in,
                                      posconst_x_in, posconst_y_in);

      // Check aligned output when expected valid is high
      if (exp_pipe[TOTAL_LAT].v) begin
        test_count++;

        assert(valid_out === 1'b1)
          else $fatal(1, "valid_out mismatch exp=1 got=%0b", valid_out);

        assert(timestamp_out === exp_pipe[TOTAL_LAT].ts)
          else $fatal(1, "timestamp_out mismatch exp=%0h got=%0h",
                      exp_pipe[TOTAL_LAT].ts, timestamp_out);

        assert(position_x_out === exp_pipe[TOTAL_LAT].px)
          else $fatal(1, "position_x_out mismatch exp=%0d got=%0d",
                      exp_pipe[TOTAL_LAT].px, position_x_out);

        assert(position_y_out === exp_pipe[TOTAL_LAT].py)
          else $fatal(1, "position_y_out mismatch exp=%0d got=%0d",
                      exp_pipe[TOTAL_LAT].py, position_y_out);

        assert(overflow_out === exp_pipe[TOTAL_LAT].ovf)
          else $fatal(1, "overflow_out mismatch exp=%0b got=%0b",
                      exp_pipe[TOTAL_LAT].ovf, overflow_out);

      end else begin
        // Optional strictness: valid_out should be low when we don't expect a valid beat
        assert(valid_out === 1'b0)
          else $fatal(1, "Unexpected valid_out=1 when exp_valid=0");
      end
    end
  end

  // ----------------------------
  // Stimulus helper
  // ----------------------------
  task automatic drive_one(
    input logic               v,
    input logic [51:0]        ts,
    input logic [15:0]        ax1, ax2, ay1, ay2,
    input logic signed [15:0] pcx, pcy
  );
    begin
      valid_in       = v;
      timestamp_in   = ts;
      tpx1_in        = ax1;
      tpx2_in        = ax2;
      tpy1_in        = ay1;
      tpy2_in        = ay2;
      posconst_x_in  = pcx;
      posconst_y_in  = pcy;
      @(posedge clk);
    end
  endtask

  // ----------------------------
  // Test sequence
  // ----------------------------
  initial begin
	 logic [15:0] t;
	 logic v;
    @(posedge rst_n);
    repeat (5) @(posedge clk);

    // Invalid cycles (should not produce valid_out)
    $display("[Position TB] Directed: invalid cycles");
    for (int i = 0; i < 200; i++) begin
      drive_one(1'b0, $urandom(), $urandom(), $urandom(), $urandom(), $urandom(),
                $signed($urandom()), $signed($urandom()));
    end

    // diff=0 => output=0 (truncation irrelevant)
    $display("[Position TB] Directed: diff=0 -> outputs 0");
    for (int i = 0; i < EDGE_CASE_TRIALS; i++) begin
      t = $urandom();
      drive_one(1'b1, $urandom(), t, t, t, t,
                $signed($urandom()), $signed($urandom()));
    end

    // Overflow-forcing corner (exactly matches your ovf rule)
    // dx = -65535, pcx = -32768 => product = +2^31 => prod[32]=0 prod[31]=1 => ovf=1
    $display("[Position TB] Directed: overflow corner (expect overflow_out=1)");
    drive_one(1'b1, 52'h12345, 16'd0, 16'hFFFF, 16'd10, 16'd10,
              16'sh8000, 16'sd1);

    // Fits exactly: dx=+65535, pcx=-32768 => product=-2^31 => prod[32]=1 prod[31]=0? (sign extend makes it fit)
    $display("[Position TB] Directed: -2^31 corner (expect overflow_out=0)");
    drive_one(1'b1, 52'h6789A, 16'hFFFF, 16'd0, 16'd10, 16'd10,
              16'sh8000, 16'sd1);

    // Random tests (mix valid/invalid)
    $display("[Position TB] Random tests");
    for (int i = 0; i < TRIALS; i++) begin
      v = ($urandom_range(0,9) != 0); // ~90% valid beats
      drive_one(v,
                {$urandom(), $urandom_range(0,(1<<20)-1)},
                $urandom(), $urandom(), $urandom(), $urandom(),
                $signed($urandom()), $signed($urandom()));
    end

    // Drain pipeline
    $display("[Position TB] Draining pipeline...");
    for (int i = 0; i < TOTAL_LAT + 10; i++) begin
      drive_one(1'b0, 52'd0, 16'd0, 16'd0, 16'd0, 16'd0, 16'sd0, 16'sd0);
    end

    $display("[Position TB] PASS. Checked valid outputs: %0d", test_count);
    $stop;
  end

endmodule

`default_nettype wire
