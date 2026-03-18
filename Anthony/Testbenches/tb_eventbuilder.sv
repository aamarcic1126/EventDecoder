`timescale 1ns/1ps

module tb_eventbuilder;

  // ============================
  // Parameters
  // ============================
  localparam integer TS_WIDTH   = 52;
  localparam integer POS_WIDTH  = 32;
  localparam integer MAG_WIDTH  = 26;
  localparam integer DEPTH_LOG2 = 3;
  localparam integer DEPTH      = (1 << DEPTH_LOG2);

  // ============================
  // Clock / Reset
  // ============================
  reg clk;
  reg rst_n;

  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ============================
  // DUT inputs
  // ============================
  reg                  pos_valid;
  reg [TS_WIDTH-1:0]   pos_time;
  reg [POS_WIDTH-1:0]  pos_data;

  reg                  chg_valid;
  reg [TS_WIDTH-1:0]   chg_time;
  reg [MAG_WIDTH-1:0]  chg_mag;
  reg                  chg_overflow;

  // DUT outputs
  wire                 out_valid;
  wire [TS_WIDTH-1:0]  out_time;
  wire [POS_WIDTH-1:0] out_pos;
  wire [MAG_WIDTH-1:0] out_mag;
  wire                 drop_pos;
  wire                 drop_chg;
  wire                 match_overflow;

  // ============================
  // DUT
  // ============================
  eventbuilder #(
    .TS_WIDTH(TS_WIDTH),
    .POS_WIDTH(POS_WIDTH),
    .MAG_WIDTH(MAG_WIDTH),
    .DEPTH_LOG2(DEPTH_LOG2)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),
    .pos_valid(pos_valid),
    .pos_time(pos_time),
    .pos_data(pos_data),
    .chg_valid(chg_valid),
    .chg_time(chg_time),
    .chg_mag(chg_mag),
    .chg_overflow(chg_overflow),
    .out_valid(out_valid),
    .out_time(out_time),
    .out_pos(out_pos),
    .out_mag(out_mag),
    .drop_pos(drop_pos),
    .drop_chg(drop_chg),
    .match_overflow(match_overflow)
  );

  // ============================================================
  // Reference model data structures
  // ============================================================
  typedef struct packed {
    reg [TS_WIDTH-1:0]  t;
    reg [POS_WIDTH-1:0] p;
  } pos_item_t;

  typedef struct packed {
    reg [TS_WIDTH-1:0]  t;
    reg [MAG_WIDTH-1:0] m;
    reg                 of;
  } chg_item_t;

  typedef struct packed {
    reg [TS_WIDTH-1:0]  t;
    reg [POS_WIDTH-1:0] p;
    reg [MAG_WIDTH-1:0] m;
    reg                 of;
  } out_item_t;

  pos_item_t pos_q[$];
  chg_item_t chg_q[$];
  out_item_t exp_q[$];

  integer errors;
  integer matches_model;
  integer drops_pos_model;
  integer drops_chg_model;
  integer drop_new_pos_full;
  integer drop_new_chg_full;

  reg prev_out_valid;
  reg prev_drop_pos;
  reg prev_drop_chg;

  // ============================================================
  // Utility tasks
  // ============================================================
  task automatic reset_stats_and_model;
    begin
      pos_q.delete();
      chg_q.delete();
      exp_q.delete();

      errors            = 0;
      matches_model     = 0;
      drops_pos_model   = 0;
      drops_chg_model   = 0;
      drop_new_pos_full = 0;
      drop_new_chg_full = 0;

      prev_out_valid    = 0;
      prev_drop_pos     = 0;
      prev_drop_chg     = 0;
    end
  endtask

  task automatic do_reset;
    begin
      rst_n         = 1'b0;
      pos_valid     = 1'b0;
      pos_time      = {TS_WIDTH{1'b0}};
      pos_data      = {POS_WIDTH{1'b0}};
      chg_valid     = 1'b0;
      chg_time      = {TS_WIDTH{1'b0}};
      chg_mag       = {MAG_WIDTH{1'b0}};
      chg_overflow  = 1'b0;

      reset_stats_and_model();

      repeat (5) @(posedge clk);
      rst_n = 1'b1;
      repeat (2) @(posedge clk);
    end
  endtask

  task automatic idle_cycles(input integer n);
    integer i;
    begin
      for (i = 0; i < n; i = i + 1)
        @(posedge clk);
    end
  endtask

  // ============================================================
  // Reference model
  // ============================================================
  task automatic model_push_pos(input pos_item_t it);
    begin
      if (pos_q.size() < DEPTH)
        pos_q.push_back(it);
      else
        drop_new_pos_full = drop_new_pos_full + 1;
    end
  endtask

  task automatic model_push_chg(input chg_item_t it);
    begin
      if (chg_q.size() < DEPTH)
        chg_q.push_back(it);
      else
        drop_new_chg_full = drop_new_chg_full + 1;
    end
  endtask

  task automatic model_step;
    pos_item_t ph;
    chg_item_t ch;
    out_item_t oi;
    begin
      if ((pos_q.size() > 0) && (chg_q.size() > 0)) begin
        ph = pos_q[0];
        ch = chg_q[0];

        if (ph.t == ch.t) begin
          oi.t  = ph.t;
          oi.p  = ph.p;
          oi.m  = ch.m;
          oi.of = ch.of;

          exp_q.push_back(oi);
          matches_model = matches_model + 1;

          pos_q.delete(0);
          chg_q.delete(0);
        end
        else if (ph.t < ch.t) begin
          pos_q.delete(0);
          drops_pos_model = drops_pos_model + 1;
        end
        else begin
          chg_q.delete(0);
          drops_chg_model = drops_chg_model + 1;
        end
      end
    end
  endtask

  // ============================================================
  // Driver tasks
  // ============================================================
  task automatic drive_pos_pulse(
    input reg [TS_WIDTH-1:0]  t,
    input reg [POS_WIDTH-1:0] p
  );
    pos_item_t item;
    begin
      @(posedge clk);
      pos_valid <= 1'b1;
      pos_time  <= t;
      pos_data  <= p;

      item.t = t;
      item.p = p;
      model_push_pos(item);

      @(posedge clk);
      pos_valid <= 1'b0;
    end
  endtask

  task automatic drive_chg_pulse(
    input reg [TS_WIDTH-1:0]  t,
    input reg [MAG_WIDTH-1:0] m,
    input reg                 of
  );
    chg_item_t item;
    begin
      @(posedge clk);
      chg_valid    <= 1'b1;
      chg_time     <= t;
      chg_mag      <= m;
      chg_overflow <= of;

      item.t  = t;
      item.m  = m;
      item.of = of;
      model_push_chg(item);

      @(posedge clk);
      chg_valid <= 1'b0;
    end
  endtask

  task automatic drive_both_pulse_same_cycle(
    input reg [TS_WIDTH-1:0]  tpos,
    input reg [POS_WIDTH-1:0] p,
    input reg [TS_WIDTH-1:0]  tchg,
    input reg [MAG_WIDTH-1:0] m,
    input reg                 of
  );
    pos_item_t pitem;
    chg_item_t citem;
    begin
      @(posedge clk);
      pos_valid    <= 1'b1;
      pos_time     <= tpos;
      pos_data     <= p;
      chg_valid    <= 1'b1;
      chg_time     <= tchg;
      chg_mag      <= m;
      chg_overflow <= of;

      pitem.t = tpos;
      pitem.p = p;
      citem.t  = tchg;
      citem.m  = m;
      citem.of = of;

      model_push_pos(pitem);
      model_push_chg(citem);

      @(posedge clk);
      pos_valid <= 1'b0;
      chg_valid <= 1'b0;
    end
  endtask

  // ============================================================
  // Scoreboard + procedural assertions
  // ============================================================
  initial begin
    out_item_t exp;

    forever begin
      @(posedge clk);
      #1;  // wait for DUT nonblocking assignments to settle

      if (rst_n) begin
        // Advance model once per cycle
        model_step();

        // ----------------------------
        // Procedural assertion checks
        // ----------------------------

        // One-cycle pulse checks
        if (prev_out_valid && out_valid) begin
          $display("[%0t] ERROR: out_valid not one-cycle pulse", $time);
          errors = errors + 1;
        end

        if (prev_drop_pos && drop_pos) begin
          $display("[%0t] ERROR: drop_pos not one-cycle pulse", $time);
          errors = errors + 1;
        end

        if (prev_drop_chg && drop_chg) begin
          $display("[%0t] ERROR: drop_chg not one-cycle pulse", $time);
          errors = errors + 1;
        end

        // Mutual exclusion
        if ((out_valid && drop_pos) ||
            (out_valid && drop_chg) ||
            (drop_pos && drop_chg)) begin
          $display("[%0t] ERROR: illegal flag combination", $time);
          errors = errors + 1;
        end

        // No X on output when valid
        if (out_valid) begin
          if ((^out_time === 1'bx) ||
              (^out_pos  === 1'bx) ||
              (^out_mag  === 1'bx) ||
              (match_overflow === 1'bx)) begin
            $display("[%0t] ERROR: X detected on outputs when out_valid=1", $time);
            errors = errors + 1;
          end
        end

        // ----------------------------
        // Scoreboard compare
        // ----------------------------
        if (out_valid) begin
          if (exp_q.size() == 0) begin
            $display("[%0t] ERROR: DUT asserted out_valid but model expected none", $time);
            errors = errors + 1;
          end
          else begin
            exp = exp_q[0];
            exp_q.delete(0);

            if (out_time !== exp.t) begin
              $display("[%0t] ERROR: out_time exp=%0h got=%0h", $time, exp.t, out_time);
              errors = errors + 1;
            end

            if (out_pos !== exp.p) begin
              $display("[%0t] ERROR: out_pos exp=%0h got=%0h", $time, exp.p, out_pos);
              errors = errors + 1;
            end

            if (out_mag !== exp.m) begin
              $display("[%0t] ERROR: out_mag exp=%0h got=%0h", $time, exp.m, out_mag);
              errors = errors + 1;
            end

            if (match_overflow !== exp.of) begin
              $display("[%0t] ERROR: match_overflow exp=%0b got=%0b", $time, exp.of, match_overflow);
              errors = errors + 1;
            end
          end
        end

        prev_out_valid = out_valid;
        prev_drop_pos  = drop_pos;
        prev_drop_chg  = drop_chg;
      end
      else begin
        prev_out_valid = 0;
        prev_drop_pos  = 0;
        prev_drop_chg  = 0;
      end
    end
  end

  // ============================================================
  // Directed tests
  // ============================================================
  task automatic test_basic_match_same_cycle;
    reg [TS_WIDTH-1:0] t;
    begin
      $display("Running test_basic_match_same_cycle...");
      t = 52'h100;
      drive_both_pulse_same_cycle(t, 32'hAAAA5555, t, 26'h01ABCDE, 1'b0);
      idle_cycles(10);
    end
  endtask

  task automatic test_skewed_match_pos_first;
    reg [TS_WIDTH-1:0] t;
    begin
      $display("Running test_skewed_match_pos_first...");
      t = 52'h200;
      fork
        begin
          drive_pos_pulse(t, 32'h11112222);
        end
        begin
          idle_cycles(3);
          drive_chg_pulse(t, 26'h0001234, 1'b1);
        end
      join
      idle_cycles(15);
    end
  endtask

  task automatic test_skewed_match_chg_first;
    reg [TS_WIDTH-1:0] t;
    begin
      $display("Running test_skewed_match_chg_first...");
      t = 52'h300;
      fork
        begin
          drive_chg_pulse(t, 26'h0007777, 1'b0);
        end
        begin
          idle_cycles(4);
          drive_pos_pulse(t, 32'h33334444);
        end
      join
      idle_cycles(15);
    end
  endtask

  task automatic test_drop_pos_older;
    begin
      $display("Running test_drop_pos_older...");
      drive_both_pulse_same_cycle(52'h0100, 32'hDEADBEEF,
                                  52'h0200, 26'h0001111, 1'b0);
      idle_cycles(10);
    end
  endtask

  task automatic test_drop_chg_older;
    begin
      $display("Running test_drop_chg_older...");
      drive_both_pulse_same_cycle(52'h0300, 32'hCAFEF00D,
                                  52'h0250, 26'h0002222, 1'b0);
      idle_cycles(10);
    end
  endtask

  task automatic test_multiple_ordered_matches;
    begin
      $display("Running test_multiple_ordered_matches...");
      drive_both_pulse_same_cycle(52'h1000, 32'h00010001, 52'h1000, 26'h1, 1'b0);
      drive_both_pulse_same_cycle(52'h1001, 32'h00020002, 52'h1001, 26'h2, 1'b1);
      drive_both_pulse_same_cycle(52'h1002, 32'h00030003, 52'h1002, 26'h3, 1'b0);
      idle_cycles(20);
    end
  endtask

  task automatic test_fifo_full_pressure;
    integer i;
    reg [TS_WIDTH-1:0] t0;
    begin
      $display("Running test_fifo_full_pressure...");
      t0 = 52'h4000;

      for (i = 0; i < DEPTH + 4; i = i + 1)
        drive_pos_pulse(t0 + i, i);

      for (i = 0; i < DEPTH + 4; i = i + 1)
        drive_chg_pulse(t0 + i, i, 1'b0);

      idle_cycles(50);
    end
  endtask

  // ============================================================
  // Final checks
  // ============================================================
  task automatic final_checks;
    begin
      if (exp_q.size() != 0) begin
        $display("ERROR: exp_q not empty at end, remaining=%0d", exp_q.size());
        errors = errors + 1;
      end

      $display(" ");
      $display("---- TEST SUMMARY ----");
      $display("errors            = %0d", errors);
      $display("matches_model     = %0d", matches_model);
      $display("drops_pos_model   = %0d", drops_pos_model);
      $display("drops_chg_model   = %0d", drops_chg_model);
      $display("drop_new_pos_full = %0d", drop_new_pos_full);
      $display("drop_new_chg_full = %0d", drop_new_chg_full);

      if (errors == 0)
        $display("PASS");
      else
        $display("FAIL");
    end
  endtask

  // ============================================================
  // Main
  // ============================================================
  initial begin
    do_reset();

    test_basic_match_same_cycle();
    test_skewed_match_pos_first();
    test_skewed_match_chg_first();
    test_drop_pos_older();
    test_drop_chg_older();
    test_multiple_ordered_matches();
    test_fifo_full_pressure();

    final_checks();
    $finish;
  end

endmodule