`timescale 1ns/1ps

module tb_pattern_gen;

  // --------------------------------------------------------------------------
  // Fast simulation parameters (override DUT defaults)
  // --------------------------------------------------------------------------
  localparam int      DIV_TB       = 4;
  localparam shortint POINT_STEP_TB = 16'sd10;
  localparam int      LEN_INC_TB    = 3;
  localparam shortint BOUND_ABS_TB  = 16'd60;

  logic clk;
  logic rst_n;
  logic enable;

  wire signed [15:0] x;
  wire signed [15:0] y;
  wire               valid;
  wire [31:0]        point_count_out;
  wire [19:0]        rot20_out;

  pattern_gen #(
    .DIV(DIV_TB),
    .POINT_STEP(POINT_STEP_TB),
    .LEN_INC(LEN_INC_TB),
    .BOUND_ABS(BOUND_ABS_TB)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),
    .enable(enable),
    .x(x),
    .y(y),
    .valid(valid),
    .point_count_out(point_count_out),
    .rot20_out(rot20_out)
  );

  // --------------------------------------------------------------------------
  // Clock
  // --------------------------------------------------------------------------
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // --------------------------------------------------------------------------
  // Reference model state (procedural checker)
  // --------------------------------------------------------------------------
  typedef enum logic [1:0] {S_IDLE=2'd0, S_RUN=2'd1, S_DONE=2'd2} st_t;
  st_t st_ref;

  int unsigned div_cnt_ref;

  logic signed [15:0] x_state_ref, y_state_ref;
  logic [1:0]         dir_ref;           // 0=R,1=U,2=L,3=D
  logic [15:0]        step_len_ref;
  logic [15:0]        step_left_ref;
  logic               seg_in_pair_ref;
  logic               step_dither_ref;

  logic [31:0] point_count_ref;
  logic [19:0] rot20_ref;

  logic enable_d_ref;

  function automatic int unsigned abs_int(input int signed v);
    abs_int = (v < 0) ? int'(-v) : int'(v);
  endfunction

  function automatic logic is_onehot20(input logic [19:0] v);
    // true if exactly one bit set
    is_onehot20 = (v != 20'd0) && ((v & (v - 20'd1)) == 20'd0);
  endfunction

  function automatic logic emit_pulse_ref();
    return (enable && (st_ref == S_RUN) && (div_cnt_ref == (DIV_TB-1)));
  endfunction

  // --------------------------------------------------------------------------
  // Simple tasks
  // --------------------------------------------------------------------------
  task automatic do_reset();
    rst_n  = 1'b0;
    enable = 1'b0;
    repeat (5) @(posedge clk);
    rst_n  = 1'b1;
    repeat (2) @(posedge clk);
  endtask

  task automatic start_enable();
    enable = 1'b0;
    repeat (2) @(posedge clk);
    enable = 1'b1;
  endtask

  task automatic stop_enable();
    enable = 1'b0;
  endtask

  // --------------------------------------------------------------------------
  // Reference model update (mirrors DUT logic enough for checking)
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      enable_d_ref     <= 1'b0;
      st_ref           <= S_IDLE;
      div_cnt_ref      <= 0;

      x_state_ref      <= 16'sd0;
      y_state_ref      <= 16'sd0;
      dir_ref          <= 2'd0;
      step_len_ref     <= 16'd1;
      step_left_ref    <= 16'd1;
      seg_in_pair_ref  <= 1'b0;
      step_dither_ref  <= 1'b0;

      point_count_ref  <= 32'd0;
      rot20_ref        <= 20'd1;
    end else begin
      enable_d_ref <= enable;

      // divider model
      if (!enable) begin
        div_cnt_ref <= 0;
      end else if (enable && !enable_d_ref) begin
        div_cnt_ref <= 0;
      end else if (st_ref != S_RUN) begin
        div_cnt_ref <= 0;
      end else if (emit_pulse_ref()) begin
        div_cnt_ref <= 0;
      end else begin
        div_cnt_ref <= div_cnt_ref + 1;
      end

      // state/pattern model
      if (!enable) begin
        st_ref          <= S_IDLE;

        x_state_ref     <= 16'sd0;
        y_state_ref     <= 16'sd0;
        dir_ref         <= 2'd0;
        step_len_ref    <= 16'd1;
        step_left_ref   <= 16'd1;
        seg_in_pair_ref <= 1'b0;
        step_dither_ref <= 1'b0;

        point_count_ref <= 32'd0;
        rot20_ref       <= 20'd1;

      end else if (enable && !enable_d_ref) begin
        st_ref          <= S_RUN;

        x_state_ref     <= 16'sd0;
        y_state_ref     <= 16'sd0;
        dir_ref         <= 2'd0;
        step_len_ref    <= 16'd1;
        step_left_ref   <= 16'd1;
        seg_in_pair_ref <= 1'b0;
        step_dither_ref <= 1'b0;

        point_count_ref <= 32'd0;
        rot20_ref       <= 20'd1;

      end else begin
        case (st_ref)
          S_IDLE: begin end

          S_RUN: begin
            if (emit_pulse_ref()) begin
              logic signed [15:0] step_amt;
              int signed x_next_w, y_next_w;
              logic hit_bound;

              step_amt = (step_dither_ref) ? (POINT_STEP_TB - 16'sd2)
                                           : (POINT_STEP_TB + 16'sd2);

              x_next_w = x_state_ref;
              y_next_w = y_state_ref;
              unique case (dir_ref)
                2'd0: x_next_w = x_state_ref + step_amt;
                2'd1: y_next_w = y_state_ref + step_amt;
                2'd2: x_next_w = x_state_ref - step_amt;
                2'd3: y_next_w = y_state_ref - step_amt;
              endcase

              hit_bound = (abs_int(x_next_w) >= BOUND_ABS_TB) ||
                          (abs_int(y_next_w) >= BOUND_ABS_TB);

              if (hit_bound) begin
                st_ref          <= S_DONE;

                x_state_ref     <= 16'sd0;
                y_state_ref     <= 16'sd0;
                dir_ref         <= 2'd0;
                step_len_ref    <= 16'd1;
                step_left_ref   <= 16'd1;
                seg_in_pair_ref <= 1'b0;
                step_dither_ref <= 1'b0;

                point_count_ref <= 32'd0;
                rot20_ref       <= 20'd1;

              end else begin
                x_state_ref <= x_next_w[15:0];
                y_state_ref <= y_next_w[15:0];

                point_count_ref <= point_count_ref + 32'd1;
                rot20_ref       <= {rot20_ref[18:0], rot20_ref[19]};

                step_dither_ref <= ~step_dither_ref;

                if (step_left_ref == 16'd1) begin
                  dir_ref <= dir_ref + 2'd1;

                  if (seg_in_pair_ref) begin
                    step_len_ref    <= step_len_ref + LEN_INC_TB[15:0];
                    step_left_ref   <= step_len_ref + LEN_INC_TB[15:0];
                    seg_in_pair_ref <= 1'b0;
                  end else begin
                    step_left_ref   <= step_len_ref;
                    seg_in_pair_ref <= 1'b1;
                  end
                end else begin
                  step_left_ref <= step_left_ref - 16'd1;
                end
              end
            end
          end

          S_DONE: begin end
          default: st_ref <= S_IDLE;
        endcase
      end
    end
  end

  // --------------------------------------------------------------------------
  // Procedural assertions (NO "property" keyword anywhere)
  // --------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst_n) begin
      // A) When valid is low, your DUT forces all outputs low every cycle
      if (!valid) begin
        assert (x == 16'sd0 && y == 16'sd0 &&
                point_count_out == 32'd0 && rot20_out == 20'd0)
          else $fatal(1, "Outputs not forced to 0 when valid=0");
      end

      // B) rot20_out must be onehot when valid=1
      if (valid) begin
        assert (is_onehot20(rot20_out))
          else $fatal(1, "rot20_out not onehot when valid=1 (rot20_out=%h)", rot20_out);
      end

      // C) On expected emits, check exact behavior
      if (emit_pulse_ref() && (st_ref == S_RUN)) begin
        logic signed [15:0] step_amt;
        int signed x_next_w, y_next_w;
        logic hit_bound;

        step_amt = (step_dither_ref) ? (POINT_STEP_TB - 16'sd2)
                                     : (POINT_STEP_TB + 16'sd2);

        x_next_w = x_state_ref;
        y_next_w = y_state_ref;
        unique case (dir_ref)
          2'd0: x_next_w = x_state_ref + step_amt;
          2'd1: y_next_w = y_state_ref + step_amt;
          2'd2: x_next_w = x_state_ref - step_amt;
          2'd3: y_next_w = y_state_ref - step_amt;
        endcase

        hit_bound = (abs_int(x_next_w) >= BOUND_ABS_TB) ||
                    (abs_int(y_next_w) >= BOUND_ABS_TB);

        if (hit_bound) begin
          // boundary hit => DUT should NOT output a point
          assert (!valid)
            else $fatal(1, "Expected valid=0 on bound hit, got valid=1");
        end else begin
          // normal emit => DUT must pulse valid and match x/y/count/rot
          assert (valid) else $fatal(1, "Expected valid=1 on emit, got valid=0");

          assert (x == x_next_w[15:0] && y == y_next_w[15:0])
            else $fatal(1, "XY mismatch. DUT=(%0d,%0d) EXP=(%0d,%0d)",
                        x, y, x_next_w[15:0], y_next_w[15:0]);

          assert (point_count_out == (point_count_ref + 32'd1))
            else $fatal(1, "point_count_out mismatch. DUT=%0d EXP=%0d",
                        point_count_out, point_count_ref + 32'd1);

          assert (rot20_out == rot20_ref)
            else $fatal(1, "rot20_out mismatch. DUT=%h EXP=%h",
                        rot20_out, rot20_ref);
        end
      end

      // D) In DONE, must stay quiet while enable remains high
      if ((st_ref == S_DONE) && enable) begin
        assert (!valid)
          else $fatal(1, "Expected valid=0 in DONE while enable=1");
      end
    end
  end

  // --------------------------------------------------------------------------
  // Test sequence
  // --------------------------------------------------------------------------
  initial begin
    rst_n  = 1'b0;
    enable = 1'b0;

    do_reset();

    // Run until bound hit happens (fast params ensure it happens)
    start_enable();
    repeat (400) @(posedge clk);

    // Restart behavior test
    stop_enable();
    repeat (10) @(posedge clk);
    start_enable();
    repeat (80) @(posedge clk);

    $display("TB completed with no assertion failures.");
    $finish;
  end

endmodule