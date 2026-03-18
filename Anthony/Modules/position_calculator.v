///////////////////////////////////////////////////////////////////////////////////////////////////
// Company: SwRI/VT
//
// File: Position_Calculator.v
// File history:
//      <Revision number>: <Date>: <Comments>
//      <Revision number>: <Date>: <Comments>
//      <Revision number>: <Date>: <Comments>
//
// Description: 
//
// Function:
//      position_x = (tpx1 - tpx2) * posconst_x
//      position_y = (tpy1 - tpy2) * posconst_y
//
// Notes:
//      - tpx*, tpy* are treated as unsigned 16-bit pulse times, but their difference is signed (17-bit).
//      - posconst_x/y are signed 16-bit.
//      - internal product is signed 33-bit.
//      - output position is signed 32-bit = product[31:0].
//      - overflow flag asserts if product does not fit in signed 32-bit after truncation.
//
// Targeted device: <Family::ProASIC3E> <Die::A3PE1500> <Package::208 PQFP>
// Author: VT MDE S26-23
//
/////////////////////////////////////////////////////////////////////////////////////////////////// 

//`timescale <time_units> / <precision>

`default_nettype none

module Position_Calculator #(
    parameter integer MULT_LATENCY = 3   // default multiplier latency = 3
)(
    input  wire                clk,
    input  wire                rst_n,         // async active-low reset
    input  wire                valid_in,

    input  wire [15:0]         tpx1_in,
    input  wire [15:0]         tpx2_in,
    input  wire [15:0]         tpy1_in,
    input  wire [15:0]         tpy2_in,

    input  wire signed [15:0]  posconst_x_in,
    input  wire signed [15:0]  posconst_y_in,

    input  wire [51:0]         timestamp_in,

    output reg  signed [31:0]  position_x_out,
    output reg  signed [31:0]  position_y_out,
    output reg  [51:0]         timestamp_out,
    output reg                 valid_out,
    output reg                 overflow_out
);

    // register inputs
    reg                 v0;
    reg [51:0]          ts0;

    reg [15:0]          tpx1_0, tpx2_0, tpy1_0, tpy2_0;
    reg signed [15:0]   pcx0, pcy0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v0      <= 1'b0;
            ts0     <= 52'd0;
            tpx1_0  <= 16'd0;  tpx2_0 <= 16'd0;
            tpy1_0  <= 16'd0;  tpy2_0 <= 16'd0;
            pcx0    <= 16'sd0;
            pcy0    <= 16'sd0;
        end else begin
            v0      <= valid_in;
            ts0     <= timestamp_in;

            tpx1_0  <= tpx1_in;  tpx2_0 <= tpx2_in;
            tpy1_0  <= tpy1_in;  tpy2_0 <= tpy2_in;

            pcx0    <= posconst_x_in;
            pcy0    <= posconst_y_in;
        end
    end


    // compute and register diffs
    wire signed [16:0] diff_x_next;
    wire signed [16:0] diff_y_next;

    assign diff_x_next = $signed({1'b0, tpx1_0}) - $signed({1'b0, tpx2_0});
    assign diff_y_next = $signed({1'b0, tpy1_0}) - $signed({1'b0, tpy2_0});

    reg signed [16:0]   diff_x_1, diff_y_1;
    reg signed [15:0]   pcx1, pcy1;
    reg                 v1;
    reg [51:0]          ts1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            diff_x_1 <= 17'sd0;
            diff_y_1 <= 17'sd0;
            pcx1     <= 16'sd0;
            pcy1     <= 16'sd0;
            v1       <= 1'b0;
            ts1      <= 52'd0;
        end else begin
            diff_x_1 <= diff_x_next;
            diff_y_1 <= diff_y_next;
            pcx1     <= pcx0;
            pcy1     <= pcy0;
            v1       <= v0;
            ts1      <= ts0;
        end
    end


    // Multipliers
    wire signed [32:0] prod_x;
    wire signed [32:0] prod_y;

    mult_17_16_3 mult_x (
        .Clock (clk),
        .DataA   (diff_x_1),
        .DataB   (pcx1),
        .Mult   (prod_x)
    );

    mult_17_16_3 mult_y (
        .Clock (clk),
        .DataA   (diff_y_1),
        .DataB   (pcy1),
        .Mult   (prod_y)
    );

    // Align valid/timestamp with multiplier output using register chain
    wire [MULT_LATENCY:0] v_stage;
    assign v_stage[0] = v1;

    wire [52*(MULT_LATENCY+1)-1:0] ts_stage;
    assign ts_stage[51:0] = ts1;

    genvar g;
    generate
        // If MULT_LATENCY==0, then loop generates zero instances. That is fine.
        for (g = 0; g < MULT_LATENCY; g = g + 1) begin : gen_ctrl_pipe
            pipeline_reg #(.WIDTH(1)) u_vreg (
                .clk (clk),
                .rst_n(rst_n),
                .din (v_stage[g]),
                .dout(v_stage[g+1])
            );

            pipeline_reg #(.WIDTH(52)) u_tsreg (
                .clk (clk),
                .rst_n(rst_n),
                .din (ts_stage[52*(g+1)-1 : 52*g]),
                .dout(ts_stage[52*(g+2)-1 : 52*(g+1)])
            );
        end
    endgenerate

    wire        v_aligned;
    wire [51:0] ts_aligned;

    assign v_aligned  = v_stage[MULT_LATENCY];
    assign ts_aligned = ts_stage[52*(MULT_LATENCY+1)-1 : 52*MULT_LATENCY];

    // Overflow detection for signed 33-bit -> signed 32-bit truncation
    wire ovf_x;
    wire ovf_y;
    assign ovf_x = (prod_x[32] ^ prod_x[31]);
    assign ovf_y = (prod_y[32] ^ prod_y[31]);

    wire signed [31:0] px_aligned;
    wire signed [31:0] py_aligned;
    wire               ovf_aligned;

    assign px_aligned  = prod_x[31:0];
    assign py_aligned  = prod_y[31:0];
    assign ovf_aligned = v_aligned & (ovf_x | ovf_y);

    // 1-cycle of output registers for timing

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            position_x_out <= 32'sd0;
            position_y_out <= 32'sd0;
            timestamp_out  <= 52'd0;
            valid_out      <= 1'b0;
            overflow_out   <= 1'b0;
        end else begin
            position_x_out <= px_aligned;
            position_y_out <= py_aligned;
            timestamp_out  <= ts_aligned;
            valid_out      <= v_aligned;
            overflow_out   <= ovf_aligned;
        end
    end

endmodule

`default_nettype wire
