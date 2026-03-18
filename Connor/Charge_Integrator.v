///////////////////////////////////////////////////////////////////////////////////////////////////
// Company: SwRI/VT
//
// File: Charge_Integrator_V1.v
// File history:
//      <Revision number>: <Date>: <Comments>
//      <Revision number>: <Date>: <Comments>
//      <Revision number>: <Date>: <Comments>
//
// Description:
//
// <Description here>
//
// Targeted device: <Family::ProASIC3E> <Die::A3PE1500> <Package::208 PQFP>
// Author: VT MDE S26-23
//
///////////////////////////////////////////////////////////////////////////////////////////////////

`default_nettype none

module Charge_Integrator_V1 #(
    parameter integer ADC_WIDTH = 16,
    parameter integer TS_WIDTH  = 64,
    parameter integer SUM_WIDTH = 26
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 in_valid,
    input  wire [ADC_WIDTH-1:0] adc0,
    input  wire [ADC_WIDTH-1:0] adc1,
    input  wire [ADC_WIDTH-1:0] adc2,
    input  wire [ADC_WIDTH-1:0] adc3,
    input  wire [TS_WIDTH-1:0]  in_timestamp,

    output reg                  out_valid,
    output reg  [SUM_WIDTH-1:0] out_sum,
    output reg  [TS_WIDTH-1:0]  out_timestamp,
    output reg                  out_overflow
);

    // Length required for sum of 4 ADC samples
    localparam integer ADC_SUM_WIDTH = ADC_WIDTH + 2;

    // Input registers
    reg                  in_valid_r;
    reg [ADC_WIDTH-1:0]  adc0_r;
    reg [ADC_WIDTH-1:0]  adc1_r;
    reg [ADC_WIDTH-1:0]  adc2_r;
    reg [ADC_WIDTH-1:0]  adc3_r;
    reg [TS_WIDTH-1:0]   in_timestamp_r;

    // Registered-domain signals
    wire [ADC_SUM_WIDTH-1:0] adc_sum;
    wire [SUM_WIDTH:0]       adc_sum_ext;

    reg                      prev_valid;
    reg  [SUM_WIDTH-1:0]     cur_sum;
    reg                      cur_overflow;
    reg  [TS_WIDTH-1:0]      registered_ts;

    wire [SUM_WIDTH:0]       cur_sum_plus_adc;
    wire                     cur_carry;
    wire [SUM_WIDTH-1:0]     cur_sum_next;

    // Register all inputs on intake
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_valid_r      <= 1'b0;
            adc0_r          <= {ADC_WIDTH{1'b0}};
            adc1_r          <= {ADC_WIDTH{1'b0}};
            adc2_r          <= {ADC_WIDTH{1'b0}};
            adc3_r          <= {ADC_WIDTH{1'b0}};
            in_timestamp_r  <= {TS_WIDTH{1'b0}};
        end
        else begin
            in_valid_r      <= in_valid;
            adc0_r          <= adc0;
            adc1_r          <= adc1;
            adc2_r          <= adc2;
            adc3_r          <= adc3;
            in_timestamp_r  <= in_timestamp;
        end
    end

    // Sum ADC samples
    assign adc_sum =
        {{(ADC_SUM_WIDTH-ADC_WIDTH){1'b0}}, adc0_r} +
        {{(ADC_SUM_WIDTH-ADC_WIDTH){1'b0}}, adc1_r} +
        {{(ADC_SUM_WIDTH-ADC_WIDTH){1'b0}}, adc2_r} +
        {{(ADC_SUM_WIDTH-ADC_WIDTH){1'b0}}, adc3_r};

    // Zero extend adc_sum to SUM_WIDTH+1 bits
    // Assumes (SUM_WIDTH+1) >= ADC_SUM_WIDTH
    assign adc_sum_ext = { {(SUM_WIDTH+1-ADC_SUM_WIDTH){1'b0}}, adc_sum };

    // Next sum value
    assign cur_sum_plus_adc = {1'b0, cur_sum} + adc_sum_ext;
    assign cur_carry        = cur_sum_plus_adc[SUM_WIDTH];
    assign cur_sum_next     = cur_sum_plus_adc[SUM_WIDTH-1:0];

    // Main state machine
    always @(posedge clk or negedge rst_n) begin // b1
        if (!rst_n) begin // b2
            // Reset values
            prev_valid     <= 1'b0;
            cur_sum        <= {SUM_WIDTH{1'b0}};
            cur_overflow   <= 1'b0;
            registered_ts  <= {TS_WIDTH{1'b0}};

            out_valid      <= 1'b0;
            out_sum        <= {SUM_WIDTH{1'b0}};
            out_timestamp  <= {TS_WIDTH{1'b0}};
            out_overflow   <= 1'b0;
        end // e2
        else begin // b3
            if (in_valid_r) begin // b4
                if (!prev_valid) begin // b5
                    // Start of burst
                    prev_valid     <= 1'b1;
                    registered_ts  <= in_timestamp_r;
                    cur_sum        <= adc_sum_ext[SUM_WIDTH-1:0];
                    cur_overflow   <= 1'b0; // overflow can only occur on accumulation adds

                    // Outputs held low during accumulation
                    out_valid      <= 1'b0;
                    out_sum        <= {SUM_WIDTH{1'b0}};
                    out_timestamp  <= {TS_WIDTH{1'b0}};
                    out_overflow   <= 1'b0;
                end // e5
                else begin // b6
                    // Middle of burst
                    prev_valid     <= 1'b1;
                    registered_ts  <= registered_ts;             // hold
                    cur_sum        <= cur_sum_next;
                    cur_overflow   <= cur_overflow | cur_carry;

                    // Outputs held low during accumulation
                    out_valid      <= 1'b0;
                    out_sum        <= {SUM_WIDTH{1'b0}};
                    out_timestamp  <= {TS_WIDTH{1'b0}};
                    out_overflow   <= 1'b0;
                end // e6
            end // e4
            else begin // b7
                // in_valid_r == 0
                if (prev_valid) begin // b8
                    // End of burst: pulse outputs for one cycle, then clear internal state
                    prev_valid     <= 1'b0;

                    out_valid      <= 1'b1;
                    out_sum        <= cur_sum;
                    out_timestamp  <= registered_ts;
                    out_overflow   <= cur_overflow;

                    cur_sum        <= {SUM_WIDTH{1'b0}};
                    cur_overflow   <= 1'b0;
                    registered_ts  <= {TS_WIDTH{1'b0}};        
                end // e8
                else begin // b9
                    // Idle: keep everything low/cleared
                    prev_valid     <= 1'b0;

                    cur_sum        <= {SUM_WIDTH{1'b0}};
                    cur_overflow   <= 1'b0;
                    registered_ts  <= {TS_WIDTH{1'b0}};

                    out_valid      <= 1'b0;
                    out_sum        <= {SUM_WIDTH{1'b0}};
                    out_timestamp  <= {TS_WIDTH{1'b0}};
                    out_overflow   <= 1'b0;
                end // e9
            end // e7
        end // e3
    end // e1

endmodule

`default_nettype wire
