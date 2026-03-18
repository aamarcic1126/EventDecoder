///////////////////////////////////////////////////////////////////////////////////////////////////
// Company: <Name>
//
// File: pipeline_reg.v
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
// Author: <Name>
//
/////////////////////////////////////////////////////////////////////////////////////////////////// 

//`timescale <time_units> / <precision>

module pipeline_reg #(
    parameter integer WIDTH = 1
)(
    input  wire             clk,
    input  wire             rst_n,   // async active-low reset
    input  wire [WIDTH-1:0] din,
    output reg  [WIDTH-1:0] dout
);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dout <= {WIDTH{1'b0}};
        else
            dout <= din;
    end
endmodule

