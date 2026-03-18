`timescale 1ns/1ps

module mult_tb;

   localparam N = 5;
   localparam EDGE_CASE_TRIALS = 50000;
   localparam TRIALS = 500000;
   localparam MAX_VAL = (1 << N) - 1;

   logic clk;
   logic reset_n;
   logic start;
   logic [N-1:0] a;
   logic [N-1:0] b;
   logic [N*2-1:0] product;
   logic busy;

   mult #(.N(N)) mult0(.clk(clk),
                       .reset_n(reset_n),
                       .start(start),
                       .a(a),
                       .b(b),
                       .product(product),
                       .busy(busy)
                      );
	
   // TODO

   int test_count = 0;

   initial begin
      clk = 0;
      forever begin
         #50ns clk = ~clk;
      end
   end

   initial begin
      reset_n = 1;
      a = 0;
      b = 0;
      start = 0;
      #10ns;
      reset_n = 0;
      #10ns;
      reset_n = 1;
   end

   initial begin
      #100ns;
      if (N <= 8 && N >= 0) begin 
         $display("Exhaustive Test");
         for (int i = 0; i <= MAX_VAL; i++) begin
            for (int j = 0; j <= MAX_VAL; j++) begin
               a = i;
               b = j;
               start = 1;
               #100ns;
               start = 0;
               @(negedge busy);
               #25ns;
               test_count++;
               assert(product == a * b);
            end
         end
      end
      else if (N >= 9 && N <= 32) begin
         $display("Begin Case 0");
         for (int i = 0; i < EDGE_CASE_TRIALS; i++) begin
            a = 0;
            b = N'($urandom());
            start = 1;
            #100ns;
            start = 0;
            @(negedge busy);
            #25ns;
            test_count++;
            assert(product == 0);
         end
         $display("Begin Case 1");
         for (int i = 0; i < EDGE_CASE_TRIALS; i++) begin
            a = 1;
            b = N'($urandom());
            start = 1;
            #100ns;
            start = 0;
            @(negedge busy);
            #25ns;
            test_count++;
            assert(product == b);
         end
         $display("Begin Case MAX_VAL");
         for (int i = 0; i < EDGE_CASE_TRIALS; i++) begin
            a = MAX_VAL;
            b = N'($urandom());
            start = 1;
            #100ns;
            start = 0;
            @(negedge busy);
            #25ns;
            test_count++;
            assert(product == MAX_VAL * b);
         end
         $display("Begin Random Value Tests");
         for (int i = 0; i < TRIALS; i++) begin
            a = N'($urandom());
            b = N'($urandom());
            start = 1;
            #100ns;
            start = 0;
            @(negedge busy);
            #25ns;
            test_count++;
            assert(product == a * b);
         end
      end
      $display("Test Case Count: %0d", test_count);
      $stop;
   end

endmodule
