`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:37:00 11/06/2012 
// Design Name: 
// Module Name:    Viterbyencoder 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module viterbi_encode(X2N,X1N,Y2N,Y1N,Y0N,clk,res);
input X2N,X1N,clk,res; output Y2N, Y1N, Y0N;

wire X1N_1, X1N_2, Y2N,Y1N,Y0N;
dff dff_1(X1N,X1N_1,clk, res); dff dff_2(X1N_1,X1N_2,clk,res);

assign Y2N=X2N; assign Y1N=X1N ^X1N_2; assign Y0N=X1N_1;
endmodule


/*
module dff (
data   , 
q		 ,
clk    , 
reset   
);
input data, clk, reset ; 

output q;

reg q;

always @ ( posedge clk)
if (~reset) begin
  q <= 1'b0;
end  else begin
  q <= data;
end

endmodule 
*/
module viterbi_distances
(Y2N, Y1N, Y0N, clk, res, in0, in1, in2, in3, in4, in5, in6, in7);
input clk, res, Y2N, Y1N, Y0N; output in0, in1, in2, in3, in4, in5, in6, in7;
reg [2:0] J, in0, in1, in2, in3, in4, in5 ,in6, in7; 
reg [2:0] d[7:0];

initial begin d[0] = 'b000; d[1]= 'b001; d[2]='b100; d[3]='b110;  d[4]='b111; d[5]='b110; d[6]='b100; d[7]='b001;
end


always @ ( Y2N or Y1N or Y0N) begin
J[0]= Y0N;J[1]= Y1N;J[2]= Y2N;
J=8-J; in0=d[J]; J=J+1; in1=d[J]; J=J+1; in2=d[J];J=J+1; in3=d[J];
in4=d[J]; in5=d[J]; in6=d[J]; in7=d[J];
end endmodule 

//asDff #(3) subout0(in0, sub0, clk, reset);
//myDff subout0[0:2] (in0, sub0, clk, reset);

//A dflipflop module
module dff (D,Q,Clock,Reset);
output Q; input D,Clock, Reset;
parameter CARDINALITY = 1; reg [CARDINALITY-1:0] Q;
wire [CARDINALITY-1:0] D;
always @(posedge Clock) if (Reset!== 0) #1 Q = D;
always 
	begin 
		wait (Reset == 0); Q = 0; wait (Reset == 1);
	end
endmodule 

module viterbi
(in0,in1,in2,in3,in4,in5,in6,in7,
	out,clk,reset,error);
	input[2:0] in0,in1,in2,in3,in4,in5,in6,in7;
	output [2:0] out;input clk,reset; output error;
	wire sout0,sout1,sout2,sout3;
	wire [2:0] s0,s1,s2,s3;
	wire [4:0] m_in0,m_in1,m_in2,m_in3;
	wire [4:0] m_out0,m_out1,m_out2,m_out3;
	wire [4:0] p0_0,p2_0,p0_1,p2_1,p1_2,p3_2,p1_3,p3_3;
	wire ACS0,ACS1,ACS2,ACS3;
	wire [4:0] out0,out1,out2,out3;
	wire [1:0] control;
	wire [2:0] p0,p1,p2,p3;
	wire [11:0] path0;
	
	subset_decode u1(in0,in1,in2,in3,in4,in5,in6,in7,s0,s1,s2,s3,sout0,sout1,sout2,sout3,clk,reset);
	metric u2(m_in0,m_in1,m_in2,m_in3,m_out0,m_out1,m_out2,m_out3,clk,reset);
	compute_metric u3(m_out0,m_out1,m_out2,m_out3,s0,s1,s2,s3,p0_0,p2_0,p2_1,p1_2,p3_2,p1_3,p3_3,error);
	compare_select u4(p0_0,p2_0,p2_1,p1_2,p3_2,p1_3,p3_3, out0,out1,out2,out3,ACS0,ACS1,ACS2,ACS3);
	reduce u5 (out0,out1,out2,out3,m_in0,m_in1,m_in2,m_in3,control);
	pathin u6(sout0,sout1,sout2,sout3,ACS0,ACS1,ACS2,ACS3,path0,clk,reset);
	path_memory u7(p0,p1,p2,p3,path0,clk,reset,ACS0,ACS1,ACS2,ACS3);
	output_decision u8(p0,p1,p2,p3,control,out);
	
endmodule

/* Module subset_decode, this module chooses the signal corresponding
to the smallest of each set, therefore there are eight input signals
and four output signals for the distance measures. The signal sout 0.... sout3, are used
to control the path memory. The statement of dff #3 instantiaties a vector of 3
d Flip flops */ 

module subset_decode
	(in0,in1,in2,in3,in4,in5,in6,in7,
	s0,s1,s2,s3,
	sout0,sout1,sout2,sout3,
	clk, reset);
input [2:0] in0,in1,in2,in3,in4,in5,in6,in7;
output[2:0] s0,s1,s2,s3;
output sout0,sout1,sout2,sout3;
input clk,reset;
wire[2:0] sub0,sub1,sub2,sub3,sub4,sub5,sub6,sub7;
	dff #(3) subout0(in0, sub0, clk, reset);
	dff #(3) subout1(in0, sub0, clk, reset);
	dff #(3) subout2(in0, sub0, clk, reset);
	dff #(3) subout3(in0, sub0, clk, reset);
	dff #(3) subout4(in0, sub0, clk, reset);
	dff #(3) subout5(in0, sub0, clk, reset);
	dff #(3) subout6(in0, sub0, clk, reset);
	dff #(3) subout7(in0, sub0, clk, reset);
function [2:0] subset_decode; input [2:0] a,b;
	begin
		subset_decode=0;
		if (a<=b) subset_decode=a; else subset_decode = b;
	end
endfunction
function set_control; input[2:0] a,b;
	begin
		if (a<=b) set_control=0; else set_control = 1;
	end
endfunction

assign s0 = subset_decode (sub0,sub4);
assign s1 = subset_decode (sub1,sub5);
assign s2 = subset_decode (sub2,sub6);
assign s3 = subset_decode (sub3,sub7);
assign sout0 = set_control(sub0,sub4);
assign sout1 = set_control(sub1,sub5);
assign sout2 = set_control(sub2,sub6);
assign sout3 = set_control(sub3,sub7);
endmodule


/* Module compute_metric. This module computes the sum of path
memory and the distance for each path entering a state of the trellis.
For the four states, there are two paths entering it; therefore eight sums are computed
in this module. The path metrics and output sums are 5 bits wide.
The output sum is bounded and should never be greater than 5 bits
for a valid input signal. The overflow from the sum is the error
output and indicates an invalid input signal. */

module compute_metric
	(m_out0,m_out1,m_out2,m_out3,
	s0,s1,s2,s3,p0_0,p2_0,
	p0_1,p2_1,p1_2,p3_2,p1_3,p3_3,
	error);
input [4:0] m_out0,m_out1,m_out2,m_out3;
input [2:0] s0,s1,s2,s3;
output [4:0] p0_0,p2_0,p0_1,p2_1,p1_2,p3_2,p1_3,p3_3;
output error;
assign
	p0_0 = m_out0 +s0,
	p2_0 = m_out2 +s2,
	p0_1 = m_out0 +s2,
	p2_1 = m_out2 +s0,
	p1_2 = m_out1 +s1,
	p3_2 = m_out3 +s3,
	p1_3 = m_out1 +s3,
	p3_3 = m_out3 +s1;
function is_error; input x1,x2,x3,x4,x5,x6,x7,x8;
begin
	if (x1||x2||x3||x4||x5||x6||x7||x8) is_error = 1;
	else is_error = 0;
end
endfunction
assign error= is_error(p0_0[4],p2_0[4],p0_1[4],p2_1[4],
	p1_2[4],p3_2[4],p1_3[4],p3_3[4]);
endmodule

/* This module compare the summations from the compute_metric module
and selects the metric and path with teh lowest value. The
output of this module is saved as the new path metric for each state.
The ACS output signals are used to control the path memory of the decoder.
*/

module compare_select
	(p0_0,p2_0,p0_1,p2_1,p1_2,p3_2,p1_3,p3_3,
	out0,out1,out2,out3,
	ACS0,ACS1,ACS2,ACS3);
input [4:0] p0_0,p2_0,p0_1,p2_1,p1_2,p3_2,p1_3,p3_3;
output [4:0] out0,out1,out2,out3;
output ACS0,ACS1,ACS2,ACS3;
function [4:0] find_min_metric; input [4:0] a,b;
	begin
		if (a <= b) find_min_metric = a; else find_min_metric = b;
	end
endfunction
function set_control; input[4:0] a,b;
	begin
		if(a<=b) set_control=0; else set_control=1;
	end
endfunction
assign out0 = find_min_metric(p0_0,p2_0);
assign out1 = find_min_metric(p0_1,p2_1);
assign out2 = find_min_metric(p1_2,p3_2);
assign out3 = find_min_metric(p1_3,p3_3);
assign ACS0 = set_control (p0_0,p2_0);
assign ACS1 = set_control (p0_1,p2_1);
assign ACS2 = set_control (p1_2,p3_2);
assign ACS3 = set_control (p1_3,p3_3);
endmodule

/* Module path.. This is the basic unit for the path memory of the Viterbi decoder. It consists
of four 3.bit D flip flops in parallel. There is a 2:1 mux at each D flip flop input. The statement
dff #(12) instantiates a vector array of 12 flip flops.*/

module path (in,out,clk,reset,ACS0,ACS1,ACS2,ACS3);
input [11:0] in; output [11:0] out;
input clk,reset,ACS0,ACS1,ACS2,ACS3; wire[11:0] p_in;
dff#(12) path0(p_in,out,clk,reset);
	function [2:0] shift_path; input[2:0] a,b; input control;
		begin
			if (control==0) shift_path =a; else shift_path=b;
		end
	endfunction
assign p_in[11:9] = shift_path(in[11:9], in[5:3], ACS0);
assign p_in[8:6] = shift_path(in[11:9], in[5:3], ACS1);
assign p_in[5:3] = shift_path(in[8:6], in[2:0], ACS2);
assign p_in[2:0] = shift_path(in[8:6], in[2:0], ACS3);
endmodule

/* Path_memory. This module consists of an array of memory elements( D
Flip flops) that store and shift the path memory as new signals are
added to the four paths (or four most likely sequences of signals).
This module instatiates 11 instances of the path module.*/

module path_memory
	(p0,p1,p2,p3,
	path0,clk,reset,
	ACS0,ACS1,ACS2,ACS3);
output [2:0] p0,p1,p2,p3; input [11:0] path0;
input clk, reset, ACS0,ACS1,ACS2,ACS3;
wire [11:0] out1,out2,out3,out4,out5,out6,out7,out8,out9,out10,out11;
	path x1 (path0, out1,clk,reset,ACS0,ACS1,ACS2,ACS3),
	x2(out1,out2,clk,reset,ACS0,ACS1,ACS2,ACS3),
	x3(out2,out3,clk,reset,ACS0,ACS1,ACS2,ACS3),	
	x4(out3,out4,clk,reset,ACS0,ACS1,ACS2,ACS3),
	x5(out4,out5,clk,reset,ACS0,ACS1,ACS2,ACS3),
	x6(out5,out6,clk,reset,ACS0,ACS1,ACS2,ACS3),
	x7(out6,out7,clk,reset,ACS0,ACS1,ACS2,ACS3),
	x8(out7,out8,clk,reset,ACS0,ACS1,ACS2,ACS3),
	x9(out8,out9,clk,reset,ACS0,ACS1,ACS2,ACS3),
	x10(out9,out10,clk,reset,ACS0,ACS1,ACS2,ACS3),	
	x11(out10,out11,clk,reset,ACS0,ACS1,ACS2,ACS3);
assign p0=out11[11:9];
assign p1=out11[8:6];
assign p2=out11[5:3];
assign p3=out11[2:0];
endmodule

/* Pathin. This module determines the input signal to the path for each of
the four paths. Control signals from the subset decoder and compare
select modules are used to store the correct signal. The statement
dff#(12) instantiates a vector array of 12 flip flops.*/

module pathin
	(sout0,sout1,sout2,sout3,
	ACS0,ACS1,ACS2,ACS3,
	path0,clk,reset);
input sout0,sout1,sout2,sout3,ACS0,ACS1,ACS2,ACS3;
input clk,reset; output [11:0] path0;
wire[2:0] sig0,sig1,sig2,sig3; wire [11:0] path_in;
dff #(12) firstpath(path_in,path0,clk,reset);
function[2:0] subset0; input sout0;
	begin
		if(sout0==0) subset0=0; else subset0=4;
	end
endfunction
function[2:0] subset1; input sout1;
	begin
		if(sout1==0) subset1=1; else subset1=5;
	end
endfunction
function[2:0] subset2; input sout2;
	begin
		if(sout2==0) subset2=2; else subset2=6;
	end
endfunction
function[2:0] subset3; input sout3;
	begin
		if(sout3==0) subset3=3; else subset3=7;
	end
endfunction
function [2:0] find_path; input[2:0] a,b; input control;
	begin
		if(control==0) find_path = a; else find_path=b;
	end
endfunction
assign sig0=subset0(sout0);
assign sig1=subset1(sout1);
assign sig2=subset2(sout2);
assign sig3=subset3(sout3);
assign path_in [11:9] = find_path(sig0,sig2,ACS0);
assign path_in [8:6] = find_path(sig2,sig0,ACS1);
assign path_in [5:3] = find_path(sig1,sig3,ACS2);
assign path_in [2:0] = find_path(sig3,sig1,ACS3);
endmodule

/*metric. The registers created in this module (using d flip flops) store
the four path metrics. Each register is 5 bits wide. The statement dff#(5)
instantiates a vector aray of 5 flipflops*/

module metric
	(m_in0,m_in1,m_in2,m_in3,
	m_out0,m_out1,m_out2,m_out3,
	clk,reset);
input [4:0] m_in0,m_in1,m_in2,m_in3;
output[4:0] m_out0,m_out1,m_out2,m_out3;
input clk,reset;
	dff #(5) metric3(m_in3,m_out3,clk,reset);
	dff #(5) metric2(m_in2,m_out2,clk,reset);
	dff #(5) metric1(m_in1,m_out1,clk,reset);
	dff #(5) metric0(m_in0,m_out0,clk,reset);
endmodule

/*output_decision. This module decides the output signal based on the path
that correpsonds to the smallest metric. The control signal comes from the
reduce module.*/

module output_decision(p0,p1,p2,p3,control,out);
input [2:0] p0,p1,p2,p3; input [1:0] control; output[2:0] out;
function[2:0] decide;
input [2:0] p0,p1,p2,p3; input[1:0] control;
begin
	if(control==0) decide = p0;
	else if (control==1) decide=p1;
	else if (control==2) decide=p2;
	else decide=p3;
	end
endfunction
assign out=decide(p0,p1,p2,p3,control);
endmodule

/*Reduce. This module reduces the metrics after the addition and
compare operations. This algorithm selects the smallest metric
and subtracts if from all the other metrics.*/
module reduce
	(in0,in1,in2,in3,
	m_in0,m_in1,m_in2,m_in3,
	control);
input [4:0] in0,in1,in2,in3;
output [4:0] m_in0,m_in1,m_in2,m_in3;
output [1:0] control; wire [4:0] smallest;
function [4:0] find_smallest;
	input [4:0] in0,in1,in2,in3; reg [4:0] a,b;
	begin
		if (in0<=in1) a=in0; else a=in1;
		if (in2<=in3) a=in2; else a=in3;
		if (a<=b) find_smallest=a;
		else find_smallest=b;
	end
endfunction
function [1:0] smallest_no;
input[4:0] in0,in1,in2,in3, smallest;
	begin	
		if(smallest==in0) smallest_no=0;
		else if (smallest==in1) smallest_no=1;
		else if (smallest==in2) smallest_no=2;
		else smallest_no=3;
	end
endfunction
assign smallest =find_smallest(in0,in1,in2,in3);
assign m_in0=in0-smallest;
assign m_in1=in1-smallest;
assign m_in2=in2-smallest;
assign m_in3=in3-smallest;
assign control=smallest_no(in0,in1,in2,in3,smallest);
endmodule

