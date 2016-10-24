module skeleton(inclock, resetn, /*ps2_clock, ps2_data,*/ debug_word, debug_addr, leds, 
					lcd_data, lcd_rw, lcd_en, lcd_rs, lcd_on, lcd_blon, 	
					seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8);

	input 			inclock, resetn;
	//inout 			ps2_data, ps2_clock;
	
	output 			lcd_rw, lcd_en, lcd_rs, lcd_on, lcd_blon;
	output 	[7:0] 	leds, lcd_data;
	output 	[6:0] 	seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8;
	output 	[31:0] 	debug_word;
	output  [11:0]  debug_addr;
	
	wire			clock;
	wire			lcd_write_en;
	wire 	[31:0]	lcd_write_data;
	wire	[7:0]	ps2_key_data;
	wire			ps2_key_pressed;
	wire	[7:0]	ps2_out;	

	
	// clock divider (by 5, i.e., 10 MHz)
	pll div(inclock,clock);
	
	// UNCOMMENT FOLLOWING LINE AND COMMENT ABOVE LINE TO RUN AT 50 MHz
	//assign clock = inclock;
	
	// your processor
	processor myprocessor(clock, ~resetn, ps2_key_pressed, ps2_out, lcd_write_en, lcd_write_data, debug_word, debug_addr);
	
	// keyboard controller
	//PS2_Interface myps2(clock, resetn, ps2_clock, ps2_data, ps2_key_data, ps2_key_pressed, ps2_out);
	
	// lcd controller
	lcd mylcd(clock, ~resetn, lcd_write_en, lcd_write_data[7:0], lcd_data, lcd_rw, lcd_en, lcd_rs, lcd_on, lcd_blon);
	
	// example for sending ps2 data to the first two seven segment displays
	Hexadecimal_To_Seven_Segment hex1(ps2_out[3:0], seg1);
	Hexadecimal_To_Seven_Segment hex2(ps2_out[7:4], seg2);
	
	// the other seven segment displays are currently set to 0
	Hexadecimal_To_Seven_Segment hex3(4'b0, seg3);
	Hexadecimal_To_Seven_Segment hex4(4'b0, seg4);
	Hexadecimal_To_Seven_Segment hex5(4'b0, seg5);
	Hexadecimal_To_Seven_Segment hex6(4'b0, seg6);
	Hexadecimal_To_Seven_Segment hex7(4'b0, seg7);
	Hexadecimal_To_Seven_Segment hex8(4'b0, seg8);
	
	// some LEDs that you could use for debugging if you wanted
	assign leds = 8'b00101011;
	
endmodule

module processor(clock, reset, ps2_key_pressed, ps2_out, lcd_write, lcd_data, debug_data, debug_addr);

// VALUES USED FOR DECODING:
// inA and inB, which are values going into the ALU
// rdtoregfile, which is the register that will be written to (doesn't account for write enable)
// datatoregfile, data that will be written to the register rdtoregfile
// pc, nextpc, which are exactly what they are
// mxbpaout, mxbpbout, which are values indicating a mx bypass into register A and register B (respectively)
// swmxbpout, value indicating a mx bypass on a save word instruction
// xmaluout, output of the alu on the output of the x/m latch
// writedataout, datawriter, data to be written into dmem on the output and input sides of the x/m latch, respectively
// j, jal, jr, outputs from the control algorithm indicating one of these jump operations
// fdir, fdpc, instruction and pc on the output of the f/d latch
// CLEARALL, value indicating a need to flush f/d and d/x latches because of jump or branch

input 
clock, reset, ps2_key_pressed;
input 
[7:0] ps2_out;
output 
lcd_write;
output 
[31:0] lcd_data;
// GRADER OUTPUTS - YOU MUST CONNECT TO YOUR DMEM
output 
[31:0] debug_data;
output
[11:0] debug_addr;
// your processor here
//
wire clockn;
not CLOCKN(clockn, clock);
//////////////////////////////////////
////// THIS IS REQUIRED FOR GRADING
// CHANGE THIS TO ASSIGN YOUR DMEM WRITE ADDRESS ALSO TO debug_addr
//assign debug_addr = (12'b000000000001);
// CHANGE THIS TO ASSIGN YOUR DMEM DATA INPUT (TO BE WRITTEN) ALSO TO debug_data
//assign debug_data = (12'b000000000001);
////////////////////////////////////////////////////////////
// You'll need to change where the dmem and imem read and write...
// PC LATCH
wire [31:0] nextpc, pc;
PC pcreg(nextpc, clock, reset, 1'b1, pc);
// FETCH SECTION
wire [31:0] pcplus1;
wire dc1, dc2;
adder pcaddr(1'b0, pc, 32'b00000000000000000000000000000001, 1'b1, pcplus1, dc1, dc2);
wire [31:0] imemout;
imem myimem(
.address (pc[11:0]),
.clken
(1'b1),
.clock
(clockn), 
.q
(imemout) 
); 

// F/D LATCH
wire [31:0] fdir, fdpc;
// FIX CLEARING ON JUMPS!!!!!
fdlatch FD(imemout, pcplus1, clock, CLEARALL, fdir, fdpc);

// DECODE SECTION
wire writeback, bne, blt, memwrite, memread;
wire j, jal, jr;
wire alusrc;
wire [4:0] aluop, regdst, regA, regB;
control controller(fdir, writeback, bne, blt, memwrite, memread, alusrc, aluop, regdst, regA, regB, jal, j, jr);


wire [31:0] dataA, dataB;
regfile registers(clockn, wbfinal, reset, rdtoregfile, regA, regB, datatoregfile, dataA, dataB);
wire [31:0] immediateval;
signextender sex(fdir[16:0], immediateval);

// D/X LATCH
wire [31:0] irout, pcout, outA, outB, ivalout;
wire wbout, bneout, bltout, memwriteout, memreadout, dxjal, dxj, dxjr;
wire alusrcout;
wire [4:0] rdout, aluopout;
dxlatch DX(fdir, fdpc, dataA, dataB, immediateval, writeback, bne, blt, memwrite, memread, regdst, alusrc, aluop, jal, j, jr, clock, CLEARALL, irout, pcout, outA, outB, ivalout, wbout, bneout, bltout, memwriteout, memreadout, rdout, alusrcout, aluopout, dxjal, dxj, dxjr);

// EXECUTION SECTION
wire [31:0] branchpc, aluout;
wire [31:0] inA, inB;
wire dc3, dc4, ne, lt;
assign inA = alusrcout ? ivalout : 32'bz;
wire mxbpaLW, mxbpaNOTLW, notLW;
and YESLW(mxbpaLW, mxbpaout, xmmemreadout);
not NOTLW(notLW, xmmemreadout);
and NOLW(mxbpaNOTLW, mxbpaout, notLW);
assign inA = mxbpaNOTLW ? xmaluout : 32'bz;
assign inA = mxbpaLW ? readdata : 32'bz;
wire wxnotmx, notmx;
not notMX(notmx, mxbpaout);
and WXNOTMX(wxnotmx, notmx, wxbpaout);

wire wxaLW, wxaNOTLW, wxnotLW;
not WXNLW(wxnotLW, memreadfinal);
and WXALW(wxaLW, wxnotmx, memreadfinal);
and WXANOTLW(wxaNOTLW, wxnotmx, wxnotLW);
assign inA = wxaNOTLW ? alufinal : 32'bz;
assign inA = wxaLW ? readdataout : 32'bz;
wire justoutA;
nor JOA(justoutA, wxbpaout, mxbpaout, alusrcout);
assign inA = justoutA ? outA : 32'bz;

wire mxbpbLW, mxbpbNOTLW;
and BYESLW(mxbpbLW, mxbpbout, xmmemreadout);
and BNOTLW(mxbpbNOTLW, mxbpbout, notLW);
assign inB = mxbpbNOTLW ? xmaluout : 32'bz;
assign inB = mxbpbLW ? readdata : 32'bz;
wire wxnotmxb, notmxb;
not NOTMXB(notmxb, mxbpbout);
and WXNOTMXB(wxnotmxb, notmxb, wxbpbout);

wire wxbLW, wxbNOTLW;
and WXBLW(wxbLW, wxnotmxb, memreadfinal);
and WXBNLW(wxbNOTLW, wxnotmxb, wxnotLW);
assign inB = wxbNOTLW ? alufinal : 32'bz;
assign inB = wxbLW ? readdataout : 32'bz;
wire justoutB;
nor JOB(justoutB, mxbpbout, wxbpbout);
assign inB = justoutB ? outB : 32'bz;

adder branchadder(1'b0, pcout, ivalout, 1'b1, branchpc, dc3, dc4);
alu myalu(inA, inB, aluopout, irout[11:7], aluout, ne, lt);

// MX BYPASSING LOGIC
	wire mxbpaout, mxbpbout, swmxbpout;
	mxbypassA mxbpa(xmwbout, xmrdout, irout, rdout, irout[21:17], bneout, bltout, mxbpaout);
	mxbypassB mxbpb(xmwbout, xmrdout, irout, irout[21:17], irout[16:12], bneout, bltout, memwriteout, memreadout, mxbpbout);
	swmxbypass swmxbp(xmwbout, xmrdout, dxjr, memwriteout, rdout, swmxbpout);


// X/M LATCH
wire [31:0] xmirout, bpcout, pc1out;
wire [31:0] xmaluout, writedataout, datawriter;
assign datawriter = swmxbpout ? xmaluout : 32'bz;
wire swnotmx;
wire swxnotmx;
not SWNOTMX(swnotmx, swmxbpout);
and SWXNOTMX(swxnotmx, swwxbpout, swnotmx);
assign datawriter = swxnotmx ? alufinal : 32'bz;
wire justA;
nor JB(justA, swwxbpout, swmxbpout);
assign datawriter = justA ? outA : 32'bz;
wire [4:0] xmrdout;
wire xmwbout, xmbneout, xmbltout, neout, ltout, xmmemwriteout, xmmemreadout, xmjal, xmj, xmjr;
xmlatch XM(irout, rdout, branchpc, pcout, wbout, bneout, bltout, ne, lt, aluout, memwriteout, memreadout, datawriter, dxjal, dxj, dxjr, clock, reset, xmirout, xmrdout, bpcout, pc1out, xmwbout, xmbneout, xmbltout, neout, ltout, xmaluout, xmmemwriteout, xmmemreadout, writedataout, xmjal, xmj, xmjr);

// MEMORY SECTION
assign debug_addr = xmaluout[11:0];
assign debug_data = wmbpout ? datatoregfile : writedataout;
wire branchlt, branchne, branch;
and BLT(branchlt, xmbltout, ltout);
and BNE(branchne, xmbneout, neout);
or BRANCH(branch, branchlt, branchne);
assign nextpc = branch ? bpcout : 32'bz;
wire [31:0] readdata;
wire CLEARALL, clearalln; 
wire branchn, jaln, jn, jrn;
not JALN(jaln, xmjal);
not JN(jn, xmj);
not JRN(jrn, xmjr);

wire [31:0] jpc; 
assign jpc[26:0] = xmirout[26:0];
assign jpc[31:27] = pc1out[31:27];
assign nextpc = xmj ? jpc : 32'bz;
assign nextpc = xmjal ? jpc : 32'bz;
assign nextpc = xmjr ? writedataout : 32'bz;


not BRANCHN(branchn, branch);
or CLEARall(CLEARALL, reset, branch, xmjal, xmjr, xmj);
wire makepcsimple;
nor NbNj(makepcsimple, branch, xmjal, xmjr, xmj);
assign nextpc = makepcsimple ? pcplus1 : 32'bz;

dmem mydmem(
.address (debug_addr),
.clock
(clockn),
.data
(debug_data),
.wren
(xmmemwriteout), 
.q
(readdata) 
);


// WX BYPASSING LOGIC
	wire wxbpaout, wxbpbout, swwxbpout;
	wxbypassA wxbpa(wbfinal, rdtoregfile, irout, rdout, irout[21:17], bneout, bltout, wxbpaout);
	wxbypassB wxbpb(wbfinal, rdtoregfile, irout, irout[21:17], irout[16:12], bneout, bltout, memwriteout, memreadout, wxbpbout);
	swwxbypass swwxbp(wbfinal, rdtoregfile, dxjr, memwriteout, rdout, swwxbpout);


// M/W LATCH
wire [4:0] rdtoregfile;
wire [31:0] readdataout, alufinal, pcjal;
wire wbfinal, memreadfinal;
mwlatch MW(xmrdout, xmaluout, xmwbout, xmmemreadout, readdata, pc1out, xmjal, clock, reset, rdtoregfile, alufinal, wbfinal, memreadfinal, readdataout, pcjal, mwjal);

// WM BYPASS LOGIC
	wire wmbpout;
	wmbypass wmbp(memreadfinal, xmmemwriteout, wmbpout);

// WRITEBACK SECTION
wire [31:0] datatoregfile;
assign datatoregfile = memreadfinal ? readdataout : 32'bz;
assign datatoregfile = mwjal ? pcjal : 32'bz;
wire nrjal;
nor NRJAL(nrjal, memreadfinal, mwjal);
assign datatoregfile = nrjal ? alufinal : 32'bz;
endmodule

module PC(d, clk, clr, wena, pcout);

input [31:0] d;
input clk, clr, wena;
output [31:0] pcout;
wire [31:0] none;
register pcreg(d, clk, clr, 1'b1, wena, 1'b1, 1'b0, pcout, none); 
endmodule

module fdlatch(insn, pc, clk, clr, irout, pcout);
input [31:0] insn, pc;
input clk, clr;
output [31:0] irout, pcout;
wire [31:0] none, none1;
register IR(insn, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, irout, none);
register PC(pc, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, pcout, none1);

endmodule

module dxlatch(insn, pcplus1, regA, regB, ival, writeback, bne, blt, memwrite, memread, regdst, alusrc, aluop, jal, j, jr, clk, clr, irout, pcout, outA, outB, ivalout, wbout, bneout, bltout, memwriteout, memreadout, rdout, alusrcout, aluopout, jalout, jout, jrout);

input [31:0] insn, pcplus1, regA, regB, ival;
input clk, clr, writeback, bne, blt, memwrite, memread, alusrc, jal, j, jr;
input [4:0] regdst, aluop;
output [31:0] irout, pcout, outA, outB, ivalout;
output wbout, bneout, bltout, memwriteout, memreadout, alusrcout, jalout, jout, jrout;
output [4:0] rdout, aluopout; 
wire [31:0] none, none1, none2, none3, none4;
wire clrn;
not CLRN(clrn, clr);
register IR(insn, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, irout, none);
register PC(pcplus1, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, pcout, none1);
register A(regA, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, outA, none2);
register B(regB, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, outB, none3);
register IVAL(ival, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, ivalout, none4);
DFFE WB(.d(writeback), .clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(wbout));
DFFE BNE(.d(bne),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(bneout));
DFFE BLT(.d(blt),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(bltout));
DFFE MEMW(.d(memwrite),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(memwriteout));
DFFE MEMR(.d(memread),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(memreadout));
DFFE ALUSRC(.d(alusrc),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(alusrcout));
DFFE JAL(.d(jal),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(jalout));
DFFE J(.d(j),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(jout));
DFFE JR(.d(jr),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(jrout));
genvar i;
generate
for( i = 0; i<5; i = i + 1) begin : latchGen
DFFE rdbit(.d(regdst[i]),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(rdout[i]));
DFFE aluopbit(.d(aluop[i]),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(aluopout[i]));
end
endgenerate

endmodule

module xmlatch(insn, regdst, branchpc, pc1, writeback, bne, blt, alune, alult, aluval, memwrite, memread, writedata, jal, j, jr, clk, clr, irout, rdout, bpcout, pc1out, wbout, bneout, bltout, aluneout, alultout, aluout, memwriteout, memreadout, writedataout, jalout, jout, jrout);

input [31:0] branchpc, aluval, writedata, pc1, insn;
input writeback, bne, blt, alune, alult, memwrite, memread, clk, clr, jal, j, jr;
input [4:0] regdst;
output [31:0] irout, bpcout, aluout, writedataout, pc1out;
output wbout, bneout, bltout, aluneout, alultout, memwriteout, memreadout, jalout, jout, jrout;
output [4:0] rdout;
wire [31:0] none, none1, none2, none3, none4;
wire clrn;
not CLRN(clrn, clr);
register IR(insn, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, irout, none);
register BPC(branchpc, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, bpcout, none1);
register PC1(pc1, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, pc1out, none2);
register ALU(aluval, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, aluout, none3);
register WD(writedata, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, writedataout, none4);
DFFE WB(.d(writeback), .clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(wbout));
DFFE BNE(.d(bne),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(bneout));
DFFE BLT(.d(blt),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(bltout));
DFFE MEMW(.d(memwrite),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(memwriteout));
DFFE MEMR(.d(memread),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(memreadout));
DFFE ALULT(.d(alult),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(alultout));
DFFE ALUNE(.d(alune),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(aluneout));
DFFE JAL(.d(jal),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(jalout));
DFFE J(.d(j),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(jout));
DFFE JR(.d(jr),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(jrout));
genvar i;
generate
for( i = 0; i<5; i = i + 1) begin : latchGen
DFFE rdbit(.d(regdst[i]),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(rdout[i]));
end
endgenerate

endmodule

module mwlatch(regdst, aluval, writeback, memread, readdata, pc1, jal, clk, clr, rdout, aluout, wbout, memreadout, readdataout, pc1out, jalout);

input [4:0] regdst;
input [31:0] aluval, readdata, pc1;
input writeback, memread, clk, clr, jal;
output [4:0] rdout;
output [31:0] aluout, readdataout, pc1out;
output wbout, memreadout, jalout;
wire [31:0] none, none1, none2;
wire clrn;
not CLRN(clrn, clr);
register ALU(aluval, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, aluout, none);
register DMEMDATA(readdata, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, readdataout, none1);
register PC1(pc1, clk, clr, 1'b1, 1'b1, 1'b1, 1'b0, pc1out, none2);
DFFE WB(.d(writeback), .clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(wbout));
DFFE MEMR(.d(memread),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(memreadout));
DFFE JAL(.d(jal),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(jalout));
genvar i;
generate
for( i = 0; i<5; i = i + 1) begin : latchGen
DFFE rdbit(.d(regdst[i]),.clk(clk),.clrn(clrn),.prn(1'b1),.ena(1'b1),.q(rdout[i]));
end
endgenerate

endmodule

module control(insn, writeback, bne, blt, memwrite, memread, alusrc, aluop, regdst, regA, regB, jal, j, jr);

input [31:0] insn;
output writeback, bne, blt, memwrite, memread, alusrc, jal, j, jr;
output [4:0] aluop, regdst, regA, regB;
// alusrc of 1 is for I instructions, 0 is for R instructions
wire [31:0] ninsn;
genvar i;
generate
for(i = 0; i <32; i = i+1) begin : neginsnGen
not notbit(ninsn[i], insn[i]);
end
endgenerate
// determining if R type operation
wire Rop;
and ROP(Rop, ninsn[31], ninsn[30], ninsn[29], ninsn[28], ninsn[27]);
// jump return
and JR(jr, ninsn[31], ninsn[30], insn[29], ninsn[28], ninsn[27]);
// jump
and J(j, ninsn[31], ninsn[30], ninsn[29], ninsn[28], insn[27]);
// bne
and BNE(bne, ninsn[31], ninsn[30], ninsn[29], insn[28], ninsn[27]);
// jal
and JAL(jal, ninsn[31], ninsn[30], ninsn[29], insn[28], insn[27]);
// blt
and BLT(blt, ninsn[31], ninsn[30], insn[29], insn[28], ninsn[27]);
// sw
and SW(memwrite, ninsn[31], ninsn[30], insn[29], insn[28], insn[27]);
// lw
wire lw;
and LW(lw, ninsn[31], insn[30], ninsn[29], ninsn[28], ninsn[27]);
// addi
wire addi;
and ADDI(addi, ninsn[31], ninsn[30], insn[29], ninsn[28], insn[27]);
or WB(writeback, Rop, jal, lw, addi);
assign memread = lw;
or ALUSRC(alusrc, lw, memwrite, addi);
assign aluop = Rop ? insn[6:2] : 5'bz;
assign aluop = bne ? 5'b00001 : 5'bz;
assign aluop = blt ? 5'b00001 : 5'bz;
assign aluop = addi ? 5'b0000 : 5'bz;
assign aluop = memwrite ? 5'b00000 : 5'bz;
assign aluop = lw ? 5'b00000 : 5'bz;
assign regdst = jal ? 5'b11111 : insn[26:22];
// determine read registers based on operation type
assign regA = Rop ? insn[21:17] : 5'bz;
assign regA = jr ? insn[26:22] : 5'bz;
wire icheck, j1;
or Icheck(icheck, bne, blt, lw, memwrite, addi);
assign regA = icheck ? insn[26:22] : 5'bz;
nor alt(j1, Rop, jr, icheck);
assign regA = j1 ? 5'b00000 : 5'bz;
assign regB = icheck ? insn[21:17] : 5'bz;
assign regB = Rop ? insn[16:12] : 5'bz;
assign regB = jr ? 5'b00000 : 5'bz;
assign regB = j1 ? 5'b00000 : 5'bz;

endmodule

module mxbypassA(xmwriteback, xmrd, dxir, dxrd, dxrs, dxbne, dxblt, bypass);

	input xmwriteback, dxbne, dxblt;
	input [4:0] xmrd, dxrd, dxrs;
	input [31:0] dxir;
	output bypass;
	
	wire [31:0] ndxir;
	genvar i;
	generate
		for(i = 0; i<32; i = i + 1) begin : ninsnGen
			not nbit(ndxir[i], dxir[i]);
		end
	endgenerate
	
	wire [4:0] xmrddxrs;
	xnor b1(xmrddxrs[4], xmrd[4], dxrs[4]);
	xnor b2(xmrddxrs[3], xmrd[3], dxrs[3]);
	xnor b3(xmrddxrs[2], xmrd[2], dxrs[2]);
	xnor b4(xmrddxrs[1], xmrd[1], dxrs[1]);
	xnor b5(xmrddxrs[0], xmrd[0], dxrs[0]);
	wire bypNotBranch;
	// Overwrite RS as regA on r operations only
	and NBByp(bypNotBranch, xmrddxrs[4],xmrddxrs[3],xmrddxrs[2],xmrddxrs[1],xmrddxrs[0], ndxir[31], ndxir[30], ndxir[29], ndxir[28], ndxir[27]);
	
	wire [4:0] xmrddxrd;
	xnor b11(xmrddxrd[4], xmrd[4], dxrd[4]);
	xnor b12(xmrddxrd[3], xmrd[3], dxrd[3]);
	xnor b13(xmrddxrd[2], xmrd[2], dxrd[2]);
	xnor b14(xmrddxrd[1], xmrd[1], dxrd[1]);
	xnor b15(xmrddxrd[0], xmrd[0], dxrd[0]);
	wire bypBranch;
	// overwrite RD as regA on branches, jr only
	wire brancher;
	or BRANCHER(brancher, dxbne, dxblt);
	and BByp(bypBranch, xmrddxrd[4], xmrddxrd[3], xmrddxrd[2], xmrddxrd[1], xmrddxrd[0], brancher);
	
	wire byptemp;
	or BYPASST(byptemp, bypBranch, bypNotBranch);
	and BYPASS(bypass, byptemp, xmwriteback);

	//use if reg in M region being written to is the same as one that's being read in X
	//special cases in bne, blt because both values need to be read
	
endmodule

module swmxbypass(xmwriteback, xmrd, dxjr, dxmemwrite, dxrd, bypass);

	input xmwriteback, dxmemwrite, dxjr;
	input [4:0] xmrd, dxrd;
	output bypass;
	wire [4:0] samer;
	xnor b1(samer[4], xmrd[4], dxrd[4]);
	xnor b2(samer[3], xmrd[3], dxrd[3]);
	xnor b3(samer[2], xmrd[2], dxrd[2]);
	xnor b4(samer[1], xmrd[1], dxrd[1]);
	xnor b5(samer[0], xmrd[0], dxrd[0]);
	wire swjr;
	or SWJR(swjr, dxmemwrite, dxjr);
	and byp(bypass, samer[4], samer[3], samer[2], samer[1], samer[0], xmwriteback, swjr);
	
endmodule

module mxbypassB(xmwriteback, xmrd, dxir, dxrs, dxrt, dxbne, dxblt, dxmemwrite, dxmemread, bypass);

	input xmwriteback, dxbne, dxblt, dxmemwrite, dxmemread;
	input [4:0] xmrd, dxrs, dxrt;
	input [31:0] dxir;
	output bypass;
	
	wire [31:0] ndxir;
	genvar i;
	generate
		for(i = 0; i<32; i = i + 1) begin : ninsnGen
			not nbit(ndxir[i], dxir[i]);
		end
	endgenerate
	
	//RS will be in slot B for branches, sw, lw, addi
	wire [4:0] xmrddxrs;
	xnor b1(xmrddxrs[4], xmrd[4], dxrs[4]);
	xnor b2(xmrddxrs[3], xmrd[3], dxrs[3]);
	xnor b3(xmrddxrs[2], xmrd[2], dxrs[2]);
	xnor b4(xmrddxrs[1], xmrd[1], dxrs[1]);
	xnor b5(xmrddxrs[0], xmrd[0], dxrs[0]);
	wire brancher, bypBranch, addi;
	and ADDI(addi, ndxir[31], ndxir[30], dxir[29], ndxir[28], dxir[27]);
	or BRANCHER(brancher, dxbne, dxblt, dxmemwrite, dxmemread, addi);
	
	and BByp(bypBranch, xmrddxrs[4],xmrddxrs[3],xmrddxrs[2],xmrddxrs[1],xmrddxrs[0], brancher);
	
	//RT in slot B for Rops only. 
	wire [4:0] xmrddxrt;
	xnor b11(xmrddxrt[4], xmrd[4], dxrt[4]);
	xnor b12(xmrddxrt[3], xmrd[3], dxrt[3]);
	xnor b13(xmrddxrt[2], xmrd[2], dxrt[2]);
	xnor b14(xmrddxrt[1], xmrd[1], dxrt[1]);
	xnor b15(xmrddxrt[0], xmrd[0], dxrt[0]);
	wire bypNotBranch;
	and BNByp(bypNotBranch, xmrddxrt[4], xmrddxrt[3], xmrddxrt[2], xmrddxrt[1], xmrddxrt[0], ndxir[31], ndxir[30], ndxir[29], ndxir[28], ndxir[27]);
	
	wire bypasstemp;
	or BYPASST(bypasstemp, bypBranch, bypNotBranch);
	and BYPASS(bypass, bypasstemp, xmwriteback);

endmodule

module wxbypassA(mwwriteback, mwrd, dxir, dxrd, dxrs, dxbne, dxblt, bypass);

	input mwwriteback, dxbne, dxblt;
	input [4:0] mwrd, dxrd, dxrs;
	input [31:0] dxir;
	output bypass;
	
	wire [31:0] ndxir;
	genvar i;
	generate
		for(i = 0; i<32; i = i + 1) begin : ninsnGen
			not nbit(ndxir[i], dxir[i]);
		end
	endgenerate
	
	wire [4:0] mwrddxrs;
	xnor b1(mwrddxrs[4], mwrd[4], dxrs[4]);
	xnor b2(mwrddxrs[3], mwrd[3], dxrs[3]);
	xnor b3(mwrddxrs[2], mwrd[2], dxrs[2]);
	xnor b4(mwrddxrs[1], mwrd[1], dxrs[1]);
	xnor b5(mwrddxrs[0], mwrd[0], dxrs[0]);
	wire bypNotBranch;
	// Overwrite RS as regA on r operations only
	and NBByp(bypNotBranch, mwrddxrs[4],mwrddxrs[3],mwrddxrs[2],mwrddxrs[1],mwrddxrs[0], ndxir[31], ndxir[30], ndxir[29], ndxir[28], ndxir[27]);
	
	wire [4:0] mwrddxrd;
	xnor b11(mwrddxrd[4], mwrd[4], dxrd[4]);
	xnor b12(mwrddxrd[3], mwrd[3], dxrd[3]);
	xnor b13(mwrddxrd[2], mwrd[2], dxrd[2]);
	xnor b14(mwrddxrd[1], mwrd[1], dxrd[1]);
	xnor b15(mwrddxrd[0], mwrd[0], dxrd[0]);
	wire bypBranch;
	// overwrite RD as regA on branches, jr only
	wire brancher;
	or BRANCHER(brancher, dxbne, dxblt);
	and BByp(bypBranch, mwrddxrd[4], mwrddxrd[3], mwrddxrd[2], mwrddxrd[1], mwrddxrd[0], brancher);
	
	wire byptemp;
	or BYPASST(byptemp, bypBranch, bypNotBranch);
	and BYPASS(bypass, byptemp, mwwriteback);

	//MX Bypass takes precedent if both require a bypass
	//use if reg being written to in W is same as one being read in X

endmodule

module wxbypassB(mwwriteback, mwrd, dxir, dxrs, dxrt, dxbne, dxblt, dxmemwrite, dxmemread, bypass);

	input mwwriteback, dxbne, dxblt, dxmemwrite, dxmemread;
	input [4:0] mwrd, dxrs, dxrt;
	input [31:0] dxir;
	output bypass;
	
	wire [31:0] ndxir;
	genvar i;
	generate
		for(i = 0; i<32; i = i + 1) begin : ninsnGen
			not nbit(ndxir[i], dxir[i]);
		end
	endgenerate
	
	//RS will be in slot B for branches, sw, lw, addi
	wire [4:0] mwrddxrs;
	xnor b1(mwrddxrs[4], mwrd[4], dxrs[4]);
	xnor b2(mwrddxrs[3], mwrd[3], dxrs[3]);
	xnor b3(mwrddxrs[2], mwrd[2], dxrs[2]);
	xnor b4(mwrddxrs[1], mwrd[1], dxrs[1]);
	xnor b5(mwrddxrs[0], mwrd[0], dxrs[0]);
	wire brancher, bypBranch, addi;
	and ADDI(addi, ndxir[31], ndxir[30], dxir[29], ndxir[28], dxir[27]);

	or BRANCHER(brancher, dxbne, dxblt, addi, dxmemwrite, dxmemread);
	
	and BByp(bypBranch, mwrddxrs[4],mwrddxrs[3],mwrddxrs[2],mwrddxrs[1],mwrddxrs[0], brancher);
	
	//RT in slot B for Rops only. 
	wire [4:0] mwrddxrt;
	xnor b11(mwrddxrt[4], mwrd[4], dxrt[4]);
	xnor b12(mwrddxrt[3], mwrd[3], dxrt[3]);
	xnor b13(mwrddxrt[2], mwrd[2], dxrt[2]);
	xnor b14(mwrddxrt[1], mwrd[1], dxrt[1]);
	xnor b15(mwrddxrt[0], mwrd[0], dxrt[0]);
	wire bypNotBranch;
	and BNByp(bypNotBranch, mwrddxrt[4], mwrddxrt[3], mwrddxrt[2], mwrddxrt[1], mwrddxrt[0], ndxir[31], ndxir[30], ndxir[29], ndxir[28], ndxir[27]);
	
	wire bypasstemp;
	or BYPASST(bypasstemp, bypBranch, bypNotBranch);
	and BYPASS(bypass, bypasstemp, mwwriteback);

endmodule

module swwxbypass(mwwriteback, mwrd, dxjr, dxmemwrite, dxrd, bypass);

	input mwwriteback, dxmemwrite, dxjr;
	input [4:0] mwrd, dxrd;
	output bypass;
	wire [4:0] samer;
	xnor b1(samer[4], mwrd[4], dxrd[4]);
	xnor b2(samer[3], mwrd[3], dxrd[3]);
	xnor b3(samer[2], mwrd[2], dxrd[2]);
	xnor b4(samer[1], mwrd[1], dxrd[1]);
	xnor b5(samer[0], mwrd[0], dxrd[0]);
	wire swjr;
	or SWJR(swjr, dxmemwrite, dxjr);
	and byp(bypass, samer[4], samer[3], samer[2], samer[1], samer[0], mwwriteback, swjr);
	
endmodule

module wmbypass(mwmemread, xmmemwrite, bypass);

	input mwmemread, xmmemwrite;
	output bypass;
	and bypasS(bypass, mwmemread, xmmemwrite);

endmodule

module regfile(clock, ctrl_writeEnable, ctrl_reset, ctrl_writeReg, ctrl_readRegA, ctrl_readRegB, data_writeReg, data_readRegA, data_readRegB);
   input clock, ctrl_writeEnable, ctrl_reset;
   input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
   input [31:0] data_writeReg;
   output [31:0] data_readRegA, data_readRegB;
   wire [31:0] writeDecoder;
   wire [31:0] readADecoder;
   wire [31:0] readBDecoder; 
   decoder write(.rd(ctrl_writeReg), .ena(ctrl_writeEnable), .out(writeDecoder));
   decoder readA(.rd(ctrl_readRegA), .ena(1'b1), .out(readADecoder));
   decoder readB(.rd(ctrl_readRegB), .ena(1'b1), .out(readBDecoder));
   allRegisters file(.d(data_writeReg), .clk(clock), .clr(ctrl_reset), .wena(writeDecoder), .rena(readADecoder), .renb(readBDecoder), .outA(data_readRegA), .outB(data_readRegB));

endmodule

module decoder(rd, ena, out);
   input [4:0] rd;
   input ena;
   output [31:0] out;
   assign out = ena << rd;
endmodule

module register(d, clk, clr, pre, wena, rena, renb, outA, outB);
   input [31:0] d;
   input clk, clr, wena, pre, rena,renb;
   wire [31:0] inBuff;
   output [31:0] outA, outB;
   wire clrn;
   not not1(clrn, clr);
   genvar i; 
   generate
      for(i = 0; i<32; i = i+1) begin : regGen
         dffe oneBit(.d(d[i]), .clk(clk), .clrn(clrn), .prn(pre), .ena(wena), .q(inBuff[i]));
      end
   endgenerate
   assign outA = rena ? inBuff : 32'bz;
   assign outB = renb ? inBuff : 32'bz;
endmodule

module allRegisters(d ,clk, clr, wena, rena, renb, outA, outB);
   input [31:0] d, wena, rena, renb;
   input clk, clr;
   output [31:0] outA, outB;
   genvar i;
   generate
         for(i = 0; i<32; i =  i+1) begin : allReg
            register regi( .d(d), .clk(clk),.clr(clr), .pre(1'b1), .wena(wena[i]), .rena(rena[i]),.renb(renb[i]), .outA(outA), .outB(outB));
         end
   endgenerate
endmodule

module signextender(in, out);

input [16:0] in;
output [31:0] out;

genvar i;
generate
for(i = 0; i<16; i = i+1) begin : firstGen
assign out[i] = in[i];
end
for(i=16; i<32; i=i+1) begin : signextGen
assign out[i] = in[16];
end
endgenerate

endmodule

module alu(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan);
	
	// INSERT CLOCK, EXCEPTION, INPUT, RESULTRDY
   input [31:0] data_operandA, data_operandB;
   input [4:0] ctrl_ALUopcode, ctrl_shiftamt;
   output [31:0] data_result;
   output isNotEqual, isLessThan;
   wire [7:0] decoder_out;
   wire overflow_ind, dontcare;
   opdecoder decoder(ctrl_ALUopcode, (1'b1), decoder_out);
   bitwise_and bit_and(data_operandA, data_operandB, decoder_out[2], data_result);
   bitwise_or bit_or(data_operandA, data_operandB, decoder_out[3], data_result);
   shift_left_logical SLL(data_operandA, ctrl_shiftamt, decoder_out[4], data_result);
   shift_right_arithmetic SRA(data_operandA, ctrl_shiftamt, decoder_out[5], data_result);
   adder CLA((1'b0), data_operandA, data_operandB, decoder_out[0], data_result, overflow_ind,dontcare);
   subtractor SUB(data_operandA, data_operandB, decoder_out[1], data_result, isNotEqual, isLessThan);   
	multdiv MULTDIV(data_operandA, data_operandB[15:0], decoder_out[6], decoder_out[7], data_result, data_exception);

endmodule

module tri_state(in, oe, out);

input [31:0] in;
input oe;
output [31:0] out;
assign out = oe ? in : 32'bz;

endmodule

module bitwise_and(data_operandA, data_operandB, output_enable, data_result);

input [31:0] data_operandA, data_operandB;
input output_enable;
output [31:0] data_result;
wire [31:0] and_to_tri;
genvar i; 
generate
for(i = 0; i<32; i = i+1) begin : bit_andGen
and one_bit(and_to_tri[i], data_operandA[i], data_operandB[i]);
end
endgenerate
tri_state out(.in(and_to_tri), .oe(output_enable), .out(data_result));

endmodule

module bitwise_or(data_operandA, data_operandB, output_enable, data_result);

input [31:0] data_operandA, data_operandB;
input output_enable;
output [31:0] data_result;
wire [31:0] or_to_tri;
genvar i; 
generate
for(i = 0; i<32; i = i+1) begin : bit_andGen
or one_bit(or_to_tri[i], data_operandA[i], data_operandB[i]);
end
endgenerate
tri_state out(.in(or_to_tri), .oe(output_enable), .out(data_result));

endmodule

module shift_left_logical(data_operandA, ctrl_shamt, output_enable, data_result);

input [31:0] data_operandA;
input [4:0] ctrl_shamt;
input output_enable;
wire [31:0] shift_to_tri;
output [31:0] data_result;
assign shift_to_tri = data_operandA << ctrl_shamt;
tri_state out(.in(shift_to_tri), .oe(output_enable), .out(data_result));

endmodule

module shift_right_arithmetic(data_operandA, ctrl_shamt, output_enable, data_result);

input signed [31:0] data_operandA;
input [4:0] ctrl_shamt;
input output_enable;
wire [31:0] shift_to_tri;
output [31:0] data_result;
assign shift_to_tri = data_operandA >>> ctrl_shamt;
tri_state out(.in(shift_to_tri), .oe(output_enable), .out(data_result));

endmodule

module opdecoder(ctrl_ALUopcode, ena, out);

// 6 outputs for each arithmetic/logical operation. just for reference:
// out[0] is the ADD output
// out[1] is the SUB output
// out[2] is the bitwise AND output
// out[3] is the bitwise OR output
// out[4] is the SLL output
// out[5] is the SRA output

input [4:0] ctrl_ALUopcode;
output[7:0] out;
input ena;
assign out = ena << ctrl_ALUopcode;

endmodule

module one_bit_block(carry_in, bitA, bitB, prop, gen,sum);

input carry_in, bitA, bitB;
output prop, gen, sum;
and GEN(gen, bitA, bitB);
or prop1(prop, bitA, bitB);
xor SUM(sum, bitA, bitB, carry_in);

endmodule

module eight_bit_adder(carry_in, operandA, operandB, sum, carry_out, last_carry_in);

input carry_in;
input [7:0] operandA, operandB;
output carry_out;
output [7:0] sum;
output last_carry_in;

// Stage 1
wire [1:0] bits0pg;
one_bit_block bits0(carry_in, operandA[0], operandB[0], bits0pg[0], bits0pg[1], sum[0]);
wire carry_out0;
wire and01;
and AND01(and01, bits0pg[0], carry_in);
or co0(carry_out0, and01, bits0pg[1]);

// Stage 2 
wire [1:0] bits1pg; 
one_bit_block bits1(carry_out0, operandA[1], operandB[1], bits1pg[0], bits1pg[1], sum[1]);
wire carry_out1;
wire and11;
and AND11(and11, carry_in, bits1pg[0], bits0pg[0]);
wire and12;
and AND12(and12, bits1pg[0], bits0pg[1]);
or co1(carry_out1, and11, and12, bits1pg[1]);

// Stage 3
wire[1:0] bits2pg;
one_bit_block bits2(carry_out1, operandA[2], operandB[2], bits2pg[0], bits2pg[1], sum[2]);
wire carry_out2;
wire and21;
and AND21(and21, bits2pg[0], bits1pg[1]);
wire and22;
and AND22(and22, bits2pg[0], bits1pg[0], bits0pg[1]);
wire and23;
and AND23(and23, bits2pg[0], bits1pg[0], bits0pg[0], carry_in);
or co2(carry_out2, and21, and22, and23, bits2pg[1]);

// Stage 4
wire [1:0] bits3pg;
one_bit_block bits3(carry_out2, operandA[3], operandB[3], bits3pg[0], bits3pg[1], sum[3]);
wire carry_out3;
wire and31;
and AND31(and31, bits3pg[0], bits2pg[1]);
wire and32;
and AND32(and32, bits3pg[0], bits2pg[0], bits1pg[1]);
wire and33;
and AND33(and33, bits3pg[0], bits2pg[0], bits1pg[0], bits0pg[1]);
wire and34;
and AND34(and34, bits3pg[0], bits2pg[0], bits1pg[0], bits0pg[0], carry_in);
or co3(carry_out3, and31, and32, and33, and34, bits3pg[1]);

// Stage 5
wire [1:0] bits4pg;
one_bit_block bits4(carry_out3, operandA[4], operandB[4], bits4pg[0], bits4pg[1], sum[4]);
wire carry_out4;
wire and41;
and AND41(and41, bits4pg[0], bits3pg[1]);
wire and42;
and AND42(and42, bits4pg[0], bits3pg[0], bits2pg[1]);
wire and43;
and AND43(and43, bits4pg[0], bits3pg[0], bits2pg[0], bits1pg[1]);
wire and44;
and AND44(and44, bits4pg[0], bits3pg[0], bits2pg[0], bits1pg[0], bits0pg[1]);
wire and451;
and AND451(and451, bits4pg[0], bits3pg[0], bits2pg[0]);
wire and452;
and AND452(and452, bits1pg[0], bits0pg[0], carry_in);
wire and45;
and AND45(and45, and451, and452);
wire co41;
or CO41(co41, and41, and42, and43);
or co4(carry_out4, co41, and44, and45, bits4pg[1]);

// Stage 6
wire [1:0] bits5pg;
one_bit_block bits5(carry_out4, operandA[5], operandB[5], bits5pg[0], bits5pg[1], sum[5]);
wire carry_out5;
wire and51;
and AND51(and51, bits5pg[0], bits4pg[1]);
wire and52;
and AND52(and52, bits5pg[0], bits4pg[0], bits3pg[1]);
wire and53;
and AND53(and53, bits5pg[0], bits4pg[0], bits3pg[0], bits2pg[1]);
wire and54;
and AND54(and54, bits5pg[0], bits4pg[0], bits3pg[0], bits2pg[0], bits1pg[1]);
wire and551;
and AND551(and551, bits5pg[0], bits4pg[0], bits3pg[0]);
wire and552;
and AND552(and552, bits2pg[0], bits1pg[0], bits0pg[1]);
wire and55;
and AND55(and55, and551, and552);
wire and561;
and AND561(and561, bits5pg[0], bits4pg[0], bits3pg[0], bits2pg[0]);
wire and562;
and AND562(and562, bits1pg[0], bits0pg[0], carry_in);
wire and56;
and AND56(and56, and561, and562);
wire co51;
or CO51(co51, and51, and52, and53);
or co5(carry_out5, co51, and54, and55, and56, bits5pg[1]);

// Stage 7
wire [1:0] bits6pg;
one_bit_block bits6(carry_out5, operandA[6], operandB[6], bits6pg[0], bits6pg[1], sum[6]);
wire carry_out6;
wire and61;
and AND61(and61, bits6pg[0], bits5pg[1]);
wire and62;
and AND62(and62, bits6pg[0], bits5pg[0], bits4pg[1]);
wire and63;
and AND63(and63, bits6pg[0], bits5pg[0], bits4pg[0], bits3pg[1]);
wire and64;
and AND64(and64, bits6pg[0], bits5pg[0], bits4pg[0], bits3pg[0], bits2pg[1]);
wire and651;
and AND651(and651, bits6pg[0], bits5pg[0], bits4pg[0]);
wire and652;
and AND652(and652, bits3pg[0], bits2pg[0], bits1pg[1]);
wire and65;
and AND65(and65, and651, and652);
wire and661;
and AND661(and661, bits6pg[0], bits5pg[0], bits4pg[0], bits3pg[0]);
wire and662;
and AND662(and662, bits2pg[0], bits1pg[0], bits0pg[1]);
wire and66;
and AND66(and66, and661, and662);
wire and67;
and AND67(and67, bits6pg[0], bits5pg[0], bits4pg[0], bits3pg[0], bits2pg[0], bits1pg[0], bits0pg[0], carry_in);
wire co61;
or CO61(co61, and61, and62, and63, and64);
or co6(last_carry_in, co61, and65, and66, and67, bits6pg[1]);

// Stage 8
wire [1:0] bits7pg;
one_bit_block bits7(last_carry_in, operandA[7], operandB[7], bits7pg[0], bits7pg[1], sum[7]);
wire and71;
and AND71(and71, bits7pg[0], bits6pg[1]);
wire and72;
and AND72(and72, bits7pg[0], bits6pg[0], bits5pg[1]);
wire and73;
and AND73(and73, bits7pg[0], bits6pg[0], bits5pg[0], bits4pg[1]);
wire and74;
and AND74(and74, bits7pg[0], bits6pg[0], bits5pg[0], bits4pg[0], bits3pg[1]);
wire and751;
and AND751(and751, bits7pg[0], bits6pg[0], bits5pg[0]);
wire and752;
and AND752(and752, bits4pg[0], bits3pg[0], bits2pg[1]);
wire and75;
and AND75(and75, and751, and752);
wire and761;
and AND761(and761, bits7pg[0], bits6pg[0], bits5pg[0], bits4pg[0]);
wire and762;
and AND762(and762, bits3pg[0], bits2pg[0], bits1pg[1]);
wire and76;
and AND76(and76, and761, and762);
wire and77;
and AND77(and77, bits7pg[0], bits6pg[0], bits5pg[0], bits4pg[0], bits3pg[0], bits2pg[0], bits1pg[0], bits0pg[1]);
wire and78;
and AND78(and78, bits7pg[0], bits6pg[0], bits5pg[0], bits4pg[0], bits3pg[0], bits2pg[0], bits1pg[0], bits0pg[0], carry_in);
wire co71;
or CO71(co71, and71, and72, and73, and74);
or co7(carry_out, co71, and75, and76, and77, and78, bits7pg[1]);

endmodule

module adder(carry_in, data_operandA, data_operandB, output_enable, out, carry_out, last_carry_in);

input carry_in, output_enable;
input [31:0] data_operandA, data_operandB;
wire [31:0] data_result;
output[31:0] out;
output carry_out;
output last_carry_in;
wire[2:0] dontcare;
wire carry_out1, carry_out2, carry_out3;
eight_bit_adder lowermost(carry_in, data_operandA[7:0], data_operandB[7:0], data_result[7:0], carry_out1, dontcare[0]);
eight_bit_adder lowermid(carry_out1, data_operandA[15:8], data_operandB[15:8], data_result[15:8], carry_out2, dontcare[1]);
eight_bit_adder uppermid(carry_out2, data_operandA[23:16], data_operandB[23:16], data_result[23:16], carry_out3, dontcare[2]);
eight_bit_adder uppermost(carry_out3, data_operandA[31:24], data_operandB[31:24], data_result[31:24], carry_out, last_carry_in);
tri_state outbuff(.in(data_result), .oe(output_enable), .out(out));

endmodule

module subtractor(data_operandA, data_operandB, output_enable, out2, isNotEqual, isLessThan);

input [31:0] data_operandA, data_operandB;
input output_enable;
wire [31:0] data_result;
wire [31:0] not_operandB;
wire carry_out;
wire last_carry_in;
output [31:0] out2;
output isNotEqual, isLessThan;
genvar i; 
generate
for(i = 0; i<32; i = i+1) begin : bit_notGen
not one_bit(not_operandB[i], data_operandB[i]);
end
endgenerate
adder add((1'b1), data_operandA, not_operandB, (1'b1), data_result, carry_out, last_carry_in);
tri_state outbuff(.in(data_result), .oe(output_enable), .out(out2));
wire overflow_less;
wire overflowcheck;
xor OVERF(overflowcheck, carry_out, last_carry_in);
xor ILT(isLessThan, data_result[31], overflow_check);
or equal_check(isNotEqual, data_result[0], data_result[1], data_result[2], data_result[3], data_result[4], data_result[5], data_result[6], data_result[7],data_result[8], data_result[9], data_result[10], data_result[11],data_result[12], data_result[13], data_result[14], data_result[15], data_result[16], data_result[17], data_result[18], data_result[19],data_result[20], data_result[21], data_result[22], data_result[23],data_result[24], data_result[25], data_result[26], data_result[27],data_result[28], data_result[29], data_result[30], data_result[31]);

endmodule

module multdiv(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, data_result, data_exception);
   input [31:0] data_operandA;
   input [15:0] data_operandB;
   input ctrl_MULT, ctrl_DIV; //clock;             
   output [31:0] data_result; 
   output data_exception; //data_inputRDY, data_resultRDY;

	//wire [4:0] counter;
	wire [31:0] data_result_mult, data_result_div;
	wire mult_oe, div_oe, data_exception_temp, data_exception_mult, data_exception_div;
	
	//count 	count1(counter, data_inputRDY, data_resultRDY, clock);
	and 		and_mult_out(mult_oe, ctrl_MULT, 1'b1);
	and 		and_div_out(div_oe, ctrl_DIV, 1'b1);
	mult 		mult1(data_result_mult, data_operandA, data_operandB, data_exception_mult);
	div		div1(data_result_div, data_operandA, data_operandB, data_exception_div);
	my_tri 	my32_div_out(data_result_div, div_oe, data_result);
	my_tri 	my32_mult_out(data_result_mult, mult_oe, data_result);
	or 		error(data_exception_temp, data_exception_mult, data_exception_div);
	assign 	data_exception = 1'b1 ? data_exception_temp : 1'b0;

endmodule

module div(quotient_out, dividend_in, divisor_in, data_exception_div);
	input [31:0] dividend_in;
	input [15:0] divisor_in;
	output data_exception_div;
	output [31:0] quotient_out;
	wire [31:0] quotient;
	wire [47:0] big;
	wire [16:0] little;
	wire [15:0] divisor;
	wire [31:0] dividend;
	wire sign,t1,t2,t3,t4,t5;
	
	//Determine final sign of quotient
	xor 	xor_sign(sign,dividend_in[31],divisor_in[15]);
	//Determine whether to 2's complement invert the divisor and/or dividend
	inverter32 	invert_dividend(dividend, dividend_in, dividend_in[31]);
	inverter16 	invert_divisor(divisor, divisor_in, divisor_in[15]);
	
	genvar i,j,k;
	generate
		//Assign dividend to lower 32 bits of big
		for(i=0; i<32; i=i+1) begin: loop1
			assign big[i]=dividend[i];
		end
		//Assign zeroes to upper 16 bits of big
		for(j=32; j<48; j=j+1) begin: loop2
			assign big[j] = 1'b0;
		end
		//Assign divisor to lower 16 bits of little
		for(k=0; k<16; k=k+1) begin: loop3
			assign little[k]=divisor[k];
		end
	endgenerate
	
	//Assign MSB of divisor to 0
	assign little[16] = 1'b0;
	
	//Detect divide by zero
	or 	or1(t1,divisor_in[0],divisor_in[1],divisor_in[2],divisor_in[3]);
	or 	or2(t2,divisor_in[4],divisor_in[5],divisor_in[6],divisor_in[7]);
	or 	or3(t3,divisor_in[8],divisor_in[9],divisor_in[10],divisor_in[11]);
	or 	or4(t4,divisor_in[12],divisor_in[13],divisor_in[14],divisor_in[15]);
	or 	div_zero(t5,t1,t2,t3,t4);
	not 	n1(data_exception_div,t5);
	
	//Cycle through all steps of a division
	wire [16:0] subs[31:0];
	divmod 	divmod1(subs[0],big[47:31],little,big[30],quotient[31]);
	genvar s,t;
	generate
		for(s=1; s<31; s=s+1) begin: loop4
			divmod	divmod2(subs[s],subs[s-1],little,big[30-s],quotient[31-s]);
		end
	endgenerate
	divmod 		divmod3(subs[31],subs[30],little,1'b0,quotient[0]);
	inverter32 	invert_quotient(quotient_out,quotient,sign);
	
endmodule

module divmod(subval,window,divisor,pull,quotient);
	input [16:0] window,divisor;
	input pull;
	output quotient;
	output [16:0] subval;
	wire [31:0] big_w, big_d, big_s, big_s_shift;
	wire [16:0] subtract, window_shift, window_shift_temp;
	wire isNotEqual,isLessThan,notLessThan;
	
	genvar i,j,k;
	//Basically putting the window and divisor values into 32 bit inputs, padded with zeroes
	generate
		for(i=0; i<17; i=i+1) begin: loop1
			assign big_w[i] = window[i];
			assign big_d[i] = divisor[i];
		end
		for(j=17; j<32; j=j+1) begin: loop2
			assign big_w[j] = 1'b0;
			assign big_d[j] = 1'b0;
		end
	endgenerate
	
	//Doing the subtraction
	adder32bit 	add(big_s, isNotEqual, isLessThan, big_w, big_d, 1'b1, 1'b1);
	//Shifting both big_s and window left by 1
	assign window_shift_temp = window << 1;
	assign big_s_shift = big_s << 1;
	
	//Reverting big_s back to a 17 bit value, leaving the LSB unassigned
	//Moving window_shift to another 17 bit value with an unassigned LSB
	generate
		for(k=1; k<17; k=k+1) begin: loop3
			assign subtract[k]=big_s_shift[k];
			assign window_shift[k]=window_shift_temp[k];
		end
	endgenerate
	//Assigning the pull-down bit as the LSB for subtract and window_shift
	assign subtract[0] = pull;
	assign window_shift[0] = pull;
	
	//Assign quotient bit to be high when !isLessThan
	not 	notLT(notLessThan,isLessThan);
	assign quotient = notLessThan;
	
	//Adding tri-states for either window or subtract output
	my_tri17 	subtract_tri(subtract, notLessThan, subval);
	my_tri17 	window_tri(window_shift, isLessThan, subval);
endmodule

//It can either output a number or its 2's complement inverse, depending on
//the value of inverse true
module inverter32(out, in, inv_true);
	input [31:0] in;
	input inv_true;
	output [31:0] out;
	
	wire isNotEqual,isLessThan;
	wire [31:0] inv,negative;
	
	genvar i;
	generate
		for (i=0; i<32; i=i+1) begin: loop1
			not not2(inv[i],in[i]);
		end
	endgenerate
	adder32bit 			add1(negative, isNotEqual, isLessThan, inv, 32'b00000000000000000000000000000001, 1'b0, 1'b0);
	mux_2_1_32bits 	m1(out, negative, in, inv_true);

endmodule

module inverter16(out, in, inv_true);
	input [15:0] in;
	input inv_true;
	output [15:0] out;
	
	wire isNotEqual,isLessThan;
	wire [31:0] negative;
	wire [31:0] inv;
	
	genvar i;
	generate
		for (i=0; i<16; i=i+1) begin: loop1
			not not2(inv[i],in[i]);
			assign inv[i+16] = 1'b0;
		end
	endgenerate
	
	adder32bit 			add1(negative, isNotEqual, isLessThan, inv, 32'b00000000000000000000000000000001, 1'b0, 1'b0);
	mux_2_1_16bits 	m1(out, negative[15:0], in, inv_true);
endmodule

module mux_2_1_32bits(out, in1, in2, select);
	input [31:0] in1, in2;
	input select;
	output [31:0] out;
	
	genvar i;
	generate
		for(i=0; i<32; i=i+1) begin: loop1
			wire w1,w2,select_not;
			not 	not1(select_not, select);
			and 	and1(w1, in1[i], select);
			and	and2(w2, in2[i], select_not);
			or 	or1(out[i],w1,w2);
		end
	endgenerate
endmodule

module mux_2_1_16bits(out, in1, in2, select);
	input [15:0] in1, in2;
	input select;
	output [15:0] out;
	
	genvar i;
	generate
		for(i=0; i<16; i=i+1) begin: loop1
			wire w1,w2,select_not;
			not 	not1(select_not, select);
			and 	and1(w1, in1[i], select);
			and	and2(w2, in2[i], select_not);
			or 	or1(out[i],w1,w2);
		end
	endgenerate
endmodule

module mult(out, cand, plier, overflow);
	input [31:0] cand;
	input [15:0] plier;
	output [31:0] out;
	output overflow;
	
	wire [31:0] p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15;
	wire [31:0] c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15;
	wire sign;		
	
	xor 		xor1(sign, cand[31],plier[15]);
	
	booth 	b1(p1,plier[0],1'b0,cand,32'b0);
	assign 	c1 = cand << 1;
	booth 	b2(p2,plier[1],plier[0],c1,p1);
	assign 	c2 = cand << 2;
	booth 	b3(p3,plier[2],plier[1],c2,p2);
	assign 	c3 = cand << 3;
	booth		b4(p4,plier[3],plier[2],c3,p3);
	assign 	c4 = cand << 4;
	booth 	b5(p5,plier[4],plier[3],c4,p4);
	assign 	c5 = cand << 5;
	booth 	b6(p6,plier[5],plier[4],c5,p5);
	assign 	c6 = cand << 6;
	booth 	b7(p7,plier[6],plier[5],c6,p6);
	assign 	c7 = cand << 7;
	booth 	b8(p8,plier[7],plier[6],c7,p7);
	assign 	c8 = cand << 8;
	booth 	b9(p9,plier[8],plier[7],c8,p8);
	assign 	c9 = cand << 9;
	booth 	b10(p10,plier[9],plier[8],c9,p9);
	assign 	c10 = cand << 10;
	booth 	b11(p11,plier[10],plier[9],c10,p10);
	assign 	c11 = cand << 11;
	booth 	b12(p12,plier[11],plier[10],c11,p11);
	assign 	c12 = cand << 12;
	booth 	b13(p13,plier[12],plier[11],c12,p12);
	assign 	c13 = cand << 13;
	booth 	b14(p14,plier[13],plier[12],c13,p13);
	assign 	c14 = cand << 14;
	booth 	b15(p15,plier[14],plier[13],c14,p14);
	assign 	c15 = cand << 15;
	booth 	b16(out,plier[15],plier[14],c15,p15);

	xor 		xor2(overflow,out[31],sign);
	
endmodule

module booth(out,lsb,extra,cand,product);
	input lsb, extra;
	input [31:0] cand,product;
	output [31:0] out;
	wire  [31:0] candy;
	wire t3,t2,t1,t0,sub,not_extra,not_lsb,isNotEqual,isLessThan;
	
	not 	not1(not_extra,extra);
	not 	not2(not_lsb,lsb);
	
	and	and1(sub,lsb,not_extra);
	and	and2(t0,lsb,extra);
	and	and3(t1,not_lsb,not_extra);
	or		or1(t2,t0,t1);
	not	not3(t3,t2);
	
	genvar i;
	generate
		for (i=0; i < 32; i=i+1) begin: loop1
			and andBit(candy[i],cand[i],t3);
		end
	endgenerate
	
	adder32bit add1(out, isNotEqual, isLessThan, product, candy, 1'b1, sub);

endmodule

module count(counter,in_rdy,out_rdy,clock);
	input clock;
	output [4:0] counter;
	output out_rdy,in_rdy;

	wire t0,t1,t2,t3,reset;
	wire n0,n1,n2,n3,n4;
	
	and	and0(t0,counter[0],counter[1]);
	and	and1(t1,t0,counter[2]);
	and	and2(t2,t1,counter[3]);
	not 	not0(n0,counter[0]);
	not 	not1(n1,counter[1]);
	not 	not2(n2,counter[2]);
	not 	not3(n3,counter[3]);
	not 	not4(n4,counter[4]);
	and 	and3(t3,n0,n1,n2,n3,counter[4]);
	and 	and4(in_rdy,n0,n1,n2,n3,n4);
	not 	notr(reset,t3);
	and 	and5(out_rdy,n4,counter[3],counter[2],counter[1],counter[0]);
	
	tffe	tff0(.t(1'b1), .clk(clock), .clrn(reset), .prn(1'b1), .ena(1'b1), .q(counter[0]));
	tffe 	tff1(.t(counter[0]), .clk(clock), .clrn(reset),.prn(1'b1), .ena(1'b1), .q(counter[1]));
	tffe 	tff2(.t(t0), .clk(clock), .clrn(reset), .prn(1'b1), .ena(1'b1), .q(counter[2]));
	tffe 	tff3(.t(t1), .clk(clock), .clrn(reset), .prn(1'b1), .ena(1'b1), .q(counter[3]));
	tffe 	tff4(.t(t2), .clk(clock), .clrn(reset), .prn(1'b1), .ena(1'b1), .q(counter[4]));

endmodule

module decoder32(out, select);
	input [4:0] select;
	output [5:0] out;
	assign out = (1'b1) << select;
endmodule

module my_tri(in, oe, out);
	input oe;
	input [31:0] in;
	output [31:0] out;
	assign out = oe ? in : 32'bz;
endmodule

module my_tri16(in, oe, out);
	input oe;
	input [15:0] in;
	output [15:0] out;
	assign out = oe ? in : 16'bz;
endmodule

module my_tri17(in, oe, out);
	input oe;
	input [16:0] in;
	output [16:0] out;
	assign out = oe ? in : 17'bz;
endmodule

module adder32bit(sum, isNotEqual, isLessThan, dataA, data_inputB, op0, op1);
	input 		[31:0] dataA, data_inputB;
	input 		op0,op1;
	wire 			oe,c1,c2,c3,c4,c71,c72,c73,c74;
	wire 			[31:0] dataB;
	output 		isNotEqual, isLessThan;
	output 		[31:0] sum;
	
	negate 		negate1(dataB,data_inputB,op1);
	
	adder8bit	adder1(sum[7:0], c1, dataA[7:0],dataB[7:0], op1,c71);
	adder8bit	adder2(sum[15:8], c2, dataA[15:8],dataB[15:8], c1,c72);
	adder8bit	adder3(sum[23:16], c3, dataA[23:16],dataB[23:16], c2,c73);
	adder8bit	adder4(sum[31:24], c4, dataA[31:24],dataB[31:24], c3,c74);
	
	wire 			t1;
	xor			xorLT1(t1,c74,c4);
	xor 			xorLT2(isLessThan, sum[31], t1);
	checkEquals eqs1(isNotEqual,sum);
	
//	or 			addTri(oe,op0,op1);
//	my_tri 		triAdd(sum,oe,out);
endmodule

module adder8bit(sum, c8, dataA, dataB, c0, c7);
	input [7:0] dataA, dataB;
	input	c0;
	output [7:0] sum;
	output c8,c7;
	
	//Stage 0
	wire 			g0,p0,c1,t0;
	adderBlock	block0(sum[0],g0,p0,c0,dataA[0],dataB[0]);
	and 			and0(t0,p0,c0);
	or 			or0(c1,t0,g0);
	
	//Stage 1
	wire 			g1,p1,c2,t1,t2;
	adderBlock 	block1(sum[1],g1,p1,c1,dataA[1],dataB[1]);
	and 			and1(t1,p1,g0);
	and 			and2(t2,p1,p0,c0);
	or 			or1(c2,t2,t1,g1);	
	
	//Stage 2
	wire 			g2,p2,c3,t3,t4,t5;
	adderBlock	block2(sum[2],g2,p2,c2,dataA[2],dataB[2]);
	and			and3(t3,p2,g1);
	and			and4(t4,p2,p1,g0);
	and 			and5(t5,p2,p1,p0,c0);
	or 			or2(c3,t3,t4,t5,g2);
	
	//Stage 3
	wire 			g3,p3,c4,t6,t7,t8,t9;
	adderBlock 	block3(sum[3],g3,p3,c3,dataA[3],dataB[3]);
	and 			and6(t6,p3,g2);
	and 			and7(t7,p3,p2,g1);
	and 			and8(t8,p3,p2,p1,g0);
	and 			and9(t9,p3,p2,p1,p0,c0);
	or				or3(c4,t6,t7,t8,t9,g3);

	//Stage 4
	wire 			g4,p4,c5,t10,t11,t12,t13,t14;
	adderBlock	block4(sum[4],g4,p4,c4,dataA[4],dataB[4]);
	and 			and10(t10,p4,g3);
	and 			and11(t11,p4,p3,g2);
	and 			and12(t12,p4,p3,p2,g1);
	and 			and13(t13,p4,p3,p2,p1,g0);
	and 			and14(t14,p4,p3,p2,p1,p0,c0);
	or 			or4(c5,t10,t11,t12,t13,t14,g4);
	
	//Stage 5
	wire 			g5,p5,c6,t15,t16,t17,t18,t19,t20;
	adderBlock	block5(sum[5],g5,p5,c5,dataA[5],dataB[5]);
	and 			and15(t15,p5,g4);
	and 			and16(t16,p5,p4,g3);
	and 			and17(t17,p5,p4,p3,g2);
	and 			and18(t18,p5,p4,p3,p2,g1);
	and 			and19(t19,p5,p4,p3,p2,p1,g0);
	and 			and20(t20,p5,p4,p3,p2,p1,p0,c0);
	or 			or5(c6,t15,t16,t17,t18,t19,t20,g5);

	//Stage 6
	wire 			g6,p6,c7,t21,t22,t23,t24,t25,t26,t27;
	adderBlock	block6(sum[6],g6,p6,c6,dataA[6],dataB[6]);
	and 			and21(t21,p6,g5);
	and 			and22(t22,p6,p5,g4);
	and 			and23(t23,p6,p5,p4,g3);
	and 			and24(t24,p6,p5,p4,p3,g2);
	and 			and25(t25,p6,p5,p4,p3,p2,g1);
	and 			and26(t26,p6,p5,p4,p3,p2,p1,g0);
	and 			and27(t27,p6,p5,p4,p3,p2,p1,p0,c0);
	or 			or6(c7,t21,t22,t23,t24,t25,t26,t27,g6);
	
	//Stage 7
	wire 			g7,p7,t28,t29,t30,t31,t32,t33,t34,t35;
	adderBlock	block7(sum[7],g7,p7,c7,dataA[7],dataB[7]);
	and 			and28(t28,p7,g6);
	and 			and29(t29,p7,p6,g5);
	and 			and30(t30,p7,p6,p5,g4);
	and 			and31(t31,p7,p6,p5,p4,g3);
	and 			and32(t32,p7,p6,p5,p4,p3,g2);
	and 			and33(t33,p7,p6,p5,p4,p3,p2,g1);
	and 			and34(t34,p7,p6,p5,p4,p3,p2,p1,g0);
	and 			and35(t35,p7,p6,p5,p4,p3,p2,p1,p0,c0);
	or 			or7(c8,t28,t29,t30,t31,t32,t33,t34,t35,g7);
	
endmodule

module adderBlock(s,g,p,c,a,b);
	input a,b,c;
	output s,g,p;
	
	and andG(g,a,b);
	or  orP(p,a,b);
	xor xorS(s,a,b,c);
endmodule

module negate(dataB,inputB,sub);
	input [31:0] inputB;
	input sub;
	output [31:0] dataB;
	
	genvar k;
	generate
		for (k=0; k<32; k=k+1) begin: loop3
			wire 	negB,subSel,t1,t2;
 			not 	not2(subSel,sub);
			not 	not3(negB,inputB[k]);
			and 	and1(t1,inputB[k],subSel);
			and 	and2(t2,negB,sub);
			or 	or1(dataB[k],t1,t2);
		end
	endgenerate
	
endmodule

module checkEquals(isNotEqual,sum);
	input [31:0] sum;
	wire w1,w2,w3,w4,t;
	output 	isNotEqual;
	or 		or1(w1,sum[0],sum[1],sum[2],sum[3],sum[4],sum[5],sum[6],sum[7]);
	or 		or2(w2,sum[8],sum[9],sum[10],sum[11],sum[12],sum[13],sum[14],sum[15]);
	or 		or3(w3,sum[16],sum[17],sum[18],sum[19],sum[20],sum[21],sum[22],sum[23]);
	or 		or4(w4,sum[24],sum[25],sum[26],sum[27],sum[28],sum[29],sum[30],sum[31]);
	or 		or5(isNotEqual,w1,w2,w3,w4);
endmodule

module checkEquals16(isNotEqual,sum);
	input [15:0] sum;
	wire w1,w2;
	output 	isNotEqual;
	or 		or1(w1,sum[0],sum[1],sum[2],sum[3],sum[4],sum[5],sum[6],sum[7]);
	or 		or2(w2,sum[8],sum[9],sum[10],sum[11],sum[12],sum[13],sum[14],sum[15]);
	or 		or5(isNotEqual,w1,w2);
endmodule
