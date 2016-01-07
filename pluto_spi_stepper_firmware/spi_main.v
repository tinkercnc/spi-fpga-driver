//    This is a component of pluto_step_spi, a stepper driver for linuxcnc over SPI.
//    based on the main.v from Jeff Epler <jepler@unpythonic.net>
//    Copyright 2013 by Matsche <matsche@play-pla.net>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//**********************************************************************

module pluto_spi_stepper(clk, SCK, MOSI, MISO, SSEL, nRESET, nPE, LED, nConfig, dout, din, step, dir);
parameter W=10;
parameter F=11;
parameter T=4;
input clk;

input SCK, SSEL, MOSI, nRESET;
output MISO, nConfig = 1'bZ, nPE;
output LED;
input [15:0] din;

assign nConfig = nRESET;
//assign nConfig = 1'b1;
assign nPE = 1'b1;

reg Spolarity;

reg[13:0] real_dout; output [13:0] dout = do_tristate ? 14'bZ : real_dout; 
wire[3:0] real_step; output [3:0] step = do_tristate ? 4'bZ : real_step ^ {4{Spolarity}};
wire[3:0] real_dir; output [3:0] dir = do_tristate ? 4'bZ : real_dir;


wire [W+F-1:0] pos0, pos1, pos2, pos3;
reg  [F:0]     vel0, vel1, vel2, vel3;
reg [T-1:0] dirtime, steptime;
reg [1:0] tap;

reg [10:0] div2048;
wire stepcnt = ~|(div2048[5:0]);

always @(posedge clk) begin
    div2048 <= div2048 + 1'd1;
end

wire do_enable_wdt, do_tristate;
wdt w(clk, do_enable_wdt, &div2048, do_tristate);

stepgen #(W,F,T) s0(clk, stepcnt, pos0, vel0, dirtime, steptime, real_step[0], real_dir[0], tap);
stepgen #(W,F,T) s1(clk, stepcnt, pos1, vel1, dirtime, steptime, real_step[1], real_dir[1], tap);
stepgen #(W,F,T) s2(clk, stepcnt, pos2, vel2, dirtime, steptime, real_step[2], real_dir[2], tap);
stepgen #(W,F,T) s3(clk, stepcnt, pos3, vel3, dirtime, steptime, real_step[3], real_dir[3], tap);

//**********************************************************************
// SPI zeugs
// synchronizing the handshakes
//
reg [2:0] SCKr;
always @(posedge clk) SCKr <= {SCKr[1:0], SCK};
wire SCK_risingedge = (SCKr[2:1]==2'b01);  // now we can detect SCK rising edges
wire SCK_fallingedge = (SCKr[2:1]==2'b10);  // and falling edges
wire SCK_high = SCKr[1];  // SCK is high

// same thing for SSEL
reg [2:0] SSELr;
always @(posedge clk) SSELr <= {SSELr[1:0], SSEL};
wire SSEL_active = ~SSELr[1];  // SSEL is active low
wire SSEL_startmessage = (SSELr[2:1]==2'b10);  // message starts at falling edge
wire SSEL_endmessage = (SSELr[2:1]==2'b01);  // message stops at rising edge

wire MOSI_data = MOSI;

// we handle SPI in 8-bits format, so we need a 3 bits counter to count the bits as they come in
reg [2:0] bitcnt;
reg byte_received;  // high when 8 bit has been received
reg [4:0] spibytecnt;
reg [7:0] data_recvd;
reg [7:0] data_sent;
reg [7:0] data_outbuf;

always @(posedge clk) begin
	if(SSEL_startmessage) begin
		//data_sent <= data_outbuf;
		bitcnt <= 3'b000;
		spibytecnt <= 5'b00000;
	end
	if(SSEL_active) begin
		if(SCK_risingedge) begin
			data_recvd <= {data_recvd[6:0], MOSI_data};
			bitcnt <= bitcnt + 3'b001;
			if(bitcnt==3'b000)
				data_sent <= data_outbuf;
		end
		else if(SCK_fallingedge) begin
			data_sent <= {data_sent[6:0], 1'b0};
			if(bitcnt==3'b000) begin
				spibytecnt <= spibytecnt + 5'b00001;
			end
		end
		byte_received <= SCK_risingedge && (bitcnt==3'b111);
	end
end
assign MISO = data_sent[7];  // send MSB first
// we assume that there is only one slave on the SPI bus
// so we don't bother with a tri-state buffer for MISO
// otherwise we would need to tri-state MISO when SSEL is inactive

reg [7:0] data_inbuf;
always @(posedge clk) begin
	if(SSEL_active) begin
		//------------------------------------------------- word 0
		if(spibytecnt == 5'b00000) begin	// 0
			data_outbuf <= pos0[7:0];
			if(byte_received)
				data_inbuf <= data_recvd;	//vel0[7:0] 
		end
		else if(spibytecnt == 5'b00001) begin	// 1
			data_outbuf <= pos0[15:8];
			if(byte_received)
				vel0 <= {data_recvd,data_inbuf};	//vel0
		end
		else if(spibytecnt == 5'b00010) begin	// 2
			data_outbuf <= pos0[W+F-1:16];
			if(byte_received)
				data_inbuf <= data_recvd;	//vel1[7:0]
		end
		else if(spibytecnt == 5'b00011) begin	// 3
			data_outbuf <= 8'b0;
			if(byte_received)
				vel1 <= {data_recvd,data_inbuf};	//vel1
		end
		//------------------------------------------------- word 1
		else if(spibytecnt == 5'b00100) begin	// 4
			data_outbuf <= pos1[7:0];
			if(byte_received)
				data_inbuf <= data_recvd;	//vel2[7:0]
		end
		else if(spibytecnt == 5'b00101) begin	// 5
			data_outbuf <= pos1[15:8];
			if(byte_received)
				vel2 <= {data_recvd,data_inbuf};	//vel2
		end
		else if(spibytecnt == 5'b00110) begin	// 6
			data_outbuf <= pos1[W+F-1:16];
			if(byte_received)
				data_inbuf <= data_recvd;	//vel3[7:0]
		end
		else if(spibytecnt == 5'b00111) begin	// 7
			data_outbuf <= 8'b0;
			if(byte_received)
				vel3 <= {data_recvd,data_inbuf};	//vel3
		end
		//------------------------------------------------- word 2
		else if(spibytecnt == 5'b01000)  begin	// 8
			data_outbuf <= pos2[7:0];
			if(byte_received)
				data_inbuf <= data_recvd;	//real_dout[7:0]
		end
		else if(spibytecnt == 5'b01001) begin	// 9
			data_outbuf <= pos2[15:8];
			if(byte_received) begin
				real_dout <= {data_recvd[5:0],data_inbuf};	//real_dout
			end
		end
		
		else if(spibytecnt == 5'b01010) begin	// 10
			data_outbuf <= pos2[W+F-1:16];
			if(byte_received)
				data_inbuf <= data_recvd;
		end
		else if(spibytecnt == 5'b01011) begin	// 11
			data_outbuf <= 8'b0;
			if(byte_received) begin
				tap <= data_recvd[7:6];
        steptime <= data_recvd[T-1:0];
				Spolarity <= data_inbuf[7];
				dirtime <= data_inbuf[T-1:0];
			end
		end
		
		//------------------------------------------------- word 3
		else if(spibytecnt == 5'b01100) data_outbuf <= pos3[7:0];
		else if(spibytecnt == 5'b01101) data_outbuf <= pos3[15:8];
		else if(spibytecnt == 5'b01110) data_outbuf <= pos3[W+F-1:16];
		else if(spibytecnt == 5'b01111) data_outbuf <= 8'b0;
		//------------------------------------------------- word 4
		else if(spibytecnt == 5'b10000) data_outbuf <= din[7:0];
		else if(spibytecnt == 5'b10001) data_outbuf <= din[15:8];
		else if(spibytecnt == 5'b10010) data_outbuf <= 8'b0;
		else if(spibytecnt == 5'b10011) data_outbuf <= 8'b0;
		else data_outbuf <= spibytecnt;
	end
end
assign LED = do_tristate ? 1'bZ : (real_step[0] ^ real_dir[0]);
assign do_enable_wdt = data_recvd[6] & (spibytecnt == 5'b01001) & byte_received;

endmodule
