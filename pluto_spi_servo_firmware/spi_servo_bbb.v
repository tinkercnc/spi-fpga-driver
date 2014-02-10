//    This is a component of pluto_servo_spi, a PWM servo driver and quadrature
//    counter for linuxcnc and the BeagleBoneBlack over SPI.
//    based on the servo.v from Jeff Epler <jepler@unpythonic.net>
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
// Open-Drain buffer
module OC_Buff(in, out);
input in;
output out;
assign out = in ? 1'bz : 1'b0;
endmodule

//**********************************************************************
module pluto_spi_bbb(clk, SCK, MOSI, MISO, SSEL, LED, nConfig, nRESET, quadA, quadB, quadZ, up, down, dout, din);
parameter QW=14;
input clk;

input SCK, SSEL, MOSI, nRESET;
output MISO, nConfig;
output LED;
output [3:0] down = 4'bZZZZ;
output [3:0] up = 4'bZZZZ;

input [7:0] din;
input [3:0] quadA;
input [3:0] quadB;
input [3:0] quadZ;

assign nConfig = nRESET;
//assign nConfig = 1'b1;

reg[9:0] real_dout; 
output [9:0] dout = 10'bZZZZZZZZZZ;
OC_Buff ocout[9:0](real_dout, dout);

wire[3:0] real_down;
OC_Buff ocdown[3:0](real_down, down);
wire[3:0] real_up;
OC_Buff ocup[3:0](real_up, up);

reg Zpolarity;

//**********************************************************************
// PWM stuff
// PWM clock is about 20kHz for clk @ 40MHz, 11-bit cnt
wire pwm_at_top;
reg [10:0] pwmcnt;
wire [10:0] top = 11'd2047;
assign pwm_at_top = (pwmcnt == top);
reg [15:0] pwm0, pwm1, pwm2, pwm3;
always @(posedge clk) begin
    if(pwm_at_top) pwmcnt <= 0;
    else pwmcnt <= pwmcnt + 11'd1;
end

wire [10:0] pwmrev = { 
    pwmcnt[4], pwmcnt[5], pwmcnt[6], pwmcnt[7], pwmcnt[8], pwmcnt[9],
    pwmcnt[10], pwmcnt[3:0]};
wire [10:0] pwmcmp0 = pwm0[14] ? pwmrev : pwmcnt;   // pwm0[14] = pdm/pwm bit
// wire [10:0] pwmcmp1 = pwm1[14] ? pwmrev : pwmcnt;
// wire [10:0] pwmcmp2 = pwm2[14] ? pwmrev : pwmcnt;
// wire [10:0] pwmcmp3 = pwm3[14] ? pwmrev : pwmcnt;
wire pwmact0 = pwm0[10:0] > pwmcmp0;
wire pwmact1 = pwm1[10:0] > pwmcmp0;
wire pwmact2 = pwm2[10:0] > pwmcmp0;
wire pwmact3 = pwm3[10:0] > pwmcmp0;
assign real_up[0] = pwm0[12] ^ (pwm0[15] ? 1'd0 : pwmact0);
assign real_up[1] = pwm1[12] ^ (pwm1[15] ? 1'd0 : pwmact1);
assign real_up[2] = pwm2[12] ^ (pwm2[15] ? 1'd0 : pwmact2);
assign real_up[3] = pwm3[12] ^ (pwm3[15] ? 1'd0 : pwmact3);
assign real_down[0] = pwm0[13] ^ (~pwm0[15] ? 1'd0 : pwmact0);
assign real_down[1] = pwm1[13] ^ (~pwm1[15] ? 1'd0 : pwmact1);
assign real_down[2] = pwm2[13] ^ (~pwm2[15] ? 1'd0 : pwmact2);
assign real_down[3] = pwm3[13] ^ (~pwm3[15] ? 1'd0 : pwmact3);

//**********************************************************************
// Quadrature stuff
// Quadrature is digitized at 40MHz into 14-bit counters
// Read up to 2^13 pulses / polling period = 8MHz for 1kHz servo period
reg qtest;
wire [2*QW:0] quad0, quad1, quad2, quad3;
wire qr0, qr1, qr2, qr3;
//quad q0(clk, qtest ? real_dout[0] : quadA[0], qtest ? real_dout[1] : quadB[0], qtest ? real_dout[2] : quadZ[0]^Zpolarity, qr0, quad0);
quad q0(clk, quadA[0], quadB[0], quadZ[0]^Zpolarity, qr0, quad0);
quad q1(clk, quadA[1], quadB[1], quadZ[1]^Zpolarity, qr1, quad1);
quad q2(clk, quadA[2], quadB[2], quadZ[2]^Zpolarity, qr2, quad2);
quad q3(clk, quadA[3], quadB[3], quadZ[3]^Zpolarity, qr3, quad3);

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
			data_outbuf <= quad0[7:0];
			if(byte_received)
				data_inbuf <= data_recvd;	//pwm0[7:0] 
		end
		else if(spibytecnt == 5'b00001) begin	// 1
			data_outbuf <= quad0[15:8];
			if(byte_received)
				pwm0 <= {data_recvd,data_inbuf};	//pwm0
		end
		else if(spibytecnt == 5'b00010) begin	// 2
			data_outbuf <= quad0[23:16];
			if(byte_received)
				data_inbuf <= data_recvd;	//pwm1[7:0]
		end
		else if(spibytecnt == 5'b00011) begin	// 3
			data_outbuf <= {4'b0, quad0[27:24]};
			if(byte_received)
				pwm1 <= {data_recvd,data_inbuf};	//pwm1
		end
		//------------------------------------------------- word 1
		else if(spibytecnt == 5'b00100) begin	// 4
			data_outbuf <= quad1[7:0];
			if(byte_received)
				data_inbuf <= data_recvd;	//pwm2[7:0]
		end
		else if(spibytecnt == 5'b00101) begin	// 5
			data_outbuf <= quad1[15:8];
			if(byte_received)
				pwm2 <= {data_recvd,data_inbuf};	//pwm2
		end
		else if(spibytecnt == 5'b00110) begin	// 6
			data_outbuf <= quad1[23:16];
			if(byte_received)
				data_inbuf <= data_recvd;	//pwm3[7:0]
		end
		else if(spibytecnt == 5'b00111) begin	// 7
			data_outbuf <= {4'b0, quad1[27:24]};
			if(byte_received)
				pwm3 <= {data_recvd,data_inbuf};	//pwm3
		end
		//------------------------------------------------- word 2
		else if(spibytecnt == 5'b01000)  begin	// 8
			data_outbuf <= quad2[7:0];
			if(byte_received)
				data_inbuf <= data_recvd;	//real_dout[7:0]
		end
		else if(spibytecnt == 5'b01001) begin	// 9
			data_outbuf <= quad2[15:8];
			if(byte_received) begin
				real_dout <= {data_recvd[1:0],data_inbuf};	//real_dout[9:0]
				Zpolarity <= data_recvd[7];	//Zpolarity
				qtest <= data_recvd[5];	//qtest
			end
		end
		else if(spibytecnt == 5'b01010) data_outbuf <= quad2[23:16];	// 10
		else if(spibytecnt == 5'b01011) data_outbuf <= {4'b0, quad2[27:24]};	// 11
		//------------------------------------------------- word 3
		else if(spibytecnt == 5'b01100) data_outbuf <= quad3[7:0];
		else if(spibytecnt == 5'b01101) data_outbuf <= quad3[15:8];
		else if(spibytecnt == 5'b01110) data_outbuf <= quad3[23:16];
		else if(spibytecnt == 5'b01111) data_outbuf <= {4'b0, quad3[27:24]};
		//------------------------------------------------- word 4
		else if(spibytecnt == 5'b10000) data_outbuf <= din;
		else if(spibytecnt == 5'b10001) data_outbuf <= {quadB, quadZ};
		else if(spibytecnt == 5'b10010) data_outbuf <= {4'b0, quadA};
		else if(spibytecnt == 5'b10011) data_outbuf <= 8'b0;
		else data_outbuf <= spibytecnt;
	end
end

assign LED = (real_up[0] ^ real_down[0]);

endmodule
