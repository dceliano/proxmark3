//-----------------------------------------------------------------------------
// The FPGA is responsible for interfacing between the A/D, the coil drivers,
// and the ARM. In the low-frequency modes it passes the data straight
// through, so that the ARM gets raw A/D samples over the SSP. In the high-
// frequency modes, the FPGA might perform some demodulation first, to
// reduce the amount of data that we must send to the ARM.
//
// I am not really an FPGA/ASIC designer, so I am sure that a lot of this
// could be improved.
//
// Jonathan Westhues, March 2006
// Added ISO14443-A support by Gerhard de Koning Gans, April 2008
// iZsh <izsh at fail0verflow.com>, June 2014
//-----------------------------------------------------------------------------

/*`include "hi_read_tx.v"
`include "hi_read_rx_xcorr.v"
`include "hi_simulate.v"
`include "hi_iso14443a.v"
`include "hi_sniffer.v"
`include "util.v"
*/
module fpga_hf(
	input spck, output miso, input mosi, input ncs,
	input pck0, input ck_1356meg, input ck_1356megb,
	output pwr_lo, output pwr_hi,
	output pwr_oe1, output pwr_oe2, output pwr_oe3, output pwr_oe4,
	input [7:0] adc_d, output adc_clk, output adc_noe,
	output ssp_frame_actual, output ssp_din, input ssp_dout, output ssp_clk_actual,
	input cross_hi, input cross_lo,
	output dbg
);

//Tie all unused pins low to get rid of errors.
/*assign pwr_lo = 1'b0;
ssign pwr_hi = 1'b0;
assign pwr_oe1 = 1'b0;
assign pwr_oe2 = 1'b0;
assign pwr_oe3 = 1'b0;
assign pwr_oe4 = 1'b0;
assign adc_clk = 1'b0;
assign adc_noe = 1'b0;
assign ssp_frame = 1'b0;
assign ssp_din = 1'b0;
assign ssp_clk = 1'b0;
*/
reg clk1 = 1'b0;
reg clk2 = 1'b0;
wire clk_source = pck0;
always @(posedge clk_source) begin
        clk1 <= ~clk1;
end
always @(negedge clk_source) begin
        clk2 <= ~clk2;
end
wire clk_copy = clk1 ^ clk2; //XOR makes it a copy of the original clock

//Divide the clk_copy (which should be 48MHz) by 3 to produce a 16MHz clock
reg [1:0] pos_count, neg_count;
wire [1:0] r_nxt;
wire pck_clkdiv;

always @(posedge clk_copy)
if (pos_count ==2) pos_count <= 0;
else pos_count<= pos_count +1;

always @(negedge clk_copy)
if (neg_count ==2) neg_count <= 0;
else neg_count<= neg_count +1;

assign pck_clkdiv = ((pos_count == 2) | (neg_count == 2));

assign dbg = pck_clkdiv;


//-----------------------------------------------------------------------------
// The SPI receiver. This sets up the configuration word, which the rest of
// the logic looks at to determine how to connect the A/D and the coil
// drivers (i.e., which section gets it). Also assign some symbolic names
// to the configuration bits, for use below.
//-----------------------------------------------------------------------------

reg [15:0] shift_reg;
reg [7:0] conf_word;

// We switch modes between transmitting to the 13.56 MHz tag and receiving
// from it, which means that we must make sure that we can do so without
// glitching, or else we will glitch the transmitted carrier.
always @(posedge ncs)
begin
	case(shift_reg[15:12])
		4'b0001: conf_word <= shift_reg[7:0];		// FPGA_CMD_SET_CONFREG
	endcase
end

always @(posedge spck)
begin
	if(~ncs)
	begin
		shift_reg[15:1] <= shift_reg[14:0];
		shift_reg[0] <= mosi;
	end
end

wire [2:0] major_mode;
assign major_mode = conf_word[7:5];

// For the high-frequency transmit configuration: modulation depth, either
// 100% (just quite driving antenna, steady LOW), or shallower (tri-state
// some fraction of the buffers)
wire hi_read_tx_shallow_modulation = conf_word[0];

// For the high-frequency receive correlator: frequency against which to
// correlate.
wire hi_read_rx_xcorr_848 = conf_word[0];
// and whether to drive the coil (reader) or just short it (snooper)
wire hi_read_rx_xcorr_snoop = conf_word[1];
// divide subcarrier frequency by 4
wire hi_read_rx_xcorr_quarter = conf_word[2];

// For the high-frequency simulated tag: what kind of modulation to use.
wire [2:0] hi_simulate_mod_type = conf_word[2:0];

//-----------------------------------------------------------------------------
// And then we instantiate the modules corresponding to each of the FPGA's
// major modes, and use muxes to connect the outputs of the active mode to
// the output pins.
//-----------------------------------------------------------------------------
/*
hi_read_tx ht(
	pck0, ck_1356meg, ck_1356megb,
	ht_pwr_lo, ht_pwr_hi, ht_pwr_oe1, ht_pwr_oe2, ht_pwr_oe3, ht_pwr_oe4,
	adc_d, ht_adc_clk,
	ht_ssp_frame, ht_ssp_din, ssp_dout, ht_ssp_clk,
	cross_hi, cross_lo,
	ht_dbg,
	hi_read_tx_shallow_modulation
);

hi_read_rx_xcorr hrxc(
	pck0, ck_1356meg, ck_1356megb,
	hrxc_pwr_lo, hrxc_pwr_hi, hrxc_pwr_oe1, hrxc_pwr_oe2, hrxc_pwr_oe3,	hrxc_pwr_oe4,
	adc_d, hrxc_adc_clk,
	hrxc_ssp_frame, hrxc_ssp_din, ssp_dout, hrxc_ssp_clk,
	cross_hi, cross_lo,
	hrxc_dbg,
	hi_read_rx_xcorr_848, hi_read_rx_xcorr_snoop, hi_read_rx_xcorr_quarter
);

hi_simulate hs(
	pck0, ck_1356meg, ck_1356megb,
	hs_pwr_lo, hs_pwr_hi, hs_pwr_oe1, hs_pwr_oe2, hs_pwr_oe3, hs_pwr_oe4,
	adc_d, hs_adc_clk,
	hs_ssp_frame, hs_ssp_din, ssp_dout, hs_ssp_clk,
	cross_hi, cross_lo,
	hs_dbg,
	hi_simulate_mod_type
);

hi_iso14443a hisn(
	pck0, ck_1356meg, ck_1356megb, pck_clkdiv,
	hisn_pwr_lo, hisn_pwr_hi, hisn_pwr_oe1, hisn_pwr_oe2, hisn_pwr_oe3,	hisn_pwr_oe4,
	adc_d, hisn_adc_clk,
	hisn_ssp_frame, hisn_ssp_din, ssp_dout, hisn_ssp_clk,
	cross_hi, cross_lo,
	hisn_dbg,
	hi_simulate_mod_type
);

hi_sniffer he(
       pck0, ck_1356meg, ck_1356megb,
       he_pwr_lo, he_pwr_hi, he_pwr_oe1, he_pwr_oe2, he_pwr_oe3,       he_pwr_oe4,
       adc_d, he_adc_clk,
       he_ssp_frame, he_ssp_din, ssp_dout, he_ssp_clk,
       cross_hi, cross_lo,
       he_dbg,
       hi_read_rx_xcorr_848, hi_read_rx_xcorr_snoop, hi_read_rx_xcorr_quarter
);

// Major modes:

//   000 --  HF reader, transmitting to tag; modulation depth selectable
//   001 --  HF reader, receiving from tag, correlating as it goes; frequency selectable
//   010 --  HF simulated tag
//   011 --  HF ISO14443-A
//   100 --  HF Snoop
//   111 --  everything off

mux8 mux_ssp_clk		(major_mode, ssp_clk,   ht_ssp_clk,   hrxc_ssp_clk,   hs_ssp_clk,   hisn_ssp_clk,   he_ssp_clk, 1'b0, 1'b0, 1'b0);
mux8 mux_ssp_din		(major_mode, ssp_din,   ht_ssp_din,   hrxc_ssp_din,   hs_ssp_din,   hisn_ssp_din,   he_ssp_din, 1'b0, 1'b0, 1'b0);
mux8 mux_ssp_frame		(major_mode, ssp_frame, ht_ssp_frame, hrxc_ssp_frame, hs_ssp_frame, hisn_ssp_frame, he_ssp_frame, 1'b0, 1'b0, 1'b0);
mux8 mux_pwr_oe1		(major_mode, pwr_oe1,   ht_pwr_oe1,   hrxc_pwr_oe1,   hs_pwr_oe1,   hisn_pwr_oe1,   he_pwr_oe1, 1'b0, 1'b0, 1'b0);
mux8 mux_pwr_oe2		(major_mode, pwr_oe2,   ht_pwr_oe2,   hrxc_pwr_oe2,   hs_pwr_oe2,   hisn_pwr_oe2,   he_pwr_oe2, 1'b0, 1'b0, 1'b0);
mux8 mux_pwr_oe3		(major_mode, pwr_oe3,   ht_pwr_oe3,   hrxc_pwr_oe3,   hs_pwr_oe3,   hisn_pwr_oe3,   he_pwr_oe3, 1'b0, 1'b0, 1'b0);
mux8 mux_pwr_oe4		(major_mode, pwr_oe4,   ht_pwr_oe4,   hrxc_pwr_oe4,   hs_pwr_oe4,   hisn_pwr_oe4,   he_pwr_oe4, 1'b0, 1'b0, 1'b0);
mux8 mux_pwr_lo			(major_mode, pwr_lo,    ht_pwr_lo,    hrxc_pwr_lo,    hs_pwr_lo,    hisn_pwr_lo,    he_pwr_lo, 1'b0, 1'b0, 1'b0);
mux8 mux_pwr_hi			(major_mode, pwr_hi,    ht_pwr_hi,    hrxc_pwr_hi,    hs_pwr_hi,    hisn_pwr_hi,    he_pwr_hi, 1'b0, 1'b0, 1'b0);
mux8 mux_adc_clk		(major_mode, adc_clk,   ht_adc_clk,   hrxc_adc_clk,   hs_adc_clk,   hisn_adc_clk,   he_adc_clk, 1'b0, 1'b0, 1'b0);
mux8 mux_dbg			(major_mode, dbg,       ht_dbg,       hrxc_dbg,       hs_dbg,       hisn_dbg,       he_dbg, 1'b0, 1'b0, 1'b0);
*/
// In all modes, let the ADC's outputs be enabled.
assign adc_noe = 1'b0;

//-----------------------------------------------------------------------------
// ISO14443-A support for the Proxmark III
// Gerhard de Koning Gans, April 2008
//-----------------------------------------------------------------------------

// constants for the different modes:
`define SNIFFER			3'b000
`define TAGSIM_LISTEN	3'b001
`define TAGSIM_MOD		3'b010
`define READER_LISTEN	3'b011
`define READER_MOD		3'b100

wire osc_clk = ck_1356meg;
assign adc_clk = osc_clk;

wire [2:0] mod_type = hi_simulate_mod_type;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tag -> PM3
// filter the input for a tag's signal. The filter box needs the 4 previous input values and is a gaussian derivative filter
// for noise reduction and edge detection.
// store 4 previous samples:
reg [7:0] input_prev_4, input_prev_3, input_prev_2, input_prev_1;

always @(negedge osc_clk)
begin
	input_prev_4 <= input_prev_3;
	input_prev_3 <= input_prev_2;
	input_prev_2 <= input_prev_1;
	input_prev_1 <= adc_d;
end	

// adc_d_filtered = 2*input_prev4 + 1*input_prev3 + 0*input_prev2 - 1*input_prev1 - 2*input
//					= (2*input_prev4 + input_prev3) - (2*input + input_prev1) 
wire [8:0] input_prev_4_times_2 = input_prev_4 << 1;
wire [8:0] adc_d_times_2 		= adc_d << 1;

wire [9:0] tmp1 = input_prev_4_times_2 + input_prev_3;
wire [9:0] tmp2 = adc_d_times_2 + input_prev_1;

// convert intermediate signals to signed and calculate the filter output
wire signed [10:0] adc_d_filtered = {1'b0, tmp1} - {1'b0, tmp2};


	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// internal FPGA timing. Maximum required period is 128 carrier clock cycles for a full 8 Bit transfer to ARM. (i.e. we need a 
// 7 bit counter). Adjust its frequency to external reader's clock when simulating a tag or sniffing.
reg [6:0] negedge_cnt;

always @(negedge osc_clk)
begin
	if (negedge_cnt == 7'd127)						// normal operation: count from 0 to 127
	begin
		negedge_cnt <= 0;
	end	
	else
	begin
		negedge_cnt <= negedge_cnt + 1;
	end
end	



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tag -> PM3:
// determine best possible time for starting/resetting the modulation detector.
// (our) reader signal changes at negedge_cnt[3:0]=9, tag response expected to start n*16+4 ticks later, further delayed by
// 3 ticks ADC conversion. The maximum filter output (edge detected) will be detected after subcarrier zero crossing (+7 ticks).
// To allow some timing variances, we want to have the maximum filter outputs well within the detection window, i.e.
// at mod_detect_reset_time+4 and mod_detect_reset_time+12  (-4 ticks).
// 9 + 4 + 3 + 7 - 4  = 19.    19 mod 16 = 3
wire [3:0] mod_detect_reset_time = 4'd4;

// modulation detector. Looks for the steepest falling and rising edges within a 16 clock period. If there is both a significant
// falling and rising edge (in any order), a modulation is detected.
reg signed [10:0] rx_mod_falling_edge_max;
reg signed [10:0] rx_mod_rising_edge_max;
reg curbit;

`define EDGE_DETECT_THRESHOLD	5

always @(negedge osc_clk)
begin
	if(negedge_cnt[3:0] == mod_detect_reset_time)
	begin
		// detect modulation signal: if modulating, there must have been a falling AND a rising edge
		if ((rx_mod_falling_edge_max > `EDGE_DETECT_THRESHOLD) && (rx_mod_rising_edge_max < -`EDGE_DETECT_THRESHOLD))
				curbit <= 1'b1;	// modulation
			else
				curbit <= 1'b0;	// no modulation
		// reset modulation detector
		rx_mod_rising_edge_max <= 0;
		rx_mod_falling_edge_max <= 0;
	end
	else											// look for steepest edges (slopes)
	begin
		if (adc_d_filtered > 0)
		begin
			if (adc_d_filtered > rx_mod_falling_edge_max)
				rx_mod_falling_edge_max <= adc_d_filtered;
		end
		else
		begin
			if (adc_d_filtered < rx_mod_rising_edge_max)
				rx_mod_rising_edge_max <= adc_d_filtered;
		end
	end

end
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tag -> PM3
// sample 4 bits of reader data 
reg [3:0] tag_data;

always @(negedge osc_clk)
begin
    if(negedge_cnt[3:0] == 4'd0)
	begin
		tag_data[3:0] <= {tag_data[2:0], curbit};
	end
end




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PM3 -> Tag
// assign a modulation signal to the antenna. This signal is undelayed when sending to a tag
reg mod_sig_coil;

always @(negedge osc_clk)
begin
		mod_sig_coil <= ssp_dout;
end


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FPGA -> ARM communication:
// buffer 8 bits data to be sent to ARM. Shift them out bit by bit.
reg [7:0] to_arm;

always @(negedge osc_clk)
begin
	if(negedge_cnt[3:0] == 4'b0000 && mod_type != `SNIFFER)
	begin
		// Don't shift if we just loaded new data, obviously.
		if(negedge_cnt[6:0] != 7'd0)
		begin
			to_arm[7:1] <= to_arm[6:0];
		end
	end
	
end


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FPGA <-> ARM communication:
// generate a ssp clock and ssp frame signal for the synchronous transfer from/to the ARM
reg ssp_clk;
reg ssp_frame;
assign ssp_clk_actual = ssp_clk;
assign ssp_frame_actual = ssp_frame;

always @(negedge osc_clk)
begin
	// (ssp_clk = adc_clk / 16, ssp_frame clock = adc_clk / 128):
	begin
		if(negedge_cnt[3:0] == 4'd0)
			ssp_clk <= 1'b1;
		if(negedge_cnt[3:0] == 4'd8) 
			ssp_clk <= 1'b0;

		if(negedge_cnt[6:0] == 7'd7)	// ssp_frame rising edge indicates start of frame
			ssp_frame <= 1'b1;
		if(negedge_cnt[6:0] == 7'd23)
			ssp_frame <= 1'b0;
	end	
end



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FPGA -> ARM communication:
// select the data to be sent to ARM
reg bit_to_arm;
reg sendbit;

always @(negedge osc_clk)
begin
	if(negedge_cnt[3:0] == 4'd0)
	begin
		// What do we communicate to the ARM
		if (mod_type == `READER_LISTEN)
			sendbit = curbit;
		else
			sendbit = 1'b0;
	end

	bit_to_arm = sendbit;
end




assign ssp_din = bit_to_arm;


// in READER_MOD: drop carrier for mod_sig_coil==1 (pause); in READER_LISTEN: carrier always on; in other modes: carrier always off
assign pwr_hi = (ck_1356megb & (((mod_type == `READER_MOD) & ~mod_sig_coil) || (mod_type == `READER_LISTEN)));	


// Enable HF antenna drivers:
assign pwr_oe1 = 1'b0;
assign pwr_oe3 = 1'b0;

// TAGSIM_MOD: short circuit antenna with different resistances (modulated by mod_sig_coil)
// for pwr_oe4 = 1 (tristate): antenna load = 10k || 33			= 32,9 Ohms
// for pwr_oe4 = 0 (active):   antenna load = 10k || 33 || 33  	= 16,5 Ohms
assign pwr_oe4 = mod_sig_coil & (mod_type == `TAGSIM_MOD);

// This is all LF, so doesn't matter.
assign pwr_oe2 = 1'b0;
assign pwr_lo = 1'b0;



endmodule
