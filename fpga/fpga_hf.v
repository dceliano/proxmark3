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

// constants for the different ISO14443a modes:
`define SNIFFER			3'b000
`define TAGSIM_LISTEN	3'b001
`define TAGSIM_MOD		3'b010
`define READER_LISTEN	3'b011
`define READER_MOD		3'b100

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

//-----------------------------------------------------------------------------
// Produce a 16MHz clock, based on pck0, used for overclocking.
//-----------------------------------------------------------------------------
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

//unused
wire hi_read_tx_shallow_modulation = conf_word[0];
//unused
wire hi_read_rx_xcorr_848 = conf_word[0];
//unused
wire hi_read_rx_xcorr_snoop = conf_word[1];
//unused
wire hi_read_rx_xcorr_quarter = conf_word[2];

// For the high-frequency simulated tag: what kind of modulation to use.
wire [2:0] hi_simulate_mod_type = conf_word[2:0];
wire [2:0] mod_type = hi_simulate_mod_type;


//-----------------------------------------------------------------------------
// Begin integrated file:
// ISO14443-A support for the Proxmark III
// Gerhard de Koning Gans, April 2008
//-----------------------------------------------------------------------------
wire osc_clk = pck_clkdiv; //change this to change the clock source.
assign adc_clk = osc_clk;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// internal FPGA timing. Maximum required period is 128 carrier clock cycles for a full 8-bit transfer to ARM. (i.e. we need a 
// 7 bit counter). Adjust its frequency to external reader's clock when simulating a tag or sniffing.
// In normal operation, negedge_cnt counts from 0 to 127, then wraps back to 0
reg [6:0] negedge_cnt;

always @(negedge osc_clk)
begin
	if (negedge_cnt == 7'd127)
	begin
		negedge_cnt <= 0;
	end	
	else
	begin
		negedge_cnt <= negedge_cnt + 1;
	end
end	

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
// First, we determine best possible time for starting/resetting the modulation detector.
// (Our) reader signal changes at negedge_cnt[3:0]=9 (based on when data is received from the SSC).
// The tag response is expected to start 4 ticks later, further delayed by
// 3 clock ticks for ADC conversion. The maximum filter output (edge detected) will be detected after subcarrier zero crossing (+7 ticks).
// To allow some timing variances, we want to have the maximum filter outputs well within the detection window, i.e.
// at mod_detect_reset_time+4 and mod_detect_reset_time+12  (-4 ticks).
// 9 + 4 + 3 + 7 - 4  = 19.    19 mod 16 = 3
wire [3:0] mod_detect_reset_time = 4'd4;

// Modulation detector for the 847kHz (fc / 16) subcarrier.
// Looks for the steepest falling and rising edges within a 16 clock period. If there is both a significant
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
	else // look for steepest edges (slopes)
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
// FPGA <-> ARM clock:
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
// select the data to be sent to the ARM
reg bit_to_arm;
reg sendbit;

always @(negedge osc_clk)
begin
	if(negedge_cnt[3:0] == 4'd0)
	begin
		if (mod_type == `READER_LISTEN)
			sendbit = curbit;
		else
			sendbit = 1'b0;
	end
	bit_to_arm = sendbit;
end

assign ssp_din = bit_to_arm;



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Send the signal out the antenna
// in READER_MOD: drop carrier for mod_sig_coil==1 (pause); in READER_LISTEN: carrier always on
assign pwr_hi = (osc_clk & (((mod_type == `READER_MOD) & ~mod_sig_coil) || (mod_type == `READER_LISTEN)));	



//-----------------------------------------------------------------------------
// Misc pin assignments, including debugging pin
//-----------------------------------------------------------------------------

// In all modes, let the ADC's outputs be enabled.
assign adc_noe = 1'b0;
// Placeholders for LF pins (doesn't matter for HF comms)
assign pwr_oe2 = 1'b0;
assign pwr_lo = 1'b0;
// Permanently enable HF antenna drivers (active low):
assign pwr_oe1 = 1'b0;
assign pwr_oe3 = 1'b0;
assign pwr_oe4 = 1'b0;

assign dbg = curbit;



endmodule

