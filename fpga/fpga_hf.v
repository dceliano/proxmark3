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
	input dbg
);

reg mod_sig_coil;
reg curbit;
reg [3:0] spck_cntr = 4'd0; //counts to 15 and then rolls over to 0

//-----------------------------------------------------------------------------
// Precise timing measurement code
//-----------------------------------------------------------------------------
reg [15:0] db_cycle_count = 16'd0; //a 16-bit cycle counter which will get reported back to the ARM.
reg count_cycles_flag = 1'b0; //used to enable and disable the counter

always @(posedge ck_1356meg) begin //Use the 13.56MHz clock for counting clock cycles right now because 48MHz might overflow the counter. 
	if(spck_cntr == 15) db_cycle_count <= 16'd0; //restart the timestamp. Might cut off a SPI bit at the end...need to double check.
	if(count_cycles_flag == 1'b1) db_cycle_count <= db_cycle_count + 1;
	//else db_cycle_count <= 16'd0;
	if(curbit == 1'b1) count_cycles_flag <= 1'b0; //Take end time stamp here by stopping clock cycle count
	if(mod_sig_coil == 1'b1) count_cycles_flag <= 1'b1; //Take beginning time stamp here by starting clock cycle count. mod_sig_coil is active low.
end

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
// From the ARM:
// Sends a 16 bit command/data pair to the FPGA.
// The bit format is:  C3 C2 C1 C0 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
// where C is the 4 bit command and D is the 12 bit data (the configuration word is bits [D7:D0])
//-----------------------------------------------------------------------------
reg [15:0] mosi_shift_reg; //ARM to FPGA
reg [7:0] conf_word;

always @(posedge ncs) //as soon as we finish receiving SPI data from the ARM (ncs goes high when the data xfer is complete)
begin
	case(mosi_shift_reg[15:12])
		4'b0001: conf_word <= mosi_shift_reg[7:0];		// FPGA_CMD_SET_CONFREG = (1<<12), as defined in the ARM code
	endcase
end

//Receive data from the ARM over SPI and feed it into the shift register bit-by-bit
always @(posedge spck)
begin
	if(~ncs)
	begin
		mosi_shift_reg[15:1] <= mosi_shift_reg[14:0];
		mosi_shift_reg[0] <= mosi;
	end
end

wire [2:0] major_mode;
assign major_mode = conf_word[7:5];

// For the high-frequency simulated tag: what kind of modulation to use.
wire [2:0] hi_simulate_mod_type = conf_word[2:0];
wire [2:0] mod_type = hi_simulate_mod_type;


//-----------------------------------------------------------------------------
// The SPI transmitter. Sends 16 bytes back to the ARM. Currently, the bits are meaningless, but what is received by the ARM should be 1010...1010
// Change the bit on the rising edge of spck because the SPI will read it on the falling edge (because NCPHA = 1 and CPOL = 0). 
//-----------------------------------------------------------------------------
//reg [15:0] miso_shift_reg; //FPGA to ARM
reg miso_sig = 0'b0;
always @(posedge spck)
begin
	miso_sig <= db_cycle_count[15 - spck_cntr]; //send out MSbit first
	spck_cntr <= spck_cntr + 1;
end

assign miso = miso_sig;

//always @(negedge ncs) //beginning a new SPI transmission to the ARM
//begin
//	miso_shift_reg <= 16'hABCD; //the 16 bits we are sending to the ARM
//end


//-----------------------------------------------------------------------------
// Begin integrated file:
// ISO14443-A support for the Proxmark III
// Gerhard de Koning Gans, April 2008
//-----------------------------------------------------------------------------
wire osc_clk = ck_1356meg; //change this to change the clock source.
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
// negedge_cnt[3:0] is a 4-bit counter, so therefore counts from 0 to 15 and then wraps back to 0.
// (Our) reader signal changes at negedge_cnt[3:0]=9.
// The tag response is expected to start 4 ticks later, further delayed by
// 3 clock ticks for ADC conversion. The maximum filter output (edge detected) will be detected after subcarrier zero crossing (+7 ticks).
// To allow some timing variances, we want to have the maximum filter outputs well within the detection window, i.e.
// at mod_detect_reset_time+4 and mod_detect_reset_time+12  (-4 ticks).
// 9 + 4 + 3 + 7 - 4  = 19.    19 mod 16 = 3
wire [3:0] mod_detect_reset_time = 4'd3;

// Modulation detector for the 848kHz (fc / 16) subcarrier.
// Looks for the steepest falling and rising edges within a period of 16 carrier clock cycles. If there is both a significant
// falling and rising edge (in any order), a modulation is detected.
// The output of this modulation detection is curbit (current bit), which eventually gets sent to the ARM.
reg signed [10:0] rx_mod_falling_edge_max;
reg signed [10:0] rx_mod_rising_edge_max;

`define EDGE_DETECT_THRESHOLD	40

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
	else // look for the steepest edges (slopes)
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
// PM3 -> Tag
// assign a modulation signal to the antenna. This signal is undelayed when sending to a tag

always @(negedge osc_clk)
begin
	mod_sig_coil <= ssp_dout;
end


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FPGA <-> ARM clock:
// Generate the ssp clock and ssp frame signal for the synchronous transfer to and from the ARM
// start of transfer/start of frame is indicated by the rise of the ssp_frame signal
// ssp_din changes on the rising edge of the ssp_clk clock and is clocked into the ARM on the falling edge of ssp_clk
// ssp_clk = osc_clk / 16, ssp_frame = osc_clk / 128. That is, we send a bit every 16 clock cycles.
// Example of ssp transfer of binary value 11001010:
//             _______________________________
// ssp_frame__|                               |__
//             _______         ___     ___
// ssp_din  __|       |_______|   |___|   |______
//         _   _   _   _   _   _   _   _   _   _
// ssp_clk  |_| |_| |_| |_| |_| |_| |_| |_| |_| |_
reg ssp_clk;
reg ssp_frame;
assign ssp_clk_actual = ssp_clk;
assign ssp_frame_actual = ssp_frame;

always @(negedge osc_clk)
begin
	begin
		if(negedge_cnt[3:0] == 4'd0)
			ssp_clk <= 1'b1;
		if(negedge_cnt[3:0] == 4'd8) 
			ssp_clk <= 1'b0;
		if(negedge_cnt[6:0] == 7'd7)
			ssp_frame <= 1'b1;
		if(negedge_cnt[6:0] == 7'd23)
			ssp_frame <= 1'b0;
	end	
end
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FPGA -> ARM communication:
// Select the bit to be sent to the ARM.
reg bit_to_arm;
reg sendbit;

always @(negedge osc_clk)
begin
	if(negedge_cnt[3:0] == 4'd0) //negedge_cnt[3:0] counts from 0 to 15. Therefore, we only select a new bit to send to the ARM every 16 carrier clock cycles.
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

//assign dbg = curbit;



endmodule
