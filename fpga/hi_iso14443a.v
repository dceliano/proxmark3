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

module hi_iso14443a(
    pck0, ck_1356meg, ck_1356megb, pck_clkdiv,
    pwr_lo, pwr_hi, pwr_oe1, pwr_oe2, pwr_oe3, pwr_oe4,
    adc_d, adc_clk,
    ssp_frame, ssp_din, ssp_dout, ssp_clk,
    cross_hi, cross_lo,
    dbg,
    mod_type
);
    input pck0, ck_1356meg, ck_1356megb, pck_clkdiv;
    output pwr_lo, pwr_hi, pwr_oe1, pwr_oe2, pwr_oe3, pwr_oe4;
    input [7:0] adc_d;
    output adc_clk;
    input ssp_dout;
    output ssp_frame, ssp_din, ssp_clk;
    input cross_hi, cross_lo;
    output dbg;
    input [2:0] mod_type;


//wire osc_clk = pck0;
wire osc_clk = ck_1356meg;
//wire osc_clk = pck_clkdiv;
wire adc_clk = ck_1356meg;

wire test2 = pck_clkdiv;
wire test3 = pck0;
reg test = 1'b0;
always @(negedge test2)
begin
	test <= ~test;
end
//assign pwr_oe2 = test;
assign dbg = ck_1356meg;



	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// internal FPGA timing. Maximum required period is 128 carrier clock cycles for a full 8 Bit transfer to ARM. (i.e. we need a 
// 7 bit counter). Adjust its frequency to external reader's clock when simulating a tag or sniffing.
reg [6:0] negedge_cnt;

always @(negedge adc_clk)
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
//                                      = (2*input_prev4 + input_prev3) - (2*input + input_prev1)
wire [8:0] input_prev_4_times_2 = input_prev_4 << 1;
wire [8:0] adc_d_times_2                = adc_d << 1;

wire [9:0] tmp1 = input_prev_4_times_2 + input_prev_3;
wire [9:0] tmp2 = adc_d_times_2 + input_prev_1;

// convert intermediate signals to signed and calculate the filter output
wire signed [10:0] adc_d_filtered = {1'b0, tmp1} - {1'b0, tmp2};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tag -> PM3
// sample 4 bits reader data and 4 bits tag data for sniffing
reg [3:0] tag_data;

always @(negedge pck_clkdiv)
begin
    if(negedge_cnt[3:0] == 4'd0)
	begin
		tag_data[3:0] <= {tag_data[2:0], curbit};
	end
end	


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PM3 -> Reader or Tag
// assign a modulation signal to the antenna. This signal is either a delayed signal (to achieve fdt when sending to a reader)
// or undelayed when sending to a tag
reg mod_sig_coil;

always @(negedge osc_clk)
begin
		mod_sig_coil <= ssp_dout;
end


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FPGA -> ARM communication:
// buffer 8 bits data to be sent to ARM. Shift them out bit by bit.
reg [7:0] to_arm;

always @(negedge adc_clk)
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

always @(negedge adc_clk)
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

always @(negedge adc_clk)
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


//assign dbg = negedge_cnt[3];

endmodule



