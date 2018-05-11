local cmds = require('commands')
local lib14a = require('read14a')

SIXTEEN_BYTES_ZEROS = "00000000000000000000000000000000"

GETVERS_INIT = "0360" -- Begins the GetVersion command
GETVERS_CONT = "03AF" -- Continues the GetVersion command
POWEROFF = "OFF"
AUTH_FIRST = "0370"
AUTH_CONT = "0372"
AUTH_NONFIRST = "0376"
PREPAREPC = "03F0"
PROXIMITYCHECK = "03F2"
VERIFYPC = "03FD"
READPLAINNOMACUNMACED = "0336"
CHANGEKEY = "03C4"

--- 
-- This is only meant to be used when errors occur
function oops(err)
	print("ERROR: ",err)
end

---
-- Used to send raw data to the firmware to subsequently forward the data to the card.
function sendRaw(rawdata, crc, power)
	print(("<sent>: 	  %s"):format(rawdata))

	local flags = lib14a.ISO14A_COMMAND.ISO14A_RAW
	if crc then
		flags = flags + lib14a.ISO14A_COMMAND.ISO14A_APPEND_CRC
	end
	if power then
		flags = flags + lib14a.ISO14A_COMMAND.ISO14A_NO_DISCONNECT
	end

	local command = Command:new{cmd = cmds.CMD_READER_ISO_14443a, 
								arg1 = flags, -- Send raw
								arg2 = string.len(rawdata) / 2, -- arg2 contains the length, which is half the length of the ASCII-string rawdata
								data = rawdata}
	local ignore_response = false
	local result, err = lib14a.sendToDevice(command, ignore_response)
	if result then
		--unpack the first 4 parts of the result as longs, and the last as an extremely long string to later be cut down based on arg1, the number of bytes returned
		local count,cmd,arg1,arg2,arg3,data = bin.unpack('LLLLH512',result)
		returned_bytes = string.sub(data, 1, arg1 * 2)
		if returned_bytes ~= "" then
			print(("<recvd>: %s"):format(returned_bytes)) -- need to multiply by 2 because the hex digits are actually two bytes when they are strings
		end
		return returned_bytes
	else
		err = "Error sending the card raw data."
		oops(err)
	end
end

function changeKey(keynum, data)
	-- Method writes 16 bytes of the string sent (data) to the specified 2-byte key number
	-- The block numbers sent to the card need to be in little endian format (i.e. block 0x0001 is sent as 0x1000)
	--blocknum_little_endian = string.sub(blocknum, 3, 4) .. string.sub(blocknum, 1, 2)
	commandString = CHANGEKEY .. keynum .. SIXTEEN_BYTES_ZEROS .. data --Write 16 bytes (32 hex chars).
	response = sendRaw(commandString, true, true) --0x90 is returned upon success
	if string.sub(response, 3, 4) ~= "00" then
		oops(("error occurred while trying to set key %s"):format(keynum))
	end
end

function authenticateAES()
	-- Used to try to authenticate with the AES keys we programmed into the card, to ensure the authentication works correctly.
	commandString = AUTH_FIRST
	commandString = commandString .. ""
end

function getVersion()
	sendRaw(GETVERS_INIT, true, true)
	sendRaw(GETVERS_CONT, true, true)
	sendRaw(GETVERS_CONT, true, true)
end


function calculateMAC(MAC_input)
	-- Pad the input if it is not a multiple of 16 bytes (32 nibbles). 
	if(string.len(MAC_input) % 32 ~= 0) then
		MAC_input = MAC_input .. "80"
	end
	while(string.len(MAC_input) % 32 ~= 0) do
		MAC_input = MAC_input .. "0"
	end
	print("Padded MAC Input = " .. MAC_input .. ", length (bytes) = " .. string.len(MAC_input) / 2)

	--The MAC would actually be calculated here, and the output stored in raw_output
	raw_output = "00010203040506070001020304050607" -- Dummy filler for now of 16-byte output. To be filled with actual MAC for testing purposes.

	-- The final 8-byte MAC output is a concatenation of every 2nd byte starting from the second MSB.
	final_output = ""
	j = 3
	for i = 1,8 do
		final_output = final_output .. string.sub(RndR, j, j + 1) .. string.sub(RndC, j, j + 1)
		j = j + 4
	end
	return final_output
end

function proximityCheck()
	--PreparePC--
	commandString = PREPAREPC
	response = sendRaw(commandString, true, true)
	if(response == "") then
		print("ERROR: This card does not support the Proximity Check command.")
		return
	end
	OPT = string.sub(response, 5, 6)
	if(tonumber(OPT) == 1) then
		pps_present = true
	else
		pps_present = false
	end
	pubRespTime = string.sub(response, 7, 10)
	if(pps_present == true) then
		pps = string.sub(response, 11, 12)
	else
		pps = nil
	end
	print("OPT = " .. OPT .. " pubRespTime = " .. pubRespTime .. " pps = " .. pps)

	--PC--
	RndC = "0001020304050607" --Random Challenge
	num_rounds = 8 --Needs to be 1, 2, 4, or 8
	part_len = 8 / num_rounds
	j = 1
	RndR = ""
	for i = 1,num_rounds do
		pRndC = ""
		for q = 1,(part_len*2) do
			pRndC = pRndC .. string.sub(RndC,j,j)
			j = j + 1
		end
		commandString = PROXIMITYCHECK .. "0" .. tostring(part_len) .. pRndC
		pRndR = string.sub(sendRaw(commandString, true, true), 3, 3+part_len)
		RndR = RndR .. pRndR
	end
	print("RndC = " .. RndC .. " RndR = " .. RndR)

	--VerifyPC--
	MAC_input = "FD" .. OPT .. pubRespTime
	if(pps_present == true) then
		MAC_input = MAC_input .. pps
	end
	rnum_concat = ""
	rnum_concat = RndR .. RndC --temporary (only works for when a single random challenge (8 bytes) is sent)
	-- j = 1
	-- for i = 1,8 do
	-- 	rnum_concat = rnum_concat .. string.sub(RndR, j, j + 1) .. string.sub(RndC, j, j + 1)
	-- 	j = j + 2
	-- end
	MAC_input = MAC_input .. rnum_concat
	print("Concatenation of random numbers = " .. rnum_concat)
	print("Final PCD concatenation before input into MAC function = " .. MAC_input)
	MAC_tag = calculateMAC(MAC_input)
	print("8-byte PCD MAC_tag (placeholder - currently incorrect) = " .. MAC_tag)
	commandString = VERIFYPC .. MAC_tag
	response = sendRaw(commandString, true, true)
	print(response)
	PICC_MAC = string.sub(response, 5, 20)
	print("8-byte MAC returned by PICC = " .. PICC_MAC)
	MAC_input = "90" .. string.sub(MAC_input, 3)
	print("Final PICC concatenation before input into MAC function = " .. MAC_input)
	MAC_tag = calculateMAC(MAC_input)
	print("8-byte PICC MAC_tag (placeholder - currently incorrect) = " .. MAC_tag)

end

---
-- The main entry point
function main(args)
	-- Initialize the card using the already-present read14a library
	info,err = lib14a.read14443a(true, false)
	--Perform RATS and PPS (Protocol and Parameter Selection) check to finish the ISO 14443-4 protocol.
	response = sendRaw("e050", true, true)
	if(response == "") then
		print("No response from RATS.")
	end
	response = sendRaw("D01100", true, true)
	if(response == "") then
		print("No response from PPS check.")
	end
	if err then
		oops(err)
		sendRaw(POWEROFF, false, false)
		return
	else
		print(("Connected to card with a UID of %s."):format(info.uid))
	end

	--initialize the card
	changeKey("21", SIXTEEN_BYTES_ZEROS) --explicitly set VCProximityKey

	--getVersion()
	--proximityCheck()

	--commandString = VERIFYPC .. "186EFDE8DDC7D30B"
	-- MAC = f5180d6e 40fdeae8 e9dd6ac7 bcd3350b
	-- response = sendRaw(commandString, true, true)

	-- attempt to read VCProximityKey at block A001
	-- commandString = READPLAINNOMACUNMACED .. "01A0" .. "01"
	-- response = sendRaw(commandString, true, true)

	-- authenticate with CardConfigurationKey
	-- commandString = AUTH_FIRST .. "0190" .. "00"
	-- response = sendRaw(commandString, true, true)

	-- Power off the Proxmark
	sendRaw(POWEROFF, false, false)



end


main(args) -- Call the main function
