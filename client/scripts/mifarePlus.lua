local cmds = require('commands')
local lib14a = require('read14a')

SIXTEEN_BYTES_ZEROS = "00000000000000000000000000000000"

GETVERS_INIT = "0360" -- Begins the GetVersion command
GETVERS_CONT = "03AF" -- Continues the GetVersion command
POWEROFF = "OFF"
WRITEPERSO = "03A8"
COMMITPERSO = "03AA"
AUTH_FIRST = "0370"
AUTH_CONT = "0372"
AUTH_NONFIRST = "0376"
PREPAREPC = "03F0"
PROXIMITYCHECK = "03F2"
VERIFYPC = "03FD"
READPLAINNOMACUNMACED = "0336"

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
		print(("<recvd>: %s"):format(returned_bytes)) -- need to multiply by 2 because the hex digits are actually two bytes when they are strings
		return returned_bytes
	else
		err = "Error sending the card raw data."
		oops(err)
	end
end

function writePerso()
	-- Used to write any data, including the keys (Key A and Key B), for all the sectors.
	-- writePerso() command parameters:
	--		1 byte  - 0xA8 - Command Code
	--		2 bytes - Address of the first block or key to be written to (40 blocks are numbered from 0x0000 to 0x00FF)
	--		X bytes - The data bytes to be written, starting from the first block. Amount of data sent can be from 16 to 240 bytes in 16 byte increments. This allows
	--				up to 15 blocks to be written at once.
	-- response from PICC:
	--		0x90 - OK
	--		0x09 - targeted block is invalid for writes, i.e. block 0, which contains manufacturer data
	--		0x0B - command invalid
	--		0x0C - unexpected command length
	

	-- First, set all the data in the card (4kB of data) to zeros. The keys, stored in the sector trailer block, are also set to zeros.
	-- The only block which cannot be explicitly set is block 0x0000, the manufacturer block.
	print("Setting values of normal blocks")
	for i=1,255,1 do --skip block 0
		--convert the number to hex with leading zeros, then use it as the block number in writeBlock()
		blocknum = string.format("%04x", i)
		writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	end
	print("Finished setting values of normal blocks")

	print("Setting AES Sector keys")
	-- Next, write to the AES sector keys
	for i=0,39 do --for each sector number
		local keyA_block = "40" .. string.format("%02x", i * 2)
		local keyB_block = "40" .. string.format("%02x", (i * 2) + 1)
		--Can also calculate the keys fancily to make them unique, if desired
		keyA = SIXTEEN_BYTES_ZEROS
		keyB = SIXTEEN_BYTES_ZEROS
		writeBlock(keyA_block, keyA)
		writeBlock(keyB_block, keyB)
	end
	print("Finished setting AES Sector keys")

	print("Setting misc keys which haven't been set yet.")
	--CardMasterKey
	blocknum = "9000"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	--CardConfigurationKey
	blocknum = "9001"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	--L3SwitchKey
	blocknum = "9003"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	--SL1CardAuthKey
	blocknum = "9004"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	--L3SectorSwitchKey
	blocknum = "9006"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	--L1L3MixSectorSwitchKey
	blocknum = "9007"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	--VC Keys
	--VCProximityKey
	blocknum = "A001"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	--VCSelectENCKey
	blocknum = "A080"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	--VCSelectMACKey
	blocknum = "A081"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	--TransactionMACKey1
	blocknum = "C000"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	--TransactionMACConfKey1
	blocknum = "C001"
	writeBlock(blocknum, SIXTEEN_BYTES_ZEROS)
	print("Finished setting misc keys.")

	print("WritePerso finished! Card is ready to move into new security level.")
end

function writeBlock(blocknum, data)
	-- Method writes 16 bytes of the string sent (data) to the specified block number
	-- The block numbers sent to the card need to be in little endian format (i.e. block 0x0001 is sent as 0x1000)
	blocknum_little_endian = string.sub(blocknum, 3, 4) .. string.sub(blocknum, 1, 2)
	commandString = WRITEPERSO .. blocknum_little_endian .. data --Write 16 bytes (32 hex chars).
	response = sendRaw(commandString, true, true) --0x90 is returned upon success
	if string.sub(response, 3, 4) ~= "90" then
		oops(("error occurred while trying to write to block %s"):format(blocknum))
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

function commitPerso()
	-- commandString = COMMITPERSO .. "01" --switch to SL1
	commandString = COMMITPERSO .. "03" --switch to SL3
	response = sendRaw(commandString, true, true) --0x90 is returned upon success
	if string.sub(response, 3, 4) ~= "90" then
		oops("error occurred while trying to switch security level")
	end
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
	--Perform PPS (Protocol and Parameter Selection) check to finish the ISO 14443-4 protocol.
	sendRaw("e050", true, true)
	sendRaw("D01100", true, true)
	if err then
		oops(err)
	else
		print(("Connected to card with a UID of %s."):format(info.uid))
	end


	-- Now, the card is initialized and we can do more interesting things.

	--writePerso()
	--commitPerso()
	--getVersion()
	proximityCheck()

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
