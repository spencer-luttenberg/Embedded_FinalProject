/*
 * fish_write_read_memory.h
 *
 *  Created on: Dec 8, 2023
 *      Author: Spencer
 */

#ifndef INC_FISH_WRITE_READ_MEMORY_H_
#define INC_FISH_WRITE_READ_MEMORY_H_
#include <string.h>

uint8_t singlePacket;
uint32_t EEPROM_Sector_5 = ((uint32_t)0x08040000);
uint64_t testDump;


/*
songID index begins at 0. Array expected to be
full, and padded with zeros.
Bit format = 0b00000MBT
*/
int writeEEPROM(int songID, uint8_t *array){
	songID = 0;
	uint32_t MEM_Location = EEPROM_Sector_5 + songID * 0xE10; // 0xE10 = 3600 Bytes offset

	if(songID > 71){
		return 1; // Input array too big or songID too big
	}




	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_ERSERR | FLASH_FLAG_PGPERR);
	FLASH_Erase_Sector(FLASH_SECTOR_5, FLASH_VOLTAGE_RANGE_1);


	for(int i = 0; i + 1 <= 3600; i++){
	  HAL_FLASH_Program(FLASH_PROGRAM_BYTE, (MEM_Location), array[i]);
	  MEM_Location = MEM_Location + 0x1;
	}

	HAL_Delay(1000);
	HAL_FLASH_Lock();

	return 0;
}


void handle_incoming_song(uint8_t* incoming_data){
	uint8_t song_id = incoming_data[0];
	uint8_t* initial_song_ptr = &incoming_data[1];
	writeEEPROM(song_id, initial_song_ptr);
}


/*
Input argument arr will be filled with data
songID index begins at 0
Bit format = 0b00000MBT
*/
int readEEPROM(int songID, uint8_t arr[]) {
	// 12 fps * 60 seconds * 5 min = 3600 samples

	if(songID > 71){
		return 1; // songID too big
	}

	uint32_t MEM_Location = EEPROM_Sector_5 + songID * 0xE10; // 0xE10 = 3600 Bytes offset

	for(int i = 0; i <= 3599; i++){
		memcpy(&singlePacket, (void *) (MEM_Location), sizeof(singlePacket));
		arr[i] = singlePacket;
		MEM_Location = MEM_Location + 0x1;
	}

	if((uint8_t)arr[0] != (uint8_t)0b01011000){
		return 1; // checksum failed, song not located in this memory location
	}

    return 0;
}







#endif /* INC_FISH_WRITE_READ_MEMORY_H_ */
