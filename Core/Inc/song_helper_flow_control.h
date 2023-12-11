/*
 * song_helper_flow_control.h
 *
 *  Created on: Dec 10, 2023
 *      Author: kelly
 */

#pragma once
#ifndef INC_SONG_HELPER_FLOW_CONTROL_H_
#define INC_SONG_HELPER_FLOW_CONTROL_H_
#include "saved_songs.h"
#include "global_fish_defines.h"

extern UART_HandleTypeDef huart6;
extern uint8_t current_song;
extern uint8_t* current_song_data;
extern uint8_t* currently_loaded_songs[MAX_FISH_SONGS];
extern uint8_t current_fish_state;
extern uint16_t i_play;
extern uint8_t initial_start_playing;

void loadSongsIntoMemory(){
	currently_loaded_songs[0] = loaded_songs[0];
	currently_loaded_songs[1] = loaded_songs[1];
	currently_loaded_songs[2] = loaded_songs[2];
	//currently_loaded_songs = loaded_songs;
}


int playSong() {
	// gather memory


	i_play = 0;
	initial_start_playing = 1;

	current_song_data = currently_loaded_songs[current_song];
	//load into memory current song

	// current state is playing
	current_fish_state = FISH_PLAYING_STATE;

	return 0;
}


int stopSong() {
	// Stop playing song, return to idle mode
	uint8_t dataTemp[] = "q";
	HAL_UART_Transmit (&huart6, dataTemp, sizeof (dataTemp), 10);

//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
//	HAL_Delay(100);
//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
//	HAL_Delay(300);
//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
//	HAL_Delay(100);

	// iterate song
	current_song++;
	if(current_song > MAX_FISH_SONGS-1){
		current_song = 0;
	}

	// reset play counter
	i_play = 0;

	// current state is paused
	current_fish_state = FISH_PAUSED_STATE;
	reset_fish_motors();
	return 0;
}




#endif /* INC_SONG_HELPER_FLOW_CONTROL_H_ */
