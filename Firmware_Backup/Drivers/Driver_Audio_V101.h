// Author			: Fabian Kung
// Date				: 24 Sep 2014
// Filename			: Driver_Audio_V101.h

#ifndef _DRIVER_AUDIO_dsPIC33E_H
#define _DRIVER_AUDIO_dsPIC33E_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

//
// --- PUBLIC VARIABLES ---
//
extern INT16 gnAudioTone[9];                    // Set the tone, from 1 to 15.  0 or otherwise will stop the speaker.
                                                // We can have up to 8 different tones.  The last tone setting should
                                                // be 0.  See example of usage for Proce_Audio_Driver.
extern  unsigned int gunAudioVolume;            // From 0 to 5 (loudest).
//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_Audio_Driver(TASK_ATTRIBUTE *);	// Audio driver.
#endif