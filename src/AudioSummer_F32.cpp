#include "AudioSummer_F32.h"

void AudioSummer4_F32::update(void) {
  audio_block_f32_t *in, *out=NULL;

  //get the first available channel
  int channel = 0;
  while  (channel < 4) {
	  out = receiveWritable_f32(channel);
	  if (out && flag_useChan[channel]) {
		  //yes, this is an audio block and use it
		  break;
	  } else {
		  //this may or may not be an audio block, but don't use it
		  if (out) AudioStream_F32::release(out);	  
	  }
	  channel++;
  }
  if (!out) return;  //there was no data available.  so exit.
  
  //add in the remaining channels, as available and if enabled
  channel++;
  while  (channel < 4) {
    in = receiveReadOnly_f32(channel);
    if (in) {
		if (flag_useChan[channel]) {
			arm_add_f32(out->data, in->data, out->data, in->length);
		}
		AudioStream_F32::release(in);
	} else {
		//do nothing, this vector is empty
	}
	channel++;
  }

  if (out) {
    AudioStream_F32::transmit(out);
    AudioStream_F32::release(out);
  }
}

/*
void AudioMixer4_F32::update(void) {
  audio_block_f32_t *audio_in[4];
  audio_block_f32_t *audio_out = allocate_f32();
  if (!audio_out) { AudioStream_F32::release(audio_out);return; }  //no memory!

  //get the input data
  for (int channel = 0; channel < 4; channel++) audio_in[channel] = receiveReadOnly_f32(channel);

  //do the actual work
  processData(audio_in, audio_out);
  
  //release the inputs
  for (int channel = 0; channel < 4; channel++) AudioStream_F32::release(audio_in[channel]);
  
  //transmit the results
  AudioStream_F32::transmit(out);
  AudioStream_F32::release(out);

}
*/

//returns true if successful
int AudioSummer4_F32::processData(audio_block_f32_t *audio_in[4], audio_block_f32_t *audio_out) {//audio_in can be read-only as no calculations are in-place
	audio_block_f32_t *in;
	
	//find the first available channel
	int channel = 0;
	bool any_input_data = false;
	while (channel < 4) {
		in = audio_in[channel];
		if (in && flag_useChan[channel]) {
			//yes, this is an audio block.  Copy it to the output
			any_input_data = true;
			for (int i=0; i < (in->length); i++) audio_out->data[i] = in->data[i];
			audio_out->id = in->id;
			audio_out->length = in->length;

			//break out of the while loop			
			break;
		}
	}
	
	if (!any_input_data) return false;  //there was no data available.  so exit.

	//add in the remaining channels, as available and if enabled
	channel++;
	while  (channel < 4) {
		in = audio_in[channel];
		if (in) {
			if (flag_useChan[channel]) {
				arm_add_f32(audio_out->data, in->data, audio_out->data, audio_out->length);
			}
		} else {
			//do nothing, this vector is empty
		}
		channel++;
	}

	return true;
}	



void AudioSummer8_F32::update(void) {
  audio_block_f32_t *in, *out=NULL;

  //get the first available channel
  int channel = 0;
  while  (channel < 8) {
	  out = receiveWritable_f32(channel);
	  if (out && flag_useChan[channel]) {
		  //yes, this is an audio block and use it
		  break;
	  } else {
		  //this may or may not be an audio block, but don't use it
		  if (out) AudioStream_F32::release(out);	  
	  }
	  channel++;
  }
  if (!out) return;  //there was no data available.  so exit.
  
  //add in the remaining channels, as available and if enabled
  channel++;
  while  (channel < 8) {
    in = receiveReadOnly_f32(channel);
    if (in) {
		if (flag_useChan[channel]) {
			arm_add_f32(out->data, in->data, out->data, in->length);
		}
		AudioStream_F32::release(in);
	} else {
		//do nothing, this vector is empty
	}
	channel++;
  }

  if (out) {
    AudioStream_F32::transmit(out);
    AudioStream_F32::release(out);
  }
}

