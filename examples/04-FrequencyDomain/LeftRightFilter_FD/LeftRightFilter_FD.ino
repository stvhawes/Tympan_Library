// LeftRightFilter_FD
//
// Is a stereo version of NoiseReduction_FD (demonstrate noise reduction via frequency domain processing).
// Given an input stream where the left and right channels are physically and acoustically distant, look
// to resolve the common elements into a single mono signal, starting with the common elements in the frequency 
// domain.
//   (a) for a standard geography, looking for arrivals within 0.1/0.2 millis, and/or similar volume.
//   (b) later maybe some kind of persistence of delta model, more overlapping of the buckets
// 
// use the existing FFT/IFFT class as-is b ut two of them.  Assume the update()'s will be in synch, but also count them
// so there can be an alert.
// not sure how to dump a flow without buffer overflow so create an extra output stage gain_R
// add a control to the gui to switch processing on and off (in stages)
//
//
// Created: Chip Audette, OpenAudio, June 2021
// Updates: Stephen Hawes, July 2024
//
// Approach:
//    * Connect a pair of AudioFreqDomainBase and record the incoming FFT
//    * Use a one-step delay to be able to compare left and right signals
//    * Push a different processed signal to the right-AudioFreqDomainBase and thus gain_R 
//
// This example code is in the public domain (MIT License)

#include <Tympan_Library.h>
#include "AudioEffectNoiseReduction_FD_F32.h"  //the local file holding your custom function
#include "SerialManager.h"
#include "State.h"

//set the sample rate and block size
const float sample_rate_Hz = 44100.f;   //24000 or 44117 (or other frequencies in the table in AudioOutputI2S_F32)
const int audio_block_samples = 32;       //for freq domain processing choose a power of 2 (16, 32, 64, 128) but no higher than 128
const int FFT_overlap_factor = 4;         //2 is 50% overlap, 4 is 75% overlap
AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);

//create audio library objects for handling the audio
Tympan                       myTympan(TympanRev::F, audio_settings);   //do TympanRev::D or E or F
AudioInputI2S_F32            i2s_in(audio_settings);                //Digital audio *from* the Tympan AIC.
AudioEffectNoiseReduction_FD_F32_L    noiseReduction_L(audio_settings); //create an example frequency-domain processing block
AudioEffectNoiseReduction_FD_F32_R    noiseReduction_R(audio_settings); //create an example frequency-domain processing block
AudioEffectGain_F32          gain_L(audio_settings);                //Applies digital gain to audio data.
AudioEffectGain_F32          gain_R(audio_settings);                //Applies digital gain to audio data.
AudioOutputI2S_F32           i2s_out(audio_settings);               //Digital audio *to* the Tympan AIC.

//Make all of the audio connections
AudioConnection_F32       patchCord1(i2s_in, 0, noiseReduction_L, 0);  //connect the Left input to the left-algorithm
AudioConnection_F32       patchCord2(i2s_in, 1, noiseReduction_R, 1);  //connect the Right input to our right-algorithm
AudioConnection_F32       patchCord5(noiseReduction_L, 0, gain_L, 0);   //connect the left-algorithm to left basic gain
AudioConnection_F32       patchCord6(noiseReduction_R, 1, gain_R, 1);   //connect the right-algorithm to right basic gain
AudioConnection_F32       patchCord11(gain_L, 0, i2s_out, 0);         //connect the left gain to the left output
AudioConnection_F32       patchCord12(gain_R, 1, i2s_out, 1);         //connect the right gain to the right output

//Create BLE
#define USE_BLE (true)
const bool use_ble = USE_BLE;
BLE& ble = myTympan.getBLE();   //myTympan owns the ble object, but we have a reference to it here

//control display and serial interaction
SerialManager serialManager(&ble);
State myState(&audio_settings, &myTympan);

//set up the serial manager
void setupSerialManager(void) {
  //register all the UI elements here
  serialManager.add_UI_element(&myState);

  //create the GUI (but don't transmit it yet.  wait for the requuest from the phone.)
  serialManager.createTympanRemoteLayout();
}


//inputs and levels
float default_mic_input_gain_dB = 20.0f; //gain on the microphone
void switchToPCBMics(void) {
  myTympan.println("Switching to PCB Mics.");
  myTympan.inputSelect(TYMPAN_INPUT_ON_BOARD_MIC); // use the microphone jack - defaults to mic bias OFF
  setInputGain_dB(default_mic_input_gain_dB);
}
void switchToLineInOnMicJack(void) {
  myTympan.println("Switching to Line-in on Mic Jack.");
  myTympan.inputSelect(TYMPAN_INPUT_JACK_AS_LINEIN); // use the microphone jack - defaults to mic bias OFF
  setInputGain_dB(0.0);
}
void switchToMicInOnMicJack(void) {
  myTympan.println("Switching to Mic-In on Mic Jack.");
  myTympan.inputSelect(TYMPAN_INPUT_JACK_AS_MIC); // use the microphone jack - defaults to mic bias OFF
  myTympan.setEnableStereoExtMicBias(true);  //put the mic bias on both channels
  setInputGain_dB(default_mic_input_gain_dB);
}

// define the setup() function, the function that is called once when the device is booting
void setup() {
  //begin the serial comms (for debugging)
  myTympan.beginBothSerial();delay(1500);
  Serial.println("LeftRightFilter_FD: starting setup()...");
  Serial.print("    : sample rate (Hz) = ");  Serial.println(audio_settings.sample_rate_Hz);
  Serial.print("    : block size (samples) = ");  Serial.println(audio_settings.audio_block_samples);

  // Allocate working memory for audio
  AudioMemory_F32(30, audio_settings);

  // Configure the FFT parameters algorithm
  int N_FFT = audio_block_samples * FFT_overlap_factor;//set to 2 or 4...which yields 50% or 75% overlap 
  Serial.print("    : N_FFT = "); Serial.println(N_FFT);
  noiseReduction_L.setup(audio_settings, N_FFT); //do after AudioMemory_F32();
  noiseReduction_R.setup(audio_settings, N_FFT); //do after AudioMemory_F32();

  //configure the noise reduction
  set_NR_enable(myState.NR_enable);
  set_NR_attack_sec(myState.NR_attack_sec); 
  set_NR_release_sec(myState.NR_release_sec);
  set_NR_max_atten_dB(myState.NR_max_atten_dB);
  set_NR_SNR_at_max_atten_dB(myState.NR_SNR_at_max_atten_dB);
  set_NR_transition_width_dB(myState.NR_transition_width_dB);
  set_NR_gain_smoothing_sec(myState.NR_gain_smooth_sec);
  set_NR_enable_noise_est_updates(myState.NR_enable_noise_est_updates);

  //configure the LR filtering
  set_LR_enable(myState.LR_enable);

 //Enable the Tympan to start the audio flowing!
  myTympan.enable(); // activate AIC

  //setup DC-blocking highpass filter running in the ADC hardware itself
  float cutoff_Hz = 60.0;  //set the default cutoff frequency for the highpass filter
  myTympan.setHPFonADC(true, cutoff_Hz, audio_settings.sample_rate_Hz); //set to false to disble

  //Choose the desired input
  switchToPCBMics();        //use PCB mics as input
  //switchToMicInOnMicJack(); //use Mic jack as mic input (ie, with mic bias)
  //switchToLineInOnMicJack();  //use Mic jack as line input (ie, no mic bias)

  //Set the desired volume levels
  setDigitalGain_dB(myState.digital_gain_dB);
  setOutputGain_dB(myState.output_gain_dB);    // headphone amplifier.  -63.6 to +24 dB in 0.5dB steps.
 
  // configure the blue potentiometer
  servicePotentiometer(millis(),0);  //update based on the knob setting the "0" is not relevant here.

  //setup BLE
  #if USE_BLE
		myTympan.setupBLE(); delay(500); //Assumes the default Bluetooth firmware. You can override!
  #endif

  //finish the setup by printing the help menu to the serial connections
  setupSerialManager();
  serialManager.printHelp();
}


// define the loop() function, the function that is repeated over and over for the life of the device
void loop() {

  //respond to Serial commands
  while (Serial.available()) serialManager.respondToByte((char)Serial.read());   //USB Serial

  //respond to BLE
  #if USE_BLE
    if (ble.available() > 0) {
      String msgFromBle; int msgLen = ble.recvBLE(&msgFromBle);
      for (int i = 0; i < msgLen; i++) serialManager.respondToByte(msgFromBle[i]); //ends up in serialManager.processCharacter()
    }
  
    //If there is no BLE connection, make sure that we keep advertising
    ble.updateAdvertising(millis(), 5000); //check every 5000 msec
  #endif

  //check the potentiometer
  servicePotentiometer(millis(), 100); //service the potentiometer every 100 msec

  //periodically print the CPU and Memory Usage
  if (myState.flag_printCPUandMemory) myState.printCPUandMemory(millis(), 3000); //print every 3000 msec
  if (myState.flag_printCPUandMemory) myState.printCPUtoGUI(millis(), 3000);


} //end loop();


// ///////////////// Servicing routines

//servicePotentiometer: listens to the blue potentiometer and sends the new pot value
//  to the audio processing algorithm as a control parameter
void servicePotentiometer(unsigned long curTime_millis, unsigned long updatePeriod_millis) {
  //static unsigned long updatePeriod_millis = 100; //how many milliseconds between updating the potentiometer reading?
  static unsigned long lastUpdate_millis = 0;
  static float prev_val = 0;

  //has enough time passed to update everything?
  if (curTime_millis < lastUpdate_millis) lastUpdate_millis = 0; //handle wrap-around of the clock
  if ((curTime_millis - lastUpdate_millis) > updatePeriod_millis) { //is it time to update the user interface?

    //read potentiometer
    float val = float(myTympan.readPotentiometer()) / 1023.0; //0.0 to 1.0
    val = (1.0/15.0) * (float)((int)(15.0 * val + 0.5)); //quantize so that it doesn't chatter...0 to 1.0

    //send the potentiometer value to your algorithm as a control parameter
    if (abs(val - prev_val) > 0.05) { //is it different than before?
      prev_val = val;  //save the value for comparison for the next time around

      #if 0
        //change the volume
        float gain_dB = 0.f + 30.0f * ((val - 0.5) * 2.0); //set volume as 0dB +/- 30 dB
        myTympan.print("Changing output volume to = "); myTympan.print(gain_dB); myTympan.println(" dB");
        setOutputGain_dB(gain_dB);
        #if USE_BLE
              serialManager.setOutputGainButtons();
        #endif
      #else
        //use the potentiometer to set the freq-domain low-pass filter
        const float min_val = 0, max_val = 40.0; //set desired range
        float new_value = min_val + (max_val - min_val)*val;
        noiseReduction_L.setMaxAttenuation_dB(new_value);
        noiseReduction_R.setMaxAttenuation_dB(new_value);
        Serial.print("servicePotentiometer: max attenuation = "); Serial.println(new_value); //print text to Serial port for debugging
      #endif
    }
    lastUpdate_millis = curTime_millis;
  } // end if
} //end servicePotentiometer();


void printGainSettings(void) {
  myTympan.print("Gain (dB): ");
  myTympan.print("  Input Gain = "); myTympan.println(myState.input_gain_dB);
  myTympan.print("  Digital Gain = "); myTympan.println(myState.digital_gain_dB, 1);
  myTympan.print("  Output Gain = "); myTympan.println(myState.output_gain_dB);
}


// ////////////////////////////// Functions to set the parameters and maintain the state

// // Gains
float setInputGain_dB(float gain_dB) { return myState.input_gain_dB = myTympan.setInputGain_dB(gain_dB); }
float setOutputGain_dB(float gain_dB) {  return myState.output_gain_dB = myTympan.volume_dB(gain_dB); }
void incrementDigitalGain(float increment_dB) { setDigitalGain_dB(myState.digital_gain_dB + increment_dB); }
void setDigitalGain_dB(float gain_dB) {  gain_L.setGain_dB(gain_dB); myState.digital_gain_dB = gain_R.setGain_dB(gain_dB); }

// // Noise reduction parameters
bool set_NR_enable(bool val) { 
  noiseReduction_L.enable(val);
  return myState.NR_enable = noiseReduction_R.enable(val); 
}
float increment_NR_attack_sec(float incr_fac) { return set_NR_attack_sec(myState.NR_attack_sec * incr_fac); }
float set_NR_attack_sec(float val_sec) { 
  noiseReduction_L.setAttack_sec(val_sec);
  myState.NR_attack_sec = noiseReduction_R.setAttack_sec(val_sec);
  //Serial.println("set_NR_attack_sec: given = " + String(val_sec) ", final = " + String(myState.NR_attack_sec))
  return myState.NR_attack_sec;
}
float increment_NR_release_sec(float incr_fac) { return set_NR_release_sec(myState.NR_release_sec * incr_fac); }
float set_NR_release_sec(float val_sec) { 
  noiseReduction_L.setRelease_sec(val_sec); 
  myState.NR_release_sec = noiseReduction_R.setRelease_sec(val_sec); 
  //Serial.println("set_NR_attack_sec: given = " + String(val_sec) ", final = " + String(myState.NR_attack_sec))
  return myState.NR_release_sec;
}
float increment_NR_max_atten_dB(float incr_dB) { return set_NR_max_atten_dB(myState.NR_max_atten_dB + incr_dB); }
float set_NR_max_atten_dB(float val_dB) { 
  noiseReduction_L.setMaxAttenuation_dB(val_dB); 
  return myState.NR_max_atten_dB = noiseReduction_R.setMaxAttenuation_dB(val_dB); 
}
float increment_NR_SNR_at_max_atten_dB(float incr_dB) { return set_NR_SNR_at_max_atten_dB(myState.NR_SNR_at_max_atten_dB + incr_dB); }
float set_NR_SNR_at_max_atten_dB(float val_dB) { 
  noiseReduction_L.setSNRforMaxAttenuation_dB(val_dB); 
  return myState.NR_SNR_at_max_atten_dB = noiseReduction_R.setSNRforMaxAttenuation_dB(val_dB); 
}
float increment_NR_transition_width_dB(float incr_dB) { return set_NR_transition_width_dB(myState.NR_transition_width_dB + incr_dB); }
float set_NR_transition_width_dB(float val_dB) { 
  noiseReduction_L.setTransitionWidth_dB(val_dB); 
  return myState.NR_transition_width_dB = noiseReduction_R.setTransitionWidth_dB(val_dB); 
}
float increment_NR_gain_smoothing_sec(float incr_fac) { return set_NR_gain_smoothing_sec(myState.NR_gain_smooth_sec * incr_fac); }
float set_NR_gain_smoothing_sec(float val_sec) {
  noiseReduction_L.setGainSmoothing_sec(val_sec);
  return myState.NR_gain_smooth_sec = noiseReduction_R.setGainSmoothing_sec(val_sec);
}
bool set_NR_enable_noise_est_updates(bool val) { 
  noiseReduction_L.setEnableNoiseEstimationUpdates(val); 
  return myState.NR_enable_noise_est_updates = noiseReduction_R.setEnableNoiseEstimationUpdates(val); 
}

// // LR Filter parameters
bool set_LR_enable(bool val) { 
  noiseReduction_L.LR_enable(val);
  return myState.LR_enable = noiseReduction_R.LR_enable(val); 
}