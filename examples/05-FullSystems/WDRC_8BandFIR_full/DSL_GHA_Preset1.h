/*
 *  Here is an alternative configuration for the algorithms.  The intention
 *  is that these settings would be for the "full-on gain" condition for ANSI
 *  testing.  These values shown are place-holders until values can be
 *  tested and improved experimentally
*/

// these values should be probably adjusted experimentally until the gains run right up to 
// the point of the output sounding bad.  The only thing to adjust is the compression-start gain.

//per-band processing parameters...all compression/expansion is defeated.  Just gain is left.
BTNRH_WDRC::CHA_DSL dsl_fullon = {5,  // attack (ms)
  115,     // maxdB.  calibration at 1kHz?  dB SPL for input signal at 0 dBFS.  Needs to be tailored to mic, spkrs, and mic gain.
  0,       // 0=left, 1=right...ignored
  N_CHAN,  // num channels
  {317.1666, 502.9734, 797.6319, 1264.9, 2005.9, 3181.1, 5044.7},   // cross frequencies (Hz)
};

// Here is the broadband limiter for the full-on gain condition.  Only the "bolt" (last value) needs to be iterated.
BTNRH_WDRC::CHA_WDRC gha_fullon = {5.f, // attack time (ms)
  50.0,     // release time (ms)
  24000.0,  // sampling rate (Hz)...ignored.  Set globally in the main program.
  115.0,    // maxdB.  calibration.  dB SPL for signal at 0dBFS.  Needs to be tailored to mic, spkrs, and mic gain.
  1.0,      // compression ratio for lowest-SPL region (ie, the expansion region) (should be < 1.0.  set to 1.0 for linear)
  0.0,      // kneepoint of end of expansion region (set very low to defeat the expansion)
  0.0,      // compression-start gain....set to zero for pure limitter
  130.0,    // compression-start kneepoint...set to some high value to make it not relevant
  10.0,      // compression ratio...set to 1.0 to make linear (to defeat)
  105.0     // limiting threshold...hardwired to compression ratio of 10.0
};
