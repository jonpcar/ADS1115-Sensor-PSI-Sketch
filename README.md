# ADS1115 Sensor PSI Sketch

This "sketch" is dedicated to record FOUR analog sensors attached to an ADS1115 chip.  Ive attached PSI transducers. I've chosen to not use the ADS1115 library and instead flattened the functions I needed into this code.  I only needed two functions provided by the library:

[1] start/configure an A2D conversion on one of the four channels (write_PSIadcConfigRegister) AND 
[2] read the A2D result from a previous A2D conversion (read_PSIadcConversionRegister)

In this code, multiple samples (PSI_SAMPLES_PER_INTERVAL) from each of the ADS1115 channels are taken and averaged before recording an interval's value

A variation of this code will be incorporated into my pool automation project.   My pool controller hardware is controlled by a Particle.io Photon.  Early on I realized that to implement ALL the functionality that I eventually want...I was going to be I/O limited on the Photon.   This is solved by adding a couple chips to my solution.  In this case, I have added one ADS1115 chip which handles FOUR analog signals and attaches to the i2c bus of  the Photon.   I am using that chip to read FOUR PSI measurements from various places in my pool plumbing.

This actually solved 2 problems for me: (1) A2D I/O limitation described earlier, this doesn't use up any of the Photons A2D channels or any additional pins because it is hooked up to the i2c bus of the Photon.  (2) The PSI transducers I chose for this project output a 5V operating range (0.5-5.5V), the Photon is a 3.3V product, although it is 5V tolerant.  However, the analog VREF voltage for the Photon is 3.3V and would have limited my ability to read the full PSI range from my chosen transducers.   So, I now feed 5V directly into the ADS1115 as its VDD (which also is its A2D VREF).   The numbers I get from the PSI transducers look pretty good and now I can accurately sample the full output voltage range that the PSI transducers produce in my system. 
