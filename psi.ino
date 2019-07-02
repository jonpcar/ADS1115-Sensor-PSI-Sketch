
#include <Particle.h>
//#include <tgmath.h>    //currently not needed

/* 
This "sketch" is dedicated to record FOUR analog sensors attached to an ADS1115 chip.  Ive attached PSI transducers. I've chosen to not use the ADS1115 
library and instead flattened the functions I needed into this code.  I only needed two functions provided by the library:  
[1] start/configure an A2D conversion on one of the four channels (write_PSIadcConfigRegister) AND 
[2] read the A2D result from a previous A2D conversion (read_PSIadcConversionRegister).

In this code, multiple samples (PSI_SAMPLES_PER_INTERVAL) from each of the ADS1115 channels are taken and averaged before recording an interval's value 

A variation of this code will be incorporated into my pool automation project.   My pool controller hardware is controlled by a Particle.io Photon.
Early on I realized that to implement ALL the functionality that I eventually want...I was going to be I/O limited on the Photon.   This is solved
by adding a couple chips to my solution.  In this case, I have added one ADS1115 chip which handles FOUR analog signals and attaches to the i2c bus of 
the Photon.   I am using that chip to read FOUR PSI measurements from various places in my pool plumbing.

This actually solved 2 problems for me: (1) A2D I/O limitation described earlier, this doesn't use up any of the Photons A2D channels or any additional
pins because it is hooked up to the i2c bus of the Photon.  (2) The PSI transducers I chose for this project output a 5V operating range (0.5-5.5V),
the Photon is a 3.3V product, although it is 5V tolerant.  However, the
analog VREF voltage for the Photon is 3.3V and would have limited my ability to read the full PSI range from my chosen transducers.   So,
I now feed 5V directly into the ADS1115 as its VDD (which also is its A2D VREF).   The numbers I get from the
PSI transducers look pretty good and now I can accurately sample the full output voltage range that the PSI transducers produce in my system. 
*/

#define PSI_ADS1115_I2C_ADDRESS   0x48    // i2c base address of the PSI ADS1115 chip in my pool controller, chip must be "hardwired" to this value
// Following is the command for PSI ADS1115 configuration.  It is identical for all four PSI samplings with the exception of the 2-bits in the 
// multiplexer select field which selects one of the four analog inputs.
// See the "ADS1115 Data Sheet" for a more complete description of these bit values...
// bit 15 = start conversion(1), bit 14:12 = input multiplexer field...compare to GND (base 100,101,110,111) , bit 11:9 = Programmable Gain Amplifier (2/3 Mode=000),
// bit 8 = conversion mode(Single Shot=1),  bit 7:5 = Data Rate (860 samples per second=111), bit 4 = compare mode (traditional mode = 0),
// bit 3 = comparator polarity (don't Care for this project =0), bit 2 = comparator latch (don't care for this project = 0),
// bit 1:0 = comparator queue and disable (Disable Comparator=11)
// Resulting PSI_ADS1115_START_COMMAND value: 1+1xx/000+1/111+0/0+0+11 = 0xc1e3 (AD0), 0xd1e3 (AD1), 0xe1e3 (AD2), 0xf1e3 (AD3)
#define PSI_ADS1115_START_COMMAND   0xc1e3  // see explanation above, only bits 13:12 will change depending on which A2D input will be started/read
#define ADS1115_REG_CONVERSION      0x00    // conversion register address for all ADS1115 devices
#define ADS1115_REG_CONFIG          0x01    // configuration register address for all ADS1115 devices
#define PSI_SAMPLE_INTERVAL         1000    // defines the PSI Sampling Interval, which will repeatedly be rescheduled
                                            // .....should be greater than: PSI_SAMPLES_PER_INTERVAL * PSI_CONVERSION_TIME * 4 (# of Sensors) + pad(tbd)
                                            // .....but an "interval check" in the rescheduling should handle this issue if this is "violated"
#define PSI_SAMPLES_PER_INTERVAL      30    // defines the number of PSI samples from each sensor and then averaged during each PSI_SAMPLE_INTERVAL
#define PSI_CONVERSION_TIME	           2    // allowed time for PSI A2D conversion (and to start the next), at 860 samples per second, must be a minimum 1ms
                                            // .....can be increased to "space out" samples within the PSI_SAMPLE_INTERVAL
#define PSI_ADS1115_VOLT_RESOLUTION  0.1875F  // in my system, this is the corresponding voltage (mv) for each bit of ADS1115 A2D resolution 

#define PSI_PUMP_VACUUM_OFFSET    -0.4F     // offset-callibration for sensor connected to ADS1115 AD0, Pool Pump: Vacuum Side
#define PSI_PUMP_PRESSURE_OFFSET  -0.12F    // offset-callibration for sensor connected to ADS1115 AD1, Pool Pump: Pressure Side
#define PSI_FILTER_OFFSET          1.16F    // offset-callibration for sensor connected to ADS1115 AD2, Filter
#define PSI_IFCS_OFFSET            0.72F    // offset-callibration for sensor connected to ADS1115 AD3, In Floor Cleaning System manifold


const float PSI_SENSOR_OFFSETS[4] = 
    {PSI_PUMP_VACUUM_OFFSET,      // experimentally measured offsets to correct/calibrate the phsical readings of my PSI transducers
     PSI_PUMP_PRESSURE_OFFSET,    // ...these offsets are simply a value to make 0 psi readings accurate (they read ~0)
     PSI_FILTER_OFFSET,           // 
     PSI_IFCS_OFFSET};            // 


//Publishing Definitions and variables
#define PUBLISH_MAX_INTERVAL       10000    // every 10 seconds  ...(these values change continuously as I am testing my system...this is 60 seconds)
#define PUBLISH_MIN_INTERVAL       5000     // every 1 second (currently 5 seconds)
unsigned long currentMillis;
bool publishNOW;                            // a particular function may request an immediate status publish by making this "true"


//Publishing Parameters for the PSI
float f_current_psi_pub[4];       // last published value of the PSIs
float f_current_psi[4];           // PSIs from most recently finished PSI sampling interval
float f_psi_min[4];               // used to store min and max values of PSIs that occur between PUBLISHED values (hi-lo values during that period)
float f_psi_max[4];               // ....not sure exactly what I will use these for yet, just testing it
int current_psis_raw[4];          // contains the current (latest) completed psi samples in raw format



// Function declarations
void write_PSIadcConfigRegister(uint16_t value);
int16_t read_PSIadcConversionRegister();
bool publishAllStatus();
bool publishNonBlocking(const char sheet_name, const char message);
bool PsiIntervalSamplingComplete();
void doPsiCalculations();
void publishData();
bool timeToPublish();


void setup() 
{
  Serial.begin(9600);
  Wire.begin();   // initialize the i2c bus
}


void loop() 
{
  currentMillis = millis();

  // Publish the status if conditions are met
  if (timeToPublish()) publishData();

  if (PsiIntervalSamplingComplete()) doPsiCalculations();
}


// function that publishes selected data...this will be expanded
void publishData(){
  if (publishAllStatus()) {     // function attempts to publish the status
    publishNOW = false;         // ...if successful then get ready for next publish
    for (uint8_t i=0; i<4; i++) {
      f_current_psi_pub[i] = f_current_psi[i];  // update the published values
      f_psi_max[i] = -14.7;                     // reset the min/max's for the publish interval
      f_psi_min[i] = 50;  
    }
    //other stuff to be added here
  }
}


// function to check if it is time to Publish: either forced (publishNOW) or a timeout of the PUBLISH_MAX_INTERVAL
// and then setup for the next publish event
bool timeToPublish() {
  static long prior_publish_time;      
  if (((currentMillis - prior_publish_time >= PUBLISH_MIN_INTERVAL) && publishNOW) ||
      (currentMillis - prior_publish_time >= PUBLISH_MAX_INTERVAL)) {
    prior_publish_time = currentMillis;                    // setup for the next publish time
    //publishNOW = false;

    return(true);
  }
  return(false);
}



  /*
  // The following code does analog sampling of the PSI transducers attached to the ADS1115 chip.  Currently, one ADS1115 chip is used in this
  // project.  All four channels are dedicated to PSI measurements throughout the system.   This makes it easy to
  // combine the code for all readings (which are similar) into one routine.  
  //
  // When an ADS1115 A2D conversion is started, the start time is recorded as a marker to know when the next can be started (PSI_CONVERSION_TIME)
  // The four analog conversions (corresponding to my
  // pool's four pressure sensors) are started and then read one by one and stored.   Based on the desired PSI_SAMPLES_PER_INTERVAL, multiple
  // readings are repeated and accumulated in the psi_accumulators.  At the end of an interval, the accumulated PSI readings are divided by 
  // the number of samples in each accumulator (PSI_SAMPLES_PER_INTERVAL) to obtain an average reading for the sampling interval
  //
  // The PSI A2D readings are controlled using the same command (with the exception of the channel selection bits).  
  // One-shot, single ended A2D readings are taken.  Each conversion are started individually by code, and the results are
  // read individually by the code.  Conversions are done at the fastest sampling speed (860 samples per second).  
  // 
  */
bool PsiIntervalSamplingComplete() {
  static unsigned long prior_psi_a2d_start = 0, prior_psi_interval_start = 0, current_psi_interval_start = 0;
  static int psi_accumulator[4];  // accumulators for the PSI raw samples, divide by PSI_SAMPLES_PER_INTERVAL to get an average reading for each interval
  static uint8_t psi_sample_count, psi_pntr;
  static bool psi_conversion_started;

  // Enter this code body if within a valid PSI sampling interval window AND any prior PSI a2d conversion has been completed
  if (((currentMillis - prior_psi_a2d_start) >= PSI_CONVERSION_TIME)  && ((currentMillis - prior_psi_interval_start) >= PSI_SAMPLE_INTERVAL)) {
        // 1) start a PSI sampling conversion 2) read a sampled PSI conversion 3) sampling for the interval is complete
    if ((!psi_conversion_started) && (psi_sample_count < PSI_SAMPLES_PER_INTERVAL)) {      
        // starts a PSI a2d conversion on the appropriate channel by bitwise ORing in channel (AD3-AD0) from current psi_pntr           
      write_PSIadcConfigRegister(PSI_ADS1115_START_COMMAND | (psi_pntr << 12)); 
      prior_psi_a2d_start = millis();                       // capture the start time for completion check reference
      psi_conversion_started = true;
      if ((psi_sample_count == 0) && (psi_pntr == 0 ))      // checks if this is the VERY FIRST conversion for the PSI sampling interval
        current_psi_interval_start = prior_psi_a2d_start;   // ...if so, record interval start time so that the next interval can be scheduled later
    }
    else if (psi_conversion_started) {
      psi_accumulator[psi_pntr] += read_PSIadcConversionRegister();    // accumulates the result of the previously started PSI a2d conversion
      psi_conversion_started = false;
      if (++psi_pntr >= 4) {  //  advance pointer to next analog channel of ADS1115
        psi_pntr = 0;         //  ...reset to 0 if 4+, there are only four channels 0:3
        psi_sample_count++;   //  increment sample count for this PSI interval, all four channels have been sampled and values read/accumulated
      }
    }
    else {   // once IP gets here, all samples have been completed, so setup for the next PSI sample interval and return "true" for FINISHED
      for (uint8_t i = 0; i < 4; i++) {
        current_psis_raw[i] = psi_accumulator[i] / PSI_SAMPLES_PER_INTERVAL;   // simple integer truncate divide, no real need for floating point
        //Serial.printlnf("%01d PSI RAW %05d", i, current_psis_raw[i]);
        psi_accumulator[i] = 0;
      }                        
      psi_sample_count = 0;   
      prior_psi_interval_start =    // just in case the sampling during the sample interval was held up or exceeded the PSI_SAMPLE_INTERVAL period
        ((currentMillis - PSI_SAMPLE_INTERVAL) > current_psi_interval_start) ? currentMillis : current_psi_interval_start;
      return(true);
    }
  }
  return(false);
}


void doPsiCalculations() {
  // All conversion for the PSI interval have been completed... 
  //
  // For my sensors:
  // Vs (from sensor to ADS1115 pin) = analogRead (of the ADS1115 sensor) * 
  //                          PSI_ADS1115_VOLT_RESOLUTION (in mv) / 1000
  //  
  // ...my sensors measure from -14.7PSI to 50PSI (total PSI range of 64.7)
  // ...my sensors' voltage output is .5V - 5.5V (total Voltage range of 5.0V)
  // ...my sensors' have an offset voltage of 0.5 (the lowest voltage output for -14.7 PSI)
  // PSI = (PSI range of sensor) / (voltage range of sensor) * [Vs - (Voffset of sensor)]
  //     = (64.7) / (5) * [Vs - 0.5]
  //
  // To normalize for ambient air pressure (in Gilbert, AZ where my pool is), 14.05 must be subtracted 
  //        (Gilbert is not at sea level)
  // Finally, an offset is needed to calibrate physical readings from the 
  //        imperfect sensor (calibration measurements were taken at 0psi)   
  // TODO: possibly a dynamic calibration at startup or other 'pump off' conditons when PSI should read 0

  for (uint8_t i = 0; i < 4; i++) {       // actual calculations to determine PSI
    f_current_psi[i] = ((current_psis_raw[i] * PSI_ADS1115_VOLT_RESOLUTION / 1000-.5) * 64.7/5) 
                          - 14.05 + PSI_SENSOR_OFFSETS[i]; 
    if (f_current_psi[i] < f_psi_min[i]) f_psi_min[i] = f_current_psi[i];       // keep a min and max per "publish period"
    else if (f_current_psi[i] > f_psi_max[i]) f_psi_max[i] = f_current_psi[i];  // .....don't know exactly what I will use this for yet
  }
     
/*
  // the following code is for testing to determine if it is worthwhile to publish more often during times when  data is changing rapidly
  // ...I'll continue to experiment with variations of this and "clean it up" if I go ahead with it (would be in prior loop)
  if (fabs(f_current_psi_pub[1] - f_current_psi[1]) > .2) {  //TODO: is it worthwhile to include <tgmath.h> library for this one command?
  //if (((f_current_psi_pub[1] - f_current_psi[1] > .2)) || ((f_current_psi_pub[1] - f_current_psi[1]) < -.2)) {
//    publishNOW = true;
    //Serial.println("Large PSI difference from published value, force a new publish"); 
    //Serial.print("PV0: "); Serial.print(f_current_psi[0],1); Serial.print(" "); Serial.println(f_current_psi_pub[0],1);
    //Serial.print("PV1: "); Serial.print(f_current_psi[1],1); Serial.print(" "); Serial.println(f_current_psi_pub[1],1);
    //Serial.print("PV2: "); Serial.print(f_current_psi[2],1); Serial.print(" "); Serial.println(f_current_psi_pub[2],1);
    //Serial.print("PV3: "); Serial.print(f_current_psi[3],1); Serial.print(" "); Serial.println(f_current_psi_pub[3],1);    
    //Serial.println(" ");
  }
*/

}


// Writes the PSI ADS1115 Configuration Register via the i2c bus
void write_PSIadcConfigRegister(uint16_t value) {
  Wire.beginTransmission((uint8_t)PSI_ADS1115_I2C_ADDRESS);
  Wire.write((uint8_t)ADS1115_REG_CONFIG);
  Wire.write((uint8_t)(value>>8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

// Reads from the PSI ADS1115 Conversion Register via the i2c bus
int16_t read_PSIadcConversionRegister() {
  Wire.beginTransmission((uint8_t)PSI_ADS1115_I2C_ADDRESS);
  Wire.write((uint8_t)ADS1115_REG_CONVERSION);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)PSI_ADS1115_I2C_ADDRESS, (uint8_t)2);
  return ((Wire.read() << 8) | Wire.read());  
}

//
// Publishes the status, in my case: specifically sends it to a Google spreadsheet and the PoolController Android app
// Formatting (using snprintf) changes as per Scruff recommendation
bool publishAllStatus() {
  const char gsSheet[] = "AllStatus";  // google sheet page
  char stats[622];  // place holder for now

snprintf(stats, sizeof(stats),
      "{\"PSIpv\":%.1f"
        ",\"PSIpp\":%.1f"
        ",\"PSIf\":%.1f"
        ",\"PSIi\":%.1f"
      "}",
      f_current_psi[0],
      f_current_psi[1],
      f_current_psi[2],
      f_current_psi[3]
    );

  return publishNonBlocking(gsSheet, stats);
}

/*  OLDE PUBLISHING CODE to show what variables were previously published
// Publishes the status
bool publishAllStatus() {
    return publishNonBlocking(
        "AllStatus",
        "{\"PSIpv\":"  + String(f_current_psi[0], 2) +
        ",\"PSIpp\":"  + String(f_current_psi[1], 2) +
        ",\"PSIf\":"   + String(f_current_psi[2], 2) +
        ",\"PSIi\":"   + String(f_current_psi[3], 2) +
        ",\"PSIpvx\":" + String(f_psi_max[0], 2) +
        ",\"PSIpvn\":" + String(f_psi_min[0], 2) +
        ",\"PSIppx\":" + String(f_psi_max[1], 2) +
        ",\"PSIppn\":" + String(f_psi_min[1], 2) +
        ",\"PSIfx\":"  + String(f_psi_max[2], 2) +
        ",\"PSIfn\":"  + String(f_psi_min[2], 2) +
        ",\"PSIix\":"  + String(f_psi_max[3], 2) +
        ",\"PSIin\":"  + String(f_psi_min[3], 2) +
        "}");
}
*/

// A wrapper around Partical.publish() to check connection first to prevent
// blocking. The prefix "pool-" is added to all names to make subscription easy.
// "name" is the "sheet" name within the google spreadsheet that this is being sent to
bool publishNonBlocking(const char* sheet_name, const char* message) {
    const char evtPrefix[] = "pool-";
    char evtName[sizeof(evtPrefix) + strlen(sheet_name)];

    snprintf(evtName, sizeof(evtName), "%s%s", evtPrefix, sheet_name);
    // TODO replace with a failure queue?
    if (Particle.connected()) {
        bool success = Particle.publish(evtName, message, PUBLIC); // TODO, need to understand ramifications of making this PRIVATE
//        Serial.printlnf("Published \"%s\" : \"%s\" with success=%d",
//                        name.c_str(), message.c_str(), success);
        return success;
    } else {
//        Serial.printlnf("Published \"%s\" : \"%s\" with success=0 no internet",
//                        name.c_str(), message.c_str());
    }
    return false;
}


