#include <arduinoFFT.h>
#include <Adafruit_NeoPixel.h>
#include <PDM.h>
#include <math.h>
#include <algorithm>
#include <numeric>
//#include <MelFilter.h>

//#include <Constants.h>

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];

// number of samples read
volatile int samplesRead;


#define NUMPIXELS 48 // Popular NeoPixel ring size
#define MFCC_BINS 48

#define BRIGHTNESS 70
#define SAMPLES 256             //Must be a power of 2
#define SAMPLING_FREQUENCY 16000 //Hz, must be less than 10000 due to ADC


#define PIN        8 // On Trinket or Gemma, suggest changing this to 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);

// 9216 / 128 = 72 /16 = 24
arduinoFFT FFT = arduinoFFT();

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[SAMPLES];
double vImag[SAMPLES];

//MelFilter melFilter(SAMPLING_FREQUENCY, 48, SAMPLES);

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

/*
 * Compute the center frequency (fc) of the specified filter band (l) (Eq. 4)
 * This where the mel-frequency scaling occurs. Filters are specified so that their
 * center frequencies are equally spaced on the mel scale
 * Used for internal computation only - not the be called directly
 */
double GetCenterFrequency(unsigned int filterBand)
{
	double centerFrequency = 0.0f;
	double exponent;

	if(filterBand == 0)
	{
		centerFrequency = 0;
	}
	else if(filterBand >= 1 && filterBand <= 14)
	{
		centerFrequency = (200.0f * filterBand) / 3.0f;
	}
	else
	{
		exponent = filterBand - 14.0f;
		centerFrequency = pow(1.0711703, exponent);
		centerFrequency *= 1073.4;
	}
	
	return centerFrequency;
}

/* 
 * Compute the band-dependent magnitude factor for the given filter band (Eq. 3)
 * Used for internal computation only - not the be called directly
 */
double GetMagnitudeFactor(unsigned int filterBand)
{
	double magnitudeFactor = 0.0f;
	
	if(filterBand >= 1 && filterBand <= 14)
	{
		magnitudeFactor = 0.015;
	}
	else if(filterBand >= 15 && filterBand <= 48)
	{
		magnitudeFactor = 2.0f / (GetCenterFrequency(filterBand + 1) - GetCenterFrequency(filterBand -1));
	}

	return magnitudeFactor;
}


/* 
 * Compute the filter parameter for the specified frequency and filter bands (Eq. 2)
 * Used for internal computation only - not the be called directly
 */
double GetFilterParameter(unsigned int samplingRate, unsigned int binSize, unsigned int frequencyBand, unsigned int filterBand)
{
	double filterParameter = 0.0f;

	double boundary = (frequencyBand * samplingRate) / binSize;		// k * Fs / N
	double prevCenterFrequency = GetCenterFrequency(filterBand - 1);		// fc(l - 1) etc.
	double thisCenterFrequency = GetCenterFrequency(filterBand);
	double nextCenterFrequency = GetCenterFrequency(filterBand + 1);

	if(boundary >= 0 && boundary < prevCenterFrequency)
	{
		filterParameter = 0.0f;
    Serial.println(boundary);
    Serial.println(prevCenterFrequency);
	}
	else if(boundary >= prevCenterFrequency && boundary < thisCenterFrequency)
	{
		filterParameter = (boundary - prevCenterFrequency) / (thisCenterFrequency - prevCenterFrequency);
		filterParameter *= GetMagnitudeFactor(filterBand);
    Serial.print("2");
	}
	else if(boundary >= thisCenterFrequency && boundary < nextCenterFrequency)
	{
		filterParameter = (boundary - nextCenterFrequency) / (thisCenterFrequency - nextCenterFrequency);
		filterParameter *= GetMagnitudeFactor(filterBand);
    Serial.print("3");
	}
	else if(boundary >= nextCenterFrequency && boundary < samplingRate)
	{
		filterParameter = 0.0f;
    Serial.print("4");
	}

	return filterParameter;
}




void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting...");
  //initialize microphone
  PDM.onReceive(onPDMdata);

  // optionally set the gain, defaults to 20
  PDM.setGain(20);

  // initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1) yield();
  }

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  pixels.clear(); // Set all pixel colors to 'off'
  //pixels.setPixelColor(3, pixels.Color(255, 255, 255, 0));
  pixels.show();
  //for(int i=0; i<48; i++){
  //  Serial.println(GetCenterFrequency(i));
  //}
  //melfilter = new Melfilter(SAMPLING_FREQUENCY, 48, SAMPLES);
}



void loop() {
  if(samplesRead >= SAMPLES){    
    //clear output buffer (not sure why that's needed, it was in the original ArduinoFFT Example)
    memset(vImag, 0, sizeof(vImag)); 
    //copy the samples from the buffer into the vReal for FFT
    std::copy(sampleBuffer, sampleBuffer+SAMPLES, vReal);

    /*FFT*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    //float peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
        //apply filter bin to get it down to a 16th
    /*int binDivider = 8; //256 samples down to 16 bins
    int binSize = SAMPLES/binDivider;
    float binnedFFT[binDivider] = {0};
    for(int i=0; i<binDivider; i++){
      binnedFFT[i] = std::accumulate(vReal+(i*binSize),vReal+(i*binSize)+binSize, 0)/binSize;
    }*/

    //Serial.println(binnedFFT[0]);
    
    /*MFCC*/
    //float *mfccResult = melFilter.GetCoefficients(vReal);
    for(int coeff = 0; coeff < 48; coeff++){
        //Serial.print(coeff);
        //Serial.print(" :");
        //Serial.println(mfccResult[coeff]);
    }
    //Serial.println();

    float mfcc_result[48];
    for(int coeff = 0; coeff < 40; coeff++)
    {
      //48 is recommended as 3rd argument, aka filter size which should be the output size
      //mfcc_result[coeff] = melFilter.GetCoefficient(vReal, coeff);
      //Serial.print(coeff);
     // Serial.print(" ");
      //Serial.println(mfcc_result[coeff],5);
      //Serial.println(GetFilterParameter(16000, 256, coeff, 11));
    }


     //Serial.print("M Peak: ");
     //Serial.println(peak);
    // The first NeoPixel in a strand is #0, second is 1, all the way up
    // to the count of pixels minus one.
    float peakOfFrame = 0;
    for (int i = 0; i < 48; i++) { 
      float maxVal = 11550*0.7;
      //float value = abs(mfcc_result[i]);
      float value = (vReal[i+1]);
      //peak recording
      if(value > peakOfFrame){
        peakOfFrame = value;
      }
      byte red, blue, green;
      if(value < maxVal/3){
        red = 0;
        green = 0;
        blue = round(value/(maxVal/3)*255);
      }else if(value < maxVal*2/3){
        red = round(value/(maxVal*2/3)*255);
        green = 255;
        blue = 255;
      }else{
        red = 0;
        if(value < maxVal){
          green = round(value/maxVal*255);
        }else{
          green = 255;
        }
        blue = 255;
        
      }
      pixels.setPixelColor(i, pixels.Color(red, green, blue));
      /*Serial.print("Pixel: ");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(red);
      Serial.print(" ");
      Serial.print(green);
      Serial.print(" ");
      Serial.println(blue);*/
      /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/
      //Serial.print(shine);
      //Serial.print(": ");
      //Serial.print((shine * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
      //Serial.print("Hz ");
      //Serial.println(vReal[shine], 1);    //View only this line in serial plotter to visualize the bins
      //print peak value comma separated for easy LibreOfficeAnalysis
      //Serial.println(peakOfFrame);
    }
    pixels.setBrightness(40);
    pixels.show();   // Send the updated pixel colors to the hardware.
    delay(10);
  }
}

