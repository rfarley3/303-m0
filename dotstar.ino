#ifndef synth303_dot_ino
#define synth303_dot_ino
/*  Create a 303-ish synth with potentiometer controllers.
 *  (c) Ryan Farley ryanjfarley@gmail.com rfarley3@github
*/
#ifdef ADAFRUIT_TRINKET_M0
// setup Dotstar LED on Trinket M0
#include <Adafruit_DotStar.h>
Adafruit_DotStar strip(DOTSTAR_NUM, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BRG);
#endif
unsigned int rainbow_loops = 0;
long firstPixelHue = 0;


void dotstar_setup() {
  /* Same purpose as primary file setup */
  // Start with built-in Dotstar RGB LED turned off
  strip.begin();
  strip.clear();
  strip.setBrightness(60);
  strip.show();
}


void rainbowHook() {
  /* See if you really want to do this, you probably don't want to loose time each loop
   * use a counter instead of timer to avoid interrupts
   */
  rainbow_loops++;
  if (rainbow_loops > 8192) {
    rainbow(firstPixelHue);
    // Color wheel has a range of 65536 and it's OK if we roll over, but % bc why not
    firstPixelHue = (firstPixelHue + 256) % 65536;
    rainbow_loops = 0;
  }
}


void rainbow(long firstPixelHue) {
  /* Per example, for each Pixel change the color, but do it so if you have more than one, they chase */
  for(int i = 0; i < strip.numPixels(); i++) {
    // Offset pixel hue by an amount to make one full revolution of the
    // color wheel (range of 65536) along the length of the strip
    int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
    // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
    // optionally add saturation and value (brightness) (each 0 to 255).
    // Here we're using just the single-argument hue variant. The result
    // is passed through strip.gamma32() to provide 'truer' colors
    // before assigning to each pixel:
    strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
  }
  strip.show(); // Update strip with new contents
}

#endif
