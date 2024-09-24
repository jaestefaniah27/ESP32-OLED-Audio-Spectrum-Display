# ESP32-8266-Audio-Spectrum-Display

The ESP8266 variant can display a flat response between 100Hz and 5.2Khz and is limited by the ADC conversion time.

The ESP32 variant, can display a flat response between 50Hz and 20Khz and is less limited by ADC conversion time and uses 256 samples for.

I've been working a few days on my own version so that it creates an animation similar to Apple music lockscreen widget.
So the number of bands is reduced to 6, and there is a smoothness curve applied to the measurements so that the appearance is better.
For this, I adapted the information seen here: https://stackoverflow.com/questions/13462001/ease-in-and-ease-out-animation-formula

The microphone amplifier Im using is the Adafruit MAX9814: https://www.adafruit.com/product/1713 (i bought it from alliexpress cheaper).
Even though it has an automatic gain controll, I've added my own automatic gain controll on top of the audio amplifier one, so that when the audio is saturating it automatically decreases the gain, and when there is less audio input, it increases the gain to hear better. For this to work, you have to connect the GAIN pin of the microphone  to the pin 13 of the esp32 (you can change it as you like). If you dont connect it, the gain will always be 60 dB.

The sketch is: ESP32-Audio-Spectrum-Display-Apple-like.ino

Thank you for reading.
