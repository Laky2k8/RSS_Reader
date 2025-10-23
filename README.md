# LakyWatch RSS Reader
<br>

I've been working on a little homemade smartwatch based on the Waveshare ESP32-S3-Touch-AMOLED-1.8 module (I literally cannot recommend these guys enough, they make absolutely goated products!).
Recently it occured to me that while I tried out most things with the device like loading from the SD card, playing music, getting data from the sensors etc., I haven't really tried out one thing, not just with this module but really with microcontrollers in general; Connecting to **The World Wide Web** (in bold for dramatic measure). <br>
So I decided to try my best at creating (what I originally imagined to be) a simple demo of Internet capabilities: An RSS reader! <br>
Of course, in the end it was a lot harder than anticipated; Turns out that ESP-IDF isn't the friendliest environment for a beginner and that HTTPS on microcontrollers is a gigantic pain :/ <br>
But I persevered and I think I managed to put together something pretty cool :]
<br>

# Features
- Automatic connection to Wifi
- HTTP and HTTPS support
- Automatic query of selected RSS feed
- Displaying said RSS feed in a nice LVGL list for your viewing pleasure :]

I also plan to add viewing the contents of the posts themselves, but I didn't really have time to add it and right now this works fine enough.

It's somewhat based on the Waveshare LVGL demo for the module: https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8#05_LVGL_WITH_RAM.
WIP image rendering (still in development) is based on https://github.com/Laky2k8/EvilAssWizardGame 0.0.1.

<br>

# How to build and run
The program probably works on and ESP32-S3 module with a touchscreen (possibly with some modifications), but I have written it for the Waveshare ESP32-S3-Touch-AMOLED-1.8 module and it is the intended running environment. <br>

To build:
- Create a new ESP-IDF project
- Get the espressif/esp_io_expander_tca9554, the esp_lcd_touch_ft5x06, the esp_lcd_sh8601 and the lvgl/lvgl library
- Unzip the contents of the repository into the project folder (Source code should end up in main)
- Click on "ESP-IDF: Build, Flash and Monitor"

And voilá: the program is running on your device! :D



