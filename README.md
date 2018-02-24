I forked the repo from fcgdam, who did some excellent work on adding functionality to the original code for the TTGO Lora ESP32 Oled board. In this repo, I will add features that I find useful, but not necessarily be of value for others.

# TTGO LoRa32 ESP32 Oled board

This repository holds some sample code, using platformio IDE, for using this board.
Use the **PRG** button, which is connected to GPIO0, to cycle through the various spreading factors. On each button press, the OLED display gets updated to show the selected spreading factor and frequency. The new SF will be used during the next transmit.

# TTN ABP

This code uses the Lora chip to connect to the Things Network, by ABP.

# WiFi Scanner - Arduino

Simple Wifi scanner that shows found networks on the Oled.
