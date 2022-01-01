# Si473x Radio using TTGO T-Display

This Si473x Radio uses the amazing library written by [PU2CLR (Ricardo Caratti)](https://github.com/pu2clr/SI4735). It is based on sketches by [PE0MGB (Gert Baak)](https://github.com/pe0mgb/SI4735-Radio-ESP32-Touchscreen-Arduino) and [Thiago Lima](https://github.com/pu2clr/SI4735/tree/master/examples/SI47XX_KITS/THIAGO_LIMA). Since my main goal is a simple portable radio, I am using a [TTGO T-Display](http://www.lilygo.cn/prod_view.aspx?Id=1126), which is a small ESP-32 board with a 1.14 inch display. To fit all the informations in the small size of the screen, it was necessary to adapt the navigation menu. I'm sharing the code and results in this repository.

Reasons to choose this boards are:

* Small footprint
* Good processor
* Enough RAM
* Good screen resolution
* Built-in battery charger circuit
* WiFi/BLE (for future improvements)
* And it's cheap

# Proof of concept

As a proof of concept, I built a pocket radio using a can of skateboard bearings. This can is smaller than an Altoids tin.

# Component Parts

* Si4732
* [TTGO T-Display](http://www.lilygo.cn/prod_view.aspx?Id=1126)
* [NS8002 Module (audio amplifier)](https://www.aliexpress.com/item/32965170167.html)
* Rotary encoder and switch (Built from a mouse wheel)
* Speaker
* LiPo Battery
* [SMA connectors for antenna](https://www.aliexpress.com/item/4000848776660.html)
* Resistors/Capacitors/Wires
* Case

# Schematics:

* Si4735
![Si4735](extras/schematics/schematic_ttgo_tdisplay.png)

# Videos:

* [Si473x - New menu - First test](https://youtu.be/CNm_gOjVeSI)
* [Si473x - New Menu and Display Animation](https://youtu.be/rIns9fEK_OY)
* [Portable Si473x Radio - Encoder and Button Test](https://youtu.be/reAo4QKGGc8)
* [Portable SI473x Radio - Fully Assembled! (Bonus: Inside view)](https://youtu.be/i_zdgfHkd_8)

# Pictures:

* Front view
<img src="extras/images/Si473x_TTGO_TDisplay_front_view.jpg" width="400px"/>

* Side view
<img src="extras/images/Si473x_TTGO_TDisplay_side_view.jpg" width="400px"/>

* Inside view
<img src="extras/images/Si473x_TTGO_TDisplay_inside_view.jpg" width="400px"/>

* Rotary encoder and antenna
<img src="extras/images/Si473x_TTGO_TDisplay_rotary_encoder_and_antenna.jpg" width="400px"/>

* Rotary encoder
<img src="extras/images/Si473x_TTGO_TDisplay_rotary_encoder.jpg" width="400px"/>

* Rotary encoder close view
<img src="extras/images/Si473x_TTGO_TDisplay_rotary_encoder_close_view.jpg" width="400px"/>

# References

1. [PU2CLR SI4735 Library for Arduino](https://github.com/pu2clr/SI4735)

2. [SI4735-Radio-ESP32-2.8 inch TFT Touchscreen-Arduino](https://github.com/pe0mgb/SI4735-Radio-ESP32-Touchscreen-Arduino)

3. [https://github.com/pu2clr/SI4735/tree/master/examples/SI47XX_KITS/THIAGO_LIMA](https://github.com/pu2clr/SI4735/tree/master/examples/SI47XX_KITS/THIAGO_LIMA)

4. [TTGO T-Display](https://github.com/Xinyuan-LilyGO/TTGO-T-Display)
