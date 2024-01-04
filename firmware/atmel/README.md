# Atmel SmartConnect 

<a href="http://www.atmel.com"><img src="http://www.atmel.com/Images/atmel.png" align="left" hspace="10" vspace="6"></a>

Atmel SmartConnect is a family of self-contained, low power and certified modules bringing wireless internet connectivity.

**ATWILC1000** is IEEE 802.11 b/g/n/ IOT link controller utilizing highly optimized 802.11 and provides mulitple peripheral interfaces like SPI, and SDIO.  

Refer to the [WiKi](https://github.com/linux4sc/wireless-driver/wiki) to find usefull instructions and information for wireless solutions.

**ATWILC3000** is IEEE 802.11 b/g/n/BT4.0 IOT link controller utilizing highly optimized 802.11-Bluetooth coexistence protocol and provides mulitple peripheral interfaces like UART, SPI, and SDIO.

This repository provides ATWILC3000 WLAN and Bluetooth firmware. The baud rate of bt_firmware.bin is 115200 and flow control is disabled in default. Refer to the [how to change baud rate and flow control](https://github.com/atwilc3000/driver/wiki/bluetooth#baud-rate-and-flow-control) to change baud rate or enable flow control. 

Refer to the [sama5d3 xplained quick start guide](https://github.com/atwilc3000/driver/wiki/sama5d3-xplained) to find how to include the firmware into the Buildroot-at91. 

Refer to the [wiki](https://github.com/atwilc3000/driver/wiki) to find usefull instructions and information for wireless solutions. 

For more information on Atmel SmartConnect, visit [Atmel SmartConnect](http://www.atmel.com/products/wireless/wifi/smart-connect.aspx).
