# Self-Checkout-Shopping-Cart

## Contributors: 
- Siddhant Jajoo 
- Satya Mehta 
- Deepesh Sonigra 


This project was completed in the course Low Power Embedded Design Techniques at University of Colorado, Boulder under the guidance of Prof. Timothy Scherr and Prof. Randall Spalding during September 2019 - December 2019.  

## Hardware Components:
- Blue Gecko EFR32BG13 ([datasheet](https://www.silabs.com/documents/public/data-sheets/efr32bg13-datasheet.pdf))
- NXP NTAG NFC Module ([datasheet](https://www.nxp.com/docs/en/data-sheet/NT3H2111_2211.pdf))
- Barcode Scanner ([datasheet](https://www.waveshare.com/w/upload/3/3c/Barcode_Scanner_Module_User_Manual_EN.pdf))
- Lipo Lithium ion battery ([datasheet](https://cdn.sparkfun.com/datasheets/Prototyping/Lithium%20Ion%20Battery%20MSDS.pdf))
- Buck Converter - TPS560430XFDBVR ([datasheet](http://www.ti.com/general/docs/suppproductinfo.tsp?distId=10&gotoUrl=http%3A%2F%2Fwww.ti.com%2Flit%2Fgpn%2Ftps560430))
- LDO - LM117 ([datasheet](http://www.ti.com/lit/ds/symlink/lm1117.pdf))
- Battery Charging Management IC - MCP73213 ([datasheet](http://ww1.microchip.com/downloads/en/devicedoc/20002190c.pdf))
- Antenna
- Resistors
- Capacitors
- Inductors
- Adapter Jack
- Micro usb connector
- Debbugger

## Software Components:
- Simplicity Studio
- Android Application

## Features:
-	The device connects with an android application using Bluetooth and Bluetooth authentication/pairing is done using NFC for faster and secure connection.
-	The Barcode Sensor scans QR codes/Barcodes to obtain the information of the product including the name and cost using Low Energy UART peripheral on EFR32BG13.
-	The scanned item information is sent to the android application using BLE with the help of an onboard designed antenna.
-	The Android application is capable to display all the scanned items and total price of the items in the cart.
-	The Barcode scanner works on 5V and the NFC module works on 3.3 V which is obtained by using Buck and LDO respectively.
-	The device is battery operated with recharging capability using a battery management circuit provided on the circuit. The Battery management circuit is powered by a 12V adapter in order to provide constant power to charge the 7.4V battery.
-	A debugger is used in order to obtain debug printfs (VCOMM) for debugging purposes and to flash code on the processor.
-	Several test points and isolation points have been provided on the board for debugging purposes.
-	The board can also be powered through other sources via USB.


## Block Diagram:


## Overview:


## Android Application

