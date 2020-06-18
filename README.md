# Self-Checkout-Shopping-Cart ([DEMO VIDEO](https://drive.google.com/file/d/1PHqSrd3S05qmIQ_HfaXPMp07Jxs_gFq5/view?pli=1))

<p align="middle">
<img src="Project_Images/project_pcb.PNG">
</p>


<p align="middle">
<img src="Project_Images/pcb.PNG">
</p>


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

### Hardware Block Diagram

<p align="middle">
<img src="Project_Images/LPEDT-BLOCK-DIAGRAM-NEW.jpg">
</p>

### Software Block Diagram

<p align="middle">
<img src="Project_Images/software_block.jpg">
</p>


## Overview:
The “Self-Checkout Shopping Cart” is an innovative consumer purchasing product that is designed to help shoppers fast-track their shopping experience! The shopping cart has an inbuilt barcode scanner which can be used to scan the items to be purchased. The device communicates with the phone over the Bluetooth and bill is generated based on the items. Android app can be used for payment and faster checkout. With the advent of energy efficient devices and low power nodes, it has become imperative to design boards that consume low power which can last longer. To that end, we are designing nodes in order to consume minimal energy and address the issues mentioned below.

PROJECT SOLVES THE FOLLOWING PROBLEMS
1.	Customers usually get annoyed because of the long queues in the billing section of the huge shopping markets. 
2.	In addition to that keeping track of all the bills and budget is a very burdensome task.
3.	Usage of lot of manpower in large supermarkets which can be expensive.
4.	Stock management in supermarkets.
All these problems could be addressed by our “Self-Checkout Shopping Cart”.


SOLUTIONS
1.	Fast self-checkout saves time of customers and helps them buy items according to their budget.
2.	Electronic bill is generated and saved in the cloud which makes it easy to keep track of all the bills and saves paper.
3.	By letting customers handle their own scanning and bagging, workers can spend their time helping customers find what they need. 
4.	Better shopping experience for the customers and an innovative way for the sellers to attract customers.



The entire project report can be found [here](https://github.com/jajoosiddhant/Self-Checkout-Shopping-Cart/blob/master/Project_Report.pdf).

The entire project ppt can be found [here](https://github.com/jajoosiddhant/Self-Checkout-Shopping-Cart/blob/master/Project_PPT.pptx).

The android application apk can be found [here](https://github.com/jajoosiddhant/Self-Checkout-Shopping-Cart/blob/master/ShoppingCart.apk).

Weekly project updates can be found [here](https://github.com/jajoosiddhant/Self-Checkout-Shopping-Cart/tree/master/Project_Updates).

PCB schematic layout can be found here [here](https://github.com/jajoosiddhant/Self-Checkout-Shopping-Cart/blob/master/Altium_layout_schematic.pdf).

Final altium Design zip file can be found [here](https://github.com/jajoosiddhant/Self-Checkout-Shopping-Cart/blob/master/Final_Design_Altium.zip).

Final code zip file can be found [here](https://github.com/jajoosiddhant/Self-Checkout-Shopping-Cart/blob/master/shopping_cart_code.zip).


There are different branches for various stages of development throughout the project.  
[cart_v1.0](https://github.com/jajoosiddhant/Self-Checkout-Shopping-Cart/tree/cart_v1.0) has the final code and master being the latest branch with all the other components.
