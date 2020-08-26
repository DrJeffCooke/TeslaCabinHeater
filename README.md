# TeslaCabinHeater
Documentation of attempts to control the Tesla Cabin Heater - Model S 2017

Working from https://www.youtube.com/watch?v=FKKzTXMfUR8 and https://www.youtube.com/watch?v=xsN6M-tzALM
I started investigating the Tesla Cabin Heater control board for use in the TesLorean

I got the CAN bus running by giving the heater 12v,Gnd, and CanH/L.  It runs at 250Kbps, which was unexpected as many other Tesla units run at 500kbps.

(Spoiler) I was untimately not successful, but I did find out a lot about how the control board is configured and how it attempts to work with the 6 IGBTs that control the flow of high voltage to the PTC elements.  There are also 4 Thermisters, two on the IGBTs and two on the PTC elements.

I wrote the Arduino code to run the heater, but I couldn't get the hardware interface correct.  I'm 99.9% sure that an electrical engineer could pick this up and make it work.

For the TesLorean I'm going to run a combination of water coolant through the stock heater unit, plus some 12v PTC elements (probably 8amps) to facilitate quick window defrosting.  Even if I had of successfully gotten the Tesla Cabin Heater running it would have been a major squeeze to get it retrofitted into the DeLorean airbox.

Good Luck
Jeff Cooke
