# Embedded speedometer

The speedometer software was created in 2018 specifically for use in the 2019 Shell Eco-Marathon on the UrbanConcept Car Z172. 

## Features
The speedometer is powered by an STM32 F407 development board and driven by FreeRTOS. The display screen is a 3.5-inch TFT LCD resistive touch screen. The speedometer is connected to other controllers on the vehicle through the CAN bus, and receives speed information detected by a Hall sensor. It is also capable of controlling the electrical components on the vehicle such as headlights and wipers.

## Acknowledgements
The current version owes much of its success to the hard work of the previous members of Zeal Eco-power's Electronic Control Group, to whom we owe a great deal of gratitude.
