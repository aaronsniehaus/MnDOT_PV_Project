# MnDOT_PV_Project

This repository contains documents associated with the MnDOT Project as of 2/13/2020.

I would like to use this as a place to host documents for collaboration into the future, as well as compile a place to show employers work samples. You will be able to find documents like PCB layouts, circuit schematics, and code for the controller. There are also a lot of isolated parts libraries scattered around the place, and I would like to consolidate these into one large library.

To access it, you'll need the following software:


Texas Instruments Code Composer Studio (CCS),

Altium Designer,

Texas Instruments C2000Ware Package for CCS


The code as of 2/19/2020 is fully functional for open loop control. ADC values are constantly updated at a sufficient rate and write to an array of 6 ADC values, at a pre-defined number of samples over time, "SampleDepth." 

EPWM 1 -6 run at 250kHz, and the filtered sine output around 60.3 Hz. 

Synchronization can be done using the two-pin header, and in software this is controlled by the "CheckMaster" function. If a master synchronization signal is not detected within 3 continuous power cycles, the software will change to master mode automatically.


Features to add later:

  Closed loop control for boost stage, output stage

  Maximum Power Point Tracking (MPPT)
