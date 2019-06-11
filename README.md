# 2019_AudioSampler

## Overview
A simple audio sampler that lets you combine 8-bit songs.

## Description
The project allows you to load wav files from a micro sd card using the SPI interface. The audio data from the files are converted to an analog signal and fed to the JACK output and properly mapped keys allow you to control the songs (play/pause). It is possible to play several tracks simultaneously (unfortunately with a loss on quality).

## Tools
### Physical components:
- STM32F407G
- MicroSD Card Adapter
- Matrix 4x4 Keypad
- CS43L22 DAC

### Software:
- System Workbench IDE
- STM32CubeMX

### Language:
- C
- MY_CS43L22 library by Mohamed Yaqoob
- MY_Keypad library by Mohamed Yaqoob
- FatFs by ChaN

## How to run
Supported files on the memory card: WAV PCM U8

## How to compile
Open .cproject file and run project by IDE (as Ac6 STM32 C/C++)

## License
MIT

## Credits
Rafał Sobański
Miłosz Dziurzyński

The project was conducted during the Microprocessor Lab course held by the Institute of Control and Information Engineering, Poznan University of Technology.
Supervisor: Tomasz Mańkowski
