# stm32_ltc_decoder

This device used for decoding LTC from VTRs or other sources. Output sent to CDC serial.

## used parts

### STM32 BluePill board

![STM32 BluePill board](docs/Bluepillpinout1.gif)

### RS485 module

![Photo](docs/MAX485-RS-485-Interface-Module-Connections.jpg)

![Schematic](docs/MAX485-Module-Schematic.jpg)

### other
* 2N7000
* Jumper
* Prototyping PCB

## connection diagram

```
           RS485 Module
        +---------------+   +----------------------------> [to 5V of STM32 Bluepill]
        |               |   |
        |       VCC +5V |---+           2N7000
        |               |              +------+      JP
        |            RO |--------------| G    |      |\--> [to PB9 of STM32 Bluepill]
LTC+ -->| A             |              |    D |------
        |            DI |   +----------| S    |      |/--> [to PA0 of STM32 Bluepill]
        |               |   |          +------+
        |           /RE |---+
LTC- -->| B             |   |
        |            DE |---+
        |               |   |
        |           GND |---+----------------------------> [to GND of STM32 Bluepill]
        +---------------+   |
                            |
                           ---
                           \ /
                            -
```

## output example

![putty](docs/putty.png)

## working model

![photo1](docs/photo1.jpg)

![photo2](docs/photo2.jpg)

## notes

