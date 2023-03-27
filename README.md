# **Raspberry Pi Pico USB LED Controller**

This project creates a uses a Raspberry Pi Pico to create an LED controller with a USB interface to PC.

## **Supported Animations**
- Solid
- Stream
- Scroll
- Pulse
- Spectrum Analyzer

## **Usage**
To get started, download [PicoUsbLedStrip.uf2](https://github.com/Disciple153/PicoUsbLedStrip/releases/latest/download/PicoUsbLedStrip.uf2) and [PicoUsbLedStripClient.exe](https://github.com/Disciple153/PicoUsbLedStrip/releases/latest/download/PicoUsbLedStripClient.exe). 

### **Flashing PicoUsbLedStrip**
Put Pico into USB Mass Storage Mode by holding the `BOOTSEL` buttton while plugging it in, then release `BOOTSEL`.

Copy PicoUsbLedStrip.uf2 onto the Pico.

### **First time set up**
With the PicoUsbLedStrip plugged into your PC, do the following.

To find the PicoUsbLedStrip, run `PicoUsbLedStripClient.exe ls`.
This will print the PortId and DeviceId in this format: `PortId:DeviceId`. 

Next you need to set the number of LEDs on your LED strip and give your PicoUsbLedStrip a name.
To do this run `PicoUsbLedStripClient.exe config -p PortId -d <DeviceId> -n <NumLeds>`.
Run this command, adjusting the value of NumLeds, until your LED strip displays exactly one green led on the last pixel.

### **Running PicoUsbLedStripClient**

Example Commands:
- Solid red: `PicoUsbLedStripClient.exe solid -d <DeviceId> -c red`
- Scrolling rainbow: `PicoUsbLedStripClient.exe scroll -d <DeviceId> -l 0x3000 -c FF0000,FFFF00,00FF00,00FFFF,0000FF,FF00FF`

### **Manual**

```
PicoUsbLedStripClient.exe: 
    Send a command to the PicoUsbLedStrip.


    List: PicoUsbLedStripClient.exe ls

        Lists all available PicoUsbLedStrips in portid:deviceid format.


    Configure: PicoUsbLedStripClient.exe config [-p <Port>] [-d <DeviceId>] 
               [-n <NumLeds>]

        Applies configuration options to the PicoUsbLedStrip. This must be
        run to set the number of leds on the led strip and the device id.

        Options:
            -p  --portid    The name of the port of the PicoUsbLedStrip to
                            be configured.
            -d  --deviceid  The device id to assign to the PicoUsbLedStrip.
            -n  --numleds   A 16 bit unsigned integer representing the number of
                            LEDs on the LED strip.


    Display: PicoUsbLedStripClient.exe <DisplayMode> [-i <DeviceId>]
             [-c <Colors>] [-l <LoopTime>]
        DisplayModes:
            Solid:              Shows a static color or gradient.
            Pulse:              Shows colors while pulsing from minimum to
                                maximum brightness over looptime.
            Stream:             Shows a static color or gradient without saving
                                to flash memory. This is useful for streaming 
                                data to the PicoUsbLedStrip.
            Scroll:             Scrolls colors across the LED strip with a
                                period of looptime.
            SpectrumAnalyzer:   Displays an audio spectrum analysis as
                                brightness over the colors while the colors
                                scroll.

        Options:
            -c  --colors    A comma delimited list of colors. Each color may be
                            in hexadecimal or plaintext format (ex: green). Each
                            color will fade into the next in a circular pattern.
            -l  --looptime  A 16 bit unsigned integer representing the number of
                            milliseconds in an animation cycle. May be in
                            decimal or hexadecimal format (ex 512: or 0x200).
            -i  --id        The id assigned to the PicoUsbLedStrip to which
                            the command will be sent.
```


## **Wiring Diagram**
![Wiring diagram](images/PicoUsbLedStrip.drawio.png)
Connecting the TRS (aux) cable is optional, and will enable the Spectrum Analyzer function.