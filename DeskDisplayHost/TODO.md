
Test this:
- Transmit like this (all asynchrynous. no timeouts.):
    1. wait for connection
    2. wait for request (0x00)
    3. respond (DeskDisplay\n)
    4. receive length of transmission (uint16_t length)
    5. receive transmission (uint8_t[length])
    6. respond with hash of transmission
    7. recieve error code (0x00 or 0x01)
    8. if 0x01, goto 4
    9. if no response within timeout, goto 1
        - free transmission.data if necesarry
    10. goto 1


- Confirm transmission using a hash instead of a repeat

- Pagify transmissions DONE
- Remove recursion DONE
- Spread out hashing operations
- Maybe statically allocate data
- Make the transmission data pointer an array object
    - Overload [] to access normally DONE
    - Length: length - 2 DONE
    - RawLength: length of raw data DONE
    - RawData: array (either dynamic or static) length divisible by 256 DONE
    - static method to load array

try to redefine USBD_PID in pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb_descriptors.c