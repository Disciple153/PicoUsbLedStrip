

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
    10. goto 1


- Confirm transmission using a hash instead of a repeat