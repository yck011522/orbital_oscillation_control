Control Repo


To check on the Raspberry Pi connected devices:

``` bash
dmesg | grep -i '\(can\|spi\)'
```
[    4.351367] CAN device driver interface
[    4.473422] mcp251x spi0.0 can0: MCP2515 successfully initialized.
[    4.506943] spi1.0: ttySC0 at I/O 0x0 (irq = 60, base_baud = 921600) is a SC16IS752
[    4.511332] spi1.0: ttySC1 at I/O 0x1 (irq = 60, base_baud = 921600) is a SC16IS752

``` bash
ls /dev
```

RS485 hat should appear as ttySC0 and ttySC1 

Update frequency with everything on:
- Actuator Output = 80Hz 
- Sensor Input (With Visualization in loop) = 35Hz 
- Sensor Input = 60Hz