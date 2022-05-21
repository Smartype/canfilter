# canfilter

This is a radar can message interceptor designed for Toyota NODSU Camry/CH-R. It is based on an STM32 mileage manipulator board.

- For CH-R, it enables full speed ACC with openpilot.
- For Camry Hybrid/XLE/XSE, it enables full speed ACC with stock system and openpilot.
- For Camry LE/SE, it bypass 28mph(45kph) speed limit. But the ABS module ignores braking request under 15mph(24kph). This limit applies to both stock system and openpilot.

![Radar Interceptor](https://github.com/Smartype/canfilter/blob/master/board/resources/radar-interceptor.jpg?raw=true)

## Features
- boot loader from commaai panda project, to support firmware updating on CAN bus
- crash detection, on crash it will switch to passthrough failsafe mode
- mode selector, it can be switched into a passthrough mode, then it is invisable on CAN bus
- init magic, enable ACC_CONTROL ACC_TYPE override, which make PCM believe the ACC is full speed range
- manually speed lockout, lock the engage speed at 30mph, disengage speed at 25mph
- fake low speed lead in stock ACC mode, so that you can engage at when there is no lead car
- control with ACC_CONTROL, when ACC_CONTROL is seen on can BUS 0, it mutes the stock radar ACC_CONTROL msg.
- control with ACC_CONTROL_SAFE, when ACC_CONTROL_SAFE is seen on can BUS 0, it mutes stock radar ACC_CONTROL msg, and if AEB engages, it mutes ACC_CONTROL_SAFE and allow stock ACC_CONTROL msg
- ACC_CONTROL_COPY, it mirrors the stock ACC_CONTROL msg with a different msg id, so it can be logged/used in openpilot(for example, the DISTANCE_REQ bit)
- stock ACC_CONTROL alert override, if OP long is engaged, while the stock ACC emits an alert(CUT IN), it will be merged into final ACC_CONTROL msg

## Wiring
Please check pictures in board/resources. The radar has 2 can buses, CAN1 is what we are going to intercept.
The CAN1 on filter board should be connected to CAN gateway(the car), CAN2 to be connected to radar CAN1.
Double check the wiring, especially GND and 12V pins, or you will fry your radar!


## Flashing
The board comes with flash read out protection, you will need STM32 ST-LINK Utility to unlock it first. then you can make flash or make flash-bootloader.

1. To put the device to DEBUG mode, first connect NRST to GND, power it and then disconnect NRST.
2. Disable Read Out Protection in Target/Option Bytes.. in STM32 ST-LINK Utility
3. Flash

```
eric@ubuntu:~$ st-info --probe
Found 1 stlink programmers
 serial: <removed>
openocd: <removed>
  flash: 0 (pagesize: 2048)
   sram: 65536
 chipid: 0x0418
  descr: F1 Connectivity line device

eric@ubuntu:~$ st-flash write BootStub.bin  0x8000000
st-flash 1.6.0
2022-05-21T06:13:18 INFO common.c: Loading device parameters....
2022-05-21T06:13:18 INFO common.c: Device connected is: F1 Connectivity line device, id 0x10016418
2022-05-21T06:13:18 INFO common.c: SRAM size: 0x10000 bytes (64 KiB), Flash: 0x40000 bytes (256 KiB) in pages of 2048 bytes
2022-05-21T06:13:18 INFO common.c: Attempting to write 9160 (0x23c8) bytes to stm32 address: 134217728 (0x8000000)
Flash page at addr: 0x08002000 erased
2022-05-21T06:13:18 INFO common.c: Finished erasing 5 pages of 2048 (0x800) bytes
2022-05-21T06:13:18 INFO common.c: Starting Flash write for VL/F0/F3/F1_XL core id
2022-05-21T06:13:18 INFO flash_loader.c: Successfully loaded flash loader in sram
  5/5 pages written
2022-05-21T06:13:18 INFO common.c: Starting verification of write complete
2022-05-21T06:13:18 INFO common.c: Flash written and verified! jolly good!
eric@ubuntu:~$ it2ul
eric@ubuntu:~$ st-flash write CanFilter.bin.signed 0x8004000
st-flash 1.6.0
2022-05-21T06:13:49 INFO common.c: Loading device parameters....
2022-05-21T06:13:49 INFO common.c: Device connected is: F1 Connectivity line device, id 0x10016418
2022-05-21T06:13:49 INFO common.c: SRAM size: 0x10000 bytes (64 KiB), Flash: 0x40000 bytes (256 KiB) in pages of 2048 bytes
2022-05-21T06:13:49 INFO common.c: Attempting to write 17168 (0x4310) bytes to stm32 address: 134234112 (0x8004000)
Flash page at addr: 0x08008000 erased
2022-05-21T06:13:50 INFO common.c: Finished erasing 9 pages of 2048 (0x800) bytes
2022-05-21T06:13:50 INFO common.c: Starting Flash write for VL/F0/F3/F1_XL core id
2022-05-21T06:13:50 INFO flash_loader.c: Successfully loaded flash loader in sram
  9/9 pages written
2022-05-21T06:13:50 INFO common.c: Starting verification of write complete
2022-05-21T06:13:51 INFO common.c: Flash written and verified! jolly good!

```

## Upgrading
The bootloader supports upgradeing over CAN bus. You can upgrade the FW with your C2/C3. You will have to stop pandad.py and boardd first.
Turn on the car into accessory mode, so that the intercepter is powered on with rader. use enter_canloader.py script to load new FW.
The board supports auto FW upgrading. If commaai accepts this, this function can be added to stock OP, to re-flash the FW on-demand.

## Feature switches
- LOCKSPEED: this applies to Camry XLE/XSE/LE/SE. when set, lock speed to engage above 30kph, disengage below 25kph (lost lead alert in dash). XSE/XLE users should unset this to get full speed. (If I have a method to do this automatically in the future, you won't need this)
- FAKELEAD: when speed below 45kph, fake a lead car, then you can press SET to engage. This is very dangerous if the button pressed unexpected.
- PASSTHRU: all features disable. The board is invisible on CAN bus. It still accepts feature updating, FW upgrading. 
- MIRRORMSG: Mirror stock ACC_CONTROL to another msg. OP detect this msg to enable long control. You can unset this to use stock long control instead.
- DISTANCEREQ: Automatically handle lead car distance button.

## License
Code in this repository is released under the MIT license.

USING ANYTHING FROM THIS REPOSITY IS AT YOUR OWN RISK!


