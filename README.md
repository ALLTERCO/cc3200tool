# CC3200 Tool

A small tool to write files in TI's CC3200 SimpleLink (TM) filesystem.

Copyright (C) 2016 Allterco Robotics

![](https://img.shields.io/badge/license-GPL_2-green.svg "License")

## Rationale

The only other tool which can officially do this is Uniflash, but good luck
using that on a modern Linux system.

There is also `cc3200prog` which [Energia](http://energia.nu) sneak in their toolchain tarball,
but it's not open source and can only write to `/sys/mcuimg.bin`.

Finally, there's the great guys at [Cesanta](https://www.cesanta.com/)
who reversed the CC3200 bootloader
protocol just enough to make day-to-day development on the platform possible.
However, their tool is specific to [smart.js](https://www.cesanta.com/products/smart-js)
and feeds on specially-crafted zip archives.

This tool is based on the work done by Cesanta but is written in Python and
exposes a generic cli interface for uploading user files and application binaries
on a CC3200-attached serial flash.

`cc3200tool` can upload NWP/MAC/PHY firmwares (`/sys/servicepack.ucf`), but it seems
this only works on a clean FS. The tool also implements the functionality
described in TI's Application Note [CC3100/CC3200 Embedded Programming](http://www.ti.com/tool/embedded-programming).

## Installation

This runs on Python 2.7 with recent [pySerial](https://github.com/pyserial/pyserial).

To install, if you have pip and want system-wide:

    pip install git+git://github.com/AlexLisnitski/cc3200tool.git

or clone this repǫ

    git clone http://github.com/AlexLisnitski/cc3200tool.git
    cd cc3200tool

then it's just like any other python package:

    python setup.py install # as root, system-wide

    # or in a virtualenv with pip
    virtualenv env && ./env/bin/activate
    pip install -e .
    # then get updates with
    git pull

## Usage

You need a serial port connected to the target's UART interface. For
programming to work, SOP2 needs to be asserted (i.e. tied to GND) and a reset
has to be peformed to switch the chip in bootloader mode. `cc3200tool` can
optionally use the RTS and DTR lines of the serial port controller to
automatically perform these actions via the `--sop2` and `--reset` options.

If you received error "timed out while waiting for ack" after message "Resetting communications",
then try to adjust `--break-duration` argument - make it more or less than the default (default is 10).

See `cc3200tool -h` and `cc3200tool <subcommand> -h` for complete description
of arguments. Some examples:

    # upload an application
    cc3200tool -p /dev/ttyUSB2 --sop2 ~dtr --reset prompt \
        write_file ./exe/myapp.bin /sys/mcuimg.bin

    # format and upload an application binary
    cc3200tool -p /dev/ttyUSB2 \
        format_flash --size 1M \
        write_file exe/program.bin /sys/mcuimg.bin

    # dump a file on stdout
    cc3200tool read_file /sys/mcuimg.bin -

    # format the flash, upload a servciepack and two files
    cc3200tool -p /dev/ttyUSB2 --sop2 ~rts --reset dtr \
        format_flash --size=1M \
        write_file --file-size=0x20000 \
            --signature ../servicepack-ota/ota_1.0.1.6-2.6.0.5.ucf.signed.bin \
            ../servicepack-ota/ota_1.0.1.6-2.6.0.5.ucf.ucf /sys/servicepack.ucf \
        write_file ../application_bootloader/gcc/exe/application_bootloader.bin /sys/mcuimg.bin \
        write_file yourapp.bin /sys/mcuimg1.bin

    # list file and filesystem statistics (occupied and free block sequences)
    cc3200tool -p /dev/ttyUSB2 list_filesystem
