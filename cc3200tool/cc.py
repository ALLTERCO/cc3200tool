#
# cc3200tool - work with TI's CC3200 SimpleLink (TM) filesystem.
# Copyright (C) 2016 Allterco Robotics
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#

import sys
import os
import time
import argparse
import struct
import math
import logging
from contextlib import contextmanager
from pkgutil import get_data
from collections import namedtuple
import json
import platform
import serial
import subprocess

log = logging.getLogger()
logging.basicConfig(stream=sys.stderr, level=logging.INFO,
                    format="%(asctime)-15s -- %(message)s")
CURRENT_PATH = os.path.dirname(os.path.realpath(__file__))

CC3200_BAUD = 921600

# erasing blocks is time consuming and depends on flash type
# so separate timeout value is used
ERASE_TIMEOUT = 120

OPCODE_START_UPLOAD = "\x21"
OPCODE_FINISH_UPLOAD = "\x22"
OPCODE_GET_LAST_STATUS = "\x23"
OPCODE_FILE_CHUNK = "\x24"
OPCODE_GET_STORAGE_LIST = "\x27"
OPCODE_FORMAT_FLASH = "\x28"
OPCODE_GET_FILE_INFO = "\x2A"
OPCODE_READ_FILE_CHUNK = "\x2B"
OPCODE_RAW_STORAGE_READ = "\x2C"
OPCODE_RAW_STORAGE_WRITE = "\x2D"
OPCODE_ERASE_FILE = "\x2E"
OPCODE_GET_VERSION_INFO = "\x2F"
OPCODE_RAW_STORAGE_ERASE = "\x30"
OPCODE_GET_STORAGE_INFO = "\x31"
OPCODE_EXEC_FROM_RAM = "\x32"
OPCODE_SWITCH_2_APPS = "\x33"
OPCODE_FS_PROGRAMMING = "\x34"

STORAGE_ID_SRAM = 0x0
STORAGE_ID_SFLASH = 0x2

FLASH_BLOCK_SIZES = [0x100, 0x400, 0x1000, 0x4000, 0x10000]

SLFS_SIZE_MAP = {
    "512": 512,
    "1M": 1024,
    "2M": 2 * 1024,
    "4M": 4 * 1024,
    "8M": 8 * 1024,
    "16M": 16 * 1024,
}

SLFS_BLOCK_SIZE = 4096

# defines from cc3200-sdk/simplelink/include/fs.h
SLFS_FILE_OPEN_FLAG_COMMIT = 0x1              # /* MIRROR - for fail safe */
SLFS_FILE_OPEN_FLAG_SECURE = 0x2              # /* SECURE */
SLFS_FILE_OPEN_FLAG_NO_SIGNATURE_TEST = 0x4   # /* Relevant to secure file only  */
SLFS_FILE_OPEN_FLAG_STATIC = 0x8              # /* Relevant to secure file only */
SLFS_FILE_OPEN_FLAG_VENDOR = 0x10             # /* Relevant to secure file only */
SLFS_FILE_PUBLIC_WRITE = 0x20                 # /* Relevant to secure file only, the file can be opened for write without Token */
SLFS_FILE_PUBLIC_READ = 0x40                  # /* Relevant to secure file only, the file can be opened for read without Token  */

SLFS_MODE_OPEN_READ = 0
SLFS_MODE_OPEN_WRITE = 1
SLFS_MODE_OPEN_CREATE = 2
SLFS_MODE_OPEN_WRITE_CREATE_IF_NOT_EXIST = 3


def hexify(s):
    return " ".join([hex(ord(x)) for x in s])


Pincfg = namedtuple('Pincfg', ['invert', 'pin'])


def pinarg(extra=None):
    choices = ['dtr', 'rts', 'gpio', 'none']
    if extra:
        choices.extend(extra)

    def _parse(apin):
        invert = False
        if apin.startswith('~'):
            invert = True
            apin = apin[1:]
        if apin not in choices:
            raise argparse.ArgumentTypeError("{} not one of {}".format(
                    apin, choices))
        return Pincfg(invert, apin)

    return _parse


def auto_int(x):
    return int(x, 0)


# TODO: replace argparse.FileType('rb') with manual file handling
parser = argparse.ArgumentParser(description='Serial flash utility for CC3200')

parser.add_argument(
        "-p", "--port", type=str, default="/dev/ttyUSB0",
        help="The serial port to use")
parser.add_argument(
        "--reset", type=pinarg(['prompt']), default="none",
        help="dtr, rts, none or prompt, optinally prefixed by ~ to invert")
parser.add_argument(
        "--reset-pin", default=None,
        help="wPi number of GPIO pin wich will be used to reset")
parser.add_argument(
        "--break-duration", default=10, type=int, choices=range(4,20),
        help="Num of UART break cycles when switching cc32xx to boot mode (10 is recommended)")
parser.add_argument(
        "--sop2", type=pinarg(), default="none",
        help="dtr, rts or none, optinally prefixed by ~ to invert")
parser.add_argument(
        "--erase_timeout", type=auto_int, default=ERASE_TIMEOUT,
        help="Specify block erase timeout for all operations which involve block erasing")
parser.add_argument(
        "--reboot-to-app", action="store_true",
        help="When finished, reboot to the application")

subparsers = parser.add_subparsers(dest="cmd")

parser_find_device = subparsers.add_parser(
        "find_device", help="Find device")

parser_format_flash = subparsers.add_parser(
        "format_flash", help="Format the flash memory")
parser_format_flash.add_argument(
        "-s", "--size", choices=SLFS_SIZE_MAP.keys(), default="1M")

parser_erase_file = subparsers.add_parser(
        "erase_file", help="Erase a file from the SL filesystem")
parser_erase_file.add_argument(
        "filename", help="file on the target to be removed")

parser_write_file = subparsers.add_parser(
        "write_file", help="Upload a file on the SL filesystem")
parser_write_file.add_argument(
        "local_file", type=argparse.FileType('rb'),
        help="file on the local file system")
parser_write_file.add_argument(
        "cc_filename", help="file name to write on the target")
parser_write_file.add_argument(
        "--signature", type=argparse.FileType('rb'),
        help="file which contains the 256 bytes of signature for secured files")
parser_write_file.add_argument(
        "--file-size", type=auto_int, default=0,
        help="allows allocating more space than needed for this upload")
parser_write_file.add_argument(
        "--commit-flag", action="store_true",
        help="enables fail safe MIRROR feature")

parser_read_file = subparsers.add_parser(
        "read_file", help="read a file from the SL filesystem")
parser_read_file.add_argument(
        "cc_filename", help="file to read from the target")
parser_read_file.add_argument(
        "local_file", type=argparse.FileType('wb'),
        help="local path to store the file contents in")

parser_write_flash = subparsers.add_parser(
        "write_flash", help="Write a Gang image on the flash")
parser_write_flash.add_argument(
        "image_file", type=argparse.FileType('rb'),
        help="gang image file prepared with Uniflash")
parser_write_flash.add_argument(
        "--no-erase", type=bool, default=False,
        help="do not perform an erase before write (for blank chips)")

parser_read_flash = subparsers.add_parser(
        "read_flash", help="Read SFFS contents into the file")
parser_read_flash.add_argument(
        "dump_file", type=argparse.FileType('w+'),
        help="path to store the SFFS dump")
parser_read_flash.add_argument(
        "--offset", type=auto_int, default=0,
        help="starting offset (default is 0)")
parser_read_flash.add_argument(
        "--size", type=auto_int, default=-1,
        help="dump size (default is complete SFFS)")


parser_list_filesystem = subparsers.add_parser(
        "list_filesystem",
        help="List SFFS contents and statistics (blocks total/used, inter-file gaps, etc)")
parser_list_filesystem.add_argument(
        "--json-output", action="store_true",
        help="output in JSON format to stdout")
parser_list_filesystem.add_argument(
        "--inactive", action="store_true",
        help="output inactive FAT copy")

def load_file(fname):
    data = []
    with open(fname, 'r') as f:
        data = f.read()
        assert len(data) > 0
    return data

def dll_data(fname):
    return get_data('cc3200tool', os.path.join('dll', fname))


class CC3200Error(Exception):
    pass

#   Chip ID   Part Number
# 0x31000000    CC3120
# 0x31100000    CC3135
# 0x31000011    CC3220R
# 0x31000018    CC3220S
# 0x31100018    CC3235S
# 0x31000019    CC3220SF
# 0x31100019    CC3235SF
# 0x31120000    CC3130
# 0x31120018    CC3230S
# 0x31120019    CC3230SF

class CC3x00VersionInfo(object):
    def __init__(self, bootloader, nwp, mac, phy, chip_type):
        self.bootloader = bootloader
        self.nwp = nwp
        self.mac = mac
        self.phy = phy
        self.chip_type = chip_type

    @property
    def is_cc32xx(self):
        return (self.chip_type[0] & 0x10) != 0
    @property
    def is_cc3220(self):
        return (self.chip_type[0] == 0x10 and self.chip_type[1] == 0)
    @property
    def is_cc3220s(self):
        return (self.chip_type[0] == 0x18 and self.chip_type[1] != 0xB1)
    @property
    def is_cc3220sf(self):
        return (self.chip_type[0] == 0x19)
    @property
    def is_cc3230sm(self):
        return (self.chip_type[0] == 0x18 and self.chip_type[1] == 0xB1)
    @property
    def chip_name(self):
        if self.is_cc3220: return "CC3220"
        if self.is_cc3220s: return "CC3220S"
        if self.is_cc3220sf: return "CC3220SF"
        if self.is_cc3230sm: return "CC3230SM"
        return "CC3210"

    @classmethod
    def from_packet(cls, data):
        bootloader = tuple(map(ord, data[0:4]))
        nwp = tuple(map(ord, data[4:8]))
        mac = tuple(map(ord, data[8:12]))
        phy = tuple(map(ord, data[12:16]))
        chip_type = tuple(map(ord, data[16:20]))
        return cls(bootloader, nwp, mac, phy, chip_type)

    def __repr__(self):
        return "CC3x00VersionInfo({}, {}, {}, {}, {})".format(
            self.bootloader, self.nwp, self.mac, self.phy, self.chip_type)


class CC3x00StorageList(object):
    FLASH_BIT = 0x02
    SFLASH_BIT = 0x04
    SRAM_BIT = 0x80

    def __init__(self, value):
        self.value = value

    @property
    def flash(self):
        return (self.value & self.FLASH_BIT) != 0

    @property
    def sflash(self):
        return (self.value & self.SFLASH_BIT) != 0

    @property
    def sram(self):
        return (self.value & self.SRAM_BIT) != 0

    def __repr__(self):
        return "{}({})".format(self.__class__.__name__, hex(self.value))


class CC3x00StorageInfo(object):
    def __init__(self, block_size, block_count):
        self.block_size = block_size
        self.block_count = block_count

    @classmethod
    def from_packet(cls, data):
        bsize, bcount = struct.unpack(">HH", data[:4])
        return cls(bsize, bcount)

    def __repr__(self):
        return "{}(block_size={}, block_count={})".format(
            self.__class__.__name__, self.block_size, self.block_count)


class CC3x00Status(object):
    def __init__(self, value):
        self.value = value

    @property
    def is_ok(self):
        return self.value == 0x40

    @classmethod
    def from_packet(cls, packet):
        return cls(ord(packet[3]))


class CC3x00FileInfo(object):
    def __init__(self, exists, size=0):
        self.exists = exists
        self.size = size

    @classmethod
    def from_packet(cls, data):
        exists = data[0] == '\x01'
        size = struct.unpack(">I", data[4:8])[0]
        return cls(exists, size)


class CC3x00SffsStatsFileEntry(object):
    def __init__(self, index, start_block, size_blocks, mirrored, flags, fname):
        self.index = index
        self.start_block = start_block
        self.size_blocks = size_blocks
        self.mirrored = mirrored
        self.flags = flags
        self.fname = fname

        self.total_blocks = self.size_blocks
        if self.mirrored:
            self.total_blocks = self.total_blocks * 2


class CC3x00SffsHole(object):
    def __init__(self, start_block, size_blocks):
        self.start_block = start_block
        self.size_blocks = size_blocks


class CC3x00SffsHeader(object):
    SFFS_HEADER_SIGNATURE = 0x534c

    def __init__(self, fat_index, fat_bytes, storage_info):
        self.is_valid = False
        self.storage_info = storage_info

        if len(fat_bytes) != storage_info.block_size:
            raise CC3200Error("incorrect FAT size")

        """
        perform just a basic parsing for now, a caller will select a more
        relevant fat and then call get_sffs_stats() in order to initiate
        complete parsing
        """

        fat_commit_revision, header_sign = struct.unpack("<HH", fat_bytes[:4])

        if fat_commit_revision == 0xffff or header_sign == 0xffff:
            # empty FAT
            return

        if header_sign != self.SFFS_HEADER_SIGNATURE:
            log.warning("broken FAT: (invalid header signature: 0x%08x, 0x%08x)",
                        fat_commit_revision, header_sign)
            return

        self.fat_bytes = fat_bytes
        self.fat_commit_revision = fat_commit_revision
        log.info("[%d] detected a valid FAT revision: %d", fat_index, self.fat_commit_revision)
        self.is_valid = True


class CC3x00SffsInfo(object):
    SFFS_FAT_METADATA2_OFFSET = 0x774
    SFFS_FAT_FILE_NAME_ARRAY_OFFSET = 0x974

    def __init__(self, fat_header, storage_info):
        self.fat_commit_revision = fat_header.fat_commit_revision

        self.block_size = storage_info.block_size
        self.block_count = storage_info.block_count

        occupied_block_snippets = []

        self.used_blocks = 5  # FAT table size, as per documentation
        occupied_block_snippets.append((0, 5))

        self.files = []

        """
        TI's doc: "Total number of files is limited to 128 files, including
        system and configuration files"
        """
        for i in range(128):
            # scan the complete FAT table (as it appears to be)
            meta = fat_header.fat_bytes[(i + 1) * 4:(i + 2) * 4]

            if meta == "\xff\xff\xff\xff" or meta == struct.pack("BBBB", 0xff, i, 0xff, 0x7f):
                # empty entry in the middle of the FAT table
                continue

            index, size_blocks, start_block_lsb, flags_sb_msb = struct.unpack("BBBB", meta)
            if index != i:
                raise CC3200Error("incorrect FAT entry (index %d != %d)" % (index, i))

            """
            It's not completely clear, what all of these flags do mean, and
            where does the boundary between 'start block MSB' and 'flags'
            exactly lie.

            According to observations:
            - 0x8 seems to be set to '1' for all the files except for
                  /sys/mcuimg.bin (looks like this is the mark of the
                  user's app image for the CC3200's ROM bootloader)
            - 0x4 seems to be a negated flag of the mirrored/commit option

            - 4 LSB bits should be exactly enough to address the SFFS
                max size of 16 MB using 4K blocks
            """

            flags = flags_sb_msb >> 4
            start_block_msb = flags_sb_msb & 0xf

            mirrored = (flags & 0x4) == 0
            start_block = (start_block_msb << 8) + start_block_lsb

            meta2 = fat_header.fat_bytes[self.SFFS_FAT_METADATA2_OFFSET + i * 4:
                                         self.SFFS_FAT_METADATA2_OFFSET + (i + 1) * 4]
            fname_offset, fname_len = struct.unpack("<HH", meta2)
            fo_abs = self.SFFS_FAT_FILE_NAME_ARRAY_OFFSET + fname_offset
            fname = fat_header.fat_bytes[fo_abs:fo_abs + fname_len]

            entry = CC3x00SffsStatsFileEntry(i, start_block, size_blocks,
                                             mirrored, flags, fname)
            self.files.append(entry)

            occupied_block_snippets.append((start_block, entry.total_blocks))
            self.used_blocks = self.used_blocks + entry.total_blocks

        # in order to track the trailing "hole", like uniflash does
        occupied_block_snippets.append((self.block_count, 0))

        self.holes = []
        occupied_block_snippets.sort(key=lambda e: e[0])
        prev_end_block = 0
        for snippet in occupied_block_snippets:
            if snippet[0] < prev_end_block:
                for f in self.files:
                    log.info("[%d] block %d..%d fname=%s" %
                             (f.index, f.start_block, f.start_block + f.total_blocks, f.fname))
                raise CC3200Error("broken FAT: overlapping entry at block %d (prev end was %d)" %
                                  (snippet[0], prev_end_block))
            if snippet[0] > prev_end_block:
                hole = CC3x00SffsHole(prev_end_block, snippet[0] - prev_end_block - 1)
                self.holes.append(hole)
            prev_end_block = snippet[0] + snippet[1]

    def print_sffs_info(self):
        log.info("Serial Flash block size:\t%d bytes", self.block_size)
        log.info("Serial Flash capacity:\t%d blocks", self.block_count)
        log.info("")
        log.info("\tfile\tstart\tsize\tfail\tflags\ttotal size\tfilename")
        log.info("\tindex\tblock\t[BLKs]\tsafe\t\t[BLKs]")
        log.info("----------------------------------------------------------------------------")
        log.info("\tN/A\t0\t5\tN/A\tN/A\t5\t\tFATFS")
        for f in self.files:
            log.info("\t%d\t%d\t%d\t%s\t0x%x\t%d\t\t%s" %
                     (f.index, f.start_block, f.size_blocks,
                      f.mirrored and "yes" or "no",
                      f.flags, f.total_blocks, f.fname))

        log.info("")
        log.info("   Flash usage")
        log.info("-------------------------")
        log.info("used space:\t%d blocks", self.used_blocks)
        log.info("free space:\t%d blocks",
                 self.block_count - self.used_blocks)

        for h in self.holes:
            log.info("memory hole:\t[%d-%d]", h.start_block,
                     h.start_block + h.size_blocks)

    def print_sffs_info_short(self):
        log.info("FAT r%d, num files: %d, used/free blocks: %d/%d",
                 self.fat_commit_revision, len(self.files), self.used_blocks,
                 self.block_count - self.used_blocks)

    def print_sffs_info_json(self):
        print json.dumps(self, cls=CustomJsonEncoder)


class CustomJsonEncoder(json.JSONEncoder):
    def default(self, o):
        return o.__dict__


class CC3200Connection(object):

    TIMEOUT = 60
    DEFAULT_SLFS_SIZE = "1M"

    def __init__(self, port, reset=None, break_cycles=10, sop2=None, reset_pin=None, erase_timeout=ERASE_TIMEOUT):
        self.port = port
        port.timeout = self.TIMEOUT
        self._reset = reset
        self._break_cycles = break_cycles
        self._reset_pin = reset_pin
        self._sop2 = sop2
        self._erase_timeout = erase_timeout

        self.vinfo = None
        self.vinfo_apps = None

    @contextmanager
    def _serial_timeout(self, timeout=None):
        if timeout is None:
            yield self.port
            return
        if timeout == self.port.timeout:
            yield self.port
            return
        orig_timeout, self.port.timeout = self.port.timeout, timeout
        yield self.port
        self.port.timeout = orig_timeout

    def _set_sop2(self, level):
        if self._sop2.pin == "none":
            return

        toset = level ^ self._sop2.invert
        if self._sop2.pin == 'dtr':
            self.port.dtr = toset
        if self._sop2.pin == 'rts':
            self.port.rts = toset

    def _do_reset(self, sop2):
        self._set_sop2(sop2)

        if self._reset.pin == "none":
            return

        if self._reset.pin == "prompt":
            print("Reset the device with SOP2 {}asserted and press Enter".format(
                '' if sop2 else 'de'
            ))
            raw_input()
            return
        
        # in_reset = True ^ self._reset.invert
        self._set_reset_pin(True)
        time.sleep(.1)
        self._set_reset_pin(False)
        
    def _set_reset_pin(self, state):
        state = not state if self._reset.invert else state
        if self._reset.pin == 'dtr':
            log.info('Setting dtr pin: {}'.format(state))
            self.port.dtr = int(state)

        if self._reset.pin == 'rts':
            log.info('Setting rst pin: {}'.format(state))
            self.port.rts = int(state)

        if self._reset.pin == 'gpio':
            _pin = self._reset_pin
            log.info('Setting GPIO {} pin: {}'.format(_pin, state))
            subprocess.call(['gpio', 'mode', _pin, 'out'])
            subprocess.call(['gpio', 'write', _pin, str(int(state))])


    def _read_ack(self, timeout=None):
        ack_bytes = []
        with self._serial_timeout(timeout) as port:
            while True:
                b = port.read(1)
                if not b:
                    log.error("timed out while waiting for ack")
                    return False
                ack_bytes.append(b)
                if len(ack_bytes) > 2:
                    ack_bytes.pop(0)
                if ack_bytes == ['\x00', '\xCC']:
                    return True

    def _read_packet(self, timeout=None):
        with self._serial_timeout(timeout) as port:
            header = port.read(3)
            if len(header) != 3:
                raise CC3200Error("read_packed timed out on header")
            len_bytes = header[:2]
            csum_byte = header[2]

        data_len = struct.unpack(">H", len_bytes)[0] - 2
        with self._serial_timeout(timeout):
            data = self.port.read(data_len)

        if (len(data) != data_len):
            raise CC3200Error("did not get entire response")

        ccsum = 0
        for x in data:
            ccsum += ord(x)
        ccsum = ccsum & 0xff
        if ccsum != ord(csum_byte):
            raise CC3200Error("rx csum failed")

        self._send_ack()
        return data

    def _send_packet(self, data, timeout=None):
        assert len(data)
        checksum = 0
        for b in data:
            checksum += ord(b)
        len_blob = struct.pack(">H", len(data) + 2)
        csum = struct.pack("B", checksum & 0xff)
        if self.port.in_waiting or self.port.out_waiting:
            log.info("There are bytes in waiting")
        self.port.write(len_blob + csum + data)
        time.sleep(0.001)
        if not self._read_ack(timeout):
            raise CC3200Error(
                    "No ack for packet opcode=0x{:02x}".format(ord(data[0])))

    def _send_ack(self):
        self.port.write('\x00\xCC')

    def _get_last_status(self):
        self._send_packet(OPCODE_GET_LAST_STATUS)
        status = self._read_packet()
        log.debug("get last status got %s", hexify(status))
        return CC3x00Status(ord(status))

    def _do_break(self, timeout, break_cycles):

        time.sleep(0.8)
        self._set_reset_pin(True)
        log.info("break_on")
        for i in range(break_cycles):
            self.port.send_break()
        log.info("break_off")

        if self._read_ack(0.1):
            return True
        self.port.send_break(1.0)
        if self._read_ack(timeout):
            return True
        else:
            self._set_reset_pin(False)
            return False
        
    def _do_break_rpi(self, timeout, break_cycles):
        log.info("break_on")
        self.port.break_condition = True
        self._set_reset_pin(True)
        time.sleep(0.1)
        self._set_reset_pin(False)
        ack = self._read_ack(timeout=5)
        self.port.break_condition = False
        log.info("break_off")
        return ack


    def _try_breaking(self, tries=7, timeout=2):
        if platform.system() == 'Darwin': # For mac os
            break_cycles = 3
        elif platform.system() == 'Linux':
            break_cycles = 10
        for _ in range(tries):
            if self._reset.pin == 'gpio':
                if self._do_break_rpi(timeout, break_cycles):
                    break
            else:
                if self._do_break(timeout, break_cycles):
                    break
            if platform.system() == 'Linux':
                break_cycles = break_cycles + 1

    def _get_version(self):
        self._send_packet(OPCODE_GET_VERSION_INFO)
        version_data = self._read_packet()
        if len(version_data) != 28:
            raise CC3200Error("Version info should be 28 bytes, got {}"
                              .format(len(version_data)))
        return CC3x00VersionInfo.from_packet(version_data)

    def _get_storage_list(self):
        log.info("Getting storage list...")
        self._send_packet(OPCODE_GET_STORAGE_LIST)
        with self._serial_timeout(.5):
            slist_byte = self.port.read(1)
            if len(slist_byte) != 1:
                raise CC3200Error("Did not receive storage list byte")
        return CC3x00StorageList(ord(slist_byte))

    def _get_storage_info(self, storage_id=STORAGE_ID_SRAM):
        log.info("Getting storage info...")
        self._send_packet(OPCODE_GET_STORAGE_INFO +
                          struct.pack(">I", storage_id))
        sinfo = self._read_packet()
        if len(sinfo) < 4:
            raise CC3200Error("getting storage info got {} bytes"
                              .format(len(sinfo)))
        log.info("storage #%d info bytes: %s", storage_id, ", "
                 .join([hex(ord(x)) for x in sinfo]))
        return CC3x00StorageInfo.from_packet(sinfo)

    def erase_raw_storage(self, storage_id, start, count):
        log.info('erasing storage %d, start: %d, blocks: %d', storage_id, start, count)
        self._erase_blocks(start, count, storage_id)
        s = self._get_last_status()
        if not s.is_ok:
            raise CC3200Error('Raw Storage Erase failed, storage_id: 0')

    def write_raw_storage(self, storage_id, offset, data):
        log.info('writting %d bytes to storage %d+%d...', len(data), storage_id, offset)
        chunk_size = 4080
        sent = 0
        while sent < len(data):
            chunk = data[sent:sent+chunk_size]
            self._send_chunk(offset + sent, chunk, storage_id)
            s = self._get_last_status()
            if not s.is_ok:
                raise CC3200Error('Raw Stroage Write failed, sent: %d', sent)
            else:
                log.info('file chunk %d: %d write success', sent, len(chunk))
            sent += len(chunk)
        s = self._get_last_status()
        if not s.is_ok:
            raise CC3200Error('Raw Stroage Write failed, sent: %d', sent)
        log.info('write %d bytes to storage %d+%d success', len(data), storage_id, offset)

    def _fs_programming(self, flags, chunk, key=''):
        command = OPCODE_FS_PROGRAMMING + \
            struct.pack(">HHI", len(key), len(chunk), flags)
        #log.info('FS programming header: %s', hexify(command + key))
        self._send_packet(command + key + chunk)
        response = self.port.read(4)
        assert len(response) == 4
        status = 0
        for b in response:
            status = (status << 8) + ord(b)
        #log.info('FS programming request: %d, response %s: %d', len(chunk), hexify(response), status)
        return status
    
    def _erase_blocks(self, start, count, storage_id=STORAGE_ID_SRAM):
        command = OPCODE_RAW_STORAGE_ERASE + \
            struct.pack(">III", storage_id, start, count)
        self._send_packet(command, timeout=self._erase_timeout)

    def _send_chunk(self, offset, data, storage_id=STORAGE_ID_SRAM):
        command = OPCODE_RAW_STORAGE_WRITE + \
            struct.pack(">III", storage_id, offset, len(data))
        self._send_packet(command + data)

    def _raw_write(self, offset, data, storage_id=STORAGE_ID_SRAM):
        slist = self._get_storage_list()
        if storage_id == STORAGE_ID_SFLASH and not slist.sflash:
            raise CC3200Error("no serial flash?!")
        if storage_id == STORAGE_ID_SRAM and not slist.sram:
            raise CC3200Error("no sram?!")

        sinfo = self._get_storage_info(storage_id)
        bs = sinfo.block_size
        if bs > 0:
            count = len(data) / bs
            if count % bs:
                count += 1

        chunk_size = 4080
        sent = 0
        data_len = len(data)
        while sent < data_len:
            chunk = data[sent:sent + chunk_size]
            self._send_chunk(offset + sent, chunk, storage_id)
            sent += len(chunk)
            sys.stdout.write('\rProgress:%d%% ' % (sent * 100 / data_len))
            sys.stdout.flush()
        sys.stdout.write(os.linesep)

    def _raw_write_file(self, offset, filename, storage_id=STORAGE_ID_SRAM):
        with open(filename, 'r') as f:
            data = f.read()
            return self._raw_write(offset, data, storage_id)

    def _read_chunk(self, offset, size, storage_id=STORAGE_ID_SRAM):
        # log.info("Reading chunk at 0x%x size 0x%x..." % (offset, size))
        command = OPCODE_RAW_STORAGE_READ + \
            struct.pack(">III", storage_id, offset, size)
        self._send_packet(command)
        data = self._read_packet()
        if len(data) != size:
            raise CC3200Error("invalid received size: %d vs %d" % (len(data), size))
        return data

    def _raw_read(self, offset, size, storage_id=STORAGE_ID_SRAM, sinfo=None):
        slist = self._get_storage_list()
        if storage_id == STORAGE_ID_SFLASH and not slist.sflash:
            raise CC3200Error("no serial flash?!")
        if storage_id == STORAGE_ID_SRAM and not slist.sram:
            raise CC3200Error("no sram?!")

        if not sinfo:
            sinfo = self._get_storage_info(storage_id)
        storage_size = sinfo.block_count * sinfo.block_size

        if offset > storage_size:
            raise CC3200Error("offset %d is bigger than available mem %d" %
                              (offset, storage_size))

        if size < 1:
            size = storage_size - offset
            log.info("Setting raw read size to maximum: %d", size)
        elif size + offset > storage_size:
            raise CC3200Error("size %d + offset %d is bigger than available mem %d" %
                              (size, offset, storage_size))

        log.info("Reading raw storage #%d start 0x%x, size 0x%x..." %
                 (storage_id, offset, size))

        # XXX 4096 works faster, but 256 was sniffed from the uniflash
        chunk_size = 4096
        rx_data = ''
        while size - len(rx_data) > 0:
            rx_data += self._read_chunk(offset + len(rx_data),
                                        min(chunk_size, size - len(rx_data)),
                                        storage_id)
            sys.stderr.write('.')
        sys.stderr.write("\n")
        return rx_data

    def _exec_from_ram(self):
        self._send_packet(OPCODE_EXEC_FROM_RAM)

    def _get_file_info(self, filename):
        command = OPCODE_GET_FILE_INFO \
            + struct.pack(">I", len(filename)) \
            + filename
        self._send_packet(command)
        finfo = self._read_packet()
        if len(finfo) < 5:
            raise CC3200Error()
        return CC3x00FileInfo.from_packet(finfo)

    def _open_file_for_write(self, filename, file_len, fs_flags=None):
        for bsize_idx, bsize in enumerate(FLASH_BLOCK_SIZES):
            if (bsize * 255) >= file_len:
                blocks = int(math.ceil(float(file_len) / bsize))
                break
        else:
            raise CC3200Error("file is too big")

        fs_access = SLFS_MODE_OPEN_WRITE_CREATE_IF_NOT_EXIST
        flags = (((fs_access & 0x0f) << 12) |
                 ((bsize_idx & 0x0f) << 8) |
                 (blocks & 0xff))

        if fs_flags is not None:
            flags |= (fs_flags & 0xff) << 16

        return self._open_file(filename, flags)

    def _open_file_for_read(self, filename):
        return self._open_file(filename, 0)

    def _open_file(self, filename, slfs_flags):
        command = OPCODE_START_UPLOAD + struct.pack(">II", slfs_flags, 0) + \
            filename + '\x00\x00'
        self._send_packet(command)

        token = self.port.read(4)
        if not len(token) == 4:
            raise CC3200Error("open")

    def _close_file(self, signature=None):
        if signature is None:
            signature = '\x46' * 256
        if len(signature) != 256:
            raise CC3200Error("bad signature length")
        command = OPCODE_FINISH_UPLOAD
        command += '\x00' * 63
        command += signature
        command += '\x00'
        self._send_packet(command)
        s = self._get_last_status()
        if not s.is_ok:
            raise CC3200Error("closing file failed")

    def connect(self):
        log.info("Connecting to target...")
        self.port.flushInput()
        self._try_breaking(tries=5, timeout=2)
        log.info("Connected, reading version...")
        self.vinfo = self._get_version()

    def reboot_to_app(self):
        log.info("Rebooting to application")
        self._do_reset(False)

    def switch_to_nwp_bootloader(self):
        log.info("Switching to NWP bootloader...")
        vinfo = self._get_version()
        if not vinfo.is_cc32xx:
            log.debug("This looks like the NWP already")
            return

        if vinfo.bootloader[1] < 1:
            log.warning("Unsupported bootloader")

        if vinfo.bootloader[1] == 3:
            # cesanta upload and exec rbtl3101_132.dll for this version
            # then do the UART switch
            raise CC3200Error("Not yet supported device (bootloader=3)")

        self.switch_uart_to_apps()

        if vinfo.bootloader[1] == 3:
            # should upload rbtl3100.dll
            raise CC3200Error("Not yet supported device (NWP bootloader=3)")

        if vinfo.bootloader[1] >= 4:
            log.info("Uploading rbtl3100s.dll...")
            sram_patches = os.path.join(CURRENT_PATH, 'dll/rbtl3100s.dll')
            self._raw_write(0, load_file(sram_patches))
            self._exec_from_ram()

            if not self._read_ack():
                raise CC3200Error("got no ACK after exec from ram")
        elif vinfo.bootloader[1] == 1:
            self._get_storage_info(0) # step 4
            self.erase_raw_storage(0, 0, 3) # step 5
            sram_patches = os.path.join(CURRENT_PATH, 'dll/gen2/BTL_ram.ptc')
            self.write_raw_storage(0, 0, load_file(sram_patches)) # step 6
            self._exec_from_ram() # step 7, initialization is completed.
            if not self._read_ack():
                raise CC3200Error('no second ACK after execute from RAM command')
            self._get_storage_info(2) # step 8
            self.erase_raw_storage(2, 33, 2) # step 9
            sflash_patches = os.path.join(CURRENT_PATH, 'dll/gen2/BTL_sflash.ptc')
            self.write_raw_storage(2, 33*4096+8, load_file(sflash_patches)) # step 10

    def switch_uart_to_apps(self):
        # ~ 1 sec delay by the APPS MCU
        log.info("Switching UART to APPS...")
        command = OPCODE_SWITCH_2_APPS + struct.pack(">I", 26666667)
        self._send_packet(command)
        log.info("Resetting communications ...")
        if self._reset.pin == 'gpio':
            for i in range(4):
                self.port.break_condition = True
                time.sleep(0.1)
                if self._read_ack():
                    self.port.break_condition = False
                    break
            else:
                raise CC3200Error("no ACK after Switch UART to APPS MCU command")
        elif platform.system() == 'Darwin': # mac os
            for i in range(3):
                self.port.send_break()
            if not self._read_ack():
                raise CC3200Error("no ACK after Switch UART to APPS MCU command")
        elif platform.system() == 'Linux':
            for i in range(self._break_cycles):
                self.port.send_break()
            if not self._read_ack():
                raise CC3200Error("no ACK after Switch UART to APPS MCU command")
        else:
            time.sleep(1)
            self.port.send_break(0.2)
            if not self._read_ack():
                raise CC3200Error("no ACK after Switch UART to APPS MCU command")
        for i in range(8):
            self.vinfo_apps = self._get_version()
            if self.vinfo.bootloader[1] >= 4:
                break

    def format_slfs(self, size=None):
        if size is None:
            size = self.DEFAULT_SLFS_SIZE

        if size not in SLFS_SIZE_MAP:
            raise CC3200Error("invalid SLFS size")

        size = SLFS_SIZE_MAP[size]

        log.info("Formatting flash with size=%s", size)
        command = OPCODE_FORMAT_FLASH \
            + struct.pack(">IIIII", 2, size/4, 0, 0, 2)

        self._send_packet(command)

        s = self._get_last_status()
        if not s.is_ok:
            raise CC3200Error("Format failed")

    def erase_file(self, filename, force=False):
        if not force:
            finfo = self._get_file_info(filename)
            if not finfo.exists:
                log.warn("File '%s' does not exist, won't erase", filename)
                return

        log.info("Erasing file %s...", filename)
        command = OPCODE_ERASE_FILE + struct.pack(">I", 0) + \
            filename + '\x00'
        self._send_packet(command)
        s = self._get_last_status()
        if not s.is_ok:
            raise CC3200Error("Erasing file failed: 0x{:02x}}".format(s.value))

    def write_file(self, local_file, cc_filename, sign_file=None, size=0, commit_flag=False):
        # size must be known in advance, so read the whole thing
        data = local_file.read()
        file_len = len(data)

        if not file_len:
            log.warn("Won't upload empty file")
            return

        sign_data = None
        fs_flags = None

        if commit_flag:
            fs_flags = SLFS_FILE_OPEN_FLAG_COMMIT

        if sign_file:
            sign_data = sign_file.read(256)
            fs_flags = (
                    SLFS_FILE_OPEN_FLAG_COMMIT |
                    SLFS_FILE_OPEN_FLAG_SECURE |
                    SLFS_FILE_PUBLIC_WRITE)

        finfo = self._get_file_info(cc_filename)
        if finfo.exists:
            log.info("File exists on target, erasing")
            self.erase_file(cc_filename)

        alloc_size_effective = alloc_size = max(size, file_len)

        if (fs_flags and fs_flags & SLFS_FILE_OPEN_FLAG_COMMIT):
            alloc_size_effective *= 2

        timeout = self.port.timeout
        if (alloc_size_effective > 200000):
            timeout = max(timeout, 5 * ((alloc_size_effective / 200000) + 1))  # empirical value is ~252925 bytes for 5 sec timeout

        log.info("Uploading file %s -> %s [%d, disk=%d]...",
                 local_file.name, cc_filename, alloc_size, alloc_size_effective)

        with self._serial_timeout(timeout):
            self._open_file_for_write(cc_filename, alloc_size, fs_flags)

        pos = 0
        while pos < file_len:
            chunk = data[pos:pos+SLFS_BLOCK_SIZE]
            command = OPCODE_FILE_CHUNK + struct.pack(">I", pos)
            command += chunk
            self._send_packet(command)
            res = self._get_last_status()
            if not res.is_ok:
                raise CC3200Error("writing at pos {} failed".format(pos))
            pos += len(chunk)
            sys.stderr.write('.')

        sys.stderr.write("\n")
        log.debug("Closing file ...")
        return self._close_file(sign_data)

    def read_file(self, cc_fname, local_file):
        finfo = self._get_file_info(cc_fname)
        if not finfo.exists:
            raise CC3200Error("{} does not exist on target".format(cc_fname))

        log.info("Reading file %s -> %s", cc_fname, local_file.name)

        self._open_file_for_read(cc_fname)

        pos = 0
        while pos < finfo.size:
            toread = min(finfo.size - pos, SLFS_BLOCK_SIZE)
            command = OPCODE_READ_FILE_CHUNK + struct.pack(">II", pos, toread)
            self._send_packet(command)
            resp = self._read_packet()
            if len(resp) != toread:
                raise CC3200Error("reading chunk failed")

            local_file.write(resp)
            pos += toread

        self._close_file()

    def write_flash(self, image, erase=True):
        data = image.read()
        data_len = len(data)

        if self.vinfo.bootloader[1] >= 4:
            if erase:
                count = int(math.ceil(data_len / float(SLFS_BLOCK_SIZE)))
                self._erase_blocks(0, count, storage_id=STORAGE_ID_SFLASH)
    
            self._raw_write(8, data[8:], storage_id=STORAGE_ID_SFLASH)
            self._raw_write(0, data[:8], storage_id=STORAGE_ID_SFLASH)
        else:
            log.info('flash image size: %d', data_len)
    
            flags = 0
            key_data = ''
            key_size = 0
            chunk_size = 4096
            # TODO: add decryption
            # if key:
            #     key_size = 16
            #     key_data = key.read()[:16]
            sent = 0
            while sent < data_len:
                chunk = data[sent: sent + chunk_size]
                status = self._fs_programming(flags, chunk, key_data)
                # assert (len(chunk) == chunk_size and status == sent) or status == 0
                log.debug('FS programming chunk %d:%d, status %d', sent, len(chunk), status)
                if (status != sent + chunk_size) and status != 0:
                    break
                sent += len(chunk)
                sys.stdout.write('\rProgress:%d%% ' % (sent * 100 / data_len))
                sys.stdout.flush()
            if data_len % chunk_size == 0:
                status = self._fs_programming(flags, '', '')
                log.info('FS programming status %d', status)
            if status:
                log.info('FS programming aborted, bad response')
            sys.stdout.write(os.linesep)   

    def read_flash(self, image_file, offset, size):
        data = self._raw_read(offset, size, storage_id=STORAGE_ID_SFLASH)
        image_file.write(data)

    def get_fat_info(self, inactive=False):
        sinfo = self._get_storage_info(storage_id=STORAGE_ID_SFLASH)

        fat_table_bytes = self._raw_read(0, 2 * sinfo.block_size,
                                         storage_id=STORAGE_ID_SFLASH,
                                         sinfo=sinfo)

        fat_table_bytes1 = fat_table_bytes[:sinfo.block_size]
        fat_table_bytes2 = fat_table_bytes[sinfo.block_size:]

        """
        In SFFS there're 2 entries of FAT, none of which has a fixed primary
        or secondary role. Instead, these entries are written interchangeably,
        with the newest one being marked with a larger 2-byte number, referred
        in this source code as 'fat_commit_revision' (this is a made-up term).

        The algorithm is described in detail here:
        http://processors.wiki.ti.com/index.php/CC3100_%26_CC3200_Serial_Flash_Guide#File_appending

        It was also noticed that after the successful write to a newer FAT,
        the older one might got overwritten with 0xFF by the CC3200's SFFS
        driver (effectively marking it as invalid), but not always.
        """

        fat_hdr1 = CC3x00SffsHeader(0, fat_table_bytes1, sinfo)
        fat_hdr2 = CC3x00SffsHeader(1, fat_table_bytes2, sinfo)

        fat_hdrs = []
        if fat_hdr1.is_valid:
            fat_hdrs.append(fat_hdr1)
        if fat_hdr2.is_valid:
            fat_hdrs.append(fat_hdr2)

        if len(fat_hdrs) == 0:
            raise CC3200Error("no valid fat tables found")

        if len(fat_hdrs) > 1:
            # find the latest
            fat_hdrs.sort(reverse=True, key=lambda e: e.fat_commit_revision)

        if inactive:
            if len(fat_hdrs) > 1:
                fat_hdr = fat_hdrs[1]
            else:
                raise CC3200Error("no valid inactive fat table found")
        else:
            fat_hdr = fat_hdrs[0]
        log.info("selected FAT revision: %d (%s)", fat_hdr.fat_commit_revision, inactive and 'inactive' or 'active')
        return CC3x00SffsInfo(fat_hdr, sinfo)

    def list_filesystem(self, json_output=False, inactive=False):
        fat_info = self.get_fat_info(inactive=inactive)
        fat_info.print_sffs_info()
        if json_output:
            fat_info.print_sffs_info_json()


def split_argv(cmdline_args):
    """Manually split sys.argv into subcommand sections

    The first returned element should contain all global options along with
    the first command. Subsequent elements will contain a command each, with
    options applicable for the specific command. This is needed so we can
    specify different --file-size for different write_file commands.
    """
    args = []
    have_cmd = False
    for x in cmdline_args:
        if x in subparsers.choices:
            if have_cmd:
                yield args
                args = []
            have_cmd = True
            args.append(x)
        else:
            args.append(x)

    if args:
        yield args


def main():
    commands = []
    for cmdargs in split_argv(sys.argv[1:]):
        commands.append(parser.parse_args(cmdargs))

    if len(commands) == 0:
        parser.print_help()
        sys.exit(-1)

    args = commands[0]

    sop2_method = args.sop2
    reset_method = args.reset
    break_cycles = int(args.break_duration)
    reset_pin = args.reset_pin
    if sop2_method.pin == reset_method.pin and reset_method.pin != 'none':
        log.error("sop2 and reset methods cannot be the same output pin")
        sys.exit(-3)

    port_name = args.port

    try:
        p = serial.Serial()
        p.baudrate=CC3200_BAUD
        p.port=port_name
        p.parity=serial.PARITY_NONE
        p.stopbits=serial.STOPBITS_ONE
        p.break_condition=True
        p.open()
    except (Exception, ) as e:
        log.warn("unable to open serial port %s: %s", port_name, e)
        sys.exit(-2)

    cc = CC3200Connection(p, reset_method, break_cycles, sop2_method, reset_pin, erase_timeout=args.erase_timeout)
    try:
        cc.connect()
        log.info("connected to target")
    except (Exception, ) as e:
        log.error("Could not connect to target: {}".format(e))
        sys.exit(-3)

    log.info("Version: %s", cc.vinfo)

    # TODO: sane error handling

    if cc.vinfo.is_cc32xx:
        log.info("This is a %s device", cc.vinfo.chip_name)
        if "find_device" in sys.argv:
            sys.exit(0)
        cc.switch_to_nwp_bootloader()
        log.info("APPS version: %s", cc.vinfo_apps)

    check_fat = False

    for command in commands:
        if command.cmd == "format_flash":
            cc.format_slfs(command.size)

        if command.cmd == 'write_file':
            cc.write_file(command.local_file, command.cc_filename,
                          command.signature, command.file_size,
                          command.commit_flag)
            check_fat = True

        if command.cmd == "read_file":
            cc.read_file(command.cc_filename, command.local_file)

        if command.cmd == "erase_file":
            log.info("Erasing file %s", command.filename)
            cc.erase_file(command.filename)

        if command.cmd == "write_flash":
            cc.write_flash(command.image_file, not command.no_erase)

        if command.cmd == "read_flash":
            cc.read_flash(command.dump_file, command.offset, command.size)

        if command.cmd == "list_filesystem":
            cc.list_filesystem(command.json_output, command.inactive)


    if check_fat:
        fat_info = cc.get_fat_info()  # check FAT after each write_file operation
        fat_info.print_sffs_info_short()

    if args.reboot_to_app:
        cc.reboot_to_app()

    log.info("All commands done, bye.")


if __name__ == '__main__':
    main()
