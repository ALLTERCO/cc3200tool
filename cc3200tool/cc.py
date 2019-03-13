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

import serial

log = logging.getLogger()
logging.basicConfig(stream=sys.stderr, level=logging.INFO,
                    format="%(asctime)-15s -- %(message)s")
CURRENT_PATH = os.path.dirname(os.path.realpath(__file__))

CC3200_BAUD = 921600

OPCODE_START_UPLOAD = "\x21"
OPCODE_FINISH_UPLOAD = "\x22"
OPCODE_GET_LAST_STATUS = "\x23"
OPCODE_FILE_CHUNK = "\x24"
OPCODE_GET_STORAGE_LIST = "\x27"
OPCODE_FORMAT_FLASH = "\x28"
OPCODE_GET_FILE_INFO = "\x2A"
OPCODE_READ_FILE_CHUNK = "\x2B"
OPCODE_RAW_STORAGE_WRITE = "\x2D"
OPCODE_ERASE_FILE = "\x2E"
OPCODE_GET_VERSION_INFO = "\x2F"
OPCODE_RAW_STORAGE_ERASE = "\x30"
OPCODE_GET_STORAGE_INFO = "\x31"
OPCODE_EXEC_FROM_RAM = "\x32"
OPCODE_SWITCH_2_APPS = "\x33"
OPCODE_FS_PROGRAMMING = "\x34"

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
    ret = []
    for x in s:
        ret.append(hex(ord(x)))
    return " ".join(ret)

pincfg = namedtuple('pincfg', ['invert', 'pin'])


def pinarg(extra=None):
    choices = ['dtr', 'rts', 'none']
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
        return pincfg(invert, apin)

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
        "--sop2", type=pinarg(), default="none",
        help="dtr, rts or none, optinally prefixed by ~ to invert")

subparsers = parser.add_subparsers(dest="cmd")

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
        "--key", type=argparse.FileType('rb'), default=None,
        help="the image encrypting key")


def load_file(fname):
    data = []
    with open(fname, 'r') as f:
        data = f.read()
        assert len(data) > 0
    return data

def dll_data(fname):
    return get_data('cc3200tool', 'dll/{}'.format(fname))

class CC3200Error(Exception):
    pass


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
        return (self.chip_type[0] == 0x10)
    @property
    def is_cc3220s(self):
        return (self.chip_type[0] == 0x18)
    @property
    def is_cc3220sf(self):
        return (self.chip_type[0] == 0x19)
    @property
    def chip_name(self):
        if self.is_cc3220: return "CC3220"
        if self.is_cc3220s: return "CC3220S"
        if self.is_cc3220sf: return "CC3220SF"
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
        bsize, bcount = struct.unpack("<HH", data[:4])
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


class CC3200Connection(object):

    TIMEOUT = 60
    DEFAULT_SLFS_SIZE = "1M"

    def __init__(self, port, reset=None, sop2=None):
        self.port = port
        port.timeout = self.TIMEOUT
        self._reset = reset
        self._sop2 = sop2

        self.vinfo = None
        self.vinfo_apps = None
        self.sram_info = None
        self.sflash_info = None
        self.storage_list = None
        self.storage_infos = {}

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

    def _do_reset(self):
        if self._reset.pin == "none":
            return

        if self._reset.pin == "prompt":
            print("Reset the device with SOP2 asserted and press Enter")
            raw_input()
            return

        in_reset = True ^ self._reset.invert
        if self._reset.pin == 'dtr':
            self.port.dtr = in_reset
            time.sleep(.1)
            self.port.dtr = not in_reset

        if self._reset.pin == 'rts':
            self.port.rts = in_reset
            time.sleep(.1)
            self.port.rts = not in_reset

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
        self.port.write(len_blob + csum + data)
        # time.sleep(0.05)
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

    def _do_break(self, timeout, duration):
        time.sleep(duration)
        self.port.dtr = 0
        self.port.send_break(duration)
        self.port.send_break(duration)
        self.port.send_break(duration)
        return self._read_ack(timeout)

    def _try_breaking(self, tries=5, timeout=2, duration=1.5):
        for _ in range(tries):
            if self._do_break(timeout, duration):
                return True
            self.port.dtr = 1
        return False

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

    def _get_storage_info(self, storage_id=0):
        log.info("Getting storage info...")
        self._send_packet(OPCODE_GET_STORAGE_INFO +
                          struct.pack(">I", storage_id))
        sinfo = self._read_packet()
        if len(sinfo) < 4:
            raise CC3200Error("getting storage info got {} bytes"
                              .format(len(sinfo)))
        log.info("storage info bytes: %s", ", "
                 .join([hex(ord(x)) for x in sinfo]))
        return CC3x00StorageInfo.from_packet(sinfo)

    def _erase_blocks(self, start, count, storage_id=0):
        command = OPCODE_RAW_STORAGE_ERASE + \
            struct.pack(">III", storage_id, start, count)
        self._send_packet(command)

    def _send_chunk(self, offset, data, storage_id=0):
        command = OPCODE_RAW_STORAGE_WRITE + \
            struct.pack(">III", storage_id, offset, len(data))
        self._send_packet(command + data)

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

    def _raw_write(self, offset, data, storage_id=0):
        slist = self._get_storage_list()
        if not slist.sflash:
            raise CC3200Error("no serial flash?!")

        sinfo = self._get_storage_info(storage_id)
        bs = sinfo.block_size
        if bs > 0:
            start = offset / bs
            count = len(data) / bs
            if count % bs:
                count += 1

        chunk_size = 4080
        sent = 0
        while sent < len(data):
            chunk = data[sent:sent+chunk_size]
            self._send_chunk(offset + sent, chunk, storage_id)
            sent += len(chunk)

    def _raw_write_file(self, offset, filename, storage_id=0):
        with open(filename, 'r') as f:
            data = f.read()
            return self._raw_write(offset, data, storage_id)

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
            flags |= (fs_flags << 16) & 0xff

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
        self._set_sop2(True)
        self._do_reset()
        if not self._try_breaking(tries=5, timeout=2):
            raise CC3200Error("Did not get ACK on break condition")
        log.info("Connected, get storage list...")
        self.storage_list = self._get_storage_list()
        log.info('Get Storage List:' + str(self.storage_list))

    def detect_target_type(self):
        self.vinfo = self._get_version()
        log.info("got version ACK: %s", self.vinfo)
        log.info("This is a %s device", self.vinfo.chip_name)

    def switch_to_nwp_bootloader(self):
        log.info("Switching to NWP bootloader...")
        if not self.vinfo or not self.vinfo.is_cc32xx:
            log.debug("This looks like the NWP already")
            return

        log.info("vinfo: " + str(self.vinfo))
        if not self.vinfo.is_cc32xx:
            raise CC3200Error("Unsupported device")

        if self.vinfo.bootloader[1] == 3:
            # cesanta upload and exec rbtl3101_132.dll for this version
            # then do the UART switch
            raise CC3200Error("Not yet supported device (bootloader=3)")

        self.switch_uart_to_apps()

        if self.vinfo.bootloader[1] == 3:
            # should upload rbtl3100.dll
            raise CC3200Error("Not yet supported device (NWP bootloader=3)")

        for i in range(8):
            self.vinfo_apps = self._get_version()


    def switch_uart_to_apps(self):
        # ~ 1 sec delay by the APPS MCU
        log.info("Switching UART to APPS...")
        command = OPCODE_SWITCH_2_APPS + struct.pack(">I", 26666667)
        self._send_packet(command)
        log.info("Resetting communications ...")
        self.port.send_break()
        self.port.send_break()
        self.port.send_break()
        if not self._read_ack():
            raise CC3200Error("no ACK after Switch UART to APPS MCU command")
        else:
            log.info('got ACK after Switch UART to APPS MCU command')

    def get_storage_info(self, storage_id):
        sinfo = self._get_storage_info(storage_id)
        log.info('got stroage %d info: %s', storage_id, str(sinfo))
        self.storage_infos[storage_id] = sinfo

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

    def write_file(self, local_file, cc_filename, sign_file=None, size=0):
        # size must be known in advance, so read the whole thing
        data = local_file.read()
        file_len = len(data)

        if not file_len:
            log.warn("Won't upload empty file")
            return

        sign_data = None
        fs_flags = None
        if sign_file:
            sign_data = sign_file.read(256)
            fs_flags = (
                    SLFS_FILE_OPEN_FLAG_SECURE |
                    SLFS_FILE_PUBLIC_WRITE |
                    SLFS_FILE_PUBLIC_READ)

        finfo = self._get_file_info(cc_filename)
        log.info('get file info: ' + str(finfo))
        if finfo.exists:
            log.info("File exists on target, erasing")
            self.erase_file(cc_filename)

        alloc_size = max(size, file_len)

        timeout = self.port.timeout
        if (alloc_size > 200000):
            timeout = max(timeout, 5 * ((alloc_size / 200000) + 1)) # empirical value is ~252925 bytes for 5 sec timeout

        log.info("Uploading file %s -> %s [%d]...",
                local_file.name, cc_filename, alloc_size)

        with self._serial_timeout(timeout):
            self._open_file_for_write(cc_filename, alloc_size)

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

    def write_flash(self, image, key):
        data = image.read()
        data_len = len(data)
        log.info('flash image size: %d', data_len)

        flags = 0
        key_data = ''
        key_size = 0
        chunk_size = 4096
        if key:
            key_size = 16
            key_data = key.read()[:16]
        sent = 0
        while sent < data_len:
            chunk = data[sent: sent + chunk_size]
            status = self._fs_programming(flags, chunk, key_data)
            # assert (len(chunk) == chunk_size and status == sent) or status == 0
            log.info('FS programming chunk %d:%d, status %d', sent, len(chunk), status)
            sent += len(chunk)
        if data_len % chunk_size == 0:
            status = self._fs_programming(flags, '', '')
            log.info('FS programming status %d', status)
            # assert status == 0

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

    cc = CC3200Connection(p, reset_method, sop2_method)
    try:
        cc.connect() # step 1
        log.info("connected to target")
    except (Exception, ) as e:
        log.error("Could not connect to target: {}".format(e))
        sys.exit(-3)

    # TODO: sane error handling
    cc.detect_target_type() # step 2

    if cc.vinfo.is_cc32xx:
        cc.switch_to_nwp_bootloader() # step 3
        log.info("APPS version: %s", cc.vinfo_apps)

    cc.get_storage_info(0) # step 4
    cc.erase_raw_storage(0, 0, 3) # step 5

    sram_patches = os.path.join(CURRENT_PATH, 'dll/gen2/BTL_ram.ptc')
    cc.write_raw_storage(0, 0, load_file(sram_patches)) # step 6
    cc._exec_from_ram() # step 7, initialization is completed.
    if not cc._read_ack():
        raise CC3200Error('no second ACK after execute from RAM command')

    cc.get_storage_info(2) # step 8
    cc.erase_raw_storage(2, 33, 2) # step 9
    sflash_patches = os.path.join(CURRENT_PATH, 'dll/gen2/BTL_sflash.ptc')
    cc.write_raw_storage(2, 33*4096+8, load_file(sflash_patches)) # step 10

    for command in commands:
        if command.cmd == "format_flash":
            cc.format_slfs(command.size)

        if command.cmd == 'write_file':
            cc.write_file(command.local_file, command.cc_filename,
                          command.signature, command.file_size)

        if command.cmd == "read_file":
            cc.read_file(command.cc_filename, command.local_file)

        if command.cmd == "erase_file":
            log.info("Erasing file %s", command.filename)
            cc.erase_file(command.filename)

        if command.cmd == "write_flash":
            cc.write_flash(command.image_file, command.key)

    log.info("All commands done, bye.")

if __name__ == '__main__':
    main()
