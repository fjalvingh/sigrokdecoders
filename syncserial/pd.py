##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2010-2016 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

# TODO: Look into arbitration, collision detection, clock synchronisation, etc.
# TODO: Implement support for inverting SDA/SCL levels (0->1 and 1->0).
# TODO: Implement support for detecting various bus errors.

from common.srdhelper import bitpack_msb
import sigrokdecode as srd
from math import floor, ceil

'''
OUTPUT_PYTHON format:

Packet:
[<ptype>, <pdata>]

<ptype>:
 - 'WORD' (a single word read)
 - 'BITS' (<pdata>: list of data/address bits and their ss/es numbers)

<pdata> is the data or address byte associated with the 'ADDRESS*' and 'DATA*'
command. Slave addresses do not include bit 0 (the READ/WRITE indication bit).
For example, a slave address field could be 0x51 (instead of 0xa2).
For 'WORD' <pdata> is None.
For 'BITS' <pdata> is a sequence of tuples of bit values and their start and
stop positions, in LSB first order (although the I2C protocol is MSB first).
'''

# Meaning of table items:
# command -> [annotation class, annotation text in order of decreasing length]
proto = {
    'BIT':           [0, '{b:1d}'],
}

class Decoder(srd.Decoder):
    api_version = 3
    id = 'syncserial'
    name = 'syncserial'
    longname = 'Synchronous serial communication'
    desc = 'Two-wire serial bus.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['words']
    tags = ['Embedded/industrial']
    channels = (
            {'id': 'clk', 'type': 8, 'name': 'Clock', 'desc': 'Serial clock line', 'idn':'dec_sysy_chan_clk'},
            {'id': 'data', 'type': 108, 'name': 'data', 'desc': 'Serial data line', 'idn':'dec_sysy_chan_data'},
    )
    options = (
        {'id' : 'clock_edge', 'desc': 'Edge of the clock that samples the data',
          'default': 'rising', 'values': ('rising', 'falling')
         },
        {'id': 'bits', 'desc': 'The #bits in a word', 'default': 8 },
        {'id': 'bitorder', 'desc': 'Reverse the bit order, i.e the 1st bit received is a high-order bit',
            'default': 'msb first', 'values': ('lsb first', 'msb first')}
    )
    annotations = (
        ('bit', 'Data/address bit'),
        ('word', 'Word read'),
    )
    annotation_rows = (
        ('bits', 'Bits', (0, )),
        ('Words', 'Words', (1, )),
    )
    binary = (
        ('word-read', 'Word read'),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.pdu_start = None
        self.pdu_bits = 0
        self.data_bits = []
        self.bitwidth = 0
        self.lastsamplenum = 0
        self.mindelta = None
        self.bitstartsample = 0
        self.lastbit = 0
        self.bitlencount = 0
        self.bitlensum = 0
        self.nibblecount = 0            # bit counter inside nibble
        self.nibble = 0                 # The actual nibble being collected
        self.nibblestartsample = 0

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)
        self.out_bitrate = self.register(srd.OUTPUT_META,
                meta=(int, 'Bitrate', 'Bitrate from Start bit to Stop bit'))

    def putg(self, ss, es, cls, text):
        self.put(ss, es, self.out_ann, [cls, text])

    def putp(self, ss, es, data):
        self.put(ss, es, self.out_python, data)

    def putb(self, ss, es, data):
        self.put(ss, es, self.out_binary, data)

    def decode(self):
        clock_edge = self.options['clock_edge']
        wordlen = self.options['bits']
        lsbfirst = self.options['bitorder'] == 'lsb first'

        while True:
            try:
                (clk, sda) = self.wait([{0: 'r'}, {0: 'f'}])      # Rising or falling clock
            except:
                # print("EOF Error!!!")
                # Samples exhausted. Finish the last run, if present
                if self.nibblecount > 0:
                    if lsbfirst:
                        self.nibble = (self.nibble << 1) | self.lastbit
                    else:
                        self.nibble |= (self.lastbit << self.nibblecount)
                    bitlen = self.samplenum - self.bitstartsample  # bit length
                    average = 0 if self.bitlencount == 0 else ceil(self.bitlensum / self.bitlencount)
                    self.put(self.bitstartsample, self.bitstartsample + average, self.out_ann, [0, [str(self.lastbit)]])
                    self.put(self.nibblestartsample, self.bitstartsample + average, self.out_ann, [1, ["%0.1X" % self.nibble]])
                    raise EOFError("No more samples")

            if (clock_edge == 'rising' and clk == 1) or (clock_edge == 'falling' and clk == 0):
                # We are at an active edge. If a previous bit is pending render it, then store the bit read now
                if self.bitlencount > 0:
                    bitlen = self.samplenum - self.bitstartsample  # bit length
                    es = self.samplenum  # proposed ann end
                    average = 0 if self.bitlencount == 0 else ceil(self.bitlensum / self.bitlencount)
                    truncate = False
                    if self.bitlencount > 3:
                        if bitlen > average * 4:
                            es = self.bitstartsample + average * 4
                            bitlen = average * 4
                            # print("bitlen truncated")
                            truncate = True

                    # put the bit
                    self.put(self.bitstartsample, es, self.out_ann, [0, [str(self.lastbit)]])

                    # collect the nibble and put it if we have 4 bits (or a truncate)
                    if lsbfirst:
                        self.nibble = (self.nibble << 1) | self.lastbit
                    else:
                        self.nibble |= (self.lastbit << self.nibblecount)
                    self.nibblecount += 1
                    if self.nibblestartsample == 0:
                        self.nibblestartsample = self.bitstartsample

                    if self.nibblecount >= wordlen or truncate:
                        self.put(self.nibblestartsample, es, self.out_ann, [1, ["%0.1X" % self.nibble]])
                        self.nibblecount = 0
                        self.nibble = 0
                        self.nibblestartsample = 0

                    # calculate bit length
                    if self.bitstartsample != 0:
                        self.bitlencount += 1
                        self.bitlensum += bitlen
                    # print(f"{self.bitlencount=} {self.bitlensum=} {bitlen=} {average=}")

                self.bitstartsample = self.samplenum
                self.lastbit = sda
                self.bitlencount += 1

                self.lastsamplenum = self.samplenum

            elif self.lastsamplenum == 0:
                self.bitstartsample = self.samplenum
                self.nibblestartsample = self.samplenum
            # else:
                # Inactive edge: show the last bit read
                # bitlen = self.samplenum - self.bitstartsample       # bit length
                # es = self.samplenum                                 # proposed ann end
                # average = 0 if self.bitlencount == 0 else ceil(self.bitlensum / self.bitlencount)
                # truncate = False
                # if self.bitlencount > 3:
                #     if bitlen > average * 4:
                #         es = self.bitstartsample + average * 4
                #         bitlen = average * 4
                #         # print("bitlen truncated")
                #         truncate = True
                #
                # # put the bit
                # self.put(self.bitstartsample, es, self.out_ann, [0, [str(self.lastbit)]])
                #
                # # collect the nibble and put it if we have 4 bits (or a truncate)
                # self.nibble = (self.nibble << 1) | self.lastbit
                # self.nibblecount += 1
                # if self.nibblestartsample == 0:
                #     self.nibblestartsample = self.bitstartsample
                #
                # if self.nibblecount >= 4 or truncate:
                #     self.put(self.nibblestartsample, es, self.out_ann, [1, ["%0.1X" % self.nibble]])
                #     self.nibblecount = 0
                #     self.nibble = 0
                #     self.nibblestartsample = 0

                # print(f"{self.bitstartsample} {self.samplenum} {self.lastbit}")

            #     # calculate bit length
            #     if self.bitstartsample != 0:
            #         self.bitlencount += 1
            #         self.bitlensum += bitlen
            #     # print(f"{self.bitlencount=} {self.bitlensum=} {bitlen=} {average=}")
            #     self.bitstartsample = self.samplenum
            #
            # self.lastsamplenum = self.samplenum

