#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2020, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
########################################################################################################################
import time
import os
import csv

from cnspy_rosbag2csv.CSVLine2ROSMsg import CSVLine2ROSMsg
from cnspy_spatial_csv_formats.CSVSpatialFormat import CSVSpatialFormat
from cnspy_rosbag2csv.ROSMessageTypes import ROSMessageTypes

class CSVParser:
    curr_msg = None
    done = False
    t = None
    fmt = None  # CSVSpatialFormat
    file = None
    line_number = 0
    msg_type = None
    fn = None
    header = None

    def __init__(self, fn, msg_type=ROSMessageTypes.GEOMETRY_MSGS_POSEWITHCOVARIANCESTAMPED):
        self.fmt = CSVSpatialFormat.identify_format(fn)
        assert (self.fmt is not CSVSpatialFormat.type.none)
        self.fn = fn
        self.msg_type = msg_type

        self.file = open(fn, "r")
        line = self.file.readline()
        self.header = str(line).rstrip("\n\r")
        self.next_line()

    def __del__(self):
        # Closing files
        self.file.close()

    def next_line(self):
        if self.done:
            return False

        line = self.file.readline()
        if not line:
            self.done = True
            return False
        line = str(line).rstrip("\n\r")
        self.curr_msg, self.t = CSVLine2ROSMsg.to(self.fmt.type, line, self.line_number, self.msg_type)
        self.line_number += 1
        return True