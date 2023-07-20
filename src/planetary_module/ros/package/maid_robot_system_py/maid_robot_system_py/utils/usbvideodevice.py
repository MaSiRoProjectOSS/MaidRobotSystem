#!/usr/bin/env python3.10

import subprocess


class UsbVideoDevice():
    _deviceList = []

    def display(self):
        for (row_id, row_name, row_path) in self._deviceList:
            print("{} : {}".format(row_path, row_name))

    def get_id_from_name(self, name):
        for (row_id, row_name, row_path) in self._deviceList:
            if (name in row_name):
                return row_id
        return -1

    def get_id_from_id(self, id):
        for (row_id, row_name, row_path) in self._deviceList:
            if (id == row_id):
                return row_id
        return -1

    def get_path(self, path):
        for (_, name, p) in self._deviceList:
            if (path in p):
                return p
        return ''

    def __init__(self, device_name):
        self._deviceList = []

        try:
            cmd = 'ls -la /dev/' + device_name + '/by-path'
            res = subprocess.check_output(cmd.split())
            by_path = res.decode()

            for line in by_path.split('\n'):
                if ('usb' in line):
                    tmp = self._split(line, ' ')
                    name = tmp[8]
                    tmp2 = self._split(tmp[10], '../../video')
                    deviceId = int(tmp2[0])
                    self._deviceList.append(
                        (deviceId, name, '/dev/video' + str(deviceId)))
        except:
            return

    def _split(self, str, val):
        tmp = str.split(val)
        if ('' in tmp):
            tmp.remove('')
        return tmp
