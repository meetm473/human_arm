#!/usr/bin/env python3

import threading
import socket
from d_sensorFusion import Fusion


class RecvWifiData(threading.Thread):

    def __init__(self, port=1620, buffer=2048):
        threading.Thread.__init__(self)
        self.port = port
        self.buffer = buffer
        self.show_raw_data = False
        self.start_server = True
        self.th_fusion = Fusion()

    def run(self):
        print(f'Starting the server on port: {self.port}')
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('', self.port))
        s.listen(1)
        
        
        self.th_fusion.daemon = True
        self.th_fusion.start()
        
        while self.start_server:
            client, _ = s.accept()
            data = str(client.recv(self.buffer).decode('utf-8')).split(',')
            if (len(data) == 6 or len(data) == 9):
                self.th_fusion.rawData.put(data)
            if self.show_raw_data:
                print(data)
        try:
            s.shutdown(socket.SHUT_RDWR)
        except (socket.error, OSError, ValueError, ConnectionResetError):
            pass
        finally:
            s.close()
            self.th_fusion.stop_fusion = True
            self.th_fusion.join()
            print('Server closed')


class SendWifiData(threading.Thread):
    def __init__(self, port=1620):
        threading.Thread.__init__(self)
        self.port = port

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('127.0.0.1', self.port))
        print('Sending close command...')
        try:
            s.send(bytes('8', 'utf-8'))
        finally:
            s.close()

if __name__ == '__main__':
	print('Please run the script a_imu_control.py')
