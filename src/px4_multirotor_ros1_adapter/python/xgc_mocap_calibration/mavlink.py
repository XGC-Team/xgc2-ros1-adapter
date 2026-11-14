"""Small MAVLink common-dialect reader for calibration-only messages.

IDs, layouts and CRC extras follow the pinned common MAVLink headers. Outgoing
operations are restricted to heartbeat, TIMESYNC and message-interval commands.
No flight-control command is exposed.
"""
import socket
import struct
import time

CRC = {0:50, 76:152, 77:143, 105:93, 111:34, 244:95}


def checksum(data):
    value=0xffff
    for byte in data:
        tmp=byte^(value&0xff);tmp^=(tmp<<4)&0xff
        value=(value>>8)^(tmp<<8)^(tmp<<3)^(tmp>>4)
    return value


class Link:
    def __init__(self, system_id):
        self.system_id=system_id;self.component=None;self.buffer=bytearray();self.sequence=0
        self.socket=socket.create_connection(('127.0.0.1',5760),timeout=5)
        self.socket.settimeout(.1)

    def send(self, message_id,payload):
        if message_id not in (0,76,111): raise ValueError('calibration message only')
        header=struct.pack('<BBBBBB',len(payload),self.sequence,253,197,message_id,0)[:5]
        self.sequence=(self.sequence+1)%256
        crc=checksum(header+payload+bytes([CRC[message_id]]))
        self.socket.sendall(b'\xfe'+header+payload+struct.pack('<H',crc))

    def heartbeat(self):
        self.send(0,struct.pack('<IBBBBB',0,6,8,0,0,3))

    def interval_command(self, command,value):
        if self.component is None: raise RuntimeError("flight-controller identity not established")
        if command not in (510,511): raise ValueError('only GET/SET_MESSAGE_INTERVAL')
        self.send(76,struct.pack('<7fHBBB',105,value,0,0,0,0,0,command,self.system_id,self.component,0))

    def timesync(self,stamp):
        self.send(111,struct.pack('<qq',0,stamp))

    def receive(self):
        while True:
            while self.buffer and self.buffer[0] not in (0xfe,0xfd): del self.buffer[0]
            if len(self.buffer)>=3:
                v2=self.buffer[0]==0xfd;header=10 if v2 else 6
                length=header+self.buffer[1]+2+(13 if v2 and self.buffer[2]&1 else 0)
                if len(self.buffer)>=length:
                    frame=bytes(self.buffer[:length]);del self.buffer[:length]
                    msgid=int.from_bytes(frame[7:10],'little') if v2 else frame[5]
                    sysid,compid=(frame[5],frame[6]) if v2 else (frame[3],frame[4])
                    payload=frame[header:header+frame[1]]
                    if sysid!=self.system_id or msgid not in CRC:continue
                    expected=checksum(frame[1:header]+payload+bytes([CRC[msgid]]))
                    if expected!=int.from_bytes(frame[header+len(payload):header+len(payload)+2],'little'):continue
                    if msgid==0:
                        # MAV_AUTOPILOT_INVALID identifies non-autopilot components.
                        if payload.ljust(9,b'\0')[5]==8:continue
                        if self.component is None:self.component=compid
                    if compid!=self.component:continue
                    return msgid,payload,time.time_ns(),time.monotonic_ns()
            try:data=self.socket.recv(8192)
            except socket.timeout:return None
            if not data:raise ConnectionError('flight-controller router disconnected')
            self.buffer.extend(data)
            if len(self.buffer)>65536:raise ValueError('MAVLink receive buffer exceeded')

    def wait(self, message_id,predicate=lambda _:True,seconds=4):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            result=self.receive()
            if result and result[0]==message_id and predicate(result[1]):return result[1]
        raise TimeoutError('flight controller did not respond')

    def interval(self):
        self.interval_command(510,0)
        payload=self.wait(244,lambda p:struct.unpack_from('<H',p.ljust(6,b'\0'),4)[0]==105)
        return struct.unpack_from('<i',payload.ljust(6,b'\0'))[0]

    def set_interval(self,value):
        self.interval_command(511,value)
        payload=self.wait(77,lambda p:struct.unpack_from('<H',p.ljust(3,b'\0'))[0]==511)
        if payload.ljust(3,b'\0')[2]!=0:raise RuntimeError('IMU interval command rejected')
        if self.interval()!=value:raise RuntimeError('IMU interval readback differs')

    def close(self):self.socket.close()
