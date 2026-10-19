"""Test read-only motor discovery and raw-byte diagnostics."""
import importlib.util
import os
from pathlib import Path
import pty
import select
import threading
import time
import unittest
import serial

spec = importlib.util.spec_from_file_location('motor_identification',
    Path(__file__).resolve().parents[1]/'tools/identify_motors.py')
tool = importlib.util.module_from_spec(spec); spec.loader.exec_module(tool)


def response(ident, data, error=0):
    body = bytes([ident,len(data)+2,error])+bytes(data)
    return b'\xff\xff'+body+bytes([(~sum(body)) & 255])


class IdentificationTests(unittest.TestCase):
    def test_documented_read_packet_and_no_broadcast_or_setting_write(self):
        self.assertEqual(tool.read_request(1,3,3),bytes.fromhex('ff ff 01 04 02 03 03 f2'))
        for ident,address,count in [(254,3,3),(-1,3,3),(True,3,3),(1,42,2),(1,40,2),(1,5,2)]:
            with self.assertRaises(ValueError): tool.read_request(ident,address,count)

    def test_complete_reply_and_id_zero(self):
        for ident in (0,1,253):
            request=tool.read_request(ident,5,1)
            result=tool.decode(response(ident,[ident]),request,ident,1)
            self.assertEqual(tool.one_reply(result),[ident])
            self.assertFalse(result['unmatched_hex'])

    def test_echo_is_not_a_reply(self):
        request=tool.read_request(1,5,1)
        result=tool.decode(request,request,1,1)
        self.assertIsNone(tool.one_reply(result));self.assertEqual(result['echoes'],1)

    def test_echo_and_leading_bytes_remain_visible(self):
        request=tool.read_request(1,5,1)
        raw=b'boot\r\n'+request+response(1,[1])
        result=tool.decode(raw,request,1,1)
        self.assertEqual(tool.one_reply(result),[1])
        self.assertEqual(result['echoes'],1);self.assertTrue(result['unmatched_hex'])

    def test_checksum_truncation_and_wrong_id_not_accepted(self):
        request=tool.read_request(1,5,1)
        good=response(1,[1]);bad=good[:-1]+bytes([good[-1]^1])
        for raw in (bad,good[:-1],response(2,[2]),b'\x00\x00'):
            self.assertIsNone(tool.one_reply(tool.decode(raw,request,1,1)))

    def test_fault_and_duplicate_replies_not_accepted_as_one_motor(self):
        request=tool.read_request(1,5,1)
        for raw in (response(1,[1],4),response(1,[1])*2):
            self.assertIsNone(tool.one_reply(tool.decode(raw,request,1,1)))

    def test_pseudo_terminal_scan_only_sends_reads(self):
        master,slave=pty.openpty();stop=threading.Event();sent=[];errors=[]
        memory=bytearray(256)
        memory[3:6]=bytes([9,3,2]);memory[56:58]=bytes([0,8]);memory[62:64]=bytes([126,30])
        def serve():
            buffer=b''
            try:
                while not stop.is_set():
                    ready,_,_=select.select([master],[],[],.01)
                    if not ready:continue
                    buffer+=os.read(master,1024)
                    while len(buffer)>=4 and len(buffer)>=buffer[3]+4:
                        frame,buffer=buffer[:buffer[3]+4],buffer[buffer[3]+4:]
                        self.assertEqual(frame[:2],b'\xff\xff')
                        self.assertEqual(sum(frame[2:]) & 255,255)
                        self.assertEqual(frame[4],2)
                        self.assertIn((frame[5],frame[6]),tool.READS)
                        sent.append(frame)
                        if frame[2]==2:
                            address,count=frame[5:7]
                            os.write(master,b'\x00'+frame+response(2,memory[address:address+count]))
            except Exception as exc:errors.append(exc)
        thread=threading.Thread(target=serve);thread.start()
        try:
            with serial.Serial(os.ttyname(slave),1000000,timeout=.002,write_timeout=.2,exclusive=True) as port:
                result=tool.identify(port,[1,2,3],window=.03)
            self.assertFalse(errors,errors)
            self.assertEqual([r['id'] for r in result['motors']],[2])
            row=result['motors'][0]
            self.assertEqual(row['reported_model_number'],777)
            self.assertEqual(row['fields']['position_counts'],2048)
            self.assertEqual(row['fields']['voltage_v'],12.6)
            self.assertFalse(row['clean_capture'])
            self.assertTrue(sent);self.assertTrue(all(f[4]==2 for f in sent))
        finally:
            stop.set();thread.join(1);os.close(master);os.close(slave)


if __name__ == '__main__': unittest.main()
