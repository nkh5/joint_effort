"""EEPROM assignment logic and actual packet checks, with no real motor adapter."""
import os
import pty
import select
import threading
import unittest
import serial
from joint_effort_hardware.id_assignment import IDBus, assign_isolated, verify_isolated


class IDEmulator:
    def __init__(self, ident=7):
        self.master,self.slave=pty.openpty()
        self.port=serial.Serial(os.ttyname(self.slave),1000000,timeout=.001,write_timeout=.01,exclusive=True)
        data=bytearray(256)
        data[3:6]=bytes([9,3,ident]);data[8]=1;data[11:13]=bytes([255,15])
        data[18]=12;data[30]=1;data[55]=1;data[56:58]=bytes([0,8]);data[62]=126;data[63]=30
        self.regs={ident:data};self.commands=[];self.errors=[];self.stop=False
        self.ack='new';self.drop_id_write=False;self.bad_id_reply=False
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def run(self):
        buffer=bytearray()
        while not self.stop:
            if not select.select([self.master],[],[],.01)[0]:continue
            try:buffer.extend(os.read(self.master,1024))
            except OSError:break
            while len(buffer)>=4 and len(buffer)>=buffer[3]+4:
                size=buffer[3]+4;frame=bytes(buffer[:size]);del buffer[:size]
                ident,opcode=frame[2],frame[4];args=list(frame[5:-1])
                self.commands.append((ident,opcode,args))
                if (sum(frame[2:])&255)!=255:self.errors.append('Bad outbound checksum');continue
                if ident not in self.regs:continue
                data=self.regs[ident];reply_id=ident
                if opcode==2:reply=list(data[args[0]:args[0]+args[1]])
                elif opcode==3:
                    address,value=args
                    if address not in (5,40,55) or (address==40 and value!=0):self.errors.append('Forbidden write')
                    if address==5:
                        if data[55]:self.errors.append('ID write while locked')
                        if not self.drop_id_write:
                            del self.regs[ident];self.regs[value]=data;data[5]=value
                        if self.ack=='none':continue
                        if self.ack=='new':reply_id=value
                    else:data[address]=value
                    reply=[]
                else:self.errors.append('Forbidden opcode');continue
                body=bytes([reply_id,len(reply)+2,0,*reply]);checksum=(~sum(body))&255
                if self.bad_id_reply and opcode==3 and args[0]==5:checksum^=1
                os.write(self.master,b'\xff\xff'+body+bytes([checksum]))

    def power_cycle(self):
        # Keep EEPROM bytes (including ID 5), clear runtime torque, relock settings.
        self.regs={data[5]:data for data in self.regs.values()}
        for data in self.regs.values():data[40]=0;data[55]=1

    def close(self):
        self.stop=True;self.thread.join(1);self.port.close();os.close(self.master);os.close(self.slave)


class IDAssignmentTests(unittest.TestCase):
    def setUp(self):
        self.emu=IDEmulator();self.bus=IDBus(self.emu.port)
        # Most tests focus on writes and readback; full-address scan is tested separately.
        self.bus.scan=lambda:sorted(self.emu.regs)
    def tearDown(self):self.emu.close()
    def assign(self,**kwargs):return assign_isolated(self.bus,7,3,single_motor_connected=True,**kwargs)

    def test_permanent_id_write_unlocks_verifies_and_relocks_without_motion(self):
        result=self.assign()
        self.assertEqual(result['id'],3);self.assertTrue(result['eeprom_locked'])
        self.assertEqual(self.bus.writes,[{'id':7,'address':55,'value':0},
                         {'id':7,'address':5,'value':3},{'id':3,'address':55,'value':1}])
        self.assertEqual(self.emu.errors,[])
        self.emu.power_cycle()
        before=list(self.bus.writes)
        self.assertEqual(verify_isolated(self.bus,3,single_motor_connected=True)['result'],'PASS')
        self.assertEqual(self.bus.writes,before)
        self.assertEqual(self.emu.regs[3][40],0)

    def test_ack_can_use_old_id(self):
        self.emu.ack='old';self.assertEqual(self.assign()['result'],'PASS')

    def test_missing_id_ack_requires_new_address_readback(self):
        self.emu.ack='none';self.assertEqual(self.assign()['result'],'PASS')

    def test_lost_id_write_fails_and_relocks_old_address(self):
        self.emu.drop_id_write=True;self.emu.ack='old'
        with self.assertRaises(RuntimeError):self.assign()
        self.assertEqual(self.emu.regs[7][55],1)
        self.assertEqual(sum(w['address']==5 for w in self.bus.writes),1)

    def test_corrupt_id_ack_fails_and_relocks_new_address(self):
        self.emu.bad_id_reply=True
        with self.assertRaisesRegex(RuntimeError,'checksum'):self.assign()
        self.assertEqual(self.emu.regs[3][55],1)
        self.assertEqual(sum(w['address']==5 for w in self.bus.writes),1)

    def test_multiple_addresses_or_wrong_old_id_refuse_all_writes(self):
        for found in ([7,2],[8],[]):
            self.bus.scan=lambda:found
            with self.assertRaises(RuntimeError):self.assign()
        self.assertEqual(self.bus.writes,[])

    def test_physical_isolation_acknowledgement_required(self):
        with self.assertRaises(RuntimeError):assign_isolated(self.bus,7,3,single_motor_connected=False)
        self.assertEqual(self.bus.writes,[])

    def test_torque_is_not_released_without_explicit_flag(self):
        self.emu.regs[7][40]=1
        with self.assertRaisesRegex(RuntimeError,'disable-torque'):self.assign()
        self.assertEqual(self.bus.writes,[])
        self.assertEqual(self.assign(disable_torque=True)['result'],'PASS')
        self.assertEqual(self.bus.writes[0],{'id':7,'address':40,'value':0})
        self.assertEqual(self.emu.regs[3][40],0)

    def test_already_correct_id_does_not_rewrite_eeprom(self):
        result=assign_isolated(self.bus,7,1,single_motor_connected=True)
        self.assertTrue(result['id_changed'])
        self.bus.writes.clear()
        result=assign_isolated(self.bus,1,1,single_motor_connected=True)
        self.assertFalse(result['id_changed']);self.assertEqual(self.bus.writes,[])

    def test_invalid_target_or_model_refuses_all_writes(self):
        for target in (0,7,254,True):
            with self.assertRaises(RuntimeError):assign_isolated(self.bus,7,target,single_motor_connected=True)
        self.emu.regs[7][3]=8
        with self.assertRaisesRegex(RuntimeError,'identity'):self.assign()
        self.assertEqual(self.bus.writes,[])

    def test_ids_4_through_6_can_be_assigned_without_motion(self):
        old=7
        for target in (4,5,6):
            result=assign_isolated(self.bus,old,target,single_motor_connected=True)
            self.assertEqual(result['id'],target)
            self.assertEqual(self.emu.regs[target][55],1)
            self.assertEqual(self.emu.regs[target][40],0)
            old=target
        self.assertTrue(all(w['address'] in (5,55) for w in self.bus.writes))

    def test_full_scan_is_read_only_and_supports_old_id_zero(self):
        self.emu.regs[0]=self.emu.regs.pop(7);self.emu.regs[0][5]=0
        self.assertEqual(IDBus(self.emu.port).scan(),[0])
        self.assertTrue(all(op==2 for _,op,_ in self.emu.commands))
        self.assertEqual(assign_isolated(self.bus,0,2,single_motor_connected=True)['id'],2)

    def test_write_scope_blocks_goal_torque_enable_and_broadcast(self):
        for address,value in ((41,1),(40,1),(6,0),(5,7)):
            with self.assertRaises(RuntimeError):self.bus.write(7,address,value)
        with self.assertRaises(RuntimeError):self.bus.read(254,5)
        self.assertEqual(self.bus.writes,[])

if __name__=='__main__':unittest.main()
