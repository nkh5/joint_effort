"""Check three-motor tests and the fixed base against a serial-bus emulator."""
import importlib.util
import os
from pathlib import Path
import pty
import select
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import serial

FILE = Path(__file__).resolve().parents[1]/'src/joint_effort_hardware/joint_effort_hardware/commission.py'
spec = importlib.util.spec_from_file_location('commission', FILE)
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)


class Emulator:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.port = serial.Serial(os.ttyname(self.slave), 1000000, timeout=.001, write_timeout=.01, exclusive=True)
        self.regs = {}
        for ident, pos in [(1,3046),(2,196),(3,2048)]:
            data = bytearray(256)
            data[3:5] = bytes([9,3]); data[5] = ident; data[8] = 1
            data[11:13] = bytes([255,15]); data[18] = 12; data[30] = 1
            data[55] = 1; data[56:58] = pos.to_bytes(2,'little')
            data[62] = 127 if ident == 1 else 126; data[63] = 30
            self.regs[ident] = data
        self.commands = []; self.bad = None; self.stop = False; self.lag = 0
        self.thread = threading.Thread(target=self.run, daemon=True); self.thread.start()

    def run(self):
        buffer = bytearray()
        while not self.stop:
            if not select.select([self.master],[],[],.01)[0]: continue
            try: buffer.extend(os.read(self.master,1024))
            except OSError: break
            while len(buffer) >= 4 and len(buffer) >= buffer[3]+4:
                size = buffer[3]+4; tx = bytes(buffer[:size]); del buffer[:size]
                if sum(tx[2:]) & 255 != 255: continue
                ident, opcode = tx[2], tx[4]; args = list(tx[5:-1])
                self.commands.append((ident,opcode,args))
                if ident not in self.regs: continue
                data = self.regs[ident]
                if opcode == 2: reply = list(data[args[0]:args[0]+args[1]])
                else:
                    data[args[0]:args[0]+len(args)-1] = bytes(args[1:]); reply=[]
                    if data[40] and args[0] == 41:
                        target=c.word(data,42);current=c.word(data,56)
                        position=target if target==current else target-(self.lag if target>current else -self.lag)
                        data[56:58]=position.to_bytes(2,'little')
                raw = c.frame(ident,0,reply)
                bad, self.bad = self.bad, None
                if bad == 'drop':continue
                if bad == 'checksum': raw = raw[:-1]+bytes([raw[-1]^1])
                if bad == 'id': raw = c.frame(2 if ident == 1 else 1,0,reply)
                if bad == 'fault': raw = c.frame(ident,4,reply)
                if bad == 'prefix': raw=b'X'+raw
                os.write(self.master,raw)

    def close(self):
        self.stop=True;self.thread.join(1);self.port.close();os.close(self.master);os.close(self.slave)


class CommissionTests(unittest.TestCase):
    def setUp(self):
        self.emu=Emulator();self.bus=c.Bus(self.emu.port)
        self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)/'session.json'
    def tearDown(self):self.emu.close();self.tmp.cleanup()
    def prepare(self):return c.prepare(self.bus,'test-port',self.path,seconds=.06)
    def test_prepare_is_read_only_and_has_bounded_windows(self):
        result=self.prepare()
        self.assertEqual(result['writes_sent'],0)
        self.assertTrue(all(command[1]==2 for command in self.emu.commands))
        self.assertEqual((result['motors']['1']['min'],result['motors']['1']['max']),(3038,3054))
    def test_move_elbow_and_hold_base_with_torque_on(self):
        session=self.prepare();result=c.step(self.bus,'test-port',session,'elbow',4)
        self.assertEqual(result['measured_change_counts'],4)
        writes=[x for x in self.emu.commands if x[1]==3]
        self.assertTrue(all(x[0] in (1,3) and x[2][0] in (40,41) for x in writes))
        self.assertEqual(self.emu.regs[1][40],1);self.assertEqual(self.emu.regs[2][40],0)
        self.assertEqual(self.emu.regs[3][40],1)
        self.assertEqual(c.word(self.emu.regs[2],56),196)
        goals=[c.word(x[2],2) for x in writes if x[0]==3 and x[2][0]==41]
        self.assertEqual(goals,[2048,2049,2050,2051,2052])
    def test_move_shoulder_negative(self):
        result=c.step(self.bus,'test-port',self.prepare(),'shoulder',-4)
        self.assertEqual(result['measured_change_counts'],-4)
        self.assertEqual(self.emu.regs[1][40],1)
        self.assertEqual(c.word(self.emu.regs[1],56),3046)
    def test_already_enabled_motor_can_be_tested_without_releasing_it(self):
        self.emu.regs[1][40]=1;self.emu.regs[1][42:44]=(3046).to_bytes(2,'little')
        self.emu.regs[3][40]=1;self.emu.regs[3][42:44]=(2048).to_bytes(2,'little')
        result=c.step(self.bus,'test-port',self.prepare(),'elbow',1)
        self.assertEqual(result['result'],'PASS')
        self.assertFalse(any(x['address']==40 for x in self.bus.writes))
    def test_rejects_large_or_zero_steps_without_writes(self):
        session=self.prepare()
        for delta in (0,9,-9):
            with self.assertRaises(RuntimeError):c.step(self.bus,'test-port',session,'elbow',delta)
        self.assertEqual(self.bus.writes,[])
    def test_cumulative_window_not_reset_by_each_step(self):
        session=self.prepare();self.emu.regs[3][56:58]=(2055).to_bytes(2,'little')
        with self.assertRaisesRegex(RuntimeError,'prepared'):c.step(self.bus,'test-port',session,'elbow',4)
        self.assertEqual(self.bus.writes,[])
    def test_expired_session_and_wrong_port_refuse_writes(self):
        session=self.prepare()
        with self.assertRaises(RuntimeError):c.step(self.bus,'other-port',session,'elbow',4)
        session['created_unix']-=901
        with self.assertRaises(RuntimeError):c.step(self.bus,'test-port',session,'elbow',4)
        self.assertEqual(self.bus.writes,[])
    def test_phase_mismatch_refuses_without_changes(self):
        self.emu.regs[1][18]=8
        with self.assertRaisesRegex(RuntimeError,'phase'):self.prepare()
        self.assertEqual(self.bus.writes,[])
    def test_bad_mode_status_temperature_voltage_and_wrap(self):
        for address,value in [(33,1),(65,1),(63,60),(62,140),(57,128)]:
            with self.subTest(address=address):
                old=self.emu.regs[1][address];self.emu.regs[1][address]=value
                with self.assertRaises(RuntimeError):c.sample(self.bus)
                self.emu.regs[1][address]=old
        self.assertEqual(self.bus.writes,[])
    def test_bad_serial_replies_latch_and_forbid_followup_writes(self):
        for fault in ('checksum','id','fault','prefix','drop'):
            with self.subTest(fault=fault):
                self.bus=c.Bus(self.emu.port);self.emu.bad=fault
                with self.assertRaises(RuntimeError):self.bus.read(1,30)
                self.assertFalse(self.bus.healthy)
                self.bus.selected=3;self.bus.bounds=(2040,2056)
                with self.assertRaises(RuntimeError):self.bus.goal(3,2048)
                self.emu.port.reset_input_buffer()
    def test_pending_motion_is_refused(self):
        session=self.prepare();self.emu.regs[1][40]=1
        self.emu.regs[1][42:44]=(3100).to_bytes(2,'little')
        with self.assertRaisesRegex(RuntimeError,'pending movement'):c.step(self.bus,'test-port',session,'elbow',4)
        self.assertFalse(any(x[1]==3 for x in self.emu.commands))
    def test_unselected_motor_movement_stops_commands(self):
        session=self.prepare();original=self.bus.enable
        def enable(ident):
            original(ident);self.emu.regs[2][56:58]=(200).to_bytes(2,'little')
        self.bus.enable=enable
        with self.assertRaisesRegex(RuntimeError,'Unselected'):c.step(self.bus,'test-port',session,'elbow',4)
        self.assertEqual(c.word(self.emu.regs[1],56),3046)
        self.assertEqual(self.emu.regs[1][40],1)
    def test_edited_window_refuses_writes(self):
        session=self.prepare();session['motors']['1']['max']+=100
        with self.assertRaisesRegex(RuntimeError,'Invalid prepared'):c.step(self.bus,'test-port',session,'elbow',4)
        self.assertEqual(self.bus.writes,[])
    def test_serial_loss_after_enable_does_not_retry_or_release(self):
        session=self.prepare();original=self.bus.enable
        def enable(ident):
            original(ident);self.emu.bad='drop'
        self.bus.enable=enable
        with self.assertRaises(RuntimeError):c.step(self.bus,'test-port',session,'elbow',4)
        self.assertFalse(self.bus.healthy)
        self.assertEqual([x['address'] for x in self.bus.writes],[41,40])
        self.assertEqual(self.emu.regs[1][40],1)
    def test_interrupt_holds_fresh_measured_pose_without_releasing(self):
        session=self.prepare();original=self.bus.enable
        def enable(ident):
            original(ident);raise KeyboardInterrupt()
        self.bus.enable=enable
        with self.assertRaises(KeyboardInterrupt):c.step(self.bus,'test-port',session,'elbow',4)
        self.assertEqual(self.bus.writes[-1],{'id':3,'address':41,'target':2048})
        self.assertEqual(self.emu.regs[1][40],1)
    def test_ten_degree_turn_verifies_minimum_with_two_count_error(self):
        session=c.prepare_turn(self.bus,'test-port',self.path,'elbow',1,seconds=.06)
        self.emu.lag=2
        with patch.object(c,'MOVE_INTERVAL',.005),patch.object(c,'PERIOD',.005),\
             patch.object(c,'SETTLE_SECONDS',.01),patch.object(c,'OBSERVE_SECONDS',.05),\
             patch.object(c,'FEEDBACK_DEADLINE',.1):
            result=c.step(self.bus,'test-port',session,'elbow',116,ten_degree=True)
        self.assertGreaterEqual(result['shaft_change_degrees'],10)
        self.assertEqual(result['measured_change_counts'],114)
        self.assertTrue(all(w['id'] in (1,3) for w in self.bus.writes))
        self.assertEqual(c.word(self.emu.regs[1],56),3046)
        self.assertEqual(c.word(self.emu.regs[2],56),196)
    def test_ten_degree_session_rejects_wrong_motor_and_small_step_command(self):
        session=c.prepare_turn(self.bus,'test-port',self.path,'elbow',1,seconds=.06)
        with self.assertRaises(RuntimeError):c.step(self.bus,'test-port',session,'shoulder',116,ten_degree=True)
        with self.assertRaises(RuntimeError):c.step(self.bus,'test-port',session,'elbow',4)
        with self.assertRaises(RuntimeError):c.step(self.bus,'test-port',session,'elbow',117,ten_degree=True)
        self.assertEqual(self.bus.writes,[])
    def test_ten_degree_preparation_cannot_cross_motor_boundary(self):
        self.emu.regs[3][56:58]=(4050).to_bytes(2,'little')
        with self.assertRaisesRegex(RuntimeError,'boundary'):
            c.prepare_turn(self.bus,'test-port',self.path,'elbow',1,seconds=.06)
        self.assertFalse(self.path.exists());self.assertEqual(self.bus.writes,[])
    def test_ten_degree_negative_turn(self):
        session=c.prepare_turn(self.bus,'test-port',self.path,'elbow',-1,seconds=.06)
        with patch.object(c,'MOVE_INTERVAL',.005),patch.object(c,'PERIOD',.005),\
             patch.object(c,'SETTLE_SECONDS',.01),patch.object(c,'OBSERVE_SECONDS',.05),\
             patch.object(c,'FEEDBACK_DEADLINE',.1):
            result=c.step(self.bus,'test-port',session,'elbow',-116,ten_degree=True)
        self.assertLessEqual(result['shaft_change_degrees'],-10)
    def test_write_guard_rejects_wrong_motor_and_bounds(self):
        self.bus.selected=3;self.bus.bounds=(2040,2056)
        for ident,target in [(2,196),(3,0),(3,2057),(1,3046)]:
            with self.assertRaises(RuntimeError):self.bus.goal(ident,target)
        with self.assertRaises(RuntimeError):self.bus.enable(2)
        for ident in (4,5,6,254):
            with self.assertRaises(RuntimeError):c.frame(ident,3,[40,1])
        self.assertEqual(self.bus.writes,[])

    def test_three_motors_are_all_required_and_old_sessions_rejected(self):
        session=self.prepare()
        self.assertEqual(set(session['motors']),{'1','2','3'})
        self.assertEqual(session['profile'],c.PROFILE)
        del session['motors']['3']
        with self.assertRaisesRegex(RuntimeError,'three-motor'):c.step(self.bus,'test-port',session,'shoulder',1)
        self.assertEqual(self.bus.writes,[])

    def test_missing_motor_3_prevents_preparation_and_writes(self):
        del self.emu.regs[3]
        with self.assertRaisesRegex(RuntimeError,'deadline'):self.prepare()
        self.assertFalse(self.bus.healthy);self.assertEqual(self.bus.writes,[])

    def test_base_cannot_be_selected_for_small_or_ten_degree_motion(self):
        session=self.prepare()
        for call in (lambda:c.step(self.bus,'test-port',session,'base',1),
                     lambda:c.step(self.bus,'test-port',session,'base',116,ten_degree=True),
                     lambda:c.prepare_turn(self.bus,'test-port',self.path,'base',1,seconds=.06)):
            with self.assertRaisesRegex(RuntimeError,'stationary'):call()
        self.assertEqual(self.bus.writes,[])

    def test_base_hold_preload_is_verified_before_torque_enable(self):
        session=self.prepare();c.step(self.bus,'test-port',session,'shoulder',1)
        writes=[x for x in self.emu.commands if x[1]==3]
        base_goals=[c.word(x[2],2) for x in writes if x[0]==1 and x[2][0]==41]
        self.assertEqual(base_goals,[3046])
        self.assertEqual(writes[0],(1,3,[41,1,230,11,0,0,10,0]))
        self.assertEqual(writes[1],(1,3,[40,1]))
        self.assertEqual(c.word(self.emu.regs[1],56),3046)
        self.assertEqual(self.emu.regs[1][40],1)
        self.assertFalse(any(x[0]==3 for x in writes))

    def test_stationary_goal_guard_does_not_follow_drift(self):
        self.bus.hold_target=3046
        self.bus.goal(1,3046)
        with self.assertRaisesRegex(RuntimeError,'stationary'):self.bus.goal(1,3047)
        self.assertEqual(self.bus.writes,[{'id':1,'address':41,'target':3046}])

    def test_base_drift_since_prepare_requires_new_preparation(self):
        session=self.prepare();self.emu.regs[1][56:58]=(3049).to_bytes(2,'little')
        with self.assertRaisesRegex(RuntimeError,'since preparation'):
            c.step(self.bus,'test-port',session,'shoulder',1)
        self.assertEqual(self.bus.writes,[])

    def test_stationary_drift_during_motion_stops_selected_goal(self):
        session=self.prepare();original=self.bus.goal
        def goal(ident,target):
            original(ident,target)
            if ident==2 and target==197:self.emu.regs[1][56:58]=(3050).to_bytes(2,'little')
        self.bus.goal=goal
        with self.assertRaisesRegex(RuntimeError,'Stationary motor 1'):
            c.step(self.bus,'test-port',session,'shoulder',4)
        self.assertLessEqual(c.word(self.emu.regs[2],56),197)
        self.assertEqual([w['target'] for w in self.bus.writes if w['id']==1 and w['address']==41],[3046])

    def test_stationary_torque_loss_stops_selected_goal(self):
        session=self.prepare();original=self.bus.goal
        def goal(ident,target):
            original(ident,target)
            if ident==3 and target==2049:self.emu.regs[1][40]=0
        self.bus.goal=goal
        with self.assertRaisesRegex(RuntimeError,'lost hold torque'):
            c.step(self.bus,'test-port',session,'elbow',4)
        self.assertLessEqual(c.word(self.emu.regs[3],56),2049)

    def test_lost_base_hold_preload_never_enables_torque(self):
        session=self.prepare();original=self.bus.goal
        def goal(ident,target):
            original(ident,target)
            if ident==1:self.emu.regs[1][42:44]=(3000).to_bytes(2,'little')
        self.bus.goal=goal
        with self.assertRaisesRegex(RuntimeError,'preload did not verify'):
            c.step(self.bus,'test-port',session,'shoulder',1)
        self.assertFalse(any(w['address']==40 for w in self.bus.writes))
        self.assertEqual(self.emu.regs[1][40],0)
        self.assertEqual(c.word(self.emu.regs[1],56),3046)

    def test_production_three_motor_read_budget_is_enforced(self):
        class Snapshot:
            def read(inner,ident,address,count):
                return list(self.emu.regs[ident][address:address+count])
        self.assertEqual(c.FEEDBACK_DEADLINE,.015)
        with patch.object(c.time,'monotonic',side_effect=[100.,100.014]):
            self.assertEqual(len(c.sample(Snapshot())[0]),3)
        with patch.object(c.time,'monotonic',side_effect=[100.,100.016]):
            with self.assertRaisesRegex(RuntimeError,'15 ms'):c.sample(Snapshot())

if __name__=='__main__':unittest.main()
