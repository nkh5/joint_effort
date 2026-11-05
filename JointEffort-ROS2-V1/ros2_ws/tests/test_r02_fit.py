"""Calibration CLI must write only accepted R02 data, and keep old motor data separate."""
import contextlib
import csv
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
import yaml

W=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('r02_fit_cli',W/'tools/fit_r02_calibration.py')
fitter=importlib.util.module_from_spec(spec);spec.loader.exec_module(fitter)

class FitCommandTests(unittest.TestCase):
    def test_cli_writes_good_data_and_keeps_checks_false(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);cal=p/'cal.yaml';source=W/'src/joint_effort_iron/config/sts_calibration.yaml'
            cal.write_text(source.read_text());data=p/'data.csv';report=p/'report.json'
            with data.open('w',newline='') as f:
                writer=csv.writer(f);writer.writerow(['joint','angle_deg','encoder_counts'])
                for i in range(1,6):
                    for a in (-10,0,10):writer.writerow([f'joint_{i}',a,round(2048+(-1)**i*a*4096/360)])
                writer.writerow(['Motor']);writer.writerow(['Angle / Encoder value','1'])
                for a in (-10,0,10):writer.writerow([a,round(3000+a*4096/360)])
            with contextlib.redirect_stdout(io.StringIO()):
                code=fitter.main(['--csv',str(data),'--cal',str(cal),'--write','--report',str(report)])
            self.assertEqual(code,0);fitted=yaml.safe_load(cal.read_text())
            for i in range(1,6):
                self.assertEqual(fitted['joints'][f'joint_{i}']['center_steps'],2048)
                self.assertEqual(fitted['joints'][f'joint_{i}']['encoder_direction'],(-1)**i)
            self.assertFalse(any(fitted['physical_checks'].values()))
            r=json.loads(report.read_text());self.assertTrue(r['calibration_written'])
            self.assertEqual(r['motor_reference']['joint_1']['center_steps'],3000)
            self.assertEqual(fitted['urdf_sha256'],hashlib.sha256((W/'src/joint_effort_iron/urdf/iron.urdf').read_bytes()).hexdigest())

    def test_rejected_measurement_keeps_active_file_unchanged(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);cal=p/'cal.yaml';data=p/'data.csv';report=p/'report.json'
            cal.write_text((W/'src/joint_effort_iron/config/sts_calibration.yaml').read_text());before=cal.read_bytes()
            with data.open('w',newline='') as f:
                writer=csv.writer(f);writer.writerow(['joint','angle_deg','encoder_counts'])
                for i in range(1,6):
                    for a in (-10,0,10):writer.writerow([f'joint_{i}',a,round(2048+a*4096/360)+(60 if i==5 and a==0 else 0)])
            with contextlib.redirect_stdout(io.StringIO()):
                code=fitter.main(['--csv',str(data),'--cal',str(cal),'--write','--report',str(report)])
            self.assertEqual(code,1);self.assertEqual(cal.read_bytes(),before)
            r=json.loads(report.read_text());self.assertEqual(r['result'],'FAIL');self.assertFalse(r['calibration_written'])
            self.assertFalse(r['joints']['joint_5']['accepted'])

if __name__=='__main__':unittest.main()
