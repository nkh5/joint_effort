#!/usr/bin/env python3
"""Visual-review controls; sends only parameters to the isolated pose publisher."""
import sys,math,signal
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from PyQt5.QtCore import Qt,QTimer
from PyQt5.QtWidgets import QApplication,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QSlider,QPushButton
rclpy.init();node=Node('cad_review_sliders');client=node.create_client(SetParametersAtomically,'/cad_review_pose/set_parameters_atomically')
app=QApplication(sys.argv)
signal.signal(signal.SIGINT,lambda *_:app.quit())
signal.signal(signal.SIGTERM,lambda *_:app.quit())
window=QWidget();window.setWindowTitle('Joint Effort CAD review — visual only')
layout=QVBoxLayout(window);layout.addWidget(QLabel('Saved CAD pose = 0°. These sliders do not command motors.'))
desired=[0.]*5;sent=None;pending=None;sliders=[]
for i in range(5):
 row=QHBoxLayout();label=QLabel(f'Joint {i+1}: 0.0°');slider=QSlider(Qt.Horizontal);slider.setRange(-300,300);slider.setValue(0)
 def changed(value,index=i,text=label):desired[index]=math.radians(value/10);text.setText(f'Joint {index+1}: {value/10:.1f}°')
 slider.valueChanged.connect(changed);row.addWidget(label);row.addWidget(slider);layout.addLayout(row);sliders.append(slider)
reset=QPushButton('Reset to saved CAD pose');reset.clicked.connect(lambda:[s.setValue(0) for s in sliders]);layout.addWidget(reset)
status=QLabel('Connecting to the visual pose publisher…');layout.addWidget(status)
def tick():
 global sent,pending
 for _ in range(3):rclpy.spin_once(node,timeout_sec=0.)
 if pending is not None:
  if not pending.done():return
  result=pending.result();status.setText('Visual pose updated' if result.result.successful else result.result.reason);pending=None
 if client.service_is_ready() and sent!=desired:
  req=SetParametersAtomically.Request()
  for i,val in enumerate(desired):
   p=Parameter();p.name=f'joint_{i+1}';p.value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=val);req.parameters.append(p)
  pending=client.call_async(req);sent=desired.copy()
timer=QTimer();timer.timeout.connect(tick);timer.start(40);window.resize(650,300);window.show()
try:app.exec_()
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
