import os
import json
import numpy as np
from hand_eye_calibration.dual_quaternion import DualQuaternion
import yaml

class ExtrinsicCalibration:
  def __init__(self, time_offset, pose_dual_quat):
    self.time_offset = time_offset
    self.pose_dq = pose_dual_quat

  def writeJson(self, out_file, camera, switchConvention = False):
    pose = self.pose_dq.to_pose()
    if switchConvention:
      pose[3:6] *= -1.0 # convert to JPL
    calib = {
      camera: {
      'timeOffset' : self.time_offset, 
      'q(x,y,z,w)' : [ float(pose[i + 3]) for i, name in enumerate('xyzw') ],
      't(x,y,z)metres' : [ float(pose[i]) for i, name in enumerate('xyz') ]
      }
    }
    with open(out_file, 'w') as f:
      json.dump(calib, f, indent = 3, sort_keys=True)

  def writeYaml(self, out_file, camera, switchConvention = False):
    pose = self.pose_dq.to_pose()
    if switchConvention:
      pose[3:6] *= -1.0
    calib = {
      camera: {
      'timeOffset' : self.time_offset, 
      'q(x,y,z,w)' : [ float(pose[i + 3]) for i, name in enumerate('xyzw') ],
      't(x,y,z)metres' : [ float(pose[i]) for i, name in enumerate('xyz') ]
      }
    }
    configOut = yaml.dump(calib)
    configFile = open(out_file, 'w')
    configFile.write(configOut)

  @classmethod
  def fromJson(cls, in_file, switchConvention = False):
    with open(in_file, 'r') as f:
      data = json.load(f)
    
    p = [ float(data['translation'][name]) for name in 'xyz' ]
    
    q = np.array([ float(data['rotation'][name]) for name in 'ijkw' ])
    if switchConvention:
      q[:3] *= -1.0

    dq = DualQuaternion.from_pose_vector(np.hstack((p, q)))
    return ExtrinsicCalibration(float(data['delay']), dq)
  
  def __str__(self):
    return "[delta_time: %f, delta_pose: %s]" %(self.time_offset, str(self.pose_dq.to_pose()))

  def __mul__(self, other):    
    if not isinstance(other, ExtrinsicCalibration):
        return NotImplemented
    return ExtrinsicCalibration(self.time_offset + other.time_offset, self.pose_dq * other.pose_dq)
