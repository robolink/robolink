"""autogenerated by genpy from robolink/RobolinkInfo.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import genpy

class RobolinkInfo(genpy.Message):
  _md5sum = "03f18ab74164f6bff2ac6ce14e2d9583"
  _type = "robolink/RobolinkInfo"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# This message will be used monitor information specific to the robolink
# joint_name is a string representing the name of the joint
# joint_num is the number of the joint
# joint_angle is the angle of the joint in degrees
# joint_velocity is the velocity of the joint in degrees per second


geometry_msgs/Pose current_position
int32[] joint_angles
int32[] joint_velocities
time stamp

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  __slots__ = ['current_position','joint_angles','joint_velocities','stamp']
  _slot_types = ['geometry_msgs/Pose','int32[]','int32[]','time']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       current_position,joint_angles,joint_velocities,stamp

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(RobolinkInfo, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.current_position is None:
        self.current_position = geometry_msgs.msg.Pose()
      if self.joint_angles is None:
        self.joint_angles = []
      if self.joint_velocities is None:
        self.joint_velocities = []
      if self.stamp is None:
        self.stamp = genpy.Time()
    else:
      self.current_position = geometry_msgs.msg.Pose()
      self.joint_angles = []
      self.joint_velocities = []
      self.stamp = genpy.Time()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_7d.pack(_x.current_position.position.x, _x.current_position.position.y, _x.current_position.position.z, _x.current_position.orientation.x, _x.current_position.orientation.y, _x.current_position.orientation.z, _x.current_position.orientation.w))
      length = len(self.joint_angles)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.joint_angles))
      length = len(self.joint_velocities)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.joint_velocities))
      _x = self
      buff.write(_struct_2I.pack(_x.stamp.secs, _x.stamp.nsecs))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.current_position is None:
        self.current_position = geometry_msgs.msg.Pose()
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.current_position.position.x, _x.current_position.position.y, _x.current_position.position.z, _x.current_position.orientation.x, _x.current_position.orientation.y, _x.current_position.orientation.z, _x.current_position.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_angles = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_velocities = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 8
      (_x.stamp.secs, _x.stamp.nsecs,) = _struct_2I.unpack(str[start:end])
      self.stamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_7d.pack(_x.current_position.position.x, _x.current_position.position.y, _x.current_position.position.z, _x.current_position.orientation.x, _x.current_position.orientation.y, _x.current_position.orientation.z, _x.current_position.orientation.w))
      length = len(self.joint_angles)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.joint_angles.tostring())
      length = len(self.joint_velocities)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.joint_velocities.tostring())
      _x = self
      buff.write(_struct_2I.pack(_x.stamp.secs, _x.stamp.nsecs))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.current_position is None:
        self.current_position = geometry_msgs.msg.Pose()
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.current_position.position.x, _x.current_position.position.y, _x.current_position.position.z, _x.current_position.orientation.x, _x.current_position.orientation.y, _x.current_position.orientation.z, _x.current_position.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_angles = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_velocities = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      _x = self
      start = end
      end += 8
      (_x.stamp.secs, _x.stamp.nsecs,) = _struct_2I.unpack(str[start:end])
      self.stamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_7d = struct.Struct("<7d")
_struct_2I = struct.Struct("<2I")
