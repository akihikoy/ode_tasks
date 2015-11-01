"""autogenerated by genpy from ode1/ODESensor2.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ODESensor2(genpy.Message):
  _md5sum = "c7b01b1aa3f1520ef797252f66d2f811"
  _type = "ode1/ODESensor2"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64[] joint_angles

# Poses (x,y,z,qx,qy,qz,qw) of links; [7]*(12+JointNum+1)
float64[] link_x

# Force/torque (fx,fy,fz,tx,ty,tz) of sensors and joints; [6]*(8+JointNum)
float64[] forces

# Masses of links; [12+JointNum+1]
float64[] masses

# Pose (x,y,z,qx,qy,qz,qw) of box1; [7]
float64[] box1_x

# Simulation time
float64 time

"""
  __slots__ = ['joint_angles','link_x','forces','masses','box1_x','time']
  _slot_types = ['float64[]','float64[]','float64[]','float64[]','float64[]','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       joint_angles,link_x,forces,masses,box1_x,time

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ODESensor2, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.joint_angles is None:
        self.joint_angles = []
      if self.link_x is None:
        self.link_x = []
      if self.forces is None:
        self.forces = []
      if self.masses is None:
        self.masses = []
      if self.box1_x is None:
        self.box1_x = []
      if self.time is None:
        self.time = 0.
    else:
      self.joint_angles = []
      self.link_x = []
      self.forces = []
      self.masses = []
      self.box1_x = []
      self.time = 0.

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
      length = len(self.joint_angles)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.joint_angles))
      length = len(self.link_x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.link_x))
      length = len(self.forces)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.forces))
      length = len(self.masses)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.masses))
      length = len(self.box1_x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.box1_x))
      buff.write(_struct_d.pack(self.time))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_angles = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.link_x = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.forces = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.masses = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.box1_x = struct.unpack(pattern, str[start:end])
      start = end
      end += 8
      (self.time,) = _struct_d.unpack(str[start:end])
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
      length = len(self.joint_angles)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.joint_angles.tostring())
      length = len(self.link_x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.link_x.tostring())
      length = len(self.forces)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.forces.tostring())
      length = len(self.masses)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.masses.tostring())
      length = len(self.box1_x)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.box1_x.tostring())
      buff.write(_struct_d.pack(self.time))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_angles = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.link_x = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.forces = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.masses = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.box1_x = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 8
      (self.time,) = _struct_d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_d = struct.Struct("<d")
