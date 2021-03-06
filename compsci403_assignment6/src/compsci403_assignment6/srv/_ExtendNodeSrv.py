# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from compsci403_assignment6/ExtendNodeSrvRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class ExtendNodeSrvRequest(genpy.Message):
  _md5sum = "8df2b28522a9692779fb75035225f38c"
  _type = "compsci403_assignment6/ExtendNodeSrvRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """geometry_msgs/Point[] P
geometry_msgs/Point q

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['P','q']
  _slot_types = ['geometry_msgs/Point[]','geometry_msgs/Point']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       P,q

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ExtendNodeSrvRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.P is None:
        self.P = []
      if self.q is None:
        self.q = geometry_msgs.msg.Point()
    else:
      self.P = []
      self.q = geometry_msgs.msg.Point()

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
      length = len(self.P)
      buff.write(_struct_I.pack(length))
      for val1 in self.P:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_3d().pack(_x.q.x, _x.q.y, _x.q.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.P is None:
        self.P = None
      if self.q is None:
        self.q = geometry_msgs.msg.Point()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.P = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.P.append(val1)
      _x = self
      start = end
      end += 24
      (_x.q.x, _x.q.y, _x.q.z,) = _get_struct_3d().unpack(str[start:end])
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
      length = len(self.P)
      buff.write(_struct_I.pack(length))
      for val1 in self.P:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_3d().pack(_x.q.x, _x.q.y, _x.q.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.P is None:
        self.P = None
      if self.q is None:
        self.q = geometry_msgs.msg.Point()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.P = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.P.append(val1)
      _x = self
      start = end
      end += 24
      (_x.q.x, _x.q.y, _x.q.z,) = _get_struct_3d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from compsci403_assignment6/ExtendNodeSrvResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class ExtendNodeSrvResponse(genpy.Message):
  _md5sum = "519eb737cd9604c1a839cf9555d83b5e"
  _type = "compsci403_assignment6/ExtendNodeSrvResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 q_near_index
geometry_msgs/Point q_new


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['q_near_index','q_new']
  _slot_types = ['int32','geometry_msgs/Point']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       q_near_index,q_new

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ExtendNodeSrvResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.q_near_index is None:
        self.q_near_index = 0
      if self.q_new is None:
        self.q_new = geometry_msgs.msg.Point()
    else:
      self.q_near_index = 0
      self.q_new = geometry_msgs.msg.Point()

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
      buff.write(_get_struct_i3d().pack(_x.q_near_index, _x.q_new.x, _x.q_new.y, _x.q_new.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.q_new is None:
        self.q_new = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 28
      (_x.q_near_index, _x.q_new.x, _x.q_new.y, _x.q_new.z,) = _get_struct_i3d().unpack(str[start:end])
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
      buff.write(_get_struct_i3d().pack(_x.q_near_index, _x.q_new.x, _x.q_new.y, _x.q_new.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.q_new is None:
        self.q_new = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 28
      (_x.q_near_index, _x.q_new.x, _x.q_new.y, _x.q_new.z,) = _get_struct_i3d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i3d = None
def _get_struct_i3d():
    global _struct_i3d
    if _struct_i3d is None:
        _struct_i3d = struct.Struct("<i3d")
    return _struct_i3d
class ExtendNodeSrv(object):
  _type          = 'compsci403_assignment6/ExtendNodeSrv'
  _md5sum = '39895a811c277641b38420f358a62ecd'
  _request_class  = ExtendNodeSrvRequest
  _response_class = ExtendNodeSrvResponse
