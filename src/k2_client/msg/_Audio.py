"""autogenerated by genpy from k2_client/Audio.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Audio(genpy.Message):
  _md5sum = "37cc5db6e0123e6d300ef4b2f9d18939"
  _type = "k2_client/Audio"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
float64 beamAngle
float64 beamAngleConfidence
float32[] audioStream
int16 numBytesPerSample
int16 numSamplesPerFrame
float64 frameLifeTime
int16 samplingFrequency
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header','beamAngle','beamAngleConfidence','audioStream','numBytesPerSample','numSamplesPerFrame','frameLifeTime','samplingFrequency']
  _slot_types = ['std_msgs/Header','float64','float64','float32[]','int16','int16','float64','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,beamAngle,beamAngleConfidence,audioStream,numBytesPerSample,numSamplesPerFrame,frameLifeTime,samplingFrequency

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Audio, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.beamAngle is None:
        self.beamAngle = 0.
      if self.beamAngleConfidence is None:
        self.beamAngleConfidence = 0.
      if self.audioStream is None:
        self.audioStream = []
      if self.numBytesPerSample is None:
        self.numBytesPerSample = 0
      if self.numSamplesPerFrame is None:
        self.numSamplesPerFrame = 0
      if self.frameLifeTime is None:
        self.frameLifeTime = 0.
      if self.samplingFrequency is None:
        self.samplingFrequency = 0
    else:
      self.header = std_msgs.msg.Header()
      self.beamAngle = 0.
      self.beamAngleConfidence = 0.
      self.audioStream = []
      self.numBytesPerSample = 0
      self.numSamplesPerFrame = 0
      self.frameLifeTime = 0.
      self.samplingFrequency = 0

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d.pack(_x.beamAngle, _x.beamAngleConfidence))
      length = len(self.audioStream)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.audioStream))
      _x = self
      buff.write(_struct_2hdh.pack(_x.numBytesPerSample, _x.numSamplesPerFrame, _x.frameLifeTime, _x.samplingFrequency))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.beamAngle, _x.beamAngleConfidence,) = _struct_2d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.audioStream = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 14
      (_x.numBytesPerSample, _x.numSamplesPerFrame, _x.frameLifeTime, _x.samplingFrequency,) = _struct_2hdh.unpack(str[start:end])
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d.pack(_x.beamAngle, _x.beamAngleConfidence))
      length = len(self.audioStream)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.audioStream.tostring())
      _x = self
      buff.write(_struct_2hdh.pack(_x.numBytesPerSample, _x.numSamplesPerFrame, _x.frameLifeTime, _x.samplingFrequency))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.beamAngle, _x.beamAngleConfidence,) = _struct_2d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.audioStream = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 14
      (_x.numBytesPerSample, _x.numSamplesPerFrame, _x.frameLifeTime, _x.samplingFrequency,) = _struct_2hdh.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2d = struct.Struct("<2d")
_struct_3I = struct.Struct("<3I")
_struct_2hdh = struct.Struct("<2hdh")