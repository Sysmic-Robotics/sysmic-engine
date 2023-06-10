from . import messages_robocup_ssl_detection_pb2 as _messages_robocup_ssl_detection_pb2
from . import messages_robocup_ssl_geometry_pb2 as _messages_robocup_ssl_geometry_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class SSL_WrapperPacket(_message.Message):
    __slots__ = ["detection", "geometry"]
    DETECTION_FIELD_NUMBER: _ClassVar[int]
    GEOMETRY_FIELD_NUMBER: _ClassVar[int]
    detection: _messages_robocup_ssl_detection_pb2.SSL_DetectionFrame
    geometry: _messages_robocup_ssl_geometry_pb2.SSL_GeometryData
    def __init__(self, detection: _Optional[_Union[_messages_robocup_ssl_detection_pb2.SSL_DetectionFrame, _Mapping]] = ..., geometry: _Optional[_Union[_messages_robocup_ssl_geometry_pb2.SSL_GeometryData, _Mapping]] = ...) -> None: ...
