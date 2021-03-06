# Python stubs generated by omniidl from idl/DepthCamera.idl
# DO NOT EDIT THIS FILE!

import omniORB, _omnipy
from omniORB import CORBA, PortableServer
_0_CORBA = CORBA


_omnipy.checkVersion(4,2, __file__, 1)

try:
    property
except NameError:
    def property(*args):
        return None


# #include "BasicDataType.idl"
import BasicDataType_idl
_0_RTC = omniORB.openModule("RTC")
_0_RTC__POA = omniORB.openModule("RTC__POA")

# #include "ExtendedDataTypes.idl"
import ExtendedDataTypes_idl
_0_RTC = omniORB.openModule("RTC")
_0_RTC__POA = omniORB.openModule("RTC__POA")

# #include "CameraCommonInterface.idl"
import CameraCommonInterface_idl
_0_Img = omniORB.openModule("Img")
_0_Img__POA = omniORB.openModule("Img__POA")

#
# Start of module "RGBDCamera"
#
__name__ = "RGBDCamera"
_0_RGBDCamera = omniORB.openModule("RGBDCamera", r"idl/DepthCamera.idl")
_0_RGBDCamera__POA = omniORB.openModule("RGBDCamera__POA", r"idl/DepthCamera.idl")


# struct DepthImage
_0_RGBDCamera.DepthImage = omniORB.newEmptyClass()
class DepthImage (omniORB.StructBase):
    _NP_RepositoryId = "IDL:RGBDCamera/DepthImage:1.0"

    def __init__(self, width, height, verticalFOV, horizontalFOV, raw_data):
        self.width = width
        self.height = height
        self.verticalFOV = verticalFOV
        self.horizontalFOV = horizontalFOV
        self.raw_data = raw_data

_0_RGBDCamera.DepthImage = DepthImage
_0_RGBDCamera._d_DepthImage  = (omniORB.tcInternal.tv_struct, DepthImage, DepthImage._NP_RepositoryId, "DepthImage", "width", omniORB.tcInternal.tv_long, "height", omniORB.tcInternal.tv_long, "verticalFOV", omniORB.tcInternal.tv_double, "horizontalFOV", omniORB.tcInternal.tv_double, "raw_data", (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_double, 0))
_0_RGBDCamera._tc_DepthImage = omniORB.tcInternal.createTypeCode(_0_RGBDCamera._d_DepthImage)
omniORB.registerType(DepthImage._NP_RepositoryId, _0_RGBDCamera._d_DepthImage, _0_RGBDCamera._tc_DepthImage)
del DepthImage

# struct RGBDCameraImage
_0_RGBDCamera.RGBDCameraImage = omniORB.newEmptyClass()
class RGBDCameraImage (omniORB.StructBase):
    _NP_RepositoryId = "IDL:RGBDCamera/RGBDCameraImage:1.0"

    def __init__(self, cameraImage, depthImage, geometry):
        self.cameraImage = cameraImage
        self.depthImage = depthImage
        self.geometry = geometry

_0_RGBDCamera.RGBDCameraImage = RGBDCameraImage
_0_RGBDCamera._d_RGBDCameraImage  = (omniORB.tcInternal.tv_struct, RGBDCameraImage, RGBDCameraImage._NP_RepositoryId, "RGBDCameraImage", "cameraImage", omniORB.typeMapping["IDL:Img/CameraImage:1.0"], "depthImage", omniORB.typeMapping["IDL:RGBDCamera/DepthImage:1.0"], "geometry", omniORB.typeMapping["IDL:RTC/Geometry3D:1.0"])
_0_RGBDCamera._tc_RGBDCameraImage = omniORB.tcInternal.createTypeCode(_0_RGBDCamera._d_RGBDCameraImage)
omniORB.registerType(RGBDCameraImage._NP_RepositoryId, _0_RGBDCamera._d_RGBDCameraImage, _0_RGBDCamera._tc_RGBDCameraImage)
del RGBDCameraImage

# struct TimedDepthImage
_0_RGBDCamera.TimedDepthImage = omniORB.newEmptyClass()
class TimedDepthImage (omniORB.StructBase):
    _NP_RepositoryId = "IDL:RGBDCamera/TimedDepthImage:1.0"

    def __init__(self, tm, data):
        self.tm = tm
        self.data = data

_0_RGBDCamera.TimedDepthImage = TimedDepthImage
_0_RGBDCamera._d_TimedDepthImage  = (omniORB.tcInternal.tv_struct, TimedDepthImage, TimedDepthImage._NP_RepositoryId, "TimedDepthImage", "tm", omniORB.typeMapping["IDL:RTC/Time:1.0"], "data", omniORB.typeMapping["IDL:RGBDCamera/DepthImage:1.0"])
_0_RGBDCamera._tc_TimedDepthImage = omniORB.tcInternal.createTypeCode(_0_RGBDCamera._d_TimedDepthImage)
omniORB.registerType(TimedDepthImage._NP_RepositoryId, _0_RGBDCamera._d_TimedDepthImage, _0_RGBDCamera._tc_TimedDepthImage)
del TimedDepthImage

# struct TimedRGBDCameraImage
_0_RGBDCamera.TimedRGBDCameraImage = omniORB.newEmptyClass()
class TimedRGBDCameraImage (omniORB.StructBase):
    _NP_RepositoryId = "IDL:RGBDCamera/TimedRGBDCameraImage:1.0"

    def __init__(self, tm, data):
        self.tm = tm
        self.data = data

_0_RGBDCamera.TimedRGBDCameraImage = TimedRGBDCameraImage
_0_RGBDCamera._d_TimedRGBDCameraImage  = (omniORB.tcInternal.tv_struct, TimedRGBDCameraImage, TimedRGBDCameraImage._NP_RepositoryId, "TimedRGBDCameraImage", "tm", omniORB.typeMapping["IDL:RTC/Time:1.0"], "data", omniORB.typeMapping["IDL:RGBDCamera/RGBDCameraImage:1.0"])
_0_RGBDCamera._tc_TimedRGBDCameraImage = omniORB.tcInternal.createTypeCode(_0_RGBDCamera._d_TimedRGBDCameraImage)
omniORB.registerType(TimedRGBDCameraImage._NP_RepositoryId, _0_RGBDCamera._d_TimedRGBDCameraImage, _0_RGBDCamera._tc_TimedRGBDCameraImage)
del TimedRGBDCameraImage

#
# End of module "RGBDCamera"
#
__name__ = "DepthCamera_idl"

_exported_modules = ( "RGBDCamera", )

# The end.
