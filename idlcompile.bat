@echo off
cd /d %~dp0
setlocal
for %%I in (python.exe) do if exist %%~$path:I set f=%%~$path:I
if exist %f% (
  %f:python.exe=%omniidl.exe -bpython -I"C:\Program Files\OpenRTM-aist\1.2.0\rtm\idl" idl/TidyUpManager.idl idl/ManipulatorCommonInterface_DataTypes.idl idl/BasicDataType.idl idl/DepthCamera.idl idl/ManipulatorCommonInterface_MiddleLevel.idl idl/Img.idl idl/ManipulatorCommonInterface_Common.idl idl/CameraCommonInterface.idl
) else (
  echo "python.exe" can not be found.
  echo Please modify PATH environmental variable for python command.
)
endlocal
