@echo off
if "%VS120COMNTOOLS%" == "" (
  msg "%username%" "Visual Studio 12 not detected"
  exit 1
)

call "%VS120COMNTOOLS%\..\..\VC\vcvarsall.bat"

@mkdir 12bit
@mkdir 10bit
@mkdir 8bit

@cd 12bit
cmake -G "Visual Studio 12 Win64" ../../../source -DHIGH_BIT_DEPTH=ON -DEXPORT_C_API=OFF -DENABLE_SHARED=OFF -DENABLE_CLI=OFF -DMAIN12=ON
if exist s265.sln (
  MSBuild /property:Configuration="Release" s265.sln
  copy/y Release\s265-static.lib ..\8bit\s265-static-main12.lib
)

@cd ..\10bit
cmake -G "Visual Studio 12 Win64" ../../../source -DHIGH_BIT_DEPTH=ON -DEXPORT_C_API=OFF -DENABLE_SHARED=OFF -DENABLE_CLI=OFF
if exist s265.sln (
  MSBuild /property:Configuration="Release" s265.sln
  copy/y Release\s265-static.lib ..\8bit\s265-static-main10.lib
)

@cd ..\8bit
if not exist s265-static-main10.lib (
  msg "%username%" "10bit build failed"
  exit 1
)
if not exist s265-static-main12.lib (
  msg "%username%" "12bit build failed"
  exit 1
)
cmake -G "Visual Studio 12 Win64" ../../../source -DEXTRA_LIB="s265-static-main10.lib;s265-static-main12.lib" -DLINKED_10BIT=ON -DLINKED_12BIT=ON
if exist s265.sln (
  MSBuild /property:Configuration="Release" s265.sln
  :: combine static libraries (ignore warnings caused by winxp.cpp hacks)
  move Release\s265-static.lib s265-static-main.lib
  LIB.EXE /ignore:4006 /ignore:4221 /OUT:Release\s265-static.lib s265-static-main.lib s265-static-main10.lib s265-static-main12.lib
)

pause
