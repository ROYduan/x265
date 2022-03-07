@echo off
if "%VS120COMNTOOLS%" == "" (
  msg "%username%" "Visual Studio 12 not detected"
  exit 1
)
if not exist s265.sln (
  call make-solutions.bat
)
if exist s265.sln (
  call "%VS120COMNTOOLS%\..\..\VC\vcvarsall.bat"
  MSBuild /property:Configuration="Release" s265.sln
  MSBuild /property:Configuration="Debug" s265.sln
  MSBuild /property:Configuration="RelWithDebInfo" s265.sln
)
