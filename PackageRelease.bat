@echo off
setlocal enabledelayedexpansion

set project_name=Xidi
set project_platforms=Win32 x64

set project_has_sdk=yes
set project_has_third_party_license=yes

set files_release=LICENSE README.md
set files_release_build=dinput.dll dinput8.dll winmm.dll
set files_release_build_Win32=Xidi.32.dll Xidi.HookModule.32.dll
set files_release_build_x64=Xidi.64.dll Xidi.HookModule.64.dll

set files_sdk_include=Include\Xidi\*.h

set third_party_license=Hookshot Boost XstdBitSet

set infra_command="Modules\Infra\Build\Scripts\PackageRelease.bat" %* 
call %%infra_command%%
