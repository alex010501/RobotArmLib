@REM Install Eigen using vcpkg

@ECHO OFF

if NOT EXIST vcpkg (
	git clone https://github.com/microsoft/vcpkg

	call .\vcpkg\bootstrap-vcpkg.bat

    call .\vcpkg\vcpkg.exe integrate install
)

@REM x64 and x86 branch
reg Query "HKLM\Hardware\Description\System\CentralProcessor\0" | find /i "x86" > NUL && set OS=32BIT || set OS=64BIT

if %OS%==32BIT (
    @ECHO "x86 branch"
    call .\vcpkg\vcpkg.exe install eigen3:x86-windows-static
)

if %OS%==64BIT (
    @ECHO "x64 branch"
    call .\vcpkg\vcpkg.exe install eigen3:x64-windows-static
)