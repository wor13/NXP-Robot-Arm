# Run with: powershell -ExecutionPolicy Bypass -File .\build.ps1
# All generated files go to this example's build directory.
param([ValidateSet('debug', 'release')][string]$Configuration = 'debug')
$ErrorActionPreference = 'Stop'

# Walk upward to the SDK root so this example can move with your SDK checkout.
$sdkDirectory = Get-Item -LiteralPath $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $sdkDirectory.FullName 'cmake\extension\mcux.cmake'))) {
    $sdkDirectory = $sdkDirectory.Parent
    if ($null -eq $sdkDirectory) { throw 'Cannot locate the surrounding MCUX SDK.' }
}

# These versions match your installed MCUXpresso tools. If you upgrade the
# tools later, update these paths (or configure CMake directly with your tools).
$toolsDirectory = Join-Path $env:USERPROFILE '.mcuxpressotools'
$env:ARMGCC_DIR = Join-Path $toolsDirectory 'arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi'
$env:MCUX_VENV_PATH = Join-Path $toolsDirectory '.mcux-venv-3.12\Scripts'
$cmakeExecutable = Join-Path $toolsDirectory 'cmake-3.30.0-windows-x86_64\bin\cmake.exe'
$ninjaExecutable = Join-Path $toolsDirectory 'ninja-1.13.2\ninja.exe'
$savedPath = $env:PATH
try {
    $env:PATH = "$env:MCUX_VENV_PATH;$savedPath"
    $buildDirectory = Join-Path $PSScriptRoot "build\$Configuration"
    # & runs an executable whose path is stored in a variable.
    # An argument array preserves spaces in paths without shell string building.
    $configureArguments = @(
        '-S', $PSScriptRoot, '-B', $buildDirectory, '-G', 'Ninja',
        "-DCMAKE_MAKE_PROGRAM=$ninjaExecutable",
        "-DCMAKE_TOOLCHAIN_FILE=$($sdkDirectory.FullName)/cmake/toolchain/armgcc.cmake",
        "-DSdkRootDirPath=$($sdkDirectory.FullName)",
        "-DPython3_EXECUTABLE=$env:MCUX_VENV_PATH/python.exe",
        '-DCONFIG_TOOLCHAIN=armgcc', '-Dboard=frdmmcxn947',
        '-Dcore_id=cm33_core0', "-DCMAKE_BUILD_TYPE=$Configuration"
    )
    & $cmakeExecutable @configureArguments
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
    & $cmakeExecutable --build $buildDirectory
    if ($LASTEXITCODE -ne 0) { throw 'Firmware compilation/linking failed.' }
} finally {
    $env:PATH = $savedPath
}
