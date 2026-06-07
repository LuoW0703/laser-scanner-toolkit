param(
    [string]$BuildDir = "$PSScriptRoot\build",
    [string]$BuildType = "Release",
    [switch]$Tests,
    [switch]$Benchmarks,
    [switch]$ValidationTests,
    [switch]$WarningsAsErrors,
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

function Import-VsDevEnvironment {
    if (Get-Command cl.exe -ErrorAction SilentlyContinue) {
        return
    }

    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) {
        throw "vswhere.exe not found. Install Visual Studio 2022 Build Tools or run from a Developer PowerShell."
    }

    $vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    $vcvarsPath = if ($vsPath) { Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat" } else { $null }

    if (-not $vcvarsPath -or -not (Test-Path $vcvarsPath)) {
        $vsRoot = Join-Path $env:ProgramFiles "Microsoft Visual Studio"
        if (Test-Path $vsRoot) {
            $candidate = Get-ChildItem -Path $vsRoot -Recurse -Filter vcvars64.bat -ErrorAction SilentlyContinue |
                Select-Object -First 1
            if ($candidate) {
                $vcvarsPath = $candidate.FullName
            }
        }
    }

    if (-not $vcvarsPath -or -not (Test-Path $vcvarsPath)) {
        throw "Visual Studio C++ toolchain was not found."
    }

    $cmdOutput = cmd /c "`"$vcvarsPath`" >nul 2>&1 && set"
    if ($LASTEXITCODE -ne 0) {
        throw "vcvars64.bat failed."
    }

    foreach ($line in $cmdOutput -split "`r`n") {
        if ($line -match "^([^=]+)=(.*)$") {
            [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], "Process")
        }
    }
}

function Find-CMake {
    $cmd = Get-Command cmake.exe -ErrorAction SilentlyContinue
    if ($cmd) {
        return $cmd.Source
    }

    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $vsPath = & $vswhere -latest -products * -property installationPath
        if ($vsPath) {
            $candidate = Join-Path $vsPath "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
            if (Test-Path $candidate) {
                return $candidate
            }
        }
    }

    $vsRoot = Join-Path $env:ProgramFiles "Microsoft Visual Studio"
    if (Test-Path $vsRoot) {
        $candidate = Get-ChildItem -Path $vsRoot -Recurse -Filter cmake.exe -ErrorAction SilentlyContinue |
            Where-Object { $_.FullName -like "*CommonExtensions*Microsoft*CMake*" } |
            Select-Object -First 1
        if ($candidate) {
            return $candidate.FullName
        }
    }

    throw "cmake.exe not found. Install CMake or Visual Studio's CMake component."
}

function Find-OpenCVDir {
    if ($env:OpenCV_DIR -and (Test-Path (Join-Path $env:OpenCV_DIR "OpenCVConfig.cmake"))) {
        return $env:OpenCV_DIR
    }

    $systemDrive = if ($env:SystemDrive) { $env:SystemDrive } else { "C:" }
    $roots = @(
        (Join-Path $systemDrive "OpenCV"),
        (Join-Path $systemDrive "tools\opencv")
    )
    if ($env:ProgramFiles) {
        $roots += (Join-Path $env:ProgramFiles "OpenCV")
    }
    $programFilesX86 = [Environment]::GetEnvironmentVariable("ProgramFiles(x86)")
    if ($programFilesX86) {
        $roots += (Join-Path $programFilesX86 "OpenCV")
    }

    foreach ($root in $roots) {
        if (-not (Test-Path $root)) {
            continue
        }

        $configs = @(Get-ChildItem -Path $root -Recurse -Filter OpenCVConfig.cmake -ErrorAction SilentlyContinue)
        $config = $configs |
            Where-Object { $_.DirectoryName -match "x64[\\/]+vc[0-9]+[\\/]+lib$" } |
            Select-Object -First 1
        if (-not $config) {
            $config = $configs | Select-Object -First 1
        }
        if ($config) {
            return $config.DirectoryName
        }
    }

    return $null
}

$ProjectDir = $PSScriptRoot
$CMake = Find-CMake
$DetectedOpenCVDir = Find-OpenCVDir

Write-Host "========================================" -ForegroundColor Cyan
Write-Host " Building Laser Scanner Toolkit" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

Import-VsDevEnvironment

if ($Clean -and (Test-Path $BuildDir)) {
    Remove-Item -Recurse -Force $BuildDir
}
New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null

$cmakeArgs = @(
    "-S", $ProjectDir,
    "-B", $BuildDir,
    "-G", "NMake Makefiles",
    "-DCMAKE_BUILD_TYPE=$BuildType"
)

if ($DetectedOpenCVDir) {
    Write-Host "  OpenCV_DIR: $DetectedOpenCVDir" -ForegroundColor Gray
    $cmakeArgs += "-DOpenCV_DIR=$DetectedOpenCVDir"
}
if ($Tests) {
    $cmakeArgs += "-DLSC_BUILD_TESTS=ON"
}
if ($Benchmarks) {
    $cmakeArgs += "-DLSC_BUILD_BENCHMARKS=ON"
}
if ($ValidationTests) {
    $cmakeArgs += "-DLSC_REGISTER_VALIDATION_TESTS=ON"
}
if ($WarningsAsErrors) {
    $cmakeArgs += "-DLSC_WARNINGS_AS_ERRORS=ON"
}

Write-Host ""
Write-Host "[1/2] Configuring with CMake..." -ForegroundColor Yellow
& $CMake @cmakeArgs
if ($LASTEXITCODE -ne 0) {
    throw "CMake configuration failed."
}

Write-Host ""
Write-Host "[2/2] Building $BuildType..." -ForegroundColor Yellow
& $CMake --build $BuildDir --config $BuildType
if ($LASTEXITCODE -ne 0) {
    throw "Build failed."
}

Write-Host ""
Write-Host "Build completed successfully." -ForegroundColor Green
Get-ChildItem -Path $BuildDir -Recurse -Filter "*.exe" | ForEach-Object {
    Write-Host "  $($_.FullName)" -ForegroundColor Gray
}
