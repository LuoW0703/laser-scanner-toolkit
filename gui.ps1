param(
    [ValidateSet("Build", "Deploy", "Run", "All")]
    [string]$Action = "Run",
    [string]$BuildDir = "$PSScriptRoot\build_gui",
    [string]$CoreBuildDir = "$PSScriptRoot\build",
    [string]$AppDir = "$PSScriptRoot\app"
)

$ErrorActionPreference = "Stop"

function Find-CMake {
    $command = Get-Command cmake.exe -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    $roots = @(
        "$env:ProgramFiles\Microsoft Visual Studio",
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio"
    )
    foreach ($root in $roots) {
        if (-not (Test-Path $root)) {
            continue
        }
        $candidate = Get-ChildItem $root -Recurse -Filter cmake.exe -ErrorAction SilentlyContinue |
            Where-Object { $_.FullName -like "*CommonExtensions*Microsoft*CMake*" } |
            Select-Object -First 1
        if ($candidate) {
            return $candidate.FullName
        }
    }
    throw "cmake.exe was not found."
}

function Find-Qt5Dir {
    if ($env:QT5_DIR -and (Test-Path (Join-Path $env:QT5_DIR "Qt5Config.cmake"))) {
        return (Resolve-Path $env:QT5_DIR).Path
    }

    $roots = @("C:\Qt", (Join-Path $env:USERPROFILE "Qt"))
    foreach ($root in $roots) {
        if (-not (Test-Path $root)) {
            continue
        }
        $candidate = Get-ChildItem $root -Recurse -Filter Qt5Config.cmake -ErrorAction SilentlyContinue |
            Where-Object { $_.FullName -match "mingw.*[\\/]lib[\\/]cmake[\\/]Qt5[\\/]Qt5Config\.cmake$" } |
            Sort-Object FullName -Descending |
            Select-Object -First 1
        if ($candidate) {
            return $candidate.DirectoryName
        }
    }
    throw "Qt5Config.cmake was not found. Set QT5_DIR to Qt's lib\cmake\Qt5 directory."
}

function Find-MinGwBin([string]$Qt5Dir) {
    if ($env:MINGW_BIN -and (Test-Path (Join-Path $env:MINGW_BIN "g++.exe"))) {
        return (Resolve-Path $env:MINGW_BIN).Path
    }

    $searchRoot = $Qt5Dir
    for ($i = 0; $i -lt 6 -and $searchRoot; ++$i) {
        $toolsRoot = Join-Path $searchRoot "Tools"
        if (Test-Path $toolsRoot) {
            $candidate = Get-ChildItem $toolsRoot -Recurse -Filter g++.exe -ErrorAction SilentlyContinue |
                Select-Object -First 1
            if ($candidate) {
                return $candidate.DirectoryName
            }
        }
        $searchRoot = Split-Path $searchRoot
    }
    throw "MinGW was not found. Set MINGW_BIN to the directory containing g++.exe."
}

function Build-Gui {
    $cmake = Find-CMake
    $qt5Dir = Find-Qt5Dir
    $mingwBin = Find-MinGwBin $qt5Dir
    $env:PATH = "$mingwBin;$env:PATH"

    Write-Host "Building Qt GUI..." -ForegroundColor Cyan
    & $cmake -S $PSScriptRoot -B $BuildDir -G "MinGW Makefiles" `
        "-DQt5_DIR=$qt5Dir" `
        "-DLSC_BUILD_GUI=ON" `
        "-DCMAKE_BUILD_TYPE=Release"
    if ($LASTEXITCODE -ne 0) {
        throw "GUI configuration failed."
    }

    & $cmake --build $BuildDir --target laser_scanner_gui -j 4
    if ($LASTEXITCODE -ne 0) {
        throw "GUI build failed."
    }
}

function Deploy-Gui {
    $guiExe = Join-Path $BuildDir "LaserScannerToolkit.exe"
    $demoExe = Join-Path $CoreBuildDir "demo_full_pipeline.exe"
    if (-not (Test-Path $guiExe)) {
        throw "GUI executable not found. Run .\gui.ps1 Build first."
    }
    if (-not (Test-Path $demoExe)) {
        throw "Demo executable not found. Run .\build.ps1 first."
    }

    New-Item -ItemType Directory -Path $AppDir -Force | Out-Null
    Copy-Item $guiExe $AppDir -Force
    Copy-Item $demoExe $AppDir -Force
    Get-ChildItem $CoreBuildDir -Filter "*.dll" -File -ErrorAction SilentlyContinue |
        Copy-Item -Destination $AppDir -Force

    $qt5Dir = Find-Qt5Dir
    $qtBin = Resolve-Path (Join-Path $qt5Dir "..\..\..\bin")
    $mingwBin = Find-MinGwBin $qt5Dir
    $env:PATH = "$($qtBin.Path);$mingwBin;$env:PATH"

    foreach ($runtime in "libgcc_s_seh-1.dll", "libstdc++-6.dll", "libwinpthread-1.dll") {
        $source = Join-Path $mingwBin $runtime
        if (Test-Path $source) {
            Copy-Item $source $AppDir -Force
        }
    }

    $deployTool = Join-Path $qtBin.Path "windeployqt.exe"
    if (Test-Path $deployTool) {
        & $deployTool (Join-Path $AppDir "LaserScannerToolkit.exe") `
            --no-translations --no-compiler-runtime
        if ($LASTEXITCODE -ne 0) {
            throw "windeployqt failed."
        }
    }

    $launcherSource = Join-Path $PSScriptRoot "launcher\main.cpp"
    $launcherExe = Join-Path $PSScriptRoot "LaserScannerToolkit.exe"
    $launcherBuildExe = Join-Path $PSScriptRoot "launcher\LaserScannerToolkit.exe"
    $compiler = Join-Path $mingwBin "g++.exe"
    & $compiler $launcherSource -std=c++17 -O2 -s -mwindows -municode `
        -static -static-libgcc -static-libstdc++ -o $launcherBuildExe
    if ($LASTEXITCODE -ne 0) {
        throw "Launcher build failed."
    }
    Copy-Item $launcherBuildExe $launcherExe -Force
    Remove-Item $launcherBuildExe -Force
    Write-Host "GUI deployment completed." -ForegroundColor Green
}

function Run-Gui {
    $guiExe = Join-Path $PSScriptRoot "LaserScannerToolkit.exe"
    if (-not (Test-Path $guiExe)) {
        throw "GUI executable not found. Run .\gui.ps1 All first."
    }
    Start-Process -FilePath $guiExe -WorkingDirectory $AppDir
}

switch ($Action) {
    "Build" { Build-Gui }
    "Deploy" { Deploy-Gui }
    "Run" { Run-Gui }
    "All" {
        Build-Gui
        Deploy-Gui
        Run-Gui
    }
}
