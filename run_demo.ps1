param(
    [string]$Executable = "$PSScriptRoot\build\demo_full_pipeline.exe"
)

$ErrorActionPreference = "Stop"

if ($env:OpenCV_BIN) {
    $env:PATH = "$env:OpenCV_BIN;$env:PATH"
} elseif ($env:OpenCV_DIR) {
    $candidateBins = @(
        (Join-Path $env:OpenCV_DIR "bin"),
        (Join-Path $env:OpenCV_DIR "..\bin"),
        (Join-Path $env:OpenCV_DIR "x64\vc15\bin"),
        (Join-Path $env:OpenCV_DIR "x64\vc16\bin")
    )
    foreach ($bin in $candidateBins) {
        $resolved = Resolve-Path $bin -ErrorAction SilentlyContinue
        if ($resolved) {
            $env:PATH = "$resolved;$env:PATH"
            break
        }
    }
}

Set-Location $PSScriptRoot
New-Item -ItemType Directory -Path "output" -Force | Out-Null

Write-Host "========================================" -ForegroundColor Cyan
Write-Host " Running: $Executable" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

$stdoutLog = Join-Path $env:TEMP "lsc_demo_stdout.txt"
$stderrLog = Join-Path $env:TEMP "lsc_demo_stderr.txt"

$proc = Start-Process -FilePath $Executable `
    -NoNewWindow -Wait -PassThru `
    -RedirectStandardOutput $stdoutLog `
    -RedirectStandardError $stderrLog

Write-Host "Exit code: $($proc.ExitCode)" -ForegroundColor $(if($proc.ExitCode -eq 0){'Green'}else{'Red'})

Write-Host ""
Write-Host "========== STDOUT =========="
if (Test-Path $stdoutLog) {
    Get-Content $stdoutLog | Select-Object -First 200
}

Write-Host ""
Write-Host "========== STDERR (last 30 lines) =========="
if (Test-Path $stderrLog) {
    $stderr = Get-Content $stderrLog
    if ($stderr) {
        $stderr | Select-Object -Last 30 | ForEach-Object { Write-Host $_ -ForegroundColor Yellow }
    } else {
        Write-Host "(empty)" -ForegroundColor Green
    }
}
