param(
    [Parameter(Mandatory = $true)]
    [string]$Output,
    [string]$SampleDir = "C:\Users\Public\MicroscopeAutomationTransfer\code\HorizontalPython\tisgrabber\samples",
    [ValidateSet("JPEG", "BMP")]
    [string]$ImageType = "JPEG",
    [int]$JpegQuality = 90,
    [int]$SnapTimeoutMs = 2000,
    [double]$SettleSeconds = 0,
    [double]$ExposureSeconds = -1,
    [double]$Gain = -1,
    [int]$Brightness = -1,
    [switch]$ShowLive,
    [string]$DeviceUniqueName = ""
)

$ErrorActionPreference = "Stop"

function Test-WindowsPython {
    param([string]$PythonPath)

    if (-not $PythonPath -or -not (Test-Path $PythonPath)) {
        return $false
    }

    & $PythonPath -c "import ctypes" *> $null
    return $LASTEXITCODE -eq 0
}

function Find-WindowsPython {
    $candidates = New-Object System.Collections.Generic.List[string]

    if ($env:TISGRABBER_PYTHON) {
        $candidates.Add($env:TISGRABBER_PYTHON)
    }

    $candidates.Add("C:\Users\$env:USERNAME\miniconda3\envs\control\python.exe")
    $candidates.Add("C:\Users\$env:USERNAME\miniconda3\python.exe")
    $candidates.Add("C:\Users\KWON MIN KYUNG\miniconda3\envs\control\python.exe")
    $candidates.Add("C:\Users\KWON MIN KYUNG\miniconda3\python.exe")

    foreach ($candidate in ($candidates | Select-Object -Unique)) {
        if (Test-WindowsPython $candidate) {
            return $candidate
        }
    }

    $pathPython = Get-Command python -ErrorAction SilentlyContinue
    if ($pathPython -and (Test-WindowsPython $pathPython.Source)) {
        return $pathPython.Source
    }

    throw "No Windows Python was found. Set TISGRABBER_PYTHON to python.exe."
}

$python = Find-WindowsPython
$script = Join-Path $PSScriptRoot "tisgrabber_capture_file.py"
$argsForPython = @(
    $script,
    "--sample-dir", $SampleDir,
    "--output", $Output,
    "--image-type", $ImageType,
    "--jpeg-quality", [string]$JpegQuality,
    "--snap-timeout-ms", [string]$SnapTimeoutMs,
    "--settle-s", [string]$SettleSeconds
)

if ($ExposureSeconds -ge 0) {
    $argsForPython += @("--exposure-s", [string]$ExposureSeconds)
}

if ($Gain -ge 0) {
    $argsForPython += @("--gain", [string]$Gain)
}

if ($Brightness -ge 0) {
    $argsForPython += @("--brightness", [string]$Brightness)
}

if ($ShowLive) {
    $argsForPython += "--show-live"
}

if ($DeviceUniqueName -ne "") {
    $argsForPython += @("--device-unique-name", $DeviceUniqueName)
}

& $python @argsForPython
exit $LASTEXITCODE
