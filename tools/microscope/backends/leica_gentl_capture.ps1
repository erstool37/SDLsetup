param(
    [ValidateSet("list", "capture")]
    [string]$Action = "capture",
    [string]$Output = "",
    [string]$OnlineOutput = "",
    [string]$Serial = "700011170655",
    [double]$ExposureUs = 60000,
    [double]$Gain = 1.0,
    [int]$TimeoutMs = 5000,
    [int]$Quality = 95,
    [string]$CtiPath = "C:\Windows\twain_64\Leica Microsystems\bin64\bgapi2_usb.cti"
)

$ErrorActionPreference = "Stop"

function Test-GentlPython {
    param([string]$PythonPath)

    if (-not $PythonPath -or -not (Test-Path $PythonPath)) {
        return $false
    }

    & $PythonPath -c "import harvesters, numpy, PIL" *> $null
    return $LASTEXITCODE -eq 0
}

function Find-GentlPython {
    $candidates = New-Object System.Collections.Generic.List[string]

    if ($env:LEICA_GENTL_PYTHON) {
        $candidates.Add($env:LEICA_GENTL_PYTHON)
    }

    $candidates.Add("C:\Users\$env:USERNAME\AppData\Local\Programs\Python\Python311\python.exe")
    $candidates.Add("C:\Users\$env:USERNAME\miniconda3\envs\control\python.exe")
    $candidates.Add("C:\Users\$env:USERNAME\miniconda3\python.exe")

    foreach ($candidate in ($candidates | Select-Object -Unique)) {
        if (Test-GentlPython $candidate) {
            return $candidate
        }
    }

    $pathPython = Get-Command python -ErrorAction SilentlyContinue
    if ($pathPython -and (Test-GentlPython $pathPython.Source)) {
        return $pathPython.Source
    }

    throw "No Windows Python with harvesters, numpy, and Pillow was found. Install with: python -m pip install harvesters genicam numpy pillow"
}

if ($Action -eq "capture" -and -not $Output) {
    throw "-Output is required for capture"
}

$python = Find-GentlPython
$script = Join-Path $PSScriptRoot "leica_gentl_capture_file.py"

$argsForPython = @(
    $script,
    $Action,
    "--cti-path", $CtiPath
)

if ($Serial) {
    $argsForPython += @("--serial", $Serial)
}

if ($Action -eq "capture") {
    $argsForPython += @(
        "--output", $Output,
        "--exposure-us", ([string]$ExposureUs),
        "--gain", ([string]$Gain),
        "--timeout-ms", ([string]$TimeoutMs),
        "--quality", ([string]$Quality)
    )
    if ($OnlineOutput) {
        $argsForPython += @("--online-output", $OnlineOutput)
    }
}

& $python @argsForPython
exit $LASTEXITCODE
