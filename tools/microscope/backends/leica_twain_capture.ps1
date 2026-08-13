param(
    [ValidateSet("sources", "capabilities", "capture")]
    [string]$Action = "sources",
    [string]$Source = "Leica Microsystems Camera",
    [string]$Output = "",
    [string]$OnlineOutput = "",
    [switch]$ShowUI,
    [switch]$ModalUI
)

$ErrorActionPreference = "Stop"

function Test-TwainPython {
    param([string]$PythonPath)

    if (-not $PythonPath -or -not (Test-Path $PythonPath)) {
        return $false
    }

    & $PythonPath -c "import twain" *> $null
    return $LASTEXITCODE -eq 0
}

function Find-TwainPython {
    $candidates = New-Object System.Collections.Generic.List[string]

    if ($env:LEICA_TWAIN_PYTHON) {
        $candidates.Add($env:LEICA_TWAIN_PYTHON)
    }

    $candidates.Add("C:\Users\$env:USERNAME\AppData\Local\Programs\Python\Python311\python.exe")
    $candidates.Add("C:\Users\$env:USERNAME\miniconda3\envs\control\python.exe")
    $candidates.Add("C:\Users\$env:USERNAME\miniconda3\python.exe")
    $candidates.Add("C:\Users\KWON MIN KYUNG\miniconda3\envs\control\python.exe")

    foreach ($candidate in ($candidates | Select-Object -Unique)) {
        if (Test-TwainPython $candidate) {
            return $candidate
        }
    }

    $pathPython = Get-Command python -ErrorAction SilentlyContinue
    if ($pathPython -and (Test-TwainPython $pathPython.Source)) {
        return $pathPython.Source
    }

    throw "No Windows Python with importable 'twain' was found. Set LEICA_TWAIN_PYTHON to python.exe from an environment with pytwain installed."
}

if ($Action -eq "capture" -and -not $Output) {
    throw "-Output is required for capture"
}

$python = Find-TwainPython
$script = Join-Path $PSScriptRoot "leica_twain_capture_file.py"
$argsForPython = @($script, $Action, "--source", $Source)

if ($Action -eq "capture") {
    $argsForPython += @("--output", $Output)
    if ($OnlineOutput) {
        $argsForPython += @("--online-output", $OnlineOutput)
    }
    if ($ShowUI) {
        $argsForPython += "--show-ui"
    }
    if ($ModalUI) {
        $argsForPython += "--modal-ui"
    }
}

& $python @argsForPython
exit $LASTEXITCODE
