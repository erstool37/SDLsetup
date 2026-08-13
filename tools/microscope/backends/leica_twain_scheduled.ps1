param(
    [Parameter(Mandatory = $true)]
    [string]$Output,
    [string]$Source = "Leica Microsystems Camera",
    [string]$OnlineOutput = "",
    [int]$TimeoutSeconds = 45,
    [ValidateSet("native", "file")]
    [string]$TransferMech = "native",
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

function Convert-ToLocalWindowsPath {
    param(
        [string]$Path,
        [string]$RunDir,
        [string]$FileName
    )

    if ($Path -like "\\wsl.localhost\*") {
        return Join-Path $RunDir $FileName
    }
    return $Path
}

function Get-RunFileName {
    param(
        [string]$Path,
        [string]$BaseName,
        [string]$DefaultExtension
    )

    $extension = [System.IO.Path]::GetExtension($Path)
    if (-not $extension) {
        $extension = $DefaultExtension
    }
    return "$BaseName$extension"
}

$runId = "leica_twain_" + (Get-Date -Format "yyyyMMdd_HHmmss_ffff")
$runDir = Join-Path $env:TEMP $runId
New-Item -ItemType Directory -Force -Path $runDir | Out-Null

$python = Find-TwainPython
$helperSource = Join-Path $PSScriptRoot "leica_twain_capture_file.py"
$helper = Join-Path $runDir "leica_twain_capture_file.py"
Copy-Item -Force $helperSource $helper

$localOutput = Convert-ToLocalWindowsPath `
    -Path $Output `
    -RunDir $runDir `
    -FileName (Get-RunFileName -Path $Output -BaseName "capture" -DefaultExtension ".bmp")
$localOnline = ""
if ($OnlineOutput) {
    $localOnline = Convert-ToLocalWindowsPath `
        -Path $OnlineOutput `
        -RunDir $runDir `
        -FileName (Get-RunFileName -Path $OnlineOutput -BaseName "online" -DefaultExtension ".bmp")
}

$log = Join-Path $runDir "capture.log"
$exitCodeFile = Join-Path $runDir "capture.exitcode"
$cmd = Join-Path $runDir "capture.cmd"
$taskName = "CodexLeicaTwain_$runId"

$captureArgs = @(
    "capture",
    "--source", $Source,
    "--output", $localOutput,
    "--transfer-mech", $TransferMech
)
if ($localOnline) {
    $captureArgs += @("--online-output", $localOnline)
}
if ($ShowUI) {
    $captureArgs += "--show-ui"
}
if ($ModalUI) {
    $captureArgs += "--modal-ui"
}

$quotedArgs = ($captureArgs | ForEach-Object { '"' + ($_ -replace '"', '\"') + '"' }) -join " "
$cmdText = @"
@echo off
echo task_session: > "$log"
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "[Diagnostics.Process]::GetCurrentProcess().SessionId" >> "$log" 2>&1
"$python" "$helper" $quotedArgs >> "$log" 2>&1
echo %ERRORLEVEL% > "$exitCodeFile"
"@
Set-Content -Path $cmd -Value $cmdText -Encoding ASCII

$startTime = (Get-Date).AddMinutes(1).ToString("HH:mm")
try {
    schtasks.exe /Create /TN $taskName /TR "`"$cmd`"" /SC ONCE /ST $startTime /F /IT | Out-Null
    schtasks.exe /Run /TN $taskName | Out-Null

    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $deadline -and -not (Test-Path $exitCodeFile)) {
        Start-Sleep -Milliseconds 500
    }

    $exitCode = $null
    if (Test-Path $exitCodeFile) {
        $exitCode = [int](Get-Content $exitCodeFile -Raw).Trim()
    }
    $logText = ""
    if (Test-Path $log) {
        $logText = [string](Get-Content $log -Raw)
    }

    if ($null -eq $exitCode) {
        throw "Scheduled TWAIN capture did not finish within $TimeoutSeconds seconds. Log: $logText"
    }
    if ($exitCode -ne 0) {
        throw "Scheduled TWAIN capture failed with exit code $exitCode. Log: $logText"
    }
    if (-not (Test-Path $localOutput)) {
        throw "Scheduled TWAIN capture reported success but no output was created: $localOutput"
    }

    if ($localOutput -ne $Output) {
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Output) | Out-Null
        Copy-Item -Force $localOutput $Output
    }
    if ($OnlineOutput -and $localOnline -and (Test-Path $localOnline)) {
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $OnlineOutput) | Out-Null
        Copy-Item -Force $localOnline $OnlineOutput
    }

    [ordered]@{
        ok = $true
        output = $Output
        output_bytes = (Get-Item $Output).Length
        online_output = $OnlineOutput
        online_output_bytes = if ($OnlineOutput -and (Test-Path $OnlineOutput)) { (Get-Item $OnlineOutput).Length } else { 0 }
        task_name = $taskName
        run_dir = $runDir
        log = $logText
    } | ConvertTo-Json -Depth 4
} finally {
    schtasks.exe /Delete /TN $taskName /F *> $null
}
