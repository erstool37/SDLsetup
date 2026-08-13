param(
    [ValidateSet("status", "capture", "acquire", "stop")]
    [string]$Action = "status",
    [int]$WaitSeconds = 3
)

$ErrorActionPreference = "Stop"

Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes

function Find-LasXProcess {
    $proc = Get-Process LMSApplication -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $proc) {
        throw "LAS X process LMSApplication.exe is not running"
    }
    return $proc
}

function Find-ElementByAutomationId {
    param([string]$AutomationId)

    $proc = Find-LasXProcess
    $root = [System.Windows.Automation.AutomationElement]::RootElement
    $procCond = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ProcessIdProperty,
        $proc.Id
    )
    $idCond = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::AutomationIdProperty,
        $AutomationId
    )
    $cond = New-Object System.Windows.Automation.AndCondition($procCond, $idCond)
    $element = $root.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $cond)
    if (-not $element) {
        throw "LAS X UI element not found: $AutomationId"
    }
    return $element
}

function Get-ToggleState {
    param([string]$AutomationId)

    $element = Find-ElementByAutomationId $AutomationId
    $pattern = $element.GetCurrentPattern([System.Windows.Automation.TogglePattern]::Pattern)
    return [string]$pattern.Current.ToggleState
}

function Set-ToggleState {
    param(
        [string]$AutomationId,
        [string]$DesiredState
    )

    $element = Find-ElementByAutomationId $AutomationId
    $pattern = $element.GetCurrentPattern([System.Windows.Automation.TogglePattern]::Pattern)
    $before = [string]$pattern.Current.ToggleState
    if ($before -ne $DesiredState) {
        $pattern.Toggle()
        Start-Sleep -Milliseconds 500
    }
    $after = [string]$pattern.Current.ToggleState
    return @{
        id = $AutomationId
        before = $before
        after = $after
    }
}

function Get-LasXState {
    $ids = @(
        "CheckBoxSingleImageMode",
        "ToggleButtonAcquire",
        "StateInfoLive",
        "StateInfoSingleScan",
        "StateInfoCaptureImage",
        "StateInfoStart"
    )
    $state = @{}
    foreach ($id in $ids) {
        try {
            $state[$id] = Get-ToggleState $id
        } catch {
            $state[$id] = "missing: $($_.Exception.Message)"
        }
    }
    return $state
}

function Get-RecentImageFiles {
    param([datetime]$Since)

    $paths = @(
        "$env:USERPROFILE\Desktop\camera_captures",
        "$env:USERPROFILE\Documents",
        "$env:USERPROFILE\Pictures",
        "$env:APPDATA\Leica Microsystems"
    )
    $files = @()
    foreach ($path in $paths) {
        if (-not (Test-Path $path)) {
            continue
        }
        $files += Get-ChildItem $path -Recurse -File -ErrorAction SilentlyContinue |
            Where-Object {
                $_.LastWriteTime -ge $Since -and
                $_.Extension -match '^\.(lif|lof|xlef|tif|tiff|png|jpg|jpeg)$'
            } |
            Select-Object FullName, Length, LastWriteTime
    }
    return @($files | Sort-Object LastWriteTime -Descending)
}

$started = Get-Date
$result = [ordered]@{
    ok = $false
    action = $Action
    started_at = $started.ToString("o")
    message = ""
    states_before = $null
    states_after = $null
    recent_files = @()
}

try {
    $result.states_before = Get-LasXState

    if ($Action -eq "capture") {
        Set-ToggleState "StateInfoCaptureImage" "On" | Out-Null
        Start-Sleep -Seconds $WaitSeconds
        Set-ToggleState "StateInfoCaptureImage" "Off" | Out-Null
        $result.message = "Toggled LAS X Capture Image control. File export depends on LAS X project/export settings."
    } elseif ($Action -eq "acquire") {
        Set-ToggleState "ToggleButtonAcquire" "On" | Out-Null
        Start-Sleep -Seconds $WaitSeconds
        Set-ToggleState "ToggleButtonAcquire" "Off" | Out-Null
        $result.message = "Toggled LAS X Acquire control. File export depends on LAS X project/export settings."
    } elseif ($Action -eq "stop") {
        Set-ToggleState "StateInfoCaptureImage" "Off" | Out-Null
        Set-ToggleState "ToggleButtonAcquire" "Off" | Out-Null
        $result.message = "Stopped LAS X capture/acquire toggles where they were on."
    } else {
        $result.message = "Read LAS X UI acquisition state."
    }

    $result.states_after = Get-LasXState
    $result.recent_files = Get-RecentImageFiles $started
    $result.ok = $true
} catch {
    $result.message = $_.Exception.Message
}

$result | ConvertTo-Json -Depth 5
