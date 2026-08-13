<#
  bmg_bridge.ps1 — Windows-side bridge to the BMG LABTECH reader ActiveX control.

  Run under 32-bit PowerShell only:
      C:\Windows\SysWOW64\WindowsPowerShell\v1.0\powershell.exe
  BMG_ActiveX.ocx is an x86 in-proc COM server.

  Emits exactly one JSON object on stdout; diagnostics go to stderr.

  ===========================================================================
  FOUR RULES, EACH ESTABLISHED THE HARD WAY ON 2026-07-29. Do not "clean up"
  any of them without re-running the probes.

  1. DYNAMIC DISPATCH ONLY. Never $com.GetType(), Get-Member, or
     Type.InvokeMember against this object. Its IDispatch::GetIDsOfNames raises
     an ACCESS VIOLATION for names it does not implement, and an AV is a
     corrupted-state exception that try/catch CANNOT intercept -- the whole
     process dies with exit 5 and an exception that will not even stringify.
     $com.GetType() alone triggers it ("GetType" is not a COM method here).

  2. USE THE ...V OVERLOADS. OpenConnection(string, Variant) fails with
     0x80020008 DISP_E_BADVARTYPE from PowerShell; OpenConnectionV(Variant,
     Variant) succeeds. Same for GetInfoV. Execute/ExecuteAndWait are already
     Variant-typed and need no suffix.

  3. THE OUT STATUS STRING IS UNREADABLE FROM POWERSHELL. Every method's
     trailing parameter is a by-ref OUT status (empty = OK, else error text),
     but PowerShell never marshals a value back into it -- it is empty even on
     calls that demonstrably reached the instrument. So an empty OUT proves
     NOTHING. Success is therefore inferred from "no exception was raised", and
     callers must verify real effects by observable artifacts (new export
     files, operator observation) rather than by this string.

  4. DO NOT DRIVE THIS FROM cscript/VBScript. It hard-crashes on OpenConnection
     (and on GetVersion). Not a container problem -- the control's typelib has
     coclass_sources = [], so it is an automation server, not a visual control;
     no message pump is needed. VBScript simply cannot survive its dispatch.

  5. *** THE ONE THAT ACTUALLY MATTERS ***
     Execute/ExecuteAndWait take VT_BYREF|VT_VARIANT and require the command as
     an ARRAY, not a bare string:
         WORKS :  $com.ExecuteAndWait(@('PlateOut'), [ref]$r)
         FAILS :  $com.ExecuteAndWait('PlateOut',   [ref]$r)   -> 0x8000FFFF
     @(...) marshals as a SAFEARRAY of VARIANT, which is exactly what the
     working Python clients send (comtypes converts lists this way). A bare
     string throws E_UNEXPECTED, which reads like "not connected" and sends you
     hunting the wrong bug for hours. Parameters are extra array elements:
     @('Temp','37'), @('Run', $protocol, $id1).
     Verified 2026-07-29 by three "(Plate command) / Processing Plate command"
     entries in the reader's own run log.

  6. NEVER call ExecuteAndWait2 or sExecuteAndWait2 (DISPIDs 11/12). Both
     hard-crash the process. The sExecute* family needs VT_LPSTR, which
     PowerShell cannot marshal at all (0x80020008).
  ===========================================================================

  Command verbs are the PLAIN ones (Init, PlateIn, PlateOut, Run, Temp), NOT the
  R_-prefixed script verbs. BMG's manual states script mode is unavailable while
  the software is in ActiveX/DDE mode, so R_PlateOut can never work here.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('connect', 'status', 'init', 'plate_in', 'plate_out', 'run', 'temp')]
    [string]$Action,

    [string]$Reader   = 'SPECTROstar_Nano',
    [string]$Protocol = '',
    [string]$PlateId1 = '',
    [string]$PlateId2 = '',
    [string]$PlateId3 = '',
    [string]$Target   = '',
    [switch]$KeepAlive
)

$ErrorActionPreference = 'Stop'

try {
    [Console]::OutputEncoding = New-Object System.Text.UTF8Encoding $false
    $OutputEncoding = [Console]::OutputEncoding
} catch { }

function Out-Json($obj) { $obj | ConvertTo-Json -Depth 6 -Compress }

# COM exceptions here can fail to stringify under the Korean locale.
function Fmt-Err($e) {
    $h = '?'; $m = ''
    try { $h = '0x' + ([Runtime.InteropServices.Marshal]::GetHRForException($e.Exception)).ToString('X8') } catch { }
    try { $m = [string]$e.Exception.Message } catch { }
    return ("HR=$h $m").Trim()
}

if ([IntPtr]::Size -ne 4) {
    Out-Json @{ ok = $false; error = 'bitness'; detail = 'must run under 32-bit PowerShell (SysWOW64)' }
    exit 2
}

try { $com = New-Object -ComObject BMG_ActiveX.BMGRemoteControl }
catch {
    Out-Json @{ ok = $false; error = 'com_create'; detail = (Fmt-Err $_) }
    exit 3
}

$opened = $false
try {
    # Rule 2: the V overload is the one that works.
    $res = ''
    $com.OpenConnectionV($Reader, [ref]$res)
    $opened = $true

    switch ($Action) {
        'connect' {
            Out-Json @{ ok = $true; action = $Action; reader = $Reader
                        note = 'connected; OUT status unreadable from PowerShell (see rule 3)' }
        }
        'status' {
            $info = ''
            $com.GetInfoV('Status', [ref]$info)
            Out-Json @{ ok = $true; action = $Action; status = [string]$info
                        note = 'empty status is expected -- OUT params do not marshal back (rule 3)' }
        }
        'init' {
            $r = ''; $com.ExecuteAndWait(@('Init'), [ref]$r)
            Out-Json @{ ok = $true; action = $Action }
        }
        'plate_out' {
            $r = ''; $com.ExecuteAndWait(@('PlateOut'), [ref]$r)
            Out-Json @{ ok = $true; action = $Action; carrier = 'out' }
        }
        'plate_in' {
            $r = ''; $com.ExecuteAndWait(@('PlateIn'), [ref]$r)
            Out-Json @{ ok = $true; action = $Action; carrier = 'in' }
        }
        'temp' {
            if ($Target -eq '') { throw 'temp requires -Target <degC>' }
            $r = ''
            $com.ExecuteAndWait(@('Temp', $Target), [ref]$r)
            Out-Json @{ ok = $true; action = $Action; target_c = $Target }
        }
        'run' {
            if ($Protocol -eq '') { throw 'run requires -Protocol <test protocol name>' }
            $r = ''
            $argv = @('Run', $Protocol)
            foreach ($id in @($PlateId1, $PlateId2, $PlateId3)) { if ($id -ne '') { $argv += $id } }
            $com.ExecuteAndWait($argv, [ref]$r)
            Out-Json @{ ok = $true; action = $Action; protocol = $Protocol }
        }
    }
    exit 0
}
catch {
    Out-Json @{ ok = $false; error = 'execute'; action = $Action; detail = (Fmt-Err $_) }
    exit 5
}
finally {
    if ($opened) {
        try {
            if ($KeepAlive) { $com.CloseConnectionWithoutTerminatingControlSoftware() }
            else { $com.CloseConnection() }
        } catch { }
    }
}
