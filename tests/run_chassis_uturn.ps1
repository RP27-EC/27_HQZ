param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$testDirectory = Join-Path ([IO.Path]::GetTempPath()) ('chassis-uturn-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testDirectory | Out-Null

function Read-Production([string]$relativePath) {
    [IO.File]::ReadAllText((Join-Path $repoRoot $relativePath))
}

function Remove-Includes([string]$source) {
    [regex]::Replace($source, '(?m)^\s*#include[^\r\n]*', '')
}

function Get-Function([string]$source, [string]$signature) {
    $start = $source.IndexOf($signature, [StringComparison]::Ordinal)
    if ($start -lt 0) { throw "Missing production function: $signature" }
    $end = $source.IndexOf("`n}", $start, [StringComparison]::Ordinal)
    if ($end -lt 0) { throw "Missing function end: $signature" }
    $source.Substring($start, $end + 2 - $start)
}

try {
    $protocol = Read-Production 'task_down/Application/ProtocolLayer/board_protocol.c'
    $control = Read-Production 'task_down/Application/TaskLayer/control_task.c'
    $controlEnd = $control.IndexOf('uint8_t open_ui', [StringComparison]::Ordinal)
    if ($controlEnd -lt 0) { throw 'Missing control task boundary' }
    $protocolHeader = Read-Production 'task_down/Application/ProtocolLayer/board_protocol.h'
    $remoteHeader = Read-Production 'task_down/Application/DeviceLayer/Sensor/rc_sensor.h'
    $definitions = @(
        [regex]::Matches($protocolHeader, '(?m)^\s*#define\s+(BOARD_|ID_PKT_0[125])[^\r\n]*') | ForEach-Object Value
        [regex]::Matches($remoteHeader, '(?m)^\s*#define\s+(RC_SW_|KEY_PRESSED_OFFSET_)[^\r\n]*') | ForEach-Object Value
    ) -join "`n"
    $production = @(
        $definitions
        Get-Function (Read-Production 'task_down/Application/DriverLayer/drv_can.c') 'HAL_StatusTypeDef CAN_SendData('
        Remove-Includes (Read-Production 'task_down/Application/ModuleLayer/chassis_input.c')
        Remove-Includes $control.Substring(0, $controlEnd)
        Get-Function $protocol 'void Board_Tx_Pkt_01('
        Get-Function $protocol 'void Board_Tx_Pkt_02('
        Get-Function $protocol 'static float Board_Remote_Axis_To_Rate('
        Get-Function $protocol 'void Board_Tx_Pkt_05('
        Remove-Includes (Read-Production 'task_down/Application/ModuleLayer/chassis_follow.c')
    ) -join "`n"
    [IO.File]::WriteAllText((Join-Path $testDirectory 'production.inc'), $production)
    $executable = Join-Path $testDirectory 'chassis_uturn_test.exe'
    & $Compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-Wno-unused-parameter' `
        '-I' $testDirectory `
        '-I' (Join-Path $repoRoot 'task_down/Application/ModuleLayer') `
        '-I' (Join-Path $repoRoot 'task_down/Application/ConfigLayer') `
        (Join-Path $PSScriptRoot 'chassis_uturn_test.c') '-lm' '-o' $executable
    if ($LASTEXITCODE -ne 0) { throw 'Regression test compilation failed' }
    & $executable
    $testExitCode = $LASTEXITCODE
} finally {
    $resolvedTestDirectory = [IO.Path]::GetFullPath($testDirectory)
    $tempRoot = [IO.Path]::GetFullPath([IO.Path]::GetTempPath())
    if (-not $resolvedTestDirectory.StartsWith($tempRoot, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Temporary directory is outside the temporary root'
    }
    Remove-Item -LiteralPath $resolvedTestDirectory -Recurse -Force
}
exit $testExitCode
