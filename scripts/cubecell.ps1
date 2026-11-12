param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('build', 'upload', 'flash', 'monitor')]
    [string]$Action,
    [string]$Sketch = ''
)

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Cli = Join-Path $env:LOCALAPPDATA 'Programs\arduino-cli\arduino-cli.exe'
$Config = Join-Path $Root 'arduino-cli.yaml'
$SketchPath = if ($Sketch) {
    if ([System.IO.Path]::IsPathRooted($Sketch)) { $Sketch } else { Join-Path $Root $Sketch }
} else {
    $Root
}
$BuildName = if ($Sketch) { 'build-' + (Split-Path $SketchPath -Leaf) } else { 'build' }
$Build = Join-Path $Root $BuildName

# HTCC-AB01, EU868, Class A, OTAA, ADR on, unconfirmed uplinks, AT commands on, RGB active.
$Fqbn = 'CubeCell:CubeCell-Arduino:CubeCell-Board:LORAWAN_REGION=6,LORAWAN_CLASS=0,LORAWAN_DEVEUI=0,LORAWAN_NETMODE=0,LORAWAN_ADR=0,LORAWAN_UPLINKMODE=1,LORAWAN_Net_Reserve=0,LORAWAN_AT_SUPPORT=0,LORAWAN_RGB=1,LORAWAN_PREAMBLE_LENGTH=0,LORAWAN_DebugLevel=0'

function Get-UploadPort {
    $localFile = Join-Path $Root 'cubecell.local.json'
    if (Test-Path $localFile) {
        $settings = Get-Content $localFile -Raw | ConvertFrom-Json
        if ($settings.port) {
            return [string]$settings.port
        }
    }

    $raw = & $Cli --config-file $Config board list --format json
    if ($LASTEXITCODE -ne 0) {
        throw 'Could not list serial ports.'
    }

    $ports = @()
    if ($raw) {
        $parsed = $raw | ConvertFrom-Json
        foreach ($entry in @($parsed.detected_ports)) {
            if ($entry.port.protocol -eq 'serial' -and $entry.port.address) {
                $ports += [string]$entry.port.address
            }
        }
    }

    $ports = @($ports | Select-Object -Unique)
    if ($ports.Count -eq 1) {
        return $ports[0]
    }
    if ($ports.Count -eq 0) {
        throw 'No serial port found. Connect the board, or set "port" in cubecell.local.json, for example {"port":"COM5"}.'
    }

    throw ('More than one serial port ({0}). Set "port" in cubecell.local.json.' -f ($ports -join ', '))
}

if (-not (Test-Path $Cli)) {
    throw "arduino-cli was not found at $Cli"
}

function Invoke-Build {
    & $Cli compile --config-file $Config --fqbn $Fqbn --build-path $Build $SketchPath
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

function Invoke-Upload {
    $port = Get-UploadPort
    Write-Host "Uploading to $port"
    & $Cli upload --config-file $Config --fqbn $Fqbn --port $port --input-dir $Build $SketchPath
    exit $LASTEXITCODE
}

switch ($Action) {
    'build' {
        Invoke-Build
        exit 0
    }
    'upload' {
        Invoke-Upload
    }
    'flash' {
        Invoke-Build
        Invoke-Upload
    }
    'monitor' {
        $port = Get-UploadPort
        Write-Host "Serial monitor on $port at 115200 baud"
        & $Cli monitor --config-file $Config --port $port --config baudrate=115200
        exit $LASTEXITCODE
    }
}
