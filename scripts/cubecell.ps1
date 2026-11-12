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
$ArduinoHome = Join-Path $env:LOCALAPPDATA 'arduino-cli-cubecell'
$env:ARDUINO_DIRECTORIES_DATA = $ArduinoHome
$env:ARDUINO_DIRECTORIES_DOWNLOADS = Join-Path $ArduinoHome 'staging'
$env:ARDUINO_DIRECTORIES_USER = Join-Path $ArduinoHome 'sketchbook'
$SketchPath = if ($Sketch) {
    if ([System.IO.Path]::IsPathRooted($Sketch)) { $Sketch } else { Join-Path $Root $Sketch }
} else {
    $Root
}
$BuildName = if ($Sketch) { 'build-' + (Split-Path $SketchPath -Leaf) } else { 'build' }
$Build = Join-Path $Root $BuildName

# HTCC-AB01 board options. FQBN values are menu IDs from boards.txt (not 1=on).
$Board = 'CubeCell:CubeCell-Arduino:CubeCell-Board'
$BoardOptions = @(
    'LORAWAN_REGION=6'          # 6=EU868
    'LORAWAN_CLASS=0'           # 0=Class A, 2=Class C
    'LORAWAN_DEVEUI=1'          # 0=from credentials.h, 1=from ChipID
    'LORAWAN_NETMODE=0'         # 0=OTAA, 1=ABP
    'LORAWAN_ADR=0'             # 0=ON, 1=OFF
    'LORAWAN_UPLINKMODE=1'      # 0=confirmed, 1=unconfirmed
    'LORAWAN_Net_Reserve=0'     # 0=OFF, 1=ON (persist session in flash)
    'LORAWAN_AT_SUPPORT=0'      # 0=ON, 1=OFF
    'LORAWAN_RGB=1'             # 0=off, 1=on
    'LORAWAN_PREAMBLE_LENGTH=0' # 0=8 symbols, 1=16 (M00/M00L)
    'LORAWAN_DebugLevel=0'      # 0=none, 1=freq, 2=freq+DIO
)
$Fqbn = $Board + ':' + ($BoardOptions -join ',')

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
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

function Invoke-Monitor {
    $port = Get-UploadPort
    Write-Host "Serial monitor on $port at 115200 baud"
    & $Cli monitor --config-file $Config --port $port --config baudrate=115200
    exit $LASTEXITCODE
}

switch ($Action) {
    'build' {
        Invoke-Build
        exit 0
    }
    'upload' {
        Invoke-Upload
        exit 0
    }
    'flash' {
        Invoke-Build
        Invoke-Upload
        # Port comes back after reboot; give the board a moment.
        Start-Sleep -Seconds 2
        Invoke-Monitor
    }
    'monitor' {
        Invoke-Monitor
    }
}
