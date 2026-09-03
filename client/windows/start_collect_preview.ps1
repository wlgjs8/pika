[CmdletBinding()]
param([string]$EnvFile = "")

$ErrorActionPreference = "Stop"
$scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($scriptDirectory)) {
    throw "스크립트 경로를 확인할 수 없습니다. start_collect_preview.ps1을 파일로 실행하세요."
}
if ([string]::IsNullOrWhiteSpace($EnvFile)) {
    $EnvFile = Join-Path $scriptDirectory ".env"
}
. (Join-Path $scriptDirectory "_common.ps1")

try {
    $values = Read-PikaEnv $EnvFile
    $connection = Get-PikaConnection $values
    if (-not (Get-Command ssh.exe -ErrorAction SilentlyContinue)) {
        throw "Windows OpenSSH client(ssh.exe)가 없습니다."
    }
    if (-not (Test-Path -LiteralPath $connection.Key -PathType Leaf)) {
        throw "SSH 키가 없습니다: $($connection.Key)`n먼저 setup_ssh_key.ps1을 실행하세요."
    }

    $remoteDirectory = Get-PikaValue $values "PIKA_REMOTE_DIR" "/home/plaif/workspace/pika"
    if ($remoteDirectory -notmatch '^/[A-Za-z0-9._/-]+$') {
        throw "PIKA_REMOTE_DIR은 공백 없는 절대 Linux 경로여야 합니다: $remoteDirectory"
    }
    $localPort = Assert-PikaInteger "PIKA_LOCAL_PREVIEW_PORT" (Get-PikaValue $values "PIKA_LOCAL_PREVIEW_PORT" "8765") 1 65535
    $remotePort = Assert-PikaInteger "PIKA_REMOTE_PREVIEW_PORT" (Get-PikaValue $values "PIKA_REMOTE_PREVIEW_PORT" "8765") 1 65535
    $previewFps = Assert-PikaDecimal "PIKA_PREVIEW_FPS" (Get-PikaValue $values "PIKA_PREVIEW_FPS" "10") 0.1 30
    $tileWidth = Assert-PikaInteger "PIKA_PREVIEW_TILE_WIDTH" (Get-PikaValue $values "PIKA_PREVIEW_TILE_WIDTH" "320") 80 1280
    $jpegQuality = Assert-PikaInteger "PIKA_PREVIEW_JPEG_QUALITY" (Get-PikaValue $values "PIKA_PREVIEW_JPEG_QUALITY" "70") 20 95

    # 이미 사용 중인 포트에 -L이 잘못 붙는 상황을 실행 전에 명확히 알린다.
    $listener = [Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback, $localPort)
    try { $listener.Start() }
    catch { throw "localhost:$localPort 포트가 이미 사용 중입니다. 기존 preview/SSH tunnel을 종료하거나 .env 포트를 바꾸세요." }
    finally { $listener.Stop() }

    $url = "http://127.0.0.1:$localPort/"
    $healthUrl = "http://127.0.0.1:$localPort/healthz"
    $browserJob = Start-Job -ScriptBlock {
        param($HealthUrl, $Url)
        $deadline = [DateTime]::UtcNow.AddMinutes(3)
        while ([DateTime]::UtcNow -lt $deadline) {
            try {
                $response = Invoke-WebRequest -Uri $HealthUrl -UseBasicParsing -TimeoutSec 1
                if ($response.StatusCode -eq 200) {
                    Start-Process $Url
                    return
                }
            } catch { }
            Start-Sleep -Milliseconds 500
        }
    } -ArgumentList $healthUrl, $url

    $forward = "${localPort}:127.0.0.1:${remotePort}"
    $sshArgs = @(
        "-tt", "-o", "ExitOnForwardFailure=yes",
        "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3",
        "-o", "BatchMode=yes",
        "-o", "IdentitiesOnly=yes", "-i", $connection.Key,
        "-p", [string]$connection.Port, "-L", $forward
    )
    $remoteCommand = "cd -- $remoteDirectory && exec env PIKA_PREVIEW=1 PIKA_PREVIEW_MODE=web PIKA_PREVIEW_PORT=$remotePort PIKA_PREVIEW_FPS=$previewFps PIKA_PREVIEW_TILE_WIDTH=$tileWidth PIKA_PREVIEW_JPEG_QUALITY=$jpegQuality ./scripts/run_collect_fast.sh"

    Write-Host "원격 수집을 시작합니다: $($connection.Target)" -ForegroundColor Cyan
    Write-Host "프리뷰: $url (준비되면 자동으로 열립니다)" -ForegroundColor Cyan
    Write-Host "이 창에서 b=녹화 토글, Ctrl+C=저장 후 종료" -ForegroundColor Yellow
    & ssh.exe @sshArgs $connection.Target $remoteCommand
    $sshExit = $LASTEXITCODE
    if ($sshExit -ne 0) { throw "SSH/원격 수집 종료 코드: $sshExit" }
}
catch {
    Write-Error $_
    exit 1
}
finally {
    if (Get-Variable browserJob -ErrorAction SilentlyContinue) {
        Stop-Job $browserJob -ErrorAction SilentlyContinue
        Remove-Job $browserJob -Force -ErrorAction SilentlyContinue
    }
}
