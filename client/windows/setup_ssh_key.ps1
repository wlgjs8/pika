[CmdletBinding()]
param([string]$EnvFile = "")

$ErrorActionPreference = "Stop"
$scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($scriptDirectory)) {
    throw "스크립트 경로를 확인할 수 없습니다. setup_ssh_key.ps1을 파일로 실행하세요."
}
if ([string]::IsNullOrWhiteSpace($EnvFile)) {
    $EnvFile = Join-Path $scriptDirectory ".env"
}
. (Join-Path $scriptDirectory "_common.ps1")

try {
    $values = Read-PikaEnv $EnvFile
    $connection = Get-PikaConnection $values
    if (-not (Get-Command ssh.exe -ErrorAction SilentlyContinue)) {
        throw "Windows OpenSSH client(ssh.exe)가 없습니다. Windows 선택적 기능에서 OpenSSH Client를 설치하세요."
    }
    if (-not (Get-Command ssh-keygen.exe -ErrorAction SilentlyContinue)) {
        throw "Windows OpenSSH client(ssh-keygen.exe)가 없습니다."
    }

    $keyDirectory = Split-Path -Parent $connection.Key
    if (-not (Test-Path -LiteralPath $keyDirectory)) {
        New-Item -ItemType Directory -Path $keyDirectory -Force | Out-Null
    }
    if (-not (Test-Path -LiteralPath $connection.Key -PathType Leaf)) {
        Write-Host "새 Ed25519 키를 만듭니다: $($connection.Key)" -ForegroundColor Cyan
        Write-Host "완전 자동 실행을 원하면 passphrase 질문에서 Enter를 두 번 누르세요." -ForegroundColor Yellow
        & ssh-keygen.exe -t ed25519 -f $connection.Key -C "pika-preview@$env:COMPUTERNAME"
        if ($LASTEXITCODE -ne 0) { throw "ssh-keygen 실패(exit=$LASTEXITCODE)" }
    }
    $publicKeyPath = "$($connection.Key).pub"
    if (-not (Test-Path -LiteralPath $publicKeyPath -PathType Leaf)) {
        throw "공개키가 없습니다: $publicKeyPath"
    }
    $publicKey = (Get-Content -LiteralPath $publicKeyPath -Raw).Trim()
    if ($publicKey -notmatch '^ssh-ed25519 [A-Za-z0-9+/=]+(?: [A-Za-z0-9@._-]+)?$') {
        throw "예상하지 못한 Ed25519 공개키 형식입니다: $publicKeyPath"
    }

    $testArgs = @(
        "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
        "-o", "IdentitiesOnly=yes", "-i", $connection.Key,
        "-p", [string]$connection.Port, $connection.Target, "true"
    )
    & ssh.exe @testArgs
    if ($LASTEXITCODE -eq 0) {
        Write-Host "SSH 키가 이미 등록되어 있습니다." -ForegroundColor Green
        exit 0
    }

    Write-Host "원격 PC에 공개키를 등록합니다. SSH 비밀번호를 이번 한 번 입력하세요." -ForegroundColor Cyan
    $install = 'umask 077; mkdir -p "$HOME/.ssh"; touch "$HOME/.ssh/authorized_keys"; key=''{0}''; grep -qxF "$key" "$HOME/.ssh/authorized_keys" || printf "%s\n" "$key" >> "$HOME/.ssh/authorized_keys"; chmod 700 "$HOME/.ssh"; chmod 600 "$HOME/.ssh/authorized_keys"' -f $publicKey
    & ssh.exe -p ([string]$connection.Port) $connection.Target $install
    if ($LASTEXITCODE -ne 0) { throw "원격 공개키 등록 실패(exit=$LASTEXITCODE)" }

    & ssh.exe @testArgs
    if ($LASTEXITCODE -ne 0) { throw "공개키 등록 후 SSH 접속 확인 실패(exit=$LASTEXITCODE)" }
    Write-Host "설정 완료. 이제 start_collect_preview.ps1을 실행하세요." -ForegroundColor Green
}
catch {
    Write-Error $_
    exit 1
}
