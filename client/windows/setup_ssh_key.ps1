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
        Write-Host "자동 실행 전용 Ed25519 키를 만듭니다: $($connection.Key)" -ForegroundColor Cyan
        & ssh-keygen.exe -q -t ed25519 -N '""' -f $connection.Key -C "pika-preview@$env:COMPUTERNAME"
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
    # Windows PowerShell 5.1의 native-command quoting을 거치면 공개키 안의 공백과
    # 원격 shell 따옴표가 사라질 수 있다. 공백 없는 Base64 payload로 전달해 피한다.
    $publicKeyBytes = [Text.Encoding]::UTF8.GetBytes($publicKey + "`n")
    $publicKeyBase64 = [Convert]::ToBase64String($publicKeyBytes)

    $testArgs = @(
        "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
        "-o", "IdentitiesOnly=yes", "-i", $connection.Key,
        "-p", [string]$connection.Port, $connection.Target, "true"
    )
    # Windows PowerShell 5.1은 native stderr도 ErrorActionPreference=Stop일 때
    # terminating error로 올릴 수 있다. 첫 미등록 확인만 예외 승격을 잠시 끈다.
    $savedErrorAction = $ErrorActionPreference
    try {
        $ErrorActionPreference = "SilentlyContinue"
        & ssh.exe @testArgs 2>$null
        $preflightExit = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $savedErrorAction
    }
    if ($preflightExit -eq 0) {
        Write-Host "SSH 키가 이미 등록되어 있습니다." -ForegroundColor Green
        exit 0
    }

    Write-Host "원격 PC에 공개키를 등록합니다. SSH 비밀번호를 이번 한 번 입력하세요." -ForegroundColor Cyan
    # 따옴표가 없어도 안전한 원격 명령만 사용한다. temp 파일과 grep -f를 써서
    # 공개키의 공백을 shell 변수로 다시 해석하지 않고, 재실행해도 중복 추가하지 않는다.
    $install = "umask 077; mkdir -p .ssh; touch .ssh/authorized_keys; printf %s $publicKeyBase64 | base64 -d > .ssh/.pika_preview_key.tmp; grep -qxF -f .ssh/.pika_preview_key.tmp .ssh/authorized_keys || cat .ssh/.pika_preview_key.tmp >> .ssh/authorized_keys; rm -f .ssh/.pika_preview_key.tmp; chmod 700 .ssh; chmod 600 .ssh/authorized_keys"
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
