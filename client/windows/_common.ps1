Set-StrictMode -Version Latest

function Read-PikaEnv {
    param([Parameter(Mandatory = $true)][string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw ".env 파일이 없습니다: $Path`n.env.example을 .env로 복사한 뒤 값을 확인하세요."
    }
    $values = @{}
    foreach ($rawLine in Get-Content -LiteralPath $Path -Encoding UTF8) {
        $line = $rawLine.Trim()
        if ($line.Length -eq 0 -or $line.StartsWith("#")) { continue }
        $separator = $line.IndexOf("=")
        if ($separator -le 0) {
            throw "잘못된 .env 줄입니다: $rawLine"
        }
        $key = $line.Substring(0, $separator).Trim()
        $value = $line.Substring($separator + 1).Trim()
        if ($key -notmatch '^[A-Z][A-Z0-9_]*$') {
            throw "잘못된 .env 키입니다: $key"
        }
        if ($value.Length -ge 2) {
            $first = $value.Substring(0, 1)
            $last = $value.Substring($value.Length - 1, 1)
            if (($first -eq '"' -and $last -eq '"') -or
                ($first -eq "'" -and $last -eq "'")) {
                $value = $value.Substring(1, $value.Length - 2)
            }
        }
        $values[$key] = $value
    }
    return $values
}

function Get-PikaValue {
    param(
        [Parameter(Mandatory = $true)][hashtable]$Values,
        [Parameter(Mandatory = $true)][string]$Key,
        [string]$Default = "",
        [switch]$Required
    )

    $value = if ($Values.ContainsKey($Key)) { [string]$Values[$Key] } else { $Default }
    if ($Required -and [string]::IsNullOrWhiteSpace($value)) {
        throw ".env에 $Key 값을 지정하세요."
    }
    return $value
}

function Resolve-PikaPath {
    param([Parameter(Mandatory = $true)][string]$Path)

    if ($Path -eq "~") { return $HOME }
    if ($Path.StartsWith("~\") -or $Path.StartsWith("~/")) {
        return Join-Path $HOME $Path.Substring(2)
    }
    return [Environment]::ExpandEnvironmentVariables($Path)
}

function Assert-PikaInteger {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)][string]$Value,
        [int]$Minimum,
        [int]$Maximum
    )

    $number = 0
    if (-not [int]::TryParse($Value, [ref]$number) -or
        $number -lt $Minimum -or $number -gt $Maximum) {
        throw "$Name 값은 $Minimum~$Maximum 정수여야 합니다: $Value"
    }
    return $number
}

function Assert-PikaDecimal {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)][string]$Value,
        [double]$Minimum,
        [double]$Maximum
    )

    $style = [Globalization.NumberStyles]::Float
    $culture = [Globalization.CultureInfo]::InvariantCulture
    $number = 0.0
    if (-not [double]::TryParse($Value, $style, $culture, [ref]$number) -or
        $number -lt $Minimum -or $number -gt $Maximum) {
        throw "$Name 값은 $Minimum~$Maximum 숫자여야 합니다: $Value"
    }
    return $number.ToString([Globalization.CultureInfo]::InvariantCulture)
}

function Get-PikaConnection {
    param([Parameter(Mandatory = $true)][hashtable]$Values)

    $hostName = Get-PikaValue $Values "PIKA_HOST" -Required
    $userName = Get-PikaValue $Values "PIKA_SSH_USER" -Required
    if ($hostName -notmatch '^[A-Za-z0-9.-]+$') {
        throw "PIKA_HOST 형식이 안전하지 않습니다: $hostName"
    }
    if ($userName -notmatch '^[A-Za-z0-9._-]+$') {
        throw "PIKA_SSH_USER 형식이 안전하지 않습니다: $userName"
    }
    $sshPort = Assert-PikaInteger "PIKA_SSH_PORT" (Get-PikaValue $Values "PIKA_SSH_PORT" "22") 1 65535
    $keyPath = Resolve-PikaPath (Get-PikaValue $Values "PIKA_SSH_KEY" "~\.ssh\id_ed25519_pika")
    return @{
        Host = $hostName
        User = $userName
        Port = $sshPort
        Key = $keyPath
        Target = "${userName}@${hostName}"
    }
}
