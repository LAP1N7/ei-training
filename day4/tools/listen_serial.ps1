# 보드를 리셋하지 않고 시리얼만 듣는다.
#
# day3 의 read_serial.ps1 은 DTR/RTS 를 올려서 보드를 리셋시킨다. ESP32-S3 의
# USB-Serial/JTAG 는 리셋되면 USB 를 재열거하기 때문에 호스트가 쥐고 있던 포트
# 핸들이 무효가 되고, 그래서 부팅 로그 첫 줄만 잡히고 그 뒤가 통째로 날아간다.
# 이미 돌고 있는 펌웨어가 뭘 찍는지 볼 때는 이 스크립트를 쓴다.
#
#   .\listen_serial.ps1 -Port COM10 -Seconds 20
param(
    [string]$Port = "COM10",
    [int]$Seconds = 20,
    [int]$Baud = 115200
)

if (-not ([System.IO.Ports.SerialPort]::GetPortNames() -contains $Port)) {
    Write-Output "ERROR: $Port 없음. 사용 가능: $([System.IO.Ports.SerialPort]::GetPortNames() -join ', ')"
    exit 1
}

$p = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$p.ReadTimeout = 1000
$p.NewLine = "`n"
# 보드는 UTF-8 로 찍는다. 기본값(ASCII)이면 한글이 전부 ? 로 깨진다.
$p.Encoding = [System.Text.Encoding]::UTF8
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$p.DtrEnable = $false      # <- 이 두 줄이 핵심. 올리면 보드가 리셋된다.
$p.RtsEnable = $false

try { $p.Open() } catch {
    Write-Output "ERROR: $Port 열기 실패 ($($_.Exception.Message)). 다른 프로세스가 쥐고 있을 수 있음."
    exit 1
}

$deadline = (Get-Date).AddSeconds($Seconds)
while ((Get-Date) -lt $deadline) {
    try { Write-Output $p.ReadLine() } catch [TimeoutException] {}
}
$p.Close()
