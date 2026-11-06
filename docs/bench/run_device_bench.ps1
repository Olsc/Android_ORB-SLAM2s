# 离线回放基准：构建 arm64 Release 引擎 + slam_bench，推到设备，用固定帧序列运行并取回结果。
#
# 用法（在仓库根目录）：
#   powershell -File docs\bench\run_device_bench.ps1 -Tag base            # 构建 + 运行，结果保存为 docs\bench\results\base.txt
#   powershell -File docs\bench\run_device_bench.ps1 -Tag opt1 -Runs 2    # 跑两遍（第一遍预热页缓存，取第二遍的耗时）
#   powershell -File docs\bench\run_device_bench.ps1 -Tag x -SkipBuild     # 只运行不构建
#   powershell -File docs\bench\run_device_bench.ps1 -Compare base,opt1    # 逐位比较两份结果的位姿序列
#
# 帧文件由 ffmpeg 生成（1280x720 灰度 raw，与相机分辨率一致）：
#   ffmpeg -i 测试用视频.avi -an -vf "scale=1280:720:flags=bilinear,format=gray" -f rawvideo -pix_fmt gray frames.raw
param(
    [string]$Tag = "run",
    [int]$Runs = 2,
    [int]$MaxFrames = 1017,
    [int]$Sync = 1,
    [switch]$SkipBuild,
    [switch]$SkipPush,
    [string[]]$Compare,
    [string]$Adb = "E:\工具箱\env\platform-tools\adb.exe",
    [string]$FramesLocal = "D:\slam_bench_data\frames_1280x720_gray.raw",
    [string]$DeviceDir = "/data/local/tmp/slam"
)
$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$resDir = Join-Path $PSScriptRoot "results"
New-Item -ItemType Directory -Force $resDir | Out-Null

if ($Compare) {
    if ($Compare.Count -eq 1) { $Compare = $Compare[0] -split ',' }
    $a = Get-Content (Join-Path $resDir "$($Compare[0]).txt") | Where-Object { $_ -notmatch '^#' } | ForEach-Object { ($_ -split ' ')[0,1,2] + ($_ -split ' ')[4..99] -join ' ' }
    $b = Get-Content (Join-Path $resDir "$($Compare[1]).txt") | Where-Object { $_ -notmatch '^#' } | ForEach-Object { ($_ -split ' ')[0,1,2] + ($_ -split ' ')[4..99] -join ' ' }
    $n = [Math]::Min($a.Count, $b.Count); $diff = 0; $first = -1
    for ($i = 0; $i -lt $n; $i++) { if ($a[$i] -ne $b[$i]) { $diff++; if ($first -lt 0) { $first = $i } } }
    if ($a.Count -ne $b.Count) { $diff += [Math]::Abs($a.Count - $b.Count) }
    if ($diff -eq 0) { Write-Host "IDENTICAL: $($Compare[0]) == $($Compare[1]) ($n frames, poses bit-exact)" -ForegroundColor Green }
    else { Write-Host "DIFFER: $diff frame(s) differ, first at frame $first" -ForegroundColor Yellow; Write-Host "  A: $($a[$first])"; Write-Host "  B: $($b[$first])" }
    foreach ($t in $Compare) {
        Write-Host "--- $t"
        $sum = 0; $cnt = 0; $trMin = 1e9; $low = 0
        foreach ($line in (Get-Content (Join-Path $resDir "$t.txt"))) {
            if ($line -match '^#') { continue }
            $p = $line.Split(' ')
            if ($p[1] -ne '2') { continue }
            $v = [int]$p[2]; $sum += $v; $cnt++
            if ($v -lt $trMin) { $trMin = $v }
            if ($v -lt 50) { $low++ }
        }
        if ($cnt -gt 0) { Write-Host ("# tracked/frame(OK): mean={0:F1} min={1} frames<50={2}" -f ($sum / $cnt), $trMin, $low) }
        Get-Content (Join-Path $resDir "$t.txt") | Where-Object { $_ -match '^#' }
    }
    exit 0
}

if (-not $SkipBuild) {
    $env:JAVA_HOME = "C:\Users\Olsc\.gradle\jdks\eclipse_adoptium-21-amd64-windows.2"
    $g = "$env:USERPROFILE\.gradle\wrapper\dists\gradle-9.8.2-bin\8wepis63pzuhrlfaiy04bagwu\gradle-9.8.2\bin\gradle.bat"
    Push-Location $root
    $out = cmd /c "`"$g`" -Dorg.gradle.java.installations.auto-download=false -PmenthaBench -Pandroid.injected.build.abi=arm64-v8a :MenthaAR:externalNativeBuildRelease --console=plain -q 2>&1"
    $code = $LASTEXITCODE
    Pop-Location
    if ($code -ne 0) { $out | Select-Object -Last 60; throw "gradle build failed ($code)" }
    $out | Where-Object { $_ -match 'error:|warning: unused|BUILD' } | Select-Object -First 20
}

$obj = Get-ChildItem (Join-Path $root "MenthaAR\build\intermediates\cxx\RelWithDebInfo\*\obj\arm64-v8a\slam_bench") | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (-not $obj) { throw "slam_bench not found (build with -PmenthaBench)" }
$objDir = $obj.DirectoryName
$strip = "D:\android-sdk\ndk\27.3.13750724\toolchains\llvm\prebuilt\windows-x86_64\bin\llvm-strip.exe"
$tmp = Join-Path $env:TEMP "slam_bench_push"; New-Item -ItemType Directory -Force $tmp | Out-Null
& $strip -o (Join-Path $tmp "slam_bench") $obj.FullName
& $strip -o (Join-Path $tmp "libMenthaAR_Engine.so") (Join-Path $objDir "libMenthaAR_Engine.so")

if (-not $SkipPush) {
    & $Adb shell "mkdir -p $DeviceDir" | Out-Null
    & $Adb push (Join-Path $tmp "slam_bench") "$DeviceDir/slam_bench" | Out-Null
    & $Adb push (Join-Path $tmp "libMenthaAR_Engine.so") "$DeviceDir/libMenthaAR_Engine.so" | Out-Null
    & $Adb shell "chmod 755 $DeviceDir/slam_bench" | Out-Null
    $have = & $Adb shell "ls -l $DeviceDir/frames.raw 2>/dev/null | awk '{print `$5}'"
    if ("$have".Trim() -ne (Get-Item $FramesLocal).Length.ToString()) { & $Adb push $FramesLocal "$DeviceDir/frames.raw" | Out-Null }
}

for ($r = 1; $r -le $Runs; $r++) {
    $line = & $Adb shell "cd $DeviceDir && LD_LIBRARY_PATH=. ./slam_bench frames.raw 1280 720 out.txt $MaxFrames $Sync 1"
    Write-Host "[$Tag run $r] $line"
}
& $Adb pull "$DeviceDir/out.txt" (Join-Path $resDir "$Tag.txt") | Out-Null
Get-Content (Join-Path $resDir "$Tag.txt") | Where-Object { $_ -match '^#' }
