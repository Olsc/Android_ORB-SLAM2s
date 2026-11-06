# 加载地图场景回放：构建当前工作树的引擎 + slam_bench，推到设备 /data/local/tmp/slam/<Tag>/，
# 依次运行：建图并保存 map（0-400 帧）→ 预加载地图回放 revisit → 先扫 100 帧再加载地图回放 revisit / jump。
# 用法：powershell -File docs\bench\run_loaded_map_bench.ps1 -Tag head [-SkipBuild] [-SkipPush]
param(
    [Parameter(Mandatory=$true)][string]$Tag,
    [switch]$SkipBuild,
    [switch]$SkipPush,
    [string]$Adb = "E:\工具箱\env\platform-tools\adb.exe",
    [string]$OutDir = "D:\slam_bench_data"
)
$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$strip = "D:\android-sdk\ndk\27.3.13750724\toolchains\llvm\prebuilt\windows-x86_64\bin\llvm-strip.exe"
if (-not $SkipBuild) {
    $env:JAVA_HOME = "C:\Users\Olsc\.gradle\jdks\eclipse_adoptium-21-amd64-windows.2"
    $g = "$env:USERPROFILE\.gradle\wrapper\dists\gradle-9.8.2-bin\8wepis63pzuhrlfaiy04bagwu\gradle-9.8.2\bin\gradle.bat"
    Push-Location $root
    $out = cmd /c "`"$g`" -Dorg.gradle.java.installations.auto-download=false -PmenthaBench -Pandroid.injected.build.abi=arm64-v8a :MenthaAR:externalNativeBuildRelease --console=plain -q 2>&1"
    $code = $LASTEXITCODE
    Pop-Location
    if ($code -ne 0) { $out | Select-Object -Last 40; throw "gradle build failed ($code)" }
}
$obj = Get-ChildItem (Join-Path $root "MenthaAR\build\intermediates\cxx\RelWithDebInfo\*\obj\arm64-v8a\slam_bench") | Sort-Object LastWriteTime -Descending | Select-Object -First 1
$o = $obj.DirectoryName
if (-not $SkipPush) {
    & $strip -o "$env:TEMP\sb_$Tag" (Join-Path $o "slam_bench")
    & $strip -o "$env:TEMP\so_$Tag.so" (Join-Path $o "libMenthaAR_Engine.so")
    & $Adb shell "mkdir -p /data/local/tmp/slam/$Tag" | Out-Null
    & $Adb push "$env:TEMP\sb_$Tag" "/data/local/tmp/slam/$Tag/slam_bench" | Out-Null
    & $Adb push "$env:TEMP\so_$Tag.so" "/data/local/tmp/slam/$Tag/libMenthaAR_Engine.so" | Out-Null
    & $Adb shell "chmod 755 /data/local/tmp/slam/$Tag/slam_bench" | Out-Null
}
function RunScenario($name, $frames, $envs) {
    $cmd = "cd /data/local/tmp/slam && KMP_BLOCKTIME=60 LD_LIBRARY_PATH=./$Tag $envs ./$Tag/slam_bench $frames 1280 720 $Tag/$name.txt 2000 1 1"
    $line = & $Adb shell $cmd
    $local = Join-Path $OutDir ("{0}_{1}.txt" -f $Tag, $name)
    & $Adb pull "/data/local/tmp/slam/$Tag/$name.txt" $local | Out-Null
    $prev = -1; $runs = @()
    foreach ($l in Get-Content $local) {
        if ($l -match '^#') { continue }
        $p = $l.Split(' ')
        if ([int]$p[1] -ne $prev) { $runs += "$($p[0]):s$($p[1])"; $prev = [int]$p[1] }
    }
    Write-Host ("[{0}/{1}] {2}" -f $Tag, $name, $line)
    Write-Host ("    transitions: " + ($runs -join ' '))
}
$map = "/data/local/tmp/slam/$Tag/map400.bin"
RunScenario "mk"         "frames.raw"      "SLAM_SAVE_MAP=$map"
RunScenario "rv_ld"      "frames_rv.raw"   "SLAM_LOAD_MAP=$map"
RunScenario "rv_ld100"   "frames_rv.raw"   "SLAM_LOAD_MAP=$map SLAM_LOAD_MAP_AT=100"
RunScenario "jump_ld100" "frames_jump.raw" "SLAM_LOAD_MAP=$map SLAM_LOAD_MAP_AT=100"
