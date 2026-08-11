$vidpid = "vid_3652&pid_0b5c"
$classes = @(
    "{e5323777-f976-4f5b-9b55-b94699c46e44}",  # KSCATEGORY_VIDEO_CAMERA
    "{65e8773d-8f56-11d0-a3b9-00a0c9223196}"   # KSCATEGORY_CAPTURE
)
foreach ($c in $classes) {
    Get-ChildItem "HKLM:\SYSTEM\CurrentControlSet\Control\DeviceClasses\$c" -EA SilentlyContinue |
    Where-Object { $_.PSChildName -match $vidpid } |
    ForEach-Object {
        $dp = Join-Path $_.PSPath "#GLOBAL\Device Parameters"
        if (Test-Path $dp) {
            # 数字后缀是 pin 索引,composite 接口上 depth/IR 两个 pin 都要开
            Set-ItemProperty $dp -Name "MetadataBufferSizeInKB0" -Value 5 -Type DWord
            Set-ItemProperty $dp -Name "MetadataBufferSizeInKB1" -Value 5 -Type DWord
            Write-Host "Patched: $($_.PSChildName)"
        }
    }
}