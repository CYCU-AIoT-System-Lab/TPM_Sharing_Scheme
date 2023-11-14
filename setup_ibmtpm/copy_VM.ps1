$baseDir = "E:/Virtual Machines"
$srcVM1 = "Ubuntu 64-bit 18.04 Desktop 0x12 Bak"
$srcVM2 = "Ubuntu 64-bit 18.04 Desktop 0x13 Bak"
$dstVM1 = "Ubuntu 64-bit 18.04 Desktop 0x17"
$dstVM2 = "Ubuntu 64-bit 18.04 Desktop 0x18"

# $cp_command = "cp -R -force"
#$cp_command = "cmd /c xcopy /h /i /c /k /e /r /y"
$cp_command = "robocopy -E /ETA"
$jobs = "$cp_command `"$baseDir/$srcVM1`" `"$baseDir/$dstVM1`"", 
        "$cp_command `"$baseDir/$srcVM1`" `"$baseDir/$dstVM2`""

echo "Copying VMs, check disk usage in task manager."
foreach($job in $jobs) {
    echo "Running: $job"
    Invoke-Expression $job
}
echo "Done."

