$baseDir = "E:/Virtual Machines"
$srcVM1 = "Ubuntu 64-bit 18.04 Desktop 0x12 Bak"
$dstVM1 = "Ubuntu 64-bit 18.04 Desktop 0x20"
$dstVM2 = "Ubuntu 64-bit 18.04 Desktop 0x21"
$dstVM3 = "Ubuntu 64-bit 18.04 Desktop 0x22"
$dstVM4 = "Ubuntu 64-bit 18.04 Desktop 0x23"

# $cp_command = "cp -R -force"
#$cp_command = "cmd /c xcopy /h /i /c /k /e /r /y"
$cp_command = "robocopy -E /ETA"
$jobs = "$cp_command `"$baseDir/$srcVM1`" `"$baseDir/$dstVM1`"", 
        "$cp_command `"$baseDir/$srcVM1`" `"$baseDir/$dstVM2`"",
		"$cp_command `"$baseDir/$srcVM1`" `"$baseDir/$dstVM3`"",
		"$cp_command `"$baseDir/$srcVM1`" `"$baseDir/$dstVM4`""

echo "Copying VMs, check disk usage in task manager."
foreach($job in $jobs) {
    echo "Running: $job"
    Invoke-Expression $job
}
echo "Done."

