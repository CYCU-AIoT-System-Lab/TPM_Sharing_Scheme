$baseDir = "E:\Virtual Machines"
$srcVM1 = "Ubuntu 64-bit 18.04 Desktop 0x12 Bak"
$srcVM2 = "Ubuntu 64-bit 18.04 Desktop 0x13 Bak"
$dstVM1 = "Ubuntu 64-bit 18.04 Desktop 0x15"
$dstVM2 = "Ubuntu 64-bit 18.04 Desktop 0x16"

$cp_command = "cp -R -force"
$jobs = "$cp_command '$baseDir\$srcVM1' '$baseDir\$dstVM1'", 
        "$cp_command '$baseDir\$srcVM1' '$baseDir\$dstVM2'",
        "echo Done!"

echo "Copying VMs in background, check progress with 'Get-Job', 'Receive-Job', or check disk usage in task manager."
foreach($job in $jobs) {
    echo "Starting job: $job"
    Invoke-Expression $job
}
