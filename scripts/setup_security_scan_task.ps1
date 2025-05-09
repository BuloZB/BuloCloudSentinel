# Setup Security Scan Task
# This script sets up a scheduled task to run security scans daily

# Get the current directory
$scriptPath = Split-Path -Parent $MyInvocation.MyCommand.Path
$rootPath = Split-Path -Parent $scriptPath

# Define the task name
$taskName = "BuloCloudSentinel_SecurityScan"

# Define the task description
$taskDescription = "Run security scans for Bulo.Cloud Sentinel"

# Define the task action
$pythonPath = "python"
$scriptPath = Join-Path $rootPath "scripts\scheduled_security_scan.py"
$workingDirectory = $rootPath

# Create the action
$action = New-ScheduledTaskAction -Execute $pythonPath -Argument $scriptPath -WorkingDirectory $workingDirectory

# Define the task trigger (daily at midnight)
$trigger = New-ScheduledTaskTrigger -Daily -At 12am

# Define the task settings
$settings = New-ScheduledTaskSettingsSet -StartWhenAvailable -DontStopOnIdleEnd -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries

# Define the task principal (run as current user)
$principal = New-ScheduledTaskPrincipal -UserId ([System.Security.Principal.WindowsIdentity]::GetCurrent().Name) -LogonType S4U -RunLevel Highest

# Check if the task already exists
$existingTask = Get-ScheduledTask -TaskName $taskName -ErrorAction SilentlyContinue

if ($existingTask) {
    # Update the existing task
    Write-Host "Updating existing task: $taskName"
    Set-ScheduledTask -TaskName $taskName -Action $action -Trigger $trigger -Settings $settings -Principal $principal -Description $taskDescription
} else {
    # Create a new task
    Write-Host "Creating new task: $taskName"
    Register-ScheduledTask -TaskName $taskName -Action $action -Trigger $trigger -Settings $settings -Principal $principal -Description $taskDescription
}

Write-Host "Task setup complete. Security scans will run daily at midnight."
Write-Host "You can view and modify the task in Task Scheduler."

# Create a manual trigger script
$manualTriggerScript = @"
# Run Security Scan Manually
# This script runs the security scan manually

# Get the current directory
`$scriptPath = Split-Path -Parent `$MyInvocation.MyCommand.Path
`$rootPath = Split-Path -Parent `$scriptPath

# Run the security scan
python `$rootPath\scripts\scheduled_security_scan.py

Write-Host "Security scan complete. Press any key to exit."
`$null = `$Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
"@

$manualTriggerPath = Join-Path $rootPath "run_security_scan.ps1"
Set-Content -Path $manualTriggerPath -Value $manualTriggerScript

Write-Host "Created manual trigger script: $manualTriggerPath"
Write-Host "You can run this script to trigger a security scan manually."

# Create a desktop shortcut for the manual trigger script
$desktopPath = [Environment]::GetFolderPath("Desktop")
$shortcutPath = Join-Path $desktopPath "Run Security Scan.lnk"

$WshShell = New-Object -ComObject WScript.Shell
$Shortcut = $WshShell.CreateShortcut($shortcutPath)
$Shortcut.TargetPath = "powershell.exe"
$Shortcut.Arguments = "-ExecutionPolicy Bypass -File `"$manualTriggerPath`""
$Shortcut.WorkingDirectory = $rootPath
$Shortcut.Description = "Run security scan for Bulo.Cloud Sentinel"
$Shortcut.IconLocation = "powershell.exe,0"
$Shortcut.Save()

Write-Host "Created desktop shortcut: $shortcutPath"
Write-Host "You can use this shortcut to trigger a security scan manually."

# Pause to allow the user to read the output
Write-Host "Press any key to exit."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
