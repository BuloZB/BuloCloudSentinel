# Setup Security Review Task
# This script sets up a scheduled task to generate security review templates

# Get the current directory
$scriptPath = Split-Path -Parent $MyInvocation.MyCommand.Path
$rootPath = Split-Path -Parent $scriptPath

# Define the task names
$dailyTaskName = "BuloCloudSentinel_DailySecurityReview"
$weeklyTaskName = "BuloCloudSentinel_WeeklySecurityReview"
$monthlyTaskName = "BuloCloudSentinel_MonthlySecurityReview"
$quarterlyTaskName = "BuloCloudSentinel_QuarterlySecurityReview"

# Define the task descriptions
$dailyTaskDescription = "Generate daily security review template for Bulo.Cloud Sentinel"
$weeklyTaskDescription = "Generate weekly security review template for Bulo.Cloud Sentinel"
$monthlyTaskDescription = "Generate monthly security review template for Bulo.Cloud Sentinel"
$quarterlyTaskDescription = "Generate quarterly security review template for Bulo.Cloud Sentinel"

# Define the task actions
$pythonPath = "python"
$scriptPath = Join-Path $rootPath "scripts\security_review.py"
$workingDirectory = $rootPath

# Create the actions
$dailyAction = New-ScheduledTaskAction -Execute $pythonPath -Argument "$scriptPath --daily" -WorkingDirectory $workingDirectory
$weeklyAction = New-ScheduledTaskAction -Execute $pythonPath -Argument "$scriptPath --weekly" -WorkingDirectory $workingDirectory
$monthlyAction = New-ScheduledTaskAction -Execute $pythonPath -Argument "$scriptPath --monthly" -WorkingDirectory $workingDirectory
$quarterlyAction = New-ScheduledTaskAction -Execute $pythonPath -Argument "$scriptPath --quarterly" -WorkingDirectory $workingDirectory

# Define the task triggers
$dailyTrigger = New-ScheduledTaskTrigger -Daily -At 8am
$weeklyTrigger = New-ScheduledTaskTrigger -Weekly -DaysOfWeek Monday -At 8am
$monthlyTrigger = New-ScheduledTaskTrigger -Monthly -DaysOfMonth 1 -At 8am

# Define quarterly trigger (first day of Jan, Apr, Jul, Oct)
$quarterlyTrigger1 = New-ScheduledTaskTrigger -Monthly -DaysOfMonth 1 -At 8am -Month January
$quarterlyTrigger2 = New-ScheduledTaskTrigger -Monthly -DaysOfMonth 1 -At 8am -Month April
$quarterlyTrigger3 = New-ScheduledTaskTrigger -Monthly -DaysOfMonth 1 -At 8am -Month July
$quarterlyTrigger4 = New-ScheduledTaskTrigger -Monthly -DaysOfMonth 1 -At 8am -Month October

# Define the task settings
$settings = New-ScheduledTaskSettingsSet -StartWhenAvailable -DontStopOnIdleEnd -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries

# Define the task principal (run as current user)
$principal = New-ScheduledTaskPrincipal -UserId ([System.Security.Principal.WindowsIdentity]::GetCurrent().Name) -LogonType S4U -RunLevel Highest

# Setup daily task
try {
    # Check if the task already exists
    $existingTask = Get-ScheduledTask -TaskName $dailyTaskName -ErrorAction SilentlyContinue

    if ($existingTask) {
        # Update the existing task
        Write-Host "Updating existing task: $dailyTaskName"
        Set-ScheduledTask -TaskName $dailyTaskName -Action $dailyAction -Trigger $dailyTrigger -Settings $settings -Principal $principal -Description $dailyTaskDescription
    } else {
        # Create a new task
        Write-Host "Creating new task: $dailyTaskName"
        Register-ScheduledTask -TaskName $dailyTaskName -Action $dailyAction -Trigger $dailyTrigger -Settings $settings -Principal $principal -Description $dailyTaskDescription
    }
} catch {
    Write-Host "Error setting up daily task: $_"
}

# Setup weekly task
try {
    # Check if the task already exists
    $existingTask = Get-ScheduledTask -TaskName $weeklyTaskName -ErrorAction SilentlyContinue

    if ($existingTask) {
        # Update the existing task
        Write-Host "Updating existing task: $weeklyTaskName"
        Set-ScheduledTask -TaskName $weeklyTaskName -Action $weeklyAction -Trigger $weeklyTrigger -Settings $settings -Principal $principal -Description $weeklyTaskDescription
    } else {
        # Create a new task
        Write-Host "Creating new task: $weeklyTaskName"
        Register-ScheduledTask -TaskName $weeklyTaskName -Action $weeklyAction -Trigger $weeklyTrigger -Settings $settings -Principal $principal -Description $weeklyTaskDescription
    }
} catch {
    Write-Host "Error setting up weekly task: $_"
}

# Setup monthly task
try {
    # Check if the task already exists
    $existingTask = Get-ScheduledTask -TaskName $monthlyTaskName -ErrorAction SilentlyContinue

    if ($existingTask) {
        # Update the existing task
        Write-Host "Updating existing task: $monthlyTaskName"
        Set-ScheduledTask -TaskName $monthlyTaskName -Action $monthlyAction -Trigger $monthlyTrigger -Settings $settings -Principal $principal -Description $monthlyTaskDescription
    } else {
        # Create a new task
        Write-Host "Creating new task: $monthlyTaskName"
        Register-ScheduledTask -TaskName $monthlyTaskName -Action $monthlyAction -Trigger $monthlyTrigger -Settings $settings -Principal $principal -Description $monthlyTaskDescription
    }
} catch {
    Write-Host "Error setting up monthly task: $_"
}

# Setup quarterly task
try {
    # Check if the task already exists
    $existingTask = Get-ScheduledTask -TaskName $quarterlyTaskName -ErrorAction SilentlyContinue

    if ($existingTask) {
        # Update the existing task
        Write-Host "Updating existing task: $quarterlyTaskName"
        Set-ScheduledTask -TaskName $quarterlyTaskName -Action $quarterlyAction -Trigger @($quarterlyTrigger1, $quarterlyTrigger2, $quarterlyTrigger3, $quarterlyTrigger4) -Settings $settings -Principal $principal -Description $quarterlyTaskDescription
    } else {
        # Create a new task
        Write-Host "Creating new task: $quarterlyTaskName"
        Register-ScheduledTask -TaskName $quarterlyTaskName -Action $quarterlyAction -Trigger @($quarterlyTrigger1, $quarterlyTrigger2, $quarterlyTrigger3, $quarterlyTrigger4) -Settings $settings -Principal $principal -Description $quarterlyTaskDescription
    }
} catch {
    Write-Host "Error setting up quarterly task: $_"
}

Write-Host "Task setup complete. Security reviews will be generated according to the schedule."
Write-Host "You can view and modify the tasks in Task Scheduler."

# Create a manual trigger script
$manualTriggerScript = @"
# Run Security Review Manually
# This script generates security review templates manually

# Get the current directory
`$scriptPath = Split-Path -Parent `$MyInvocation.MyCommand.Path
`$rootPath = Split-Path -Parent `$scriptPath

# Parse command line arguments
param(
    [switch]`$daily,
    [switch]`$weekly,
    [switch]`$monthly,
    [switch]`$quarterly,
    [switch]`$all
)

# If no specific review is requested, show help
if (-not (`$daily -or `$weekly -or `$monthly -or `$quarterly -or `$all)) {
    Write-Host "Usage: .\run_security_review.ps1 [-daily] [-weekly] [-monthly] [-quarterly] [-all]"
    Write-Host "  -daily      Generate daily security review template"
    Write-Host "  -weekly     Generate weekly security review template"
    Write-Host "  -monthly    Generate monthly security review template"
    Write-Host "  -quarterly  Generate quarterly security review template"
    Write-Host "  -all        Generate all security review templates"
    exit
}

# Generate requested review templates
if (`$daily -or `$all) {
    Write-Host "Generating daily security review template..."
    python `$rootPath\scripts\security_review.py --daily
}

if (`$weekly -or `$all) {
    Write-Host "Generating weekly security review template..."
    python `$rootPath\scripts\security_review.py --weekly
}

if (`$monthly -or `$all) {
    Write-Host "Generating monthly security review template..."
    python `$rootPath\scripts\security_review.py --monthly
}

if (`$quarterly -or `$all) {
    Write-Host "Generating quarterly security review template..."
    python `$rootPath\scripts\security_review.py --quarterly
}

Write-Host "Security review templates generated. Press any key to exit."
`$null = `$Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
"@

$manualTriggerPath = Join-Path $rootPath "run_security_review.ps1"
Set-Content -Path $manualTriggerPath -Value $manualTriggerScript

Write-Host "Created manual trigger script: $manualTriggerPath"
Write-Host "You can run this script to generate security review templates manually."

# Create a desktop shortcut for the manual trigger script
$desktopPath = [Environment]::GetFolderPath("Desktop")
$shortcutPath = Join-Path $desktopPath "Generate Security Review.lnk"

$WshShell = New-Object -ComObject WScript.Shell
$Shortcut = $WshShell.CreateShortcut($shortcutPath)
$Shortcut.TargetPath = "powershell.exe"
$Shortcut.Arguments = "-ExecutionPolicy Bypass -File `"$manualTriggerPath`" -all"
$Shortcut.WorkingDirectory = $rootPath
$Shortcut.Description = "Generate security review templates for Bulo.Cloud Sentinel"
$Shortcut.IconLocation = "powershell.exe,0"
$Shortcut.Save()

Write-Host "Created desktop shortcut: $shortcutPath"
Write-Host "You can use this shortcut to generate security review templates manually."

# Pause to allow the user to read the output
Write-Host "Press any key to exit."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
