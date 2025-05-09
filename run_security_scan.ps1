# Run Security Scan Manually
# This script runs the security scan manually

# Get the current directory
$scriptPath = Split-Path -Parent $MyInvocation.MyCommand.Path
$rootPath = Split-Path -Parent $scriptPath

# Run the security scan
python $rootPath\scripts\scheduled_security_scan.py

Write-Host "Security scan complete. Press any key to exit."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
