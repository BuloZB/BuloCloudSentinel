# Run Security Review Manually
# This script generates security review templates manually

# Get the current directory
$scriptPath = Split-Path -Parent $MyInvocation.MyCommand.Path
$rootPath = Split-Path -Parent $scriptPath

# Parse command line arguments
param (
    [switch]$daily,
    [switch]$weekly,
    [switch]$monthly,
    [switch]$quarterly,
    [switch]$all
)

# If no specific review is requested, show help
if (-not ($daily -or $weekly -or $monthly -or $quarterly -or $all)) {
    Write-Host "Usage: .\run_security_review.ps1 [-daily] [-weekly] [-monthly] [-quarterly] [-all]"
    Write-Host "  -daily      Generate daily security review template"
    Write-Host "  -weekly     Generate weekly security review template"
    Write-Host "  -monthly    Generate monthly security review template"
    Write-Host "  -quarterly  Generate quarterly security review template"
    Write-Host "  -all        Generate all security review templates"
    exit
}

# Generate requested review templates
if ($daily -or $all) {
    Write-Host "Generating daily security review template..."
    python $rootPath\scripts\security_review.py --daily
}

if ($weekly -or $all) {
    Write-Host "Generating weekly security review template..."
    python $rootPath\scripts\security_review.py --weekly
}

if ($monthly -or $all) {
    Write-Host "Generating monthly security review template..."
    python $rootPath\scripts\security_review.py --monthly
}

if ($quarterly -or $all) {
    Write-Host "Generating quarterly security review template..."
    python $rootPath\scripts\security_review.py --quarterly
}

Write-Host "Security review templates generated. Press any key to exit."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
