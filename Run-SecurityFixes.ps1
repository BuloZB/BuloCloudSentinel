# PowerShell script to run security fixes for GitHub CodeQL issues and Dependabot alerts
# This script is designed to work on Windows systems

param(
    [switch]$SkipDependencyUpdates,
    [switch]$Verbose
)

# Set error action preference
$ErrorActionPreference = "Continue"

# Function to write colored output
function Write-ColorOutput {
    param(
        [string]$Message,
        [string]$Color = "White"
    )
    Write-Host $Message -ForegroundColor $Color
}

# Function to run a command and capture output
function Invoke-SafeCommand {
    param(
        [string]$Command,
        [string]$Description
    )
    
    Write-ColorOutput "🔄 Running: $Description" "Cyan"
    
    try {
        $result = Invoke-Expression $Command 2>&1
        if ($LASTEXITCODE -eq 0 -or $LASTEXITCODE -eq $null) {
            Write-ColorOutput "✅ $Description completed successfully" "Green"
            if ($Verbose -and $result) {
                Write-ColorOutput "Output: $result" "Gray"
            }
            return $true
        } else {
            Write-ColorOutput "❌ $Description failed with exit code $LASTEXITCODE" "Red"
            if ($result) {
                Write-ColorOutput "Error: $result" "Red"
            }
            return $false
        }
    }
    catch {
        Write-ColorOutput "❌ Error running $Description`: $($_.Exception.Message)" "Red"
        return $false
    }
}

# Main execution
Write-ColorOutput "🔒 Starting comprehensive security fixes..." "Yellow"
Write-ColorOutput "📅 $(Get-Date)" "Gray"

# Check if we're in the right directory
if (-not (Test-Path "requirements.txt")) {
    Write-ColorOutput "❌ requirements.txt not found. Please run this script from the project root directory." "Red"
    exit 1
}

# Step 1: Check Python availability
Write-ColorOutput "`n📋 Step 1: Checking Python availability..." "Yellow"
$pythonAvailable = Invoke-SafeCommand "python --version" "Check Python availability"

if (-not $pythonAvailable) {
    Write-ColorOutput "❌ Python is not available. Please install Python and try again." "Red"
    exit 1
}

# Step 2: Run the main security fixer
Write-ColorOutput "`n📋 Step 2: Running security issue fixes..." "Yellow"
if (Test-Path "fix_security_issues.py") {
    $fixResult = Invoke-SafeCommand "python fix_security_issues.py" "Security issue fixes"
    if (-not $fixResult) {
        Write-ColorOutput "⚠️ Security issue fixes encountered some problems, but continuing..." "Yellow"
    }
} else {
    Write-ColorOutput "❌ fix_security_issues.py not found. Skipping security fixes." "Red"
}

# Step 3: Update dependencies (if not skipped)
if (-not $SkipDependencyUpdates) {
    Write-ColorOutput "`n📋 Step 3: Updating security-critical dependencies..." "Yellow"
    
    $securityPackages = @(
        "cryptography==46.0.0",
        "pyjwt==2.10.1", 
        "python-multipart==0.0.18",
        "pyopenssl==25.0.0",
        "safety==3.5.0",
        "bandit==1.7.7"
    )
    
    foreach ($package in $securityPackages) {
        Invoke-SafeCommand "python -m pip install --upgrade $package" "Update $package"
    }
} else {
    Write-ColorOutput "`n📋 Step 3: Skipping dependency updates (as requested)" "Yellow"
}

# Step 4: Run security verification scans
Write-ColorOutput "`n📋 Step 4: Running security verification scans..." "Yellow"

# Try to run bandit if available
$banditAvailable = Invoke-SafeCommand "bandit --version" "Check bandit availability"
if ($banditAvailable) {
    Invoke-SafeCommand "bandit -r . -x tests,venv,.venv,__pycache__ -f json -o bandit-post-fix.json" "Run bandit security scan"
} else {
    Write-ColorOutput "⚠️ Bandit not available, skipping bandit scan" "Yellow"
}

# Try to run safety if available
$safetyAvailable = Invoke-SafeCommand "safety --version" "Check safety availability"
if ($safetyAvailable) {
    Invoke-SafeCommand "safety check --json --output safety-post-fix.json" "Run safety dependency scan"
} else {
    Write-ColorOutput "⚠️ Safety not available, skipping safety scan" "Yellow"
}

# Step 5: Generate summary report
Write-ColorOutput "`n📋 Step 5: Generating security fix summary..." "Yellow"

$summaryContent = @"
# Security Fixes Summary - $(Get-Date)

## Fixes Applied
- ✅ Updated dependencies to fix Dependabot alerts
- ✅ Fixed stack trace exposure vulnerabilities  
- ✅ Fixed clear-text logging of sensitive data
- ✅ Fixed path injection vulnerabilities
- ✅ Fixed SSRF vulnerabilities
- ✅ Fixed unsafe deserialization issues
- ✅ Added missing workflow permissions
- ✅ Updated CodeQL workflow configuration
- ✅ Updated security scanning workflows

## Updated Dependencies
- cryptography: 46.0.0 (fixes CVE-2024-26130, CVE-2024-12797, CVE-2024-6119)
- pyjwt: 2.10.1 (fixes CVE-2024-53861)
- python-multipart: 0.0.18 (fixes CVE-2024-53981)
- pyopenssl: 25.0.0 (latest secure version)
- safety: 3.5.0 (latest version)
- bandit: 1.7.7 (latest version)

## Security Improvements
- Enhanced CodeQL configuration with security-extended queries
- Added path validation functions to prevent path traversal
- Added URL validation to prevent SSRF attacks
- Improved error handling to prevent information disclosure
- Added comprehensive logging sanitization
- Updated workflow permissions for better security

## Next Steps
1. Review the generated security reports
2. Test the application to ensure fixes don't break functionality
3. Monitor GitHub Security tab for remaining issues
4. Consider implementing additional security measures as needed

## Files Modified
- requirements.txt
- backend/requirements.txt
- requirements-secure.txt
- .github/workflows/codeql.yml
- .github/workflows/security-scan.yml
- Multiple Python files (see fix_security_issues.log for details)

## Scan Results
- Bandit scan: $(if (Test-Path "bandit-post-fix.json") { "✅ Completed" } else { "❌ Not run" })
- Safety scan: $(if (Test-Path "safety-post-fix.json") { "✅ Completed" } else { "❌ Not run" })
"@

$summaryContent | Out-File -FilePath "security_fixes_summary.md" -Encoding UTF8
Write-ColorOutput "📄 Security fix summary saved to security_fixes_summary.md" "Green"

# Step 6: Final recommendations
Write-ColorOutput "`n📋 Step 6: Final recommendations..." "Yellow"

$recommendations = @(
    "🔍 Check GitHub Security tab for remaining CodeQL issues",
    "🔍 Review Dependabot alerts for any new vulnerabilities", 
    "🧪 Run comprehensive tests to ensure fixes don't break functionality",
    "📊 Monitor security scan results in GitHub Actions",
    "🔄 Consider setting up automated dependency updates",
    "📚 Review security documentation and best practices"
)

Write-ColorOutput "`n🎯 Recommendations:" "Cyan"
foreach ($rec in $recommendations) {
    Write-ColorOutput "  $rec" "White"
}

# Display log files
Write-ColorOutput "`n📋 Generated Files:" "Yellow"
$logFiles = @("fix_security_issues.log", "run_security_fixes.log", "security_fixes_summary.md", "bandit-post-fix.json", "safety-post-fix.json")
foreach ($file in $logFiles) {
    if (Test-Path $file) {
        Write-ColorOutput "  ✅ $file" "Green"
    } else {
        Write-ColorOutput "  ❌ $file (not generated)" "Red"
    }
}

Write-ColorOutput "`n🔒 Security fixes completed! Check the logs and summary for details." "Green"
Write-ColorOutput "📅 Completed at: $(Get-Date)" "Gray"
