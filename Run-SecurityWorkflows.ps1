# Run Security Workflows
# This script runs all security workflows for Bulo.Cloud Sentinel

# Show menu
function Show-Menu {
    Clear-Host
    Write-Host "Bulo.Cloud Sentinel Security Workflows" -ForegroundColor Cyan
    Write-Host "=====================================" -ForegroundColor Cyan
    Write-Host
    Write-Host "1. Review Dependabot Alerts" -ForegroundColor Yellow
    Write-Host "2. Run Security Scan" -ForegroundColor Yellow
    Write-Host "3. Update Dependencies" -ForegroundColor Yellow
    Write-Host "4. Run Security Review" -ForegroundColor Yellow
    Write-Host "5. Run All Workflows" -ForegroundColor Yellow
    Write-Host "Q. Quit" -ForegroundColor Yellow
    Write-Host
}

# Get GitHub token
function Get-GitHubToken {
    $token = $env:GITHUB_TOKEN
    
    if (-not $token) {
        $token = Read-Host -Prompt "Enter your GitHub token" -AsSecureString
        $BSTR = [System.Runtime.InteropServices.Marshal]::SecureStringToBSTR($token)
        $token = [System.Runtime.InteropServices.Marshal]::PtrToStringAuto($BSTR)
    }
    
    return $token
}

# Review Dependabot alerts
function Review-DependabotAlerts {
    $token = Get-GitHubToken
    
    if (-not $token) {
        Write-Host "GitHub token not provided" -ForegroundColor Red
        return
    }
    
    Write-Host "Fetching Dependabot alerts..." -ForegroundColor Yellow
    python scripts\fetch_dependabot_alerts.py --token $token
}

# Run security scan
function Run-SecurityScan {
    $token = Get-GitHubToken
    
    if (-not $token) {
        Write-Host "GitHub token not provided" -ForegroundColor Red
        return
    }
    
    $wait = Read-Host -Prompt "Wait for workflow to complete? (y/n)"
    
    if ($wait -eq "y") {
        Write-Host "Running security scan and waiting for completion..." -ForegroundColor Yellow
        & scripts\Run-SecurityScan.ps1 -GitHubToken $token -Wait
    }
    else {
        Write-Host "Running security scan..." -ForegroundColor Yellow
        & scripts\Run-SecurityScan.ps1 -GitHubToken $token
    }
}

# Update dependencies
function Update-Dependencies {
    $token = Get-GitHubToken
    
    if (-not $token) {
        Write-Host "GitHub token not provided" -ForegroundColor Red
        return
    }
    
    $wait = Read-Host -Prompt "Wait for workflow to complete? (y/n)"
    
    if ($wait -eq "y") {
        Write-Host "Running dependency update and waiting for completion..." -ForegroundColor Yellow
        & scripts\Run-DependencyUpdate.ps1 -GitHubToken $token -Wait
    }
    else {
        Write-Host "Running dependency update..." -ForegroundColor Yellow
        & scripts\Run-DependencyUpdate.ps1 -GitHubToken $token
    }
}

# Run security review
function Run-SecurityReview {
    $token = Get-GitHubToken
    
    if (-not $token) {
        Write-Host "GitHub token not provided" -ForegroundColor Red
        return
    }
    
    $wait = Read-Host -Prompt "Wait for workflow to complete? (y/n)"
    
    if ($wait -eq "y") {
        Write-Host "Running security review and waiting for completion..." -ForegroundColor Yellow
        & scripts\Run-SecurityReview.ps1 -GitHubToken $token -Wait
    }
    else {
        Write-Host "Running security review..." -ForegroundColor Yellow
        & scripts\Run-SecurityReview.ps1 -GitHubToken $token
    }
}

# Run all workflows
function Run-AllWorkflows {
    $token = Get-GitHubToken
    
    if (-not $token) {
        Write-Host "GitHub token not provided" -ForegroundColor Red
        return
    }
    
    Write-Host "Running all security workflows..." -ForegroundColor Yellow
    
    Write-Host "1. Fetching Dependabot alerts..." -ForegroundColor Yellow
    python scripts\fetch_dependabot_alerts.py --token $token
    
    Write-Host "2. Running security scan..." -ForegroundColor Yellow
    & scripts\Run-SecurityScan.ps1 -GitHubToken $token
    
    Write-Host "3. Running dependency update..." -ForegroundColor Yellow
    & scripts\Run-DependencyUpdate.ps1 -GitHubToken $token
    
    Write-Host "4. Running security review..." -ForegroundColor Yellow
    & scripts\Run-SecurityReview.ps1 -GitHubToken $token
}

# Main script
do {
    Show-Menu
    $choice = Read-Host -Prompt "Enter your choice"
    
    switch ($choice) {
        "1" {
            Review-DependabotAlerts
            pause
        }
        "2" {
            Run-SecurityScan
            pause
        }
        "3" {
            Update-Dependencies
            pause
        }
        "4" {
            Run-SecurityReview
            pause
        }
        "5" {
            Run-AllWorkflows
            pause
        }
        "q" {
            return
        }
        default {
            Write-Host "Invalid choice. Please try again." -ForegroundColor Red
            pause
        }
    }
} while ($choice -ne "q")
