# Run Security Review
# This script runs a security review using the security-review.yml workflow

param (
    [Parameter(Mandatory=$true)]
    [string]$GitHubToken,
    
    [Parameter(Mandatory=$false)]
    [switch]$Wait,
    
    [Parameter(Mandatory=$false)]
    [int]$TimeoutSeconds = 600
)

# Define the repository information
$repoOwner = "BuloZB"
$repoName = "BuloCloudSentinel"
$workflowId = "security-review.yml"

# Function to trigger the workflow
function Trigger-Workflow {
    param (
        [string]$Token
    )
    
    Write-Host "Triggering workflow $workflowId..."
    
    $url = "https://api.github.com/repos/$repoOwner/$repoName/actions/workflows/$workflowId/dispatches"
    
    $headers = @{
        "Accept" = "application/vnd.github.v3+json"
        "Authorization" = "token $Token"
        "Content-Type" = "application/json"
    }
    
    $body = @{
        "ref" = "main"
    } | ConvertTo-Json
    
    try {
        $response = Invoke-RestMethod -Uri $url -Headers $headers -Method Post -Body $body
        Write-Host "Workflow triggered successfully" -ForegroundColor Green
        return $true
    }
    catch {
        Write-Host "Failed to trigger workflow: $_" -ForegroundColor Red
        return $false
    }
}

# Function to check the workflow status
function Get-WorkflowStatus {
    param (
        [string]$Token
    )
    
    Write-Host "Checking workflow status..."
    
    $url = "https://api.github.com/repos/$repoOwner/$repoName/actions/workflows/$workflowId/runs"
    
    $headers = @{
        "Accept" = "application/vnd.github.v3+json"
        "Authorization" = "token $Token"
    }
    
    try {
        $response = Invoke-RestMethod -Uri $url -Headers $headers -Method Get
        
        if ($response.workflow_runs.Count -gt 0) {
            $latestRun = $response.workflow_runs[0]
            
            return @{
                "id" = $latestRun.id
                "status" = $latestRun.status
                "conclusion" = $latestRun.conclusion
                "created_at" = $latestRun.created_at
                "updated_at" = $latestRun.updated_at
                "html_url" = $latestRun.html_url
            }
        }
        else {
            Write-Host "No workflow runs found" -ForegroundColor Red
            return $null
        }
    }
    catch {
        Write-Host "Failed to check workflow status: $_" -ForegroundColor Red
        return $null
    }
}

# Function to wait for the workflow to complete
function Wait-WorkflowCompletion {
    param (
        [string]$Token,
        [int]$Timeout
    )
    
    Write-Host "Waiting for workflow to complete (timeout: $Timeout seconds)..."
    
    $startTime = Get-Date
    
    while ((Get-Date) - $startTime).TotalSeconds -lt $Timeout) {
        $status = Get-WorkflowStatus -Token $Token
        
        if ($null -eq $status) {
            Write-Host "Failed to get workflow status" -ForegroundColor Red
            return $null
        }
        
        if ($status.status -eq "completed") {
            Write-Host "Workflow completed with conclusion: $($status.conclusion)" -ForegroundColor $(if ($status.conclusion -eq "success") { "Green" } else { "Red" })
            return $status
        }
        
        Write-Host "Workflow status: $($status.status)" -ForegroundColor Yellow
        
        # Wait for 30 seconds before checking again
        Write-Host "Waiting 30 seconds before checking again..."
        Start-Sleep -Seconds 30
    }
    
    Write-Host "Workflow did not complete within $Timeout seconds" -ForegroundColor Red
    return $status
}

# Function to download workflow artifacts
function Get-WorkflowArtifacts {
    param (
        [string]$Token,
        [int]$RunId
    )
    
    Write-Host "Getting workflow artifacts..."
    
    $url = "https://api.github.com/repos/$repoOwner/$repoName/actions/runs/$RunId/artifacts"
    
    $headers = @{
        "Accept" = "application/vnd.github.v3+json"
        "Authorization" = "token $Token"
    }
    
    try {
        $response = Invoke-RestMethod -Uri $url -Headers $headers -Method Get
        
        if ($response.artifacts.Count -gt 0) {
            Write-Host "Found $($response.artifacts.Count) artifacts" -ForegroundColor Green
            
            foreach ($artifact in $response.artifacts) {
                Write-Host "Downloading artifact: $($artifact.name)" -ForegroundColor Yellow
                
                $downloadUrl = $artifact.archive_download_url
                $outputPath = Join-Path -Path (Get-Location) -ChildPath "$($artifact.name).zip"
                
                $downloadHeaders = @{
                    "Authorization" = "token $Token"
                }
                
                Invoke-WebRequest -Uri $downloadUrl -Headers $downloadHeaders -OutFile $outputPath
                
                Write-Host "Downloaded artifact to: $outputPath" -ForegroundColor Green
                
                # Extract the artifact
                $extractPath = Join-Path -Path (Get-Location) -ChildPath $artifact.name
                Expand-Archive -Path $outputPath -DestinationPath $extractPath -Force
                
                Write-Host "Extracted artifact to: $extractPath" -ForegroundColor Green
            }
            
            return $true
        }
        else {
            Write-Host "No artifacts found" -ForegroundColor Yellow
            return $false
        }
    }
    catch {
        Write-Host "Failed to get workflow artifacts: $_" -ForegroundColor Red
        return $false
    }
}

# Main script

# Trigger the workflow
if (-not (Trigger-Workflow -Token $GitHubToken)) {
    exit 1
}

# Wait for the workflow to complete if requested
if ($Wait) {
    $status = Wait-WorkflowCompletion -Token $GitHubToken -Timeout $TimeoutSeconds
    
    if ($null -eq $status) {
        exit 1
    }
    
    if ($status.conclusion -ne "success") {
        Write-Host "Workflow failed with conclusion: $($status.conclusion)" -ForegroundColor Red
        Write-Host "See details at: $($status.html_url)" -ForegroundColor Yellow
        exit 1
    }
    
    # Download artifacts
    Get-WorkflowArtifacts -Token $GitHubToken -RunId $status.id
}
else {
    # Just check the status once
    $status = Get-WorkflowStatus -Token $GitHubToken
    
    if ($null -ne $status) {
        Write-Host "Workflow status: $($status.status)" -ForegroundColor Yellow
        Write-Host "See details at: $($status.html_url)" -ForegroundColor Yellow
    }
}

Write-Host "Done." -ForegroundColor Green
exit 0
