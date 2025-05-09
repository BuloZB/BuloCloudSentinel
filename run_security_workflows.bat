@echo off
echo Bulo.Cloud Sentinel Security Workflows
echo =====================================
echo.
echo This script will trigger the security workflows on GitHub.
echo You need to provide a GitHub personal access token with the 'workflow' scope.
echo.

set /p GITHUB_TOKEN=Enter your GitHub token: 

echo.
echo 1. Run security scan
echo 2. Run dependency update
echo 3. Run security review
echo 4. Run all workflows
echo.

set /p CHOICE=Enter your choice (1-4): 

if "%CHOICE%"=="1" (
    echo.
    echo Running security scan...
    python scripts\trigger_security_scan.py --token %GITHUB_TOKEN%
) else if "%CHOICE%"=="2" (
    echo.
    echo Running dependency update...
    python scripts\trigger_dependency_update.py --token %GITHUB_TOKEN%
) else if "%CHOICE%"=="3" (
    echo.
    echo Running security review...
    python scripts\trigger_security_review.py --token %GITHUB_TOKEN%
) else if "%CHOICE%"=="4" (
    echo.
    echo Running all workflows...
    echo.
    echo Running security scan...
    python scripts\trigger_security_scan.py --token %GITHUB_TOKEN%
    echo.
    echo Running dependency update...
    python scripts\trigger_dependency_update.py --token %GITHUB_TOKEN%
    echo.
    echo Running security review...
    python scripts\trigger_security_review.py --token %GITHUB_TOKEN%
) else (
    echo.
    echo Invalid choice. Please run the script again and enter a number between 1 and 4.
)

echo.
echo Done.
pause
