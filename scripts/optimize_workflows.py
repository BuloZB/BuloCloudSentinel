#!/usr/bin/env python3
"""
GitHub Workflow Optimizer

This script analyzes and optimizes GitHub workflow files in the repository.
It checks for common issues, suggests improvements, and can automatically fix some problems.

Usage:
  python optimize_workflows.py [--fix] [--check-secrets] [--verbose]

Options:
  --fix            Automatically fix common issues
  --check-secrets  Check for hardcoded secrets (warning: may have false positives)
  --verbose        Show detailed output
"""

import os
import sys
import re
import yaml
import argparse
from pathlib import Path
from typing import Dict, List, Any, Tuple, Optional

# ANSI color codes for terminal output
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BLUE = "\033[94m"
RESET = "\033[0m"

class WorkflowOptimizer:
    def __init__(self, fix: bool = False, check_secrets: bool = False, verbose: bool = False):
        self.fix = fix
        self.check_secrets = check_secrets
        self.verbose = verbose
        self.workflows_dir = Path(".github/workflows")
        self.issues_found = 0
        self.issues_fixed = 0
        
    def log(self, message: str, level: str = "info") -> None:
        """Log a message with appropriate color coding."""
        if level == "info" and not self.verbose:
            return
            
        color = {
            "info": BLUE,
            "warning": YELLOW,
            "error": RED,
            "success": GREEN
        }.get(level, RESET)
        
        print(f"{color}{message}{RESET}")
    
    def run(self) -> int:
        """Run the workflow optimizer."""
        if not self.workflows_dir.exists():
            self.log(f"Workflows directory not found: {self.workflows_dir}", "error")
            return 1
            
        self.log(f"Analyzing workflows in {self.workflows_dir}", "info")
        
        workflow_files = list(self.workflows_dir.glob("*.yml")) + list(self.workflows_dir.glob("*.yaml"))
        if not workflow_files:
            self.log("No workflow files found", "warning")
            return 0
            
        self.log(f"Found {len(workflow_files)} workflow files", "info")
        
        for workflow_file in workflow_files:
            self.analyze_workflow(workflow_file)
            
        self.log(f"\nSummary:", "info")
        self.log(f"- Issues found: {self.issues_found}", "warning" if self.issues_found > 0 else "info")
        self.log(f"- Issues fixed: {self.issues_fixed}", "success" if self.issues_fixed > 0 else "info")
        
        return 0 if self.issues_found == 0 else 1
    
    def analyze_workflow(self, workflow_file: Path) -> None:
        """Analyze a single workflow file for issues."""
        self.log(f"\nAnalyzing {workflow_file}", "info")
        
        try:
            with open(workflow_file, 'r') as f:
                content = f.read()
                
            # Parse YAML
            try:
                workflow = yaml.safe_load(content)
                if not workflow:
                    self.log(f"Empty workflow file: {workflow_file}", "warning")
                    self.issues_found += 1
                    return
            except yaml.YAMLError as e:
                self.log(f"YAML parsing error in {workflow_file}: {e}", "error")
                self.issues_found += 1
                return
                
            # Run checks
            modified_workflow = workflow.copy()
            modified = False
            
            # Check 1: Missing continue-on-error for notification steps
            modified_workflow, notify_modified = self.check_notification_steps(modified_workflow)
            if notify_modified:
                modified = True
                
            # Check 2: Missing conditional checks for file existence
            modified_workflow, file_checks_modified = self.check_file_existence(modified_workflow)
            if file_checks_modified:
                modified = True
                
            # Check 3: Missing caching
            modified_workflow, cache_modified = self.check_caching(modified_workflow)
            if cache_modified:
                modified = True
                
            # Check 4: Check for hardcoded secrets
            if self.check_secrets:
                self.check_for_secrets(content)
                
            # Save changes if requested
            if modified and self.fix:
                with open(workflow_file, 'w') as f:
                    yaml.dump(modified_workflow, f, default_flow_style=False, sort_keys=False)
                self.log(f"Fixed issues in {workflow_file}", "success")
                self.issues_fixed += 1
                
        except Exception as e:
            self.log(f"Error analyzing {workflow_file}: {e}", "error")
            self.issues_found += 1
    
    def check_notification_steps(self, workflow: Dict) -> Tuple[Dict, bool]:
        """Check for notification steps without continue-on-error."""
        modified = False
        
        if 'jobs' not in workflow:
            return workflow, modified
            
        for job_id, job in workflow['jobs'].items():
            if 'steps' not in job:
                continue
                
            for i, step in enumerate(job['steps']):
                # Check for notification steps (Slack, email, etc.)
                is_notification = False
                
                if 'uses' in step and any(x in step['uses'] for x in ['slack', 'notify', 'email', 'notification']):
                    is_notification = True
                elif 'name' in step and any(x in step['name'].lower() for x in ['slack', 'notify', 'email', 'notification']):
                    is_notification = True
                    
                if is_notification:
                    # Check if continue-on-error is missing
                    if 'continue-on-error' not in step:
                        self.log(f"Notification step without continue-on-error: {step.get('name', 'unnamed step')}", "warning")
                        self.issues_found += 1
                        
                        if self.fix:
                            workflow['jobs'][job_id]['steps'][i]['continue-on-error'] = True
                            modified = True
                            
                    # Check if conditional check for webhook is missing
                    uses_webhook = False
                    if 'env' in step and any('WEBHOOK' in key for key in step['env'].keys()):
                        uses_webhook = True
                        
                    if uses_webhook and 'if' not in step:
                        self.log(f"Notification step without webhook existence check: {step.get('name', 'unnamed step')}", "warning")
                        self.issues_found += 1
                        
                        if self.fix:
                            # Add conditional check for webhook secret
                            for env_key in step['env'].keys():
                                if 'WEBHOOK' in env_key:
                                    webhook_var = env_key
                                    webhook_expr = step['env'][webhook_var]
                                    if '${{' in webhook_expr and 'secrets.' in webhook_expr:
                                        secret_name = re.search(r'secrets\.([a-zA-Z0-9_]+)', webhook_expr)
                                        if secret_name:
                                            workflow['jobs'][job_id]['steps'][i]['if'] = f"${{ secrets.{secret_name.group(1)} != '' }}"
                                            modified = True
        
        return workflow, modified
    
    def check_file_existence(self, workflow: Dict) -> Tuple[Dict, bool]:
        """Check for steps that should have file existence checks."""
        modified = False
        
        if 'jobs' not in workflow:
            return workflow, modified
            
        for job_id, job in workflow['jobs'].items():
            if 'steps' not in job:
                continue
                
            for i, step in enumerate(job['steps']):
                # Check for steps that might need file existence checks
                if 'run' in step:
                    command = step['run']
                    
                    # Check for pip install -r
                    pip_install_matches = re.findall(r'pip install -r ([^\s]+)', command)
                    for req_file in pip_install_matches:
                        if 'if [ -f ' not in command and '&&' not in command:
                            self.log(f"pip install without file existence check: {req_file}", "warning")
                            self.issues_found += 1
                            
                            if self.fix:
                                new_command = command.replace(
                                    f"pip install -r {req_file}",
                                    f"if [ -f {req_file} ]; then pip install -r {req_file}; else echo 'Warning: {req_file} not found'; fi"
                                )
                                workflow['jobs'][job_id]['steps'][i]['run'] = new_command
                                modified = True
                                
                    # Check for npm/yarn commands
                    if ('npm ' in command or 'yarn ' in command) and 'test' in command and 'if ' not in command:
                        self.log(f"npm/yarn test without package.json check", "warning")
                        self.issues_found += 1
                        
                        if self.fix and 'cd ' in command:
                            # Extract directory
                            dir_match = re.search(r'cd ([^\s]+)', command)
                            if dir_match:
                                dir_name = dir_match.group(1)
                                lines = command.split('\n')
                                new_lines = []
                                
                                for line in lines:
                                    if 'npm test' in line or 'yarn test' in line:
                                        new_lines.append(f'if [ -f {dir_name}/package.json ]; then')
                                        new_lines.append(f'  {line}')
                                        new_lines.append('else')
                                        new_lines.append('  echo "Warning: package.json not found, skipping tests"')
                                        new_lines.append('fi')
                                    else:
                                        new_lines.append(line)
                                        
                                workflow['jobs'][job_id]['steps'][i]['run'] = '\n'.join(new_lines)
                                modified = True
        
        return workflow, modified
    
    def check_caching(self, workflow: Dict) -> Tuple[Dict, bool]:
        """Check for missing dependency caching."""
        modified = False
        
        if 'jobs' not in workflow:
            return workflow, modified
            
        for job_id, job in workflow['jobs'].items():
            if 'steps' not in job:
                continue
                
            # Check if job uses pip or npm/yarn but doesn't have caching
            uses_pip = False
            uses_npm = False
            has_pip_cache = False
            has_npm_cache = False
            
            for step in job['steps']:
                if 'run' in step:
                    if 'pip install' in step['run']:
                        uses_pip = True
                    if 'npm ' in step['run'] or 'yarn ' in step['run']:
                        uses_npm = True
                        
                if 'uses' in step and 'actions/cache@' in step['uses']:
                    if 'with' in step and 'path' in step['with']:
                        if '~/.cache/pip' in step['with']['path']:
                            has_pip_cache = True
                        if 'node_modules' in step['with']['path']:
                            has_npm_cache = True
            
            if uses_pip and not has_pip_cache:
                self.log(f"Job '{job_id}' uses pip but doesn't cache dependencies", "warning")
                self.issues_found += 1
                
                if self.fix:
                    # Add pip caching step after checkout but before pip install
                    for i, step in enumerate(job['steps']):
                        if 'uses' in step and 'actions/checkout@' in step['uses']:
                            cache_step = {
                                'name': 'Cache pip dependencies',
                                'uses': 'actions/cache@v3',
                                'with': {
                                    'path': '~/.cache/pip',
                                    'key': "${{ runner.os }}-pip-${{ hashFiles('**/requirements*.txt') }}",
                                    'restore-keys': "${{ runner.os }}-pip-"
                                }
                            }
                            workflow['jobs'][job_id]['steps'].insert(i + 1, cache_step)
                            modified = True
                            break
                            
            if uses_npm and not has_npm_cache:
                self.log(f"Job '{job_id}' uses npm/yarn but doesn't cache dependencies", "warning")
                self.issues_found += 1
                
                if self.fix:
                    # Add npm caching step after checkout but before npm install
                    for i, step in enumerate(job['steps']):
                        if 'uses' in step and 'actions/checkout@' in step['uses']:
                            cache_step = {
                                'name': 'Cache npm dependencies',
                                'uses': 'actions/cache@v3',
                                'with': {
                                    'path': '**/node_modules',
                                    'key': "${{ runner.os }}-node-${{ hashFiles('**/package-lock.json') }}",
                                    'restore-keys': "${{ runner.os }}-node-"
                                }
                            }
                            workflow['jobs'][job_id]['steps'].insert(i + 1, cache_step)
                            modified = True
                            break
        
        return workflow, modified
    
    def check_for_secrets(self, content: str) -> None:
        """Check for potentially hardcoded secrets."""
        # Simple patterns that might indicate hardcoded secrets
        secret_patterns = [
            r'password\s*[:=]\s*["\'](?!\\$\\{)[^"\']+["\']',
            r'token\s*[:=]\s*["\'](?!\\$\\{)[^"\']+["\']',
            r'secret\s*[:=]\s*["\'](?!\\$\\{)[^"\']+["\']',
            r'key\s*[:=]\s*["\'](?!\\$\\{)[^"\']+["\']',
            r'api[-_]?key\s*[:=]\s*["\'](?!\\$\\{)[^"\']+["\']',
        ]
        
        for pattern in secret_patterns:
            matches = re.finditer(pattern, content, re.IGNORECASE)
            for match in matches:
                self.log(f"Potential hardcoded secret found: {match.group(0)}", "warning")
                self.issues_found += 1

def main():
    parser = argparse.ArgumentParser(description="GitHub Workflow Optimizer")
    parser.add_argument("--fix", action="store_true", help="Automatically fix common issues")
    parser.add_argument("--check-secrets", action="store_true", help="Check for hardcoded secrets")
    parser.add_argument("--verbose", action="store_true", help="Show detailed output")
    
    args = parser.parse_args()
    
    optimizer = WorkflowOptimizer(
        fix=args.fix,
        check_secrets=args.check_secrets,
        verbose=args.verbose
    )
    
    return optimizer.run()

if __name__ == "__main__":
    sys.exit(main())