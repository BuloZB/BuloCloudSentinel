"""
Secure Development Lifecycle (SDL) module for Bulo.Cloud Sentinel.

This module provides functionality for implementing a secure development lifecycle,
including security requirements, threat modeling, secure coding, security testing,
and security review.
"""

from .models import (
    SDLPhase,
    SDLStatus,
    SDLSeverity,
    SDLRequirement,
    SDLThreatModel,
    SDLCodeReview,
    SDLSecurityTest,
    SDLVulnerability,
    SDLProject
)

from .lifecycle import (
    create_project,
    get_project,
    update_project,
    delete_project,
    get_projects,
    advance_phase,
    set_status,
    add_requirement,
    update_requirement,
    delete_requirement,
    add_threat,
    update_threat,
    delete_threat,
    add_code_review,
    update_code_review,
    delete_code_review,
    add_security_test,
    update_security_test,
    delete_security_test,
    add_vulnerability,
    update_vulnerability,
    delete_vulnerability,
    get_project_metrics
)

from .requirements import (
    get_requirements_template,
    validate_requirements,
    import_requirements,
    export_requirements
)

from .threat_modeling import (
    create_threat_model,
    analyze_threats,
    generate_mitigations,
    export_threat_model
)

from .code_review import (
    create_code_review,
    assign_code_review,
    complete_code_review,
    get_code_review_checklist
)

from .testing import (
    create_security_test_plan,
    run_security_tests,
    analyze_test_results,
    generate_test_report
)

from .deployment import (
    create_deployment_checklist,
    validate_deployment,
    generate_deployment_report
)

from .incident_response import (
    create_incident_response_plan,
    simulate_security_incident,
    analyze_incident_response
)

__all__ = [
    # Models
    "SDLPhase",
    "SDLStatus",
    "SDLSeverity",
    "SDLRequirement",
    "SDLThreatModel",
    "SDLCodeReview",
    "SDLSecurityTest",
    "SDLVulnerability",
    "SDLProject",
    
    # Lifecycle management
    "create_project",
    "get_project",
    "update_project",
    "delete_project",
    "get_projects",
    "advance_phase",
    "set_status",
    "add_requirement",
    "update_requirement",
    "delete_requirement",
    "add_threat",
    "update_threat",
    "delete_threat",
    "add_code_review",
    "update_code_review",
    "delete_code_review",
    "add_security_test",
    "update_security_test",
    "delete_security_test",
    "add_vulnerability",
    "update_vulnerability",
    "delete_vulnerability",
    "get_project_metrics",
    
    # Requirements
    "get_requirements_template",
    "validate_requirements",
    "import_requirements",
    "export_requirements",
    
    # Threat modeling
    "create_threat_model",
    "analyze_threats",
    "generate_mitigations",
    "export_threat_model",
    
    # Code review
    "create_code_review",
    "assign_code_review",
    "complete_code_review",
    "get_code_review_checklist",
    
    # Security testing
    "create_security_test_plan",
    "run_security_tests",
    "analyze_test_results",
    "generate_test_report",
    
    # Deployment
    "create_deployment_checklist",
    "validate_deployment",
    "generate_deployment_report",
    
    # Incident response
    "create_incident_response_plan",
    "simulate_security_incident",
    "analyze_incident_response"
]
