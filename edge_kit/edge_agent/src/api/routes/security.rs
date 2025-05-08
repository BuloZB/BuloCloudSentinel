use std::sync::Arc;

use axum::{
    extract::{Path, State},
    Json,
};
use serde::{Deserialize, Serialize};

use crate::api::state::AppState;

/// Certificate list response
#[derive(Debug, Serialize)]
pub struct CertificateListResponse {
    /// Certificates
    pub certificates: Vec<CertificateInfo>,
}

/// Certificate information
#[derive(Debug, Serialize)]
pub struct CertificateInfo {
    /// Certificate ID
    pub id: String,
    
    /// Subject
    pub subject: String,
    
    /// Issuer
    pub issuer: String,
    
    /// Not before
    pub not_before: String,
    
    /// Not after
    pub not_after: String,
    
    /// Serial number
    pub serial_number: String,
    
    /// Fingerprint
    pub fingerprint: String,
}

/// Certificate details response
#[derive(Debug, Serialize)]
pub struct CertificateDetailsResponse {
    /// Certificate
    pub certificate: CertificateDetails,
}

/// Certificate details
#[derive(Debug, Serialize)]
pub struct CertificateDetails {
    /// Certificate ID
    pub id: String,
    
    /// Subject
    pub subject: String,
    
    /// Issuer
    pub issuer: String,
    
    /// Not before
    pub not_before: String,
    
    /// Not after
    pub not_after: String,
    
    /// Serial number
    pub serial_number: String,
    
    /// Fingerprint
    pub fingerprint: String,
    
    /// Subject alternative names
    pub subject_alternative_names: Vec<String>,
    
    /// Key usage
    pub key_usage: Vec<String>,
    
    /// Extended key usage
    pub extended_key_usage: Vec<String>,
    
    /// Public key algorithm
    pub public_key_algorithm: String,
    
    /// Signature algorithm
    pub signature_algorithm: String,
    
    /// Version
    pub version: u32,
}

/// Certificate rotation request
#[derive(Debug, Deserialize)]
pub struct CertificateRotationRequest {
    /// Force rotation
    #[serde(default)]
    pub force: bool,
}

/// Certificate rotation response
#[derive(Debug, Serialize)]
pub struct CertificateRotationResponse {
    /// Status
    pub status: String,
    
    /// Message
    pub message: String,
    
    /// New certificates
    pub new_certificates: Vec<CertificateInfo>,
}

/// List certificates handler
pub async fn list_certificates(
    State(state): State<Arc<AppState>>,
) -> Json<CertificateListResponse> {
    let certificates = state.security_manager.list_certificates().await;
    
    Json(CertificateListResponse { certificates })
}

/// Get certificate handler
pub async fn get_certificate(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Json<CertificateDetailsResponse> {
    let certificate = state.security_manager.get_certificate(&id).await.unwrap_or_else(|_| {
        CertificateDetails {
            id: id.clone(),
            subject: "unknown".to_string(),
            issuer: "unknown".to_string(),
            not_before: "unknown".to_string(),
            not_after: "unknown".to_string(),
            serial_number: "unknown".to_string(),
            fingerprint: "unknown".to_string(),
            subject_alternative_names: Vec::new(),
            key_usage: Vec::new(),
            extended_key_usage: Vec::new(),
            public_key_algorithm: "unknown".to_string(),
            signature_algorithm: "unknown".to_string(),
            version: 0,
        }
    });
    
    Json(CertificateDetailsResponse { certificate })
}

/// Rotate certificates handler
pub async fn rotate_certificates(
    State(state): State<Arc<AppState>>,
    Json(request): Json<CertificateRotationRequest>,
) -> Json<CertificateRotationResponse> {
    match state.security_manager.rotate_certificates(request.force).await {
        Ok(new_certificates) => Json(CertificateRotationResponse {
            status: "success".to_string(),
            message: "Certificates rotated successfully".to_string(),
            new_certificates,
        }),
        Err(e) => Json(CertificateRotationResponse {
            status: "error".to_string(),
            message: format!("Failed to rotate certificates: {}", e),
            new_certificates: Vec::new(),
        }),
    }
}
