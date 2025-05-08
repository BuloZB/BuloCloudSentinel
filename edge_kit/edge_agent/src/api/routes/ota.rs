use std::sync::Arc;

use axum::{
    extract::State,
    Json,
};
use serde::{Deserialize, Serialize};

use crate::api::state::AppState;

/// Update check request
#[derive(Debug, Deserialize)]
pub struct UpdateCheckRequest {
    /// Force check
    #[serde(default)]
    pub force: bool,
}

/// Update check response
#[derive(Debug, Serialize)]
pub struct UpdateCheckResponse {
    /// Update available
    pub update_available: bool,
    
    /// Current version
    pub current_version: String,
    
    /// Latest version
    pub latest_version: Option<String>,
    
    /// Update details
    pub update_details: Option<UpdateDetails>,
}

/// Update details
#[derive(Debug, Serialize)]
pub struct UpdateDetails {
    /// Version
    pub version: String,
    
    /// Release date
    pub release_date: String,
    
    /// Release notes
    pub release_notes: String,
    
    /// Size in bytes
    pub size: u64,
}

/// Update request
#[derive(Debug, Deserialize)]
pub struct UpdateRequest {
    /// Version to update to
    pub version: String,
    
    /// Force update
    #[serde(default)]
    pub force: bool,
}

/// Update response
#[derive(Debug, Serialize)]
pub struct UpdateResponse {
    /// Update ID
    pub update_id: String,
    
    /// Status
    pub status: String,
    
    /// Message
    pub message: String,
}

/// Rollback request
#[derive(Debug, Deserialize)]
pub struct RollbackRequest {
    /// Force rollback
    #[serde(default)]
    pub force: bool,
}

/// Rollback response
#[derive(Debug, Serialize)]
pub struct RollbackResponse {
    /// Rollback ID
    pub rollback_id: String,
    
    /// Status
    pub status: String,
    
    /// Message
    pub message: String,
}

/// Check for updates handler
pub async fn check_updates(
    State(state): State<Arc<AppState>>,
    Json(request): Json<UpdateCheckRequest>,
) -> Json<UpdateCheckResponse> {
    let mut ota_manager = state.ota_manager.lock().await;
    let update_check = ota_manager.check_updates(request.force).await;
    
    Json(update_check)
}

/// Update handler
pub async fn update(
    State(state): State<Arc<AppState>>,
    Json(request): Json<UpdateRequest>,
) -> Json<UpdateResponse> {
    let mut ota_manager = state.ota_manager.lock().await;
    let update_result = ota_manager.update(request.version, request.force).await;
    
    Json(update_result)
}

/// Rollback handler
pub async fn rollback(
    State(state): State<Arc<AppState>>,
    Json(request): Json<RollbackRequest>,
) -> Json<RollbackResponse> {
    let mut ota_manager = state.ota_manager.lock().await;
    let rollback_result = ota_manager.rollback(request.force).await;
    
    Json(rollback_result)
}
