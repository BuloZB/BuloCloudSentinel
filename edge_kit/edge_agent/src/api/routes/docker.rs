use std::sync::Arc;

use axum::{
    extract::{Path, Query, State},
    Json,
};
use serde::{Deserialize, Serialize};

use crate::api::state::AppState;

/// Container list response
#[derive(Debug, Serialize)]
pub struct ContainerListResponse {
    /// Containers
    pub containers: Vec<ContainerInfo>,
}

/// Container information
#[derive(Debug, Serialize)]
pub struct ContainerInfo {
    /// Container ID
    pub id: String,
    
    /// Container name
    pub name: String,
    
    /// Image
    pub image: String,
    
    /// Status
    pub status: String,
    
    /// Created at
    pub created_at: String,
    
    /// Ports
    pub ports: Vec<PortMapping>,
    
    /// Health status
    pub health: Option<String>,
}

/// Port mapping
#[derive(Debug, Serialize)]
pub struct PortMapping {
    /// Host IP
    pub host_ip: String,
    
    /// Host port
    pub host_port: u16,
    
    /// Container port
    pub container_port: u16,
    
    /// Protocol
    pub protocol: String,
}

/// Container details response
#[derive(Debug, Serialize)]
pub struct ContainerDetailsResponse {
    /// Container information
    pub container: ContainerDetails,
}

/// Container details
#[derive(Debug, Serialize)]
pub struct ContainerDetails {
    /// Container ID
    pub id: String,
    
    /// Container name
    pub name: String,
    
    /// Image
    pub image: String,
    
    /// Status
    pub status: String,
    
    /// Created at
    pub created_at: String,
    
    /// Ports
    pub ports: Vec<PortMapping>,
    
    /// Health status
    pub health: Option<String>,
    
    /// Environment variables
    pub environment: Vec<String>,
    
    /// Volumes
    pub volumes: Vec<VolumeMapping>,
    
    /// Networks
    pub networks: Vec<NetworkInfo>,
    
    /// Command
    pub command: String,
    
    /// Entrypoint
    pub entrypoint: String,
    
    /// Working directory
    pub working_dir: String,
    
    /// User
    pub user: String,
    
    /// Labels
    pub labels: std::collections::HashMap<String, String>,
}

/// Volume mapping
#[derive(Debug, Serialize)]
pub struct VolumeMapping {
    /// Host path
    pub host_path: String,
    
    /// Container path
    pub container_path: String,
    
    /// Mode
    pub mode: String,
}

/// Network information
#[derive(Debug, Serialize)]
pub struct NetworkInfo {
    /// Network name
    pub name: String,
    
    /// IP address
    pub ip_address: String,
    
    /// Gateway
    pub gateway: String,
    
    /// MAC address
    pub mac_address: String,
}

/// Container logs query
#[derive(Debug, Deserialize)]
pub struct ContainerLogsQuery {
    /// Number of lines
    #[serde(default = "default_lines")]
    pub lines: usize,
    
    /// Follow logs
    #[serde(default)]
    pub follow: bool,
    
    /// Show timestamps
    #[serde(default)]
    pub timestamps: bool,
}

fn default_lines() -> usize {
    100
}

/// Container logs response
#[derive(Debug, Serialize)]
pub struct ContainerLogsResponse {
    /// Logs
    pub logs: String,
}

/// Container restart response
#[derive(Debug, Serialize)]
pub struct ContainerRestartResponse {
    /// Status
    pub status: String,
    
    /// Message
    pub message: String,
}

/// List containers handler
pub async fn list_containers(
    State(state): State<Arc<AppState>>,
) -> Json<ContainerListResponse> {
    let containers = state.docker_client.list_containers().await.unwrap_or_default();
    
    Json(ContainerListResponse { containers })
}

/// Get container handler
pub async fn get_container(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Json<ContainerDetailsResponse> {
    let container = state.docker_client.get_container(&id).await.unwrap_or_else(|_| {
        ContainerDetails {
            id: id.clone(),
            name: "unknown".to_string(),
            image: "unknown".to_string(),
            status: "unknown".to_string(),
            created_at: "unknown".to_string(),
            ports: Vec::new(),
            health: None,
            environment: Vec::new(),
            volumes: Vec::new(),
            networks: Vec::new(),
            command: "".to_string(),
            entrypoint: "".to_string(),
            working_dir: "".to_string(),
            user: "".to_string(),
            labels: std::collections::HashMap::new(),
        }
    });
    
    Json(ContainerDetailsResponse { container })
}

/// Get container logs handler
pub async fn get_container_logs(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
    Query(query): Query<ContainerLogsQuery>,
) -> Json<ContainerLogsResponse> {
    let logs = state.docker_client.get_container_logs(&id, query.lines, query.timestamps).await.unwrap_or_default();
    
    Json(ContainerLogsResponse { logs })
}

/// Restart container handler
pub async fn restart_container(
    State(state): State<Arc<AppState>>,
    Path(id): Path<String>,
) -> Json<ContainerRestartResponse> {
    match state.docker_client.restart_container(&id).await {
        Ok(_) => Json(ContainerRestartResponse {
            status: "success".to_string(),
            message: format!("Container {} restarted successfully", id),
        }),
        Err(e) => Json(ContainerRestartResponse {
            status: "error".to_string(),
            message: format!("Failed to restart container {}: {}", id, e),
        }),
    }
}
