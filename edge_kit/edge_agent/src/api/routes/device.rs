use std::sync::Arc;

use axum::{
    extract::{Query, State},
    Json,
};
use serde::{Deserialize, Serialize};

use crate::api::state::AppState;

/// Device information response
#[derive(Debug, Serialize, Deserialize)]
pub struct DeviceInfoResponse {
    /// Device ID
    pub id: String,
    
    /// Device type (jetson or rpi)
    pub device_type: String,
    
    /// Hardware information
    pub hardware: HardwareInfo,
    
    /// Software information
    pub software: SoftwareInfo,
    
    /// Network information
    pub network: NetworkInfo,
}

/// Hardware information
#[derive(Debug, Serialize, Deserialize)]
pub struct HardwareInfo {
    /// CPU model
    pub cpu_model: String,
    
    /// CPU cores
    pub cpu_cores: u32,
    
    /// Total memory in bytes
    pub total_memory: u64,
    
    /// Total disk space in bytes
    pub total_disk: u64,
    
    /// GPU model (if available)
    pub gpu_model: Option<String>,
}

/// Software information
#[derive(Debug, Serialize, Deserialize)]
pub struct SoftwareInfo {
    /// OS name
    pub os_name: String,
    
    /// OS version
    pub os_version: String,
    
    /// Kernel version
    pub kernel_version: String,
    
    /// Docker version
    pub docker_version: String,
    
    /// Edge Kit version
    pub edge_kit_version: String,
}

/// Network information
#[derive(Debug, Serialize, Deserialize)]
pub struct NetworkInfo {
    /// Hostname
    pub hostname: String,
    
    /// IP addresses
    pub ip_addresses: Vec<String>,
    
    /// MAC addresses
    pub mac_addresses: Vec<String>,
    
    /// Connected to internet
    pub internet_connected: bool,
}

/// Logs query parameters
#[derive(Debug, Deserialize)]
pub struct LogsQuery {
    /// Number of lines to return
    #[serde(default = "default_lines")]
    pub lines: usize,
    
    /// Service to get logs for
    pub service: Option<String>,
}

fn default_lines() -> usize {
    100
}

/// Logs response
#[derive(Debug, Serialize, Deserialize)]
pub struct LogsResponse {
    /// Log entries
    pub logs: Vec<LogEntry>,
}

/// Log entry
#[derive(Debug, Serialize, Deserialize)]
pub struct LogEntry {
    /// Timestamp
    pub timestamp: String,
    
    /// Service
    pub service: String,
    
    /// Log level
    pub level: String,
    
    /// Message
    pub message: String,
}

/// Get device information handler
pub async fn get_device_info(
    State(state): State<Arc<AppState>>,
) -> Json<DeviceInfoResponse> {
    let health_monitor = state.health_monitor.lock().await;
    let device_info = health_monitor.get_device_info().await;
    
    Json(device_info)
}

/// Get logs handler
pub async fn get_logs(
    State(state): State<Arc<AppState>>,
    Query(query): Query<LogsQuery>,
) -> Json<LogsResponse> {
    let health_monitor = state.health_monitor.lock().await;
    let logs = health_monitor.get_logs(query.lines, query.service).await;
    
    Json(LogsResponse { logs })
}
