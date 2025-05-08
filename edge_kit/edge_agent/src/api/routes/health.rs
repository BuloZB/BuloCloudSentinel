use std::sync::Arc;

use axum::{
    extract::State,
    Json,
};
use serde::{Deserialize, Serialize};

use crate::api::state::AppState;

/// Health check response
#[derive(Debug, Serialize, Deserialize)]
pub struct HealthResponse {
    /// Status of the agent
    pub status: String,
    
    /// Version of the agent
    pub version: String,
    
    /// Device ID
    pub device_id: String,
    
    /// Device type
    pub device_type: String,
    
    /// Uptime in seconds
    pub uptime: u64,
}

/// Metrics response
#[derive(Debug, Serialize, Deserialize)]
pub struct MetricsResponse {
    /// CPU usage percentage
    pub cpu_usage: f64,
    
    /// Memory usage percentage
    pub memory_usage: f64,
    
    /// Disk usage percentage
    pub disk_usage: f64,
    
    /// Network stats
    pub network: NetworkStats,
    
    /// Container stats
    pub containers: Vec<ContainerStats>,
}

/// Network statistics
#[derive(Debug, Serialize, Deserialize)]
pub struct NetworkStats {
    /// Bytes received
    pub rx_bytes: u64,
    
    /// Bytes transmitted
    pub tx_bytes: u64,
    
    /// Packets received
    pub rx_packets: u64,
    
    /// Packets transmitted
    pub tx_packets: u64,
}

/// Container statistics
#[derive(Debug, Serialize, Deserialize)]
pub struct ContainerStats {
    /// Container ID
    pub id: String,
    
    /// Container name
    pub name: String,
    
    /// CPU usage percentage
    pub cpu_usage: f64,
    
    /// Memory usage percentage
    pub memory_usage: f64,
    
    /// Status
    pub status: String,
}

/// Health check handler
pub async fn health_check(
    State(state): State<Arc<AppState>>,
) -> Json<HealthResponse> {
    let health_monitor = state.health_monitor.lock().await;
    let health_info = health_monitor.get_health_info().await;
    
    Json(HealthResponse {
        status: "ok".to_string(),
        version: env!("CARGO_PKG_VERSION").to_string(),
        device_id: state.config.device_id.clone(),
        device_type: state.config.device_type.clone(),
        uptime: health_info.uptime,
    })
}

/// Metrics handler
pub async fn metrics(
    State(state): State<Arc<AppState>>,
) -> Json<MetricsResponse> {
    let health_monitor = state.health_monitor.lock().await;
    let metrics = health_monitor.get_metrics().await;
    
    Json(metrics)
}
