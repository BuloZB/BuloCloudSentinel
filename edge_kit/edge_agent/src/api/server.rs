use std::net::SocketAddr;
use std::sync::Arc;

use anyhow::{Result, Context};
use axum::{
    routing::{get, post},
    Router,
};
use tower_http::cors::{Any, CorsLayer};
use tower_http::trace::TraceLayer;
use tracing::{info, error};

use super::routes;
use super::state::AppState;

/// API server for the Edge Agent
pub struct ApiServer {
    state: Arc<AppState>,
}

impl ApiServer {
    /// Create a new API server
    pub fn new(state: Arc<AppState>) -> Self {
        Self { state }
    }
    
    /// Run the API server
    pub async fn run(&self) -> Result<()> {
        let cors = CorsLayer::new()
            .allow_origin(Any)
            .allow_methods(Any)
            .allow_headers(Any);
        
        let app = Router::new()
            // Health routes
            .route("/health", get(routes::health::health_check))
            .route("/metrics", get(routes::health::metrics))
            
            // Device routes
            .route("/device", get(routes::device::get_device_info))
            .route("/device/logs", get(routes::device::get_logs))
            
            // OTA routes
            .route("/ota/check", post(routes::ota::check_updates))
            .route("/ota/update", post(routes::ota::update))
            .route("/ota/rollback", post(routes::ota::rollback))
            
            // Docker routes
            .route("/docker/containers", get(routes::docker::list_containers))
            .route("/docker/containers/:id", get(routes::docker::get_container))
            .route("/docker/containers/:id/logs", get(routes::docker::get_container_logs))
            .route("/docker/containers/:id/restart", post(routes::docker::restart_container))
            
            // Security routes
            .route("/security/certificates", get(routes::security::list_certificates))
            .route("/security/certificates/:id", get(routes::security::get_certificate))
            .route("/security/rotate", post(routes::security::rotate_certificates))
            
            // Add middleware
            .layer(TraceLayer::new_for_http())
            .layer(cors)
            .with_state(self.state.clone());
        
        let port = self.state.config.api_port;
        let addr = SocketAddr::from(([0, 0, 0, 0], port));
        
        info!("Starting API server on {}", addr);
        
        axum::Server::bind(&addr)
            .serve(app.into_make_service())
            .await
            .context("Failed to start API server")?;
        
        Ok(())
    }
}
