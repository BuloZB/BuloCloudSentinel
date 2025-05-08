use std::sync::Arc;
use std::time::Duration;

use anyhow::Result;
use clap::Parser;
use tokio::sync::Mutex;
use tracing::{info, error, warn, debug};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

mod api;
mod config;
mod docker;
mod health;
mod mqtt;
mod ota;
mod security;
mod vault;

use crate::api::server::ApiServer;
use crate::config::Config;
use crate::docker::DockerClient;
use crate::health::HealthMonitor;
use crate::mqtt::MqttClient;
use crate::ota::OtaManager;
use crate::security::SecurityManager;
use crate::vault::VaultClient;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to config file
    #[arg(short, long, default_value = "/config/config.yaml")]
    config: String,
    
    /// Log level
    #[arg(short, long, default_value = "info")]
    log_level: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    // Parse command line arguments
    let args = Args::parse();
    
    // Initialize logging
    let log_level = std::env::var("LOG_LEVEL").unwrap_or_else(|_| args.log_level.clone());
    let log_filter = format!("edge_agent={}", log_level.to_lowercase());
    
    tracing_subscriber::registry()
        .with(tracing_subscriber::EnvFilter::new(log_filter))
        .with(tracing_subscriber::fmt::layer())
        .init();
    
    info!("Starting Bulo.CloudSentinel Edge Agent");
    
    // Load configuration
    let config = match Config::from_file(&args.config).await {
        Ok(config) => config,
        Err(e) => {
            warn!("Failed to load config from file: {}", e);
            info!("Using environment variables for configuration");
            Config::from_env()?
        }
    };
    
    // Initialize components
    let docker_client = DockerClient::new().await?;
    let vault_client = VaultClient::new(&config.vault_addr, &config.vault_token, &config.vault_path).await?;
    let security_manager = SecurityManager::new(&config, &vault_client).await?;
    let mqtt_client = MqttClient::new(&config, &security_manager).await?;
    let health_monitor = HealthMonitor::new(&config, docker_client.clone()).await?;
    let ota_manager = OtaManager::new(&config, docker_client.clone(), mqtt_client.clone()).await?;
    
    // Create shared state
    let state = Arc::new(api::state::AppState {
        config: config.clone(),
        docker_client: docker_client.clone(),
        mqtt_client: mqtt_client.clone(),
        health_monitor: Mutex::new(health_monitor),
        ota_manager: Mutex::new(ota_manager),
        security_manager: security_manager.clone(),
    });
    
    // Start API server
    let api_server = ApiServer::new(state.clone());
    let server_handle = tokio::spawn(async move {
        if let Err(e) = api_server.run().await {
            error!("API server error: {}", e);
        }
    });
    
    // Start MQTT client
    let mqtt_handle = tokio::spawn(async move {
        if let Err(e) = mqtt_client.run().await {
            error!("MQTT client error: {}", e);
        }
    });
    
    // Start health monitor
    let health_handle = tokio::spawn(async move {
        if let Err(e) = health_monitor.run().await {
            error!("Health monitor error: {}", e);
        }
    });
    
    // Start OTA manager
    let ota_handle = tokio::spawn(async move {
        if let Err(e) = ota_manager.run().await {
            error!("OTA manager error: {}", e);
        }
    });
    
    // Register with central server
    info!("Registering device with central server");
    if let Err(e) = register_device(&config).await {
        warn!("Failed to register device: {}", e);
    }
    
    // Wait for all tasks to complete
    tokio::select! {
        _ = server_handle => {
            error!("API server exited unexpectedly");
        }
        _ = mqtt_handle => {
            error!("MQTT client exited unexpectedly");
        }
        _ = health_handle => {
            error!("Health monitor exited unexpectedly");
        }
        _ = ota_handle => {
            error!("OTA manager exited unexpectedly");
        }
        _ = tokio::signal::ctrl_c() => {
            info!("Received shutdown signal");
        }
    }
    
    info!("Shutting down Edge Agent");
    
    Ok(())
}

async fn register_device(config: &Config) -> Result<()> {
    // This is a placeholder for device registration
    // In a real implementation, you would register the device with the central server
    // using mTLS with SPIFFE/SPIRE
    
    // Simulate registration delay
    tokio::time::sleep(Duration::from_secs(1)).await;
    
    info!("Device registered successfully: {}", config.device_id);
    
    Ok(())
}
