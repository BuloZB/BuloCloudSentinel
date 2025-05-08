use anyhow::{Result, Context};
use serde::{Deserialize, Serialize};
use std::env;
use std::path::Path;
use tokio::fs;

/// Configuration for the Edge Agent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    /// Device ID
    pub device_id: String,
    
    /// Device type (jetson or rpi)
    pub device_type: String,
    
    /// MQTT broker URL
    pub mqtt_broker: String,
    
    /// MQTT username
    pub mqtt_username: Option<String>,
    
    /// MQTT password
    pub mqtt_password: Option<String>,
    
    /// MQTT client ID
    pub mqtt_client_id: String,
    
    /// MQTT topic prefix
    pub mqtt_topic_prefix: String,
    
    /// Vault address
    pub vault_addr: String,
    
    /// Vault token
    pub vault_token: Option<String>,
    
    /// Vault path
    pub vault_path: String,
    
    /// Update check interval in seconds
    pub update_check_interval: u64,
    
    /// API server port
    pub api_port: u16,
    
    /// Path to certificates directory
    pub certs_dir: String,
}

impl Config {
    /// Load configuration from a file
    pub async fn from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = fs::read_to_string(path).await
            .context("Failed to read config file")?;
        
        let config: Config = serde_yaml::from_str(&content)
            .context("Failed to parse config file")?;
        
        Ok(config)
    }
    
    /// Load configuration from environment variables
    pub fn from_env() -> Result<Self> {
        let config = Config {
            device_id: env::var("DEVICE_ID").unwrap_or_else(|_| "edge-device-001".to_string()),
            device_type: env::var("DEVICE_TYPE").unwrap_or_else(|_| "jetson".to_string()),
            mqtt_broker: env::var("MQTT_BROKER").unwrap_or_else(|_| "mqtt://bulocloud-mqtt:1883".to_string()),
            mqtt_username: env::var("MQTT_USERNAME").ok(),
            mqtt_password: env::var("MQTT_PASSWORD").ok(),
            mqtt_client_id: env::var("MQTT_CLIENT_ID").unwrap_or_else(|_| "edge-device-001".to_string()),
            mqtt_topic_prefix: env::var("MQTT_TOPIC_PREFIX").unwrap_or_else(|_| "bulocloud/edge".to_string()),
            vault_addr: env::var("VAULT_ADDR").unwrap_or_else(|_| "http://bulocloud-vault:8200".to_string()),
            vault_token: env::var("VAULT_TOKEN").ok(),
            vault_path: env::var("VAULT_PATH").unwrap_or_else(|_| "secret/edge".to_string()),
            update_check_interval: env::var("UPDATE_CHECK_INTERVAL")
                .unwrap_or_else(|_| "3600".to_string())
                .parse()
                .context("Failed to parse UPDATE_CHECK_INTERVAL")?,
            api_port: env::var("API_PORT")
                .unwrap_or_else(|_| "9090".to_string())
                .parse()
                .context("Failed to parse API_PORT")?,
            certs_dir: env::var("CERTS_DIR").unwrap_or_else(|_| "/certs".to_string()),
        };
        
        Ok(config)
    }
    
    /// Save configuration to a file
    pub async fn save_to_file<P: AsRef<Path>>(&self, path: P) -> Result<()> {
        let content = serde_yaml::to_string(self)
            .context("Failed to serialize config")?;
        
        fs::write(path, content).await
            .context("Failed to write config file")?;
        
        Ok(())
    }
}
