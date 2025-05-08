use std::sync::Arc;
use tokio::sync::Mutex;

use crate::config::Config;
use crate::docker::DockerClient;
use crate::health::HealthMonitor;
use crate::mqtt::MqttClient;
use crate::ota::OtaManager;
use crate::security::SecurityManager;

/// Application state shared between API handlers
pub struct AppState {
    /// Configuration
    pub config: Config,
    
    /// Docker client
    pub docker_client: DockerClient,
    
    /// MQTT client
    pub mqtt_client: MqttClient,
    
    /// Health monitor
    pub health_monitor: Mutex<HealthMonitor>,
    
    /// OTA manager
    pub ota_manager: Mutex<OtaManager>,
    
    /// Security manager
    pub security_manager: SecurityManager,
}
