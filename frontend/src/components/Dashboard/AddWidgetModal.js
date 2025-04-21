import React from 'react';
import './Dashboard.css';

const AddWidgetModal = ({ availableWidgets, onAddWidget, onClose }) => {
  // Widget icons mapping
  const widgetIcons = {
    'drone-status': '🚁',
    'map': '🗺️',
    'telemetry': '📊',
    'video': '📹',
    'battery': '🔋',
    'mission': '📝',
    'weather': '🌤️',
    'default': '📦'
  };

  return (
    <div className="add-widget-modal" onClick={onClose}>
      <div className="modal-content" onClick={(e) => e.stopPropagation()}>
        <div className="modal-header">
          <h2 className="modal-title">Add Widget</h2>
          <button className="modal-close" onClick={onClose}>&times;</button>
        </div>
        
        <div className="modal-body">
          <div className="widget-grid">
            {availableWidgets.map(widget => (
              <div 
                key={widget.id} 
                className="widget-card"
                onClick={() => onAddWidget(widget.widget_type)}
              >
                <div className="widget-icon">
                  {widgetIcons[widget.widget_type] || widgetIcons.default}
                </div>
                <div className="widget-name">{widget.name}</div>
                <div className="widget-description">{widget.description}</div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};

export default AddWidgetModal;
