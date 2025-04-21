import React, { useState, useEffect, useCallback } from 'react';
import { Responsive, WidthProvider } from 'react-grid-layout';
import 'react-grid-layout/css/styles.css';
import 'react-resizable/css/styles.css';
import './Dashboard.css';

import DroneStatusWidget from '../Widgets/DroneStatusWidget';
import MapWidget from '../Widgets/MapWidget';
import TelemetryWidget from '../Widgets/TelemetryWidget';
import VideoWidget from '../Widgets/VideoWidget';
import BatteryWidget from '../Widgets/BatteryWidget';
import MissionWidget from '../Widgets/MissionWidget';
import WeatherWidget from '../Widgets/WeatherWidget';
import AddWidgetModal from './AddWidgetModal';

const ResponsiveGridLayout = WidthProvider(Responsive);

const Dashboard = () => {
  const [layouts, setLayouts] = useState({ lg: [] });
  const [widgets, setWidgets] = useState([]);
  const [isAddWidgetModalOpen, setIsAddWidgetModalOpen] = useState(false);
  const [availableWidgets, setAvailableWidgets] = useState([]);
  const [isEditing, setIsEditing] = useState(false);
  const [selectedLayout, setSelectedLayout] = useState(null);
  const [savedLayouts, setSavedLayouts] = useState([]);
  const [isLoading, setIsLoading] = useState(true);

  // Fetch available widgets and saved layouts on component mount
  useEffect(() => {
    const fetchData = async () => {
      try {
        // Fetch available widgets
        const widgetsResponse = await fetch('/api/dashboard/widgets');
        const widgetsData = await widgetsResponse.json();
        setAvailableWidgets(widgetsData);

        // Fetch saved layouts
        const layoutsResponse = await fetch('/api/dashboard/layouts');
        const layoutsData = await layoutsResponse.json();
        setSavedLayouts(layoutsData);

        // Set default layout if available
        const defaultLayout = layoutsData.find(layout => layout.is_default);
        if (defaultLayout) {
          setSelectedLayout(defaultLayout);
          setLayouts({ lg: defaultLayout.layout_data.map(widget => widget.position) });
          setWidgets(defaultLayout.layout_data);
        } else if (layoutsData.length > 0) {
          // Use first layout if no default
          setSelectedLayout(layoutsData[0]);
          setLayouts({ lg: layoutsData[0].layout_data.map(widget => widget.position) });
          setWidgets(layoutsData[0].layout_data);
        } else {
          // Create default layout if none exists
          const defaultWidgets = [
            {
              id: 'drone-status',
              type: 'drone-status',
              title: 'Drone Status',
              position: { i: 'drone-status', x: 0, y: 0, w: 4, h: 3, minW: 2, minH: 2 },
              settings: {}
            },
            {
              id: 'map',
              type: 'map',
              title: 'Map',
              position: { i: 'map', x: 4, y: 0, w: 8, h: 6, minW: 4, minH: 4 },
              settings: {}
            },
            {
              id: 'telemetry',
              type: 'telemetry',
              title: 'Telemetry',
              position: { i: 'telemetry', x: 0, y: 3, w: 4, h: 3, minW: 2, minH: 2 },
              settings: {}
            },
            {
              id: 'video',
              type: 'video',
              title: 'Video Feed',
              position: { i: 'video', x: 0, y: 6, w: 6, h: 4, minW: 4, minH: 3 },
              settings: {}
            },
            {
              id: 'battery',
              type: 'battery',
              title: 'Battery Status',
              position: { i: 'battery', x: 6, y: 6, w: 6, h: 4, minW: 2, minH: 2 },
              settings: {}
            }
          ];

          setWidgets(defaultWidgets);
          setLayouts({ lg: defaultWidgets.map(widget => widget.position) });
        }

        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching dashboard data:', error);
        setIsLoading(false);
      }
    };

    fetchData();
  }, []);

  // Handle layout change
  const handleLayoutChange = useCallback((layout, layouts) => {
    setLayouts(layouts);
    
    // Update widget positions
    const updatedWidgets = widgets.map(widget => {
      const layoutItem = layout.find(item => item.i === widget.id);
      if (layoutItem) {
        return {
          ...widget,
          position: layoutItem
        };
      }
      return widget;
    });
    
    setWidgets(updatedWidgets);
  }, [widgets]);

  // Add a new widget
  const handleAddWidget = useCallback((widgetType) => {
    const widgetTemplate = availableWidgets.find(w => w.widget_type === widgetType);
    
    if (!widgetTemplate) return;
    
    const newId = `${widgetType}-${Date.now()}`;
    const newWidget = {
      id: newId,
      type: widgetType,
      title: widgetTemplate.name,
      position: { 
        i: newId, 
        x: 0, 
        y: Infinity, // Add to bottom
        w: 4, 
        h: 3,
        minW: 2,
        minH: 2
      },
      settings: widgetTemplate.default_config || {}
    };
    
    setWidgets([...widgets, newWidget]);
    setLayouts({
      ...layouts,
      lg: [...layouts.lg, newWidget.position]
    });
    
    setIsAddWidgetModalOpen(false);
  }, [availableWidgets, widgets, layouts]);

  // Remove a widget
  const handleRemoveWidget = useCallback((widgetId) => {
    setWidgets(widgets.filter(widget => widget.id !== widgetId));
    setLayouts({
      ...layouts,
      lg: layouts.lg.filter(item => item.i !== widgetId)
    });
  }, [widgets, layouts]);

  // Save current layout
  const handleSaveLayout = useCallback(async (name, isDefault = false) => {
    try {
      const layoutData = widgets.map(widget => ({
        id: widget.id,
        type: widget.type,
        title: widget.title,
        position: widget.position,
        settings: widget.settings
      }));
      
      const payload = {
        name,
        is_default: isDefault,
        layout_data: layoutData
      };
      
      if (selectedLayout) {
        // Update existing layout
        const response = await fetch(`/api/dashboard/layouts/${selectedLayout.id}`, {
          method: 'PUT',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify(payload)
        });
        
        const updatedLayout = await response.json();
        setSavedLayouts(savedLayouts.map(layout => 
          layout.id === updatedLayout.id ? updatedLayout : layout
        ));
        setSelectedLayout(updatedLayout);
      } else {
        // Create new layout
        const response = await fetch('/api/dashboard/layouts', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify(payload)
        });
        
        const newLayout = await response.json();
        setSavedLayouts([...savedLayouts, newLayout]);
        setSelectedLayout(newLayout);
      }
      
      setIsEditing(false);
    } catch (error) {
      console.error('Error saving layout:', error);
    }
  }, [widgets, selectedLayout, savedLayouts]);

  // Load a saved layout
  const handleLoadLayout = useCallback((layoutId) => {
    const layout = savedLayouts.find(l => l.id === layoutId);
    if (layout) {
      setSelectedLayout(layout);
      setWidgets(layout.layout_data);
      setLayouts({ lg: layout.layout_data.map(widget => widget.position) });
    }
  }, [savedLayouts]);

  // Render widget based on type
  const renderWidget = useCallback((widget) => {
    const { id, type, title, settings } = widget;
    
    const props = {
      id,
      title,
      settings,
      onRemove: isEditing ? () => handleRemoveWidget(id) : null
    };
    
    switch (type) {
      case 'drone-status':
        return <DroneStatusWidget {...props} />;
      case 'map':
        return <MapWidget {...props} />;
      case 'telemetry':
        return <TelemetryWidget {...props} />;
      case 'video':
        return <VideoWidget {...props} />;
      case 'battery':
        return <BatteryWidget {...props} />;
      case 'mission':
        return <MissionWidget {...props} />;
      case 'weather':
        return <WeatherWidget {...props} />;
      default:
        return <div>Unknown widget type: {type}</div>;
    }
  }, [handleRemoveWidget, isEditing]);

  if (isLoading) {
    return <div className="dashboard-loading">Loading dashboard...</div>;
  }

  return (
    <div className="dashboard-container">
      <div className="dashboard-header">
        <div className="dashboard-title">
          <h1>{selectedLayout ? selectedLayout.name : 'Dashboard'}</h1>
        </div>
        
        <div className="dashboard-controls">
          <select 
            value={selectedLayout ? selectedLayout.id : ''} 
            onChange={(e) => handleLoadLayout(e.target.value)}
            disabled={isEditing}
          >
            <option value="" disabled>Select Layout</option>
            {savedLayouts.map(layout => (
              <option key={layout.id} value={layout.id}>
                {layout.name} {layout.is_default ? '(Default)' : ''}
              </option>
            ))}
          </select>
          
          <button 
            className={`edit-button ${isEditing ? 'active' : ''}`}
            onClick={() => setIsEditing(!isEditing)}
          >
            {isEditing ? 'Done Editing' : 'Edit Dashboard'}
          </button>
          
          {isEditing && (
            <>
              <button 
                className="add-widget-button"
                onClick={() => setIsAddWidgetModalOpen(true)}
              >
                Add Widget
              </button>
              
              <button 
                className="save-button"
                onClick={() => handleSaveLayout(
                  selectedLayout ? selectedLayout.name : 'New Dashboard',
                  selectedLayout ? selectedLayout.is_default : false
                )}
              >
                Save Layout
              </button>
            </>
          )}
        </div>
      </div>
      
      <div className="dashboard-content">
        <ResponsiveGridLayout
          className="layout"
          layouts={layouts}
          breakpoints={{ lg: 1200, md: 996, sm: 768, xs: 480, xxs: 0 }}
          cols={{ lg: 12, md: 10, sm: 6, xs: 4, xxs: 2 }}
          rowHeight={100}
          onLayoutChange={handleLayoutChange}
          isDraggable={isEditing}
          isResizable={isEditing}
          compactType="vertical"
          margin={[16, 16]}
        >
          {widgets.map(widget => (
            <div key={widget.id} className="widget-container">
              {renderWidget(widget)}
            </div>
          ))}
        </ResponsiveGridLayout>
      </div>
      
      {isAddWidgetModalOpen && (
        <AddWidgetModal
          availableWidgets={availableWidgets}
          onAddWidget={handleAddWidget}
          onClose={() => setIsAddWidgetModalOpen(false)}
        />
      )}
    </div>
  );
};

export default Dashboard;
