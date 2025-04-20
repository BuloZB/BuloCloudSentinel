import IncidentTimeline from './components/IncidentTimeline';
import DroneCommandHub from './components/DroneCommandHub';
import AIToolsPanel from './components/AIToolsPanel';

function App() {
  return (
    <div className="App">
      <IncidentTimeline />
      <DroneCommandHub />
      <AIToolsPanel />
    </div>
  );
}

export default App;
