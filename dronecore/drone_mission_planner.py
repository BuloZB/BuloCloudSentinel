class DroneMissionPlanner:
    def __init__(self):
        self._current_mission = None
        self._waypoints = []

    def load_mission(self, waypoints):
        """Load a list of waypoints for the mission."""
        self._waypoints = waypoints
        self._current_mission = iter(self._waypoints)

    def get_next_waypoint(self):
        """Get the next waypoint in the mission."""
        try:
            return next(self._current_mission)
        except StopIteration:
            return None

    def clear_mission(self):
        """Clear the current mission."""
        self._waypoints = []
        self._current_mission = None
