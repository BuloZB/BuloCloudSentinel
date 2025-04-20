class FailsafeManager:
    def __init__(self):
        self._failsafe_engaged = False

    def engage_failsafe(self):
        """Engage failsafe mode to safely handle emergencies."""
        self._failsafe_engaged = True
        # Implement actions like return-to-home, hover, or land

    def disengage_failsafe(self):
        """Disengage failsafe mode."""
        self._failsafe_engaged = False

    def is_failsafe_engaged(self) -> bool:
        return self._failsafe_engaged
