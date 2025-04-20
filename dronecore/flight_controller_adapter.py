from abc import ABC, abstractmethod

class FlightControllerAdapter(ABC):
    @abstractmethod
    def connect(self):
        """Establish connection to the flight controller."""
        pass

    @abstractmethod
    def disconnect(self):
        """Disconnect from the flight controller."""
        pass

    @abstractmethod
    def send_command(self, command):
        """Send control command to the flight controller."""
        pass

    @abstractmethod
    def receive_telemetry(self):
        """Receive telemetry data from the flight controller."""
        pass
