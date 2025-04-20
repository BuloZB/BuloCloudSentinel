import asyncio

class DroneTelemetryService:
    def __init__(self):
        self._subscribers = []

    def subscribe(self, callback):
        """Subscribe to telemetry updates."""
        self._subscribers.append(callback)

    def unsubscribe(self, callback):
        """Unsubscribe from telemetry updates."""
        self._subscribers.remove(callback)

    async def notify(self, telemetry_data):
        """Notify all subscribers with new telemetry data."""
        for callback in self._subscribers:
            await callback(telemetry_data)

    async def start_listening(self, adapter):
        """Start listening to telemetry from the flight controller adapter."""
        while True:
            telemetry = adapter.receive_telemetry()
            if telemetry:
                await self.notify(telemetry)
            await asyncio.sleep(0.01)  # 100 Hz
