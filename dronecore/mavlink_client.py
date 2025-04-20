import asyncio
from pymavlink import mavutil

class MAVLinkClient:
    def __init__(self, connection_string: str, baud: int = 115200):
        self.connection_string = connection_string
        self.baud = baud
        self.master = None
        self._running = False

    async def connect(self):
        self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baud)
        self._running = True
        await self._listen()

    async def _listen(self):
        while self._running:
            msg = self.master.recv_match(blocking=False)
            if msg:
                self.handle_message(msg)
            await asyncio.sleep(0.01)  # 100 Hz

    def handle_message(self, msg):
        # Override this method to process incoming MAVLink messages
        print(f"Received message: {msg}")

    def send_message(self, msg):
        if self.master:
            self.master.mav.send(msg)

    def disconnect(self):
        self._running = False
        if self.master:
            self.master.close()
