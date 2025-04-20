class CommandBridge:
    def __init__(self, adapter):
        self.adapter = adapter

    def takeoff(self, altitude: float):
        # Construct and send takeoff command
        command = self._build_takeoff_command(altitude)
        self.adapter.send_command(command)

    def land(self):
        # Construct and send land command
        command = self._build_land_command()
        self.adapter.send_command(command)

    def rtl(self):
        # Construct and send Return-To-Launch command
        command = self._build_rtl_command()
        self.adapter.send_command(command)

    def loiter(self, duration: float):
        # Construct and send loiter command
        command = self._build_loiter_command(duration)
        self.adapter.send_command(command)

    def _build_takeoff_command(self, altitude: float):
        # Placeholder for building takeoff command message
        return {"type": "takeoff", "altitude": altitude}

    def _build_land_command(self):
        # Placeholder for building land command message
        return {"type": "land"}

    def _build_rtl_command(self):
        # Placeholder for building RTL command message
        return {"type": "rtl"}

    def _build_loiter_command(self, duration: float):
        # Placeholder for building loiter command message
        return {"type": "loiter", "duration": duration}
