"""
Blender Add-on for Drone Show Choreography Export

This add-on provides tools for creating and exporting drone show choreographies from Blender.
"""

bl_info = {
    "name": "Drone Show Exporter",
    "author": "Bulo.Cloud Sentinel Team",
    "version": (0, 1, 0),
    "blender": (2, 93, 0),
    "location": "View3D > Sidebar > Drone Show",
    "description": "Tools for creating and exporting drone show choreographies",
    "warning": "Experimental",
    "doc_url": "https://bulocloud-sentinel.example.com/docs/drone-show/blender",
    "category": "Animation",
}

import bpy
import json
import math
import os
import time
from bpy.props import (
    StringProperty,
    BoolProperty,
    IntProperty,
    FloatProperty,
    FloatVectorProperty,
    EnumProperty,
    PointerProperty,
)
from bpy.types import (
    Panel,
    Operator,
    PropertyGroup,
    AddonPreferences,
)


# Utility functions
def get_drone_objects():
    """Get all drone objects in the scene."""
    return [obj for obj in bpy.context.scene.objects if obj.get("is_drone", False)]


def create_drone_object(name=None):
    """Create a new drone object."""
    if name is None:
        # Find the next available drone number
        existing_drones = get_drone_objects()
        existing_numbers = []
        for drone in existing_drones:
            if drone.name.startswith("drone_"):
                try:
                    num = int(drone.name[6:])
                    existing_numbers.append(num)
                except ValueError:
                    pass
        
        if existing_numbers:
            next_num = max(existing_numbers) + 1
        else:
            next_num = 1
        
        name = f"drone_{next_num}"
    
    # Create empty object
    bpy.ops.object.empty_add(type='PLAIN_AXES', location=(0, 0, 0))
    obj = bpy.context.active_object
    obj.name = name
    
    # Mark as drone
    obj["is_drone"] = True
    
    # Add default LED properties
    obj["led_color"] = (1.0, 0.0, 0.0)  # Red
    obj["led_effect"] = "solid"
    
    return obj


def set_led_color(obj, color):
    """Set the LED color for a drone object."""
    if obj.get("is_drone", False):
        obj["led_color"] = color


def set_led_effect(obj, effect, params=None):
    """Set the LED effect for a drone object."""
    if obj.get("is_drone", False):
        obj["led_effect"] = effect
        
        # Set effect parameters
        if params:
            for key, value in params.items():
                obj[f"led_effect_{key}"] = value


def arrange_in_grid(drones, rows, cols, spacing):
    """Arrange drones in a grid formation."""
    if not drones:
        return
    
    # Calculate center of grid
    center_x = (cols - 1) * spacing / 2
    center_y = (rows - 1) * spacing / 2
    
    # Arrange drones
    index = 0
    for row in range(rows):
        for col in range(cols):
            if index < len(drones):
                drone = drones[index]
                drone.location.x = col * spacing - center_x
                drone.location.y = row * spacing - center_y
                index += 1


def arrange_in_circle(drones, radius):
    """Arrange drones in a circle formation."""
    if not drones:
        return
    
    # Arrange drones
    for i, drone in enumerate(drones):
        angle = 2 * math.pi * i / len(drones)
        drone.location.x = radius * math.cos(angle)
        drone.location.y = radius * math.sin(angle)
        
        # Set rotation to face center
        drone.rotation_euler.z = angle + math.pi / 2


def arrange_in_line(drones, spacing, axis='X'):
    """Arrange drones in a line formation."""
    if not drones:
        return
    
    # Calculate center of line
    center = (len(drones) - 1) * spacing / 2
    
    # Arrange drones
    for i, drone in enumerate(drones):
        if axis == 'X':
            drone.location.x = i * spacing - center
            drone.location.y = 0
        elif axis == 'Y':
            drone.location.x = 0
            drone.location.y = i * spacing - center
        elif axis == 'Z':
            drone.location.x = 0
            drone.location.y = 0
            drone.location.z = i * spacing


def export_choreography(filepath, reference_lat, reference_lon, reference_alt, frame_rate, time_scale):
    """Export the choreography to a JSON file."""
    # Get all drone objects
    drones = get_drone_objects()
    
    # Get animation range
    scene = bpy.context.scene
    frame_start = scene.frame_start
    frame_end = scene.frame_end
    
    # Create animation data
    animation_data = {
        "metadata": {
            "blender_version": bpy.app.version_string,
            "export_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "reference_lat": reference_lat,
            "reference_lon": reference_lon,
            "reference_alt": reference_alt,
            "frame_rate": frame_rate,
            "time_scale": time_scale,
            "frame_start": frame_start,
            "frame_end": frame_end,
        },
        "objects": []
    }
    
    # Process each drone
    for drone in drones:
        drone_data = {
            "name": drone.name,
            "is_drone": True,
            "frames": []
        }
        
        # Process each frame
        for frame in range(frame_start, frame_end + 1):
            # Set current frame
            scene.frame_set(frame)
            
            # Get drone position and rotation
            loc = drone.matrix_world.translation
            rot = drone.matrix_world.to_euler()
            
            # Convert to GPS coordinates
            # Simple conversion for demonstration purposes
            # In a real implementation, this would use proper geodetic calculations
            lat = reference_lat + loc.y / 111320  # 1 degree latitude = ~111.32 km
            lon = reference_lon + loc.x / (111320 * math.cos(reference_lat * math.pi / 180))  # 1 degree longitude = ~111.32 km * cos(lat)
            alt = reference_alt + loc.z  # Altitude in meters
            
            # Get heading (in degrees, 0-360)
            heading = math.degrees(rot.z) % 360
            
            # Get LED color and effect
            led_color = drone.get("led_color", (1.0, 0.0, 0.0))
            led_effect = drone.get("led_effect", "solid")
            
            # Get effect parameters
            effect_params = {}
            for key in drone.keys():
                if key.startswith("led_effect_"):
                    param_name = key[10:]  # Remove "led_effect_" prefix
                    effect_params[param_name] = drone[key]
            
            # Create frame data
            frame_data = {
                "time": (frame - frame_start) / frame_rate * time_scale,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "heading": heading,
                "color": {
                    "r": int(led_color[0] * 255),
                    "g": int(led_color[1] * 255),
                    "b": int(led_color[2] * 255)
                },
                "effect": led_effect
            }
            
            # Add effect parameters if any
            if effect_params:
                frame_data["effect_params"] = effect_params
            
            # Add frame data
            drone_data["frames"].append(frame_data)
        
        # Add drone data
        animation_data["objects"].append(drone_data)
    
    # Write to file
    with open(filepath, 'w') as f:
        json.dump(animation_data, f, indent=2)
    
    return len(drones), len(range(frame_start, frame_end + 1))


# Property definitions
class DroneShowProperties(PropertyGroup):
    """Properties for the Drone Show add-on."""
    
    reference_lat: FloatProperty(
        name="Reference Latitude",
        description="Latitude of the reference point (origin) in degrees",
        default=0.0,
    )
    
    reference_lon: FloatProperty(
        name="Reference Longitude",
        description="Longitude of the reference point (origin) in degrees",
        default=0.0,
    )
    
    reference_alt: FloatProperty(
        name="Reference Altitude",
        description="Altitude of the reference point (origin) in meters",
        default=0.0,
    )
    
    frame_rate: FloatProperty(
        name="Frame Rate",
        description="Frame rate of the animation in frames per second",
        default=30.0,
        min=1.0,
    )
    
    time_scale: FloatProperty(
        name="Time Scale",
        description="Time scale factor (1.0 = real-time)",
        default=1.0,
        min=0.1,
    )
    
    grid_rows: IntProperty(
        name="Rows",
        description="Number of rows in the grid formation",
        default=3,
        min=1,
    )
    
    grid_cols: IntProperty(
        name="Columns",
        description="Number of columns in the grid formation",
        default=3,
        min=1,
    )
    
    grid_spacing: FloatProperty(
        name="Spacing",
        description="Spacing between drones in the grid formation",
        default=5.0,
        min=0.1,
    )
    
    circle_radius: FloatProperty(
        name="Radius",
        description="Radius of the circle formation",
        default=10.0,
        min=0.1,
    )
    
    line_spacing: FloatProperty(
        name="Spacing",
        description="Spacing between drones in the line formation",
        default=5.0,
        min=0.1,
    )
    
    line_axis: EnumProperty(
        name="Axis",
        description="Axis along which to arrange the line formation",
        items=[
            ('X', "X", "Arrange along X axis"),
            ('Y', "Y", "Arrange along Y axis"),
            ('Z', "Z", "Arrange along Z axis"),
        ],
        default='X',
    )
    
    led_color: FloatVectorProperty(
        name="LED Color",
        description="Color of the LED",
        subtype='COLOR',
        size=3,
        min=0.0,
        max=1.0,
        default=(1.0, 0.0, 0.0),  # Red
    )
    
    led_effect: EnumProperty(
        name="LED Effect",
        description="Effect of the LED",
        items=[
            ('solid', "Solid", "Solid color"),
            ('blink', "Blink", "Blinking effect"),
            ('pulse', "Pulse", "Pulsing effect"),
            ('rainbow', "Rainbow", "Rainbow effect"),
            ('chase', "Chase", "Chase effect"),
            ('custom', "Custom", "Custom effect"),
        ],
        default='solid',
    )
    
    blink_frequency: FloatProperty(
        name="Blink Frequency",
        description="Frequency of the blinking effect in Hz",
        default=1.0,
        min=0.1,
    )
    
    pulse_frequency: FloatProperty(
        name="Pulse Frequency",
        description="Frequency of the pulsing effect in Hz",
        default=0.5,
        min=0.1,
    )
    
    rainbow_speed: FloatProperty(
        name="Rainbow Speed",
        description="Speed of the rainbow effect",
        default=1.0,
        min=0.1,
    )
    
    chase_speed: FloatProperty(
        name="Chase Speed",
        description="Speed of the chase effect",
        default=1.0,
        min=0.1,
    )
    
    num_drones: IntProperty(
        name="Number of Drones",
        description="Number of drones to create",
        default=5,
        min=1,
    )


# Operator definitions
class DRONE_SHOW_OT_create_drone(Operator):
    """Create a new drone object"""
    bl_idname = "drone_show.create_drone"
    bl_label = "Create Drone"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        create_drone_object()
        return {'FINISHED'}


class DRONE_SHOW_OT_set_as_drone(Operator):
    """Mark the selected object as a drone"""
    bl_idname = "drone_show.set_as_drone"
    bl_label = "Set as Drone"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        return context.active_object is not None
    
    def execute(self, context):
        obj = context.active_object
        obj["is_drone"] = True
        obj["led_color"] = (1.0, 0.0, 0.0)  # Red
        obj["led_effect"] = "solid"
        return {'FINISHED'}


class DRONE_SHOW_OT_set_led_color(Operator):
    """Set the LED color for the selected drone"""
    bl_idname = "drone_show.set_led_color"
    bl_label = "Set LED Color"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.get("is_drone", False)
    
    def execute(self, context):
        obj = context.active_object
        props = context.scene.drone_show
        set_led_color(obj, props.led_color)
        
        # Insert keyframe if in animation mode
        if bpy.context.scene.tool_settings.use_keyframe_insert_auto:
            obj.keyframe_insert(data_path='["led_color"]')
        
        return {'FINISHED'}


class DRONE_SHOW_OT_set_led_effect(Operator):
    """Set the LED effect for the selected drone"""
    bl_idname = "drone_show.set_led_effect"
    bl_label = "Set LED Effect"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.get("is_drone", False)
    
    def execute(self, context):
        obj = context.active_object
        props = context.scene.drone_show
        
        # Get effect parameters
        params = {}
        if props.led_effect == 'blink':
            params['frequency'] = props.blink_frequency
        elif props.led_effect == 'pulse':
            params['frequency'] = props.pulse_frequency
        elif props.led_effect == 'rainbow':
            params['speed'] = props.rainbow_speed
        elif props.led_effect == 'chase':
            params['speed'] = props.chase_speed
        
        set_led_effect(obj, props.led_effect, params)
        
        # Insert keyframe if in animation mode
        if bpy.context.scene.tool_settings.use_keyframe_insert_auto:
            obj.keyframe_insert(data_path='["led_effect"]')
            for key in params:
                obj.keyframe_insert(data_path=f'["led_effect_{key}"]')
        
        return {'FINISHED'}


class DRONE_SHOW_OT_batch_create_drones(Operator):
    """Create multiple drone objects"""
    bl_idname = "drone_show.batch_create_drones"
    bl_label = "Batch Create Drones"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        props = context.scene.drone_show
        for i in range(props.num_drones):
            create_drone_object()
        return {'FINISHED'}


class DRONE_SHOW_OT_arrange_grid(Operator):
    """Arrange selected drones in a grid formation"""
    bl_idname = "drone_show.arrange_grid"
    bl_label = "Arrange in Grid"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        return len([obj for obj in context.selected_objects if obj.get("is_drone", False)]) > 0
    
    def execute(self, context):
        props = context.scene.drone_show
        drones = [obj for obj in context.selected_objects if obj.get("is_drone", False)]
        arrange_in_grid(drones, props.grid_rows, props.grid_cols, props.grid_spacing)
        return {'FINISHED'}


class DRONE_SHOW_OT_arrange_circle(Operator):
    """Arrange selected drones in a circle formation"""
    bl_idname = "drone_show.arrange_circle"
    bl_label = "Arrange in Circle"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        return len([obj for obj in context.selected_objects if obj.get("is_drone", False)]) > 0
    
    def execute(self, context):
        props = context.scene.drone_show
        drones = [obj for obj in context.selected_objects if obj.get("is_drone", False)]
        arrange_in_circle(drones, props.circle_radius)
        return {'FINISHED'}


class DRONE_SHOW_OT_arrange_line(Operator):
    """Arrange selected drones in a line formation"""
    bl_idname = "drone_show.arrange_line"
    bl_label = "Arrange in Line"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        return len([obj for obj in context.selected_objects if obj.get("is_drone", False)]) > 0
    
    def execute(self, context):
        props = context.scene.drone_show
        drones = [obj for obj in context.selected_objects if obj.get("is_drone", False)]
        arrange_in_line(drones, props.line_spacing, props.line_axis)
        return {'FINISHED'}


class DRONE_SHOW_OT_export(Operator):
    """Export the drone show choreography"""
    bl_idname = "drone_show.export"
    bl_label = "Export Drone Show"
    bl_options = {'REGISTER'}
    
    filepath: StringProperty(
        name="File Path",
        description="Path to save the exported file",
        default="",
        subtype='FILE_PATH',
    )
    
    @classmethod
    def poll(cls, context):
        return len(get_drone_objects()) > 0
    
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}
    
    def execute(self, context):
        props = context.scene.drone_show
        
        # Export choreography
        num_drones, num_frames = export_choreography(
            self.filepath,
            props.reference_lat,
            props.reference_lon,
            props.reference_alt,
            props.frame_rate,
            props.time_scale,
        )
        
        self.report({'INFO'}, f"Exported {num_drones} drones with {num_frames} frames to {self.filepath}")
        return {'FINISHED'}


# Panel definitions
class DRONE_SHOW_PT_main_panel(Panel):
    """Main panel for the Drone Show add-on"""
    bl_label = "Drone Show"
    bl_idname = "DRONE_SHOW_PT_main_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Drone Show'
    
    def draw(self, context):
        layout = self.layout
        props = context.scene.drone_show
        
        # Drone creation
        box = layout.box()
        box.label(text="Drone Creation")
        row = box.row()
        row.operator("drone_show.create_drone")
        row.operator("drone_show.set_as_drone")
        
        # Batch creation
        box.prop(props, "num_drones")
        box.operator("drone_show.batch_create_drones")
        
        # Formation tools
        box = layout.box()
        box.label(text="Formation Tools")
        
        # Grid formation
        row = box.row()
        row.label(text="Grid Formation")
        col = box.column(align=True)
        col.prop(props, "grid_rows")
        col.prop(props, "grid_cols")
        col.prop(props, "grid_spacing")
        col.operator("drone_show.arrange_grid")
        
        # Circle formation
        row = box.row()
        row.label(text="Circle Formation")
        col = box.column(align=True)
        col.prop(props, "circle_radius")
        col.operator("drone_show.arrange_circle")
        
        # Line formation
        row = box.row()
        row.label(text="Line Formation")
        col = box.column(align=True)
        col.prop(props, "line_spacing")
        col.prop(props, "line_axis", expand=True)
        col.operator("drone_show.arrange_line")
        
        # LED tools
        box = layout.box()
        box.label(text="LED Tools")
        
        # LED color
        row = box.row()
        row.label(text="LED Color")
        col = box.column(align=True)
        col.prop(props, "led_color", text="")
        col.operator("drone_show.set_led_color")
        
        # LED effect
        row = box.row()
        row.label(text="LED Effect")
        col = box.column(align=True)
        col.prop(props, "led_effect", text="")
        
        # Effect parameters
        if props.led_effect == 'blink':
            col.prop(props, "blink_frequency")
        elif props.led_effect == 'pulse':
            col.prop(props, "pulse_frequency")
        elif props.led_effect == 'rainbow':
            col.prop(props, "rainbow_speed")
        elif props.led_effect == 'chase':
            col.prop(props, "chase_speed")
        
        col.operator("drone_show.set_led_effect")
        
        # Export
        box = layout.box()
        box.label(text="Export")
        
        # Reference point
        row = box.row()
        row.label(text="Reference Point")
        col = box.column(align=True)
        col.prop(props, "reference_lat")
        col.prop(props, "reference_lon")
        col.prop(props, "reference_alt")
        
        # Animation settings
        row = box.row()
        row.label(text="Animation Settings")
        col = box.column(align=True)
        col.prop(props, "frame_rate")
        col.prop(props, "time_scale")
        
        # Export button
        box.operator("drone_show.export")


# Registration
classes = (
    DroneShowProperties,
    DRONE_SHOW_OT_create_drone,
    DRONE_SHOW_OT_set_as_drone,
    DRONE_SHOW_OT_set_led_color,
    DRONE_SHOW_OT_set_led_effect,
    DRONE_SHOW_OT_batch_create_drones,
    DRONE_SHOW_OT_arrange_grid,
    DRONE_SHOW_OT_arrange_circle,
    DRONE_SHOW_OT_arrange_line,
    DRONE_SHOW_OT_export,
    DRONE_SHOW_PT_main_panel,
)


def register():
    """Register the add-on."""
    for cls in classes:
        bpy.utils.register_class(cls)
    
    bpy.types.Scene.drone_show = PointerProperty(type=DroneShowProperties)


def unregister():
    """Unregister the add-on."""
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    
    del bpy.types.Scene.drone_show


if __name__ == "__main__":
    register()
