bl_info = {
    "name": "SLRBS Scene Exporter",
    "author": "Your Name",
    "version": (1, 1),
    "blender": (4, 3, 0),
    "location": "File > Export > SLRBS Scene (.json)",
    "description": "Export scenes to SLRBS JSON format",
    "category": "Import-Export",
}

import bpy
import json
import mathutils
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, FloatProperty, IntProperty
from bpy.types import Operator

# ------------------------------------------------------------------------
#    Conversion Utilities
# ------------------------------------------------------------------------

def convert_vector3_to_json(vec, scale=1.0):
    # Blender Z-up to SLRBS Y-up: (x, y, z) -> (x, z, -y)
    return {"x": vec.x * scale, "y": vec.z * scale, "z": -vec.y * scale}


def convert_quaternion_to_json(quat):
    # Rotate quaternion from Z-up to Y-up
    rot = mathutils.Quaternion((0.7071068, -0.7071068, 0, 0))
    cq = rot @ quat @ rot.conjugated()
    return {"w": cq.w, "x": cq.x, "y": cq.y, "z": cq.z}


def convert_color_to_json(color):
    return {"r": color[0], "g": color[1], "b": color[2]}


def create_geometry_json(obj, scale=1.0):
    dims = obj.dimensions * scale
    # Sphere if roughly equal extents
    if abs(dims.x - dims.y) < 0.01 and abs(dims.x - dims.z) < 0.01:
        return {"type": "sphere", "radius": dims.x / 2.0}
    # Cylinder if one axis much smaller
    if (dims.x < 0.1 * dims.y and dims.x < 0.1 * dims.z) or \
            (dims.y < 0.1 * dims.x and dims.y < 0.1 * dims.z) or \
            (dims.z < 0.1 * dims.x and dims.z < 0.1 * dims.y):
        if dims.z > dims.x and dims.z > dims.y:
            height, radius = dims.z, max(dims.x, dims.y) / 2.0
        elif dims.y > dims.x and dims.y > dims.z:
            height, radius = dims.y, max(dims.x, dims.z) / 2.0
        else:
            height, radius = dims.x, max(dims.y, dims.z) / 2.0
        return {"type": "cylinder", "height": height, "radius": radius}
    # Default box
    return {"type": "box", "dimensions": {"x": dims.x, "y": dims.z, "z": dims.y}}


def convert_gravity(gravity, scale=1.0):
    # Blender (0,0,-g) to SLRBS (0,-g,0)
    return {"x": 0.0, "y": -gravity * scale, "z": 0.0}

# ------------------------------------------------------------------------
#    Main Export Operator
# ------------------------------------------------------------------------

class ExportSLRBSScene(Operator, ExportHelper):
    bl_idname = "export_scene.slrbs"
    bl_label = "Export SLRBS Scene"
    filename_ext = ".json"

    filter_glob: StringProperty(default="*.json", options={'HIDDEN'})

    export_selected: BoolProperty(
        name="Export Selected Only",
        default=False,
    )
    convert_units: BoolProperty(
        name="Convert Units to Meters",
        default=True,
    )
    scene_name: StringProperty(
        name="Scene Name",
        default="BlenderScene",
    )
    scene_description: StringProperty(
        name="Description",
        default="Scene exported from Blender",
    )
    gravity_magnitude: FloatProperty(
        name="Gravity",
        default=9.81,
    )
    time_step: FloatProperty(
        name="Time Step",
        default=0.01,
        precision=3,
    )
    solver_iterations: IntProperty(
        name="Solver Iterations",
        default=10,
        min=1,
    )

    def execute(self, context):
        scale = 1.0
        if self.convert_units and context.scene.unit_settings.system == 'METRIC':
            scale = context.scene.unit_settings.scale_length

        objs = context.selected_objects if self.export_selected else context.scene.objects
        valid = [o for o in objs if o.type in {'MESH', 'EMPTY'}]
        if not valid:
            self.report({'ERROR'}, "No valid objects to export.")
            return {'CANCELLED'}

        scene = {
            "name": self.scene_name,
            "description": self.scene_description,
            "version": 1.0,
            "bodies": [],
            "joints": [],
            "physics": {
                "gravity": convert_gravity(self.gravity_magnitude, scale),
                "time_step": self.time_step,
                "solver_iterations": self.solver_iterations
            }
        }

        body_map = {}
        for idx, obj in enumerate(valid):
            mass = getattr(obj.rigid_body, 'mass', 1.0)
            fixed = getattr(obj.rigid_body, 'type', 'ACTIVE') == 'PASSIVE'
            pos = convert_vector3_to_json(obj.location, scale)
            rot = convert_quaternion_to_json(
                obj.rotation_quaternion if obj.rotation_mode=='QUATERNION' else obj.rotation_euler.to_quaternion()
            )
            lv = {"x":0,"y":0,"z":0}
            av = {"x":0,"y":0,"z":0}
            # include velocities if available
            if hasattr(obj.rigid_body, 'linear_velocity'):
                lv = convert_vector3_to_json(obj.rigid_body.linear_velocity, scale)
            if hasattr(obj.rigid_body, 'angular_velocity'):
                av = convert_vector3_to_json(obj.rigid_body.angular_velocity, scale)

            geom = create_geometry_json(obj, scale)
            # visual
            mat = obj.active_material
            if mat:
                col = mat.diffuse_color[:3]
                transp = 1.0 - mat.diffuse_color[3]
            else:
                col, transp = (0.8,0.8,0.8), 0.0

            vis = {
                "color": convert_color_to_json(col),
                "transparency": transp,
                "smooth_shade": obj.data.use_auto_smooth if hasattr(obj.data, 'use_auto_smooth') else True,
                "edge_width": 1.0
            }

            body = {
                "id": idx,
                "mass": mass,
                "fixed": fixed,
                "position": pos,
                "orientation": rot,
                "linear_velocity": lv,
                "angular_velocity": av,
                "geometry": geom,
                "visual": vis
            }
            scene["bodies"].append(body)
            body_map[obj.name] = idx

        # joints: process Rigid Body Constraints
        for idx, c in enumerate(bpy.context.scene.rigid_body_constraints):
            o0, o1 = c.object1, c.object2
            if o0 and o1 and o0.name in body_map and o1.name in body_map:
                j = {"id": idx, "body0_id": body_map[o0.name], "body1_id": body_map[o1.name]}
                if c.type == 'HINGE':
                    j["type"] = 'hinge'
                else:
                    j["type"] = 'spherical'
                # offsets at constraint location
                co = c.pivot_type == 'GENERIC' and c.pivot_point or (0,0,0)
                off0 = convert_vector3_to_json(mathutils.Vector(co), scale)
                off1 = off0.copy()
                j["body0_offset"], j["body1_offset"] = off0, off1
                scene["joints"].append(j)

        with open(self.filepath, 'w') as f:
            json.dump(scene, f, indent=4)

        self.report({'INFO'}, f"Exported {len(scene['bodies'])} bodies and {len(scene['joints'])} joints.")
        return {'FINISHED'}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "scene_name")
        layout.prop(self, "scene_description")
        layout.prop(self, "export_selected")
        layout.prop(self, "convert_units")
        layout.prop(self, "gravity_magnitude")
        layout.prop(self, "time_step")
        layout.prop(self, "solver_iterations")


def menu_func_export(self, context):
    self.layout.operator(ExportSLRBSScene.bl_idname, text="SLRBS Scene (.json)")


def register():
    bpy.utils.register_class(ExportSLRBSScene)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(ExportSLRBSScene)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)


if __name__ == "__main__":
    register()