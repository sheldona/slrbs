bl_info = {
    "name": "SLRBS Scene Importer",
    "author": "Your Name",
    "version": (1, 0),
    "blender": (4, 3, 0),
    "location": "File > Import > SLRBS Scene (.json)",
    "description": "Import scenes in SLRBS JSON format",
    "category": "Import-Export",
}

import bpy
import json
import mathutils
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty, BoolProperty
from bpy.types import Operator

# -------------------------------------------------------------
# Conversion functions (SLRBS Y-up -> Blender Z-up)
# -------------------------------------------------------------

def from_json_vec(d):
    # SLRBS Y-up (x, y, z) -> Blender Z-up (x, -z, y)
    return mathutils.Vector((d['x'], -d['z'], d['y']))


def from_json_quat(d):
    # Quaternion from SLRBS to Blender: apply inverse of conversion rotation
    rot = mathutils.Quaternion((0.7071068, -0.7071068, 0, 0))
    q = mathutils.Quaternion((d['w'], d['x'], d['y'], d['z']))
    return rot.conjugated() @ q @ rot

# -------------------------------------------------------------
# Geometry creation from JSON
# -------------------------------------------------------------

def create_geometry_from_json(body):
    geom = body['geometry']
    typ = geom['type']
    if typ == 'box':
        dims = geom['dimensions']
        bpy.ops.mesh.primitive_cube_add(size=1)
        obj = bpy.context.active_object
        obj.scale = (dims['x'], dims['y'], dims['z'])
    elif typ == 'sphere':
        bpy.ops.mesh.primitive_uv_sphere_add(radius=geom['radius'])
        obj = bpy.context.active_object
    elif typ == 'cylinder':
        bpy.ops.mesh.primitive_cylinder_add(radius=geom['radius'], depth=geom['height'])
        obj = bpy.context.active_object
    elif typ == 'plane':
        bpy.ops.mesh.primitive_plane_add(size=10)
        obj = bpy.context.active_object
        # orient plane to match normal
        n = from_json_vec(geom['normal']).normalized()
        q = mathutils.Vector((0, 0, 1)).rotation_difference(n)
        obj.rotation_mode = 'QUATERNION'
        obj.rotation_quaternion = q
    else:
        bpy.ops.mesh.primitive_cube_add(size=1)
        obj = bpy.context.active_object
    return obj

# -------------------------------------------------------------
# Import Operator
# -------------------------------------------------------------

class ImportSLRBSScene(Operator, ImportHelper):
    bl_idname = "import_scene.slrbs"
    bl_label = "Import SLRBS Scene"
    filename_ext = ".json"

    filter_glob: StringProperty(default="*.json", options={'HIDDEN'})
    clear_scene: BoolProperty(
        name="Clear Scene",
        description="Delete existing objects before import",
        default=True
    )

    def execute(self, context):
        # Load JSON
        with open(self.filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # Optionally clear existing scene
        if self.clear_scene:
            bpy.ops.object.select_all(action='SELECT')
            bpy.ops.object.delete()

        bodies = data.get('bodies', [])
        for b in bodies:
            obj = create_geometry_from_json(b)
            obj.name = f"Body_{b['id']}"
            # Set transform
            obj.location = from_json_vec(b['position'])
            obj.rotation_mode = 'QUATERNION'
            obj.rotation_quaternion = from_json_quat(b['orientation'])
            # Apply material color
            vis = b.get('visual', {})
            col = vis.get('color', {})
            mat = bpy.data.materials.new(name=obj.name + "_mat")
            r = col.get('r', 0.8)
            g = col.get('g', 0.8)
            bcol = col.get('b', 0.8)
            mat.diffuse_color = (r, g, bcol, 1.0)
            if obj.data and hasattr(obj.data, 'materials'):
                obj.data.materials.append(mat)

        self.report({'INFO'}, f"Imported {len(bodies)} bodies from {self.filepath}")
        return {'FINISHED'}

# -------------------------------------------------------------
# Registration
# -------------------------------------------------------------

def menu_func_import(self, context):
    self.layout.operator(ImportSLRBSScene.bl_idname, text="SLRBS Scene (.json)")


def register():
    bpy.utils.register_class(ImportSLRBSScene)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.utils.unregister_class(ImportSLRBSScene)


if __name__ == "__main__":
    register()
