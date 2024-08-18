bl_info = {
"name": "TvC Animation Importer",
"description":"For importing Animation files.",
"author":"Original by ploaj, ported by Eternal Yoshi",
"version":(0,1,0),
"blender":(3,0,0),
"location": "File > Import",
"category":"Import-Export",
}

import struct
from typing import List
import bpy
import math
import os
import re
import xml.etree.ElementTree as ET
from math import atan2, ceil, cos, degrees, floor, isclose, pi, radians, sin,tan
from bpy_extras.io_utils import ImportHelper

# Joint Type Flags
JOINT_TRANSLATE     = (1 << 0) # 0001
JOINT_SCALE         = (1 << 1) # 0002
JOINT_ROTATE_EULER  = (1 << 2) # 0004
JOINT_ROTATE_AXIS   = (1 << 3) # 0008
JOINT_UNKNOWN_0010  = (1 << 4) # 0010
JOINT_ENABLED       = (1 << 5) # 0020
JOINT_DISABLED      = (1 << 6) # 0040
JOINT_UNKNOWN_0080  = (1 << 7) # 0080

JOINT_UNKNOWN_0100  = (1 << 8) # 0100
JOINT_UNKNOWN_0200  = (1 << 9) # 0200
JOINT_UNKNOWN_0400  = (1 << 10) # 0400
JOINT_UNKNOWN_0800  = (1 << 11) # 0800
JOINT_UNKNOWN_1000  = (1 << 12) # 1000
JOINT_UNKNOWN_2000  = (1 << 13) # 2000
JOINT_UNKNOWN_4000  = (1 << 14) # 4000
JOINT_UNKNOWN_8000  = (1 << 15) # 8000

class MOTKey:
    time: float
    x: float
    y: float
    z: float
    w: float
    
    def __init__(self):
        self.time = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0
        return

class MOTJoint:
    flag1: int
    flag2: int
    type: int
    bone_id: int

    max_time: float
    unknown: int

    keys: List[MOTKey]
    
    def __init__(self):
        self.flag1 = 0
        self.flag2 = 0
        self.type = 0
        self.bone_id = 0
        self.max_time = 0.0
        self.unknown = 0
        self.keys = []
        return

    def parse(self, file, start):
        header = struct.unpack('>bbhhhfiiiii', file.read(0x20))
        self.flag1 = header[0]
        self.flag2 = header[1]
        self.type = header[2]
        self.bone_id = header[3]
        float_count = header[4]
        self.max_time = header[5]
        self.unknown = header[6]
        time_offset = header[7]
        offset2 = header[8]
        offset3 = header[9]
        offset4 = header[10]

        # print(f"{self.flag1} {self.flag2} {self.type:#2x} {self.bone_id} {self.max_time}")

        assert self.flag1 == 2 or self.flag1 == 0, "Unknown Flag1 Type"
        assert self.flag2 == 4 or self.flag2 == 2 or self.flag2 == 0, f"Unknown Flag2 Type {self.flag2}"
        assert self.unknown == 0, "Unknown section not supported"
        assert offset3 == 0, "Section 3 not supported"
        assert offset4 == 0, "Section 3 not supported"

        temp = file.tell()
        for i in range(0,float_count):
            key = MOTKey()

            file.seek(start + time_offset + 4 * i)
            # print(f"{file.tell():#8x} {start:#8x} {time_offset:#8x}")
            key.time = struct.unpack('>f', file.read(4))

            if offset2 != 0:
                file.seek(start + offset2 + 8 * i)
                key.x = struct.unpack('>f', file.read(2) + bytes([0, 0]))
                key.y = struct.unpack('>f', file.read(2) + bytes([0, 0]))
                key.z = struct.unpack('>f', file.read(2) + bytes([0, 0]))
                key.w = struct.unpack('>f', file.read(2) + bytes([0, 0]))

            self.keys.append(key)

        file.seek(temp)

        return

class MOTEntry:
    playback_speed: float
    end_time: float
    joints: List[MOTJoint]
    
    def __init__(self):
        self.playback_speed = 1.0
        self.end_time = 1.0
        self.joints = []
        return
    
    def length(self):
        return int((self.playback_speed / self.end_time) * 100)

    def parse(self, file):
        start = file.tell()
        header = struct.unpack('>iiff', file.read(0x10))
        self.playback_speed = header[2]
        self.end_time = header[3]

        assert header[1] == 0x10, f"MOT Entry has unexpected size {header[1]}"

        for i in range(0, header[0]):
            joint = MOTJoint()
            joint.parse(file, start)
            self.joints.append(joint)

        return

class MOTFile:
    unknown: int
    entries: List[MOTEntry]

    def __init__(self):
        self.unknown = 0
        self.entries = []
        return

    def parse(self, file):
        header = struct.unpack('>iiii', file.read(0x10))
        self.unknown = header[0]

        for i in range(0, header[1]):
            file.seek(0x10 + i * 4)
            offset = struct.unpack('>i', file.read(4))[0]

            if offset == 0:
                self.entries.append(None)
            else:
                entry = MOTEntry()
                file.seek(offset)
                entry.parse(file)
                self.entries.append(entry)
        return

def parse_joint_table(file_path):
    with open(file_path, 'rb') as file:
        header = struct.unpack('>iiii', file.read(0x10))

        assert header[0] == 8, "Unsupported or Invalid JCV header"
        assert header[1] == 0, "Unsupported or Invalid JCV header"

        file.seek(header[2])
        count1 = struct.unpack('>h', file.read(2))[0]
        return struct.unpack('>' + 'h' * count1, file.read(count1 * 2))

        # file.seek(header[3])
        # count2 = struct.unpack('>h', file.read(2))

def parse_mot_file(self, context, filepath, read_SaveFakeUser):
    with open(filepath, 'rb') as file:
        mot = MOTFile()
        mot.parse(file)
    
    for i in range(0, len(mot.entries)):
        if not mot.entries[i] is None:
            import_mot_action(self, context, f"MOT_{i:#03d}", mot.entries[i], 60, read_SaveFakeUser)


def bpy_reset_scene(context):
    prev_auto = context.scene.tool_settings.use_keyframe_insert_auto
    context.scene.tool_settings.use_keyframe_insert_auto = False
    
    bpy.ops.object.mode_set(mode='POSE')
    
    bpy.ops.pose.select_all(action='SELECT')
    bpy.ops.pose.loc_clear()
    bpy.ops.pose.rot_clear()
    bpy.ops.pose.scale_clear()
    
    context.scene.tool_settings.use_keyframe_insert_auto = prev_auto

def create_key(curve, time, value):
    kf = curve.keyframe_points.insert(frame = time, value = value)
    kf.interpolation = 'LINEAR'
    return kf

def get_bone_name(context, bone_id):
    bnames = {
        0 : "Y",
        1 : "X",
        2 : "Z",
        3 : "hip",
        4 : "waist",
        5 : "breast",
        6 : "neck",
        7 : "head",
        8 : "L_collar",
        9 : "L_harm",
        10 : "L_larm",
        11 : "L_hand",
        12 : "R_collar",
        13 : "R_harm",
        14 : "R_larm",
        15 : "R_hand",
        16 : "L_hleg",
        17 : "L_lleg",
        18 : "L_foot",
        19 : "R_hleg",
        20 : "R_lleg",
        21 : "R_foot",
        30 : "L_harmEX",
        31 : "R_harmEX",
        32 : "L_hlegEX",
        33 : "R_hlegEX",
        50 : "L_toe",
        51 : "R_toe",
        59 : "ago",
        60 : "mouth_UNDER",
        61 : "mouth_TOP",
        62 : "mouth_R",
        63 : "mouth_L",
        100 : "Lhand00",
        101 : "Lhand01",
        102 : "Lhand02",
        103 : "Lhand03",
        104 : "Lhand04",
        105 : "Lhand05",
        106 : "Lhand06",
        107 : "Lhand07",
        108 : "Lhand08",
        109 : "Lhand09",
        110 : "W_Lhand00",
        111 : "W_Lhand01",
        112 : "W_Lhand02",
        113 : "W_Lhand03",
        114 : "W_Lhand04",
        115 : "W_Lhand05",
        116 : "W_Lhand06",
        117 : "W_Lhand07",
        118 : "W_Lhand08",
        119 : "W_Lhand09",
        120 : "W2_Lhand00",
        121 : "W2_Lhand01",
        122 : "W2_Lhand02",
        123 : "W2_Lhand03",
        124 : "W2_Lhand04",
        125 : "W2_Lhand05",
        126 : "W2_Lhand06",
        127 : "W2_Lhand07",
        128 : "W2_Lhand08",
        129 : "W2_Lhand09",
        130 : "Rhand00",
        131 : "Rhand01",
        132 : "Rhand02",
        133 : "Rhand03",
        134 : "Rhand04",
        135 : "Rhand05",
        136 : "Rhand06",
        137 : "Rhand07",
        138 : "Rhand08",
        139 : "Rhand09",
        140 : "W_Rhand00",
        141 : "W_Rhand01",
        142 : "W_Rhand02",
        143 : "W_Rhand03",
        144 : "W_Rhand04",
        145 : "W_Rhand05",
        146 : "W_Rhand06",
        147 : "W_Rhand07",
        148 : "W_Rhand08",
        149 : "W_Rhand09",
        150 : "W2_Rhand00",
        151 : "W2_Rhand01",
        152 : "W2_Rhand02",
        153 : "W2_Rhand03",
        154 : "W2_Rhand04",
        155 : "W2_Rhand05",
        156 : "W2_Rhand06",
        157 : "W2_Rhand07",
        158 : "W2_Rhand08",
        159 : "W2_Rhand09",
        160 : "J_160",
        161 : "J_161",
        162 : "J_162",
        163 : "J_163",
        164 : "J_164",
        165 : "J_165",
        166 : "J_166",
        167 : "J_167",
        168 : "J_168",
        169 : "J_169",
    }

#   Check if bone name in lookup table and in skeleton
    if bone_id in bnames:
        if bnames[bone_id] in context.object.data.bones:
            return context.object.data.bones[bnames[bone_id]]
    
#   Search for bone number
    bone_num = f"{bone_id:#03d}"
    for bone in context.object.data.bones:
        if bone_num in bone.name:
            return bone;
    
    print(f"Bone {bone_id} not found")
    return None

def import_mot_action(self, context, filepath, anim, fps, read_SaveFakeUser):
    bpy_reset_scene(context) # reset scene
    
    bpy.context.scene.render.fps = fps

    #name = os.path.join(os.path.dirname(filepath), AnimYAML.name)
    action = context.blend_data.actions.new(filepath)

    if context.active_object.animation_data is None:
        context.active_object.animation_data_create()

    context.active_object.animation_data.action = action
    

    anim_length = int(fps * anim.end_time)
    
#   TODO: get correct start and end times
    context.scene.frame_preview_start = 1
    context.scene.frame_preview_end = anim_length
    context.scene.use_preview_range = True
    
    
    for j in anim.joints:
#       check if joint has bone index
        if j.bone_id == -1:
            continue
        
#       Check if joint enabled
        if not j.type & JOINT_ENABLED:
            continue
        
#       Get targeted bone
        bone = get_bone_name(context, j.bone_id)
        
        if bone is None:
            print(f"Bone {j.bone_id} not found")
            continue
        
#       Make sure action group exists
        if bone.name not in action.groups:
            anim_group = action.groups.new(bone.name)
            anim_group.name = bone.name
            
#       Get pose bone and convert to axis angle
        pose_bone = context.object.pose.bones[bone.name]
        pose_bone.rotation_mode = 'XYZ'
    
#       Create translation
        if j.type & JOINT_TRANSLATE:
            curve_x = action.fcurves.new(f'pose.bones[\"{bone.name}\"].location', index = 0, action_group = bone.name)
            curve_y = action.fcurves.new(f'pose.bones[\"{bone.name}\"].location', index = 1, action_group = bone.name)
            curve_z = action.fcurves.new(f'pose.bones[\"{bone.name}\"].location', index = 2, action_group = bone.name)
            
            for key in j.keys:
                time = key.time[0] * fps
                create_key(curve_x, time, key.x[0] / 10)
                create_key(curve_y, time, key.y[0] / 10)
                create_key(curve_z, time, key.z[0] / 10)
    
#       Create rotation euler
        if j.type & JOINT_ROTATE_EULER:
            curve_x = action.fcurves.new(f'pose.bones[\"{bone.name}\"].rotation_euler', index = 0, action_group = bone.name)
            curve_y = action.fcurves.new(f'pose.bones[\"{bone.name}\"].rotation_euler', index = 1, action_group = bone.name)
            curve_z = action.fcurves.new(f'pose.bones[\"{bone.name}\"].rotation_euler', index = 2, action_group = bone.name)
            
            for key in j.keys:
                time = key.time[0] * fps
                create_key(curve_x, time, key.x[0])
                create_key(curve_y, time, key.y[0])
                create_key(curve_z, time, key.z[0])
                
#       Create rotation axis
        if j.type & JOINT_ROTATE_AXIS:
            curve_x = action.fcurves.new(f'pose.bones[\"{bone.name}\"].rotation_euler', index = 0, action_group = bone.name)
            curve_y = action.fcurves.new(f'pose.bones[\"{bone.name}\"].rotation_euler', index = 1, action_group = bone.name)
            curve_z = action.fcurves.new(f'pose.bones[\"{bone.name}\"].rotation_euler', index = 2, action_group = bone.name)
            
            for key in j.keys:
                time = key.time[0] * fps
                create_key(curve_z, time, math.atan2(key.y[0], key.x[0]))
                length = math.sqrt(key.y[0] * key.y[0] + key.x[0] * key.x[0])
                create_key(curve_y, time, math.atan2(-key.z[0], length))
                create_key(curve_x, time, math.radians(key.w[0]))
        
#       Create scale
        if j.type & JOINT_SCALE:
            curve_x = action.fcurves.new(f'pose.bones[\"{bone.name}\"].scale', index = 0, action_group = bone.name)
            curve_y = action.fcurves.new(f'pose.bones[\"{bone.name}\"].scale', index = 1, action_group = bone.name)
            curve_z = action.fcurves.new(f'pose.bones[\"{bone.name}\"].scale', index = 2, action_group = bone.name)
            
            for key in j.keys:
                time = key.time[0] * fps
                create_key(curve_x, time, key.x[0])
                create_key(curve_y, time, key.y[0])
                create_key(curve_z, time, key.z[0])
                
        
    # Create dummy data at frame -1
    frame = -1
    
    # Insert keyframes for all pose bones
    for pose_bone in context.object.pose.bones:
        # Set pose bone properties
        pose_bone.location = pose_bone.location
        pose_bone.rotation_euler = pose_bone.rotation_euler
        pose_bone.scale = pose_bone.scale
        
        # Insert keyframes for location, rotation, and scale
        pose_bone.keyframe_insert(data_path='location', frame=frame)
        pose_bone.keyframe_insert(data_path='rotation_euler', frame=frame)
        pose_bone.keyframe_insert(data_path='scale', frame=frame)

    if self.read_SaveFakeUser is True:
        action.use_fake_user = True    

#def import_mot(bpy.types.Operator, ImportHelper):
#    mot = parse_mot_file(mot_path)
#    
#    fps = 60
#    for i in range(0, len(mot.entries)):
#        if not mot.entries[i] is None:
 #           import_mot_action(context, f"MOT_{i:#03d}", mot.entries[i], 60)
#            break

class TVCMOT_Import_Handler(bpy.types.Operator, ImportHelper):
    """Imports animation data from TvC Anim files"""
    bl_idname = ("screen.mot_import")
    bl_label = ("TvC Anim Import")
    bl_options = {'PRESET','UNDO'}
    
    #Filters out non .mot files.
    filename_ext = ".mot"
    filter_glob: bpy.props.StringProperty(default="*.mot", options={'HIDDEN'})
    files: bpy.props.CollectionProperty(type=bpy.types.OperatorFileListElement)
    
    read_SaveFakeUser: bpy.props.BoolProperty(
        name = "Import With Fake User",
        description="Imports Each Animation with a saved fake user. Allows the animations to persist when saving and closing the scene after importing.",
        default=False,
    )
    
    def execute(self, context):
        keywords = self.as_keywords(ignore=("filter_glob","files",))
        parse_mot_file(self, context, **keywords)
        context.view_layer.update()
        return {"FINISHED"}
    
    #Checks if object selected is an armature.
    @classmethod
    def poll(self, context):
        if context.active_object is not None:
            if (context.active_object.type == 'ARMATURE'):
                return True
        return False    

#mot_path = '......\\0000.mot'
#import_mot(bpy.context, mot_path)

#Menu Things. These allow the plugin to show up in the import menu.
def menu_func_import(self, context):
    self.layout.operator(TVCMOT_Import_Handler.bl_idname, text="Tatsunoko vs Cacpcom Animation (.mot)")

def register():
    bpy.utils.register_class(TVCMOT_Import_Handler)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)

def unregister():
    bpy.utils.unregister_class(TVCMOT_Import_Handler)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)

if __name__ == "__main__":
    register()