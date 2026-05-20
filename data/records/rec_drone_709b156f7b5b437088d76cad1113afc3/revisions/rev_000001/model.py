from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# --- Top-level dimensions (compact folding quadcopter, ~Mavic Air class) ---
BODY_L = 0.140  # along X (front-back)
BODY_W = 0.072  # along Y (side-to-side)
BODY_H = 0.034  # along Z (top-bottom)
BODY_Z = 0.060  # body center height above ground

# Body upper canopy (smaller footprint so corner knuckles stay clear)
CANOPY_L = 0.082
CANOPY_W = 0.034
CANOPY_H = 0.012

# Battery hatch trim on canopy top (narrow strip)
HATCH_L = 0.060
HATCH_W = 0.026
HATCH_H = 0.003

# Arm hinge locations on the body (corners). The hinge sits a few mm above
# body top, supported by a short hinge post on the body.
HINGE_DX = 0.062
HINGE_DY = 0.030
HINGE_POST_H = 0.006  # vertical post height above body top
HINGE_DZ = BODY_H / 2 + HINGE_POST_H / 2  # post mid-height
HINGE_DZ_TOP = BODY_H / 2 + HINGE_POST_H  # joint origin (top of post)

# Arm geometry: cylinder boom + motor pod at tip. Arm-frame z=0 is the joint
# axis; boom and motor are lifted by ARM_Z_OFFSET so the arm sits above body.
ARM_LEN = 0.110  # hinge to motor center
ARM_R = 0.0070
ARM_Z_OFFSET = ARM_R + 0.001
ROOT_HOUSING_R = 0.0120  # hinge knuckle around the joint axis
MOTOR_R = 0.0125
MOTOR_H = 0.020

# Rotor (small folding-drone-class prop)
ROTOR_OR = 0.040  # outer radius (~80mm prop)
ROTOR_HUB_R = 0.006
ROTOR_THICK = 0.0035

# Camera gimbal
CAM_BODY_L = 0.030
CAM_BODY_W = 0.024
CAM_BODY_H = 0.020
CAM_LENS_R = 0.0085
CAM_LENS_L = 0.006
CAM_TILT_AXIS_X = BODY_L / 2 + 0.018  # in front of the body face, on the yoke tip
CAM_TILT_AXIS_Z = -BODY_H / 2 - 0.003  # below the body, hanging from the yoke

# Landing skids (two side rails with vertical struts)
SKID_RAIL_LEN = 0.090
SKID_RAIL_R = 0.0050
SKID_Y = 0.038
SKID_RAIL_Z = 0.005  # rail center height (top of rail at z=0.010)
SKID_STRUT_DX = 0.030
SKID_STRUT_R = 0.0035

# Per-corner deploy yaw (X-config, deployed pose at q=0)
ARMS = [
    # name,              corner xyz (in body frame),         deploy_yaw,   fold_axis_z
    ("front_left_arm",  ( HINGE_DX,  HINGE_DY, HINGE_DZ),  math.pi / 4,    1.0),
    ("front_right_arm", ( HINGE_DX, -HINGE_DY, HINGE_DZ), -math.pi / 4,   -1.0),
    ("rear_left_arm",   (-HINGE_DX,  HINGE_DY, HINGE_DZ),  3 * math.pi / 4, -1.0),
    ("rear_right_arm",  (-HINGE_DX, -HINGE_DY, HINGE_DZ), -3 * math.pi / 4, 1.0),
]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_quadcopter")

    # --- Materials ---
    shell = model.material("shell_dark", rgba=(0.16, 0.17, 0.20, 1.0))
    canopy = model.material("canopy_gray", rgba=(0.22, 0.23, 0.27, 1.0))
    accent = model.material("accent_gold", rgba=(0.78, 0.66, 0.32, 1.0))
    motor_metal = model.material("motor_silver", rgba=(0.62, 0.63, 0.66, 1.0))
    rotor_mat = model.material("rotor_black", rgba=(0.08, 0.08, 0.09, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.05, 0.08, 0.16, 1.0))
    lens_ring = model.material("lens_ring", rgba=(0.30, 0.30, 0.33, 1.0))
    indicator = model.material("led_red", rgba=(0.85, 0.10, 0.12, 1.0))

    # ---------------- BODY ----------------
    body = model.part("body")

    # Main body shell (lower hull)
    body.visual(
        Box((BODY_L, BODY_W, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z)),
        material=shell,
        name="body_hull",
    )
    # Upper canopy (smaller stacked block, keeps corner hinges clear)
    body.visual(
        Box((CANOPY_L, CANOPY_W, CANOPY_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z + BODY_H / 2 + CANOPY_H / 2)),
        material=canopy,
        name="body_canopy",
    )
    # Battery hatch trim on canopy top
    body.visual(
        Box((HATCH_L, HATCH_W, HATCH_H)),
        origin=Origin(
            xyz=(-0.005, 0.0, BODY_Z + BODY_H / 2 + CANOPY_H + HATCH_H / 2)
        ),
        material=accent,
        name="battery_hatch",
    )
    # Rear status LED
    body.visual(
        Sphere(radius=0.0035),
        origin=Origin(xyz=(-BODY_L / 2 + 0.004, 0.0, BODY_Z)),
        material=indicator,
        name="rear_led",
    )

    # Hinge posts at each corner of the body top: short cylinders that lift
    # the arm pivot above the body top so the arm clears the hull.
    for arm_name, (cx, cy, _cz), _yaw, _fz in ARMS:
        post_name = arm_name.replace("_arm", "_hinge_post")
        body.visual(
            Cylinder(radius=ROOT_HOUSING_R + 0.0010, length=HINGE_POST_H),
            origin=Origin(
                xyz=(cx, cy, BODY_Z + BODY_H / 2 + HINGE_POST_H / 2)
            ),
            material=shell,
            name=post_name,
        )

    # Front camera mount yoke: bridges body front face to the gimbal tilt axis.
    YOKE_BACK_X = BODY_L / 2 - 0.006  # buried 6mm into body for solid joint
    YOKE_FRONT_X = CAM_TILT_AXIS_X      # touches the tilt axis
    YOKE_LEN = YOKE_FRONT_X - YOKE_BACK_X
    YOKE_CX = (YOKE_BACK_X + YOKE_FRONT_X) / 2
    body.visual(
        Box((YOKE_LEN, 0.026, 0.014)),
        origin=Origin(
            xyz=(YOKE_CX, 0.0, BODY_Z - BODY_H / 2 - 0.003)
        ),
        material=shell,
        name="camera_yoke",
    )

    # Landing skid struts (4) and rails (2), fixed to body
    for sx in (-SKID_STRUT_DX, SKID_STRUT_DX):
        for sy in (-SKID_Y, SKID_Y):
            strut_z_top = BODY_Z - BODY_H / 2  # body bottom in world
            strut_z_bot = SKID_RAIL_Z + SKID_RAIL_R  # top of rail
            strut_len = strut_z_top - strut_z_bot
            strut_center_z = (strut_z_top + strut_z_bot) / 2
            tag_x = "front" if sx > 0 else "rear"
            tag_y = "left" if sy > 0 else "right"
            body.visual(
                Cylinder(radius=SKID_STRUT_R, length=strut_len),
                origin=Origin(xyz=(sx, sy, strut_center_z)),
                material=shell,
                name=f"skid_strut_{tag_x}_{tag_y}",
            )
    for sy in (-SKID_Y, SKID_Y):
        tag_y = "left" if sy > 0 else "right"
        body.visual(
            Cylinder(radius=SKID_RAIL_R, length=SKID_RAIL_LEN),
            origin=Origin(
                xyz=(0.0, sy, SKID_RAIL_Z),
                rpy=(0.0, math.pi / 2, 0.0),  # rotate cylinder along +X
            ),
            material=shell,
            name=f"skid_rail_{tag_y}",
        )

    # ---------------- ARMS, MOTORS, ROTORS ----------------
    for arm_name, (cx, cy, cz), deploy_yaw, fold_axis_z in ARMS:
        arm = model.part(arm_name)

        # Boom: cylinder along arm-local +X, lifted above the body top.
        arm.visual(
            Cylinder(radius=ARM_R, length=ARM_LEN),
            origin=Origin(
                xyz=(ARM_LEN / 2, 0.0, ARM_Z_OFFSET),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=shell,
            name=f"{arm_name}_boom",
        )
        # Small hinge knuckle on the arm wrapping the post (joint axis).
        arm.visual(
            Cylinder(radius=ROOT_HOUSING_R - 0.0005, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, ARM_Z_OFFSET / 2 + 0.001)),
            material=canopy,
            name=f"{arm_name}_knuckle",
        )
        # Motor pod at tip (silver cylinder, axis +Z)
        arm.visual(
            Cylinder(radius=MOTOR_R, length=MOTOR_H),
            origin=Origin(
                xyz=(ARM_LEN, 0.0, ARM_Z_OFFSET + MOTOR_H / 2 - 0.001)
            ),
            material=motor_metal,
            name=f"{arm_name}_motor_pod",
        )
        # Motor base ring (small dark plate where motor meets boom)
        arm.visual(
            Cylinder(radius=MOTOR_R + 0.0015, length=0.003),
            origin=Origin(xyz=(ARM_LEN, 0.0, ARM_Z_OFFSET + 0.0015)),
            material=shell,
            name=f"{arm_name}_motor_base",
        )

        # Body -> arm hinge (revolute, rotates about Z; fold inward over body).
        # Origin sits on top of the hinge post so the arm clears the body hull.
        model.articulation(
            f"{arm_name}_fold",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(
                xyz=(cx, cy, BODY_Z + HINGE_DZ_TOP),
                rpy=(0.0, 0.0, deploy_yaw),
            ),
            axis=(0.0, 0.0, fold_axis_z),
            motion_limits=MotionLimits(
                effort=2.0, velocity=4.0, lower=0.0, upper=math.pi / 2
            ),
        )

        # Rotor child part
        rotor_name = arm_name.replace("_arm", "_rotor")
        rotor = model.part(rotor_name)
        rotor.visual(
            mesh_from_geometry(
                FanRotorGeometry(
                    ROTOR_OR,
                    ROTOR_HUB_R,
                    2,  # 2-blade prop
                    thickness=ROTOR_THICK,
                    blade_pitch_deg=18.0,
                    blade_sweep_deg=22.0,
                    blade=FanRotorBlade(shape="narrow", camber=0.10, tip_pitch_deg=10.0),
                    hub=FanRotorHub(style="domed"),
                ),
                rotor_name,
            ),
            # Embed by 0.5mm into the motor pod for guaranteed contact.
            origin=Origin(xyz=(0.0, 0.0, ROTOR_THICK / 2 - 0.0005)),
            material=rotor_mat,
            name=f"{rotor_name}_disc",
        )

        # Rotor spin axis (continuous, about Z at top of motor pod). Motor top
        # in arm frame is z = ARM_Z_OFFSET + MOTOR_H - 0.001.
        model.articulation(
            f"{rotor_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=rotor,
            origin=Origin(
                xyz=(ARM_LEN, 0.0, ARM_Z_OFFSET + MOTOR_H - 0.001)
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=200.0),
        )

    # ---------------- CAMERA GIMBAL ----------------
    camera = model.part("camera_gimbal")
    # Camera body (back face at tilt axis, extends forward from there)
    camera.visual(
        Box((CAM_BODY_L, CAM_BODY_W, CAM_BODY_H)),
        origin=Origin(xyz=(CAM_BODY_L / 2, 0.0, 0.0)),
        material=shell,
        name="camera_body",
    )
    # Lens ring just ahead of camera body, slightly overlapping it
    LENS_RING_LEN = 0.004
    camera.visual(
        Cylinder(radius=CAM_LENS_R + 0.0015, length=LENS_RING_LEN),
        origin=Origin(
            xyz=(CAM_BODY_L + LENS_RING_LEN / 2 - 0.001, 0.0, 0.0),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=lens_ring,
        name="camera_lens_ring",
    )
    # Lens glass nested into the ring (tip protrudes forward)
    camera.visual(
        Cylinder(radius=CAM_LENS_R, length=CAM_LENS_L),
        origin=Origin(
            xyz=(CAM_BODY_L + LENS_RING_LEN + CAM_LENS_L / 2 - 0.0025, 0.0, 0.0),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=lens_glass,
        name="camera_lens",
    )

    # Body -> camera tilt (revolute, axis along Y)
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=camera,
        origin=Origin(
            xyz=(CAM_TILT_AXIS_X, 0.0, BODY_Z + CAM_TILT_AXIS_Z)
        ),
        # +q = lens pitches downward (positive rotation about +Y rotates +X toward -Z)
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=2.0,
            lower=-math.pi / 6,   # ~30° up
            upper=math.pi / 2.2,  # ~82° down
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    camera = object_model.get_part("camera_gimbal")

    # Each rotor sits above its arm motor pod and clears the body
    for prefix in ("front_left", "front_right", "rear_left", "rear_right"):
        arm = object_model.get_part(f"{prefix}_arm")
        rotor = object_model.get_part(f"{prefix}_rotor")
        ctx.expect_gap(
            rotor,
            arm,
            axis="z",
            max_penetration=0.001,
            name=f"{prefix}_rotor_seated_on_motor",
        )
        # Rotors do not collide with the body in the deployed pose
        ctx.expect_origin_distance(
            rotor,
            body,
            axes="xy",
            min_dist=0.10,
            name=f"{prefix}_rotor_clear_of_body",
        )

    # Camera lens projects in front of the body in the rest pose
    ctx.expect_gap(
        camera,
        body,
        axis="x",
        min_gap=0.0,
        positive_elem="camera_lens",
        name="camera_lens_in_front_of_body",
    )

    # Folding mechanism: at full fold, front-left arm tip should swing rearward
    front_left_fold = object_model.get_articulation("front_left_arm_fold")
    front_left_arm = object_model.get_part("front_left_arm")

    rest_pos = ctx.part_world_aabb(front_left_arm)
    with ctx.pose({front_left_fold: math.pi / 2}):
        folded_pos = ctx.part_world_aabb(front_left_arm)
    ctx.check(
        "front_left_arm_folds_inward",
        rest_pos is not None
        and folded_pos is not None
        and folded_pos[1][0] < rest_pos[1][0] - 0.03,
        details=f"rest_max_x={rest_pos and rest_pos[1][0]}, folded_max_x={folded_pos and folded_pos[1][0]}",
    )

    # Camera tilt actually pitches the lens
    cam_tilt = object_model.get_articulation("camera_tilt")
    rest_lens = ctx.part_element_world_aabb(camera, elem="camera_lens")
    with ctx.pose({cam_tilt: math.pi / 3}):
        tilted_lens = ctx.part_element_world_aabb(camera, elem="camera_lens")
    ctx.check(
        "camera_tilts_downward",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[0][2] < rest_lens[0][2] - 0.005,
        details=f"rest_min_z={rest_lens and rest_lens[0][2]}, tilted_min_z={tilted_lens and tilted_lens[0][2]}",
    )

    return ctx.report()


object_model = build_object_model()
