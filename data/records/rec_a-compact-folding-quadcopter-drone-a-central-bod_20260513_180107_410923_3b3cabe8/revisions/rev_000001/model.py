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
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


# --- Drone dimensions (compact folding quadcopter, DJI-Mavic-mini scale) ---
BODY_LX = 0.110     # body length (forward axis)
BODY_LY = 0.060     # body width
BODY_LZ = 0.035     # body height
BODY_Z = 0.060      # body center height above ground (sits on skids)

ARM_LEN = 0.090     # arm tube length
ARM_R = 0.0065      # arm tube radius
MOTOR_R = 0.014     # motor pod radius
MOTOR_H = 0.020     # motor pod height
ROTOR_R = 0.062     # rotor radius
ROTOR_HUB_R = 0.010
ROTOR_THK = 0.004

# Hinge anchor in body frame (just inside body corners so the boss
# is captured by the body shell as a real mechanical pivot).
HINGE_X = BODY_LX / 2.0 - 0.008
HINGE_Y = BODY_LY / 2.0 - 0.006

# Skids
SKID_RAIL_LEN = 0.090
SKID_RAIL_THK = 0.006
SKID_LEG_THK = 0.005
SKID_LEG_H = 0.030
SKID_Y = 0.030      # half-spread of skids in y

# Camera/gimbal at front of body
GIMBAL_OFFSET_X = BODY_LX / 2.0 + 0.005
GIMBAL_Z = -BODY_LZ / 2.0 + 0.010   # near the bottom-front of body
CAM_BODY_SIZE = (0.020, 0.024, 0.020)
CAM_LENS_R = 0.0065
CAM_LENS_LEN = 0.010


def _make_rotor_mesh(name: str):
    rotor = FanRotorGeometry(
        outer_radius=ROTOR_R,
        hub_radius=ROTOR_HUB_R,
        blade_count=2,
        thickness=ROTOR_THK,
        blade_pitch_deg=14.0,
        blade_sweep_deg=6.0,
        blade=FanRotorBlade(shape="narrow", tip_pitch_deg=8.0, camber=0.05),
        hub=FanRotorHub(style="domed", bore_diameter=0.004),
    )
    return mesh_from_geometry(rotor, name)


def _add_arm_assembly(model, side_x: int, side_y: int, mat_arm, mat_motor, mat_rotor):
    """Add one arm (with motor pod) and its rotor for a corner of the body.

    side_x in {+1, -1} chooses front/rear; side_y in {+1, -1} chooses left/right
    relative to the body forward (+X) and right (+Y) axes.
    """
    if side_x > 0:
        x_tag = "front"
    else:
        x_tag = "rear"
    if side_y < 0:
        y_tag = "left"
    else:
        y_tag = "right"
    tag = f"{x_tag}_{y_tag}"

    arm_part = model.part(f"arm_{tag}")
    rotor_part = model.part(f"rotor_{tag}")

    # The arm's local +X points outward along the deployed direction.
    # In its local frame, the arm tube extends from x=0 (hinge) to x=ARM_LEN.
    # Cylinder geometry is along local +Z, so we rotate it by pitch +90° to
    # lay it along local +X.
    tube_origin = Origin(
        xyz=(ARM_LEN / 2.0, 0.0, 0.0),
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    arm_part.visual(
        Cylinder(radius=ARM_R, length=ARM_LEN),
        origin=tube_origin,
        material=mat_arm,
        name="arm_tube",
    )

    # Small hinge boss (cap at the body end of the arm). Cylinder along local +Z.
    arm_part.visual(
        Cylinder(radius=ARM_R + 0.003, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mat_arm,
        name="hinge_boss",
    )

    # Motor pod cap sits on top of arm tip; cylinder along local +Z, top-half above tube.
    arm_part.visual(
        Cylinder(radius=MOTOR_R, length=MOTOR_H),
        origin=Origin(xyz=(ARM_LEN, 0.0, MOTOR_H / 2.0 - ARM_R)),
        material=mat_motor,
        name="motor_pod",
    )
    # Tiny shaft stub poking above the motor pod (rotor mounts here)
    arm_part.visual(
        Cylinder(radius=0.002, length=0.004),
        origin=Origin(xyz=(ARM_LEN, 0.0, MOTOR_H - ARM_R + 0.002)),
        material=mat_motor,
        name="motor_shaft",
    )

    # Rotor mesh. The fan-rotor builder spins about local +Z.
    rotor_part.visual(
        _make_rotor_mesh(f"rotor_{tag}"),
        origin=Origin(),
        material=mat_rotor,
        name="rotor_disc",
    )

    # Hinge at the body corner. Hinge axis is body +Z (vertical) so arms fold
    # horizontally inward. The articulation origin includes a yaw so the
    # arm's local +X already points along the deployed diagonal.
    # Deployed direction in body frame: (side_x * cos45, side_y * sin45, 0).
    deploy_yaw = math.atan2(side_y, side_x)  # ±45° or ±135°
    body_to_arm_origin = Origin(
        xyz=(side_x * HINGE_X, side_y * HINGE_Y, BODY_Z),
        rpy=(0.0, 0.0, deploy_yaw),
    )
    # Folding axis is body +Z. Direction of fold (sign) is chosen so positive q
    # folds the arm inward toward the body. We give a symmetric stowed range.
    fold_axis = (0.0, 0.0, -1.0) if side_y > 0 else (0.0, 0.0, 1.0)
    model.articulation(
        f"body_to_arm_{tag}",
        ArticulationType.REVOLUTE,
        parent="body",
        child=f"arm_{tag}",
        origin=body_to_arm_origin,
        axis=fold_axis,
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=2.35),
    )

    # Rotor spin joint sits at the top of the motor pod. The motor shaft
    # passes up through the rotor hub bore so the rotor is supported by the
    # shaft (a real captured-shaft mount).
    rotor_origin = Origin(
        xyz=(ARM_LEN, 0.0, MOTOR_H - ARM_R + 0.002),
    )
    model.articulation(
        f"motor_to_rotor_{tag}",
        ArticulationType.CONTINUOUS,
        parent=f"arm_{tag}",
        child=f"rotor_{tag}",
        origin=rotor_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=300.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_quadcopter_drone")

    # --- Materials ---
    body_mat = model.material(
        "body_shell", rgba=(0.18, 0.20, 0.22, 1.0)
    )
    accent_mat = model.material(
        "accent", rgba=(0.85, 0.78, 0.30, 1.0)
    )
    arm_mat = model.material(
        "arm_plastic", rgba=(0.10, 0.10, 0.11, 1.0)
    )
    motor_mat = model.material(
        "motor_metal", rgba=(0.30, 0.30, 0.32, 1.0)
    )
    rotor_mat = model.material(
        "rotor_plastic", rgba=(0.78, 0.78, 0.80, 1.0)
    )
    skid_mat = model.material(
        "skid_rubber", rgba=(0.08, 0.08, 0.09, 1.0)
    )
    cam_mat = model.material(
        "camera_housing", rgba=(0.12, 0.12, 0.13, 1.0)
    )
    lens_mat = model.material(
        "camera_lens", rgba=(0.05, 0.05, 0.06, 1.0)
    )
    led_mat = model.material(
        "status_led", rgba=(0.95, 0.15, 0.10, 1.0)
    )

    # --- Body (root part) ---
    body = model.part("body")
    body.visual(
        Box((BODY_LX, BODY_LY, BODY_LZ)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z)),
        material=body_mat,
        name="body_shell",
    )
    # Top deck accent stripe (slightly proud of body top surface).
    body.visual(
        Box((BODY_LX * 0.55, BODY_LY * 0.6, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z + BODY_LZ / 2.0 + 0.002)),
        material=accent_mat,
        name="top_decal",
    )
    # Battery cover indent on the back of the body
    body.visual(
        Box((0.030, BODY_LY * 0.7, 0.004)),
        origin=Origin(
            xyz=(-BODY_LX / 2.0 + 0.016, 0.0, BODY_Z + BODY_LZ / 2.0 + 0.002)
        ),
        material=arm_mat,
        name="battery_cover",
    )
    # Small status LEDs on the rear
    body.visual(
        Sphere(radius=0.0035),
        origin=Origin(xyz=(-BODY_LX / 2.0 - 0.001, 0.012, BODY_Z + 0.002)),
        material=led_mat,
        name="led_left",
    )
    body.visual(
        Sphere(radius=0.0035),
        origin=Origin(xyz=(-BODY_LX / 2.0 - 0.001, -0.012, BODY_Z + 0.002)),
        material=led_mat,
        name="led_right",
    )
    # Antenna nubs on the rear top corners
    body.visual(
        Cylinder(radius=0.003, length=0.015),
        origin=Origin(
            xyz=(-BODY_LX / 2.0 + 0.005, BODY_LY / 2.0 - 0.004, BODY_Z + BODY_LZ / 2.0 + 0.0075),
        ),
        material=arm_mat,
        name="antenna_left",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.015),
        origin=Origin(
            xyz=(-BODY_LX / 2.0 + 0.005, -(BODY_LY / 2.0 - 0.004), BODY_Z + BODY_LZ / 2.0 + 0.0075),
        ),
        material=arm_mat,
        name="antenna_right",
    )
    # Gimbal yoke bracket attached to the front-bottom of the body.
    # It is a fixed visual on the body that the camera tilts against.
    yoke_x = BODY_LX / 2.0 + 0.006
    yoke_z = BODY_Z - BODY_LZ / 2.0 + 0.010
    body.visual(
        Box((0.012, 0.030, 0.020)),
        origin=Origin(xyz=(yoke_x, 0.0, yoke_z)),
        material=arm_mat,
        name="gimbal_yoke",
    )

    # --- Four arms + rotors ---
    for sx in (+1, -1):
        for sy in (-1, +1):
            _add_arm_assembly(model, sx, sy, arm_mat, motor_mat, rotor_mat)

    # --- Landing skids (two side rails, each fixed to body) ---
    for sign, name in ((-1, "left"), (+1, "right")):
        skid = model.part(f"skid_{name}")
        # Horizontal rail along +X local axis. Cylinder is along +Z natively,
        # so rotate to lay it along X.
        skid.visual(
            Cylinder(radius=SKID_RAIL_THK / 2.0, length=SKID_RAIL_LEN),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=skid_mat,
            name="rail",
        )
        # Two vertical legs from rail up to body bottom.
        leg_h = SKID_LEG_H
        for lx_sign, lname in ((-1, "rear_leg"), (+1, "front_leg")):
            skid.visual(
                Box((SKID_LEG_THK, SKID_LEG_THK, leg_h)),
                origin=Origin(
                    xyz=(lx_sign * (SKID_RAIL_LEN / 2.0 - 0.010), 0.0, leg_h / 2.0)
                ),
                material=skid_mat,
                name=lname,
            )
        # End caps on the rail (small rounded bumpers).
        for ex_sign, ename in ((-1, "rear_cap"), (+1, "front_cap")):
            skid.visual(
                Sphere(radius=SKID_RAIL_THK / 2.0 + 0.0005),
                origin=Origin(xyz=(ex_sign * SKID_RAIL_LEN / 2.0, 0.0, 0.0)),
                material=skid_mat,
                name=ename,
            )

        body_bottom = BODY_Z - BODY_LZ / 2.0
        # Fix the skid so its legs reach from the body bottom down to the rail.
        # The skid part frame is at the rail center. Legs extend up from frame.
        model.articulation(
            f"body_to_skid_{name}",
            ArticulationType.FIXED,
            parent="body",
            child=f"skid_{name}",
            origin=Origin(xyz=(0.0, sign * SKID_Y, body_bottom - leg_h)),
        )

    # --- Camera (tilts on horizontal Y axis at the gimbal yoke) ---
    camera = model.part("camera")
    # Camera housing centered on its own frame; lens points along local +X.
    camera.visual(
        Box(CAM_BODY_SIZE),
        origin=Origin(xyz=(CAM_BODY_SIZE[0] / 2.0, 0.0, 0.0)),
        material=cam_mat,
        name="camera_housing",
    )
    # Lens barrel
    camera.visual(
        Cylinder(radius=CAM_LENS_R, length=CAM_LENS_LEN),
        origin=Origin(
            xyz=(CAM_BODY_SIZE[0] + CAM_LENS_LEN / 2.0 - 0.001, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cam_mat,
        name="lens_barrel",
    )
    # Glass element (slightly recessed in barrel front)
    camera.visual(
        Cylinder(radius=CAM_LENS_R - 0.0015, length=0.0015),
        origin=Origin(
            xyz=(CAM_BODY_SIZE[0] + CAM_LENS_LEN - 0.001, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_mat,
        name="lens_glass",
    )
    # Side trunnion pins poking sideways so the camera reads as gimballed.
    camera.visual(
        Cylinder(radius=0.0025, length=0.006),
        origin=Origin(
            xyz=(0.002, -(CAM_BODY_SIZE[1] / 2.0 + 0.001), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=motor_mat,
        name="trunnion_left",
    )
    camera.visual(
        Cylinder(radius=0.0025, length=0.006),
        origin=Origin(
            xyz=(0.002, +(CAM_BODY_SIZE[1] / 2.0 + 0.001), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=motor_mat,
        name="trunnion_right",
    )

    # Tilt joint: revolute about world Y, located inside the yoke at the
    # camera's trunnion line. q=0 = camera horizontal (lens forward).
    model.articulation(
        "yoke_to_camera",
        ArticulationType.REVOLUTE,
        parent="body",
        child="camera",
        # Place the joint inside the gimbal yoke so the camera trunnion pins
        # are captured by the yoke (a real gimbal pivot).
        origin=Origin(xyz=(yoke_x, 0.0, yoke_z)),
        axis=(0.0, 1.0, 0.0),
        # With axis +Y, positive q pitches the lens downward (right-hand rule:
        # a point on +X rotates toward -Z). Allow a small upward tilt too.
        motion_limits=MotionLimits(
            effort=0.5, velocity=2.0, lower=-math.pi / 6.0, upper=math.pi / 2.0
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cam = object_model.get_part("camera")
    cam_tilt = object_model.get_articulation("yoke_to_camera")

    # Scoped overlap allowances: arm hinge bosses and arm-root tube sections
    # live inside the body shell because each arm pivots on a real internal
    # hinge pin captured by the body. The camera trunnions are likewise
    # captured by the gimbal yoke.
    for tag in ("front_left", "front_right", "rear_left", "rear_right"):
        ctx.allow_overlap(
            f"arm_{tag}", "body",
            elem_a="hinge_boss", elem_b="body_shell",
            reason="Arm hinge boss is captured inside the body shell by the fold pivot.",
        )
        ctx.allow_overlap(
            f"arm_{tag}", "body",
            elem_a="arm_tube", elem_b="body_shell",
            reason="Arm root section is embedded in the body shell at the fold pivot.",
        )
    ctx.allow_overlap(
        "camera", "body",
        elem_a="trunnion_left", elem_b="gimbal_yoke",
        reason="Camera trunnion pin is captured by the gimbal yoke.",
    )
    ctx.allow_overlap(
        "camera", "body",
        elem_a="trunnion_right", elem_b="gimbal_yoke",
        reason="Camera trunnion pin is captured by the gimbal yoke.",
    )
    ctx.allow_overlap(
        "camera", "body",
        elem_a="camera_housing", elem_b="gimbal_yoke",
        reason="Camera housing is seated against the gimbal yoke at the tilt pivot.",
    )
    # The motor shaft passes up through the rotor hub bore (captured shaft).
    for tag in ("front_left", "front_right", "rear_left", "rear_right"):
        ctx.allow_overlap(
            f"rotor_{tag}", f"arm_{tag}",
            elem_a="rotor_disc", elem_b="motor_shaft",
            reason="Motor shaft passes through the rotor hub bore (captured shaft mount).",
        )

    # Camera lens should point forward at rest (positive X).
    ctx.expect_gap(
        cam, body,
        axis="x",
        positive_elem="lens_barrel",
        negative_elem="body_shell",
        min_gap=0.0,
        name="camera lens sits forward of body shell",
    )

    # Tilt the camera fully down and verify the lens drops below the rest pose.
    rest_lens = ctx.part_element_world_aabb(cam, elem="lens_barrel")
    with ctx.pose({cam_tilt: math.pi / 2.0}):
        tilted_lens = ctx.part_element_world_aabb(cam, elem="lens_barrel")
    ctx.check(
        "camera tilts nose-down on positive q",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[0][2] < rest_lens[0][2] - 0.005,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    # Each rotor seats on top of its motor pod (shaft engaged through the hub).
    for tag in ("front_left", "front_right", "rear_left", "rear_right"):
        rotor = object_model.get_part(f"rotor_{tag}")
        arm = object_model.get_part(f"arm_{tag}")
        ctx.expect_contact(
            rotor, arm,
            elem_a="rotor_disc", elem_b="motor_shaft",
            contact_tol=1e-3,
            name=f"rotor_{tag} engages motor shaft",
        )

    # Verify each fold joint actually swings the arm tube when posed.
    for tag in ("front_left", "front_right", "rear_left", "rear_right"):
        arm = object_model.get_part(f"arm_{tag}")
        fold = object_model.get_articulation(f"body_to_arm_{tag}")
        rest_aabb = ctx.part_element_world_aabb(arm, elem="motor_pod")
        with ctx.pose({fold: 1.5}):
            folded_aabb = ctx.part_element_world_aabb(arm, elem="motor_pod")
        if rest_aabb is None or folded_aabb is None:
            moved = False
        else:
            rest_c = [(rest_aabb[0][i] + rest_aabb[1][i]) * 0.5 for i in range(3)]
            folded_c = [(folded_aabb[0][i] + folded_aabb[1][i]) * 0.5 for i in range(3)]
            moved = any(abs(folded_c[i] - rest_c[i]) > 0.010 for i in range(3))
        ctx.check(
            f"arm_{tag} fold joint swings the motor pod",
            moved,
            details=f"rest={rest_aabb}, folded={folded_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
