from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HINGE_BARREL_RADIUS = 0.014
HINGE_BARREL_LENGTH = 0.028
HINGE_EAR_THICKNESS = 0.006
HINGE_EAR_CENTER_Y = HINGE_BARREL_LENGTH * 0.5 + HINGE_EAR_THICKNESS * 0.5


def _add_clevis(
    part,
    *,
    joint_x: float,
    joint_z: float = 0.0,
    ear_height: float,
    ear_depth: float,
    material,
    name_prefix: str,
) -> None:
    part.visual(
        Box((ear_depth, HINGE_EAR_THICKNESS, ear_height)),
        origin=Origin(xyz=(joint_x, HINGE_EAR_CENTER_Y, joint_z)),
        material=material,
        name=f"{name_prefix}_left_ear",
    )
    part.visual(
        Box((ear_depth, HINGE_EAR_THICKNESS, ear_height)),
        origin=Origin(xyz=(joint_x, -HINGE_EAR_CENTER_Y, joint_z)),
        material=material,
        name=f"{name_prefix}_right_ear",
    )


def _visual_center(ctx: TestContext, part, elem: str) -> tuple[float, float, float] | None:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_clamp_task_lamp")

    painted_steel = model.material("painted_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    graphite = model.material("graphite", rgba=(0.13, 0.14, 0.15, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.86, 0.92, 0.97, 0.35))
    warm_led = model.material("warm_led", rgba=(0.96, 0.92, 0.78, 0.55))

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        Box((0.028, 0.048, 0.165)),
        origin=Origin(xyz=(-0.055, 0.0, 0.1125)),
        material=painted_steel,
        name="back_spine",
    )
    clamp_base.visual(
        Box((0.110, 0.048, 0.022)),
        origin=Origin(xyz=(-0.005, 0.0, 0.188)),
        material=painted_steel,
        name="upper_jaw",
    )
    clamp_base.visual(
        Box((0.060, 0.048, 0.020)),
        origin=Origin(xyz=(-0.025, 0.0, 0.046)),
        material=painted_steel,
        name="lower_rear_jaw",
    )
    clamp_base.visual(
        Box((0.018, 0.048, 0.020)),
        origin=Origin(xyz=(0.008, 0.0, 0.046)),
        material=painted_steel,
        name="lower_front_neck",
    )
    clamp_base.visual(
        Box((0.022, 0.048, 0.032)),
        origin=Origin(xyz=(0.020, 0.0, 0.060)),
        material=painted_steel,
        name="screw_bridge",
    )
    clamp_base.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.020, 0.0, 0.088)),
        material=painted_steel,
        name="screw_boss",
    )
    clamp_base.visual(
        Cylinder(radius=0.0085, length=0.175),
        origin=Origin(xyz=(0.020, 0.0, 0.073)),
        material=satin_aluminum,
        name="clamp_screw",
    )
    clamp_base.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.020, 0.0, 0.157)),
        material=rubber,
        name="swivel_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.005, length=0.072),
        origin=Origin(
            xyz=(0.020, 0.0, -0.012),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_aluminum,
        name="screw_handle",
    )
    clamp_base.visual(
        Box((0.026, 0.036, 0.005)),
        origin=Origin(xyz=(0.025, 0.0, 0.1745)),
        material=rubber,
        name="upper_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(-0.020, 0.0, 0.219)),
        material=painted_steel,
        name="pivot_socket",
    )
    clamp_base.visual(
        Box((0.022, 0.046, 0.024)),
        origin=Origin(xyz=(-0.020, 0.0, 0.230)),
        material=painted_steel,
        name="shoulder_pedestal",
    )
    _add_clevis(
        clamp_base,
        joint_x=-0.020,
        joint_z=0.258,
        ear_height=0.050,
        ear_depth=0.024,
        material=painted_steel,
        name_prefix="shoulder",
    )
    clamp_base.inertial = Inertial.from_geometry(
        Box((0.140, 0.080, 0.285)),
        mass=2.8,
        origin=Origin(xyz=(-0.010, 0.0, 0.120)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="shoulder_barrel",
    )
    lower_arm.visual(
        Box((0.040, 0.032, 0.028)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=graphite,
        name="shoulder_hub",
    )
    lower_arm.visual(
        Box((0.292, 0.034, 0.018)),
        origin=Origin(xyz=(0.164, 0.0, 0.0)),
        material=satin_aluminum,
        name="main_beam",
    )
    lower_arm.visual(
        Box((0.180, 0.020, 0.010)),
        origin=Origin(xyz=(0.150, 0.0, -0.014)),
        material=graphite,
        name="cable_channel",
    )
    lower_arm.visual(
        Box((0.032, 0.036, 0.032)),
        origin=Origin(xyz=(0.322, 0.0, 0.0)),
        material=graphite,
        name="elbow_block",
    )
    _add_clevis(
        lower_arm,
        joint_x=0.350,
        ear_height=0.050,
        ear_depth=0.024,
        material=graphite,
        name_prefix="elbow",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.370, 0.070, 0.070)),
        mass=0.95,
        origin=Origin(xyz=(0.185, 0.0, 0.0)),
    )

    middle_arm = model.part("middle_arm")
    middle_arm.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_barrel",
    )
    middle_arm.visual(
        Box((0.040, 0.032, 0.028)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=graphite,
        name="elbow_hub",
    )
    middle_arm.visual(
        Box((0.252, 0.032, 0.018)),
        origin=Origin(xyz=(0.144, 0.0, 0.0)),
        material=satin_aluminum,
        name="main_beam",
    )
    middle_arm.visual(
        Box((0.160, 0.018, 0.008)),
        origin=Origin(xyz=(0.140, 0.0, 0.013)),
        material=graphite,
        name="top_rib",
    )
    middle_arm.visual(
        Box((0.032, 0.036, 0.032)),
        origin=Origin(xyz=(0.283, 0.0, 0.0)),
        material=graphite,
        name="wrist_block",
    )
    _add_clevis(
        middle_arm,
        joint_x=0.310,
        ear_height=0.046,
        ear_depth=0.022,
        material=graphite,
        name_prefix="wrist",
    )
    middle_arm.inertial = Inertial.from_geometry(
        Box((0.330, 0.070, 0.070)),
        mass=0.78,
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.013, length=HINGE_BARREL_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="wrist_barrel",
    )
    forearm.visual(
        Box((0.038, 0.030, 0.026)),
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
        material=graphite,
        name="wrist_hub",
    )
    forearm.visual(
        Box((0.180, 0.030, 0.016)),
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        material=satin_aluminum,
        name="main_beam",
    )
    forearm.visual(
        Box((0.120, 0.018, 0.007)),
        origin=Origin(xyz=(0.115, 0.0, 0.0115)),
        material=graphite,
        name="top_channel",
    )
    forearm.visual(
        Box((0.030, 0.042, 0.026)),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=graphite,
        name="pan_pedestal",
    )
    forearm.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.240, 0.0, -0.003)),
        material=graphite,
        name="pan_turntable",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.270, 0.065, 0.070)),
        mass=0.55,
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
    )

    pan_hub = model.part("pan_hub")
    pan_hub.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=graphite,
        name="pan_base",
    )
    pan_hub.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=graphite,
        name="pan_column",
    )
    pan_hub.visual(
        Box((0.020, 0.040, 0.007)),
        origin=Origin(xyz=(0.010, 0.0, 0.0565)),
        material=graphite,
        name="yoke_bridge",
    )
    pan_hub.visual(
        Box((0.012, 0.014, 0.020)),
        origin=Origin(xyz=(0.006, 0.0, 0.043)),
        material=graphite,
        name="yoke_support",
    )
    _add_clevis(
        pan_hub,
        joint_x=0.026,
        joint_z=0.040,
        ear_height=0.040,
        ear_depth=0.018,
        material=graphite,
        name_prefix="tilt",
    )
    pan_hub.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.080)),
        mass=0.22,
        origin=Origin(xyz=(0.012, 0.0, 0.032)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.012, length=HINGE_BARREL_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="tilt_trunnion",
    )
    lamp_head.visual(
        Box((0.032, 0.028, 0.028)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=graphite,
        name="neck_block",
    )
    lamp_head.visual(
        Cylinder(radius=0.032, length=0.090),
        origin=Origin(
            xyz=(0.066, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=painted_steel,
        name="head_shell",
    )
    for index, fin_x in enumerate((0.028, 0.034, 0.040)):
        lamp_head.visual(
            Cylinder(radius=0.034, length=0.002),
            origin=Origin(
                xyz=(fin_x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=graphite,
            name=f"cooling_fin_{index}",
        )
    lamp_head.visual(
        Box((0.050, 0.020, 0.012)),
        origin=Origin(xyz=(0.058, 0.0, 0.031)),
        material=graphite,
        name="driver_housing",
    )
    lamp_head.visual(
        Cylinder(radius=0.036, length=0.006),
        origin=Origin(
            xyz=(0.110, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_aluminum,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.030, length=0.003),
        origin=Origin(
            xyz=(0.114, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.026, length=0.002),
        origin=Origin(
            xyz=(0.1145, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=warm_led,
        name="led_emitter",
    )
    lamp_head.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(
            xyz=(0.018, 0.0, -0.026),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="rear_cable_gland",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.120),
        mass=0.42,
        origin=Origin(
            xyz=(0.060, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(xyz=(-0.020, 0.0, 0.258)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-1.10,
            upper=1.25,
        ),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=middle_arm,
        origin=Origin(xyz=(0.350, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-1.45,
            upper=1.45,
        ),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=middle_arm,
        child=forearm,
        origin=Origin(xyz=(0.310, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.2,
            lower=-1.30,
            upper=1.30,
        ),
    )
    model.articulation(
        "head_pan",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=pan_hub,
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-1.80,
            upper=1.80,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_hub,
        child=lamp_head,
        origin=Origin(xyz=(0.026, 0.0, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-1.00,
            upper=0.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp_base = object_model.get_part("clamp_base")
    lower_arm = object_model.get_part("lower_arm")
    middle_arm = object_model.get_part("middle_arm")
    forearm = object_model.get_part("forearm")
    pan_hub = object_model.get_part("pan_hub")
    lamp_head = object_model.get_part("lamp_head")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")
    head_pan = object_model.get_articulation("head_pan")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        lower_arm,
        clamp_base,
        elem_a="shoulder_barrel",
        elem_b="shoulder_left_ear",
        name="shoulder barrel seats in clamp clevis",
    )
    ctx.expect_contact(
        middle_arm,
        lower_arm,
        elem_a="elbow_barrel",
        elem_b="elbow_left_ear",
        name="elbow barrel seats in lower arm clevis",
    )
    ctx.expect_contact(
        forearm,
        middle_arm,
        elem_a="wrist_barrel",
        elem_b="wrist_left_ear",
        name="wrist barrel seats in middle arm clevis",
    )
    ctx.expect_contact(
        pan_hub,
        forearm,
        elem_a="pan_base",
        elem_b="pan_turntable",
        name="pan hub sits on wrist turntable",
    )
    ctx.expect_contact(
        lamp_head,
        pan_hub,
        elem_a="tilt_trunnion",
        elem_b="tilt_left_ear",
        name="head trunnion seats in tilt yoke",
    )

    rest_head = ctx.part_world_position(lamp_head)
    with ctx.pose({shoulder: 0.75}):
        raised_head = ctx.part_world_position(lamp_head)
    ctx.check(
        "shoulder raises the lamp",
        rest_head is not None and raised_head is not None and raised_head[2] > rest_head[2] + 0.18,
        details=f"rest={rest_head}, raised={raised_head}",
    )

    with ctx.pose({shoulder: 0.35, elbow: 0.0, wrist: 0.0, head_pan: 0.0, head_tilt: 0.0}):
        elbow_rest = ctx.part_world_position(lamp_head)
    with ctx.pose({shoulder: 0.35, elbow: 0.85, wrist: 0.0, head_pan: 0.0, head_tilt: 0.0}):
        elbow_bent = ctx.part_world_position(lamp_head)
    ctx.check(
        "elbow lifts the distal arm",
        elbow_rest is not None and elbow_bent is not None and elbow_bent[2] > elbow_rest[2] + 0.10,
        details=f"rest={elbow_rest}, bent={elbow_bent}",
    )

    with ctx.pose({shoulder: 0.35, elbow: 0.55, wrist: 0.0, head_pan: 0.0, head_tilt: 0.0}):
        wrist_rest = _visual_center(ctx, lamp_head, "front_lens")
    with ctx.pose({shoulder: 0.35, elbow: 0.55, wrist: 0.65, head_pan: 0.0, head_tilt: 0.0}):
        wrist_bent = _visual_center(ctx, lamp_head, "front_lens")
    ctx.check(
        "wrist pitches the head mount upward",
        wrist_rest is not None and wrist_bent is not None and wrist_bent[2] > wrist_rest[2] + 0.05,
        details=f"rest={wrist_rest}, bent={wrist_bent}",
    )

    with ctx.pose({shoulder: 0.25, elbow: 0.45, wrist: -0.20, head_pan: 0.0, head_tilt: 0.0}):
        pan_rest = _visual_center(ctx, lamp_head, "front_lens")
    with ctx.pose({shoulder: 0.25, elbow: 0.45, wrist: -0.20, head_pan: 0.90, head_tilt: 0.0}):
        pan_turned = _visual_center(ctx, lamp_head, "front_lens")
    ctx.check(
        "head pan rotates around a vertical axis",
        pan_rest is not None and pan_turned is not None and abs(pan_turned[1] - pan_rest[1]) > 0.05,
        details=f"rest={pan_rest}, turned={pan_turned}",
    )

    with ctx.pose({shoulder: 0.25, elbow: 0.45, wrist: -0.20, head_pan: 0.0, head_tilt: 0.0}):
        tilt_rest = _visual_center(ctx, lamp_head, "front_lens")
    with ctx.pose({shoulder: 0.25, elbow: 0.45, wrist: -0.20, head_pan: 0.0, head_tilt: 0.50}):
        tilt_up = _visual_center(ctx, lamp_head, "front_lens")
    ctx.check(
        "head tilt raises the beam aim",
        tilt_rest is not None and tilt_up is not None and tilt_up[2] > tilt_rest[2] + 0.04,
        details=f"rest={tilt_rest}, tilted={tilt_up}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
