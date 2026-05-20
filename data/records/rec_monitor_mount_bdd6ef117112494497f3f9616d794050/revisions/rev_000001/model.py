from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LOWER_ARM_LENGTH = 0.260
UPPER_ARM_LENGTH = 0.235

JOINT_GAP = 0.020
EAR_THICKNESS = 0.008
JOINT_WIDTH = JOINT_GAP + 2.0 * EAR_THICKNESS
PITCH_JOINT_RADIUS = 0.009
TILT_JOINT_RADIUS = 0.008

ARM_WIDTH = 0.024
LOWER_ARM_HEIGHT = 0.022
UPPER_ARM_HEIGHT = 0.020

MAST_HEIGHT = 0.295
SWIVEL_POST_RADIUS = 0.012
SWIVEL_HOUSING_RADIUS = 0.022
SWIVEL_HOUSING_HEIGHT = 0.026
TILT_AXIS_OFFSET = 0.044
SWIVEL_STACK_HEIGHT = 0.010
SWIVEL_CAP_HEIGHT = 0.008
TILT_AXIS_Z = 0.018

VESA_PLATE_SIZE = 0.118
VESA_PLATE_THICKNESS = 0.006
VESA_PATTERN = 0.100
def _cylinder_y(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate((x, y, z))


def _cylinder_x(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate((x, y, z))


def _pitch_clevis(*, joint_radius: float, height: float) -> cq.Workplane:
    bridge_len = 0.016
    ear_len = 0.022
    ear_center_x = -0.011
    ear_center_y = JOINT_GAP / 2.0 + EAR_THICKNESS / 2.0

    back_bridge = cq.Workplane("XY").box(bridge_len, JOINT_WIDTH, height).translate((ear_center_x - ear_len / 2.0 - bridge_len / 2.0, 0.0, 0.0))
    ear_left = cq.Workplane("XY").box(ear_len, EAR_THICKNESS, height).translate((ear_center_x, ear_center_y, 0.0))
    ear_right = cq.Workplane("XY").box(ear_len, EAR_THICKNESS, height).translate((ear_center_x, -ear_center_y, 0.0))
    hole = _cylinder_y(joint_radius, JOINT_WIDTH + 0.010)
    return back_bridge.union(ear_left).union(ear_right).cut(hole)


def _pitch_tongue(length: float, width: float, height: float) -> cq.Workplane:
    tongue = cq.Workplane("XY").box(length, width, height).translate((length / 2.0, 0.0, 0.0))
    return tongue.edges("|X").fillet(min(width, height) * 0.18)


def _build_clamp_base_shape() -> cq.Workplane:
    shoulder_clevis = _pitch_clevis(joint_radius=PITCH_JOINT_RADIUS, height=0.050)

    mast = cq.Workplane("XY").circle(0.018).extrude(MAST_HEIGHT).translate((-0.030, 0.0, -MAST_HEIGHT))
    top_jaw = cq.Workplane("XY").box(0.078, 0.084, 0.014).translate((-0.006, 0.0, -0.198))
    back_plate = cq.Workplane("XY").box(0.018, 0.082, 0.096).translate((-0.044, 0.0, -0.238))
    lower_block = cq.Workplane("XY").box(0.056, 0.058, 0.020).translate((-0.004, 0.0, -0.246))
    screw = cq.Workplane("XY").circle(0.0075).extrude(0.060).translate((0.010, 0.0, -0.303))
    screw_pad = cq.Workplane("XY").circle(0.012).extrude(0.006).translate((0.010, 0.0, -0.243))
    knob_hub = cq.Workplane("XY").circle(0.010).extrude(0.012).translate((0.010, 0.0, -0.315))
    knob_handle = _cylinder_y(0.004, 0.048, x=0.010, z=-0.282)
    gusset = cq.Workplane("XY").box(0.030, 0.044, 0.050).translate((-0.020, 0.0, -0.032))

    return (
        shoulder_clevis
        .union(mast)
        .union(top_jaw)
        .union(back_plate)
        .union(lower_block)
        .union(screw)
        .union(screw_pad)
        .union(knob_hub)
        .union(knob_handle)
        .union(gusset)
    )


def _build_lower_arm_shape() -> cq.Workplane:
    root = _pitch_tongue(0.026, JOINT_GAP, 0.028)
    beam = (
        cq.Workplane("YZ")
        .rect(ARM_WIDTH, LOWER_ARM_HEIGHT)
        .extrude(LOWER_ARM_LENGTH - 0.044)
        .translate((0.026, 0.0, 0.0))
        .edges("|X")
        .fillet(0.004)
    )
    beam_cheek = cq.Workplane("XY").box(0.054, 0.028, 0.026).translate((0.060, 0.0, 0.0))
    elbow_clevis = _pitch_clevis(joint_radius=PITCH_JOINT_RADIUS, height=0.040).translate((LOWER_ARM_LENGTH, 0.0, 0.0))

    return root.union(beam).union(beam_cheek).union(elbow_clevis)


def _build_upper_arm_shape() -> cq.Workplane:
    root = _pitch_tongue(0.024, JOINT_GAP, 0.026)
    beam = (
        cq.Workplane("YZ")
        .rect(0.022, UPPER_ARM_HEIGHT)
        .extrude(UPPER_ARM_LENGTH - 0.040)
        .translate((0.024, 0.0, 0.0))
        .edges("|X")
        .fillet(0.0035)
    )
    neck = cq.Workplane("XY").box(0.040, 0.030, 0.018).translate((UPPER_ARM_LENGTH - 0.020, 0.0, -0.010))
    swivel_base = (
        cq.Workplane("XY")
        .circle(0.021)
        .extrude(0.010)
        .translate((UPPER_ARM_LENGTH, 0.0, -0.010))
    )
    swivel_flange = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(0.004)
        .translate((UPPER_ARM_LENGTH, 0.0, -0.004))
    )

    return root.union(beam).union(neck).union(swivel_base).union(swivel_flange)


def _build_swivel_yoke_shape() -> cq.Workplane:
    swivel_disc = cq.Workplane("XY").circle(0.018).extrude(0.006)
    center_cap = cq.Workplane("XY").circle(0.012).extrude(0.004).translate((0.0, 0.0, 0.006))
    neck = cq.Workplane("XY").box(0.036, 0.022, 0.018).translate((0.018, 0.0, TILT_AXIS_Z))
    tilt_clevis = _pitch_clevis(joint_radius=TILT_JOINT_RADIUS, height=0.036).translate((TILT_AXIS_OFFSET, 0.0, TILT_AXIS_Z))

    return swivel_disc.union(center_cap).union(neck).union(tilt_clevis)


def _build_vesa_head_shape() -> cq.Workplane:
    root = _pitch_tongue(0.026, JOINT_GAP, 0.026)
    neck_bar = cq.Workplane("XY").box(0.060, 0.018, 0.022).translate((0.042, 0.0, 0.0))
    support_block = cq.Workplane("XY").box(0.024, 0.050, 0.050).translate((0.066, 0.0, 0.0))
    standoff = cq.Workplane("XY").box(0.020, 0.060, 0.026).translate((0.086, 0.0, 0.0))
    plate = (
        cq.Workplane("YZ")
        .rect(VESA_PLATE_SIZE, VESA_PLATE_SIZE)
        .extrude(VESA_PLATE_THICKNESS)
        .translate((0.096, 0.0, 0.0))
    )
    plate = (
        plate.faces(">X")
        .workplane()
        .pushPoints(
            [
                (-VESA_PATTERN / 2.0, -VESA_PATTERN / 2.0),
                (-VESA_PATTERN / 2.0, VESA_PATTERN / 2.0),
                (VESA_PATTERN / 2.0, -VESA_PATTERN / 2.0),
                (VESA_PATTERN / 2.0, VESA_PATTERN / 2.0),
            ]
        )
        .hole(0.006)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(0.032, 0.020)
        .cutBlind(-VESA_PLATE_THICKNESS)
    )

    return root.union(neck_bar).union(support_block).union(standoff).union(plate)


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_clamped_monitor_mount")

    model.material("powder_black", rgba=(0.13, 0.14, 0.16, 1.0))
    model.material("graphite", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("satin_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("hardware_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        mesh_from_cadquery(_build_clamp_base_shape(), "clamp_base"),
        material="powder_black",
        name="body",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_build_lower_arm_shape(), "lower_arm"),
        material="satin_aluminum",
        name="body",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_build_upper_arm_shape(), "upper_arm"),
        material="satin_aluminum",
        name="body",
    )

    swivel_yoke = model.part("swivel_yoke")
    swivel_yoke.visual(
        mesh_from_cadquery(_build_swivel_yoke_shape(), "swivel_yoke"),
        material="graphite",
        name="body",
    )

    vesa_head = model.part("vesa_head")
    vesa_head.visual(
        mesh_from_cadquery(_build_vesa_head_shape(), "vesa_head"),
        material="hardware_dark",
        name="body",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.10, effort=30.0, velocity=1.3),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.20, upper=2.20, effort=22.0, velocity=1.6),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=swivel_yoke,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.80, upper=2.80, effort=8.0, velocity=2.3),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_yoke,
        child=vesa_head,
        origin=Origin(xyz=(TILT_AXIS_OFFSET, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=0.55, effort=6.0, velocity=2.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp_base = object_model.get_part("clamp_base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    swivel_yoke = object_model.get_part("swivel_yoke")
    vesa_head = object_model.get_part("vesa_head")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    head_swivel = object_model.get_articulation("head_swivel")
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
    ctx.allow_overlap(
        clamp_base,
        lower_arm,
        reason="The shoulder pivot is modeled as a captured tongue inside the clamp-head clevis shroud, so the enclosed joint cartridge is intentionally represented as nested hardware.",
    )
    ctx.allow_overlap(
        upper_arm,
        swivel_yoke,
        reason="The monitor-head swivel is represented as a compact coaxial bearing stack with overlapping cosmetic shrouds around the vertical pivot.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(clamp_base, lower_arm, name="shoulder_joint_is_supported")
    ctx.expect_contact(lower_arm, upper_arm, name="elbow_joint_is_supported")
    ctx.expect_contact(upper_arm, swivel_yoke, name="swivel_joint_is_supported")
    ctx.expect_contact(swivel_yoke, vesa_head, name="tilt_joint_is_supported")

    ctx.check("shoulder_axis_is_pitch", shoulder.axis == (0.0, -1.0, 0.0), f"axis={shoulder.axis}")
    ctx.check("elbow_axis_is_pitch", elbow.axis == (0.0, -1.0, 0.0), f"axis={elbow.axis}")
    ctx.check("swivel_axis_is_vertical", head_swivel.axis == (0.0, 0.0, 1.0), f"axis={head_swivel.axis}")
    ctx.check("tilt_axis_is_horizontal", head_tilt.axis == (0.0, -1.0, 0.0), f"axis={head_tilt.axis}")

    lower_rest = _aabb_center(ctx.part_world_aabb(lower_arm))
    upper_rest = _aabb_center(ctx.part_world_aabb(upper_arm))
    head_rest = _aabb_center(ctx.part_world_aabb(vesa_head))

    with ctx.pose({shoulder: 0.65}):
        lower_raised = _aabb_center(ctx.part_world_aabb(lower_arm))
    ctx.check(
        "positive_shoulder_raises_lower_arm",
        lower_raised[2] > lower_rest[2] + 0.040,
        f"rest_z={lower_rest[2]:.4f}, posed_z={lower_raised[2]:.4f}",
    )

    with ctx.pose({elbow: 0.90}):
        upper_raised = _aabb_center(ctx.part_world_aabb(upper_arm))
    ctx.check(
        "positive_elbow_raises_upper_arm",
        upper_raised[2] > upper_rest[2] + 0.035,
        f"rest_z={upper_rest[2]:.4f}, posed_z={upper_raised[2]:.4f}",
    )

    with ctx.pose({head_swivel: 0.80}):
        head_swiveled = _aabb_center(ctx.part_world_aabb(vesa_head))
    ctx.check(
        "swivel_moves_head_laterally",
        abs(head_swiveled[1] - head_rest[1]) > 0.020,
        f"rest_y={head_rest[1]:.4f}, posed_y={head_swiveled[1]:.4f}",
    )

    with ctx.pose({head_tilt: 0.35}):
        head_tilted = _aabb_center(ctx.part_world_aabb(vesa_head))
    ctx.check(
        "positive_tilt_lifts_head_plate",
        head_tilted[2] > head_rest[2] + 0.008,
        f"rest_z={head_rest[2]:.4f}, posed_z={head_tilted[2]:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
