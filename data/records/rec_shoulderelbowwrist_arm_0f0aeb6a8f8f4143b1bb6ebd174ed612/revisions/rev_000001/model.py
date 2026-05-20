from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

import cadquery as cq

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
    mesh_from_cadquery,
)


UPPER_ARM_LENGTH = 0.280
FOREARM_LENGTH = 0.240
WRIST_LENGTH = 0.102

SHOULDER_GAP = 0.030
ELBOW_GAP = 0.030
WRIST_GAP = 0.026
CHEEK_THICKNESS = 0.012


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _y_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    cx, cy, cz = center
    shape = cq.Workplane("XZ").center(cx, cz).circle(radius).extrude(length / 2.0, both=True)
    if not isclose(cy, 0.0, abs_tol=1e-9):
        shape = shape.translate((0.0, cy, 0.0))
    return shape


def _z_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .center(cx, cy)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, 0.0, cz))
    )


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _make_root_bracket_shape() -> cq.Workplane:
    ear_center_y = (SHOULDER_GAP / 2.0) + (CHEEK_THICKNESS / 2.0)

    base_plate = _box((0.140, 0.100, 0.014), (-0.070, 0.0, -0.127))
    rear_flange = _box((0.020, 0.100, 0.050), (-0.130, 0.0, -0.102))
    vertical_web = _box((0.044, 0.056, 0.104), (-0.060, 0.0, -0.052))
    top_saddle = _box((0.050, 0.050, 0.022), (-0.030, 0.0, -0.018))
    buttress = (
        cq.Workplane("XZ")
        .polyline([(-0.136, -0.120), (-0.030, -0.120), (-0.012, -0.022), (-0.080, -0.022)])
        .close()
        .extrude(0.018, both=True)
    )
    shoulder_bridge = _box((0.022, SHOULDER_GAP + 2.0 * CHEEK_THICKNESS, 0.010), (-0.011, 0.0, 0.022))
    nose_web = _box((0.018, SHOULDER_GAP, 0.020), (-0.009, 0.0, -0.012))
    left_ear = _box((0.028, CHEEK_THICKNESS, 0.056), (-0.014, ear_center_y, 0.000))
    right_ear = _box((0.028, CHEEK_THICKNESS, 0.056), (-0.014, -ear_center_y, 0.000))

    bracket = _union_all(
        [
            base_plate,
            rear_flange,
            vertical_web,
            top_saddle,
            buttress,
            shoulder_bridge,
            nose_web,
            left_ear,
            right_ear,
        ]
    )

    return bracket


def _make_upper_arm_shape() -> cq.Workplane:
    return _union_all(
        [
            _box((0.026, 0.022, 0.022), (0.013, 0.0, 0.0)),
            _y_cylinder(radius=0.013, length=SHOULDER_GAP, center=(0.013, 0.0, 0.0)),
            _box((0.190, 0.040, 0.034), (0.118, 0.0, 0.0)),
            _box((0.110, 0.020, 0.012), (0.100, 0.0, -0.017)),
            _box((0.058, 0.022, 0.022), (0.235, 0.0, 0.0)),
            _box((0.032, CHEEK_THICKNESS, 0.050), (0.264, ELBOW_GAP / 2.0 + CHEEK_THICKNESS / 2.0, 0.0)),
            _box((0.032, CHEEK_THICKNESS, 0.050), (0.264, -ELBOW_GAP / 2.0 - CHEEK_THICKNESS / 2.0, 0.0)),
            _box((0.024, ELBOW_GAP + 2.0 * CHEEK_THICKNESS, 0.010), (0.258, 0.0, 0.020)),
        ]
    )


def _make_forearm_shape() -> cq.Workplane:
    return _union_all(
        [
            _box((0.022, 0.020, 0.020), (0.011, 0.0, 0.0)),
            _y_cylinder(radius=0.012, length=ELBOW_GAP, center=(0.011, 0.0, 0.0)),
            _box((0.164, 0.036, 0.032), (0.104, 0.0, 0.0)),
            _box((0.108, 0.018, 0.012), (0.094, 0.0, -0.017)),
            _box((0.052, 0.020, 0.020), (0.204, 0.0, 0.0)),
            _box((0.030, CHEEK_THICKNESS, 0.046), (0.223, WRIST_GAP / 2.0 + CHEEK_THICKNESS / 2.0, 0.0)),
            _box((0.030, CHEEK_THICKNESS, 0.046), (0.223, -WRIST_GAP / 2.0 - CHEEK_THICKNESS / 2.0, 0.0)),
            _box((0.022, WRIST_GAP + 2.0 * CHEEK_THICKNESS, 0.010), (0.217, 0.0, 0.018)),
        ]
    )


def _make_wrist_shape() -> cq.Workplane:
    return _union_all(
        [
            _box((0.018, 0.018, 0.018), (0.009, 0.0, 0.0)),
            _y_cylinder(radius=0.010, length=WRIST_GAP, center=(0.009, 0.0, 0.0)),
            _box((0.050, 0.028, 0.030), (0.035, 0.0, 0.0)),
            _box((0.030, 0.042, 0.040), (0.070, 0.0, 0.0)),
            _box((0.026, 0.018, 0.014), (0.058, 0.0, -0.020)),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_mounted_articulated_arm")

    model.material("powder_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("anodized_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("graphite", rgba=(0.27, 0.29, 0.32, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_make_root_bracket_shape(), "root_bracket"),
        material="powder_steel",
        name="root_bracket_body",
    )
    root_bracket.inertial = Inertial.from_geometry(
        Box((0.120, 0.092, 0.140)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.013, length=SHOULDER_GAP),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="upper_arm_shoulder_hub",
    )
    upper_arm.visual(
        Box((0.028, 0.022, 0.022)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material="graphite",
        name="upper_arm_root_block",
    )
    upper_arm.visual(
        Box((0.196, 0.038, 0.032)),
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
        material="anodized_aluminum",
        name="upper_arm_beam",
    )
    upper_arm.visual(
        Box((0.110, 0.020, 0.012)),
        origin=Origin(xyz=(0.100, 0.0, -0.017)),
        material="graphite",
        name="upper_arm_lower_rib",
    )
    upper_arm.visual(
        Box((0.058, 0.032, 0.024)),
        origin=Origin(xyz=(0.233, 0.0, 0.0)),
        material="anodized_aluminum",
        name="upper_arm_elbow_anchor",
    )
    upper_arm.visual(
        Box((0.034, CHEEK_THICKNESS, 0.050)),
        origin=Origin(xyz=(0.263, ELBOW_GAP / 2.0 + CHEEK_THICKNESS / 2.0, 0.0)),
        material="graphite",
        name="upper_arm_elbow_ear_left",
    )
    upper_arm.visual(
        Box((0.034, CHEEK_THICKNESS, 0.050)),
        origin=Origin(xyz=(0.263, -ELBOW_GAP / 2.0 - CHEEK_THICKNESS / 2.0, 0.0)),
        material="graphite",
        name="upper_arm_elbow_ear_right",
    )
    upper_arm.visual(
        Box((0.028, ELBOW_GAP + 2.0 * CHEEK_THICKNESS, 0.010)),
        origin=Origin(xyz=(0.255, 0.0, 0.020)),
        material="graphite",
        name="upper_arm_elbow_bridge",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.294, 0.054, 0.054)),
        mass=1.4,
        origin=Origin(xyz=(0.147, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.012, length=ELBOW_GAP),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="forearm_elbow_hub",
    )
    forearm.visual(
        Box((0.024, 0.020, 0.020)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material="graphite",
        name="forearm_root_block",
    )
    forearm.visual(
        Box((0.168, 0.036, 0.030)),
        origin=Origin(xyz=(0.104, 0.0, 0.0)),
        material="anodized_aluminum",
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.108, 0.018, 0.012)),
        origin=Origin(xyz=(0.094, 0.0, -0.017)),
        material="graphite",
        name="forearm_lower_rib",
    )
    forearm.visual(
        Box((0.056, 0.030, 0.022)),
        origin=Origin(xyz=(0.207, 0.0, 0.0)),
        material="anodized_aluminum",
        name="forearm_wrist_anchor",
    )
    forearm.visual(
        Box((0.034, CHEEK_THICKNESS, 0.046)),
        origin=Origin(xyz=(0.223, WRIST_GAP / 2.0 + CHEEK_THICKNESS / 2.0, 0.0)),
        material="graphite",
        name="forearm_wrist_ear_left",
    )
    forearm.visual(
        Box((0.034, CHEEK_THICKNESS, 0.046)),
        origin=Origin(xyz=(0.223, -WRIST_GAP / 2.0 - CHEEK_THICKNESS / 2.0, 0.0)),
        material="graphite",
        name="forearm_wrist_ear_right",
    )
    forearm.visual(
        Box((0.028, WRIST_GAP + 2.0 * CHEEK_THICKNESS, 0.010)),
        origin=Origin(xyz=(0.216, 0.0, 0.018)),
        material="graphite",
        name="forearm_wrist_bridge",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.246, 0.050, 0.050)),
        mass=1.0,
        origin=Origin(xyz=(0.123, 0.0, 0.0)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.010, length=WRIST_GAP),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="wrist_hub",
    )
    wrist.visual(
        Box((0.020, 0.018, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material="graphite",
        name="wrist_root_block",
    )
    wrist.visual(
        Box((0.060, 0.028, 0.026)),
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        material="graphite",
        name="wrist_beam",
    )
    wrist.visual(
        Box((0.040, 0.042, 0.040)),
        origin=Origin(xyz=(0.082, 0.0, 0.0)),
        material="graphite",
        name="wrist_terminal_head",
    )
    wrist.visual(
        Box((0.028, 0.018, 0.014)),
        origin=Origin(xyz=(0.064, 0.0, -0.016)),
        material="graphite",
        name="wrist_tool_pad",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((WRIST_LENGTH, 0.050, 0.054)),
        mass=0.45,
        origin=Origin(xyz=(WRIST_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=upper_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.35, upper=1.20),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.90, upper=1.10),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=-0.80, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    wrist_terminal_head = wrist.get_visual("wrist_terminal_head")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

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

    ctx.check(
        "serial_arm_parts_present",
        all(part is not None for part in (root_bracket, upper_arm, forearm, wrist)),
        "Expected root bracket, upper arm, forearm, and wrist parts.",
    )
    ctx.check(
        "three_supported_pitch_joints",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, -1.0, 0.0)
            for joint in (shoulder_joint, elbow_joint, wrist_joint)
        ),
        "Shoulder, elbow, and wrist should all be revolute pitch joints about local -Y.",
    )

    ctx.expect_contact(root_bracket, upper_arm, name="shoulder_joint_has_real_support_contact")
    ctx.expect_contact(upper_arm, forearm, name="elbow_joint_has_real_support_contact")
    ctx.expect_contact(forearm, wrist, name="wrist_joint_has_real_support_contact")

    with ctx.pose({shoulder_joint: 0.75}):
        ctx.expect_origin_gap(
            forearm,
            root_bracket,
            axis="z",
            min_gap=0.10,
            name="shoulder_joint_lifts_elbow_above_bracket",
        )

    with ctx.pose({shoulder_joint: 0.45, elbow_joint: 0.70}):
        ctx.expect_origin_gap(
            wrist,
            forearm,
            axis="z",
            min_gap=0.12,
            name="elbow_joint_lifts_wrist_axis",
        )

    with ctx.pose({shoulder_joint: 0.45, elbow_joint: 0.40, wrist_joint: 0.0}):
        wrist_rest_aabb = ctx.part_element_world_aabb(wrist, elem=wrist_terminal_head)
    with ctx.pose({shoulder_joint: 0.45, elbow_joint: 0.40, wrist_joint: 0.60}):
        wrist_raised_aabb = ctx.part_element_world_aabb(wrist, elem=wrist_terminal_head)

    ctx.check(
        "wrist_joint_reorients_terminal_member",
        wrist_rest_aabb is not None
        and wrist_raised_aabb is not None
        and wrist_raised_aabb[1][2] > wrist_rest_aabb[1][2] + 0.010,
        "Positive wrist motion should raise the terminal wrist section's free end.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
