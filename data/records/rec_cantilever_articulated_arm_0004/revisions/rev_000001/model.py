from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

PLATE_LENGTH = 0.22
PLATE_WIDTH = 0.16
PLATE_THICKNESS = 0.016

COLUMN_LENGTH = 0.07
COLUMN_WIDTH = 0.09
COLUMN_HEIGHT = 0.13

SHOULDER_AXIS_X = 0.055
SHOULDER_AXIS_Z = PLATE_THICKNESS + COLUMN_HEIGHT - 0.006

CLEVIS_EAR_THICKNESS = 0.009
CLEVIS_GAP = 0.018
CLEVIS_EAR_HEIGHT = 0.042
CLEVIS_EAR_LENGTH = 0.034
HINGE_BARREL_RADIUS = 0.007
HINGE_PIN_RADIUS = 0.0035

UPPER_ARM_LENGTH = 0.205
UPPER_ARM_WIDTH = 0.028
UPPER_ARM_HEIGHT = 0.038

FOREARM_LENGTH = 0.175
FOREARM_WIDTH = 0.026
FOREARM_HEIGHT = 0.034

WRIST_HUB_RADIUS = 0.009
WRIST_HUB_LENGTH = 0.022
WRIST_SUPPORT_PLATE_THICKNESS = 0.008
WRIST_SUPPORT_WIDTH = 0.04
WRIST_SUPPORT_HEIGHT = 0.048

TOOL_PLATE_LENGTH = 0.062
TOOL_PLATE_WIDTH = 0.058
TOOL_PLATE_HEIGHT = 0.042

SHOULDER_RANGE = math.radians(120.0)
ELBOW_RANGE = math.radians(135.0)
WRIST_RANGE = math.radians(90.0)


def _box_span(
    x0: float,
    x1: float,
    y_size: float,
    z_size: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, y_size, z_size).translate(((x0 + x1) * 0.5, y, z))


def _cylinder_y(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, y - length * 0.5, z))


def _cylinder_x(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x - length * 0.5, y, z))


def _base_clevis_shape() -> cq.Workplane:
    web = _box_span(
        COLUMN_LENGTH * 0.28,
        SHOULDER_AXIS_X - 0.011,
        0.03,
        0.014,
        z=SHOULDER_AXIS_Z - 0.014,
    )
    brace = _box_span(
        SHOULDER_AXIS_X - 0.018,
        SHOULDER_AXIS_X - 0.011,
        0.038,
        0.024,
        z=SHOULDER_AXIS_Z - 0.006,
    )
    ear_y = 0.013
    ears = (
        _box_span(
            SHOULDER_AXIS_X - 0.01,
            SHOULDER_AXIS_X + 0.01,
            0.006,
            0.038,
            y=ear_y,
            z=SHOULDER_AXIS_Z,
        )
        .union(
            _box_span(
                SHOULDER_AXIS_X - 0.01,
                SHOULDER_AXIS_X + 0.01,
                0.006,
                0.038,
                y=-ear_y,
                z=SHOULDER_AXIS_Z,
            )
        )
    )
    return web.union(brace).union(ears)


def _upper_arm_beam_shape() -> cq.Workplane:
    half_h = UPPER_ARM_HEIGHT * 0.5
    profile = [
        (0.01, -half_h * 0.45),
        (0.022, -half_h * 0.9),
        (0.06, -half_h),
        (UPPER_ARM_LENGTH - 0.055, -half_h * 0.92),
        (UPPER_ARM_LENGTH - 0.02, -half_h * 0.62),
        (UPPER_ARM_LENGTH - 0.02, half_h * 0.62),
        (UPPER_ARM_LENGTH - 0.055, half_h * 0.92),
        (0.06, half_h),
        (0.022, half_h * 0.9),
        (0.01, half_h * 0.45),
    ]
    beam = cq.Workplane("XZ").polyline(profile).close().extrude(0.026).translate((0.0, -0.013, 0.0))
    rib = _box_span(0.05, UPPER_ARM_LENGTH - 0.07, 0.012, 0.042, z=0.0)
    return beam.union(rib)


def _shoulder_knuckle_shape() -> cq.Workplane:
    block = _box_span(-0.008, 0.012, 0.016, 0.026, z=0.0)
    cap = _cylinder_y(0.008, 0.014, x=0.002, z=0.0)
    return block.union(cap)


def _shoulder_body_shape() -> cq.Workplane:
    return _upper_arm_beam_shape().union(_upper_arm_clevis_shape())


def _upper_arm_clevis_shape() -> cq.Workplane:
    bridge = _box_span(
        UPPER_ARM_LENGTH - 0.022,
        UPPER_ARM_LENGTH - 0.008,
        0.028,
        0.012,
        z=-0.011,
    )
    ear_y = 0.013
    ears = (
        _box_span(
            UPPER_ARM_LENGTH - 0.008,
            UPPER_ARM_LENGTH + 0.01,
            0.006,
            0.036,
            y=ear_y,
            z=0.0,
        )
        .union(
            _box_span(
                UPPER_ARM_LENGTH - 0.008,
                UPPER_ARM_LENGTH + 0.01,
                0.006,
                0.036,
                y=-ear_y,
                z=0.0,
            )
        )
    )
    return bridge.union(ears)


def _forearm_beam_shape() -> cq.Workplane:
    half_h = FOREARM_HEIGHT * 0.5
    profile = [
        (0.01, -half_h * 0.45),
        (0.022, -half_h * 0.9),
        (0.05, -half_h),
        (FOREARM_LENGTH - 0.05, -half_h * 0.92),
        (FOREARM_LENGTH - 0.024, -half_h * 0.66),
        (FOREARM_LENGTH - 0.024, half_h * 0.66),
        (FOREARM_LENGTH - 0.05, half_h * 0.92),
        (0.05, half_h),
        (0.022, half_h * 0.9),
        (0.01, half_h * 0.45),
    ]
    beam = cq.Workplane("XZ").polyline(profile).close().extrude(0.024).translate((0.0, -0.012, 0.0))
    rib = _box_span(0.04, FOREARM_LENGTH - 0.08, 0.01, 0.038, z=0.0)
    return beam.union(rib)


def _elbow_knuckle_shape() -> cq.Workplane:
    block = _box_span(-0.008, 0.012, 0.016, 0.024, z=0.0)
    cap = _cylinder_y(0.0075, 0.014, x=0.002, z=0.0)
    return block.union(cap)


def _elbow_body_shape() -> cq.Workplane:
    root_bridge = _box_span(-0.002, 0.028, 0.014, 0.018, z=0.0)
    return root_bridge.union(_forearm_beam_shape()).union(_wrist_cage_shape())


def _wrist_cage_shape() -> cq.Workplane:
    mount_block = _box_span(FOREARM_LENGTH - 0.03, FOREARM_LENGTH - 0.006, 0.028, 0.024, z=0.0)
    ear_y = 0.018
    supports = (
        _box_span(
            FOREARM_LENGTH - 0.006,
            FOREARM_LENGTH + 0.01,
            0.006,
            0.04,
            y=ear_y,
            z=0.0,
        )
        .union(
            _box_span(
                FOREARM_LENGTH - 0.006,
                FOREARM_LENGTH + 0.01,
                0.006,
                0.04,
                y=-ear_y,
                z=0.0,
            )
        )
    )
    return mount_block.union(supports)


def _tool_plate_shape() -> cq.Workplane:
    neck = _box_span(0.01, 0.022, 0.022, 0.024, z=0.0)
    plate = _box_span(0.022, TOOL_PLATE_LENGTH, TOOL_PLATE_WIDTH, TOOL_PLATE_HEIGHT, z=0.0)
    slot_1 = _cylinder_x(0.0032, 0.05, x=0.04, y=0.014, z=0.0)
    slot_2 = _cylinder_x(0.0032, 0.05, x=0.04, y=-0.014, z=0.0)
    return neck.union(plate).cut(slot_1).cut(slot_2)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_tooling_arm", assets=ASSETS)

    dark_base = model.material("dark_base", rgba=(0.18, 0.20, 0.22, 1.0))
    arm_gray = model.material("arm_gray", rgba=(0.52, 0.57, 0.63, 1.0))
    steel = model.material("steel", rgba=(0.71, 0.73, 0.76, 1.0))
    tool_plate_color = model.material("tool_plate_color", rgba=(0.30, 0.36, 0.42, 1.0))

    base = model.part("base")
    base.visual(
        Box((PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS * 0.5)),
        material=dark_base,
        name="base_plate",
    )
    base.visual(
        Box((COLUMN_LENGTH, COLUMN_WIDTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS + COLUMN_HEIGHT * 0.5)),
        material=dark_base,
        name="column",
    )
    base.visual(
        mesh_from_cadquery(_base_clevis_shape(), "base_clevis.obj", assets=ASSETS),
        material=dark_base,
        name="shoulder_clevis",
    )
    base.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, COLUMN_HEIGHT + PLATE_THICKNESS)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_HEIGHT + PLATE_THICKNESS) * 0.5)),
    )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        mesh_from_cadquery(_shoulder_knuckle_shape(), "shoulder_knuckle.obj", assets=ASSETS),
        material=steel,
        name="shoulder_knuckle",
    )
    shoulder.visual(
        mesh_from_cadquery(_shoulder_body_shape(), "shoulder_body.obj", assets=ASSETS),
        material=arm_gray,
        name="shoulder_body",
    )
    shoulder.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH + 0.02, 0.04, 0.05)),
        mass=2.2,
        origin=Origin(xyz=(UPPER_ARM_LENGTH * 0.5, 0.0, 0.0)),
    )

    elbow = model.part("elbow_link")
    elbow.visual(
        mesh_from_cadquery(_elbow_knuckle_shape(), "elbow_knuckle.obj", assets=ASSETS),
        material=steel,
        name="elbow_knuckle",
    )
    elbow.visual(
        mesh_from_cadquery(_elbow_body_shape(), "elbow_body.obj", assets=ASSETS),
        material=arm_gray,
        name="elbow_body",
    )
    elbow.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH + 0.03, 0.04, 0.05)),
        mass=1.6,
        origin=Origin(xyz=(FOREARM_LENGTH * 0.5, 0.0, 0.0)),
    )

    wrist = model.part("wrist_tool_plate")
    wrist.visual(
        Cylinder(radius=WRIST_HUB_RADIUS, length=WRIST_HUB_LENGTH),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="roll_hub",
    )
    wrist.visual(
        mesh_from_cadquery(_tool_plate_shape(), "tool_plate.obj", assets=ASSETS),
        material=tool_plate_color,
        name="tool_plate",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((TOOL_PLATE_LENGTH, TOOL_PLATE_WIDTH, TOOL_PLATE_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=(TOOL_PLATE_LENGTH * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.6,
            lower=0.0,
            upper=SHOULDER_RANGE,
        ),
    )
    model.articulation(
        "shoulder_to_elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=elbow,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=2.0,
            lower=-ELBOW_RANGE,
            upper=0.0,
        ),
    )
    model.articulation(
        "elbow_to_wrist",
        ArticulationType.REVOLUTE,
        parent=elbow,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=-WRIST_RANGE,
            upper=WRIST_RANGE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    shoulder = object_model.get_part("shoulder_link")
    elbow = object_model.get_part("elbow_link")
    wrist = object_model.get_part("wrist_tool_plate")

    shoulder_joint = object_model.get_articulation("base_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_elbow")
    wrist_joint = object_model.get_articulation("elbow_to_wrist")

    base_clevis = base.get_visual("shoulder_clevis")
    shoulder_knuckle = shoulder.get_visual("shoulder_knuckle")
    shoulder_body = shoulder.get_visual("shoulder_body")
    elbow_knuckle = elbow.get_visual("elbow_knuckle")
    elbow_body = elbow.get_visual("elbow_body")
    wrist_cage = elbow.get_visual("elbow_body")
    wrist_hub = wrist.get_visual("roll_hub")
    tool_plate = wrist.get_visual("tool_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        shoulder,
        base,
        elem_a=shoulder_knuckle,
        elem_b=base_clevis,
        reason="shoulder trunnion nests inside the fixed base clevis as an intentional hinge seat",
    )
    ctx.allow_overlap(
        elbow,
        shoulder,
        elem_a=elbow_knuckle,
        elem_b=shoulder_body,
        reason="elbow trunnion nests inside the shoulder clevis as an intentional hinge seat",
    )
    ctx.allow_overlap(
        wrist,
        elbow,
        elem_a=wrist_hub,
        elem_b=wrist_cage,
        reason="wrist roll hub is intentionally captured inside the clevis support cradle",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "parts_present",
        all(part is not None for part in (base, shoulder, elbow, wrist)),
        "base, shoulder, elbow, and wrist parts must all exist",
    )
    ctx.check(
        "joint_axes",
        shoulder_joint.axis == (0.0, 1.0, 0.0)
        and elbow_joint.axis == (0.0, 1.0, 0.0)
        and wrist_joint.axis == (1.0, 0.0, 0.0),
        "shoulder and elbow must share a horizontal Y-axis while wrist rolls about X",
    )
    ctx.check(
        "motion_ranges",
        abs((shoulder_joint.motion_limits.upper - shoulder_joint.motion_limits.lower) - SHOULDER_RANGE) < 1e-6
        and abs((elbow_joint.motion_limits.upper - elbow_joint.motion_limits.lower) - ELBOW_RANGE) < 1e-6
        and abs((wrist_joint.motion_limits.upper - wrist_joint.motion_limits.lower) - 2.0 * WRIST_RANGE) < 1e-6,
        "joint travel should be about 120°, 135°, and ±90° respectively",
    )

    ctx.expect_origin_distance(
        shoulder,
        base,
        axes="y",
        max_dist=0.002,
        name="shoulder_axis_centered_in_base_clevis",
    )
    ctx.expect_overlap(
        shoulder,
        base,
        axes="xz",
        min_overlap=0.012,
        elem_a=shoulder_knuckle,
        elem_b=base_clevis,
        name="shoulder_knuckle_overlaps_base_clevis_window",
    )
    ctx.expect_within(
        elbow,
        shoulder,
        axes="yz",
        margin=0.004,
        inner_elem=elbow_knuckle,
        outer_elem=shoulder_body,
        name="elbow_knuckle_within_shoulder_clevis_envelope",
    )
    ctx.expect_overlap(
        elbow,
        shoulder,
        axes="xz",
        min_overlap=0.012,
        elem_a=elbow_knuckle,
        elem_b=shoulder_body,
        name="elbow_knuckle_overlaps_shoulder_clevis_window",
    )
    ctx.expect_within(
        wrist,
        elbow,
        axes="yz",
        margin=0.004,
        inner_elem=wrist_hub,
        outer_elem=wrist_cage,
        name="wrist_hub_within_support_cage_envelope",
    )
    ctx.expect_overlap(
        wrist,
        elbow,
        axes="yz",
        min_overlap=0.018,
        elem_a=wrist_hub,
        elem_b=wrist_cage,
        name="wrist_hub_centered_inside_support_cage",
    )
    ctx.expect_overlap(
        wrist,
        elbow,
        axes="yz",
        min_overlap=0.03,
        elem_a=tool_plate,
        elem_b=wrist_cage,
        name="tool_plate_aligned_with_support_cage",
    )

    with ctx.pose({shoulder_joint: SHOULDER_RANGE * 0.55, elbow_joint: -ELBOW_RANGE * 0.65, wrist_joint: WRIST_RANGE * 0.5}):
        ctx.expect_contact(
            shoulder,
            shoulder,
            elem_a=shoulder_knuckle,
            elem_b=shoulder_body,
            name="shoulder_knuckle_remains_connected_to_arm_in_raised_pose",
        )
        ctx.expect_contact(
            elbow,
            elbow,
            elem_a=elbow_knuckle,
            elem_b=elbow_body,
            name="elbow_knuckle_remains_connected_to_forearm_in_raised_pose",
        )
        ctx.expect_within(
            wrist,
            elbow,
            axes="yz",
            margin=0.004,
            inner_elem=wrist_hub,
            outer_elem=wrist_cage,
            name="wrist_mount_stays_seated_in_raised_pose",
        )

    with ctx.pose({wrist_joint: -WRIST_RANGE}):
        ctx.expect_within(
            wrist,
            elbow,
            axes="yz",
            margin=0.004,
            inner_elem=wrist_hub,
            outer_elem=wrist_cage,
            name="wrist_mount_stays_seated_at_negative_roll_limit",
        )
    with ctx.pose({wrist_joint: WRIST_RANGE}):
        ctx.expect_within(
            wrist,
            elbow,
            axes="yz",
            margin=0.004,
            inner_elem=wrist_hub,
            outer_elem=wrist_cage,
            name="wrist_mount_stays_seated_at_positive_roll_limit",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
