from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.22, 0.23, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.37, 0.40, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.30, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="base_plate",
    )
    frame.visual(
        Cylinder(radius=0.33, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_iron,
        name="base_rim",
    )
    frame.visual(
        Cylinder(radius=0.085, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=cast_iron,
        name="column_socket",
    )
    frame.visual(
        Cylinder(radius=0.055, length=1.28),
        origin=Origin(xyz=(0.0, 0.0, 0.69)),
        material=steel,
        name="column",
    )
    frame.visual(
        Box((0.018, 0.028, 0.70)),
        origin=Origin(xyz=(0.065, 0.0, 0.48)),
        material=dark_steel,
        name="rack_strip",
    )
    for idx, tooth_z in enumerate((0.16, 0.23, 0.30, 0.37, 0.44, 0.51, 0.58, 0.65, 0.72)):
        frame.visual(
            Box((0.008, 0.028, 0.022)),
            origin=Origin(xyz=(0.078, 0.0, tooth_z)),
            material=dark_steel,
            name=f"rack_tooth_{idx + 1}",
        )

    frame.visual(
        Box((0.14, 0.14, 0.18)),
        origin=Origin(xyz=(0.02, 0.0, 1.26)),
        material=cast_iron,
        name="head_collar",
    )
    frame.visual(
        Box((0.08, 0.10, 0.20)),
        origin=Origin(xyz=(0.11, 0.0, 1.25)),
        material=cast_iron,
        name="head_spine",
    )
    frame.visual(
        Box((0.34, 0.24, 0.10)),
        origin=Origin(xyz=(0.10, 0.0, 1.37)),
        material=cast_iron,
        name="head_top",
    )
    frame.visual(
        Box((0.28, 0.05, 0.18)),
        origin=Origin(xyz=(0.06, 0.095, 1.28)),
        material=cast_iron,
        name="head_left",
    )
    frame.visual(
        Box((0.28, 0.05, 0.18)),
        origin=Origin(xyz=(0.06, -0.095, 1.28)),
        material=cast_iron,
        name="head_right",
    )
    frame.visual(
        Box((0.14, 0.05, 0.16)),
        origin=Origin(xyz=(0.18, 0.055, 1.21)),
        material=cast_iron,
        name="nose_left",
    )
    frame.visual(
        Box((0.14, 0.05, 0.16)),
        origin=Origin(xyz=(0.18, -0.055, 1.21)),
        material=cast_iron,
        name="nose_right",
    )
    frame.visual(
        Box((0.26, 0.24, 0.16)),
        origin=Origin(xyz=(-0.05, 0.0, 1.44)),
        material=cast_iron,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.05, length=0.24),
        origin=Origin(xyz=(0.20, 0.0, 1.19)),
        material=dark_steel,
        name="quill_sleeve",
    )

    carriage = model.part("table_carriage")
    carriage.visual(
        Box((0.08, 0.03, 0.18)),
        origin=Origin(xyz=(0.040, 0.070, 0.0)),
        material=cast_iron,
        name="clamp_left",
    )
    carriage.visual(
        Box((0.08, 0.03, 0.18)),
        origin=Origin(xyz=(0.040, -0.070, 0.0)),
        material=cast_iron,
        name="clamp_right",
    )
    carriage.visual(
        Box((0.12, 0.03, 0.06)),
        origin=Origin(xyz=(0.090, 0.070, -0.045)),
        material=cast_iron,
        name="arm_left",
    )
    carriage.visual(
        Box((0.12, 0.03, 0.06)),
        origin=Origin(xyz=(0.090, -0.070, -0.045)),
        material=cast_iron,
        name="arm_right",
    )
    carriage.visual(
        Box((0.06, 0.15, 0.03)),
        origin=Origin(xyz=(0.120, 0.0, -0.060)),
        material=cast_iron,
        name="front_bridge_bottom",
    )
    carriage.visual(
        Box((0.035, 0.03, 0.12)),
        origin=Origin(xyz=(0.120, 0.082, 0.03)),
        material=cast_iron,
        name="yoke_left",
    )
    carriage.visual(
        Box((0.035, 0.03, 0.12)),
        origin=Origin(xyz=(0.120, -0.082, 0.03)),
        material=cast_iron,
        name="yoke_right",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.04),
        origin=Origin(xyz=(0.04, -0.105, -0.03), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handwheel_hub",
    )
    carriage.visual(
        Box((0.01, 0.04, 0.01)),
        origin=Origin(xyz=(0.04, -0.145, 0.0)),
        material=steel,
        name="handwheel_handle",
    )
    carriage.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.04, -0.165, 0.0)),
        material=handle_black,
        name="handwheel_knob",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.018, length=0.134),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion",
    )
    table.visual(
        Box((0.24, 0.07, 0.04)),
        origin=Origin(xyz=(0.12, 0.0, 0.03)),
        material=cast_iron,
        name="support_arm",
    )
    table.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.16, 0.0, 0.055)),
        material=cast_iron,
        name="table_boss",
    )
    table.visual(
        Box((0.12, 0.03, 0.05)),
        origin=Origin(xyz=(0.18, 0.0, 0.055)),
        material=cast_iron,
        name="rib",
    )
    table.visual(
        Cylinder(radius=0.15, length=0.025),
        origin=Origin(xyz=(0.24, 0.0, 0.085)),
        material=cast_iron,
        name="table_top",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.032, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, -0.13)),
        material=steel,
        name="quill_body",
    )
    spindle.visual(
        Cylinder(radius=0.040, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.31)),
        material=dark_steel,
        name="chuck_body",
    )
    spindle.visual(
        Cylinder(radius=0.020, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, -0.39)),
        material=dark_steel,
        name="chuck_nose",
    )
    spindle.visual(
        Cylinder(radius=0.005, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.49)),
        material=steel,
        name="drill_bit",
    )

    feed_lever = model.part("feed_lever")
    feed_lever.visual(
        Cylinder(radius=0.028, length=0.05),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    for idx, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        arm_pitch = (pi / 2.0) - angle
        arm_center = (0.07 * cos(angle), 0.0, 0.07 * sin(angle))
        knob_center = (0.156 * cos(angle), 0.0, 0.156 * sin(angle))
        feed_lever.visual(
            Cylinder(radius=0.008, length=0.14),
            origin=Origin(xyz=arm_center, rpy=(0.0, arm_pitch, 0.0)),
            material=steel,
            name=f"arm_{idx + 1}",
        )
        feed_lever.visual(
            Sphere(radius=0.016),
            origin=Origin(xyz=knob_center),
            material=handle_black,
            name=f"knob_{idx + 1}",
        )

    model.articulation(
        "table_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.30),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=table,
        origin=Origin(xyz=(0.12, 0.0, 0.03)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.00, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "quill_feed",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=spindle,
        origin=Origin(xyz=(0.20, 0.0, 1.31)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.12, lower=0.0, upper=0.08),
    )
    model.articulation(
        "feed_lever_rotation",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=feed_lever,
        origin=Origin(xyz=(0.10, -0.145, 1.24)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.50, lower=-1.20, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    def _aabb_max_z(aabb):
        if aabb is None:
            return None
        _, maxs = aabb
        return maxs[2]

    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    spindle = object_model.get_part("spindle")
    feed_lever = object_model.get_part("feed_lever")

    table_lift = object_model.get_articulation("table_lift")
    table_tilt = object_model.get_articulation("table_tilt")
    quill_feed = object_model.get_articulation("quill_feed")
    feed_lever_rotation = object_model.get_articulation("feed_lever_rotation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        frame,
        spindle,
        elem_a="quill_sleeve",
        elem_b="quill_body",
        reason="The quill body is intentionally represented sliding inside the head sleeve.",
    )

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
        "table lift axis is vertical",
        tuple(table_lift.axis) == (0.0, 0.0, 1.0),
        details=f"axis={table_lift.axis}",
    )
    ctx.check(
        "table tilt axis is lateral",
        tuple(table_tilt.axis) == (0.0, -1.0, 0.0),
        details=f"axis={table_tilt.axis}",
    )
    ctx.check(
        "quill feed axis is downward",
        tuple(quill_feed.axis) == (0.0, 0.0, -1.0),
        details=f"axis={quill_feed.axis}",
    )
    ctx.check(
        "feed lever rotates on head side shaft",
        tuple(feed_lever_rotation.axis) == (0.0, -1.0, 0.0),
        details=f"axis={feed_lever_rotation.axis}",
    )

    ctx.expect_contact(
        carriage,
        frame,
        elem_a="clamp_left",
        elem_b="column",
        name="left clamp jaw bears on the column",
    )
    ctx.expect_contact(
        carriage,
        frame,
        elem_a="clamp_right",
        elem_b="column",
        name="right clamp jaw bears on the column",
    )
    ctx.expect_contact(
        table,
        carriage,
        elem_a="trunnion",
        elem_b="yoke_left",
        name="table trunnion seats in left yoke",
    )
    ctx.expect_contact(
        table,
        carriage,
        elem_a="trunnion",
        elem_b="yoke_right",
        name="table trunnion seats in right yoke",
    )
    ctx.expect_contact(
        feed_lever,
        frame,
        elem_a="hub",
        elem_b="head_right",
        name="feed lever hub mounts against the head",
    )
    ctx.expect_within(
        spindle,
        frame,
        axes="xy",
        inner_elem="quill_body",
        outer_elem="quill_sleeve",
        margin=0.002,
        name="quill stays centered within the sleeve",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="z",
        elem_a="quill_body",
        elem_b="quill_sleeve",
        min_overlap=0.18,
        name="retracted quill has deep sleeve engagement",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({table_lift: table_lift.motion_limits.upper}):
        carriage_raised = ctx.part_world_position(carriage)
    ctx.check(
        "table carriage climbs the column",
        carriage_rest is not None
        and carriage_raised is not None
        and carriage_raised[2] > carriage_rest[2] + 0.25,
        details=f"rest={carriage_rest}, raised={carriage_raised}",
    )

    table_rest_top = _aabb_max_z(ctx.part_element_world_aabb(table, elem="table_top"))
    with ctx.pose({table_tilt: 0.35}):
        table_tilted_top = _aabb_max_z(ctx.part_element_world_aabb(table, elem="table_top"))
    ctx.check(
        "table front rises when tilted",
        table_rest_top is not None
        and table_tilted_top is not None
        and table_tilted_top > table_rest_top + 0.05,
        details=f"rest={table_rest_top}, tilted={table_tilted_top}",
    )

    spindle_rest = ctx.part_world_position(spindle)
    with ctx.pose({quill_feed: 0.04}):
        spindle_advanced = ctx.part_world_position(spindle)
        ctx.expect_overlap(
            spindle,
            frame,
            axes="z",
            elem_a="quill_body",
            elem_b="quill_sleeve",
            min_overlap=0.13,
            name="advanced quill retains insertion in the sleeve",
        )
    ctx.check(
        "quill advances downward",
        spindle_rest is not None
        and spindle_advanced is not None
        and spindle_advanced[2] < spindle_rest[2] - 0.035,
        details=f"rest={spindle_rest}, advanced={spindle_advanced}",
    )

    knob_rest = _aabb_center(ctx.part_element_world_aabb(feed_lever, elem="knob_1"))
    with ctx.pose({feed_lever_rotation: 0.60}):
        knob_raised = _aabb_center(ctx.part_element_world_aabb(feed_lever, elem="knob_1"))
    ctx.check(
        "feed lever swings a handle through an arc",
        knob_rest is not None
        and knob_raised is not None
        and knob_raised[2] > knob_rest[2] + 0.05,
        details=f"rest={knob_rest}, raised={knob_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
