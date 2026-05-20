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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_drill_press")

    machine_green = model.material("machine_green", rgba=(0.24, 0.44, 0.31, 1.0))
    cast_gray = model.material("cast_gray", rgba=(0.47, 0.50, 0.52, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.95, 0.70, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_gray,
        name="base_plinth",
    )
    frame.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(-0.22, 0.0, 0.13)),
        material=cast_gray,
        name="column_base_flange",
    )
    frame.visual(
        Cylinder(radius=0.10, length=1.38),
        origin=Origin(xyz=(-0.22, 0.0, 0.77)),
        material=machine_green,
        name="column",
    )
    frame.visual(
        Cylinder(radius=0.15, length=0.06),
        origin=Origin(xyz=(-0.22, 0.0, 1.49)),
        material=dark_steel,
        name="swing_bearing_cap",
    )
    frame.visual(
        Box((0.09, 0.16, 0.14)),
        origin=Origin(xyz=(-0.105, 0.0, 0.68)),
        material=cast_gray,
        name="table_clamp",
    )
    frame.visual(
        Box((0.02, 0.02, 0.12)),
        origin=Origin(xyz=(-0.05, -0.04, 0.68)),
        material=dark_steel,
        name="left_knuckle_support",
    )
    frame.visual(
        Box((0.02, 0.02, 0.12)),
        origin=Origin(xyz=(-0.05, 0.04, 0.68)),
        material=dark_steel,
        name="right_knuckle_support",
    )
    frame.visual(
        Cylinder(radius=0.02, length=0.02),
        origin=Origin(xyz=(-0.02, -0.04, 0.68), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_knuckle",
    )
    frame.visual(
        Cylinder(radius=0.02, length=0.02),
        origin=Origin(xyz=(-0.02, 0.04, 0.68), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_knuckle",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.95, 0.70, 1.52)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
    )

    arm = model.part("radial_arm")
    arm.visual(
        Cylinder(radius=0.17, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_steel,
        name="turret_turntable",
    )
    arm.visual(
        Box((0.18, 0.24, 0.12)),
        origin=Origin(xyz=(0.09, 0.0, 0.09)),
        material=cast_gray,
        name="arm_root_casting",
    )
    arm.visual(
        Box((0.92, 0.20, 0.14)),
        origin=Origin(xyz=(0.61, 0.0, 0.09)),
        material=machine_green,
        name="arm_beam",
    )
    arm.visual(
        Box((0.90, 0.12, 0.03)),
        origin=Origin(xyz=(0.61, 0.0, 0.015)),
        material=cast_gray,
        name="lower_way",
    )
    arm.inertial = Inertial.from_geometry(
        Box((1.12, 0.26, 0.22)),
        mass=60.0,
        origin=Origin(xyz=(0.56, 0.0, 0.07)),
    )

    head = model.part("drill_head")
    head.visual(
        Box((0.22, 0.16, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material=dark_steel,
        name="carriage",
    )
    head.visual(
        Box((0.20, 0.03, 0.18)),
        origin=Origin(xyz=(-0.02, 0.055, -0.11)),
        material=machine_green,
        name="left_cheek",
    )
    head.visual(
        Box((0.20, 0.03, 0.18)),
        origin=Origin(xyz=(-0.02, -0.055, -0.11)),
        material=machine_green,
        name="right_cheek",
    )
    head.visual(
        Box((0.08, 0.14, 0.10)),
        origin=Origin(xyz=(-0.10, 0.0, -0.08)),
        material=machine_green,
        name="motor_housing",
    )
    head.visual(
        Box((0.08, 0.12, 0.10)),
        origin=Origin(xyz=(0.08, 0.0, -0.09)),
        material=cast_gray,
        name="gearbox_nose",
    )
    head.visual(
        Box((0.02, 0.05, 0.24)),
        origin=Origin(xyz=(-0.05, 0.0, -0.14)),
        material=dark_steel,
        name="left_rail",
    )
    head.visual(
        Box((0.02, 0.05, 0.24)),
        origin=Origin(xyz=(0.05, 0.0, -0.14)),
        material=dark_steel,
        name="right_rail",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.30, 0.18, 0.28)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
    )

    quill = model.part("quill")
    quill.visual(
        Box((0.08, 0.05, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=steel,
        name="guide_bridge",
    )
    quill.visual(
        Box((0.02, 0.05, 0.16)),
        origin=Origin(xyz=(-0.03, 0.0, -0.11)),
        material=steel,
        name="left_pad",
    )
    quill.visual(
        Box((0.02, 0.05, 0.16)),
        origin=Origin(xyz=(0.03, 0.0, -0.11)),
        material=steel,
        name="right_pad",
    )
    quill.visual(
        Box((0.06, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=dark_steel,
        name="spindle_collar",
    )
    quill.visual(
        Cylinder(radius=0.022, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
        material=steel,
        name="quill_body",
    )
    quill.inertial = Inertial.from_geometry(
        Box((0.10, 0.06, 0.28)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, -0.14)),
    )

    chuck = model.part("chuck")
    chuck.visual(
        Cylinder(radius=0.028, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=dark_steel,
        name="chuck_body",
    )
    chuck.inertial = Inertial.from_geometry(Cylinder(radius=0.028, length=0.08), mass=2.0)

    bit = model.part("bit")
    bit.visual(
        Cylinder(radius=0.008, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=steel,
        name="drill_bit",
    )
    bit.inertial = Inertial.from_geometry(Cylinder(radius=0.008, length=0.18), mass=0.3)

    table = model.part("work_table")
    table.visual(
        Cylinder(radius=0.02, length=0.06),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="center_knuckle",
    )
    table.visual(
        Box((0.18, 0.08, 0.04)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        material=cast_gray,
        name="table_support_arm",
    )
    table.visual(
        Cylinder(radius=0.04, length=0.14),
        origin=Origin(xyz=(0.24, 0.0, 0.09)),
        material=dark_steel,
        name="table_post",
    )
    table.visual(
        Cylinder(radius=0.16, length=0.03),
        origin=Origin(xyz=(0.24, 0.0, 0.17)),
        material=steel,
        name="table_top",
    )
    table.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=0.24),
        mass=18.0,
        origin=Origin(xyz=(0.24, 0.0, 0.12)),
    )

    model.articulation(
        "arm_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=arm,
        origin=Origin(xyz=(-0.22, 0.0, 1.52)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.6,
            lower=-math.radians(125.0),
            upper=math.radians(125.0),
        ),
    )
    model.articulation(
        "head_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.20,
            lower=0.0,
            upper=0.34,
        ),
    )
    model.articulation(
        "quill_feed",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.16,
            lower=0.0,
            upper=0.07,
        ),
    )
    model.articulation(
        "chuck_spin",
        ArticulationType.CONTINUOUS,
        parent=quill,
        child=chuck,
        origin=Origin(xyz=(0.0, 0.0, -0.27)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=12.0),
    )
    model.articulation(
        "chuck_to_bit",
        ArticulationType.FIXED,
        parent=chuck,
        child=bit,
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=table,
        origin=Origin(xyz=(-0.02, 0.0, 0.68)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.7,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    arm = object_model.get_part("radial_arm")
    head = object_model.get_part("drill_head")
    quill = object_model.get_part("quill")
    chuck = object_model.get_part("chuck")
    bit = object_model.get_part("bit")
    table = object_model.get_part("work_table")

    arm_swing = object_model.get_articulation("arm_swing")
    head_slide = object_model.get_articulation("head_slide")
    quill_feed = object_model.get_articulation("quill_feed")
    chuck_spin = object_model.get_articulation("chuck_spin")
    table_tilt = object_model.get_articulation("table_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("arm_swing_axis_vertical", arm_swing.axis == (0.0, 0.0, 1.0), str(arm_swing.axis))
    ctx.check("head_slide_axis_along_arm", head_slide.axis == (1.0, 0.0, 0.0), str(head_slide.axis))
    ctx.check("quill_feed_axis_down", quill_feed.axis == (0.0, 0.0, -1.0), str(quill_feed.axis))
    ctx.check("table_tilt_axis_lateral", table_tilt.axis == (0.0, 1.0, 0.0), str(table_tilt.axis))
    ctx.check("chuck_spin_axis_vertical", chuck_spin.axis == (0.0, 0.0, 1.0), str(chuck_spin.axis))

    ctx.expect_contact(arm, frame, elem_a="turret_turntable", elem_b="swing_bearing_cap")
    ctx.expect_contact(head, arm, elem_a="carriage", elem_b="lower_way")
    ctx.expect_contact(quill, head, elem_a="left_pad", elem_b="left_rail")
    ctx.expect_contact(quill, head, elem_a="right_pad", elem_b="right_rail")
    ctx.expect_contact(chuck, quill, elem_a="chuck_body", elem_b="quill_body")
    ctx.expect_contact(bit, chuck, elem_a="drill_bit", elem_b="chuck_body")
    ctx.expect_contact(table, frame, elem_a="center_knuckle", elem_b="left_knuckle")
    ctx.expect_contact(table, frame, elem_a="center_knuckle", elem_b="right_knuckle")

    ctx.expect_overlap(bit, table, axes="xy", elem_a="drill_bit", elem_b="table_top", min_overlap=0.01)
    ctx.expect_gap(
        bit,
        table,
        axis="z",
        positive_elem="drill_bit",
        negative_elem="table_top",
        min_gap=0.12,
        max_gap=0.16,
        name="bit_rest_gap_above_table",
    )

    rest_head = ctx.part_world_position(head)
    assert rest_head is not None
    with ctx.pose({head_slide: 0.35}):
        moved_head = ctx.part_world_position(head)
        assert moved_head is not None
        ctx.check(
            "head_slide_moves_forward",
            moved_head[0] > rest_head[0] + 0.30,
            f"rest={rest_head}, moved={moved_head}",
        )
        ctx.expect_contact(head, arm, elem_a="carriage", elem_b="lower_way")

    rest_bit_origin = ctx.part_world_position(bit)
    assert rest_bit_origin is not None
    with ctx.pose({arm_swing: math.radians(90.0)}):
        swung_bit_origin = ctx.part_world_position(bit)
        assert swung_bit_origin is not None
        ctx.check(
            "arm_swing_reaches_side_position",
            swung_bit_origin[1] > rest_bit_origin[1] + 0.30,
            f"rest={rest_bit_origin}, swung={swung_bit_origin}",
        )
        ctx.expect_contact(arm, frame, elem_a="turret_turntable", elem_b="swing_bearing_cap")

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    assert rest_table_aabb is not None
    rest_table_thickness = rest_table_aabb[1][2] - rest_table_aabb[0][2]
    with ctx.pose({table_tilt: math.radians(40.0)}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
        assert tilted_table_aabb is not None
        tilted_table_thickness = tilted_table_aabb[1][2] - tilted_table_aabb[0][2]
        ctx.check(
            "table_tilt_changes_table_plane",
            tilted_table_thickness > rest_table_thickness + 0.10,
            f"rest={rest_table_thickness}, tilted={tilted_table_thickness}",
        )
        ctx.expect_contact(table, frame, elem_a="center_knuckle", elem_b="left_knuckle")

    with ctx.pose({quill_feed: 0.07}):
        lowered_bit_origin = ctx.part_world_position(bit)
        assert lowered_bit_origin is not None
        ctx.check(
            "quill_feed_lowers_bit",
            lowered_bit_origin[2] < rest_bit_origin[2] - 0.06,
            f"rest={rest_bit_origin}, lowered={lowered_bit_origin}",
        )
        ctx.expect_contact(quill, head, elem_a="right_pad", elem_b="right_rail")
        ctx.expect_gap(
            bit,
            table,
            axis="z",
            positive_elem="drill_bit",
            negative_elem="table_top",
            min_gap=0.05,
            name="bit_clear_of_table_at_full_quill",
        )

    def check_bounded_joint_limits(joint_name: str) -> None:
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            return
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint_name}_lower_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint_name}_upper_no_floating")

    for bounded_joint_name in ("arm_swing", "head_slide", "quill_feed", "table_tilt"):
        check_bounded_joint_limits(bounded_joint_name)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
