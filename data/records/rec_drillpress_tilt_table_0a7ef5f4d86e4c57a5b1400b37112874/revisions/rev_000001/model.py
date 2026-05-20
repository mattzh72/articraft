from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_drill_press")

    frame_color = model.material("frame_green", rgba=(0.18, 0.34, 0.26, 1.0))
    column_color = model.material("column_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    table_color = model.material("table_iron", rgba=(0.34, 0.35, 0.37, 1.0))
    steel_color = model.material("steel", rgba=(0.70, 0.71, 0.73, 1.0))
    grip_color = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.72, 0.50, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=frame_color,
        name="base_plate",
    )
    frame.visual(
        Box((0.22, 0.24, 0.12)),
        origin=Origin(xyz=(0.0, -0.10, 0.12)),
        material=frame_color,
        name="base_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.055, length=1.10),
        origin=Origin(xyz=(0.0, -0.12, 0.73)),
        material=column_color,
        name="column",
    )
    frame.visual(
        Box((0.34, 0.22, 0.24)),
        origin=Origin(xyz=(0.0, -0.01, 1.18)),
        material=frame_color,
        name="head_casting",
    )
    frame.visual(
        Box((0.30, 0.22, 0.11)),
        origin=Origin(xyz=(0.0, -0.02, 1.325)),
        material=frame_color,
        name="belt_cover",
    )
    frame.visual(
        Cylinder(radius=0.095, length=0.34),
        origin=Origin(xyz=(0.0, -0.02, 1.40), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_color,
        name="motor_housing",
    )
    frame.visual(
        Box((0.14, 0.10, 0.09)),
        origin=Origin(xyz=(0.0, 0.10, 1.19)),
        material=frame_color,
        name="spindle_nose",
    )
    spindle_sleeve_shell = LatheGeometry.from_shell_profiles(
        [(0.058, -0.11), (0.058, 0.11)],
        [(0.041, -0.11), (0.041, 0.11)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    frame.visual(
        mesh_from_geometry(spindle_sleeve_shell, "spindle_sleeve"),
        origin=Origin(xyz=(0.0, 0.15, 1.00)),
        material=column_color,
        name="spindle_sleeve",
    )
    frame.visual(
        Box((0.018, 0.03, 0.10)),
        origin=Origin(xyz=(0.047, 0.15, 1.05)),
        material=steel_color,
        name="quill_guide_pad",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.09),
        origin=Origin(xyz=(0.195, 0.03, 1.17), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_color,
        name="feed_hub",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.18),
        origin=Origin(xyz=(0.195, 0.03, 1.24)),
        material=steel_color,
        name="feed_handle_upper",
    )
    frame.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.195, 0.03, 1.33)),
        material=grip_color,
        name="feed_grip_upper",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.18),
        origin=Origin(xyz=(0.195, 0.0906, 1.135), rpy=(-2.0 * pi / 3.0, 0.0, 0.0)),
        material=steel_color,
        name="feed_handle_front",
    )
    frame.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.195, 0.1686, 1.09)),
        material=grip_color,
        name="feed_grip_front",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.18),
        origin=Origin(xyz=(0.195, -0.0306, 1.135), rpy=(2.0 * pi / 3.0, 0.0, 0.0)),
        material=steel_color,
        name="feed_handle_rear",
    )
    frame.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.195, -0.1086, 1.09)),
        material=grip_color,
        name="feed_grip_rear",
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Box((0.18, 0.05, 0.14)),
        origin=Origin(xyz=(0.0, 0.08, 0.0)),
        material=frame_color,
        name="carriage_front_bridge",
    )
    table_carriage.visual(
        Box((0.04, 0.12, 0.14)),
        origin=Origin(xyz=(0.086, 0.025, 0.0)),
        material=frame_color,
        name="carriage_right_cheek",
    )
    table_carriage.visual(
        Box((0.04, 0.12, 0.14)),
        origin=Origin(xyz=(-0.086, 0.025, 0.0)),
        material=frame_color,
        name="carriage_left_cheek",
    )
    table_carriage.visual(
        Box((0.09, 0.21, 0.03)),
        origin=Origin(xyz=(0.0, 0.185, -0.059)),
        material=frame_color,
        name="table_arm",
    )
    table_carriage.visual(
        Box((0.16, 0.055, 0.042)),
        origin=Origin(xyz=(0.0, 0.29, -0.059)),
        material=frame_color,
        name="tilt_bracket_lower",
    )
    table_carriage.visual(
        Box((0.04, 0.055, 0.12)),
        origin=Origin(xyz=(0.095, 0.29, -0.015)),
        material=frame_color,
        name="tilt_bracket_right_ear",
    )
    table_carriage.visual(
        Box((0.04, 0.055, 0.12)),
        origin=Origin(xyz=(-0.095, 0.29, -0.015)),
        material=frame_color,
        name="tilt_bracket_left_ear",
    )
    table = model.part("table")
    table.visual(
        Cylinder(radius=0.038, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=table_color,
        name="table_trunnion",
    )
    table.visual(
        Box((0.12, 0.18, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=table_color,
        name="table_support_web",
    )
    table.visual(
        Cylinder(radius=0.20, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=table_color,
        name="table_top",
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.038, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=steel_color,
        name="quill_body",
    )
    quill.visual(
        Cylinder(radius=0.030, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, -0.215)),
        material=steel_color,
        name="chuck_body",
    )
    quill.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.275)),
        material=steel_color,
        name="spindle_tip",
    )

    model.articulation(
        "frame_to_table_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_carriage,
        origin=Origin(xyz=(0.0, -0.12, 0.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.18,
            lower=0.0,
            upper=0.40,
        ),
    )
    model.articulation(
        "carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.0, 0.29, 0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-0.80,
            upper=0.80,
        ),
    )
    model.articulation(
        "frame_to_quill",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=quill,
        origin=Origin(xyz=(0.0, 0.15, 1.09)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.25,
            lower=0.0,
            upper=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    table_carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    quill = object_model.get_part("quill")

    carriage_slide = object_model.get_articulation("frame_to_table_carriage")
    table_tilt = object_model.get_articulation("carriage_to_table")
    quill_slide = object_model.get_articulation("frame_to_quill")

    ctx.expect_contact(
        table_carriage,
        frame,
        elem_a="carriage_front_bridge",
        elem_b="column",
        contact_tol=0.0005,
        name="table carriage is mounted on the round column",
    )
    ctx.expect_overlap(
        table,
        quill,
        axes="xy",
        elem_a="table_top",
        elem_b="quill_body",
        min_overlap=0.06,
        name="table sits beneath the spindle line",
    )
    ctx.expect_within(
        quill,
        frame,
        axes="xy",
        inner_elem="quill_body",
        outer_elem="spindle_sleeve",
        margin=0.004,
        name="quill stays centered in the spindle sleeve",
    )

    rest_table_pos = ctx.part_world_position(table)
    with ctx.pose({carriage_slide: 0.32}):
        raised_table_pos = ctx.part_world_position(table)
    ctx.check(
        "table raises up the column",
        rest_table_pos is not None
        and raised_table_pos is not None
        and raised_table_pos[2] > rest_table_pos[2] + 0.25,
        details=f"rest={rest_table_pos}, raised={raised_table_pos}",
    )

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    with ctx.pose({table_tilt: 0.55}):
        tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    ctx.check(
        "table tilt articulation changes the tabletop orientation",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and (tilted_table_aabb[1][2] - tilted_table_aabb[0][2])
        > (rest_table_aabb[1][2] - rest_table_aabb[0][2]) + 0.08,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    rest_quill_pos = ctx.part_world_position(quill)
    with ctx.pose({quill_slide: 0.10}):
        extended_quill_pos = ctx.part_world_position(quill)
        ctx.expect_within(
            quill,
            frame,
            axes="xy",
            inner_elem="quill_body",
            outer_elem="spindle_sleeve",
            margin=0.004,
            name="extended quill remains guided by the spindle sleeve",
        )
    ctx.check(
        "quill feed drops the spindle downward",
        rest_quill_pos is not None
        and extended_quill_pos is not None
        and extended_quill_pos[2] < rest_quill_pos[2] - 0.08,
        details=f"rest={rest_quill_pos}, extended={extended_quill_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
