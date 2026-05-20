from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_swivel_stool")

    powder_coat = model.material("powder_coat", rgba=(0.16, 0.16, 0.17, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    vinyl = model.material("vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    back_vinyl = model.material("back_vinyl", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.22, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=powder_coat,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.09, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=powder_coat,
        name="lower_shroud",
    )
    base.visual(
        Cylinder(radius=0.05, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=powder_coat,
        name="column_sleeve",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.035, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
        material=brushed_steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.05, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material=brushed_steel,
        name="swivel_cap",
    )

    seat_frame = model.part("seat_frame")
    seat_frame.visual(
        Cylinder(radius=0.075, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=powder_coat,
        name="swivel_mount",
    )
    seat_frame.visual(
        Box((0.21, 0.21, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=powder_coat,
        name="underseat_plate",
    )
    seat_frame.visual(
        Box((0.39, 0.39, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=powder_coat,
        name="seat_pan",
    )
    seat_frame.visual(
        Box((0.42, 0.42, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material=vinyl,
        name="seat_cushion",
    )
    seat_frame.visual(
        Box((0.04, 0.07, 0.012)),
        origin=Origin(xyz=(0.10, 0.0, 0.034)),
        material=powder_coat,
        name="tab_bridge",
    )
    seat_frame.visual(
        Box((0.028, 0.012, 0.032)),
        origin=Origin(xyz=(0.11, 0.017, 0.02)),
        material=powder_coat,
        name="tab_ear_0",
    )
    seat_frame.visual(
        Box((0.028, 0.012, 0.032)),
        origin=Origin(xyz=(0.11, -0.017, 0.02)),
        material=powder_coat,
        name="tab_ear_1",
    )
    seat_frame.visual(
        Box((0.03, 0.03, 0.18)),
        origin=Origin(xyz=(-0.155, 0.145, 0.175)),
        material=powder_coat,
        name="back_post_0",
    )
    seat_frame.visual(
        Box((0.03, 0.03, 0.18)),
        origin=Origin(xyz=(-0.155, -0.145, 0.175)),
        material=powder_coat,
        name="back_post_1",
    )
    seat_frame.visual(
        Box((0.04, 0.38, 0.18)),
        origin=Origin(xyz=(-0.17, 0.0, 0.355)),
        material=back_vinyl,
        name="back_panel",
    )

    release_tab = model.part("release_tab")
    release_tab.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=brushed_steel,
        name="pivot_barrel",
    )
    release_tab.visual(
        Box((0.07, 0.018, 0.012)),
        origin=Origin(xyz=(0.034, 0.0, -0.01)),
        material=powder_coat,
        name="tab_arm",
    )
    release_tab.visual(
        Box((0.03, 0.028, 0.016)),
        origin=Origin(xyz=(0.07, 0.0, -0.024)),
        material=powder_coat,
        name="tab_paddle",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.18,
            effort=250.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "column_to_seat_frame",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=seat_frame,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.5),
    )
    model.articulation(
        "seat_frame_to_release_tab",
        ArticulationType.REVOLUTE,
        parent=seat_frame,
        child=release_tab,
        origin=Origin(xyz=(0.13, 0.0, 0.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.45,
            effort=8.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    seat_frame = object_model.get_part("seat_frame")
    release_tab = object_model.get_part("release_tab")

    lift = object_model.get_articulation("base_to_column")
    swivel = object_model.get_articulation("column_to_seat_frame")
    tab_joint = object_model.get_articulation("seat_frame_to_release_tab")

    ctx.allow_overlap(
        base,
        column,
        elem_a="lower_shroud",
        elem_b="column_shaft",
        reason="The bell-shaped lower shroud is represented as a solid proxy around the lift column.",
    )
    ctx.allow_overlap(
        base,
        column,
        elem_a="column_sleeve",
        elem_b="column_shaft",
        reason="The telescoping outer sleeve is represented by a solid cylinder proxy enclosing the sliding inner shaft.",
    )

    ctx.expect_within(
        column,
        base,
        axes="xy",
        inner_elem="column_shaft",
        outer_elem="column_sleeve",
        margin=0.001,
        name="column stays centered in the sleeve at rest",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="column_shaft",
        elem_b="column_sleeve",
        min_overlap=0.28,
        name="column remains deeply inserted at the low height",
    )
    ctx.expect_origin_distance(
        seat_frame,
        column,
        axes="xy",
        max_dist=0.001,
        name="seat support stays coaxial with the lift column",
    )

    rest_seat_pos = ctx.part_world_position(seat_frame)
    rest_paddle_aabb = ctx.part_element_world_aabb(release_tab, elem="tab_paddle")

    with ctx.pose({lift: 0.18, swivel: 1.57079632679}):
        ctx.expect_within(
            column,
            base,
            axes="xy",
            inner_elem="column_shaft",
            outer_elem="column_sleeve",
            margin=0.001,
            name="column stays centered in the sleeve at full height",
        )
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="column_shaft",
            elem_b="column_sleeve",
            min_overlap=0.20,
            name="column retains insertion at full extension",
        )
        ctx.expect_origin_distance(
            seat_frame,
            column,
            axes="xy",
            max_dist=0.001,
            name="seat remains coaxial while swiveled",
        )
        high_seat_pos = ctx.part_world_position(seat_frame)

    with ctx.pose({tab_joint: 0.45}):
        raised_paddle_aabb = ctx.part_element_world_aabb(release_tab, elem="tab_paddle")

    rest_top_z = None if rest_paddle_aabb is None else rest_paddle_aabb[1][2]
    raised_top_z = None if raised_paddle_aabb is None else raised_paddle_aabb[1][2]

    ctx.check(
        "seat height adjustment raises the seat",
        rest_seat_pos is not None
        and high_seat_pos is not None
        and high_seat_pos[2] > rest_seat_pos[2] + 0.10,
        details=f"rest={rest_seat_pos}, high={high_seat_pos}",
    )
    ctx.check(
        "front release tab lifts when actuated",
        rest_top_z is not None and raised_top_z is not None and raised_top_z > rest_top_z + 0.015,
        details=f"rest_top_z={rest_top_z}, raised_top_z={raised_top_z}",
    )

    return ctx.report()


object_model = build_object_model()
