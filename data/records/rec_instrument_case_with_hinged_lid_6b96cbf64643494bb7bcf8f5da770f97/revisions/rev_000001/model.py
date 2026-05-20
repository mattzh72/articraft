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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drum_machine_case")

    model.material("case_body", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("lid_body", rgba=(0.15, 0.16, 0.17, 1.0))
    model.material("hardware", rgba=(0.65, 0.67, 0.70, 1.0))
    model.material("latch_finish", rgba=(0.78, 0.79, 0.81, 1.0))

    outer_depth = 0.24
    outer_width = 0.32
    base_height = 0.06
    wall_thickness = 0.005
    floor_thickness = 0.006

    lid_depth = 0.248
    lid_width = 0.332
    lid_wall_thickness = 0.004
    lid_top_thickness = 0.004
    lid_wall_height = 0.017
    lid_wall_center_z = -0.0015
    lid_top_center_z = 0.005

    hinge_radius = 0.004
    hinge_axis_z = 0.072
    base_knuckle_length = 0.012
    lid_knuckle_length = 0.014
    left_hinge_base_y = (-0.132, -0.104)
    right_hinge_base_y = (0.104, 0.132)
    lid_hinge_y = (-0.118, 0.118)

    latch_pivot_x = lid_depth - 0.006
    latch_pivot_z = 0.002

    base = model.part("base")
    base.visual(
        Box((outer_depth, outer_width, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material="case_body",
        name="base_floor",
    )
    base.visual(
        Box((outer_depth, wall_thickness, base_height)),
        origin=Origin(xyz=(0.0, outer_width / 2.0 - wall_thickness / 2.0, base_height / 2.0)),
        material="case_body",
        name="base_right_wall",
    )
    base.visual(
        Box((outer_depth, wall_thickness, base_height)),
        origin=Origin(xyz=(0.0, -outer_width / 2.0 + wall_thickness / 2.0, base_height / 2.0)),
        material="case_body",
        name="base_left_wall",
    )
    base.visual(
        Box((wall_thickness, outer_width, base_height)),
        origin=Origin(xyz=(outer_depth / 2.0 - wall_thickness / 2.0, 0.0, base_height / 2.0)),
        material="case_body",
        name="base_front_wall",
    )
    base.visual(
        Box((wall_thickness, outer_width, base_height)),
        origin=Origin(xyz=(-outer_depth / 2.0 + wall_thickness / 2.0, 0.0, base_height / 2.0)),
        material="case_body",
        name="base_rear_wall",
    )

    for index, hinge_y in enumerate(left_hinge_base_y + right_hinge_base_y, start=1):
        base.visual(
            Cylinder(radius=hinge_radius, length=base_knuckle_length),
            origin=Origin(xyz=(-outer_depth / 2.0, hinge_y, hinge_axis_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material="hardware",
            name=f"base_hinge_knuckle_{index}",
        )
        base.visual(
            Box((0.006, base_knuckle_length, 0.016)),
            origin=Origin(xyz=(-outer_depth / 2.0, hinge_y, 0.064)),
            material="hardware",
            name=f"base_hinge_bracket_{index}",
        )

    base.visual(
        Box((0.006, 0.026, 0.014)),
        origin=Origin(xyz=(outer_depth / 2.0, 0.0, 0.053)),
        material="hardware",
        name="base_latch_strike_plate",
    )
    base.visual(
        Cylinder(radius=0.0018, length=0.020),
        origin=Origin(xyz=(outer_depth / 2.0 + 0.002, 0.0, 0.056), rpy=(pi / 2.0, 0.0, 0.0)),
        material="hardware",
        name="base_latch_strike_bar",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_depth, lid_width, lid_top_thickness)),
        origin=Origin(xyz=(lid_depth / 2.0, 0.0, lid_top_center_z)),
        material="lid_body",
        name="lid_top_panel",
    )
    lid.visual(
        Box((lid_depth, lid_wall_thickness, lid_wall_height)),
        origin=Origin(xyz=(lid_depth / 2.0, lid_width / 2.0 - lid_wall_thickness / 2.0, lid_wall_center_z)),
        material="lid_body",
        name="lid_right_wall",
    )
    lid.visual(
        Box((lid_depth, lid_wall_thickness, lid_wall_height)),
        origin=Origin(xyz=(lid_depth / 2.0, -lid_width / 2.0 + lid_wall_thickness / 2.0, lid_wall_center_z)),
        material="lid_body",
        name="lid_left_wall",
    )
    lid.visual(
        Box((lid_wall_thickness, lid_width, lid_wall_height)),
        origin=Origin(xyz=(lid_depth - lid_wall_thickness / 2.0, 0.0, lid_wall_center_z)),
        material="lid_body",
        name="lid_front_wall",
    )

    for side_name, latch_ear_y in (("left", -0.012), ("right", 0.012)):
        lid.visual(
            Box((0.010, 0.006, 0.012)),
            origin=Origin(xyz=(latch_pivot_x - 0.002, latch_ear_y, 0.002)),
            material="hardware",
            name=f"lid_latch_ear_{side_name}",
        )

    for index, hinge_y in enumerate(lid_hinge_y, start=1):
        lid.visual(
            Cylinder(radius=hinge_radius, length=lid_knuckle_length),
            origin=Origin(xyz=(0.0, hinge_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="hardware",
            name=f"lid_hinge_knuckle_{index}",
        )
        lid.visual(
            Box((0.008, lid_knuckle_length, 0.010)),
            origin=Origin(xyz=(0.004, hinge_y, 0.002)),
            material="hardware",
            name=f"lid_hinge_leaf_{index}",
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.002, length=0.018),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="hardware",
        name="latch_pivot_barrel",
    )
    latch.visual(
        Box((0.003, 0.018, 0.022)),
        origin=Origin(xyz=(0.0025, 0.0, -0.011)),
        material="latch_finish",
        name="latch_plate",
    )
    latch.visual(
        Box((0.008, 0.018, 0.003)),
        origin=Origin(xyz=(0.006, 0.0, -0.0225)),
        material="latch_finish",
        name="latch_hook",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-outer_depth / 2.0, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=2.1,
        ),
    )
    model.articulation(
        "lid_to_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(latch_pivot_x, 0.0, latch_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=1.3,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    lid_hinge = object_model.get_articulation("base_to_lid")
    latch_pivot = object_model.get_articulation("lid_to_latch")

    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    latch_upper = latch_pivot.motion_limits.upper if latch_pivot.motion_limits is not None else None

    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.22,
        name="closed lid covers the case footprint",
    )

    ctx.check(
        "lid limit opens beyond vertical",
        lid_upper is not None and lid_upper > 1.57,
        details=f"upper={lid_upper}",
    )
    ctx.check(
        "latch has usable swing range",
        latch_upper is not None and latch_upper >= 1.0,
        details=f"upper={latch_upper}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_wall")
        closed_latch_plate = ctx.part_element_world_aabb(latch, elem="latch_plate")

    with ctx.pose({lid_hinge: lid_upper if lid_upper is not None else 0.0}):
        open_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_wall")

    with ctx.pose({latch_pivot: latch_upper if latch_upper is not None else 0.0}):
        open_latch_plate = ctx.part_element_world_aabb(latch, elem="latch_plate")

    lid_front_lifts = (
        closed_lid_front is not None
        and open_lid_front is not None
        and open_lid_front[0][2] > closed_lid_front[0][2] + 0.08
        and open_lid_front[1][0] < closed_lid_front[1][0] - 0.03
    )
    ctx.check(
        "lid rotates upward around the rear hinge line",
        lid_front_lifts,
        details=f"closed={closed_lid_front}, open={open_lid_front}",
    )

    latch_swings = (
        closed_latch_plate is not None
        and open_latch_plate is not None
        and open_latch_plate[0][2] > closed_latch_plate[0][2] + 0.01
        and open_latch_plate[1][0] > closed_latch_plate[1][0] + 0.006
    )
    ctx.check(
        "front latch rotates on its local pivot",
        latch_swings,
        details=f"closed={closed_latch_plate}, open={open_latch_plate}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
