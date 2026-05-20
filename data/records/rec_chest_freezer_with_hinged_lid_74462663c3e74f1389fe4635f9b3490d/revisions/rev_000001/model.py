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


OUTER_X = 2.30
OUTER_Y = 0.92
RAIL_TOP_Z = 0.805
LID_X = 1.04
LID_Y = 0.76
LID_TRAVEL = 0.34


def _add_lid_panel(
    lid,
    *,
    material_frame,
    material_glass,
    material_rubber,
    handle_at: str,
    front_runner_y: float,
    rear_runner_y: float,
) -> None:
    """Author one framed glass sliding lid panel in its own joint frame."""

    frame_t = 0.050
    frame_z = 0.036
    runner_z = 0.018
    frame_center_z = runner_z + frame_z / 2.0

    # Low polymer runners are the physical interface to the guide rails.
    lid.visual(
        Box((0.82, 0.028, runner_z)),
        origin=Origin(xyz=(0.0, front_runner_y, runner_z / 2.0)),
        material=material_rubber,
        name="front_skid",
    )
    lid.visual(
        Box((0.82, 0.028, runner_z)),
        origin=Origin(xyz=(0.0, rear_runner_y, runner_z / 2.0)),
        material=material_rubber,
        name="rear_skid",
    )

    # Aluminum perimeter frame.  The members touch at the corners and rest on
    # the runners so the moving panel reads as a single manufactured assembly.
    lid.visual(
        Box((LID_X, frame_t, frame_z)),
        origin=Origin(xyz=(0.0, -LID_Y / 2.0 + frame_t / 2.0, frame_center_z)),
        material=material_frame,
        name="front_frame",
    )
    lid.visual(
        Box((LID_X, frame_t, frame_z)),
        origin=Origin(xyz=(0.0, LID_Y / 2.0 - frame_t / 2.0, frame_center_z)),
        material=material_frame,
        name="rear_frame",
    )
    lid.visual(
        Box((frame_t, LID_Y, frame_z)),
        origin=Origin(xyz=(-LID_X / 2.0 + frame_t / 2.0, 0.0, frame_center_z)),
        material=material_frame,
        name="outer_frame",
    )
    lid.visual(
        Box((frame_t, LID_Y, frame_z)),
        origin=Origin(xyz=(LID_X / 2.0 - frame_t / 2.0, 0.0, frame_center_z)),
        material=material_frame,
        name="inner_frame",
    )

    # Slightly oversize glass tucks underneath the frame on all four sides.
    lid.visual(
        Box((LID_X - 0.080, LID_Y - 0.080, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, frame_center_z + 0.002)),
        material=material_glass,
        name="glass_pane",
    )

    inner_sign = 1.0 if handle_at == "right" else -1.0
    front_frame_y = -LID_Y / 2.0 + frame_t / 2.0
    rear_frame_y = LID_Y / 2.0 - frame_t / 2.0
    lid.visual(
        Box((0.78, abs(front_runner_y - front_frame_y) + 0.050, 0.012)),
        origin=Origin(xyz=(0.0, (front_runner_y + front_frame_y) / 2.0, runner_z)),
        material=material_rubber,
        name="front_skid_web",
    )
    lid.visual(
        Box((0.78, abs(rear_runner_y - rear_frame_y) + 0.050, 0.012)),
        origin=Origin(xyz=(0.0, (rear_runner_y + rear_frame_y) / 2.0, runner_z)),
        material=material_rubber,
        name="rear_skid_web",
    )

    handle_x = inner_sign * (LID_X / 2.0 - 0.095)
    lid.visual(
        Box((0.090, 0.52, 0.030)),
        origin=Origin(xyz=(handle_x, 0.0, frame_center_z + frame_z / 2.0 + 0.009)),
        material=material_rubber,
        name="grip",
    )
    lid.visual(
        Box((0.024, 0.68, 0.044)),
        origin=Origin(
            xyz=(
                inner_sign * (LID_X / 2.0 - 0.012),
                0.0,
                frame_center_z + 0.003,
            )
        ),
        material=material_rubber,
        name="meeting_gasket",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_ice_cream_chest_freezer")

    white = model.material("white_powder_coated_steel", rgba=(0.92, 0.96, 0.97, 1.0))
    liner = model.material("cool_white_liner", rgba=(0.78, 0.90, 0.95, 1.0))
    plinth = model.material("charcoal_plinth", rgba=(0.05, 0.055, 0.060, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.007, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.62, 0.83, 0.96, 0.36))
    dark = model.material("shadowed_freezer_well", rgba=(0.015, 0.025, 0.035, 1.0))
    blue = model.material("commercial_blue_stripe", rgba=(0.02, 0.18, 0.68, 1.0))

    body = model.part("body")

    # Insulated rectangular freezer cabinet with an open top well.
    body.visual(
        Box((OUTER_X + 0.02, OUTER_Y + 0.02, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=plinth,
        name="lower_plinth",
    )
    body.visual(
        Box((OUTER_X, 0.060, 0.660)),
        origin=Origin(xyz=(0.0, -OUTER_Y / 2.0 + 0.030, 0.420)),
        material=white,
        name="front_wall",
    )
    body.visual(
        Box((OUTER_X, 0.060, 0.660)),
        origin=Origin(xyz=(0.0, OUTER_Y / 2.0 - 0.030, 0.420)),
        material=white,
        name="rear_wall",
    )
    body.visual(
        Box((0.080, OUTER_Y, 0.660)),
        origin=Origin(xyz=(-OUTER_X / 2.0 + 0.040, 0.0, 0.420)),
        material=white,
        name="end_wall_0",
    )
    body.visual(
        Box((0.080, OUTER_Y, 0.660)),
        origin=Origin(xyz=(OUTER_X / 2.0 - 0.040, 0.0, 0.420)),
        material=white,
        name="end_wall_1",
    )

    # Visible liner and dark cold well inside the hollow chest.
    body.visual(
        Box((2.08, 0.76, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=dark,
        name="well_floor",
    )
    body.visual(
        Box((2.08, 0.016, 0.460)),
        origin=Origin(xyz=(0.0, -0.392, 0.360)),
        material=liner,
        name="front_liner",
    )
    body.visual(
        Box((2.08, 0.016, 0.460)),
        origin=Origin(xyz=(0.0, 0.392, 0.360)),
        material=liner,
        name="rear_liner",
    )
    body.visual(
        Box((0.016, 0.76, 0.460)),
        origin=Origin(xyz=(-1.044, 0.0, 0.360)),
        material=liner,
        name="end_liner_0",
    )
    body.visual(
        Box((0.016, 0.76, 0.460)),
        origin=Origin(xyz=(1.044, 0.0, 0.360)),
        material=liner,
        name="end_liner_1",
    )

    # Thick molded top rim around the opening.
    body.visual(
        Box((OUTER_X, 0.100, 0.050)),
        origin=Origin(xyz=(0.0, -0.430, 0.765)),
        material=white,
        name="front_top_rim",
    )
    body.visual(
        Box((OUTER_X, 0.100, 0.050)),
        origin=Origin(xyz=(0.0, 0.430, 0.765)),
        material=white,
        name="rear_top_rim",
    )
    body.visual(
        Box((0.100, OUTER_Y, 0.050)),
        origin=Origin(xyz=(-1.100, 0.0, 0.765)),
        material=white,
        name="end_top_rim_0",
    )
    body.visual(
        Box((0.100, OUTER_Y, 0.050)),
        origin=Origin(xyz=(1.100, 0.0, 0.765)),
        material=white,
        name="end_top_rim_1",
    )

    # Two separate pairs of guide rails: outer pair for the left sliding lid,
    # inner pair for the right sliding lid.
    body.visual(
        Box((2.18, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.340, RAIL_TOP_Z - 0.010)),
        material=aluminum,
        name="left_front_rail",
    )
    body.visual(
        Box((2.18, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, 0.340, RAIL_TOP_Z - 0.010)),
        material=aluminum,
        name="left_rear_rail",
    )
    body.visual(
        Box((2.18, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.285, RAIL_TOP_Z - 0.010)),
        material=aluminum,
        name="right_front_rail",
    )
    body.visual(
        Box((2.18, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, 0.285, RAIL_TOP_Z - 0.010)),
        material=aluminum,
        name="right_rear_rail",
    )

    # Commercial fascia, ventilation slots, and feet.
    body.visual(
        Box((1.86, 0.014, 0.080)),
        origin=Origin(xyz=(0.0, -0.467, 0.520)),
        material=blue,
        name="brand_stripe",
    )
    for i, x in enumerate((-0.38, -0.27, -0.16, -0.05, 0.06, 0.17, 0.28, 0.39)):
        body.visual(
            Box((0.065, 0.016, 0.018)),
            origin=Origin(xyz=(x, -0.468, 0.220)),
            material=plinth,
            name=f"vent_slot_{i}",
        )
    for i, (x, y) in enumerate(
        (
            (-0.96, -0.34),
            (-0.96, 0.34),
            (0.96, -0.34),
            (0.96, 0.34),
        )
    ):
        body.visual(
            Cylinder(radius=0.045, length=0.060),
            origin=Origin(xyz=(x, y, -0.028)),
            material=rubber,
            name=f"foot_{i}",
        )

    left_lid = model.part("left_lid")
    _add_lid_panel(
        left_lid,
        material_frame=aluminum,
        material_glass=glass,
        material_rubber=rubber,
        handle_at="right",
        front_runner_y=-0.340,
        rear_runner_y=0.340,
    )

    right_lid = model.part("right_lid")
    _add_lid_panel(
        right_lid,
        material_frame=aluminum,
        material_glass=glass,
        material_rubber=rubber,
        handle_at="left",
        front_runner_y=-0.285,
        rear_runner_y=0.285,
    )

    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_lid,
        origin=Origin(xyz=(-0.535, 0.0, RAIL_TOP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=0.0, upper=LID_TRAVEL),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_lid,
        origin=Origin(xyz=(0.535, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=0.0, upper=LID_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")

    ctx.check(
        "left lid slides prismatically to the left",
        left_slide.articulation_type == ArticulationType.PRISMATIC
        and left_slide.axis == (-1.0, 0.0, 0.0)
        and left_slide.motion_limits is not None
        and left_slide.motion_limits.upper == LID_TRAVEL,
        details=f"type={left_slide.articulation_type}, axis={left_slide.axis}, limits={left_slide.motion_limits}",
    )
    ctx.check(
        "right lid slides prismatically to the right",
        right_slide.articulation_type == ArticulationType.PRISMATIC
        and right_slide.axis == (1.0, 0.0, 0.0)
        and right_slide.motion_limits is not None
        and right_slide.motion_limits.upper == LID_TRAVEL,
        details=f"type={right_slide.articulation_type}, axis={right_slide.axis}, limits={right_slide.motion_limits}",
    )

    ctx.expect_gap(
        right_lid,
        left_lid,
        axis="x",
        min_gap=0.020,
        max_gap=0.040,
        name="closed lids meet with a narrow central split",
    )
    ctx.expect_contact(
        left_lid,
        body,
        elem_a="front_skid",
        elem_b="left_front_rail",
        name="left lid rides on its front rail",
    )
    ctx.expect_contact(
        right_lid,
        body,
        elem_a="rear_skid",
        elem_b="right_rear_rail",
        name="right lid rides on its separate rear rail",
    )

    rest_left = ctx.part_world_position(left_lid)
    rest_right = ctx.part_world_position(right_lid)
    with ctx.pose({left_slide: LID_TRAVEL, right_slide: LID_TRAVEL}):
        ctx.expect_gap(
            right_lid,
            left_lid,
            axis="x",
            min_gap=0.68,
            name="opened lids reveal a wide central access opening",
        )
        ctx.expect_overlap(
            left_lid,
            body,
            axes="x",
            min_overlap=0.45,
            elem_a="front_skid",
            elem_b="left_front_rail",
            name="left runner remains captured on left rail when open",
        )
        ctx.expect_overlap(
            right_lid,
            body,
            axes="x",
            min_overlap=0.45,
            elem_a="rear_skid",
            elem_b="right_rear_rail",
            name="right runner remains captured on right rail when open",
        )
        open_left = ctx.part_world_position(left_lid)
        open_right = ctx.part_world_position(right_lid)

    ctx.check(
        "upper-limit pose moves lids apart",
        rest_left is not None
        and rest_right is not None
        and open_left is not None
        and open_right is not None
        and open_left[0] < rest_left[0] - 0.30
        and open_right[0] > rest_right[0] + 0.30,
        details=f"rest_left={rest_left}, open_left={open_left}, rest_right={rest_right}, open_right={open_right}",
    )

    return ctx.report()


object_model = build_object_model()
