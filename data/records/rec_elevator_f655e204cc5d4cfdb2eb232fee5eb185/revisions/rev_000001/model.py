from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_stretcher_elevator")

    frame_paint = model.material("frame_paint", rgba=(0.84, 0.86, 0.88, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_rail = model.material("dark_rail", rgba=(0.24, 0.26, 0.29, 1.0))
    floor_finish = model.material("floor_finish", rgba=(0.32, 0.35, 0.38, 1.0))
    trim = model.material("trim", rgba=(0.60, 0.62, 0.64, 1.0))

    shaft_frame = model.part("shaft_frame")
    shaft_frame.visual(
        Box((3.00, 3.40, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=frame_paint,
        name="base_slab",
    )
    shaft_frame.visual(
        Box((0.12, 0.12, 4.10)),
        origin=Origin(xyz=(-1.34, 1.18, 2.13)),
        material=dark_rail,
        name="left_guide_column",
    )
    shaft_frame.visual(
        Box((0.12, 0.12, 4.10)),
        origin=Origin(xyz=(1.34, 1.18, 2.13)),
        material=dark_rail,
        name="right_guide_column",
    )
    shaft_frame.visual(
        Box((2.80, 0.18, 0.18)),
        origin=Origin(xyz=(0.0, 1.18, 4.09)),
        material=frame_paint,
        name="rear_top_tie",
    )
    shaft_frame.visual(
        Box((2.80, 0.18, 0.08)),
        origin=Origin(xyz=(0.0, 1.18, 0.04)),
        material=frame_paint,
        name="rear_base_tie",
    )
    shaft_frame.visual(
        Box((0.16, 0.24, 2.90)),
        origin=Origin(xyz=(-1.26, -1.58, 1.53)),
        material=frame_paint,
        name="left_portal_jamb",
    )
    shaft_frame.visual(
        Box((0.16, 0.24, 2.90)),
        origin=Origin(xyz=(1.26, -1.58, 1.53)),
        material=frame_paint,
        name="right_portal_jamb",
    )
    shaft_frame.visual(
        Box((2.68, 0.24, 0.22)),
        origin=Origin(xyz=(0.0, -1.58, 2.95)),
        material=frame_paint,
        name="portal_header",
    )
    shaft_frame.visual(
        Box((2.40, 0.26, 0.04)),
        origin=Origin(xyz=(0.0, -1.58, 0.10)),
        material=trim,
        name="landing_threshold",
    )
    shaft_frame.inertial = Inertial.from_geometry(
        Box((3.00, 3.40, 4.18)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, 2.09)),
    )

    car = model.part("car")
    car.visual(
        Box((2.50, 2.68, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=floor_finish,
        name="car_floor",
    )
    car.visual(
        Box((2.50, 2.68, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 2.305)),
        material=stainless,
        name="car_roof",
    )
    car.visual(
        Box((2.38, 0.05, 2.24)),
        origin=Origin(xyz=(0.0, 1.315, 1.16)),
        material=stainless,
        name="rear_wall",
    )
    car.visual(
        Box((0.06, 2.58, 2.24)),
        origin=Origin(xyz=(-1.19, 0.0, 1.16)),
        material=stainless,
        name="left_wall",
    )
    car.visual(
        Box((0.06, 2.58, 2.24)),
        origin=Origin(xyz=(1.19, 0.0, 1.16)),
        material=stainless,
        name="right_wall",
    )
    car.visual(
        Box((2.50, 0.14, 0.18)),
        origin=Origin(xyz=(0.0, -1.27, 2.18)),
        material=stainless,
        name="door_header",
    )
    car.visual(
        Box((1.42, 0.14, 0.06)),
        origin=Origin(xyz=(0.0, -1.27, 0.03)),
        material=trim,
        name="door_sill",
    )
    car.visual(
        Box((0.10, 0.05, 2.14)),
        origin=Origin(xyz=(-0.66, -1.315, 1.10)),
        material=stainless,
        name="left_jamb",
    )
    car.visual(
        Box((0.10, 0.05, 2.14)),
        origin=Origin(xyz=(0.66, -1.315, 1.10)),
        material=stainless,
        name="right_jamb",
    )
    car.visual(
        Box((0.45, 0.05, 2.05)),
        origin=Origin(xyz=(-0.935, -1.315, 1.065)),
        material=stainless,
        name="left_pocket_wall",
    )
    car.visual(
        Box((0.45, 0.05, 2.05)),
        origin=Origin(xyz=(0.935, -1.315, 1.065)),
        material=stainless,
        name="right_pocket_wall",
    )
    car.visual(
        Box((1.10, 0.05, 0.04)),
        origin=Origin(xyz=(-0.55, -1.25, 2.09)),
        material=dark_rail,
        name="left_header_track",
    )
    car.visual(
        Box((1.10, 0.05, 0.04)),
        origin=Origin(xyz=(0.55, -1.25, 2.09)),
        material=dark_rail,
        name="right_header_track",
    )
    car.visual(
        Box((0.06, 1.90, 0.08)),
        origin=Origin(xyz=(-1.145, 0.08, 0.95)),
        material=trim,
        name="left_handrail",
    )
    car.visual(
        Box((0.06, 1.90, 0.08)),
        origin=Origin(xyz=(1.145, 0.08, 0.95)),
        material=trim,
        name="right_handrail",
    )
    car.visual(
        Box((1.80, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 1.26, 0.95)),
        material=trim,
        name="rear_handrail",
    )
    car.visual(
        Box((0.18, 0.03, 0.32)),
        origin=Origin(xyz=(1.145, -0.90, 1.25)),
        material=dark_rail,
        name="control_panel",
    )
    car.visual(
        Box((0.08, 0.10, 2.10)),
        origin=Origin(xyz=(-0.88, 1.14, 1.13)),
        material=dark_rail,
        name="left_back_stile",
    )
    car.visual(
        Box((0.08, 0.10, 2.10)),
        origin=Origin(xyz=(0.88, 1.14, 1.13)),
        material=dark_rail,
        name="right_back_stile",
    )
    car.visual(
        Box((1.84, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 1.14, 2.23)),
        material=dark_rail,
        name="back_crosshead",
    )
    car.visual(
        Box((1.84, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 1.14, 0.18)),
        material=dark_rail,
        name="back_lower_tie",
    )
    car.visual(
        Box((0.32, 0.10, 0.08)),
        origin=Origin(xyz=(-1.04, 1.18, 1.82)),
        material=dark_rail,
        name="left_upper_bracket",
    )
    car.visual(
        Box((0.32, 0.10, 0.08)),
        origin=Origin(xyz=(1.04, 1.18, 1.82)),
        material=dark_rail,
        name="right_upper_bracket",
    )
    car.visual(
        Box((0.32, 0.10, 0.08)),
        origin=Origin(xyz=(-1.04, 1.18, 0.56)),
        material=dark_rail,
        name="left_lower_bracket",
    )
    car.visual(
        Box((0.32, 0.10, 0.08)),
        origin=Origin(xyz=(1.04, 1.18, 0.56)),
        material=dark_rail,
        name="right_lower_bracket",
    )
    car.visual(
        Box((0.06, 0.12, 0.18)),
        origin=Origin(xyz=(-1.25, 1.18, 1.82)),
        material=trim,
        name="left_upper_shoe",
    )
    car.visual(
        Box((0.06, 0.12, 0.18)),
        origin=Origin(xyz=(1.25, 1.18, 1.82)),
        material=trim,
        name="right_upper_shoe",
    )
    car.visual(
        Box((0.06, 0.12, 0.18)),
        origin=Origin(xyz=(-1.25, 1.18, 0.56)),
        material=trim,
        name="left_lower_shoe",
    )
    car.visual(
        Box((0.06, 0.12, 0.18)),
        origin=Origin(xyz=(1.25, 1.18, 0.56)),
        material=trim,
        name="right_lower_shoe",
    )
    car.inertial = Inertial.from_geometry(
        Box((2.50, 2.68, 2.38)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 1.19)),
    )

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft_frame,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2000.0,
            velocity=0.8,
            lower=0.0,
            upper=1.45,
        ),
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((0.60, 0.04, 1.96)),
        origin=Origin(xyz=(-0.30, 0.0, 1.03)),
        material=stainless,
        name="left_panel",
    )
    left_door.visual(
        Box((0.60, 0.045, 0.05)),
        origin=Origin(xyz=(-0.30, 0.0, 0.075)),
        material=trim,
        name="left_kick_plate",
    )
    left_door.visual(
        Box((0.09, 0.05, 0.04)),
        origin=Origin(xyz=(-0.10, 0.0, 2.01)),
        material=dark_rail,
        name="left_inner_hanger",
    )
    left_door.visual(
        Box((0.09, 0.05, 0.04)),
        origin=Origin(xyz=(-0.50, 0.0, 2.01)),
        material=dark_rail,
        name="left_outer_hanger",
    )
    left_door.visual(
        Box((0.03, 0.015, 0.72)),
        origin=Origin(xyz=(-0.05, 0.0275, 1.08)),
        material=trim,
        name="left_pull_stile",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((0.62, 0.06, 2.07)),
        mass=1.6,
        origin=Origin(xyz=(-0.31, 0.0, 1.035)),
    )

    model.articulation(
        "car_to_left_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=left_door,
        origin=Origin(xyz=(-0.01, -1.25, 0.04)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.5,
            lower=0.0,
            upper=0.49,
        ),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((0.60, 0.04, 1.96)),
        origin=Origin(xyz=(0.30, 0.0, 1.03)),
        material=stainless,
        name="right_panel",
    )
    right_door.visual(
        Box((0.60, 0.045, 0.05)),
        origin=Origin(xyz=(0.30, 0.0, 0.075)),
        material=trim,
        name="right_kick_plate",
    )
    right_door.visual(
        Box((0.09, 0.05, 0.04)),
        origin=Origin(xyz=(0.10, 0.0, 2.01)),
        material=dark_rail,
        name="right_inner_hanger",
    )
    right_door.visual(
        Box((0.09, 0.05, 0.04)),
        origin=Origin(xyz=(0.50, 0.0, 2.01)),
        material=dark_rail,
        name="right_outer_hanger",
    )
    right_door.visual(
        Box((0.03, 0.015, 0.72)),
        origin=Origin(xyz=(0.05, 0.0275, 1.08)),
        material=trim,
        name="right_pull_stile",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((0.62, 0.06, 2.07)),
        mass=1.6,
        origin=Origin(xyz=(0.31, 0.0, 1.035)),
    )

    model.articulation(
        "car_to_right_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=right_door,
        origin=Origin(xyz=(0.01, -1.25, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.5,
            lower=0.0,
            upper=0.49,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("shaft_frame")
    car = object_model.get_part("car")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")

    car_lift = object_model.get_articulation("shaft_to_car")
    left_slide = object_model.get_articulation("car_to_left_door")
    right_slide = object_model.get_articulation("car_to_right_door")

    lift_upper = car_lift.motion_limits.upper or 0.0
    door_upper = left_slide.motion_limits.upper or 0.0

    ctx.expect_contact(
        car,
        frame,
        elem_a="left_upper_shoe",
        elem_b="left_guide_column",
        name="left upper guide shoe bears on the left guide column",
    )
    ctx.expect_contact(
        car,
        frame,
        elem_a="right_upper_shoe",
        elem_b="right_guide_column",
        name="right upper guide shoe bears on the right guide column",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        positive_elem="right_panel",
        negative_elem="left_panel",
        min_gap=0.0,
        max_gap=0.03,
        name="closed door panels meet at the center seam without overlap",
    )

    rest_car = ctx.part_world_position(car)
    rest_left = ctx.part_world_position(left_door)
    rest_right = ctx.part_world_position(right_door)

    with ctx.pose({car_lift: lift_upper}):
        ctx.expect_contact(
            car,
            frame,
            elem_a="left_lower_shoe",
            elem_b="left_guide_column",
            name="left lower guide shoe stays engaged on the rail at the upper landing",
        )
        ctx.expect_contact(
            car,
            frame,
            elem_a="right_lower_shoe",
            elem_b="right_guide_column",
            name="right lower guide shoe stays engaged on the rail at the upper landing",
        )
        raised_car = ctx.part_world_position(car)

    with ctx.pose({left_slide: door_upper, right_slide: door_upper}):
        ctx.expect_overlap(
            left_door,
            car,
            axes="x",
            elem_a="left_panel",
            elem_b="left_pocket_wall",
            min_overlap=0.35,
            name="left sliding panel retreats into the left pocket",
        )
        ctx.expect_overlap(
            right_door,
            car,
            axes="x",
            elem_a="right_panel",
            elem_b="right_pocket_wall",
            min_overlap=0.35,
            name="right sliding panel retreats into the right pocket",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            positive_elem="right_panel",
            negative_elem="left_panel",
            min_gap=0.95,
            name="door opening widens when both panels slide apart",
        )
        open_left = ctx.part_world_position(left_door)
        open_right = ctx.part_world_position(right_door)

    ctx.check(
        "car moves upward on its vertical guide rails",
        rest_car is not None and raised_car is not None and raised_car[2] > rest_car[2] + 1.0,
        details=f"rest={rest_car}, raised={raised_car}",
    )
    ctx.check(
        "left door slides leftward when opened",
        rest_left is not None and open_left is not None and open_left[0] < rest_left[0] - 0.35,
        details=f"rest={rest_left}, open={open_left}",
    )
    ctx.check(
        "right door slides rightward when opened",
        rest_right is not None and open_right is not None and open_right[0] > rest_right[0] + 0.35,
        details=f"rest={rest_right}, open={open_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
