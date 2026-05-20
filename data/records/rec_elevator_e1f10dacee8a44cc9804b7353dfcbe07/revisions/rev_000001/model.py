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
    model = ArticulatedObject(name="panoramic_glass_elevator")

    structural_steel = model.material("structural_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.73, 0.77, 1.0))
    cabin_floor = model.material("cabin_floor", rgba=(0.28, 0.26, 0.24, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.72, 0.87, 0.96, 0.36))
    dark_rail = model.material("dark_rail", rgba=(0.16, 0.17, 0.19, 1.0))

    shaft_frame = model.part("shaft_frame")
    shaft_frame.visual(
        Box((1.38, 1.58, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=structural_steel,
        name="shaft_base",
    )
    shaft_frame.visual(
        Box((0.10, 0.10, 3.38)),
        origin=Origin(xyz=(-0.66, -0.62, 1.69)),
        material=structural_steel,
        name="left_support_column",
    )
    shaft_frame.visual(
        Box((0.10, 0.10, 3.38)),
        origin=Origin(xyz=(-0.66, 0.62, 1.69)),
        material=structural_steel,
        name="right_support_column",
    )
    shaft_frame.visual(
        Box((0.18, 1.46, 0.12)),
        origin=Origin(xyz=(-0.66, 0.0, 3.41)),
        material=structural_steel,
        name="top_crossbeam",
    )
    shaft_frame.visual(
        Box((0.12, 0.08, 3.05)),
        origin=Origin(xyz=(-0.61, -0.62, 1.615)),
        material=dark_rail,
        name="left_guide_rail",
    )
    shaft_frame.visual(
        Box((0.12, 0.08, 3.05)),
        origin=Origin(xyz=(-0.61, 0.62, 1.615)),
        material=dark_rail,
        name="right_guide_rail",
    )
    shaft_frame.inertial = Inertial.from_geometry(
        Box((1.38, 1.58, 3.47)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.735)),
    )

    car = model.part("car")
    car.visual(
        Box((1.00, 1.34, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=cabin_floor,
        name="car_floor",
    )
    car.visual(
        Box((1.02, 1.36, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.16)),
        material=brushed_aluminum,
        name="car_roof",
    )
    car.visual(
        Box((0.06, 0.06, 2.10)),
        origin=Origin(xyz=(-0.49, -0.64, 1.09)),
        material=brushed_aluminum,
        name="rear_left_post",
    )
    car.visual(
        Box((0.06, 0.06, 2.10)),
        origin=Origin(xyz=(-0.49, 0.64, 1.09)),
        material=brushed_aluminum,
        name="rear_right_post",
    )
    car.visual(
        Box((0.06, 0.06, 2.10)),
        origin=Origin(xyz=(0.49, -0.64, 1.09)),
        material=brushed_aluminum,
        name="front_left_post",
    )
    car.visual(
        Box((0.06, 0.06, 2.10)),
        origin=Origin(xyz=(0.49, 0.64, 1.09)),
        material=brushed_aluminum,
        name="front_right_post",
    )
    car.visual(
        Box((0.08, 1.34, 0.10)),
        origin=Origin(xyz=(0.49, 0.0, 2.02)),
        material=brushed_aluminum,
        name="front_header",
    )
    car.visual(
        Box((0.08, 1.34, 0.05)),
        origin=Origin(xyz=(0.49, 0.0, 0.025)),
        material=brushed_aluminum,
        name="front_sill",
    )
    car.visual(
        Box((0.96, 0.012, 1.88)),
        origin=Origin(xyz=(0.0, -0.625, 1.10)),
        material=tinted_glass,
        name="left_side_glass",
    )
    car.visual(
        Box((0.96, 0.012, 1.88)),
        origin=Origin(xyz=(0.0, 0.625, 1.10)),
        material=tinted_glass,
        name="right_side_glass",
    )
    car.visual(
        Box((0.012, 1.22, 1.88)),
        origin=Origin(xyz=(-0.475, 0.0, 1.10)),
        material=tinted_glass,
        name="rear_glass",
    )
    car.visual(
        Box((0.012, 0.64, 1.88)),
        origin=Origin(xyz=(0.495, 0.32, 1.10)),
        material=tinted_glass,
        name="front_pocket_glass",
    )
    car.visual(
        Box((0.06, 1.24, 0.035)),
        origin=Origin(xyz=(0.42, 0.0, 1.935)),
        material=dark_rail,
        name="door_top_track",
    )
    car.visual(
        Box((0.04, 1.22, 0.02)),
        origin=Origin(xyz=(0.42, 0.0, 0.055)),
        material=dark_rail,
        name="door_bottom_guide",
    )
    car.visual(
        Box((0.09, 0.05, 0.20)),
        origin=Origin(xyz=(-0.505, -0.62, 0.55)),
        material=brushed_aluminum,
        name="left_guide_shoe_lower",
    )
    car.visual(
        Box((0.09, 0.05, 0.20)),
        origin=Origin(xyz=(-0.505, -0.62, 1.65)),
        material=brushed_aluminum,
        name="left_guide_shoe_upper",
    )
    car.visual(
        Box((0.09, 0.05, 0.20)),
        origin=Origin(xyz=(-0.505, 0.62, 0.55)),
        material=brushed_aluminum,
        name="right_guide_shoe_lower",
    )
    car.visual(
        Box((0.09, 0.05, 0.20)),
        origin=Origin(xyz=(-0.505, 0.62, 1.65)),
        material=brushed_aluminum,
        name="right_guide_shoe_upper",
    )
    car.inertial = Inertial.from_geometry(
        Box((1.02, 1.36, 2.20)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )

    door_panel = model.part("door_panel")
    door_panel.visual(
        Box((0.028, 0.60, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=brushed_aluminum,
        name="door_bottom_rail",
    )
    door_panel.visual(
        Box((0.028, 0.60, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 1.825)),
        material=brushed_aluminum,
        name="door_top_rail",
    )
    door_panel.visual(
        Box((0.028, 0.04, 1.86)),
        origin=Origin(xyz=(0.0, -0.28, 0.93)),
        material=brushed_aluminum,
        name="door_leading_stile",
    )
    door_panel.visual(
        Box((0.028, 0.04, 1.86)),
        origin=Origin(xyz=(0.0, 0.28, 0.93)),
        material=brushed_aluminum,
        name="door_trailing_stile",
    )
    door_panel.visual(
        Box((0.012, 0.54, 1.72)),
        origin=Origin(xyz=(0.0, 0.0, 0.93)),
        material=tinted_glass,
        name="door_glass",
    )
    door_panel.visual(
        Box((0.03, 0.07, 0.055)),
        origin=Origin(xyz=(0.0, -0.18, 1.8275)),
        material=dark_rail,
        name="left_hanger",
    )
    door_panel.visual(
        Box((0.03, 0.07, 0.055)),
        origin=Origin(xyz=(0.0, 0.18, 1.8275)),
        material=dark_rail,
        name="right_hanger",
    )
    door_panel.inertial = Inertial.from_geometry(
        Box((0.03, 0.60, 1.93)),
        mass=45.0,
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
    )

    lift_joint = model.articulation(
        "shaft_to_car_lift",
        ArticulationType.PRISMATIC,
        parent=shaft_frame,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=1.0, lower=0.0, upper=0.95),
    )
    model.articulation(
        "car_to_door_panel",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door_panel,
        origin=Origin(xyz=(0.455, -0.31, 0.06)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.8, lower=0.0, upper=0.62),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft_frame = object_model.get_part("shaft_frame")
    car = object_model.get_part("car")
    door_panel = object_model.get_part("door_panel")
    lift_joint = object_model.get_articulation("shaft_to_car_lift")
    door_joint = object_model.get_articulation("car_to_door_panel")

    ctx.check("shaft frame exists", shaft_frame is not None)
    ctx.check("car exists", car is not None)
    ctx.check("door panel exists", door_panel is not None)

    with ctx.pose({lift_joint: 0.0, door_joint: 0.0}):
        ctx.expect_gap(
            car,
            shaft_frame,
            axis="x",
            positive_elem="left_guide_shoe_lower",
            negative_elem="left_guide_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="left lower guide shoe bears on left rail at rest",
        )
        ctx.expect_gap(
            car,
            shaft_frame,
            axis="x",
            positive_elem="right_guide_shoe_lower",
            negative_elem="right_guide_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="right lower guide shoe bears on right rail at rest",
        )
        ctx.expect_gap(
            car,
            door_panel,
            axis="z",
            positive_elem="door_top_track",
            negative_elem="left_hanger",
            max_gap=0.004,
            max_penetration=0.0,
            name="left hanger sits just beneath the top track at rest",
        )
        ctx.expect_gap(
            car,
            door_panel,
            axis="z",
            positive_elem="door_top_track",
            negative_elem="right_hanger",
            max_gap=0.004,
            max_penetration=0.0,
            name="right hanger sits just beneath the top track at rest",
        )
        ctx.expect_overlap(
            door_panel,
            car,
            axes="y",
            elem_a="door_glass",
            elem_b="front_sill",
            min_overlap=0.52,
            name="closed door spans the entrance opening width",
        )

    rest_car_pos = ctx.part_world_position(car)
    with ctx.pose({lift_joint: 0.85}):
        raised_car_pos = ctx.part_world_position(car)
        ctx.expect_gap(
            car,
            shaft_frame,
            axis="x",
            positive_elem="left_guide_shoe_upper",
            negative_elem="left_guide_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="left upper guide shoe remains on the rail when raised",
        )
        ctx.expect_gap(
            car,
            shaft_frame,
            axis="x",
            positive_elem="right_guide_shoe_upper",
            negative_elem="right_guide_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="right upper guide shoe remains on the rail when raised",
        )
    ctx.check(
        "car travels upward on the shaft rails",
        rest_car_pos is not None
        and raised_car_pos is not None
        and raised_car_pos[2] > rest_car_pos[2] + 0.80,
        details=f"rest={rest_car_pos}, raised={raised_car_pos}",
    )

    rest_door_pos = ctx.part_world_position(door_panel)
    with ctx.pose({door_joint: 0.62}):
        open_door_pos = ctx.part_world_position(door_panel)
        ctx.expect_within(
            door_panel,
            car,
            axes="y",
            inner_elem="left_hanger",
            outer_elem="door_top_track",
            margin=0.0,
            name="left hanger stays within the horizontal top track when open",
        )
        ctx.expect_within(
            door_panel,
            car,
            axes="y",
            inner_elem="right_hanger",
            outer_elem="door_top_track",
            margin=0.0,
            name="right hanger stays within the horizontal top track when open",
        )
        ctx.expect_overlap(
            door_panel,
            car,
            axes="y",
            elem_a="door_glass",
            elem_b="front_pocket_glass",
            min_overlap=0.50,
            name="opened door parks over the side pocket glazing zone",
        )
    ctx.check(
        "door panel slides sideways to open",
        rest_door_pos is not None
        and open_door_pos is not None
        and open_door_pos[1] > rest_door_pos[1] + 0.55,
        details=f"rest={rest_door_pos}, open={open_door_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
