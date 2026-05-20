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
    model = ArticulatedObject(name="cable_drum_residential_elevator")

    shaft_paint = model.material("shaft_paint", color=(0.82, 0.84, 0.87))
    steel = model.material("steel", color=(0.28, 0.30, 0.33))
    car_finish = model.material("car_finish", color=(0.92, 0.91, 0.87))
    door_finish = model.material("door_finish", color=(0.95, 0.93, 0.89))
    counterweight_finish = model.material("counterweight_finish", color=(0.42, 0.44, 0.47))
    floor_finish = model.material("floor_finish", color=(0.42, 0.30, 0.20))

    shaft = model.part("shaft")
    shaft.visual(
        Box((1.48, 1.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=shaft_paint,
        name="base",
    )
    shaft.visual(
        Box((0.04, 1.08, 3.15)),
        origin=Origin(xyz=(-0.72, 0.0, 1.625)),
        material=shaft_paint,
        name="left_wall",
    )
    shaft.visual(
        Box((0.04, 1.08, 3.15)),
        origin=Origin(xyz=(0.72, 0.0, 1.625)),
        material=shaft_paint,
        name="right_wall",
    )
    shaft.visual(
        Box((1.40, 0.04, 3.15)),
        origin=Origin(xyz=(0.0, -0.52, 1.625)),
        material=shaft_paint,
        name="rear_wall",
    )
    shaft.visual(
        Box((1.48, 1.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 3.225)),
        material=shaft_paint,
        name="roof",
    )
    shaft.visual(
        Box((0.28, 0.06, 3.15)),
        origin=Origin(xyz=(-0.58, 0.51, 1.625)),
        material=shaft_paint,
        name="left_front_return",
    )
    shaft.visual(
        Box((0.28, 0.06, 3.15)),
        origin=Origin(xyz=(0.58, 0.51, 1.625)),
        material=shaft_paint,
        name="right_front_return",
    )
    shaft.visual(
        Box((0.88, 0.06, 1.00)),
        origin=Origin(xyz=(0.0, 0.51, 2.70)),
        material=shaft_paint,
        name="door_header",
    )
    shaft.visual(
        Box((0.88, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, 0.51, 0.065)),
        material=steel,
        name="door_sill",
    )

    shaft.visual(
        Box((0.07, 0.06, 3.00)),
        origin=Origin(xyz=(-0.54, -0.41, 1.55)),
        material=steel,
        name="left_car_rail",
    )
    shaft.visual(
        Box((0.07, 0.12, 3.00)),
        origin=Origin(xyz=(-0.54, -0.47, 1.55)),
        material=steel,
        name="left_car_rail_bracket",
    )
    shaft.visual(
        Box((0.07, 0.06, 3.00)),
        origin=Origin(xyz=(0.34, -0.41, 1.55)),
        material=steel,
        name="right_car_rail",
    )
    shaft.visual(
        Box((0.07, 0.12, 3.00)),
        origin=Origin(xyz=(0.34, -0.47, 1.55)),
        material=steel,
        name="right_car_rail_bracket",
    )

    shaft.visual(
        Box((0.04, 0.06, 3.00)),
        origin=Origin(xyz=(0.44, -0.41, 1.55)),
        material=steel,
        name="left_counterweight_rail",
    )
    shaft.visual(
        Box((0.04, 0.12, 3.00)),
        origin=Origin(xyz=(0.44, -0.47, 1.55)),
        material=steel,
        name="left_counterweight_rail_bracket",
    )
    shaft.visual(
        Box((0.04, 0.06, 3.00)),
        origin=Origin(xyz=(0.66, -0.41, 1.55)),
        material=steel,
        name="right_counterweight_rail",
    )
    shaft.visual(
        Box((0.04, 0.12, 3.00)),
        origin=Origin(xyz=(0.66, -0.47, 1.55)),
        material=steel,
        name="right_counterweight_rail_bracket",
    )

    shaft.visual(
        Box((0.72, 0.44, 0.10)),
        origin=Origin(xyz=(0.12, -0.30, 3.30)),
        material=steel,
        name="machine_bed",
    )
    shaft.visual(
        Cylinder(radius=0.06, length=0.55),
        origin=Origin(xyz=(0.02, -0.30, 3.41), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="winding_drum",
    )
    shaft.visual(
        Box((0.20, 0.24, 0.18)),
        origin=Origin(xyz=(0.38, -0.30, 3.44)),
        material=steel,
        name="drive_motor",
    )
    shaft.visual(
        Cylinder(radius=0.007, length=0.16),
        origin=Origin(xyz=(-0.08, -0.24, 3.33)),
        material=steel,
        name="left_cable_stub",
    )
    shaft.visual(
        Cylinder(radius=0.007, length=0.16),
        origin=Origin(xyz=(0.14, -0.24, 3.33)),
        material=steel,
        name="right_cable_stub",
    )

    car = model.part("car")
    car.visual(
        Box((0.82, 0.76, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=steel,
        name="floor_pan",
    )
    car.visual(
        Box((0.74, 0.66, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=floor_finish,
        name="floor_finish",
    )
    car.visual(
        Box((0.03, 0.76, 1.98)),
        origin=Origin(xyz=(-0.395, 0.0, 1.03)),
        material=car_finish,
        name="left_wall",
    )
    car.visual(
        Box((0.03, 0.76, 1.98)),
        origin=Origin(xyz=(0.395, 0.0, 1.03)),
        material=car_finish,
        name="right_wall",
    )
    car.visual(
        Box((0.82, 0.03, 1.98)),
        origin=Origin(xyz=(0.0, -0.365, 1.03)),
        material=car_finish,
        name="rear_wall",
    )
    car.visual(
        Box((0.82, 0.76, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 2.035)),
        material=car_finish,
        name="roof_panel",
    )
    car.visual(
        Box((0.07, 0.07, 2.01)),
        origin=Origin(xyz=(-0.44, -0.365, 1.045)),
        material=steel,
        name="left_guide_channel",
    )
    car.visual(
        Box((0.07, 0.07, 2.01)),
        origin=Origin(xyz=(0.44, -0.365, 1.045)),
        material=steel,
        name="right_guide_channel",
    )
    car.visual(
        Box((0.16, 0.03, 0.24)),
        origin=Origin(xyz=(0.33, -0.12, 1.15)),
        material=steel,
        name="control_panel",
    )

    left_door = model.part("left_door")
    left_door.visual(
        Cylinder(radius=0.012, length=2.12),
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        material=steel,
        name="hinge_barrel",
    )
    left_door.visual(
        Box((0.43, 0.03, 2.12)),
        origin=Origin(xyz=(0.215, 0.0, 1.06)),
        material=door_finish,
        name="leaf_panel",
    )
    left_door.visual(
        Box((0.02, 0.012, 0.30)),
        origin=Origin(xyz=(0.35, 0.021, 1.14)),
        material=steel,
        name="pull_handle",
    )

    right_door = model.part("right_door")
    right_door.visual(
        Cylinder(radius=0.012, length=2.12),
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        material=steel,
        name="hinge_barrel",
    )
    right_door.visual(
        Box((0.43, 0.03, 2.12)),
        origin=Origin(xyz=(-0.215, 0.0, 1.06)),
        material=door_finish,
        name="leaf_panel",
    )
    right_door.visual(
        Box((0.02, 0.012, 0.30)),
        origin=Origin(xyz=(-0.35, 0.021, 1.14)),
        material=steel,
        name="pull_handle",
    )

    counterweight = model.part("counterweight")
    counterweight.visual(
        Box((0.18, 0.18, 1.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=counterweight_finish,
        name="weight_stack",
    )
    counterweight.visual(
        Box((0.24, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=steel,
        name="bottom_yoke",
    )
    counterweight.visual(
        Box((0.24, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=steel,
        name="top_yoke",
    )
    counterweight.visual(
        Box((0.04, 0.09, 1.25)),
        origin=Origin(xyz=(-0.11, -0.095, 0.625)),
        material=steel,
        name="left_guide_block",
    )
    counterweight.visual(
        Box((0.04, 0.09, 1.25)),
        origin=Origin(xyz=(0.11, -0.095, 0.625)),
        material=steel,
        name="right_guide_block",
    )

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(-0.10, 0.02, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.5, lower=0.0, upper=1.0),
    )
    model.articulation(
        "shaft_to_left_door",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=left_door,
        origin=Origin(xyz=(-0.44, 0.465, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "shaft_to_right_door",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=right_door,
        origin=Origin(xyz=(0.44, 0.465, 0.08)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "shaft_to_counterweight",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=counterweight,
        origin=Origin(xyz=(0.55, -0.24, 1.80)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.5, lower=0.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    counterweight = object_model.get_part("counterweight")
    car_lift = object_model.get_articulation("shaft_to_car")
    left_hinge = object_model.get_articulation("shaft_to_left_door")
    right_hinge = object_model.get_articulation("shaft_to_right_door")
    counterweight_slide = object_model.get_articulation("shaft_to_counterweight")

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

    with ctx.pose(
        {
            car_lift: 0.0,
            left_hinge: 0.0,
            right_hinge: 0.0,
            counterweight_slide: 0.0,
        }
    ):
        ctx.expect_contact(
            car,
            shaft,
            elem_a="left_guide_channel",
            elem_b="left_car_rail",
            name="car left guide channel rides the left guide rail",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a="right_guide_channel",
            elem_b="right_car_rail",
            name="car right guide channel rides the right guide rail",
        )
        ctx.expect_contact(
            counterweight,
            shaft,
            elem_a="left_guide_block",
            elem_b="left_counterweight_rail",
            name="counterweight left guide stays on its rail",
        )
        ctx.expect_contact(
            counterweight,
            shaft,
            elem_a="right_guide_block",
            elem_b="right_counterweight_rail",
            name="counterweight right guide stays on its rail",
        )
        ctx.expect_contact(
            left_door,
            shaft,
            elem_a="leaf_panel",
            elem_b="left_front_return",
            name="left door leaf mounts against the left frame side",
        )
        ctx.expect_contact(
            right_door,
            shaft,
            elem_a="leaf_panel",
            elem_b="right_front_return",
            name="right door leaf mounts against the right frame side",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.0,
            max_gap=0.03,
            max_penetration=0.0,
            name="closed bi-parting door leaves meet at the center seam",
        )

    car_rest = ctx.part_world_position(car)
    counterweight_rest = ctx.part_world_position(counterweight)
    closed_left_aabb = ctx.part_element_world_aabb(left_door, elem="leaf_panel")
    closed_right_aabb = ctx.part_element_world_aabb(right_door, elem="leaf_panel")

    with ctx.pose({car_lift: 1.0, counterweight_slide: 1.0}):
        car_high = ctx.part_world_position(car)
        counterweight_low = ctx.part_world_position(counterweight)
        ctx.expect_contact(
            car,
            shaft,
            elem_a="left_guide_channel",
            elem_b="left_car_rail",
            name="car left guide remains engaged at the upper landing",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a="right_guide_channel",
            elem_b="right_car_rail",
            name="car right guide remains engaged at the upper landing",
        )
        ctx.expect_contact(
            counterweight,
            shaft,
            elem_a="left_guide_block",
            elem_b="left_counterweight_rail",
            name="counterweight left guide remains engaged through travel",
        )
        ctx.expect_contact(
            counterweight,
            shaft,
            elem_a="right_guide_block",
            elem_b="right_counterweight_rail",
            name="counterweight right guide remains engaged through travel",
        )

    ctx.check(
        "car travels upward on the main hoist joint",
        car_rest is not None and car_high is not None and car_high[2] > car_rest[2] + 0.9,
        details=f"rest={car_rest}, high={car_high}",
    )
    ctx.check(
        "counterweight moves downward opposite the car",
        counterweight_rest is not None
        and counterweight_low is not None
        and counterweight_low[2] < counterweight_rest[2] - 0.9,
        details=f"rest={counterweight_rest}, low={counterweight_low}",
    )

    with ctx.pose({left_hinge: 1.2, right_hinge: 1.2}):
        opened_left_aabb = ctx.part_element_world_aabb(left_door, elem="leaf_panel")
        opened_right_aabb = ctx.part_element_world_aabb(right_door, elem="leaf_panel")
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.45,
            name="opened door leaves clear the central opening",
        )

    ctx.check(
        "left door swings outward from the frame",
        closed_left_aabb is not None
        and opened_left_aabb is not None
        and opened_left_aabb[1][1] > closed_left_aabb[1][1] + 0.20,
        details=f"closed={closed_left_aabb}, open={opened_left_aabb}",
    )
    ctx.check(
        "right door swings outward from the frame",
        closed_right_aabb is not None
        and opened_right_aabb is not None
        and opened_right_aabb[1][1] > closed_right_aabb[1][1] + 0.20,
        details=f"closed={closed_right_aabb}, open={opened_right_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
