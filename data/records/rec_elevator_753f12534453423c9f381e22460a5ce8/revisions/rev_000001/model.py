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
    model = ArticulatedObject(name="dumbwaiter")

    painted_steel = model.material("painted_steel", rgba=(0.46, 0.49, 0.52, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.30, 0.32, 0.34, 1.0))
    motor_grey = model.material("motor_grey", rgba=(0.22, 0.24, 0.26, 1.0))
    wood_panel = model.material("wood_panel", rgba=(0.63, 0.45, 0.27, 1.0))
    wood_trim = model.material("wood_trim", rgba=(0.47, 0.31, 0.18, 1.0))
    counterweight_iron = model.material("counterweight_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    hardware = model.material("hardware", rgba=(0.70, 0.69, 0.63, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.72, 0.40, 0.06)),
        origin=Origin(xyz=(0.0, 0.04, 0.03)),
        material=painted_steel,
        name="base_plinth",
    )
    frame.visual(
        Box((0.64, 0.036, 1.44)),
        origin=Origin(xyz=(0.0, 0.148, 0.78)),
        material=painted_steel,
        name="backplate",
    )
    frame.visual(
        Box((0.06, 0.06, 1.38)),
        origin=Origin(xyz=(-0.33, 0.13, 0.75)),
        material=painted_steel,
        name="left_upright",
    )
    frame.visual(
        Box((0.06, 0.06, 1.38)),
        origin=Origin(xyz=(0.33, 0.13, 0.75)),
        material=painted_steel,
        name="right_upright",
    )
    frame.visual(
        Box((0.68, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.105, 1.53)),
        material=painted_steel,
        name="head_beam",
    )
    frame.visual(
        Box((0.22, 0.11, 0.12)),
        origin=Origin(xyz=(-0.18, 0.105, 1.62)),
        material=motor_grey,
        name="motor_housing",
    )
    frame.visual(
        Box((0.024, 0.018, 1.36)),
        origin=Origin(xyz=(-0.175, 0.102, 0.78)),
        material=rail_steel,
        name="left_guide_rail",
    )
    frame.visual(
        Box((0.024, 0.018, 1.36)),
        origin=Origin(xyz=(0.175, 0.102, 0.78)),
        material=rail_steel,
        name="right_guide_rail",
    )
    frame.visual(
        Box((0.022, 0.018, 1.36)),
        origin=Origin(xyz=(0.275, 0.102, 0.78)),
        material=rail_steel,
        name="counterweight_guide",
    )
    frame.visual(
        Box((0.032, 0.021, 1.30)),
        origin=Origin(xyz=(-0.175, 0.1205, 0.78)),
        material=painted_steel,
        name="left_rail_mount",
    )
    frame.visual(
        Box((0.032, 0.021, 1.30)),
        origin=Origin(xyz=(0.175, 0.1205, 0.78)),
        material=painted_steel,
        name="right_rail_mount",
    )
    frame.visual(
        Box((0.030, 0.021, 1.30)),
        origin=Origin(xyz=(0.275, 0.1205, 0.78)),
        material=painted_steel,
        name="counterweight_rail_mount",
    )
    frame.visual(
        Cylinder(radius=0.038, length=0.026),
        origin=Origin(xyz=(0.0, 0.087, 1.522), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="car_sheave",
    )
    frame.visual(
        Cylinder(radius=0.033, length=0.024),
        origin=Origin(xyz=(0.275, 0.087, 1.522), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="counterweight_sheave",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 0.40, 1.68)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.04, 0.84)),
    )

    car = model.part("car")
    car.visual(
        Box((0.30, 0.24, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=wood_trim,
        name="car_floor",
    )
    car.visual(
        Box((0.30, 0.018, 0.32)),
        origin=Origin(xyz=(0.0, 0.111, 0.18)),
        material=wood_panel,
        name="back_panel",
    )
    car.visual(
        Box((0.018, 0.224, 0.32)),
        origin=Origin(xyz=(-0.141, 0.0, 0.18)),
        material=wood_panel,
        name="left_panel",
    )
    car.visual(
        Box((0.018, 0.224, 0.32)),
        origin=Origin(xyz=(0.141, 0.0, 0.18)),
        material=wood_panel,
        name="right_panel",
    )
    car.visual(
        Box((0.30, 0.24, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.347)),
        material=wood_trim,
        name="roof_panel",
    )
    car.visual(
        Box((0.264, 0.018, 0.046)),
        origin=Origin(xyz=(0.0, -0.111, 0.317)),
        material=wood_trim,
        name="front_header",
    )
    car.visual(
        Box((0.03, 0.007, 0.08)),
        origin=Origin(xyz=(-0.165, 0.0895, 0.10)),
        material=rail_steel,
        name="left_lower_shoe",
    )
    car.visual(
        Box((0.03, 0.007, 0.08)),
        origin=Origin(xyz=(-0.165, 0.0895, 0.26)),
        material=rail_steel,
        name="left_upper_shoe",
    )
    car.visual(
        Box((0.03, 0.007, 0.08)),
        origin=Origin(xyz=(0.165, 0.0895, 0.10)),
        material=rail_steel,
        name="right_lower_shoe",
    )
    car.visual(
        Box((0.03, 0.007, 0.08)),
        origin=Origin(xyz=(0.165, 0.0895, 0.26)),
        material=rail_steel,
        name="right_upper_shoe",
    )
    car.inertial = Inertial.from_geometry(
        Box((0.36, 0.26, 0.36)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    door = model.part("access_door")
    door.visual(
        Box((0.258, 0.016, 0.274)),
        origin=Origin(xyz=(0.0, -0.008, 0.137)),
        material=wood_panel,
        name="door_panel",
    )
    door.visual(
        Box((0.224, 0.006, 0.214)),
        origin=Origin(xyz=(0.0, -0.005, 0.142)),
        material=wood_trim,
        name="door_inset",
    )
    door.visual(
        Box((0.018, 0.010, 0.050)),
        origin=Origin(xyz=(0.086, -0.021, 0.165)),
        material=hardware,
        name="door_pull",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.26, 0.03, 0.28)),
        mass=2.0,
        origin=Origin(xyz=(0.0, -0.008, 0.14)),
    )

    counterweight = model.part("counterweight")
    counterweight.visual(
        Box((0.070, 0.092, 0.32)),
        origin=Origin(xyz=(0.0, 0.04, 0.16)),
        material=counterweight_iron,
        name="weight_block",
    )
    counterweight.visual(
        Box((0.056, 0.086, 0.08)),
        origin=Origin(xyz=(0.0, 0.04, 0.05)),
        material=motor_grey,
        name="lower_keeper",
    )
    counterweight.visual(
        Box((0.056, 0.086, 0.08)),
        origin=Origin(xyz=(0.0, 0.04, 0.27)),
        material=motor_grey,
        name="upper_keeper",
    )
    counterweight.visual(
        Box((0.036, 0.007, 0.26)),
        origin=Origin(xyz=(0.024, 0.0895, 0.16)),
        material=rail_steel,
        name="guide_pad",
    )
    counterweight.inertial = Inertial.from_geometry(
        Box((0.08, 0.10, 0.32)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.04, 0.16)),
    )

    model.articulation(
        "frame_to_car_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.25,
            lower=0.0,
            upper=0.78,
        ),
    )
    model.articulation(
        "car_to_access_door",
        ArticulationType.REVOLUTE,
        parent=car,
        child=door,
        origin=Origin(xyz=(0.0, -0.12, 0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "frame_to_counterweight",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=counterweight,
        origin=Origin(xyz=(0.245, 0.0, 0.88)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.25,
            lower=-0.78,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    frame = object_model.get_part("frame")
    car = object_model.get_part("car")
    door = object_model.get_part("access_door")
    counterweight = object_model.get_part("counterweight")

    car_lift = object_model.get_articulation("frame_to_car_lift")
    door_hinge = object_model.get_articulation("car_to_access_door")
    counterweight_slide = object_model.get_articulation("frame_to_counterweight")

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
        "car lift runs vertically",
        tuple(car_lift.axis) == (0.0, 0.0, 1.0),
        f"Unexpected car lift axis: {car_lift.axis}",
    )
    ctx.check(
        "door hinge runs along car width",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0),
        f"Unexpected door hinge axis: {door_hinge.axis}",
    )
    ctx.check(
        "counterweight runs vertically",
        tuple(counterweight_slide.axis) == (0.0, 0.0, 1.0),
        f"Unexpected counterweight axis: {counterweight_slide.axis}",
    )

    ctx.expect_contact(
        car,
        frame,
        elem_a="left_upper_shoe",
        elem_b="left_guide_rail",
        name="car left guide shoe contacts left rail",
    )
    ctx.expect_contact(
        car,
        frame,
        elem_a="right_upper_shoe",
        elem_b="right_guide_rail",
        name="car right guide shoe contacts right rail",
    )
    ctx.expect_contact(
        counterweight,
        frame,
        elem_a="guide_pad",
        elem_b="counterweight_guide",
        name="counterweight guide pad contacts guide rail",
    )
    ctx.expect_contact(
        door,
        car,
        elem_a="door_panel",
        elem_b="car_floor",
        name="closed door seats on car sill",
    )

    car_rest = ctx.part_world_position(car)
    counterweight_rest = ctx.part_world_position(counterweight)
    closed_door_aabb = ctx.part_world_aabb(door)
    assert car_rest is not None
    assert counterweight_rest is not None
    assert closed_door_aabb is not None

    with ctx.pose({car_lift: 0.62}):
        car_high = ctx.part_world_position(car)
        assert car_high is not None
        ctx.check(
            "car raises on lift joint",
            car_high[2] > car_rest[2] + 0.60,
            f"Rest z={car_rest[2]:.3f}, raised z={car_high[2]:.3f}",
        )
        ctx.expect_contact(
            car,
            frame,
            elem_a="left_lower_shoe",
            elem_b="left_guide_rail",
            name="car remains guided when raised",
        )

    with ctx.pose({counterweight_slide: -0.55}):
        counterweight_low = ctx.part_world_position(counterweight)
        assert counterweight_low is not None
        ctx.check(
            "counterweight descends on slide",
            counterweight_low[2] < counterweight_rest[2] - 0.50,
            f"Rest z={counterweight_rest[2]:.3f}, lowered z={counterweight_low[2]:.3f}",
        )
        ctx.expect_contact(
            counterweight,
            frame,
            elem_a="guide_pad",
            elem_b="counterweight_guide",
            name="counterweight stays on guide when lowered",
        )

    with ctx.pose({door_hinge: math.radians(70.0)}):
        open_door_aabb = ctx.part_world_aabb(door)
        assert open_door_aabb is not None
        ctx.check(
            "door folds downward from bottom hinge",
            open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10
            and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.12,
            (
                f"Closed aabb={closed_door_aabb}, "
                f"open aabb={open_door_aabb}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
