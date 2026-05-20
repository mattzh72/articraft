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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rack_and_pinion_residential_elevator")

    tower_steel = model.material("tower_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    rack_steel = model.material("rack_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    car_shell = model.material("car_shell", rgba=(0.86, 0.87, 0.89, 1.0))
    car_trim = model.material("car_trim", rgba=(0.72, 0.74, 0.77, 1.0))
    door_finish = model.material("door_finish", rgba=(0.79, 0.80, 0.82, 1.0))
    gearbox_dark = model.material("gearbox_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.90, 0.48, 0.10, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((1.35, 1.35, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=tower_steel,
        name="base_plinth",
    )
    tower.visual(
        Box((0.08, 0.10, 3.10)),
        origin=Origin(xyz=(-0.58, -0.50, 1.65)),
        material=tower_steel,
        name="left_rear_upright",
    )
    tower.visual(
        Box((0.08, 0.10, 3.10)),
        origin=Origin(xyz=(-0.58, 0.50, 1.65)),
        material=tower_steel,
        name="left_front_upright",
    )
    tower.visual(
        Box((0.10, 1.10, 3.10)),
        origin=Origin(xyz=(0.59, 0.0, 1.65)),
        material=tower_steel,
        name="rack_mast",
    )
    tower.visual(
        Box((1.27, 0.08, 0.08)),
        origin=Origin(xyz=(0.005, -0.50, 3.16)),
        material=tower_steel,
        name="rear_top_beam",
    )
    tower.visual(
        Box((1.27, 0.08, 0.08)),
        origin=Origin(xyz=(0.005, 0.50, 3.16)),
        material=tower_steel,
        name="front_top_beam",
    )
    tower.visual(
        Box((0.04, 0.82, 2.90)),
        origin=Origin(xyz=(0.52, 0.0, 1.55)),
        material=car_trim,
        name="guide_rail",
    )
    tower.visual(
        Box((0.025, 0.12, 2.86)),
        origin=Origin(xyz=(0.5275, 0.0, 1.59)),
        material=rack_steel,
        name="rack_spine",
    )
    for tooth_index in range(24):
        tower.visual(
            Box((0.018, 0.055, 0.045)),
            origin=Origin(xyz=(0.506, 0.0, 0.24 + tooth_index * 0.11)),
            material=rack_steel,
            name=f"rack_tooth_{tooth_index:02d}",
        )

    car = model.part("car")
    car.visual(
        Box((0.96, 1.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=car_trim,
        name="floor",
    )
    car.visual(
        Box((0.96, 1.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 2.10)),
        material=car_shell,
        name="roof",
    )
    car.visual(
        Box((0.03, 1.08, 2.03)),
        origin=Origin(xyz=(-0.465, 0.0, 1.065)),
        material=car_shell,
        name="left_wall",
    )
    car.visual(
        Box((0.03, 1.08, 2.03)),
        origin=Origin(xyz=(0.465, 0.0, 1.065)),
        material=car_shell,
        name="right_wall",
    )
    car.visual(
        Box((0.90, 0.03, 2.03)),
        origin=Origin(xyz=(0.0, -0.525, 1.065)),
        material=car_shell,
        name="rear_wall",
    )
    car.visual(
        Box((0.10, 0.03, 1.89)),
        origin=Origin(xyz=(-0.43, 0.525, 0.995)),
        material=car_shell,
        name="front_jamb_left",
    )
    car.visual(
        Box((0.20, 0.03, 1.89)),
        origin=Origin(xyz=(0.38, 0.525, 0.995)),
        material=car_shell,
        name="front_pocket_right",
    )
    car.visual(
        Box((0.96, 0.03, 0.14)),
        origin=Origin(xyz=(0.0, 0.525, 2.01)),
        material=car_shell,
        name="front_header",
    )
    car.visual(
        Box((1.44, 0.05, 0.04)),
        origin=Origin(xyz=(0.24, 0.59, 1.97)),
        material=car_trim,
        name="door_track",
    )
    car.visual(
        Box((0.05, 0.025, 0.06)),
        origin=Origin(xyz=(-0.22, 0.5525, 2.02)),
        material=car_trim,
        name="door_track_bracket_left",
    )
    car.visual(
        Box((0.05, 0.025, 0.06)),
        origin=Origin(xyz=(0.06, 0.5525, 2.02)),
        material=car_trim,
        name="door_track_bracket_mid",
    )
    car.visual(
        Box((0.05, 0.025, 0.06)),
        origin=Origin(xyz=(0.34, 0.5525, 2.02)),
        material=car_trim,
        name="door_track_bracket_right",
    )
    car.visual(
        Box((0.02, 0.18, 0.10)),
        origin=Origin(xyz=(0.49, 0.0, 0.46)),
        material=gearbox_dark,
        name="lower_guide_shoe",
    )
    car.visual(
        Box((0.02, 0.18, 0.10)),
        origin=Origin(xyz=(0.49, 0.0, 1.66)),
        material=gearbox_dark,
        name="upper_guide_shoe",
    )
    car.visual(
        Box((0.14, 0.26, 0.18)),
        origin=Origin(xyz=(0.40, -0.12, 1.93)),
        material=gearbox_dark,
        name="drive_housing",
    )
    car.visual(
        Cylinder(radius=0.045, length=0.06),
        origin=Origin(
            xyz=(0.435, -0.12, 1.84),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gearbox_dark,
        name="pinion_cover",
    )
    car.visual(
        Box((0.02, 0.02, 0.06)),
        origin=Origin(xyz=(0.43, 0.10, 2.05)),
        material=car_trim,
        name="catch_bracket_rear",
    )
    car.visual(
        Box((0.02, 0.02, 0.06)),
        origin=Origin(xyz=(0.43, 0.22, 2.05)),
        material=car_trim,
        name="catch_bracket_front",
    )

    door = model.part("door_panel")
    door.visual(
        Box((0.70, 0.02, 1.88)),
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
        material=door_finish,
        name="door_panel",
    )
    door.visual(
        Box((0.05, 0.018, 0.02)),
        origin=Origin(xyz=(-0.20, 0.0, 1.94)),
        material=car_trim,
        name="left_hanger",
    )
    door.visual(
        Box((0.05, 0.018, 0.02)),
        origin=Origin(xyz=(0.20, 0.0, 1.94)),
        material=car_trim,
        name="right_hanger",
    )
    door.visual(
        Box((0.03, 0.025, 0.55)),
        origin=Origin(xyz=(-0.23, 0.0, 1.08)),
        material=gearbox_dark,
        name="handle_rail",
    )

    safety_arm = model.part("safety_catch_arm")
    safety_arm.visual(
        Cylinder(radius=0.016, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gearbox_dark,
        name="hinge_barrel",
    )
    safety_arm.visual(
        Box((0.16, 0.03, 0.02)),
        origin=Origin(xyz=(-0.08, 0.0, -0.01)),
        material=safety_orange,
        name="arm_body",
    )
    safety_arm.visual(
        Box((0.05, 0.03, 0.09)),
        origin=Origin(xyz=(-0.145, 0.0, -0.055)),
        material=safety_orange,
        name="arm_tip",
    )

    model.articulation(
        "tower_to_car",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.25,
            lower=0.0,
            upper=0.85,
        ),
    )
    model.articulation(
        "car_to_door_panel",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door,
        origin=Origin(xyz=(-0.04, 0.565, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.40,
            lower=0.0,
            upper=0.60,
        ),
    )
    model.articulation(
        "car_to_safety_catch_arm",
        ArticulationType.REVOLUTE,
        parent=car,
        child=safety_arm,
        origin=Origin(xyz=(0.43, 0.16, 2.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    car = object_model.get_part("car")
    door = object_model.get_part("door_panel")
    safety_arm = object_model.get_part("safety_catch_arm")
    lift_joint = object_model.get_articulation("tower_to_car")
    door_joint = object_model.get_articulation("car_to_door_panel")
    arm_joint = object_model.get_articulation("car_to_safety_catch_arm")

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

    ctx.check(
        "car lifts on the vertical axis",
        tuple(lift_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lift_joint.axis}",
    )
    ctx.check(
        "door slides laterally across the opening",
        tuple(door_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "safety arm hinges below the roof",
        tuple(arm_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={arm_joint.axis}",
    )

    ctx.expect_contact(
        car,
        tower,
        elem_a="lower_guide_shoe",
        elem_b="guide_rail",
        name="lower guide shoe bears on the guide rail",
    )
    ctx.expect_contact(
        car,
        tower,
        elem_a="upper_guide_shoe",
        elem_b="guide_rail",
        name="upper guide shoe bears on the guide rail",
    )
    ctx.expect_contact(
        door,
        car,
        elem_a="left_hanger",
        elem_b="door_track",
        name="door left hanger is supported by the track",
    )
    ctx.expect_contact(
        door,
        car,
        elem_a="right_hanger",
        elem_b="door_track",
        name="door right hanger is supported by the track",
    )
    ctx.expect_gap(
        door,
        car,
        axis="y",
        positive_elem="door_panel",
        negative_elem="front_header",
        min_gap=0.010,
        max_gap=0.030,
        name="door panel rides slightly proud of the front frame",
    )
    ctx.expect_contact(
        safety_arm,
        car,
        elem_a="hinge_barrel",
        elem_b="catch_bracket_front",
        name="safety arm barrel is captured by the front bracket",
    )
    ctx.expect_contact(
        safety_arm,
        car,
        elem_a="hinge_barrel",
        elem_b="catch_bracket_rear",
        name="safety arm barrel is captured by the rear bracket",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    rest_car_pos = ctx.part_world_position(car)
    rest_door_pos = ctx.part_world_position(door)
    rest_arm_tip = _aabb_center(ctx.part_element_world_aabb(safety_arm, elem="arm_tip"))

    lift_upper = lift_joint.motion_limits.upper if lift_joint.motion_limits is not None else None
    door_upper = door_joint.motion_limits.upper if door_joint.motion_limits is not None else None
    arm_upper = arm_joint.motion_limits.upper if arm_joint.motion_limits is not None else None

    with ctx.pose({lift_joint: lift_upper}):
        raised_car_pos = ctx.part_world_position(car)
        ctx.expect_contact(
            car,
            tower,
            elem_a="upper_guide_shoe",
            elem_b="guide_rail",
            name="car remains guided at full lift",
        )

    with ctx.pose({door_joint: door_upper}):
        open_door_pos = ctx.part_world_position(door)
        ctx.expect_contact(
            door,
            car,
            elem_a="right_hanger",
            elem_b="door_track",
            name="door stays hung on the track when open",
        )

    with ctx.pose({arm_joint: arm_upper}):
        deployed_arm_tip = _aabb_center(ctx.part_element_world_aabb(safety_arm, elem="arm_tip"))

    ctx.check(
        "car rises through meaningful travel",
        rest_car_pos is not None
        and raised_car_pos is not None
        and raised_car_pos[2] > rest_car_pos[2] + 0.70,
        details=f"rest={rest_car_pos}, raised={raised_car_pos}",
    )
    ctx.check(
        "door clears laterally when opened",
        rest_door_pos is not None
        and open_door_pos is not None
        and open_door_pos[0] > rest_door_pos[0] + 0.50,
        details=f"rest={rest_door_pos}, open={open_door_pos}",
    )
    ctx.check(
        "safety arm tip drops on deployment",
        rest_arm_tip is not None
        and deployed_arm_tip is not None
        and deployed_arm_tip[2] < rest_arm_tip[2] - 0.06,
        details=f"rest_tip={rest_arm_tip}, deployed_tip={deployed_arm_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
