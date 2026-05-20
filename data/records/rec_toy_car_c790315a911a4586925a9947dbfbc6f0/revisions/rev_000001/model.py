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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_coupe_car")

    body_red = model.material("body_red", color=(0.84, 0.16, 0.12))
    glass_smoke = model.material("glass_smoke", rgba=(0.18, 0.22, 0.28, 0.55))
    tire_black = model.material("tire_black", color=(0.08, 0.08, 0.08))
    rim_gray = model.material("rim_gray", color=(0.77, 0.78, 0.80))
    axle_gray = model.material("axle_gray", color=(0.34, 0.36, 0.39))

    wheel_radius = 0.034
    wheel_width = 0.022
    wheel_y = 0.069
    front_axle_x = 0.092
    rear_axle_x = -0.092
    axle_z = wheel_radius
    door_hinge_x = 0.043
    door_hinge_y = 0.056
    door_hinge_z = 0.062
    door_open_limit = 1.15

    body = model.part("body")
    body.visual(
        Box((0.260, 0.110, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.026)),
        material=body_red,
        name="floor_pan",
    )
    body.visual(
        Box((0.175, 0.100, 0.020)),
        origin=Origin(xyz=(-0.010, 0.000, 0.038)),
        material=body_red,
        name="cabin_core",
    )
    body.visual(
        Box((0.090, 0.110, 0.040)),
        origin=Origin(xyz=(0.095, 0.000, 0.055)),
        material=body_red,
        name="hood",
    )
    body.visual(
        Box((0.034, 0.104, 0.028)),
        origin=Origin(xyz=(0.147, 0.000, 0.042)),
        material=body_red,
        name="nose",
    )
    body.visual(
        Box((0.080, 0.110, 0.038)),
        origin=Origin(xyz=(-0.106, 0.000, 0.055)),
        material=body_red,
        name="trunk",
    )
    body.visual(
        Box((0.032, 0.102, 0.026)),
        origin=Origin(xyz=(-0.148, 0.000, 0.041)),
        material=body_red,
        name="tail",
    )
    body.visual(
        Box((0.112, 0.092, 0.028)),
        origin=Origin(xyz=(-0.008, 0.000, 0.091)),
        material=body_red,
        name="roof",
    )
    body.visual(
        Box((0.024, 0.086, 0.020)),
        origin=Origin(xyz=(0.037, 0.000, 0.078), rpy=(0.000, -0.52, 0.000)),
        material=glass_smoke,
        name="windshield",
    )
    body.visual(
        Box((0.025, 0.080, 0.018)),
        origin=Origin(xyz=(-0.066, 0.000, 0.078), rpy=(0.000, 0.42, 0.000)),
        material=glass_smoke,
        name="rear_window",
    )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((0.146, 0.010, 0.026)),
            origin=Origin(xyz=(-0.004, side_sign * 0.048, 0.046)),
            material=body_red,
            name=f"{side_name}_sill",
        )
        body.visual(
            Box((0.014, 0.010, 0.044)),
            origin=Origin(xyz=(0.046, side_sign * 0.048, 0.068)),
            material=body_red,
            name=f"{side_name}_a_pillar",
        )
        body.visual(
            Box((0.018, 0.010, 0.044)),
            origin=Origin(xyz=(-0.054, side_sign * 0.048, 0.067)),
            material=body_red,
            name=f"{side_name}_c_pillar",
        )
        body.visual(
            Box((0.068, 0.012, 0.020)),
            origin=Origin(xyz=(front_axle_x, side_sign * 0.050, 0.082)),
            material=body_red,
            name=f"{side_name}_front_arch",
        )
        body.visual(
            Box((0.068, 0.012, 0.020)),
            origin=Origin(xyz=(rear_axle_x, side_sign * 0.050, 0.081)),
            material=body_red,
            name=f"{side_name}_rear_arch",
        )

    body.visual(
        Box((0.022, 0.030, 0.016)),
        origin=Origin(xyz=(front_axle_x, 0.000, 0.038)),
        material=axle_gray,
        name="front_axle_support",
    )
    body.visual(
        Box((0.022, 0.030, 0.016)),
        origin=Origin(xyz=(rear_axle_x, 0.000, 0.038)),
        material=axle_gray,
        name="rear_axle_support",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.116),
        origin=Origin(xyz=(front_axle_x, 0.000, axle_z), rpy=(pi / 2.0, 0.000, 0.000)),
        material=axle_gray,
        name="front_axle_beam",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.116),
        origin=Origin(xyz=(rear_axle_x, 0.000, axle_z), rpy=(pi / 2.0, 0.000, 0.000)),
        material=axle_gray,
        name="rear_axle_beam",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.300, 0.120, 0.105)),
        mass=0.85,
        origin=Origin(xyz=(0.000, 0.000, 0.053)),
    )

    for side_name, side_sign, axis_sign in (
        ("left", 1.0, -1.0),
        ("right", -1.0, 1.0),
    ):
        door = model.part(f"{side_name}_door")
        door.visual(
            Box((0.084, 0.006, 0.040)),
            origin=Origin(xyz=(-0.043, 0.000, -0.006)),
            material=body_red,
            name="door_skin",
        )
        door.visual(
            Box((0.056, 0.004, 0.026)),
            origin=Origin(xyz=(-0.047, 0.000, 0.018)),
            material=glass_smoke,
            name="door_glass",
        )
        door.visual(
            Box((0.014, 0.006, 0.022)),
            origin=Origin(xyz=(-0.076, 0.000, 0.010)),
            material=body_red,
            name="rear_frame",
        )
        door.inertial = Inertial.from_geometry(
            Box((0.086, 0.006, 0.062)),
            mass=0.05,
            origin=Origin(xyz=(-0.043, 0.000, 0.000)),
        )
        model.articulation(
            f"body_to_{side_name}_door",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(door_hinge_x, side_sign * door_hinge_y, door_hinge_z)),
            axis=(0.000, 0.000, axis_sign),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=2.5,
                lower=0.0,
                upper=door_open_limit,
            ),
        )

    wheel_specs = (
        ("left_front_wheel", front_axle_x, wheel_y, 1.0),
        ("right_front_wheel", front_axle_x, -wheel_y, -1.0),
        ("left_rear_wheel", rear_axle_x, wheel_y, 1.0),
        ("right_rear_wheel", rear_axle_x, -wheel_y, -1.0),
    )
    for wheel_name, wheel_x, wheel_y_pos, outer_sign in wheel_specs:
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=wheel_radius, length=wheel_width),
            origin=Origin(rpy=(pi / 2.0, 0.000, 0.000)),
            material=tire_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.019, length=wheel_width + 0.002),
            origin=Origin(rpy=(pi / 2.0, 0.000, 0.000)),
            material=rim_gray,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(
                xyz=(0.000, outer_sign * 0.010, 0.000),
                rpy=(pi / 2.0, 0.000, 0.000),
            ),
            material=axle_gray,
            name="hub_cap",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=wheel_width),
            mass=0.08,
            origin=Origin(rpy=(pi / 2.0, 0.000, 0.000)),
        )
        model.articulation(
            f"body_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(wheel_x, wheel_y_pos, axle_z)),
            axis=(0.000, 1.000, 0.000),
            motion_limits=MotionLimits(effort=0.6, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")
    left_rear_wheel = object_model.get_part("left_rear_wheel")
    right_rear_wheel = object_model.get_part("right_rear_wheel")

    left_hinge = object_model.get_articulation("body_to_left_door")
    right_hinge = object_model.get_articulation("body_to_right_door")
    wheel_joints = [
        object_model.get_articulation("body_to_left_front_wheel"),
        object_model.get_articulation("body_to_right_front_wheel"),
        object_model.get_articulation("body_to_left_rear_wheel"),
        object_model.get_articulation("body_to_right_rear_wheel"),
    ]

    ctx.check(
        "left door hinge is vertical at the A-pillar",
        left_hinge.articulation_type == ArticulationType.REVOLUTE
        and left_hinge.axis == (0.0, 0.0, -1.0),
        details=f"type={left_hinge.articulation_type}, axis={left_hinge.axis}",
    )
    ctx.check(
        "right door hinge is vertical at the A-pillar",
        right_hinge.articulation_type == ArticulationType.REVOLUTE
        and right_hinge.axis == (0.0, 0.0, 1.0),
        details=f"type={right_hinge.articulation_type}, axis={right_hinge.axis}",
    )

    for joint in wheel_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is a free-spinning wheel joint",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=(
                f"type={joint.articulation_type}, axis={joint.axis}, "
                f"limits={limits}"
            ),
        )

    ctx.expect_origin_gap(
        left_front_wheel,
        body,
        axis="y",
        min_gap=0.060,
        max_gap=0.080,
        name="left front wheel sits outboard of the body",
    )
    ctx.expect_origin_gap(
        body,
        right_front_wheel,
        axis="y",
        min_gap=0.060,
        max_gap=0.080,
        name="right front wheel sits outboard of the body",
    )
    ctx.expect_origin_gap(
        left_door,
        body,
        axis="y",
        min_gap=0.050,
        max_gap=0.065,
        name="left door is mounted on the left side opening",
    )
    ctx.expect_origin_gap(
        body,
        right_door,
        axis="y",
        min_gap=0.050,
        max_gap=0.065,
        name="right door is mounted on the right side opening",
    )
    ctx.expect_origin_gap(
        left_front_wheel,
        left_rear_wheel,
        axis="x",
        min_gap=0.170,
        max_gap=0.190,
        name="left wheels define a short toy-car wheelbase",
    )
    ctx.expect_contact(
        body,
        left_front_wheel,
        elem_a="front_axle_beam",
        elem_b="tire",
        name="front axle beam reaches the left front tire",
    )
    ctx.expect_contact(
        body,
        right_front_wheel,
        elem_a="front_axle_beam",
        elem_b="tire",
        name="front axle beam reaches the right front tire",
    )
    ctx.expect_contact(
        body,
        left_rear_wheel,
        elem_a="rear_axle_beam",
        elem_b="tire",
        name="rear axle beam reaches the left rear tire",
    )
    ctx.expect_contact(
        body,
        right_rear_wheel,
        elem_a="rear_axle_beam",
        elem_b="tire",
        name="rear axle beam reaches the right rear tire",
    )

    left_closed_aabb = ctx.part_element_world_aabb(left_door, elem="door_skin")
    right_closed_aabb = ctx.part_element_world_aabb(right_door, elem="door_skin")
    with ctx.pose({left_hinge: 0.95, right_hinge: 0.95}):
        left_open_aabb = ctx.part_element_world_aabb(left_door, elem="door_skin")
        right_open_aabb = ctx.part_element_world_aabb(right_door, elem="door_skin")

    ctx.check(
        "left door swings outward",
        left_closed_aabb is not None
        and left_open_aabb is not None
        and left_open_aabb[1][1] > left_closed_aabb[1][1] + 0.020,
        details=f"closed={left_closed_aabb}, open={left_open_aabb}",
    )
    ctx.check(
        "right door swings outward",
        right_closed_aabb is not None
        and right_open_aabb is not None
        and right_open_aabb[0][1] < right_closed_aabb[0][1] - 0.020,
        details=f"closed={right_closed_aabb}, open={right_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
