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
    model = ArticulatedObject(name="travel_metronome")

    shell_black = model.material("shell_black", rgba=(0.12, 0.11, 0.10, 1.0))
    frame_black = model.material("frame_black", rgba=(0.18, 0.18, 0.17, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.63, 0.34, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.73, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.16, 0.16, 0.16, 1.0))

    body_width = 0.074
    body_depth = 0.036
    body_height = 0.108
    wall = 0.003
    body_bottom = 0.016
    stand_open = math.radians(28.0)

    housing = model.part("housing")
    housing.visual(
        Box((body_width, wall, body_height)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0 + wall / 2.0, body_bottom + body_height / 2.0)),
        material=shell_black,
        name="back_wall",
    )
    housing.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(
            xyz=(-body_width / 2.0 + wall / 2.0, 0.0, body_bottom + body_height / 2.0)
        ),
        material=shell_black,
        name="left_wall",
    )
    housing.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(
            xyz=(body_width / 2.0 - wall / 2.0, 0.0, body_bottom + body_height / 2.0)
        ),
        material=shell_black,
        name="right_wall",
    )
    housing.visual(
        Box((body_width, body_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + wall / 2.0)),
        material=shell_black,
        name="bottom_plate",
    )
    housing.visual(
        Box((body_width, body_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + body_height - wall / 2.0)),
        material=shell_black,
        name="top_plate",
    )
    housing.visual(
        Box((body_width - 2.0 * wall, wall, wall)),
        origin=Origin(
            xyz=(0.0, body_depth / 2.0 - wall / 2.0, body_bottom + body_height - wall / 2.0)
        ),
        material=frame_black,
        name="front_top_rail",
    )
    housing.visual(
        Box((wall, wall, body_height - 2.0 * wall)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + wall / 2.0,
                body_depth / 2.0 - wall / 2.0,
                body_bottom + body_height / 2.0,
            )
        ),
        material=frame_black,
        name="front_left_rail",
    )
    housing.visual(
        Box((wall, wall, body_height - 2.0 * wall)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - wall / 2.0,
                body_depth / 2.0 - wall / 2.0,
                body_bottom + body_height / 2.0,
            )
        ),
        material=frame_black,
        name="front_right_rail",
    )
    housing.visual(
        Box((body_width - 2.0 * wall, wall * 1.5, wall * 2.0)),
        origin=Origin(
            xyz=(0.0, body_depth / 2.0 - 0.00075, body_bottom + 0.010),
        ),
        material=frame_black,
        name="front_lower_lip",
    )
    housing.visual(
        Box((0.014, 0.0016, 0.074)),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0 + wall + 0.0008, body_bottom + 0.050),
        ),
        material=brass,
        name="tempo_scale",
    )
    housing.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(
            xyz=(0.0, 0.0, body_bottom + body_height - 0.012),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="pendulum_pivot_shaft",
    )
    housing.visual(
        Box((0.008, 0.010, 0.013)),
        origin=Origin(
            xyz=(0.0, 0.0, body_bottom + body_height - 0.0095),
        ),
        material=frame_black,
        name="pivot_bracket",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0 + 0.001, body_bottom + 0.044),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="key_escutcheon",
    )
    hinge_z = body_bottom
    housing.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(
            xyz=(-0.021, -body_depth / 2.0 - 0.0035, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=frame_black,
        name="stand_hinge_left",
    )
    housing.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(
            xyz=(0.021, -body_depth / 2.0 - 0.0035, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=frame_black,
        name="stand_hinge_right",
    )
    housing.visual(
        Box((0.012, 0.010, body_bottom + 0.004)),
        origin=Origin(
            xyz=(-body_width * 0.28, body_depth / 2.0 - 0.007, (body_bottom + 0.004) / 2.0),
        ),
        material=rubber,
        name="left_front_skid",
    )
    housing.visual(
        Box((0.012, 0.010, body_bottom + 0.004)),
        origin=Origin(
            xyz=(body_width * 0.28, body_depth / 2.0 - 0.007, (body_bottom + 0.004) / 2.0),
        ),
        material=rubber,
        name="right_front_skid",
    )
    housing.inertial = Inertial.from_geometry(
        Box((body_width, body_depth + 0.008, body_height + body_bottom)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, (body_bottom + body_height) / 2.0)),
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=frame_black,
        name="hinge_knuckle",
    )
    stand.visual(
        Box((body_width * 0.72, 0.042, 0.003)),
        origin=Origin(xyz=(0.0, -0.021, -0.0015)),
        material=frame_black,
        name="stand_panel",
    )
    stand.visual(
        Box((body_width * 0.52, 0.007, 0.0035)),
        origin=Origin(xyz=(0.0, -0.0385, -0.00175)),
        material=rubber,
        name="stand_tip",
    )
    stand.inertial = Inertial.from_geometry(
        Box((body_width * 0.72, 0.042, 0.006)),
        mass=0.05,
        origin=Origin(xyz=(0.0, -0.021, -0.0015)),
    )

    model.articulation(
        "housing_to_stand",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=stand,
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0 - 0.0035, hinge_z),
            rpy=(stand_open, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=1.2,
            lower=-stand_open,
            upper=0.0,
        ),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Box((0.006, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="pendulum_hanger",
    )
    pendulum.visual(
        Cylinder(radius=0.0013, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=steel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.0022, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.0, -0.009),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="pendulum_pivot_eye",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.010, 0.010, 0.090)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.0, body_bottom + body_height - 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=-0.34,
            upper=0.34,
        ),
    )

    weight = model.part("weight")
    weight.visual(
        Cylinder(radius=0.0095, length=0.0022),
        origin=Origin(
            xyz=(0.0, 0.0036, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="weight_disc_front",
    )
    weight.visual(
        Cylinder(radius=0.0095, length=0.0022),
        origin=Origin(
            xyz=(0.0, -0.0036, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="weight_disc_back",
    )
    weight.visual(
        Box((0.006, 0.0095, 0.010)),
        origin=Origin(xyz=(0.0075, 0.0, 0.0)),
        material=brass,
        name="weight_bridge",
    )
    weight.visual(
        Box((0.0034, 0.004, 0.016)),
        origin=Origin(xyz=(0.0030, 0.0, 0.0)),
        material=frame_black,
        name="slider_shoe",
    )
    weight.visual(
        Box((0.0035, 0.002, 0.016)),
        origin=Origin(xyz=(0.0045, 0.0038, 0.0)),
        material=frame_black,
        name="guide_front",
    )
    weight.visual(
        Box((0.0035, 0.002, 0.016)),
        origin=Origin(xyz=(0.0045, -0.0038, 0.0)),
        material=frame_black,
        name="guide_back",
    )
    weight.inertial = Inertial.from_geometry(
        Box((0.022, 0.010, 0.020)),
        mass=0.04,
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
    )

    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=0.05,
            lower=-0.018,
            upper=0.018,
        ),
    )

    key = model.part("winding_key")
    key.visual(
        Cylinder(radius=0.0075, length=0.004),
        origin=Origin(
            xyz=(0.0, -0.002, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="key_disc",
    )
    key.visual(
        Cylinder(radius=0.0025, length=0.002),
        origin=Origin(
            xyz=(0.0, -0.005, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="key_shaft_stub",
    )
    key.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0075, length=0.004),
        mass=0.01,
        origin=Origin(
            xyz=(0.0, -0.002, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
    )

    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=key,
        origin=Origin(xyz=(0.0, -body_depth / 2.0, body_bottom + 0.044)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.08,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    stand = object_model.get_part("stand")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    key = object_model.get_part("winding_key")
    stand_joint = object_model.get_articulation("housing_to_stand")
    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    weight_joint = object_model.get_articulation("pendulum_to_weight")

    ctx.expect_gap(
        stand,
        housing,
        axis="y",
        max_gap=0.0,
        max_penetration=0.050,
        positive_elem="stand_panel",
        negative_elem="back_wall",
        name="deployed stand stays directly behind the housing",
    )

    deployed_tip = ctx.part_element_world_aabb(stand, elem="stand_tip")
    closed_tip = None
    with ctx.pose({stand_joint: stand_joint.motion_limits.lower}):
        ctx.expect_gap(
            housing,
            stand,
            axis="z",
            min_gap=0.0,
            max_gap=0.012,
            positive_elem="bottom_plate",
            negative_elem="stand_panel",
            name="closed stand tucks near the housing underside",
        )
        closed_tip = ctx.part_element_world_aabb(stand, elem="stand_tip")

    deployed_z = deployed_tip[1][2] if deployed_tip is not None else None
    closed_z = closed_tip[1][2] if closed_tip is not None else None
    ctx.check(
        "stand folds upward toward the housing",
        deployed_z is not None and closed_z is not None and closed_z > deployed_z + 0.015,
        details=f"deployed_tip_max_z={deployed_z}, closed_tip_max_z={closed_z}",
    )

    ctx.expect_gap(
        housing,
        key,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="back_wall",
        negative_elem="key_disc",
        name="winding key sits flush with the back face",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((a + b) * 0.5 for a, b in zip(mins, maxs))

    left_weight = None
    right_weight = None
    with ctx.pose({pendulum_joint: pendulum_joint.motion_limits.lower}):
        left_weight = _aabb_center(ctx.part_element_world_aabb(weight, elem="weight_disc_front"))
    with ctx.pose({pendulum_joint: pendulum_joint.motion_limits.upper}):
        right_weight = _aabb_center(ctx.part_element_world_aabb(weight, elem="weight_disc_front"))

    ctx.check(
        "pendulum swings left and right from the top pivot",
        left_weight is not None
        and right_weight is not None
        and abs(left_weight[0]) > 0.010
        and abs(right_weight[0]) > 0.010
        and left_weight[0] * right_weight[0] < 0.0,
        details=f"left_weight_center={left_weight}, right_weight_center={right_weight}",
    )

    upper_weight = None
    lower_weight = None
    with ctx.pose({pendulum_joint: 0.0, weight_joint: weight_joint.motion_limits.lower}):
        upper_weight = _aabb_center(ctx.part_element_world_aabb(weight, elem="weight_disc_front"))
    with ctx.pose({pendulum_joint: 0.0, weight_joint: weight_joint.motion_limits.upper}):
        lower_weight = _aabb_center(ctx.part_element_world_aabb(weight, elem="weight_disc_front"))

    ctx.check(
        "weight slides downward along the pendulum rod",
        upper_weight is not None
        and lower_weight is not None
        and lower_weight[2] < upper_weight[2] - 0.020,
        details=f"upper_weight_center={upper_weight}, lower_weight_center={lower_weight}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
