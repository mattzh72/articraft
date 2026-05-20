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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_metronome")

    walnut = model.material("walnut", rgba=(0.42, 0.27, 0.16, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.30, 0.19, 0.11, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.66, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.08, 1.0))

    foot_w = 0.170
    foot_d = 0.140
    foot_h = 0.018

    housing_w = 0.120
    housing_d = 0.090
    housing_h = 0.260
    wall_t = 0.008
    front_t = 0.008
    housing_z = foot_h - 0.002

    slot_w = 0.082
    slot_bottom_z = housing_z + 0.030
    slot_top_z = housing_z + housing_h - 0.028
    stile_w = (housing_w - slot_w) * 0.5
    rail_overlap = 0.008

    housing = model.part("housing")
    housing.visual(
        Box((foot_w, foot_d, foot_h)),
        origin=Origin(xyz=(0.0, 0.0, foot_h * 0.5)),
        material=dark_walnut,
        name="foot_base",
    )
    housing.visual(
        Box((housing_w, housing_d, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, housing_z + 0.005)),
        material=walnut,
        name="housing_plinth",
    )
    housing.visual(
        Box((wall_t, housing_d, housing_h)),
        origin=Origin(
            xyz=(-(housing_w * 0.5) + wall_t * 0.5, 0.0, housing_z + housing_h * 0.5)
        ),
        material=walnut,
        name="left_wall",
    )
    housing.visual(
        Box((wall_t, housing_d, housing_h)),
        origin=Origin(
            xyz=((housing_w * 0.5) - wall_t * 0.5, 0.0, housing_z + housing_h * 0.5)
        ),
        material=walnut,
        name="right_wall",
    )
    housing.visual(
        Box((housing_w - 2.0 * wall_t, wall_t, housing_h)),
        origin=Origin(
            xyz=(0.0, (housing_d * 0.5) - wall_t * 0.5, housing_z + housing_h * 0.5)
        ),
        material=walnut,
        name="back_panel",
    )
    housing.visual(
        Box((housing_w - 2.0 * wall_t, housing_d - wall_t, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, housing_z + wall_t * 0.5)),
        material=walnut,
        name="floor_panel",
    )
    housing.visual(
        Box((housing_w - 2.0 * wall_t, housing_d - wall_t, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, housing_z + housing_h - wall_t * 0.5)),
        material=walnut,
        name="roof_panel",
    )
    housing.visual(
        Box((stile_w, front_t, housing_h)),
        origin=Origin(
            xyz=(
                -(slot_w * 0.5) - stile_w * 0.5,
                -(housing_d * 0.5) + front_t * 0.5,
                housing_z + housing_h * 0.5,
            )
        ),
        material=walnut,
        name="left_stile",
    )
    housing.visual(
        Box((stile_w, front_t, housing_h)),
        origin=Origin(
            xyz=(
                (slot_w * 0.5) + stile_w * 0.5,
                -(housing_d * 0.5) + front_t * 0.5,
                housing_z + housing_h * 0.5,
            )
        ),
        material=walnut,
        name="right_stile",
    )
    housing.visual(
        Box((slot_w + rail_overlap, front_t, slot_bottom_z - housing_z)),
        origin=Origin(
            xyz=(
                0.0,
                -(housing_d * 0.5) + front_t * 0.5,
                housing_z + (slot_bottom_z - housing_z) * 0.5,
            )
        ),
        material=walnut,
        name="lower_rail",
    )
    housing.visual(
        Box((slot_w + rail_overlap, front_t, housing_z + housing_h - slot_top_z)),
        origin=Origin(
            xyz=(
                0.0,
                -(housing_d * 0.5) + front_t * 0.5,
                slot_top_z + (housing_z + housing_h - slot_top_z) * 0.5,
            )
        ),
        material=walnut,
        name="upper_rail",
    )
    housing.visual(
        Box((0.020, 0.002, slot_top_z - slot_bottom_z - 0.018)),
        origin=Origin(
            xyz=(
                0.0,
                (housing_d * 0.5) - wall_t - 0.001,
                (slot_bottom_z + slot_top_z) * 0.5,
            )
        ),
        material=brass,
        name="tempo_scale",
    )
    housing.visual(
        Box((0.024, 0.024, 0.036)),
        origin=Origin(
            xyz=(
                0.0,
                -(housing_d * 0.5) + front_t + 0.027,
                housing_z + housing_h - 0.022,
            )
        ),
        material=dark_walnut,
        name="pivot_mount",
    )
    housing.inertial = Inertial.from_geometry(
        Box((foot_w, foot_d, housing_z + housing_h)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, (housing_z + housing_h) * 0.5)),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="pendulum_hub",
    )
    pendulum.visual(
        Cylinder(radius=0.0017, length=0.208),
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
        material=steel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.0036, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.203)),
        material=brass,
        name="pendulum_tip",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.016, 0.014, 0.214)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
    )

    weight_outer_profile = [
        (-0.014, 0.019),
        (0.014, 0.019),
        (0.0115, 0.004),
        (0.0085, -0.022),
        (-0.0085, -0.022),
        (-0.0115, 0.004),
    ]
    weight_hole_profile = [
        (-0.0028, 0.0135),
        (0.0028, 0.0135),
        (0.0028, -0.0185),
        (-0.0028, -0.0185),
    ]
    weight_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            weight_outer_profile,
            [weight_hole_profile],
            0.014,
            center=True,
        ),
        "metronome_weight",
    )

    weight = model.part("weight")
    weight.visual(
        weight_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="sliding_weight",
    )
    weight.inertial = Inertial.from_geometry(
        Box((0.028, 0.014, 0.042)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.0015)),
    )

    key = model.part("key")
    key.visual(
        Cylinder(radius=0.003, length=0.010),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="key_stem",
    )
    key.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brass,
        name="key_disc",
    )
    key.visual(
        Cylinder(radius=0.0026, length=0.010),
        origin=Origin(xyz=(-0.014, 0.010, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=black,
        name="key_grip",
    )
    key.inertial = Inertial.from_geometry(
        Box((0.030, 0.028, 0.020)),
        mass=0.03,
        origin=Origin(xyz=(-0.013, 0.0, 0.0)),
    )

    pendulum_joint_origin = Origin(
        xyz=(
            0.0,
            -(housing_d * 0.5) + front_t + 0.012,
            housing_z + housing_h - 0.024,
        )
    )
    model.articulation(
        "pendulum_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=pendulum_joint_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=1.8,
            lower=-0.15,
            upper=0.15,
        ),
    )

    model.articulation(
        "weight_slide",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.12,
            lower=0.0,
            upper=0.085,
        ),
    )

    model.articulation(
        "winding_key_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=key,
        origin=Origin(
            xyz=(
                -(housing_w * 0.5),
                0.0,
                housing_z + housing_h * 0.5,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    weight = object_model.get_part("weight")
    key = object_model.get_part("key")

    pendulum_joint = object_model.get_articulation("pendulum_pivot")
    weight_joint = object_model.get_articulation("weight_slide")
    key_joint = object_model.get_articulation("winding_key_spin")

    ctx.check(
        "articulations match metronome mechanism",
        pendulum_joint.joint_type == ArticulationType.REVOLUTE
        and weight_joint.joint_type == ArticulationType.PRISMATIC
        and key_joint.joint_type == ArticulationType.CONTINUOUS,
        details=(
            f"pendulum={pendulum_joint.joint_type}, "
            f"weight={weight_joint.joint_type}, key={key_joint.joint_type}"
        ),
    )

    rest_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({weight_joint: weight_joint.motion_limits.upper}):
        lowered_weight_pos = ctx.part_world_position(weight)
    ctx.check(
        "weight slides downward on the rod",
        rest_weight_pos is not None
        and lowered_weight_pos is not None
        and lowered_weight_pos[2] < rest_weight_pos[2] - 0.05,
        details=f"rest={rest_weight_pos}, lowered={lowered_weight_pos}",
    )

    mid_travel = (weight_joint.motion_limits.upper or 0.0) * 0.5
    with ctx.pose({pendulum_joint: pendulum_joint.motion_limits.lower, weight_joint: mid_travel}):
        right_swing_pos = ctx.part_world_position(weight)
    with ctx.pose({pendulum_joint: pendulum_joint.motion_limits.upper, weight_joint: mid_travel}):
        left_swing_pos = ctx.part_world_position(weight)
    ctx.check(
        "pendulum swings across the front slot",
        left_swing_pos is not None
        and right_swing_pos is not None
        and left_swing_pos[0] < -0.01
        and right_swing_pos[0] > 0.01,
        details=f"left={left_swing_pos}, right={right_swing_pos}",
    )

    with ctx.pose(
        {pendulum_joint: pendulum_joint.motion_limits.upper, weight_joint: weight_joint.motion_limits.upper}
    ):
        ctx.expect_gap(
            weight,
            housing,
            axis="x",
            min_gap=0.002,
            positive_elem="sliding_weight",
            negative_elem="left_stile",
            name="weight clears left stile at left swing",
        )
        ctx.expect_gap(
            housing,
            weight,
            axis="x",
            min_gap=0.010,
            positive_elem="right_stile",
            negative_elem="sliding_weight",
            name="weight remains inside slot at left swing",
        )

    with ctx.pose(
        {pendulum_joint: pendulum_joint.motion_limits.lower, weight_joint: weight_joint.motion_limits.upper}
    ):
        ctx.expect_gap(
            weight,
            housing,
            axis="x",
            min_gap=0.010,
            positive_elem="sliding_weight",
            negative_elem="left_stile",
            name="weight remains inside slot at right swing",
        )
        ctx.expect_gap(
            housing,
            weight,
            axis="x",
            min_gap=0.002,
            positive_elem="right_stile",
            negative_elem="sliding_weight",
            name="weight clears right stile at right swing",
        )

    with ctx.pose({weight_joint: 0.0}):
        ctx.expect_gap(
            housing,
            weight,
            axis="z",
            min_gap=0.006,
            positive_elem="upper_rail",
            negative_elem="sliding_weight",
            name="weight stays below upper rail",
        )

    with ctx.pose({weight_joint: weight_joint.motion_limits.upper}):
        ctx.expect_gap(
            weight,
            housing,
            axis="z",
            min_gap=0.008,
            positive_elem="sliding_weight",
            negative_elem="lower_rail",
            name="weight stays above lower rail",
        )

    ctx.expect_contact(
        key,
        housing,
        contact_tol=0.0005,
        elem_a="key_stem",
        elem_b="left_wall",
        name="key stem mounts on left side wall",
    )
    with ctx.pose({key_joint: 1.3}):
        ctx.expect_contact(
            key,
            housing,
            contact_tol=0.0005,
            elem_a="key_stem",
            elem_b="left_wall",
            name="key remains seated while rotating",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
