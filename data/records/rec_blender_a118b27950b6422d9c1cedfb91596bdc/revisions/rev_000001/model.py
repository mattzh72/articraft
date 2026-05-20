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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_mesh(name: str, outer_profile, inner_profile):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="centrifugal_fruit_juicer")

    body_metal = model.material("body_metal", rgba=(0.84, 0.85, 0.86, 1.0))
    base_black = model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.62, 0.70, 0.76, 0.42))
    dark_plastic = model.material("dark_plastic", rgba=(0.13, 0.13, 0.14, 1.0))
    basket_metal = model.material("basket_metal", rgba=(0.78, 0.79, 0.81, 1.0))
    accent_black = model.material("accent_black", rgba=(0.06, 0.06, 0.07, 1.0))

    housing = model.part("housing")
    housing.visual(
        _shell_mesh(
            "juicer_housing_shell",
            [
                (0.030, 0.000),
                (0.095, 0.008),
                (0.130, 0.028),
                (0.145, 0.090),
                (0.142, 0.150),
                (0.136, 0.175),
            ],
            [
                (0.000, 0.012),
                (0.085, 0.020),
                (0.118, 0.032),
                (0.125, 0.090),
                (0.122, 0.148),
                (0.116, 0.166),
            ],
        ),
        material=body_metal,
        name="housing_shell",
    )
    housing.visual(
        Cylinder(radius=0.103, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=base_black,
        name="motor_base",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.163, 0.0, 0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_metal,
        name="juice_spout",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.203, 0.0, 0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_black,
        name="spout_lip",
    )
    housing.visual(
        Box((0.050, 0.140, 0.010)),
        origin=Origin(xyz=(-0.124, 0.0, 0.171)),
        material=dark_plastic,
        name="rear_hinge_bridge",
    )
    housing.visual(
        Box((0.018, 0.030, 0.022)),
        origin=Origin(xyz=(-0.135, -0.045, 0.163)),
        material=dark_plastic,
        name="left_hinge_shoulder",
    )
    housing.visual(
        Box((0.018, 0.030, 0.022)),
        origin=Origin(xyz=(-0.135, 0.045, 0.163)),
        material=dark_plastic,
        name="right_hinge_shoulder",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=accent_black,
        name="drive_hub",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.410, 0.310, 0.185)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0925)),
    )

    cover = model.part("loading_cover")
    cover.visual(
        _shell_mesh(
            "juicer_cover_dome",
            [
                (0.052, 0.028),
                (0.088, 0.022),
                (0.120, 0.011),
                (0.140, 0.000),
            ],
            [
                (0.044, 0.027),
                (0.082, 0.019),
                (0.114, 0.009),
                (0.133, 0.002),
            ],
        ),
        origin=Origin(xyz=(0.140, 0.0, -0.001)),
        material=clear_smoke,
        name="cover_dome",
    )
    cover.visual(
        _shell_mesh(
            "juicer_feed_chute",
            [
                (0.050, 0.000),
                (0.049, 0.055),
                (0.047, 0.140),
            ],
            [
                (0.040, 0.004),
                (0.039, 0.055),
                (0.037, 0.138),
            ],
        ),
        origin=Origin(xyz=(0.160, 0.0, 0.025)),
        material=clear_smoke,
        name="feed_chute_shell",
    )
    cover.visual(
        Box((0.024, 0.074, 0.010)),
        origin=Origin(xyz=(0.100, 0.0, 0.049)),
        material=dark_plastic,
        name="chute_pivot_block",
    )
    cover.visual(
        Box((0.012, 0.012, 0.022)),
        origin=Origin(xyz=(0.100, -0.034, 0.060)),
        material=dark_plastic,
        name="left_pivot_ear",
    )
    cover.visual(
        Box((0.012, 0.012, 0.022)),
        origin=Origin(xyz=(0.100, 0.034, 0.060)),
        material=dark_plastic,
        name="right_pivot_ear",
    )
    cover.visual(
        Box((0.060, 0.110, 0.012)),
        origin=Origin(xyz=(0.030, 0.0, 0.006)),
        material=dark_plastic,
        name="cover_rear_bridge",
    )
    cover.visual(
        Cylinder(radius=0.009, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="cover_hinge_barrel",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.300, 0.290, 0.180)),
        mass=0.9,
        origin=Origin(xyz=(0.145, 0.0, 0.070)),
    )

    feed_arm = model.part("feed_arm")
    feed_arm.visual(
        Cylinder(radius=0.004, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_black,
        name="arm_pivot_pin",
    )
    feed_arm.visual(
        Box((0.008, 0.020, 0.140)),
        origin=Origin(xyz=(-0.002, 0.0, 0.070)),
        material=dark_plastic,
        name="arm_spine",
    )
    feed_arm.visual(
        Box((0.100, 0.024, 0.018)),
        origin=Origin(xyz=(0.035, 0.0, 0.145)),
        material=dark_plastic,
        name="arm_handle",
    )
    feed_arm.visual(
        Box((0.012, 0.018, 0.092)),
        origin=Origin(xyz=(0.060, 0.0, 0.099)),
        material=dark_plastic,
        name="guide_finger",
    )
    feed_arm.visual(
        Box((0.026, 0.028, 0.010)),
        origin=Origin(xyz=(0.055, 0.0, 0.050)),
        material=dark_plastic,
        name="guide_paddle",
    )
    feed_arm.inertial = Inertial.from_geometry(
        Box((0.070, 0.030, 0.165)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
    )

    basket = model.part("basket")
    basket.visual(
        _shell_mesh(
            "juicer_basket_shell",
            [
                (0.018, 0.000),
                (0.070, 0.008),
                (0.090, 0.014),
                (0.092, 0.062),
                (0.086, 0.074),
            ],
            [
                (0.000, 0.006),
                (0.060, 0.012),
                (0.080, 0.016),
                (0.082, 0.060),
                (0.076, 0.070),
            ],
        ),
        material=basket_metal,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.012, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=basket_metal,
        name="basket_spindle",
    )
    basket.visual(
        Cylinder(radius=0.075, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=basket_metal,
        name="grater_disk",
    )
    basket.visual(
        Box((0.090, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=accent_black,
        name="blade_x",
    )
    basket.visual(
        Box((0.006, 0.090, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=accent_black,
        name="blade_y",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.092, length=0.084),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )

    model.articulation(
        "housing_to_cover",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cover,
        origin=Origin(xyz=(-0.140, 0.0, 0.185)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    model.articulation(
        "cover_to_feed_arm",
        ArticulationType.REVOLUTE,
        parent=cover,
        child=feed_arm,
        origin=Origin(xyz=(0.100, 0.0, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(52.0),
        ),
    )

    model.articulation(
        "housing_to_basket",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    cover = object_model.get_part("loading_cover")
    feed_arm = object_model.get_part("feed_arm")
    basket = object_model.get_part("basket")

    cover_hinge = object_model.get_articulation("housing_to_cover")
    arm_hinge = object_model.get_articulation("cover_to_feed_arm")
    basket_spin = object_model.get_articulation("housing_to_basket")

    ctx.expect_within(
        basket,
        housing,
        axes="xy",
        inner_elem="basket_shell",
        outer_elem="housing_shell",
        margin=0.0,
        name="basket stays centered within the round housing",
    )

    with ctx.pose({cover_hinge: 0.0, arm_hinge: 0.0}):
        ctx.expect_overlap(
            cover,
            housing,
            axes="xy",
            elem_a="cover_dome",
            elem_b="housing_shell",
            min_overlap=0.22,
            name="cover closes over the housing mouth",
        )
        ctx.expect_gap(
            cover,
            housing,
            axis="z",
            positive_elem="cover_dome",
            negative_elem="housing_shell",
            min_gap=0.0,
            max_gap=0.020,
            name="cover sits just above the housing rim when closed",
        )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: math.radians(70.0)}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "cover opens upward from the rear hinge",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.10
        and open_cover_aabb[0][0] < closed_cover_aabb[0][0] - 0.02,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    with ctx.pose({cover_hinge: 0.0, arm_hinge: 0.0}):
        rest_arm_aabb = ctx.part_world_aabb(feed_arm)
    with ctx.pose({cover_hinge: 0.0, arm_hinge: math.radians(45.0)}):
        tilted_arm_aabb = ctx.part_world_aabb(feed_arm)
    ctx.check(
        "feed arm tilts downward into the chute",
        rest_arm_aabb is not None
        and tilted_arm_aabb is not None
        and tilted_arm_aabb[0][2] < rest_arm_aabb[0][2] - 0.010
        and tilted_arm_aabb[1][0] > rest_arm_aabb[1][0] + 0.010,
        details=f"rest={rest_arm_aabb}, tilted={tilted_arm_aabb}",
    )

    basket_limits = basket_spin.motion_limits
    ctx.check(
        "basket articulation is a vertical continuous spin",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and basket_spin.axis == (0.0, 0.0, 1.0)
        and basket_limits is not None
        and basket_limits.lower is None
        and basket_limits.upper is None,
        details=(
            f"type={basket_spin.articulation_type}, axis={basket_spin.axis}, "
            f"limits={basket_limits}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
