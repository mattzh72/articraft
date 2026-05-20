from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


def _roll_for_radial(angle: float) -> float:
    return angle - (pi * 0.5)


def _add_wheel_geometry(wheel, *, wood, iron) -> None:
    spin_axis = Origin(rpy=(0.0, pi * 0.5, 0.0))

    wheel.visual(
        Cylinder(radius=0.032, length=0.62),
        origin=spin_axis,
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.100, length=0.18),
        origin=spin_axis,
        material=iron,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.370, length=0.030),
        origin=Origin(xyz=(-0.180, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=iron,
        name="left_rim_disc",
    )
    wheel.visual(
        Cylinder(radius=0.370, length=0.030),
        origin=Origin(xyz=(0.180, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=iron,
        name="right_rim_disc",
    )

    for spoke_index in range(8):
        angle = (2.0 * pi * spoke_index) / 8.0
        radius = 0.210
        wheel.visual(
            Box((0.360, 0.036, 0.240)),
            origin=Origin(
                xyz=(0.0, radius * cos(angle), radius * sin(angle)),
                rpy=(_roll_for_radial(angle), 0.0, 0.0),
            ),
            material=wood,
            name=f"spoke_{spoke_index}",
        )

    for paddle_index in range(10):
        angle = (2.0 * pi * paddle_index) / 10.0
        radius = 0.312
        wheel.visual(
            Box((0.410, 0.050, 0.120)),
            origin=Origin(
                xyz=(0.0, radius * cos(angle), radius * sin(angle)),
                rpy=(_roll_for_radial(angle), 0.0, 0.0),
            ),
            material=wood,
            name=f"paddle_{paddle_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    timber = model.material("timber", rgba=(0.54, 0.38, 0.22, 1.0))
    wet_wood = model.material("wet_wood", rgba=(0.42, 0.29, 0.17, 1.0))
    iron = model.material("iron", rgba=(0.28, 0.29, 0.31, 1.0))

    support = model.part("support_frame")
    support.inertial = Inertial.from_geometry(
        Box((0.84, 1.08, 0.92)),
        mass=120.0,
        origin=Origin(xyz=(0.0, -0.02, 0.46)),
    )

    for x, side_name in [(-0.270, "left"), (0.270, "right")]:
        support.visual(
            Box((0.120, 1.020, 0.080)),
            origin=Origin(xyz=(x, -0.02, 0.040)),
            material=timber,
            name=f"{side_name}_skid",
        )
        support.visual(
            Box((0.080, 0.100, 0.700)),
            origin=Origin(xyz=(x, 0.220, 0.350)),
            material=timber,
            name=f"{side_name}_front_post",
        )
        support.visual(
            Box((0.080, 0.100, 0.880)),
            origin=Origin(xyz=(x, -0.240, 0.440)),
            material=timber,
            name=f"{side_name}_rear_post",
        )
        support.visual(
            Box((0.080, 0.560, 0.100)),
            origin=Origin(xyz=(x, -0.010, 0.650)),
            material=timber,
            name=f"{side_name}_top_cap",
        )
        support.visual(
            Box((0.080, 0.120, 0.060)),
            origin=Origin(xyz=(x, 0.020, 0.488)),
            material=timber,
            name=f"{side_name}_bearing_block",
        )
        support.visual(
            Box((0.080, 0.030, 0.170)),
            origin=Origin(xyz=(x, -0.045, 0.545)),
            material=timber,
            name=f"{side_name}_rear_bearing_cheek",
        )
        support.visual(
            Box((0.080, 0.030, 0.170)),
            origin=Origin(xyz=(x, 0.085, 0.545)),
            material=timber,
            name=f"{side_name}_front_bearing_cheek",
        )
        support.visual(
            Box((0.080, 0.160, 0.040)),
            origin=Origin(xyz=(x, 0.020, 0.620)),
            material=timber,
            name=f"{side_name}_axle_hanger",
        )
        support.visual(
            Box((0.080, 0.540, 0.080)),
            origin=Origin(xyz=(x, -0.030, 0.300), rpy=(0.62, 0.0, 0.0)),
            material=timber,
            name=f"{side_name}_brace",
        )

    for y, beam_name in [(-0.340, "rear_tie"), (0.000, "center_tie"), (0.320, "front_tie")]:
        support.visual(
            Box((0.500, 0.100, 0.080)),
            origin=Origin(xyz=(0.0, y, 0.090)),
            material=timber,
            name=beam_name,
        )

    support.visual(
        Box((0.740, 0.220, 0.040)),
        origin=Origin(xyz=(0.0, -0.270, 0.900)),
        material=timber,
        name="trough_floor",
    )
    support.visual(
        Box((0.740, 0.040, 0.100)),
        origin=Origin(xyz=(0.0, -0.160, 0.980)),
        material=timber,
        name="trough_lip",
    )
    support.visual(
        Box((0.040, 0.220, 0.140)),
        origin=Origin(xyz=(-0.350, -0.270, 0.970)),
        material=timber,
        name="left_trough_wall",
    )
    support.visual(
        Box((0.040, 0.220, 0.140)),
        origin=Origin(xyz=(0.350, -0.270, 0.970)),
        material=timber,
        name="right_trough_wall",
    )
    support.visual(
        Box((0.740, 0.060, 0.180)),
        origin=Origin(xyz=(0.0, -0.360, 0.980)),
        material=timber,
        name="back_splash",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.370, length=0.410),
        mass=34.0,
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
    )
    _add_wheel_geometry(wheel, wood=wet_wood, iron=iron)

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.020, 0.550)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.expect_origin_gap(
        wheel,
        support,
        axis="z",
        min_gap=0.48,
        max_gap=0.60,
        name="wheel axle stays low to the ground frame",
    )
    ctx.expect_gap(
        support,
        wheel,
        axis="x",
        positive_elem="right_bearing_block",
        negative_elem="right_rim_disc",
        min_gap=0.015,
        max_gap=0.060,
        name="right bearing block clears the wheel side",
    )
    ctx.expect_gap(
        wheel,
        support,
        axis="x",
        positive_elem="left_rim_disc",
        negative_elem="left_bearing_block",
        min_gap=0.015,
        max_gap=0.060,
        name="left bearing block clears the wheel side",
    )
    ctx.expect_gap(
        support,
        wheel,
        axis="z",
        positive_elem="trough_lip",
        min_gap=0.005,
        max_gap=0.090,
        name="trough lip sits above the wheel",
    )
    ctx.expect_contact(
        support,
        wheel,
        elem_a="left_bearing_block",
        elem_b="axle",
        contact_tol=0.0005,
        name="left bearing block supports the axle",
    )
    ctx.expect_contact(
        support,
        wheel,
        elem_a="right_bearing_block",
        elem_b="axle",
        contact_tol=0.0005,
        name="right bearing block supports the axle",
    )

    with ctx.pose({spin: pi / 10.0}):
        ctx.expect_gap(
            support,
            wheel,
            axis="x",
            positive_elem="right_bearing_block",
            negative_elem="right_rim_disc",
            min_gap=0.010,
            max_gap=0.070,
            name="right bearing stays clear as the wheel turns",
        )
        ctx.expect_gap(
            wheel,
            support,
            axis="x",
            positive_elem="left_rim_disc",
            negative_elem="left_bearing_block",
            min_gap=0.010,
            max_gap=0.070,
            name="left bearing stays clear as the wheel turns",
        )
        ctx.expect_gap(
            support,
            wheel,
            axis="z",
            positive_elem="trough_lip",
            min_gap=0.005,
            max_gap=0.120,
            name="turned paddles remain below the trough edge",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
