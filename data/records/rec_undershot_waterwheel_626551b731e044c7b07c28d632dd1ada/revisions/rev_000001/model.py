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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    weathered_timber = model.material("weathered_timber", rgba=(0.48, 0.34, 0.22, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.34, 0.23, 0.15, 1.0))
    damp_timber = model.material("damp_timber", rgba=(0.28, 0.19, 0.12, 1.0))
    iron = model.material("iron", rgba=(0.33, 0.35, 0.38, 1.0))

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((1.02, 1.02, 1.18)),
        mass=180.0,
        origin=Origin(xyz=(0.0, -0.14, 0.59)),
    )

    support_frame.visual(
        Box((0.08, 0.86, 1.10)),
        origin=Origin(xyz=(0.47, -0.14, 0.55)),
        material=weathered_timber,
        name="left_cheek",
    )
    support_frame.visual(
        Box((0.08, 0.86, 1.10)),
        origin=Origin(xyz=(-0.47, -0.14, 0.55)),
        material=weathered_timber,
        name="right_cheek",
    )
    support_frame.visual(
        Box((1.02, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.18, 1.04)),
        material=dark_timber,
        name="top_tie",
    )
    support_frame.visual(
        Box((1.02, 0.22, 0.12)),
        origin=Origin(xyz=(0.0, -0.54, 0.10)),
        material=dark_timber,
        name="bottom_sill",
    )
    support_frame.visual(
        Box((0.94, 0.44, 0.08)),
        origin=Origin(xyz=(0.0, -0.40, 0.76)),
        material=damp_timber,
        name="trough_floor",
    )
    support_frame.visual(
        Box((0.94, 0.08, 0.28)),
        origin=Origin(xyz=(0.0, -0.58, 0.86)),
        material=weathered_timber,
        name="rear_stop",
    )
    support_frame.visual(
        Box((0.14, 0.16, 0.16)),
        origin=Origin(xyz=(0.36, 0.06, 0.56)),
        material=dark_timber,
        name="left_bearing",
    )
    support_frame.visual(
        Box((0.14, 0.16, 0.16)),
        origin=Origin(xyz=(-0.36, 0.06, 0.56)),
        material=dark_timber,
        name="right_bearing",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.36, length=0.24),
        mass=55.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    wheel.visual(
        Cylinder(radius=0.045, length=0.58),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.080, length=0.24),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_timber,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.35, length=0.02),
        origin=Origin(xyz=(0.11, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=weathered_timber,
        name="left_wheel_side",
    )
    wheel.visual(
        Cylinder(radius=0.35, length=0.02),
        origin=Origin(xyz=(-0.11, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=weathered_timber,
        name="right_wheel_side",
    )

    for spoke_index in range(4):
        angle = spoke_index * (pi / 2.0)
        wheel.visual(
            Box((0.24, 0.03, 0.22)),
            origin=Origin(
                xyz=(0.0, -0.15 * sin(angle), 0.15 * cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_timber,
            name=f"spoke_{spoke_index}",
        )

    for paddle_index in range(8):
        angle = paddle_index * (pi / 4.0)
        wheel.visual(
            Box((0.24, 0.06, 0.18)),
            origin=Origin(
                xyz=(0.0, -0.27 * sin(angle), 0.27 * cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=weathered_timber,
            name=f"paddle_{paddle_index}",
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.16, 0.56)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.expect_contact(
        frame,
        wheel,
        elem_a="left_bearing",
        elem_b="axle",
        name="left bearing carries the axle",
    )
    ctx.expect_contact(
        frame,
        wheel,
        elem_a="right_bearing",
        elem_b="axle",
        name="right bearing carries the axle",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="top_tie",
        max_gap=0.18,
        max_penetration=0.0,
        name="wheel stays below the top tie beam",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({spin: pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)

    position_stable = (
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6
    )
    ctx.check(
        "wheel rotates in place about the axle",
        position_stable,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
