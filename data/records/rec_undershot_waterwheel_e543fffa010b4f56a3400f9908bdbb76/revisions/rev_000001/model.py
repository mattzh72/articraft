from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin

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
    TorusGeometry,
    mesh_from_geometry,
)


def _add_side_frame(
    support,
    *,
    y_center: float,
    wood,
    dark_wood,
) -> None:
    base_z = -0.92
    front_post_height = 1.02
    rear_post_height = 1.14
    brace_dx = 0.44
    brace_dz = 0.68
    brace_angle = -atan2(brace_dz, brace_dx)
    brace_length = (brace_dx**2 + brace_dz**2) ** 0.5

    support.visual(
        Box((1.12, 0.08, 0.08)),
        origin=Origin(xyz=(0.06, y_center, base_z)),
        material=dark_wood,
    )
    support.visual(
        Box((0.08, 0.08, front_post_height)),
        origin=Origin(xyz=(-0.34, y_center, base_z + 0.04 + front_post_height * 0.5)),
        material=wood,
    )
    support.visual(
        Box((0.08, 0.08, rear_post_height)),
        origin=Origin(xyz=(0.40, y_center, base_z + 0.04 + rear_post_height * 0.5)),
        material=wood,
    )
    support.visual(
        Box((0.82, 0.08, 0.08)),
        origin=Origin(xyz=(0.03, y_center, 0.16)),
        material=dark_wood,
    )
    support.visual(
        Box((brace_length, 0.05, 0.05)),
        origin=Origin(xyz=(-0.20, y_center, -0.36), rpy=(0.0, brace_angle, 0.0)),
        material=dark_wood,
    )


def _build_wheel_part(
    part,
    *,
    rim_radius: float,
    rim_tube: float,
    rim_offset: float,
    hub_radius: float,
    hub_length: float,
    axle_radius: float,
    axle_length: float,
    wheel_width: float,
    wood,
    iron,
) -> None:
    ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=rim_radius, tube=rim_tube, radial_segments=16, tubular_segments=40).rotate_x(pi / 2.0),
        "waterwheel_rim",
    )

    part.visual(ring_mesh, origin=Origin(xyz=(0.0, rim_offset, 0.0)), material=iron, name="left_rim")
    part.visual(ring_mesh, origin=Origin(xyz=(0.0, -rim_offset, 0.0)), material=iron, name="right_rim")
    part.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub",
    )

    spoke_radius = 0.29
    spoke_length = 0.58
    for spoke_index in range(8):
        angle = 2.0 * pi * spoke_index / 8.0
        part.visual(
            Box((spoke_length, wheel_width * 0.82, 0.06)),
            origin=Origin(
                xyz=(cos(angle) * spoke_radius, 0.0, sin(angle) * spoke_radius),
                rpy=(0.0, -angle, 0.0),
            ),
            material=wood,
            name=f"spoke_{spoke_index}",
        )

    paddle_radius = 0.51
    for paddle_index in range(12):
        angle = 2.0 * pi * paddle_index / 12.0
        part.visual(
            Box((0.17, wheel_width, 0.035)),
            origin=Origin(
                xyz=(cos(angle) * paddle_radius, 0.0, sin(angle) * paddle_radius),
                rpy=(0.0, -(angle + pi / 2.0), 0.0),
            ),
            material=wood,
            name=f"paddle_{paddle_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    timber = model.material("timber", rgba=(0.56, 0.40, 0.24, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.39, 0.28, 0.16, 1.0))
    iron = model.material("iron", rgba=(0.35, 0.37, 0.40, 1.0))

    support = model.part("support_frame")
    support.inertial = Inertial.from_geometry(
        Box((1.24, 0.84, 1.48)),
        mass=180.0,
        origin=Origin(xyz=(0.08, 0.0, -0.26)),
    )

    _add_side_frame(support, y_center=0.34, wood=timber, dark_wood=dark_timber)
    _add_side_frame(support, y_center=-0.34, wood=timber, dark_wood=dark_timber)

    support.visual(
        Box((0.10, 0.76, 0.10)),
        origin=Origin(xyz=(-0.34, 0.0, -0.74)),
        material=dark_timber,
    )
    support.visual(
        Box((0.10, 0.76, 0.10)),
        origin=Origin(xyz=(0.40, 0.0, -0.76)),
        material=dark_timber,
    )
    support.visual(
        Box((0.12, 0.10, 0.18)),
        origin=Origin(xyz=(0.04, 0.34, 0.03)),
        material=dark_timber,
        name="left_bearing",
    )
    support.visual(
        Box((0.12, 0.10, 0.18)),
        origin=Origin(xyz=(0.04, -0.34, 0.03)),
        material=dark_timber,
        name="right_bearing",
    )

    support.visual(
        Box((0.86, 0.72, 0.08)),
        origin=Origin(xyz=(0.18, 0.0, -0.68)),
        material=dark_timber,
        name="channel_floor",
    )
    support.visual(
        Box((0.86, 0.06, 0.32)),
        origin=Origin(xyz=(0.18, 0.37, -0.48)),
        material=timber,
        name="left_channel_wall",
    )
    support.visual(
        Box((0.86, 0.06, 0.32)),
        origin=Origin(xyz=(0.18, -0.37, -0.48)),
        material=timber,
        name="right_channel_wall",
    )
    support.visual(
        Box((0.10, 0.72, 0.28)),
        origin=Origin(xyz=(0.56, 0.0, -0.50)),
        material=timber,
        name="trough_backstop",
    )

    wheel = model.part("waterwheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.60, length=0.40),
        mass=60.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _build_wheel_part(
        wheel,
        rim_radius=0.53,
        rim_tube=0.035,
        rim_offset=0.18,
        hub_radius=0.12,
        hub_length=0.30,
        axle_radius=0.045,
        axle_length=0.58,
        wheel_width=0.38,
        wood=timber,
        iron=iron,
    )

    model.articulation(
        "axle_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("waterwheel")
    spin = object_model.get_articulation("axle_spin")

    ctx.expect_origin_distance(
        wheel,
        support,
        axes="xz",
        min_dist=0.0,
        max_dist=1e-6,
        name="wheel axis is centered on the support",
    )
    ctx.expect_gap(
        support,
        wheel,
        axis="y",
        positive_elem="left_bearing",
        negative_elem="left_rim",
        min_gap=0.06,
        max_gap=0.10,
        name="left rim clears the left bearing block",
    )
    ctx.expect_gap(
        wheel,
        support,
        axis="y",
        positive_elem="right_rim",
        negative_elem="right_bearing",
        min_gap=0.06,
        max_gap=0.10,
        name="right rim clears the right bearing block",
    )
    ctx.expect_gap(
        wheel,
        support,
        axis="z",
        negative_elem="channel_floor",
        min_gap=0.04,
        max_gap=0.12,
        name="wheel sits just above the trough floor",
    )

    rest_position = ctx.part_world_position(wheel)
    with ctx.pose({spin: pi / 3.0}):
        turned_position = ctx.part_world_position(wheel)

    ctx.check(
        "continuous spin keeps the wheel on the same axle line",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0] - turned_position[0]) <= 1e-6
        and abs(rest_position[1] - turned_position[1]) <= 1e-6
        and abs(rest_position[2] - turned_position[2]) <= 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
