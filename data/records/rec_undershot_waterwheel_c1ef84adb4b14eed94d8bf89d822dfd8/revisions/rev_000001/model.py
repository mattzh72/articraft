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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    stone = model.material("stone", rgba=(0.63, 0.61, 0.57, 1.0))
    timber = model.material("timber", rgba=(0.41, 0.28, 0.16, 1.0))
    weathered_timber = model.material("weathered_timber", rgba=(0.54, 0.39, 0.24, 1.0))
    iron = model.material("iron", rgba=(0.23, 0.24, 0.25, 1.0))

    support = model.part("support_frame")
    support.inertial = Inertial.from_geometry(
        Box((2.60, 1.30, 2.25)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.125)),
    )

    # Stone plinth tying the two timber side frames together.
    support.visual(
        Box((2.60, 1.30, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stone,
        name="plinth",
    )
    support.visual(
        Box((2.10, 0.82, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=stone,
        name="upper_base",
    )

    for y in (-0.44, 0.44):
        support.visual(
            Box((0.14, 0.12, 1.96)),
            origin=Origin(xyz=(-0.36, y, 1.10)),
            material=timber,
            name=f"post_rear_{'left' if y > 0 else 'right'}",
        )
        support.visual(
            Box((0.14, 0.12, 1.96)),
            origin=Origin(xyz=(0.36, y, 1.10)),
            material=timber,
            name=f"post_front_{'left' if y > 0 else 'right'}",
        )
        support.visual(
            Box((0.86, 0.12, 0.14)),
            origin=Origin(xyz=(0.0, y, 2.14)),
            material=timber,
            name=f"cap_{'left' if y > 0 else 'right'}",
        )
        support.visual(
            Box((0.86, 0.12, 0.14)),
            origin=Origin(xyz=(0.0, y, 0.48)),
            material=timber,
            name=f"sill_{'left' if y > 0 else 'right'}",
        )
        support.visual(
            Box((0.12, 0.12, 1.78)),
            origin=Origin(xyz=(0.0, y, 1.18), rpy=(0.0, 0.40, 0.0)),
            material=weathered_timber,
            name=f"brace_rising_{'left' if y > 0 else 'right'}",
        )
        support.visual(
            Box((0.12, 0.12, 1.78)),
            origin=Origin(xyz=(0.0, y, 1.18), rpy=(0.0, -0.40, 0.0)),
            material=weathered_timber,
            name=f"brace_falling_{'left' if y > 0 else 'right'}",
        )
        support.visual(
            Box((0.20, 0.10, 0.20)),
            origin=Origin(xyz=(0.0, y * 0.95, 1.50)),
            material=iron,
            name=f"bearing_block_{'left' if y > 0 else 'right'}",
        )

    support.visual(
        Box((0.16, 0.94, 0.16)),
        origin=Origin(xyz=(-0.36, 0.0, 2.10)),
        material=timber,
        name="rear_tie_beam",
    )
    support.visual(
        Box((0.16, 0.94, 0.16)),
        origin=Origin(xyz=(0.36, 0.0, 2.10)),
        material=timber,
        name="front_tie_beam",
    )
    support.visual(
        Cylinder(radius=0.05, length=0.12),
        origin=Origin(xyz=(0.0, 0.50, 1.50), rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_bed_left",
    )
    support.visual(
        Cylinder(radius=0.05, length=0.12),
        origin=Origin(xyz=(0.0, -0.50, 1.50), rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_bed_right",
    )

    # Simple elevated mill race approaching the wheel on the upstream side.
    support.visual(
        Box((0.98, 0.74, 0.08)),
        origin=Origin(xyz=(-0.80, 0.0, 0.78)),
        material=weathered_timber,
        name="race_floor",
    )
    support.visual(
        Box((0.98, 0.08, 0.28)),
        origin=Origin(xyz=(-0.80, -0.33, 0.92)),
        material=weathered_timber,
        name="race_wall_right",
    )
    support.visual(
        Box((0.98, 0.08, 0.28)),
        origin=Origin(xyz=(-0.80, 0.33, 0.92)),
        material=weathered_timber,
        name="race_wall_left",
    )
    support.visual(
        Box((0.12, 0.74, 0.18)),
        origin=Origin(xyz=(-0.31, 0.0, 0.82)),
        material=weathered_timber,
        name="race_lip",
    )
    support.visual(
        Box((0.12, 0.12, 0.95)),
        origin=Origin(xyz=(-0.92, -0.24, 0.59)),
        material=timber,
        name="race_support_right",
    )
    support.visual(
        Box((0.12, 0.12, 0.95)),
        origin=Origin(xyz=(-0.92, 0.24, 0.59)),
        material=timber,
        name="race_support_left",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Box((1.10, 0.58, 1.10)),
        mass=95.0,
        origin=Origin(),
    )

    wheel.visual(
        Cylinder(radius=0.045, length=0.736),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.11, length=0.40),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.16, length=0.06),
        origin=Origin(xyz=(0.0, 0.17, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_plate_left",
    )
    wheel.visual(
        Cylinder(radius=0.16, length=0.06),
        origin=Origin(xyz=(0.0, -0.17, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_plate_right",
    )

    rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.50, tube=0.030, radial_segments=18, tubular_segments=72).rotate_x(pi / 2.0),
        "waterwheel_rim",
    )
    wheel.visual(rim_mesh, origin=Origin(xyz=(0.0, 0.23, 0.0)), material=iron, name="rim_left")
    wheel.visual(rim_mesh, origin=Origin(xyz=(0.0, -0.23, 0.0)), material=iron, name="rim_right")

    for spoke_index in range(8):
        angle = (2.0 * pi * spoke_index) / 8.0
        spoke_radius = 0.28
        wheel.visual(
            Box((0.05, 0.44, 0.38)),
            origin=Origin(
                xyz=(sin(angle) * spoke_radius, 0.0, cos(angle) * spoke_radius),
                rpy=(0.0, angle, 0.0),
            ),
            material=weathered_timber,
            name=f"spoke_{spoke_index:02d}",
        )

    for paddle_index in range(10):
        angle = (2.0 * pi * paddle_index) / 10.0
        paddle_radius = 0.47
        wheel.visual(
            Box((0.16, 0.54, 0.045)),
            origin=Origin(
                xyz=(sin(angle) * paddle_radius, 0.0, cos(angle) * paddle_radius),
                rpy=(0.0, angle, 0.0),
            ),
            material=weathered_timber,
            name=f"paddle_{paddle_index:02d}",
        )

    wheel.visual(
        Cylinder(radius=0.07, length=0.06),
        origin=Origin(xyz=(0.0, 0.33, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_collar_left",
    )
    wheel.visual(
        Cylinder(radius=0.07, length=0.06),
        origin=Origin(xyz=(0.0, -0.33, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_collar_right",
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.0),
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

    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.expect_contact(
        wheel,
        support,
        elem_a="axle",
        elem_b="bearing_block_left",
        name="axle touches left bearing block",
    )
    ctx.expect_contact(
        wheel,
        support,
        elem_a="axle",
        elem_b="bearing_block_right",
        name="axle touches right bearing block",
    )
    ctx.expect_gap(
        wheel,
        support,
        axis="z",
        negative_elem="upper_base",
        min_gap=0.70,
        name="wheel sits high above the base",
    )
    ctx.check(
        "wheel articulation is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )

    rest_aabb = ctx.part_element_world_aabb(wheel, elem="paddle_00")
    with ctx.pose({spin: pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(wheel, elem="paddle_00")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_center = aabb_center(rest_aabb)
    turned_center = aabb_center(turned_aabb)
    ctx.check(
        "wheel rotation moves a paddle around the axle",
        rest_center is not None
        and turned_center is not None
        and abs(rest_center[0] - turned_center[0]) > 0.20
        and abs(rest_center[2] - turned_center[2]) > 0.20,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
