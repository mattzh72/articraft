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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    stone = model.material("stone", rgba=(0.55, 0.56, 0.53, 1.0))
    timber = model.material("timber", rgba=(0.46, 0.31, 0.18, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.28, 0.18, 0.11, 1.0))
    wet_wood = model.material("wet_wood", rgba=(0.39, 0.25, 0.14, 1.0))
    iron = model.material("iron", rgba=(0.29, 0.30, 0.32, 1.0))

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((1.82, 1.02, 1.44)),
        mass=1250.0,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
    )
    support_frame.visual(
        Box((1.82, 1.02, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=stone,
        name="foundation_bed",
    )
    support_frame.visual(
        Box((1.70, 0.22, 0.12)),
        origin=Origin(xyz=(0.0, 0.34, 0.16)),
        material=dark_timber,
        name="front_sill",
    )
    support_frame.visual(
        Box((1.70, 0.22, 0.12)),
        origin=Origin(xyz=(0.0, -0.34, 0.16)),
        material=dark_timber,
        name="rear_sill",
    )
    support_frame.visual(
        Box((1.54, 0.38, 0.10)),
        origin=Origin(xyz=(0.0, 0.10, 0.19)),
        material=stone,
        name="stream_bed",
    )

    for side_name, side_x in (("left", -0.72), ("right", 0.72)):
        bearing_name = "left_bearing" if side_name == "left" else "right_bearing"
        support_frame.visual(
            Box((0.22, 0.34, 1.06)),
            origin=Origin(xyz=(side_x, 0.0, 0.67)),
            material=timber,
            name=f"{side_name}_upright",
        )
        support_frame.visual(
            Box((0.22, 0.18, 0.74)),
            origin=Origin(xyz=(side_x, 0.22, 0.47), rpy=(-0.58, 0.0, 0.0)),
            material=dark_timber,
            name=f"{side_name}_front_brace",
        )
        support_frame.visual(
            Box((0.22, 0.18, 0.74)),
            origin=Origin(xyz=(side_x, -0.22, 0.47), rpy=(0.58, 0.0, 0.0)),
            material=dark_timber,
            name=f"{side_name}_rear_brace",
        )
        support_frame.visual(
            Box((0.32, 0.40, 0.28)),
            origin=Origin(xyz=(side_x, 0.0, 1.08)),
            material=dark_timber,
            name=bearing_name,
        )

    support_frame.visual(
        Box((0.32, 0.18, 0.18)),
        origin=Origin(xyz=(-0.72, -0.20, 1.31)),
        material=dark_timber,
        name="left_rear_head",
    )
    support_frame.visual(
        Box((0.32, 0.18, 0.18)),
        origin=Origin(xyz=(0.72, -0.20, 1.31)),
        material=dark_timber,
        name="right_rear_head",
    )
    support_frame.visual(
        Box((1.54, 0.26, 0.08)),
        origin=Origin(xyz=(0.0, 0.88, 1.55)),
        material=timber,
        name="trough_floor",
    )
    support_frame.visual(
        Box((1.54, 0.08, 0.20)),
        origin=Origin(xyz=(0.0, 0.78, 1.62)),
        material=timber,
        name="trough_lip",
    )
    support_frame.visual(
        Box((1.54, 0.08, 0.26)),
        origin=Origin(xyz=(0.0, 0.99, 1.68)),
        material=timber,
        name="trough_back_wall",
    )
    support_frame.visual(
        Box((0.10, 0.84, 0.72)),
        origin=Origin(xyz=(-0.72, 0.60, 1.48)),
        material=timber,
        name="left_trough_cheek",
    )
    support_frame.visual(
        Box((0.10, 0.84, 0.72)),
        origin=Origin(xyz=(0.72, 0.60, 1.48)),
        material=timber,
        name="right_trough_cheek",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.72, length=0.46),
        mass=220.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    wheel.visual(
        Cylinder(radius=0.065, length=1.12),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.16, length=0.42),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_timber,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.21, length=0.06),
        origin=Origin(xyz=(-0.10, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_timber,
        name="left_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=0.21, length=0.06),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_timber,
        name="right_hub_flange",
    )

    rim_mesh = _save_mesh(
        "waterwheel_rim",
        TorusGeometry(radius=0.64, tube=0.035, radial_segments=18, tubular_segments=72).rotate_y(pi / 2.0),
    )
    wheel.visual(rim_mesh, origin=Origin(xyz=(-0.19, 0.0, 0.0)), material=dark_timber, name="left_rim")
    wheel.visual(rim_mesh, origin=Origin(xyz=(0.19, 0.0, 0.0)), material=dark_timber, name="right_rim")

    for side_name, side_x in (("left", -0.19), ("right", 0.19)):
        for index in range(8):
            angle = 2.0 * pi * index / 8.0
            wheel.visual(
                Box((0.032, 0.055, 0.50)),
                origin=Origin(
                    xyz=(side_x, 0.385 * sin(angle), 0.385 * cos(angle)),
                    rpy=(-angle, 0.0, 0.0),
                ),
                material=dark_timber,
                name=f"{side_name}_spoke_{index:02d}",
            )

    for index in range(12):
        angle = 2.0 * pi * index / 12.0
        wheel.visual(
            Box((0.46, 0.06, 0.22)),
            origin=Origin(
                xyz=(0.0, 0.57 * sin(angle), 0.57 * cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=wet_wood,
            name=f"paddle_{index:02d}",
        )

    model.articulation(
        "axle_spin",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.05, 1.00)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=3.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    axle_spin = object_model.get_articulation("axle_spin")

    ctx.expect_gap(
        support_frame,
        wheel,
        axis="x",
        positive_elem="right_bearing",
        negative_elem="axle",
        max_gap=0.003,
        max_penetration=0.001,
        name="right bearing housing meets the axle journal",
    )
    ctx.expect_gap(
        wheel,
        support_frame,
        axis="x",
        positive_elem="axle",
        negative_elem="left_bearing",
        max_gap=0.003,
        max_penetration=0.001,
        name="left bearing housing meets the axle journal",
    )
    ctx.expect_gap(
        support_frame,
        wheel,
        axis="z",
        positive_elem="trough_lip",
        negative_elem="axle",
        min_gap=0.35,
        max_gap=0.65,
        name="the axle sits below the trough lip",
    )

    rest_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem="paddle_00"))
    with ctx.pose({axle_spin: pi / 2.0}):
        turned_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem="paddle_00"))

    ctx.check(
        "continuous joint rotates the paddle wheel around the axle",
        rest_center is not None
        and turned_center is not None
        and turned_center[1] < rest_center[1] - 0.45
        and abs(turned_center[2] - 1.00) < 0.15,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
