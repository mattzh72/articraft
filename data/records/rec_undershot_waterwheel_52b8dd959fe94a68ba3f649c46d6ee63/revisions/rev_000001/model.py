from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_disc(outer_radius: float, inner_radius: float, width: float) -> cq.Workplane:
    """A simple wheel hoop whose axis is the local Y axis."""
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(width, both=True)
    )


def _bearing_block() -> cq.Workplane:
    """Split-stone/iron pillow block with a real through-bore for the axle."""
    block = cq.Workplane("XY").box(0.38, 0.20, 0.30)
    bore = cq.Workplane("XZ").circle(0.098).extrude(0.26, both=True)
    return block.cut(bore)


def _add_xz_beam(
    part,
    *,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    section: tuple[float, float] = (0.12, 0.12),
    material=None,
) -> None:
    """Add a rectangular timber between two points in a constant-Y side frame."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return
    yaw = math.atan2(dy, dx)
    horizontal = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(-dz, horizontal)
    part.visual(
        Box((length, section[0], section[1])),
        origin=Origin(
            xyz=(
                (start[0] + end[0]) * 0.5,
                (start[1] + end[1]) * 0.5,
                (start[2] + end[2]) * 0.5,
            ),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    aged_oak = model.material("aged_oak", rgba=(0.47, 0.27, 0.12, 1.0))
    dark_oak = model.material("dark_wet_oak", rgba=(0.28, 0.16, 0.075, 1.0))
    end_grain = model.material("wood_end_grain", rgba=(0.58, 0.37, 0.17, 1.0))
    iron = model.material("blackened_iron", rgba=(0.05, 0.048, 0.044, 1.0))
    worn_iron = model.material("worn_iron_edges", rgba=(0.20, 0.19, 0.17, 1.0))
    stone = model.material("fieldstone", rgba=(0.36, 0.35, 0.32, 1.0))
    water = model.material("moving_water", rgba=(0.15, 0.42, 0.70, 0.58))
    foam = model.material("foam", rgba=(0.85, 0.93, 0.95, 0.75))

    frame = model.part("frame")

    # Low stone race and sill timbers: the undershot wheel is driven by water
    # passing under the wheel rather than falling from above.
    frame.visual(
        Box((3.70, 1.08, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=stone,
        name="race_bed",
    )
    frame.visual(
        Box((3.72, 0.080, 0.20)),
        origin=Origin(xyz=(0.0, -0.50, 0.125)),
        material=stone,
        name="race_wall_0",
    )
    frame.visual(
        Box((3.72, 0.080, 0.20)),
        origin=Origin(xyz=(0.0, 0.50, 0.125)),
        material=stone,
        name="race_wall_1",
    )
    frame.visual(
        Box((3.60, 0.78, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=water,
        name="water_race",
    )
    for i, x in enumerate((-1.35, -0.95, 0.95, 1.35)):
        frame.visual(
            Box((0.20, 0.74, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.056), rpy=(0.0, 0.0, 0.08 if x < 0 else -0.08)),
            material=foam,
            name=f"water_crest_{i}",
        )

    # Heavy wooden base sills outside the raceway, tied together by crossbeams.
    for i, y in enumerate((-0.76, 0.76)):
        frame.visual(
            Box((1.85, 0.14, 0.14)),
            origin=Origin(xyz=(0.0, y, 0.115)),
            material=dark_oak,
            name=f"side_sill_{i}",
        )
    for i, x in enumerate((-0.86, 0.86)):
        frame.visual(
            Box((0.14, 1.66, 0.12)),
            origin=Origin(xyz=(x, 0.0, 0.135)),
            material=dark_oak,
            name=f"cross_sill_{i}",
        )

    axle_z = 1.14
    for side, y in enumerate((-0.76, 0.76)):
        _add_xz_beam(
            frame,
            name=f"raker_{side}_0",
            start=(-0.78, y, 0.18),
            end=(-0.30, y, axle_z - 0.20),
            section=(0.13, 0.13),
            material=aged_oak,
        )
        _add_xz_beam(
            frame,
            name=f"raker_{side}_1",
            start=(0.78, y, 0.18),
            end=(0.30, y, axle_z - 0.20),
            section=(0.13, 0.13),
            material=aged_oak,
        )
        _add_xz_beam(
            frame,
            name=f"brace_{side}_0",
            start=(-0.62, y, 0.30),
            end=(0.58, y, 0.86),
            section=(0.075, 0.075),
            material=aged_oak,
        )
        _add_xz_beam(
            frame,
            name=f"brace_{side}_1",
            start=(0.62, y, 0.30),
            end=(-0.58, y, 0.86),
            section=(0.075, 0.075),
            material=aged_oak,
        )
        frame.visual(
            Box((0.56, 0.16, 0.12)),
            origin=Origin(xyz=(0.0, y, axle_z - 0.20)),
            material=dark_oak,
            name=f"bearing_plinth_{side}",
        )
        bearing_name = "bearing_0" if side == 0 else "bearing_1"
        frame.visual(
            mesh_from_cadquery(_bearing_block(), f"bearing_block_{side}", tolerance=0.001),
            origin=Origin(xyz=(0.0, y, axle_z)),
            material=worn_iron,
            name=bearing_name,
        )
        frame.visual(
            Box((0.46, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y - 0.086 if y > 0 else y + 0.086, axle_z + 0.1675)),
            material=iron,
            name=f"bearing_strap_{side}",
        )

    # A downstream walkway plank makes the machine read as a practical mill-site
    # mechanism, while also tying the two side frames visibly together.
    frame.visual(
        Box((0.42, 1.60, 0.055)),
        origin=Origin(xyz=(1.12, 0.0, 0.43)),
        material=aged_oak,
        name="service_plank",
    )
    frame.visual(
        Box((0.48, 1.60, 0.10)),
        origin=Origin(xyz=(1.12, 0.0, 0.375)),
        material=dark_oak,
        name="plank_support",
    )
    for i, y in enumerate((-0.76, 0.76)):
        frame.visual(
            Box((0.10, 0.12, 0.28)),
            origin=Origin(xyz=(0.90, y, 0.285)),
            material=dark_oak,
            name=f"plank_post_{i}",
        )

    wheel = model.part("wheel")

    # The rotating assembly is built around the axle frame: local Y is the
    # axle, and local X/Z form the wheel plane.
    wheel.visual(
        Cylinder(radius=0.075, length=1.72),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.155, length=0.78),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_oak,
        name="hub",
    )
    for i, y in enumerate((-0.21, 0.21)):
        wheel.visual(
            Cylinder(radius=0.172, length=0.055),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=f"hub_band_{i}",
        )
    for i, y in enumerate((-0.645, 0.645)):
        wheel.visual(
            Cylinder(radius=0.115, length=0.030),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=f"journal_collar_{i}",
        )

    side_y = (-0.32, 0.32)
    for i, y in enumerate(side_y):
        wheel.visual(
            mesh_from_cadquery(_annular_disc(0.865, 0.745, 0.060), f"wooden_rim_{i}"),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=aged_oak,
            name=f"rim_{i}",
        )
        wheel.visual(
            mesh_from_cadquery(_annular_disc(0.895, 0.867, 0.050), f"iron_tire_{i}"),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=iron,
            name=f"tire_{i}",
        )

    spoke_count = 12
    for side, y in enumerate(side_y):
        for i in range(spoke_count):
            angle = 2.0 * math.pi * i / spoke_count
            radial_mid = 0.47
            wheel.visual(
                Box((0.82, 0.070, 0.075)),
                origin=Origin(
                    xyz=(radial_mid * math.cos(angle), y, radial_mid * math.sin(angle)),
                    rpy=(0.0, -angle, 0.0),
                ),
                material=dark_oak,
                name=f"spoke_{side}_{i}",
            )

    paddle_count = 16
    for i in range(paddle_count):
        angle = 2.0 * math.pi * i / paddle_count
        radius = 0.925
        paddle_name = {0: "paddle_0", 12: "paddle_12"}.get(i, f"paddle_{i}")
        # Local Z points radially; the board face is broad across Y and radial
        # height, with its thin edge along the tangent.
        wheel.visual(
            Box((0.055, 0.76, 0.260)),
            origin=Origin(
                xyz=(radius * math.cos(angle), 0.0, radius * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0 - angle, 0.0),
            ),
            material=aged_oak,
            name=paddle_name,
        )
        wheel.visual(
            Box((0.030, 0.80, 0.045)),
            origin=Origin(
                xyz=((radius + 0.135) * math.cos(angle), 0.0, (radius + 0.135) * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0 - angle, 0.0),
            ),
            material=end_grain,
            name=f"paddle_edge_{i}",
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    # The axle projects through both pillow blocks while the bearings keep the
    # rotating assembly centered and visibly supported.
    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="bearing_0",
        margin=0.005,
        name="axle centered in first bearing",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="bearing_1",
        margin=0.005,
        name="axle centered in second bearing",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bearing_0",
        min_overlap=0.15,
        name="axle passes through first bearing",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bearing_1",
        min_overlap=0.15,
        name="axle passes through second bearing",
    )

    # At rest and after a quarter turn the low paddle remains in the water-race
    # footprint, close enough to read as water-driven, without treating the
    # water surface as solid collision geometry.
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem="paddle_12",
        negative_elem="water_race",
        min_gap=0.025,
        max_gap=0.060,
        name="rest paddle skims the water",
    )
    rest_aabb = ctx.part_element_world_aabb(wheel, elem="paddle_0")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(wheel, elem="paddle_0")
        ctx.expect_gap(
            wheel,
            frame,
            axis="z",
            positive_elem="paddle_0",
            negative_elem="water_race",
            min_gap=0.025,
            max_gap=0.060,
            name="turned paddle skims the water",
        )
        ctx.expect_within(
            wheel,
            frame,
            axes="xy",
            inner_elem="paddle_0",
            outer_elem="water_race",
            margin=0.010,
            name="turned paddle sits in race footprint",
        )
    ctx.check(
        "wheel quarter turn lowers leading paddle",
        rest_aabb is not None
        and turned_aabb is not None
        and turned_aabb[0][2] < rest_aabb[0][2] - 0.55,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
