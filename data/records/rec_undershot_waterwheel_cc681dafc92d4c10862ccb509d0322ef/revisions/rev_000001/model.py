from __future__ import annotations

from math import atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cylinder_along_y() -> tuple[float, float, float]:
    """Rotate a default Z-axis cylinder so its axis lies along world Y."""

    return (pi / 2.0, 0.0, 0.0)


def _add_member(part, start, end, width, depth, *, material, name):
    """Add a rectangular strut between two XZ points at a fixed Y coordinate."""

    x0, y, z0 = start
    x1, _, z1 = end
    dx = x1 - x0
    dz = z1 - z0
    length = sqrt(dx * dx + dz * dz)
    angle = -atan2(dz, dx)
    part.visual(
        Box((length, width, depth)),
        origin=Origin(
            xyz=((x0 + x1) / 2.0, y, (z0 + z1) / 2.0),
            rpy=(0.0, angle, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_undershot_waterwheel")

    warm_wood = Material("sealed_bamboo", rgba=(0.72, 0.46, 0.22, 1.0))
    dark_wood = Material("dark_endgrain", rgba=(0.38, 0.22, 0.10, 1.0))
    steel = Material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    graphite = Material("graphite_bearings", rgba=(0.05, 0.055, 0.06, 1.0))
    acrylic = Material("smoked_clear_acrylic", rgba=(0.45, 0.68, 0.72, 0.38))
    water = Material("shallow_water", rgba=(0.10, 0.45, 0.85, 0.42))
    rubber = Material("soft_feet", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.300, 0.180, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=acrylic,
        name="base_plate",
    )
    base.visual(
        Box((0.250, 0.086, 0.004)),
        origin=Origin(xyz=(-0.005, 0.0, 0.016)),
        material=water,
        name="water_strip",
    )
    for y, name in ((0.086, "side_lip_0"), (-0.086, "side_lip_1")):
        base.visual(
            Box((0.300, 0.008, 0.022)),
            origin=Origin(xyz=(0.0, y, 0.025)),
            material=acrylic,
            name=name,
        )
    for x, name in ((0.146, "end_lip_0"), (-0.146, "end_lip_1")):
        base.visual(
            Box((0.008, 0.180, 0.022)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=acrylic,
            name=name,
        )
    for x, y, name in (
        (-0.096, 0.064, "foot_pad_0"),
        (0.096, 0.064, "foot_pad_1"),
        (-0.096, -0.064, "foot_pad_2"),
        (0.096, -0.064, "foot_pad_3"),
    ):
        base.visual(
            Box((0.038, 0.022, 0.008)),
            origin=Origin(xyz=(x, y, -0.004)),
            material=rubber,
            name=name,
        )
    base.visual(
        Cylinder(0.004, 0.124),
        origin=Origin(xyz=(-0.158, 0.0, 0.031), rpy=_cylinder_along_y()),
        material=steel,
        name="chute_pin",
    )
    for y, name in ((0.058, "chute_tab_0"), (-0.058, "chute_tab_1")):
        base.visual(
            Box((0.020, 0.012, 0.018)),
            origin=Origin(xyz=(-0.151, y, 0.024)),
            material=steel,
            name=name,
        )

    side_frame = model.part("side_frame")
    # Two narrow cheek frames sit outside the wheel envelope and are tied by
    # crossbars, so the bearings read as a single supported desktop stand.
    for y, suffix in ((0.064, "0"), (-0.064, "1")):
        side_frame.visual(
            Box((0.220, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.041)),
            material=warm_wood,
            name=f"bottom_rail_{suffix}",
        )
        for x, foot_name in ((-0.096, "front_mount"), (0.096, "rear_mount")):
            side_frame.visual(
                Box((0.024, 0.012, 0.027)),
                origin=Origin(xyz=(x, y, 0.0275)),
                material=warm_wood,
                name=f"{foot_name}_{suffix}",
            )
        _add_member(
            side_frame,
            (-0.096, y, 0.041),
            (-0.018, y, 0.108),
            0.012,
            0.010,
            material=warm_wood,
            name=f"front_strut_{suffix}",
        )
        _add_member(
            side_frame,
            (0.096, y, 0.041),
            (0.018, y, 0.108),
            0.012,
            0.010,
            material=warm_wood,
            name=f"rear_strut_{suffix}",
        )
        for x, block_name in ((-0.024, "front_bearing_block"), (0.024, "rear_bearing_block")):
            side_frame.visual(
                Box((0.028, 0.012, 0.024)),
                origin=Origin(xyz=(x, y, 0.110)),
                material=warm_wood,
                name=f"{block_name}_{suffix}",
            )
        side_frame.visual(
            Cylinder(0.014, 0.020),
            origin=Origin(xyz=(0.0, y * 0.90, 0.118), rpy=_cylinder_along_y()),
            material=graphite,
            name=f"bearing_{suffix}",
        )
        side_frame.visual(
            Box((0.010, 0.010, 0.074)),
            origin=Origin(xyz=(0.0, y, 0.165)),
            material=warm_wood,
            name=f"guard_post_{suffix}",
        )
    for x, z, name in (
        (-0.096, 0.041, "front_crossbar"),
        (0.096, 0.041, "rear_crossbar"),
        (0.0, 0.205, "top_guard"),
    ):
        side_frame.visual(
            Box((0.018, 0.140, 0.010)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=warm_wood,
            name=name,
        )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(0.006, 0.150),
        origin=Origin(rpy=_cylinder_along_y()),
        material=steel,
        name="axle",
    )
    wheel.visual(
        Cylinder(0.018, 0.074),
        origin=Origin(rpy=_cylinder_along_y()),
        material=dark_wood,
        name="hub",
    )
    rim_mesh = mesh_from_geometry(TorusGeometry(0.073, 0.004, radial_segments=16, tubular_segments=48), "rim")
    for y, name in ((0.040, "rim_0"), (-0.040, "rim_1")):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=_cylinder_along_y()),
            material=dark_wood,
            name=name,
        )
    for i in range(8):
        theta = i * 2.0 * pi / 8.0
        mid_r = 0.046
        wheel.visual(
            Box((0.060, 0.082, 0.005)),
            origin=Origin(
                xyz=(mid_r * cos(theta), 0.0, mid_r * sin(theta)),
                rpy=(0.0, -theta, 0.0),
            ),
            material=warm_wood,
            name=f"spoke_{i}",
        )
    for i in range(12):
        theta = i * 2.0 * pi / 12.0
        radius = 0.080
        wheel.visual(
            Box((0.024, 0.088, 0.010)),
            origin=Origin(
                xyz=(radius * cos(theta), 0.0, radius * sin(theta)),
                rpy=(0.0, pi / 2.0 - theta, 0.0),
            ),
            material=warm_wood,
            name=f"paddle_{i}",
        )

    inlet_chute = model.part("inlet_chute")
    inlet_chute.visual(
        Cylinder(0.006, 0.106),
        origin=Origin(rpy=_cylinder_along_y()),
        material=steel,
        name="chute_knuckle",
    )
    inlet_chute.visual(
        Box((0.070, 0.100, 0.006)),
        origin=Origin(xyz=(-0.035, 0.0, -0.002)),
        material=acrylic,
        name="chute_panel",
    )
    for y, name in ((0.053, "chute_lip_0"), (-0.053, "chute_lip_1")):
        inlet_chute.visual(
            Box((0.070, 0.006, 0.014)),
            origin=Origin(xyz=(-0.035, y, 0.003)),
            material=acrylic,
            name=name,
        )

    model.articulation(
        "base_to_frame",
        ArticulationType.FIXED,
        parent=base,
        child=side_frame,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=side_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=18.0),
    )
    model.articulation(
        "base_to_chute",
        ArticulationType.REVOLUTE,
        parent=base,
        child=inlet_chute,
        origin=Origin(xyz=(-0.158, 0.0, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=2.0, lower=0.0, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    side_frame = object_model.get_part("side_frame")
    wheel = object_model.get_part("wheel")
    inlet_chute = object_model.get_part("inlet_chute")
    spin = object_model.get_articulation("frame_to_wheel")
    fold = object_model.get_articulation("base_to_chute")

    for bearing in ("bearing_0", "bearing_1"):
        ctx.allow_overlap(
            side_frame,
            wheel,
            elem_a=bearing,
            elem_b="axle",
            reason="The centered steel axle is intentionally captured inside the side bearing collar.",
        )
        ctx.expect_within(
            wheel,
            side_frame,
            axes="xz",
            inner_elem="axle",
            outer_elem=bearing,
            margin=0.001,
            name=f"axle centered in {bearing}",
        )
        ctx.expect_overlap(
            wheel,
            side_frame,
            axes="y",
            elem_a="axle",
            elem_b=bearing,
            min_overlap=0.010,
            name=f"axle retained by {bearing}",
        )

    ctx.allow_overlap(
        base,
        inlet_chute,
        elem_a="chute_pin",
        elem_b="chute_knuckle",
        reason="The folding inlet chute sleeve rotates around the base hinge pin.",
    )
    ctx.expect_within(
        inlet_chute,
        base,
        axes="yz",
        inner_elem="chute_knuckle",
        outer_elem="chute_pin",
        margin=0.003,
        name="chute hinge concentric on pin",
    )
    ctx.expect_gap(
        wheel,
        base,
        axis="z",
        min_gap=0.008,
        negative_elem="base_plate",
        name="paddle wheel clears desktop base",
    )
    ctx.expect_gap(
        side_frame,
        wheel,
        axis="y",
        min_gap=0.003,
        positive_elem="bearing_0",
        negative_elem="paddle_0",
        name="paddles clear side bearing",
    )

    rest_chute_aabb = ctx.part_world_aabb(inlet_chute)
    with ctx.pose({spin: pi / 2.0}):
        ctx.expect_gap(
            wheel,
            base,
            axis="z",
            min_gap=0.008,
            negative_elem="base_plate",
            name="rotated paddles keep base clearance",
        )
    with ctx.pose({fold: 1.10}):
        stowed_chute_aabb = ctx.part_world_aabb(inlet_chute)

    ctx.check(
        "folding inlet reduces footprint",
        rest_chute_aabb is not None
        and stowed_chute_aabb is not None
        and stowed_chute_aabb[0][0] > rest_chute_aabb[0][0] + 0.010,
        details=f"rest={rest_chute_aabb}, stowed={stowed_chute_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
