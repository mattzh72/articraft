from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cylinder_between(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material: Material,
    *,
    overlap: float = 0.012,
) -> None:
    """Add a round steel member whose local cylinder axis runs from p0 to p1."""
    vx = p1[0] - p0[0]
    vy = p1[1] - p0[1]
    vz = p1[2] - p0[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        return

    # Extend slightly through the joint nodes so the skeletal part compiles as
    # one connected welded assembly instead of a collection of just-touching rods.
    length_with_overlap = length + 2.0 * overlap
    cx = (p0[0] + p1[0]) * 0.5
    cy = (p0[1] + p1[1]) * 0.5
    cz = (p0[2] + p1[2]) * 0.5
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.sqrt(vx * vx + vy * vy), vz)
    part.visual(
        Cylinder(radius=radius, length=length_with_overlap),
        origin=Origin(xyz=(cx, cy, cz), rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skeletal_steel_lighthouse")

    steel = model.material("galvanized_steel", rgba=(0.36, 0.39, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.48, 0.47, 0.43, 1.0))
    glass = model.material("pale_lantern_glass", rgba=(0.62, 0.82, 0.95, 0.34))
    roof_red = model.material("oxide_red_roof", rgba=(0.55, 0.10, 0.07, 1.0))
    brass = model.material("warm_brass", rgba=(0.93, 0.70, 0.28, 1.0))
    amber = model.material("glowing_amber_glass", rgba=(1.00, 0.67, 0.15, 0.72))

    tower = model.part("tower")

    # Grounded concrete plinth and steel base grill.
    tower.visual(
        Box((2.35, 2.35, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="concrete_pad",
    )
    tower.visual(
        Box((1.95, 1.95, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.2075)),
        material=dark_steel,
        name="base_grillage",
    )

    levels = [0.25, 1.20, 2.15, 3.10, 4.05, 4.58]

    def half_width(z: float) -> float:
        return 0.86 - (0.86 - 0.43) * (z - levels[0]) / (levels[-1] - levels[0])

    def corner(z: float, sx: int, sy: int) -> tuple[float, float, float]:
        h = half_width(z)
        return (sx * h, sy * h, z)

    for sx in (-1, 1):
        for sy in (-1, 1):
            p0 = corner(levels[0], sx, sy)
            p1 = corner(levels[-1], sx, sy)
            _cylinder_between(
                tower,
                f"tapered_leg_{sx}_{sy}",
                p0,
                p1,
                0.035,
                steel,
                overlap=0.025,
            )
            tower.visual(
                Box((0.22, 0.22, 0.030)),
                origin=Origin(xyz=(p0[0], p0[1], 0.245)),
                material=dark_steel,
                name=f"foot_plate_{sx}_{sy}",
            )

    # Horizontal rings and X-bracing on each tower face.
    for i, z in enumerate(levels):
        pts = {
            "++": corner(z, 1, 1),
            "-+": corner(z, -1, 1),
            "--": corner(z, -1, -1),
            "+-": corner(z, 1, -1),
        }
        _cylinder_between(tower, f"front_ring_{i}", pts["-+"], pts["++"], 0.018, steel)
        _cylinder_between(tower, f"rear_ring_{i}", pts["--"], pts["+-"], 0.018, steel)
        _cylinder_between(tower, f"side_ring_a_{i}", pts["++"], pts["+-"], 0.018, steel)
        _cylinder_between(tower, f"side_ring_b_{i}", pts["-+"], pts["--"], 0.018, steel)

    for i in range(len(levels) - 1):
        z0 = levels[i]
        z1 = levels[i + 1]
        for face, endpoints in (
            ("front", ((-1, 1), (1, 1), (1, 1), (-1, 1))),
            ("rear", ((-1, -1), (1, -1), (1, -1), (-1, -1))),
            ("side_a", ((1, -1), (1, 1), (1, 1), (1, -1))),
            ("side_b", ((-1, -1), (-1, 1), (-1, 1), (-1, -1))),
        ):
            a0 = corner(z0, endpoints[0][0], endpoints[0][1])
            b1 = corner(z1, endpoints[1][0], endpoints[1][1])
            b0 = corner(z0, endpoints[2][0], endpoints[2][1])
            a1 = corner(z1, endpoints[3][0], endpoints[3][1])
            _cylinder_between(tower, f"{face}_brace_a_{i}", a0, b1, 0.014, steel)
            _cylinder_between(tower, f"{face}_brace_b_{i}", b0, a1, 0.014, steel)

    # Compact square lantern room with glass wall panels and roof.
    tower.visual(
        Box((1.10, 1.10, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 4.60)),
        material=dark_steel,
        name="lantern_floor",
    )
    for sx in (-1, 1):
        for sy in (-1, 1):
            _cylinder_between(
                tower,
                f"lantern_post_{sx}_{sy}",
                (sx * 0.48, sy * 0.48, 4.64),
                (sx * 0.48, sy * 0.48, 5.30),
                0.022,
                steel,
                overlap=0.010,
            )
    for z, label in ((4.66, "lower"), (5.30, "upper")):
        _cylinder_between(tower, f"{label}_front_lantern_rail", (-0.50, 0.50, z), (0.50, 0.50, z), 0.020, steel)
        _cylinder_between(tower, f"{label}_rear_lantern_rail", (-0.50, -0.50, z), (0.50, -0.50, z), 0.020, steel)
        _cylinder_between(tower, f"{label}_side_lantern_rail_0", (0.50, -0.50, z), (0.50, 0.50, z), 0.020, steel)
        _cylinder_between(tower, f"{label}_side_lantern_rail_1", (-0.50, -0.50, z), (-0.50, 0.50, z), 0.020, steel)

    tower.visual(
        Box((0.96, 0.018, 0.64)),
        origin=Origin(xyz=(0.0, -0.505, 4.98)),
        material=glass,
        name="rear_glass_panel",
    )
    tower.visual(
        Box((0.018, 0.96, 0.64)),
        origin=Origin(xyz=(0.505, 0.0, 4.98)),
        material=glass,
        name="side_glass_panel_0",
    )
    tower.visual(
        Box((0.018, 0.96, 0.64)),
        origin=Origin(xyz=(-0.505, 0.0, 4.98)),
        material=glass,
        name="side_glass_panel_1",
    )
    tower.visual(
        Box((0.32, 0.018, 0.46)),
        origin=Origin(xyz=(0.31, 0.505, 4.96)),
        material=glass,
        name="front_glass_panel",
    )

    # Door/hatch jamb in the front lantern wall.
    tower.visual(
        Cylinder(radius=0.016, length=0.70),
        origin=Origin(xyz=(-0.18, 0.50, 4.97)),
        material=steel,
        name="hatch_jamb_hinge",
    )
    _cylinder_between(tower, "hatch_jamb_latch", (0.11, 0.50, 4.64), (0.11, 0.50, 5.30), 0.016, steel)
    _cylinder_between(tower, "hatch_jamb_sill", (-0.19, 0.50, 4.73), (0.12, 0.50, 4.73), 0.014, steel)
    _cylinder_between(tower, "hatch_jamb_head", (-0.19, 0.50, 5.12), (0.12, 0.50, 5.12), 0.014, steel)

    roof_mesh = mesh_from_geometry(ConeGeometry(0.68, 0.30, radial_segments=32), "conical_lantern_roof")
    tower.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, 5.43)),
        material=roof_red,
        name="conical_roof",
    )

    # Fixed vertical spindle and keeper collars that capture the rotating beacon.
    tower.visual(
        Cylinder(radius=0.026, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 5.03)),
        material=dark_steel,
        name="spindle",
    )
    tower.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 5.005)),
        material=dark_steel,
        name="lower_keeper",
    )
    tower.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 5.055)),
        material=dark_steel,
        name="upper_keeper",
    )

    beacon = model.part("beacon")
    clip_mesh = mesh_from_geometry(TorusGeometry(0.037, 0.012, radial_segments=20, tubular_segments=40), "beacon_spindle_clip")
    beacon.visual(
        clip_mesh,
        origin=Origin(),
        material=brass,
        name="spindle_clip",
    )
    hub_mesh = mesh_from_geometry(TorusGeometry(0.057, 0.008, radial_segments=20, tubular_segments=40), "beacon_central_hub")
    beacon.visual(hub_mesh, origin=Origin(), material=brass, name="central_hub")
    beacon.visual(
        Box((0.145, 0.036, 0.024)),
        origin=Origin(xyz=(-0.125, 0.0, 0.0)),
        material=brass,
        name="negative_lens_arm",
    )
    beacon.visual(
        Box((0.030, 0.210, 0.145)),
        origin=Origin(xyz=(-0.212, 0.0, 0.0)),
        material=amber,
        name="negative_fresnel_lens",
    )
    beacon.visual(
        Box((0.014, 0.235, 0.020)),
        origin=Origin(xyz=(-0.212, 0.0, 0.082)),
        material=brass,
        name="negative_lens_cap",
    )
    beacon.visual(
        Box((0.014, 0.235, 0.020)),
        origin=Origin(xyz=(-0.212, 0.0, -0.082)),
        material=brass,
        name="negative_lens_base",
    )
    beacon.visual(
        Box((0.145, 0.036, 0.024)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=brass,
        name="positive_lens_arm",
    )
    beacon.visual(
        Box((0.030, 0.210, 0.145)),
        origin=Origin(xyz=(0.212, 0.0, 0.0)),
        material=amber,
        name="positive_fresnel_lens",
    )
    beacon.visual(
        Box((0.014, 0.235, 0.020)),
        origin=Origin(xyz=(0.212, 0.0, 0.082)),
        material=brass,
        name="positive_lens_cap",
    )
    beacon.visual(
        Box((0.014, 0.235, 0.020)),
        origin=Origin(xyz=(0.212, 0.0, -0.082)),
        material=brass,
        name="positive_lens_base",
    )
    lamp_mesh = mesh_from_geometry(TorusGeometry(0.046, 0.006, radial_segments=18, tubular_segments=36), "hollow_lamp_core")
    beacon.visual(lamp_mesh, origin=Origin(), material=amber, name="lamp_core")

    model.articulation(
        "tower_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 5.03)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.6),
    )

    hatch = model.part("hatch")
    hatch.visual(
        Box((0.235, 0.024, 0.36)),
        origin=Origin(xyz=(0.1475, 0.022, 0.0)),
        material=dark_steel,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.180, 0.010, 0.28)),
        origin=Origin(xyz=(0.165, 0.038, 0.0)),
        material=glass,
        name="hatch_window",
    )
    for z, strap_name in ((0.12, "upper_hinge_leaf"), (-0.12, "lower_hinge_leaf")):
        hatch.visual(
            Box((0.050, 0.010, 0.055)),
            origin=Origin(xyz=(0.042, 0.028, z)),
            material=steel,
            name=strap_name,
        )
    hatch.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    hatch.visual(
        Cylinder(radius=0.022, length=0.036),
        origin=Origin(xyz=(0.235, 0.057, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="round_pull",
    )
    model.articulation(
        "tower_to_hatch",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hatch,
        origin=Origin(xyz=(-0.18, 0.468, 4.93)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    hatch = object_model.get_part("hatch")
    beacon_spin = object_model.get_articulation("tower_to_beacon")
    hatch_hinge = object_model.get_articulation("tower_to_hatch")

    ctx.allow_overlap(
        beacon,
        tower,
        elem_a="spindle_clip",
        elem_b="spindle",
        reason="The rotating beacon clip is a close bearing around the fixed vertical spindle.",
    )
    ctx.allow_overlap(
        hatch,
        tower,
        elem_a="hinge_barrel",
        elem_b="hatch_jamb_hinge",
        reason="The hatch hinge barrel is modeled as captured around the fixed side hinge pin.",
    )

    ctx.expect_overlap(
        beacon,
        tower,
        axes="z",
        elem_a="spindle_clip",
        elem_b="spindle",
        min_overlap=0.015,
        name="beacon clip surrounds the vertical spindle height",
    )
    ctx.expect_gap(
        tower,
        beacon,
        axis="z",
        positive_elem="upper_keeper",
        negative_elem="spindle_clip",
        min_gap=0.002,
        max_gap=0.025,
        name="upper keeper sits just above beacon clip",
    )
    ctx.expect_gap(
        beacon,
        tower,
        axis="z",
        positive_elem="spindle_clip",
        negative_elem="lower_keeper",
        min_gap=0.002,
        max_gap=0.025,
        name="beacon clip sits just above lower keeper",
    )
    ctx.expect_overlap(
        hatch,
        tower,
        axes="z",
        elem_a="hinge_barrel",
        elem_b="hatch_jamb_hinge",
        min_overlap=0.30,
        name="hatch barrel shares the side hinge pin height",
    )
    ctx.expect_within(
        beacon,
        tower,
        axes="xy",
        inner_elem="positive_fresnel_lens",
        outer_elem="lantern_floor",
        margin=0.005,
        name="rotating beacon remains inside lantern footprint",
    )

    closed_hatch = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({beacon_spin: math.pi / 2.0, hatch_hinge: 1.20}):
        ctx.expect_within(
            beacon,
            tower,
            axes="xy",
            inner_elem="positive_fresnel_lens",
            outer_elem="lantern_floor",
            margin=0.005,
            name="quarter-turn beacon remains captured in cage",
        )
        opened_hatch = ctx.part_element_world_aabb(hatch, elem="hatch_panel")

    ctx.check(
        "hatch swings outward from side hinge",
        closed_hatch is not None
        and opened_hatch is not None
        and opened_hatch[1][1] > closed_hatch[1][1] + 0.10,
        details=f"closed={closed_hatch}, opened={opened_hatch}",
    )

    return ctx.report()


object_model = build_object_model()
