from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOWER_HEIGHT = 2.35
CAP_PIVOT_Z = TOWER_HEIGHT
HUB_LOCAL_X = 0.455
HUB_LOCAL_Z = 0.240


def _tapered_tower() -> cq.Workplane:
    """Octagonal tapered masonry tower in meters."""
    return (
        cq.Workplane("XY")
        .polygon(8, 1.08)
        .workplane(offset=TOWER_HEIGHT)
        .polygon(8, 0.58)
        .loft(combine=True)
    )


def _bar_between_yz(
    part,
    *,
    x: float,
    start_yz: tuple[float, float],
    end_yz: tuple[float, float],
    depth: float,
    thickness: float,
    material,
    name: str,
) -> None:
    """Add a rectangular bar whose long axis runs between two points in a local YZ plane."""
    y0, z0 = start_yz
    y1, z1 = end_yz
    dy = y1 - y0
    dz = z1 - z0
    length = math.hypot(dy, dz)
    if length <= 0.0:
        return
    # A box with its length along local +Z.  Roll rotates that local +Z into
    # the desired direction in the blade plane.
    phi = math.atan2(dy, dz)
    part.visual(
        Box((depth, thickness, length)),
        origin=Origin(xyz=(x, 0.5 * (y0 + y1), 0.5 * (z0 + z1)), rpy=(-phi, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _blade_point(phi: float, radius: float, tangent: float = 0.0) -> tuple[float, float]:
    """Point in the rotor local YZ plane using radial/tangential blade coordinates."""
    return (
        math.sin(phi) * radius + math.cos(phi) * tangent,
        math.cos(phi) * radius - math.sin(phi) * tangent,
    )


def _add_lattice_blade(part, index: int, phi: float, *, material) -> None:
    """One traditional sail: central spar, side rails, cross battens, and diagonals."""
    x = 0.040
    # Hub-to-tip spar.  It intentionally overlaps the hub, making the sail
    # visibly bolted to the rotating stage rather than floating beside it.
    _bar_between_yz(
        part,
        x=x,
        start_yz=_blade_point(phi, 0.085, 0.0),
        end_yz=_blade_point(phi, 1.040, 0.0),
        depth=0.034,
        thickness=0.026,
        material=material,
        name=f"blade_{index}_spar",
    )

    for side, tangent in (("a", -0.090), ("b", 0.090)):
        _bar_between_yz(
            part,
            x=x,
            start_yz=_blade_point(phi, 0.230, tangent),
            end_yz=_blade_point(phi, 1.000, tangent),
            depth=0.026,
            thickness=0.018,
            material=material,
            name=f"blade_{index}_rail_{side}",
        )

    for batten_index, radius in enumerate((0.260, 0.430, 0.600, 0.770, 0.940)):
        half_width = 0.070 + 0.025 * (radius - 0.260) / (0.940 - 0.260)
        _bar_between_yz(
            part,
            x=x + 0.002,
            start_yz=_blade_point(phi, radius, -half_width),
            end_yz=_blade_point(phi, radius, half_width),
            depth=0.024,
            thickness=0.016,
            material=material,
            name=f"blade_{index}_batten_{batten_index}",
        )

    diagonal_specs = (
        (0.300, -0.082, 0.545, 0.082),
        (0.545, 0.082, 0.790, -0.094),
        (0.790, -0.094, 0.995, 0.100),
    )
    for diag_index, (r0, t0, r1, t1) in enumerate(diagonal_specs):
        _bar_between_yz(
            part,
            x=x + 0.004,
            start_yz=_blade_point(phi, r0, t0),
            end_yz=_blade_point(phi, r1, t1),
            depth=0.020,
            thickness=0.014,
            material=material,
            name=f"blade_{index}_brace_{diag_index}",
        )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    whitewash = model.material("whitewashed_masonry", rgba=(0.86, 0.82, 0.72, 1.0))
    dark_wood = model.material("weathered_dark_wood", rgba=(0.27, 0.16, 0.08, 1.0))
    warm_wood = model.material("warm_oak", rgba=(0.55, 0.34, 0.16, 1.0))
    roof_thatch = model.material("aged_thatch", rgba=(0.50, 0.36, 0.18, 1.0))
    iron = model.material("blackened_iron", rgba=(0.07, 0.07, 0.065, 1.0))
    glass = model.material("dark_glass", rgba=(0.04, 0.06, 0.08, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_tapered_tower(), "tapered_tower"),
        material=whitewash,
        name="tapered_tower",
    )
    tower.visual(
        Cylinder(radius=0.350, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT - 0.035)),
        material=dark_wood,
        name="cap_curb",
    )
    tower.visual(
        Cylinder(radius=0.215, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT - 0.0375)),
        material=iron,
        name="yaw_bearing_plate",
    )
    tower.visual(
        Box((0.040, 0.245, 0.500)),
        origin=Origin(xyz=(0.505, 0.0, 0.270)),
        material=dark_wood,
        name="arched_door",
    )
    for i, (z, y) in enumerate(((1.000, -0.165), (1.500, 0.160))):
        tower.visual(
            Box((0.030, 0.150, 0.190)),
            origin=Origin(xyz=(0.410 - 0.040 * i, y, z)),
            material=glass,
            name=f"small_window_{i}",
        )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.225, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=iron,
        name="turntable",
    )
    cap.visual(
        Box((0.700, 0.500, 0.350)),
        origin=Origin(xyz=(0.035, 0.0, 0.235)),
        material=warm_wood,
        name="cap_house",
    )
    cap.visual(
        Box((0.780, 0.330, 0.060)),
        origin=Origin(xyz=(0.005, 0.130, 0.435), rpy=(0.38, 0.0, 0.0)),
        material=roof_thatch,
        name="roof_panel_0",
    )
    cap.visual(
        Box((0.780, 0.330, 0.060)),
        origin=Origin(xyz=(0.005, -0.130, 0.435), rpy=(-0.38, 0.0, 0.0)),
        material=roof_thatch,
        name="roof_panel_1",
    )
    cap.visual(
        Box((0.100, 0.180, 0.180)),
        origin=Origin(xyz=(0.380, 0.0, HUB_LOCAL_Z)),
        material=dark_wood,
        name="front_boss",
    )
    cap.visual(
        Cylinder(radius=0.062, length=0.140),
        origin=Origin(xyz=(0.350, 0.0, HUB_LOCAL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="shaft_bearing",
    )

    sail_hub = model.part("sail_hub")
    sail_hub.visual(
        Cylinder(radius=0.115, length=0.100),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_wood,
        name="hub_drum",
    )
    sail_hub.visual(
        Cylinder(radius=0.145, length=0.035),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="front_strap",
    )
    sail_hub.visual(
        Sphere(radius=0.070),
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material=iron,
        name="nose_cap",
    )
    for blade_index in range(4):
        _add_lattice_blade(
            sail_hub,
            blade_index,
            blade_index * math.pi / 2.0,
            material=warm_wood,
        )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0),
    )
    model.articulation(
        "cap_to_sail_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=sail_hub,
        origin=Origin(xyz=(HUB_LOCAL_X, 0.0, HUB_LOCAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    sail_hub = object_model.get_part("sail_hub")
    yaw = object_model.get_articulation("tower_to_cap")
    spin = object_model.get_articulation("cap_to_sail_hub")

    ctx.check("tower_cap_hub_parts_present", all((tower, cap, sail_hub)), "Expected tower, cap, and sail_hub parts.")
    ctx.check("cap_yaws_on_vertical_axis", yaw is not None and tuple(yaw.axis) == (0.0, 0.0, 1.0), f"axis={None if yaw is None else yaw.axis!r}")
    ctx.check("hub_spins_on_shaft_axis", spin is not None and tuple(spin.axis) == (1.0, 0.0, 0.0), f"axis={None if spin is None else spin.axis!r}")

    if cap is not None and sail_hub is not None:
        ctx.expect_gap(
            sail_hub,
            cap,
            axis="x",
            min_gap=0.0,
            max_gap=0.030,
            positive_elem="hub_drum",
            negative_elem="shaft_bearing",
            name="rotating hub is compact and close to the bearing",
        )
        ctx.expect_overlap(
            sail_hub,
            cap,
            axes="yz",
            min_overlap=0.090,
            elem_a="hub_drum",
            elem_b="shaft_bearing",
            name="hub aligns with the cap shaft bearing",
        )

    if yaw is not None and sail_hub is not None:
        rest_pos = ctx.part_world_position(sail_hub)
        with ctx.pose({yaw: math.pi / 2.0}):
            yawed_pos = ctx.part_world_position(sail_hub)
        ctx.check(
            "cap rotation carries hub around tower",
            rest_pos is not None
            and yawed_pos is not None
            and rest_pos[0] > 0.40
            and abs(rest_pos[1]) < 0.030
            and abs(yawed_pos[0]) < 0.040
            and yawed_pos[1] > 0.40,
            details=f"rest={rest_pos}, yawed={yawed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
