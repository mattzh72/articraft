from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _frustum(bottom_radius: float, top_radius: float, height: float, z0: float = 0.0):
    """CadQuery frustum with its bottom face on z0."""

    return (
        cq.Workplane("XY")
        .circle(bottom_radius)
        .workplane(offset=height)
        .circle(top_radius)
        .loft(combine=True)
        .translate((0.0, 0.0, z0))
    )


def _rotating_bar_origin(
    angle: float,
    radius: float,
    *,
    x: float,
    offset: float = 0.0,
    extra_angle: float = 0.0,
) -> Origin:
    """Place a box's long local +Y axis in the hub's YZ sail plane."""

    y = radius * math.cos(angle) - offset * math.sin(angle)
    z = radius * math.sin(angle) + offset * math.cos(angle)
    return Origin(xyz=(x, y, z), rpy=(angle + extra_angle, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_traditional_windmill")

    stone = model.material("warm_stone", rgba=(0.67, 0.63, 0.55, 1.0))
    plaster = model.material("lime_plaster", rgba=(0.86, 0.82, 0.70, 1.0))
    shadow = model.material("dark_opening", rgba=(0.06, 0.045, 0.035, 1.0))
    roof_mat = model.material("weathered_thatch", rgba=(0.48, 0.34, 0.16, 1.0))
    wood = model.material("aged_oak", rgba=(0.50, 0.28, 0.12, 1.0))
    metal = model.material("dark_iron", rgba=(0.08, 0.08, 0.075, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.58, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stone,
        name="broad_plinth",
    )
    tower.visual(
        mesh_from_cadquery(_frustum(0.37, 0.235, 0.60, z0=0.10), "tower_body"),
        material=plaster,
        name="tower_body",
    )
    for i, (z, radius) in enumerate(((0.15, 0.365), (0.31, 0.325), (0.47, 0.285), (0.64, 0.265))):
        tower.visual(
            Cylinder(radius=radius, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=stone,
            name=f"stone_band_{i}",
        )
    tower.visual(
        Box((0.018, 0.17, 0.25)),
        origin=Origin(xyz=(0.355, 0.0, 0.245)),
        material=shadow,
        name="front_door",
    )
    tower.visual(
        Box((0.014, 0.105, 0.12)),
        origin=Origin(xyz=(0.287, 0.0, 0.515)),
        material=shadow,
        name="front_window",
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.305, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=wood,
        name="cap_skirt",
    )
    cap.visual(
        mesh_from_cadquery(_frustum(0.315, 0.075, 0.22, z0=0.08), "cap_roof"),
        material=roof_mat,
        name="cap_roof",
    )
    cap.visual(
        Box((0.26, 0.17, 0.14)),
        origin=Origin(xyz=(0.245, 0.0, 0.170)),
        material=wood,
        name="front_bearing_block",
    )
    cap.visual(
        Cylinder(radius=0.055, length=0.14),
        origin=Origin(xyz=(0.38, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="shaft_bearing",
    )

    sail_hub = model.part("sail_hub")
    sail_hub.visual(
        Cylinder(radius=0.085, length=0.080),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub_barrel",
    )
    sail_hub.visual(
        Cylinder(radius=0.115, length=0.030),
        origin=Origin(xyz=(0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub_face",
    )
    sail_hub.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=metal,
        name="nose_cap",
    )

    blade_angles = (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)
    for blade_index, angle in enumerate(blade_angles):
        sail_hub.visual(
            Box((0.036, 0.54, 0.026)),
            origin=_rotating_bar_origin(angle, 0.315, x=0.080),
            material=wood,
            name=f"spar_{blade_index}",
        )
        for side_index, offset in enumerate((-0.060, 0.060)):
            sail_hub.visual(
                Box((0.026, 0.390, 0.018)),
                origin=_rotating_bar_origin(angle, 0.365, x=0.090, offset=offset),
                material=wood,
                name=f"rail_{blade_index}_{side_index}",
            )
        for slat_index, radius in enumerate((0.205, 0.315, 0.425, 0.535)):
            sail_hub.visual(
                Box((0.024, 0.165, 0.014)),
                origin=_rotating_bar_origin(
                    angle,
                    radius,
                    x=0.100,
                    extra_angle=math.pi / 2.0,
                ),
                material=wood,
                name=f"cross_slat_{blade_index}_{slat_index}",
            )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.45),
    )
    model.articulation(
        "cap_to_sail_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=sail_hub,
        origin=Origin(xyz=(0.450, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=7.0),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    sail_hub = object_model.get_part("sail_hub")
    yaw = object_model.get_articulation("tower_to_cap")
    spin = object_model.get_articulation("cap_to_sail_hub")

    ctx.check(
        "cap yaw joint is vertical",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "sail hub spins on shaft axis",
        tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        positive_elem="cap_skirt",
        negative_elem="tower_body",
        max_gap=0.002,
        max_penetration=0.0,
        name="cap sits on tower crown",
    )
    ctx.expect_gap(
        sail_hub,
        cap,
        axis="x",
        positive_elem="hub_barrel",
        negative_elem="shaft_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotating hub is seated on front shaft",
    )

    base_aabb = ctx.part_element_world_aabb(tower, elem="broad_plinth")
    hub_aabb = ctx.part_world_aabb(sail_hub)
    if base_aabb is not None and hub_aabb is not None:
        base_width = base_aabb[1][0] - base_aabb[0][0]
        hub_center_z = 0.5 * (hub_aabb[0][2] + hub_aabb[1][2])
        ctx.check(
            "low broad support keeps rotating mass low",
            base_width > 1.0 and hub_center_z < 1.05,
            details=f"base_width={base_width:.3f}, hub_center_z={hub_center_z:.3f}",
        )
    else:
        ctx.fail("low broad support keeps rotating mass low", "missing base or hub bounds")

    rest_hub_position = ctx.part_world_position(sail_hub)
    with ctx.pose({yaw: math.pi / 2.0}):
        turned_hub_position = ctx.part_world_position(sail_hub)
    ctx.check(
        "cap yaw carries hub around tower",
        rest_hub_position is not None
        and turned_hub_position is not None
        and rest_hub_position[0] > 0.40
        and abs(rest_hub_position[1]) < 0.01
        and turned_hub_position[1] > 0.40
        and abs(turned_hub_position[0]) < 0.01,
        details=f"rest={rest_hub_position}, turned={turned_hub_position}",
    )

    def _elem_center_z(part, elem: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        return 0.5 * (bounds[0][2] + bounds[1][2])

    rest_spar_z = _elem_center_z(sail_hub, "spar_0")
    with ctx.pose({spin: math.pi / 2.0}):
        raised_spar_z = _elem_center_z(sail_hub, "spar_0")
    ctx.check(
        "hub rotation moves a sail through the vertical plane",
        rest_spar_z is not None and raised_spar_z is not None and raised_spar_z > rest_spar_z + 0.22,
        details=f"rest_spar_z={rest_spar_z}, raised_spar_z={raised_spar_z}",
    )

    return ctx.report()


object_model = build_object_model()
