from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A centered masonry ring with an open middle for the belfry."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _onion_dome() -> LatheGeometry:
    """Revolved Eastern Orthodox onion dome profile, local base at z=0."""
    profile = [
        (0.0, 0.0),
        (0.55, 0.0),
        (0.72, 0.18),
        (0.79, 0.45),
        (0.70, 0.78),
        (0.48, 1.08),
        (0.23, 1.30),
        (0.09, 1.48),
        (0.0, 1.54),
    ]
    return LatheGeometry(profile, segments=72, closed=True)


def _arched_belfry_frame() -> cq.Workplane:
    """A flat arched masonry face with a through-opening, local base at z=0."""
    outer_w = 1.26
    inner_w = 0.92
    outer_leg_h = 0.77
    inner_leg_h = 0.76
    depth = 0.12

    outer = (
        cq.Workplane("XZ")
        .moveTo(-outer_w / 2.0, 0.0)
        .lineTo(-outer_w / 2.0, outer_leg_h)
        .threePointArc((0.0, outer_leg_h + outer_w / 2.0), (outer_w / 2.0, outer_leg_h))
        .lineTo(outer_w / 2.0, 0.0)
        .close()
        .extrude(depth)
        .translate((0.0, -depth / 2.0, 0.0))
    )

    inner = (
        cq.Workplane("XZ")
        .moveTo(-inner_w / 2.0, 0.08)
        .lineTo(-inner_w / 2.0, inner_leg_h)
        .threePointArc((0.0, inner_leg_h + inner_w / 2.0), (inner_w / 2.0, inner_leg_h))
        .lineTo(inner_w / 2.0, 0.08)
        .close()
        .extrude(depth * 2.0)
        .translate((0.0, -depth, 0.0))
    )
    return outer.cut(inner)


def _bell_cup() -> LatheGeometry:
    """A hollow bronze bell mantle, local coordinates relative to the swing axis."""
    outer = [
        (0.10, -0.24),
        (0.18, -0.27),
        (0.25, -0.42),
        (0.29, -0.70),
        (0.37, -0.96),
        (0.47, -1.10),
    ]
    inner = [
        (0.07, -0.24),
        (0.10, -0.30),
        (0.18, -0.50),
        (0.23, -0.86),
        (0.31, -1.06),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthodox_onion_dome_bell_tower")

    stone = model.material("warm_limestone", rgba=(0.72, 0.66, 0.54, 1.0))
    stone_dark = model.material("shadowed_mortar", rgba=(0.45, 0.41, 0.34, 1.0))
    aged_wood = model.material("aged_oak", rgba=(0.36, 0.22, 0.12, 1.0))
    gilded = model.material("gilded_dome", rgba=(0.95, 0.72, 0.20, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.64, 0.39, 0.18, 1.0))
    dark_metal = model.material("blackened_iron", rgba=(0.05, 0.045, 0.04, 1.0))

    tower = model.part("tower")

    # Round cylindrical masonry shaft and plinth.
    tower.visual(
        Cylinder(radius=1.12, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=stone_dark,
        name="stone_plinth",
    )
    tower.visual(
        Cylinder(radius=0.86, length=3.95),
        origin=Origin(xyz=(0.0, 0.0, 2.275)),
        material=stone,
        name="cylindrical_shaft",
    )
    for idx, z in enumerate((0.62, 1.42, 2.22, 3.02, 3.82)):
        tower.visual(
            Cylinder(radius=0.875, length=0.045),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=stone_dark,
            name=f"mortar_belt_{idx}",
        )

    # Open belfry: annular sill/cornice rings, diagonal piers, and a front arch.
    tower.visual(
        mesh_from_cadquery(_annular_ring(1.00, 0.58, 0.22), "belfry_lower_ring"),
        origin=Origin(xyz=(0.0, 0.0, 4.33)),
        material=stone_dark,
        name="belfry_lower_ring",
    )
    tower.visual(
        mesh_from_cadquery(_annular_ring(0.96, 0.54, 0.22), "belfry_upper_ring"),
        origin=Origin(xyz=(0.0, 0.0, 5.76)),
        material=stone_dark,
        name="belfry_upper_ring",
    )
    pier_radius = 0.13
    pier_center_radius = 0.78
    for idx, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        tower.visual(
            Cylinder(radius=pier_radius, length=1.35),
            origin=Origin(
                xyz=(
                    pier_center_radius * math.cos(angle),
                    pier_center_radius * math.sin(angle),
                    5.05,
                )
            ),
            material=stone,
            name=f"belfry_pier_{idx}",
        )
    for idx, x in enumerate((-0.78, 0.78)):
        tower.visual(
            Cylinder(radius=0.105, length=1.35),
            origin=Origin(xyz=(x, 0.0, 5.05)),
            material=stone,
            name=f"headstock_post_{idx}",
        )
    tower.visual(
        mesh_from_cadquery(_arched_belfry_frame(), "front_arch_frame"),
        origin=Origin(xyz=(0.0, 0.74, 4.42)),
        material=stone,
        name="front_arch_frame",
    )

    # The wooden headstock beam visibly spans the arched belfry opening.
    tower.visual(
        Box((1.74, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 5.61)),
        material=aged_wood,
        name="headstock_beam",
    )

    # Onion dome, spire, and Orthodox cross mounted to the upper ring.
    tower.visual(
        mesh_from_geometry(_onion_dome(), "onion_dome"),
        origin=Origin(xyz=(0.0, 0.0, 5.82)),
        material=gilded,
        name="onion_dome",
    )
    tower.visual(
        Cylinder(radius=0.055, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 7.53)),
        material=gilded,
        name="dome_spire",
    )
    tower.visual(
        Box((0.075, 0.075, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, 7.92)),
        material=gilded,
        name="cross_upright",
    )
    tower.visual(
        Box((0.42, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 8.05)),
        material=gilded,
        name="cross_bar",
    )
    tower.visual(
        Box((0.28, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 7.86), rpy=(0.0, 0.0, 0.24)),
        material=gilded,
        name="cross_foot_bar",
    )

    bell = model.part("bell")
    bell.visual(
        mesh_from_geometry(_bell_cup(), "bell_cup"),
        material=bronze,
        name="bell_cup",
    )
    bell.visual(
        Cylinder(radius=0.12, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        material=bronze,
        name="bell_crown",
    )
    bell.visual(
        Cylinder(radius=0.045, length=0.78),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="bell_axle",
    )
    for idx, x in enumerate((-0.18, 0.18)):
        bell.visual(
            Box((0.055, 0.055, 0.25)),
            origin=Origin(xyz=(x, 0.0, -0.165)),
            material=dark_metal,
            name=f"hanger_strap_{idx}",
        )
    bell.visual(
        Cylinder(radius=0.016, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, -0.65)),
        material=dark_metal,
        name="clapper_rod",
    )
    bell.visual(
        Sphere(radius=0.075),
        origin=Origin(xyz=(0.0, 0.0, -1.04)),
        material=dark_metal,
        name="clapper_ball",
    )

    model.articulation(
        "tower_to_bell",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 5.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-0.48, upper=0.48),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell")
    hinge = object_model.get_articulation("tower_to_bell")

    # The axle is intentionally represented as a captured shaft through the
    # solid wooden headstock beam; the hinge axis is the beam centerline.
    ctx.allow_overlap(
        tower,
        bell,
        elem_a="headstock_beam",
        elem_b="bell_axle",
        reason="The bell axle is a captured shaft passing through the simplified wooden headstock beam.",
    )
    ctx.expect_overlap(
        tower,
        bell,
        axes="x",
        elem_a="headstock_beam",
        elem_b="bell_axle",
        min_overlap=0.60,
        name="axle spans inside the headstock beam",
    )
    ctx.expect_gap(
        tower,
        bell,
        axis="z",
        positive_elem="headstock_beam",
        negative_elem="bell_axle",
        max_penetration=0.06,
        name="axle is locally captured in the beam",
    )

    ctx.check(
        "bell uses a horizontal headstock hinge",
        hinge is not None and tuple(round(v, 6) for v in hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={getattr(hinge, 'axis', None)}",
    )
    ctx.check(
        "bell swing limits are symmetric and restrained",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower is not None
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.lower < -0.35
        and hinge.motion_limits.upper > 0.35
        and hinge.motion_limits.upper <= 0.55,
        details=f"limits={getattr(hinge, 'motion_limits', None)}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_within(
            bell,
            tower,
            axes="xy",
            inner_elem="bell_cup",
            outer_elem="belfry_lower_ring",
            margin=0.05,
            name="resting bell cup fits inside the round belfry",
        )
        rest_aabb = ctx.part_element_world_aabb(bell, elem="bell_cup")

    with ctx.pose({hinge: 0.40}):
        open_aabb = ctx.part_element_world_aabb(bell, elem="bell_cup")

    rest_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
    open_y = None if open_aabb is None else (open_aabb[0][1] + open_aabb[1][1]) / 2.0
    ctx.check(
        "positive hinge angle swings the bell through the belfry opening",
        rest_y is not None and open_y is not None and open_y > rest_y + 0.15,
        details=f"rest_y={rest_y}, open_y={open_y}",
    )

    return ctx.report()


object_model = build_object_model()
