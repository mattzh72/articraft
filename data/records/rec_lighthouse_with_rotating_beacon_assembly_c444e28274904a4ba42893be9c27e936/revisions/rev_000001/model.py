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


def _square_frustum(bottom: float, top: float, height: float) -> cq.Workplane:
    """A simple tapered square concrete shaft, authored in meters."""
    return (
        cq.Workplane("XY")
        .rect(bottom, bottom)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _annular_collar(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Hollow vertical ring used as the beacon's captured clip collar."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_square_lighthouse")

    concrete = model.material("warm_concrete", rgba=(0.64, 0.63, 0.58, 1.0))
    dark_concrete = model.material("recessed_concrete", rgba=(0.34, 0.35, 0.34, 1.0))
    dark_metal = model.material("anodized_black_metal", rgba=(0.02, 0.025, 0.03, 1.0))
    brushed_metal = model.material("brushed_motor_metal", rgba=(0.55, 0.56, 0.53, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.62, 0.86, 1.0, 0.34))
    warm_glass = model.material("warm_fresnel_glass", rgba=(1.0, 0.84, 0.30, 0.75))
    light_core = model.material("lit_beacon_core", rgba=(1.0, 0.93, 0.42, 1.0))

    tower = model.part("tower")

    # Concrete square tower with a subtly tapering modern lighthouse silhouette.
    tower.visual(
        Box((2.25, 2.25, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=concrete,
        name="base_plinth",
    )
    tower.visual(
        mesh_from_cadquery(_square_frustum(1.34, 1.02, 6.75), "tapered_tower"),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=concrete,
        name="tower_shaft",
    )
    tower.visual(
        Box((0.36, 0.035, 0.72)),
        origin=Origin(xyz=(0.0, -0.650, 0.76)),
        material=dark_concrete,
        name="entry_door",
    )
    for i, z in enumerate((2.45, 3.75, 5.05)):
        tower.visual(
            Box((0.17, 0.120, 0.46)),
            origin=Origin(xyz=(0.0, -0.570, z)),
            material=dark_concrete,
            name=f"slit_window_{i}",
        )

    # Compact square lantern room, fixed to the concrete cap.
    tower.visual(
        Box((1.36, 1.36, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 7.08)),
        material=concrete,
        name="gallery_slab",
    )
    tower.visual(
        Box((0.94, 0.94, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 7.22)),
        material=dark_metal,
        name="lantern_floor",
    )

    post_z = 7.63
    for ix, x in enumerate((-0.43, 0.43)):
        for iy, y in enumerate((-0.43, 0.43)):
            tower.visual(
                Box((0.055, 0.055, 0.70)),
                origin=Origin(xyz=(x, y, post_z)),
                material=dark_metal,
                name=f"corner_post_{ix}_{iy}",
            )

    for z_name, z in (("lower", 7.31), ("upper", 7.96)):
        tower.visual(
            Box((0.92, 0.052, 0.055)),
            origin=Origin(xyz=(0.0, -0.43, z)),
            material=dark_metal,
            name=f"{z_name}_rear_rail",
        )
        tower.visual(
            Box((0.92, 0.052, 0.055)),
            origin=Origin(xyz=(0.0, 0.43, z)),
            material=dark_metal,
            name=f"{z_name}_front_rail",
        )
        tower.visual(
            Box((0.052, 0.92, 0.055)),
            origin=Origin(xyz=(-0.43, 0.0, z)),
            material=dark_metal,
            name=f"{z_name}_side_rail_0",
        )
        tower.visual(
            Box((0.052, 0.92, 0.055)),
            origin=Origin(xyz=(0.43, 0.0, z)),
            material=dark_metal,
            name=f"{z_name}_side_rail_1",
        )

    glass_z = 7.64
    tower.visual(
        Box((0.84, 0.014, 0.62)),
        origin=Origin(xyz=(0.0, -0.462, glass_z)),
        material=glass,
        name="rear_glass",
    )
    tower.visual(
        Box((0.014, 0.84, 0.62)),
        origin=Origin(xyz=(-0.462, 0.0, glass_z)),
        material=glass,
        name="side_glass_0",
    )
    tower.visual(
        Box((0.014, 0.84, 0.62)),
        origin=Origin(xyz=(0.462, 0.0, glass_z)),
        material=glass,
        name="side_glass_1",
    )
    tower.visual(
        Box((0.42, 0.014, 0.62)),
        origin=Origin(xyz=(0.17, 0.462, glass_z)),
        material=glass,
        name="front_fixed_glass",
    )
    tower.visual(
        Box((0.055, 0.026, 0.66)),
        origin=Origin(xyz=(-0.34, 0.464, 7.64)),
        material=dark_metal,
        name="hinge_backplate",
    )
    tower.visual(
        mesh_from_cadquery(_square_frustum(1.02, 0.42, 0.30), "lantern_roof"),
        origin=Origin(xyz=(0.0, 0.0, 7.985)),
        material=dark_metal,
        name="tapered_roof",
    )
    tower.visual(
        Cylinder(radius=0.055, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 8.365)),
        material=dark_metal,
        name="roof_finial",
    )

    # Motor pedestal is intentionally short and visible through the glass.
    tower.visual(
        Cylinder(radius=0.055, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 7.39)),
        material=brushed_metal,
        name="motor_pedestal",
    )

    # Rotating beacon: frame origin is on the vertical motor axis at the
    # pedestal top. The hollow collar drops over the pedestal so the beacon reads
    # as captured while still clearing the fixed motor cylinder.
    beacon = model.part("beacon")
    beacon.visual(
        mesh_from_cadquery(_annular_collar(0.095, 0.066, 0.13), "clip_collar"),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=dark_metal,
        name="clip_collar",
    )
    beacon.visual(
        Cylinder(radius=0.110, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_metal,
        name="turntable_cap",
    )
    beacon.visual(
        Cylinder(radius=0.026, length=0.27),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=brushed_metal,
        name="center_stem",
    )
    beacon.visual(
        Cylinder(radius=0.040, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.315), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="lamp_barrel",
    )
    beacon.visual(
        Sphere(radius=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=light_core,
        name="lamp_core",
    )
    beacon.visual(
        Box((0.18, 0.095, 0.23)),
        origin=Origin(xyz=(0.245, 0.0, 0.315)),
        material=warm_glass,
        name="lens_0",
    )
    beacon.visual(
        Box((0.18, 0.095, 0.23)),
        origin=Origin(xyz=(-0.245, 0.0, 0.315)),
        material=warm_glass,
        name="lens_1",
    )
    beacon.visual(
        Box((0.040, 0.040, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_metal,
        name="clip_bridge",
    )

    model.articulation(
        "beacon_axis",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 7.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5),
    )

    # Narrow hinged glass access panel on the front side of the lantern.
    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((0.235, 0.020, 0.56)),
        origin=Origin(xyz=(0.1175, 0.0, 0.28)),
        material=glass,
        name="glass_panel",
    )
    access_panel.visual(
        Box((0.025, 0.030, 0.58)),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=dark_metal,
        name="hinge_stile",
    )
    access_panel.visual(
        Box((0.025, 0.030, 0.58)),
        origin=Origin(xyz=(0.235, 0.0, 0.29)),
        material=dark_metal,
        name="latch_stile",
    )
    access_panel.visual(
        Box((0.235, 0.030, 0.025)),
        origin=Origin(xyz=(0.1175, 0.0, 0.0125)),
        material=dark_metal,
        name="bottom_rail",
    )
    access_panel.visual(
        Box((0.235, 0.030, 0.025)),
        origin=Origin(xyz=(0.1175, 0.0, 0.5675)),
        material=dark_metal,
        name="top_rail",
    )
    access_panel.visual(
        Cylinder(radius=0.018, length=0.58),
        origin=Origin(xyz=(0.0, 0.032, 0.29)),
        material=dark_metal,
        name="hinge_barrel",
    )
    access_panel.visual(
        Box((0.025, 0.025, 0.11)),
        origin=Origin(xyz=(0.205, 0.012, 0.31)),
        material=brushed_metal,
        name="pull_handle",
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=access_panel,
        origin=Origin(xyz=(-0.34, 0.492, 7.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    panel = object_model.get_part("access_panel")
    beacon_axis = object_model.get_articulation("beacon_axis")
    panel_hinge = object_model.get_articulation("panel_hinge")

    ctx.check(
        "beacon rotates continuously",
        beacon_axis.articulation_type == ArticulationType.CONTINUOUS
        and tuple(beacon_axis.axis) == (0.0, 0.0, 1.0),
        details=f"type={beacon_axis.articulation_type}, axis={beacon_axis.axis}",
    )

    limits = panel_hinge.motion_limits
    ctx.check(
        "access panel has a side hinge swing",
        panel_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(panel_hinge.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper > 1.0,
        details=f"type={panel_hinge.articulation_type}, axis={panel_hinge.axis}, limits={limits}",
    )

    # The collar surrounds the motor pedestal in plan and remains vertically
    # engaged with it; no overlap allowance is needed because the collar is a
    # true hollow annulus with radial clearance.
    ctx.expect_within(
        tower,
        beacon,
        axes="xy",
        inner_elem="motor_pedestal",
        outer_elem="clip_collar",
        margin=0.002,
        name="pedestal sits inside beacon collar",
    )
    ctx.expect_overlap(
        beacon,
        tower,
        axes="z",
        elem_a="clip_collar",
        elem_b="motor_pedestal",
        min_overlap=0.080,
        name="collar remains clipped over pedestal",
    )

    for q, name in ((0.0, "closed direction"), (math.pi / 2.0, "quarter turn")):
        with ctx.pose({beacon_axis: q}):
            ctx.expect_within(
                beacon,
                tower,
                axes="xy",
                outer_elem="lantern_floor",
                margin=0.0,
                name=f"beacon stays inside compact lantern at {name}",
            )

    closed_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_hinge: 1.1}):
        opened_aabb = ctx.part_world_aabb(panel)
    closed_max_y = closed_aabb[1][1] if closed_aabb is not None else None
    opened_max_y = opened_aabb[1][1] if opened_aabb is not None else None
    ctx.check(
        "access panel opens outward",
        closed_max_y is not None and opened_max_y is not None and opened_max_y > closed_max_y + 0.10,
        details=f"closed_max_y={closed_max_y}, opened_max_y={opened_max_y}",
    )

    return ctx.report()


object_model = build_object_model()
