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


def _tube_mesh(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    z0: float = 0.0,
    name: str,
):
    """Closed annular tube mesh with real center clearance."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, z0))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0007, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_pedestal_music_stand")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    graphite = model.material("graphite", rgba=(0.10, 0.105, 0.115, 1.0))
    edge_black = model.material("edge_black", rgba=(0.025, 0.025, 0.028, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    pivot_metal = model.material("dark_pivot_metal", rgba=(0.20, 0.21, 0.23, 1.0))

    lower_mast = model.part("lower_mast")
    lower_mast.visual(
        Cylinder(radius=0.185, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=satin_black,
        name="round_pedestal",
    )
    lower_mast.visual(
        Cylinder(radius=0.165, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="rubber_pad",
    )
    lower_mast.visual(
        Cylinder(radius=0.055, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=edge_black,
        name="pedestal_boss",
    )
    lower_mast.visual(
        _tube_mesh(0.025, 0.0165, 0.610, z0=0.052, name="lower_tube_mesh"),
        material=satin_black,
        name="lower_tube",
    )
    lower_mast.visual(
        _tube_mesh(0.035, 0.0175, 0.060, z0=0.620, name="collar_ring_mesh"),
        material=edge_black,
        name="top_collar",
    )
    lower_mast.visual(
        Cylinder(radius=0.0055, length=0.048),
        origin=Origin(xyz=(0.050, 0.0, 0.652), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pivot_metal,
        name="clamp_screw",
    )
    lower_mast.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.080, 0.0, 0.652)),
        material=rubber,
        name="height_knob",
    )
    lower_mast.visual(
        Box((0.012, 0.022, 0.030)),
        origin=Origin(xyz=(0.015, 0.0, 0.652)),
        material=pivot_metal,
        name="clamp_pad",
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        Cylinder(radius=0.0120, length=1.100),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=graphite,
        name="inner_tube",
    )
    upper_mast.visual(
        Cylinder(radius=0.0145, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.400)),
        material=pivot_metal,
        name="sliding_bushing",
    )
    upper_mast.visual(
        Cylinder(radius=0.017, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.665)),
        material=edge_black,
        name="top_cap",
    )
    upper_mast.visual(
        Box((0.034, 0.065, 0.045)),
        origin=Origin(xyz=(0.0, -0.027, 0.650)),
        material=edge_black,
        name="yoke_stem",
    )
    upper_mast.visual(
        Box((0.210, 0.034, 0.026)),
        origin=Origin(xyz=(0.0, -0.065, 0.660)),
        material=edge_black,
        name="yoke_crossbar",
    )
    upper_mast.visual(
        Box((0.235, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, -0.085, 0.660)),
        material=edge_black,
        name="yoke_bridge",
    )
    for x, name in ((-0.094, "yoke_ear_0"), (0.094, "yoke_ear_1")):
        upper_mast.visual(
            Box((0.028, 0.052, 0.076)),
            origin=Origin(xyz=(x, -0.065, 0.700)),
            material=edge_black,
            name=name,
        )
        upper_mast.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, -0.065, 0.700), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pivot_metal,
            name=f"{name}_bushing",
        )
    upper_mast.visual(
        Cylinder(radius=0.007, length=0.230),
        origin=Origin(xyz=(0.0, -0.065, 0.700), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pivot_metal,
        name="hinge_pin",
    )

    mast_slide = model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=lower_mast,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=85.0, velocity=0.18, lower=0.0, upper=0.320),
    )
    mast_slide.meta["qc_samples"] = [0.0, 0.16, 0.32]

    tray = model.part("tray")
    tray.visual(
        Box((0.620, 0.014, 0.360)),
        origin=Origin(xyz=(0.0, -0.045, 0.060)),
        material=graphite,
        name="back_panel",
    )
    tray.visual(
        Box((0.640, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, -0.026, 0.252)),
        material=edge_black,
        name="top_rail",
    )
    tray.visual(
        Box((0.640, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, -0.026, -0.134)),
        material=edge_black,
        name="bottom_rail",
    )
    tray.visual(
        Box((0.575, 0.066, 0.022)),
        origin=Origin(xyz=(0.0, -0.065, -0.150), rpy=(0.10, 0.0, 0.0)),
        material=edge_black,
        name="page_shelf",
    )
    tray.visual(
        Box((0.575, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, -0.102, -0.130), rpy=(0.10, 0.0, 0.0)),
        material=edge_black,
        name="shelf_lip",
    )
    for x, name in ((-0.323, "side_rail_0"), (0.323, "side_rail_1")):
        tray.visual(
            Box((0.018, 0.026, 0.360)),
            origin=Origin(xyz=(x, -0.026, 0.060)),
            material=edge_black,
            name=name,
        )
    tray.visual(
        Cylinder(radius=0.018, length=0.135),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pivot_metal,
        name="hinge_barrel",
    )
    tray.visual(
        Box((0.125, 0.030, 0.064)),
        origin=Origin(xyz=(0.0, -0.031, 0.006)),
        material=edge_black,
        name="hinge_saddle",
    )
    for x, name in ((-0.205, "clip_boss_0"), (0.205, "clip_boss_1")):
        tray.visual(
            Box((0.048, 0.023, 0.016)),
            origin=Origin(xyz=(x, -0.0505, 0.269)),
            material=edge_black,
            name=name,
        )

    tray_hinge = model.articulation(
        "tray_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_mast,
        child=tray,
        origin=Origin(xyz=(0.0, -0.065, 0.700), rpy=(-0.22, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.45, upper=0.55),
    )
    tray_hinge.meta["qc_samples"] = [-0.35, 0.0, 0.45]

    for x, suffix in ((-0.205, "0"), (0.205, "1")):
        clip = model.part(f"page_clip_{suffix}")
        clip.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pivot_metal,
            name="pivot",
        )
        clip.visual(
            Box((0.034, 0.026, 0.104)),
            origin=Origin(xyz=(0.0, -0.013, -0.054)),
            material=edge_black,
            name="spring_tab",
        )
        clip.visual(
            Sphere(radius=0.008),
            origin=Origin(xyz=(0.0, -0.013, -0.109)),
            material=rubber,
            name="rubber_tip",
        )
        clip_joint = model.articulation(
            f"clip_pivot_{suffix}",
            ArticulationType.REVOLUTE,
            parent=tray,
            child=clip,
            origin=Origin(xyz=(x, -0.068, 0.269)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=1.05),
        )
        clip_joint.meta["qc_samples"] = [0.0, 0.65, 1.05]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_mast = object_model.get_part("lower_mast")
    upper_mast = object_model.get_part("upper_mast")
    tray = object_model.get_part("tray")
    clip_0 = object_model.get_part("page_clip_0")
    clip_1 = object_model.get_part("page_clip_1")
    mast_slide = object_model.get_articulation("mast_slide")
    tray_hinge = object_model.get_articulation("tray_hinge")
    clip_pivot_0 = object_model.get_articulation("clip_pivot_0")

    ctx.allow_overlap(
        lower_mast,
        upper_mast,
        elem_a="clamp_pad",
        elem_b="inner_tube",
        reason="The height-lock clamp pad intentionally presses into the sliding upper mast tube.",
    )
    ctx.allow_overlap(
        upper_mast,
        tray,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The support yoke pin is intentionally captured inside the tray hinge barrel.",
    )

    ctx.expect_gap(
        lower_mast,
        upper_mast,
        axis="x",
        positive_elem="clamp_pad",
        negative_elem="inner_tube",
        max_gap=0.001,
        max_penetration=0.004,
        name="clamp pad bears on upper mast tube",
    )
    ctx.expect_overlap(
        lower_mast,
        upper_mast,
        axes="yz",
        elem_a="clamp_pad",
        elem_b="inner_tube",
        min_overlap=0.015,
        name="clamp pad crosses the sliding tube wall",
    )
    ctx.expect_within(
        upper_mast,
        tray,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.002,
        name="hinge pin is centered inside tray barrel",
    )
    ctx.expect_overlap(
        upper_mast,
        tray,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.12,
        name="hinge pin runs through tray barrel",
    )

    ctx.expect_within(
        upper_mast,
        lower_mast,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="lower_tube",
        margin=0.002,
        name="upper mast is centered inside lower mast",
    )
    ctx.expect_overlap(
        upper_mast,
        lower_mast,
        axes="z",
        elem_a="inner_tube",
        elem_b="lower_tube",
        min_overlap=0.18,
        name="collapsed mast remains inserted",
    )

    mast_rest = ctx.part_world_position(upper_mast)
    with ctx.pose({mast_slide: 0.320}):
        ctx.expect_within(
            upper_mast,
            lower_mast,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_tube",
            margin=0.002,
            name="extended mast stays centered",
        )
        ctx.expect_overlap(
            upper_mast,
            lower_mast,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_tube",
            min_overlap=0.11,
            name="extended mast retains insertion",
        )
        mast_extended = ctx.part_world_position(upper_mast)
    ctx.check(
        "mast slide raises upper mast",
        mast_rest is not None and mast_extended is not None and mast_extended[2] > mast_rest[2] + 0.30,
        details=f"rest={mast_rest}, extended={mast_extended}",
    )

    ctx.expect_overlap(
        tray,
        upper_mast,
        axes="x",
        elem_a="hinge_barrel",
        elem_b="yoke_bridge",
        min_overlap=0.10,
        name="tray hinge spans the support yoke",
    )
    ctx.expect_overlap(
        clip_0,
        tray,
        axes="x",
        elem_a="pivot",
        elem_b="clip_boss_0",
        min_overlap=0.030,
        name="first page clip sits on top rail pivot",
    )
    ctx.expect_overlap(
        clip_1,
        tray,
        axes="x",
        elem_a="pivot",
        elem_b="clip_boss_1",
        min_overlap=0.030,
        name="second page clip sits on top rail pivot",
    )

    top_rest = ctx.part_element_world_aabb(tray, elem="top_rail")
    with ctx.pose({tray_hinge: 0.45}):
        top_tilted = ctx.part_element_world_aabb(tray, elem="top_rail")
    if top_rest is not None and top_tilted is not None:
        rest_center_y = 0.5 * (top_rest[0][1] + top_rest[1][1])
        tilted_center_y = 0.5 * (top_tilted[0][1] + top_tilted[1][1])
        rest_center_z = 0.5 * (top_rest[0][2] + top_rest[1][2])
        tilted_center_z = 0.5 * (top_tilted[0][2] + top_tilted[1][2])
        tray_moves = abs(tilted_center_y - rest_center_y) > 0.04 or abs(tilted_center_z - rest_center_z) > 0.02
    else:
        tray_moves = False
    ctx.check("tray hinge rotates the rectangular tray", tray_moves)

    tip_rest = ctx.part_element_world_aabb(clip_0, elem="rubber_tip")
    with ctx.pose({clip_pivot_0: 0.80}):
        tip_open = ctx.part_element_world_aabb(clip_0, elem="rubber_tip")
    if tip_rest is not None and tip_open is not None:
        rest_tip_y = 0.5 * (tip_rest[0][1] + tip_rest[1][1])
        open_tip_y = 0.5 * (tip_open[0][1] + tip_open[1][1])
        clip_opens = open_tip_y < rest_tip_y - 0.045
    else:
        clip_opens = False
    ctx.check("page clip pivots outward from the top rail", clip_opens)

    return ctx.report()


object_model = build_object_model()
