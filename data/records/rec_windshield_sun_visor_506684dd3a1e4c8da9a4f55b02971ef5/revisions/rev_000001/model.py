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


PANEL_LENGTH = 0.440
PANEL_WIDTH = 0.170
PANEL_THICKNESS = 0.026
ROD_Z = -0.024
COVER_WIDTH = 0.180
COVER_HEIGHT = 0.076
COVER_THICKNESS = 0.004


def _rounded_panel_body() -> object:
    """Soft, rounded rectangular padded visor slab in local centered coordinates."""
    body = cq.Workplane("XY").box(PANEL_LENGTH, PANEL_WIDTH, PANEL_THICKNESS)
    # Large vertical-edge radius gives the padded visor its rounded-corner silhouette;
    # a smaller pass softens the manufactured cushion edges.
    body = body.edges("|Z").fillet(0.020)
    body = body.edges().fillet(0.003)
    return body


def _rounded_flap(width: float, height: float, thickness: float, radius: float) -> object:
    flap = cq.Workplane("XY").box(width, height, thickness)
    flap = flap.edges("|Z").fillet(radius)
    flap = flap.edges().fillet(0.0012)
    return flap


def _tube_x(length: float, outer_radius: float, inner_radius: float) -> object:
    """Annular tube centered at the origin with its axis along local +X."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )
    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="driver_sun_visor")

    headliner_mat = model.material("warm_gray_headliner", rgba=(0.62, 0.60, 0.55, 1.0))
    visor_mat = model.material("soft_beige_vinyl", rgba=(0.78, 0.72, 0.62, 1.0))
    seam_mat = model.material("slightly_darker_seam", rgba=(0.54, 0.49, 0.40, 1.0))
    plastic_mat = model.material("molded_taupe_plastic", rgba=(0.43, 0.40, 0.34, 1.0))
    dark_plastic_mat = model.material("dark_shadow_plastic", rgba=(0.07, 0.065, 0.055, 1.0))
    metal_mat = model.material("satin_metal_rod", rgba=(0.74, 0.72, 0.68, 1.0))
    mirror_mat = model.material("cool_mirror_glass", rgba=(0.62, 0.77, 0.86, 0.72))

    roof_liner = model.part("roof_liner")
    roof_liner.visual(
        Box((0.610, 0.300, 0.012)),
        origin=Origin(xyz=(0.245, -0.078, 0.006)),
        material=headliner_mat,
        name="headliner_patch",
    )
    roof_liner.visual(
        Box((0.545, 0.010, 0.002)),
        origin=Origin(xyz=(0.275, 0.020, -0.001)),
        material=seam_mat,
        name="roof_edge_shadow",
    )

    roof_bracket = model.part("roof_bracket")
    roof_bracket.visual(
        Box((0.086, 0.060, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        material=plastic_mat,
        name="bracket_base",
    )
    roof_bracket.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, -0.021)),
        material=plastic_mat,
        name="pivot_boss",
    )
    roof_bracket.visual(
        Box((0.030, 0.018, 0.020)),
        origin=Origin(xyz=(0.035, -0.002, -0.027)),
        material=plastic_mat,
        name="lower_web",
    )
    roof_bracket.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(-0.023, -0.015, -0.013)),
        material=dark_plastic_mat,
        name="screw_cap_0",
    )
    roof_bracket.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.023, 0.015, -0.013)),
        material=dark_plastic_mat,
        name="screw_cap_1",
    )

    side_clip = model.part("side_clip")
    side_clip.visual(
        Box((0.060, 0.046, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, -0.005)),
        material=plastic_mat,
        name="clip_roof_pad",
    )
    side_clip.visual(
        Box((0.040, 0.036, 0.020)),
        origin=Origin(xyz=(0.000, 0.000, -0.020)),
        material=plastic_mat,
        name="clip_bridge",
    )
    side_clip.visual(
        Box((0.030, 0.008, 0.039)),
        origin=Origin(xyz=(0.000, -0.014, -0.046)),
        material=plastic_mat,
        name="clip_jaw_0",
    )
    side_clip.visual(
        Box((0.030, 0.008, 0.039)),
        origin=Origin(xyz=(0.000, 0.014, -0.046)),
        material=plastic_mat,
        name="clip_jaw_1",
    )
    side_clip.visual(
        Box((0.030, 0.010, 0.006)),
        origin=Origin(xyz=(0.000, -0.014, -0.058)),
        material=dark_plastic_mat,
        name="clip_lip_0",
    )
    side_clip.visual(
        Box((0.030, 0.010, 0.006)),
        origin=Origin(xyz=(0.000, 0.014, -0.058)),
        material=dark_plastic_mat,
        name="clip_lip_1",
    )

    hinge_rod = model.part("hinge_rod")
    hinge_rod.visual(
        Cylinder(radius=0.0085, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.014), rpy=(0.0, 0.0, 0.0)),
        material=metal_mat,
        name="vertical_pin",
    )
    hinge_rod.visual(
        Cylinder(radius=0.0060, length=0.458),
        origin=Origin(xyz=(0.229, 0.0, ROD_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="rod_core",
    )
    hinge_rod.visual(
        Sphere(radius=0.0080),
        origin=Origin(xyz=(0.458, 0.0, ROD_Z)),
        material=metal_mat,
        name="clip_end_button",
    )
    hinge_rod.visual(
        Box((0.028, 0.018, 0.014)),
        origin=Origin(xyz=(0.014, 0.0, ROD_Z)),
        material=metal_mat,
        name="rod_elbow_block",
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        mesh_from_cadquery(_rounded_panel_body(), "visor_padded_panel", tolerance=0.0008),
        origin=Origin(xyz=(0.235, -0.092, -0.025)),
        material=visor_mat,
        name="visor_body",
    )
    visor_panel.visual(
        mesh_from_cadquery(_tube_x(0.360, 0.011, 0.0072), "visor_hinge_sleeve", tolerance=0.0008),
        origin=Origin(xyz=(0.230, 0.0, 0.0)),
        material=visor_mat,
        name="fabric_sleeve",
    )
    visor_panel.visual(
        Box((0.360, 0.008, 0.008)),
        origin=Origin(xyz=(0.230, -0.011, -0.011)),
        material=visor_mat,
        name="sleeve_web",
    )
    # Slightly proud seam lines and the vanity recess are part of the soft panel.
    visor_panel.visual(
        Box((0.390, 0.006, 0.0012)),
        origin=Origin(xyz=(0.235, -0.016, -0.0376)),
        material=seam_mat,
        name="upper_stitch",
    )
    visor_panel.visual(
        Box((0.390, 0.006, 0.0012)),
        origin=Origin(xyz=(0.235, -0.168, -0.0376)),
        material=seam_mat,
        name="lower_stitch",
    )
    visor_panel.visual(
        Box((0.006, 0.140, 0.0012)),
        origin=Origin(xyz=(0.032, -0.092, -0.0376)),
        material=seam_mat,
        name="inboard_stitch",
    )
    visor_panel.visual(
        Box((0.006, 0.140, 0.0012)),
        origin=Origin(xyz=(0.438, -0.092, -0.0376)),
        material=seam_mat,
        name="outboard_stitch",
    )
    visor_panel.visual(
        Box((0.200, 0.095, 0.0010)),
        origin=Origin(xyz=(0.235, -0.093, -0.0378)),
        material=dark_plastic_mat,
        name="mirror_recess_shadow",
    )
    visor_panel.visual(
        Box((0.152, 0.052, 0.0008)),
        origin=Origin(xyz=(0.235, -0.096, -0.0381)),
        material=mirror_mat,
        name="mirror_glass",
    )

    mirror_cover = model.part("mirror_cover")
    mirror_cover.visual(
        mesh_from_cadquery(_rounded_flap(COVER_WIDTH, COVER_HEIGHT, COVER_THICKNESS, 0.006), "vanity_cover", tolerance=0.0006),
        origin=Origin(xyz=(0.0, -COVER_HEIGHT / 2.0, 0.0)),
        material=visor_mat,
        name="cover_panel",
    )
    mirror_cover.visual(
        Cylinder(radius=0.0025, length=0.170),
        origin=Origin(xyz=(0.0, 0.002, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_mat,
        name="cover_hinge_barrel",
    )
    mirror_cover.visual(
        Box((0.045, 0.006, 0.0012)),
        origin=Origin(xyz=(0.0, -COVER_HEIGHT + 0.006, -0.0022)),
        material=dark_plastic_mat,
        name="finger_pull_notch",
    )

    model.articulation(
        "roof_to_bracket",
        ArticulationType.FIXED,
        parent=roof_liner,
        child=roof_bracket,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
    )
    model.articulation(
        "roof_to_clip",
        ArticulationType.FIXED,
        parent=roof_liner,
        child=side_clip,
        origin=Origin(xyz=(0.458, 0.000, 0.000)),
    )
    model.articulation(
        "secondary_pivot",
        ArticulationType.REVOLUTE,
        parent=roof_bracket,
        child=hinge_rod,
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "main_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_rod,
        child=visor_panel,
        origin=Origin(xyz=(0.0, 0.0, ROD_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=visor_panel,
        child=mirror_cover,
        origin=Origin(xyz=(0.235, -0.055, -0.0408)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    roof = object_model.get_part("roof_liner")
    bracket = object_model.get_part("roof_bracket")
    clip = object_model.get_part("side_clip")
    rod = object_model.get_part("hinge_rod")
    panel = object_model.get_part("visor_panel")
    cover = object_model.get_part("mirror_cover")

    secondary = object_model.get_articulation("secondary_pivot")
    main = object_model.get_articulation("main_hinge")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.expect_contact(
        roof,
        bracket,
        elem_a="headliner_patch",
        elem_b="bracket_base",
        contact_tol=0.001,
        name="roof bracket is mounted to headliner",
    )
    ctx.expect_contact(
        roof,
        clip,
        elem_a="headliner_patch",
        elem_b="clip_roof_pad",
        contact_tol=0.001,
        name="side clip is mounted to headliner",
    )
    ctx.expect_within(
        rod,
        clip,
        axes="yz",
        inner_elem="rod_core",
        margin=0.002,
        name="clip surrounds free end of visor rod",
    )
    ctx.expect_overlap(
        rod,
        clip,
        axes="x",
        elem_a="rod_core",
        elem_b="clip_jaw_0",
        min_overlap=0.012,
        name="rod passes through clip capture zone",
    )
    ctx.expect_within(
        cover,
        panel,
        axes="xy",
        inner_elem="cover_panel",
        outer_elem="visor_body",
        margin=0.002,
        name="vanity cover is inset within visor face",
    )
    ctx.expect_gap(
        panel,
        cover,
        axis="z",
        positive_elem="mirror_recess_shadow",
        negative_elem="cover_panel",
        min_gap=0.0002,
        max_gap=0.004,
        name="cover sits just proud of recessed mirror pocket without penetrating",
    )

    closed_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({main: 1.25}):
        lowered_panel_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "main hinge lowers visor panel",
        closed_panel_aabb is not None
        and lowered_panel_aabb is not None
        and lowered_panel_aabb[0][2] < closed_panel_aabb[0][2] - 0.050,
        details=f"closed={closed_panel_aabb}, lowered={lowered_panel_aabb}",
    )

    closed_panel_aabb_for_pivot = ctx.part_world_aabb(panel)
    with ctx.pose({secondary: 1.0}):
        side_panel_aabb = ctx.part_world_aabb(panel)
    closed_panel_y_center = None
    side_panel_y_center = None
    if closed_panel_aabb_for_pivot is not None:
        closed_panel_y_center = (closed_panel_aabb_for_pivot[0][1] + closed_panel_aabb_for_pivot[1][1]) / 2.0
    if side_panel_aabb is not None:
        side_panel_y_center = (side_panel_aabb[0][1] + side_panel_aabb[1][1]) / 2.0
    ctx.check(
        "secondary pivot swings visor sideways",
        closed_panel_y_center is not None
        and side_panel_y_center is not None
        and side_panel_y_center > closed_panel_y_center + 0.080,
        details=f"closed={closed_panel_aabb_for_pivot}, side={side_panel_aabb}",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.0}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "vanity cover flips out from panel face",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][2] < closed_cover_aabb[0][2] - 0.020,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
