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


def _rounded_rect_prism(length: float, depth: float, thickness: float, radius: float):
    """A rounded-corner, cushion-like rectangular prism centered on the origin."""
    return (
        cq.Workplane("XY")
        .box(length, depth, thickness)
        .edges("|Z")
        .fillet(radius)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_sun_visor")

    headliner_mat = model.material("warm_grey_headliner", color=(0.66, 0.65, 0.61, 1.0))
    visor_mat = model.material("padded_greige_vinyl", color=(0.76, 0.70, 0.61, 1.0))
    seam_mat = model.material("slightly_darker_recess", color=(0.50, 0.46, 0.39, 1.0))
    hardware_mat = model.material("dark_plastic_hardware", color=(0.07, 0.075, 0.075, 1.0))
    satin_metal_mat = model.material("satin_black_metal", color=(0.02, 0.02, 0.018, 1.0))
    mirror_mat = model.material("dim_mirror_glass", color=(0.70, 0.76, 0.78, 0.78))

    headliner = model.part("headliner")
    headliner.visual(
        Box((0.56, 0.28, 0.012)),
        origin=Origin(xyz=(0.21, 0.070, 0.006)),
        material=headliner_mat,
        name="headliner_pad",
    )
    headliner.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=hardware_mat,
        name="bracket_rosette",
    )
    headliner.visual(
        Box((0.038, 0.007, 0.030)),
        origin=Origin(xyz=(0.0, -0.014, -0.023)),
        material=hardware_mat,
        name="bracket_ear_0",
    )
    headliner.visual(
        Box((0.038, 0.007, 0.030)),
        origin=Origin(xyz=(0.0, 0.014, -0.023)),
        material=hardware_mat,
        name="bracket_ear_1",
    )
    headliner.visual(
        Box((0.044, 0.034, 0.008)),
        origin=Origin(xyz=(0.410, 0.0, -0.004)),
        material=hardware_mat,
        name="clip_base",
    )
    headliner.visual(
        Box((0.012, 0.006, 0.024)),
        origin=Origin(xyz=(0.410, -0.012, -0.020)),
        material=hardware_mat,
        name="clip_jaw_0",
    )
    headliner.visual(
        Box((0.012, 0.006, 0.024)),
        origin=Origin(xyz=(0.410, 0.012, -0.020)),
        material=hardware_mat,
        name="clip_jaw_1",
    )

    hinge_rod = model.part("hinge_rod")
    hinge_rod.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_metal_mat,
        name="vertical_pivot_pin",
    )
    hinge_rod.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.014, 0.0, -0.014)),
        material=satin_metal_mat,
        name="elbow_boss",
    )
    hinge_rod.visual(
        Cylinder(radius=0.006, length=0.374),
        origin=Origin(xyz=(0.207, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal_mat,
        name="horizontal_rod",
    )
    hinge_rod.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.397, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_mat,
        name="clip_end_cap",
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        mesh_from_cadquery(
            _rounded_rect_prism(0.405, 0.165, 0.024, 0.030),
            "padded_visor_panel",
            tolerance=0.0007,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.205, 0.090, -0.020)),
        material=visor_mat,
        name="panel_shell",
    )
    visor_panel.visual(
        Box((0.360, 0.008, 0.006)),
        origin=Origin(xyz=(0.215, 0.010, -0.011)),
        material=visor_mat,
        name="hinge_padded_flange",
    )
    visor_panel.visual(
        Cylinder(radius=0.0035, length=0.350),
        origin=Origin(xyz=(0.215, 0.006, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=visor_mat,
        name="hinge_roll_edge",
    )
    # The mirror lies in a shallow inset on the face of the visor.  It is mostly
    # hidden when the cover is closed, but becomes visible in the opened pose.
    visor_panel.visual(
        Box((0.132, 0.052, 0.0012)),
        origin=Origin(xyz=(0.235, 0.086, -0.0326)),
        material=mirror_mat,
        name="mirror_glass",
    )
    visor_panel.visual(
        Box((0.190, 0.006, 0.003)),
        origin=Origin(xyz=(0.235, 0.040, -0.0328)),
        material=seam_mat,
        name="mirror_frame_top",
    )
    visor_panel.visual(
        Box((0.190, 0.006, 0.003)),
        origin=Origin(xyz=(0.235, 0.129, -0.0328)),
        material=seam_mat,
        name="mirror_frame_bottom",
    )
    visor_panel.visual(
        Box((0.006, 0.083, 0.003)),
        origin=Origin(xyz=(0.137, 0.0845, -0.0328)),
        material=seam_mat,
        name="mirror_frame_side_0",
    )
    visor_panel.visual(
        Box((0.006, 0.083, 0.003)),
        origin=Origin(xyz=(0.333, 0.0845, -0.0328)),
        material=seam_mat,
        name="mirror_frame_side_1",
    )
    visor_panel.visual(
        Box((0.012, 0.010, 0.006)),
        origin=Origin(xyz=(0.140, 0.045, -0.034)),
        material=seam_mat,
        name="cover_hinge_lug_0",
    )
    visor_panel.visual(
        Box((0.012, 0.010, 0.006)),
        origin=Origin(xyz=(0.330, 0.045, -0.034)),
        material=seam_mat,
        name="cover_hinge_lug_1",
    )
    visor_panel.visual(
        Cylinder(radius=0.003, length=0.016),
        origin=Origin(xyz=(0.140, 0.045, -0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seam_mat,
        name="cover_hinge_socket_0",
    )
    visor_panel.visual(
        Cylinder(radius=0.003, length=0.016),
        origin=Origin(xyz=(0.330, 0.045, -0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seam_mat,
        name="cover_hinge_socket_1",
    )

    mirror_cover = model.part("mirror_cover")
    mirror_cover.visual(
        mesh_from_cadquery(
            _rounded_rect_prism(0.170, 0.078, 0.004, 0.008),
            "vanity_mirror_cover",
            tolerance=0.0005,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.041, 0.0)),
        material=visor_mat,
        name="cover_plate",
    )
    mirror_cover.visual(
        Cylinder(radius=0.003, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seam_mat,
        name="cover_hinge_barrel",
    )

    model.articulation(
        "headliner_to_hinge_rod",
        ArticulationType.REVOLUTE,
        parent=headliner,
        child=hinge_rod,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "hinge_rod_to_visor_panel",
        ArticulationType.REVOLUTE,
        parent=hinge_rod,
        child=visor_panel,
        origin=Origin(xyz=(0.018, 0.0, -0.014)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "visor_panel_to_mirror_cover",
        ArticulationType.REVOLUTE,
        parent=visor_panel,
        child=mirror_cover,
        origin=Origin(xyz=(0.235, 0.045, -0.036)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel = object_model.get_part("visor_panel")
    rod = object_model.get_part("hinge_rod")
    cover = object_model.get_part("mirror_cover")
    hinge = object_model.get_articulation("hinge_rod_to_visor_panel")
    swivel = object_model.get_articulation("headliner_to_hinge_rod")
    cover_hinge = object_model.get_articulation("visor_panel_to_mirror_cover")

    ctx.expect_gap(
        rod,
        panel,
        axis="z",
        positive_elem="horizontal_rod",
        negative_elem="panel_shell",
        min_gap=0.001,
        max_gap=0.006,
        name="visor panel is visibly separate from hinge rod",
    )
    ctx.expect_within(
        cover,
        panel,
        axes="xy",
        inner_elem="cover_plate",
        outer_elem="panel_shell",
        margin=0.003,
        name="vanity cover sits within the visor panel footprint",
    )
    ctx.expect_gap(
        panel,
        cover,
        axis="z",
        positive_elem="panel_shell",
        negative_elem="cover_plate",
        min_gap=0.001,
        max_gap=0.006,
        name="vanity cover is a distinct inset flap below panel face",
    )

    rest_panel_aabb = ctx.part_world_aabb(panel)
    rest_cover_aabb = ctx.part_world_aabb(cover)
    rest_panel_aabb_for_swivel = ctx.part_world_aabb(panel)
    with ctx.pose({hinge: 1.15}):
        lowered_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({swivel: 0.75}):
        swiveled_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({cover_hinge: 1.0}):
        opened_cover_aabb = ctx.part_world_aabb(cover)

    ctx.check(
        "main visor rotates downward on the hinge rod",
        rest_panel_aabb is not None
        and lowered_panel_aabb is not None
        and lowered_panel_aabb[0][2] < rest_panel_aabb[0][2] - 0.08,
        details=f"rest={rest_panel_aabb}, lowered={lowered_panel_aabb}",
    )
    ctx.check(
        "visor swivels sideways at the roof bracket",
        rest_panel_aabb_for_swivel is not None
        and swiveled_panel_aabb is not None
        and (
            0.5 * (swiveled_panel_aabb[0][1] + swiveled_panel_aabb[1][1])
            > 0.5 * (rest_panel_aabb_for_swivel[0][1] + rest_panel_aabb_for_swivel[1][1]) + 0.10
        ),
        details=f"rest={rest_panel_aabb_for_swivel}, swiveled={swiveled_panel_aabb}",
    )
    ctx.check(
        "vanity cover opens away from the panel face",
        rest_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[0][2] < rest_cover_aabb[0][2] - 0.035,
        details=f"rest={rest_cover_aabb}, opened={opened_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
