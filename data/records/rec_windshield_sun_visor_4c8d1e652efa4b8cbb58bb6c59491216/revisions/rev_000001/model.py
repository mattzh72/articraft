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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PANEL_WIDTH = 0.360
PANEL_DEPTH = 0.160
PANEL_THICKNESS = 0.026
PANEL_CENTER_Z = -0.017
PANEL_TOP_GAP = 0.004
MIRROR_POCKET_WIDTH = 0.150
MIRROR_POCKET_DEPTH = 0.090
MIRROR_POCKET_CUT = 0.0045
MIRROR_COVER_WIDTH = 0.146
MIRROR_COVER_DEPTH = 0.086
MIRROR_COVER_THICKNESS = 0.0024
ROD_AXIS_Z = -0.008


def _panel_shell_shape():
    pocket_center_y_local = 0.012
    pocket_center_z_local = (-PANEL_THICKNESS * 0.5) + (MIRROR_POCKET_CUT * 0.5)

    shell = (
        cq.Workplane("XY")
        .box(PANEL_WIDTH, PANEL_DEPTH, PANEL_THICKNESS)
        .edges("|Z")
        .fillet(0.022)
    )
    pocket = (
        cq.Workplane("XY")
        .box(MIRROR_POCKET_WIDTH, MIRROR_POCKET_DEPTH, MIRROR_POCKET_CUT)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, pocket_center_y_local, pocket_center_z_local))
    )
    return shell.cut(pocket)


def _cover_shape():
    return (
        cq.Workplane("XY")
        .box(MIRROR_COVER_WIDTH, MIRROR_COVER_DEPTH, MIRROR_COVER_THICKNESS)
        .edges("|Z")
        .fillet(0.010)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_sun_visor")

    headliner = model.material("headliner", rgba=(0.78, 0.76, 0.71, 1.0))
    bracket_plastic = model.material("bracket_plastic", rgba=(0.58, 0.57, 0.54, 1.0))
    clip_plastic = model.material("clip_plastic", rgba=(0.62, 0.60, 0.56, 1.0))
    visor_vinyl = model.material("visor_vinyl", rgba=(0.73, 0.71, 0.66, 1.0))
    cover_plastic = model.material("cover_plastic", rgba=(0.69, 0.67, 0.62, 1.0))
    rod_metal = model.material("rod_metal", rgba=(0.35, 0.36, 0.38, 1.0))

    headliner_part = model.part("headliner")
    headliner_part.visual(
        Box((0.460, 0.070, 0.012)),
        origin=Origin(xyz=(0.170, 0.000, 0.021)),
        material=headliner,
        name="headliner_pad",
    )

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.062, 0.040, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=bracket_plastic,
        name="roof_pad",
    )
    bracket.visual(
        Box((0.012, 0.020, 0.020)),
        origin=Origin(xyz=(-0.012, 0.000, 0.002)),
        material=bracket_plastic,
        name="side_cheek_0",
    )
    bracket.visual(
        Box((0.012, 0.020, 0.020)),
        origin=Origin(xyz=(0.012, 0.000, 0.002)),
        material=bracket_plastic,
        name="side_cheek_1",
    )
    bracket.visual(
        Box((0.036, 0.010, 0.010)),
        origin=Origin(xyz=(0.000, -0.010, 0.006)),
        material=bracket_plastic,
        name="rear_web",
    )

    clip = model.part("clip")
    clip.visual(
        Box((0.026, 0.022, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=clip_plastic,
        name="clip_base",
    )
    clip.visual(
        Box((0.010, 0.012, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=clip_plastic,
        name="clip_post",
    )
    clip.visual(
        Box((0.022, 0.006, 0.006)),
        origin=Origin(xyz=(0.000, 0.005, -0.005)),
        material=clip_plastic,
        name="clip_upper_jaw",
    )
    clip.visual(
        Box((0.022, 0.006, 0.005)),
        origin=Origin(xyz=(0.000, -0.004, -0.0073)),
        material=clip_plastic,
        name="clip_lower_jaw",
    )

    hinge_rod = model.part("hinge_rod")
    hinge_rod.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=rod_metal,
        name="pivot_stem",
    )
    hinge_rod.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        material=rod_metal,
        name="pivot_collar",
    )
    hinge_rod.visual(
        Box((0.032, 0.018, 0.010)),
        origin=Origin(xyz=(0.016, 0.000, ROD_AXIS_Z)),
        material=rod_metal,
        name="elbow_block",
    )
    hinge_rod.visual(
        Cylinder(radius=0.005, length=0.275),
        origin=Origin(xyz=(0.1675, 0.000, ROD_AXIS_Z), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=rod_metal,
        name="rod_shaft",
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        mesh_from_cadquery(_panel_shell_shape(), "visor_panel_shell"),
        origin=Origin(xyz=(0.000, PANEL_DEPTH * 0.5, PANEL_CENTER_Z)),
        material=visor_vinyl,
        name="panel_shell",
    )

    mirror_cover = model.part("mirror_cover")
    mirror_cover.visual(
        mesh_from_cadquery(_cover_shape(), "visor_cover"),
        origin=Origin(
            xyz=(
                0.000,
                MIRROR_COVER_DEPTH * 0.5,
                MIRROR_COVER_THICKNESS * 0.5,
            )
        ),
        material=cover_plastic,
        name="cover",
    )

    model.articulation(
        "headliner_to_bracket",
        ArticulationType.FIXED,
        parent=headliner_part,
        child=bracket,
        origin=Origin(),
    )
    model.articulation(
        "headliner_to_clip",
        ArticulationType.FIXED,
        parent=headliner_part,
        child=clip,
        origin=Origin(xyz=(0.338, 0.000, 0.000)),
    )
    model.articulation(
        "rod_swivel",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=hinge_rod,
        origin=Origin(),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.000,
            upper=1.45,
        ),
    )
    model.articulation(
        "panel_drop",
        ArticulationType.REVOLUTE,
        parent=hinge_rod,
        child=visor_panel,
        origin=Origin(xyz=(0.176, 0.000, ROD_AXIS_Z)),
        axis=(-1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.2,
            lower=0.000,
            upper=1.55,
        ),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=visor_panel,
        child=mirror_cover,
        origin=Origin(xyz=(0.000, 0.047, -0.0286)),
        axis=(-1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=0.000,
            upper=2.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headliner = object_model.get_part("headliner")
    visor_panel = object_model.get_part("visor_panel")
    clip = object_model.get_part("clip")
    mirror_cover = object_model.get_part("mirror_cover")

    rod_swivel = object_model.get_articulation("rod_swivel")
    panel_drop = object_model.get_articulation("panel_drop")
    cover_hinge = object_model.get_articulation("cover_hinge")

    with ctx.pose({rod_swivel: 0.0, panel_drop: 0.0, cover_hinge: 0.0}):
        ctx.expect_gap(
            headliner,
            visor_panel,
            axis="z",
            positive_elem="headliner_pad",
            negative_elem="panel_shell",
            min_gap=0.010,
            max_gap=0.030,
            name="closed visor panel sits just below the headliner",
        )
        ctx.expect_within(
            mirror_cover,
            visor_panel,
            axes="xy",
            inner_elem="cover",
            outer_elem="panel_shell",
            margin=0.008,
            name="mirror cover stays within the visor panel footprint",
        )
        ctx.expect_gap(
            clip,
            visor_panel,
            axis="z",
            positive_elem="clip_lower_jaw",
            negative_elem="panel_shell",
            min_gap=0.002,
            max_gap=0.025,
            name="retaining clip hovers just above the stowed visor edge",
        )

    closed_panel_aabb = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")
    with ctx.pose({panel_drop: 1.30}):
        opened_panel_aabb = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")
    ctx.check(
        "visor panel rotates downward",
        closed_panel_aabb is not None
        and opened_panel_aabb is not None
        and opened_panel_aabb[0][2] < closed_panel_aabb[0][2] - 0.080,
        details=f"closed={closed_panel_aabb}, opened={opened_panel_aabb}",
    )

    rest_panel_pos = ctx.part_world_position(visor_panel)
    with ctx.pose({rod_swivel: 1.20}):
        swung_panel_pos = ctx.part_world_position(visor_panel)
    ctx.check(
        "visor panel swings sideways at the bracket",
        rest_panel_pos is not None
        and swung_panel_pos is not None
        and swung_panel_pos[1] > rest_panel_pos[1] + 0.120,
        details=f"rest={rest_panel_pos}, swung={swung_panel_pos}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(mirror_cover, elem="cover")
    with ctx.pose({cover_hinge: 1.80}):
        opened_cover_aabb = ctx.part_element_world_aabb(mirror_cover, elem="cover")
    ctx.check(
        "mirror cover opens away from the visor face",
        closed_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[0][2] < closed_cover_aabb[0][2] - 0.020,
        details=f"closed={closed_cover_aabb}, opened={opened_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
