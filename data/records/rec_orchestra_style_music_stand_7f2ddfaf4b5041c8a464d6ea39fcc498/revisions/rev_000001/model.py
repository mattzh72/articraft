from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_THICKNESS = 0.018
BASE_BOSS_HEIGHT = 0.024
LOWER_TUBE_HEIGHT = 0.620
MAST_PIVOT_Z = 0.570
MAST_TRAVEL = 0.300
DESK_WIDTH = 0.500
DESK_HEIGHT = 0.340
DESK_THICKNESS = 0.006
DESK_ANGLE = 1.31
PANEL_CENTER = (0.0, -0.035, 0.180)


def _rotate_x(v: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = v
    c = cos(angle)
    s = sin(angle)
    return (x, c * y - s * z, s * y + c * z)


def _desk_local_to_head(v: tuple[float, float, float]) -> tuple[float, float, float]:
    rx, ry, rz = _rotate_x(v, DESK_ANGLE)
    return (PANEL_CENTER[0] + rx, PANEL_CENTER[1] + ry, PANEL_CENTER[2] + rz)


def _build_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .rect(0.380, 0.300)
        .extrude(BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.030)
    )
    boss = cq.Workplane("XY").circle(0.050).extrude(BASE_BOSS_HEIGHT).translate(
        (0.0, 0.0, BASE_THICKNESS)
    )
    return plate.union(boss)


def _build_lower_tube_shape() -> cq.Workplane:
    main_tube = cq.Workplane("XY").circle(0.024).circle(0.016).extrude(LOWER_TUBE_HEIGHT)
    collar = cq.Workplane("XY").circle(0.034).circle(0.016).extrude(0.055).translate(
        (0.0, 0.0, LOWER_TUBE_HEIGHT - 0.055)
    )
    lower_flange = cq.Workplane("XY").circle(0.033).circle(0.017).extrude(0.026)
    return main_tube.union(collar).union(lower_flange)


def _build_desk_geometry():
    geom = PerforatedPanelGeometry(
        (DESK_WIDTH, DESK_HEIGHT),
        DESK_THICKNESS,
        hole_diameter=0.010,
        pitch=(0.026, 0.024),
        frame=0.033,
        corner_radius=0.016,
        stagger=True,
    )

    # Raised sheet-metal rim and a shallow bottom ledge are merged into the same
    # mesh so the desk reads as one punched-metal tray instead of loose bars.
    rim_z = DESK_THICKNESS * 0.5 + 0.006
    geom.merge(BoxGeometry((DESK_WIDTH + 0.018, 0.020, 0.016)).translate(0.0, DESK_HEIGHT * 0.5 - 0.010, rim_z))
    geom.merge(BoxGeometry((DESK_WIDTH + 0.018, 0.034, 0.036)).translate(0.0, -DESK_HEIGHT * 0.5 + 0.017, DESK_THICKNESS * 0.5 + 0.018))
    geom.merge(BoxGeometry((0.020, DESK_HEIGHT, 0.014)).translate(-DESK_WIDTH * 0.5 + 0.010, 0.0, DESK_THICKNESS * 0.5 + 0.005))
    geom.merge(BoxGeometry((0.020, DESK_HEIGHT, 0.014)).translate(DESK_WIDTH * 0.5 - 0.010, 0.0, DESK_THICKNESS * 0.5 + 0.005))
    return geom


def _clip_blade_origin() -> Origin:
    # The blade lies just proud of the desk's front face and points down from
    # its hinge barrel at rest.
    x, y, z = _rotate_x((0.0, -0.047, 0.022), DESK_ANGLE)
    return Origin(xyz=(x, y, z), rpy=(DESK_ANGLE, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="perforated_rehearsal_music_stand")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.012, 1.0))
    satin_black = model.material("satin_black", rgba=(0.03, 0.032, 0.033, 1.0))
    dark_sheet = model.material("dark_punched_sheet", rgba=(0.09, 0.10, 0.10, 1.0))
    worn_edge = model.material("worn_black_edge", rgba=(0.02, 0.02, 0.018, 1.0))
    zinc = model.material("zinc_pins", rgba=(0.65, 0.66, 0.62, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "weighted_base", tolerance=0.001),
        material=matte_black,
        name="weighted_base",
    )

    lower_tube = model.part("lower_tube")
    lower_tube.visual(
        mesh_from_cadquery(_build_lower_tube_shape(), "lower_tube_shell", tolerance=0.001),
        material=satin_black,
        name="lower_tube_shell",
    )

    model.articulation(
        "base_to_lower_tube",
        ArticulationType.FIXED,
        parent=base,
        child=lower_tube,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + BASE_BOSS_HEIGHT)),
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0162, length=1.000),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=satin_black,
        name="inner_post",
    )
    mast.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.132, 0.052, 0.082),
                span_width=0.078,
                trunnion_diameter=0.020,
                trunnion_center_z=0.050,
                base_thickness=0.014,
                corner_radius=0.004,
                center=False,
            ),
            "tilt_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, MAST_PIVOT_Z - 0.050)),
        material=worn_edge,
        name="tilt_yoke",
    )

    model.articulation(
        "tube_to_mast",
        ArticulationType.PRISMATIC,
        parent=lower_tube,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TUBE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=MAST_TRAVEL),
    )

    desk_head = model.part("desk_head")
    desk_head.visual(
        mesh_from_geometry(_build_desk_geometry(), "punched_desk"),
        origin=Origin(xyz=PANEL_CENTER, rpy=(DESK_ANGLE, 0.0, 0.0)),
        material=dark_sheet,
        name="punched_desk",
    )
    desk_head.visual(
        Cylinder(radius=0.0102, length=0.126),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=zinc,
        name="tilt_pin",
    )
    desk_head.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=worn_edge,
        name="tilt_barrel",
    )
    desk_head.visual(
        Box((0.064, 0.110, 0.050)),
        origin=Origin(xyz=(0.0, -0.045, 0.025)),
        material=worn_edge,
        name="tilt_tab",
    )

    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk_head,
        origin=Origin(xyz=(0.0, 0.0, MAST_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=-0.45, upper=0.55),
    )

    clip_pivots = [
        (-0.155, *_desk_local_to_head((0.0, DESK_HEIGHT * 0.5 + 0.005, 0.019))[1:]),
        (0.155, *_desk_local_to_head((0.0, DESK_HEIGHT * 0.5 + 0.005, 0.019))[1:]),
    ]
    for idx, pivot_xyz in enumerate(clip_pivots):
        clip = model.part(f"page_clip_{idx}")
        clip.visual(
            Cylinder(radius=0.0065, length=0.055),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=zinc,
            name="pivot_barrel",
        )
        clip.visual(
            Box((0.045, 0.080, 0.006)),
            origin=_clip_blade_origin(),
            material=zinc,
            name="spring_blade",
        )
        clip.visual(
            Box((0.043, 0.030, 0.018)),
            origin=Origin(xyz=_rotate_x((0.0, -0.006, 0.010), DESK_ANGLE), rpy=(DESK_ANGLE, 0.0, 0.0)),
            material=zinc,
            name="blade_root",
        )
        model.articulation(
            f"desk_to_clip_{idx}",
            ArticulationType.REVOLUTE,
            parent=desk_head,
            child=clip,
            origin=Origin(xyz=pivot_xyz),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_tube = object_model.get_part("lower_tube")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk_head")
    clip_0 = object_model.get_part("page_clip_0")
    clip_1 = object_model.get_part("page_clip_1")
    tube_to_mast = object_model.get_articulation("tube_to_mast")
    mast_to_desk = object_model.get_articulation("mast_to_desk")
    desk_to_clip_0 = object_model.get_articulation("desk_to_clip_0")

    ctx.expect_within(
        mast,
        lower_tube,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="lower_tube_shell",
        margin=0.002,
        name="inner post is centered in lower tube",
    )
    ctx.allow_overlap(
        lower_tube,
        mast,
        elem_a="lower_tube_shell",
        elem_b="inner_post",
        reason="The inner post is intentionally modeled as a snug telescoping member bearing inside the lower tube bushing.",
    )
    ctx.expect_overlap(
        mast,
        lower_tube,
        axes="z",
        elem_a="inner_post",
        elem_b="lower_tube_shell",
        min_overlap=0.35,
        name="collapsed mast remains deeply inserted",
    )

    with ctx.pose({tube_to_mast: MAST_TRAVEL}):
        ctx.expect_within(
            mast,
            lower_tube,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="lower_tube_shell",
            margin=0.002,
            name="extended post stays centered in tube",
        )
        ctx.expect_overlap(
            mast,
            lower_tube,
            axes="z",
            elem_a="inner_post",
            elem_b="lower_tube_shell",
            min_overlap=0.12,
            name="extended mast keeps retained insertion",
        )

    ctx.expect_overlap(desk, mast, axes="x", min_overlap=0.050, name="desk trunnion is captured by bracket span")
    ctx.expect_overlap(
        desk,
        mast,
        axes="x",
        elem_a="tilt_pin",
        elem_b="tilt_yoke",
        min_overlap=0.10,
        name="tilt pin spans the yoke cheeks",
    )
    ctx.allow_overlap(
        desk,
        mast,
        elem_a="tilt_pin",
        elem_b="tilt_yoke",
        reason="The tilt pin is intentionally modeled as a close captured fit through the yoke bores.",
    )
    ctx.expect_overlap(clip_0, desk, axes="x", min_overlap=0.035, name="first clip sits on top rim")
    ctx.expect_overlap(clip_1, desk, axes="x", min_overlap=0.035, name="second clip sits on top rim")
    ctx.allow_overlap(
        desk,
        clip_0,
        elem_a="punched_desk",
        elem_b="pivot_barrel",
        reason="The page clip barrel is intentionally seated into the desk rim as a short captured pivot.",
    )
    ctx.allow_overlap(
        desk,
        clip_1,
        elem_a="punched_desk",
        elem_b="pivot_barrel",
        reason="The page clip barrel is intentionally seated into the desk rim as a short captured pivot.",
    )

    closed_desk_aabb = ctx.part_world_aabb(desk)
    with ctx.pose({mast_to_desk: 0.55}):
        tilted_desk_aabb = ctx.part_world_aabb(desk)
    ctx.check(
        "desk tilt changes the desk envelope",
        closed_desk_aabb is not None
        and tilted_desk_aabb is not None
        and abs(tilted_desk_aabb[0][1] - closed_desk_aabb[0][1]) > 0.050,
        details=f"closed={closed_desk_aabb}, tilted={tilted_desk_aabb}",
    )

    closed_clip_aabb = ctx.part_world_aabb(clip_0)
    with ctx.pose({desk_to_clip_0: 1.05}):
        lifted_clip_aabb = ctx.part_world_aabb(clip_0)
    ctx.check(
        "page clip flips on its short pivot",
        closed_clip_aabb is not None
        and lifted_clip_aabb is not None
        and abs(lifted_clip_aabb[0][2] - closed_clip_aabb[0][2]) > 0.015,
        details=f"closed={closed_clip_aabb}, lifted={lifted_clip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
