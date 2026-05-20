from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ROOF_W = 0.62
ROOF_D = 0.22
ROOF_T = 0.012
ROOF_Y = -0.085

SWIVEL_X = 0.055
SWIVEL_Y = -0.020

BODY_W = 0.46
BODY_H = 0.17
BODY_T = 0.026
BODY_TOP_CLEARANCE = 0.004
BODY_TOP_SETBACK = 0.006
BODY_CENTER_Z = -(BODY_T / 2.0 + BODY_TOP_CLEARANCE)
BODY_FACE_Z = BODY_CENTER_Z - BODY_T / 2.0

HINGE_ARM_X = 0.040
HINGE_ARM_Z = 0.001

EXT_SLOT_L = 0.20
EXT_SLOT_H = 0.118
EXT_T = 0.014
EXT_TOTAL_L = 0.18
EXT_CENTER_Y = -0.095

COVER_W = 0.17
COVER_H = 0.082
COVER_T = 0.0045
COVER_RECESS_D = 0.0055
COVER_CENTER_X = 0.18
COVER_TOP_Y = -0.043
COVER_CENTER_Y = COVER_TOP_Y - COVER_H / 2.0


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def make_roof_panel() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(ROOF_W, ROOF_D, ROOF_T)
        .translate((ROOF_W / 2.0, ROOF_Y, ROOF_T / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

def make_hinge_bracket() -> cq.Workplane:
    bracket_base = (
        cq.Workplane("XY")
        .box(0.034, 0.028, 0.002)
        .translate((SWIVEL_X, SWIVEL_Y, -0.001))
        .edges("|Z")
        .fillet(0.003)
    )
    swivel_boss = (
        cq.Workplane("XY")
        .circle(0.010)
        .extrude(-0.002)
        .translate((SWIVEL_X, SWIVEL_Y, 0.0))
    )
    return bracket_base.union(swivel_boss)


def make_retaining_clip() -> cq.Workplane:
    clip_stem = (
        cq.Workplane("XY")
        .box(0.014, 0.024, 0.034)
        .translate((0.582, -0.030, -0.017))
    )
    clip_outer = (
        cq.Workplane("XY")
        .box(0.032, 0.030, 0.042)
        .translate((0.568, -0.030, -0.016))
    )
    clip_void = (
        cq.Workplane("XY")
        .box(0.026, 0.024, 0.028)
        .translate((0.560, -0.030, -0.017))
    )
    retaining_clip = clip_outer.cut(clip_void)

    return clip_stem.union(retaining_clip)


def make_pivot_arm() -> cq.Workplane:
    spindle = (
        cq.Workplane("XY")
        .circle(0.0065)
        .extrude(-0.014)
        .translate((0.0, 0.0, 0.0))
    )
    arm_beam = (
        cq.Workplane("XY")
        .box(0.042, 0.012, 0.009)
        .translate((0.029, 0.006, -0.004))
        .edges("|Z")
        .fillet(0.003)
    )
    hinge_rod = (
        cq.Workplane("YZ")
        .circle(0.0055)
        .extrude(HINGE_ARM_X)
        .translate((0.0, 0.0, HINGE_ARM_Z))
    )
    rod_end = (
        cq.Workplane("XY")
        .box(0.010, 0.018, 0.012)
        .translate((HINGE_ARM_X - 0.004, 0.0, -0.002))
        .edges("|Z")
        .fillet(0.003)
    )
    return spindle.union(arm_beam).union(hinge_rod).union(rod_end)


def make_visor_body() -> cq.Workplane:
    body = _rounded_box((BODY_W, BODY_H, BODY_T), 0.004).translate(
        (BODY_W / 2.0, -(BODY_H / 2.0 + BODY_TOP_SETBACK), BODY_CENTER_Z)
    )

    extender_slot = (
        cq.Workplane("XY")
        .box(EXT_SLOT_L, EXT_SLOT_H, BODY_T + 0.004)
        .translate((BODY_W - EXT_SLOT_L / 2.0, EXT_CENTER_Y, BODY_CENTER_Z))
    )
    mirror_recess = (
        cq.Workplane("XY")
        .box(COVER_W + 0.008, COVER_H + 0.008, COVER_RECESS_D)
        .translate((COVER_CENTER_X, COVER_CENTER_Y, BODY_FACE_Z + COVER_RECESS_D / 2.0))
    )

    return body.cut(extender_slot).cut(mirror_recess)


def make_extender() -> cq.Workplane:
    slide_panel = _rounded_box((EXT_TOTAL_L, EXT_SLOT_H - 0.010, EXT_T), 0.0035).translate(
        (-EXT_TOTAL_L / 2.0 + 0.002, 0.0, 0.0)
    )
    face_flange = (
        cq.Workplane("XY")
        .box(0.004, 0.136, EXT_T + 0.002)
        .translate((0.002, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.001)
    )
    return slide_panel.union(face_flange)


def make_mirror_cover() -> cq.Workplane:
    cover_panel = _rounded_box((COVER_W, COVER_H, COVER_T), 0.002).translate((0.0, -COVER_H / 2.0, COVER_T / 2.0))
    hinge_barrel_length = COVER_W * 0.78
    hinge_barrel = (
        cq.Workplane("YZ")
        .circle(0.0025)
        .extrude(hinge_barrel_length)
        .translate((-hinge_barrel_length / 2.0, 0.0, -0.0015))
    )
    finger_lip = (
        cq.Workplane("XY")
        .box(0.034, 0.008, 0.003)
        .translate((0.0, -(COVER_H + 0.001), -0.0010))
        .edges("|Z")
        .fillet(0.0015)
    )
    return cover_panel.union(hinge_barrel).union(finger_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vehicle_sun_visor")

    headliner = model.material("headliner", rgba=(0.82, 0.80, 0.76, 1.0))
    hardware = model.material("hardware", rgba=(0.16, 0.17, 0.18, 1.0))
    visor_vinyl = model.material("visor_vinyl", rgba=(0.76, 0.73, 0.67, 1.0))
    visor_trim = model.material("visor_trim", rgba=(0.71, 0.68, 0.62, 1.0))

    roof_panel = model.part("roof_panel")
    roof_panel.visual(
        mesh_from_cadquery(make_roof_panel(), "roof_panel"),
        material=headliner,
        name="roof_panel",
    )

    hinge_bracket = model.part("hinge_bracket")
    hinge_bracket.visual(
        mesh_from_cadquery(make_hinge_bracket(), "hinge_bracket"),
        material=headliner,
        name="hinge_bracket",
    )

    retaining_clip = model.part("retaining_clip")
    retaining_clip.visual(
        mesh_from_cadquery(make_retaining_clip(), "retaining_clip"),
        material=headliner,
        name="retaining_clip",
    )

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        mesh_from_cadquery(make_pivot_arm(), "pivot_arm"),
        material=hardware,
        name="pivot_arm",
    )

    visor_body = model.part("visor_body")
    visor_body.visual(
        mesh_from_cadquery(make_visor_body(), "visor_body"),
        material=visor_vinyl,
        name="visor_body",
    )

    extender = model.part("extender")
    extender.visual(
        mesh_from_cadquery(make_extender(), "extender"),
        material=visor_trim,
        name="extender",
    )

    mirror_cover = model.part("mirror_cover")
    mirror_cover.visual(
        mesh_from_cadquery(make_mirror_cover(), "mirror_cover"),
        material=visor_trim,
        name="mirror_cover",
    )

    model.articulation(
        "roof_panel_to_bracket",
        ArticulationType.FIXED,
        parent=roof_panel,
        child=hinge_bracket,
        origin=Origin(),
    )
    model.articulation(
        "roof_panel_to_clip",
        ArticulationType.FIXED,
        parent=roof_panel,
        child=retaining_clip,
        origin=Origin(),
    )
    model.articulation(
        "roof_swivel",
        ArticulationType.REVOLUTE,
        parent=hinge_bracket,
        child=pivot_arm,
        origin=Origin(xyz=(SWIVEL_X, SWIVEL_Y, -0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "roof_hinge",
        ArticulationType.REVOLUTE,
        parent=pivot_arm,
        child=visor_body,
        origin=Origin(xyz=(HINGE_ARM_X, 0.0, HINGE_ARM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "extender_slide",
        ArticulationType.PRISMATIC,
        parent=visor_body,
        child=extender,
        origin=Origin(xyz=(BODY_W, EXT_CENTER_Y, BODY_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.11, effort=12.0, velocity=0.25),
    )
    model.articulation(
        "mirror_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=visor_body,
        child=mirror_cover,
        origin=Origin(xyz=(COVER_CENTER_X, COVER_TOP_Y, BODY_FACE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.8, effort=2.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    roof_swivel = object_model.get_articulation("roof_swivel")
    roof_hinge = object_model.get_articulation("roof_hinge")
    extender_slide = object_model.get_articulation("extender_slide")
    mirror_cover_hinge = object_model.get_articulation("mirror_cover_hinge")

    visor_body = object_model.get_part("visor_body")
    extender = object_model.get_part("extender")
    mirror_cover = object_model.get_part("mirror_cover")

    ctx.expect_contact(
        mirror_cover,
        visor_body,
        name="mirror cover seats into the visor face when closed",
    )
    ctx.expect_within(
        extender,
        visor_body,
        axes="yz",
        margin=0.002,
        name="extender stays centered in the visor sleeve at rest",
    )
    ctx.expect_overlap(
        extender,
        visor_body,
        axes="x",
        min_overlap=0.16,
        name="collapsed extender remains substantially nested inside the visor body",
    )

    rest_body_aabb = ctx.part_world_aabb(visor_body)
    rest_swivel_pos = ctx.part_world_position(visor_body)
    rest_extender_pos = ctx.part_world_position(extender)
    rest_cover_aabb = ctx.part_world_aabb(mirror_cover)

    roof_hinge_limits = roof_hinge.motion_limits
    if roof_hinge_limits is not None and roof_hinge_limits.upper is not None:
        with ctx.pose({roof_hinge: roof_hinge_limits.upper}):
            opened_body_aabb = ctx.part_world_aabb(visor_body)
        ctx.check(
            "roof hinge lowers the visor into the cabin",
            rest_body_aabb is not None
            and opened_body_aabb is not None
            and opened_body_aabb[0][2] < rest_body_aabb[0][2] - 0.09,
            details=f"rest={rest_body_aabb}, opened={opened_body_aabb}",
        )

    roof_swivel_limits = roof_swivel.motion_limits
    if roof_swivel_limits is not None and roof_swivel_limits.upper is not None:
        with ctx.pose({roof_swivel: roof_swivel_limits.upper}):
            swivel_body_aabb = ctx.part_world_aabb(visor_body)
            swivel_body_pos = ctx.part_world_position(visor_body)
        ctx.check(
            "secondary pivot swings the visor toward the side window",
            rest_swivel_pos is not None
            and swivel_body_pos is not None
            and swivel_body_aabb is not None
            and rest_body_aabb is not None
            and abs(swivel_body_pos[1] - rest_swivel_pos[1]) > 0.03
            and abs(swivel_body_aabb[1][1] - rest_body_aabb[1][1]) > 0.10,
            details=f"rest_pos={rest_swivel_pos}, swivel_pos={swivel_body_pos}, rest_aabb={rest_body_aabb}, swivel_aabb={swivel_body_aabb}",
        )

    slide_limits = extender_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({extender_slide: slide_limits.upper}):
            ctx.expect_within(
                extender,
                visor_body,
                axes="yz",
                margin=0.002,
                name="extended visor extender stays centered in the sleeve",
            )
            ctx.expect_overlap(
                extender,
                visor_body,
                axes="x",
                min_overlap=0.065,
                name="extended visor extender keeps retained insertion",
            )
            extended_extender_pos = ctx.part_world_position(extender)
        ctx.check(
            "visor extender slides outward from the body",
            rest_extender_pos is not None
            and extended_extender_pos is not None
            and extended_extender_pos[0] > rest_extender_pos[0] + 0.08,
            details=f"rest={rest_extender_pos}, extended={extended_extender_pos}",
        )

    mirror_limits = mirror_cover_hinge.motion_limits
    if mirror_limits is not None and mirror_limits.upper is not None:
        with ctx.pose({mirror_cover_hinge: mirror_limits.upper}):
            open_cover_aabb = ctx.part_world_aabb(mirror_cover)
        ctx.check(
            "mirror cover opens outward away from the visor face",
            rest_cover_aabb is not None
            and open_cover_aabb is not None
            and open_cover_aabb[0][2] < rest_cover_aabb[0][2] - 0.03,
            details=f"rest={rest_cover_aabb}, open={open_cover_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
