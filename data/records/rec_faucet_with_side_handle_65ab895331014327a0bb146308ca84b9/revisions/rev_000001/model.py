from __future__ import annotations

import math

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


BASE_POST_CENTER_Z = 0.060
SPOUT_TUBE_LIFT = 0.009
SOCKET_LENGTH = 0.034
HEAD_TRAVEL = 0.105
SLIDER_TRAVEL = 0.008

SPOUT_TUBE_END = (0.148, 0.158)
SPOUT_LIP = (0.1744, 0.1365)
NOZZLE_VECTOR_XZ = (
    SPOUT_LIP[0] - SPOUT_TUBE_END[0],
    SPOUT_LIP[1] - SPOUT_TUBE_END[1],
)
NOZZLE_PITCH = math.atan2(NOZZLE_VECTOR_XZ[0], NOZZLE_VECTOR_XZ[1])


def _ring(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(length)
        .faces(">Z")
        .workplane()
        .hole(2.0 * inner_radius)
    )


def _base_shape() -> cq.Workplane:
    deck_flange = cq.Workplane("XY").circle(0.031).extrude(0.006)

    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .circle(0.022)
        .workplane(offset=0.040)
        .circle(0.018)
        .loft(combine=True)
    )
    swivel_post = (
        cq.Workplane("XY")
        .workplane(offset=0.046)
        .circle(0.012)
        .extrude(0.014)
    )
    lever_boss = (
        cq.Workplane("XY")
        .box(0.020, 0.016, 0.020)
        .translate((0.0, 0.023, 0.046))
    )
    return deck_flange.union(pedestal).union(swivel_post).union(lever_boss)


def _spout_path():
    return (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, 0.108)
        .threePointArc((0.055, 0.168), (0.118, 0.168))
        .lineTo(0.148, 0.144)
        .wire()
        .val()
    )


def _spout_arc_shape() -> cq.Workplane:
    path = _spout_path()
    tube = cq.Workplane("XY").circle(0.0135).circle(0.0112).sweep(path, isFrenet=True)
    return tube.translate((0.0, 0.0, SPOUT_TUBE_LIFT))


def _spout_collar_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.018).extrude(0.018)


def _outlet_socket_shape() -> cq.Workplane:
    socket = cq.Workplane("XY").circle(0.0138).extrude(0.036).translate((0.0, 0.0, -0.036))
    socket = socket.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), math.degrees(NOZZLE_PITCH))
    return socket.translate((SPOUT_LIP[0], 0.0, SPOUT_LIP[1]))


def _lever_shape() -> cq.Workplane:
    profile = (
        cq.Workplane("YZ")
        .moveTo(-0.003, -0.007)
        .lineTo(0.012, -0.007)
        .threePointArc((0.040, 0.008), (0.054, 0.018))
        .lineTo(0.050, 0.022)
        .threePointArc((0.020, 0.012), (-0.003, 0.005))
        .close()
    )
    blade = profile.extrude(0.010).translate((-0.005, 0.0, 0.0))
    cap = cq.Workplane("XZ").circle(0.010).extrude(0.006).translate((0.0, -0.003, 0.0))
    return blade.union(cap)


def _head_shell_shape() -> cq.Workplane:
    shoulder = cq.Workplane("XY").circle(0.0135).extrude(0.010)
    body = cq.Workplane("XY").circle(0.0130).extrude(0.074).translate((0.0, 0.0, 0.010))
    nose = cq.Workplane("XY").circle(0.0140).extrude(0.016).translate((0.0, 0.0, 0.076))
    shell = shoulder.union(body).union(nose)
    slot_cut = cq.Workplane("XY").box(0.010, 0.006, 0.022).translate((0.0, 0.011, 0.040))
    return shell.cut(slot_cut)


def _hose_stem_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.0085).extrude(0.008).translate((0.0, 0.0, -0.004))


def _spray_face_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.0105).extrude(0.003).translate((0.0, 0.0, 0.089))


def _slider_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(0.006, 0.005, 0.012)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prep_sink_faucet")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    satin = model.material("satin_black", rgba=(0.16, 0.17, 0.19, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base_body"), material=chrome, name="body")

    spout = model.part("spout")
    spout.visual(mesh_from_cadquery(_spout_collar_shape(), "spout_collar"), material=chrome, name="collar")
    spout.visual(mesh_from_cadquery(_spout_arc_shape(), "spout_arc"), material=chrome, name="tube")
    spout.visual(
        mesh_from_cadquery(_outlet_socket_shape(), "outlet_socket"),
        material=chrome,
        name="outlet_socket",
    )

    lever = model.part("lever")
    lever.visual(mesh_from_cadquery(_lever_shape(), "side_lever"), material=chrome, name="lever")

    spray_head = model.part("spray_head")
    spray_head.visual(
        mesh_from_cadquery(_head_shell_shape(), "spray_head_shell"),
        material=chrome,
        name="head_shell",
    )
    spray_head.visual(
        mesh_from_cadquery(_hose_stem_shape(), "spray_head_stem"),
        material=rubber,
        name="hose_stem",
    )
    spray_head.visual(
        mesh_from_cadquery(_spray_face_shape(), "spray_face"),
        material=satin,
        name="spray_face",
    )

    slider = model.part("mode_slider")
    slider.visual(
        mesh_from_cadquery(_slider_shape(), "mode_slider_tab"),
        material=satin,
        name="mode_tab",
    )

    model.articulation(
        "base_to_spout",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, BASE_POST_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )
    model.articulation(
        "base_to_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(0.0, 0.040, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.25, upper=0.9),
    )
    model.articulation(
        "spout_to_spray_head",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray_head,
        origin=Origin(
            xyz=(
                SPOUT_LIP[0] + 0.004 * math.sin(NOZZLE_PITCH),
                0.0,
                SPOUT_LIP[1] + 0.004 * math.cos(NOZZLE_PITCH),
            ),
            rpy=(0.0, NOZZLE_PITCH, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.25, lower=0.0, upper=HEAD_TRAVEL),
    )
    model.articulation(
        "spray_head_to_mode_slider",
        ArticulationType.PRISMATIC,
        parent=spray_head,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0105, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.04,
            lower=-SLIDER_TRAVEL / 2.0,
            upper=SLIDER_TRAVEL / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("lever")
    spray_head = object_model.get_part("spray_head")
    slider = object_model.get_part("mode_slider")

    spout_swivel = object_model.get_articulation("base_to_spout")
    lever_pivot = object_model.get_articulation("base_to_lever")
    head_slide = object_model.get_articulation("spout_to_spray_head")
    mode_slider = object_model.get_articulation("spray_head_to_mode_slider")

    ctx.allow_overlap(
        spout,
        spray_head,
        elem_a="outlet_socket",
        elem_b="hose_stem",
        reason="The short hose connector is intentionally represented as slightly seated inside the solid outlet-socket proxy.",
    )

    ctx.expect_contact(
        spray_head,
        spout,
        elem_a="hose_stem",
        elem_b="outlet_socket",
        contact_tol=0.0005,
        name="spray head connector seats at the spout outlet",
    )

    rest_head_pos = ctx.part_world_position(spray_head)
    with ctx.pose({head_slide: head_slide.motion_limits.upper}):
        extended_head_pos = ctx.part_world_position(spray_head)
    ctx.check(
        "spray head pulls forward and downward",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[0] > rest_head_pos[0] + 0.06
        and extended_head_pos[2] < rest_head_pos[2] - 0.04,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({mode_slider: mode_slider.motion_limits.upper}):
        shifted_slider_pos = ctx.part_world_position(slider)
    ctx.check(
        "spray mode slider travels in its top slot",
        rest_slider_pos is not None
        and shifted_slider_pos is not None
        and shifted_slider_pos[0] > rest_slider_pos[0] + 0.002
        and shifted_slider_pos[2] < rest_slider_pos[2] - 0.0015,
        details=f"rest={rest_slider_pos}, shifted={shifted_slider_pos}",
    )

    rest_lever_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({lever_pivot: lever_pivot.motion_limits.upper}):
        raised_lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "side lever lifts upward on its pivot",
        rest_lever_aabb is not None
        and raised_lever_aabb is not None
        and raised_lever_aabb[1][2] > rest_lever_aabb[1][2] + 0.02,
        details=f"rest={rest_lever_aabb}, raised={raised_lever_aabb}",
    )

    rest_spout_pos = ctx.part_world_position(spray_head)
    with ctx.pose({spout_swivel: math.pi / 2.0}):
        swiveled_spout_pos = ctx.part_world_position(spray_head)
    ctx.check(
        "spout swivels around the base axis",
        rest_spout_pos is not None
        and swiveled_spout_pos is not None
        and swiveled_spout_pos[1] > rest_spout_pos[1] + 0.12
        and abs(swiveled_spout_pos[0]) < rest_spout_pos[0],
        details=f"rest={rest_spout_pos}, swiveled={swiveled_spout_pos}",
    )

    ctx.check(
        "spout sits above the base post",
        ctx.part_world_position(spout) is not None
        and ctx.part_world_position(base) is not None
        and ctx.part_world_position(spout)[2] > ctx.part_world_position(base)[2] + 0.04,
        details=f"base={ctx.part_world_position(base)}, spout={ctx.part_world_position(spout)}",
    )

    return ctx.report()


object_model = build_object_model()
