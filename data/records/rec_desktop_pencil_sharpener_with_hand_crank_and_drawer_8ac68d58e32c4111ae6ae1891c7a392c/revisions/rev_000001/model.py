from __future__ import annotations

from math import pi

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


BODY_D = 0.112
BODY_W = 0.074
BODY_H = 0.096
BODY_FRONT_X = BODY_D / 2.0

TRAY_DEPTH = 0.060
TRAY_W = 0.058
TRAY_H = 0.028
TRAY_WALL = 0.002
TRAY_FRONT_T = 0.004
TRAY_TRAVEL = 0.032
TRAY_CAVITY_Z = 0.004
TRAY_OPENING_H = 0.035
TRAY_OPENING_W = 0.063

PORT_R = 0.0055
PORT_Z = 0.060
PORT_DEPTH = 0.048
PORT_BEZEL_R = 0.0095
PORT_BEZEL_LEN = 0.006

FLAP_W = 0.021
FLAP_H = 0.016
FLAP_T = 0.0016
FLAP_BARREL_R = 0.0018
FLAP_HINGE_X = BODY_FRONT_X + PORT_BEZEL_LEN - 0.001
FLAP_HINGE_Z = PORT_Z + PORT_R + 0.004
FLAP_OPEN = 1.20

CRANK_X = -0.006
CRANK_Z = 0.055
BUSH_R = 0.010
BUSH_LEN = 0.005
CRANK_JOINT_Y = (BODY_W / 2.0) + BUSH_LEN - 0.001
CRANK_HUB_R = 0.008
CRANK_HUB_LEN = 0.005
CRANK_ARM_R = 0.0026
CRANK_ARM_LEN = 0.034
CRANK_HANDLE_REACH = 0.018
CRANK_KNOB_R = 0.006
CRANK_KNOB_LEN = 0.014


def _cylinder_along_x(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length)


def _cylinder_along_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length)


def _build_body_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(BODY_D, BODY_W, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .faces(">Z")
        .edges()
        .fillet(0.014)
    )

    tray_cut_len = TRAY_DEPTH + 0.010
    tray_cut = (
        cq.Workplane("XY")
        .box(tray_cut_len, TRAY_OPENING_W, TRAY_OPENING_H, centered=(True, True, False))
        .translate((BODY_FRONT_X - (tray_cut_len / 2.0) + 0.003, 0.0, TRAY_CAVITY_Z))
    )

    port_bezel = _cylinder_along_x(PORT_BEZEL_R, PORT_BEZEL_LEN).translate(
        (BODY_FRONT_X - 0.001, 0.0, PORT_Z)
    )
    port_cut = _cylinder_along_x(PORT_R, PORT_DEPTH).translate(
        (BODY_FRONT_X - 0.002, 0.0, PORT_Z)
    )

    crank_bushing = _cylinder_along_y(BUSH_R, BUSH_LEN).translate(
        (CRANK_X, (BODY_W / 2.0) - 0.001, CRANK_Z)
    )

    return housing.union(port_bezel).union(crank_bushing).cut(tray_cut).cut(port_cut)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_pencil_sharpener")

    model.material("shell", rgba=(0.26, 0.29, 0.31, 1.0))
    model.material("tray", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("crank", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("knob", rgba=(0.88, 0.86, 0.80, 1.0))
    model.material("flap", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "sharpener_body"),
        material="shell",
        name="housing",
    )

    tray = model.part("tray")
    tray_front_h = TRAY_H + 0.004
    tray_bin_len = TRAY_DEPTH - TRAY_FRONT_T
    tray.visual(
        Box((TRAY_FRONT_T, TRAY_W, tray_front_h)),
        origin=Origin(xyz=(-TRAY_FRONT_T / 2.0, 0.0, tray_front_h / 2.0)),
        material="tray",
        name="tray_front",
    )
    tray.visual(
        Box((tray_bin_len, TRAY_W - (2.0 * TRAY_WALL), TRAY_WALL)),
        origin=Origin(
            xyz=(
                -TRAY_FRONT_T - (tray_bin_len / 2.0),
                0.0,
                TRAY_WALL / 2.0,
            )
        ),
        material="tray",
        name="tray_floor",
    )
    tray.visual(
        Box((tray_bin_len, TRAY_WALL, TRAY_H)),
        origin=Origin(
            xyz=(
                -TRAY_FRONT_T - (tray_bin_len / 2.0),
                (TRAY_W / 2.0) - (TRAY_WALL / 2.0),
                TRAY_H / 2.0,
            )
        ),
        material="tray",
        name="tray_wall_0",
    )
    tray.visual(
        Box((tray_bin_len, TRAY_WALL, TRAY_H)),
        origin=Origin(
            xyz=(
                -TRAY_FRONT_T - (tray_bin_len / 2.0),
                -(TRAY_W / 2.0) + (TRAY_WALL / 2.0),
                TRAY_H / 2.0,
            )
        ),
        material="tray",
        name="tray_wall_1",
    )
    tray.visual(
        Box((TRAY_WALL, TRAY_W - (2.0 * TRAY_WALL), TRAY_H)),
        origin=Origin(
            xyz=(
                -TRAY_DEPTH + (TRAY_WALL / 2.0),
                0.0,
                TRAY_H / 2.0,
            )
        ),
        material="tray",
        name="tray_back",
    )
    tray.visual(
        Box((0.003, 0.030, 0.010)),
        origin=Origin(xyz=(0.0015, 0.0, 0.015)),
        material="tray",
        name="tray_pull",
    )

    crank = model.part("crank")
    crank_y = CRANK_HUB_LEN / 2.0
    crank.visual(
        Cylinder(radius=CRANK_HUB_R, length=CRANK_HUB_LEN),
        origin=Origin(xyz=(0.0, crank_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="crank",
        name="hub",
    )
    crank.visual(
        Cylinder(radius=CRANK_ARM_R, length=CRANK_ARM_LEN),
        origin=Origin(
            xyz=(0.0, crank_y, -(CRANK_ARM_LEN / 2.0)),
        ),
        material="crank",
        name="arm",
    )
    crank.visual(
        Cylinder(radius=CRANK_ARM_R, length=CRANK_HANDLE_REACH),
        origin=Origin(
            xyz=(CRANK_HANDLE_REACH / 2.0, crank_y, -CRANK_ARM_LEN),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="crank",
        name="handle_stem",
    )
    crank.visual(
        Cylinder(radius=CRANK_KNOB_R, length=CRANK_KNOB_LEN),
        origin=Origin(
            xyz=(CRANK_HANDLE_REACH, crank_y, -CRANK_ARM_LEN),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="knob",
        name="handle_knob",
    )

    flap = model.part("flap")
    flap.visual(
        Box((FLAP_T, FLAP_W, FLAP_H)),
        origin=Origin(xyz=(FLAP_T / 2.0, 0.0, -(FLAP_H / 2.0))),
        material="flap",
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=FLAP_BARREL_R, length=0.014),
        origin=Origin(
            xyz=(FLAP_BARREL_R, 0.0, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="flap",
        name="hinge_barrel",
    )

    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, TRAY_CAVITY_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TRAY_TRAVEL,
            effort=25.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "crank_axle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(CRANK_X, CRANK_JOINT_Y, CRANK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(FLAP_HINGE_X, 0.0, FLAP_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FLAP_OPEN,
            effort=1.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    tray = object_model.get_part("tray")
    crank = object_model.get_part("crank")
    flap = object_model.get_part("flap")

    tray_slide = object_model.get_articulation("tray_slide")
    crank_axle = object_model.get_articulation("crank_axle")
    flap_hinge = object_model.get_articulation("flap_hinge")

    ctx.expect_within(
        tray,
        body,
        axes="yz",
        margin=0.008,
        name="tray stays centered in the lower opening",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        min_overlap=0.050,
        name="closed tray remains nested inside the housing",
    )

    closed_tray_pos = ctx.part_world_position(tray)
    tray_limits = tray_slide.motion_limits
    if tray_limits is not None and tray_limits.upper is not None:
        with ctx.pose({tray_slide: tray_limits.upper}):
            ctx.expect_within(
                tray,
                body,
                axes="yz",
                margin=0.008,
                name="opened tray stays aligned with the housing slot",
            )
            ctx.expect_overlap(
                tray,
                body,
                axes="x",
                min_overlap=0.020,
                name="opened tray still retains insertion inside the body",
            )
            opened_tray_pos = ctx.part_world_position(tray)
        ctx.check(
            "tray opens forward",
            closed_tray_pos is not None
            and opened_tray_pos is not None
            and opened_tray_pos[0] > closed_tray_pos[0] + 0.025,
            details=f"closed={closed_tray_pos}, opened={opened_tray_pos}",
        )

    closed_knob = _aabb_center(ctx.part_element_world_aabb(crank, elem="handle_knob"))
    with ctx.pose({crank_axle: pi / 2.0}):
        quarter_turn_knob = _aabb_center(ctx.part_element_world_aabb(crank, elem="handle_knob"))
    ctx.check(
        "crank rotates about the side axle",
        closed_knob is not None
        and quarter_turn_knob is not None
        and abs(quarter_turn_knob[0] - closed_knob[0]) > 0.012
        and abs(quarter_turn_knob[2] - closed_knob[2]) > 0.012,
        details=f"closed={closed_knob}, quarter_turn={quarter_turn_knob}",
    )

    closed_flap = _aabb_center(ctx.part_element_world_aabb(flap, elem="flap_panel"))
    with ctx.pose({flap_hinge: FLAP_OPEN}):
        opened_flap = _aabb_center(ctx.part_element_world_aabb(flap, elem="flap_panel"))
    ctx.check(
        "flap rests over the pencil port when closed",
        closed_flap is not None
        and (BODY_FRONT_X + 0.003) <= closed_flap[0] <= (BODY_FRONT_X + 0.012)
        and (PORT_Z - 0.006) <= closed_flap[2] <= (PORT_Z + 0.006),
        details=f"closed_flap_center={closed_flap}",
    )
    ctx.check(
        "flap opens upward away from the port",
        closed_flap is not None
        and opened_flap is not None
        and opened_flap[0] > closed_flap[0] + 0.004
        and opened_flap[2] > closed_flap[2] + 0.004,
        details=f"closed={closed_flap}, opened={opened_flap}",
    )

    return ctx.report()


object_model = build_object_model()
