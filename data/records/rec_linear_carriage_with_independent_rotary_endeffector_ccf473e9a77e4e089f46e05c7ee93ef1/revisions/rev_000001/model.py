from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GUIDE_L = 0.42
GUIDE_W = 0.12
BASE_T = 0.018
RAIL_L = 0.32
RAIL_W = 0.018
RAIL_H = 0.028
RAIL_Y = 0.032
END_PAD_L = 0.048
END_PAD_W = 0.094
END_PAD_H = 0.014

CARR_BODY_L = 0.110
CARR_W = 0.104
CARR_H = 0.074
POCKET_L = CARR_BODY_L + 0.004
POCKET_W = 0.072
POCKET_D = 0.028
TOP_RECESS_L = 0.062
TOP_RECESS_W = 0.054
TOP_RECESS_D = 0.010
FACE_PLATE_L = 0.014
FACE_PLATE_W = 0.072
FACE_PLATE_H = 0.050
BEARING_R = 0.026
BEARING_L = 0.018
BEARING_Z = 0.043
SHOE_W = 0.020
SHOE_H = 0.024
BRIDGE_W = 0.078
BRIDGE_H = 0.030
BRIDGE_Z0 = 0.022
REAR_WEB_L = 0.016
REAR_WEB_W = 0.068
REAR_WEB_H = 0.040

FLANGE_HUB_R = 0.021
FLANGE_HUB_L = 0.010
FLANGE_R = 0.038
FLANGE_T = 0.012
FLANGE_NOSE_R = 0.014
FLANGE_NOSE_L = 0.010
FLANGE_BOLT_CIRCLE_R = 0.026
FLANGE_BOLT_D = 0.008
FLAG_LUG_X = 0.008
FLAG_LUG_Y = 0.016
FLAG_LUG_Z = 0.010

SLIDE_TRAVEL = 0.160
SLIDE_Q0_X = -0.080
SLIDE_Z = BASE_T + RAIL_H
ROTARY_ORIGIN_X = (CARR_BODY_L / 2.0) + FACE_PLATE_L + BEARING_L


def _close_vec(a: tuple[float, float, float], b: tuple[float, float, float], tol: float = 1e-9) -> bool:
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def _guide_body_shape() -> cq.Workplane:
    mount_points = [
        (-0.165, -0.040),
        (-0.165, 0.040),
        (0.165, -0.040),
        (0.165, 0.040),
    ]

    base = (
        cq.Workplane("XY")
        .box(GUIDE_L, GUIDE_W, BASE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )

    left_rail = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_H, centered=(True, True, False))
        .translate((0.0, RAIL_Y, BASE_T))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_H, centered=(True, True, False))
        .translate((0.0, -RAIL_Y, BASE_T))
    )
    center_spine = (
        cq.Workplane("XY")
        .box(RAIL_L - 0.060, 0.024, 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_T))
    )

    left_pad = (
        cq.Workplane("XY")
        .box(END_PAD_L, END_PAD_W, END_PAD_H, centered=(True, True, False))
        .translate((-0.175, 0.0, BASE_T))
    )
    right_pad = (
        cq.Workplane("XY")
        .box(END_PAD_L, END_PAD_W, END_PAD_H, centered=(True, True, False))
        .translate((0.175, 0.0, BASE_T))
    )

    body = base.union(left_rail).union(right_rail).union(center_spine).union(left_pad).union(right_pad)

    mount_cuts = (
        cq.Workplane("XY")
        .pushPoints(mount_points)
        .circle(0.005)
        .extrude(BASE_T + 0.002)
    )

    rail_end_chamfers = (
        cq.Workplane("XZ")
        .center((-RAIL_L / 2.0) + 0.012, 0.0)
        .rect(0.010, RAIL_H + 0.002)
        .extrude(RAIL_W / 2.0 + 0.002, both=True)
    )
    rail_end_chamfers = rail_end_chamfers.union(rail_end_chamfers.mirror("YZ"))

    body = body.cut(mount_cuts)
    body = body.cut(rail_end_chamfers)
    return body


def _carriage_shape() -> cq.Workplane:
    left_shoe = (
        cq.Workplane("XY")
        .box(CARR_BODY_L, SHOE_W, SHOE_H, centered=(True, True, False))
        .translate((0.0, RAIL_Y, 0.0))
    )
    right_shoe = (
        cq.Workplane("XY")
        .box(CARR_BODY_L, SHOE_W, SHOE_H, centered=(True, True, False))
        .translate((0.0, -RAIL_Y, 0.0))
    )
    bridge = (
        cq.Workplane("XY")
        .box(CARR_BODY_L, BRIDGE_W, BRIDGE_H, centered=(True, True, False))
        .translate((0.0, 0.0, BRIDGE_Z0))
    )
    rear_web = (
        cq.Workplane("XY")
        .box(REAR_WEB_L, REAR_WEB_W, REAR_WEB_H, centered=(True, True, False))
        .translate((-(CARR_BODY_L / 2.0) + (REAR_WEB_L / 2.0), 0.0, 0.0))
    )

    top_recess = (
        cq.Workplane("XY")
        .box(TOP_RECESS_L, TOP_RECESS_W, TOP_RECESS_D + 0.001, centered=(True, True, False))
        .translate((0.0, 0.0, BRIDGE_Z0 + BRIDGE_H - TOP_RECESS_D))
    )

    face_plate = (
        cq.Workplane("XY")
        .box(FACE_PLATE_L, FACE_PLATE_W, FACE_PLATE_H, centered=(False, True, False))
        .translate((CARR_BODY_L / 2.0, 0.0, BEARING_Z))
    )

    bearing_housing = (
        cq.Workplane("YZ")
        .circle(BEARING_R)
        .extrude(BEARING_L)
        .translate(((CARR_BODY_L / 2.0) + FACE_PLATE_L, 0.0, BEARING_Z))
    )

    carriage = left_shoe.union(right_shoe).union(bridge).union(rear_web).union(face_plate).union(bearing_housing)
    carriage = carriage.cut(top_recess)

    bolt_pattern = [
        (0.018, 0.018),
        (0.018, -0.018),
        (-0.018, 0.018),
        (-0.018, -0.018),
    ]
    bolt_cuts = (
        cq.Workplane("YZ")
        .pushPoints(bolt_pattern)
        .circle(0.003)
        .extrude(FACE_PLATE_L * 0.85)
        .translate((CARR_BODY_L / 2.0, 0.0, BEARING_Z + (FACE_PLATE_H / 2.0)))
    )
    carriage = carriage.cut(bolt_cuts)
    return carriage


def _rotary_flange_shape() -> cq.Workplane:
    hub = cq.Workplane("YZ").circle(FLANGE_HUB_R).extrude(FLANGE_HUB_L)

    disk = (
        cq.Workplane("YZ")
        .circle(FLANGE_R)
        .extrude(FLANGE_T)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (FLANGE_BOLT_CIRCLE_R, 0.0),
                (-FLANGE_BOLT_CIRCLE_R, 0.0),
                (0.0, FLANGE_BOLT_CIRCLE_R),
                (0.0, -FLANGE_BOLT_CIRCLE_R),
            ]
        )
        .hole(FLANGE_BOLT_D, depth=FLANGE_T)
        .translate((FLANGE_HUB_L, 0.0, 0.0))
    )

    nose = (
        cq.Workplane("YZ")
        .circle(FLANGE_NOSE_R)
        .extrude(FLANGE_NOSE_L)
        .translate((FLANGE_HUB_L + FLANGE_T, 0.0, 0.0))
    )

    index_lug = (
        cq.Workplane("XY")
        .box(FLAG_LUG_X, FLAG_LUG_Y, FLAG_LUG_Z, centered=(False, True, True))
        .translate((FLANGE_HUB_L + 0.002, FLANGE_R + (FLAG_LUG_Y / 2.0), 0.0))
    )

    return hub.union(disk).union(nose).union(index_lug)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_rotary_service_fixture")

    model.material("guide_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    model.material("carriage_orange", rgba=(0.89, 0.46, 0.16, 1.0))
    model.material("flange_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("dark_fastener", rgba=(0.14, 0.15, 0.16, 1.0))

    guide_body = model.part("guide_body")
    guide_body.visual(
        mesh_from_cadquery(_guide_body_shape(), "guide_body"),
        material="guide_gray",
        name="guide_shell",
    )

    carriage = model.part("moving_block")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "moving_block"),
        material="carriage_orange",
        name="block_shell",
    )

    rotary_flange = model.part("rotary_flange")
    rotary_flange.visual(
        mesh_from_cadquery(_rotary_flange_shape(), "rotary_flange"),
        material="flange_steel",
        name="flange_shell",
    )
    rotary_flange.visual(
        Box((0.004, 0.012, 0.012)),
        origin=Origin(xyz=(FLANGE_HUB_L + FLANGE_T + FLANGE_NOSE_L + 0.002, 0.0, 0.0)),
        material="dark_fastener",
        name="center_stub",
    )

    model.articulation(
        "guide_to_block",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=carriage,
        origin=Origin(xyz=(SLIDE_Q0_X, 0.0, SLIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_TRAVEL, effort=250.0, velocity=0.25),
    )
    model.articulation(
        "block_to_flange",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=rotary_flange,
        origin=Origin(xyz=(ROTARY_ORIGIN_X, 0.0, BEARING_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=40.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_body = object_model.get_part("guide_body")
    carriage = object_model.get_part("moving_block")
    rotary_flange = object_model.get_part("rotary_flange")
    slide = object_model.get_articulation("guide_to_block")
    spin = object_model.get_articulation("block_to_flange")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "slide_axis_along_x",
        _close_vec(slide.axis, (1.0, 0.0, 0.0)),
        f"expected slide axis (1, 0, 0), got {slide.axis}",
    )
    ctx.check(
        "flange_axis_along_x",
        _close_vec(spin.axis, (1.0, 0.0, 0.0)),
        f"expected flange axis (1, 0, 0), got {spin.axis}",
    )

    ctx.expect_contact(
        carriage,
        guide_body,
        contact_tol=5e-4,
        name="moving_block_is_supported_by_guide_body",
    )
    ctx.expect_contact(
        rotary_flange,
        carriage,
        contact_tol=5e-4,
        name="rotary_flange_seats_on_bearing_housing",
    )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            guide_body,
            contact_tol=5e-4,
            name="moving_block_stays_supported_at_full_extension",
        )

    carriage_x0 = None
    carriage_x1 = None
    flange_x0 = None
    flange_x1 = None
    flange_aabb_0 = None
    flange_aabb_90 = None

    with ctx.pose({slide: 0.0, spin: 0.0}):
        carriage_pos = ctx.part_world_position(carriage)
        flange_pos = ctx.part_world_position(rotary_flange)
        flange_aabb_0 = ctx.part_world_aabb(rotary_flange)
        if carriage_pos is not None:
            carriage_x0 = carriage_pos[0]
        if flange_pos is not None:
            flange_x0 = flange_pos[0]

    with ctx.pose({slide: SLIDE_TRAVEL, spin: 0.0}):
        carriage_pos = ctx.part_world_position(carriage)
        flange_pos = ctx.part_world_position(rotary_flange)
        if carriage_pos is not None:
            carriage_x1 = carriage_pos[0]
        if flange_pos is not None:
            flange_x1 = flange_pos[0]

    with ctx.pose({spin: pi / 2.0}):
        ctx.expect_contact(
            rotary_flange,
            carriage,
            contact_tol=5e-4,
            name="rotary_flange_stays_mounted_when_rotated",
        )
        flange_aabb_90 = ctx.part_world_aabb(rotary_flange)

    ctx.check(
        "slide_moves_block_in_positive_x",
        carriage_x0 is not None
        and carriage_x1 is not None
        and abs((carriage_x1 - carriage_x0) - SLIDE_TRAVEL) <= 1e-4,
        f"expected carriage x travel {SLIDE_TRAVEL}, got {None if carriage_x0 is None or carriage_x1 is None else carriage_x1 - carriage_x0}",
    )
    ctx.check(
        "slide_carries_rotary_output_in_positive_x",
        flange_x0 is not None
        and flange_x1 is not None
        and abs((flange_x1 - flange_x0) - SLIDE_TRAVEL) <= 1e-4,
        f"expected flange x travel {SLIDE_TRAVEL}, got {None if flange_x0 is None or flange_x1 is None else flange_x1 - flange_x0}",
    )
    ctx.check(
        "flange_rotation_changes_yz_orientation",
        flange_aabb_0 is not None
        and flange_aabb_90 is not None
        and (flange_aabb_0[1][1] - flange_aabb_90[1][1]) > 0.010
        and (flange_aabb_90[1][2] - flange_aabb_0[1][2]) > 0.010,
        f"aabb_0={flange_aabb_0}, aabb_90={flange_aabb_90}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
