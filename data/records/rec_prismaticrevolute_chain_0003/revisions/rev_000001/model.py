from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_L = 0.38
BASE_W = 0.16
FLOOR_T = 0.012
WALL_T = 0.008
WALL_H = 0.050
PEDESTAL_W = 0.026
PEDESTAL_H = 0.016
RAIL_W = 0.020
RAIL_H = 0.014
RAIL_LEN = 0.330
RAIL_Y = 0.048
RAIL_TOP_Z = FLOOR_T + PEDESTAL_H + RAIL_H
SCREW_Z = FLOOR_T + 0.013
SCREW_R = 0.006
SCREW_SPAN = 0.288
STOP_L = 0.014
STOP_W = 0.050
STOP_H = 0.026
COVER_L = 0.190
COVER_H = 0.034
COVER_T = 0.003

CARR_L = 0.155
CARR_W = 0.126
CARR_BLOCK_L = 0.040
CARR_BLOCK_W = 0.028
CARR_BLOCK_H = 0.024
CARR_PLATE_T = 0.012
CARR_FRONT_X = 0.108
CARR_REAR_X = 0.034
HINGE_AXIS_X = 0.122
HINGE_AXIS_Z = CARR_BLOCK_H + CARR_PLATE_T + 0.054
SUPPORT_L = 0.030
SUPPORT_W = 0.018
SUPPORT_H = 0.060
SUPPORT_Y = 0.034
HINGE_BORE_R = 0.010

HEAD_SHAFT_R = 0.0092
HEAD_COLLAR_R = 0.017
HEAD_COLLAR_T = 0.006
HEAD_TONGUE_W = 0.022
HEAD_TONGUE_H = 0.018
HEAD_TONGUE_L = 0.080
HEAD_RIB_H = 0.012
HEAD_RIB_W = 0.015
HEAD_FORK_X0 = 0.060
HEAD_FORK_L = 0.040
HEAD_EAR_W = 0.010
HEAD_EAR_GAP = 0.016
HEAD_EAR_H = 0.030
HEAD_PIN_R = 0.0055

SLIDE_ORIGIN_X = -0.145
SLIDE_TRAVEL = 0.160


def _export(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS)


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .center(center[1], center[2])
        .circle(radius)
        .extrude(length)
        .translate((center[0] - length / 2.0, 0.0, 0.0))
    )


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _z_bolt_heads(
    points: list[tuple[float, float]],
    radius: float,
    height: float,
    base_z: float,
) -> cq.Workplane:
    bolts = cq.Workplane("XY")
    for x_pos, y_pos in points:
        bolts = bolts.union(
            cq.Workplane("XY")
            .center(x_pos, y_pos)
            .circle(radius)
            .extrude(height)
            .translate((0.0, 0.0, base_z))
        )
    return bolts


def make_base_shell() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_L, BASE_W, FLOOR_T, centered=(True, True, False))
    for side in (-1.0, 1.0):
        wall = (
            cq.Workplane("XY")
            .center(0.0, side * (BASE_W / 2.0 - WALL_T / 2.0))
            .box(BASE_L, WALL_T, WALL_H, centered=(True, True, False))
        )
        pedestal = (
            cq.Workplane("XY")
            .center(0.0, side * RAIL_Y)
            .box(RAIL_LEN + 0.016, PEDESTAL_W, PEDESTAL_H, centered=(True, True, False))
            .translate((0.0, 0.0, FLOOR_T))
        )
        base = base.union(wall).union(pedestal)

    for x_pos in (-BASE_L / 2.0 + 0.016, BASE_L / 2.0 - 0.016):
        brace = (
            cq.Workplane("XY")
            .center(x_pos, 0.0)
            .box(0.020, BASE_W - 0.020, 0.030, centered=(True, True, False))
        )
        screw_housing = (
            cq.Workplane("XY")
            .center(x_pos * 0.83, 0.0)
            .box(0.028, 0.042, 0.030, centered=(True, True, False))
            .translate((0.0, 0.0, FLOOR_T))
        )
        base = base.union(brace).union(screw_housing)

    center_web = (
        cq.Workplane("XY")
        .box(0.060, 0.030, 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, FLOOR_T))
    )
    base = base.union(center_web)

    top_window = (
        cq.Workplane("XY")
        .box(RAIL_LEN - 0.030, 0.050, 0.050, centered=(True, True, False))
        .translate((0.0, 0.0, FLOOR_T + 0.008))
    )
    base = base.cut(top_window)

    for side in (-1.0, 1.0):
        access_window = (
            cq.Workplane("XY")
            .center(0.0, side * (BASE_W / 2.0 - WALL_T / 2.0))
            .box(COVER_L - 0.020, WALL_T + 0.002, COVER_H - 0.006, centered=(True, True, False))
            .translate((0.0, 0.0, 0.010))
        )
        base = base.cut(access_window)

    return base


def make_cover(side: float) -> cq.Workplane:
    cover = (
        cq.Workplane("XY")
        .center(0.0, side * (BASE_W / 2.0 + COVER_T / 2.0))
        .box(COVER_L, COVER_T, COVER_H, centered=(True, True, False))
        .translate((0.0, 0.0, 0.010))
    )
    bolt_y = side * (BASE_W / 2.0 + COVER_T)
    bolts = _z_bolt_heads(
        [
            (-0.072, bolt_y),
            (0.072, bolt_y),
            (-0.072, bolt_y - side * 0.0005),
            (0.072, bolt_y - side * 0.0005),
        ],
        radius=0.003,
        height=0.002,
        base_z=0.010 + COVER_H,
    )
    return cover.union(bolts)


def make_rail(side: float) -> cq.Workplane:
    rail = (
        cq.Workplane("XY")
        .center(0.0, side * RAIL_Y)
        .box(RAIL_LEN, RAIL_W, RAIL_H, centered=(True, True, False))
        .translate((0.0, 0.0, FLOOR_T + PEDESTAL_H))
    )
    clamp_points = [(-0.120, side * RAIL_Y), (-0.040, side * RAIL_Y), (0.040, side * RAIL_Y), (0.120, side * RAIL_Y)]
    clamps = _z_bolt_heads(
        clamp_points,
        radius=0.0026,
        height=0.002,
        base_z=RAIL_TOP_Z,
    )
    return rail.union(clamps)


def make_travel_stop(x_pos: float) -> cq.Workplane:
    stop = (
        cq.Workplane("XY")
        .center(x_pos, 0.0)
        .box(STOP_L, STOP_W, STOP_H, centered=(True, True, False))
        .translate((0.0, 0.0, FLOOR_T))
    )
    pad = (
        cq.Workplane("XY")
        .center(x_pos, 0.0)
        .box(STOP_L - 0.004, STOP_W - 0.012, 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, FLOOR_T + STOP_H))
    )
    return stop.union(pad)


def make_leadscrew() -> cq.Workplane:
    screw = _x_cylinder(SCREW_R, SCREW_SPAN, (0.0, 0.0, SCREW_Z))
    for x_pos in (-0.080, 0.0, 0.080):
        screw = screw.union(
            _x_cylinder(SCREW_R * 1.08, 0.014, (x_pos, 0.0, SCREW_Z))
        )
    return screw


def make_carriage_shell() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(CARR_L, CARR_W, CARR_PLATE_T, centered=(False, True, False))
        .translate((CARR_L / 2.0, 0.0, CARR_BLOCK_H))
    )

    shell = plate
    block_positions = [
        (CARR_REAR_X, -RAIL_Y),
        (CARR_FRONT_X, -RAIL_Y),
        (CARR_REAR_X, RAIL_Y),
        (CARR_FRONT_X, RAIL_Y),
    ]
    for x_pos, y_pos in block_positions:
        block = (
            cq.Workplane("XY")
            .center(x_pos, y_pos)
            .box(CARR_BLOCK_L, CARR_BLOCK_W, CARR_BLOCK_H, centered=(True, True, False))
        )
        shell = shell.union(block)

    nut_housing = (
        cq.Workplane("XY")
        .center(0.074, 0.0)
        .box(0.038, 0.034, CARR_BLOCK_H - 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, 0.004))
    )
    nut_relief = _x_cylinder(SCREW_R + 0.003, 0.046, (0.074, 0.0, SCREW_Z - RAIL_TOP_Z))
    shell = shell.union(nut_housing.cut(nut_relief))

    return shell


def make_bearing_support(side: float) -> cq.Workplane:
    support = (
        cq.Workplane("XY")
        .center(HINGE_AXIS_X, side * SUPPORT_Y)
        .box(SUPPORT_L, SUPPORT_W, SUPPORT_H, centered=(True, True, False))
        .translate((0.0, 0.0, CARR_BLOCK_H + CARR_PLATE_T))
    )
    bore = _y_cylinder(
        HINGE_BORE_R,
        SUPPORT_W + 0.002,
        (
            HINGE_AXIS_X,
            side * SUPPORT_Y,
            HINGE_AXIS_Z,
        ),
    )
    window = (
        cq.Workplane("XY")
        .center(HINGE_AXIS_X, side * SUPPORT_Y)
        .box(SUPPORT_L - 0.012, SUPPORT_W + 0.002, SUPPORT_H - 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, CARR_BLOCK_H + CARR_PLATE_T + 0.008))
    )
    bolts = _z_bolt_heads(
        [
            (HINGE_AXIS_X - 0.008, side * SUPPORT_Y),
            (HINGE_AXIS_X + 0.008, side * SUPPORT_Y),
        ],
        radius=0.0025,
        height=0.0025,
        base_z=CARR_BLOCK_H + CARR_PLATE_T + SUPPORT_H,
    )
    return support.cut(bore).cut(window).union(bolts)


def make_head_body() -> cq.Workplane:
    tongue_center_x = 0.068
    tongue_center_z = 0.020
    body = (
        cq.Workplane("XY")
        .center(tongue_center_x, 0.0)
        .box(HEAD_TONGUE_L, HEAD_TONGUE_W, HEAD_TONGUE_H, centered=(True, True, True))
        .translate((0.0, 0.0, tongue_center_z))
    )
    gusset = (
        cq.Workplane("XY")
        .center(0.032, 0.0)
        .box(0.034, HEAD_RIB_W, HEAD_RIB_H, centered=(True, True, True))
        .translate((0.0, 0.0, 0.010))
    )
    hub_cheek = (
        cq.Workplane("XY")
        .center(0.010, 0.0)
        .box(0.016, 0.020, 0.028, centered=(True, True, True))
        .translate((0.0, 0.0, 0.010))
    )
    body = body.union(gusset).union(hub_cheek)

    left_ear_y = HEAD_EAR_GAP / 2.0 + HEAD_EAR_W / 2.0
    right_ear_y = -left_ear_y
    for y_pos in (left_ear_y, right_ear_y):
        ear = (
            cq.Workplane("XY")
            .center(HEAD_FORK_X0 + HEAD_FORK_L / 2.0, y_pos)
            .box(HEAD_FORK_L, HEAD_EAR_W, HEAD_EAR_H, centered=(True, True, True))
            .translate((0.0, 0.0, 0.022))
        )
        body = body.union(ear)

    pin_hole = _y_cylinder(HEAD_PIN_R, HEAD_EAR_GAP + 2.0 * HEAD_EAR_W + 0.004, (HEAD_FORK_X0 + 0.024, 0.0, 0.022))
    return body.cut(pin_hole)


def make_head_pivot() -> cq.Workplane:
    shaft_len = 2.0 * (SUPPORT_Y + SUPPORT_W / 2.0 + HEAD_COLLAR_T)
    shaft = _y_cylinder(HEAD_SHAFT_R, shaft_len, (0.0, 0.0, 0.0))
    hub = _y_cylinder(0.015, 0.022, (0.0, 0.0, 0.0))
    left_collar = _y_cylinder(
        HEAD_COLLAR_R,
        HEAD_COLLAR_T,
        (0.0, SUPPORT_Y + SUPPORT_W / 2.0 + HEAD_COLLAR_T / 2.0, 0.0),
    )
    right_collar = _y_cylinder(
        HEAD_COLLAR_R,
        HEAD_COLLAR_T,
        (0.0, -(SUPPORT_Y + SUPPORT_W / 2.0 + HEAD_COLLAR_T / 2.0), 0.0),
    )
    return shaft.union(hub).union(left_collar).union(right_collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_study", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.30, 0.32, 0.35, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.53, 0.56, 0.60, 1.0))
    steel_bright = model.material("steel_bright", rgba=(0.70, 0.72, 0.75, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.42, 0.44, 0.47, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(_export(make_base_shell(), "base_shell.obj"), material=steel_dark, name="base_shell")
    base_frame.visual(_export(make_cover(-1.0), "left_cover.obj"), material=cover_gray, name="left_cover")
    base_frame.visual(_export(make_cover(1.0), "right_cover.obj"), material=cover_gray, name="right_cover")
    base_frame.visual(_export(make_rail(-1.0), "left_rail.obj"), material=steel_mid, name="left_rail")
    base_frame.visual(_export(make_rail(1.0), "right_rail.obj"), material=steel_mid, name="right_rail")
    base_frame.visual(_export(make_travel_stop(-0.160), "rear_stop.obj"), material=steel_mid, name="rear_stop")
    base_frame.visual(_export(make_travel_stop(0.160), "front_stop.obj"), material=steel_mid, name="front_stop")
    base_frame.visual(_export(make_leadscrew(), "leadscrew.obj"), material=steel_bright, name="leadscrew")
    base_frame.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.085)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
    )

    carriage = model.part("sliding_carriage")
    carriage.visual(_export(make_carriage_shell(), "carriage_shell.obj"), material=steel_mid, name="carriage_shell")
    carriage.visual(
        _export(make_bearing_support(-1.0), "left_bearing_support.obj"),
        material=steel_dark,
        name="left_bearing_support",
    )
    carriage.visual(
        _export(make_bearing_support(1.0), "right_bearing_support.obj"),
        material=steel_dark,
        name="right_bearing_support",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARR_L, CARR_W, SUPPORT_H + CARR_BLOCK_H + CARR_PLATE_T)),
        mass=3.2,
        origin=Origin(xyz=(CARR_L / 2.0, 0.0, (SUPPORT_H + CARR_BLOCK_H + CARR_PLATE_T) / 2.0)),
    )

    head = model.part("clevis_head")
    head.visual(_export(make_head_body(), "head_body.obj"), material=steel_bright, name="head_body")
    head.visual(_export(make_head_pivot(), "head_pivot.obj"), material=steel_mid, name="head_pivot")
    head.inertial = Inertial.from_geometry(
        Box((HEAD_FORK_X0 + HEAD_FORK_L, 0.090, HEAD_EAR_H)),
        mass=1.4,
        origin=Origin(xyz=((HEAD_FORK_X0 + HEAD_FORK_L) / 2.0, 0.0, 0.018)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=carriage,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.25, lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "carriage_to_head",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.8, lower=-0.60, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    carriage = object_model.get_part("sliding_carriage")
    head = object_model.get_part("clevis_head")
    slide = object_model.get_articulation("base_to_carriage")
    hinge = object_model.get_articulation("carriage_to_head")

    left_rail = base_frame.get_visual("left_rail")
    right_rail = base_frame.get_visual("right_rail")
    left_support = carriage.get_visual("left_bearing_support")
    right_support = carriage.get_visual("right_bearing_support")
    front_stop = base_frame.get_visual("front_stop")
    rear_stop = base_frame.get_visual("rear_stop")
    head_pivot = head.get_visual("head_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.allow_overlap(
        head,
        carriage,
        elem_a=head_pivot,
        elem_b=left_support,
        reason="pivot shaft runs inside the left bearing support bore as an intentional nested bearing fit",
    )
    ctx.allow_overlap(
        head,
        carriage,
        elem_a=head_pivot,
        elem_b=right_support,
        reason="pivot shaft runs inside the right bearing support bore as an intentional nested bearing fit",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(carriage, base_frame, elem_b=left_rail, name="carriage rides on left guide rail")
    ctx.expect_contact(carriage, base_frame, elem_b=right_rail, name="carriage rides on right guide rail")
    ctx.expect_overlap(carriage, base_frame, axes="x", min_overlap=0.12, name="carriage remains supported over slide base")
    ctx.expect_within(carriage, base_frame, axes="y", margin=0.0, name="carriage stays laterally within base frame")
    ctx.expect_contact(head, carriage, elem_a=head_pivot, elem_b=left_support, name="head bears on left hinge support")
    ctx.expect_contact(head, carriage, elem_a=head_pivot, elem_b=right_support, name="head bears on right hinge support")

    with ctx.pose({slide: 0.120}):
        ctx.expect_contact(carriage, base_frame, elem_b=left_rail, name="left guide contact persists mid-stroke")
        ctx.expect_contact(carriage, base_frame, elem_b=right_rail, name="right guide contact persists mid-stroke")
        ctx.expect_overlap(carriage, base_frame, axes="x", min_overlap=0.10, name="carriage remains within base travel envelope")

    with ctx.pose({slide: SLIDE_TRAVEL - 0.010}):
        ctx.expect_overlap(carriage, base_frame, axes="x", min_overlap=0.03, name="carriage still overlaps base near forward stop")
        ctx.expect_contact(base_frame, carriage, elem_a=front_stop, name="forward stop stays close enough to arrest travel")

    with ctx.pose({hinge: 0.75}):
        ctx.expect_contact(head, carriage, elem_a=head_pivot, elem_b=left_support, name="left hinge bearing support stays engaged at positive angle")
        ctx.expect_contact(head, carriage, elem_a=head_pivot, elem_b=right_support, name="right hinge bearing support stays engaged at positive angle")

    with ctx.pose({slide: 0.0, hinge: -0.55}):
        ctx.expect_contact(head, carriage, elem_a=head_pivot, elem_b=left_support, name="left hinge bearing support stays engaged at negative angle")
        ctx.expect_contact(head, carriage, elem_a=head_pivot, elem_b=right_support, name="right hinge bearing support stays engaged at negative angle")
        ctx.expect_contact(base_frame, carriage, elem_a=rear_stop, name="rear stop remains aligned to catch retracted carriage")

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="articulations remain clearance-safe across sampled poses")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
