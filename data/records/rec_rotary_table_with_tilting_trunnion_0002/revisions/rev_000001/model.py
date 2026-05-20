from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

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

BASE_LENGTH = 0.52
BASE_DEPTH = 0.30
BASE_THICKNESS = 0.03
PEDESTAL_CENTER_X = 0.175
PEDESTAL_WIDTH = 0.06
PEDESTAL_DEPTH = 0.12
PEDESTAL_HEIGHT = 0.20
AXIS_Z = 0.20
BASE_INNER_FACE_X = PEDESTAL_CENTER_X - PEDESTAL_WIDTH / 2.0
BASE_OUTER_FACE_X = PEDESTAL_CENTER_X + PEDESTAL_WIDTH / 2.0

TRUNNION_SIDE_CENTER_X = 0.122
TRUNNION_SIDE_THICKNESS = 0.022
TRUNNION_OUTER_FACE_X = TRUNNION_SIDE_CENTER_X + TRUNNION_SIDE_THICKNESS / 2.0
TRUNNION_FRAME_DEPTH = 0.095
TRUNNION_FRAME_HEIGHT = 0.24
TRUNNION_FRAME_FRONT_Y = 0.035
TRUNNION_FRAME_REAR_Y = -0.035
TRUNNION_JOURNAL_RADIUS = 0.018
TRUNNION_BORE_RADIUS = 0.0195
TRUNNION_COLLAR_RADIUS = 0.038

TABLE_RADIUS = 0.10
TABLE_FACE_THICKNESS = 0.028
TABLE_FACE_FRONT_Y = 0.031
SPINDLE_RADIUS = 0.022
REAR_THRUST_RADIUS = 0.040
FRONT_CHEEK_OUTER_RADIUS = 0.056
FRONT_CHEEK_INNER_RADIUS = 0.028
FRONT_CHEEK_THICKNESS = 0.010
REAR_CHEEK_OUTER_RADIUS = 0.070
REAR_CHEEK_INNER_RADIUS = 0.024
REAR_CHEEK_THICKNESS = 0.016
FACE_CLAMP_RING_OUTER_RADIUS = 0.058
FACE_CLAMP_RING_INNER_RADIUS = 0.038
FACE_CLAMP_RING_THICKNESS = 0.008
RUNNER_THICKNESS = 0.0035
SERVICE_COVER_WIDTH = 0.050
SERVICE_COVER_HEIGHT = 0.032
SERVICE_COVER_THICKNESS = 0.006


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(
        shape,
        filename,
        assets=ASSETS,
        tolerance=0.0008,
        angular_tolerance=0.08,
    )


def _polar_points(radius: float, angles_deg: list[float]) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(math.radians(angle)),
            radius * math.sin(math.radians(angle)),
        )
        for angle in angles_deg
    ]


def _annular_sector_points(
    inner_radius: float,
    outer_radius: float,
    start_deg: float,
    end_deg: float,
    samples: int = 28,
) -> list[tuple[float, float]]:
    outer = [
        (
            outer_radius * math.cos(math.radians(angle)),
            outer_radius * math.sin(math.radians(angle)),
        )
        for angle in [
            start_deg + (end_deg - start_deg) * i / samples for i in range(samples + 1)
        ]
    ]
    inner = [
        (
            inner_radius * math.cos(math.radians(angle)),
            inner_radius * math.sin(math.radians(angle)),
        )
        for angle in [
            end_deg - (end_deg - start_deg) * i / samples for i in range(samples + 1)
        ]
    ]
    return outer + inner


def _make_base_structure() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_LENGTH, BASE_DEPTH, BASE_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS / 2.0)
    )

    for x_sign in (-1.0, 1.0):
        x_pos = x_sign * PEDESTAL_CENTER_X
        pedestal = cq.Workplane("XY").box(
            PEDESTAL_WIDTH,
            PEDESTAL_DEPTH,
            PEDESTAL_HEIGHT,
        ).translate((x_pos, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0))
        base = base.union(pedestal)

        axis_boss = (
            cq.Workplane("YZ")
            .circle(0.050)
            .circle(0.028)
            .extrude(0.010)
            .translate((x_sign * (BASE_OUTER_FACE_X - 0.010), 0.0, AXIS_Z))
        )
        if x_sign < 0.0:
            axis_boss = axis_boss.mirror("YZ")
        base = base.union(axis_boss)

        for y_pos in (-0.076, 0.076):
            gusset = (
                cq.Workplane("XZ")
                .polyline(
                    [
                        (x_sign * 0.088, BASE_THICKNESS),
                        (x_sign * (PEDESTAL_CENTER_X - 0.008), BASE_THICKNESS),
                        (x_sign * (PEDESTAL_CENTER_X - 0.008), AXIS_Z - 0.062),
                    ]
                )
                .close()
                .extrude(0.014)
                .translate((0.0, y_pos - 0.007, 0.0))
            )
            base = base.union(gusset)

    mounting_slots = cq.Workplane("XY")
    for point in (
        (-0.185, -0.105),
        (-0.185, 0.105),
        (0.185, -0.105),
        (0.185, 0.105),
    ):
        mounting_slots = mounting_slots.union(
            cq.Workplane("XY")
            .center(point[0], point[1])
            .slot2D(0.044, 0.014, angle=0.0)
            .extrude(BASE_THICKNESS + 0.004)
        )
    base = base.cut(mounting_slots.translate((0.0, 0.0, -0.002)))

    for x_pos in (-PEDESTAL_CENTER_X, PEDESTAL_CENTER_X):
        saddle = (
            cq.Workplane("YZ")
            .center(0.0, AXIS_Z)
            .circle(0.041)
            .extrude(PEDESTAL_WIDTH + 0.025)
            .translate((x_pos - (PEDESTAL_WIDTH + 0.025) / 2.0, 0.0, 0.0))
        )
        split_relief = (
            cq.Workplane("YZ")
            .center(0.0, AXIS_Z)
            .rect(0.050, 0.048)
            .extrude(0.020)
            .translate((x_pos - 0.010, 0.0, 0.0))
        )
        base = base.cut(saddle).cut(split_relief)

    return base


def _make_bearing_cap(side: str) -> cq.Workplane:
    cap = (
        cq.Workplane("YZ")
        .center(0.0, AXIS_Z)
        .circle(0.052)
        .circle(0.024)
        .extrude(0.012)
    )
    bolts = cq.Workplane("YZ")
    for y_pos, z_pos in _polar_points(0.040, [20.0, 80.0, 140.0, 200.0, 260.0, 320.0]):
        bolts = bolts.union(
            cq.Workplane("YZ")
            .center(y_pos, AXIS_Z + z_pos)
            .circle(0.0042)
            .extrude(0.015)
        )
    cap = cap.union(bolts)
    if side == "left":
        return cap.translate((-BASE_OUTER_FACE_X - 0.015, 0.0, 0.0))
    return cap.translate((BASE_OUTER_FACE_X, 0.0, 0.0))


def _make_access_cover(side: str) -> cq.Workplane:
    cover = cq.Workplane("YZ").rect(0.060, 0.085).extrude(0.008)
    bolts = cq.Workplane("YZ")
    for point in [(-0.022, -0.032), (-0.022, 0.032), (0.022, -0.032), (0.022, 0.032)]:
        bolts = bolts.union(
            cq.Workplane("YZ")
            .center(point[0], AXIS_Z - 0.075 + point[1])
            .circle(0.0035)
            .extrude(0.011)
        )
    cover = cover.union(bolts).translate((0.0, 0.0, AXIS_Z - 0.075))
    if side == "left":
        return cover.translate((-BASE_OUTER_FACE_X - 0.008, 0.0, 0.0))
    return cover.translate((BASE_OUTER_FACE_X, 0.0, 0.0))


def _make_sector_stop_bracket() -> cq.Workplane:
    anchor_x = BASE_OUTER_FACE_X + 0.008
    arm_center_x = BASE_OUTER_FACE_X + 0.050
    outboard_x = BASE_OUTER_FACE_X + 0.082

    outboard_rail = cq.Workplane("XY").box(0.012, 0.140, 0.160).translate(
        (outboard_x, 0.0, AXIS_Z + 0.002)
    )
    upper_anchor = cq.Workplane("XY").box(0.014, 0.060, 0.020).translate(
        (anchor_x, 0.0, AXIS_Z + 0.086)
    )
    lower_anchor = cq.Workplane("XY").box(0.014, 0.060, 0.020).translate(
        (anchor_x, 0.0, AXIS_Z - 0.098)
    )
    upper_arm = cq.Workplane("XY").box(0.078, 0.020, 0.020).translate(
        (arm_center_x, 0.0, AXIS_Z + 0.086)
    )
    lower_arm = cq.Workplane("XY").box(0.078, 0.020, 0.020).translate(
        (arm_center_x, 0.0, AXIS_Z - 0.098)
    )
    bracket = outboard_rail.union(upper_anchor).union(lower_anchor).union(upper_arm).union(lower_arm)

    for y_pos in (-0.026, 0.026):
        upper_bolt = cq.Workplane("YZ").circle(0.004).extrude(0.010).translate(
            (anchor_x + 0.006, y_pos, AXIS_Z + 0.086)
        )
        lower_bolt = cq.Workplane("YZ").circle(0.004).extrude(0.010).translate(
            (anchor_x + 0.006, y_pos, AXIS_Z - 0.098)
        )
        bracket = bracket.union(upper_bolt).union(lower_bolt)

    for y_pos, z_pos in ((0.060, AXIS_Z + 0.052), (-0.060, AXIS_Z + 0.052)):
        stop_block = cq.Workplane("XY").box(0.018, 0.024, 0.032).translate(
            (outboard_x, y_pos, z_pos)
        )
        stop_puck = (
            cq.Workplane("YZ")
            .circle(0.008)
            .extrude(0.008)
            .translate((outboard_x + 0.004, y_pos, z_pos))
        )
        bracket = bracket.union(stop_block).union(stop_puck)
    return bracket


def _make_side_plate(x_center: float) -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .rect(TRUNNION_FRAME_DEPTH + 0.014, TRUNNION_FRAME_HEIGHT + 0.008)
        .extrude(TRUNNION_SIDE_THICKNESS)
    )
    main_window = (
        cq.Workplane("YZ")
        .circle(0.090)
        .extrude(TRUNNION_SIDE_THICKNESS + 0.004)
        .translate((-0.002, 0.0, 0.0))
    )
    lower_relief = (
        cq.Workplane("YZ")
        .center(0.0, -0.103)
        .slot2D(0.052, 0.022, angle=90.0)
        .extrude(TRUNNION_SIDE_THICKNESS + 0.004)
        .translate((-0.002, 0.0, 0.0))
    )
    inspection_relief = (
        cq.Workplane("YZ")
        .center(0.0, 0.090)
        .slot2D(0.040, 0.016, angle=0.0)
        .extrude(TRUNNION_SIDE_THICKNESS + 0.004)
        .translate((-0.002, 0.0, 0.0))
    )
    bearing_boss = (
        cq.Workplane("YZ")
        .circle(0.066)
        .circle(0.030)
        .extrude(0.008)
        .translate((TRUNNION_SIDE_THICKNESS, 0.0, 0.0))
    )
    plate = plate.cut(main_window).cut(lower_relief).cut(inspection_relief).union(bearing_boss)
    plate = plate.translate((abs(x_center) - TRUNNION_SIDE_THICKNESS / 2.0, 0.0, 0.0))
    if x_center < 0.0:
        plate = plate.mirror("YZ")
    return plate


def _make_frame_body() -> cq.Workplane:
    frame = _make_side_plate(-TRUNNION_SIDE_CENTER_X).union(
        _make_side_plate(TRUNNION_SIDE_CENTER_X)
    )
    top_beam = cq.Workplane("XY").box(0.254, 0.054, 0.036).translate((0.0, 0.0, 0.128))
    bottom_beam = cq.Workplane("XY").box(0.254, 0.042, 0.040).translate(
        (0.0, 0.0, -0.130)
    )
    for x_sign in (-1.0, 1.0):
        rear_rib = cq.Workplane("XY").box(0.018, 0.016, 0.086).translate(
            (x_sign * 0.118, -0.030, -0.020)
        )
        front_rib = cq.Workplane("XY").box(0.014, 0.016, 0.060).translate(
            (x_sign * 0.118, 0.030, -0.075)
        )
        frame = frame.union(rear_rib).union(front_rib)
    return frame.union(top_beam).union(bottom_beam)


def _make_front_bearing_cheek() -> cq.Workplane:
    cheek = (
        cq.Workplane("XZ")
        .circle(FRONT_CHEEK_OUTER_RADIUS)
        .circle(FRONT_CHEEK_INNER_RADIUS)
        .extrude(FRONT_CHEEK_THICKNESS)
        .translate((0.0, TRUNNION_FRAME_FRONT_Y, 0.0))
    )
    bolts = cq.Workplane("XZ")
    for x_pos, z_pos in _polar_points(
        0.046,
        [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0],
    ):
        bolts = bolts.union(
            cq.Workplane("XZ")
            .center(x_pos, z_pos)
            .circle(0.0035)
            .extrude(0.013)
            .translate((0.0, TRUNNION_FRAME_FRONT_Y, 0.0))
        )
    tabs = cq.Workplane("XY")
    for x_sign in (-1.0, 1.0):
        for z_pos in (-0.096, 0.096):
            tab = cq.Workplane("XY").box(0.078, FRONT_CHEEK_THICKNESS, 0.022).translate(
                (x_sign * 0.096, TRUNNION_FRAME_FRONT_Y - FRONT_CHEEK_THICKNESS / 2.0, z_pos)
            )
            tabs = tabs.union(tab)
    return cheek.union(bolts).union(tabs)


def _make_rear_bearing_cheek() -> cq.Workplane:
    cheek = (
        cq.Workplane("XZ")
        .circle(REAR_CHEEK_OUTER_RADIUS)
        .circle(REAR_CHEEK_INNER_RADIUS)
        .extrude(REAR_CHEEK_THICKNESS)
        .translate((0.0, TRUNNION_FRAME_REAR_Y - REAR_CHEEK_THICKNESS, 0.0))
    )
    cover_pad = (
        cq.Workplane("XZ")
        .circle(0.044)
        .circle(0.020)
        .extrude(0.007)
        .translate((0.0, TRUNNION_FRAME_REAR_Y - REAR_CHEEK_THICKNESS - 0.007, 0.0))
    )
    bolts = cq.Workplane("XZ")
    for x_pos, z_pos in _polar_points(0.055, [15.0, 75.0, 135.0, 195.0, 255.0, 315.0]):
        bolts = bolts.union(
            cq.Workplane("XZ")
            .center(x_pos, z_pos)
            .circle(0.0035)
            .extrude(0.010)
            .translate((0.0, TRUNNION_FRAME_REAR_Y - REAR_CHEEK_THICKNESS - 0.010, 0.0))
        )
    tabs = cq.Workplane("XY")
    rear_y = TRUNNION_FRAME_REAR_Y - REAR_CHEEK_THICKNESS / 2.0
    for x_sign in (-1.0, 1.0):
        for z_pos in (-0.044, 0.044):
            tab = cq.Workplane("XY").box(0.050, REAR_CHEEK_THICKNESS, 0.018).translate(
                (x_sign * 0.085, rear_y, z_pos)
            )
            tabs = tabs.union(tab)
    return cheek.union(cover_pad).union(bolts).union(tabs)


def _make_trunnion_shaft(side: str) -> cq.Workplane:
    collar = (
        cq.Workplane("YZ")
        .circle(TRUNNION_COLLAR_RADIUS)
        .circle(TRUNNION_BORE_RADIUS)
        .extrude(BASE_INNER_FACE_X - TRUNNION_OUTER_FACE_X)
        .translate((TRUNNION_OUTER_FACE_X, 0.0, 0.0))
    )
    journal = (
        cq.Workplane("YZ")
        .circle(TRUNNION_JOURNAL_RADIUS)
        .extrude(BASE_OUTER_FACE_X - BASE_INNER_FACE_X + 0.020)
        .translate((BASE_INNER_FACE_X, 0.0, 0.0))
    )
    shoulder = (
        cq.Workplane("YZ")
        .circle(0.030)
        .circle(TRUNNION_JOURNAL_RADIUS)
        .extrude(0.006)
        .translate((TRUNNION_OUTER_FACE_X - 0.006, 0.0, 0.0))
    )
    end_pad = cq.Workplane("YZ").circle(0.014).extrude(0.006).translate(
        (BASE_OUTER_FACE_X + 0.020, 0.0, 0.0)
    )
    shaft = shoulder.union(collar).union(journal).union(end_pad)
    if side == "left":
        shaft = shaft.mirror("YZ")
    return shaft


def _make_sector_plate() -> cq.Workplane:
    profile_points = _annular_sector_points(0.060, 0.086, 205.0, 335.0)
    plate = cq.Workplane("YZ").polyline(profile_points).close().extrude(0.014)
    web = cq.Workplane("YZ").center(0.0, -0.054).rect(0.022, 0.048).extrude(0.014)
    clamp_pad = cq.Workplane("YZ").center(0.0, 0.060).rect(0.018, 0.028).extrude(0.014)
    neck = cq.Workplane("YZ").center(0.0, -0.024).rect(0.018, 0.036).extrude(0.014)
    shaft_collar = (
        cq.Workplane("YZ")
        .circle(0.032)
        .circle(0.012)
        .extrude(0.014)
    )
    sector = plate.union(web).union(clamp_pad).union(neck).union(shaft_collar).translate(
        (BASE_OUTER_FACE_X + 0.012, 0.0, 0.0)
    )
    return sector


def _make_faceplate() -> cq.Workplane:
    plate = (
        cq.Workplane("XZ")
        .circle(TABLE_RADIUS)
        .extrude(TABLE_FACE_THICKNESS)
        .translate((0.0, TABLE_FACE_FRONT_Y - TABLE_FACE_THICKNESS, 0.0))
    )
    pilot_bore = (
        cq.Workplane("XZ")
        .circle(0.016)
        .extrude(TABLE_FACE_THICKNESS + 0.004)
        .translate((0.0, TABLE_FACE_FRONT_Y - TABLE_FACE_THICKNESS - 0.002, 0.0))
    )
    plate = plate.cut(pilot_bore)

    for angle_deg in (0.0, 60.0, 120.0, 180.0, 240.0, 300.0):
        x_pos = 0.053 * math.cos(math.radians(angle_deg))
        z_pos = 0.053 * math.sin(math.radians(angle_deg))
        slot = (
            cq.Workplane("XZ")
            .center(x_pos, z_pos)
            .slot2D(0.050, 0.012, angle=angle_deg)
            .extrude(TABLE_FACE_THICKNESS + 0.004)
            .translate((0.0, TABLE_FACE_FRONT_Y - TABLE_FACE_THICKNESS - 0.002, 0.0))
        )
        plate = plate.cut(slot)

    annular_relief = (
        cq.Workplane("XZ")
        .circle(0.090)
        .circle(0.074)
        .extrude(0.0035)
        .translate((0.0, TABLE_FACE_FRONT_Y - 0.0035, 0.0))
    )
    hub_relief = (
        cq.Workplane("XZ")
        .circle(0.046)
        .circle(0.029)
        .extrude(0.0025)
        .translate((0.0, TABLE_FACE_FRONT_Y - 0.0025, 0.0))
    )
    plate = plate.cut(annular_relief).cut(hub_relief)

    bolt_recesses = cq.Workplane("XZ")
    for x_pos, z_pos in _polar_points(
        0.080,
        [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0],
    ):
        bolt_recesses = bolt_recesses.union(
            cq.Workplane("XZ")
            .center(x_pos, z_pos)
            .circle(0.004)
            .extrude(0.008)
            .translate((0.0, TABLE_FACE_FRONT_Y - 0.008, 0.0))
        )
    return plate.cut(bolt_recesses)


def _make_spindle_hub() -> cq.Workplane:
    shaft = cq.Workplane("XZ").circle(SPINDLE_RADIUS).extrude(0.084).translate((0.0, -0.057, 0.0))
    hub_barrel = (
        cq.Workplane("XZ")
        .circle(0.032)
        .extrude(0.044)
        .translate((0.0, -0.013, 0.0))
    )
    rear_thrust = (
        cq.Workplane("XZ")
        .circle(REAR_THRUST_RADIUS)
        .circle(SPINDLE_RADIUS)
        .extrude(0.008)
        .translate((0.0, -0.051, 0.0))
    )
    rear_drive_hub = (
        cq.Workplane("XZ")
        .circle(0.030)
        .circle(0.018)
        .extrude(0.012)
        .translate((0.0, -0.094, 0.0))
    )
    locknut = cq.Workplane("XZ").circle(0.026).extrude(0.008).translate((0.0, -0.098, 0.0))
    front_register = cq.Workplane("XZ").circle(0.040).extrude(0.016).translate((0.0, 0.003, 0.0))
    return shaft.union(hub_barrel).union(rear_thrust).union(rear_drive_hub).union(locknut).union(front_register)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_trunnion_table", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.32, 0.34, 0.37, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.67, 0.69, 0.71, 1.0))
    dark_cover = model.material("dark_cover", rgba=(0.23, 0.25, 0.28, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.57, 0.60, 0.63, 1.0))

    base = model.part("base")
    base.visual(_mesh(_make_base_structure(), "base_structure.obj"), material=cast_iron, name="base_structure")
    base.visual(_mesh(_make_bearing_cap("left"), "left_bearing_cap.obj"), material=machined_steel, name="left_bearing_cap")
    base.visual(_mesh(_make_bearing_cap("right"), "right_bearing_cap.obj"), material=machined_steel, name="right_bearing_cap")
    base.visual(_mesh(_make_access_cover("left"), "left_access_cover.obj"), material=dark_cover, name="left_access_cover")
    base.visual(_mesh(_make_access_cover("right"), "right_access_cover.obj"), material=dark_cover, name="right_access_cover")
    base.visual(_mesh(_make_sector_stop_bracket(), "sector_stop_bracket.obj"), material=machined_steel, name="sector_stop_bracket")
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_DEPTH, 0.23)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
    )

    trunnion = model.part("trunnion")
    trunnion.visual(_mesh(_make_frame_body(), "frame_body.obj"), material=cast_iron, name="frame_body")
    trunnion.visual(_mesh(_make_front_bearing_cheek(), "front_bearing_cheek.obj"), material=machined_steel, name="front_bearing_cheek")
    trunnion.visual(_mesh(_make_rear_bearing_cheek(), "rear_bearing_cheek.obj"), material=dark_cover, name="rear_bearing_cheek")
    trunnion.visual(_mesh(_make_trunnion_shaft("left"), "left_trunnion_shaft.obj"), material=satin_steel, name="left_trunnion_shaft")
    trunnion.visual(_mesh(_make_trunnion_shaft("right"), "right_trunnion_shaft.obj"), material=satin_steel, name="right_trunnion_shaft")
    trunnion.visual(_mesh(_make_sector_plate(), "sector_plate.obj"), material=machined_steel, name="sector_plate")
    trunnion.inertial = Inertial.from_geometry(
        Box((0.29, 0.11, 0.25)),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    table = model.part("table")
    table.visual(_mesh(_make_faceplate(), "faceplate.obj"), material=machined_steel, name="faceplate")
    table.visual(_mesh(_make_spindle_hub(), "spindle_hub.obj"), material=satin_steel, name="spindle_hub")
    table.inertial = Inertial.from_geometry(
        Box((0.21, 0.10, 0.21)),
        mass=11.0,
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
    )

    model.articulation(
        "base_to_trunnion_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=trunnion,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "trunnion_to_table_spin",
        ArticulationType.CONTINUOUS,
        parent=trunnion,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    trunnion = object_model.get_part("trunnion")
    table = object_model.get_part("table")
    tilt = object_model.get_articulation("base_to_trunnion_tilt")
    spin = object_model.get_articulation("trunnion_to_table_spin")

    base_structure = base.get_visual("base_structure")
    sector_stop_bracket = base.get_visual("sector_stop_bracket")
    left_cap = base.get_visual("left_bearing_cap")
    right_cap = base.get_visual("right_bearing_cap")

    frame_body = trunnion.get_visual("frame_body")
    rear_cheek = trunnion.get_visual("rear_bearing_cheek")
    front_cheek = trunnion.get_visual("front_bearing_cheek")
    left_shaft = trunnion.get_visual("left_trunnion_shaft")
    right_shaft = trunnion.get_visual("right_trunnion_shaft")
    sector_plate = trunnion.get_visual("sector_plate")

    faceplate = table.get_visual("faceplate")
    spindle_hub = table.get_visual("spindle_hub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        trunnion,
        base,
        elem_a=left_shaft,
        elem_b=base_structure,
        reason="Left trunnion journal runs inside the split base bearing saddle.",
    )
    ctx.allow_overlap(
        trunnion,
        base,
        elem_a=right_shaft,
        elem_b=base_structure,
        reason="Right trunnion journal runs inside the split base bearing saddle.",
    )
    ctx.allow_overlap(
        table,
        trunnion,
        elem_a=spindle_hub,
        elem_b=rear_cheek,
        reason="The rotary spindle hub is intentionally captured inside the rear bearing cheek.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "mechanism_parts_present",
        len(object_model.parts) == 3 and len(object_model.articulations) == 2,
        "Expected base, trunnion, and table with two articulations.",
    )
    ctx.expect_origin_distance(trunnion, base, axes="xy", max_dist=0.001, name="trunnion_axis_centered_over_base")
    ctx.expect_overlap(
        trunnion,
        base,
        elem_a=left_shaft,
        elem_b=base_structure,
        axes="yz",
        min_overlap=0.070,
        name="left_trunnion_shaft_aligned_with_left_support",
    )
    ctx.expect_overlap(
        trunnion,
        base,
        elem_a=right_shaft,
        elem_b=base_structure,
        axes="yz",
        min_overlap=0.070,
        name="right_trunnion_shaft_aligned_with_right_support",
    )
    ctx.expect_contact(
        table,
        trunnion,
        elem_a=spindle_hub,
        elem_b=rear_cheek,
        contact_tol=0.001,
        name="table_spindle_thrust_seated_in_rear_cheek",
    )
    ctx.expect_gap(
        trunnion,
        table,
        axis="y",
        positive_elem=front_cheek,
        negative_elem=faceplate,
        min_gap=0.008,
        max_gap=0.025,
        name="front_faceplate_clears_front_bearing_cheek",
    )
    ctx.expect_within(
        table,
        trunnion,
        axes="xz",
        margin=0.030,
        inner_elem=faceplate,
        outer_elem=frame_body,
        name="faceplate_nested_inside_trunnion_window",
    )
    ctx.expect_overlap(
        base,
        base,
        axes="yz",
        elem_a=left_cap,
        elem_b=base_structure,
        min_overlap=0.05,
        name="left_bearing_cap_mounted",
    )
    ctx.expect_overlap(
        base,
        base,
        axes="yz",
        elem_a=right_cap,
        elem_b=base_structure,
        min_overlap=0.05,
        name="right_bearing_cap_mounted",
    )
    ctx.expect_within(
        trunnion,
        base,
        axes="yz",
        margin=0.12,
        inner_elem=sector_plate,
        outer_elem=sector_stop_bracket,
        name="sector_plate_lives_within_stop_bracket_envelope",
    )

    with ctx.pose({tilt: 0.85}):
        ctx.expect_overlap(
            trunnion,
            base,
            elem_a=left_shaft,
            elem_b=base_structure,
            axes="yz",
            min_overlap=0.060,
            name="left_trunnion_shaft_stays_registered_at_positive_tilt",
        )
        ctx.expect_contact(
            table,
            trunnion,
            elem_a=spindle_hub,
            elem_b=rear_cheek,
            contact_tol=0.001,
            name="spindle_contact_persists_at_positive_tilt",
        )
        ctx.expect_origin_distance(
            table,
            trunnion,
            axes="xyz",
            max_dist=0.001,
            name="spin_axis_coincident_at_positive_tilt",
        )

    with ctx.pose({tilt: -0.85, spin: 1.7}):
        ctx.expect_overlap(
            trunnion,
            base,
            elem_a=right_shaft,
            elem_b=base_structure,
            axes="yz",
            min_overlap=0.060,
            name="right_trunnion_shaft_stays_registered_at_negative_tilt",
        )
        ctx.expect_contact(
            table,
            trunnion,
            elem_a=spindle_hub,
            elem_b=rear_cheek,
            contact_tol=0.001,
            name="spindle_contact_persists_while_spinning",
        )
        ctx.expect_within(
            table,
            trunnion,
            axes="xz",
            margin=0.030,
            inner_elem=faceplate,
            outer_elem=frame_body,
            name="faceplate_remains_nested_during_combined_motion",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
