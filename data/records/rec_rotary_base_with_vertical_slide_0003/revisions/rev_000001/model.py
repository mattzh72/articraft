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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_LENGTH = 0.340
BASE_WIDTH = 0.260
BASE_THICKNESS = 0.018
FOOT_LENGTH = 0.048
FOOT_WIDTH = 0.036
FOOT_HEIGHT = 0.010

PEDESTAL_LENGTH = 0.192
PEDESTAL_WIDTH = 0.160
PEDESTAL_HEIGHT = 0.088
PEDESTAL_WALL = 0.012
TOP_PLATE_THICKNESS = 0.014
TOP_PLATE_BORE_RADIUS = 0.043
BASE_OPENING_LENGTH = 0.134
BASE_OPENING_HEIGHT = 0.056
ACCESS_COVER_THICKNESS = 0.006

YAW_BEARING_BORE_RADIUS = 0.044
YAW_BEARING_OUTER_RADIUS = 0.072
YAW_BEARING_FLANGE_RADIUS = 0.082
YAW_BEARING_HEIGHT = 0.038
YAW_BEARING_FLANGE_THICKNESS = 0.008

YAW_HUB_RADIUS = 0.041
YAW_THRUST_FLANGE_RADIUS = 0.085
YAW_THRUST_FLANGE_BOTTOM = YAW_BEARING_HEIGHT
YAW_THRUST_FLANGE_THICKNESS = 0.014
YAW_SADDLE_BOTTOM = YAW_THRUST_FLANGE_BOTTOM + YAW_THRUST_FLANGE_THICKNESS
YAW_SADDLE_HEIGHT = 0.050
YAW_MOUNT_TOP = 0.116

COLUMN_BASE_LENGTH = 0.104
COLUMN_WIDTH = 0.075
COLUMN_BASE_THICKNESS = 0.012
COLUMN_BODY_LENGTH = 0.112
COLUMN_BODY_HEIGHT = 0.362
COLUMN_WALL = 0.010
COLUMN_TOP_CAP_THICKNESS = 0.018
COLUMN_REAR_OPENING_LENGTH = 0.074
COLUMN_REAR_OPENING_HEIGHT = 0.228
COLUMN_REAR_COVER_THICKNESS = 0.004

COLUMN_FRONT_FACE_Y = 0.5 * COLUMN_WIDTH
GUIDE_AXIS_Y = COLUMN_FRONT_FACE_Y + 0.018
GUIDE_SPAN = 0.300
GUIDE_RADIUS = 0.008
GUIDE_X = 0.036
LEADSCREW_RADIUS = 0.0065
LOWER_BRACKET_Z = 0.055
UPPER_BRACKET_Z = LOWER_BRACKET_Z + 0.260

COLLAR_OUTER_RADIUS = 0.015
COLLAR_HEIGHT = 0.010
LOWER_COLLAR_Z = 0.043
UPPER_COLLAR_Z = 0.334

CARRIAGE_REST_Z = 0.168
CARRIAGE_TRAVEL = 0.082
CARRIAGE_WIDTH = 0.128
CARRIAGE_BLOCK_DEPTH = 0.024
CARRIAGE_BLOCK_HEIGHT = 0.108
CARRIAGE_FRONT_PLATE_WIDTH = 0.104
CARRIAGE_FRONT_PLATE_DEPTH = 0.014
CARRIAGE_FRONT_PLATE_HEIGHT = 0.132
CARRIAGE_FRONT_OFFSET = 0.046
CARRIAGE_ARM_WIDTH = 0.018
CARRIAGE_ARM_DEPTH = 0.050
CARRIAGE_ARM_HEIGHT = 0.060
CARRIAGE_NUT_HOUSING_WIDTH = 0.040
CARRIAGE_NUT_HOUSING_DEPTH = 0.050
CARRIAGE_NUT_HOUSING_HEIGHT = 0.052
CARRIAGE_GUIDE_BORE_RADIUS = 0.0116
CARRIAGE_SCREW_BORE_RADIUS = 0.0126

GUIDE_CARTRIDGE_SLEEVE_OUTER_RADIUS = 0.0112
GUIDE_CARTRIDGE_HEIGHT = 0.066
GUIDE_CARTRIDGE_PAD_WIDTH = 0.022
GUIDE_CARTRIDGE_PAD_DEPTH = 0.010
GUIDE_CARTRIDGE_PAD_HEIGHT = 0.060

LEADNUT_CARTRIDGE_SLEEVE_OUTER_RADIUS = 0.0120
LEADNUT_CARTRIDGE_HEIGHT = 0.058
LEADNUT_CARTRIDGE_PAD_WIDTH = 0.048
LEADNUT_CARTRIDGE_PAD_DEPTH = 0.016
LEADNUT_CARTRIDGE_PAD_HEIGHT = 0.058


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(
        shape,
        filename,
        assets=ASSETS,
        tolerance=0.0005,
        angular_tolerance=0.05,
    )


def _polar_points(radius: float, count: int) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / count),
            radius * math.sin(2.0 * math.pi * index / count),
        )
        for index in range(count)
    ]


def _cover_shape(length: float, height: float, thickness: float, outward_sign: float = 1.0) -> cq.Workplane:
    standoff = 0.004
    pad_depth = 0.0035
    pad_width = 0.014
    pad_height = 0.014

    panel = (
        cq.Workplane("XY")
        .box(length, thickness, height, centered=(True, True, True))
        .translate((0.0, outward_sign * (standoff + 0.5 * thickness), 0.0))
    )
    side_rail = (
        cq.Workplane("XY")
        .box(0.012, thickness + 0.006, height - 0.018, centered=(True, True, True))
        .translate((0.0, outward_sign * (standoff + 0.5 * (thickness + 0.006)), 0.0))
    )
    top_rail = (
        cq.Workplane("XY")
        .box(length - 0.024, thickness + 0.004, 0.010, centered=(True, True, True))
        .translate((0.0, outward_sign * (standoff + 0.5 * (thickness + 0.004)), 0.5 * (height - 0.010)))
    )
    bottom_rail = (
        cq.Workplane("XY")
        .box(length - 0.024, thickness + 0.004, 0.010, centered=(True, True, True))
        .translate((0.0, outward_sign * (standoff + 0.5 * (thickness + 0.004)), -0.5 * (height - 0.010)))
    )
    cover = (
        panel
        .union(side_rail.translate((-0.5 * (length - 0.012), 0.0, 0.0)))
        .union(side_rail.translate((0.5 * (length - 0.012), 0.0, 0.0)))
        .union(top_rail)
        .union(bottom_rail)
    )
    center_bead = (
        cq.Workplane("XY")
        .box(length - 0.040, 0.003, height - 0.038, centered=(True, True, True))
        .translate((0.0, outward_sign * (standoff + thickness + 0.0015), 0.0))
    )
    cover = cover.union(center_bead)

    pad = (
        cq.Workplane("XY")
        .box(pad_width, pad_depth, pad_height, centered=(True, True, True))
        .translate((0.0, outward_sign * (0.5 * pad_depth), 0.0))
    )
    web = (
        cq.Workplane("XY")
        .box(0.008, standoff, 0.020, centered=(True, True, True))
        .translate((0.0, outward_sign * (0.5 * standoff), 0.0))
    )
    screw_boss = (
        cq.Workplane("XY")
        .circle(0.0038)
        .extrude(thickness * 0.6)
        .translate((0.0, outward_sign * (standoff + thickness), 0.0))
    )
    for x_pos, z_pos in (
        (-0.5 * (length - 0.024), 0.5 * (height - 0.024)),
        (0.5 * (length - 0.024), 0.5 * (height - 0.024)),
        (-0.5 * (length - 0.024), -0.5 * (height - 0.024)),
        (0.5 * (length - 0.024), -0.5 * (height - 0.024)),
    ):
        cover = cover.union(pad.translate((x_pos, 0.0, z_pos)))
        cover = cover.union(web.translate((x_pos, outward_sign * 0.5 * standoff, z_pos)))
        cover = cover.union(screw_boss.translate((x_pos, 0.0, z_pos)))
    return cover


def _base_frame_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
    foot_offsets = [
        (-0.5 * (BASE_LENGTH - FOOT_LENGTH - 0.020), -0.5 * (BASE_WIDTH - FOOT_WIDTH - 0.020)),
        (0.5 * (BASE_LENGTH - FOOT_LENGTH - 0.020), -0.5 * (BASE_WIDTH - FOOT_WIDTH - 0.020)),
        (-0.5 * (BASE_LENGTH - FOOT_LENGTH - 0.020), 0.5 * (BASE_WIDTH - FOOT_WIDTH - 0.020)),
        (0.5 * (BASE_LENGTH - FOOT_LENGTH - 0.020), 0.5 * (BASE_WIDTH - FOOT_WIDTH - 0.020)),
    ]
    for x_pos, y_pos in foot_offsets:
        base = base.union(
            cq.Workplane("XY")
            .box(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT, centered=(True, True, False))
            .translate((x_pos, y_pos, -FOOT_HEIGHT))
        )

    pedestal_outer = (
        cq.Workplane("XY")
        .box(PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    pedestal_inner = (
        cq.Workplane("XY")
        .box(
            PEDESTAL_LENGTH - 2.0 * PEDESTAL_WALL,
            PEDESTAL_WIDTH - 2.0 * PEDESTAL_WALL,
            PEDESTAL_HEIGHT - PEDESTAL_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_THICKNESS + PEDESTAL_WALL))
    )
    pedestal = pedestal_outer.cut(pedestal_inner)

    front_opening = (
        cq.Workplane("XY")
        .box(BASE_OPENING_LENGTH, 0.050, BASE_OPENING_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.5 * PEDESTAL_WIDTH - 0.015, BASE_THICKNESS + 0.018))
    )
    rear_opening = (
        cq.Workplane("XY")
        .box(BASE_OPENING_LENGTH, 0.050, BASE_OPENING_HEIGHT, centered=(True, True, False))
        .translate((0.0, -0.5 * PEDESTAL_WIDTH + 0.015, BASE_THICKNESS + 0.018))
    )
    pedestal = pedestal.cut(front_opening).cut(rear_opening)

    top_plate = (
        cq.Workplane("XY")
        .box(PEDESTAL_LENGTH, PEDESTAL_WIDTH, TOP_PLATE_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT))
    )
    top_bore = (
        cq.Workplane("XY")
        .circle(TOP_PLATE_BORE_RADIUS)
        .extrude(TOP_PLATE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT))
    )
    top_plate = top_plate.cut(top_bore)

    side_rib = (
        cq.Workplane("XZ")
        .polyline([(0.0, 0.0), (0.0, 0.055), (0.050, 0.0)])
        .close()
        .extrude(0.010)
    )
    left_rib = side_rib.translate(
        (
            -0.5 * PEDESTAL_LENGTH + 0.020,
            0.5 * PEDESTAL_WIDTH - 0.010,
            BASE_THICKNESS,
        )
    )
    right_rib = side_rib.mirror("YZ").translate(
        (
            0.5 * PEDESTAL_LENGTH - 0.020,
            -0.5 * PEDESTAL_WIDTH,
            BASE_THICKNESS,
        )
    )

    return base.union(pedestal).union(top_plate).union(left_rib).union(right_rib)


def _yaw_bearing_shape() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .circle(YAW_BEARING_FLANGE_RADIUS)
        .circle(YAW_BEARING_BORE_RADIUS)
        .extrude(YAW_BEARING_FLANGE_THICKNESS)
    )
    sleeve = (
        cq.Workplane("XY")
        .circle(YAW_BEARING_OUTER_RADIUS)
        .circle(YAW_BEARING_BORE_RADIUS)
        .extrude(YAW_BEARING_HEIGHT)
    )
    shape = flange.union(sleeve)
    for x_pos, y_pos in _polar_points(0.064, 6):
        shape = shape.cut(
            cq.Workplane("XY")
            .center(x_pos, y_pos)
            .circle(0.0045)
            .extrude(YAW_BEARING_HEIGHT)
        )
    return shape


def _yaw_rotor_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(YAW_HUB_RADIUS).extrude(YAW_THRUST_FLANGE_BOTTOM + YAW_THRUST_FLANGE_THICKNESS)
    thrust_flange = (
        cq.Workplane("XY")
        .circle(YAW_THRUST_FLANGE_RADIUS)
        .extrude(YAW_THRUST_FLANGE_THICKNESS)
        .translate((0.0, 0.0, YAW_THRUST_FLANGE_BOTTOM))
    )
    saddle = (
        cq.Workplane("XY")
        .box(0.144, 0.100, YAW_SADDLE_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, YAW_SADDLE_BOTTOM))
    )
    saddle_window = (
        cq.Workplane("XY")
        .box(0.086, 0.060, 0.032, centered=(True, True, False))
        .translate((0.0, 0.0, YAW_SADDLE_BOTTOM + 0.008))
    )
    saddle = saddle.cut(saddle_window)

    mount_plate = (
        cq.Workplane("XY")
        .box(0.104, 0.078, YAW_MOUNT_TOP - (YAW_SADDLE_BOTTOM + YAW_SADDLE_HEIGHT), centered=(True, True, False))
        .translate((0.0, 0.0, YAW_SADDLE_BOTTOM + YAW_SADDLE_HEIGHT))
    )
    cheek = (
        cq.Workplane("XZ")
        .polyline([(0.0, 0.0), (0.020, 0.0), (0.0, 0.030)])
        .close()
        .extrude(0.018)
    )
    left_cheek = cheek.translate((-0.055, 0.041, YAW_SADDLE_BOTTOM + 0.010))
    right_cheek = cheek.mirror("YZ").translate((0.055, -0.059, YAW_SADDLE_BOTTOM + 0.010))
    return hub.union(thrust_flange).union(saddle).union(mount_plate).union(left_cheek).union(right_cheek)


def _column_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(COLUMN_BASE_LENGTH, COLUMN_WIDTH, COLUMN_BASE_THICKNESS, centered=(True, True, False))
    mast_outer = (
        cq.Workplane("XY")
        .box(COLUMN_BODY_LENGTH, COLUMN_WIDTH, COLUMN_BODY_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, COLUMN_BASE_THICKNESS))
    )
    mast_inner = (
        cq.Workplane("XY")
        .box(
            COLUMN_BODY_LENGTH - 2.0 * COLUMN_WALL,
            COLUMN_WIDTH - 2.0 * COLUMN_WALL,
            COLUMN_BODY_HEIGHT - COLUMN_TOP_CAP_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, COLUMN_BASE_THICKNESS + COLUMN_WALL))
    )
    mast = mast_outer.cut(mast_inner)
    rear_opening = (
        cq.Workplane("XY")
        .box(COLUMN_REAR_OPENING_LENGTH, 0.020, COLUMN_REAR_OPENING_HEIGHT, centered=(True, True, False))
        .translate((0.0, -0.5 * COLUMN_WIDTH + 0.010, 0.082))
    )
    mast = mast.cut(rear_opening)
    front_land = (
        cq.Workplane("XY")
        .box(0.088, 0.006, 0.292, centered=(True, True, False))
        .translate((0.0, 0.5 * COLUMN_WIDTH - 0.006, 0.050))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.096, 0.070, COLUMN_TOP_CAP_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, COLUMN_BASE_THICKNESS + COLUMN_BODY_HEIGHT))
    )
    side_web = (
        cq.Workplane("YZ")
        .polyline([(0.0, 0.0), (0.0, 0.055), (0.022, 0.0)])
        .close()
        .extrude(0.012)
    )
    left_web = side_web.translate((-0.050, 0.020, COLUMN_BASE_THICKNESS))
    right_web = side_web.mirror("XZ").translate((0.062, -0.020, COLUMN_BASE_THICKNESS))
    return base.union(mast).union(front_land).union(top_cap).union(left_web).union(right_web)


def _bridge_bracket_shape(upper: bool) -> cq.Workplane:
    top_plate_z = -0.012 if upper else 0.0
    back_wall_z = -0.050 if upper else 0.0
    rib_z = -0.040 if upper else 0.0

    plate = (
        cq.Workplane("XY")
        .box(0.108, 0.018, 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, top_plate_z))
    )
    back_pad = (
        cq.Workplane("XY")
        .box(0.094, 0.010, 0.050, centered=(True, False, False))
        .translate((0.0, -0.0135, back_wall_z))
    )
    rib_left = (
        cq.Workplane("XY")
        .box(0.012, 0.022, 0.040, centered=(True, False, False))
        .translate((-0.038, -0.010, rib_z))
    )
    rib_right = (
        cq.Workplane("XY")
        .box(0.012, 0.022, 0.040, centered=(True, False, False))
        .translate((0.038, -0.010, rib_z))
    )
    center_rib = (
        cq.Workplane("XY")
        .box(0.014, 0.020, 0.032, centered=(True, False, False))
        .translate((0.0, -0.010, -0.032 if upper else 0.0))
    )
    shape = plate.union(back_pad).union(rib_left).union(rib_right).union(center_rib)
    support_specs = (
        (-GUIDE_X, 0.011),
        (0.0, 0.010),
        (GUIDE_X, 0.011),
    )
    for x_pos, radius in support_specs:
        bore_radius = 0.0085 if abs(x_pos) < 1e-9 else 0.0095
        boss = (
            cq.Workplane("XY")
            .center(x_pos, 0.0)
            .circle(radius)
            .extrude(0.008)
        )
        if upper:
            boss = boss.translate((0.0, 0.0, -0.008))
        else:
            boss = boss.translate((0.0, 0.0, 0.0))
        shape = shape.union(boss)
        bore = (
            cq.Workplane("XY")
            .center(x_pos, 0.0)
            .circle(bore_radius)
            .extrude(0.008)
        )
        if upper:
            bore = bore.translate((0.0, 0.0, -0.008))
        shape = shape.cut(bore)
    return shape


def _rod_shape(radius: float, length: float, shoulder_radius: float | None = None, shoulder_length: float = 0.008) -> cq.Workplane:
    rod = cq.Workplane("XY").circle(radius).extrude(length)
    if shoulder_radius is not None:
        lower = cq.Workplane("XY").circle(shoulder_radius).extrude(shoulder_length)
        upper = cq.Workplane("XY").circle(shoulder_radius).extrude(shoulder_length).translate((0.0, 0.0, length - shoulder_length))
        rod = rod.union(lower).union(upper)
    return rod


def _guide_shaft_shape() -> cq.Workplane:
    body = cq.Workplane("XY").circle(GUIDE_RADIUS).extrude(GUIDE_SPAN + 0.024).translate((0.0, 0.0, -0.012))
    lower_journal = cq.Workplane("XY").circle(0.0095).extrude(0.010).translate((0.0, 0.0, -0.012))
    upper_journal = cq.Workplane("XY").circle(0.0095).extrude(0.010).translate((0.0, 0.0, GUIDE_SPAN + 0.002))
    return body.union(lower_journal).union(upper_journal)


def _split_collar_shape(inner_radius: float, outer_radius: float, height: float, outward_sign: float = 1.0) -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )
    slit = (
        cq.Workplane("XY")
        .box(outer_radius * 2.0, 0.004, height + 0.002, centered=(True, True, False))
        .translate((outward_sign * outer_radius, 0.0, 0.0))
    )
    ear = (
        cq.Workplane("XY")
        .box(0.010, 0.008, height, centered=(True, True, False))
        .translate((outward_sign * (outer_radius + 0.005), 0.0, 0.0))
    )
    bolt = (
        cq.Workplane("YZ")
        .workplane(offset=outward_sign * (outer_radius + 0.010))
        .circle(0.0025)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.001))
    )
    return collar.cut(slit).union(ear).union(bolt)


def _linear_bearing_cartridge_shape(side_sign: float) -> cq.Workplane:
    sleeve = (
        cq.Workplane("XY")
        .circle(GUIDE_CARTRIDGE_SLEEVE_OUTER_RADIUS)
        .circle(GUIDE_RADIUS)
        .extrude(GUIDE_CARTRIDGE_HEIGHT)
        .translate((0.0, 0.0, -0.5 * GUIDE_CARTRIDGE_HEIGHT))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.010, 0.012, GUIDE_CARTRIDGE_PAD_HEIGHT, centered=(True, True, True))
        .translate((side_sign * (GUIDE_CARTRIDGE_SLEEVE_OUTER_RADIUS + 0.005), 0.0, 0.0))
    )
    pad_center_x = side_sign * (GUIDE_CARTRIDGE_SLEEVE_OUTER_RADIUS + 0.5 * GUIDE_CARTRIDGE_PAD_WIDTH)
    front_pad = (
        cq.Workplane("XY")
        .box(
            GUIDE_CARTRIDGE_PAD_WIDTH,
            GUIDE_CARTRIDGE_PAD_DEPTH,
            GUIDE_CARTRIDGE_PAD_HEIGHT,
            centered=(True, True, True),
        )
        .translate((pad_center_x, 0.5 * CARRIAGE_BLOCK_DEPTH + 0.5 * GUIDE_CARTRIDGE_PAD_DEPTH, 0.0))
    )
    ridge = (
        cq.Workplane("XY")
        .box(0.008, 0.006, GUIDE_CARTRIDGE_PAD_HEIGHT * 0.88, centered=(True, True, True))
        .translate((pad_center_x, 0.5 * CARRIAGE_BLOCK_DEPTH + GUIDE_CARTRIDGE_PAD_DEPTH + 0.003, 0.0))
    )
    shape = sleeve.union(bridge).union(front_pad).union(ridge)
    for z_pos in (-0.020, 0.020):
        hole = (
            cq.Workplane("XZ")
            .center(pad_center_x, z_pos)
            .circle(0.0028)
            .extrude(GUIDE_CARTRIDGE_PAD_DEPTH + 0.008)
            .translate((0.0, 0.5 * CARRIAGE_BLOCK_DEPTH - 0.002, 0.0))
        )
        shape = shape.cut(hole)
    return shape


def _leadnut_cartridge_shape() -> cq.Workplane:
    sleeve = (
        cq.Workplane("XY")
        .circle(LEADNUT_CARTRIDGE_SLEEVE_OUTER_RADIUS)
        .circle(LEADSCREW_RADIUS)
        .extrude(LEADNUT_CARTRIDGE_HEIGHT)
        .translate((0.0, 0.0, -0.5 * LEADNUT_CARTRIDGE_HEIGHT))
    )
    front_pad = (
        cq.Workplane("XY")
        .box(
            LEADNUT_CARTRIDGE_PAD_WIDTH,
            LEADNUT_CARTRIDGE_PAD_DEPTH,
            LEADNUT_CARTRIDGE_PAD_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, 0.5 * CARRIAGE_BLOCK_DEPTH + 0.5 * LEADNUT_CARTRIDGE_PAD_DEPTH, 0.0))
    )
    center_boss = (
        cq.Workplane("XY")
        .circle(0.016)
        .extrude(0.006)
        .translate((0.0, 0.5 * CARRIAGE_BLOCK_DEPTH + LEADNUT_CARTRIDGE_PAD_DEPTH, -0.003))
    )
    leadnut = sleeve.union(front_pad).union(center_boss)
    for x_pos in (-0.015, 0.015):
        bolt = (
            cq.Workplane("XZ")
            .center(x_pos, 0.0)
            .circle(0.0030)
            .extrude(LEADNUT_CARTRIDGE_PAD_DEPTH + 0.008)
            .translate((0.0, 0.5 * CARRIAGE_BLOCK_DEPTH - 0.002, 0.0))
        )
        leadnut = leadnut.cut(bolt)
    pocket = (
        cq.Workplane("XY")
        .box(0.026, LEADNUT_CARTRIDGE_PAD_DEPTH + 0.002, 0.030, centered=(True, True, True))
        .translate((0.0, 0.5 * CARRIAGE_BLOCK_DEPTH + 0.5 * LEADNUT_CARTRIDGE_PAD_DEPTH, 0.0))
    )
    return leadnut.cut(pocket)


def _carriage_shape() -> cq.Workplane:
    back_block = cq.Workplane("XY").box(CARRIAGE_WIDTH, CARRIAGE_BLOCK_DEPTH, CARRIAGE_BLOCK_HEIGHT)
    front_plate = (
        cq.Workplane("XY")
        .box(CARRIAGE_FRONT_PLATE_WIDTH, CARRIAGE_FRONT_PLATE_DEPTH, CARRIAGE_FRONT_PLATE_HEIGHT)
        .translate((0.0, CARRIAGE_FRONT_OFFSET, 0.006))
    )
    left_arm = (
        cq.Workplane("XY")
        .box(CARRIAGE_ARM_WIDTH, CARRIAGE_ARM_DEPTH, CARRIAGE_ARM_HEIGHT)
        .translate((-0.045, 0.020, 0.0))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(CARRIAGE_ARM_WIDTH, CARRIAGE_ARM_DEPTH, CARRIAGE_ARM_HEIGHT)
        .translate((0.045, 0.020, 0.0))
    )
    gusset = (
        cq.Workplane("YZ")
        .polyline([(0.0, -0.030), (0.0, 0.030), (0.020, -0.008)])
        .close()
        .extrude(0.010)
    )
    left_gusset = gusset.translate((-0.020, 0.010, 0.0))
    right_gusset = gusset.mirror("XZ").translate((0.010, 0.010, 0.0))
    carriage = back_block.union(front_plate).union(left_arm).union(right_arm).union(left_gusset).union(right_gusset)

    for x_pos in (-GUIDE_X, GUIDE_X):
        carriage = carriage.cut(
            cq.Workplane("XY")
            .center(x_pos, 0.0)
            .circle(CARRIAGE_GUIDE_BORE_RADIUS)
            .extrude(CARRIAGE_BLOCK_HEIGHT + 0.020)
            .translate((0.0, 0.0, -0.5 * (CARRIAGE_BLOCK_HEIGHT + 0.020)))
        )
    carriage = carriage.cut(
        cq.Workplane("XY")
        .center(0.0, 0.0)
        .circle(CARRIAGE_SCREW_BORE_RADIUS)
        .extrude(CARRIAGE_BLOCK_HEIGHT + 0.020)
        .translate((0.0, 0.0, -0.5 * (CARRIAGE_BLOCK_HEIGHT + 0.020)))
    )

    pocket = (
        cq.Workplane("XY")
        .box(0.070, 0.012, 0.070)
        .translate((0.0, 0.0, 0.0))
    )
    front_window = (
        cq.Workplane("XY")
        .box(0.058, CARRIAGE_FRONT_PLATE_DEPTH + 0.004, 0.086)
        .translate((0.0, CARRIAGE_FRONT_OFFSET, 0.012))
    )
    carriage = carriage.cut(pocket).cut(front_window)
    for x_pos, z_pos in (
        (-0.034, 0.040),
        (0.034, 0.040),
        (-0.034, -0.040),
        (0.034, -0.040),
    ):
        carriage = carriage.cut(
            cq.Workplane("XZ")
            .center(x_pos, z_pos)
            .circle(0.0032)
            .extrude(CARRIAGE_FRONT_PLATE_DEPTH + 0.008)
            .translate((0.0, CARRIAGE_FRONT_OFFSET - 0.5 * CARRIAGE_FRONT_PLATE_DEPTH - 0.004, 0.0))
        )
    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_lift_study", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.31, 0.34, 0.37, 1.0))
    light_cover = model.material("light_cover", rgba=(0.63, 0.66, 0.69, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.18, 0.19, 0.21, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(_mesh(_base_frame_shape(), "base_frame.obj"), material=painted_steel, name="base_frame_body")

    front_access_cover = model.part("front_access_cover")
    front_access_cover.visual(
        _mesh(
            _cover_shape(
                BASE_OPENING_LENGTH + 0.024,
                BASE_OPENING_HEIGHT + 0.020,
                ACCESS_COVER_THICKNESS,
                outward_sign=1.0,
            ),
            "front_access_cover.obj",
        ),
        material=light_cover,
        name="front_access_cover_body",
    )

    rear_access_cover = model.part("rear_access_cover")
    rear_access_cover.visual(
        _mesh(
            _cover_shape(
                BASE_OPENING_LENGTH + 0.024,
                BASE_OPENING_HEIGHT + 0.020,
                ACCESS_COVER_THICKNESS,
                outward_sign=-1.0,
            ),
            "rear_access_cover.obj",
        ),
        material=light_cover,
        name="rear_access_cover_body",
    )

    yaw_bearing_outer = model.part("yaw_bearing_outer")
    yaw_bearing_outer.visual(_mesh(_yaw_bearing_shape(), "yaw_bearing_outer.obj"), material=ground_steel, name="yaw_bearing_outer_body")

    yaw_rotor = model.part("yaw_rotor")
    yaw_rotor.visual(_mesh(_yaw_rotor_shape(), "yaw_rotor.obj"), material=painted_steel, name="yaw_rotor_body")

    column = model.part("column")
    column.visual(_mesh(_column_shape(), "column.obj"), material=painted_steel, name="column_body")

    column_rear_cover = model.part("column_rear_cover")
    column_rear_cover.visual(
        _mesh(
            _cover_shape(
                COLUMN_REAR_OPENING_LENGTH + 0.022,
                COLUMN_REAR_OPENING_HEIGHT + 0.022,
                COLUMN_REAR_COVER_THICKNESS,
                outward_sign=-1.0,
            ),
            "column_rear_cover.obj",
        ),
        material=light_cover,
        name="column_rear_cover_body",
    )

    lower_guide_bracket = model.part("lower_guide_bracket")
    lower_guide_bracket.visual(
        _mesh(_bridge_bracket_shape(upper=False), "lower_guide_bracket.obj"),
        material=painted_steel,
        name="lower_guide_bracket_body",
    )

    upper_guide_bracket = model.part("upper_guide_bracket")
    upper_guide_bracket.visual(
        _mesh(_bridge_bracket_shape(upper=True), "upper_guide_bracket.obj"),
        material=painted_steel,
        name="upper_guide_bracket_body",
    )

    left_guide_shaft = model.part("left_guide_shaft")
    left_guide_shaft.visual(
        _mesh(_guide_shaft_shape(), "left_guide_shaft.obj"),
        material=ground_steel,
        name="left_guide_shaft_body",
    )

    right_guide_shaft = model.part("right_guide_shaft")
    right_guide_shaft.visual(
        _mesh(_guide_shaft_shape(), "right_guide_shaft.obj"),
        material=ground_steel,
        name="right_guide_shaft_body",
    )

    lead_screw = model.part("lead_screw")
    lead_screw.visual(
        _mesh(_rod_shape(LEADSCREW_RADIUS, GUIDE_SPAN, shoulder_radius=0.0085), "lead_screw.obj"),
        material=dark_oxide,
        name="lead_screw_body",
    )

    lower_stop_collar = model.part("lower_stop_collar")
    lower_stop_collar.visual(
        _mesh(_split_collar_shape(GUIDE_RADIUS - 0.00025, COLLAR_OUTER_RADIUS, COLLAR_HEIGHT, outward_sign=-1.0), "lower_stop_collar.obj"),
        material=dark_oxide,
        name="lower_stop_collar_body",
    )

    upper_stop_collar = model.part("upper_stop_collar")
    upper_stop_collar.visual(
        _mesh(_split_collar_shape(GUIDE_RADIUS - 0.00025, COLLAR_OUTER_RADIUS, COLLAR_HEIGHT, outward_sign=1.0), "upper_stop_collar.obj"),
        material=dark_oxide,
        name="upper_stop_collar_body",
    )

    lift_carriage = model.part("lift_carriage")
    lift_carriage.visual(_mesh(_carriage_shape(), "lift_carriage.obj"), material=light_cover, name="lift_carriage_body")

    left_linear_bearing_cartridge = model.part("left_linear_bearing_cartridge")
    left_linear_bearing_cartridge.visual(
        _mesh(_linear_bearing_cartridge_shape(side_sign=-1.0), "left_linear_bearing_cartridge.obj"),
        material=ground_steel,
        name="left_linear_bearing_cartridge_body",
    )

    right_linear_bearing_cartridge = model.part("right_linear_bearing_cartridge")
    right_linear_bearing_cartridge.visual(
        _mesh(_linear_bearing_cartridge_shape(side_sign=1.0), "right_linear_bearing_cartridge.obj"),
        material=ground_steel,
        name="right_linear_bearing_cartridge_body",
    )

    leadnut_cartridge = model.part("leadnut_cartridge")
    leadnut_cartridge.visual(
        _mesh(_leadnut_cartridge_shape(), "leadnut_cartridge.obj"),
        material=dark_oxide,
        name="leadnut_cartridge_body",
    )

    base_top_z = BASE_THICKNESS + PEDESTAL_HEIGHT + TOP_PLATE_THICKNESS

    model.articulation(
        "base_to_front_access_cover",
        ArticulationType.FIXED,
        parent=base_frame,
        child=front_access_cover,
        origin=Origin(
            xyz=(
                0.0,
                0.5 * PEDESTAL_WIDTH,
                BASE_THICKNESS + 0.018 + 0.5 * BASE_OPENING_HEIGHT,
            )
        ),
    )
    model.articulation(
        "base_to_rear_access_cover",
        ArticulationType.FIXED,
        parent=base_frame,
        child=rear_access_cover,
        origin=Origin(
            xyz=(
                0.0,
                -0.5 * PEDESTAL_WIDTH,
                BASE_THICKNESS + 0.018 + 0.5 * BASE_OPENING_HEIGHT,
            ),
        ),
    )
    model.articulation(
        "base_to_yaw_bearing_outer",
        ArticulationType.FIXED,
        parent=base_frame,
        child=yaw_bearing_outer,
        origin=Origin(xyz=(0.0, 0.0, base_top_z)),
    )
    model.articulation(
        "base_to_yaw_rotor",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=yaw_rotor,
        origin=Origin(xyz=(0.0, 0.0, base_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.8),
    )
    model.articulation(
        "yaw_rotor_to_column",
        ArticulationType.FIXED,
        parent=yaw_rotor,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, YAW_MOUNT_TOP)),
    )
    model.articulation(
        "column_to_rear_cover",
        ArticulationType.FIXED,
        parent=column,
        child=column_rear_cover,
        origin=Origin(
            xyz=(
                0.0,
                -0.5 * COLUMN_WIDTH,
                0.082 + 0.5 * COLUMN_REAR_OPENING_HEIGHT,
            ),
        ),
    )
    model.articulation(
        "column_to_lower_bracket",
        ArticulationType.FIXED,
        parent=column,
        child=lower_guide_bracket,
        origin=Origin(xyz=(0.0, GUIDE_AXIS_Y, LOWER_BRACKET_Z)),
    )
    model.articulation(
        "column_to_upper_bracket",
        ArticulationType.FIXED,
        parent=column,
        child=upper_guide_bracket,
        origin=Origin(xyz=(0.0, GUIDE_AXIS_Y, UPPER_BRACKET_Z)),
    )
    model.articulation(
        "column_to_left_guide_shaft",
        ArticulationType.FIXED,
        parent=column,
        child=left_guide_shaft,
        origin=Origin(xyz=(-GUIDE_X, GUIDE_AXIS_Y, LOWER_BRACKET_Z)),
    )
    model.articulation(
        "column_to_right_guide_shaft",
        ArticulationType.FIXED,
        parent=column,
        child=right_guide_shaft,
        origin=Origin(xyz=(GUIDE_X, GUIDE_AXIS_Y, LOWER_BRACKET_Z)),
    )
    model.articulation(
        "column_to_lead_screw",
        ArticulationType.FIXED,
        parent=column,
        child=lead_screw,
        origin=Origin(xyz=(0.0, GUIDE_AXIS_Y, LOWER_BRACKET_Z)),
    )
    model.articulation(
        "column_to_lower_stop_collar",
        ArticulationType.FIXED,
        parent=column,
        child=lower_stop_collar,
        origin=Origin(xyz=(-GUIDE_X, GUIDE_AXIS_Y, LOWER_COLLAR_Z)),
    )
    model.articulation(
        "column_to_upper_stop_collar",
        ArticulationType.FIXED,
        parent=column,
        child=upper_stop_collar,
        origin=Origin(xyz=(GUIDE_X, GUIDE_AXIS_Y, UPPER_COLLAR_Z)),
    )
    model.articulation(
        "column_to_lift_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=lift_carriage,
        origin=Origin(xyz=(0.0, GUIDE_AXIS_Y, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=CARRIAGE_TRAVEL),
    )
    model.articulation(
        "carriage_to_left_linear_bearing_cartridge",
        ArticulationType.FIXED,
        parent=lift_carriage,
        child=left_linear_bearing_cartridge,
        origin=Origin(xyz=(-GUIDE_X, 0.0, 0.0)),
    )
    model.articulation(
        "carriage_to_right_linear_bearing_cartridge",
        ArticulationType.FIXED,
        parent=lift_carriage,
        child=right_linear_bearing_cartridge,
        origin=Origin(xyz=(GUIDE_X, 0.0, 0.0)),
    )
    model.articulation(
        "carriage_to_leadnut_cartridge",
        ArticulationType.FIXED,
        parent=lift_carriage,
        child=leadnut_cartridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    front_access_cover = object_model.get_part("front_access_cover")
    rear_access_cover = object_model.get_part("rear_access_cover")
    yaw_bearing_outer = object_model.get_part("yaw_bearing_outer")
    yaw_rotor = object_model.get_part("yaw_rotor")
    column = object_model.get_part("column")
    column_rear_cover = object_model.get_part("column_rear_cover")
    lower_guide_bracket = object_model.get_part("lower_guide_bracket")
    upper_guide_bracket = object_model.get_part("upper_guide_bracket")
    left_guide_shaft = object_model.get_part("left_guide_shaft")
    right_guide_shaft = object_model.get_part("right_guide_shaft")
    lead_screw = object_model.get_part("lead_screw")
    lower_stop_collar = object_model.get_part("lower_stop_collar")
    upper_stop_collar = object_model.get_part("upper_stop_collar")
    lift_carriage = object_model.get_part("lift_carriage")
    left_linear_bearing_cartridge = object_model.get_part("left_linear_bearing_cartridge")
    right_linear_bearing_cartridge = object_model.get_part("right_linear_bearing_cartridge")
    leadnut_cartridge = object_model.get_part("leadnut_cartridge")
    yaw_joint = object_model.get_articulation("base_to_yaw_rotor")
    lift_joint = object_model.get_articulation("column_to_lift_carriage")

    ctx.check("yaw_joint_is_continuous", yaw_joint.articulation_type == ArticulationType.CONTINUOUS, "yaw joint must be continuous")
    ctx.check("lift_joint_is_prismatic", lift_joint.articulation_type == ArticulationType.PRISMATIC, "lift stage must be prismatic")
    ctx.check("yaw_joint_axis_z", tuple(yaw_joint.axis) == (0.0, 0.0, 1.0), f"unexpected yaw axis: {yaw_joint.axis}")
    ctx.check("lift_joint_axis_z", tuple(lift_joint.axis) == (0.0, 0.0, 1.0), f"unexpected lift axis: {lift_joint.axis}")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap(lower_guide_bracket, left_guide_shaft, reason="lower bracket uses an integrated guide bushing around the left shaft")
    ctx.allow_overlap(lower_guide_bracket, right_guide_shaft, reason="lower bracket uses an integrated guide bushing around the right shaft")
    ctx.allow_overlap(lower_guide_bracket, lead_screw, reason="lower bracket captures the lead screw support bearing cartridge")
    ctx.allow_overlap(upper_guide_bracket, left_guide_shaft, reason="upper bracket uses an integrated guide bushing around the left shaft")
    ctx.allow_overlap(upper_guide_bracket, right_guide_shaft, reason="upper bracket uses an integrated guide bushing around the right shaft")
    ctx.allow_overlap(upper_guide_bracket, lead_screw, reason="upper bracket captures the lead screw support bearing cartridge")
    ctx.allow_overlap(lower_stop_collar, left_guide_shaft, reason="lower stop collar is modeled as a clamped split collar on the guide shaft")
    ctx.allow_overlap(upper_stop_collar, right_guide_shaft, reason="upper stop collar is modeled as a clamped split collar on the guide shaft")
    ctx.allow_overlap(yaw_bearing_outer, yaw_rotor, reason="rotary hub nests concentrically inside the slewing bearing envelope")
    ctx.allow_overlap(front_access_cover, base_frame, reason="front access cover seats into the pedestal service opening with a recessed perimeter frame")
    ctx.allow_overlap(rear_access_cover, base_frame, reason="rear access cover seats into the pedestal service opening with a recessed perimeter frame")
    ctx.allow_overlap(left_linear_bearing_cartridge, left_guide_shaft, reason="left cartridge is a sleeved linear bearing wrapped around the left guide shaft")
    ctx.allow_overlap(right_linear_bearing_cartridge, right_guide_shaft, reason="right cartridge is a sleeved linear bearing wrapped around the right guide shaft")
    ctx.allow_overlap(lift_carriage, left_linear_bearing_cartridge, reason="left bearing cartridge is captured inside the carriage body")
    ctx.allow_overlap(lift_carriage, right_linear_bearing_cartridge, reason="right bearing cartridge is captured inside the carriage body")
    ctx.allow_overlap(lead_screw, leadnut_cartridge, reason="leadnut cartridge encloses the traveling screw nut around the lead screw axis")
    ctx.allow_overlap(lift_carriage, leadnut_cartridge, reason="leadnut cartridge is seated into the carriage center pocket")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(front_access_cover, base_frame, name="front_cover_contacts_base")
    ctx.expect_contact(rear_access_cover, base_frame, name="rear_cover_contacts_base")
    ctx.expect_overlap(front_access_cover, base_frame, axes="xz", min_overlap=0.074, name="front_cover_spans_base_opening")
    ctx.expect_overlap(rear_access_cover, base_frame, axes="xz", min_overlap=0.074, name="rear_cover_spans_base_opening")

    ctx.expect_contact(yaw_bearing_outer, base_frame, name="yaw_bearing_mounts_to_base")
    ctx.expect_contact(yaw_rotor, yaw_bearing_outer, name="yaw_rotor_supported_by_bearing")
    ctx.expect_origin_distance(yaw_rotor, yaw_bearing_outer, axes="xy", max_dist=0.0005, name="yaw_rotor_concentric_with_bearing")

    ctx.expect_contact(column, yaw_rotor, name="column_mounts_to_rotor")
    ctx.expect_overlap(column, yaw_rotor, axes="xy", min_overlap=0.075, name="column_has_rotor_footprint_support")
    ctx.expect_contact(column_rear_cover, column, name="column_cover_contacts_column")

    ctx.expect_gap(lower_guide_bracket, column, axis="y", max_gap=0.0005, max_penetration=1e-5, name="lower_bracket_seats_on_column_face")
    ctx.expect_gap(upper_guide_bracket, column, axis="y", max_gap=0.0005, max_penetration=1e-5, name="upper_bracket_seats_on_column_face")
    ctx.expect_contact(left_guide_shaft, lower_guide_bracket, name="left_shaft_supported_at_lower_bracket")
    ctx.expect_contact(left_guide_shaft, upper_guide_bracket, name="left_shaft_supported_at_upper_bracket")
    ctx.expect_contact(right_guide_shaft, lower_guide_bracket, name="right_shaft_supported_at_lower_bracket")
    ctx.expect_contact(right_guide_shaft, upper_guide_bracket, name="right_shaft_supported_at_upper_bracket")
    ctx.expect_contact(lead_screw, lower_guide_bracket, name="lead_screw_supported_at_lower_bracket")
    ctx.expect_contact(lead_screw, upper_guide_bracket, name="lead_screw_supported_at_upper_bracket")

    ctx.expect_origin_distance(lower_stop_collar, left_guide_shaft, axes="xy", max_dist=0.0005, name="lower_stop_collar_coaxial_to_left_shaft")
    ctx.expect_origin_distance(upper_stop_collar, right_guide_shaft, axes="xy", max_dist=0.0005, name="upper_stop_collar_coaxial_to_right_shaft")
    ctx.expect_overlap(lower_stop_collar, left_guide_shaft, axes="xz", min_overlap=0.010, name="lower_stop_collar_wraps_left_shaft")
    ctx.expect_overlap(upper_stop_collar, right_guide_shaft, axes="xz", min_overlap=0.010, name="upper_stop_collar_wraps_right_shaft")

    ctx.expect_overlap(lift_carriage, left_guide_shaft, axes="xz", min_overlap=0.016, name="carriage_left_bearing_tracks_left_shaft")
    ctx.expect_overlap(lift_carriage, right_guide_shaft, axes="xz", min_overlap=0.016, name="carriage_right_bearing_tracks_right_shaft")
    ctx.expect_contact(left_linear_bearing_cartridge, lift_carriage, name="left_linear_bearing_cartridge_mounts_to_carriage")
    ctx.expect_contact(right_linear_bearing_cartridge, lift_carriage, name="right_linear_bearing_cartridge_mounts_to_carriage")
    ctx.expect_contact(leadnut_cartridge, lift_carriage, name="leadnut_cartridge_mounts_to_carriage")
    ctx.expect_overlap(left_linear_bearing_cartridge, left_guide_shaft, axes="xz", min_overlap=0.016, name="left_linear_bearing_cartridge_tracks_left_shaft")
    ctx.expect_overlap(right_linear_bearing_cartridge, right_guide_shaft, axes="xz", min_overlap=0.016, name="right_linear_bearing_cartridge_tracks_right_shaft")
    ctx.expect_overlap(leadnut_cartridge, lead_screw, axes="xz", min_overlap=0.016, name="leadnut_cartridge_tracks_lead_screw")
    ctx.expect_origin_distance(lift_carriage, column, axes="x", max_dist=0.0005, name="carriage_centered_on_column")
    ctx.expect_gap(lift_carriage, lower_stop_collar, axis="z", min_gap=0.010, name="carriage_clears_lower_stop_at_rest")
    ctx.expect_gap(upper_stop_collar, lift_carriage, axis="z", min_gap=0.010, name="carriage_clears_upper_stop_at_rest")

    with ctx.pose({yaw_joint: 1.20}):
        ctx.expect_contact(yaw_rotor, yaw_bearing_outer, name="yaw_support_persists_at_positive_pose")
        ctx.expect_gap(column, base_frame, axis="z", min_gap=0.080, name="column_stays_above_base_during_yaw")

    with ctx.pose({yaw_joint: -1.40}):
        ctx.expect_contact(yaw_rotor, yaw_bearing_outer, name="yaw_support_persists_at_negative_pose")

    with ctx.pose({lift_joint: CARRIAGE_TRAVEL}):
        ctx.expect_overlap(lift_carriage, left_guide_shaft, axes="xz", min_overlap=0.016, name="carriage_left_bearing_tracks_left_shaft_at_upper_pose")
        ctx.expect_overlap(lift_carriage, right_guide_shaft, axes="xz", min_overlap=0.016, name="carriage_right_bearing_tracks_right_shaft_at_upper_pose")
        ctx.expect_overlap(left_linear_bearing_cartridge, left_guide_shaft, axes="xz", min_overlap=0.016, name="left_linear_bearing_cartridge_tracks_left_shaft_at_upper_pose")
        ctx.expect_overlap(right_linear_bearing_cartridge, right_guide_shaft, axes="xz", min_overlap=0.016, name="right_linear_bearing_cartridge_tracks_right_shaft_at_upper_pose")
        ctx.expect_overlap(leadnut_cartridge, lead_screw, axes="xz", min_overlap=0.016, name="leadnut_cartridge_tracks_lead_screw_at_upper_pose")
        ctx.expect_gap(upper_stop_collar, lift_carriage, axis="z", min_gap=0.010, name="carriage_clears_upper_stop_at_top_pose")
        ctx.expect_gap(lift_carriage, lower_stop_collar, axis="z", min_gap=0.090, name="carriage_stays_well_above_lower_stop_at_top_pose")

    with ctx.pose({lift_joint: 0.0}):
        ctx.expect_gap(lift_carriage, lower_stop_collar, axis="z", min_gap=0.010, name="carriage_clears_lower_stop_at_bottom_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
