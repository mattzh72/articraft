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
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_X = 0.36
BASE_Y = 0.24
BASE_THK = 0.02
FOOT_X = 0.05
FOOT_Y = 0.036
FOOT_H = 0.012
SADDLE_X = 0.17
SADDLE_Y = 0.18
SADDLE_H = 0.04

COLUMN_D = 0.09
COLUMN_W = 0.12
COLUMN_H = 0.62
COLUMN_WALL = 0.008
COLUMN_BASE_PLATE_T = 0.012
COLUMN_TOP_PLATE_T = 0.012
COLUMN_RAIL_PAD_T = 0.006
COLUMN_SCREW_PAD_T = 0.008

RAIL_D = 0.014
RAIL_W = 0.018
RAIL_H = 0.46
RAIL_Y = 0.033
RAIL_Z = 0.31

SCREW_R = 0.008
SCREW_H = 0.50
SCREW_Z = 0.31

SLIDE_REST_Z = 0.17
SLIDE_TRAVEL = 0.28
TRUCK_X = 0.069
TRUCK_Z = 0.055
CHEEK_Y = 0.047
CHEEK_T = 0.014
PIVOT_X = 0.143
PIVOT_R = 0.0095
PIVOT_SHAFT_L = 0.140

COLUMN_BASE_Z = BASE_THK / 2.0 + SADDLE_H
COLUMN_FRONT_FACE_X = COLUMN_D / 2.0
COLUMN_BACK_FACE_X = -COLUMN_D / 2.0
RAIL_PAD_X = COLUMN_FRONT_FACE_X + COLUMN_RAIL_PAD_T / 2.0
SCREW_PAD_X = COLUMN_FRONT_FACE_X + COLUMN_SCREW_PAD_T / 2.0
RAIL_X = COLUMN_FRONT_FACE_X + COLUMN_RAIL_PAD_T + RAIL_D / 2.0
SCREW_X = COLUMN_FRONT_FACE_X + COLUMN_SCREW_PAD_T + 0.03


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(
        shape,
        filename,
        assets=ASSETS,
        tolerance=0.0008,
        angular_tolerance=0.08,
    )


def _box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x - length / 2.0, y, z))


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, y - length / 2.0, z))


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((x, y, z - length / 2.0))


def _ring_y(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_y(outer_radius, length, center).cut(_cyl_y(inner_radius, length + 0.004, center))


def _base_bracket(sign: float) -> cq.Workplane:
    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.06, 0.0),
                (0.06, 0.0),
                (0.06, 0.018),
                (0.012, 0.11),
                (-0.06, 0.11),
            ]
        )
        .close()
        .extrude(0.014, both=True)
    )
    return gusset.translate((0.0, sign * 0.079, BASE_THK / 2.0))


def _column_shell_shape() -> cq.Workplane:
    outer = _box_shape((COLUMN_D, COLUMN_W, COLUMN_H), (0.0, 0.0, COLUMN_H / 2.0))
    inner = _box_shape(
        (
            COLUMN_D - 2.0 * COLUMN_WALL,
            COLUMN_W - 2.0 * COLUMN_WALL,
            COLUMN_H - COLUMN_BASE_PLATE_T - COLUMN_TOP_PLATE_T,
        ),
        (
            0.0,
            0.0,
            COLUMN_BASE_PLATE_T
            + 0.5 * (COLUMN_H - COLUMN_BASE_PLATE_T - COLUMN_TOP_PLATE_T),
        ),
    )
    rear_service_slot = _box_shape(
        (COLUMN_WALL + 0.004, 0.078, 0.38),
        (COLUMN_BACK_FACE_X + COLUMN_WALL / 2.0, 0.0, 0.35),
    )
    upper_lightening_window = _box_shape((0.032, 0.05, 0.16), (0.0, 0.0, 0.47))
    return outer.cut(inner).cut(rear_service_slot).cut(upper_lightening_window)


def _rail_hardware_shape(rail_y: float) -> cq.Workplane:
    strap = _box_shape((0.004, 0.010, 0.42), (RAIL_X - 0.001, rail_y, RAIL_Z))
    heads = cq.Workplane("XY")
    for z in (0.16, 0.24, 0.32, 0.40, 0.48):
        head = _box_shape((0.004, 0.008, 0.008), (RAIL_X, rail_y, z))
        heads = heads.add(head)
    end_stop_low = _box_shape((0.006, 0.014, 0.014), (RAIL_X - 0.001, rail_y, 0.132))
    end_stop_high = _box_shape((0.006, 0.014, 0.014), (RAIL_X - 0.001, rail_y, 0.588))
    return strap.union(heads).union(end_stop_low).union(end_stop_high)


def _drive_support_block(z_center: float) -> cq.Workplane:
    body = _box_shape((0.034, 0.042, 0.028), (0.070, 0.0, z_center))
    nose = _cyl_x(0.015, 0.024, (0.082, 0.0, z_center))
    clamp = _box_shape((0.012, 0.044, 0.010), (0.060, 0.0, z_center - 0.009))
    return body.union(nose).union(clamp)


def _column_cover_shape() -> cq.Workplane:
    plate = _box_shape((0.006, 0.09, 0.40), (COLUMN_BACK_FACE_X - 0.003, 0.0, 0.35))
    stiffener = _box_shape((0.012, 0.05, 0.28), (COLUMN_BACK_FACE_X - 0.009, 0.0, 0.35))
    flange = _box_shape((0.010, 0.10, 0.04), (COLUMN_BACK_FACE_X - 0.005, 0.0, 0.14))
    heads = cq.Workplane("XY")
    for y in (-0.03, 0.03):
        for z in (0.16, 0.28, 0.42, 0.54):
            head = _box_shape((0.004, 0.008, 0.008), (COLUMN_BACK_FACE_X - 0.006, y, z))
            heads = heads.add(head)
    return plate.union(stiffener).union(flange).union(heads)


def _truck_shape(sign: float, z_center: float) -> cq.Workplane:
    y = sign * RAIL_Y
    back_block = _box_shape((0.018, 0.032, 0.058), (0.079, y, z_center))
    upper_jaw = _box_shape((0.012, 0.007, 0.058), (0.076, y + sign * 0.0125, z_center))
    lower_jaw = _box_shape((0.012, 0.007, 0.058), (0.076, y - sign * 0.0125, z_center))
    top_cap = _box_shape((0.020, 0.020, 0.010), (0.076, y, z_center + 0.020))
    bottom_cap = _box_shape((0.020, 0.020, 0.010), (0.076, y, z_center - 0.020))
    return back_block.union(upper_jaw).union(lower_jaw).union(top_cap).union(bottom_cap)


def _truck_bearing_pad_shape(sign: float, z_center: float) -> cq.Workplane:
    return _box_shape((0.005, 0.012, 0.046), (RAIL_X + RAIL_D / 2.0 + 0.0025, sign * RAIL_Y, z_center))


def _truck_hardware_shape() -> cq.Workplane:
    heads = cq.Workplane("XY")
    for sign in (-1.0, 1.0):
        for z_center in (-TRUCK_Z, TRUCK_Z):
            for y_offset in (-0.009, 0.009):
                for z_offset in (-0.015, 0.015):
                    head = _box_shape(
                        (0.005, 0.007, 0.007),
                        (TRUCK_X + 0.0205, sign * RAIL_Y + y_offset, z_center + z_offset),
                    )
                    heads = heads.add(head)
    return heads


def _carriage_frame_shape() -> cq.Workplane:
    backbone = _box_shape((0.020, 0.100, 0.164), (0.097, 0.0, 0.0))
    window = _box_shape((0.014, 0.072, 0.090), (0.097, 0.0, 0.0))
    screw_corridor = _box_shape((0.040, 0.034, 0.18), (SCREW_X, 0.0, 0.0))
    front_bridge = _box_shape((0.022, 0.070, 0.104), (0.118, 0.0, 0.0))
    upper_cross = _box_shape((0.028, 0.100, 0.018), (0.084, 0.0, 0.058))
    lower_cross = _box_shape((0.028, 0.100, 0.018), (0.084, 0.0, -0.058))
    left_rib = _box_shape((0.018, 0.024, 0.078), (0.127, 0.042, 0.0))
    right_rib = _box_shape((0.018, 0.024, 0.078), (0.127, -0.042, 0.0))
    frame = (
        backbone.cut(window)
        .union(front_bridge)
        .union(upper_cross)
        .union(lower_cross)
        .union(left_rib)
        .union(right_rib)
    )
    return frame.cut(screw_corridor)


def _nut_housing_shape() -> cq.Workplane:
    body = _box_shape((0.040, 0.048, 0.078), (0.093, 0.0, 0.0))
    ears = _box_shape((0.014, 0.082, 0.050), (0.099, 0.0, 0.0))
    screw_clearance = _cyl_z(SCREW_R + 0.0015, 0.13, (SCREW_X, 0.0, 0.0))
    rear_relief = _box_shape((0.020, 0.024, 0.036), (0.071, 0.0, 0.0))
    return body.union(ears).cut(screw_clearance).cut(rear_relief)


def _cheek_shape(sign: float) -> cq.Workplane:
    y = sign * CHEEK_Y
    plate = _box_shape((0.040, CHEEK_T, 0.112), (PIVOT_X - 0.005, y, 0.0))
    window = _box_shape((0.022, CHEEK_T + 0.004, 0.050), (PIVOT_X - 0.018, y, 0.0))
    pivot_bore = _cyl_y(0.0175, CHEEK_T + 0.006, (PIVOT_X, y, 0.0))
    boss = _ring_y(0.024, 0.0175, CHEEK_T, (PIVOT_X, y, 0.0))
    return plate.cut(window).cut(pivot_bore).union(boss)


def _bearing_cartridge_shape(sign: float) -> cq.Workplane:
    return _ring_y(
        0.018,
        PIVOT_R + 0.0007,
        CHEEK_T + 0.002,
        (PIVOT_X, sign * CHEEK_Y, 0.0),
    )


def _carriage_cover_shape() -> cq.Workplane:
    plate = _box_shape((0.006, 0.042, 0.094), (0.070, 0.0, 0.0))
    stiffener = _box_shape((0.010, 0.022, 0.058), (0.063, 0.0, 0.0))
    upper_header = _box_shape((0.008, 0.034, 0.014), (0.066, 0.0, 0.032))
    lower_header = _box_shape((0.008, 0.034, 0.014), (0.066, 0.0, -0.032))
    heads = cq.Workplane("XY")
    for y in (-0.014, 0.014):
        for z in (-0.030, 0.030):
            head = _box_shape((0.004, 0.008, 0.008), (0.068, y, z))
            heads = heads.add(head)
    return plate.union(stiffener).union(upper_header).union(lower_header).union(heads)


def _wrist_body_shape() -> cq.Workplane:
    pivot_block = _box_shape((0.022, 0.026, 0.028), (0.011, 0.0, 0.0))
    neck = _box_shape((0.024, 0.024, 0.044), (0.032, 0.0, 0.0))
    body = _box_shape((0.056, 0.030, 0.072), (0.070, 0.0, 0.0))
    window = _box_shape((0.028, 0.016, 0.030), (0.074, 0.0, -0.004))
    lower_rib = _box_shape((0.022, 0.020, 0.014), (0.076, 0.0, -0.026))
    upper_land = _box_shape((0.018, 0.022, 0.014), (0.052, 0.0, 0.025))
    return pivot_block.union(neck).union(body).cut(window).union(lower_rib).union(upper_land)


def _wrist_flange_shape() -> cq.Workplane:
    plate = _box_shape((0.012, 0.09, 0.10), (0.106, 0.0, 0.0))
    spigot = _cyl_x(0.020, 0.024, (0.094, 0.0, 0.0))
    center_bore = _cyl_x(0.022, 0.018, (0.106, 0.0, 0.0))
    holes = cq.Workplane("XY")
    for y in (-0.026, 0.026):
        for z in (-0.032, 0.032):
            hole = _cyl_x(0.005, 0.018, (0.106, y, z))
            holes = holes.add(hole)
    return plate.union(spigot).cut(center_bore).cut(holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_slide_wrist_study", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.50, 0.52, 0.55, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.65, 0.67, 0.70, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.60, 0.62, 0.65, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.43, 0.45, 0.48, 1.0))
    bearing_bronze = model.material("bearing_bronze", rgba=(0.63, 0.53, 0.34, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(Box((BASE_X, BASE_Y, BASE_THK)), material=dark_steel, name="base_plate")
    base_frame.visual(
        Box((SADDLE_X, SADDLE_Y, SADDLE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THK / 2.0 + SADDLE_H / 2.0)),
        material=painted_steel,
        name="saddle_block",
    )
    base_frame.visual(
        Box((0.11, 0.08, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THK / 2.0 + 0.006)),
        material=painted_steel,
        name="column_mount_plate",
    )
    for name, x, y in (
        ("foot_fl", 0.12, 0.084),
        ("foot_fr", 0.12, -0.084),
        ("foot_rl", -0.12, 0.084),
        ("foot_rr", -0.12, -0.084),
    ):
        base_frame.visual(
            Box((FOOT_X, FOOT_Y, FOOT_H)),
            origin=Origin(xyz=(x, y, -(BASE_THK + FOOT_H) / 2.0)),
            material=dark_steel,
            name=name,
        )
    base_frame.visual(_mesh(_base_bracket(1.0), "base_bracket_left.obj"), material=painted_steel, name="left_bracket")
    base_frame.visual(_mesh(_base_bracket(-1.0), "base_bracket_right.obj"), material=painted_steel, name="right_bracket")

    column = model.part("column")
    column.visual(_mesh(_column_shell_shape(), "column_shell.obj"), material=painted_steel, name="column_shell")
    column.visual(
        Box((COLUMN_RAIL_PAD_T, 0.024, 0.52)),
        origin=Origin(xyz=(RAIL_PAD_X, RAIL_Y, 0.31)),
        material=dark_steel,
        name="left_rail_pad",
    )
    column.visual(
        Box((COLUMN_RAIL_PAD_T, 0.024, 0.52)),
        origin=Origin(xyz=(RAIL_PAD_X, -RAIL_Y, 0.31)),
        material=dark_steel,
        name="right_rail_pad",
    )
    column.visual(
        Box((COLUMN_SCREW_PAD_T, 0.028, 0.56)),
        origin=Origin(xyz=(SCREW_PAD_X, 0.0, 0.31)),
        material=dark_steel,
        name="screw_bridge_pad",
    )

    left_guide_rail = model.part("left_guide_rail")
    left_guide_rail.visual(
        Box((RAIL_D, RAIL_W, RAIL_H)),
        origin=Origin(xyz=(RAIL_X, RAIL_Y, RAIL_Z)),
        material=rail_steel,
        name="rail_body",
    )
    left_guide_rail.visual(
        _mesh(_rail_hardware_shape(RAIL_Y), "left_rail_hardware.obj"),
        material=screw_steel,
        name="rail_hardware",
    )

    right_guide_rail = model.part("right_guide_rail")
    right_guide_rail.visual(
        Box((RAIL_D, RAIL_W, RAIL_H)),
        origin=Origin(xyz=(RAIL_X, -RAIL_Y, RAIL_Z)),
        material=rail_steel,
        name="rail_body",
    )
    right_guide_rail.visual(
        _mesh(_rail_hardware_shape(-RAIL_Y), "right_rail_hardware.obj"),
        material=screw_steel,
        name="rail_hardware",
    )

    drive_screw = model.part("drive_screw")
    drive_screw.visual(
        Cylinder(radius=SCREW_R, length=SCREW_H),
        origin=Origin(xyz=(SCREW_X, 0.0, SCREW_Z)),
        material=screw_steel,
        name="screw_shaft",
    )
    drive_screw.visual(
        _mesh(_drive_support_block(0.07), "drive_support_lower.obj"),
        material=dark_steel,
        name="lower_support",
    )
    drive_screw.visual(
        _mesh(_drive_support_block(0.55), "drive_support_upper.obj"),
        material=dark_steel,
        name="upper_support",
    )
    drive_screw.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(SCREW_X, 0.0, 0.082)),
        material=screw_steel,
        name="lower_locknut",
    )
    drive_screw.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(SCREW_X, 0.0, 0.538)),
        material=screw_steel,
        name="upper_locknut",
    )

    column_access_cover = model.part("column_access_cover")
    column_access_cover.visual(
        _mesh(_column_cover_shape(), "column_access_cover.obj"),
        material=cover_gray,
        name="cover_plate",
    )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.visual(
        _mesh(_truck_shape(1.0, TRUCK_Z), "upper_left_truck.obj"),
        material=painted_steel,
        name="upper_left_truck",
    )
    slide_carriage.visual(
        _mesh(_truck_bearing_pad_shape(1.0, TRUCK_Z), "upper_left_bearing_shoe.obj"),
        material=bearing_bronze,
        name="upper_left_bearing_shoe",
    )
    slide_carriage.visual(
        _mesh(_truck_shape(1.0, -TRUCK_Z), "lower_left_truck.obj"),
        material=painted_steel,
        name="lower_left_truck",
    )
    slide_carriage.visual(
        _mesh(_truck_bearing_pad_shape(1.0, -TRUCK_Z), "lower_left_bearing_shoe.obj"),
        material=bearing_bronze,
        name="lower_left_bearing_shoe",
    )
    slide_carriage.visual(
        _mesh(_truck_shape(-1.0, TRUCK_Z), "upper_right_truck.obj"),
        material=painted_steel,
        name="upper_right_truck",
    )
    slide_carriage.visual(
        _mesh(_truck_bearing_pad_shape(-1.0, TRUCK_Z), "upper_right_bearing_shoe.obj"),
        material=bearing_bronze,
        name="upper_right_bearing_shoe",
    )
    slide_carriage.visual(
        _mesh(_truck_shape(-1.0, -TRUCK_Z), "lower_right_truck.obj"),
        material=painted_steel,
        name="lower_right_truck",
    )
    slide_carriage.visual(
        _mesh(_truck_bearing_pad_shape(-1.0, -TRUCK_Z), "lower_right_bearing_shoe.obj"),
        material=bearing_bronze,
        name="lower_right_bearing_shoe",
    )
    slide_carriage.visual(
        _mesh(_truck_hardware_shape(), "truck_hardware.obj"),
        material=screw_steel,
        name="truck_hardware",
    )
    slide_carriage.visual(
        _mesh(_carriage_frame_shape(), "slide_frame.obj"),
        material=painted_steel,
        name="carriage_frame",
    )
    slide_carriage.visual(
        _mesh(_nut_housing_shape(), "slide_nut_housing.obj"),
        material=dark_steel,
        name="nut_housing",
    )
    slide_carriage.visual(
        _mesh(_cheek_shape(1.0), "left_cheek.obj"),
        material=painted_steel,
        name="left_cheek",
    )
    slide_carriage.visual(
        _mesh(_cheek_shape(-1.0), "right_cheek.obj"),
        material=painted_steel,
        name="right_cheek",
    )
    slide_carriage.visual(
        _mesh(_bearing_cartridge_shape(1.0), "left_bearing.obj"),
        material=bearing_bronze,
        name="left_bearing",
    )
    slide_carriage.visual(
        _mesh(_bearing_cartridge_shape(-1.0), "right_bearing.obj"),
        material=bearing_bronze,
        name="right_bearing",
    )

    carriage_rear_cover = model.part("carriage_rear_cover")
    carriage_rear_cover.visual(
        _mesh(_carriage_cover_shape(), "carriage_rear_cover.obj"),
        material=cover_gray,
        name="rear_cover",
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Box((0.024, 0.024, 0.034)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_block",
    )
    wrist_head.visual(
        Box((0.024, 0.022, 0.042)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=dark_steel,
        name="neck_block",
    )
    wrist_head.visual(
        Box((0.060, 0.028, 0.074)),
        origin=Origin(xyz=(0.077, 0.0, 0.0)),
        material=dark_steel,
        name="wrist_body",
    )
    wrist_head.visual(
        Box((0.022, 0.020, 0.014)),
        origin=Origin(xyz=(0.080, 0.0, -0.026)),
        material=dark_steel,
        name="lower_rib",
    )
    wrist_head.visual(
        Cylinder(radius=PIVOT_R, length=PIVOT_SHAFT_L),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="pivot_shaft",
    )
    wrist_head.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.074, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=screw_steel,
        name="left_shaft_collar",
    )
    wrist_head.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, -0.074, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=screw_steel,
        name="right_shaft_collar",
    )
    wrist_head.visual(
        _mesh(_wrist_flange_shape(), "wrist_front_flange.obj"),
        material=painted_steel,
        name="front_flange",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base_frame,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_BASE_Z)),
    )
    model.articulation(
        "column_to_left_rail",
        ArticulationType.FIXED,
        parent=column,
        child=left_guide_rail,
        origin=Origin(),
    )
    model.articulation(
        "column_to_right_rail",
        ArticulationType.FIXED,
        parent=column,
        child=right_guide_rail,
        origin=Origin(),
    )
    model.articulation(
        "column_to_drive_screw",
        ArticulationType.FIXED,
        parent=column,
        child=drive_screw,
        origin=Origin(),
    )
    model.articulation(
        "column_to_access_cover",
        ArticulationType.FIXED,
        parent=column,
        child=column_access_cover,
        origin=Origin(),
    )
    model.articulation(
        "column_to_slide",
        ArticulationType.PRISMATIC,
        parent=column,
        child=slide_carriage,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1400.0, velocity=0.35, lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "slide_to_cover",
        ArticulationType.FIXED,
        parent=slide_carriage,
        child=carriage_rear_cover,
        origin=Origin(),
    )
    model.articulation(
        "slide_to_wrist",
        ArticulationType.REVOLUTE,
        parent=slide_carriage,
        child=wrist_head,
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=480.0, velocity=1.6, lower=0.0, upper=1.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    column = object_model.get_part("column")
    left_guide_rail = object_model.get_part("left_guide_rail")
    right_guide_rail = object_model.get_part("right_guide_rail")
    drive_screw = object_model.get_part("drive_screw")
    column_access_cover = object_model.get_part("column_access_cover")
    slide_carriage = object_model.get_part("slide_carriage")
    carriage_rear_cover = object_model.get_part("carriage_rear_cover")
    wrist_head = object_model.get_part("wrist_head")

    slide_joint = object_model.get_articulation("column_to_slide")
    wrist_joint = object_model.get_articulation("slide_to_wrist")

    saddle_block = base_frame.get_visual("saddle_block")
    column_shell = column.get_visual("column_shell")
    left_rail_pad = column.get_visual("left_rail_pad")
    right_rail_pad = column.get_visual("right_rail_pad")
    screw_bridge_pad = column.get_visual("screw_bridge_pad")
    left_rail_body = left_guide_rail.get_visual("rail_body")
    right_rail_body = right_guide_rail.get_visual("rail_body")
    screw_shaft = drive_screw.get_visual("screw_shaft")
    lower_support = drive_screw.get_visual("lower_support")
    upper_support = drive_screw.get_visual("upper_support")
    cover_plate = column_access_cover.get_visual("cover_plate")
    upper_left_truck = slide_carriage.get_visual("upper_left_truck")
    lower_left_truck = slide_carriage.get_visual("lower_left_truck")
    upper_right_truck = slide_carriage.get_visual("upper_right_truck")
    lower_right_truck = slide_carriage.get_visual("lower_right_truck")
    upper_left_bearing_shoe = slide_carriage.get_visual("upper_left_bearing_shoe")
    lower_left_bearing_shoe = slide_carriage.get_visual("lower_left_bearing_shoe")
    upper_right_bearing_shoe = slide_carriage.get_visual("upper_right_bearing_shoe")
    lower_right_bearing_shoe = slide_carriage.get_visual("lower_right_bearing_shoe")
    carriage_frame = slide_carriage.get_visual("carriage_frame")
    nut_housing = slide_carriage.get_visual("nut_housing")
    left_cheek = slide_carriage.get_visual("left_cheek")
    right_cheek = slide_carriage.get_visual("right_cheek")
    left_bearing = slide_carriage.get_visual("left_bearing")
    right_bearing = slide_carriage.get_visual("right_bearing")
    rear_cover = carriage_rear_cover.get_visual("rear_cover")
    wrist_body = wrist_head.get_visual("wrist_body")
    pivot_shaft = wrist_head.get_visual("pivot_shaft")
    front_flange = wrist_head.get_visual("front_flange")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        slide_carriage,
        wrist_head,
        reason="Wrist pivot shaft intentionally passes through the carriage cheek bores as a captured hinge axle.",
        elem_a=left_bearing,
        elem_b=pivot_shaft,
    )
    ctx.allow_overlap(
        slide_carriage,
        wrist_head,
        reason="Wrist pivot shaft intentionally passes through the carriage cheek bores as a captured hinge axle.",
        elem_a=right_bearing,
        elem_b=pivot_shaft,
    )
    ctx.allow_overlap(
        slide_carriage,
        wrist_head,
        reason="Cheek-side hinge bores intentionally envelop the wrist pivot shaft with a captured axle fit.",
        elem_a=left_cheek,
        elem_b=pivot_shaft,
    )
    ctx.allow_overlap(
        slide_carriage,
        wrist_head,
        reason="Cheek-side hinge bores intentionally envelop the wrist pivot shaft with a captured axle fit.",
        elem_a=right_cheek,
        elem_b=pivot_shaft,
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        column,
        base_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem=saddle_block,
        positive_elem=column_shell,
        name="column_seats_on_saddle",
    )
    ctx.expect_overlap(column, base_frame, axes="xy", min_overlap=0.09, elem_b=saddle_block, name="column_over_saddle")

    ctx.expect_gap(
        left_guide_rail,
        column,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=left_rail_body,
        negative_elem=left_rail_pad,
        name="left_rail_seats_on_pad",
    )
    ctx.expect_gap(
        right_guide_rail,
        column,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=right_rail_body,
        negative_elem=right_rail_pad,
        name="right_rail_seats_on_pad",
    )
    ctx.expect_overlap(
        left_guide_rail,
        column,
        axes="yz",
        min_overlap=0.015,
        elem_a=left_rail_body,
        elem_b=left_rail_pad,
        name="left_rail_pad_footprint",
    )
    ctx.expect_overlap(
        right_guide_rail,
        column,
        axes="yz",
        min_overlap=0.015,
        elem_a=right_rail_body,
        elem_b=right_rail_pad,
        name="right_rail_pad_footprint",
    )

    ctx.expect_gap(
        drive_screw,
        column,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=lower_support,
        negative_elem=screw_bridge_pad,
        name="lower_screw_support_mounts_to_bridge",
    )
    ctx.expect_gap(
        drive_screw,
        column,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=upper_support,
        negative_elem=screw_bridge_pad,
        name="upper_screw_support_mounts_to_bridge",
    )

    ctx.expect_gap(
        column,
        column_access_cover,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=column_shell,
        negative_elem=cover_plate,
        name="column_cover_seats_flush",
    )
    ctx.expect_overlap(
        column_access_cover,
        column,
        axes="yz",
        min_overlap=0.08,
        elem_a=cover_plate,
        elem_b=column_shell,
        name="column_cover_spans_service_opening",
    )

    ctx.expect_contact(
        slide_carriage,
        left_guide_rail,
        elem_a=upper_left_bearing_shoe,
        elem_b=left_rail_body,
        name="upper_left_bearing_shoe_contacts_left_rail",
    )
    ctx.expect_contact(
        slide_carriage,
        left_guide_rail,
        elem_a=lower_left_bearing_shoe,
        elem_b=left_rail_body,
        name="lower_left_bearing_shoe_contacts_left_rail",
    )
    ctx.expect_contact(
        slide_carriage,
        right_guide_rail,
        elem_a=upper_right_bearing_shoe,
        elem_b=right_rail_body,
        name="upper_right_bearing_shoe_contacts_right_rail",
    )
    ctx.expect_contact(
        slide_carriage,
        right_guide_rail,
        elem_a=lower_right_bearing_shoe,
        elem_b=right_rail_body,
        name="lower_right_bearing_shoe_contacts_right_rail",
    )
    ctx.expect_gap(
        slide_carriage,
        left_guide_rail,
        axis="x",
        min_gap=0.0005,
        max_gap=0.002,
        positive_elem=upper_left_truck,
        negative_elem=left_rail_body,
        name="upper_left_truck_body_clears_left_rail",
    )
    ctx.expect_gap(
        slide_carriage,
        left_guide_rail,
        axis="x",
        min_gap=0.0005,
        max_gap=0.002,
        positive_elem=lower_left_truck,
        negative_elem=left_rail_body,
        name="lower_left_truck_body_clears_left_rail",
    )
    ctx.expect_gap(
        slide_carriage,
        right_guide_rail,
        axis="x",
        min_gap=0.0005,
        max_gap=0.002,
        positive_elem=upper_right_truck,
        negative_elem=right_rail_body,
        name="upper_right_truck_body_clears_right_rail",
    )
    ctx.expect_gap(
        slide_carriage,
        right_guide_rail,
        axis="x",
        min_gap=0.0005,
        max_gap=0.002,
        positive_elem=lower_right_truck,
        negative_elem=right_rail_body,
        name="lower_right_truck_body_clears_right_rail",
    )
    ctx.expect_gap(
        slide_carriage,
        column,
        axis="x",
        min_gap=0.01,
        positive_elem=carriage_frame,
        negative_elem=column_shell,
        name="carriage_frame_clears_column_shell",
    )
    ctx.expect_overlap(
        slide_carriage,
        drive_screw,
        axes="xy",
        min_overlap=0.015,
        elem_a=nut_housing,
        elem_b=screw_shaft,
        name="nut_housing_centered_on_screw_axis",
    )
    ctx.expect_gap(
        slide_carriage,
        carriage_rear_cover,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=nut_housing,
        negative_elem=rear_cover,
        name="carriage_cover_seats_on_nut_housing",
    )
    ctx.expect_contact(
        wrist_head,
        slide_carriage,
        elem_a=pivot_shaft,
        elem_b=left_bearing,
        contact_tol=0.001,
        name="left_wrist_bearing_supports_pivot",
    )
    ctx.expect_contact(
        wrist_head,
        slide_carriage,
        elem_a=pivot_shaft,
        elem_b=right_bearing,
        contact_tol=0.001,
        name="right_wrist_bearing_supports_pivot",
    )
    ctx.expect_gap(
        wrist_head,
        slide_carriage,
        axis="x",
        min_gap=0.006,
        positive_elem=wrist_body,
        negative_elem=carriage_frame,
        name="wrist_body_clears_carriage_frame",
    )

    ctx.expect_origin_distance(slide_carriage, column, axes="xy", max_dist=0.001, name="slide_axis_remains_on_column_centerline")
    ctx.expect_origin_gap(
        slide_carriage,
        column,
        axis="z",
        min_gap=SLIDE_REST_Z - 0.001,
        max_gap=SLIDE_REST_Z + 0.001,
        name="slide_rest_height",
    )
    ctx.expect_origin_gap(
        wrist_head,
        slide_carriage,
        axis="x",
        min_gap=PIVOT_X - 0.001,
        max_gap=PIVOT_X + 0.001,
        name="wrist_pivot_projects_forward_of_slide_axis",
    )

    rest_slide_pos = ctx.part_world_position(slide_carriage)
    raised_slide_pos = None
    with ctx.pose({slide_joint: 0.22}):
        raised_slide_pos = ctx.part_world_position(slide_carriage)
        ctx.expect_origin_distance(slide_carriage, column, axes="xy", max_dist=0.001, name="raised_slide_stays_on_axis")
        ctx.expect_origin_gap(
            slide_carriage,
            column,
            axis="z",
            min_gap=SLIDE_REST_Z + 0.219,
            max_gap=SLIDE_REST_Z + 0.221,
            name="slide_raised_height",
        )
        ctx.expect_contact(
            slide_carriage,
            left_guide_rail,
            elem_a=upper_left_bearing_shoe,
            elem_b=left_rail_body,
            name="upper_left_bearing_shoe_contacts_left_rail_raised",
        )
        ctx.expect_contact(
            slide_carriage,
            left_guide_rail,
            elem_a=lower_left_bearing_shoe,
            elem_b=left_rail_body,
            name="lower_left_bearing_shoe_contacts_left_rail_raised",
        )
        ctx.expect_contact(
            slide_carriage,
            right_guide_rail,
            elem_a=upper_right_bearing_shoe,
            elem_b=right_rail_body,
            name="upper_right_bearing_shoe_contacts_right_rail_raised",
        )
        ctx.expect_contact(
            slide_carriage,
            right_guide_rail,
            elem_a=lower_right_bearing_shoe,
            elem_b=right_rail_body,
            name="lower_right_bearing_shoe_contacts_right_rail_raised",
        )
        ctx.expect_gap(
            slide_carriage,
            left_guide_rail,
            axis="x",
            min_gap=0.0005,
            max_gap=0.002,
            positive_elem=upper_left_truck,
            negative_elem=left_rail_body,
            name="upper_left_truck_body_clears_left_rail_raised",
        )
        ctx.expect_gap(
            slide_carriage,
            left_guide_rail,
            axis="x",
            min_gap=0.0005,
            max_gap=0.002,
            positive_elem=lower_left_truck,
            negative_elem=left_rail_body,
            name="lower_left_truck_body_clears_left_rail_raised",
        )
        ctx.expect_gap(
            slide_carriage,
            right_guide_rail,
            axis="x",
            min_gap=0.0005,
            max_gap=0.002,
            positive_elem=upper_right_truck,
            negative_elem=right_rail_body,
            name="upper_right_truck_body_clears_right_rail_raised",
        )
        ctx.expect_gap(
            slide_carriage,
            right_guide_rail,
            axis="x",
            min_gap=0.0005,
            max_gap=0.002,
            positive_elem=lower_right_truck,
            negative_elem=right_rail_body,
            name="lower_right_truck_body_clears_right_rail_raised",
        )
        ctx.expect_gap(
            wrist_head,
            slide_carriage,
            axis="x",
            min_gap=0.006,
            positive_elem=wrist_body,
            negative_elem=carriage_frame,
            name="wrist_body_clears_carriage_frame_raised",
        )

    ctx.check(
        "slide_prismatic_travel_matches_command",
        rest_slide_pos is not None
        and raised_slide_pos is not None
        and abs((raised_slide_pos[2] - rest_slide_pos[2]) - 0.22) < 0.002
        and abs(raised_slide_pos[0] - rest_slide_pos[0]) < 1e-6
        and abs(raised_slide_pos[1] - rest_slide_pos[1]) < 1e-6,
        details=f"rest={rest_slide_pos}, raised={raised_slide_pos}",
    )

    flange_rest = ctx.part_element_world_aabb(wrist_head, elem=front_flange)
    flange_pitched = None
    with ctx.pose({slide_joint: 0.12, wrist_joint: 0.85}):
        flange_pitched = ctx.part_element_world_aabb(wrist_head, elem=front_flange)
        ctx.expect_contact(
            wrist_head,
            slide_carriage,
            elem_a=pivot_shaft,
            elem_b=left_bearing,
            contact_tol=0.001,
            name="left_wrist_bearing_contact_pitched",
        )
        ctx.expect_contact(
            wrist_head,
            slide_carriage,
            elem_a=pivot_shaft,
            elem_b=right_bearing,
            contact_tol=0.001,
            name="right_wrist_bearing_contact_pitched",
        )
        ctx.expect_gap(
            wrist_head,
            slide_carriage,
            axis="x",
            min_gap=0.004,
            positive_elem=wrist_body,
            negative_elem=carriage_frame,
            name="wrist_body_clears_carriage_frame_pitched",
        )

    flange_rest_z = None
    flange_pitched_z = None
    if flange_rest is not None:
        flange_rest_z = 0.5 * (flange_rest[0][2] + flange_rest[1][2])
    if flange_pitched is not None:
        flange_pitched_z = 0.5 * (flange_pitched[0][2] + flange_pitched[1][2])
    ctx.check(
        "wrist_pitch_moves_front_flange",
        flange_rest_z is not None
        and flange_pitched_z is not None
        and abs(flange_pitched_z - flange_rest_z) > 0.045,
        details=f"rest_z={flange_rest_z}, pitched_z={flange_pitched_z}",
    )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
