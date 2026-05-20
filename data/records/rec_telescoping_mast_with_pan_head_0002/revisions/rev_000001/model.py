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

BASE_BEAM_LEN = 0.64
BASE_BEAM_W = 0.08
BASE_BEAM_H = 0.07
BASE_BEAM_Y = 0.18
BASE_CROSS_LEN = 0.44
BASE_CROSS_W = 0.08
PEDESTAL_W = 0.28
PEDESTAL_D = 0.24
PEDESTAL_H = 0.15
PEDESTAL_Z0 = BASE_BEAM_H
PEDESTAL_Z1 = PEDESTAL_Z0 + PEDESTAL_H

OUTER_FLANGE = 0.22
OUTER_FLANGE_T = 0.018
OUTER_SIZE = 0.16
OUTER_WALL = 0.010
OUTER_LEN = 0.92
OUTER_TOTAL_H = OUTER_FLANGE_T + OUTER_LEN

MID_SIZE = 0.126
MID_WALL = 0.009
MID_LEN = 0.84
MID_REST_BOTTOM = 0.36

INNER_SIZE = 0.096
INNER_WALL = 0.008
INNER_LEN = 0.74
HEAD_MOUNT_T = 0.014
INNER_TOP = INNER_LEN + HEAD_MOUNT_T
INNER_REST_BOTTOM = 0.28

OUTER_CLAMP_H = 0.082
OUTER_CLAMP_OUTER = 0.186
OUTER_CLAMP_INNER = 0.136

MID_CLAMP_H = 0.076
MID_CLAMP_OUTER = 0.150
MID_CLAMP_INNER = 0.104

BASE_COVER_T = 0.006
MID_COVER_T = 0.005
MID_COVER_Z = OUTER_FLANGE_T + 0.40

SUPPORT_BASE_DROP = 0.080
HEAD_MOUNT_W = 0.150
HEAD_MOUNT_D = 0.220


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS)


def _fuse_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _box_tube(outer: float, wall: float, length: float) -> cq.Workplane:
    outer_box = cq.Workplane("XY", origin=(0.0, 0.0, length / 2.0)).box(outer, outer, length)
    inner_box = cq.Workplane("XY", origin=(0.0, 0.0, length / 2.0)).box(
        outer - (2.0 * wall),
        outer - (2.0 * wall),
        length + 0.004,
    )
    return outer_box.cut(inner_box)


def _annulus_z(outer_r: float, inner_r: float, length: float, center_z: float) -> cq.Workplane:
    outer = cq.Workplane("XY", origin=(0.0, 0.0, center_z - (length / 2.0))).circle(outer_r).extrude(length)
    inner = cq.Workplane("XY", origin=(0.0, 0.0, center_z - ((length + 0.004) / 2.0))).circle(inner_r).extrude(
        length + 0.004
    )
    return outer.cut(inner)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("YZ", origin=(cx - (length / 2.0), cy, cz)).circle(radius).extrude(length)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XZ", origin=(cx, cy - (length / 2.0), cz)).circle(radius).extrude(length)


def _bolt_heads_on_plate(
    width: float,
    height: float,
    y_offset: float,
    radius: float = 0.005,
    length: float = 0.010,
) -> cq.Workplane:
    bolt_x = width / 2.0
    bolt_z = height / 2.0
    points = [
        (-bolt_x, y_offset, -bolt_z),
        (bolt_x, y_offset, -bolt_z),
        (-bolt_x, y_offset, bolt_z),
        (bolt_x, y_offset, bolt_z),
        (-bolt_x * 0.55, y_offset, 0.0),
        (bolt_x * 0.55, y_offset, 0.0),
    ]
    return _fuse_all([_cyl_y(radius, length, point) for point in points])


def _make_cover(width: float, height: float, thickness: float, bolt_side: float) -> cq.Workplane:
    sign = 1.0 if bolt_side >= 0.0 else -1.0
    shell = cq.Workplane("XY", origin=(0.0, sign * (thickness / 2.0), 0.0)).box(width, thickness, height)
    pocket = cq.Workplane("XY", origin=(0.0, sign * (thickness / 2.0), 0.0)).box(
        width - 0.022,
        thickness + 0.002,
        height - 0.022,
    )
    frame = shell.cut(pocket)

    rib_y = sign * (thickness + 0.003)
    ribs = [
        cq.Workplane("XY", origin=(0.0, rib_y, 0.0)).box(0.020, 0.006, height - 0.048),
        cq.Workplane("XY", origin=(0.0, rib_y, (height / 2.0) - 0.016)).box(width - 0.038, 0.006, 0.012),
        cq.Workplane("XY", origin=(0.0, rib_y, -(height / 2.0) + 0.016)).box(width - 0.038, 0.006, 0.012),
    ]

    cover_lip = cq.Workplane("XY", origin=(0.0, sign * (thickness + 0.0015), -(height / 2.0) + 0.014)).box(
        width * 0.28,
        0.003,
        0.010,
    )
    bolts = _bolt_heads_on_plate(width - 0.028, height - 0.028, sign * (thickness + 0.005))
    return _fuse_all([frame, cover_lip, bolts] + ribs)


def _make_mount_frame(width: float, height: float, depth: float, y_center: float, z_center: float) -> cq.Workplane:
    ring = cq.Workplane("XY", origin=(0.0, y_center, z_center)).box(width, depth, height)
    opening = cq.Workplane("XY", origin=(0.0, y_center, z_center)).box(width - 0.028, depth + 0.002, height - 0.028)
    return ring.cut(opening)


def _make_side_access_cover(width: float, height: float, thickness: float) -> cq.Workplane:
    face = cq.Workplane("XY", origin=(thickness / 2.0, 0.0, 0.0)).box(thickness, width, height)
    pocket = cq.Workplane("XY", origin=(thickness / 2.0, 0.0, 0.0)).box(thickness + 0.002, width - 0.020, height - 0.020)
    frame = face.cut(pocket)
    rib_x = thickness + 0.004
    ribs = [
        cq.Workplane("XY", origin=(rib_x, 0.0, 0.0)).box(0.008, width * 0.36, 0.016),
        cq.Workplane("XY", origin=(rib_x, 0.0, (height / 2.0) - 0.014)).box(0.008, width - 0.022, 0.012),
        cq.Workplane("XY", origin=(rib_x, 0.0, -(height / 2.0) + 0.014)).box(0.008, width - 0.022, 0.012),
    ]
    latch = cq.Workplane("XY", origin=(thickness + 0.005, 0.0, 0.0)).box(0.010, 0.022, 0.028)
    bolts = _fuse_all(
        [
            _cyl_x(0.0035, 0.006, (thickness + 0.001, sy, sz))
            for sy in (-0.046, 0.046)
            for sz in (-0.030, 0.030)
        ]
    )
    return _fuse_all([frame, latch, bolts] + ribs)


def _make_flush_side_cover(width: float, height: float, depth: float) -> cq.Workplane:
    plate = cq.Workplane("XY", origin=(depth / 2.0, 0.0, 0.0)).box(depth, width, height)
    recess = cq.Workplane("XY", origin=((depth / 2.0) - 0.001, 0.0, 0.0)).box(depth, width - 0.018, height - 0.018)
    frame = plate.cut(recess)
    ribs = [
        cq.Workplane("XY", origin=(depth * 0.65, 0.0, 0.0)).box(depth * 0.30, width * 0.30, 0.010),
        cq.Workplane("XY", origin=(depth * 0.65, 0.0, (height / 2.0) - 0.012)).box(depth * 0.30, width - 0.020, 0.008),
        cq.Workplane("XY", origin=(depth * 0.65, 0.0, -(height / 2.0) + 0.012)).box(depth * 0.30, width - 0.020, 0.008),
    ]
    fasteners = _fuse_all(
        [
            _cyl_x(0.0025, depth * 0.5, (depth * 0.72, sy, sz))
            for sy in (-width * 0.34, width * 0.34)
            for sz in (-height * 0.28, height * 0.28)
        ]
    )
    return _fuse_all([frame, fasteners] + ribs)


def _make_flush_rear_cover(width: float, height: float, depth: float) -> cq.Workplane:
    plate = cq.Workplane("XY", origin=(0.0, -(depth / 2.0), 0.0)).box(width, depth, height)
    pocket = cq.Workplane("XY", origin=(0.0, -(depth / 2.0) + 0.001, 0.0)).box(width - 0.018, depth, height - 0.018)
    frame = plate.cut(pocket)
    ribs = [
        cq.Workplane("XY", origin=(0.0, -depth * 0.65, 0.0)).box(width * 0.22, depth * 0.30, 0.010),
        cq.Workplane("XY", origin=(0.0, -depth * 0.65, (height / 2.0) - 0.012)).box(width - 0.020, depth * 0.30, 0.008),
        cq.Workplane("XY", origin=(0.0, -depth * 0.65, -(height / 2.0) + 0.012)).box(width - 0.020, depth * 0.30, 0.008),
    ]
    fasteners = _fuse_all(
        [
            _cyl_y(0.0025, depth * 0.5, (sx, -depth * 0.72, sz))
            for sx in (-width * 0.28, width * 0.28)
            for sz in (-height * 0.35, height * 0.35)
        ]
    )
    return _fuse_all([frame, fasteners] + ribs)


def _make_outward_cover(width: float, height: float, depth: float) -> cq.Workplane:
    shell = cq.Workplane("XY", origin=(0.0, depth / 2.0, 0.0)).box(width, depth, height)
    pocket = cq.Workplane("XY", origin=(0.0, depth / 2.0, 0.0)).box(width - 0.020, depth + 0.002, height - 0.020)
    frame = shell.cut(pocket)
    ribs = [
        cq.Workplane("XY", origin=(0.0, depth * 0.72, 0.0)).box(width * 0.22, depth * 0.30, 0.010),
        cq.Workplane("XY", origin=(0.0, depth * 0.72, (height / 2.0) - 0.012)).box(width - 0.022, depth * 0.28, 0.008),
        cq.Workplane("XY", origin=(0.0, depth * 0.72, -(height / 2.0) + 0.012)).box(width - 0.022, depth * 0.28, 0.008),
    ]
    fasteners = _fuse_all(
        [
            _cyl_y(0.0025, depth * 0.42, (sx, depth * 0.72, sz))
            for sx in (-width * 0.30, width * 0.30)
            for sz in (-height * 0.34, height * 0.34)
        ]
    )
    return _fuse_all([frame, fasteners] + ribs)


def _make_base_frame() -> cq.Workplane:
    long_beams = [
        cq.Workplane("XY", origin=(0.0, sy, BASE_BEAM_H / 2.0)).box(BASE_BEAM_LEN, BASE_BEAM_W, BASE_BEAM_H)
        for sy in (-BASE_BEAM_Y, BASE_BEAM_Y)
    ]
    cross_beams = [
        cq.Workplane("XY", origin=(sx, 0.0, BASE_BEAM_H / 2.0)).box(BASE_CROSS_W, BASE_CROSS_LEN, BASE_BEAM_H)
        for sx in (-0.17, 0.17)
    ]
    pedestal = cq.Workplane("XY", origin=(0.0, 0.0, PEDESTAL_Z0 + (PEDESTAL_H / 2.0))).box(
        PEDESTAL_W,
        PEDESTAL_D,
        PEDESTAL_H,
    )
    pedestal = pedestal.cut(
        cq.Workplane("XY", origin=(0.0, (PEDESTAL_D / 2.0) - 0.046, PEDESTAL_Z0 + 0.075)).box(0.165, 0.116, 0.092)
    )
    pedestal = pedestal.cut(
        cq.Workplane("XY", origin=((PEDESTAL_W / 2.0) - 0.015, 0.0, PEDESTAL_Z0 + 0.075)).box(0.040, 0.112, 0.092)
    )
    braces = []
    for sx, angle in ((-0.142, -15.0), (0.142, 15.0)):
        braces.append(
            cq.Workplane("XY", origin=(sx, 0.0, 0.112))
            .box(0.014, 0.150, 0.120)
            .rotate((sx, 0.0, 0.112), (sx, 1.0, 0.0), angle)
        )
    for sy, angle in ((-0.132, 15.0), (0.132, -15.0)):
        braces.append(
            cq.Workplane("XY", origin=(0.0, sy, 0.112))
            .box(0.150, 0.014, 0.120)
            .rotate((0.0, sy, 0.112), (1.0, sy, 0.0), angle)
        )

    feet = []
    for sx in (-0.27, 0.27):
        for sy in (-BASE_BEAM_Y, BASE_BEAM_Y):
            feet.append(cq.Workplane("XY", origin=(sx, sy, 0.029)).circle(0.013).extrude(0.058))
            feet.append(cq.Workplane("XY", origin=(sx, sy, 0.006)).circle(0.028).extrude(0.012))

    return _fuse_all(long_beams + cross_beams + [pedestal] + braces + feet)


def _make_base_cover() -> cq.Workplane:
    plate = cq.Workplane("XY", origin=(BASE_COVER_T / 2.0, 0.0, 0.0)).box(BASE_COVER_T, 0.186, 0.106)
    stiffener = cq.Workplane("XY", origin=(BASE_COVER_T + 0.006, 0.0, 0.0)).box(0.012, 0.080, 0.060)
    bolts = (
        cq.Workplane("YZ", origin=(BASE_COVER_T * 0.55, 0.0, 0.0))
        .pushPoints([(-0.052, -0.034), (0.052, -0.034), (-0.052, 0.034), (0.052, 0.034)])
        .circle(0.004)
        .extrude(0.002)
    )
    return _fuse_all([plate, stiffener, bolts])


def _make_mid_cover() -> cq.Workplane:
    plate = cq.Workplane("XY", origin=(0.0, MID_COVER_T / 2.0, 0.0)).box(0.088, MID_COVER_T, 0.206)
    center_rib = cq.Workplane("XY", origin=(0.0, MID_COVER_T + 0.004, 0.0)).box(0.022, 0.008, 0.090)
    screws = (
        cq.Workplane("XZ", origin=(0.0, MID_COVER_T * 0.55, 0.0))
        .pushPoints([(-0.024, -0.072), (0.024, -0.072), (-0.024, 0.072), (0.024, 0.072)])
        .circle(0.003)
        .extrude(0.0015)
    )
    return _fuse_all([plate, center_rib, screws])


def _make_outer_mast_tube() -> cq.Workplane:
    tube = _box_tube(OUTER_SIZE, OUTER_WALL, OUTER_LEN).translate((0.0, 0.0, OUTER_FLANGE_T))
    slot_front = cq.Workplane(
        "XY",
        origin=(0.0, (OUTER_SIZE / 2.0) - 0.004, OUTER_FLANGE_T + 0.56),
    ).box(0.060, 0.028, 0.280)
    slot_rear = cq.Workplane(
        "XY",
        origin=(0.0, -(OUTER_SIZE / 2.0) + 0.004, OUTER_FLANGE_T + 0.40),
    ).box(0.050, 0.028, 0.170)
    return tube.cut(slot_front).cut(slot_rear)


def _make_clamp(collar_outer: float, collar_inner: float, height: float) -> tuple[cq.Workplane, cq.Workplane]:
    ring = _box_tube(collar_outer, (collar_outer - collar_inner) / 2.0, height).translate((0.0, 0.0, -(height / 2.0)))
    split = cq.Workplane("XY", origin=((collar_outer / 2.0) - 0.004, 0.0, 0.0)).box(0.014, 0.060, height + 0.004)
    ring = ring.cut(split)

    ears = []
    bolts = []
    for sz in (-0.022, 0.022):
        ears.append(cq.Workplane("XY", origin=((collar_outer / 2.0) + 0.014, 0.0, sz)).box(0.028, 0.044, 0.022))
        bolts.append(_cyl_x(0.0055, 0.050, ((collar_outer / 2.0) + 0.014, 0.0, sz)))

    lugs = []
    for sy in (-1.0, 1.0):
        lug = cq.Workplane("XY", origin=(0.0, sy * ((collar_outer / 2.0) + 0.006), 0.0)).box(0.040, 0.012, 0.050)
        lug_hole = _cyl_y(0.008, 0.020, (0.0, sy * ((collar_outer / 2.0) + 0.006), 0.0))
        lugs.append(lug.cut(lug_hole))

    return _fuse_all([ring] + ears + lugs), _fuse_all(bolts)


def _make_mid_tube() -> cq.Workplane:
    tube = _box_tube(MID_SIZE, MID_WALL, MID_LEN)
    rear_window = cq.Workplane("XY", origin=(0.0, -(MID_SIZE / 2.0) + 0.004, MID_COVER_Z)).box(0.072, 0.026, 0.190)
    front_slot = cq.Workplane("XY", origin=(0.0, (MID_SIZE / 2.0) - 0.004, 0.56)).box(0.040, 0.022, 0.230)
    return tube.cut(rear_window).cut(front_slot)


def _make_stage_wear_pads(stage_size: float, pad_depth: float, pad_width: float, pad_height: float) -> cq.Workplane:
    pads = []
    for zc in (0.14, 0.50):
        pads.extend(
            [
                cq.Workplane("XY", origin=((stage_size / 2.0) + (pad_depth / 2.0), 0.0, zc)).box(
                    pad_depth,
                    pad_width,
                    pad_height,
                ),
                cq.Workplane("XY", origin=(-(stage_size / 2.0) - (pad_depth / 2.0), 0.0, zc)).box(
                    pad_depth,
                    pad_width,
                    pad_height,
                ),
                cq.Workplane("XY", origin=(0.0, (stage_size / 2.0) + (pad_depth / 2.0), zc)).box(
                    pad_width,
                    pad_depth,
                    pad_height,
                ),
                cq.Workplane("XY", origin=(0.0, -(stage_size / 2.0) - (pad_depth / 2.0), zc)).box(
                    pad_width,
                    pad_depth,
                    pad_height,
                ),
            ]
        )
    return _fuse_all(pads)


def _make_head_support() -> tuple[cq.Workplane, cq.Workplane]:
    base_plate = cq.Workplane("XY", origin=(0.0, 0.0, -0.073)).box(0.14, 0.14, HEAD_MOUNT_T)
    pedestal = cq.Workplane("XY", origin=(0.0, 0.0, -0.043)).box(0.100, 0.100, 0.046)
    bearing_sleeve = _annulus_z(0.028, 0.0185, 0.028, -0.006)
    stanchions = [
        cq.Workplane("XY", origin=(sx, 0.0, -0.020)).box(0.012, 0.054, 0.044) for sx in (-0.056, 0.056)
    ]
    bridge_bars = [
        cq.Workplane("XY", origin=(0.0, sy, -0.002)).box(0.112, 0.010, 0.012) for sy in (-0.022, 0.022)
    ]
    cheek_braces = [
        cq.Workplane("XY", origin=(sx, 0.0, -0.028))
        .box(0.010, 0.068, 0.038)
        .rotate((sx, 0.0, -0.028), (sx, 1.0, -0.028), ang)
        for sx, ang in ((-0.038, -18.0), (0.038, 18.0))
    ]

    lugs = []
    for sy in (-1.0, 1.0):
        lug = cq.Workplane("XY", origin=(0.0, sy * 0.088, -0.056)).box(0.012, 0.036, 0.040)
        lug_hole = _cyl_x(0.007, 0.018, (0.0, sy * 0.088, -0.056))
        lugs.append(lug.cut(lug_hole))

    support_base = _fuse_all([base_plate, pedestal, bearing_sleeve] + stanchions + bridge_bars + cheek_braces + lugs)
    center_bore = cq.Workplane("XY", origin=(0.0, 0.0, -0.095)).circle(0.0185).extrude(0.124)
    support_base = support_base.cut(center_bore)

    upper_cap = _annulus_z(0.026, 0.0185, 0.006, 0.011)
    return support_base, upper_cap


def _make_pan_head() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    thrust_flange = cq.Workplane("XY", origin=(0.0, 0.0, 0.014)).circle(0.024).extrude(0.006)
    spindle = cq.Workplane("XY", origin=(0.0, 0.0, 0.020)).circle(0.017).extrude(0.014)
    riser = cq.Workplane("XY", origin=(0.0, 0.0, 0.034)).circle(0.018).extrude(0.010)
    lock_nut = _annulus_z(0.027, 0.018, 0.006, 0.047)
    deck_pedestal = cq.Workplane("XY", origin=(0.0, 0.0, 0.040)).circle(0.020).extrude(0.008)
    hub = _fuse_all([spindle, thrust_flange, riser, lock_nut, deck_pedestal])

    deck = cq.Workplane("XY", origin=(0.0, 0.0, 0.056)).box(0.180, 0.100, 0.012)
    cheek_plates = []
    for sx in (-0.060, 0.060):
        cheek = cq.Workplane("XY", origin=(sx, 0.0, 0.116)).box(0.010, 0.100, 0.100)
        cheek_plates.append(cheek.cut(cq.Workplane("XY", origin=(sx, 0.0, 0.116)).box(0.012, 0.050, 0.052)))
    tie_top = cq.Workplane("XY", origin=(0.0, 0.0, 0.170)).box(0.120, 0.040, 0.010)
    tie_mid = cq.Workplane("XY", origin=(0.0, 0.0, 0.108)).box(0.082, 0.032, 0.010)
    gussets = [
        cq.Workplane("XY", origin=(sx, 0.0, 0.078))
        .box(0.014, 0.020, 0.050)
        .rotate((sx, 0.0, 0.078), (sx, 1.0, 0.078), ang)
        for sx, ang in ((-0.022, -20.0), (0.022, 20.0))
    ]
    deck_assembly = _fuse_all([deck] + cheek_plates + [tie_top, tie_mid] + gussets)

    hardware = [
        cq.Workplane("XY", origin=(0.0, 0.0, 0.059))
        .pushPoints([(0.030, 0.0), (-0.030, 0.0), (0.0, 0.030), (0.0, -0.030)])
        .circle(0.004)
        .extrude(0.007)
    ]
    return hub, deck_assembly, _fuse_all(hardware)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_mast_pan_head", assets=ASSETS)

    paint_dark = model.material("paint_dark", rgba=(0.21, 0.23, 0.24, 1.0))
    paint_gray = model.material("paint_gray", rgba=(0.42, 0.44, 0.46, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.50, 0.52, 0.54, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.71, 1.0))
    bronze = model.material("bronze", rgba=(0.63, 0.52, 0.31, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(_mesh(_make_base_frame(), "base_frame.obj"), material=paint_dark, name="frame")
    base_frame.inertial = Inertial.from_geometry(
        Box((BASE_BEAM_LEN, BASE_CROSS_LEN, PEDESTAL_Z1)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_Z1 / 2.0)),
    )

    base_cover = model.part("base_cover")
    base_cover_geom = _make_outward_cover(0.150, 0.090, BASE_COVER_T)
    base_cover.visual(_mesh(base_cover_geom, "base_cover.obj"), material=cover_gray, name="cover_plate")
    base_cover.inertial = Inertial.from_geometry(Box((0.150, BASE_COVER_T, 0.090)), mass=0.8)

    outer_mast = model.part("outer_mast")
    outer_tube = _make_outer_mast_tube()
    outer_flange = cq.Workplane("XY", origin=(0.0, 0.0, OUTER_FLANGE_T / 2.0)).box(
        OUTER_FLANGE,
        OUTER_FLANGE,
        OUTER_FLANGE_T,
    )
    outer_mast.visual(_mesh(outer_flange, "outer_flange.obj"), material=paint_gray, name="base_flange")
    outer_mast.visual(_mesh(outer_tube, "outer_tube.obj"), material=paint_dark, name="outer_tube")
    outer_mast.inertial = Inertial.from_geometry(
        Box((OUTER_FLANGE, OUTER_FLANGE, OUTER_TOTAL_H)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TOTAL_H / 2.0)),
    )

    outer_clamp = model.part("outer_clamp")
    outer_collar, outer_hardware = _make_clamp(OUTER_CLAMP_OUTER, OUTER_CLAMP_INNER, OUTER_CLAMP_H)
    outer_clamp.visual(_mesh(outer_collar, "outer_clamp.obj"), material=paint_gray, name="outer_collar")
    outer_clamp.visual(_mesh(outer_hardware, "outer_clamp_hardware.obj"), material=steel, name="clamp_hardware")
    outer_clamp.inertial = Inertial.from_geometry(Box((OUTER_CLAMP_OUTER, OUTER_CLAMP_OUTER, OUTER_CLAMP_H)), mass=2.2)

    mid_stage = model.part("mid_stage")
    mid_tube = _make_mid_tube()
    mid_pads = _make_stage_wear_pads(MID_SIZE, 0.002, 0.020, 0.090)
    mid_stage.visual(_mesh(mid_tube, "mid_stage_tube.obj"), material=paint_gray, name="mid_tube")
    mid_stage.visual(_mesh(mid_pads, "mid_stage_pads.obj"), material=bronze, name="wear_pads")
    mid_stage.inertial = Inertial.from_geometry(
        Box((MID_SIZE + 0.004, MID_SIZE + 0.004, MID_LEN)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, MID_LEN / 2.0)),
    )

    mid_cover = model.part("mid_cover")
    mid_cover_geom = _make_flush_rear_cover(0.064, 0.176, MID_COVER_T)
    mid_cover.visual(_mesh(mid_cover_geom, "mid_cover.obj"), material=cover_gray, name="cover_plate")
    mid_cover.inertial = Inertial.from_geometry(Box((0.064, MID_COVER_T, 0.176)), mass=0.45)

    mid_clamp = model.part("mid_clamp")
    mid_collar, mid_hardware = _make_clamp(MID_CLAMP_OUTER, MID_CLAMP_INNER, MID_CLAMP_H)
    mid_clamp.visual(_mesh(mid_collar, "mid_clamp.obj"), material=paint_gray, name="mid_collar")
    mid_clamp.visual(_mesh(mid_hardware, "mid_clamp_hardware.obj"), material=steel, name="clamp_hardware")
    mid_clamp.inertial = Inertial.from_geometry(Box((MID_CLAMP_OUTER, MID_CLAMP_OUTER, MID_CLAMP_H)), mass=1.6)

    inner_stage = model.part("inner_stage")
    inner_tube = _box_tube(INNER_SIZE, INNER_WALL, INNER_LEN)
    inner_pads = _make_stage_wear_pads(INNER_SIZE, 0.002, 0.016, 0.080)
    head_mount_plate = cq.Workplane("XY", origin=(0.0, 0.0, INNER_LEN + (HEAD_MOUNT_T / 2.0))).box(
        HEAD_MOUNT_W,
        HEAD_MOUNT_D,
        HEAD_MOUNT_T,
    )
    inner_stage.visual(_mesh(inner_tube, "inner_stage_tube.obj"), material=paint_gray, name="inner_tube")
    inner_stage.visual(_mesh(inner_pads, "inner_stage_pads.obj"), material=bronze, name="wear_pads")
    inner_stage.visual(_mesh(head_mount_plate, "inner_head_mount.obj"), material=paint_dark, name="head_mount_plate")
    inner_stage.inertial = Inertial.from_geometry(
        Box((HEAD_MOUNT_W, HEAD_MOUNT_D, INNER_TOP)),
        mass=5.4,
        origin=Origin(xyz=(0.0, 0.0, INNER_TOP / 2.0)),
    )

    head_support = model.part("head_support")
    support_base, upper_cap = _make_head_support()
    head_support.visual(_mesh(support_base, "head_support_base.obj"), material=paint_dark, name="support_base")
    head_support.visual(_mesh(upper_cap, "head_support_cap.obj"), material=steel, name="upper_cap")
    head_support.inertial = Inertial.from_geometry(Box((0.18, 0.18, 0.14)), mass=3.1)

    pan_head = model.part("pan_head")
    pan_hub, deck_assembly, pan_hardware = _make_pan_head()
    pan_head.visual(_mesh(pan_hub, "pan_head_hub.obj"), material=steel, name="hub_spindle")
    pan_head.visual(_mesh(deck_assembly, "pan_head_deck.obj"), material=paint_gray, name="deck_assembly")
    pan_head.visual(_mesh(pan_hardware, "pan_head_hardware.obj"), material=steel, name="head_hardware")
    pan_head.inertial = Inertial.from_geometry(Box((0.18, 0.10, 0.23)), mass=4.0, origin=Origin(xyz=(0.0, 0.0, 0.09)))

    model.articulation(
        "base_to_cover",
        ArticulationType.FIXED,
        parent=base_frame,
        child=base_cover,
        origin=Origin(xyz=(0.0, PEDESTAL_D / 2.0, PEDESTAL_Z0 + 0.075)),
    )
    model.articulation(
        "base_to_outer",
        ArticulationType.FIXED,
        parent=base_frame,
        child=outer_mast,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_Z1)),
    )
    model.articulation(
        "outer_to_outer_clamp",
        ArticulationType.FIXED,
        parent=outer_mast,
        child=outer_clamp,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TOTAL_H + (OUTER_CLAMP_H / 2.0))),
    )
    model.articulation(
        "outer_to_mid",
        ArticulationType.PRISMATIC,
        parent=outer_mast,
        child=mid_stage,
        origin=Origin(xyz=(0.0, 0.0, MID_REST_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=0.45, lower=-0.08, upper=0.20),
    )
    model.articulation(
        "mid_to_cover",
        ArticulationType.FIXED,
        parent=outer_mast,
        child=mid_cover,
        origin=Origin(xyz=(0.0, -(OUTER_SIZE / 2.0), OUTER_FLANGE_T + 0.40)),
    )
    model.articulation(
        "mid_to_mid_clamp",
        ArticulationType.FIXED,
        parent=mid_stage,
        child=mid_clamp,
        origin=Origin(xyz=(0.0, 0.0, MID_LEN + (MID_CLAMP_H / 2.0))),
    )
    model.articulation(
        "mid_to_inner",
        ArticulationType.PRISMATIC,
        parent=mid_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, INNER_REST_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.40, lower=-0.06, upper=0.22),
    )
    model.articulation(
        "inner_to_support",
        ArticulationType.FIXED,
        parent=inner_stage,
        child=head_support,
        origin=Origin(xyz=(0.0, 0.0, INNER_TOP + SUPPORT_BASE_DROP)),
    )
    model.articulation(
        "support_to_pan",
        ArticulationType.CONTINUOUS,
        parent=head_support,
        child=pan_head,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    base_cover = object_model.get_part("base_cover")
    outer_mast = object_model.get_part("outer_mast")
    outer_clamp = object_model.get_part("outer_clamp")
    mid_stage = object_model.get_part("mid_stage")
    mid_cover = object_model.get_part("mid_cover")
    mid_clamp = object_model.get_part("mid_clamp")
    inner_stage = object_model.get_part("inner_stage")
    head_support = object_model.get_part("head_support")
    pan_head = object_model.get_part("pan_head")

    mid_slide = object_model.get_articulation("outer_to_mid")
    inner_slide = object_model.get_articulation("mid_to_inner")
    pan = object_model.get_articulation("support_to_pan")

    base_cover_plate = base_cover.get_visual("cover_plate")
    base_frame_visual = base_frame.get_visual("frame")
    outer_flange = outer_mast.get_visual("base_flange")
    outer_tube = outer_mast.get_visual("outer_tube")
    outer_collar = outer_clamp.get_visual("outer_collar")
    mid_tube = mid_stage.get_visual("mid_tube")
    mid_cover_plate = mid_cover.get_visual("cover_plate")
    mid_collar = mid_clamp.get_visual("mid_collar")
    inner_tube = inner_stage.get_visual("inner_tube")
    head_mount_plate = inner_stage.get_visual("head_mount_plate")
    support_base = head_support.get_visual("support_base")
    support_cap = head_support.get_visual("upper_cap")
    pan_deck = pan_head.get_visual("deck_assembly")

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
        base_cover,
        base_frame,
        elem_a=base_cover_plate,
        elem_b=base_frame_visual,
        reason="Pedestal access cover is retained inside the base-frame service opening, so its recessed flange intentionally nests into the surrounding welded frame cutout.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=18)

    ctx.expect_contact(base_cover, base_frame, elem_a=base_cover_plate, name="base_cover_mounted")
    ctx.expect_contact(outer_mast, base_frame, elem_a=outer_flange, name="outer_mast_seated")
    ctx.expect_contact(outer_clamp, outer_mast, elem_a=outer_collar, name="outer_clamp_seated")
    ctx.expect_contact(mid_cover, outer_mast, elem_a=mid_cover_plate, elem_b=outer_tube, name="mid_cover_mounted")
    ctx.expect_contact(mid_clamp, mid_stage, elem_a=mid_collar, name="mid_clamp_seated")
    ctx.expect_contact(
        head_support,
        inner_stage,
        elem_a=support_base,
        elem_b=head_mount_plate,
        name="head_support_seated",
    )

    ctx.expect_overlap(mid_stage, outer_mast, axes="xy", min_overlap=0.12, elem_a=mid_tube, elem_b=outer_tube)
    ctx.expect_origin_distance(mid_stage, outer_mast, axes="xy", max_dist=0.001, name="mid_stage_centered")
    ctx.expect_overlap(inner_stage, mid_stage, axes="xy", min_overlap=0.09, elem_a=inner_tube, elem_b=mid_tube)
    ctx.expect_origin_distance(inner_stage, mid_stage, axes="xy", max_dist=0.001, name="inner_stage_centered")
    ctx.expect_overlap(pan_head, head_support, axes="xy", min_overlap=0.05, name="pan_head_over_support")
    ctx.expect_origin_distance(pan_head, head_support, axes="xy", max_dist=0.001, name="pan_axis_centered")
    ctx.expect_gap(
        pan_head,
        head_support,
        axis="z",
        positive_elem=pan_deck,
        negative_elem=support_cap,
        min_gap=0.010,
        max_gap=0.040,
        name="pan_deck_clears_support_cap",
    )

    for label, pose_map in (
        ("rest", {}),
        ("extended", {mid_slide: 0.18, inner_slide: 0.20}),
        ("retracted", {mid_slide: -0.05, inner_slide: -0.04}),
    ):
        with ctx.pose(pose_map):
            ctx.expect_overlap(
                mid_stage,
                outer_mast,
                axes="xy",
                min_overlap=0.12,
                elem_a=mid_tube,
                elem_b=outer_tube,
                name=f"mid_stage_guided_{label}",
            )
            ctx.expect_overlap(
                inner_stage,
                mid_stage,
                axes="xy",
                min_overlap=0.09,
                elem_a=inner_tube,
                elem_b=mid_tube,
                name=f"inner_stage_guided_{label}",
            )

    for angle, label in ((0.0, "zero"), (math.pi / 2.0, "quarter_turn"), (-math.pi / 3.0, "negative_sixty")):
        with ctx.pose({pan: angle}):
            ctx.expect_overlap(
                pan_head,
                head_support,
                axes="xy",
                min_overlap=0.05,
                name=f"pan_support_overlap_{label}",
            )
            ctx.expect_gap(
                pan_head,
                head_support,
                axis="z",
                positive_elem=pan_deck,
                negative_elem=support_cap,
                min_gap=0.010,
                max_gap=0.040,
                name=f"pan_support_clearance_{label}",
            )

    def _delta(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    with ctx.pose({mid_slide: -0.08}):
        mid_low = ctx.part_world_position(mid_stage)
    with ctx.pose({mid_slide: 0.20}):
        mid_high = ctx.part_world_position(mid_stage)
    mid_travel = _delta(mid_high, mid_low)
    ctx.check(
        "mid_stage_prismatic_travel",
        abs(mid_travel[2] - 0.28) < 1e-4 and abs(mid_travel[0]) < 1e-6 and abs(mid_travel[1]) < 1e-6,
        details=f"Observed travel delta {mid_travel!r}",
    )

    with ctx.pose({inner_slide: -0.06}):
        inner_low = ctx.part_world_position(inner_stage)
    with ctx.pose({inner_slide: 0.22}):
        inner_high = ctx.part_world_position(inner_stage)
    inner_travel = _delta(inner_high, inner_low)
    ctx.check(
        "inner_stage_prismatic_travel",
        abs(inner_travel[2] - 0.28) < 1e-4 and abs(inner_travel[0]) < 1e-6 and abs(inner_travel[1]) < 1e-6,
        details=f"Observed travel delta {inner_travel!r}",
    )

    with ctx.pose({pan: 0.0}):
        pan_zero_pos = ctx.part_world_position(pan_head)
        pan_zero_aabb = ctx.part_world_aabb(pan_head)
    with ctx.pose({pan: math.pi / 2.0}):
        pan_quarter_pos = ctx.part_world_position(pan_head)
        pan_quarter_aabb = ctx.part_world_aabb(pan_head)
    zero_dx = pan_zero_aabb[1][0] - pan_zero_aabb[0][0]
    zero_dy = pan_zero_aabb[1][1] - pan_zero_aabb[0][1]
    quarter_dx = pan_quarter_aabb[1][0] - pan_quarter_aabb[0][0]
    quarter_dy = pan_quarter_aabb[1][1] - pan_quarter_aabb[0][1]
    ctx.check(
        "pan_head_rotates_about_fixed_axis",
        all(abs(a - b) < 1e-6 for a, b in zip(pan_zero_pos, pan_quarter_pos))
        and zero_dx > zero_dy + 0.03
        and quarter_dy > quarter_dx + 0.03,
        details=(
            f"origin_zero={pan_zero_pos!r}, origin_quarter={pan_quarter_pos!r}, "
            f"zero_dims=({zero_dx:.4f}, {zero_dy:.4f}), quarter_dims=({quarter_dx:.4f}, {quarter_dy:.4f})"
        ),
    )

    outer_box = ctx.part_element_world_aabb(outer_mast, elem=outer_tube)
    mid_box = ctx.part_element_world_aabb(mid_stage, elem=mid_tube)
    inner_box = ctx.part_element_world_aabb(inner_stage, elem=inner_tube)
    outer_width = outer_box[1][0] - outer_box[0][0]
    mid_width = mid_box[1][0] - mid_box[0][0]
    inner_width = inner_box[1][0] - inner_box[0][0]
    ctx.check(
        "stage_size_hierarchy",
        outer_width > mid_width > inner_width and (outer_width - inner_width) > 0.05,
        details=f"outer={outer_width:.4f}, mid={mid_width:.4f}, inner={inner_width:.4f}",
    )

    base_box = ctx.part_world_aabb(base_frame)
    pan_box = ctx.part_world_aabb(pan_head)
    total_height = pan_box[1][2] - base_box[0][2]
    ctx.check(
        "assembly_height_reads_as_mast",
        1.70 < total_height < 1.95,
        details=f"rest height {total_height:.4f} m",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
