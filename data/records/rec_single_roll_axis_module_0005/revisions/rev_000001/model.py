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
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_L = 0.76
BASE_W = 0.30
BASE_T = 0.022
BASE_PAD_L = 0.132
BASE_PAD_W = 0.170
BASE_PAD_T = 0.008

AXIS_Z = 0.20
SUPPORT_X = 0.25
SUPPORT_T = 0.036
SUPPORT_PASSAGE_R = 0.0295
SUPPORT_BOSS_OUTER_R = 0.056
SUPPORT_BOSS_T = 0.010
SUPPORT_BOTTOM_Z = BASE_T + BASE_PAD_T - AXIS_Z

COVER_T = 0.006
CARTRIDGE_FLANGE_T = 0.010
CARTRIDGE_GUIDE_T = 0.028
GUIDE_INNER_R = 0.0235
GUIDE_OUTER_R = 0.0282

JOURNAL_R = 0.0215
COLLAR_BORE_R = JOURNAL_R
COLLAR_OUTER_R = 0.038
COLLAR_T = 0.014

FRAME_RING_CENTER_X = 0.108
RING_T = 0.018
RING_OUTER_R = 0.123
RING_INNER_R = 0.090
BARREL_OUTER_R = 0.084
BARREL_INNER_R = 0.070
STRUT_W = 0.020
STRUT_H = 0.016
STRUT_OFFSET = 0.080

CROSS_TIE_L = 2.0 * (SUPPORT_X - SUPPORT_T / 2.0)
CROSS_TIE_W = 0.032
CROSS_TIE_H = 0.040
CROSS_TIE_Y = -0.084
CROSS_TIE_Z = 0.055


def _hand_start(proto_start: float, length: float, hand: int) -> float:
    return proto_start if hand > 0 else -proto_start - length


def _box_xyz(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _cyl_x(radius: float, start_x: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((start_x, 0.0, 0.0))


def _annulus_x(outer_r: float, inner_r: float, start_x: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(length)
        .translate((start_x, 0.0, 0.0))
    )


def _union_all(solids: list[cq.Workplane]) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _bolt_circle_x(
    start_x: float,
    length: float,
    head_r: float,
    bolt_circle_r: float,
    angles_deg: tuple[float, ...],
) -> cq.Workplane:
    return _union_all(
        [
            _cyl_x(head_r, start_x, length).translate(
                (
                    0.0,
                    bolt_circle_r * math.cos(math.radians(angle_deg)),
                    bolt_circle_r * math.sin(math.radians(angle_deg)),
                )
            )
            for angle_deg in angles_deg
        ]
    )


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material, name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, filename, assets=ASSETS, tolerance=0.0008, angular_tolerance=0.08),
        material=material,
        name=name,
    )


def _base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
    for x_pos in (-SUPPORT_X, SUPPORT_X):
        base = base.union(_box_xyz(BASE_PAD_L, BASE_PAD_W, BASE_PAD_T, (x_pos, 0.0, BASE_T + BASE_PAD_T / 2.0)))
    for x_pos, y_pos in (
        (-SUPPORT_X, -0.104),
        (-SUPPORT_X, 0.104),
        (SUPPORT_X, -0.104),
        (SUPPORT_X, 0.104),
    ):
        slot = (
            cq.Workplane("XY")
            .center(x_pos, y_pos)
            .slot2D(0.058, 0.014)
            .extrude(BASE_T + BASE_PAD_T + 0.004)
            .translate((0.0, 0.0, -0.002))
        )
        base = base.cut(slot)
    for x_pos in (-0.16, 0.16):
        base = base.cut(_box_xyz(0.080, 0.110, 0.010, (x_pos, 0.0, BASE_T - 0.005)))
    return base


def _support_shape(hand: int) -> cq.Workplane:
    profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-0.092, SUPPORT_BOTTOM_Z),
                (0.092, SUPPORT_BOTTOM_Z),
                (0.092, SUPPORT_BOTTOM_Z + 0.030),
                (0.062, SUPPORT_BOTTOM_Z + 0.030),
                (0.062, -0.038),
                (0.074, 0.000),
                (0.061, 0.069),
                (0.050, 0.082),
                (-0.050, 0.082),
                (-0.061, 0.069),
                (-0.074, 0.000),
                (-0.062, -0.038),
                (-0.062, SUPPORT_BOTTOM_Z + 0.030),
                (-0.092, SUPPORT_BOTTOM_Z + 0.030),
            ]
        )
        .close()
        .extrude(SUPPORT_T / 2.0, both=True)
    )
    hole = _cyl_x(SUPPORT_PASSAGE_R, -SUPPORT_T, 2.0 * SUPPORT_T)
    lower_window = (
        cq.Workplane("YZ")
        .center(0.0, -0.082)
        .rect(0.072, 0.054)
        .extrude(SUPPORT_T / 2.0 + 0.002, both=True)
    )
    crown_relief = (
        cq.Workplane("YZ")
        .center(0.0, 0.030)
        .rect(0.066, 0.040)
        .extrude(SUPPORT_T / 2.0 + 0.002, both=True)
    )
    foot_web = _box_xyz(SUPPORT_T, 0.126, 0.012, (0.0, 0.0, SUPPORT_BOTTOM_Z + 0.036))
    inboard_boss = _annulus_x(
        SUPPORT_BOSS_OUTER_R,
        SUPPORT_PASSAGE_R + 0.0035,
        _hand_start(SUPPORT_T / 2.0 - SUPPORT_BOSS_T, SUPPORT_BOSS_T, hand),
        SUPPORT_BOSS_T,
    )
    return profile.cut(hole).cut(lower_window).cut(crown_relief).union(foot_web).union(inboard_boss)


def _bearing_cartridge_shape(hand: int) -> tuple[cq.Workplane, cq.Workplane]:
    flange_start = _hand_start(-SUPPORT_T / 2.0 - CARTRIDGE_FLANGE_T, CARTRIDGE_FLANGE_T, hand)
    guide_start = _hand_start(-SUPPORT_T / 2.0, CARTRIDGE_GUIDE_T, hand)
    flange = _annulus_x(0.056, GUIDE_OUTER_R, flange_start, CARTRIDGE_FLANGE_T)
    guide = _annulus_x(GUIDE_OUTER_R, GUIDE_INNER_R, guide_start, CARTRIDGE_GUIDE_T)
    bolts = _bolt_circle_x(
        _hand_start(-SUPPORT_T / 2.0 - CARTRIDGE_FLANGE_T - 0.004, 0.004, hand),
        0.004,
        0.0046,
        0.046,
        (35.0, 145.0, 215.0, 325.0),
    )
    return flange.union(bolts), guide


def _access_cover_shape(hand: int) -> cq.Workplane:
    cover_start = _hand_start(SUPPORT_T / 2.0, COVER_T, hand)
    cover = (
        cq.Workplane("YZ")
        .center(0.0, -0.082)
        .rect(0.108, 0.082)
        .extrude(COVER_T)
        .translate((cover_start, 0.0, 0.0))
    )
    inspection_relief = (
        cq.Workplane("YZ")
        .center(0.0, -0.082)
        .rect(0.076, 0.050)
        .extrude(COVER_T * 0.55)
        .translate((cover_start, 0.0, 0.0))
    )
    sight_hole = _cyl_x(0.016, cover_start - 0.001, COVER_T + 0.002).translate((0.0, 0.0, -0.082))
    perimeter = (
        cq.Workplane("YZ")
        .center(0.0, -0.082)
        .rect(0.108, 0.082)
        .rect(0.094, 0.068)
        .extrude(0.0028)
        .translate((_hand_start(SUPPORT_T / 2.0 + COVER_T, 0.0028, hand), 0.0, 0.0))
    )
    bolts = _union_all(
        [
            _cyl_x(
                0.0042,
                _hand_start(SUPPORT_T / 2.0 + COVER_T, 0.004, hand),
                0.004,
            ).translate((0.0, y_pos, z_pos))
            for y_pos, z_pos in ((0.040, -0.054), (0.040, -0.110), (-0.040, -0.054), (-0.040, -0.110))
        ]
    )
    return cover.cut(inspection_relief).cut(sight_hole).union(perimeter).union(bolts)


def _toe_clamp_shape(hand: int) -> cq.Workplane:
    block_length = 0.026
    block_height = 0.020
    block_center_x = hand * (-SUPPORT_T / 2.0 - block_length / 2.0)
    blocks = [
        _box_xyz(block_length, 0.028, block_height, (block_center_x, 0.064, SUPPORT_BOTTOM_Z + block_height / 2.0)),
        _box_xyz(block_length, 0.028, block_height, (block_center_x, -0.064, SUPPORT_BOTTOM_Z + block_height / 2.0)),
        _box_xyz(block_length, 0.150, 0.008, (block_center_x, 0.0, SUPPORT_BOTTOM_Z + 0.004)),
    ]
    studs = _union_all(
        [
            cq.Workplane("XY")
            .circle(0.0045)
            .extrude(0.006)
            .translate((block_center_x, y_pos, SUPPORT_BOTTOM_Z + block_height))
            for y_pos in (0.064, -0.064)
        ]
    )
    return _union_all(blocks).union(studs)


def _cross_tie_shape() -> cq.Workplane:
    beam = _box_xyz(CROSS_TIE_L, CROSS_TIE_W, CROSS_TIE_H, (0.0, CROSS_TIE_Y, CROSS_TIE_Z))
    inner = _box_xyz(
        CROSS_TIE_L - 0.060,
        CROSS_TIE_W - 0.014,
        CROSS_TIE_H - 0.014,
        (0.0, CROSS_TIE_Y, CROSS_TIE_Z + 0.002),
    )
    end_caps = [
        _box_xyz(0.012, 0.042, 0.050, (-CROSS_TIE_L / 2.0 + 0.006, CROSS_TIE_Y, CROSS_TIE_Z)),
        _box_xyz(0.012, 0.042, 0.050, (CROSS_TIE_L / 2.0 - 0.006, CROSS_TIE_Y, CROSS_TIE_Z)),
    ]
    return beam.cut(inner).union(_union_all(end_caps))


def _roll_frame_shape() -> cq.Workplane:
    left_ring = _annulus_x(RING_OUTER_R, RING_INNER_R, -FRAME_RING_CENTER_X - RING_T / 2.0, RING_T)
    right_ring = _annulus_x(RING_OUTER_R, RING_INNER_R, FRAME_RING_CENTER_X - RING_T / 2.0, RING_T)
    barrel = _annulus_x(BARREL_OUTER_R, BARREL_INNER_R, -0.099, 0.198)
    left_hub_pad = _cyl_x(0.074, -FRAME_RING_CENTER_X - RING_T / 2.0, RING_T)
    right_hub_pad = _cyl_x(0.074, FRAME_RING_CENTER_X - RING_T / 2.0, RING_T)
    strut_len = 2.0 * FRAME_RING_CENTER_X - RING_T
    struts = [
        _box_xyz(strut_len, STRUT_W, STRUT_H, (0.0, STRUT_OFFSET, STRUT_OFFSET)),
        _box_xyz(strut_len, STRUT_W, STRUT_H, (0.0, STRUT_OFFSET, -STRUT_OFFSET)),
        _box_xyz(strut_len, STRUT_W, STRUT_H, (0.0, -STRUT_OFFSET, STRUT_OFFSET)),
        _box_xyz(strut_len, STRUT_W, STRUT_H, (0.0, -STRUT_OFFSET, -STRUT_OFFSET)),
    ]
    return _union_all([left_ring, right_ring, barrel, left_hub_pad, right_hub_pad] + struts)


def _hub_shell_shape(hand: int) -> cq.Workplane:
    stages = [
        _cyl_x(0.026, _hand_start(0.044, 0.020, hand), 0.020),
        _cyl_x(0.034, _hand_start(0.064, 0.020, hand), 0.020),
        _cyl_x(0.050, _hand_start(0.084, 0.018, hand), 0.018),
        _cyl_x(0.074, _hand_start(0.102, 0.031, hand), 0.031),
    ]
    wrench_flat = _box_xyz(0.020, 0.070, 0.014, (hand * 0.074, 0.0, 0.0))
    return _union_all(stages).cut(wrench_flat)


def _collar_shape(hand: int) -> cq.Workplane:
    collar_start = _hand_start(0.028, COLLAR_T, hand)
    body = (
        cq.Workplane("YZ")
        .circle(COLLAR_OUTER_R)
        .extrude(COLLAR_T)
        .translate((collar_start, 0.0, 0.0))
    )
    bore = _cyl_x(COLLAR_BORE_R, collar_start - 0.001, COLLAR_T + 0.002)
    split_gap = _box_xyz(COLLAR_T + 0.004, 0.004, 0.030, (hand * (0.028 + COLLAR_T / 2.0), 0.034, 0.0))
    upper_lug = _box_xyz(COLLAR_T, 0.010, 0.012, (hand * (0.028 + COLLAR_T / 2.0), 0.039, 0.010))
    lower_lug = _box_xyz(COLLAR_T, 0.010, 0.012, (hand * (0.028 + COLLAR_T / 2.0), 0.039, -0.010))
    clamp_hole = (
        cq.Workplane("XY")
        .circle(0.0028)
        .extrude(0.028)
        .translate((hand * (0.028 + COLLAR_T / 2.0), 0.039, -0.014))
    )
    screw_head = (
        cq.Workplane("XY")
        .circle(0.0048)
        .extrude(0.004)
        .translate((hand * (0.028 + COLLAR_T / 2.0), 0.039, 0.014))
    )
    nut_pad = _box_xyz(COLLAR_T, 0.010, 0.004, (hand * (0.028 + COLLAR_T / 2.0), 0.039, -0.014))
    return body.cut(bore).cut(split_gap).union(upper_lug).union(lower_lug).cut(clamp_hole).union(screw_head).union(nut_pad)


def _index_band_shape() -> cq.Workplane:
    band = _annulus_x(
        RING_OUTER_R + 0.014,
        RING_OUTER_R - 0.006,
        -FRAME_RING_CENTER_X - RING_T / 2.0 - 0.008,
        0.008,
    )
    ticks: list[cq.Workplane] = []
    for angle_deg in range(0, 360, 15):
        angle = math.radians(angle_deg)
        tick_r = RING_OUTER_R + (0.010 if angle_deg % 45 == 0 else 0.007)
        tick_h = 0.009 if angle_deg % 45 == 0 else 0.006
        ticks.append(
            _box_xyz(
                0.004,
                0.004,
                tick_h,
                (
                    -FRAME_RING_CENTER_X - RING_T / 2.0 - 0.010,
                    tick_r * math.cos(angle),
                    tick_r * math.sin(angle),
                ),
            )
        )
    return band.union(_union_all(ticks))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_roll_axis_module", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.34, 0.37, 0.41, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.67, 0.70, 0.73, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.16, 0.17, 0.19, 1.0))
    index_finish = model.material("index_finish", rgba=(0.78, 0.71, 0.23, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _base_shape(), "base_plate.obj", painted_steel, "base_shell")

    left_support = model.part("left_support")
    _add_mesh_visual(left_support, _support_shape(hand=1), "left_support.obj", painted_steel, "support_shell")

    right_support = model.part("right_support")
    _add_mesh_visual(right_support, _support_shape(hand=-1), "right_support.obj", painted_steel, "support_shell")

    cross_tie = model.part("cross_tie")
    _add_mesh_visual(cross_tie, _cross_tie_shape(), "cross_tie.obj", painted_steel, "tie_shell")

    left_cartridge = model.part("left_bearing_cartridge")
    left_flange, left_guide = _bearing_cartridge_shape(hand=1)
    _add_mesh_visual(left_cartridge, left_flange, "left_cartridge_flange.obj", black_oxide, "flange_shell")
    _add_mesh_visual(left_cartridge, left_guide, "left_cartridge_guide.obj", machined_steel, "guide_ring")

    right_cartridge = model.part("right_bearing_cartridge")
    right_flange, right_guide = _bearing_cartridge_shape(hand=-1)
    _add_mesh_visual(right_cartridge, right_flange, "right_cartridge_flange.obj", black_oxide, "flange_shell")
    _add_mesh_visual(right_cartridge, right_guide, "right_cartridge_guide.obj", machined_steel, "guide_ring")

    left_access_cover = model.part("left_access_cover")
    _add_mesh_visual(left_access_cover, _access_cover_shape(hand=1), "left_access_cover.obj", black_oxide, "cover_shell")

    right_access_cover = model.part("right_access_cover")
    _add_mesh_visual(right_access_cover, _access_cover_shape(hand=-1), "right_access_cover.obj", black_oxide, "cover_shell")

    left_clamp = model.part("left_clamp_block")
    _add_mesh_visual(left_clamp, _toe_clamp_shape(hand=1), "left_toe_clamp.obj", black_oxide, "clamp_shell")

    right_clamp = model.part("right_clamp_block")
    _add_mesh_visual(right_clamp, _toe_clamp_shape(hand=-1), "right_toe_clamp.obj", black_oxide, "clamp_shell")

    roll_frame = model.part("roll_frame")
    _add_mesh_visual(roll_frame, _roll_frame_shape(), "roll_frame.obj", painted_steel, "frame_shell")

    left_hub = model.part("left_hub")
    _add_mesh_visual(left_hub, _hub_shell_shape(hand=1), "left_hub_shell.obj", machined_steel, "hub_shell")
    left_hub.visual(
        Cylinder(radius=JOURNAL_R, length=0.080),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="journal",
    )

    right_hub = model.part("right_hub")
    _add_mesh_visual(right_hub, _hub_shell_shape(hand=-1), "right_hub_shell.obj", machined_steel, "hub_shell")
    right_hub.visual(
        Cylinder(radius=JOURNAL_R, length=0.080),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="journal",
    )

    left_collar = model.part("left_retaining_collar")
    _add_mesh_visual(left_collar, _collar_shape(hand=1), "left_collar.obj", black_oxide, "collar_shell")

    right_collar = model.part("right_retaining_collar")
    _add_mesh_visual(right_collar, _collar_shape(hand=-1), "right_collar.obj", black_oxide, "collar_shell")

    index_band = model.part("index_band")
    _add_mesh_visual(index_band, _index_band_shape(), "index_band.obj", index_finish, "index_shell")

    model.articulation(
        "base_to_left_support",
        ArticulationType.FIXED,
        parent=base,
        child=left_support,
        origin=Origin(xyz=(-SUPPORT_X, 0.0, AXIS_Z)),
    )
    model.articulation(
        "base_to_right_support",
        ArticulationType.FIXED,
        parent=base,
        child=right_support,
        origin=Origin(xyz=(SUPPORT_X, 0.0, AXIS_Z)),
    )
    model.articulation(
        "base_to_cross_tie",
        ArticulationType.FIXED,
        parent=base,
        child=cross_tie,
        origin=Origin(),
    )

    model.articulation(
        "left_support_to_cartridge",
        ArticulationType.FIXED,
        parent=left_support,
        child=left_cartridge,
        origin=Origin(),
    )
    model.articulation(
        "right_support_to_cartridge",
        ArticulationType.FIXED,
        parent=right_support,
        child=right_cartridge,
        origin=Origin(),
    )
    model.articulation(
        "left_support_to_cover",
        ArticulationType.FIXED,
        parent=left_support,
        child=left_access_cover,
        origin=Origin(),
    )
    model.articulation(
        "right_support_to_cover",
        ArticulationType.FIXED,
        parent=right_support,
        child=right_access_cover,
        origin=Origin(),
    )
    model.articulation(
        "left_support_to_clamp",
        ArticulationType.FIXED,
        parent=left_support,
        child=left_clamp,
        origin=Origin(),
    )
    model.articulation(
        "right_support_to_clamp",
        ArticulationType.FIXED,
        parent=right_support,
        child=right_clamp,
        origin=Origin(),
    )

    model.articulation(
        "base_to_roll_frame",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=roll_frame,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.5),
    )
    model.articulation(
        "roll_frame_to_left_hub",
        ArticulationType.FIXED,
        parent=roll_frame,
        child=left_hub,
        origin=Origin(xyz=(-SUPPORT_X, 0.0, 0.0)),
    )
    model.articulation(
        "roll_frame_to_right_hub",
        ArticulationType.FIXED,
        parent=roll_frame,
        child=right_hub,
        origin=Origin(xyz=(SUPPORT_X, 0.0, 0.0)),
    )
    model.articulation(
        "left_hub_to_collar",
        ArticulationType.FIXED,
        parent=left_hub,
        child=left_collar,
        origin=Origin(),
    )
    model.articulation(
        "right_hub_to_collar",
        ArticulationType.FIXED,
        parent=right_hub,
        child=right_collar,
        origin=Origin(),
    )
    model.articulation(
        "roll_frame_to_index_band",
        ArticulationType.FIXED,
        parent=roll_frame,
        child=index_band,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    left_support = object_model.get_part("left_support")
    right_support = object_model.get_part("right_support")
    cross_tie = object_model.get_part("cross_tie")
    left_cartridge = object_model.get_part("left_bearing_cartridge")
    right_cartridge = object_model.get_part("right_bearing_cartridge")
    left_access_cover = object_model.get_part("left_access_cover")
    right_access_cover = object_model.get_part("right_access_cover")
    left_clamp = object_model.get_part("left_clamp_block")
    right_clamp = object_model.get_part("right_clamp_block")
    roll_frame = object_model.get_part("roll_frame")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")
    left_collar = object_model.get_part("left_retaining_collar")
    right_collar = object_model.get_part("right_retaining_collar")
    index_band = object_model.get_part("index_band")
    roll_joint = object_model.get_articulation("base_to_roll_frame")

    left_journal = left_hub.get_visual("journal")
    right_journal = right_hub.get_visual("journal")
    left_guide = left_cartridge.get_visual("guide_ring")
    right_guide = right_cartridge.get_visual("guide_ring")
    left_collar_shell = left_collar.get_visual("collar_shell")
    right_collar_shell = right_collar.get_visual("collar_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        left_hub,
        left_collar,
        elem_a=left_journal,
        elem_b=left_collar_shell,
        reason="split retaining collar intentionally clamps the left journal with a slight interference seat",
    )
    ctx.allow_overlap(
        right_hub,
        right_collar,
        elem_a=right_journal,
        elem_b=right_collar_shell,
        reason="split retaining collar intentionally clamps the right journal with a slight interference seat",
    )

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(max_pose_samples=4)
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

    ctx.expect_gap(left_support, base, axis="z", max_gap=0.001, max_penetration=0.0, name="left_support_seated")
    ctx.expect_gap(right_support, base, axis="z", max_gap=0.001, max_penetration=0.0, name="right_support_seated")
    ctx.expect_overlap(left_support, base, axes="xy", min_overlap=0.03, name="left_support_footprint")
    ctx.expect_overlap(right_support, base, axes="xy", min_overlap=0.03, name="right_support_footprint")

    ctx.expect_gap(cross_tie, base, axis="z", max_gap=0.001, max_penetration=0.0, name="cross_tie_seated")
    ctx.expect_contact(cross_tie, left_support, name="cross_tie_left_support_contact")
    ctx.expect_contact(cross_tie, right_support, name="cross_tie_right_support_contact")

    ctx.expect_contact(left_cartridge, left_support, name="left_cartridge_mounted")
    ctx.expect_contact(right_cartridge, right_support, name="right_cartridge_mounted")
    ctx.expect_contact(left_access_cover, left_support, name="left_cover_mounted")
    ctx.expect_contact(right_access_cover, right_support, name="right_cover_mounted")
    ctx.expect_gap(left_clamp, base, axis="z", max_gap=0.001, max_penetration=0.0, name="left_clamp_seated")
    ctx.expect_gap(right_clamp, base, axis="z", max_gap=0.001, max_penetration=0.0, name="right_clamp_seated")
    ctx.expect_contact(left_clamp, left_support, name="left_clamp_against_support")
    ctx.expect_contact(right_clamp, right_support, name="right_clamp_against_support")

    ctx.expect_origin_distance(roll_frame, base, axes="xy", max_dist=0.001, name="roll_axis_coaxial_xy")
    ctx.expect_origin_gap(
        roll_frame,
        base,
        axis="z",
        min_gap=AXIS_Z - 0.001,
        max_gap=AXIS_Z + 0.001,
        name="roll_axis_height",
    )
    ctx.expect_contact(left_hub, roll_frame, name="left_hub_seated_to_frame")
    ctx.expect_contact(right_hub, roll_frame, name="right_hub_seated_to_frame")
    ctx.expect_overlap(left_hub, roll_frame, axes="yz", min_overlap=0.12, name="left_hub_frame_footprint")
    ctx.expect_overlap(right_hub, roll_frame, axes="yz", min_overlap=0.12, name="right_hub_frame_footprint")
    ctx.expect_contact(index_band, roll_frame, name="index_band_attached")
    ctx.expect_contact(left_collar, left_hub, elem_b=left_journal, name="left_collar_retains_journal")
    ctx.expect_contact(right_collar, right_hub, elem_b=right_journal, name="right_collar_retains_journal")

    ctx.expect_within(
        left_hub,
        left_cartridge,
        axes="yz",
        inner_elem=left_journal,
        outer_elem=left_guide,
        margin=0.002,
        name="left_journal_within_bearing",
    )
    ctx.expect_within(
        right_hub,
        right_cartridge,
        axes="yz",
        inner_elem=right_journal,
        outer_elem=right_guide,
        margin=0.002,
        name="right_journal_within_bearing",
    )
    ctx.expect_gap(
        left_collar,
        left_access_cover,
        axis="z",
        min_gap=0.002,
        max_gap=0.004,
        name="left_cover_to_collar_clearance",
    )
    ctx.expect_gap(
        right_collar,
        right_access_cover,
        axis="z",
        min_gap=0.002,
        max_gap=0.004,
        name="right_cover_to_collar_clearance",
    )

    for angle, tag in ((0.0, "rest"), (math.pi / 2.0, "quarter_turn"), (math.pi, "inverted")):
        with ctx.pose({roll_joint: angle}):
            ctx.expect_origin_distance(
                roll_frame,
                base,
                axes="xy",
                max_dist=0.001,
                name=f"roll_axis_coaxial_xy_{tag}",
            )
            ctx.expect_origin_gap(
                roll_frame,
                base,
                axis="z",
                min_gap=AXIS_Z - 0.001,
                max_gap=AXIS_Z + 0.001,
                name=f"roll_axis_height_{tag}",
            )
            ctx.expect_within(
                left_hub,
                left_cartridge,
                axes="yz",
                inner_elem=left_journal,
                outer_elem=left_guide,
                margin=0.002,
                name=f"left_journal_supported_{tag}",
            )
            ctx.expect_within(
                right_hub,
                right_cartridge,
                axes="yz",
                inner_elem=right_journal,
                outer_elem=right_guide,
                margin=0.002,
                name=f"right_journal_supported_{tag}",
            )
            ctx.expect_gap(
                left_collar,
                left_access_cover,
                axis="z",
                min_gap=0.002,
                max_gap=0.004,
                name=f"left_collar_clearance_{tag}",
            )
            ctx.expect_gap(
                right_collar,
                right_access_cover,
                axis="z",
                min_gap=0.002,
                max_gap=0.004,
                name=f"right_collar_clearance_{tag}",
            )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=6,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
