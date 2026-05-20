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

OUTER_LEN = 0.38
MIDDLE_LEN = 0.28
INNER_LEN = 0.20

OUTER_Y = 0.110
OUTER_Z = 0.072
OUTER_WALL = 0.0045

MIDDLE_Y = 0.084
MIDDLE_Z = 0.054
MIDDLE_WALL = 0.0040

INNER_Y = 0.058
INNER_Z = 0.036
INNER_WALL = 0.0035

OUTER_CENTER_Z = 0.086

MIDDLE_REST_X = -0.045
MIDDLE_TRAVEL = 0.295
INNER_REST_X = -0.035
INNER_TRAVEL = 0.210

OUTER_INNER_Y = OUTER_Y - 2.0 * OUTER_WALL
OUTER_INNER_Z = OUTER_Z - 2.0 * OUTER_WALL
MIDDLE_INNER_Y = MIDDLE_Y - 2.0 * MIDDLE_WALL
MIDDLE_INNER_Z = MIDDLE_Z - 2.0 * MIDDLE_WALL

GUIDE_SLEEVE_T = 0.0018
OUTER_SIDE_PAD_T = OUTER_INNER_Y / 2.0 - (MIDDLE_Y / 2.0 + GUIDE_SLEEVE_T)
OUTER_TOP_PAD_T = OUTER_INNER_Z / 2.0 - (MIDDLE_Z / 2.0 + GUIDE_SLEEVE_T)
INNER_SIDE_PAD_T = MIDDLE_INNER_Y / 2.0 - (INNER_Y / 2.0 + GUIDE_SLEEVE_T)
INNER_TOP_PAD_T = MIDDLE_INNER_Z / 2.0 - (INNER_Z / 2.0 + GUIDE_SLEEVE_T)


def _box(size: tuple[float, float, float], offset: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> cq.Workplane:
    return cq.Workplane("XY").box(*size, centered=(True, True, True)).translate(offset)


def _tube_member(
    *,
    length: float,
    outer_y: float,
    outer_z: float,
    wall: float,
    side_windows: list[tuple[float, float, float]],
    top_ports: list[tuple[float, float, float]],
    side_slot_height: float | None = None,
) -> cq.Workplane:
    shape = _box((length, outer_y, outer_z))
    shape = shape.cut(_box((length + 0.006, outer_y - 2.0 * wall, outer_z - 2.0 * wall)))

    for x_center, x_len, z_len in side_windows:
        for side in (-1.0, 1.0):
            shape = shape.cut(
                _box(
                    (x_len, wall + 0.020, z_len),
                    (x_center, side * (outer_y / 2.0 - wall / 2.0), 0.0),
                )
            )

    for x_center, x_len, y_len in top_ports:
        shape = shape.cut(
            _box(
                (x_len, y_len, wall + 0.020),
                (x_center, 0.0, outer_z / 2.0 - wall / 2.0),
            )
        )

    if side_slot_height is not None:
        for side in (-1.0, 1.0):
            shape = shape.cut(
                _box(
                    (0.030, wall + 0.020, side_slot_height),
                    (
                        length / 2.0 - 0.040,
                        side * (outer_y / 2.0 - wall / 2.0),
                        0.0,
                    ),
                )
            )

    return shape


def _support_frame_shape() -> cq.Workplane:
    shape = _box((0.100, 0.165, 0.012), (-0.200, 0.0, 0.006))
    shape = shape.union(_box((0.100, 0.165, 0.012), (0.200, 0.0, 0.006)))
    shape = shape.union(_box((0.300, 0.040, 0.014), (0.0, 0.0, 0.014)))
    shape = shape.union(_box((0.024, 0.072, 0.022), (-0.200, 0.0, 0.020)))
    shape = shape.union(_box((0.024, 0.072, 0.022), (0.200, 0.0, 0.020)))
    shape = shape.union(_box((0.070, 0.075, 0.006), (-0.200, 0.0, 0.018)))
    shape = shape.union(_box((0.070, 0.075, 0.006), (0.200, 0.0, 0.018)))
    shape = shape.union(_box((0.120, 0.018, 0.012), (-0.140, 0.0, 0.020)))
    shape = shape.union(_box((0.120, 0.018, 0.012), (0.140, 0.0, 0.020)))
    return shape


def _access_cover_shape(length: float = 0.090, width: float = 0.046, thickness: float = 0.004) -> cq.Workplane:
    plate = _box((length, width, thickness))
    for sx in (-0.030, 0.030):
        for sy in (-0.014, 0.014):
            boss = cq.Workplane("XY").cylinder(0.003, 0.0032).translate((sx, sy, thickness / 2.0 + 0.0015))
            plate = plate.union(boss)
    plate = plate.cut(_box((0.032, 0.012, 0.003), (0.0, 0.0, 0.0015)))
    return plate


def _boxed_end_bracket_shape(member_y: float, member_z: float) -> cq.Workplane:
    y_outer = member_y + 0.022
    z_outer = member_z + 0.020
    plate_t = 0.008
    wrap_len = 0.028
    wall_t = 0.006

    shape = cq.Workplane("XY").box(plate_t, y_outer, z_outer, centered=(False, True, True))
    shape = shape.union(
        cq.Workplane("XY")
        .box(wrap_len, wall_t, z_outer, centered=(False, True, True))
        .translate((-wrap_len, member_y / 2.0 + 0.008, 0.0))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(wrap_len, wall_t, z_outer, centered=(False, True, True))
        .translate((-wrap_len, -(member_y / 2.0 + 0.008), 0.0))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(wrap_len, y_outer, wall_t, centered=(False, True, True))
        .translate((-wrap_len, 0.0, member_z / 2.0 + 0.007))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(wrap_len, y_outer, wall_t, centered=(False, True, True))
        .translate((-wrap_len, 0.0, -(member_z / 2.0 + 0.007)))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(0.038, 0.064, 0.010, centered=(True, True, True))
        .translate((0.010, 0.0, -(member_z / 2.0 + 0.014)))
    )
    return shape


def _inner_end_bracket_shape(member_y: float, member_z: float) -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.010, member_y + 0.030, member_z + 0.030, centered=(False, True, True))
    plate = plate.union(
        cq.Workplane("XY")
        .box(0.070, 0.016, 0.050, centered=(False, True, True))
        .translate((0.010, member_y / 2.0 + 0.008, 0.0))
    )
    plate = plate.union(
        cq.Workplane("XY")
        .box(0.070, 0.016, 0.050, centered=(False, True, True))
        .translate((0.010, -(member_y / 2.0 + 0.008), 0.0))
    )
    plate = plate.union(
        cq.Workplane("XY").box(0.080, member_y + 0.036, 0.008, centered=(False, True, True)).translate((0.020, 0.0, 0.0))
    )
    return plate


def _guide_sleeve_shape(length: float, member_y: float, member_z: float) -> cq.Workplane:
    rail_t = 0.0025
    relief = 0.0010
    half_y = member_y / 2.0 + relief + rail_t / 2.0
    half_z = member_z / 2.0 + relief + rail_t / 2.0
    span_y = member_y + 2.0 * (relief + rail_t)
    span_z = member_z + 2.0 * (relief + rail_t)

    shape = _box((length, rail_t, span_z), (0.0, half_y, 0.0))
    shape = shape.union(_box((length, rail_t, span_z), (0.0, -half_y, 0.0)))
    shape = shape.union(_box((length, span_y, rail_t), (0.0, 0.0, half_z)))
    shape = shape.union(_box((length, span_y, rail_t), (0.0, 0.0, -half_z)))
    return shape


def _mesh_visual(part, shape: cq.Workplane, filename: str, *, material, name: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.30, 0.32, 0.35, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.47, 0.49, 0.52, 1.0))
    zinc = model.material("zinc", rgba=(0.69, 0.72, 0.76, 1.0))
    bronze = model.material("bronze_polymer", rgba=(0.59, 0.45, 0.27, 1.0))
    pad_gray = model.material("pad_gray", rgba=(0.79, 0.79, 0.76, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    support_frame = model.part("support_frame")
    _mesh_visual(
        support_frame,
        _support_frame_shape(),
        "support_frame.obj",
        material=steel_dark,
        name="frame_weldment",
    )

    outer_member = model.part("outer_member")
    outer_shape = _tube_member(
        length=OUTER_LEN,
        outer_y=OUTER_Y,
        outer_z=OUTER_Z,
        wall=OUTER_WALL,
        side_windows=[(0.0, 0.110, 0.034)],
        top_ports=[(-0.085, 0.070, 0.030), (0.085, 0.070, 0.030)],
        side_slot_height=0.026,
    )
    _mesh_visual(outer_member, outer_shape, "outer_member.obj", material=steel_mid, name="outer_shell")
    outer_member.visual(
        Box((0.010, 0.020, 0.005)),
        origin=Origin(xyz=(0.161, 0.0, -(OUTER_INNER_Z / 2.0 - 0.0025))),
        material=zinc,
        name="outer_stop_block",
    )

    outer_front_bracket = model.part("outer_front_bracket")
    _mesh_visual(
        outer_front_bracket,
        _boxed_end_bracket_shape(OUTER_Y, OUTER_Z),
        "outer_front_bracket.obj",
        material=steel_dark,
        name="bracket_shell",
    )
    outer_front_bracket.visual(
        Box((0.004, OUTER_Y + 0.006, 0.004)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=rubber,
        name="wiper_lip",
    )
    outer_front_bracket.visual(
        Box((0.006, 0.024, 0.008)),
        origin=Origin(xyz=(-0.016, 0.0, -(OUTER_Z / 2.0 - 0.013))),
        material=zinc,
        name="travel_stop_face",
    )

    outer_rear_bracket = model.part("outer_rear_bracket")
    _mesh_visual(
        outer_rear_bracket,
        _boxed_end_bracket_shape(OUTER_Y, OUTER_Z),
        "outer_rear_bracket.obj",
        material=steel_dark,
        name="bracket_shell",
    )
    outer_rear_bracket.visual(
        Box((0.004, OUTER_Y + 0.006, 0.004)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=rubber,
        name="wiper_lip",
    )

    outer_cover_a = model.part("outer_cover_a")
    _mesh_visual(
        outer_cover_a,
        _access_cover_shape(),
        "outer_cover_a.obj",
        material=zinc,
        name="cover_plate",
    )

    outer_cover_b = model.part("outer_cover_b")
    _mesh_visual(
        outer_cover_b,
        _access_cover_shape(),
        "outer_cover_b.obj",
        material=zinc,
        name="cover_plate",
    )

    middle_member = model.part("middle_member")
    middle_shape = _tube_member(
        length=MIDDLE_LEN,
        outer_y=MIDDLE_Y,
        outer_z=MIDDLE_Z,
        wall=MIDDLE_WALL,
        side_windows=[(0.0, 0.080, 0.026)],
        top_ports=[(0.045, 0.050, 0.020)],
    )
    _mesh_visual(middle_member, middle_shape, "middle_member.obj", material=steel_mid, name="middle_shell")
    middle_member.visual(
        Box((0.012, 0.018, 0.003)),
        origin=Origin(xyz=(-0.100, 0.0, -(MIDDLE_Z / 2.0 - 0.0015))),
        material=zinc,
        name="outer_stop_dog",
    )
    middle_member.visual(
        Box((0.010, 0.016, 0.006)),
        origin=Origin(xyz=(0.101, 0.0, -(MIDDLE_INNER_Z / 2.0 - 0.003))),
        material=zinc,
        name="middle_stop_block",
    )

    middle_guide_cage = model.part("middle_guide_cage")
    _mesh_visual(
        middle_guide_cage,
        _guide_sleeve_shape(0.190, MIDDLE_Y, MIDDLE_Z),
        "middle_guide_cage.obj",
        material=steel_dark,
        name="guide_sleeve",
    )
    for sign, side_name in ((1.0, "left"), (-1.0, "right")):
        for x_pos, end_name in ((-0.078, "rear"), (0.078, "front")):
            middle_guide_cage.visual(
                Box((0.026, OUTER_SIDE_PAD_T, 0.015)),
                origin=Origin(
                    xyz=(
                        x_pos,
                        sign * (MIDDLE_Y / 2.0 + GUIDE_SLEEVE_T + OUTER_SIDE_PAD_T / 2.0),
                        0.0,
                    )
                ),
                material=bronze,
                name=f"side_pad_{side_name}_{end_name}",
            )
    middle_guide_cage.visual(
        Box((0.044, MIDDLE_Y * 0.60, OUTER_TOP_PAD_T)),
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_Z / 2.0 + GUIDE_SLEEVE_T + OUTER_TOP_PAD_T / 2.0)),
        material=pad_gray,
        name="top_pad",
    )
    middle_guide_cage.visual(
        Box((0.044, MIDDLE_Y * 0.60, OUTER_TOP_PAD_T)),
        origin=Origin(xyz=(0.0, 0.0, -(MIDDLE_Z / 2.0 + GUIDE_SLEEVE_T + OUTER_TOP_PAD_T / 2.0))),
        material=pad_gray,
        name="bottom_pad",
    )
    for sign, side_name in ((1.0, "left"), (-1.0, "right")):
        for x_pos in (-0.024, 0.024):
            middle_guide_cage.visual(
                Cylinder(radius=0.005, length=0.010),
                origin=Origin(
                    xyz=(
                        x_pos,
                        sign * (OUTER_INNER_Y / 2.0 - 0.005),
                        0.0,
                    ),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=zinc,
                name=f"roller_{side_name}_{'rear' if x_pos < 0 else 'front'}",
            )

    inner_member = model.part("inner_member")
    inner_shape = _tube_member(
        length=INNER_LEN,
        outer_y=INNER_Y,
        outer_z=INNER_Z,
        wall=INNER_WALL,
        side_windows=[(0.0, 0.050, 0.016)],
        top_ports=[],
    )
    _mesh_visual(inner_member, inner_shape, "inner_member.obj", material=steel_mid, name="inner_shell")
    inner_member.visual(
        Box((0.012, 0.014, 0.003)),
        origin=Origin(xyz=(-0.085, 0.0, -(INNER_Z / 2.0 - 0.0015))),
        material=zinc,
        name="middle_stop_dog",
    )

    inner_guide_cage = model.part("inner_guide_cage")
    _mesh_visual(
        inner_guide_cage,
        _guide_sleeve_shape(0.140, INNER_Y, INNER_Z),
        "inner_guide_cage.obj",
        material=steel_dark,
        name="guide_sleeve",
    )
    for sign, side_name in ((1.0, "left"), (-1.0, "right")):
        for x_pos, end_name in ((-0.058, "rear"), (0.058, "front")):
            inner_guide_cage.visual(
                Box((0.024, INNER_SIDE_PAD_T, 0.011)),
                origin=Origin(
                    xyz=(
                        x_pos,
                        sign * (INNER_Y / 2.0 + GUIDE_SLEEVE_T + INNER_SIDE_PAD_T / 2.0),
                        0.0,
                    )
                ),
                material=bronze,
                name=f"side_pad_{side_name}_{end_name}",
            )
    inner_guide_cage.visual(
        Box((0.038, INNER_Y * 0.55, INNER_TOP_PAD_T)),
        origin=Origin(xyz=(0.0, 0.0, INNER_Z / 2.0 + GUIDE_SLEEVE_T + INNER_TOP_PAD_T / 2.0)),
        material=pad_gray,
        name="top_pad",
    )
    inner_guide_cage.visual(
        Box((0.038, INNER_Y * 0.55, INNER_TOP_PAD_T)),
        origin=Origin(xyz=(0.0, 0.0, -(INNER_Z / 2.0 + GUIDE_SLEEVE_T + INNER_TOP_PAD_T / 2.0))),
        material=pad_gray,
        name="bottom_pad",
    )
    for sign, side_name in ((1.0, "left"), (-1.0, "right")):
        inner_guide_cage.visual(
            Cylinder(radius=0.004, length=0.008),
            origin=Origin(
                xyz=(0.0, sign * (MIDDLE_INNER_Y / 2.0 - 0.004), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc,
            name=f"roller_{side_name}",
        )

    inner_end_bracket = model.part("inner_end_bracket")
    _mesh_visual(
        inner_end_bracket,
        _inner_end_bracket_shape(INNER_Y, INNER_Z),
        "inner_end_bracket.obj",
        material=steel_dark,
        name="bracket_shell",
    )
    inner_end_bracket.visual(
        Box((0.004, INNER_Y + 0.010, 0.004)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=rubber,
        name="wiper_lip",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=support_frame,
        child=outer_member,
        origin=Origin(xyz=(0.0, 0.0, OUTER_CENTER_Z)),
    )
    model.articulation(
        "outer_to_front_bracket",
        ArticulationType.FIXED,
        parent=outer_member,
        child=outer_front_bracket,
        origin=Origin(xyz=(OUTER_LEN / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "outer_to_rear_bracket",
        ArticulationType.FIXED,
        parent=outer_member,
        child=outer_rear_bracket,
        origin=Origin(xyz=(-OUTER_LEN / 2.0, 0.0, 0.0), rpy=(0.0, 0.0, math.pi)),
    )
    model.articulation(
        "outer_to_cover_a",
        ArticulationType.FIXED,
        parent=outer_member,
        child=outer_cover_a,
        origin=Origin(xyz=(-0.085, 0.0, OUTER_Z / 2.0 + 0.002)),
    )
    model.articulation(
        "outer_to_cover_b",
        ArticulationType.FIXED,
        parent=outer_member,
        child=outer_cover_b,
        origin=Origin(xyz=(0.085, 0.0, OUTER_Z / 2.0 + 0.002)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_member,
        child=middle_member,
        origin=Origin(xyz=(MIDDLE_REST_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.40, lower=0.0, upper=MIDDLE_TRAVEL),
    )
    model.articulation(
        "middle_to_guide_cage",
        ArticulationType.FIXED,
        parent=middle_member,
        child=middle_guide_cage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_member,
        child=inner_member,
        origin=Origin(xyz=(INNER_REST_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.45, lower=0.0, upper=INNER_TRAVEL),
    )
    model.articulation(
        "inner_to_guide_cage",
        ArticulationType.FIXED,
        parent=inner_member,
        child=inner_guide_cage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "inner_to_end_bracket",
        ArticulationType.FIXED,
        parent=inner_member,
        child=inner_end_bracket,
        origin=Origin(xyz=(INNER_LEN / 2.0, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support_frame = object_model.get_part("support_frame")
    outer_member = object_model.get_part("outer_member")
    outer_front_bracket = object_model.get_part("outer_front_bracket")
    outer_rear_bracket = object_model.get_part("outer_rear_bracket")
    outer_cover_a = object_model.get_part("outer_cover_a")
    outer_cover_b = object_model.get_part("outer_cover_b")
    middle_member = object_model.get_part("middle_member")
    middle_guide_cage = object_model.get_part("middle_guide_cage")
    inner_member = object_model.get_part("inner_member")
    inner_guide_cage = object_model.get_part("inner_guide_cage")
    inner_end_bracket = object_model.get_part("inner_end_bracket")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    outer_stop_block = outer_member.get_visual("outer_stop_block")
    outer_travel_stop_face = outer_front_bracket.get_visual("travel_stop_face")
    middle_stop_dog = middle_member.get_visual("outer_stop_dog")
    middle_stop_block = middle_member.get_visual("middle_stop_block")
    inner_stop_dog = inner_member.get_visual("middle_stop_dog")
    middle_side_pad_front = middle_guide_cage.get_visual("side_pad_left_front")
    inner_side_pad_front = inner_guide_cage.get_visual("side_pad_left_front")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    expected_parts = (
        support_frame,
        outer_member,
        outer_front_bracket,
        outer_rear_bracket,
        outer_cover_a,
        outer_cover_b,
        middle_member,
        middle_guide_cage,
        inner_member,
        inner_guide_cage,
        inner_end_bracket,
    )
    for part in expected_parts:
        ctx.check(f"part_present:{part.name}", part is not None, f"Missing part {part.name}")

    outer_rest = ctx.part_world_position(outer_member)
    middle_rest = ctx.part_world_position(middle_member)
    inner_rest = ctx.part_world_position(inner_member)
    with ctx.pose({outer_to_middle: 0.180}):
        middle_mid = ctx.part_world_position(middle_member)
    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.140}):
        inner_mid = ctx.part_world_position(inner_member)

    ctx.check(
        "outer_to_middle translates along +x",
        outer_rest is not None
        and middle_rest is not None
        and middle_mid is not None
        and abs((middle_mid[0] - middle_rest[0]) - 0.180) < 1e-6
        and abs(middle_mid[1] - middle_rest[1]) < 1e-6
        and abs(middle_mid[2] - middle_rest[2]) < 1e-6,
        "Middle stage should translate only on x by commanded travel.",
    )
    ctx.check(
        "middle_to_inner translates along +x",
        inner_rest is not None
        and inner_mid is not None
        and abs((inner_mid[0] - inner_rest[0]) - 0.140) < 1e-6
        and abs(inner_mid[1] - inner_rest[1]) < 1e-6
        and abs(inner_mid[2] - inner_rest[2]) < 1e-6,
        "Inner stage should translate only on x by commanded travel.",
    )

    ctx.expect_gap(outer_front_bracket, support_frame, axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_gap(outer_rear_bracket, support_frame, axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_contact(outer_front_bracket, outer_member)
    ctx.expect_contact(outer_rear_bracket, outer_member)
    ctx.expect_gap(outer_cover_a, outer_member, axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_gap(outer_cover_b, outer_member, axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_contact(inner_end_bracket, inner_member)

    ctx.expect_contact(middle_guide_cage, middle_member)
    ctx.expect_contact(inner_guide_cage, inner_member)
    ctx.expect_contact(middle_guide_cage, outer_member, elem_a=middle_side_pad_front)
    ctx.expect_contact(inner_guide_cage, middle_member, elem_a=inner_side_pad_front)

    ctx.expect_within(middle_member, outer_member, axes="yz", margin=0.0005)
    ctx.expect_within(inner_member, middle_member, axes="yz", margin=0.0005)
    ctx.expect_overlap(middle_member, outer_member, axes="x", min_overlap=0.27)
    ctx.expect_overlap(inner_member, middle_member, axes="x", min_overlap=0.16)

    with ctx.pose({outer_to_middle: 0.180, middle_to_inner: 0.090}):
        ctx.expect_within(middle_member, outer_member, axes="yz", margin=0.0005, name="middle_within_outer_yz_mid_stroke")
        ctx.expect_within(inner_member, middle_member, axes="yz", margin=0.0005, name="inner_within_middle_yz_mid_stroke")
        ctx.expect_overlap(middle_member, outer_member, axes="x", min_overlap=0.10, name="middle_overlap_outer_mid_stroke")
        ctx.expect_overlap(inner_member, middle_member, axes="x", min_overlap=0.10, name="inner_overlap_middle_mid_stroke")

    with ctx.pose({outer_to_middle: MIDDLE_TRAVEL, middle_to_inner: INNER_TRAVEL}):
        ctx.expect_overlap(middle_member, outer_member, axes="x", min_overlap=0.075, name="middle_overlap_outer_full_extension")
        ctx.expect_overlap(inner_member, middle_member, axes="x", min_overlap=0.060, name="inner_overlap_middle_full_extension")
        ctx.expect_gap(
            outer_front_bracket,
            middle_member,
            axis="x",
            positive_elem=outer_travel_stop_face,
            negative_elem=middle_stop_dog,
            min_gap=0.0,
            max_gap=0.004,
            name="outer_stage_end_stop_engagement",
        )
        ctx.expect_gap(
            middle_member,
            inner_member,
            axis="x",
            positive_elem=middle_stop_block,
            negative_elem=inner_stop_dog,
            min_gap=0.0,
            max_gap=0.004,
            name="inner_stage_end_stop_engagement",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
