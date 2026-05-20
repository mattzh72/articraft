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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_PLATE_LENGTH = 0.118
BASE_PLATE_WIDTH = 0.094
BASE_THICKNESS = 0.016
LUG_LENGTH = 0.026
LUG_WIDTH = 0.022
LUG_HOLE_RADIUS = 0.0042

LANE_OFFSET_Y = 0.032
ROOT_AXIS_Z = 0.040

BARREL_RADIUS = 0.0055
BARREL_LENGTH = 0.014
CHEEK_THICKNESS = 0.004
OUTER_KNUCKLE_WIDTH = BARREL_LENGTH + 2.0 * CHEEK_THICKNESS

BODY_WIDTH = 0.014
BODY_HEIGHT = 0.018
SHIELD_THICKNESS = 0.0018

BARREL_CLEARANCE = 0.0006
BEARING_THICKNESS = 0.002
BEARING_RADIUS = 0.0075
TONGUE_LENGTH = 0.010
TONGUE_WIDTH = 0.006
TONGUE_HEIGHT = 0.010

PROXIMAL_JOINT_X = 0.050
MIDDLE_JOINT_X = 0.040
DISTAL_LENGTH = 0.038


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cylinder_y(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _combine(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS, tolerance=0.0008, angular_tolerance=0.08)


def _make_base_plate() -> cq.Workplane:
    plate = _cq_box(
        (BASE_PLATE_LENGTH, BASE_PLATE_WIDTH, BASE_THICKNESS),
        (0.0, 0.0, -BASE_THICKNESS / 2.0),
    )

    lug_x = BASE_PLATE_LENGTH / 2.0 - 0.020
    lug_y = BASE_PLATE_WIDTH / 2.0 + 0.011
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            lug_center = (x_sign * lug_x, y_sign * lug_y, -BASE_THICKNESS / 2.0)
            plate = plate.union(_cq_box((LUG_LENGTH, LUG_WIDTH, BASE_THICKNESS), lug_center))
            plate = plate.cut(
                cq.Workplane("XY")
                .cylinder(BASE_THICKNESS + 0.006, LUG_HOLE_RADIUS)
                .translate((lug_center[0], lug_center[1], -BASE_THICKNESS - 0.003))
            )

    plate = plate.cut(
        _cq_box(
            (0.056, 0.044, 0.004),
            (0.0, 0.0, -0.002),
        )
    )
    plate = plate.cut(
        _cq_box(
            (0.038, 0.018, 0.004),
            (-0.023, 0.0, -0.002),
        )
    )
    return plate


def _make_root_mount(lane_y: float) -> cq.Workplane:
    base_block = _cq_box((0.020, OUTER_KNUCKLE_WIDTH + 0.014, 0.020), (-0.004, lane_y, 0.010))
    rear_riser = _cq_box((0.010, 0.018, 0.022), (-0.007, lane_y, 0.028))
    top_bridge = _cq_box((0.012, OUTER_KNUCKLE_WIDTH, 0.004), (0.000, lane_y, ROOT_AXIS_Z + 0.010))
    cheek_left = _cq_box(
        (0.010, CHEEK_THICKNESS, 0.022),
        (0.000, lane_y + (BARREL_LENGTH + CHEEK_THICKNESS) / 2.0, ROOT_AXIS_Z),
    )
    cheek_right = _cq_box(
        (0.010, CHEEK_THICKNESS, 0.022),
        (0.000, lane_y - (BARREL_LENGTH + CHEEK_THICKNESS) / 2.0, ROOT_AXIS_Z),
    )
    mount = _combine(base_block, rear_riser, top_bridge, cheek_left, cheek_right)
    mount = mount.cut(
        _cq_cylinder_y(
            BARREL_RADIUS + BARREL_CLEARANCE,
            OUTER_KNUCKLE_WIDTH + 0.010,
            (0.000, lane_y, ROOT_AXIS_Z),
        )
    )
    return mount


def _make_bearing_pair(axis_x: float = 0.0, axis_z: float = 0.0) -> cq.Workplane:
    y_offset = BARREL_LENGTH / 2.0 + BEARING_THICKNESS / 2.0
    left = _cq_cylinder_y(BEARING_RADIUS, BEARING_THICKNESS, (axis_x, y_offset, axis_z))
    right = _cq_cylinder_y(BEARING_RADIUS, BEARING_THICKNESS, (axis_x, -y_offset, axis_z))
    return _combine(left, right)


def _make_barrel_with_tongue() -> cq.Workplane:
    barrel = _cq_cylinder_y(BARREL_RADIUS, BARREL_LENGTH, (0.0, 0.0, 0.0))
    tongue = _cq_box((TONGUE_LENGTH, TONGUE_WIDTH, TONGUE_HEIGHT), (0.0055 + TONGUE_LENGTH / 2.0, 0.0, 0.0))
    return _combine(barrel, tongue)


def _make_intermediate_frame(joint_x: float) -> cq.Workplane:
    beam_start_x = 0.0055 + TONGUE_LENGTH
    beam_end_x = joint_x - 0.011
    beam_length = beam_end_x - beam_start_x
    beam = _cq_box((beam_length, BODY_WIDTH, BODY_HEIGHT * 0.84), ((beam_start_x + beam_end_x) / 2.0, 0.0, 0.0))
    lower_beam = _cq_box((beam_length - 0.004, BODY_WIDTH * 0.72, 0.006), ((beam_start_x + beam_end_x) / 2.0 + 0.001, 0.0, -0.006))
    fork_block = _cq_box((0.012, OUTER_KNUCKLE_WIDTH, BODY_HEIGHT), (joint_x - 0.006, 0.0, 0.0))
    top_tie = _cq_box((0.008, OUTER_KNUCKLE_WIDTH, 0.004), (joint_x - 0.009, 0.0, 0.007))
    bottom_tie = _cq_box((0.008, OUTER_KNUCKLE_WIDTH, 0.004), (joint_x - 0.009, 0.0, -0.007))
    frame = _combine(beam, lower_beam, fork_block, top_tie, bottom_tie)
    frame = frame.cut(
        _cq_cylinder_y(
            BARREL_RADIUS + BARREL_CLEARANCE,
            OUTER_KNUCKLE_WIDTH + 0.010,
            (joint_x, 0.0, 0.0),
        )
    )
    return frame


def _make_distal_frame() -> cq.Workplane:
    beam_start_x = 0.0055 + TONGUE_LENGTH
    main_beam = _cq_box((0.022, BODY_WIDTH, BODY_HEIGHT * 0.88), (beam_start_x + 0.011, 0.0, 0.0))
    lower_rib = _cq_box((0.018, BODY_WIDTH * 0.72, 0.006), (beam_start_x + 0.012, 0.0, -0.006))
    nose = _cq_box((0.010, BODY_WIDTH * 0.9, BODY_HEIGHT * 0.76), (0.034, 0.0, -0.001))
    top_guard = _cq_box((0.018, BODY_WIDTH + 0.004, 0.003), (0.027, 0.0, 0.007))
    return _combine(main_beam, lower_rib, nose, top_guard)


def _author_intermediate_link(
    part,
    mesh_prefix: str,
    joint_x: float,
    structural_material: str,
    oxide_material: str,
    cover_material: str,
) -> None:
    part.visual(
        _mesh(_make_barrel_with_tongue(), f"{mesh_prefix}_rear_barrel.obj"),
        material=oxide_material,
        name="rear_barrel",
    )
    part.visual(
        _mesh(_make_intermediate_frame(joint_x), f"{mesh_prefix}_frame.obj"),
        material=structural_material,
        name="frame",
    )
    part.visual(
        _mesh(_make_bearing_pair(axis_x=joint_x, axis_z=0.0), f"{mesh_prefix}_front_bearings.obj"),
        material=oxide_material,
        name="front_bearings",
    )

    shield_length = joint_x - 0.030
    shield_center_x = 0.5 * (0.010 + (joint_x - 0.020))
    for name_suffix, y_sign in (("left", 1.0), ("right", -1.0)):
        part.visual(
            Box((shield_length, SHIELD_THICKNESS, 0.013)),
            origin=Origin(xyz=(shield_center_x, y_sign * (BODY_WIDTH / 2.0 + SHIELD_THICKNESS / 2.0), 0.0015)),
            material=cover_material,
            name=f"shield_{name_suffix}",
        )
        part.visual(
            Box((0.012, 0.0022, 0.013)),
            origin=Origin(
                xyz=(
                    joint_x - 0.002,
                    y_sign * (OUTER_KNUCKLE_WIDTH / 2.0 + 0.0011),
                    0.0005,
                )
            ),
            material=cover_material,
            name=f"access_cover_{name_suffix}",
        )

    part.inertial = Inertial.from_geometry(
        Box((joint_x + 0.004, OUTER_KNUCKLE_WIDTH, BODY_HEIGHT + 0.006)),
        mass=0.18 if joint_x > 0.045 else 0.14,
        origin=Origin(xyz=(joint_x / 2.0, 0.0, 0.0)),
    )


def _author_distal_link(
    part,
    mesh_prefix: str,
    structural_material: str,
    oxide_material: str,
    cover_material: str,
    pad_material: str,
) -> None:
    part.visual(
        _mesh(_make_barrel_with_tongue(), f"{mesh_prefix}_rear_barrel.obj"),
        material=oxide_material,
        name="rear_barrel",
    )
    part.visual(
        _mesh(_make_distal_frame(), f"{mesh_prefix}_frame.obj"),
        material=structural_material,
        name="frame",
    )

    for name_suffix, y_sign in (("left", 1.0), ("right", -1.0)):
        part.visual(
            Box((0.017, SHIELD_THICKNESS, 0.012)),
            origin=Origin(xyz=(0.019, y_sign * (BODY_WIDTH / 2.0 + SHIELD_THICKNESS / 2.0), 0.0015)),
            material=cover_material,
            name=f"shield_{name_suffix}",
        )

    pad_specs = (
        ("pad_root", 0.015),
        ("pad_mid", 0.024),
        ("pad_tip", 0.033),
    )
    for pad_name, x_center in pad_specs:
        part.visual(
            Box((0.007, 0.010, 0.004)),
            origin=Origin(xyz=(x_center, 0.0, -(BODY_HEIGHT / 2.0 + 0.0005))),
            material=pad_material,
            name=pad_name,
        )

    part.inertial = Inertial.from_geometry(
        Box((DISTAL_LENGTH, OUTER_KNUCKLE_WIDTH, BODY_HEIGHT + 0.008)),
        mass=0.11,
        origin=Origin(xyz=(DISTAL_LENGTH / 2.0, 0.0, -0.001)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_independent_finger_chains", assets=ASSETS)

    base_material = model.material("base_steel", rgba=(0.28, 0.31, 0.34, 1.0))
    arm_material = model.material("arm_steel", rgba=(0.56, 0.59, 0.62, 1.0))
    oxide_material = model.material("oxide_hardware", rgba=(0.14, 0.15, 0.16, 1.0))
    cover_material = model.material("cover_plate", rgba=(0.42, 0.44, 0.47, 1.0))
    pad_material = model.material("pad_block", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(_mesh(_make_base_plate(), "base_plate.obj"), material=base_material, name="base_plate")
    base.visual(
        Box((0.020, 0.060, 0.020)),
        origin=Origin(xyz=(-0.020, 0.000, 0.010)),
        material=base_material,
        name="backbone_block",
    )
    base.visual(
        Box((0.050, 0.008, 0.006)),
        origin=Origin(xyz=(0.026, 0.018, 0.003)),
        material=cover_material,
        name="left_guide_rail",
    )
    base.visual(
        Box((0.050, 0.008, 0.006)),
        origin=Origin(xyz=(0.026, -0.018, 0.003)),
        material=cover_material,
        name="right_guide_rail",
    )
    base.visual(
        _mesh(_make_root_mount(LANE_OFFSET_Y), "left_root_mount.obj"),
        material=base_material,
        name="left_knuckle_mount",
    )
    base.visual(
        _mesh(_make_bearing_pair(axis_x=0.0, axis_z=ROOT_AXIS_Z), "left_root_bearings.obj"),
        material=oxide_material,
        name="left_root_bearings",
    )
    base.visual(
        _mesh(_make_root_mount(-LANE_OFFSET_Y), "right_root_mount.obj"),
        material=base_material,
        name="right_knuckle_mount",
    )
    base.visual(
        _mesh(_make_bearing_pair(axis_x=0.0, axis_z=ROOT_AXIS_Z).translate((0.0, -2.0 * LANE_OFFSET_Y, 0.0)), "right_root_bearings.obj"),
        material=oxide_material,
        name="right_root_bearings",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.126, 0.132, 0.060)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    left_proximal = model.part("left_proximal")
    _author_intermediate_link(
        left_proximal,
        "left_proximal",
        PROXIMAL_JOINT_X,
        arm_material,
        oxide_material,
        cover_material,
    )

    left_middle = model.part("left_middle")
    _author_intermediate_link(
        left_middle,
        "left_middle",
        MIDDLE_JOINT_X,
        arm_material,
        oxide_material,
        cover_material,
    )

    left_distal = model.part("left_distal")
    _author_distal_link(
        left_distal,
        "left_distal",
        arm_material,
        oxide_material,
        cover_material,
        pad_material,
    )

    right_proximal = model.part("right_proximal")
    _author_intermediate_link(
        right_proximal,
        "right_proximal",
        PROXIMAL_JOINT_X,
        arm_material,
        oxide_material,
        cover_material,
    )

    right_middle = model.part("right_middle")
    _author_intermediate_link(
        right_middle,
        "right_middle",
        MIDDLE_JOINT_X,
        arm_material,
        oxide_material,
        cover_material,
    )

    right_distal = model.part("right_distal")
    _author_distal_link(
        right_distal,
        "right_distal",
        arm_material,
        oxide_material,
        cover_material,
        pad_material,
    )

    model.articulation(
        "base_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_proximal,
        origin=Origin(xyz=(0.0, LANE_OFFSET_Y, ROOT_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.5, lower=-0.10, upper=1.10),
    )
    model.articulation(
        "left_proximal_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(PROXIMAL_JOINT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=-0.05, upper=1.25),
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(MIDDLE_JOINT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=3.2, lower=-0.05, upper=1.25),
    )

    model.articulation(
        "base_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_proximal,
        origin=Origin(xyz=(0.0, -LANE_OFFSET_Y, ROOT_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.5, lower=-0.10, upper=1.10),
    )
    model.articulation(
        "right_proximal_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(PROXIMAL_JOINT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=-0.05, upper=1.25),
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(MIDDLE_JOINT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=3.2, lower=-0.05, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    base_to_left_proximal = object_model.get_articulation("base_to_left_proximal")
    left_proximal_to_left_middle = object_model.get_articulation("left_proximal_to_left_middle")
    left_middle_to_left_distal = object_model.get_articulation("left_middle_to_left_distal")
    base_to_right_proximal = object_model.get_articulation("base_to_right_proximal")
    right_proximal_to_right_middle = object_model.get_articulation("right_proximal_to_right_middle")
    right_middle_to_right_distal = object_model.get_articulation("right_middle_to_right_distal")

    left_root_mount = base.get_visual("left_knuckle_mount")
    right_root_mount = base.get_visual("right_knuckle_mount")
    left_proximal_barrel = left_proximal.get_visual("rear_barrel")
    left_middle_barrel = left_middle.get_visual("rear_barrel")
    left_distal_barrel = left_distal.get_visual("rear_barrel")
    right_proximal_barrel = right_proximal.get_visual("rear_barrel")
    right_middle_barrel = right_middle.get_visual("rear_barrel")
    right_distal_barrel = right_distal.get_visual("rear_barrel")
    left_proximal_frame = left_proximal.get_visual("frame")
    left_middle_frame = left_middle.get_visual("frame")
    right_proximal_frame = right_proximal.get_visual("frame")
    right_middle_frame = right_middle.get_visual("frame")

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

    ctx.expect_contact(
        left_proximal,
        base,
        elem_a=left_proximal_barrel,
        elem_b=left_root_mount,
        name="left proximal barrel seats in left base clevis",
    )
    ctx.expect_overlap(
        left_proximal,
        base,
        axes="xz",
        min_overlap=0.009,
        elem_a=left_proximal_barrel,
        elem_b=left_root_mount,
        name="left root barrel remains centered inside left base mount",
    )
    ctx.expect_contact(
        left_middle,
        left_proximal,
        elem_a=left_middle_barrel,
        elem_b=left_proximal_frame,
        name="left middle barrel seats in proximal clevis",
    )
    ctx.expect_overlap(
        left_middle,
        left_proximal,
        axes="xz",
        min_overlap=0.009,
        elem_a=left_middle_barrel,
        elem_b=left_proximal_frame,
        name="left middle barrel stays captured by proximal fork",
    )
    ctx.expect_contact(
        left_distal,
        left_middle,
        elem_a=left_distal_barrel,
        elem_b=left_middle_frame,
        name="left distal barrel seats in middle clevis",
    )
    ctx.expect_overlap(
        left_distal,
        left_middle,
        axes="xz",
        min_overlap=0.009,
        elem_a=left_distal_barrel,
        elem_b=left_middle_frame,
        name="left distal barrel stays captured by middle fork",
    )

    ctx.expect_contact(
        right_proximal,
        base,
        elem_a=right_proximal_barrel,
        elem_b=right_root_mount,
        name="right proximal barrel seats in right base clevis",
    )
    ctx.expect_overlap(
        right_proximal,
        base,
        axes="xz",
        min_overlap=0.009,
        elem_a=right_proximal_barrel,
        elem_b=right_root_mount,
        name="right root barrel remains centered inside right base mount",
    )
    ctx.expect_contact(
        right_middle,
        right_proximal,
        elem_a=right_middle_barrel,
        elem_b=right_proximal_frame,
        name="right middle barrel seats in proximal clevis",
    )
    ctx.expect_overlap(
        right_middle,
        right_proximal,
        axes="xz",
        min_overlap=0.009,
        elem_a=right_middle_barrel,
        elem_b=right_proximal_frame,
        name="right middle barrel stays captured by proximal fork",
    )
    ctx.expect_contact(
        right_distal,
        right_middle,
        elem_a=right_distal_barrel,
        elem_b=right_middle_frame,
        name="right distal barrel seats in middle clevis",
    )
    ctx.expect_overlap(
        right_distal,
        right_middle,
        axes="xz",
        min_overlap=0.009,
        elem_a=right_distal_barrel,
        elem_b=right_middle_frame,
        name="right distal barrel stays captured by middle fork",
    )

    ctx.expect_origin_distance(
        left_proximal,
        right_proximal,
        axes="y",
        min_dist=0.062,
        max_dist=0.066,
        name="root lanes are laid out side by side",
    )
    ctx.expect_gap(
        left_proximal,
        right_proximal,
        axis="y",
        min_gap=0.035,
        max_penetration=0.0,
        name="proximal links keep lateral clearance",
    )
    ctx.expect_gap(
        left_distal,
        right_distal,
        axis="y",
        min_gap=0.035,
        max_penetration=0.0,
        name="distal links keep lateral clearance",
    )

    base_aabb = ctx.part_world_aabb(base)
    left_tip_aabb = ctx.part_world_aabb(left_distal)
    right_tip_aabb = ctx.part_world_aabb(right_distal)
    if base_aabb is not None:
        base_dims = tuple(base_aabb[1][i] - base_aabb[0][i] for i in range(3))
        ctx.check(
            "base reads as a palm-like support fixture",
            0.115 <= base_dims[0] <= 0.140 and 0.120 <= base_dims[1] <= 0.145 and 0.050 <= base_dims[2] <= 0.070,
            f"base_dims={base_dims}",
        )
    else:
        ctx.fail("base reads as a palm-like support fixture", "Base AABB unavailable")

    if left_tip_aabb is not None and right_tip_aabb is not None:
        ctx.check(
            "finger chains reach forward beyond the base",
            left_tip_aabb[1][0] > 0.120 and right_tip_aabb[1][0] > 0.120,
            f"left_tip_max_x={left_tip_aabb[1][0]:.4f}, right_tip_max_x={right_tip_aabb[1][0]:.4f}",
        )
    else:
        ctx.fail("finger chains reach forward beyond the base", "Distal AABB unavailable")

    left_rest_pos = ctx.part_world_position(left_distal)
    right_rest_pos = ctx.part_world_position(right_distal)
    if left_rest_pos is not None and right_rest_pos is not None:
        with ctx.pose(
            {
                base_to_left_proximal: 0.38,
                left_proximal_to_left_middle: 0.58,
                left_middle_to_left_distal: 0.42,
            }
        ):
            left_flex_pos = ctx.part_world_position(left_distal)
            right_static_pos = ctx.part_world_position(right_distal)
            ctx.expect_contact(
                left_proximal,
                base,
                elem_a=left_proximal_barrel,
                elem_b=left_root_mount,
                name="left root hinge contact persists in flexed pose",
            )
            ctx.expect_contact(
                left_middle,
                left_proximal,
                elem_a=left_middle_barrel,
                elem_b=left_proximal_frame,
                name="left middle hinge contact persists in flexed pose",
            )
            ctx.expect_contact(
                left_distal,
                left_middle,
                elem_a=left_distal_barrel,
                elem_b=left_middle_frame,
                name="left distal hinge contact persists in flexed pose",
            )
            if left_flex_pos is not None:
                ctx.check(
                    "left chain curls downward when actuated",
                    left_flex_pos[2] < left_rest_pos[2] - 0.012,
                    f"rest_z={left_rest_pos[2]:.4f}, flex_z={left_flex_pos[2]:.4f}",
                )
            else:
                ctx.fail("left chain curls downward when actuated", "Left flexed position unavailable")
            if right_static_pos is not None:
                right_delta = max(abs(right_static_pos[i] - right_rest_pos[i]) for i in range(3))
                ctx.check(
                    "right chain stays independent while left chain moves",
                    right_delta <= 1e-6,
                    f"right_delta={right_delta:.6e}",
                )
            else:
                ctx.fail("right chain stays independent while left chain moves", "Right posed position unavailable")

        with ctx.pose(
            {
                base_to_left_proximal: 0.24,
                left_proximal_to_left_middle: 0.34,
                left_middle_to_left_distal: 0.22,
                base_to_right_proximal: 0.18,
                right_proximal_to_right_middle: 0.48,
                right_middle_to_right_distal: 0.30,
            }
        ):
            ctx.expect_gap(
                left_distal,
                right_distal,
                axis="y",
                min_gap=0.035,
                max_penetration=0.0,
                name="dual chains keep side clearance in asymmetric operating pose",
            )
            ctx.expect_contact(
                right_distal,
                right_middle,
                elem_a=right_distal_barrel,
                elem_b=right_middle_frame,
                name="right distal hinge contact persists in asymmetric pose",
            )
    else:
        ctx.fail("left chain curls downward when actuated", "Rest distal positions unavailable")
        ctx.fail("right chain stays independent while left chain moves", "Rest distal positions unavailable")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
