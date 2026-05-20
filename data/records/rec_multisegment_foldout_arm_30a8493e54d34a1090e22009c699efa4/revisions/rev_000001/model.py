from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_L = 0.22
FOOT_W = 0.12
FOOT_T = 0.012
BASE_JOINT_X = 0.055
JOINT_Z = 0.086

BLOCK_L = 0.024
BLOCK_W = 0.010
BLOCK_H = 0.022

PLATE_T = 0.003
SIDE_CLEAR = 0.001
FORK_OUTER_W = BLOCK_W + 2.0 * (PLATE_T + SIDE_CLEAR)
FORK_REACH_BACK = 0.019
PLATE_H = 0.030
HINGE_BLOCK_L = 0.012
HINGE_BLOCK_H = 0.020

BEAM_W = 0.012
BEAM_H = 0.010

LOWER_LINK_L = 0.160
MID_LINK_L = 0.140
UPPER_LINK_L = 0.120
PLATFORM_REACH = 0.072


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _side_plates(joint_x: float, joint_z: float = 0.0) -> cq.Workplane:
    y_offset = BLOCK_W / 2.0 + SIDE_CLEAR + PLATE_T / 2.0
    plate_center_x = joint_x - FORK_REACH_BACK / 2.0

    left_plate = _box((FORK_REACH_BACK, PLATE_T, PLATE_H), (plate_center_x, y_offset, joint_z))
    right_plate = _box((FORK_REACH_BACK, PLATE_T, PLATE_H), (plate_center_x, -y_offset, joint_z))
    return left_plate.union(right_plate)


def _hinge_support_block(joint_x: float, joint_z: float = 0.0) -> cq.Workplane:
    return _box(
        (HINGE_BLOCK_L, FORK_OUTER_W, HINGE_BLOCK_H),
        (joint_x - HINGE_BLOCK_L / 2.0, 0.0, joint_z),
    )


def _link_body(length: float) -> cq.Workplane:
    proximal_block = _box((BLOCK_L, BEAM_W, BLOCK_H), (BLOCK_L / 2.0, 0.0, 0.0))
    beam_start = BLOCK_L
    beam_end = length - 0.020
    beam_len = beam_end - beam_start
    beam = _box((beam_len, BEAM_W, BEAM_H), ((beam_start + beam_end) / 2.0, 0.0, 0.0))
    shoulder = _box((0.032, BEAM_W, 0.014), (0.030, 0.0, 0.0))
    distal_root = _box((0.020, BEAM_W, 0.014), (length - 0.018, 0.0, 0.0))

    return proximal_block.union(beam).union(shoulder).union(distal_root)


def _platform_bracket_body() -> cq.Workplane:
    proximal_block = _box((BLOCK_L, BEAM_W, BLOCK_H), (BLOCK_L / 2.0, 0.0, 0.0))
    arm = _box((0.040, BEAM_W, 0.008), (0.030, 0.0, 0.003))
    riser = _box((0.010, 0.012, 0.022), (0.052, 0.0, 0.011))
    deck = _box((0.050, 0.040, 0.004), (0.078, 0.0, 0.020))
    lip = _box((0.004, 0.040, 0.016), (0.101, 0.0, 0.026))
    brace = (
        cq.Workplane("XZ")
        .polyline([(0.048, 0.004), (0.084, 0.020), (0.048, 0.020)])
        .close()
        .extrude(0.008, both=True)
    )

    return proximal_block.union(arm).union(riser).union(deck).union(lip).union(brace)


def _platform_pad() -> cq.Workplane:
    return _box((0.044, 0.034, 0.0016), (0.078, 0.0, 0.0228))


def _base_body() -> cq.Workplane:
    foot = _box((FOOT_L, FOOT_W, FOOT_T), (0.0, 0.0, FOOT_T / 2.0))
    heel_mass = _box((0.070, 0.078, 0.018), (-0.045, 0.0, FOOT_T + 0.009))
    column = _box((0.036, 0.028, 0.056), (0.018, 0.0, FOOT_T + 0.028))
    top_mount = _box((0.020, 0.024, 0.016), (BASE_JOINT_X - 0.014, 0.0, JOINT_Z - 0.010))
    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.000, FOOT_T),
                (0.036, FOOT_T),
                (BASE_JOINT_X - 0.004, JOINT_Z - 0.008),
                (0.018, JOINT_Z - 0.008),
            ]
        )
        .close()
        .extrude(0.020, both=True)
    )
    return foot.union(heel_mass).union(column).union(top_mount).union(gusset)


def _base_pad() -> cq.Workplane:
    return _box((0.180, 0.090, 0.002), (-0.020, 0.0, 0.001))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_foldout_arm")

    dark_metal = model.material("dark_metal", rgba=(0.29, 0.31, 0.34, 1.0))
    link_metal = model.material("link_metal", rgba=(0.56, 0.58, 0.62, 1.0))
    clear_plate = model.material("clear_plate", rgba=(0.78, 0.88, 1.0, 0.35))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body(), "base_body"),
        material=dark_metal,
        name="body",
    )
    base.visual(
        mesh_from_cadquery(_hinge_support_block(BASE_JOINT_X, JOINT_Z), "base_hinge_block"),
        material=dark_metal,
        name="joint_block",
    )
    base.visual(
        mesh_from_cadquery(_side_plates(BASE_JOINT_X, JOINT_Z), "base_side_plates"),
        material=clear_plate,
        name="plates",
    )
    base.visual(
        mesh_from_cadquery(_base_pad(), "base_pad"),
        material=rubber,
        name="bench_pad",
    )

    lower_link = model.part("lower_link")
    lower_link.visual(
        mesh_from_cadquery(_link_body(LOWER_LINK_L), "lower_link_body"),
        material=link_metal,
        name="body",
    )
    lower_link.visual(
        mesh_from_cadquery(_hinge_support_block(LOWER_LINK_L), "lower_link_hinge_block"),
        material=dark_metal,
        name="joint_block",
    )
    lower_link.visual(
        mesh_from_cadquery(_side_plates(LOWER_LINK_L), "lower_link_side_plates"),
        material=clear_plate,
        name="plates",
    )

    mid_link = model.part("mid_link")
    mid_link.visual(
        mesh_from_cadquery(_link_body(MID_LINK_L), "mid_link_body"),
        material=link_metal,
        name="body",
    )
    mid_link.visual(
        mesh_from_cadquery(_hinge_support_block(MID_LINK_L), "mid_link_hinge_block"),
        material=dark_metal,
        name="joint_block",
    )
    mid_link.visual(
        mesh_from_cadquery(_side_plates(MID_LINK_L), "mid_link_side_plates"),
        material=clear_plate,
        name="plates",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_link_body(UPPER_LINK_L), "upper_link_body"),
        material=link_metal,
        name="body",
    )
    upper_link.visual(
        mesh_from_cadquery(_hinge_support_block(UPPER_LINK_L), "upper_link_hinge_block"),
        material=dark_metal,
        name="joint_block",
    )
    upper_link.visual(
        mesh_from_cadquery(_side_plates(UPPER_LINK_L), "upper_link_side_plates"),
        material=clear_plate,
        name="plates",
    )

    platform_bracket = model.part("platform_bracket")
    platform_bracket.visual(
        mesh_from_cadquery(_platform_bracket_body(), "platform_bracket_body"),
        material=dark_metal,
        name="body",
    )
    platform_bracket.visual(
        mesh_from_cadquery(_platform_pad(), "platform_pad"),
        material=rubber,
        name="deck_pad",
    )

    limits = MotionLimits(effort=12.0, velocity=1.8, lower=-1.2, upper=1.25)
    axis = (0.0, -1.0, 0.0)

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_link,
        origin=Origin(xyz=(BASE_JOINT_X, 0.0, JOINT_Z)),
        axis=axis,
        motion_limits=limits,
    )
    model.articulation(
        "lower_to_mid",
        ArticulationType.REVOLUTE,
        parent=lower_link,
        child=mid_link,
        origin=Origin(xyz=(LOWER_LINK_L, 0.0, 0.0)),
        axis=axis,
        motion_limits=limits,
    )
    model.articulation(
        "mid_to_upper",
        ArticulationType.REVOLUTE,
        parent=mid_link,
        child=upper_link,
        origin=Origin(xyz=(MID_LINK_L, 0.0, 0.0)),
        axis=axis,
        motion_limits=limits,
    )
    model.articulation(
        "upper_to_platform",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=platform_bracket,
        origin=Origin(xyz=(UPPER_LINK_L, 0.0, 0.0)),
        axis=axis,
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_link = object_model.get_part("lower_link")
    mid_link = object_model.get_part("mid_link")
    upper_link = object_model.get_part("upper_link")
    platform_bracket = object_model.get_part("platform_bracket")

    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_mid = object_model.get_articulation("lower_to_mid")
    mid_to_upper = object_model.get_articulation("mid_to_upper")
    upper_to_platform = object_model.get_articulation("upper_to_platform")
    joints = (base_to_lower, lower_to_mid, mid_to_upper, upper_to_platform)

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

    ctx.expect_contact(base, lower_link, name="base_joint_is_supported")
    ctx.expect_contact(lower_link, mid_link, name="lower_joint_is_supported")
    ctx.expect_contact(mid_link, upper_link, name="mid_joint_is_supported")
    ctx.expect_contact(upper_link, platform_bracket, name="distal_joint_is_supported")

    expected_axis = (0.0, -1.0, 0.0)
    axes_aligned = all(
        tuple(round(value, 6) for value in joint.axis) == expected_axis and abs(joint.origin.xyz[1]) < 1e-9
        for joint in joints
    )
    ctx.check(
        "joint_axes_share_one_motion_plane",
        axes_aligned,
        details="All four revolute joints should use the same Y-axis hinge direction and stay centered on y=0.",
    )

    ctx.expect_origin_gap(
        platform_bracket,
        base,
        axis="x",
        min_gap=0.42,
        name="platform_reaches_forward_in_rest_pose",
    )

    with ctx.pose(
        {
            base_to_lower: 0.85,
            lower_to_mid: 0.55,
            mid_to_upper: 0.35,
            upper_to_platform: 0.20,
        }
    ):
        ctx.expect_origin_gap(
            platform_bracket,
            base,
            axis="z",
            min_gap=0.30,
            name="platform_lifts_when_arm_opens",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
