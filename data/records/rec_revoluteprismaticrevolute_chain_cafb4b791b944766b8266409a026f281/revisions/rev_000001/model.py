from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.24
BASE_WIDTH = 0.18
BASE_THICKNESS = 0.018
HINGE_X = -0.072
HINGE_Z = 0.056

PIVOT_BARREL_RADIUS = 0.018
PIVOT_BARREL_LENGTH = 0.068
BASE_CHEEK_THICKNESS = 0.014
BASE_CHEEK_WIDTH = 0.026
BASE_CHEEK_Y = (PIVOT_BARREL_LENGTH / 2.0) + (BASE_CHEEK_THICKNESS / 2.0)
BASE_CHEEK_HEIGHT = 0.066

GUIDE_LENGTH = 0.18
GUIDE_WIDTH = 0.056
GUIDE_HEIGHT = 0.034
GUIDE_CENTER_Z = 0.045
GUIDE_TOP_Z = GUIDE_CENTER_Z + (GUIDE_HEIGHT / 2.0)

SLIDE_ORIGIN_X = 0.0
SLIDE_ORIGIN_Z = GUIDE_TOP_Z
SLIDE_MAX = 0.08
STAGE_BEAM_LENGTH = 0.17
STAGE_WIDTH = 0.034
STAGE_HEIGHT = 0.018
STAGE_Z = 0.0
WRIST_X = 0.17
WRIST_BLOCK_LENGTH = 0.014
WRIST_BLOCK_WIDTH = 0.038
WRIST_BLOCK_HEIGHT = 0.024
WRIST_BLOCK_Z = 0.012

TIP_BODY_LENGTH = 0.042
TIP_BODY_WIDTH = 0.022
TIP_BODY_HEIGHT = 0.018
TIP_NOSE_RADIUS = 0.009


def _box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(xyz)


def _cylinder_y(radius: float, length: float, xyz: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(xyz)
    )


def _make_base_shape() -> cq.Workplane:
    base_plate = _box(
        (BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS),
        (0.0, 0.0, BASE_THICKNESS / 2.0),
    )
    pedestal = _box(
        (0.052, 0.092, 0.022),
        (HINGE_X, 0.0, BASE_THICKNESS + 0.011),
    )
    rear_rib = _box(
        (0.018, 0.092, 0.03),
        (HINGE_X - 0.028, 0.0, BASE_THICKNESS + 0.015),
    )
    left_cheek = _box(
        (BASE_CHEEK_WIDTH, BASE_CHEEK_THICKNESS, BASE_CHEEK_HEIGHT),
        (HINGE_X, BASE_CHEEK_Y, BASE_CHEEK_HEIGHT / 2.0 + BASE_THICKNESS),
    )
    right_cheek = _box(
        (BASE_CHEEK_WIDTH, BASE_CHEEK_THICKNESS, BASE_CHEEK_HEIGHT),
        (HINGE_X, -BASE_CHEEK_Y, BASE_CHEEK_HEIGHT / 2.0 + BASE_THICKNESS),
    )
    left_front_rib = _box(
        (0.018, BASE_CHEEK_THICKNESS, 0.028),
        (HINGE_X + 0.022, BASE_CHEEK_Y, BASE_THICKNESS + 0.014),
    )
    right_front_rib = _box(
        (0.018, BASE_CHEEK_THICKNESS, 0.028),
        (HINGE_X + 0.022, -BASE_CHEEK_Y, BASE_THICKNESS + 0.014),
    )
    return (
        base_plate.union(pedestal)
        .union(rear_rib)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_front_rib)
        .union(right_front_rib)
    )


def _make_pivot_frame_shape() -> cq.Workplane:
    guide_body = _box(
        (GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT),
        (GUIDE_LENGTH / 2.0, 0.0, GUIDE_CENTER_Z),
    )
    rear_mount = _box((0.03, PIVOT_BARREL_LENGTH, 0.018), (0.015, 0.0, 0.037))

    return guide_body.union(rear_mount)


def _make_extension_stage_shape() -> cq.Workplane:
    beam = _box(
        (STAGE_BEAM_LENGTH, STAGE_WIDTH, STAGE_HEIGHT),
        (STAGE_BEAM_LENGTH / 2.0, 0.0, STAGE_HEIGHT / 2.0),
    )
    wrist_block = _box(
        (WRIST_BLOCK_LENGTH, WRIST_BLOCK_WIDTH, WRIST_BLOCK_HEIGHT),
        (
            WRIST_X - (WRIST_BLOCK_LENGTH / 2.0),
            0.0,
            WRIST_BLOCK_Z,
        ),
    )

    return beam.union(wrist_block)


def _make_tip_shape() -> cq.Workplane:
    body = _box(
        (TIP_BODY_LENGTH, TIP_BODY_WIDTH, TIP_BODY_HEIGHT),
        (TIP_BODY_LENGTH / 2.0, 0.0, 0.0),
    )
    nose = _cylinder_y(TIP_NOSE_RADIUS, 0.012, (0.046, 0.0, 0.0))
    underside_relief = _box((0.018, 0.024, 0.008), (0.032, 0.0, -0.009))

    return body.union(nose).cut(underside_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_hinge_slider_wrist_module")

    model.material("base_dark", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("frame_dark", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("stage_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("tip_black", rgba=(0.12, 0.13, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_shell"),
        material="base_dark",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.084)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )

    pivot_frame = model.part("pivot_frame")
    pivot_frame.visual(
        mesh_from_cadquery(_make_pivot_frame_shape(), "pivot_frame_shell"),
        material="frame_dark",
        name="pivot_frame_shell",
    )
    pivot_frame.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH, PIVOT_BARREL_LENGTH, GUIDE_TOP_Z + 0.004)),
        mass=1.6,
        origin=Origin(xyz=(GUIDE_LENGTH / 2.0, 0.0, (GUIDE_TOP_Z + 0.004) / 2.0)),
    )

    extension_stage = model.part("extension_stage")
    extension_stage.visual(
        mesh_from_cadquery(_make_extension_stage_shape(), "extension_stage_shell"),
        material="stage_silver",
        name="extension_stage_shell",
    )
    extension_stage.inertial = Inertial.from_geometry(
        Box((WRIST_X, WRIST_BLOCK_WIDTH, WRIST_BLOCK_HEIGHT)),
        mass=0.85,
        origin=Origin(xyz=(WRIST_X / 2.0, 0.0, WRIST_BLOCK_HEIGHT / 2.0)),
    )

    tip_member = model.part("tip_member")
    tip_member.visual(
        mesh_from_cadquery(_make_tip_shape(), "tip_member_shell"),
        material="tip_black",
        name="tip_shell",
    )
    tip_member.inertial = Inertial.from_geometry(
        Box((0.055, TIP_BODY_WIDTH, TIP_BODY_HEIGHT)),
        mass=0.28,
        origin=Origin(xyz=(0.0275, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_pivot_frame",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pivot_frame,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.1, effort=28.0, velocity=1.2),
    )
    model.articulation(
        "pivot_frame_to_extension_stage",
        ArticulationType.PRISMATIC,
        parent=pivot_frame,
        child=extension_stage,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, SLIDE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_MAX, effort=20.0, velocity=0.18),
    )
    model.articulation(
        "extension_stage_to_tip_member",
        ArticulationType.REVOLUTE,
        parent=extension_stage,
        child=tip_member,
        origin=Origin(xyz=(WRIST_X, 0.0, WRIST_BLOCK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.7, upper=1.0, effort=8.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pivot_frame = object_model.get_part("pivot_frame")
    extension_stage = object_model.get_part("extension_stage")
    tip_member = object_model.get_part("tip_member")
    base_to_pivot = object_model.get_articulation("base_to_pivot_frame")
    frame_to_stage = object_model.get_articulation("pivot_frame_to_extension_stage")
    stage_to_tip = object_model.get_articulation("extension_stage_to_tip_member")

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

    ctx.expect_contact(
        base,
        pivot_frame,
        name="pivot frame is supported by the base hinge cheeks",
    )
    ctx.expect_contact(
        pivot_frame,
        extension_stage,
        name="extension stage rides on the pivot-frame guide",
    )
    ctx.expect_contact(
        extension_stage,
        tip_member,
        name="tip member is carried by the stage clevis",
    )

    ctx.expect_within(
        extension_stage,
        pivot_frame,
        axes="y",
        margin=0.002,
        name="stage stays centered on the pivot-frame rail",
    )
    ctx.expect_gap(
        extension_stage,
        pivot_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="stage sits on the top rail without sinking into it",
    )
    ctx.expect_overlap(
        extension_stage,
        pivot_frame,
        axes="x",
        min_overlap=0.16,
        name="collapsed stage remains deeply inserted in the guide",
    )

    with ctx.pose({frame_to_stage: SLIDE_MAX}):
        ctx.expect_within(
            extension_stage,
            pivot_frame,
            axes="y",
            margin=0.002,
            name="extended stage stays centered on the top rail",
        )
        ctx.expect_gap(
            extension_stage,
            pivot_frame,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="extended stage still rides on the top rail",
        )
        ctx.expect_overlap(
            extension_stage,
            pivot_frame,
            axes="x",
            min_overlap=0.099,
            name="extended stage retains insertion in the guide",
        )

    rest_stage_box = ctx.part_element_world_aabb(extension_stage, elem="extension_stage_shell")
    with ctx.pose({base_to_pivot: 0.8}):
        raised_stage_box = ctx.part_element_world_aabb(extension_stage, elem="extension_stage_shell")
    ctx.check(
        "pivot frame lifts the stage front upward",
        rest_stage_box is not None
        and raised_stage_box is not None
        and raised_stage_box[1][2] > rest_stage_box[1][2] + 0.02,
        details=f"rest={rest_stage_box}, raised={raised_stage_box}",
    )

    with ctx.pose({base_to_pivot: 0.0}):
        collapsed_stage_pos = ctx.part_world_position(extension_stage)
    with ctx.pose({base_to_pivot: 0.0, frame_to_stage: SLIDE_MAX}):
        extended_stage_pos = ctx.part_world_position(extension_stage)
    ctx.check(
        "prismatic stage extends forward",
        collapsed_stage_pos is not None
        and extended_stage_pos is not None
        and extended_stage_pos[0] > collapsed_stage_pos[0] + 0.07,
        details=f"collapsed={collapsed_stage_pos}, extended={extended_stage_pos}",
    )

    rest_tip_box = ctx.part_element_world_aabb(tip_member, elem="tip_shell")
    with ctx.pose({stage_to_tip: 0.8}):
        tipped_tip_box = ctx.part_element_world_aabb(tip_member, elem="tip_shell")
    ctx.check(
        "wrist joint lifts the tip nose",
        rest_tip_box is not None
        and tipped_tip_box is not None
        and tipped_tip_box[1][2] > rest_tip_box[1][2] + 0.012,
        details=f"rest={rest_tip_box}, tipped={tipped_tip_box}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
