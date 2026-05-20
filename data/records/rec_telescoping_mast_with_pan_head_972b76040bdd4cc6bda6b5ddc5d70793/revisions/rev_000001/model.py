from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import ArticulatedObject, ArticulationType, Box, Cylinder, MotionLimits, Origin, TestContext, TestReport


OUTER_STAGE_OUTER = 0.120
OUTER_STAGE_WALL = 0.008
OUTER_STAGE_LENGTH = 0.900

MID_STAGE_OUTER = 0.090
MID_STAGE_WALL = 0.006
MID_STAGE_LENGTH = 0.840

UPPER_STAGE_OUTER = 0.064
UPPER_STAGE_WALL = 0.005
UPPER_STAGE_LENGTH = 0.720

BASE_TO_MID_Z = 0.240
MID_TO_UPPER_Z = 0.140
UPPER_TO_HEAD_Z = UPPER_STAGE_LENGTH

MID_STAGE_TRAVEL = 0.340
UPPER_STAGE_TRAVEL = 0.280

def _box_visual(part, size, xyz, material, name: str) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_square_tube_boxes(part, *, prefix: str, outer: float, wall: float, length: float, z0: float, material) -> None:
    inner = outer - 2.0 * wall
    side_center = outer / 2.0 - wall / 2.0
    zc = z0 + length / 2.0

    _box_visual(part, (wall, outer, length), (side_center, 0.0, zc), material, f"{prefix}_right_wall")
    _box_visual(part, (wall, outer, length), (-side_center, 0.0, zc), material, f"{prefix}_left_wall")
    _box_visual(part, (inner, wall, length), (0.0, side_center, zc), material, f"{prefix}_front_wall")
    _box_visual(part, (inner, wall, length), (0.0, -side_center, zc), material, f"{prefix}_rear_wall")


def _add_square_ring_boxes(part, *, prefix: str, outer: float, inner: float, height: float, z0: float, material) -> None:
    wall = (outer - inner) / 2.0
    center = outer / 2.0 - wall / 2.0
    zc = z0 + height / 2.0

    _box_visual(part, (wall, outer, height), (center, 0.0, zc), material, f"{prefix}_right")
    _box_visual(part, (wall, outer, height), (-center, 0.0, zc), material, f"{prefix}_left")
    _box_visual(part, (inner, wall, height), (0.0, center, zc), material, f"{prefix}_front")
    _box_visual(part, (inner, wall, height), (0.0, -center, zc), material, f"{prefix}_rear")


def _add_guide_pads(
    part,
    *,
    prefix: str,
    outer: float,
    wall: float,
    child_outer: float,
    guide_height: float,
    z0: float,
    material,
    span_ratio: float = 0.62,
) -> None:
    guide_thickness = (outer - 2.0 * wall - child_outer) / 2.0
    pad_span = child_outer * span_ratio
    pad_center = child_outer / 2.0 + guide_thickness / 2.0
    zc = z0 + guide_height / 2.0

    _box_visual(part, (guide_thickness, pad_span, guide_height), (pad_center, 0.0, zc), material, f"{prefix}_right")
    _box_visual(part, (guide_thickness, pad_span, guide_height), (-pad_center, 0.0, zc), material, f"{prefix}_left")
    _box_visual(part, (pad_span, guide_thickness, guide_height), (0.0, pad_center, zc), material, f"{prefix}_front")
    _box_visual(part, (pad_span, guide_thickness, guide_height), (0.0, -pad_center, zc), material, f"{prefix}_rear")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_telescoping_mast")

    dark_base = model.material("dark_base", rgba=(0.19, 0.20, 0.22, 1.0))
    aluminum = model.material("anodized_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    black_head = model.material("head_black", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    _box_visual(base, (0.460, 0.360, 0.020), (0.0, 0.0, 0.010), dark_base, "base_foot")
    _box_visual(base, (0.400, 0.300, 0.120), (0.0, 0.0, 0.080), dark_base, "base_lower_body")
    _box_visual(base, (0.320, 0.220, 0.040), (0.0, 0.0, 0.160), dark_base, "base_top_body")
    _add_square_ring_boxes(base, prefix="base_pedestal", outer=0.180, inner=OUTER_STAGE_OUTER, height=0.030, z0=0.180, material=dark_base)
    _add_square_tube_boxes(
        base,
        prefix="base_outer_tube",
        outer=OUTER_STAGE_OUTER,
        wall=OUTER_STAGE_WALL,
        length=OUTER_STAGE_LENGTH,
        z0=0.180,
        material=dark_base,
    )
    _add_guide_pads(
        base,
        prefix="base_guides",
        outer=OUTER_STAGE_OUTER,
        wall=OUTER_STAGE_WALL,
        child_outer=MID_STAGE_OUTER,
        guide_height=0.120,
        z0=0.180 + OUTER_STAGE_LENGTH - 0.120,
        material=dark_base,
    )
    _add_square_ring_boxes(
        base,
        prefix="base_top_collar",
        outer=0.140,
        inner=OUTER_STAGE_OUTER - 2.0 * OUTER_STAGE_WALL,
        height=0.030,
        z0=0.180 + OUTER_STAGE_LENGTH - 0.030,
        material=dark_base,
    )

    stage_mid = model.part("stage_mid")
    _add_square_tube_boxes(
        stage_mid,
        prefix="stage_mid_tube",
        outer=MID_STAGE_OUTER,
        wall=MID_STAGE_WALL,
        length=MID_STAGE_LENGTH,
        z0=0.0,
        material=aluminum,
    )
    _add_guide_pads(
        stage_mid,
        prefix="stage_mid_guides",
        outer=MID_STAGE_OUTER,
        wall=MID_STAGE_WALL,
        child_outer=UPPER_STAGE_OUTER,
        guide_height=0.120,
        z0=MID_STAGE_LENGTH - 0.120,
        material=aluminum,
    )
    _add_square_ring_boxes(
        stage_mid,
        prefix="stage_mid_top_collar",
        outer=0.098,
        inner=MID_STAGE_OUTER - 2.0 * MID_STAGE_WALL,
        height=0.025,
        z0=MID_STAGE_LENGTH - 0.025,
        material=aluminum,
    )

    stage_upper = model.part("stage_upper")
    _add_square_tube_boxes(
        stage_upper,
        prefix="stage_upper_tube",
        outer=UPPER_STAGE_OUTER,
        wall=UPPER_STAGE_WALL,
        length=UPPER_STAGE_LENGTH,
        z0=0.0,
        material=aluminum,
    )
    _add_square_ring_boxes(
        stage_upper,
        prefix="stage_upper_wear_band",
        outer=0.072,
        inner=UPPER_STAGE_OUTER,
        height=0.020,
        z0=0.060,
        material=aluminum,
    )
    _add_square_ring_boxes(
        stage_upper,
        prefix="stage_upper_head_seat",
        outer=0.078,
        inner=0.050,
        height=0.010,
        z0=UPPER_STAGE_LENGTH - 0.010,
        material=aluminum,
    )

    yaw_head = model.part("yaw_head")
    yaw_head.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black_head,
        name="yaw_mount_flange",
    )
    yaw_head.visual(
        Cylinder(radius=0.046, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0395)),
        material=black_head,
        name="yaw_rotary_body",
    )
    yaw_head.visual(
        Box((0.110, 0.110, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=black_head,
        name="output_plate",
    )
    yaw_head.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=black_head,
        name="output_boss",
    )

    model.articulation(
        "base_to_stage_mid",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_mid,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_MID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.35,
            lower=0.0,
            upper=MID_STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "stage_mid_to_stage_upper",
        ArticulationType.PRISMATIC,
        parent=stage_mid,
        child=stage_upper,
        origin=Origin(xyz=(0.0, 0.0, MID_TO_UPPER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=0.0,
            upper=UPPER_STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "stage_upper_to_yaw_head",
        ArticulationType.REVOLUTE,
        parent=stage_upper,
        child=yaw_head,
        origin=Origin(xyz=(0.0, 0.0, UPPER_TO_HEAD_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage_mid = object_model.get_part("stage_mid")
    stage_upper = object_model.get_part("stage_upper")
    yaw_head = object_model.get_part("yaw_head")

    lift_mid = object_model.get_articulation("base_to_stage_mid")
    lift_upper = object_model.get_articulation("stage_mid_to_stage_upper")
    yaw = object_model.get_articulation("stage_upper_to_yaw_head")

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

    ctx.expect_contact(base, stage_mid, name="mid_stage_supported_in_retracted_pose")
    ctx.expect_contact(stage_mid, stage_upper, name="upper_stage_supported_in_retracted_pose")
    ctx.expect_contact(stage_upper, yaw_head, name="yaw_head_seated_on_upper_stage")

    prismatics_vertical = (
        lift_mid.articulation_type == ArticulationType.PRISMATIC
        and lift_upper.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in lift_mid.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 6) for v in lift_upper.axis) == (0.0, 0.0, 1.0)
    )
    ctx.check(
        "serial_prismatics_are_vertical",
        prismatics_vertical,
        details="Both telescoping stages should translate upward along +Z.",
    )
    ctx.check(
        "yaw_head_revolves_about_vertical",
        yaw.articulation_type == ArticulationType.REVOLUTE and tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details="The rotary head should yaw on a vertical revolute axis.",
    )

    closed_head_z = ctx.part_world_position(yaw_head)[2]
    with ctx.pose({lift_mid: MID_STAGE_TRAVEL, lift_upper: UPPER_STAGE_TRAVEL, yaw: 0.85}):
        extended_head_z = ctx.part_world_position(yaw_head)[2]
        ctx.expect_contact(base, stage_mid, name="mid_stage_remains_guided_when_extended")
        ctx.expect_contact(stage_mid, stage_upper, name="upper_stage_remains_guided_when_extended")
        ctx.expect_contact(stage_upper, yaw_head, name="yaw_head_remains_seated_when_yawed")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_full_extension")

    ctx.check(
        "mast_extension_raises_rotary_head",
        extended_head_z > closed_head_z + MID_STAGE_TRAVEL + UPPER_STAGE_TRAVEL - 0.01,
        details="Fully extending both prismatic stages should lift the yaw head by the combined travel.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
