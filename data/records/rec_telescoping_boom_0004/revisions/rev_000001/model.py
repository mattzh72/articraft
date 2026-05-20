from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
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

BOOM_AXIS = (1.0, 0.0, 0.0)

HOUSING_LEN = 0.40
HOUSING_WIDTH = 0.090
HOUSING_HEIGHT = 0.110
HOUSING_WALL = 0.006
HOUSING_REAR_BULKHEAD = 0.030

STAGE1_LEN = 0.50
STAGE1_WIDTH = 0.074
STAGE1_HEIGHT = 0.094
STAGE1_WALL = 0.005

STAGE2_LEN = 0.48
STAGE2_WIDTH = 0.058
STAGE2_HEIGHT = 0.078
STAGE2_WALL = 0.0045
STAGE2_CAP_THICK = 0.010

SLIDE_REST_OFFSET = 0.030
STAGE_TRAVEL = 0.250

HEAD_BASE_LEN = 0.018
HEAD_REACH = 0.060
HEAD_WIDTH = 0.050
HEAD_HEIGHT = 0.040
HEAD_ARM_THICK = 0.012
HEAD_PIN_RADIUS = 0.0045

HOUSING_INNER_WIDTH = HOUSING_WIDTH - 2.0 * HOUSING_WALL
HOUSING_INNER_HEIGHT = HOUSING_HEIGHT - 2.0 * HOUSING_WALL
STAGE1_INNER_WIDTH = STAGE1_WIDTH - 2.0 * STAGE1_WALL
STAGE1_INNER_HEIGHT = STAGE1_HEIGHT - 2.0 * STAGE1_WALL

STAGE1_GUIDE_Y = (HOUSING_INNER_WIDTH - STAGE1_WIDTH) / 2.0
STAGE1_GUIDE_Z = (HOUSING_INNER_HEIGHT - STAGE1_HEIGHT) / 2.0
STAGE2_GUIDE_Y = (STAGE1_INNER_WIDTH - STAGE2_WIDTH) / 2.0
STAGE2_GUIDE_Z = (STAGE1_INNER_HEIGHT - STAGE2_HEIGHT) / 2.0

GUIDE_PAD_LENGTH = 0.090
GUIDE_PAD_WIDTH = 0.030
GUIDE_PAD_HEIGHT = 0.028


def _box_from_zero(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate((length / 2.0, 0.0, 0.0))


def _tube_shell(length: float, width: float, height: float, wall: float) -> cq.Workplane:
    outer = _box_from_zero(length, width, height)
    inner = _box_from_zero(length + 0.004, width - 2.0 * wall, height - 2.0 * wall).translate(
        (-0.002, 0.0, 0.0)
    )
    return outer.cut(inner)


def _housing_sleeve_shape() -> cq.Workplane:
    outer = _box_from_zero(HOUSING_LEN, HOUSING_WIDTH, HOUSING_HEIGHT)
    inner_length = HOUSING_LEN - HOUSING_REAR_BULKHEAD + 0.002
    inner = (
        cq.Workplane("XY")
        .box(
            inner_length,
            HOUSING_WIDTH - 2.0 * HOUSING_WALL,
            HOUSING_HEIGHT - 2.0 * HOUSING_WALL,
        )
        .translate((HOUSING_REAR_BULKHEAD + inner_length / 2.0 - 0.001, 0.0, 0.0))
    )
    nose_gusset = _box_from_zero(0.020, HOUSING_WIDTH * 0.70, HOUSING_HEIGHT * 0.72).translate(
        (HOUSING_LEN - 0.020, 0.0, 0.0)
    )
    return outer.cut(inner).union(nose_gusset)


def _head_bracket_shape() -> cq.Workplane:
    base = _box_from_zero(HEAD_BASE_LEN, HEAD_WIDTH, HEAD_HEIGHT)

    arm_length = HEAD_REACH - HEAD_BASE_LEN
    left_arm = _box_from_zero(arm_length, HEAD_ARM_THICK, HEAD_HEIGHT).translate(
        (HEAD_BASE_LEN, -(HEAD_WIDTH - HEAD_ARM_THICK) / 2.0, 0.0)
    )
    right_arm = _box_from_zero(arm_length, HEAD_ARM_THICK, HEAD_HEIGHT).translate(
        (HEAD_BASE_LEN, (HEAD_WIDTH - HEAD_ARM_THICK) / 2.0, 0.0)
    )

    bracket = base.union(left_arm).union(right_arm)
    pin = (
        cq.Workplane("YZ")
        .circle(HEAD_PIN_RADIUS)
        .extrude(HEAD_WIDTH + 0.004)
        .translate((HEAD_BASE_LEN + arm_length * 0.70, 0.0, 0.0))
        .translate((0.0, -(HEAD_WIDTH + 0.004) / 2.0, 0.0))
    )
    return bracket.cut(pin)


def _add_open_rect_tube(
    part,
    *,
    prefix: str,
    start_x: float,
    length: float,
    width: float,
    height: float,
    wall: float,
    material,
) -> None:
    part.visual(
        Box((length, wall, height)),
        origin=Origin(xyz=(start_x + length / 2.0, (width - wall) / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_left_wall",
    )
    part.visual(
        Box((length, wall, height)),
        origin=Origin(xyz=(start_x + length / 2.0, -(width - wall) / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_right_wall",
    )
    part.visual(
        Box((length, width - 2.0 * wall, wall)),
        origin=Origin(xyz=(start_x + length / 2.0, 0.0, (height - wall) / 2.0)),
        material=material,
        name=f"{prefix}_top_wall",
    )
    part.visual(
        Box((length, width - 2.0 * wall, wall)),
        origin=Origin(xyz=(start_x + length / 2.0, 0.0, -(height - wall) / 2.0)),
        material=material,
        name=f"{prefix}_bottom_wall",
    )


def _add_guide_pads(
    part,
    *,
    prefix: str,
    x_center: float,
    outer_width: float,
    outer_height: float,
    pad_y: float,
    pad_z: float,
    material,
) -> None:
    part.visual(
        Box((GUIDE_PAD_LENGTH, GUIDE_PAD_WIDTH, pad_z)),
        origin=Origin(xyz=(x_center, 0.0, outer_height / 2.0 + pad_z / 2.0)),
        material=material,
        name=f"{prefix}_top_pad",
    )
    part.visual(
        Box((GUIDE_PAD_LENGTH, GUIDE_PAD_WIDTH, pad_z)),
        origin=Origin(xyz=(x_center, 0.0, -(outer_height / 2.0 + pad_z / 2.0))),
        material=material,
        name=f"{prefix}_bottom_pad",
    )
    part.visual(
        Box((GUIDE_PAD_LENGTH, pad_y, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(x_center, outer_width / 2.0 + pad_y / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_left_pad",
    )
    part.visual(
        Box((GUIDE_PAD_LENGTH, pad_y, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(x_center, -(outer_width / 2.0 + pad_y / 2.0), 0.0)),
        material=material,
        name=f"{prefix}_right_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_boom", assets=ASSETS)

    charcoal = model.material("charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.61, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.40, 0.43, 0.46, 1.0))
    yellow = model.material("boom_yellow", rgba=(0.88, 0.72, 0.12, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_REAR_BULKHEAD, HOUSING_WIDTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=(HOUSING_REAR_BULKHEAD / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="housing_rear_bulkhead",
    )
    _add_open_rect_tube(
        housing,
        prefix="housing_sleeve",
        start_x=HOUSING_REAR_BULKHEAD,
        length=HOUSING_LEN - HOUSING_REAR_BULKHEAD,
        width=HOUSING_WIDTH,
        height=HOUSING_HEIGHT,
        wall=HOUSING_WALL,
        material=charcoal,
    )
    housing.visual(
        Box((0.090, 0.120, 0.020)),
        origin=Origin(xyz=(0.045, 0.0, -(HOUSING_HEIGHT + 0.020) / 2.0)),
        material=dark_steel,
        name="housing_mount",
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_LEN, HOUSING_WIDTH, HOUSING_HEIGHT)),
        mass=7.0,
        origin=Origin(xyz=(HOUSING_LEN / 2.0, 0.0, 0.0)),
    )

    mid_stage = model.part("mid_stage")
    _add_open_rect_tube(
        mid_stage,
        prefix="stage1_tube",
        start_x=0.0,
        length=STAGE1_LEN,
        width=STAGE1_WIDTH,
        height=STAGE1_HEIGHT,
        wall=STAGE1_WALL,
        material=yellow,
    )
    _add_guide_pads(
        mid_stage,
        prefix="stage1_guides",
        x_center=0.085,
        outer_width=STAGE1_WIDTH,
        outer_height=STAGE1_HEIGHT,
        pad_y=STAGE1_GUIDE_Y,
        pad_z=STAGE1_GUIDE_Z,
        material=dark_steel,
    )
    mid_stage.visual(
        Box((0.012, STAGE1_WIDTH, STAGE1_HEIGHT)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=dark_steel,
        name="stage1_rear_stop",
    )
    mid_stage.visual(
        Box((0.012, STAGE1_WIDTH, STAGE1_HEIGHT)),
        origin=Origin(xyz=(STAGE1_LEN - 0.006, 0.0, 0.0)),
        material=dark_steel,
        name="stage1_front_collar",
    )
    mid_stage.inertial = Inertial.from_geometry(
        Box((STAGE1_LEN, STAGE1_WIDTH, STAGE1_HEIGHT)),
        mass=3.0,
        origin=Origin(xyz=(STAGE1_LEN / 2.0, 0.0, 0.0)),
    )

    inner_stage = model.part("inner_stage")
    _add_open_rect_tube(
        inner_stage,
        prefix="stage2_tube",
        start_x=0.0,
        length=STAGE2_LEN,
        width=STAGE2_WIDTH,
        height=STAGE2_HEIGHT,
        wall=STAGE2_WALL,
        material=steel,
    )
    _add_guide_pads(
        inner_stage,
        prefix="stage2_guides",
        x_center=0.080,
        outer_width=STAGE2_WIDTH,
        outer_height=STAGE2_HEIGHT,
        pad_y=STAGE2_GUIDE_Y,
        pad_z=STAGE2_GUIDE_Z,
        material=dark_steel,
    )
    inner_stage.visual(
        Box((STAGE2_CAP_THICK, STAGE2_WIDTH, STAGE2_HEIGHT)),
        origin=Origin(xyz=(STAGE2_LEN - STAGE2_CAP_THICK / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="stage2_front_cap",
    )
    inner_stage.inertial = Inertial.from_geometry(
        Box((STAGE2_LEN, STAGE2_WIDTH, STAGE2_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(STAGE2_LEN / 2.0, 0.0, 0.0)),
    )
    head_bracket = model.part("head_bracket")
    head_bracket.visual(
        mesh_from_cadquery(_head_bracket_shape(), "head_bracket.obj", assets=ASSETS),
        material=dark_steel,
        name="head_bracket_shell",
    )
    head_bracket.inertial = Inertial.from_geometry(
        Box((HEAD_REACH, HEAD_WIDTH, HEAD_HEIGHT)),
        mass=0.7,
        origin=Origin(xyz=(HEAD_REACH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_mid_stage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=mid_stage,
        origin=Origin(xyz=(SLIDE_REST_OFFSET, 0.0, 0.0)),
        axis=BOOM_AXIS,
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=0.20,
            lower=0.0,
            upper=STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "mid_stage_to_inner_stage",
        ArticulationType.PRISMATIC,
        parent=mid_stage,
        child=inner_stage,
        origin=Origin(xyz=(SLIDE_REST_OFFSET, 0.0, 0.0)),
        axis=BOOM_AXIS,
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.22,
            lower=0.0,
            upper=STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "inner_stage_to_head_bracket",
        ArticulationType.FIXED,
        parent=inner_stage,
        child=head_bracket,
        origin=Origin(xyz=(STAGE2_LEN, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    mid_stage = object_model.get_part("mid_stage")
    inner_stage = object_model.get_part("inner_stage")
    head_bracket = object_model.get_part("head_bracket")

    housing_to_mid = object_model.get_articulation("housing_to_mid_stage")
    mid_to_inner = object_model.get_articulation("mid_stage_to_inner_stage")

    stage2_front_cap = inner_stage.get_visual("stage2_front_cap")
    head_shell = head_bracket.get_visual("head_bracket_shell")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_origin_distance(mid_stage, housing, axes="yz", max_dist=1e-6)
    ctx.expect_origin_distance(inner_stage, mid_stage, axes="yz", max_dist=1e-6)
    ctx.expect_contact(mid_stage, housing, name="mid_stage_guides_contact_housing")
    ctx.expect_contact(inner_stage, mid_stage, name="inner_stage_guides_contact_mid_stage")
    ctx.expect_within(
        mid_stage,
        housing,
        axes="yz",
        margin=0.0,
        name="mid_stage_centered_in_housing",
    )
    ctx.expect_within(
        inner_stage,
        mid_stage,
        axes="yz",
        margin=0.0,
        name="inner_stage_centered_in_mid_stage",
    )
    ctx.expect_overlap(
        mid_stage,
        housing,
        axes="x",
        min_overlap=0.37,
        name="mid_stage_rest_overlap",
    )
    ctx.expect_overlap(
        inner_stage,
        mid_stage,
        axes="x",
        min_overlap=0.45,
        name="inner_stage_rest_overlap",
    )
    ctx.expect_contact(
        head_bracket,
        inner_stage,
        elem_a=head_shell,
        elem_b=stage2_front_cap,
        name="head_bracket_seated_on_inner_stage_cap",
    )

    mid_rest_x = ctx.part_world_position(mid_stage)[0]
    inner_rest_x = ctx.part_world_position(inner_stage)[0]
    head_rest_x = ctx.part_world_position(head_bracket)[0]

    with ctx.pose({housing_to_mid: STAGE_TRAVEL}):
        ctx.expect_overlap(
            mid_stage,
            housing,
            axes="x",
            min_overlap=0.12,
            name="mid_stage_overlap_at_full_extension",
        )
        mid_extended_x = ctx.part_world_position(mid_stage)[0]
        ctx.check(
            "mid_stage_prismatic_travel",
            abs((mid_extended_x - mid_rest_x) - STAGE_TRAVEL) <= 1e-6,
            details=f"expected {STAGE_TRAVEL:.3f} m travel, got {mid_extended_x - mid_rest_x:.6f} m",
        )

    with ctx.pose({mid_to_inner: STAGE_TRAVEL}):
        ctx.expect_overlap(
            inner_stage,
            mid_stage,
            axes="x",
            min_overlap=0.215,
            name="inner_stage_overlap_at_full_extension",
        )
        inner_extended_x = ctx.part_world_position(inner_stage)[0]
        ctx.check(
            "inner_stage_prismatic_travel",
            abs((inner_extended_x - inner_rest_x) - STAGE_TRAVEL) <= 1e-6,
            details=f"expected {STAGE_TRAVEL:.3f} m travel, got {inner_extended_x - inner_rest_x:.6f} m",
        )

    with ctx.pose({housing_to_mid: STAGE_TRAVEL, mid_to_inner: STAGE_TRAVEL}):
        combined_head_x = ctx.part_world_position(head_bracket)[0]
        ctx.check(
            "head_bracket_combined_extension",
            abs((combined_head_x - head_rest_x) - 2.0 * STAGE_TRAVEL) <= 1e-6,
            details=(
                f"expected head bracket to advance {2.0 * STAGE_TRAVEL:.3f} m, "
                f"got {combined_head_x - head_rest_x:.6f} m"
            ),
        )
        ctx.expect_contact(
            head_bracket,
            inner_stage,
            elem_a=head_shell,
            elem_b=stage2_front_cap,
            name="head_bracket_remains_seated_when_fully_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
