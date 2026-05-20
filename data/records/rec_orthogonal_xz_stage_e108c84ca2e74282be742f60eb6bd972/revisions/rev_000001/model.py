from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.42
BEAM_DEPTH = 0.084
BEAM_HEIGHT = 0.080
FOOT_LENGTH = 0.080
FOOT_DEPTH = 0.064
FOOT_HEIGHT = 0.012
FOOT_OFFSET_X = 0.145
BEAM_CENTER_Y = -(0.010 / 2.0 + BEAM_DEPTH / 2.0)

X_RAIL_LENGTH = 0.355
X_RAIL_WIDTH = 0.010
X_RAIL_HEIGHT = 0.012
X_RAIL_CENTER_Z = 0.022

LOWER_TRAVEL = 0.105

CARRIAGE_LENGTH = 0.125
CARRIAGE_FRONT_DEPTH = 0.050
CARRIAGE_FRONT_Y = 0.032
CARRIAGE_FRONT_HEIGHT = 0.092
CARRIAGE_FRONT_Z = 0.010

BEARING_BLOCK_DEPTH = 0.022
BEARING_BLOCK_Y = 0.011
BEARING_BLOCK_HEIGHT = 0.028

MAST_PEDESTAL_WIDTH = 0.086
MAST_PEDESTAL_DEPTH = 0.030
MAST_PEDESTAL_Y = 0.026
MAST_PEDESTAL_HEIGHT = 0.040
MAST_PEDESTAL_Z = 0.074

BACKPLATE_WIDTH = 0.100
BACKPLATE_DEPTH = 0.012
BACKPLATE_Y = 0.018
BACKPLATE_HEIGHT = 0.120
BACKPLATE_Z = 0.150

UPRIGHT_WIDTH = 0.012
UPRIGHT_DEPTH = 0.016
UPRIGHT_Y = 0.024
UPRIGHT_HEIGHT = 0.126
UPRIGHT_Z = 0.156
UPRIGHT_X = 0.046

GUIDE_ROD_RADIUS = 0.006
GUIDE_ROD_X = 0.026
GUIDE_ROD_Y = 0.040
GUIDE_ROD_LENGTH = 0.185
GUIDE_ROD_CENTER_Z = 0.1485

GUIDE_HEAD_WIDTH = 0.102
GUIDE_HEAD_DEPTH = 0.040
GUIDE_HEAD_HEIGHT = 0.018
GUIDE_HEAD_Y = 0.045
GUIDE_HEAD_Z = 0.241

RAM_ORIGIN_Z = 0.095
UPPER_TRAVEL = 0.112

RAM_LOWER_BLOCK_W = 0.078
RAM_LOWER_BLOCK_D = 0.024
RAM_LOWER_BLOCK_H = 0.038

RAM_UPPER_BLOCK_W = 0.070
RAM_UPPER_BLOCK_D = 0.020
RAM_UPPER_BLOCK_H = 0.028
RAM_UPPER_BLOCK_Z = 0.060

RAM_BRIDGE_W = 0.036
RAM_BRIDGE_D = 0.010
RAM_BRIDGE_H = 0.128
RAM_BRIDGE_Y = 0.006
RAM_BRIDGE_Z = 0.076

RAM_COLUMN_W = 0.028
RAM_COLUMN_D = 0.018
RAM_COLUMN_H = 0.180
RAM_COLUMN_Y = 0.016
RAM_COLUMN_Z = 0.110

PLATFORM_W = 0.090
PLATFORM_D = 0.065
PLATFORM_T = 0.010
PLATFORM_Y = 0.016
PLATFORM_Z = 0.209

PLATFORM_BOSS_W = 0.048
PLATFORM_BOSS_D = 0.032
PLATFORM_BOSS_H = 0.014
PLATFORM_BOSS_Z = 0.197


def _beam_body_shape() -> cq.Workplane:
    beam = cq.Workplane("XY").box(BASE_LENGTH, BEAM_DEPTH, BEAM_HEIGHT)
    beam = beam.cut(
        cq.Workplane("XY")
        .box(BASE_LENGTH * 0.72, 0.010, 0.042)
        .translate((0.0, (BEAM_DEPTH / 2.0) - 0.010, 0.0))
    )
    beam = beam.cut(
        cq.Workplane("XY")
        .box(BASE_LENGTH * 0.54, BEAM_DEPTH * 0.32, 0.008)
        .translate((0.0, 0.0, (BEAM_HEIGHT / 2.0) - 0.004))
    )
    for x_pos in (-FOOT_OFFSET_X, FOOT_OFFSET_X):
        beam = beam.union(
            cq.Workplane("XY")
            .box(FOOT_LENGTH, FOOT_DEPTH, FOOT_HEIGHT)
            .translate((x_pos, 0.0, -(BEAM_HEIGHT / 2.0) - (FOOT_HEIGHT / 2.0)))
        )
    return beam


def _x_rail_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT)
    rail = (
        rail.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .rarray(X_RAIL_LENGTH / 4.8, 1.0, 4, 1)
        .circle(0.003)
        .cutBlind(-0.003)
    )
    return rail


def _carriage_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_FRONT_DEPTH,
        CARRIAGE_FRONT_HEIGHT,
    ).translate((0.0, CARRIAGE_FRONT_Y, CARRIAGE_FRONT_Z))

    body = body.union(
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, BEARING_BLOCK_DEPTH, BEARING_BLOCK_HEIGHT)
        .translate((0.0, BEARING_BLOCK_Y, X_RAIL_CENTER_Z))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, BEARING_BLOCK_DEPTH, BEARING_BLOCK_HEIGHT)
        .translate((0.0, BEARING_BLOCK_Y, -X_RAIL_CENTER_Z))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(MAST_PEDESTAL_WIDTH, MAST_PEDESTAL_DEPTH, MAST_PEDESTAL_HEIGHT)
        .translate((0.0, MAST_PEDESTAL_Y, MAST_PEDESTAL_Z))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(BACKPLATE_WIDTH, BACKPLATE_DEPTH, BACKPLATE_HEIGHT)
        .translate((0.0, BACKPLATE_Y, BACKPLATE_Z))
    )
    for x_pos in (-UPRIGHT_X, UPRIGHT_X):
        body = body.union(
            cq.Workplane("XY")
            .box(UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT)
            .translate((x_pos, UPRIGHT_Y, UPRIGHT_Z))
        )

    for z_pos in (X_RAIL_CENTER_Z, -X_RAIL_CENTER_Z):
        body = body.cut(
            cq.Workplane("XY")
            .box(CARRIAGE_LENGTH * 0.90, 0.016, X_RAIL_HEIGHT)
            .translate((0.0, -0.003, z_pos))
        )

    body = body.cut(
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH * 0.76, 0.020, 0.024)
        .translate((0.0, 0.006, 0.0))
    )
    for x_pos in (-0.028, 0.028):
        body = body.cut(
            cq.Workplane("XY")
            .box(0.028, 0.014, 0.050)
            .translate((x_pos, 0.050, 0.002))
        )
    body = body.cut(
        cq.Workplane("XY")
        .box(0.082, 0.040, 0.162)
        .translate((0.0, 0.037, 0.128))
    )

    return body


def _guide_head_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(GUIDE_HEAD_WIDTH, GUIDE_HEAD_DEPTH, GUIDE_HEAD_HEIGHT)
    cap = cap.cut(
        cq.Workplane("XY")
        .box(0.042, 0.034, GUIDE_HEAD_HEIGHT + 0.004)
        .translate((0.0, 0.010, 0.0))
    )
    cap = cap.cut(
        cq.Workplane("XY")
        .pushPoints([(-GUIDE_ROD_X, -0.005), (GUIDE_ROD_X, -0.005)])
        .circle(GUIDE_ROD_RADIUS)
        .extrude(GUIDE_HEAD_HEIGHT + 0.004)
        .translate((0.0, 0.0, -(GUIDE_HEAD_HEIGHT + 0.004) / 2.0))
    )
    return cap


def _ram_carriage_shape() -> cq.Workplane:
    ram = cq.Workplane("XY").box(
        RAM_LOWER_BLOCK_W,
        RAM_LOWER_BLOCK_D,
        RAM_LOWER_BLOCK_H,
    ).translate((0.0, 0.0, RAM_LOWER_BLOCK_H / 2.0))
    ram = ram.union(
        cq.Workplane("XY")
        .box(RAM_UPPER_BLOCK_W, RAM_UPPER_BLOCK_D, RAM_UPPER_BLOCK_H)
        .translate((0.0, 0.0, RAM_UPPER_BLOCK_Z))
    )
    ram = ram.union(
        cq.Workplane("XY")
        .box(RAM_BRIDGE_W, RAM_BRIDGE_D, RAM_BRIDGE_H)
        .translate((0.0, RAM_BRIDGE_Y, RAM_BRIDGE_Z))
    )
    ram = ram.union(
        cq.Workplane("XY")
        .box(RAM_COLUMN_W, RAM_COLUMN_D, RAM_COLUMN_H)
        .translate((0.0, RAM_COLUMN_Y, RAM_COLUMN_Z))
    )
    for x_pos in (-0.018, 0.018):
        ram = ram.union(
            cq.Workplane("XY")
            .box(0.010, 0.014, 0.102)
            .translate((x_pos, 0.012, 0.050))
        )
    ram = ram.cut(
        cq.Workplane("XY")
        .pushPoints([(-GUIDE_ROD_X, 0.0), (GUIDE_ROD_X, 0.0)])
        .circle(GUIDE_ROD_RADIUS + 0.0012)
        .extrude(0.240)
        .translate((0.0, 0.0, -0.010))
    )
    ram = ram.cut(
        cq.Workplane("XY")
        .box(0.048, 0.010, 0.018)
        .translate((0.0, 0.010, 0.019))
    )
    return ram


def _platform_shape() -> cq.Workplane:
    platform = cq.Workplane("XY").box(PLATFORM_W, PLATFORM_D, PLATFORM_T)
    platform = platform.union(
        cq.Workplane("XY")
        .box(PLATFORM_BOSS_W, PLATFORM_BOSS_D, PLATFORM_BOSS_H)
        .translate((0.0, 0.0, -(PLATFORM_T / 2.0) - (PLATFORM_BOSS_H / 2.0)))
    )
    platform = platform.cut(
        cq.Workplane("XY")
        .box(0.050, 0.020, 0.010)
        .translate((0.0, 0.0, -(PLATFORM_T / 2.0)))
    )
    return platform


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_fed_lift_stage")

    model.material("painted_base", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("rail_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("machined_carriage", rgba=(0.63, 0.66, 0.71, 1.0))
    model.material("guide_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    model.material("ram_gray", rgba=(0.54, 0.57, 0.61, 1.0))
    model.material("platform_aluminum", rgba=(0.80, 0.82, 0.85, 1.0))

    base_beam = model.part("base_beam")
    base_beam.visual(
        mesh_from_cadquery(_beam_body_shape().translate((0.0, BEAM_CENTER_Y, 0.0)), "base_beam_body"),
        material="painted_base",
        name="beam_body",
    )
    base_beam.visual(
        mesh_from_cadquery(_x_rail_shape(), "upper_x_rail"),
        origin=Origin(xyz=(0.0, 0.0, X_RAIL_CENTER_Z)),
        material="rail_steel",
        name="upper_x_rail",
    )
    base_beam.visual(
        mesh_from_cadquery(_x_rail_shape(), "lower_x_rail"),
        origin=Origin(xyz=(0.0, 0.0, -X_RAIL_CENTER_Z)),
        material="rail_steel",
        name="lower_x_rail",
    )
    base_beam.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BEAM_DEPTH, BEAM_HEIGHT + FOOT_HEIGHT)),
        mass=9.0,
        origin=Origin(xyz=(0.0, BEAM_CENTER_Y, -(FOOT_HEIGHT / 2.0))),
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        Box((CARRIAGE_LENGTH, 0.022, 0.028)),
        origin=Origin(xyz=(0.0, 0.011, X_RAIL_CENTER_Z)),
        material="machined_carriage",
        name="upper_bearing",
    )
    lower_carriage.visual(
        Box((CARRIAGE_LENGTH, 0.022, 0.028)),
        origin=Origin(xyz=(0.0, 0.011, -X_RAIL_CENTER_Z)),
        material="machined_carriage",
        name="lower_bearing",
    )
    lower_carriage.visual(
        Box((CARRIAGE_LENGTH, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, 0.032, 0.015)),
        material="machined_carriage",
        name="bridge_web",
    )
    lower_carriage.visual(
        Box((0.086, 0.030, 0.100)),
        origin=Origin(xyz=(0.0, 0.026, 0.080)),
        material="machined_carriage",
        name="mast_base",
    )
    lower_carriage.visual(
        Box((0.100, 0.012, 0.110)),
        origin=Origin(xyz=(0.0, 0.018, 0.150)),
        material="machined_carriage",
        name="backplate",
    )
    lower_carriage.visual(
        Box((0.012, 0.016, 0.120)),
        origin=Origin(xyz=(-0.046, 0.024, 0.176)),
        material="machined_carriage",
        name="left_upright",
    )
    lower_carriage.visual(
        Box((0.012, 0.016, 0.120)),
        origin=Origin(xyz=(0.046, 0.024, 0.176)),
        material="machined_carriage",
        name="right_upright",
    )
    lower_carriage.visual(
        Box((0.010, 0.012, 0.176)),
        origin=Origin(xyz=(-0.026, 0.030, 0.154)),
        material="guide_steel",
        name="left_guide_rail",
    )
    lower_carriage.visual(
        Box((0.010, 0.012, 0.176)),
        origin=Origin(xyz=(0.026, 0.030, 0.154)),
        material="guide_steel",
        name="right_guide_rail",
    )
    lower_carriage.visual(
        Box((0.028, 0.040, 0.018)),
        origin=Origin(xyz=(-0.033, 0.045, 0.240)),
        material="machined_carriage",
        name="left_guide_cap",
    )
    lower_carriage.visual(
        Box((0.028, 0.040, 0.018)),
        origin=Origin(xyz=(0.033, 0.045, 0.240)),
        material="machined_carriage",
        name="right_guide_cap",
    )
    lower_carriage.visual(
        Box((0.098, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.031, 0.240)),
        material="machined_carriage",
        name="rear_head_bridge",
    )
    lower_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, 0.060, 0.260)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.030, 0.090)),
    )

    ram_stage = model.part("ram_stage")
    ram_stage.visual(
        Box((0.018, 0.020, 0.040)),
        origin=Origin(xyz=(-0.026, 0.006, 0.002)),
        material="ram_gray",
        name="left_bearing_cap",
    )
    ram_stage.visual(
        Box((0.018, 0.020, 0.040)),
        origin=Origin(xyz=(0.026, 0.006, 0.002)),
        material="ram_gray",
        name="right_bearing_cap",
    )
    ram_stage.visual(
        Box((0.070, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.014, 0.004)),
        material="ram_gray",
        name="guide_bridge",
    )
    ram_stage.visual(
        Box((0.034, 0.020, 0.104)),
        origin=Origin(xyz=(0.0, 0.028, 0.058)),
        material="ram_gray",
        name="front_stand",
    )
    ram_stage.visual(
        Box((0.026, 0.016, 0.170)),
        origin=Origin(xyz=(0.0, 0.038, 0.128)),
        material="ram_gray",
        name="ram_column",
    )
    ram_stage.visual(
        Box((PLATFORM_W, PLATFORM_D, PLATFORM_T)),
        origin=Origin(xyz=(0.0, 0.038, 0.212)),
        material="platform_aluminum",
        name="platform_plate",
    )
    ram_stage.visual(
        Box((PLATFORM_BOSS_W, PLATFORM_BOSS_D, PLATFORM_BOSS_H)),
        origin=Origin(xyz=(0.0, 0.038, 0.200)),
        material="platform_aluminum",
        name="platform_boss",
    )
    ram_stage.inertial = Inertial.from_geometry(
        Box((0.090, 0.070, 0.230)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.018, 0.105)),
    )

    model.articulation(
        "base_to_lower_carriage",
        ArticulationType.PRISMATIC,
        parent=base_beam,
        child=lower_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-LOWER_TRAVEL,
            upper=LOWER_TRAVEL,
            effort=1800.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "lower_carriage_to_ram",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=ram_stage,
        origin=Origin(xyz=(0.0, GUIDE_ROD_Y, RAM_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=UPPER_TRAVEL,
            effort=900.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_beam = object_model.get_part("base_beam")
    lower_carriage = object_model.get_part("lower_carriage")
    ram_stage = object_model.get_part("ram_stage")
    horizontal_slide = object_model.get_articulation("base_to_lower_carriage")
    vertical_slide = object_model.get_articulation("lower_carriage_to_ram")

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

    ctx.check(
        "horizontal_slide_axis_is_x",
        tuple(horizontal_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected horizontal axis (1,0,0), got {horizontal_slide.axis}",
    )
    ctx.check(
        "vertical_slide_axis_is_z",
        tuple(vertical_slide.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical axis (0,0,1), got {vertical_slide.axis}",
    )

    ctx.expect_contact(
        lower_carriage,
        base_beam,
        elem_b="upper_x_rail",
        elem_a="upper_bearing",
        name="carriage_bears_on_upper_x_rail",
    )
    ctx.expect_contact(
        lower_carriage,
        base_beam,
        elem_b="lower_x_rail",
        elem_a="lower_bearing",
        name="carriage_bears_on_lower_x_rail",
    )
    ctx.expect_contact(
        ram_stage,
        lower_carriage,
        elem_a="left_bearing_cap",
        elem_b="left_guide_rail",
        name="ram_engages_left_vertical_guide",
    )
    ctx.expect_contact(
        ram_stage,
        lower_carriage,
        elem_a="right_bearing_cap",
        elem_b="right_guide_rail",
        name="ram_engages_right_vertical_guide",
    )
    ctx.expect_gap(
        ram_stage,
        base_beam,
        axis="z",
        positive_elem="platform_boss",
        min_gap=0.20,
        name="platform_sits_clear_above_base_beam",
    )
    ctx.expect_gap(
        ram_stage,
        lower_carriage,
        axis="z",
        positive_elem="platform_boss",
        negative_elem="rear_head_bridge",
        min_gap=0.034,
        name="platform_clears_fixed_guide_head",
    )

    with ctx.pose({horizontal_slide: LOWER_TRAVEL}):
        ctx.expect_overlap(
            lower_carriage,
            base_beam,
            axes="x",
            elem_a="upper_bearing",
            elem_b="upper_x_rail",
            min_overlap=0.110,
            name="carriage_stays_supported_on_upper_rail_at_positive_limit",
        )

    with ctx.pose({horizontal_slide: -LOWER_TRAVEL}):
        ctx.expect_overlap(
            lower_carriage,
            base_beam,
            axes="x",
            elem_a="lower_bearing",
            elem_b="lower_x_rail",
            min_overlap=0.110,
            name="carriage_stays_supported_on_lower_rail_at_negative_limit",
        )

    with ctx.pose({vertical_slide: UPPER_TRAVEL}):
        ctx.expect_contact(
            ram_stage,
            lower_carriage,
            elem_a="left_bearing_cap",
            elem_b="left_guide_rail",
            name="ram_remains_on_left_guide_at_full_raise",
        )
        ctx.expect_contact(
            ram_stage,
            lower_carriage,
            elem_a="right_bearing_cap",
            elem_b="right_guide_rail",
            name="ram_remains_on_right_guide_at_full_raise",
        )
        ctx.expect_gap(
            ram_stage,
            lower_carriage,
            axis="z",
            positive_elem="platform_boss",
            negative_elem="rear_head_bridge",
            min_gap=0.145,
            name="platform_rises_clear_of_guide_head",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
