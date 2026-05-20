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


SUPPORT_WIDTH = 0.76
SUPPORT_THICKNESS = 0.028
SUPPORT_HEIGHT = 0.92

LOWER_SLIDE_Z = -0.22
LOWER_BEAM_WIDTH = 0.62
LOWER_BEAM_DEPTH = 0.032
LOWER_BEAM_HEIGHT = 0.11
LOWER_RAIL_LENGTH = 0.58
LOWER_RAIL_DEPTH = 0.010
LOWER_RAIL_HEIGHT = 0.018
LOWER_RAIL_Z_OFFSET = 0.030

CARRIAGE_WIDTH = 0.18
CARRIAGE_DEPTH = 0.034
CARRIAGE_HEIGHT = 0.15

TOWER_WIDTH = 0.09
TOWER_DEPTH = 0.036
TOWER_HEIGHT = 0.50
TOWER_CENTER_Y = 0.017
TOWER_CENTER_Z = CARRIAGE_HEIGHT / 2.0 + TOWER_HEIGHT / 2.0

VERTICAL_RAIL_WIDTH = 0.012
VERTICAL_RAIL_DEPTH = 0.010
VERTICAL_RAIL_HEIGHT = 0.38
VERTICAL_RAIL_X_OFFSET = 0.026
VERTICAL_RAIL_CENTER_Y = TOWER_CENTER_Y + TOWER_DEPTH / 2.0 + VERTICAL_RAIL_DEPTH / 2.0
VERTICAL_RAIL_CENTER_Z = 0.31

STAGE_SHOE_WIDTH = 0.11
STAGE_SHOE_DEPTH = 0.026
STAGE_SHOE_HEIGHT = 0.16

HORIZONTAL_TRAVEL = 0.18
VERTICAL_TRAVEL = 0.12

SUPPORT_RAIL_FRONT_Y = (
    SUPPORT_THICKNESS / 2.0 + LOWER_BEAM_DEPTH + LOWER_RAIL_DEPTH
)
CARRIAGE_CENTER_Y = SUPPORT_RAIL_FRONT_Y + CARRIAGE_DEPTH / 2.0
STAGE_CENTER_Y = VERTICAL_RAIL_CENTER_Y + VERTICAL_RAIL_DEPTH / 2.0 + STAGE_SHOE_DEPTH / 2.0


def _support_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(SUPPORT_WIDTH, SUPPORT_THICKNESS, SUPPORT_HEIGHT)
        .edges("|Z")
        .fillet(0.009)
        .faces(">Y")
        .workplane()
        .pushPoints(
            [
                (-0.30, 0.33),
                (0.30, 0.33),
                (-0.30, -0.33),
                (0.30, -0.33),
            ]
        )
        .hole(0.016)
    )

    top_cap = (
        cq.Workplane("XY")
        .box(0.68, 0.040, 0.060)
        .translate((0.0, 0.006, 0.39))
        .edges("|X")
        .fillet(0.004)
    )
    center_spine = cq.Workplane("XY").box(0.14, 0.040, 0.42).translate((0.0, 0.006, 0.10))
    lower_beam = (
        cq.Workplane("XY")
        .box(LOWER_BEAM_WIDTH, LOWER_BEAM_DEPTH, LOWER_BEAM_HEIGHT)
        .translate((0.0, SUPPORT_THICKNESS / 2.0 + LOWER_BEAM_DEPTH / 2.0, LOWER_SLIDE_Z))
    )
    lower_rail_a = (
        cq.Workplane("XY")
        .box(LOWER_RAIL_LENGTH, LOWER_RAIL_DEPTH, LOWER_RAIL_HEIGHT)
        .translate(
            (
                0.0,
                SUPPORT_THICKNESS / 2.0 + LOWER_BEAM_DEPTH + LOWER_RAIL_DEPTH / 2.0,
                LOWER_SLIDE_Z - LOWER_RAIL_Z_OFFSET,
            )
        )
    )
    lower_rail_b = (
        cq.Workplane("XY")
        .box(LOWER_RAIL_LENGTH, LOWER_RAIL_DEPTH, LOWER_RAIL_HEIGHT)
        .translate(
            (
                0.0,
                SUPPORT_THICKNESS / 2.0 + LOWER_BEAM_DEPTH + LOWER_RAIL_DEPTH / 2.0,
                LOWER_SLIDE_Z + LOWER_RAIL_Z_OFFSET,
            )
        )
    )

    return (
        plate.union(top_cap)
        .union(center_spine)
        .union(lower_beam)
        .union(lower_rail_a)
        .union(lower_rail_b)
    )


def _lower_carriage_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(CARRIAGE_WIDTH, CARRIAGE_DEPTH, CARRIAGE_HEIGHT)
    rear_relief = (
        cq.Workplane("XY")
        .box(0.14, 0.010, 0.050)
        .translate((0.0, -0.012, 0.0))
    )
    body = body.cut(rear_relief)

    tower = cq.Workplane("XY").box(TOWER_WIDTH, TOWER_DEPTH, TOWER_HEIGHT).translate(
        (0.0, TOWER_CENTER_Y, TOWER_CENTER_Z)
    )
    rail_left = cq.Workplane("XY").box(
        VERTICAL_RAIL_WIDTH, VERTICAL_RAIL_DEPTH, VERTICAL_RAIL_HEIGHT
    ).translate((-VERTICAL_RAIL_X_OFFSET, VERTICAL_RAIL_CENTER_Y, VERTICAL_RAIL_CENTER_Z))
    rail_right = cq.Workplane("XY").box(
        VERTICAL_RAIL_WIDTH, VERTICAL_RAIL_DEPTH, VERTICAL_RAIL_HEIGHT
    ).translate((VERTICAL_RAIL_X_OFFSET, VERTICAL_RAIL_CENTER_Y, VERTICAL_RAIL_CENTER_Z))
    top_bridge = cq.Workplane("XY").box(0.13, 0.028, 0.030).translate((0.0, 0.012, 0.58))

    left_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.055, 0.075),
                (-0.055, 0.24),
                (-0.020, 0.24),
                (-0.020, 0.075),
            ]
        )
        .close()
        .extrude(0.016)
        .translate((0.0, 0.001, 0.0))
    )
    right_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.020, 0.075),
                (0.020, 0.24),
                (0.055, 0.24),
                (0.055, 0.075),
            ]
        )
        .close()
        .extrude(0.016)
        .translate((0.0, 0.001, 0.0))
    )

    return (
        body.union(tower)
        .union(rail_left)
        .union(rail_right)
        .union(top_bridge)
        .union(left_gusset)
        .union(right_gusset)
    )


def _vertical_stage_shape() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(STAGE_SHOE_WIDTH, STAGE_SHOE_DEPTH, STAGE_SHOE_HEIGHT)
    rear_relief = (
        cq.Workplane("XY")
        .box(0.045, 0.008, 0.100)
        .translate((0.0, -0.009, 0.0))
    )
    shoe = shoe.cut(rear_relief)

    stage_plate = cq.Workplane("XY").box(0.075, 0.018, 0.38).translate((0.0, 0.009, 0.24))
    tool_plate = cq.Workplane("XY").box(0.13, 0.022, 0.045).translate((0.0, 0.011, 0.445))
    top_lip = cq.Workplane("XY").box(0.09, 0.014, 0.020).translate((0.0, 0.024, 0.475))

    return shoe.union(stage_plate).union(tool_plate).union(top_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_xz_module")

    support_gray = model.material("support_gray", rgba=(0.27, 0.29, 0.32, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.66, 0.68, 0.71, 1.0))
    stage_silver = model.material("stage_silver", rgba=(0.78, 0.80, 0.83, 1.0))

    side_support = model.part("side_support")
    side_support.visual(
        mesh_from_cadquery(_support_shape(), "side_support"),
        material=support_gray,
        name="support_shell",
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        mesh_from_cadquery(_lower_carriage_shape(), "lower_carriage"),
        material=carriage_gray,
        name="carriage_shell",
    )

    vertical_stage = model.part("vertical_stage")
    vertical_stage.visual(
        mesh_from_cadquery(_vertical_stage_shape(), "vertical_stage"),
        material=stage_silver,
        name="stage_shell",
    )

    model.articulation(
        "support_to_lower_slide",
        ArticulationType.PRISMATIC,
        parent=side_support,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_CENTER_Y, LOWER_SLIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.20,
            lower=-HORIZONTAL_TRAVEL,
            upper=HORIZONTAL_TRAVEL,
        ),
    )

    model.articulation(
        "lower_slide_to_vertical_stage",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=vertical_stage,
        origin=Origin(xyz=(0.0, STAGE_CENTER_Y, VERTICAL_RAIL_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.15,
            lower=-VERTICAL_TRAVEL,
            upper=VERTICAL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_support = object_model.get_part("side_support")
    lower_carriage = object_model.get_part("lower_carriage")
    vertical_stage = object_model.get_part("vertical_stage")
    slide_x = object_model.get_articulation("support_to_lower_slide")
    slide_z = object_model.get_articulation("lower_slide_to_vertical_stage")

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
        "horizontal_prismatic_axis",
        tuple(round(v, 6) for v in slide_x.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide_x.axis}",
    )
    ctx.check(
        "vertical_prismatic_axis",
        tuple(round(v, 6) for v in slide_z.axis) == (0.0, 0.0, 1.0),
        details=f"axis={slide_z.axis}",
    )
    ctx.check(
        "horizontal_prismatic_limits",
        slide_x.motion_limits is not None
        and slide_x.motion_limits.lower == -HORIZONTAL_TRAVEL
        and slide_x.motion_limits.upper == HORIZONTAL_TRAVEL,
        details=f"limits={slide_x.motion_limits}",
    )
    ctx.check(
        "vertical_prismatic_limits",
        slide_z.motion_limits is not None
        and slide_z.motion_limits.lower == -VERTICAL_TRAVEL
        and slide_z.motion_limits.upper == VERTICAL_TRAVEL,
        details=f"limits={slide_z.motion_limits}",
    )

    ctx.expect_contact(
        lower_carriage,
        side_support,
        contact_tol=0.002,
        name="lower_carriage_contacts_support_slide",
    )
    ctx.expect_gap(
        lower_carriage,
        side_support,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        name="lower_carriage_flush_to_support_rails",
    )
    ctx.expect_overlap(
        lower_carriage,
        side_support,
        axes="xz",
        min_overlap=0.12,
        name="lower_carriage_overlaps_support_track",
    )
    ctx.expect_contact(
        vertical_stage,
        lower_carriage,
        contact_tol=0.002,
        name="vertical_stage_contacts_carriage_tower",
    )
    ctx.expect_gap(
        vertical_stage,
        lower_carriage,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        name="vertical_stage_flush_to_guide_rails",
    )
    ctx.expect_overlap(
        vertical_stage,
        lower_carriage,
        axes="xz",
        min_overlap=0.08,
        name="vertical_stage_overlaps_vertical_guide",
    )

    base_carriage_pos = ctx.part_world_position(lower_carriage)
    with ctx.pose({slide_x: 0.16}):
        moved_carriage_pos = ctx.part_world_position(lower_carriage)
        ctx.expect_contact(
            lower_carriage,
            side_support,
            contact_tol=0.002,
            name="lower_carriage_stays_supported_at_positive_x",
        )
    carriage_moves_correctly = (
        base_carriage_pos is not None
        and moved_carriage_pos is not None
        and moved_carriage_pos[0] - base_carriage_pos[0] > 0.15
        and abs(moved_carriage_pos[1] - base_carriage_pos[1]) < 1e-4
        and abs(moved_carriage_pos[2] - base_carriage_pos[2]) < 1e-4
    )
    ctx.check(
        "lower_slide_moves_only_along_x",
        carriage_moves_correctly,
        details=f"base={base_carriage_pos}, moved={moved_carriage_pos}",
    )

    with ctx.pose({slide_x: 0.10, slide_z: 0.0}):
        low_stage_pos = ctx.part_world_position(vertical_stage)
    with ctx.pose({slide_x: 0.10, slide_z: 0.10}):
        high_stage_pos = ctx.part_world_position(vertical_stage)
        ctx.expect_contact(
            vertical_stage,
            lower_carriage,
            contact_tol=0.002,
            name="vertical_stage_stays_supported_at_positive_z",
        )
    stage_moves_correctly = (
        low_stage_pos is not None
        and high_stage_pos is not None
        and high_stage_pos[2] - low_stage_pos[2] > 0.09
        and abs(high_stage_pos[0] - low_stage_pos[0]) < 1e-4
        and abs(high_stage_pos[1] - low_stage_pos[1]) < 1e-4
    )
    ctx.check(
        "vertical_stage_moves_only_along_z",
        stage_moves_correctly,
        details=f"low={low_stage_pos}, high={high_stage_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
