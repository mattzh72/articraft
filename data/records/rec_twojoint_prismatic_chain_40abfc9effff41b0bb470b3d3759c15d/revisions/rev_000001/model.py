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


BASE_L = 0.48
BASE_W = 0.20
BASE_PLATE_T = 0.016
LOWER_GUIDE_L = 0.38
LOWER_GUIDE_W = 0.08
LOWER_GUIDE_H = 0.020
LOWER_RAIL_L = 0.34
LOWER_RAIL_W = 0.020
LOWER_RAIL_H = 0.014
LOWER_RAIL_OFFSET_Y = 0.062
LOWER_RAIL_TOP_Z = BASE_PLATE_T + LOWER_RAIL_H
LOWER_STAGE_Z = LOWER_RAIL_TOP_Z

LOWER_CARRIAGE_L = 0.22
LOWER_CARRIAGE_W = 0.19
LOWER_BEAM_W = 0.036
LOWER_BEAM_H = 0.034
LOWER_BEAM_OFFSET_Y = LOWER_RAIL_OFFSET_Y
LOWER_CROSS_T = 0.014
LOWER_COVER_T = 0.008
LOWER_END_CAP_L = 0.028
LOWER_END_CAP_H = 0.010

UPPER_GUIDE_X = 0.10
UPPER_GUIDE_Y = 0.17
UPPER_GUIDE_H = 0.008
UPPER_RAIL_X = 0.018
UPPER_RAIL_Y = 0.15
UPPER_RAIL_H = 0.012
UPPER_RAIL_OFFSET_X = 0.066
UPPER_STAGE_Z = LOWER_BEAM_H + LOWER_CROSS_T + UPPER_RAIL_H

UPPER_CARRIAGE_X = 0.18
UPPER_CARRIAGE_Y = 0.17
UPPER_RUNNER_X = 0.026
UPPER_RUNNER_H = 0.018
UPPER_RUNNER_OFFSET_X = UPPER_RAIL_OFFSET_X
UPPER_BRIDGE_H = 0.016
SERVICE_PAD_X = 0.15
SERVICE_PAD_Y = 0.12
SERVICE_PAD_H = 0.010
TOP_LOCATOR_R = 0.022
TOP_LOCATOR_H = 0.006
TOP_TOTAL_TOP_Z = UPPER_RUNNER_H + UPPER_BRIDGE_H + SERVICE_PAD_H + TOP_LOCATOR_H

LOWER_STAGE_TRAVEL = 0.090
UPPER_STAGE_TRAVEL = 0.055


def _base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_PLATE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )
    guide = (
        cq.Workplane("XY")
        .box(LOWER_GUIDE_L, LOWER_GUIDE_W, LOWER_GUIDE_H, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_PLATE_T))
    )
    rail_left = (
        cq.Workplane("XY")
        .box(LOWER_RAIL_L, LOWER_RAIL_W, LOWER_RAIL_H, centered=(True, True, False))
        .translate((0.0, -LOWER_RAIL_OFFSET_Y, BASE_PLATE_T))
    )
    rail_right = (
        cq.Workplane("XY")
        .box(LOWER_RAIL_L, LOWER_RAIL_W, LOWER_RAIL_H, centered=(True, True, False))
        .translate((0.0, LOWER_RAIL_OFFSET_Y, BASE_PLATE_T))
    )
    service_pads = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.17, -0.076),
                (-0.17, 0.076),
                (0.17, -0.076),
                (0.17, 0.076),
            ]
        )
        .rect(0.040, 0.024)
        .extrude(0.008)
    )
    mounting_ribs = (
        cq.Workplane("XY")
        .pushPoints([(-0.12, 0.0), (0.12, 0.0)])
        .rect(0.028, 0.14)
        .extrude(0.010)
        .translate((0.0, 0.0, BASE_PLATE_T))
    )
    return (
        plate.union(guide)
        .union(rail_left)
        .union(rail_right)
        .union(service_pads)
        .union(mounting_ribs)
    )


def _mid_stage_shape() -> cq.Workplane:
    beam_left = (
        cq.Workplane("XY")
        .box(LOWER_CARRIAGE_L, LOWER_BEAM_W, LOWER_BEAM_H, centered=(True, True, False))
        .translate((0.0, -LOWER_BEAM_OFFSET_Y, 0.0))
    )
    beam_right = (
        cq.Workplane("XY")
        .box(LOWER_CARRIAGE_L, LOWER_BEAM_W, LOWER_BEAM_H, centered=(True, True, False))
        .translate((0.0, LOWER_BEAM_OFFSET_Y, 0.0))
    )
    cross_plate = (
        cq.Workplane("XY")
        .box(LOWER_CARRIAGE_L, LOWER_CARRIAGE_W, LOWER_CROSS_T, centered=(True, True, False))
        .translate((0.0, 0.0, LOWER_BEAM_H))
    )
    end_caps = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-(LOWER_CARRIAGE_L / 2.0) + (LOWER_END_CAP_L / 2.0), 0.0),
                ((LOWER_CARRIAGE_L / 2.0) - (LOWER_END_CAP_L / 2.0), 0.0),
            ]
        )
        .rect(LOWER_END_CAP_L, 0.11)
        .extrude(LOWER_END_CAP_H)
        .translate((0.0, 0.0, LOWER_BEAM_H + LOWER_CROSS_T))
    )
    guide = (
        cq.Workplane("XY")
        .box(UPPER_GUIDE_X, UPPER_GUIDE_Y, UPPER_GUIDE_H, centered=(True, True, False))
        .translate((0.0, 0.0, LOWER_BEAM_H + LOWER_CROSS_T))
    )
    rail_left = (
        cq.Workplane("XY")
        .box(UPPER_RAIL_X, UPPER_RAIL_Y, UPPER_RAIL_H, centered=(True, True, False))
        .translate((-UPPER_RAIL_OFFSET_X, 0.0, LOWER_BEAM_H + LOWER_CROSS_T))
    )
    rail_right = (
        cq.Workplane("XY")
        .box(UPPER_RAIL_X, UPPER_RAIL_Y, UPPER_RAIL_H, centered=(True, True, False))
        .translate((UPPER_RAIL_OFFSET_X, 0.0, LOWER_BEAM_H + LOWER_CROSS_T))
    )
    cover = (
        cq.Workplane("XY")
        .box(0.14, 0.10, LOWER_COVER_T, centered=(True, True, False))
        .translate((0.0, 0.0, LOWER_BEAM_H + LOWER_CROSS_T + UPPER_RAIL_H))
    )
    return (
        beam_left.union(beam_right)
        .union(cross_plate)
        .union(end_caps)
        .union(guide)
        .union(rail_left)
        .union(rail_right)
        .union(cover)
    )


def _top_stage_shape() -> cq.Workplane:
    runner_left = (
        cq.Workplane("XY")
        .box(UPPER_RUNNER_X, UPPER_CARRIAGE_Y, UPPER_RUNNER_H, centered=(True, True, False))
        .translate((-UPPER_RUNNER_OFFSET_X, 0.0, 0.0))
    )
    runner_right = (
        cq.Workplane("XY")
        .box(UPPER_RUNNER_X, UPPER_CARRIAGE_Y, UPPER_RUNNER_H, centered=(True, True, False))
        .translate((UPPER_RUNNER_OFFSET_X, 0.0, 0.0))
    )
    bridge = (
        cq.Workplane("XY")
        .box(UPPER_CARRIAGE_X, UPPER_CARRIAGE_Y, UPPER_BRIDGE_H, centered=(True, True, False))
        .translate((0.0, 0.0, UPPER_RUNNER_H))
    )
    pad = (
        cq.Workplane("XY")
        .box(SERVICE_PAD_X, SERVICE_PAD_Y, SERVICE_PAD_H, centered=(True, True, False))
        .translate((0.0, 0.0, UPPER_RUNNER_H + UPPER_BRIDGE_H))
    )
    locator = (
        cq.Workplane("XY")
        .circle(TOP_LOCATOR_R)
        .extrude(TOP_LOCATOR_H)
        .translate((0.0, 0.0, UPPER_RUNNER_H + UPPER_BRIDGE_H + SERVICE_PAD_H))
    )
    clamp_lugs = (
        cq.Workplane("XY")
        .pushPoints([(-0.050, 0.0), (0.050, 0.0)])
        .rect(0.018, SERVICE_PAD_Y + 0.012)
        .extrude(0.010)
        .translate((0.0, 0.0, UPPER_RUNNER_H + UPPER_BRIDGE_H + 0.002))
    )
    return runner_left.union(runner_right).union(bridge).union(pad).union(locator).union(clamp_lugs)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_service_fixture")

    model.material("fixture_base", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("carriage_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("service_blue", rgba=(0.28, 0.40, 0.62, 1.0))

    base = model.part("base_guide")
    base.visual(
        Box((BASE_L, BASE_W, BASE_PLATE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_T / 2.0)),
        material="fixture_base",
        name="base_plate",
    )
    base.visual(
        Box((LOWER_GUIDE_L, LOWER_GUIDE_W, LOWER_GUIDE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_T + (LOWER_GUIDE_H / 2.0))),
        material="fixture_base",
        name="center_guide",
    )
    base.visual(
        Box((LOWER_RAIL_L, LOWER_RAIL_W, LOWER_RAIL_H)),
        origin=Origin(
            xyz=(0.0, -LOWER_RAIL_OFFSET_Y, BASE_PLATE_T + (LOWER_RAIL_H / 2.0))
        ),
        material="carriage_aluminum",
        name="lower_rail_left",
    )
    base.visual(
        Box((LOWER_RAIL_L, LOWER_RAIL_W, LOWER_RAIL_H)),
        origin=Origin(
            xyz=(0.0, LOWER_RAIL_OFFSET_Y, BASE_PLATE_T + (LOWER_RAIL_H / 2.0))
        ),
        material="carriage_aluminum",
        name="lower_rail_right",
    )
    base.visual(
        Box((0.040, 0.024, 0.008)),
        origin=Origin(xyz=(-0.17, -0.076, 0.004)),
        material="fixture_base",
        name="foot_pad_fl",
    )
    base.visual(
        Box((0.040, 0.024, 0.008)),
        origin=Origin(xyz=(-0.17, 0.076, 0.004)),
        material="fixture_base",
        name="foot_pad_rl",
    )
    base.visual(
        Box((0.040, 0.024, 0.008)),
        origin=Origin(xyz=(0.17, -0.076, 0.004)),
        material="fixture_base",
        name="foot_pad_fr",
    )
    base.visual(
        Box((0.040, 0.024, 0.008)),
        origin=Origin(xyz=(0.17, 0.076, 0.004)),
        material="fixture_base",
        name="foot_pad_rr",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_PLATE_T + LOWER_GUIDE_H)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_PLATE_T + LOWER_GUIDE_H) / 2.0)),
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        Box((LOWER_CARRIAGE_L, LOWER_BEAM_W, LOWER_BEAM_H)),
        origin=Origin(
            xyz=(0.0, -LOWER_BEAM_OFFSET_Y, LOWER_BEAM_H / 2.0)
        ),
        material="carriage_aluminum",
        name="lower_runner_left",
    )
    lower_carriage.visual(
        Box((LOWER_CARRIAGE_L, LOWER_BEAM_W, LOWER_BEAM_H)),
        origin=Origin(
            xyz=(0.0, LOWER_BEAM_OFFSET_Y, LOWER_BEAM_H / 2.0)
        ),
        material="carriage_aluminum",
        name="lower_runner_right",
    )
    lower_carriage.visual(
        Box((LOWER_CARRIAGE_L, 0.11, LOWER_CROSS_T)),
        origin=Origin(
            xyz=(0.0, 0.0, LOWER_BEAM_H + (LOWER_CROSS_T / 2.0))
        ),
        material="carriage_aluminum",
        name="cross_stage",
    )
    lower_carriage.visual(
        Box((UPPER_GUIDE_X, UPPER_GUIDE_Y, UPPER_GUIDE_H)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                LOWER_BEAM_H + LOWER_CROSS_T + (UPPER_GUIDE_H / 2.0),
            )
        ),
        material="fixture_base",
        name="upper_guide_bed",
    )
    lower_carriage.visual(
        Box((UPPER_RAIL_X, UPPER_RAIL_Y, UPPER_RAIL_H)),
        origin=Origin(
            xyz=(
                -UPPER_RAIL_OFFSET_X,
                0.0,
                LOWER_BEAM_H + LOWER_CROSS_T + (UPPER_RAIL_H / 2.0),
            )
        ),
        material="carriage_aluminum",
        name="upper_rail_left",
    )
    lower_carriage.visual(
        Box((UPPER_RAIL_X, UPPER_RAIL_Y, UPPER_RAIL_H)),
        origin=Origin(
            xyz=(
                UPPER_RAIL_OFFSET_X,
                0.0,
                LOWER_BEAM_H + LOWER_CROSS_T + (UPPER_RAIL_H / 2.0),
            )
        ),
        material="carriage_aluminum",
        name="upper_rail_right",
    )
    lower_carriage.visual(
        Box((0.090, 0.090, LOWER_COVER_T)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                LOWER_BEAM_H + LOWER_CROSS_T + UPPER_GUIDE_H + (LOWER_COVER_T / 2.0),
            )
        ),
        material="fixture_base",
        name="center_cover",
    )
    lower_carriage_bottom = 0.0
    lower_carriage_top = LOWER_BEAM_H + LOWER_CROSS_T + UPPER_RAIL_H + LOWER_COVER_T
    lower_carriage.inertial = Inertial.from_geometry(
        Box((LOWER_CARRIAGE_L, LOWER_CARRIAGE_W, lower_carriage_top - lower_carriage_bottom)),
        mass=4.0,
        origin=Origin(
            xyz=(0.0, 0.0, (lower_carriage_top + lower_carriage_bottom) / 2.0)
        ),
    )

    upper_carriage = model.part("upper_carriage")
    upper_carriage.visual(
        Box((UPPER_RUNNER_X, UPPER_CARRIAGE_Y, UPPER_RUNNER_H)),
        origin=Origin(
            xyz=(-UPPER_RUNNER_OFFSET_X, 0.0, UPPER_RUNNER_H / 2.0)
        ),
        material="service_blue",
        name="upper_runner_left",
    )
    upper_carriage.visual(
        Box((UPPER_RUNNER_X, UPPER_CARRIAGE_Y, UPPER_RUNNER_H)),
        origin=Origin(
            xyz=(UPPER_RUNNER_OFFSET_X, 0.0, UPPER_RUNNER_H / 2.0)
        ),
        material="service_blue",
        name="upper_runner_right",
    )
    upper_carriage.visual(
        Box((UPPER_CARRIAGE_X, 0.12, UPPER_BRIDGE_H)),
        origin=Origin(
            xyz=(0.0, 0.0, UPPER_RUNNER_H + (UPPER_BRIDGE_H / 2.0))
        ),
        material="service_blue",
        name="upper_bridge",
    )
    upper_carriage.visual(
        Box((SERVICE_PAD_X, SERVICE_PAD_Y, SERVICE_PAD_H)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                UPPER_RUNNER_H + UPPER_BRIDGE_H + (SERVICE_PAD_H / 2.0),
            )
        ),
        material="service_blue",
        name="service_pad",
    )
    upper_carriage.visual(
        Cylinder(radius=TOP_LOCATOR_R, length=TOP_LOCATOR_H),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                UPPER_RUNNER_H + UPPER_BRIDGE_H + SERVICE_PAD_H + (TOP_LOCATOR_H / 2.0),
            )
        ),
        material="carriage_aluminum",
        name="locator_spigot",
    )
    upper_carriage.visual(
        Box((0.018, SERVICE_PAD_Y + 0.012, 0.010)),
        origin=Origin(
            xyz=(0.050, 0.0, UPPER_RUNNER_H + UPPER_BRIDGE_H + 0.005)
        ),
        material="service_blue",
        name="clamp_lug_right",
    )
    upper_carriage.visual(
        Box((0.018, SERVICE_PAD_Y + 0.012, 0.010)),
        origin=Origin(
            xyz=(-0.050, 0.0, UPPER_RUNNER_H + UPPER_BRIDGE_H + 0.005)
        ),
        material="service_blue",
        name="clamp_lug_left",
    )
    upper_carriage_bottom = 0.0
    upper_carriage.inertial = Inertial.from_geometry(
        Box((UPPER_CARRIAGE_X, UPPER_CARRIAGE_Y, TOP_TOTAL_TOP_Z - upper_carriage_bottom)),
        mass=2.2,
        origin=Origin(
            xyz=(0.0, 0.0, (TOP_TOTAL_TOP_Z + upper_carriage_bottom) / 2.0)
        ),
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-LOWER_STAGE_TRAVEL,
            upper=LOWER_STAGE_TRAVEL,
            effort=220.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=upper_carriage,
        origin=Origin(xyz=(0.0, 0.0, UPPER_STAGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-UPPER_STAGE_TRAVEL,
            upper=UPPER_STAGE_TRAVEL,
            effort=160.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    lower = object_model.get_part("lower_carriage")
    upper = object_model.get_part("upper_carriage")
    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")

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

    ctx.expect_contact(base, lower, name="lower_stage_supported_at_rest")
    ctx.expect_contact(lower, upper, name="upper_stage_supported_at_rest")
    ctx.expect_overlap(base, lower, axes="xy", min_overlap=0.10, name="lower_stage_has_clear_overlap")
    ctx.expect_overlap(lower, upper, axes="xy", min_overlap=0.08, name="upper_stage_has_clear_overlap")

    with ctx.pose({base_to_lower: LOWER_STAGE_TRAVEL * 0.75, lower_to_upper: UPPER_STAGE_TRAVEL * 0.75}):
        ctx.expect_contact(base, lower, name="lower_stage_supported_in_extended_pose")
        ctx.expect_contact(lower, upper, name="upper_stage_supported_in_extended_pose")
        ctx.expect_overlap(
            base,
            lower,
            axes="xy",
            min_overlap=0.08,
            name="lower_stage_keeps_overlap_when_extended",
        )
        ctx.expect_overlap(
            lower,
            upper,
            axes="xy",
            min_overlap=0.06,
            name="upper_stage_keeps_overlap_when_extended",
        )

    rest_lower_pos = ctx.part_world_position(lower)
    with ctx.pose({base_to_lower: LOWER_STAGE_TRAVEL * 0.75}):
        moved_lower_pos = ctx.part_world_position(lower)
    lower_ok = (
        rest_lower_pos is not None
        and moved_lower_pos is not None
        and moved_lower_pos[0] > rest_lower_pos[0] + 0.06
        and abs(moved_lower_pos[1] - rest_lower_pos[1]) < 1e-6
        and abs(moved_lower_pos[2] - rest_lower_pos[2]) < 1e-6
    )
    ctx.check(
        "lower_stage_moves_along_x",
        lower_ok,
        details=f"rest={rest_lower_pos}, moved={moved_lower_pos}",
    )

    with ctx.pose({base_to_lower: 0.03, lower_to_upper: 0.0}):
        ref_lower_pos = ctx.part_world_position(lower)
        ref_upper_pos = ctx.part_world_position(upper)
    with ctx.pose({base_to_lower: 0.03, lower_to_upper: UPPER_STAGE_TRAVEL * 0.75}):
        moved_lower_for_upper = ctx.part_world_position(lower)
        moved_upper_pos = ctx.part_world_position(upper)
    upper_ok = (
        ref_lower_pos is not None
        and ref_upper_pos is not None
        and moved_lower_for_upper is not None
        and moved_upper_pos is not None
        and (moved_upper_pos[1] - moved_lower_for_upper[1]) > (ref_upper_pos[1] - ref_lower_pos[1]) + 0.035
        and abs((moved_upper_pos[0] - moved_lower_for_upper[0]) - (ref_upper_pos[0] - ref_lower_pos[0])) < 1e-6
        and abs((moved_upper_pos[2] - moved_lower_for_upper[2]) - (ref_upper_pos[2] - ref_lower_pos[2])) < 1e-6
    )
    ctx.check(
        "upper_stage_moves_along_y",
        upper_ok,
        details=(
            f"ref_lower={ref_lower_pos}, ref_upper={ref_upper_pos}, "
            f"moved_lower={moved_lower_for_upper}, moved_upper={moved_upper_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
