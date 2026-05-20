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


BASE_LENGTH = 0.180
BASE_WIDTH = 0.120
BASE_THICKNESS = 0.016
PEDESTAL_LENGTH = 0.064
PEDESTAL_WIDTH = 0.050
PEDESTAL_HEIGHT = 0.050
TURNTABLE_RADIUS = 0.034
TURNTABLE_THICKNESS = 0.010
PIVOT_Z = BASE_THICKNESS + PEDESTAL_HEIGHT + TURNTABLE_THICKNESS

ARM_LENGTH = 0.315
ARM_WIDTH = 0.056
ARM_HUB_RADIUS = 0.032
ARM_HUB_THICKNESS = 0.012
ARM_BOSS_RADIUS = 0.024
ARM_BOSS_HEIGHT = 0.014
RAIL_START = 0.145
RAIL_LENGTH = 0.165
RAIL_WIDTH = 0.034
RAIL_HEIGHT = 0.010
RAIL_SUPPORT_WIDTH = 0.044
RAIL_SUPPORT_HEIGHT = 0.012
RAIL_BOTTOM_Z = 0.056
RAIL_TOP_Z = RAIL_BOTTOM_Z + RAIL_HEIGHT

CARRIAGE_LENGTH = 0.086
CARRIAGE_WIDTH = 0.054
CARRIAGE_BASE_HEIGHT = 0.024
CARRIAGE_SKID_WIDTH = 0.034
CARRIAGE_SKID_HEIGHT = 0.004
CARRIAGE_TOP_LENGTH = 0.050
CARRIAGE_TOP_WIDTH = 0.060
CARRIAGE_TOP_HEIGHT = 0.028
CARRIAGE_NOSE_RADIUS = 0.015
CARRIAGE_NOSE_LENGTH = 0.022
SLIDE_TRAVEL = 0.085


def _base_support_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS / 2.0)
    )
    pedestal = cq.Workplane("XY").box(
        PEDESTAL_LENGTH,
        PEDESTAL_WIDTH,
        PEDESTAL_HEIGHT,
    ).translate((0.0, 0.0, BASE_THICKNESS + (PEDESTAL_HEIGHT / 2.0)))
    turntable = cq.Workplane("XY").circle(TURNTABLE_RADIUS).extrude(TURNTABLE_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT)
    )
    front_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.028, BASE_THICKNESS),
                (0.028, BASE_THICKNESS),
                (0.020, BASE_THICKNESS + 0.018),
                (0.012, BASE_THICKNESS + PEDESTAL_HEIGHT),
                (-0.012, BASE_THICKNESS + PEDESTAL_HEIGHT),
                (-0.020, BASE_THICKNESS + 0.018),
            ]
        )
        .close()
        .extrude(0.012, both=True)
        .translate((0.0, 0.020, 0.0))
    )
    rear_rib = front_rib.mirror("XZ")
    return foot.union(pedestal).union(turntable).union(front_rib).union(rear_rib)


def _rotary_arm_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.020, 0.010),
                (0.058, 0.012),
                (0.250, 0.016),
                (ARM_LENGTH, 0.010),
                (ARM_LENGTH, 0.046),
                (0.255, 0.058),
                (0.090, 0.056),
                (0.030, 0.032),
            ]
        )
        .close()
        .extrude(ARM_WIDTH / 2.0, both=True)
    )
    hub = cq.Workplane("XY").circle(ARM_HUB_RADIUS).extrude(ARM_HUB_THICKNESS)
    boss = (
        cq.Workplane("XY")
        .workplane(offset=ARM_HUB_THICKNESS)
        .circle(ARM_BOSS_RADIUS)
        .extrude(ARM_BOSS_HEIGHT)
    )
    rail_support = cq.Workplane("XY").box(
        RAIL_LENGTH - 0.010,
        RAIL_SUPPORT_WIDTH,
        RAIL_SUPPORT_HEIGHT,
    ).translate(
        (
            RAIL_START + ((RAIL_LENGTH - 0.010) / 2.0),
            0.0,
            RAIL_BOTTOM_Z - (RAIL_SUPPORT_HEIGHT / 2.0),
        )
    )
    rail = cq.Workplane("XY").box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT).translate(
        (
            RAIL_START + (RAIL_LENGTH / 2.0),
            0.0,
            RAIL_BOTTOM_Z + (RAIL_HEIGHT / 2.0),
        )
    )
    rail_nose = (
        cq.Workplane("YZ")
        .center(0.0, RAIL_BOTTOM_Z + (RAIL_HEIGHT / 2.0))
        .circle(RAIL_HEIGHT / 2.0)
        .extrude(RAIL_WIDTH, both=True)
        .translate((RAIL_START + RAIL_LENGTH, 0.0, 0.0))
    )
    return hub.union(boss).union(body).union(rail_support).union(rail).union(rail_nose)


def _sliding_carriage_shape() -> cq.Workplane:
    skid = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_SKID_WIDTH,
        CARRIAGE_SKID_HEIGHT,
    ).translate(
        (
            CARRIAGE_LENGTH / 2.0,
            0.0,
            CARRIAGE_SKID_HEIGHT / 2.0,
        )
    )
    base = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_WIDTH,
        CARRIAGE_BASE_HEIGHT,
    ).translate(
        (
            CARRIAGE_LENGTH / 2.0,
            0.0,
            CARRIAGE_SKID_HEIGHT + (CARRIAGE_BASE_HEIGHT / 2.0),
        )
    )
    top_block = cq.Workplane("XY").box(
        CARRIAGE_TOP_LENGTH,
        CARRIAGE_TOP_WIDTH,
        CARRIAGE_TOP_HEIGHT,
    ).translate(
        (
            0.040,
            0.0,
            CARRIAGE_SKID_HEIGHT + CARRIAGE_BASE_HEIGHT + (CARRIAGE_TOP_HEIGHT / 2.0) - 0.004,
        )
    )
    nose = (
        cq.Workplane("YZ")
        .center(0.0, 0.040)
        .circle(CARRIAGE_NOSE_RADIUS)
        .extrude(CARRIAGE_NOSE_LENGTH)
        .translate((CARRIAGE_LENGTH - 0.004, 0.0, 0.0))
    )
    roof = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.014, CARRIAGE_SKID_HEIGHT + CARRIAGE_BASE_HEIGHT + 0.002),
                (0.032, CARRIAGE_SKID_HEIGHT + CARRIAGE_BASE_HEIGHT + 0.020),
                (0.062, CARRIAGE_SKID_HEIGHT + CARRIAGE_BASE_HEIGHT + 0.020),
                (0.074, CARRIAGE_SKID_HEIGHT + CARRIAGE_BASE_HEIGHT + 0.006),
                (0.074, CARRIAGE_SKID_HEIGHT + CARRIAGE_BASE_HEIGHT + 0.002),
                (0.014, CARRIAGE_SKID_HEIGHT + CARRIAGE_BASE_HEIGHT + 0.002),
            ]
        )
        .close()
        .extrude(0.020, both=True)
    )
    return skid.union(base).union(top_block).union(nose).union(roof)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_arm")

    model.material("base_gray", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("arm_silver", rgba=(0.68, 0.71, 0.75, 1.0))
    model.material("head_dark", rgba=(0.16, 0.17, 0.19, 1.0))

    base_support = model.part("base_support")
    base_support.visual(
        mesh_from_cadquery(_base_support_shape(), "base_support"),
        material="base_gray",
        name="base_support_shell",
    )
    base_support.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, PIVOT_Z)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z / 2.0)),
    )

    rotary_arm = model.part("rotary_arm")
    rotary_arm.visual(
        mesh_from_cadquery(_rotary_arm_shape(), "rotary_arm"),
        material="arm_silver",
        name="rotary_arm_shell",
    )
    rotary_arm.inertial = Inertial.from_geometry(
        Box((ARM_LENGTH, ARM_WIDTH, 0.072)),
        mass=1.4,
        origin=Origin(xyz=(ARM_LENGTH / 2.0, 0.0, 0.036)),
    )

    sliding_head = model.part("sliding_head")
    sliding_head.visual(
        mesh_from_cadquery(_sliding_carriage_shape(), "sliding_head"),
        material="head_dark",
        name="sliding_head_shell",
    )
    sliding_head.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH + CARRIAGE_NOSE_LENGTH, CARRIAGE_TOP_WIDTH, 0.060)),
        mass=0.85,
        origin=Origin(xyz=(0.052, 0.0, 0.030)),
    )

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=base_support,
        child=rotary_arm,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.10,
            upper=1.10,
            effort=30.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "head_slide",
        ArticulationType.PRISMATIC,
        parent=rotary_arm,
        child=sliding_head,
        origin=Origin(xyz=(RAIL_START, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=18.0,
            velocity=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_support = object_model.get_part("base_support")
    rotary_arm = object_model.get_part("rotary_arm")
    sliding_head = object_model.get_part("sliding_head")
    base_swivel = object_model.get_articulation("base_swivel")
    head_slide = object_model.get_articulation("head_slide")

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
        rotary_arm,
        base_support,
        name="rotary arm is seated on the base support",
    )

    with ctx.pose({head_slide: 0.0}):
        ctx.expect_contact(
            sliding_head,
            rotary_arm,
            name="carriage is supported on the rail when retracted",
        )
        ctx.expect_overlap(
            sliding_head,
            rotary_arm,
            axes="y",
            min_overlap=0.020,
            name="retracted carriage stays centered on the arm width",
        )

    with ctx.pose({head_slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            sliding_head,
            rotary_arm,
            name="carriage remains supported at full extension",
        )
        ctx.expect_overlap(
            sliding_head,
            rotary_arm,
            axes="y",
            min_overlap=0.020,
            name="extended carriage stays laterally captured by the arm",
        )

    with ctx.pose({base_swivel: 0.0, head_slide: 0.040}):
        straight_pos = ctx.part_world_position(sliding_head)
    with ctx.pose({base_swivel: 0.90, head_slide: 0.040}):
        swung_pos = ctx.part_world_position(sliding_head)
    ctx.check(
        "swivel rotates the arm in plan",
        (
            straight_pos is not None
            and swung_pos is not None
            and swung_pos[1] > straight_pos[1] + 0.12
            and swung_pos[0] < straight_pos[0] - 0.05
        ),
        details=f"straight={straight_pos}, swung={swung_pos}",
    )

    with ctx.pose({base_swivel: 0.0, head_slide: 0.0}):
        retracted_pos = ctx.part_world_position(sliding_head)
    with ctx.pose({base_swivel: 0.0, head_slide: SLIDE_TRAVEL}):
        extended_pos = ctx.part_world_position(sliding_head)
    ctx.check(
        "prismatic joint extends the head outward",
        (
            retracted_pos is not None
            and extended_pos is not None
            and extended_pos[0] > retracted_pos[0] + 0.07
        ),
        details=f"retracted={retracted_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
