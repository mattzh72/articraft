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


BASE_LENGTH = 0.42
BASE_WIDTH = 0.28
BASE_THICKNESS = 0.045
PEDESTAL_LENGTH = 0.17
PEDESTAL_WIDTH = 0.13
PEDESTAL_HEIGHT = 0.095
TURNTABLE_HEIGHT = 0.03
CAP_HEIGHT = 0.01
SWEEP_ORIGIN_Z = BASE_THICKNESS + PEDESTAL_HEIGHT + TURNTABLE_HEIGHT + CAP_HEIGHT

ARM_LENGTH = 0.305
ARM_WIDTH = 0.078
ARM_BODY_HEIGHT = 0.04
ARM_START_X = 0.035
ARM_BODY_Z = 0.01
RAIL_START_X = 0.07
RAIL_LENGTH = 0.248
RAIL_WIDTH = 0.046
RAIL_HEIGHT = 0.01
RAIL_TOP_Z = ARM_BODY_Z + ARM_BODY_HEIGHT + RAIL_HEIGHT

HEAD_LENGTH = 0.078
HEAD_WIDTH = 0.08
HEAD_BASE_HEIGHT = 0.036
SLIDE_TRAVEL = 0.17


def _base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
        .faces(">Z")
        .edges()
        .fillet(0.006)
    )

    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    turntable = (
        cq.Workplane("XY")
        .circle(0.062)
        .extrude(TURNTABLE_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT))
    )

    cap = (
        cq.Workplane("XY")
        .circle(0.045)
        .extrude(CAP_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT + TURNTABLE_HEIGHT))
    )

    rear_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.06, BASE_THICKNESS),
                (0.06, BASE_THICKNESS),
                (0.042, BASE_THICKNESS + 0.052),
                (-0.042, BASE_THICKNESS + 0.052),
            ]
        )
        .close()
        .extrude(0.018, both=True)
        .translate((0.0, -0.04, 0.0))
    )

    front_rib = rear_rib.translate((0.0, 0.08, 0.0))

    return foot.union(pedestal).union(turntable).union(cap).union(rear_rib).union(front_rib)


def _arm_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.053).extrude(0.046)

    hub_cap = (
        cq.Workplane("XY")
        .circle(0.036)
        .extrude(0.014)
        .translate((0.0, 0.0, 0.046))
    )

    beam = (
        cq.Workplane("XY")
        .box(ARM_LENGTH, ARM_WIDTH, ARM_BODY_HEIGHT, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.008)
        .translate((ARM_START_X, 0.0, ARM_BODY_Z))
    )

    rail = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((RAIL_START_X, 0.0, ARM_BODY_Z + ARM_BODY_HEIGHT))
    )

    end_stop = (
        cq.Workplane("XY")
        .box(0.012, 0.06, 0.02, centered=(False, True, False))
        .translate((RAIL_START_X + RAIL_LENGTH, 0.0, ARM_BODY_Z + 0.03))
    )

    brace = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, 0.0),
                (0.072, 0.0),
                (0.102, 0.012),
                (0.074, 0.038),
                (0.018, 0.038),
            ]
        )
        .close()
        .extrude(0.022, both=True)
    )

    return hub.union(hub_cap).union(beam).union(rail).union(end_stop).union(brace)


def _head_shape() -> cq.Workplane:
    carriage = (
        cq.Workplane("XY")
        .box(HEAD_LENGTH, HEAD_WIDTH, HEAD_BASE_HEIGHT, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.005)
    )

    top_housing = (
        cq.Workplane("XY")
        .box(0.05, 0.054, 0.024, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((0.016, 0.0, HEAD_BASE_HEIGHT))
    )

    side_pod = (
        cq.Workplane("XZ")
        .circle(0.02)
        .extrude(0.024)
        .translate((0.054, 0.04, 0.018))
    )

    nozzle = (
        cq.Workplane("XZ")
        .circle(0.01)
        .extrude(0.018)
        .translate((0.054, 0.058, 0.018))
    )

    neck = (
        cq.Workplane("XY")
        .box(0.026, 0.024, 0.022, centered=(False, True, False))
        .translate((0.041, 0.028, 0.007))
    )

    return carriage.union(top_housing).union(side_pod).union(nozzle).union(neck)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_radial_arm")

    model.material("base_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("arm_graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("head_silver", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("accent_orange", rgba=(0.88, 0.47, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "service_radial_arm_base"),
        material="base_gray",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, SWEEP_ORIGIN_Z)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, SWEEP_ORIGIN_Z / 2.0)),
    )

    arm = model.part("sweeping_arm")
    arm.visual(
        mesh_from_cadquery(_arm_shape(), "service_radial_arm_sweeping_arm"),
        material="arm_graphite",
        name="arm_shell",
    )
    arm.inertial = Inertial.from_geometry(
        Box((ARM_START_X + ARM_LENGTH, ARM_WIDTH, 0.06)),
        mass=3.2,
        origin=Origin(xyz=((ARM_START_X + ARM_LENGTH) / 2.0, 0.0, 0.03)),
    )

    head = model.part("sliding_head")
    head.visual(
        mesh_from_cadquery(_head_shape(), "service_radial_arm_sliding_head"),
        material="head_silver",
        name="head_shell",
    )
    head.inertial = Inertial.from_geometry(
        Box((HEAD_LENGTH, HEAD_WIDTH + 0.03, 0.06)),
        mass=1.4,
        origin=Origin(xyz=(0.045, 0.015, 0.022)),
    )

    model.articulation(
        "base_to_arm_sweep",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, SWEEP_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.35,
            upper=1.35,
            effort=80.0,
            velocity=1.0,
        ),
    )

    model.articulation(
        "arm_to_head_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=head,
        origin=Origin(xyz=(RAIL_START_X, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=25.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("sweeping_arm")
    head = object_model.get_part("sliding_head")
    sweep = object_model.get_articulation("base_to_arm_sweep")
    slide = object_model.get_articulation("arm_to_head_slide")

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
        arm,
        base,
        elem_a="arm_shell",
        elem_b="base_shell",
        name="arm hub seats on the grounded base",
    )
    ctx.expect_contact(
        head,
        arm,
        name="sliding head sits on the arm rail",
    )
    ctx.expect_gap(
        head,
        base,
        axis="z",
        min_gap=0.05,
        name="head clears the grounded base in the parked pose",
    )
    ctx.expect_overlap(
        head,
        arm,
        axes="x",
        min_overlap=0.07,
        name="parked head remains engaged on the arm span",
    )

    parked_head_pos = ctx.part_world_position(head)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            head,
            arm,
            name="extended head remains supported by the arm rail",
        )
        ctx.expect_overlap(
            head,
            arm,
            axes="x",
            min_overlap=0.06,
            name="extended head still retains meaningful arm engagement",
        )
        extended_head_pos = ctx.part_world_position(head)

    ctx.check(
        "slide joint moves the head outward along the arm",
        parked_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[0] > parked_head_pos[0] + 0.12,
        details=f"parked={parked_head_pos}, extended={extended_head_pos}",
    )

    with ctx.pose({slide: SLIDE_TRAVEL, sweep: 1.0}):
        swept_head_pos = ctx.part_world_position(head)
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.05,
            name="swept head still clears the grounded base",
        )

    ctx.check(
        "sweep joint arcs the extended head around the base",
        extended_head_pos is not None
        and swept_head_pos is not None
        and swept_head_pos[1] > 0.18
        and swept_head_pos[0] < extended_head_pos[0] - 0.05,
        details=f"extended={extended_head_pos}, swept={swept_head_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
