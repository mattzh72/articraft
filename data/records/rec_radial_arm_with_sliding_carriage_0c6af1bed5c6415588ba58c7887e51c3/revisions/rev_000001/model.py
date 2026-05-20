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


BASE_LENGTH = 0.40
BASE_WIDTH = 0.28
BASE_HEIGHT = 0.028
REAR_HOUSING_LENGTH = 0.14
REAR_HOUSING_WIDTH = 0.10
REAR_HOUSING_HEIGHT = 0.05
COLUMN_RADIUS = 0.058
COLUMN_HEIGHT = 0.105
TURRET_RADIUS = 0.068
TURRET_HEIGHT = 0.052

ARM_SHOULDER_RADIUS = 0.072
ARM_SHOULDER_THICKNESS = 0.018
ARM_TAIL_LENGTH = 0.055
ARM_BEAM_START = 0.05
ARM_BEAM_LENGTH = 0.42
ARM_BEAM_WIDTH = 0.06
ARM_BEAM_HEIGHT = 0.032
ARM_NOSE_LENGTH = 0.055
ARM_RAIL_START = 0.10
ARM_RAIL_LENGTH = 0.30
ARM_RAIL_WIDTH = 0.034
ARM_RAIL_HEIGHT = 0.012

HEAD_LENGTH = 0.072
HEAD_WIDTH = 0.070
HEAD_HEIGHT = 0.055
HEAD_SHOE_LENGTH = 0.058
HEAD_SHOE_WIDTH = 0.040
HEAD_SHOE_HEIGHT = 0.018
HEAD_POD_LENGTH = 0.034
HEAD_POD_WIDTH = 0.044
HEAD_POD_HEIGHT = 0.028

ARM_JOINT_Z = BASE_HEIGHT + COLUMN_HEIGHT + TURRET_HEIGHT
HEAD_SLIDE_HOME_X = 0.14
HEAD_SLIDE_Z = ARM_SHOULDER_THICKNESS + ARM_BEAM_HEIGHT + ARM_RAIL_HEIGHT
HEAD_SLIDE_TRAVEL = 0.22


def _make_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
    )
    rear_housing = cq.Workplane("XY").box(
        REAR_HOUSING_LENGTH,
        REAR_HOUSING_WIDTH,
        REAR_HOUSING_HEIGHT,
        centered=(True, True, False),
    ).translate((-0.07, 0.0, BASE_HEIGHT))
    column = cq.Workplane("XY").circle(COLUMN_RADIUS).extrude(COLUMN_HEIGHT).translate(
        (0.0, 0.0, BASE_HEIGHT)
    )
    turret = cq.Workplane("XY").circle(TURRET_RADIUS).extrude(TURRET_HEIGHT).translate(
        (0.0, 0.0, BASE_HEIGHT + COLUMN_HEIGHT)
    )
    return plate.union(rear_housing).union(column).union(turret)


def _make_arm_shape() -> cq.Workplane:
    shoulder = cq.Workplane("XY").circle(ARM_SHOULDER_RADIUS).extrude(ARM_SHOULDER_THICKNESS)
    tail = cq.Workplane("XY").box(
        ARM_TAIL_LENGTH,
        ARM_BEAM_WIDTH * 0.86,
        ARM_BEAM_HEIGHT * 0.66,
        centered=(True, True, False),
    ).translate((-0.02, 0.0, ARM_SHOULDER_THICKNESS))
    beam = cq.Workplane("XY").box(
        ARM_BEAM_LENGTH,
        ARM_BEAM_WIDTH,
        ARM_BEAM_HEIGHT,
        centered=(True, True, False),
    ).translate((ARM_BEAM_START + ARM_BEAM_LENGTH / 2.0, 0.0, ARM_SHOULDER_THICKNESS))
    nose = cq.Workplane("XY").box(
        ARM_NOSE_LENGTH,
        ARM_BEAM_WIDTH * 0.72,
        ARM_BEAM_HEIGHT * 0.92,
        centered=(True, True, False),
    ).translate(
        (
            ARM_BEAM_START + ARM_BEAM_LENGTH - 0.012,
            0.0,
            ARM_SHOULDER_THICKNESS + 0.002,
        )
    )
    rail = cq.Workplane("XY").box(
        ARM_RAIL_LENGTH,
        ARM_RAIL_WIDTH,
        ARM_RAIL_HEIGHT,
        centered=(True, True, False),
    ).translate(
        (
            ARM_RAIL_START + ARM_RAIL_LENGTH / 2.0,
            0.0,
            ARM_SHOULDER_THICKNESS + ARM_BEAM_HEIGHT,
        )
    )
    return shoulder.union(tail).union(beam).union(nose).union(rail)


def _make_head_shape() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(
        HEAD_SHOE_LENGTH,
        HEAD_SHOE_WIDTH,
        HEAD_SHOE_HEIGHT,
        centered=(True, True, False),
    )
    carriage = cq.Workplane("XY").box(
        HEAD_LENGTH,
        HEAD_WIDTH,
        HEAD_HEIGHT - HEAD_SHOE_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, HEAD_SHOE_HEIGHT))
    pod = cq.Workplane("XY").box(
        HEAD_POD_LENGTH,
        HEAD_POD_WIDTH,
        HEAD_POD_HEIGHT,
        centered=(True, True, False),
    ).translate((0.022, 0.0, HEAD_HEIGHT - HEAD_POD_HEIGHT * 0.55))
    return shoe.union(carriage).union(pod)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_radial_arm")

    base_color = model.material("base_powdercoat", rgba=(0.20, 0.22, 0.24, 1.0))
    arm_color = model.material("arm_painted_alloy", rgba=(0.72, 0.75, 0.78, 1.0))
    head_color = model.material("head_casting", rgba=(0.16, 0.17, 0.19, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shape(), "base_shell"), material=base_color)
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, ARM_JOINT_Z)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, ARM_JOINT_Z / 2.0)),
    )

    arm = model.part("arm")
    arm.visual(mesh_from_cadquery(_make_arm_shape(), "arm_shell"), material=arm_color)
    arm.inertial = Inertial.from_geometry(
        Box((ARM_BEAM_START + ARM_BEAM_LENGTH, ARM_BEAM_WIDTH, 0.07)),
        mass=2.1,
        origin=Origin(xyz=(0.20, 0.0, 0.035)),
    )

    head = model.part("head")
    head.visual(mesh_from_cadquery(_make_head_shape(), "head_shell"), material=head_color)
    head.inertial = Inertial.from_geometry(
        Box((HEAD_LENGTH, HEAD_WIDTH, HEAD_HEIGHT)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, HEAD_HEIGHT / 2.0)),
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, ARM_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=-1.4,
            upper=1.6,
        ),
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=head,
        origin=Origin(xyz=(HEAD_SLIDE_HOME_X, 0.0, HEAD_SLIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=HEAD_SLIDE_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    sweep = object_model.get_articulation("base_to_arm")
    slide = object_model.get_articulation("arm_to_head")

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

    ctx.expect_contact(base, arm, contact_tol=5e-4, name="arm_turntable_is_seated_on_base")
    ctx.expect_contact(arm, head, contact_tol=5e-4, name="head_carriage_is_supported_on_arm")
    ctx.expect_gap(head, base, axis="z", min_gap=0.03, name="head_clears_base_in_home_pose")

    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_contact(
            arm,
            head,
            contact_tol=5e-4,
            name="head_carriage_stays_supported_when_extended",
        )
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.03,
            name="head_clears_base_when_extended",
        )

    with ctx.pose({sweep: 0.9, slide: 0.0}):
        swung_head_pos = ctx.part_world_position(head)
        ctx.check(
            "positive_sweep_moves_head_toward_positive_y",
            swung_head_pos is not None and swung_head_pos[1] > 0.08,
            f"expected head y > 0.08 m at positive sweep, got {swung_head_pos}",
        )

    with ctx.pose({sweep: 0.0, slide: 0.0}):
        home_head_pos = ctx.part_world_position(head)
    with ctx.pose({sweep: 0.0, slide: slide.motion_limits.upper}):
        extended_head_pos = ctx.part_world_position(head)
    ctx.check(
        "positive_slide_extends_head_outward",
        home_head_pos is not None
        and extended_head_pos is not None
        and (extended_head_pos[0] - home_head_pos[0]) > 0.18,
        f"expected slide extension > 0.18 m, got home={home_head_pos}, extended={extended_head_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
