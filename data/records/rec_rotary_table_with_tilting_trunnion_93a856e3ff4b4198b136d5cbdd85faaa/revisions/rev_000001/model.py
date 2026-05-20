from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)


STAGE_TO_YOKE = "support_to_rotary_yoke"
YOKE_TO_TABLE = "rotary_yoke_to_work_table"
TRUNNION_Z = -0.24

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_trunnion_positioner")

    support_finish = model.material("support_finish", rgba=(0.79, 0.81, 0.83, 1.0))
    yoke_finish = model.material("yoke_finish", rgba=(0.20, 0.23, 0.26, 1.0))
    table_finish = model.material("table_finish", rgba=(0.30, 0.42, 0.56, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.78, 0.62, 0.12)),
        material=support_finish,
        origin=Origin(xyz=(-0.92, 0.0, -0.98)),
        name="base",
    )
    support_frame.visual(
        Box((0.22, 0.30, 1.04)),
        material=support_finish,
        origin=Origin(xyz=(-0.92, 0.0, -0.40)),
        name="column",
    )
    support_frame.visual(
        Box((0.96, 0.24, 0.16)),
        material=support_finish,
        origin=Origin(xyz=(-0.44, 0.0, 0.16)),
        name="overhead_arm",
    )
    support_frame.visual(
        Box((0.20, 0.16, 0.42)),
        material=support_finish,
        origin=Origin(xyz=(-0.66, 0.0, -0.02)),
        name="gusset",
    )
    support_frame.visual(
        Cylinder(radius=0.16, length=0.10),
        material=support_finish,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="stage_housing",
    )
    support_frame.visual(
        Box((0.22, 0.18, 0.14)),
        material=support_finish,
        origin=Origin(xyz=(-0.20, 0.18, 0.15)),
        name="motor_box",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((1.28, 0.70, 1.22)),
        mass=850.0,
        origin=Origin(xyz=(-0.52, 0.0, -0.38)),
    )

    rotary_yoke = model.part("rotary_yoke")
    rotary_yoke.visual(
        Cylinder(radius=0.15, length=0.03),
        material=yoke_finish,
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        name="rotor_flange",
    )
    rotary_yoke.visual(
        Cylinder(radius=0.10, length=0.05),
        material=yoke_finish,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        name="rotor_hub",
    )
    rotary_yoke.visual(
        Box((0.18, 0.10, 0.06)),
        material=yoke_finish,
        origin=Origin(xyz=(-0.17, 0.0, -0.08)),
        name="left_shoulder",
    )
    rotary_yoke.visual(
        Box((0.18, 0.10, 0.06)),
        material=yoke_finish,
        origin=Origin(xyz=(0.17, 0.0, -0.08)),
        name="right_shoulder",
    )
    rotary_yoke.visual(
        Box((0.08, 0.12, 0.46)),
        material=yoke_finish,
        origin=Origin(xyz=(-0.30, 0.0, -0.28)),
        name="left_cheek",
    )
    rotary_yoke.visual(
        Box((0.08, 0.12, 0.46)),
        material=yoke_finish,
        origin=Origin(xyz=(0.30, 0.0, -0.28)),
        name="right_cheek",
    )
    rotary_yoke.visual(
        Cylinder(radius=0.06, length=0.02),
        material=yoke_finish,
        origin=Origin(xyz=(-0.35, 0.0, TRUNNION_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="left_trunnion_support",
    )
    rotary_yoke.visual(
        Cylinder(radius=0.06, length=0.02),
        material=yoke_finish,
        origin=Origin(xyz=(0.35, 0.0, TRUNNION_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="right_trunnion_support",
    )
    rotary_yoke.inertial = Inertial.from_geometry(
        Box((0.72, 0.18, 0.64)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
    )

    work_table = model.part("work_table")
    work_table.visual(
        Cylinder(radius=0.03, length=0.48),
        material=table_finish,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="trunnion_shaft",
    )
    work_table.visual(
        Cylinder(radius=0.05, length=0.04),
        material=table_finish,
        origin=Origin(xyz=(-0.24, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="left_trunnion",
    )
    work_table.visual(
        Cylinder(radius=0.05, length=0.04),
        material=table_finish,
        origin=Origin(xyz=(0.24, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="right_trunnion",
    )
    work_table.visual(
        Box((0.12, 0.10, 0.26)),
        material=table_finish,
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
        name="hanger_block",
    )
    work_table.visual(
        Box((0.08, 0.08, 0.28)),
        material=table_finish,
        origin=Origin(xyz=(0.0, 0.0, -0.30)),
        name="hanger_web",
    )
    work_table.visual(
        Cylinder(radius=0.28, length=0.03),
        material=table_finish,
        origin=Origin(xyz=(0.0, 0.0, -0.44)),
        name="table_plate",
    )
    work_table.visual(
        Cylinder(radius=0.08, length=0.022),
        material=table_finish,
        origin=Origin(xyz=(0.0, 0.0, -0.418)),
        name="center_boss",
    )
    work_table.visual(
        Box((0.18, 0.03, 0.05)),
        material=table_finish,
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
        name="fixture_key_x",
    )
    work_table.visual(
        Box((0.03, 0.18, 0.05)),
        material=table_finish,
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
        name="fixture_key_y",
    )
    work_table.inertial = Inertial.from_geometry(
        Box((0.56, 0.56, 0.52)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, -0.30)),
    )

    model.articulation(
        STAGE_TO_YOKE,
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=rotary_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5000.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    model.articulation(
        YOKE_TO_TABLE,
        ArticulationType.REVOLUTE,
        parent=rotary_yoke,
        child=work_table,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=1.0,
            lower=-2.2,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    rotary_yoke = object_model.get_part("rotary_yoke")
    work_table = object_model.get_part("work_table")
    stage_joint = object_model.get_articulation(STAGE_TO_YOKE)
    trunnion_joint = object_model.get_articulation(YOKE_TO_TABLE)

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
        "parts_present",
        all(part is not None for part in (support_frame, rotary_yoke, work_table)),
        "Expected support frame, rotary yoke, and work table parts.",
    )
    ctx.check(
        "stage_axis_is_vertical",
        tuple(stage_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected lower rotary axis to be +Z, got {stage_joint.axis!r}.",
    )
    ctx.check(
        "trunnion_axis_is_horizontal",
        tuple(trunnion_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected hanging table trunnion axis to be +X, got {trunnion_joint.axis!r}.",
    )

    ctx.expect_contact(
        support_frame,
        rotary_yoke,
        name="support_frame_carries_rotary_yoke",
    )
    ctx.expect_contact(
        rotary_yoke,
        work_table,
        name="yoke_supports_work_table",
    )
    ctx.expect_origin_gap(
        rotary_yoke,
        work_table,
        axis="z",
        min_gap=0.20,
        name="work_table_hangs_below_rotary_stage",
    )
    ctx.expect_within(
        work_table,
        rotary_yoke,
        axes="x",
        margin=0.0,
        name="work_table_stays_between_trunnion_cheeks",
    )

    with ctx.pose({trunnion_joint: 1.2}):
        ctx.expect_contact(
            rotary_yoke,
            work_table,
            name="trunnion_contact_persists_when_tilted",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
