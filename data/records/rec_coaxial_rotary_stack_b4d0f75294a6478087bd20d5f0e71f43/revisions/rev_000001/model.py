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


BASE_BOTTOM_Z = 0.020
BASE_HEIGHT = 0.085

LOWER_SPACER_BOTTOM_Z = BASE_BOTTOM_Z + BASE_HEIGHT
LOWER_SPACER_HEIGHT = 0.020

COLLAR_BOTTOM_Z = LOWER_SPACER_BOTTOM_Z + LOWER_SPACER_HEIGHT
COLLAR_HEIGHT = 0.070

UPPER_SPACER_BOTTOM_Z = COLLAR_BOTTOM_Z + COLLAR_HEIGHT
UPPER_SPACER_HEIGHT = 0.020

NOSE_BOTTOM_Z = UPPER_SPACER_BOTTOM_Z + UPPER_SPACER_HEIGHT
NOSE_HEIGHT = 0.090

TOP_STUD_BOTTOM_Z = NOSE_BOTTOM_Z + NOSE_HEIGHT
TOP_STUD_HEIGHT = 0.030


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_indexing_head")

    dark_base = model.material("dark_base", rgba=(0.20, 0.24, 0.28, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.18, 0.20, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.69, 0.72, 0.74, 1.0))
    spacer_steel = model.material("spacer_steel", rgba=(0.58, 0.61, 0.64, 1.0))

    core_base = model.part("core_base")
    core_base.visual(
        Cylinder(radius=0.060, length=BASE_BOTTOM_Z),
        origin=Origin(xyz=(0.0, 0.0, BASE_BOTTOM_Z / 2.0)),
        name="core_base_shell",
        material=spacer_steel,
    )
    core_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=BASE_BOTTOM_Z),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_BOTTOM_Z / 2.0)),
    )

    lower_spacer = model.part("lower_spacer")
    lower_spacer.visual(
        Cylinder(radius=0.037, length=LOWER_SPACER_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, LOWER_SPACER_HEIGHT / 2.0)),
        name="lower_spacer_shell",
        material=spacer_steel,
    )
    lower_spacer.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=LOWER_SPACER_HEIGHT),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, LOWER_SPACER_HEIGHT / 2.0)),
    )

    upper_spacer = model.part("upper_spacer")
    upper_spacer.visual(
        Cylinder(radius=0.029, length=UPPER_SPACER_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, UPPER_SPACER_HEIGHT / 2.0)),
        name="upper_spacer_shell",
        material=spacer_steel,
    )
    upper_spacer.inertial = Inertial.from_geometry(
        Cylinder(radius=0.029, length=UPPER_SPACER_HEIGHT),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, UPPER_SPACER_HEIGHT / 2.0)),
    )

    top_stud = model.part("top_stud")
    top_stud.visual(
        Cylinder(radius=0.018, length=TOP_STUD_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, TOP_STUD_HEIGHT / 2.0)),
        name="top_stud_shell",
        material=spacer_steel,
    )
    top_stud.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=TOP_STUD_HEIGHT),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, TOP_STUD_HEIGHT / 2.0)),
    )

    base_stage = model.part("base_stage")
    base_stage.visual(
        Cylinder(radius=0.160, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        name="base_flange",
        material=dark_base,
    )
    base_stage.visual(
        Cylinder(radius=0.135, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0505)),
        name="base_drum",
        material=dark_base,
    )
    base_stage.visual(
        Cylinder(radius=0.110, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        name="base_top_step",
        material=charcoal,
    )
    base_stage.visual(
        Box((0.050, 0.024, 0.030)),
        origin=Origin(xyz=(0.150, 0.0, 0.058)),
        name="base_detent_block",
        material=machined_steel,
    )
    base_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=BASE_HEIGHT),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    middle_collar = model.part("middle_collar")
    middle_collar.visual(
        Cylinder(radius=0.115, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        name="collar_lower_flange",
        material=charcoal,
    )
    middle_collar.visual(
        Cylinder(radius=0.095, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        name="collar_body",
        material=charcoal,
    )
    middle_collar.visual(
        Cylinder(radius=0.1025, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        name="collar_upper_flange",
        material=dark_base,
    )
    middle_collar.visual(
        Box((0.038, 0.018, 0.022)),
        origin=Origin(xyz=(0.104, 0.0, 0.043)),
        name="collar_index_block",
        material=machined_steel,
    )
    middle_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=COLLAR_HEIGHT),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_HEIGHT / 2.0)),
    )

    tooling_nose = model.part("tooling_nose")
    tooling_nose.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        name="nose_base_flange",
        material=machined_steel,
    )
    tooling_nose.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        name="nose_body",
        material=machined_steel,
    )
    tooling_nose.visual(
        Cylinder(radius=0.048, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        name="nose_mid_step",
        material=machined_steel,
    )
    tooling_nose.visual(
        Cylinder(radius=0.043, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        name="nose_tip_step",
        material=machined_steel,
    )
    tooling_nose.visual(
        Box((0.022, 0.014, 0.018)),
        origin=Origin(xyz=(0.049, 0.0, 0.072)),
        name="nose_drive_key",
        material=dark_base,
    )
    tooling_nose.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=NOSE_HEIGHT),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, NOSE_HEIGHT / 2.0)),
    )

    full_turn_limits = MotionLimits(
        effort=40.0,
        velocity=2.5,
        lower=-math.pi,
        upper=math.pi,
    )

    model.articulation(
        "core_base_to_lower_spacer",
        ArticulationType.FIXED,
        parent=core_base,
        child=lower_spacer,
        origin=Origin(xyz=(0.0, 0.0, LOWER_SPACER_BOTTOM_Z)),
    )
    model.articulation(
        "core_base_to_upper_spacer",
        ArticulationType.FIXED,
        parent=core_base,
        child=upper_spacer,
        origin=Origin(xyz=(0.0, 0.0, UPPER_SPACER_BOTTOM_Z)),
    )
    model.articulation(
        "core_base_to_top_stud",
        ArticulationType.FIXED,
        parent=core_base,
        child=top_stud,
        origin=Origin(xyz=(0.0, 0.0, TOP_STUD_BOTTOM_Z)),
    )
    model.articulation(
        "core_base_to_base_stage",
        ArticulationType.REVOLUTE,
        parent=core_base,
        child=base_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn_limits,
    )
    model.articulation(
        "core_base_to_middle_collar",
        ArticulationType.REVOLUTE,
        parent=core_base,
        child=middle_collar,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn_limits,
    )
    model.articulation(
        "core_base_to_tooling_nose",
        ArticulationType.REVOLUTE,
        parent=core_base,
        child=tooling_nose,
        origin=Origin(xyz=(0.0, 0.0, NOSE_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    core_base = object_model.get_part("core_base")
    lower_spacer = object_model.get_part("lower_spacer")
    upper_spacer = object_model.get_part("upper_spacer")
    top_stud = object_model.get_part("top_stud")
    base_stage = object_model.get_part("base_stage")
    middle_collar = object_model.get_part("middle_collar")
    tooling_nose = object_model.get_part("tooling_nose")

    base_joint = object_model.get_articulation("core_base_to_base_stage")
    collar_joint = object_model.get_articulation("core_base_to_middle_collar")
    nose_joint = object_model.get_articulation("core_base_to_tooling_nose")

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

    ctx.check("core_base_present", core_base.name == "core_base", "core base part lookup failed")
    ctx.check("lower_spacer_present", lower_spacer.name == "lower_spacer", "lower spacer part lookup failed")
    ctx.check("upper_spacer_present", upper_spacer.name == "upper_spacer", "upper spacer part lookup failed")
    ctx.check("top_stud_present", top_stud.name == "top_stud", "top stud part lookup failed")
    ctx.check("base_stage_present", base_stage.name == "base_stage", "base stage part lookup failed")
    ctx.check(
        "middle_collar_present",
        middle_collar.name == "middle_collar",
        "middle collar part lookup failed",
    )
    ctx.check(
        "tooling_nose_present",
        tooling_nose.name == "tooling_nose",
        "tooling nose part lookup failed",
    )

    for joint_obj, joint_name in (
        (base_joint, "core_base_to_base_stage"),
        (collar_joint, "core_base_to_middle_collar"),
        (nose_joint, "core_base_to_tooling_nose"),
    ):
        axis_ok = tuple(round(value, 6) for value in joint_obj.axis) == (0.0, 0.0, 1.0)
        ctx.check(f"{joint_name}_vertical_axis", axis_ok, f"{joint_name} axis is {joint_obj.axis}")

        limits = joint_obj.motion_limits
        span_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= -3.0
            and limits.upper >= 3.0
        )
        ctx.check(
            f"{joint_name}_wide_rotation_span",
            span_ok,
            f"{joint_name} limits are {limits}",
        )

    ctx.expect_contact(base_stage, core_base, name="base_stage_seats_on_core_base")
    ctx.expect_contact(base_stage, lower_spacer, name="lower_spacer_reaches_base_stage")
    ctx.expect_contact(middle_collar, lower_spacer, name="middle_collar_seats_on_lower_spacer")
    ctx.expect_contact(middle_collar, upper_spacer, name="upper_spacer_reaches_middle_collar")
    ctx.expect_contact(tooling_nose, upper_spacer, name="tooling_nose_seats_on_upper_spacer")
    ctx.expect_contact(tooling_nose, top_stud, name="top_stud_caps_tooling_nose")

    ctx.expect_overlap(base_stage, core_base, axes="xy", min_overlap=0.10, name="base_stage_coaxial_on_core")
    ctx.expect_overlap(
        middle_collar,
        lower_spacer,
        axes="xy",
        min_overlap=0.07,
        name="middle_collar_coaxial_on_lower_spacer",
    )
    ctx.expect_overlap(
        tooling_nose,
        upper_spacer,
        axes="xy",
        min_overlap=0.05,
        name="tooling_nose_coaxial_on_upper_spacer",
    )

    ctx.expect_origin_gap(
        lower_spacer,
        core_base,
        axis="z",
        min_gap=0.104,
        max_gap=0.106,
        name="lower_spacer_placed_above_core_base",
    )
    ctx.expect_origin_gap(
        upper_spacer,
        lower_spacer,
        axis="z",
        min_gap=0.090,
        max_gap=0.090,
        name="upper_spacer_placed_above_lower_spacer",
    )
    ctx.expect_origin_gap(
        top_stud,
        upper_spacer,
        axis="z",
        min_gap=0.109,
        max_gap=0.111,
        name="top_stud_placed_above_upper_spacer",
    )

    ctx.expect_origin_gap(
        middle_collar,
        base_stage,
        axis="z",
        min_gap=0.104,
        max_gap=0.106,
        name="middle_collar_raised_above_base_stage",
    )
    ctx.expect_origin_gap(
        tooling_nose,
        middle_collar,
        axis="z",
        min_gap=0.089,
        max_gap=0.091,
        name="tooling_nose_raised_above_middle_collar",
    )

    with ctx.pose(
        {
            base_joint: 1.15,
            collar_joint: -0.75,
            nose_joint: 0.60,
        }
    ):
        ctx.expect_contact(base_stage, core_base, name="base_stage_stays_seated_when_rotated")
        ctx.expect_contact(middle_collar, lower_spacer, name="middle_collar_stays_seated_when_rotated")
        ctx.expect_contact(tooling_nose, upper_spacer, name="tooling_nose_stays_seated_when_rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
