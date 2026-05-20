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


SHOULDER_BARREL_LENGTH = 0.050
SHOULDER_BARREL_RADIUS = 0.026
UPPER_ARM_LENGTH = 0.280
ELBOW_BARREL_LENGTH = 0.042
ELBOW_BARREL_RADIUS = 0.020
FOREARM_LENGTH = 0.220
JOINT_CONTACT_PRELOAD = 0.001


def _disc(radius: float, thickness: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(thickness / 2.0, both=True)


def _rounded_box(length: float, width: float, height: float, fillet: float) -> cq.Workplane:
    solid = cq.Workplane("XY").box(length, width, height)
    if fillet > 0.0:
        solid = solid.edges("|X").fillet(fillet)
    return solid


def _build_root_bracket_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.012, 0.110, 0.170).translate((-0.030, 0.0, 0.0))
    shoulder_mount = _disc(0.026, 0.040).translate((-0.026, 0.0, 0.0))
    left_rib = cq.Workplane("XY").box(0.024, 0.012, 0.122).translate((-0.018, 0.026, 0.0))
    right_rib = cq.Workplane("XY").box(0.024, 0.012, 0.122).translate((-0.018, -0.026, 0.0))
    top_gusset = cq.Workplane("XY").box(0.030, 0.026, 0.024).translate((-0.023, 0.0, 0.050))
    bottom_gusset = cq.Workplane("XY").box(0.030, 0.026, 0.024).translate((-0.023, 0.0, -0.050))

    holes = (
        cq.Workplane("YZ")
        .pushPoints([(-0.032, -0.055), (0.032, -0.055), (-0.032, 0.055), (0.032, 0.055)])
        .circle(0.0045)
        .extrude(0.040, both=True)
        .translate((-0.030, 0.0, 0.0))
    )

    return (
        plate.union(shoulder_mount)
        .union(left_rib)
        .union(right_rib)
        .union(top_gusset)
        .union(bottom_gusset)
        .cut(holes)
    )


def _build_upper_arm_shape() -> cq.Workplane:
    main_beam = _rounded_box(0.200, 0.056, 0.086, 0.008).translate((0.150, 0.0, 0.0))
    shoulder_barrel = _disc(SHOULDER_BARREL_RADIUS, 0.040).translate((SHOULDER_BARREL_RADIUS, 0.0, 0.0))
    top_link = cq.Workplane("XY").box(0.044, 0.040, 0.014).translate((0.258, 0.0, 0.026))
    bottom_link = cq.Workplane("XY").box(0.044, 0.040, 0.014).translate((0.258, 0.0, -0.026))
    top_elbow_cap = _disc(0.020, 0.014).translate((0.260, 0.0, 0.026))
    bottom_elbow_cap = _disc(0.020, 0.014).translate((0.260, 0.0, -0.026))

    return (
        shoulder_barrel.union(main_beam)
        .union(top_link)
        .union(bottom_link)
        .union(top_elbow_cap)
        .union(bottom_elbow_cap)
    )


def _build_forearm_shape() -> cq.Workplane:
    root_barrel = _disc(ELBOW_BARREL_RADIUS, 0.038).translate(
        (ELBOW_BARREL_RADIUS - JOINT_CONTACT_PRELOAD, 0.0, 0.0)
    )
    beam = _rounded_box(0.170, 0.042, 0.052, 0.006).translate((0.105, 0.0, 0.0))
    tip_block = _rounded_box(0.035, 0.050, 0.044, 0.004).translate((0.2025, 0.0, 0.0))

    return root_barrel.union(beam).union(tip_block)


def _build_terminal_face_shape() -> cq.Workplane:
    plate = _rounded_box(0.012, 0.080, 0.090, 0.008).translate((0.006, 0.0, 0.0))
    holes = (
        cq.Workplane("YZ")
        .pushPoints([(-0.024, -0.028), (0.024, -0.028), (-0.024, 0.028), (0.024, 0.028)])
        .circle(0.004)
        .extrude(0.020, both=True)
        .translate((0.006, 0.0, 0.0))
    )
    return plate.cut(holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_elbow_arm")

    model.material("bracket_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("arm_satin", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("terminal_dark", rgba=(0.24, 0.26, 0.29, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_build_root_bracket_shape(), "root_bracket"),
        origin=Origin(),
        material="bracket_dark",
        name="bracket_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_build_upper_arm_shape(), "upper_arm"),
        origin=Origin(),
        material="arm_satin",
        name="upper_arm_shell",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_build_forearm_shape(), "forearm"),
        origin=Origin(),
        material="arm_satin",
        name="forearm_shell",
    )

    terminal_face = model.part("terminal_face")
    terminal_face.visual(
        mesh_from_cadquery(_build_terminal_face_shape(), "terminal_face"),
        origin=Origin(),
        material="terminal_dark",
        name="terminal_shell",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=upper_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.8, lower=0.0, upper=1.7),
    )
    model.articulation(
        "forearm_to_terminal",
        ArticulationType.FIXED,
        parent=forearm,
        child=terminal_face,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    terminal_face = object_model.get_part("terminal_face")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

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

    ctx.expect_contact(root_bracket, upper_arm, name="root bracket supports shoulder barrel")
    ctx.expect_contact(upper_arm, forearm, name="upper arm supports elbow barrel")
    ctx.expect_contact(forearm, terminal_face, name="forearm carries terminal face")

    ctx.expect_origin_gap(
        forearm,
        upper_arm,
        axis="x",
        min_gap=UPPER_ARM_LENGTH - 0.005,
        max_gap=UPPER_ARM_LENGTH + 0.005,
        name="forearm joint sits at upper arm tip",
    )
    ctx.expect_origin_gap(
        terminal_face,
        forearm,
        axis="x",
        min_gap=FOREARM_LENGTH - 0.005,
        max_gap=FOREARM_LENGTH + 0.005,
        name="terminal face sits at forearm tip",
    )

    with ctx.pose(shoulder_joint=0.8, elbow_joint=0.0):
        forearm_pos = ctx.part_world_position(forearm)
        terminal_pos = ctx.part_world_position(terminal_face)
        ok = (
            forearm_pos is not None
            and terminal_pos is not None
            and forearm_pos[1] > 0.18
            and terminal_pos[1] > forearm_pos[1]
            and abs(forearm_pos[2]) < 1e-6
        )
        details = f"forearm={forearm_pos}, terminal={terminal_pos}"
        ctx.check("shoulder swings chain laterally in +Y", ok, details)

    with ctx.pose(shoulder_joint=0.0, elbow_joint=0.9):
        forearm_pos = ctx.part_world_position(forearm)
        terminal_pos = ctx.part_world_position(terminal_face)
        ok = (
            forearm_pos is not None
            and terminal_pos is not None
            and terminal_pos[1] > forearm_pos[1] + 0.10
            and terminal_pos[0] > forearm_pos[0] + 0.10
            and abs(terminal_pos[2] - forearm_pos[2]) < 1e-6
        )
        details = f"forearm={forearm_pos}, terminal={terminal_pos}"
        ctx.check("elbow folds terminal in same horizontal plane", ok, details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
