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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    dark_body = model.material("dark_body", rgba=(0.20, 0.22, 0.24, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    silver = model.material("silver", rgba=(0.72, 0.74, 0.76, 1.0))
    accent = model.material("accent", rgba=(0.68, 0.13, 0.15, 1.0))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Cylinder(radius=0.065, length=0.24),
        origin=Origin(xyz=(0.03, 0.0, 0.20), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_body,
        name="motor_shell",
    )
    motor_body.visual(
        Cylinder(radius=0.046, length=0.15),
        origin=Origin(xyz=(0.16, 0.0, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="dust_bin",
    )
    motor_body.visual(
        Box((0.055, 0.10, 0.30)),
        origin=Origin(xyz=(-0.035, 0.0, 0.20)),
        material=dark_body,
        name="handle_spine",
    )
    motor_body.visual(
        Box((0.11, 0.09, 0.09)),
        origin=Origin(xyz=(-0.08, 0.0, 0.055)),
        material=graphite,
        name="battery_pack",
    )
    motor_body.visual(
        Box((0.085, 0.075, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=accent,
        name="body_collar",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.37, 0.12, 0.36)),
        mass=2.9,
        origin=Origin(xyz=(0.03, 0.0, 0.18)),
    )

    wand = model.part("wand")
    wand.visual(
        Box((0.05, 0.05, 0.054)),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=accent,
        name="wand_knuckle",
    )
    wand.visual(
        Cylinder(radius=0.017, length=0.64),
        origin=Origin(xyz=(0.0, 0.0, -0.37)),
        material=silver,
        name="wand_tube",
    )
    wand.visual(
        Box((0.045, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.655)),
        material=graphite,
        name="wand_socket",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.06, 0.06, 0.73)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, -0.365)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Box((0.04, 0.06, 0.045)),
        origin=Origin(xyz=(0.04, 0.0, -0.0225)),
        material=graphite,
        name="head_neck",
    )
    floor_head.visual(
        Box((0.29, 0.115, 0.036)),
        origin=Origin(xyz=(0.145, 0.0, -0.028)),
        material=dark_body,
        name="head_shell",
    )
    floor_head.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.18, 0.0, -0.036), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="brush_bar",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.30, 0.12, 0.055)),
        mass=1.1,
        origin=Origin(xyz=(0.145, 0.0, -0.028)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(),
        # The straight wand extends along local -Z from the fold line.
        # Using -Y makes positive q fold the wand forward and upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.68)),
        # The floor head extends forward along +X from the hinge.
        # Using -Y makes positive q lift the nose of the head upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=math.radians(-30.0),
            upper=math.radians(35.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    head_joint = object_model.get_articulation("wand_to_floor_head")

    ctx.expect_contact(
        motor_body,
        wand,
        elem_a="body_collar",
        elem_b="wand_knuckle",
        name="fold knuckle seats against the body collar",
    )
    ctx.expect_contact(
        wand,
        floor_head,
        elem_a="wand_socket",
        elem_b="head_neck",
        name="floor head neck seats against the wand socket",
    )

    rest_head_aabb = ctx.part_world_aabb(floor_head)
    with ctx.pose({fold_joint: fold_joint.motion_limits.upper}):
        folded_head_aabb = ctx.part_world_aabb(floor_head)

    fold_ok = (
        rest_head_aabb is not None
        and folded_head_aabb is not None
        and folded_head_aabb[1][0] > rest_head_aabb[1][0] + 0.45
        and folded_head_aabb[0][2] > rest_head_aabb[0][2] + 0.30
    )
    ctx.check(
        "fold joint swings the wand upward for storage",
        fold_ok,
        details=f"rest_head_aabb={rest_head_aabb}, folded_head_aabb={folded_head_aabb}",
    )

    rest_shell = ctx.part_element_world_aabb(floor_head, elem="head_shell")
    with ctx.pose({head_joint: head_joint.motion_limits.upper}):
        pitched_shell = ctx.part_element_world_aabb(floor_head, elem="head_shell")

    rest_shell_center_z = None
    pitched_shell_center_z = None
    if rest_shell is not None:
        rest_shell_center_z = 0.5 * (rest_shell[0][2] + rest_shell[1][2])
    if pitched_shell is not None:
        pitched_shell_center_z = 0.5 * (pitched_shell[0][2] + pitched_shell[1][2])

    ctx.check(
        "floor head pitches nose upward around its hinge",
        rest_shell_center_z is not None
        and pitched_shell_center_z is not None
        and pitched_shell_center_z > rest_shell_center_z + 0.04,
        details=(
            f"rest_shell={rest_shell}, pitched_shell={pitched_shell}, "
            f"rest_center_z={rest_shell_center_z}, pitched_center_z={pitched_shell_center_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
