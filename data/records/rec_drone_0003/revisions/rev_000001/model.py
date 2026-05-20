from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return "/"


os.getcwd = _safe_getcwd
pathlib.Path.cwd = classmethod(lambda cls: cls(os.getcwd()))

os.chdir(os.getcwd())

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


BODY_LENGTH = 0.24
BODY_WIDTH = 0.14
BODY_HEIGHT = 0.038
PLATE_THICKNESS = 0.0035
WALL_THICKNESS = 0.0045

HINGE_OUTBOARD = 0.014
HINGE_BRIDGE_LENGTH = 0.032
HINGE_BRIDGE_WIDTH = 0.028
HINGE_BEARING_THICKNESS = 0.004

ARM_STACK_Z = 0.010
ARM_THICKNESS = 0.016
HINGE_BARREL_RADIUS = 0.012
HINGE_YOKE_LENGTH = 0.032
HINGE_YOKE_WIDTH = 0.030
ARM_BEAM_LENGTH = 0.120
ARM_BEAM_WIDTH = 0.022
ARM_BEAM_CENTER_X = 0.088
MOTOR_RADIUS = 0.016
MOTOR_HEIGHT = 0.018
MOTOR_CENTER_X = 0.150

PROP_HUB_RADIUS = 0.011
PROP_HUB_THICKNESS = 0.0045
PROP_BLADE_SPAN = 0.170
PROP_BLADE_CHORD = 0.018
PROP_BLADE_THICKNESS = 0.0026
PROP_BLADE_HALF_LENGTH = 0.078
PROP_BLADE_OFFSET = 0.044

ARM_FOLD_LIMIT = math.radians(78.0)


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw)


def _arm_specs() -> tuple[dict[str, object], ...]:
    raw_specs = (
        ("front_left", 1.0, 1.0, math.radians(45.0), (0.0, 0.0, -1.0), ARM_STACK_Z),
        ("front_right", 1.0, -1.0, -math.radians(45.0), (0.0, 0.0, 1.0), -ARM_STACK_Z),
        ("rear_left", -1.0, 1.0, math.radians(135.0), (0.0, 0.0, -1.0), ARM_STACK_Z),
        ("rear_right", -1.0, -1.0, -math.radians(135.0), (0.0, 0.0, 1.0), -ARM_STACK_Z),
    )
    specs: list[dict[str, object]] = []
    for key, x_sign, y_sign, yaw, axis, z_level in raw_specs:
        offset_x, offset_y = _rotate_xy(HINGE_OUTBOARD, 0.0, yaw)
        specs.append(
            {
                "key": key,
                "x_sign": x_sign,
                "y_sign": y_sign,
                "yaw": yaw,
                "axis": axis,
                "origin": (
                    x_sign * BODY_LENGTH / 2.0 + offset_x,
                    y_sign * BODY_WIDTH / 2.0 + offset_y,
                    z_level,
                ),
                "arm_name": f"{key}_arm",
                "prop_name": f"{key}_propeller",
                "arm_joint": f"body_to_{key}_arm",
                "prop_joint": f"{key}_propeller_spin",
                "bridge_visual": f"{key}_hinge_bridge",
                "bearing_visual": f"{key}_hinge_bearing",
            }
        )
    return tuple(specs)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_quadrotor")

    body_dark = model.material("body_dark", rgba=(0.11, 0.12, 0.14, 1.0))
    body_panel = model.material("body_panel", rgba=(0.17, 0.18, 0.21, 1.0))
    arm_dark = model.material("arm_dark", rgba=(0.16, 0.16, 0.17, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.37, 0.38, 0.41, 1.0))
    prop_black = model.material("prop_black", rgba=(0.05, 0.05, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0 - PLATE_THICKNESS / 2.0)),
        material=body_dark,
        name="top_shell",
    )
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -BODY_HEIGHT / 2.0 + PLATE_THICKNESS / 2.0)),
        material=body_dark,
        name="bottom_shell",
    )
    body.visual(
        Box((BODY_LENGTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, BODY_HEIGHT - 2.0 * PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, BODY_WIDTH / 2.0 - WALL_THICKNESS / 2.0, 0.0)),
        material=body_panel,
        name="left_wall",
    )
    body.visual(
        Box((BODY_LENGTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, BODY_HEIGHT - 2.0 * PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, -BODY_WIDTH / 2.0 + WALL_THICKNESS / 2.0, 0.0)),
        material=body_panel,
        name="right_wall",
    )
    body.visual(
        Box((WALL_THICKNESS, BODY_WIDTH, BODY_HEIGHT - 2.0 * PLATE_THICKNESS)),
        origin=Origin(xyz=(BODY_LENGTH / 2.0 - WALL_THICKNESS / 2.0, 0.0, 0.0)),
        material=body_panel,
        name="front_wall",
    )
    body.visual(
        Box((WALL_THICKNESS, BODY_WIDTH, BODY_HEIGHT - 2.0 * PLATE_THICKNESS)),
        origin=Origin(xyz=(-BODY_LENGTH / 2.0 + WALL_THICKNESS / 2.0, 0.0, 0.0)),
        material=body_panel,
        name="rear_wall",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)),
        mass=0.45,
    )

    for spec in _arm_specs():
        bridge_offset_x, bridge_offset_y = _rotate_xy(-0.002, 0.0, spec["yaw"])
        body.visual(
            Box((HINGE_BRIDGE_LENGTH, HINGE_BRIDGE_WIDTH, HINGE_BEARING_THICKNESS)),
            origin=Origin(
                xyz=(
                    spec["origin"][0] + bridge_offset_x,
                    spec["origin"][1] + bridge_offset_y,
                    0.0,
                ),
                rpy=(0.0, 0.0, spec["yaw"]),
            ),
            material=body_panel,
            name=spec["bridge_visual"],
        )
        body.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS * 0.92, length=HINGE_BEARING_THICKNESS),
            origin=Origin(xyz=(spec["origin"][0], spec["origin"][1], 0.0)),
            material=body_panel,
            name=spec["bearing_visual"],
        )

        arm = model.part(spec["arm_name"])
        arm.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=ARM_THICKNESS),
            material=arm_dark,
            name="hinge_barrel",
        )
        arm.visual(
            Box((HINGE_YOKE_LENGTH, HINGE_YOKE_WIDTH, ARM_THICKNESS)),
            origin=Origin(xyz=(0.014, 0.0, 0.0)),
            material=arm_dark,
            name="hinge_yoke",
        )
        arm.visual(
            Box((ARM_BEAM_LENGTH, ARM_BEAM_WIDTH, ARM_THICKNESS)),
            origin=Origin(xyz=(ARM_BEAM_CENTER_X, 0.0, 0.0)),
            material=arm_dark,
            name="beam",
        )
        arm.visual(
            Cylinder(radius=MOTOR_RADIUS, length=MOTOR_HEIGHT),
            origin=Origin(xyz=(MOTOR_CENTER_X, 0.0, ARM_THICKNESS / 2.0 + MOTOR_HEIGHT / 2.0)),
            material=motor_gray,
            name="motor_housing",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.19, 0.04, 0.035)),
            mass=0.08,
            origin=Origin(xyz=(0.09, 0.0, 0.0)),
        )

        prop = model.part(spec["prop_name"])
        prop.visual(
            Cylinder(radius=PROP_HUB_RADIUS, length=PROP_HUB_THICKNESS),
            material=motor_gray,
            name="hub",
        )
        prop.visual(
            Box((PROP_BLADE_HALF_LENGTH, PROP_BLADE_CHORD, PROP_BLADE_THICKNESS)),
            origin=Origin(xyz=(PROP_BLADE_OFFSET, 0.0, 0.0)),
            material=prop_black,
            name="blade_a",
        )
        prop.visual(
            Box((PROP_BLADE_HALF_LENGTH, PROP_BLADE_CHORD, PROP_BLADE_THICKNESS)),
            origin=Origin(xyz=(-PROP_BLADE_OFFSET, 0.0, 0.0)),
            material=prop_black,
            name="blade_b",
        )
        prop.inertial = Inertial.from_geometry(
            Box((PROP_BLADE_SPAN, PROP_BLADE_CHORD, PROP_HUB_THICKNESS)),
            mass=0.014,
        )

        model.articulation(
            spec["arm_joint"],
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=spec["origin"], rpy=(0.0, 0.0, spec["yaw"])),
            axis=spec["axis"],
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.5,
                lower=0.0,
                upper=ARM_FOLD_LIMIT,
            ),
        )
        model.articulation(
            spec["prop_joint"],
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=prop,
            origin=Origin(
                xyz=(
                    MOTOR_CENTER_X,
                    0.0,
                    ARM_THICKNESS / 2.0 + MOTOR_HEIGHT + PROP_HUB_THICKNESS / 2.0,
                )
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.12, velocity=60.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    body = object_model.get_part("body")
    top_shell = body.get_visual("top_shell")
    bottom_shell = body.get_visual("bottom_shell")
    specs = _arm_specs()

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=32)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.expect_gap(
        body,
        body,
        axis="z",
        min_gap=BODY_HEIGHT - 2.0 * PLATE_THICKNESS - 0.001,
        max_gap=BODY_HEIGHT - 2.0 * PLATE_THICKNESS + 0.001,
        positive_elem=top_shell,
        negative_elem=bottom_shell,
        name="body_shell_hollow_depth",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="xy",
        min_overlap=0.10,
        elem_a=top_shell,
        elem_b=bottom_shell,
        name="body_shell_top_bottom_registration",
    )

    body_aabb = ctx.part_world_aabb(body)
    body_pos = ctx.part_world_position(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Body AABB unavailable.")
    ctx.check("body_position_present", body_pos is not None, "Body position unavailable.")

    for spec in specs:
        arm = object_model.get_part(spec["arm_name"])
        prop = object_model.get_part(spec["prop_name"])
        arm_joint = object_model.get_articulation(spec["arm_joint"])
        prop_joint = object_model.get_articulation(spec["prop_joint"])

        hinge_yoke = arm.get_visual("hinge_yoke")
        hinge_barrel = arm.get_visual("hinge_barrel")
        motor_housing = arm.get_visual("motor_housing")
        prop_hub = prop.get_visual("hub")
        hinge_bridge = body.get_visual(spec["bridge_visual"])
        hinge_bearing = body.get_visual(spec["bearing_visual"])

        ctx.check(
            f"{spec['arm_joint']}_is_revolute",
            arm_joint.joint_type == ArticulationType.REVOLUTE,
            f"{spec['arm_joint']} should be revolute.",
        )
        ctx.check(
            f"{spec['arm_joint']}_axis",
            tuple(round(value, 3) for value in arm_joint.axis) == spec["axis"],
            f"Expected axis {spec['axis']}, got {arm_joint.axis}.",
        )
        arm_limits = arm_joint.motion_limits
        ctx.check(
            f"{spec['arm_joint']}_limits",
            arm_limits is not None
            and arm_limits.lower == 0.0
            and arm_limits.upper is not None
            and abs(arm_limits.upper - ARM_FOLD_LIMIT) < 1e-6,
            f"Unexpected fold limits on {spec['arm_joint']}.",
        )
        ctx.check(
            f"{spec['prop_joint']}_is_continuous",
            prop_joint.joint_type == ArticulationType.CONTINUOUS,
            f"{spec['prop_joint']} should be continuous.",
        )
        ctx.check(
            f"{spec['prop_joint']}_axis",
            tuple(round(value, 3) for value in prop_joint.axis) == (0.0, 0.0, 1.0),
            f"Expected vertical propeller axis, got {prop_joint.axis}.",
        )

        with ctx.pose({arm_joint: 0.0, prop_joint: 0.0}):
            ctx.expect_contact(
                arm,
                body,
                elem_a=hinge_yoke,
                elem_b=hinge_bridge,
                name=f"{spec['key']}_hinge_bridge_contact",
            )
            ctx.expect_contact(
                arm,
                body,
                elem_a=hinge_barrel,
                elem_b=hinge_bearing,
                name=f"{spec['key']}_hinge_bearing_contact",
            )
            ctx.expect_contact(
                prop,
                arm,
                elem_a=prop_hub,
                elem_b=motor_housing,
                name=f"{spec['key']}_prop_motor_contact",
            )
            ctx.expect_overlap(
                prop,
                arm,
                axes="xy",
                min_overlap=0.018,
                elem_a=prop_hub,
                elem_b=motor_housing,
                name=f"{spec['key']}_prop_motor_alignment",
            )

        rest_pos = ctx.part_world_position(prop)
        ctx.check(
            f"{spec['key']}_rest_prop_position_present",
            rest_pos is not None,
            f"{spec['prop_name']} world position unavailable.",
        )
        if body_aabb is not None and rest_pos is not None:
            x_ok = (
                rest_pos[0] > body_aabb[1][0] + 0.035
                if spec["x_sign"] > 0.0
                else rest_pos[0] < body_aabb[0][0] - 0.035
            )
            y_ok = (
                rest_pos[1] > body_aabb[1][1] + 0.035
                if spec["y_sign"] > 0.0
                else rest_pos[1] < body_aabb[0][1] - 0.035
            )
            ctx.check(
                f"{spec['key']}_prop_outboard_at_rest",
                x_ok and y_ok,
                f"{spec['prop_name']} should sit outside the body corner at rest, got {rest_pos}.",
            )

        with ctx.pose({arm_joint: ARM_FOLD_LIMIT, prop_joint: 1.1}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{spec['key']}_fold_pose_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{spec['key']}_fold_pose_no_floating")
            ctx.expect_contact(
                arm,
                body,
                elem_a=hinge_yoke,
                elem_b=hinge_bridge,
                name=f"{spec['key']}_hinge_bridge_contact_folded",
            )
            ctx.expect_contact(
                prop,
                arm,
                elem_a=prop_hub,
                elem_b=motor_housing,
                name=f"{spec['key']}_prop_motor_contact_folded",
            )
            folded_pos = ctx.part_world_position(prop)
            ctx.check(
                f"{spec['key']}_folded_prop_position_present",
                folded_pos is not None,
                f"{spec['prop_name']} folded position unavailable.",
            )
            if body_pos is not None and rest_pos is not None and folded_pos is not None:
                rest_radius = math.hypot(rest_pos[0] - body_pos[0], rest_pos[1] - body_pos[1])
                folded_radius = math.hypot(folded_pos[0] - body_pos[0], folded_pos[1] - body_pos[1])
                if spec["x_sign"] > 0.0:
                    inward_axis_ok = (
                        abs(folded_pos[1] - body_pos[1]) < BODY_WIDTH * 0.35
                        and abs(folded_pos[1] - body_pos[1]) < abs(rest_pos[1] - body_pos[1]) - 0.12
                    )
                else:
                    inward_axis_ok = (
                        abs(folded_pos[0] - body_pos[0]) < BODY_LENGTH * 0.45
                        and abs(folded_pos[0] - body_pos[0]) < abs(rest_pos[0] - body_pos[0]) - 0.12
                    )
                ctx.check(
                    f"{spec['key']}_folds_inward",
                    folded_radius < rest_radius - 0.035 and inward_axis_ok,
                    (
                        f"{spec['arm_name']} should move inward when folded "
                        f"(rest {rest_radius:.3f} m, folded {folded_radius:.3f} m; "
                        f"rest={rest_pos}, folded={folded_pos})."
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
