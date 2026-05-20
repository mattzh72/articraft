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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_remote")

    shell_dark = model.material("shell_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    shell_mid = model.material("shell_mid", rgba=(0.21, 0.22, 0.24, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")

    center_shell = ExtrudeGeometry(
        rounded_rect_profile(0.132, 0.072, 0.014, corner_segments=8),
        0.026,
        center=True,
        cap=True,
    )
    body.visual(
        _mesh("remote_center_shell", center_shell),
        material=shell_mid,
        name="center_shell",
    )

    grip_profile = rounded_rect_profile(0.034, 0.046, 0.010, corner_segments=6)
    left_grip_path = [
        (-0.040, 0.016, -0.002),
        (-0.050, 0.000, -0.006),
        (-0.061, -0.022, -0.015),
        (-0.073, -0.046, -0.029),
    ]
    body.visual(
        _mesh(
            "remote_left_grip",
            sweep_profile_along_spline(
                left_grip_path,
                profile=grip_profile,
                samples_per_segment=16,
                cap_profile=True,
                up_hint=(0.0, 0.0, 1.0),
            ),
        ),
        material=shell_mid,
        name="left_grip_shell",
    )
    body.visual(
        _mesh(
            "remote_right_grip",
            sweep_profile_along_spline(
                _mirror_x(left_grip_path),
                profile=grip_profile,
                samples_per_segment=16,
                cap_profile=True,
                up_hint=(0.0, 0.0, 1.0),
            ),
        ),
        material=shell_mid,
        name="right_grip_shell",
    )

    body.visual(
        Box((0.124, 0.074, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=shell_dark,
        name="top_plate",
    )
    body.visual(
        Box((0.032, 0.024, 0.008)),
        origin=Origin(xyz=(-0.055, 0.026, 0.015)),
        material=shell_dark,
        name="left_shoulder_pad",
    )
    body.visual(
        Box((0.032, 0.024, 0.008)),
        origin=Origin(xyz=(0.055, 0.026, 0.015)),
        material=shell_dark,
        name="right_shoulder_pad",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.160, 0.126, 0.072)),
        mass=0.48,
        origin=Origin(xyz=(0.0, -0.006, -0.001)),
    )

    def add_thumbstick(name: str) -> None:
        stick = model.part(name)
        stick.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=control_black,
            name="stick_base",
        )
        stick.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
            material=control_black,
            name="stick_stem",
        )
        stick.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.021)),
            material=control_black,
            name="stick_cap",
        )
        stick.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(0.0, 0.0, 0.028)),
            material=control_black,
            name="stick_top",
        )
        stick.inertial = Inertial.from_geometry(
            Box((0.040, 0.040, 0.045)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, 0.020)),
        )

    def add_trigger(name: str) -> None:
        trigger = model.part(name)
        trigger.visual(
            Cylinder(radius=0.0035, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=control_black,
            name="trigger_barrel",
        )
        trigger.visual(
            Box((0.012, 0.010, 0.008)),
            origin=Origin(xyz=(0.0, 0.007, -0.002)),
            material=control_black,
            name="trigger_root",
        )
        trigger.visual(
            _mesh(
                f"{name}_trigger_shell",
                sweep_profile_along_spline(
                    [
                        (0.0, 0.004, -0.002),
                        (0.0, 0.012, -0.005),
                        (0.0, 0.024, -0.011),
                        (0.0, 0.034, -0.017),
                    ],
                    profile=rounded_rect_profile(0.016, 0.010, 0.0035, corner_segments=6),
                    samples_per_segment=14,
                    cap_profile=True,
                    up_hint=(1.0, 0.0, 0.0),
                ),
            ),
            origin=Origin(),
            material=control_black,
            name="trigger_spine",
        )
        trigger.visual(
            Cylinder(radius=0.008, length=0.016),
            origin=Origin(xyz=(0.0, 0.025, -0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=control_black,
            name="trigger_body",
        )
        trigger.visual(
            Cylinder(radius=0.007, length=0.014),
            origin=Origin(xyz=(0.0, 0.036, -0.019), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=control_black,
            name="trigger_tip",
        )
        trigger.inertial = Inertial.from_geometry(
            Box((0.020, 0.050, 0.020)),
            mass=0.025,
            origin=Origin(xyz=(0.0, 0.022, -0.008)),
        )

    add_thumbstick("left_thumbstick")
    add_thumbstick("right_thumbstick")
    add_trigger("left_trigger")
    add_trigger("right_trigger")

    model.articulation(
        "body_to_left_thumbstick",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="left_thumbstick",
        origin=Origin(xyz=(-0.035, -0.005, 0.017)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )
    model.articulation(
        "body_to_right_thumbstick",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="right_thumbstick",
        origin=Origin(xyz=(0.035, -0.005, 0.017)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )
    model.articulation(
        "body_to_left_trigger",
        ArticulationType.REVOLUTE,
        parent=body,
        child="left_trigger",
        origin=Origin(xyz=(-0.055, 0.034, 0.0225)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=0.65,
        ),
    )
    model.articulation(
        "body_to_right_trigger",
        ArticulationType.REVOLUTE,
        parent=body,
        child="right_trigger",
        origin=Origin(xyz=(0.055, 0.034, 0.0225)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=0.65,
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

    def resolve_part(name: str):
        try:
            return object_model.get_part(name)
        except Exception:
            return None

    def resolve_joint(name: str):
        try:
            return object_model.get_articulation(name)
        except Exception:
            return None

    required_part_names = (
        "body",
        "left_thumbstick",
        "right_thumbstick",
        "left_trigger",
        "right_trigger",
    )
    required_joint_names = (
        "body_to_left_thumbstick",
        "body_to_right_thumbstick",
        "body_to_left_trigger",
        "body_to_right_trigger",
    )

    parts = {name: resolve_part(name) for name in required_part_names}
    joints = {name: resolve_joint(name) for name in required_joint_names}

    missing_parts = [name for name, part in parts.items() if part is None]
    missing_joints = [name for name, joint in joints.items() if joint is None]

    ctx.check(
        "prompt parts and joints exist",
        not missing_parts and not missing_joints,
        details=f"missing_parts={missing_parts}, missing_joints={missing_joints}",
    )
    if missing_parts or missing_joints:
        return ctx.report()

    body = parts["body"]
    left_thumbstick = parts["left_thumbstick"]
    right_thumbstick = parts["right_thumbstick"]
    left_trigger = parts["left_trigger"]
    right_trigger = parts["right_trigger"]

    left_thumb_joint = joints["body_to_left_thumbstick"]
    right_thumb_joint = joints["body_to_right_thumbstick"]
    left_trigger_joint = joints["body_to_left_trigger"]
    right_trigger_joint = joints["body_to_right_trigger"]

    thumbstick_joint_ok = all(
        joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(joint.axis) == (0.0, 0.0, 1.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower is None
        and joint.motion_limits.upper is None
        for joint in (left_thumb_joint, right_thumb_joint)
    )
    ctx.check(
        "thumbsticks spin continuously about vertical axes",
        thumbstick_joint_ok,
        details=(
            f"left_axis={left_thumb_joint.axis}, right_axis={right_thumb_joint.axis}, "
            f"left_limits={left_thumb_joint.motion_limits}, right_limits={right_thumb_joint.motion_limits}"
        ),
    )

    trigger_joint_ok = all(
        joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(joint.axis) == (-1.0, 0.0, 0.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower == 0.0
        and joint.motion_limits.upper is not None
        and joint.motion_limits.upper >= 0.5
        for joint in (left_trigger_joint, right_trigger_joint)
    )
    ctx.check(
        "trigger levers hinge on horizontal axes at the grips",
        trigger_joint_ok,
        details=(
            f"left_axis={left_trigger_joint.axis}, right_axis={right_trigger_joint.axis}, "
            f"left_limits={left_trigger_joint.motion_limits}, right_limits={right_trigger_joint.motion_limits}"
        ),
    )

    ctx.expect_origin_gap(
        right_thumbstick,
        left_thumbstick,
        axis="x",
        min_gap=0.060,
        name="thumbsticks are split across the remote face",
    )

    ctx.expect_gap(
        left_thumbstick,
        body,
        axis="z",
        positive_elem="stick_base",
        negative_elem="top_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="left thumbstick base sits on the top plate",
    )
    ctx.expect_overlap(
        left_thumbstick,
        body,
        axes="xy",
        elem_a="stick_base",
        elem_b="top_plate",
        min_overlap=0.020,
        name="left thumbstick overlaps the top plate footprint",
    )
    ctx.expect_gap(
        right_thumbstick,
        body,
        axis="z",
        positive_elem="stick_base",
        negative_elem="top_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="right thumbstick base sits on the top plate",
    )
    ctx.expect_overlap(
        right_thumbstick,
        body,
        axes="xy",
        elem_a="stick_base",
        elem_b="top_plate",
        min_overlap=0.020,
        name="right thumbstick overlaps the top plate footprint",
    )

    ctx.expect_contact(
        left_trigger,
        body,
        elem_a="trigger_barrel",
        elem_b="left_shoulder_pad",
        contact_tol=0.001,
        name="left trigger barrel is seated on the left shoulder pad",
    )
    ctx.expect_contact(
        right_trigger,
        body,
        elem_a="trigger_barrel",
        elem_b="right_shoulder_pad",
        contact_tol=0.001,
        name="right trigger barrel is seated on the right shoulder pad",
    )

    rest_left_tip = ctx.part_element_world_aabb(left_trigger, elem="trigger_tip")
    rest_right_tip = ctx.part_element_world_aabb(right_trigger, elem="trigger_tip")
    upper_left = left_trigger_joint.motion_limits.upper
    upper_right = right_trigger_joint.motion_limits.upper

    with ctx.pose({left_trigger_joint: upper_left, right_trigger_joint: upper_right}):
        pressed_left_tip = ctx.part_element_world_aabb(left_trigger, elem="trigger_tip")
        pressed_right_tip = ctx.part_element_world_aabb(right_trigger, elem="trigger_tip")

    ctx.check(
        "trigger levers rotate downward when pressed",
        rest_left_tip is not None
        and rest_right_tip is not None
        and pressed_left_tip is not None
        and pressed_right_tip is not None
        and pressed_left_tip[0][2] < rest_left_tip[0][2] - 0.008
        and pressed_right_tip[0][2] < rest_right_tip[0][2] - 0.008,
        details=(
            f"rest_left={rest_left_tip}, pressed_left={pressed_left_tip}, "
            f"rest_right={rest_right_tip}, pressed_right={pressed_right_tip}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
