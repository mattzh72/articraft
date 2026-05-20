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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _plus_profile(span: float, arm: float) -> list[tuple[float, float]]:
    half_span = span * 0.5
    half_arm = arm * 0.5
    return [
        (-half_arm, half_span),
        (half_arm, half_span),
        (half_arm, half_arm),
        (half_span, half_arm),
        (half_span, -half_arm),
        (half_arm, -half_arm),
        (half_arm, -half_span),
        (-half_arm, -half_span),
        (-half_arm, -half_arm),
        (-half_span, -half_arm),
        (-half_span, half_arm),
        (-half_arm, half_arm),
    ]


def _body_shell_mesh():
    return superellipse_side_loft(
        [
            (-0.054, -0.0135, 0.0140, 0.128),
            (-0.026, -0.0175, 0.0188, 0.154),
            (0.000, -0.0190, 0.0202, 0.164),
            (0.028, -0.0165, 0.0180, 0.156),
            (0.054, -0.0125, 0.0138, 0.138),
        ],
        exponents=2.7,
        segments=64,
    )


def _thumb_well_mesh():
    return LatheGeometry(
        [
            (0.0000, -0.0020),
            (0.0200, -0.0020),
            (0.0220, 0.0000),
            (0.0220, 0.0045),
            (0.0180, 0.0062),
            (0.0100, 0.0032),
            (0.0065, 0.0026),
            (0.0000, 0.0026),
            (0.0000, -0.0020),
        ],
        segments=48,
    )


def _thumb_cap_mesh():
    return LatheGeometry(
        [
            (0.0000, 0.0000),
            (0.0080, 0.0000),
            (0.0128, 0.0022),
            (0.0145, 0.0048),
            (0.0120, 0.0098),
            (0.0000, 0.0084),
            (0.0000, 0.0000),
        ],
        segments=48,
    )


def _trigger_body_mesh():
    return superellipse_side_loft(
        [
            (0.004, -0.0036, 0.0024, 0.0315),
            (0.014, -0.0044, 0.0018, 0.0310),
            (0.023, -0.0037, 0.0010, 0.0280),
        ],
        exponents=2.3,
        segments=40,
    )


def _trigger_tip_mesh():
    return superellipse_side_loft(
        [
            (0.021, -0.0031, 0.0008, 0.0275),
            (0.027, -0.0037, 0.0002, 0.0285),
            (0.032, -0.0030, -0.0001, 0.0260),
        ],
        exponents=2.2,
        segments=40,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_gaming_gamepad")

    shell_black = model.material("shell_black", rgba=(0.11, 0.12, 0.13, 1.0))
    shell_graphite = model.material("shell_graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    dpad_grey = model.material("dpad_grey", rgba=(0.30, 0.32, 0.35, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        _mesh("gamepad_body_shell", _body_shell_mesh()),
        material=shell_graphite,
        name="body_shell",
    )
    body.visual(
        Box((0.160, 0.036, 0.008)),
        origin=Origin(xyz=(0.000, -0.002, -0.014)),
        material=shell_black,
        name="underside_band",
    )
    body.visual(
        Box((0.048, 0.015, 0.008)),
        origin=Origin(xyz=(-0.045, 0.051, 0.017)),
        material=shell_black,
        name="left_trigger_saddle",
    )
    body.visual(
        Box((0.048, 0.015, 0.008)),
        origin=Origin(xyz=(0.045, 0.051, 0.017)),
        material=shell_black,
        name="right_trigger_saddle",
    )
    body.visual(
        _mesh("gamepad_left_thumb_well", _thumb_well_mesh()),
        origin=Origin(xyz=(-0.026, -0.014, 0.0128)),
        material=trim_dark,
        name="left_thumb_well",
    )
    body.visual(
        _mesh("gamepad_right_thumb_well", _thumb_well_mesh()),
        origin=Origin(xyz=(0.044, -0.010, 0.0128)),
        material=trim_dark,
        name="right_thumb_well",
    )
    body.visual(
        Cylinder(radius=0.0058, length=0.006),
        origin=Origin(xyz=(-0.026, -0.014, 0.0174)),
        material=trim_dark,
        name="left_thumb_post",
    )
    body.visual(
        Cylinder(radius=0.0058, length=0.006),
        origin=Origin(xyz=(0.044, -0.010, 0.0174)),
        material=trim_dark,
        name="right_thumb_post",
    )
    body.visual(
        _mesh(
            "gamepad_dpad_plate",
            ExtrudeGeometry.from_z0(_plus_profile(0.028, 0.010), 0.0034),
        ),
        origin=Origin(xyz=(-0.056, 0.010, 0.0168)),
        material=dpad_grey,
        name="dpad_plate",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.0042),
        origin=Origin(xyz=(-0.056, 0.010, 0.0190)),
        material=dpad_grey,
        name="dpad_hub",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.164, 0.108, 0.040)),
        mass=0.32,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
    )

    trigger_body_mesh = _mesh("gamepad_trigger_body", _trigger_body_mesh())
    trigger_tip_mesh = _mesh("gamepad_trigger_tip", _trigger_tip_mesh())

    left_thumbstick = model.part("left_thumbstick")
    left_thumbstick.visual(
        Cylinder(radius=0.0045, length=0.013),
        origin=Origin(xyz=(0.000, 0.000, 0.0065)),
        material=trim_dark,
        name="shaft",
    )
    left_thumbstick.visual(
        _mesh("gamepad_thumb_cap", _thumb_cap_mesh()),
        origin=Origin(xyz=(0.000, 0.000, 0.0100)),
        material=rubber_black,
        name="thumb_cap",
    )
    left_thumbstick.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0145, length=0.0184),
        mass=0.018,
        origin=Origin(xyz=(0.000, 0.000, 0.0092)),
    )

    right_thumbstick = model.part("right_thumbstick")
    right_thumbstick.visual(
        Cylinder(radius=0.0045, length=0.013),
        origin=Origin(xyz=(0.000, 0.000, 0.0065)),
        material=trim_dark,
        name="shaft",
    )
    right_thumbstick.visual(
        _mesh("gamepad_thumb_cap_right", _thumb_cap_mesh()),
        origin=Origin(xyz=(0.000, 0.000, 0.0100)),
        material=rubber_black,
        name="thumb_cap",
    )
    right_thumbstick.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0145, length=0.0184),
        mass=0.018,
        origin=Origin(xyz=(0.000, 0.000, 0.0092)),
    )

    left_trigger = model.part("left_trigger")
    left_trigger.visual(
        Cylinder(radius=0.0045, length=0.032),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=trim_dark,
        name="pivot_barrel",
    )
    left_trigger.visual(
        trigger_body_mesh,
        material=trim_dark,
        name="lever_body",
    )
    left_trigger.visual(
        trigger_tip_mesh,
        material=trim_dark,
        name="lever_tip",
    )
    left_trigger.inertial = Inertial.from_geometry(
        Box((0.032, 0.034, 0.012)),
        mass=0.015,
        origin=Origin(xyz=(0.000, 0.013, -0.002)),
    )

    right_trigger = model.part("right_trigger")
    right_trigger.visual(
        Cylinder(radius=0.0045, length=0.032),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=trim_dark,
        name="pivot_barrel",
    )
    right_trigger.visual(
        trigger_body_mesh,
        material=trim_dark,
        name="lever_body",
    )
    right_trigger.visual(
        trigger_tip_mesh,
        material=trim_dark,
        name="lever_tip",
    )
    right_trigger.inertial = Inertial.from_geometry(
        Box((0.032, 0.034, 0.012)),
        mass=0.015,
        origin=Origin(xyz=(0.000, 0.013, -0.002)),
    )

    model.articulation(
        "body_to_left_thumbstick",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_thumbstick,
        origin=Origin(xyz=(-0.026, -0.014, 0.0204)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=16.0),
    )
    model.articulation(
        "body_to_right_thumbstick",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_thumbstick,
        origin=Origin(xyz=(0.044, -0.010, 0.0204)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=16.0),
    )
    model.articulation(
        "body_to_left_trigger",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_trigger,
        origin=Origin(xyz=(-0.045, 0.055, 0.0255)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=0.0,
            upper=0.48,
        ),
    )
    model.articulation(
        "body_to_right_trigger",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_trigger,
        origin=Origin(xyz=(0.045, 0.055, 0.0255)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=0.0,
            upper=0.48,
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

    body = object_model.get_part("body")
    left_thumbstick = object_model.get_part("left_thumbstick")
    right_thumbstick = object_model.get_part("right_thumbstick")
    left_trigger = object_model.get_part("left_trigger")
    right_trigger = object_model.get_part("right_trigger")

    body.get_visual("body_shell")
    body.get_visual("left_thumb_well")
    body.get_visual("right_thumb_well")
    body.get_visual("left_thumb_post")
    body.get_visual("right_thumb_post")
    body.get_visual("dpad_plate")
    body.get_visual("left_trigger_saddle")
    body.get_visual("right_trigger_saddle")

    left_spin = object_model.get_articulation("body_to_left_thumbstick")
    right_spin = object_model.get_articulation("body_to_right_thumbstick")
    left_trigger_joint = object_model.get_articulation("body_to_left_trigger")
    right_trigger_joint = object_model.get_articulation("body_to_right_trigger")

    ctx.check(
        "left thumbstick uses continuous vertical rotation",
        left_spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in left_spin.axis) == (0.0, 0.0, 1.0)
        and left_spin.motion_limits is not None
        and left_spin.motion_limits.lower is None
        and left_spin.motion_limits.upper is None,
        details=str(
            {
                "type": left_spin.joint_type,
                "axis": left_spin.axis,
                "limits": left_spin.motion_limits,
            }
        ),
    )
    ctx.check(
        "right thumbstick uses continuous vertical rotation",
        right_spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in right_spin.axis) == (0.0, 0.0, 1.0)
        and right_spin.motion_limits is not None
        and right_spin.motion_limits.lower is None
        and right_spin.motion_limits.upper is None,
        details=str(
            {
                "type": right_spin.joint_type,
                "axis": right_spin.axis,
                "limits": right_spin.motion_limits,
            }
        ),
    )
    ctx.check(
        "trigger hinges use horizontal x-axis motion",
        left_trigger_joint.joint_type == ArticulationType.REVOLUTE
        and right_trigger_joint.joint_type == ArticulationType.REVOLUTE
        and tuple(round(value, 6) for value in left_trigger_joint.axis) == (-1.0, 0.0, 0.0)
        and tuple(round(value, 6) for value in right_trigger_joint.axis) == (-1.0, 0.0, 0.0),
        details=str(
            {
                "left_axis": left_trigger_joint.axis,
                "right_axis": right_trigger_joint.axis,
            }
        ),
    )

    ctx.expect_contact(
        left_trigger,
        body,
        elem_a="pivot_barrel",
        elem_b="left_trigger_saddle",
        contact_tol=0.0015,
        name="left trigger barrel is supported by the top saddle",
    )
    ctx.expect_contact(
        right_trigger,
        body,
        elem_a="pivot_barrel",
        elem_b="right_trigger_saddle",
        contact_tol=0.0015,
        name="right trigger barrel is supported by the top saddle",
    )
    ctx.expect_overlap(
        left_thumbstick,
        body,
        axes="xy",
        elem_a="shaft",
        elem_b="left_thumb_well",
        min_overlap=0.008,
        name="left thumbstick shaft stays centered over the well",
    )
    ctx.expect_overlap(
        right_thumbstick,
        body,
        axes="xy",
        elem_a="shaft",
        elem_b="right_thumb_well",
        min_overlap=0.008,
        name="right thumbstick shaft stays centered over the well",
    )
    ctx.expect_contact(
        left_thumbstick,
        body,
        elem_a="shaft",
        elem_b="left_thumb_post",
        contact_tol=0.0008,
        name="left thumbstick shaft is carried by the left center post",
    )
    ctx.expect_contact(
        right_thumbstick,
        body,
        elem_a="shaft",
        elem_b="right_thumb_post",
        contact_tol=0.0008,
        name="right thumbstick shaft is carried by the right center post",
    )
    ctx.expect_gap(
        left_thumbstick,
        body,
        axis="z",
        positive_elem="shaft",
        negative_elem="left_thumb_well",
        min_gap=0.001,
        max_gap=0.004,
        name="left thumbstick shaft rises just above the left well opening",
    )
    ctx.expect_gap(
        right_thumbstick,
        body,
        axis="z",
        positive_elem="shaft",
        negative_elem="right_thumb_well",
        min_gap=0.001,
        max_gap=0.004,
        name="right thumbstick shaft rises just above the right well opening",
    )
    ctx.expect_gap(
        left_thumbstick,
        body,
        axis="z",
        positive_elem="thumb_cap",
        negative_elem="left_thumb_well",
        min_gap=0.004,
        max_gap=0.020,
        name="left thumb cap stands proud above the well",
    )
    ctx.expect_gap(
        right_thumbstick,
        body,
        axis="z",
        positive_elem="thumb_cap",
        negative_elem="right_thumb_well",
        min_gap=0.004,
        max_gap=0.020,
        name="right thumb cap stands proud above the well",
    )

    dpad_aabb = ctx.part_element_world_aabb(body, elem="dpad_plate")
    ctx.check(
        "dpad face plate sits on the left front quadrant",
        dpad_aabb is not None and dpad_aabb[1][0] < -0.040 and dpad_aabb[0][2] > 0.016,
        details=f"dpad_aabb={dpad_aabb}",
    )

    left_tip_rest = ctx.part_element_world_aabb(left_trigger, elem="lever_tip")
    right_tip_rest = ctx.part_element_world_aabb(right_trigger, elem="lever_tip")
    with ctx.pose(
        {
            left_trigger_joint: left_trigger_joint.motion_limits.upper,
            right_trigger_joint: right_trigger_joint.motion_limits.upper,
        }
    ):
        left_tip_pressed = ctx.part_element_world_aabb(left_trigger, elem="lever_tip")
        right_tip_pressed = ctx.part_element_world_aabb(right_trigger, elem="lever_tip")
        ctx.check(
            "left trigger pulls downward when actuated",
            left_tip_rest is not None
            and left_tip_pressed is not None
            and left_tip_pressed[1][2] < left_tip_rest[1][2] - 0.004,
            details=f"rest={left_tip_rest}, pressed={left_tip_pressed}",
        )
        ctx.check(
            "right trigger pulls downward when actuated",
            right_tip_rest is not None
            and right_tip_pressed is not None
            and right_tip_pressed[1][2] < right_tip_rest[1][2] - 0.004,
            details=f"rest={right_tip_rest}, pressed={right_tip_pressed}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
