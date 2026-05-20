from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq
# from sdk import mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="speedlight_flash")

    model.material("body_plastic", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("panel_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    model.material("screen_dark", rgba=(0.08, 0.10, 0.12, 1.0))
    model.material("lamp_clear", rgba=(0.86, 0.88, 0.90, 1.0))
    model.material("shoe_metal", rgba=(0.55, 0.56, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.036, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material="body_plastic",
        name="lower_housing",
    )
    body.visual(
        Box((0.046, 0.058, 0.084)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material="body_plastic",
        name="battery_body",
    )
    body.visual(
        Box((0.038, 0.048, 0.014)),
        origin=Origin(xyz=(0.001, 0.0, 0.111)),
        material="body_plastic",
        name="top_cap",
    )
    body.visual(
        Box((0.0015, 0.022, 0.016)),
        origin=Origin(xyz=(-0.02375, 0.0, 0.078)),
        material="screen_dark",
        name="screen",
    )
    body.visual(
        Box((0.012, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material="shoe_metal",
        name="shoe_stem",
    )
    body.visual(
        Box((0.016, 0.022, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material="shoe_metal",
        name="shoe_block",
    )
    body.visual(
        Box((0.022, 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material="shoe_metal",
        name="shoe_foot",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="body_plastic",
        name="swivel_pedestal",
    )
    neck.visual(
        Box((0.014, 0.026, 0.030)),
        origin=Origin(xyz=(-0.001, 0.0, 0.020)),
        material="body_plastic",
        name="neck_column",
    )
    neck.visual(
        Box((0.010, 0.076, 0.006)),
        origin=Origin(xyz=(0.002, 0.0, 0.034)),
        material="body_plastic",
        name="yoke_bridge",
    )
    neck.visual(
        Box((0.012, 0.004, 0.024)),
        origin=Origin(xyz=(0.004, 0.038, 0.023)),
        material="body_plastic",
        name="yoke_arm_0",
    )
    neck.visual(
        Box((0.012, 0.004, 0.024)),
        origin=Origin(xyz=(0.004, -0.038, 0.023)),
        material="body_plastic",
        name="yoke_arm_1",
    )

    head = model.part("head")
    head.visual(
        Box((0.056, 0.072, 0.036)),
        origin=Origin(xyz=(0.028, 0.0, 0.020)),
        material="body_plastic",
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.066, 0.028)),
        origin=Origin(xyz=(0.058, 0.0, 0.020)),
        material="lamp_clear",
        name="lamp_panel",
    )
    head.visual(
        Box((0.016, 0.064, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, 0.006)),
        material="body_plastic",
        name="hinge_base",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.001, 0.0, 0.118)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-1.8,
            upper=1.8,
        ),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.004, 0.0, 0.034)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-0.25,
            upper=1.35,
        ),
    )

    button_specs = (
        ("mode_button", (0.0022, 0.015, 0.005), (-0.023, 0.0, 0.094)),
        ("pilot_button", (0.0022, 0.008, 0.008), (-0.023, -0.017, 0.074)),
        ("plus_button", (0.0022, 0.008, 0.008), (-0.023, 0.017, 0.076)),
        ("minus_button", (0.0022, 0.008, 0.008), (-0.023, 0.017, 0.060)),
        ("menu_button", (0.0022, 0.010, 0.006), (-0.023, -0.015, 0.058)),
        ("ok_button", (0.0022, 0.010, 0.006), (-0.023, 0.0, 0.056)),
    )
    for button_name, size, joint_xyz in button_specs:
        button = model.part(button_name)
        button.visual(
            Box(size),
            origin=Origin(xyz=(-0.5 * size[0], 0.0, 0.0)),
            material="panel_dark",
            name="button_cap",
        )
        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=joint_xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.2,
                velocity=0.03,
                lower=0.0,
                upper=0.0015,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")
    button_names = (
        "mode_button",
        "pilot_button",
        "plus_button",
        "minus_button",
        "menu_button",
        "ok_button",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.expect_contact(neck, body, name="neck seats on body")
    ctx.expect_contact(head, neck, name="head is carried by yoke")
    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.020,
        name="head stays clearly above battery body",
    )

    rest_head_center = aabb_center(ctx.part_element_world_aabb(head, elem="head_shell"))
    with ctx.pose({swivel: 1.4}):
        swivel_head_center = aabb_center(ctx.part_element_world_aabb(head, elem="head_shell"))
    ctx.check(
        "swivel turns head sideways",
        rest_head_center is not None
        and swivel_head_center is not None
        and abs(swivel_head_center[1] - rest_head_center[1]) > 0.020,
        details=f"rest={rest_head_center}, swivel={swivel_head_center}",
    )

    rest_lamp_center = aabb_center(ctx.part_element_world_aabb(head, elem="lamp_panel"))
    with ctx.pose({tilt: 1.1}):
        tilt_head_center = aabb_center(ctx.part_element_world_aabb(head, elem="lamp_panel"))
    ctx.check(
        "tilt lifts lamp head",
        rest_lamp_center is not None
        and tilt_head_center is not None
        and tilt_head_center[2] > rest_lamp_center[2] + 0.030,
        details=f"rest={rest_lamp_center}, tilt={tilt_head_center}",
    )

    rest_button_positions = {
        name: ctx.part_world_position(object_model.get_part(name)) for name in button_names
    }
    for button_name in button_names:
        button = object_model.get_part(button_name)
        joint = object_model.get_articulation(f"body_to_{button_name}")
        limits = joint.motion_limits

        ctx.expect_gap(
            body,
            button,
            axis="x",
            max_gap=0.0002,
            max_penetration=0.0,
            positive_elem="battery_body",
            name=f"{button_name} seats on rear shell",
        )
        ctx.expect_overlap(
            body,
            button,
            axes="yz",
            min_overlap=0.004,
            name=f"{button_name} stays within panel footprint",
        )

        pressed_q = 0.0 if limits is None or limits.upper is None else limits.upper
        with ctx.pose({joint: pressed_q}):
            pressed_position = ctx.part_world_position(button)
            button_moved = (
                rest_button_positions[button_name] is not None
                and pressed_position is not None
                and pressed_position[0] > rest_button_positions[button_name][0] + 0.001
            )
            others_static = True
            drift_details = []
            for other_name in button_names:
                if other_name == button_name:
                    continue
                other_rest = rest_button_positions[other_name]
                other_pressed = ctx.part_world_position(object_model.get_part(other_name))
                if (
                    other_rest is None
                    or other_pressed is None
                    or abs(other_pressed[0] - other_rest[0]) > 1e-6
                ):
                    others_static = False
                    drift_details.append((other_name, other_rest, other_pressed))
            ctx.check(
                f"{button_name} presses independently",
                button_moved and others_static,
                details=(
                    f"moved={pressed_position}, rest={rest_button_positions[button_name]}, "
                    f"other_drift={drift_details}"
                ),
            )

    return ctx.report()


object_model = build_object_model()
