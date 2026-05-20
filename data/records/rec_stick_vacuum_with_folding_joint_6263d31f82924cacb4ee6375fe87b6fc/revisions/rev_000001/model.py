from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_box_mesh(width: float, depth: float, height: float, radius: float, name: str):
    """A centered rounded-rectangle extrusion, useful for molded plastic shells."""
    profile = rounded_rect_profile(width, depth, radius, corner_segments=8)
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, height), name)


def _capsule_mesh(radius: float, straight_length: float, name: str):
    return mesh_from_geometry(
        CapsuleGeometry(radius, straight_length, radial_segments=32, height_segments=8),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sleek_stick_vacuum")

    graphite = Material("satin_graphite", rgba=(0.055, 0.060, 0.068, 1.0))
    soft_black = Material("soft_black", rgba=(0.008, 0.009, 0.011, 1.0))
    carbon = Material("carbon_black", rgba=(0.015, 0.017, 0.020, 1.0))
    cool_gray = Material("cool_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    gunmetal = Material("gunmetal", rgba=(0.20, 0.22, 0.25, 1.0))
    teal = Material("electric_teal", rgba=(0.00, 0.68, 0.82, 1.0))
    amber_clear = Material("smoky_clear_bin", rgba=(0.55, 0.62, 0.66, 0.38))
    violet = Material("deep_violet", rgba=(0.23, 0.16, 0.48, 1.0))
    bristle = Material("roller_bristles", rgba=(0.96, 0.42, 0.08, 1.0))
    rubber = Material("rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    # Root: the hand-held motor pod.  Its frame is the main folding hinge line
    # where the wand attaches, so the long stick can fold as one child assembly.
    motor = model.part("motor_body")
    motor.visual(
        _capsule_mesh(0.055, 0.185, "motor_pod_mesh"),
        origin=Origin(xyz=(-0.070, 0.0, 0.178), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="motor_pod",
    )
    motor.visual(
        Cylinder(radius=0.052, length=0.190),
        origin=Origin(xyz=(0.030, 0.0, 0.158)),
        material=amber_clear,
        name="dust_bin",
    )
    motor.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.030, 0.0, 0.258)),
        material=gunmetal,
        name="bin_top_ring",
    )
    motor.visual(
        Cylinder(radius=0.054, length=0.018),
        origin=Origin(xyz=(0.030, 0.0, 0.056)),
        material=gunmetal,
        name="bin_bottom_ring",
    )
    motor.visual(
        Box((0.084, 0.095, 0.110)),
        origin=Origin(xyz=(-0.012, 0.0, 0.110)),
        material=graphite,
        name="spine_casting",
    )
    motor.visual(
        Box((0.092, 0.116, 0.032)),
        origin=Origin(xyz=(0.012, 0.0, 0.047)),
        material=graphite,
        name="hinge_bridge",
    )

    # Two external hinge cheeks around a central wand barrel.  The central gap
    # leaves the child barrel clear, making the folding joint visibly believable.
    for idx, y in enumerate((-0.039, 0.039)):
        motor.visual(
            Cylinder(radius=0.044, length=0.022),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"fold_yoke_{idx}",
        )
    motor.visual(
        Cylinder(radius=0.026, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cool_gray,
        name="hinge_pin_cap",
    )

    # Handle loop, all slightly overlapping into the motor casting so it reads
    # as one molded hand unit rather than floating rails.
    motor.visual(
        _capsule_mesh(0.018, 0.205, "rear_grip_mesh"),
        origin=Origin(xyz=(-0.205, 0.0, 0.185)),
        material=soft_black,
        name="rear_grip",
    )
    motor.visual(
        _capsule_mesh(0.017, 0.125, "top_handle_mesh"),
        origin=Origin(xyz=(-0.145, 0.0, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="top_handle",
    )
    motor.visual(
        _capsule_mesh(0.016, 0.145, "lower_handle_mesh"),
        origin=Origin(xyz=(-0.137, 0.0, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="lower_handle",
    )
    motor.visual(
        _capsule_mesh(0.015, 0.160, "front_handle_strut_mesh"),
        origin=Origin(xyz=(-0.060, 0.0, 0.170)),
        material=soft_black,
        name="front_handle_strut",
    )

    # Fine product details: vent slashes and a teal accent rail on the slim pod.
    for i, z in enumerate((0.150, 0.164, 0.178, 0.192, 0.206)):
        motor.visual(
            Box((0.006, 0.018, 0.046)),
            origin=Origin(xyz=(-0.122 + 0.010 * i, -0.049, z), rpy=(0.0, 0.0, -0.30)),
            material=carbon,
            name=f"side_vent_{i}",
        )
    motor.visual(
        Box((0.190, 0.010, 0.010)),
        origin=Origin(xyz=(-0.070, 0.050, 0.210)),
        material=teal,
        name="accent_rail",
    )

    # Parent-side trigger hinge lugs mounted inside the handle loop.
    for idx, y in enumerate((-0.096, -0.054)):
        motor.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(-0.158, y, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"trigger_lug_{idx}",
        )
    motor.visual(
        Box((0.025, 0.055, 0.008)),
        origin=Origin(xyz=(-0.158, -0.075, 0.160)),
        material=gunmetal,
        name="trigger_yoke_bridge",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.033, length=0.048),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="fold_barrel",
    )
    wand.visual(
        Box((0.047, 0.054, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=violet,
        name="upper_collar",
    )
    wand.visual(
        Cylinder(radius=0.014, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, -0.420)),
        material=violet,
        name="carbon_tube",
    )
    wand.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=teal,
        name="upper_trim_ring",
    )
    wand.visual(
        Cylinder(radius=0.022, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.800)),
        material=gunmetal,
        name="lower_socket",
    )

    model.articulation(
        "motor_to_wand",
        ArticulationType.REVOLUTE,
        parent=motor,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=0.0, upper=1.35),
    )

    neck = model.part("neck")
    neck.visual(
        Box((0.052, 0.070, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=gunmetal,
        name="neck_top_block",
    )
    for idx, y in enumerate((-0.030, 0.030)):
        neck.visual(
            Box((0.036, 0.014, 0.075)),
            origin=Origin(xyz=(0.012, y, -0.055)),
            material=gunmetal,
            name=f"neck_fork_{idx}",
        )
    neck.visual(
        Cylinder(radius=0.027, length=0.030),
        origin=Origin(xyz=(0.012, 0.0, -0.092)),
        material=carbon,
        name="swivel_cup",
    )

    model.articulation(
        "wand_to_neck",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, -0.833)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-0.85, upper=0.70),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.023, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=carbon,
        name="swivel_post",
    )
    floor_head.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=gunmetal,
        name="swivel_pedestal",
    )
    floor_head.visual(
        _rounded_box_mesh(0.300, 0.280, 0.044, 0.036, "head_shell_mesh"),
        origin=Origin(xyz=(0.102, 0.0, -0.040)),
        material=graphite,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.014, 0.250, 0.020)),
        origin=Origin(xyz=(0.245, 0.0, -0.052)),
        material=rubber,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.178, 0.012, 0.006)),
        origin=Origin(xyz=(0.150, -0.142, -0.022)),
        material=teal,
        name="led_strip",
    )
    floor_head.visual(
        Box((0.050, 0.220, 0.006)),
        origin=Origin(xyz=(0.128, 0.0, -0.060)),
        material=amber_clear,
        name="brush_window",
    )
    for idx, y in enumerate((-0.116, 0.116)):
        floor_head.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(-0.060, y, -0.064), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"rear_wheel_{idx}",
        )
    for idx, y in enumerate((-0.128, 0.128)):
        floor_head.visual(
            Box((0.040, 0.020, 0.055)),
            origin=Origin(xyz=(0.128, y, -0.075)),
            material=carbon,
            name=f"bearing_block_{idx}",
        )

    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=floor_head,
        origin=Origin(xyz=(0.012, 0.0, -0.106)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.75, upper=0.75),
    )

    brush_roll = model.part("brush_roll")
    brush_roll.visual(
        Cylinder(radius=0.017, length=0.235),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=teal,
        name="roller_core",
    )
    brush_roll.visual(
        Cylinder(radius=0.007, length=0.285),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="roller_axle",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        brush_roll.visual(
            Box((0.010, 0.232, 0.006)),
            origin=Origin(
                xyz=(0.018 * math.cos(angle), 0.0, 0.018 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=bristle,
            name=f"brush_strip_{i}",
        )

    model.articulation(
        "head_to_brush",
        ArticulationType.CONTINUOUS,
        parent=floor_head,
        child=brush_roll,
        origin=Origin(xyz=(0.128, 0.0, -0.088)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.009, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=teal,
        name="trigger_barrel",
    )
    trigger.visual(
        _rounded_box_mesh(0.026, 0.032, 0.082, 0.007, "trigger_paddle_mesh"),
        origin=Origin(xyz=(0.020, 0.0, -0.045), rpy=(0.0, -0.18, 0.0)),
        material=teal,
        name="trigger_paddle",
    )
    model.articulation(
        "motor_to_trigger",
        ArticulationType.REVOLUTE,
        parent=motor,
        child=trigger,
        origin=Origin(xyz=(-0.158, -0.075, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0, lower=0.0, upper=0.42),
    )

    mode_button = model.part("mode_button")
    mode_button.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=teal,
        name="button_cap",
    )
    model.articulation(
        "motor_to_mode_button",
        ArticulationType.PRISMATIC,
        parent=motor,
        child=mode_button,
        origin=Origin(xyz=(0.030, 0.0, 0.268)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.12, lower=0.0, upper=0.006),
    )

    bin_latch = model.part("bin_latch")
    bin_latch.visual(
        _rounded_box_mesh(0.030, 0.014, 0.046, 0.004, "bin_latch_mesh"),
        origin=Origin(xyz=(0.010, 0.0, -0.020), rpy=(0.0, 0.18, 0.0)),
        material=teal,
        name="latch_tab",
    )
    model.articulation(
        "motor_to_bin_latch",
        ArticulationType.REVOLUTE,
        parent=motor,
        child=bin_latch,
        origin=Origin(xyz=(0.088, 0.0, 0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=0.0, upper=0.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    motor = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    neck = object_model.get_part("neck")
    floor_head = object_model.get_part("floor_head")
    brush_roll = object_model.get_part("brush_roll")
    trigger = object_model.get_part("trigger")
    mode_button = object_model.get_part("mode_button")

    fold = object_model.get_articulation("motor_to_wand")
    pitch = object_model.get_articulation("wand_to_neck")
    swivel = object_model.get_articulation("neck_to_head")
    trigger_joint = object_model.get_articulation("motor_to_trigger")
    button_joint = object_model.get_articulation("motor_to_mode_button")

    ctx.allow_overlap(
        motor,
        wand,
        elem_a="hinge_pin_cap",
        elem_b="fold_barrel",
        reason="The folding-joint pin is intentionally captured through the wand barrel.",
    )
    ctx.allow_overlap(
        floor_head,
        neck,
        elem_a="swivel_post",
        elem_b="swivel_cup",
        reason="The floor-head swivel post is intentionally seated inside the neck cup.",
    )
    for idx in (0, 1):
        ctx.allow_overlap(
            brush_roll,
            floor_head,
            elem_a="roller_axle",
            elem_b=f"bearing_block_{idx}",
            reason="The powered brush-roll axle is intentionally captured in the head bearing block.",
        )

    ctx.expect_overlap(
        motor,
        wand,
        axes="y",
        elem_a="hinge_pin_cap",
        elem_b="fold_barrel",
        min_overlap=0.040,
        name="folding hinge pin passes through the wand barrel",
    )
    ctx.expect_overlap(
        wand,
        neck,
        axes="xy",
        elem_a="lower_socket",
        elem_b="neck_top_block",
        min_overlap=0.010,
        name="neck is captured under the wand socket",
    )
    ctx.expect_overlap(
        floor_head,
        brush_roll,
        axes="y",
        elem_a="brush_window",
        elem_b="roller_core",
        min_overlap=0.150,
        name="brush roller spans the floor-head window",
    )
    for idx in (0, 1):
        ctx.expect_overlap(
            brush_roll,
            floor_head,
            axes="y",
            elem_a="roller_axle",
            elem_b=f"bearing_block_{idx}",
            min_overlap=0.010,
            name=f"brush axle remains captured in bearing {idx}",
        )

    rest_head = ctx.part_world_position(floor_head)
    with ctx.pose({fold: 0.95}):
        folded_head = ctx.part_world_position(floor_head)
    ctx.check(
        "main folding joint swings the wand assembly forward",
        rest_head is not None
        and folded_head is not None
        and folded_head[0] > rest_head[0] + 0.40,
        details=f"rest={rest_head}, folded={folded_head}",
    )

    rest_cleaning_angle = ctx.part_world_position(floor_head)
    with ctx.pose({pitch: 0.55}):
        pitched_cleaning_angle = ctx.part_world_position(floor_head)
    ctx.check(
        "floor neck pitch changes cleaning angle",
        rest_cleaning_angle is not None
        and pitched_cleaning_angle is not None
        and abs(pitched_cleaning_angle[0] - rest_cleaning_angle[0]) > 0.02,
        details=f"rest={rest_cleaning_angle}, pitched={pitched_cleaning_angle}",
    )

    rest_button = ctx.part_world_position(mode_button)
    with ctx.pose({button_joint: 0.006}):
        pressed_button = ctx.part_world_position(mode_button)
    ctx.check(
        "mode button depresses into the top pod",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[2] < rest_button[2] - 0.004,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    rest_trigger_box = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_joint: 0.35}):
        pulled_trigger_box = ctx.part_world_aabb(trigger)
    ctx.check(
        "trigger rotates around its hinge pin",
        rest_trigger_box is not None
        and pulled_trigger_box is not None
        and pulled_trigger_box[0][0] < rest_trigger_box[0][0] - 0.006,
        details=f"rest={rest_trigger_box}, pulled={pulled_trigger_box}",
    )

    with ctx.pose({swivel: 0.55}):
        ctx.expect_overlap(
            floor_head,
            neck,
            axes="z",
            elem_a="swivel_post",
            elem_b="swivel_cup",
            min_overlap=0.010,
            name="swivel post remains seated while the head steers",
        )

    return ctx.report()


object_model = build_object_model()
