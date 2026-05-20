from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _bottle_shell() -> LatheGeometry:
    """Thin blow-molded round bottle with a shoulder and molded neck finish."""

    outer = [
        (0.024, 0.000),
        (0.037, 0.010),
        (0.043, 0.032),
        (0.043, 0.135),
        (0.039, 0.158),
        (0.026, 0.181),
        (0.017, 0.194),
        (0.017, 0.214),
    ]
    inner = [
        (0.019, 0.006),
        (0.033, 0.014),
        (0.039, 0.034),
        (0.039, 0.132),
        (0.035, 0.153),
        (0.022, 0.176),
        (0.013, 0.191),
        (0.013, 0.211),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=64,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_trigger_spray_bottle")

    translucent_hdpe = model.material("translucent_hdpe", rgba=(0.72, 0.88, 0.96, 0.42))
    white_pp = model.material("white_polypropylene", rgba=(0.94, 0.94, 0.90, 1.0))
    orange_pp = model.material("orange_polypropylene", rgba=(1.0, 0.45, 0.10, 1.0))
    gray_acetal = model.material("gray_acetal", rgba=(0.55, 0.58, 0.58, 1.0))
    dark_seal = model.material("black_elastomer", rgba=(0.02, 0.025, 0.025, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        _mesh(_bottle_shell(), "thin_bottle_shell"),
        material=translucent_hdpe,
        name="thin_bottle_shell",
    )
    bottle.visual(
        Cylinder(radius=0.026, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=white_pp,
        name="screw_closure",
    )
    # Low-cost molded knurls are represented as broad ribs fused into the closure.
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        bottle.visual(
            Box((0.006, 0.010, 0.024)),
            origin=Origin(
                xyz=(0.027 * math.cos(angle), 0.027 * math.sin(angle), 0.215),
                rpy=(0.0, 0.0, angle),
            ),
            material=white_pp,
            name=f"closure_rib_{index}",
        )
    bottle.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.232)),
        material=white_pp,
        name="neck_socket",
    )
    bottle.visual(
        Box((0.125, 0.038, 0.030)),
        origin=Origin(xyz=(0.060, 0.0, 0.240)),
        material=white_pp,
        name="one_piece_head",
    )
    bottle.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.145, 0.0, 0.240), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_pp,
        name="straight_nozzle",
    )
    bottle.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(0.169, 0.0, 0.240), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_seal,
        name="spray_orifice",
    )
    bottle.visual(
        Cylinder(radius=0.0072, length=0.060),
        origin=Origin(xyz=(0.102, 0.0, 0.209), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gray_acetal,
        name="pump_sleeve",
    )
    bottle.visual(
        Box((0.064, 0.011, 0.016)),
        origin=Origin(xyz=(0.102, 0.0, 0.219)),
        material=white_pp,
        name="pump_web",
    )
    # A molded side fork captures the trigger hub while staying clear of it.
    for index, y in enumerate((-0.018, 0.018)):
        bottle.visual(
            Box((0.017, 0.006, 0.046)),
            origin=Origin(xyz=(0.055, y, 0.204)),
            material=white_pp,
            name=f"trigger_fork_{index}",
        )
    # Side snap bosses and a rear clamp shelf make the snap/clamp assembly order legible.
    for index, y in enumerate((-0.023, 0.023)):
        bottle.visual(
            Box((0.025, 0.006, 0.008)),
            origin=Origin(xyz=(0.010, y, 0.226)),
            material=white_pp,
            name=f"snap_lug_{index}",
        )
    bottle.visual(
        Box((0.034, 0.040, 0.006)),
        origin=Origin(xyz=(-0.010, 0.0, 0.224)),
        material=white_pp,
        name="clamp_shelf",
    )
    bottle.visual(
        Cylinder(radius=0.0025, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=gray_acetal,
        name="dip_tube",
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=orange_pp,
        name="pivot_hub",
    )
    trigger.visual(
        Box((0.022, 0.016, 0.014)),
        origin=Origin(xyz=(0.001, 0.0, 0.010)),
        material=orange_pp,
        name="pump_cam",
    )
    trigger.visual(
        Box((0.015, 0.018, 0.083)),
        origin=Origin(xyz=(0.020, 0.0, -0.041), rpy=(0.0, -0.44, 0.0)),
        material=orange_pp,
        name="finger_lever",
    )
    trigger.visual(
        Box((0.029, 0.024, 0.019)),
        origin=Origin(xyz=(0.040, 0.0, -0.082), rpy=(0.0, -0.35, 0.0)),
        material=orange_pp,
        name="finger_pad",
    )

    pump_plunger = model.part("pump_plunger")
    pump_plunger.visual(
        Cylinder(radius=0.0042, length=0.054),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gray_acetal,
        name="plunger_stem",
    )
    pump_plunger.visual(
        Box((0.008, 0.014, 0.014)),
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
        material=gray_acetal,
        name="plunger_pad",
    )

    model.articulation(
        "bottle_to_trigger",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=trigger,
        origin=Origin(xyz=(0.055, 0.0, 0.195)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.42),
    )
    model.articulation(
        "bottle_to_pump_plunger",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=pump_plunger,
        origin=Origin(xyz=(0.083, 0.0, 0.209)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.20, lower=0.0, upper=0.006),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    trigger = object_model.get_part("trigger")
    pump_plunger = object_model.get_part("pump_plunger")
    trigger_joint = object_model.get_articulation("bottle_to_trigger")
    pump_joint = object_model.get_articulation("bottle_to_pump_plunger")

    ctx.allow_overlap(
        bottle,
        pump_plunger,
        elem_a="pump_sleeve",
        elem_b="plunger_stem",
        reason=(
            "The plunger stem is intentionally shown as a retained sliding fit "
            "inside the simplified pump sleeve."
        ),
    )
    ctx.expect_within(
        pump_plunger,
        bottle,
        axes="yz",
        inner_elem="plunger_stem",
        outer_elem="pump_sleeve",
        margin=0.001,
        name="plunger stem is centered in the sleeve",
    )
    ctx.expect_overlap(
        pump_plunger,
        bottle,
        axes="x",
        elem_a="plunger_stem",
        elem_b="pump_sleeve",
        min_overlap=0.030,
        name="plunger remains inserted at rest",
    )
    ctx.expect_contact(
        trigger,
        pump_plunger,
        elem_a="pump_cam",
        elem_b="plunger_pad",
        contact_tol=0.002,
        name="trigger cam bears on plunger pad",
    )

    rest_pad = ctx.part_element_world_aabb(trigger, elem="finger_pad")
    rest_pump = ctx.part_world_position(pump_plunger)
    with ctx.pose({trigger_joint: 0.42, pump_joint: 0.006}):
        pressed_pad = ctx.part_element_world_aabb(trigger, elem="finger_pad")
        pressed_pump = ctx.part_world_position(pump_plunger)
        ctx.expect_overlap(
            pump_plunger,
            bottle,
            axes="x",
            elem_a="plunger_stem",
            elem_b="pump_sleeve",
            min_overlap=0.030,
            name="plunger remains inserted at full stroke",
        )

    rest_pad_x = (rest_pad[0][0] + rest_pad[1][0]) * 0.5 if rest_pad else None
    pressed_pad_x = (pressed_pad[0][0] + pressed_pad[1][0]) * 0.5 if pressed_pad else None
    ctx.check(
        "trigger pull moves finger pad toward bottle",
        rest_pad_x is not None and pressed_pad_x is not None and pressed_pad_x < rest_pad_x - 0.010,
        details=f"rest_pad_x={rest_pad_x}, pressed_pad_x={pressed_pad_x}",
    )
    ctx.check(
        "trigger stroke drives pump plunger forward",
        rest_pump is not None and pressed_pump is not None and pressed_pump[0] > rest_pump[0] + 0.004,
        details=f"rest_pump={rest_pump}, pressed_pump={pressed_pump}",
    )

    return ctx.report()


object_model = build_object_model()
