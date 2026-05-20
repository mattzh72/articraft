from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _rounded_box_mesh(name: str, size: tuple[float, float, float], radius: float):
    """Return a managed mesh for a lightly filleted electronics housing box."""
    shape = cq.Workplane("XY").box(*size).edges().fillet(radius)
    return mesh_from_cadquery(shape, name, tolerance=0.00045, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hot_shoe_flash")

    matte_black = model.material("matte_black", rgba=(0.012, 0.012, 0.012, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.035, 0.038, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.12, 0.13, 1.0))
    shoe_metal = model.material("brushed_shoe_metal", rgba=(0.62, 0.62, 0.58, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.02, 0.045, 0.075, 1.0))
    lcd_blue = model.material("lcd_blue", rgba=(0.20, 0.47, 0.72, 1.0))
    diffuser = model.material("milky_diffuser", rgba=(0.92, 0.90, 0.80, 0.82))

    # Body frame: centered left/right and front/rear, with z=0 at the bottom of
    # the battery body.  The hot-shoe foot hangs below it.
    body = model.part("battery_body")
    body.visual(
        _rounded_box_mesh("battery_body_shell", (0.058, 0.038, 0.064), 0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=matte_black,
        name="body_shell",
    )
    body.visual(
        Box((0.040, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=matte_black,
        name="shoe_stem",
    )
    body.visual(
        Box((0.054, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=shoe_metal,
        name="hot_shoe_plate",
    )
    body.visual(
        Box((0.008, 0.032, 0.004)),
        origin=Origin(xyz=(-0.023, 0.0, -0.006)),
        material=shoe_metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.008, 0.032, 0.004)),
        origin=Origin(xyz=(0.023, 0.0, -0.006)),
        material=shoe_metal,
        name="shoe_rail_1",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=graphite,
        name="swivel_socket",
    )
    body.visual(
        Box((0.040, 0.0020, 0.022)),
        origin=Origin(xyz=(0.0, -0.0196, 0.046)),
        material=graphite,
        name="screen_bezel",
    )
    body.visual(
        Box((0.034, 0.0024, 0.016)),
        origin=Origin(xyz=(0.0, -0.0212, 0.046)),
        material=screen_glass,
        name="rear_screen",
    )
    body.visual(
        Box((0.022, 0.0026, 0.0022)),
        origin=Origin(xyz=(0.0, -0.0228, 0.049)),
        material=lcd_blue,
        name="screen_status_bar",
    )
    body.visual(
        Box((0.020, 0.0026, 0.0020)),
        origin=Origin(xyz=(0.0, -0.0228, 0.043)),
        material=lcd_blue,
        name="screen_readout",
    )
    body.visual(
        Box((0.046, 0.0018, 0.020)),
        origin=Origin(xyz=(0.0, 0.0194, 0.042)),
        material=satin_black,
        name="front_battery_door",
    )

    # The neck is its own moving link and sits on the top swivel socket.  It is
    # intentionally short, with a visible yoke at the head hinge.
    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=graphite,
        name="swivel_collar",
    )
    neck.visual(
        _rounded_box_mesh("neck_column", (0.025, 0.021, 0.028), 0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=satin_black,
        name="column",
    )
    neck.visual(
        Box((0.104, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=satin_black,
        name="yoke_bridge",
    )
    neck.visual(
        Box((0.008, 0.024, 0.036)),
        origin=Origin(xyz=(-0.050, 0.0, 0.042)),
        material=satin_black,
        name="yoke_cheek_0",
    )
    neck.visual(
        Box((0.008, 0.024, 0.036)),
        origin=Origin(xyz=(0.050, 0.0, 0.042)),
        material=satin_black,
        name="yoke_cheek_1",
    )

    head = model.part("lamp_head")
    head.visual(
        _rounded_box_mesh("lamp_head_shell", (0.088, 0.052, 0.040), 0.003),
        origin=Origin(xyz=(0.0, 0.017, 0.028)),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.078, 0.0026, 0.029)),
        origin=Origin(xyz=(0.0, 0.0438, 0.029)),
        material=diffuser,
        name="front_diffuser",
    )
    head.visual(
        Box((0.086, 0.003, 0.035)),
        origin=Origin(xyz=(0.0, 0.0412, 0.029)),
        material=satin_black,
        name="diffuser_bezel",
    )
    head.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(-0.043, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_boss_0",
    )
    head.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.043, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_boss_1",
    )

    command_wheel = model.part("command_wheel")
    wheel_mesh = mesh_from_geometry(
        KnobGeometry(
            0.022,
            0.008,
            body_style="cylindrical",
            edge_radius=0.0008,
            grip=KnobGrip(style="ribbed", count=24, depth=0.0007, width=0.0010),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "command_wheel",
    )
    command_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="wheel_cap",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.35, upper=1.35),
    )
    model.articulation(
        "body_to_command_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=command_wheel,
        origin=Origin(xyz=(0.0, -0.019, 0.019)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("battery_body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("lamp_head")
    wheel = object_model.get_part("command_wheel")
    neck_swivel = object_model.get_articulation("body_to_neck")
    head_tilt = object_model.get_articulation("neck_to_head")
    wheel_spin = object_model.get_articulation("body_to_command_wheel")

    ctx.check(
        "neck uses a vertical revolute swivel",
        neck_swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(neck_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={neck_swivel.articulation_type}, axis={neck_swivel.axis}",
    )
    ctx.check(
        "command wheel uses a continuous rear-facing rotation axis",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(wheel_spin.axis) == (0.0, -1.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )

    ctx.expect_gap(
        neck,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="swivel_collar",
        negative_elem="swivel_socket",
        name="neck collar sits on body swivel socket",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.030,
        name="lamp head remains visibly above the battery body",
    )
    ctx.expect_gap(
        body,
        wheel,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="body_shell",
        negative_elem="wheel_cap",
        name="command wheel is raised from the rear face",
    )
    ctx.expect_overlap(
        wheel,
        body,
        axes="xz",
        min_overlap=0.014,
        elem_a="wheel_cap",
        elem_b="body_shell",
        name="command wheel is centered on the rear control area",
    )

    rest = ctx.part_element_world_aabb(head, elem="front_diffuser")
    with ctx.pose({head_tilt: 0.85}):
        tilted = ctx.part_element_world_aabb(head, elem="front_diffuser")
        ctx.expect_gap(
            head,
            body,
            axis="z",
            min_gap=0.025,
            name="tilted head still clears the battery body",
        )
    rest_top = rest[1][2] if rest is not None else None
    tilted_top = tilted[1][2] if tilted is not None else None
    ctx.check(
        "positive head tilt raises the front diffuser",
        rest_top is not None and tilted_top is not None and tilted_top > rest_top + 0.010,
        details=f"rest_top={rest_top}, tilted_top={tilted_top}",
    )

    with ctx.pose({wheel_spin: 2.4}):
        ctx.expect_gap(
            body,
            wheel,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="body_shell",
            negative_elem="wheel_cap",
            name="spinning wheel stays on its rear face axis",
        )

    return ctx.report()


object_model = build_object_model()
