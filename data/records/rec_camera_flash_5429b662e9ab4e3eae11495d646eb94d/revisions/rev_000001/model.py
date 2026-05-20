from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float, name: str):
    """Return a managed mesh for a lightly radiused rectangular molded shell."""
    sx, sy, sz = size
    shape = cq.Workplane("XY").box(sx, sy, sz).edges().fillet(radius)
    return mesh_from_cadquery(shape, name, tolerance=0.0006, angular_tolerance=0.08)


def _add_button(
    model: ArticulatedObject,
    *,
    parent,
    material: Material,
    name: str,
    x: float,
    z: float,
    size: tuple[float, float, float],
) -> None:
    """Create one raised rear-panel push button with its own inward travel."""
    button = model.part(name)
    button.visual(
        Box(size),
        # The child frame is on the rear panel surface.  The cap extends
        # outward toward -Y so it reads as raised at q=0.
        origin=Origin(xyz=(0.0, -size[1] * 0.5, 0.0)),
        material=material,
        name="button_cap",
    )
    # A thin bright tick mark rides on the moving cap, making each separate
    # prismatic control visibly legible without adding a static panel decal.
    button.visual(
        Box((size[0] * 0.55, 0.0005, min(size[2] * 0.18, 0.0020))),
        origin=Origin(xyz=(0.0, -size[1] - 0.00025, size[2] * 0.10)),
        material="button_mark",
        name="button_mark",
    )
    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=parent,
        child=button,
        origin=Origin(xyz=(x, -0.051, z)),
        # Positive travel moves inward from the rear face into the shell.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.003),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_speedlight")

    black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark = model.material("dark_rubber", rgba=(0.055, 0.058, 0.063, 1.0))
    panel = model.material("rear_panel_black", rgba=(0.025, 0.027, 0.030, 1.0))
    grey = model.material("button_grey", rgba=(0.16, 0.165, 0.175, 1.0))
    model.material("button_mark", rgba=(0.78, 0.80, 0.82, 1.0))
    glass = model.material("cool_lcd_glass", rgba=(0.12, 0.24, 0.30, 0.78))
    diffuser = model.material("flash_diffuser", rgba=(0.92, 0.94, 0.88, 0.66))
    metal = model.material("matte_metal", rgba=(0.50, 0.52, 0.55, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_box((0.084, 0.064, 0.126), 0.008, "body_main_shell"),
        origin=Origin(xyz=(0.0, 0.004, 0.063)),
        material=black,
        name="main_shell",
    )
    body.visual(
        _rounded_box((0.074, 0.090, 0.105), 0.010, "body_battery_housing"),
        origin=Origin(xyz=(0.0, 0.000, 0.0525)),
        material=black,
        name="battery_housing",
    )
    body.visual(
        Box((0.074, 0.006, 0.104)),
        origin=Origin(xyz=(0.0, -0.048, 0.066)),
        material=panel,
        name="rear_panel",
    )
    body.visual(
        Box((0.046, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, -0.053, 0.084)),
        material=glass,
        name="screen",
    )
    body.visual(
        Box((0.054, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, 0.000, -0.003)),
        material=metal,
        name="hotshoe_foot",
    )
    body.visual(
        Box((0.036, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.000, -0.009)),
        material=metal,
        name="hotshoe_rail",
    )
    body.visual(
        Cylinder(radius=0.027, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
        material=dark,
        name="swivel_socket",
    )

    # Independent raised rear controls around the small LCD.
    button_specs = [
        ("button_0", -0.024, 0.111, (0.016, 0.006, 0.010)),
        ("button_1", 0.000, 0.111, (0.016, 0.006, 0.010)),
        ("button_2", 0.024, 0.111, (0.016, 0.006, 0.010)),
        ("button_3", -0.033, 0.088, (0.012, 0.006, 0.014)),
        ("button_4", 0.033, 0.088, (0.012, 0.006, 0.014)),
        ("button_5", -0.033, 0.067, (0.012, 0.006, 0.014)),
        ("button_6", 0.033, 0.067, (0.012, 0.006, 0.014)),
        ("button_7", -0.018, 0.047, (0.014, 0.006, 0.011)),
        ("button_8", 0.000, 0.043, (0.014, 0.006, 0.011)),
        ("button_9", 0.018, 0.047, (0.014, 0.006, 0.011)),
    ]
    for name, x, z, size in button_specs:
        _add_button(model, parent=body, material=grey, name=name, x=x, z=z, size=size)

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark,
        name="base_collar",
    )
    neck.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=dark,
        name="swivel_post",
    )
    neck.visual(
        Box((0.176, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark,
        name="cradle_bridge",
    )
    neck.visual(
        Box((0.016, 0.024, 0.048)),
        origin=Origin(xyz=(-0.088, 0.0, 0.087)),
        material=dark,
        name="yoke_arm_0",
    )
    neck.visual(
        Box((0.016, 0.024, 0.048)),
        origin=Origin(xyz=(0.088, 0.0, 0.087)),
        material=dark,
        name="yoke_arm_1",
    )
    neck.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(-0.098, 0.0, 0.087), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_cap_0",
    )
    neck.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.098, 0.0, 0.087), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_cap_1",
    )
    model.articulation(
        "body_to_neck",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    head = model.part("head")
    head.visual(
        _rounded_box((0.128, 0.076, 0.060), 0.010, "flash_head_shell"),
        origin=Origin(xyz=(0.0, 0.026, 0.018)),
        material=black,
        name="head_shell",
    )
    # Rounded side cheeks make the flash head read as a broad molded speedlight
    # head rather than a plain rectangular block.
    head.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(-0.070, 0.026, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="side_cheek_0",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.070, 0.026, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="side_cheek_1",
    )
    head.visual(
        Box((0.104, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, 0.067, 0.022)),
        material=diffuser,
        name="flash_lens",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(-0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_boss_0",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_boss_1",
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=-0.25, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")

    ctx.check(
        "neck has continuous vertical swivel",
        swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.check(
        "head has horizontal tilt hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE and tuple(tilt.axis) == (1.0, 0.0, 0.0),
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
    )

    button_joints = [object_model.get_articulation(f"body_to_button_{i}") for i in range(10)]
    ctx.check(
        "rear button bank is independently articulated",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in button_joints),
        details=[(j.name, str(j.articulation_type)) for j in button_joints],
    )
    ctx.expect_gap(
        neck,
        body,
        axis="z",
        positive_elem="base_collar",
        negative_elem="swivel_socket",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="neck collar is seated on the body top",
    )
    ctx.expect_gap(
        body,
        "button_0",
        axis="y",
        positive_elem="rear_panel",
        negative_elem="button_cap",
        max_gap=0.001,
        max_penetration=0.0005,
        name="raised button cap sits on rear panel",
    )

    body_box = ctx.part_world_aabb(body)
    head_box = ctx.part_world_aabb(head)
    ctx.check(
        "flash head is wider than body",
        body_box is not None and head_box is not None and (head_box[1][0] - head_box[0][0]) > (body_box[1][0] - body_box[0][0]) + 0.030,
        details=f"body={body_box}, head={head_box}",
    )

    rest_head_box = ctx.part_world_aabb(head)
    with ctx.pose({tilt: 0.9}):
        tilted_head_box = ctx.part_world_aabb(head)
    ctx.check(
        "positive head tilt raises the flash face",
        rest_head_box is not None
        and tilted_head_box is not None
        and tilted_head_box[1][2] > rest_head_box[1][2] + 0.015,
        details=f"rest={rest_head_box}, tilted={tilted_head_box}",
    )

    button_0 = object_model.get_part("button_0")
    rest_button_pos = ctx.part_world_position(button_0)
    with ctx.pose({"body_to_button_0": 0.003}):
        pressed_button_pos = ctx.part_world_position(button_0)
    ctx.check(
        "button press travels inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.002,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
