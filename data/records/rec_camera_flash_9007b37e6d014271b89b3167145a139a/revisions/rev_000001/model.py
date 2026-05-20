from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_speedlight_flash")

    matte_black = model.material("matte_black", color=(0.015, 0.014, 0.013, 1.0))
    satin_black = model.material("satin_black", color=(0.035, 0.034, 0.032, 1.0))
    rubber = model.material("soft_rubber", color=(0.006, 0.006, 0.006, 1.0))
    dark_panel = model.material("dark_rear_panel", color=(0.010, 0.012, 0.015, 1.0))
    screen_glass = model.material("blue_black_screen", color=(0.03, 0.08, 0.12, 1.0))
    flash_lens = model.material("warm_frosted_lens", color=(1.0, 0.86, 0.52, 0.78))
    metal = model.material("brushed_hot_shoe", color=(0.58, 0.58, 0.55, 1.0))

    body = model.part("body")
    # Rear faces are intentionally flush while the front lower battery section
    # steps forward, matching a compact speedlight battery door/body stack.
    body.visual(
        Box((0.060, 0.074, 0.075)),
        origin=Origin(xyz=(0.000, 0.0, 0.050)),
        material=satin_black,
        name="battery_body",
    )
    body.visual(
        Box((0.050, 0.068, 0.060)),
        origin=Origin(xyz=(-0.005, 0.0, 0.112)),
        material=matte_black,
        name="upper_shell",
    )
    body.visual(
        Box((0.047, 0.003, 0.058)),
        origin=Origin(xyz=(0.001, 0.0385, 0.049)),
        material=matte_black,
        name="battery_door",
    )
    body.visual(
        Box((0.0034, 0.056, 0.084)),
        origin=Origin(xyz=(-0.0315, 0.0, 0.084)),
        material=dark_panel,
        name="rear_panel",
    )
    body.visual(
        Box((0.0020, 0.027, 0.017)),
        origin=Origin(xyz=(-0.0340, 0.0, 0.103)),
        material=screen_glass,
        name="screen",
    )
    # Hot-shoe foot below the body: central stem plus metal rails.
    body.visual(
        Box((0.026, 0.038, 0.012)),
        origin=Origin(xyz=(0.000, 0.0, 0.0065)),
        material=matte_black,
        name="shoe_stem",
    )
    body.visual(
        Box((0.044, 0.056, 0.003)),
        origin=Origin(xyz=(0.000, 0.0, 0.0000)),
        material=metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.044, 0.006, 0.006)),
        origin=Origin(xyz=(0.000, 0.026, 0.0030)),
        material=metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.044, 0.006, 0.006)),
        origin=Origin(xyz=(0.000, -0.026, 0.0030)),
        material=metal,
        name="shoe_rail_1",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=matte_black,
        name="turntable",
    )
    swivel.visual(
        Box((0.024, 0.026, 0.047)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=matte_black,
        name="neck_post",
    )
    swivel.visual(
        Box((0.030, 0.074, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=matte_black,
        name="tilt_yoke",
    )
    swivel.visual(
        Box((0.016, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.038, 0.059)),
        material=matte_black,
        name="yoke_ear_0",
    )
    swivel.visual(
        Box((0.016, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.038, 0.059)),
        material=matte_black,
        name="yoke_ear_1",
    )
    swivel.visual(
        Cylinder(radius=0.007, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.062), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="tilt_pin",
    )

    head = model.part("head")
    head.visual(
        Box((0.070, 0.092, 0.045)),
        origin=Origin(xyz=(0.047, 0.0, 0.006)),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.077, 0.031)),
        origin=Origin(xyz=(0.0835, 0.0, 0.006)),
        material=flash_lens,
        name="front_lens",
    )
    head.visual(
        Box((0.022, 0.064, 0.020)),
        origin=Origin(xyz=(0.004, 0.0, 0.000)),
        material=satin_black,
        name="rear_trunnion",
    )
    head.visual(
        Box((0.048, 0.082, 0.003)),
        origin=Origin(xyz=(0.060, 0.0, 0.030)),
        material=satin_black,
        name="top_bounce_slot",
    )

    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5, lower=-2.05, upper=2.05),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=-0.30, upper=1.55),
    )

    # Independent push controls on the rear shell.  The joint origin sits on the
    # rear-panel surface, and the cap extends outward in local -X at rest.
    button_specs = [
        ("button_0", -0.020, 0.122, (0.0040, 0.013, 0.007)),
        ("button_1", 0.020, 0.122, (0.0040, 0.013, 0.007)),
        ("button_2", -0.023, 0.103, (0.0040, 0.010, 0.010)),
        ("button_3", 0.023, 0.103, (0.0040, 0.010, 0.010)),
        ("button_4", -0.020, 0.074, (0.0040, 0.012, 0.007)),
        ("button_5", 0.000, 0.074, (0.0040, 0.012, 0.007)),
        ("button_6", 0.020, 0.074, (0.0040, 0.012, 0.007)),
        ("button_7", 0.000, 0.055, (0.0040, 0.018, 0.007)),
    ]
    for name, y, z, size in button_specs:
        button = model.part(name)
        button.visual(
            Box(size),
            origin=Origin(xyz=(-size[0] / 2.0, 0.0, 0.0)),
            material=rubber,
            name="cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(-0.0332, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=0.08, lower=0.0, upper=0.0022),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    swivel = object_model.get_part("swivel")
    head = object_model.get_part("head")
    swivel_joint = object_model.get_articulation("body_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")

    ctx.allow_overlap(
        swivel,
        head,
        elem_a="tilt_pin",
        elem_b="rear_trunnion",
        reason="The metal tilt pin is intentionally captured inside the head trunnion bearing.",
    )
    ctx.expect_within(
        swivel,
        head,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="rear_trunnion",
        margin=0.001,
        name="tilt pin is captured inside trunnion",
    )
    ctx.expect_overlap(
        swivel,
        head,
        axes="y",
        elem_a="tilt_pin",
        elem_b="rear_trunnion",
        min_overlap=0.055,
        name="tilt pin spans the head trunnion",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.035,
        name="lamp head is visibly raised above body",
    )
    ctx.expect_contact(
        swivel,
        body,
        elem_a="turntable",
        elem_b="upper_shell",
        contact_tol=0.0005,
        name="swivel turntable sits on body top",
    )

    ctx.check(
        "lower aiming joint is vertical swivel",
        swivel_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel_joint.articulation_type}, axis={swivel_joint.axis}",
    )
    ctx.check(
        "upper aiming joint is horizontal tilt",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(tilt_joint.axis) == (0.0, -1.0, 0.0),
        details=f"type={tilt_joint.articulation_type}, axis={tilt_joint.axis}",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({tilt_joint: 0.75}):
        tilted_lens_aabb = ctx.part_element_world_aabb(head, elem="front_lens")
    rest_lens_z = (rest_lens_aabb[0][2] + rest_lens_aabb[1][2]) / 2.0 if rest_lens_aabb else 0.0
    tilted_lens_z = (
        (tilted_lens_aabb[0][2] + tilted_lens_aabb[1][2]) / 2.0 if tilted_lens_aabb else 0.0
    )
    ctx.check(
        "tilt joint raises lamp face",
        tilted_lens_z > rest_lens_z + 0.025,
        details=f"rest_z={rest_lens_z:.4f}, tilted_z={tilted_lens_z:.4f}",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({swivel_joint: 0.70}):
        swiveled_lens_aabb = ctx.part_element_world_aabb(head, elem="front_lens")
    rest_lens_y = (rest_lens_aabb[0][1] + rest_lens_aabb[1][1]) / 2.0 if rest_lens_aabb else 0.0
    swiveled_lens_y = (
        (swiveled_lens_aabb[0][1] + swiveled_lens_aabb[1][1]) / 2.0 if swiveled_lens_aabb else 0.0
    )
    ctx.check(
        "swivel joint turns lamp face sideways",
        swiveled_lens_y > rest_lens_y + 0.035,
        details=f"rest_y={rest_lens_y:.4f}, swiveled_y={swiveled_lens_y:.4f}",
    )

    button_joints = [object_model.get_articulation(f"body_to_button_{i}") for i in range(8)]
    ctx.check(
        "rear button bank has independent prismatic joints",
        all(j.articulation_type == ArticulationType.PRISMATIC and tuple(j.axis) == (1.0, 0.0, 0.0) for j in button_joints),
        details=", ".join(f"{j.name}:{j.articulation_type}:{j.axis}" for j in button_joints),
    )
    for i, button_joint in enumerate(button_joints):
        button = object_model.get_part(f"button_{i}")
        ctx.expect_contact(
            button,
            body,
            elem_a="cap",
            elem_b="rear_panel",
            contact_tol=0.0005,
            name=f"button {i} cap seats in rear shell",
        )
    button_0 = object_model.get_part("button_0")
    rest_pos = ctx.part_world_position(button_0)
    with ctx.pose({button_joints[0]: 0.0022}):
        pressed_pos = ctx.part_world_position(button_0)
    ctx.check(
        "button press moves inward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[0] > rest_pos[0] + 0.0015,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
