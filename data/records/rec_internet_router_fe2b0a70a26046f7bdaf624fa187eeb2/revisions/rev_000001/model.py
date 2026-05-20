from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.260
BODY_D = 0.180
BODY_H = 0.036
BUTTON_XS = (-0.064, -0.032, 0.0, 0.032, 0.064)
BUTTON_W = 0.023
BUTTON_H = 0.010
BUTTON_T = 0.006
BUTTON_Z = 0.017


def _router_body_shell() -> cq.Workplane:
    """Low rounded router shell with shallow front button pockets."""
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .edges(">Z")
        .fillet(0.003)
    )

    # Shallow rectangular pockets let the prismatic button caps visibly slide
    # into the front lip rather than disappearing into a solid wall.
    slot_depth = 0.014
    for x in BUTTON_XS:
        cutter = (
            cq.Workplane("XY")
            .box(BUTTON_W + 0.002, slot_depth + 0.003, BUTTON_H + 0.002, centered=(True, True, True))
            .translate((x, -BODY_D / 2.0 + slot_depth / 2.0 - 0.0015, BUTTON_Z))
        )
        shell = shell.cut(cutter)

    # A thin front seam separates the button lip from the upper case.
    seam = (
        cq.Workplane("XY")
        .box(0.210, 0.005, 0.0022, centered=(True, True, True))
        .translate((0.0, -BODY_D / 2.0 + 0.002, 0.028))
    )
    shell = shell.cut(seam)
    return shell


def _add_rear_hinge_bracket(body, x: float, y: float, z: float, material) -> None:
    body.visual(
        Box((0.032, 0.020, 0.006)),
        origin=Origin(xyz=(x, y - 0.010, BODY_H + 0.003)),
        material=material,
        name=f"rear_pedestal_{'p' if x > 0 else 'n'}",
    )
    for side, dx in enumerate((-0.011, 0.011)):
        body.visual(
            Box((0.004, 0.018, 0.016)),
            origin=Origin(xyz=(x + dx, y, z)),
            material=material,
            name=f"rear_cheek_{'p' if x > 0 else 'n'}_{side}",
        )


def _add_side_hinge_bracket(body, x: float, y: float, z: float, material) -> None:
    sign = 1.0 if x > 0 else -1.0
    body.visual(
        Box((0.020, 0.032, 0.006)),
        origin=Origin(xyz=(sign * (BODY_W / 2.0 - 0.002), y, BODY_H + 0.003)),
        material=material,
        name=f"side_pedestal_{'p' if x > 0 else 'n'}",
    )
    for side, dy in enumerate((-0.011, 0.011)):
        body.visual(
            Box((0.018, 0.004, 0.016)),
            origin=Origin(xyz=(x, y + dy, z)),
            material=material,
            name=f"side_cheek_{'p' if x > 0 else 'n'}_{side}",
        )


def _add_antenna(part, *, barrel_axis: str, material) -> None:
    if barrel_axis == "x":
        barrel_rpy = (0.0, math.pi / 2.0, 0.0)
    else:
        barrel_rpy = (-math.pi / 2.0, 0.0, 0.0)

    part.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(rpy=barrel_rpy),
        material=material,
        name="barrel",
    )
    part.visual(
        Cylinder(radius=0.0043, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=material,
        name="neck",
    )
    part.visual(
        Cylinder(radius=0.0035, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=material,
        name="mast",
    )
    part.visual(
        Sphere(radius=0.0042),
        origin=Origin(xyz=(0.0, 0.0, 0.143)),
        material=material,
        name="tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_wifi_router")

    case = model.material("warm_black_plastic", rgba=(0.015, 0.016, 0.017, 1.0))
    top = model.material("charcoal_top_panel", rgba=(0.055, 0.058, 0.062, 1.0))
    satin = model.material("satin_black_hinge", rgba=(0.0, 0.0, 0.0, 1.0))
    button_mat = model.material("soft_gray_buttons", rgba=(0.18, 0.19, 0.20, 1.0))
    led_green = model.material("green_status_lights", rgba=(0.1, 0.9, 0.22, 1.0))
    label = model.material("muted_icon_labels", rgba=(0.55, 0.57, 0.60, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_router_body_shell(), "rounded_router_body", tolerance=0.0008),
        material=case,
        name="rounded_shell",
    )
    body.visual(
        Box((0.200, 0.112, 0.0016)),
        origin=Origin(xyz=(0.0, 0.006, BODY_H + 0.0008)),
        material=top,
        name="top_inset",
    )
    for idx, x in enumerate((-0.070, -0.047, -0.024, 0.024, 0.047, 0.070)):
        body.visual(
            Box((0.005, 0.082, 0.0012)),
            origin=Origin(xyz=(x, 0.012, BODY_H + 0.0020)),
            material=case,
            name=f"vent_slot_{idx}",
        )
    for idx, x in enumerate((-0.050, -0.034, -0.018, -0.002, 0.014, 0.030, 0.046)):
        body.visual(
            Box((0.006, 0.0012, 0.0035)),
            origin=Origin(xyz=(x, -BODY_D / 2.0 - 0.0006, 0.030)),
            material=led_green if idx in (1, 2, 5) else label,
            name=f"front_indicator_{idx}",
        )

    rear_y = BODY_D / 2.0 + 0.006
    side_x = BODY_W / 2.0 + 0.006
    pivot_z = BODY_H + 0.012
    rear_mounts = [(-0.102, rear_y, pivot_z), (0.102, rear_y, pivot_z)]
    side_mounts = [(-side_x, 0.018, pivot_z), (side_x, 0.018, pivot_z)]
    for x, y, z in rear_mounts:
        _add_rear_hinge_bracket(body, x, y, z, satin)
    for x, y, z in side_mounts:
        _add_side_hinge_bracket(body, x, y, z, satin)

    for idx, (x, y, z) in enumerate(rear_mounts):
        antenna = model.part(f"rear_antenna_{idx}")
        _add_antenna(antenna, barrel_axis="x", material=satin)
        model.articulation(
            f"rear_antenna_{idx}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=antenna,
            origin=Origin(xyz=(x, y, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.45, velocity=2.5, lower=0.0, upper=1.25),
        )

    for idx, (x, y, z) in enumerate(side_mounts):
        antenna = model.part(f"side_antenna_{idx}")
        _add_antenna(antenna, barrel_axis="y", material=satin)
        outward_axis = (0.0, -1.0, 0.0) if x < 0.0 else (0.0, 1.0, 0.0)
        model.articulation(
            f"side_antenna_{idx}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=antenna,
            origin=Origin(xyz=(x, y, z)),
            axis=outward_axis,
            motion_limits=MotionLimits(effort=0.45, velocity=2.5, lower=0.0, upper=1.25),
        )

    for idx, x in enumerate(BUTTON_XS):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((BUTTON_W, BUTTON_T, BUTTON_H)),
            origin=Origin(xyz=(0.0, -BUTTON_T / 2.0, 0.0)),
            material=button_mat,
            name="cap",
        )
        button.visual(
            Box((BUTTON_W + 0.002, 0.004, BUTTON_H + 0.002)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=button_mat,
            name="guide_tongue",
        )
        model.articulation(
            f"button_{idx}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -BODY_D / 2.0, BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.0055),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    ctx.check(
        "four external antennas are articulated",
        all(object_model.get_part(name) is not None for name in (
            "rear_antenna_0",
            "rear_antenna_1",
            "side_antenna_0",
            "side_antenna_1",
        )),
    )

    for idx in range(5):
        button = object_model.get_part(f"button_{idx}")
        slide = object_model.get_articulation(f"button_{idx}_slide")
        ctx.expect_gap(
            body,
            button,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="rounded_shell",
            elem_b="cap",
            name=f"button_{idx} rests on front lip",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="xz",
            elem_a="cap",
            min_overlap=0.006,
            name=f"button_{idx} aligns with its front pocket",
        )
        rest = ctx.part_world_position(button)
        with ctx.pose({slide: 0.0055}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{idx} slides inward",
            rest is not None and pressed is not None and pressed[1] > rest[1] + 0.004,
            details=f"rest={rest}, pressed={pressed}",
        )

    for idx in range(2):
        antenna = object_model.get_part(f"rear_antenna_{idx}")
        hinge = object_model.get_articulation(f"rear_antenna_{idx}_hinge")
        rest_aabb = ctx.part_element_world_aabb(antenna, elem="mast")
        with ctx.pose({hinge: 1.0}):
            tilted_aabb = ctx.part_element_world_aabb(antenna, elem="mast")
        ctx.check(
            f"rear_antenna_{idx} tilts toward rear",
            rest_aabb is not None
            and tilted_aabb is not None
            and tilted_aabb[1][1] > rest_aabb[1][1] + 0.045,
            details=f"rest={rest_aabb}, tilted={tilted_aabb}",
        )

    for idx in range(2):
        antenna = object_model.get_part(f"side_antenna_{idx}")
        hinge = object_model.get_articulation(f"side_antenna_{idx}_hinge")
        rest_aabb = ctx.part_element_world_aabb(antenna, elem="mast")
        with ctx.pose({hinge: 1.0}):
            tilted_aabb = ctx.part_element_world_aabb(antenna, elem="mast")
        if idx == 0:
            ok = rest_aabb is not None and tilted_aabb is not None and tilted_aabb[0][0] < rest_aabb[0][0] - 0.045
        else:
            ok = rest_aabb is not None and tilted_aabb is not None and tilted_aabb[1][0] > rest_aabb[1][0] + 0.045
        ctx.check(
            f"side_antenna_{idx} tilts outboard",
            ok,
            details=f"rest={rest_aabb}, tilted={tilted_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
