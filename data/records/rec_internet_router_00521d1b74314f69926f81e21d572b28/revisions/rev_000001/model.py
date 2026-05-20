from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded rectangular solid centered on its local origin."""
    x, y, z = size
    return cq.Workplane("XY").box(x, y, z).edges("|Z").fillet(radius)


def _add_antenna_visuals(part, *, side_sign: float, material, rubber) -> None:
    """A flat fin blade and an integral clevis clip around the hinge pod."""
    blade_x = 0.028
    blade_y = 0.006
    blade_z = 0.158
    part.visual(
        Box((blade_x, blade_y, blade_z)),
        origin=Origin(xyz=(side_sign * 0.030, 0.0, 0.092)),
        material=material,
        name="fin_blade",
    )
    part.visual(
        Cylinder(radius=blade_x * 0.50, length=blade_y),
        origin=Origin(xyz=(side_sign * 0.030, 0.0, 0.171), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name="rounded_tip",
    )
    # Two cheeks straddle the fixed pod along Y with a small clearance.
    for y_sign, name in ((-1.0, "rear_clip"), (1.0, "front_clip")):
        part.visual(
            Box((0.030, 0.004, 0.032)),
            origin=Origin(xyz=(side_sign * 0.012, y_sign * 0.016, 0.000)),
            material=rubber,
            name=name,
        )
    part.visual(
        Box((0.026, 0.036, 0.006)),
        origin=Origin(xyz=(side_sign * 0.012, 0.0, 0.017)),
        material=rubber,
        name="clip_bridge",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_tower_router")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.020, 1.0))
    soft_black = model.material("soft_black", rgba=(0.03, 0.034, 0.038, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.10, 0.11, 0.12, 1.0))
    pod_plastic = model.material("pod_plastic", rgba=(0.055, 0.060, 0.066, 1.0))
    glossy_panel = model.material("glossy_panel", rgba=(0.0, 0.0, 0.0, 1.0))
    led_green = model.material("led_green", rgba=(0.25, 1.0, 0.28, 1.0))
    led_blue = model.material("led_blue", rgba=(0.12, 0.42, 1.0, 1.0))
    white_mark = model.material("white_mark", rgba=(0.92, 0.94, 0.95, 1.0))

    body_width = 0.095
    body_depth = 0.044
    body_height = 0.300
    base_height = 0.024
    body_half_x = body_width * 0.5
    front_y = -body_depth * 0.5

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_box((0.150, 0.086, base_height), 0.014), "rounded_base"),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=soft_black,
        name="base_plinth",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box((body_width, body_depth, body_height), 0.009), "body_slab"),
        origin=Origin(xyz=(0.0, 0.0, base_height + body_height * 0.5)),
        material=matte_black,
        name="vertical_slab",
    )
    body.visual(
        Box((0.030, 0.003, 0.190)),
        origin=Origin(xyz=(0.0, front_y - 0.0015, base_height + 0.170)),
        material=glossy_panel,
        name="front_status_panel",
    )
    for i, (z, mat) in enumerate(
        ((base_height + 0.236, led_blue), (base_height + 0.214, led_green), (base_height + 0.192, led_green))
    ):
        body.visual(
            Cylinder(radius=0.0036, length=0.0024),
            origin=Origin(xyz=(0.0, front_y - 0.0036, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=mat,
            name=f"led_{i}",
        )

    # Short side hinge pods, each tied to the slab by a saddle block.
    pod_radius = 0.010
    pod_length = 0.026
    pod_z = base_height + 0.246
    pod_ys = (-0.026, 0.026)
    antenna_specs: list[tuple[str, float, float, str]] = []
    for side_name, side_sign in (("side_0", -1.0), ("side_1", 1.0)):
        for pod_index, pod_y in enumerate(pod_ys):
            pod_name = f"{side_name}_pod_{pod_index}"
            pod_x = side_sign * (body_half_x + pod_radius)
            body.visual(
                Box((0.011, 0.033, 0.018)),
                origin=Origin(xyz=(side_sign * (body_half_x + 0.0055), pod_y, pod_z)),
                material=pod_plastic,
                name=f"{pod_name}_saddle",
            )
            body.visual(
                Cylinder(radius=pod_radius, length=pod_length),
                origin=Origin(xyz=(pod_x, pod_y, pod_z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=pod_plastic,
                name=pod_name,
            )
            antenna_specs.append((f"antenna_{len(antenna_specs)}", side_sign, pod_y, pod_name))

    antenna_limits = MotionLimits(effort=0.35, velocity=2.0, lower=0.0, upper=1.15)
    for antenna_name, side_sign, pod_y, _pod_name in antenna_specs:
        antenna = model.part(antenna_name)
        _add_antenna_visuals(antenna, side_sign=side_sign, material=dark_gray, rubber=pod_plastic)
        model.articulation(
            f"body_to_{antenna_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=antenna,
            origin=Origin(xyz=(side_sign * (body_half_x + pod_radius), pod_y, pod_z)),
            axis=(0.0, side_sign, 0.0),
            motion_limits=antenna_limits,
        )

    switch = model.part("rocker_switch")
    switch.visual(
        Box((0.045, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, -0.0062, 0.0)),
        material=dark_gray,
        name="rocker_face",
    )
    switch.visual(
        Box((0.004, 0.0015, 0.010)),
        origin=Origin(xyz=(0.0, -0.0093, 0.0060)),
        material=white_mark,
        name="power_mark",
    )
    switch.visual(
        Cylinder(radius=0.0032, length=0.049),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=pod_plastic,
        name="switch_axis",
    )
    body.visual(
        Box((0.058, 0.003, 0.038)),
        origin=Origin(xyz=(0.0, front_y - 0.0015, base_height + 0.050)),
        material=glossy_panel,
        name="switch_recess",
    )
    model.articulation(
        "body_to_rocker_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=switch,
        origin=Origin(xyz=(0.0, front_y - 0.0005, base_height + 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=5.0, lower=-0.30, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    switch = object_model.get_part("rocker_switch")
    switch_joint = object_model.get_articulation("body_to_rocker_switch")

    ctx.expect_gap(
        body,
        switch,
        axis="y",
        positive_elem="switch_recess",
        negative_elem="rocker_face",
        min_gap=0.0005,
        max_gap=0.004,
        name="rocker face stands proud of front recess",
    )
    with ctx.pose({switch_joint: 0.30}):
        ctx.expect_overlap(
            switch,
            body,
            axes="x",
            elem_a="rocker_face",
            elem_b="switch_recess",
            min_overlap=0.030,
            name="rocker stays centered in the front recess while tilted",
        )

    for antenna_index in range(4):
        antenna = object_model.get_part(f"antenna_{antenna_index}")
        hinge = object_model.get_articulation(f"body_to_antenna_{antenna_index}")
        pod_name = f"side_{0 if antenna_index < 2 else 1}_pod_{antenna_index % 2}"
        ctx.expect_overlap(
            antenna,
            body,
            axes="xz",
            elem_a="front_clip",
            elem_b=pod_name,
            min_overlap=0.010,
            name=f"antenna {antenna_index} front clip wraps pod",
        )
        ctx.expect_overlap(
            antenna,
            body,
            axes="xz",
            elem_a="rear_clip",
            elem_b=pod_name,
            min_overlap=0.010,
            name=f"antenna {antenna_index} rear clip wraps pod",
        )
        ctx.expect_gap(
            antenna,
            body,
            axis="y",
            positive_elem="front_clip",
            negative_elem=pod_name,
            min_gap=0.0005,
            max_gap=0.003,
            name=f"antenna {antenna_index} front clip clears pod end",
        )
        ctx.expect_gap(
            body,
            antenna,
            axis="y",
            positive_elem=pod_name,
            negative_elem="rear_clip",
            min_gap=0.0005,
            max_gap=0.003,
            name=f"antenna {antenna_index} rear clip clears pod end",
        )
        rest_pos = ctx.part_world_position(antenna)
        with ctx.pose({hinge: 1.15}):
            extended_pos = ctx.part_world_position(antenna)
            ctx.expect_gap(
                *((
                    antenna,
                    body,
                ) if antenna_index >= 2 else (
                    body,
                    antenna,
                )),
                axis="x",
                positive_elem="fin_blade" if antenna_index >= 2 else "vertical_slab",
                negative_elem="vertical_slab" if antenna_index >= 2 else "fin_blade",
                min_gap=0.010,
                name=f"antenna {antenna_index} blade rotates outward from slab",
            )
        ctx.check(
            f"antenna {antenna_index} has an individual hinge",
            rest_pos is not None and extended_pos is not None and hinge.motion_limits is not None,
            details=f"rest={rest_pos}, rotated={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
