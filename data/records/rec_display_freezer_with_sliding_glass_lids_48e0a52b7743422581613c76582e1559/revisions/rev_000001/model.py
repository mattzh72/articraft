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


def _add_glass_slider(
    part,
    *,
    length: float,
    width: float,
    frame_width: float,
    frame_height: float,
    glass_thickness: float,
    frame_material,
    glass_material,
    handle_sign: float,
) -> None:
    part.visual(
        Box((length, frame_width, frame_height)),
        origin=Origin(xyz=(0.0, (width - frame_width) * 0.5, frame_height * 0.5)),
        material=frame_material,
        name="rear_frame",
    )
    part.visual(
        Box((length, frame_width, frame_height)),
        origin=Origin(xyz=(0.0, -(width - frame_width) * 0.5, frame_height * 0.5)),
        material=frame_material,
        name="front_frame",
    )
    part.visual(
        Box((frame_width, width - 2.0 * frame_width, frame_height)),
        origin=Origin(xyz=((length - frame_width) * 0.5, 0.0, frame_height * 0.5)),
        material=frame_material,
        name="end_frame_0",
    )
    part.visual(
        Box((frame_width, width - 2.0 * frame_width, frame_height)),
        origin=Origin(xyz=(-(length - frame_width) * 0.5, 0.0, frame_height * 0.5)),
        material=frame_material,
        name="end_frame_1",
    )
    part.visual(
        Box((length - 2.0 * frame_width + 0.004, width - 2.0 * frame_width + 0.004, glass_thickness)),
        origin=Origin(xyz=(0.0, 0.0, frame_height * 0.5)),
        material=glass_material,
        name="glass",
    )
    part.visual(
        Box((0.018, width * 0.22, 0.010)),
        origin=Origin(
            xyz=(handle_sign * (length * 0.5 - 0.012), 0.0, frame_height + 0.005)
        ),
        material=frame_material,
        name="pull",
    )


def _add_vertical_hinge_barrels(
    part,
    *,
    x: float,
    y: float,
    z_bottom: float,
    z_top: float,
    radius: float,
    length: float,
    material,
    prefix: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z_bottom + length * 0.5)),
        material=material,
        name=f"{prefix}_lower",
    )
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z_top - length * 0.5)),
        material=material,
        name=f"{prefix}_upper",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_freezer")

    body_white = model.material("body_white", rgba=(0.91, 0.92, 0.93, 1.0))
    liner_white = model.material("liner_white", rgba=(0.95, 0.95, 0.96, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.42, 0.44, 0.47, 1.0))
    rail_grey = model.material("rail_grey", rgba=(0.57, 0.59, 0.62, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.69, 0.84, 0.90, 0.35))
    dark_grey = model.material("dark_grey", rgba=(0.23, 0.24, 0.26, 1.0))
    lock_metal = model.material("lock_metal", rgba=(0.73, 0.75, 0.78, 1.0))

    outer_length = 1.12
    outer_width = 0.68
    base_thickness = 0.05
    wall_thickness = 0.035
    shell_height = 0.71
    top_rail_height = 0.04
    top_rail_width = 0.10
    end_cap_length = 0.06

    shell_top = base_thickness + shell_height
    outer_height = shell_top + top_rail_height
    opening_length = outer_length - 2.0 * end_cap_length
    opening_width = outer_width - 2.0 * top_rail_width
    center_rail_width = 0.04

    body = model.part("body")
    body.visual(
        Box((outer_length, outer_width, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness * 0.5)),
        material=body_white,
        name="base_pan",
    )
    body.visual(
        Box((outer_length, wall_thickness, shell_height)),
        origin=Origin(
            xyz=(0.0, -(outer_width - wall_thickness) * 0.5, base_thickness + shell_height * 0.5)
        ),
        material=body_white,
        name="front_wall",
    )
    body.visual(
        Box((outer_length, wall_thickness, shell_height)),
        origin=Origin(
            xyz=(0.0, (outer_width - wall_thickness) * 0.5, base_thickness + shell_height * 0.5)
        ),
        material=body_white,
        name="rear_wall",
    )
    body.visual(
        Box((wall_thickness, outer_width - 2.0 * wall_thickness, shell_height)),
        origin=Origin(
            xyz=(-(outer_length - wall_thickness) * 0.5, 0.0, base_thickness + shell_height * 0.5)
        ),
        material=body_white,
        name="left_wall",
    )
    body.visual(
        Box((wall_thickness, outer_width - 2.0 * wall_thickness, shell_height)),
        origin=Origin(
            xyz=((outer_length - wall_thickness) * 0.5, 0.0, base_thickness + shell_height * 0.5)
        ),
        material=body_white,
        name="right_wall",
    )
    body.visual(
        Box((outer_length - 2.0 * wall_thickness, outer_width - 2.0 * wall_thickness, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + 0.004)),
        material=liner_white,
        name="inner_floor",
    )
    body.visual(
        Box((opening_length, top_rail_width, top_rail_height)),
        origin=Origin(xyz=(0.0, -(outer_width - top_rail_width) * 0.5, shell_top + top_rail_height * 0.5)),
        material=rail_grey,
        name="front_rail",
    )
    body.visual(
        Box((opening_length, top_rail_width, top_rail_height)),
        origin=Origin(xyz=(0.0, (outer_width - top_rail_width) * 0.5, shell_top + top_rail_height * 0.5)),
        material=rail_grey,
        name="rear_rail",
    )
    body.visual(
        Box((end_cap_length, opening_width, top_rail_height)),
        origin=Origin(
            xyz=(
                -(outer_length - end_cap_length) * 0.5,
                0.0,
                shell_top + top_rail_height * 0.5,
            )
        ),
        material=trim_grey,
        name="left_cap",
    )
    body.visual(
        Box((end_cap_length, opening_width, top_rail_height)),
        origin=Origin(
            xyz=(
                (outer_length - end_cap_length) * 0.5,
                0.0,
                shell_top + top_rail_height * 0.5,
            )
        ),
        material=trim_grey,
        name="right_cap",
    )
    body.visual(
        Box((opening_length, center_rail_width, top_rail_height)),
        origin=Origin(xyz=(0.0, 0.0, shell_top + top_rail_height * 0.5)),
        material=trim_grey,
        name="center_rail",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(
            xyz=(0.405, -outer_width * 0.5 - 0.002, 0.100),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=lock_metal,
        name="drain_port",
    )
    _add_vertical_hinge_barrels(
        body,
        x=0.374,
        y=-outer_width * 0.5 - 0.003,
        z_bottom=0.060,
        z_top=0.110,
        radius=0.0042,
        length=0.016,
        material=trim_grey,
        prefix="drain_mount",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(
            xyz=(outer_length * 0.5 + 0.002, -0.136, 0.585),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=lock_metal,
        name="key_cylinder",
    )
    _add_vertical_hinge_barrels(
        body,
        x=outer_length * 0.5 + 0.003,
        y=-0.176,
        z_bottom=0.548,
        z_top=0.622,
        radius=0.0042,
        length=0.018,
        material=trim_grey,
        prefix="lock_mount",
    )

    panel_length = 0.58
    panel_width = 0.250
    frame_width = 0.03
    frame_height = 0.014
    glass_thickness = 0.006

    slider_0 = model.part("slider_0")
    _add_glass_slider(
        slider_0,
        length=panel_length,
        width=panel_width,
        frame_width=frame_width,
        frame_height=frame_height,
        glass_thickness=glass_thickness,
        frame_material=dark_grey,
        glass_material=glass_blue,
        handle_sign=1.0,
    )

    slider_1 = model.part("slider_1")
    _add_glass_slider(
        slider_1,
        length=panel_length,
        width=panel_width,
        frame_width=frame_width,
        frame_height=frame_height,
        glass_thickness=glass_thickness,
        frame_material=dark_grey,
        glass_material=glass_blue,
        handle_sign=-1.0,
    )

    track_z = outer_height
    closed_offset = 0.210
    slider_travel = 0.260

    model.articulation(
        "body_to_slider_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider_0,
        origin=Origin(xyz=(-closed_offset, -0.125, track_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=slider_travel),
    )
    model.articulation(
        "body_to_slider_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider_1,
        origin=Origin(xyz=(closed_offset, 0.125, track_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=slider_travel),
    )

    drain_flap = model.part("drain_flap")
    drain_flap_width = 0.062
    drain_flap_height = 0.050
    drain_flap_thickness = 0.008
    drain_flap.visual(
        Box((drain_flap_width, drain_flap_thickness, drain_flap_height)),
        origin=Origin(
            xyz=(
                drain_flap_width * 0.5,
                -drain_flap_thickness * 0.5,
                drain_flap_height * 0.5,
            )
        ),
        material=trim_grey,
        name="cover",
    )
    _add_vertical_hinge_barrels(
        drain_flap,
        x=0.002,
        y=-0.004,
        z_bottom=0.0,
        z_top=drain_flap_height,
        radius=0.004,
        length=0.016,
        material=dark_grey,
        prefix="hinge",
    )
    model.articulation(
        "body_to_drain_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drain_flap,
        origin=Origin(xyz=(0.374, -outer_width * 0.5 - 0.006, 0.060)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.4),
    )

    lock_flap = model.part("lock_flap")
    lock_flap_width = 0.072
    lock_flap_height = 0.074
    lock_flap_thickness = 0.008
    lock_flap.visual(
        Box((lock_flap_thickness, lock_flap_width, lock_flap_height)),
        origin=Origin(
            xyz=(
                lock_flap_thickness * 0.5,
                lock_flap_width * 0.5,
                lock_flap_height * 0.5,
            )
        ),
        material=trim_grey,
        name="cover",
    )
    _add_vertical_hinge_barrels(
        lock_flap,
        x=0.004,
        y=0.001,
        z_bottom=0.0,
        z_top=lock_flap_height,
        radius=0.004,
        length=0.018,
        material=dark_grey,
        prefix="hinge",
    )
    model.articulation(
        "body_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_flap,
        origin=Origin(xyz=(outer_length * 0.5 + 0.006, -0.176, 0.548)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))

    body = object_model.get_part("body")
    slider_0 = object_model.get_part("slider_0")
    slider_1 = object_model.get_part("slider_1")
    drain_flap = object_model.get_part("drain_flap")
    lock_flap = object_model.get_part("lock_flap")

    slider_0_joint = object_model.get_articulation("body_to_slider_0")
    slider_1_joint = object_model.get_articulation("body_to_slider_1")
    drain_joint = object_model.get_articulation("body_to_drain_flap")
    lock_joint = object_model.get_articulation("body_to_lock_flap")

    ctx.expect_gap(
        slider_0,
        body,
        axis="z",
        max_gap=0.020,
        max_penetration=0.0,
        name="front slider stays on the freezer rails",
    )
    ctx.expect_gap(
        slider_1,
        body,
        axis="z",
        max_gap=0.020,
        max_penetration=0.0,
        name="rear slider stays on the freezer rails",
    )
    ctx.expect_overlap(
        drain_flap,
        body,
        axes="xz",
        elem_a="cover",
        elem_b="drain_port",
        min_overlap=0.015,
        name="drain flap covers the drain port",
    )
    ctx.expect_overlap(
        lock_flap,
        body,
        axes="yz",
        elem_a="cover",
        elem_b="key_cylinder",
        min_overlap=0.015,
        name="lock flap covers the key cylinder",
    )
    ctx.expect_gap(
        body,
        drain_flap,
        axis="y",
        positive_elem="front_wall",
        negative_elem="cover",
        min_gap=0.004,
        max_gap=0.012,
        name="drain flap sits just proud of the front wall",
    )
    ctx.expect_gap(
        lock_flap,
        body,
        axis="x",
        positive_elem="cover",
        negative_elem="right_wall",
        min_gap=0.004,
        max_gap=0.012,
        name="lock flap sits just proud of the side wall",
    )

    slider_0_rest = ctx.part_world_position(slider_0)
    slider_1_rest = ctx.part_world_position(slider_1)
    drain_rest = _aabb_center(ctx.part_element_world_aabb(drain_flap, elem="cover"))
    lock_rest = _aabb_center(ctx.part_element_world_aabb(lock_flap, elem="cover"))

    with ctx.pose({slider_0_joint: 0.260, slider_1_joint: 0.260}):
        slider_0_open = ctx.part_world_position(slider_0)
        slider_1_open = ctx.part_world_position(slider_1)
        ctx.check(
            "left slider moves right to open",
            slider_0_rest is not None
            and slider_0_open is not None
            and slider_0_open[0] > slider_0_rest[0] + 0.20,
            details=f"rest={slider_0_rest}, open={slider_0_open}",
        )
        ctx.check(
            "right slider moves left to open",
            slider_1_rest is not None
            and slider_1_open is not None
            and slider_1_open[0] < slider_1_rest[0] - 0.20,
            details=f"rest={slider_1_rest}, open={slider_1_open}",
        )
        ctx.expect_overlap(
            slider_0,
            body,
            axes="y",
            min_overlap=0.20,
            name="front slider remains on its rail lane when opened",
        )
        ctx.expect_overlap(
            slider_1,
            body,
            axes="y",
            min_overlap=0.20,
            name="rear slider remains on its rail lane when opened",
        )

    with ctx.pose({drain_joint: 1.2, lock_joint: 1.2}):
        drain_open = _aabb_center(ctx.part_element_world_aabb(drain_flap, elem="cover"))
        lock_open = _aabb_center(ctx.part_element_world_aabb(lock_flap, elem="cover"))
        ctx.check(
            "drain flap swings outward from the front wall",
            drain_rest is not None
            and drain_open is not None
            and drain_open[1] < drain_rest[1] - 0.010,
            details=f"rest={drain_rest}, open={drain_open}",
        )
        ctx.check(
            "lock flap swings outward from the side wall",
            lock_rest is not None
            and lock_open is not None
            and lock_open[0] > lock_rest[0] + 0.010,
            details=f"rest={lock_rest}, open={lock_open}",
        )

    return ctx.report()


object_model = build_object_model()
