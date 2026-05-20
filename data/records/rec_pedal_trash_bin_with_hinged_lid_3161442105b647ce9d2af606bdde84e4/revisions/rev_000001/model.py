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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_step_bin")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("dark_hinge_plastic", rgba=(0.04, 0.045, 0.045, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    radius = 0.17
    height = 0.55
    wall = 0.006
    hinge_y = radius + 0.020
    hinge_z = height + 0.015

    body = model.part("bin_body")
    body_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (radius * 0.94, 0.000),
            (radius, 0.018),
            (radius, height - 0.018),
            (radius * 1.012, height),
        ],
        inner_profile=[
            (radius - wall * 1.4, 0.025),
            (radius - wall, height - 0.016),
            (radius - wall * 0.45, height - 0.004),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    body.visual(
        mesh_from_geometry(body_shell, "bin_shell"),
        origin=Origin(),
        material=stainless,
        name="bin_shell",
    )
    # Rear hinge band: a dark strap and two barrel leaves tied into the rim.
    body.visual(
        Box((0.245, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, radius + 0.006, height - 0.018)),
        material=dark,
        name="rear_hinge_band",
    )
    for i, x in enumerate((-0.056, 0.056)):
        body.visual(
            Box((0.050, 0.016, 0.024)),
            origin=Origin(xyz=(x, radius + 0.014, hinge_z - 0.015)),
            material=dark,
            name=f"hinge_barrel_support_{i}",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.058),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"body_hinge_barrel_{i}",
        )

    # Lower front pivot lugs for the narrow pedal bar.
    pedal_pivot_y = -radius - 0.035
    pedal_pivot_z = 0.075
    for i, x in enumerate((-0.105, 0.105)):
        body.visual(
            Box((0.026, 0.120, 0.036)),
            origin=Origin(xyz=(x, -radius - 0.022, pedal_pivot_z)),
            material=dark,
            name=f"pedal_pivot_lug_{i}",
        )

    # Rear damper mechanism under a separate movable cover.
    damper_hinge_y = radius + 0.045
    damper_hinge_z = height - 0.020
    body.visual(
        Box((0.105, 0.042, 0.135)),
        origin=Origin(xyz=(0.0, radius + 0.022, damper_hinge_z - 0.072)),
        material=dark,
        name="damper_mount",
    )
    for i, x in enumerate((-0.048, 0.048)):
        body.visual(
            Cylinder(radius=0.0075, length=0.032),
            origin=Origin(xyz=(x, damper_hinge_y, damper_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"damper_body_barrel_{i}",
        )

    lid = model.part("lid")
    lid_dome = LatheGeometry(
        [
            (0.000, 0.041),
            (radius * 0.30, 0.039),
            (radius * 0.62, 0.029),
            (radius * 0.88, 0.013),
            (radius * 0.982, -0.002),
            (radius * 0.982, -0.010),
            (radius * 0.87, -0.012),
            (0.000, -0.012),
        ],
        segments=96,
        closed=True,
    )
    lid_dome.translate(0.0, -(radius + 0.020), 0.0)
    lid.visual(
        mesh_from_geometry(lid_dome, "domed_lid_shell"),
        origin=Origin(),
        material=stainless,
        name="dome_shell",
    )
    lid.visual(
        Cylinder(radius=0.0095, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lid_knuckle",
    )
    lid.visual(
        Box((0.064, 0.035, 0.006)),
        origin=Origin(xyz=(0.0, -0.018, -0.004)),
        material=dark,
        name="lid_hinge_leaf",
    )

    pedal = model.part("pedal_bar")
    pedal.visual(
        Cylinder(radius=0.012, length=0.245),
        origin=Origin(xyz=(0.0, -0.065, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="front_pedal_bar",
    )
    pedal.visual(
        Cylinder(radius=0.006, length=0.194),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="pedal_pivot_pin",
    )
    for i, x in enumerate((-0.070, 0.070)):
        pedal.visual(
            Box((0.012, 0.070, 0.010)),
            origin=Origin(xyz=(x, -0.034, -0.009)),
            material=dark,
            name=f"pedal_arm_{i}",
        )

    damper_cover = model.part("damper_cover")
    damper_cover.visual(
        Cylinder(radius=0.0068, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="cover_hinge_barrel",
    )
    damper_cover.visual(
        Box((0.068, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.007, -0.012)),
        material=dark,
        name="cover_hinge_leaf",
    )
    damper_cover.visual(
        Box((0.120, 0.028, 0.128)),
        origin=Origin(xyz=(0.0, 0.022, -0.070)),
        material=dark,
        name="cover_panel",
    )
    damper_cover.visual(
        Box((0.088, 0.004, 0.060)),
        origin=Origin(xyz=(0.0, 0.008, -0.072)),
        material=black,
        name="cover_grip_inset",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, pedal_pivot_y, pedal_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=0.0, upper=0.36),
    )
    model.articulation(
        "body_to_damper_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=damper_cover,
        origin=Origin(xyz=(0.0, damper_hinge_y, damper_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.8, lower=0.0, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bin_body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal_bar")
    damper = object_model.get_part("damper_cover")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")
    cover_hinge = object_model.get_articulation("body_to_damper_cover")

    for barrel in ("body_hinge_barrel_0", "body_hinge_barrel_1"):
        ctx.allow_overlap(
            body,
            lid,
            elem_a=barrel,
            elem_b="lid_knuckle",
            reason="The lid knuckle is intentionally captured on the rear hinge pin carried by the rim band.",
        )
        ctx.expect_overlap(
            body,
            lid,
            axes="x",
            elem_a=barrel,
            elem_b="lid_knuckle",
            min_overlap=0.002,
            name=f"{barrel} captures the lid knuckle along the hinge axis",
        )

    for lug in ("pedal_pivot_lug_0", "pedal_pivot_lug_1"):
        ctx.allow_overlap(
            body,
            pedal,
            elem_a=lug,
            elem_b="pedal_pivot_pin",
            reason="The lower pedal pivot pin is intentionally nested through the body lug as a captured hinge.",
        )
        ctx.expect_overlap(
            body,
            pedal,
            axes="x",
            elem_a=lug,
            elem_b="pedal_pivot_pin",
            min_overlap=0.002,
            name=f"{lug} captures the pedal pivot pin",
        )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="dome_shell",
        elem_b="bin_shell",
        min_overlap=0.25,
        name="domed lid footprint covers the bin rim",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="dome_shell",
        negative_elem="bin_shell",
        min_gap=0.0,
        max_gap=0.016,
        name="closed dome sits just above the stainless rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        elem_a="lid_knuckle",
        elem_b="rear_hinge_band",
        min_overlap=0.050,
        name="lid knuckle is carried by the rear hinge band",
    )
    ctx.expect_gap(
        damper,
        body,
        axis="y",
        positive_elem="cover_panel",
        negative_elem="damper_mount",
        min_gap=0.004,
        max_gap=0.020,
        name="damper cover is a separate cap over the rear mechanism",
    )

    closed_lid_box = ctx.part_element_world_aabb(lid, elem="dome_shell")
    closed_pedal_box = ctx.part_element_world_aabb(pedal, elem="front_pedal_bar")
    closed_cover_box = ctx.part_element_world_aabb(damper, elem="cover_panel")

    with ctx.pose({lid_hinge: 1.0}):
        opened_lid_box = ctx.part_element_world_aabb(lid, elem="dome_shell")
    ctx.check(
        "rear horizontal hinge lifts the lid upward",
        closed_lid_box is not None
        and opened_lid_box is not None
        and opened_lid_box[1][2] > closed_lid_box[1][2] + 0.12,
        details=f"closed={closed_lid_box}, opened={opened_lid_box}",
    )

    with ctx.pose({pedal_hinge: 0.32}):
        pressed_pedal_box = ctx.part_element_world_aabb(pedal, elem="front_pedal_bar")
    ctx.check(
        "front pedal bar rotates downward about its lower pivot",
        closed_pedal_box is not None
        and pressed_pedal_box is not None
        and pressed_pedal_box[0][2] < closed_pedal_box[0][2] - 0.010,
        details=f"closed={closed_pedal_box}, pressed={pressed_pedal_box}",
    )

    with ctx.pose({cover_hinge: 0.75}):
        opened_cover_box = ctx.part_element_world_aabb(damper, elem="cover_panel")
    ctx.check(
        "damper cover swings rearward on a short back hinge",
        closed_cover_box is not None
        and opened_cover_box is not None
        and opened_cover_box[1][1] > closed_cover_box[1][1] + 0.025,
        details=f"closed={closed_cover_box}, opened={opened_cover_box}",
    )

    return ctx.report()


object_model = build_object_model()
