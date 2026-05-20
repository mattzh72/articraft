from __future__ import annotations

from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_axis")

    dark_rail = Material("dark_hard_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    bright_edge = Material("machined_steel_edges", rgba=(0.68, 0.70, 0.70, 1.0))
    carriage_blue = Material("blue_carriage_casting", rgba=(0.10, 0.25, 0.48, 1.0))
    frame_yellow = Material("yellow_hinged_nose_frame", rgba=(0.94, 0.66, 0.16, 1.0))
    ram_steel = Material("brushed_rectangular_ram", rgba=(0.58, 0.60, 0.58, 1.0))
    black = Material("black_fasteners", rgba=(0.015, 0.015, 0.015, 1.0))

    fixed_rail = model.part("fixed_rail")
    fixed_rail.visual(
        Box((0.90, 0.18, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_rail,
        name="base_extrusion",
    )
    fixed_rail.visual(
        Box((0.86, 0.060, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=dark_rail,
        name="center_spine",
    )
    for y, web_name, way_name in (
        (-0.060, "way_web_0", "guide_way_0"),
        (0.060, "way_web_1", "guide_way_1"),
    ):
        fixed_rail.visual(
            Box((0.82, 0.020, 0.045)),
            origin=Origin(xyz=(0.0, y * 0.72, 0.0575)),
            material=dark_rail,
            name=web_name,
        )
        fixed_rail.visual(
            Box((0.82, 0.030, 0.025)),
            origin=Origin(xyz=(0.0, y, 0.0925)),
            material=bright_edge,
            name=way_name,
        )
    for index, x in enumerate((-0.430, 0.430)):
        fixed_rail.visual(
            Box((0.035, 0.18, 0.115)),
            origin=Origin(xyz=(x, 0.0, 0.0575)),
            material=dark_rail,
            name=f"end_stop_{index}",
        )
    for index, (x, y) in enumerate(
        ((-0.34, -0.080), (-0.34, 0.080), (0.34, -0.080), (0.34, 0.080))
    ):
        fixed_rail.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.038)),
            material=black,
            name=f"rail_bolt_{index}",
        )

    carriage = model.part("carriage")
    for y, shoe_name in ((-0.060, "slide_shoe_0"), (0.060, "slide_shoe_1")):
        carriage.visual(
            Box((0.180, 0.030, 0.025)),
            origin=Origin(xyz=(0.0, y, -0.0125)),
            material=bright_edge,
            name=shoe_name,
        )
    carriage.visual(
        Box((0.220, 0.180, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=carriage_blue,
        name="saddle_plate",
    )
    carriage.visual(
        Box((0.120, 0.110, 0.035)),
        origin=Origin(xyz=(0.020, 0.0, 0.0675)),
        material=carriage_blue,
        name="raised_boss",
    )
    for y, ear_name, cap_name in (
        (-0.070, "hinge_ear_0", "hinge_pin_cap_0"),
        (0.070, "hinge_ear_1", "hinge_pin_cap_1"),
    ):
        carriage.visual(
            Box((0.050, 0.035, 0.105)),
            origin=Origin(xyz=(0.105, y, 0.1025)),
            material=carriage_blue,
            name=ear_name,
        )
        carriage.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.115, y * 1.335, 0.1025), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name=cap_name,
        )

    hinged_frame = model.part("hinged_frame")
    hinged_frame.visual(
        Cylinder(radius=0.023, length=0.080),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bright_edge,
        name="hinge_barrel",
    )
    hinged_frame.visual(
        Box((0.060, 0.125, 0.035)),
        origin=Origin(xyz=(0.045, 0.0, -0.003)),
        material=frame_yellow,
        name="proximal_bridge",
    )
    for y, rail_name in ((-0.058, "side_rail_0"), (0.058, "side_rail_1")):
        hinged_frame.visual(
            Box((0.340, 0.022, 0.040)),
            origin=Origin(xyz=(0.195, y, -0.004)),
            material=frame_yellow,
            name=rail_name,
        )
    for z, guide_name, tie_name in (
        (-0.031, "ram_guide_strap_0", "distal_tie_0"),
        (0.023, "ram_guide_strap_1", "distal_tie_1"),
    ):
        hinged_frame.visual(
            Box((0.145, 0.130, 0.014)),
            origin=Origin(xyz=(0.295, 0.0, z)),
            material=frame_yellow,
            name=guide_name,
        )
        hinged_frame.visual(
            Box((0.045, 0.130, 0.014)),
            origin=Origin(xyz=(0.365, 0.0, z)),
            material=frame_yellow,
            name=tie_name,
        )

    ram = model.part("ram")
    ram.visual(
        Box((0.220, 0.045, 0.040)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material=ram_steel,
        name="ram_bar",
    )
    ram.visual(
        Box((0.035, 0.055, 0.035)),
        origin=Origin(xyz=(0.1975, 0.0, 0.0)),
        material=ram_steel,
        name="ram_nose",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=fixed_rail,
        child=carriage,
        origin=Origin(xyz=(-0.250, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.300),
    )
    model.articulation(
        "carriage_to_frame",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=hinged_frame,
        origin=Origin(xyz=(0.115, 0.0, 0.1025)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-0.25, upper=1.05),
    )
    model.articulation(
        "frame_to_ram",
        ArticulationType.PRISMATIC,
        parent=hinged_frame,
        child=ram,
        origin=Origin(xyz=(0.235, 0.0, -0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.140),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed_rail = object_model.get_part("fixed_rail")
    carriage = object_model.get_part("carriage")
    hinged_frame = object_model.get_part("hinged_frame")
    ram = object_model.get_part("ram")
    rail_slide = object_model.get_articulation("rail_to_carriage")
    nose_hinge = object_model.get_articulation("carriage_to_frame")
    ram_slide = object_model.get_articulation("frame_to_ram")

    ctx.check(
        "service axis has three commanded mechanisms",
        len(object_model.articulations) == 3,
        details=f"articulations={len(object_model.articulations)}",
    )
    ctx.expect_contact(
        carriage,
        fixed_rail,
        elem_a="slide_shoe_0",
        elem_b="guide_way_0",
        name="carriage shoe bears on fixed rail",
    )
    ctx.expect_contact(
        hinged_frame,
        carriage,
        elem_a="proximal_bridge",
        elem_b="hinge_ear_0",
        name="hinged frame is captured at carriage fork",
    )
    ctx.expect_contact(
        ram,
        hinged_frame,
        elem_a="ram_bar",
        elem_b="ram_guide_strap_1",
        name="ram bears against frame guide",
    )
    ctx.expect_within(
        ram,
        hinged_frame,
        axes="yz",
        inner_elem="ram_bar",
        outer_elem="side_rail_0",
        margin=0.09,
        name="ram runs inside the frame envelope",
    )
    ctx.expect_overlap(
        ram,
        hinged_frame,
        axes="x",
        elem_a="ram_bar",
        elem_b="side_rail_0",
        min_overlap=0.12,
        name="retracted ram remains guided in frame",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({rail_slide: 0.300}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            fixed_rail,
            axes="y",
            inner_elem="slide_shoe_0",
            outer_elem="guide_way_0",
            margin=0.002,
            name="carriage stays laterally aligned on rail",
        )
    ctx.check(
        "carriage slides along fixed rail",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.25,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    frame_rest_aabb = ctx.part_element_world_aabb(hinged_frame, elem="distal_tie_1")
    with ctx.pose({nose_hinge: 1.05}):
        frame_raised_aabb = ctx.part_element_world_aabb(hinged_frame, elem="distal_tie_1")

    def _center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) * 0.5

    ctx.check(
        "hinged nose rotates upward",
        _center_z(frame_rest_aabb) is not None
        and _center_z(frame_raised_aabb) is not None
        and _center_z(frame_raised_aabb) > _center_z(frame_rest_aabb) + 0.12,
        details=f"rest={frame_rest_aabb}, raised={frame_raised_aabb}",
    )

    ram_rest = ctx.part_world_position(ram)
    with ctx.pose({ram_slide: 0.140}):
        ram_extended = ctx.part_world_position(ram)
        ctx.expect_overlap(
            ram,
            hinged_frame,
            axes="x",
            elem_a="ram_bar",
            elem_b="side_rail_0",
            min_overlap=0.025,
            name="extended ram retains insertion in distal frame",
        )
    ctx.check(
        "ram extends from distal frame",
        ram_rest is not None and ram_extended is not None and ram_extended[0] > ram_rest[0] + 0.12,
        details=f"rest={ram_rest}, extended={ram_extended}",
    )

    return ctx.report()


object_model = build_object_model()
