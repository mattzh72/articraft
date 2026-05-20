from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_beam_yz_axis")

    painted = model.material("painted_support_blue", color=(0.12, 0.23, 0.34, 1.0))
    slide_paint = model.material("moving_crosshead_teal", color=(0.06, 0.36, 0.42, 1.0))
    dark = model.material("blackened_steel", color=(0.02, 0.025, 0.025, 1.0))
    steel = model.material("brushed_steel", color=(0.67, 0.69, 0.66, 1.0))
    wear = model.material("dark_wear_pad", color=(0.08, 0.08, 0.075, 1.0))
    safety = model.material("yellow_end_stop", color=(0.95, 0.72, 0.10, 1.0))

    support = model.part("top_support")
    support.visual(
        Box((0.64, 1.95, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.67)),
        material=painted,
        name="ceiling_plate",
    )
    support.visual(
        Box((0.54, 1.78, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        material=painted,
        name="main_beam",
    )
    support.visual(
        Box((0.08, 1.62, 0.18)),
        origin=Origin(xyz=(-0.28, 0.0, 1.49)),
        material=painted,
        name="side_web_0",
    )
    support.visual(
        Box((0.08, 1.62, 0.18)),
        origin=Origin(xyz=(0.28, 0.0, 1.49)),
        material=painted,
        name="side_web_1",
    )
    support.visual(
        Cylinder(radius=0.035, length=1.66),
        origin=Origin(xyz=(-0.17, 0.0, 1.426), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rail_0",
    )
    support.visual(
        Cylinder(radius=0.035, length=1.66),
        origin=Origin(xyz=(0.17, 0.0, 1.426), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rail_1",
    )
    support.visual(
        Cylinder(radius=0.018, length=1.54),
        origin=Origin(xyz=(0.0, 0.0, 1.390), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="drive_screw",
    )
    for y, name in ((-0.72, "bearing_0"), (0.72, "bearing_1")):
        support.visual(
            Box((0.16, 0.08, 0.12)),
            origin=Origin(xyz=(0.0, y, 1.405)),
            material=painted,
            name=name,
        )
    for y, name in ((-0.86, "travel_stop_0"), (0.86, "travel_stop_1")):
        support.visual(
            Box((0.42, 0.045, 0.10)),
            origin=Origin(xyz=(0.0, y, 1.410)),
            material=safety,
            name=name,
        )

    crosshead = model.part("crosshead")
    crosshead.visual(
        Box((0.68, 0.28, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=slide_paint,
        name="cross_plate",
    )
    crosshead.visual(
        Box((0.14, 0.24, 0.04)),
        origin=Origin(xyz=(-0.17, 0.0, 0.071)),
        material=wear,
        name="shoe_0",
    )
    crosshead.visual(
        Box((0.14, 0.24, 0.04)),
        origin=Origin(xyz=(0.17, 0.0, 0.071)),
        material=wear,
        name="shoe_1",
    )
    crosshead.visual(
        Box((0.14, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=dark,
        name="screw_nut",
    )
    crosshead.visual(
        Box((0.10, 0.30, 0.51)),
        origin=Origin(xyz=(-0.135, 0.0, -0.305)),
        material=slide_paint,
        name="sleeve_side_0",
    )
    crosshead.visual(
        Box((0.10, 0.30, 0.51)),
        origin=Origin(xyz=(0.135, 0.0, -0.305)),
        material=slide_paint,
        name="sleeve_side_1",
    )
    crosshead.visual(
        Box((0.37, 0.07, 0.51)),
        origin=Origin(xyz=(0.0, -0.105, -0.305)),
        material=slide_paint,
        name="sleeve_face_0",
    )
    crosshead.visual(
        Box((0.37, 0.07, 0.51)),
        origin=Origin(xyz=(0.0, 0.105, -0.305)),
        material=slide_paint,
        name="sleeve_face_1",
    )

    ram = model.part("ram")
    ram.visual(
        Box((0.17, 0.14, 0.86)),
        origin=Origin(xyz=(0.0, 0.0, -0.373)),
        material=steel,
        name="ram_body",
    )
    ram.visual(
        Box((0.15, 0.10, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=dark,
        name="upper_keeper",
    )
    ram.visual(
        Cylinder(radius=0.055, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.844)),
        material=dark,
        name="tool_socket",
    )
    ram.visual(
        Box((0.24, 0.18, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.912)),
        material=dark,
        name="tool_pad",
    )

    model.articulation(
        "support_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=support,
        child=crosshead,
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.45, lower=-0.48, upper=0.48),
        motion_properties=MotionProperties(damping=30.0, friction=10.0),
    )
    model.articulation(
        "crosshead_to_ram",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=ram,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.30, lower=0.0, upper=0.38),
        motion_properties=MotionProperties(damping=24.0, friction=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    crosshead = object_model.get_part("crosshead")
    ram = object_model.get_part("ram")
    y_slide = object_model.get_articulation("support_to_crosshead")
    z_slide = object_model.get_articulation("crosshead_to_ram")

    ctx.check(
        "crosshead is horizontal prismatic",
        y_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"type={y_slide.articulation_type}, axis={y_slide.axis}",
    )
    ctx.check(
        "ram is vertical prismatic",
        z_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in z_slide.axis) == (0.0, 0.0, -1.0),
        details=f"type={z_slide.articulation_type}, axis={z_slide.axis}",
    )

    ctx.expect_gap(
        support,
        crosshead,
        axis="z",
        positive_elem="rail_0",
        negative_elem="shoe_0",
        min_gap=-0.00001,
        max_gap=0.006,
        name="left shoe runs directly below rail",
    )
    ctx.expect_gap(
        support,
        crosshead,
        axis="z",
        positive_elem="rail_1",
        negative_elem="shoe_1",
        min_gap=-0.00001,
        max_gap=0.006,
        name="right shoe runs directly below rail",
    )
    ctx.expect_overlap(
        support,
        crosshead,
        axes="xy",
        elem_a="rail_0",
        elem_b="shoe_0",
        min_overlap=0.06,
        name="left rail and shoe footprint overlap",
    )
    ctx.expect_within(
        ram,
        crosshead,
        axes="xy",
        inner_elem="ram_body",
        outer_elem="sleeve_face_0",
        margin=0.20,
        name="ram is centered in the crosshead sleeve plan",
    )
    ctx.expect_overlap(
        ram,
        crosshead,
        axes="z",
        elem_a="ram_body",
        elem_b="sleeve_side_0",
        min_overlap=0.45,
        name="retracted ram remains guided by sleeve",
    )

    rest_crosshead = ctx.part_world_position(crosshead)
    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        extended_crosshead = ctx.part_world_position(crosshead)
        ctx.expect_within(
            crosshead,
            support,
            axes="y",
            inner_elem="shoe_0",
            outer_elem="rail_0",
            margin=0.0,
            name="crosshead shoe stays on rail at y travel limit",
        )
    ctx.check(
        "crosshead moves along y",
        rest_crosshead is not None
        and extended_crosshead is not None
        and extended_crosshead[1] > rest_crosshead[1] + 0.40,
        details=f"rest={rest_crosshead}, extended={extended_crosshead}",
    )

    rest_ram = ctx.part_world_position(ram)
    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        lowered_ram = ctx.part_world_position(ram)
        ctx.expect_overlap(
            ram,
            crosshead,
            axes="z",
            elem_a="ram_body",
            elem_b="sleeve_side_0",
            min_overlap=0.10,
            name="lowered ram still retained in sleeve",
        )
    ctx.check(
        "ram moves downward",
        rest_ram is not None
        and lowered_ram is not None
        and lowered_ram[2] < rest_ram[2] - 0.30,
        details=f"rest={rest_ram}, lowered={lowered_ram}",
    )

    return ctx.report()


object_model = build_object_model()
