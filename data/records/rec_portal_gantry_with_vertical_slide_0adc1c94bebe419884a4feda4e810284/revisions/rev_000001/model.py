from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_portal_gantry")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_steel = model.material("brushed_linear_rail_steel", rgba=(0.65, 0.68, 0.70, 1.0))
    black = model.material("black_bearing_blocks", rgba=(0.015, 0.017, 0.018, 1.0))
    orange = model.material("orange_safety_plate", rgba=(0.95, 0.42, 0.08, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("frame")
    # Wide feet and two side uprights make the object read as a small tabletop bridge.
    for x in (-0.55, 0.55):
        frame.visual(
            Box((0.18, 0.44, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.0175)),
            material=anodized,
            name=f"foot_{'neg' if x < 0 else 'pos'}",
        )
        frame.visual(
            Box((0.09, 0.30, 0.61)),
            origin=Origin(xyz=(x, 0.0, 0.325)),
            material=anodized,
            name=f"upright_{'neg' if x < 0 else 'pos'}",
        )
        frame.visual(
            Box((0.14, 0.36, 0.020)),
            origin=Origin(xyz=(x, 0.0, 0.010)),
            material=rubber,
            name=f"rubber_pad_{'neg' if x < 0 else 'pos'}",
        )

    frame.visual(
        Box((1.28, 0.12, 0.13)),
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        material=anodized,
        name="top_beam",
    )
    # Front-mounted X guide paths: two bright rails and a darker belt/rack strip.
    frame.visual(
        Box((1.08, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.066, 0.720)),
        material=rail_steel,
        name="x_rail_upper",
    )
    frame.visual(
        Box((1.08, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.066, 0.650)),
        material=rail_steel,
        name="x_rail_lower",
    )
    frame.visual(
        Box((1.12, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.065, 0.685)),
        material=black,
        name="x_belt_track",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.22, 0.036, 0.23)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="carriage_plate",
    )
    # Four bearing shoes line up just in front of the X rails.
    carriage.visual(
        Box((0.080, 0.014, 0.032)),
        origin=Origin(xyz=(-0.060, 0.033, 0.035)),
        material=rail_steel,
        name="x_bearing_upper_a",
    )
    carriage.visual(
        Box((0.080, 0.014, 0.032)),
        origin=Origin(xyz=(0.060, 0.033, 0.035)),
        material=rail_steel,
        name="x_bearing_upper_b",
    )
    carriage.visual(
        Box((0.080, 0.014, 0.032)),
        origin=Origin(xyz=(-0.060, 0.033, -0.035)),
        material=rail_steel,
        name="x_bearing_lower_a",
    )
    carriage.visual(
        Box((0.080, 0.014, 0.032)),
        origin=Origin(xyz=(0.060, 0.033, -0.035)),
        material=rail_steel,
        name="x_bearing_lower_b",
    )
    for sx, sz, standoff_name in (
        (-0.060, 0.035, "x_standoff_upper_a"),
        (0.060, 0.035, "x_standoff_upper_b"),
        (-0.060, -0.035, "x_standoff_lower_a"),
        (0.060, -0.035, "x_standoff_lower_b"),
    ):
        carriage.visual(
            Box((0.070, 0.008, 0.026)),
            origin=Origin(xyz=(sx, 0.022, sz)),
            material=black,
            name=standoff_name,
        )
    # Stationary vertical bearing blocks on the carriage hold the hanging ram.
    carriage.visual(
        Box((0.030, 0.062, 0.180)),
        origin=Origin(xyz=(-0.060, -0.045, -0.070)),
        material=rail_steel,
        name="z_guide_a",
    )
    carriage.visual(
        Box((0.030, 0.062, 0.180)),
        origin=Origin(xyz=(0.060, -0.045, -0.070)),
        material=rail_steel,
        name="z_guide_b",
    )
    carriage.visual(
        Box((0.17, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -0.027, 0.045)),
        material=anodized,
        name="upper_ram_bridge",
    )

    ram = model.part("ram")
    # The ram extends upward past its joint frame so it remains captured at full down travel.
    ram.visual(
        Box((0.090, 0.036, 0.470)),
        origin=Origin(xyz=(0.0, -0.010, -0.105)),
        material=anodized,
        name="ram_body",
    )
    # Visible vertical guide paths on the hanging slide.
    for x in (-0.032, 0.032):
        ram.visual(
            Box((0.012, 0.010, 0.390)),
            origin=Origin(xyz=(x, -0.033, -0.110)),
            material=rail_steel,
            name=f"z_rail_{'a' if x < 0 else 'b'}",
        )
    ram.visual(
        Box((0.140, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, -0.012, -0.330)),
        material=orange,
        name="tool_plate",
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(-0.32, -0.112, 0.685)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.64),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=ram,
        origin=Origin(xyz=(0.0, -0.060, -0.060)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    ram = object_model.get_part("ram")
    x_slide = object_model.get_articulation("x_slide")
    z_slide = object_model.get_articulation("z_slide")

    # The carriage bearing shoes are close to, but not intersecting, the beam rails.
    for rail, shoe in (("x_rail_upper", "x_bearing_upper_a"), ("x_rail_lower", "x_bearing_lower_a")):
        ctx.expect_gap(
            frame,
            carriage,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0005,
            positive_elem=rail,
            negative_elem=shoe,
            name=f"{shoe} runs just in front of {rail}",
        )
        ctx.expect_overlap(
            frame,
            carriage,
            axes="z",
            min_overlap=0.012,
            elem_a=rail,
            elem_b=shoe,
            name=f"{shoe} aligns vertically with {rail}",
        )

    with ctx.pose({x_slide: 0.64}):
        ctx.expect_within(
            carriage,
            frame,
            axes="x",
            inner_elem="x_bearing_upper_b",
            outer_elem="x_rail_upper",
            margin=0.0,
            name="carriage remains on the X rail at far travel",
        )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({x_slide: 0.64}):
        far_carriage = ctx.part_world_position(carriage)
    ctx.check(
        "carriage travels along the beam",
        rest_carriage is not None and far_carriage is not None and far_carriage[0] > rest_carriage[0] + 0.50,
        details=f"rest={rest_carriage}, far={far_carriage}",
    )

    # The hanging ram remains captured in the vertical guide blocks even when lowered.
    ctx.expect_overlap(
        carriage,
        ram,
        axes="z",
        min_overlap=0.14,
        elem_a="z_guide_a",
        elem_b="ram_body",
        name="ram is substantially engaged in the Z guide at rest",
    )
    rest_ram = ctx.part_world_position(ram)
    with ctx.pose({z_slide: 0.20}):
        ctx.expect_overlap(
            carriage,
            ram,
            axes="z",
            min_overlap=0.025,
            elem_a="z_guide_a",
            elem_b="ram_body",
            name="ram stays retained in the Z guide when lowered",
        )
        lowered_ram = ctx.part_world_position(ram)
    ctx.check(
        "ram travels downward from the carriage",
        rest_ram is not None and lowered_ram is not None and lowered_ram[2] < rest_ram[2] - 0.15,
        details=f"rest={rest_ram}, lowered={lowered_ram}",
    )

    return ctx.report()


object_model = build_object_model()
