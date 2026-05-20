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
    model = ArticulatedObject(name="metrology_bridge_axis")

    cast = model.material("warm_cast_grey", rgba=(0.45, 0.48, 0.48, 1.0))
    dark = model.material("matte_black", rgba=(0.03, 0.035, 0.035, 1.0))
    steel = model.material("ground_steel", rgba=(0.74, 0.76, 0.72, 1.0))
    blue = model.material("blue_accent", rgba=(0.05, 0.22, 0.54, 1.0))
    scale = model.material("etched_scale", rgba=(0.92, 0.88, 0.60, 1.0))

    # Root base: a flat metrology deck with low side frames and two long
    # precision guide rails running along X.
    base = model.part("base")
    base.visual(
        Box((1.46, 0.76, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast,
        name="deck",
    )
    for y, name in ((0.365, "side_frame_0"), (-0.365, "side_frame_1")):
        base.visual(
            Box((1.40, 0.055, 0.10)),
            origin=Origin(xyz=(0.0, y, 0.13)),
            material=cast,
            name=name,
        )
    base.visual(
        Box((1.22, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.23, 0.0975)),
        material=steel,
        name="rail_0",
    )
    base.visual(
        Box((1.20, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.255, 0.104)),
        material=dark,
        name="rail_groove_0",
    )
    base.visual(
        Box((1.22, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, -0.23, 0.0975)),
        material=steel,
        name="rail_1",
    )
    base.visual(
        Box((1.20, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.255, 0.104)),
        material=dark,
        name="rail_groove_1",
    )
    for x, name in ((0.66, "end_stop_0"), (-0.66, "end_stop_1")):
        base.visual(
            Box((0.055, 0.56, 0.105)),
            origin=Origin(xyz=(x, 0.0, 0.1325)),
            material=dark,
            name=name,
        )
    base.visual(
        Box((1.08, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -0.3365, 0.181)),
        material=scale,
        name="base_scale",
    )

    # Moving bridge carriage: bearing shoes sit on both base rails, tied by low
    # cross structure, uprights, and a high crossbeam.
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.24, 0.115, 0.07)),
        origin=Origin(xyz=(0.0, 0.23, 0.035)),
        material=blue,
        name="shoe_0",
    )
    carriage.visual(
        Box((0.24, 0.115, 0.07)),
        origin=Origin(xyz=(0.0, -0.23, 0.035)),
        material=blue,
        name="shoe_1",
    )
    carriage.visual(
        Box((0.16, 0.68, 0.06)),
        origin=Origin(xyz=(0.035, 0.0, 0.10)),
        material=cast,
        name="lower_tie",
    )
    for y, name in ((0.31, "upright_0"), (-0.31, "upright_1")):
        carriage.visual(
            Box((0.12, 0.08, 0.50)),
            origin=Origin(xyz=(0.0, y, 0.32)),
            material=cast,
            name=name,
        )
    carriage.visual(
        Box((0.18, 0.78, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=cast,
        name="crossbeam",
    )
    carriage.visual(
        Box((0.018, 0.72, 0.018)),
        origin=Origin(xyz=(-0.099, 0.0, 0.525)),
        material=steel,
        name="beam_guide_0",
    )
    carriage.visual(
        Box((0.018, 0.72, 0.018)),
        origin=Origin(xyz=(-0.099, 0.0, 0.595)),
        material=steel,
        name="beam_guide_1",
    )
    carriage.visual(
        Box((0.010, 0.62, 0.010)),
        origin=Origin(xyz=(-0.093, 0.0, 0.632)),
        material=scale,
        name="beam_scale",
    )

    # Compact center truck on the crossbeam guides.  It has short bearing pads
    # on both beam rails and a small metrology-probe mounting boss on its face.
    truck = model.part("truck")
    truck.visual(
        Box((0.046, 0.22, 0.22)),
        origin=Origin(xyz=(-0.039, 0.0, 0.0)),
        material=dark,
        name="body",
    )
    truck.visual(
        Box((0.016, 0.065, 0.028)),
        origin=Origin(xyz=(-0.008, 0.055, -0.035)),
        material=steel,
        name="lower_pad_0",
    )
    truck.visual(
        Box((0.016, 0.065, 0.028)),
        origin=Origin(xyz=(-0.008, 0.055, 0.035)),
        material=steel,
        name="upper_pad_0",
    )
    truck.visual(
        Box((0.016, 0.065, 0.028)),
        origin=Origin(xyz=(-0.008, -0.055, -0.035)),
        material=steel,
        name="lower_pad_1",
    )
    truck.visual(
        Box((0.016, 0.065, 0.028)),
        origin=Origin(xyz=(-0.008, -0.055, 0.035)),
        material=steel,
        name="upper_pad_1",
    )
    truck.visual(
        Box((0.050, 0.115, 0.105)),
        origin=Origin(xyz=(-0.087, 0.0, -0.025)),
        material=blue,
        name="probe_mount",
    )
    truck.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(-0.118, 0.0, -0.025), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="datum_socket",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=550.0, velocity=0.35, lower=-0.38, upper=0.38),
    )
    model.articulation(
        "carriage_to_truck",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=truck,
        origin=Origin(xyz=(-0.108, 0.0, 0.56)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.45, lower=-0.24, upper=0.24),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    truck = object_model.get_part("truck")
    base_slide = object_model.get_articulation("base_to_carriage")
    truck_slide = object_model.get_articulation("carriage_to_truck")

    ctx.check(
        "bridge axis has two prismatic stages",
        base_slide.articulation_type == ArticulationType.PRISMATIC
        and truck_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"base={base_slide.articulation_type}, truck={truck_slide.articulation_type}",
    )

    ctx.expect_contact(
        carriage,
        base,
        elem_a="shoe_0",
        elem_b="rail_0",
        contact_tol=1e-5,
        name="carriage shoe sits on first base rail",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a="shoe_1",
        elem_b="rail_1",
        contact_tol=1e-5,
        name="carriage shoe sits on second base rail",
    )
    ctx.expect_contact(
        truck,
        carriage,
        elem_a="lower_pad_0",
        elem_b="beam_guide_0",
        contact_tol=1e-5,
        name="truck lower pad rides on beam guide",
    )
    ctx.expect_contact(
        truck,
        carriage,
        elem_a="upper_pad_0",
        elem_b="beam_guide_1",
        contact_tol=1e-5,
        name="truck upper pad rides on beam guide",
    )

    ctx.expect_within(
        carriage,
        base,
        axes="x",
        inner_elem="shoe_0",
        outer_elem="rail_0",
        margin=0.0,
        name="carriage shoe retained on rail at center",
    )
    ctx.expect_within(
        truck,
        carriage,
        axes="y",
        inner_elem="body",
        outer_elem="beam_guide_0",
        margin=0.0,
        name="truck body retained on crossbeam guide at center",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({base_slide: 0.38}):
        ctx.expect_within(
            carriage,
            base,
            axes="x",
            inner_elem="shoe_0",
            outer_elem="rail_0",
            margin=0.0,
            name="carriage remains on base rail at travel end",
        )
        carriage_extended = ctx.part_world_position(carriage)
    ctx.check(
        "carriage translates along base rails",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.30,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    truck_rest = ctx.part_world_position(truck)
    with ctx.pose({truck_slide: 0.24}):
        ctx.expect_within(
            truck,
            carriage,
            axes="y",
            inner_elem="body",
            outer_elem="beam_guide_0",
            margin=0.0,
            name="truck remains on crossbeam guide at travel end",
        )
        truck_extended = ctx.part_world_position(truck)
    ctx.check(
        "truck translates across crossbeam",
        truck_rest is not None
        and truck_extended is not None
        and truck_extended[1] > truck_rest[1] + 0.18,
        details=f"rest={truck_rest}, extended={truck_extended}",
    )

    return ctx.report()


object_model = build_object_model()
