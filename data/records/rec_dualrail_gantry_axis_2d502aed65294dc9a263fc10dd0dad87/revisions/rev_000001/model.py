from __future__ import annotations

import math

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
    model = ArticulatedObject(name="under_beam_gantry_shuttle")

    concrete = Material("brushed_concrete", rgba=(0.55, 0.55, 0.52, 1.0))
    rail_steel = Material("polished_rail_steel", rgba=(0.42, 0.45, 0.48, 1.0))
    dark_steel = Material("dark_structural_steel", rgba=(0.10, 0.12, 0.14, 1.0))
    bridge_blue = Material("painted_bridge_blue", rgba=(0.05, 0.20, 0.42, 1.0))
    safety_yellow = Material("safety_yellow", rgba=(0.95, 0.72, 0.08, 1.0))
    rubber = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    rail_pair = model.part("rail_pair")
    rail_pair.visual(
        Box((3.40, 1.78, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=concrete,
        name="floor_slab",
    )
    for x in (-1.35, -0.68, 0.0, 0.68, 1.35):
        rail_pair.visual(
            Box((0.10, 1.55, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0725)),
            material=dark_steel,
            name=f"sleeper_{x:+.2f}",
        )
    rail_pair.visual(
        Box((3.28, 0.075, 0.10)),
        origin=Origin(xyz=(0.0, -0.68, 0.145)),
        material=rail_steel,
        name="rail_0",
    )
    rail_pair.visual(
        Box((3.28, 0.15, 0.025)),
        origin=Origin(xyz=(0.0, -0.68, 0.0825)),
        material=dark_steel,
        name="rail_0_foot",
    )
    rail_pair.visual(
        Box((3.28, 0.075, 0.10)),
        origin=Origin(xyz=(0.0, 0.68, 0.145)),
        material=rail_steel,
        name="rail_1",
    )
    rail_pair.visual(
        Box((3.28, 0.15, 0.025)),
        origin=Origin(xyz=(0.0, 0.68, 0.0825)),
        material=dark_steel,
        name="rail_1_foot",
    )
    for x, name in ((-1.67, "stop_0"), (1.67, "stop_1")):
        rail_pair.visual(
            Box((0.08, 1.62, 0.22)),
            origin=Origin(xyz=(x, 0.0, 0.16)),
            material=safety_yellow,
            name=name,
        )

    bridge = model.part("bridge")
    # End trucks ride on the two grounded rails and carry the portal bridge.
    for y, suffix in ((-0.68, "0"), (0.68, "1")):
        bridge.visual(
            Box((0.52, 0.22, 0.12)),
            origin=Origin(xyz=(0.0, y, 0.28)),
            material=bridge_blue,
            name=f"truck_{suffix}",
        )
        if suffix == "0":
            bridge.visual(
                Cylinder(radius=0.045, length=0.070),
                origin=Origin(xyz=(-0.17, y, 0.240), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rubber,
                name="wheel_0_n",
            )
        else:
            bridge.visual(
                Cylinder(radius=0.045, length=0.070),
                origin=Origin(xyz=(-0.17, y, 0.240), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rubber,
                name="wheel_1_n",
            )
        bridge.visual(
            Cylinder(radius=0.045, length=0.070),
            origin=Origin(xyz=(0.17, y, 0.240), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=("wheel_0_p" if suffix == "0" else "wheel_1_p"),
        )
        bridge.visual(
            Box((0.15, 0.12, 1.08)),
            origin=Origin(xyz=(0.0, y, 0.84)),
            material=bridge_blue,
            name=f"upright_{suffix}",
        )

    # A shallow I-beam spans across the bridge; the shuttle runs underneath it.
    bridge.visual(
        Box((0.34, 1.78, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.555)),
        material=bridge_blue,
        name="top_flange",
    )
    bridge.visual(
        Box((0.11, 1.78, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 1.455)),
        material=bridge_blue,
        name="web",
    )
    bridge.visual(
        Box((0.30, 1.78, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.355)),
        material=bridge_blue,
        name="bottom_flange",
    )
    bridge.visual(
        Box((0.40, 0.08, 0.20)),
        origin=Origin(xyz=(0.0, -0.92, 1.44)),
        material=safety_yellow,
        name="end_buffer_0",
    )
    bridge.visual(
        Box((0.40, 0.08, 0.20)),
        origin=Origin(xyz=(0.0, 0.92, 1.44)),
        material=safety_yellow,
        name="end_buffer_1",
    )

    carriage = model.part("hanging_carriage")
    carriage.visual(
        Box((0.36, 0.25, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="trolley_body",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(-0.13, -0.08, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="under_roller_n_n",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(-0.13, 0.08, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="under_roller_n_p",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.13, -0.08, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="under_roller_p_n",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.13, 0.08, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="under_roller_p_p",
    )
    carriage.visual(
        Box((0.08, 0.08, 0.44)),
        origin=Origin(xyz=(0.0, 0.0, -0.260)),
        material=safety_yellow,
        name="hanger_stem",
    )
    carriage.visual(
        Box((0.30, 0.20, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, -0.56)),
        material=safety_yellow,
        name="load_block",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, -0.69), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cross_pin",
    )
    carriage.visual(
        Box((0.06, 0.18, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, -0.82)),
        material=dark_steel,
        name="lower_lug",
    )

    model.articulation(
        "rail_to_bridge",
        ArticulationType.PRISMATIC,
        parent=rail_pair,
        child=bridge,
        origin=Origin(xyz=(-0.55, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.65, lower=0.0, upper=1.10),
    )
    model.articulation(
        "bridge_to_carriage",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.45, 1.2125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.90),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_pair = object_model.get_part("rail_pair")
    bridge = object_model.get_part("bridge")
    carriage = object_model.get_part("hanging_carriage")
    bridge_slide = object_model.get_articulation("rail_to_bridge")
    carriage_slide = object_model.get_articulation("bridge_to_carriage")

    ctx.check(
        "bridge and carriage are prismatic joints",
        bridge_slide.articulation_type == ArticulationType.PRISMATIC
        and carriage_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"types={bridge_slide.articulation_type}, {carriage_slide.articulation_type}",
    )
    dot = sum(a * b for a, b in zip(bridge_slide.axis, carriage_slide.axis))
    ctx.check(
        "gantry guide directions are perpendicular",
        abs(dot) < 1e-6 and bridge_slide.axis == (1.0, 0.0, 0.0) and carriage_slide.axis == (0.0, 1.0, 0.0),
        details=f"bridge_axis={bridge_slide.axis}, carriage_axis={carriage_slide.axis}, dot={dot}",
    )

    ctx.expect_contact(
        bridge,
        rail_pair,
        elem_a="wheel_0_n",
        elem_b="rail_0",
        contact_tol=0.0005,
        name="bridge wheel is seated on grounded rail",
    )
    ctx.expect_contact(
        carriage,
        bridge,
        elem_a="under_roller_n_n",
        elem_b="bottom_flange",
        contact_tol=0.0005,
        name="carriage roller hangs from underside of bridge beam",
    )
    ctx.expect_gap(
        bridge,
        carriage,
        axis="z",
        positive_elem="bottom_flange",
        negative_elem="trolley_body",
        min_gap=0.03,
        name="carriage body is below the beam rather than riding on top",
    )

    bridge_rest = ctx.part_world_position(bridge)
    with ctx.pose({bridge_slide: 1.10}):
        bridge_extended = ctx.part_world_position(bridge)
        ctx.expect_contact(
            bridge,
            rail_pair,
            elem_a="wheel_0_n",
            elem_b="rail_0",
            contact_tol=0.0005,
            name="bridge remains supported at rail end travel",
        )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.90}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            bridge,
            elem_a="under_roller_n_n",
            elem_b="bottom_flange",
            contact_tol=0.0005,
            name="hanging carriage remains under-beam at cross travel",
        )
        ctx.expect_gap(
            bridge,
            carriage,
            axis="z",
            positive_elem="bottom_flange",
            negative_elem="trolley_body",
            min_gap=0.03,
            name="carriage stays below beam across its travel",
        )

    ctx.check(
        "bridge slide moves along rail direction",
        bridge_rest is not None
        and bridge_extended is not None
        and bridge_extended[0] > bridge_rest[0] + 1.0
        and abs(bridge_extended[1] - bridge_rest[1]) < 1e-6,
        details=f"rest={bridge_rest}, extended={bridge_extended}",
    )
    ctx.check(
        "carriage slide moves across bridge direction",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[1] > carriage_rest[1] + 0.8
        and abs(carriage_extended[0] - carriage_rest[0]) < 1e-6,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    return ctx.report()


object_model = build_object_model()
