from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_gas_stove_top")

    black_glass = _mat("black_glass", (0.015, 0.018, 0.020, 1.0))
    stainless = _mat("brushed_stainless", (0.70, 0.68, 0.62, 1.0))
    cast_iron = _mat("matte_cast_iron", (0.005, 0.005, 0.004, 1.0))
    burner_dark = _mat("burner_dark", (0.030, 0.030, 0.028, 1.0))
    white_mark = _mat("white_marking", (0.92, 0.90, 0.82, 1.0))
    red_mark = _mat("red_ignition", (0.95, 0.08, 0.04, 1.0))
    amber_clear = _mat("smoked_clear_cover", (0.55, 0.38, 0.14, 0.38))
    knob_black = _mat("satin_black_plastic", (0.020, 0.019, 0.017, 1.0))

    hob = model.part("hob")

    # Countertop-scale built-in cooktop deck: 740 mm wide by 500 mm deep.
    hob.visual(
        Box((0.740, 0.500, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=black_glass,
        name="glass_deck",
    )
    hob.visual(
        Box((0.760, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, -0.255, 0.021)),
        material=stainless,
        name="front_trim",
    )
    hob.visual(
        Box((0.760, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.251, 0.020)),
        material=stainless,
        name="rear_trim",
    )
    for x, name in [(-0.379, "side_trim_0"), (0.379, "side_trim_1")]:
        hob.visual(
            Box((0.018, 0.500, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.020)),
            material=stainless,
            name=name,
        )

    # Narrow front rail carrying all user controls.
    hob.visual(
        Box((0.760, 0.075, 0.065)),
        origin=Origin(xyz=(0.0, -0.2875, 0.0325)),
        material=stainless,
        name="front_rail",
    )
    hob.visual(
        Box((0.220, 0.037, 0.0018)),
        origin=Origin(xyz=(0.0, -0.286, 0.0640)),
        material=burner_dark,
        name="ignition_recess",
    )

    # Burner assemblies: drip pans, torus burner heads, caps, grates, and feet.
    burner_positions = [
        (-0.185, -0.105),
        (0.185, -0.105),
        (-0.185, 0.115),
        (0.185, 0.115),
    ]
    drip_pan_mesh = mesh_from_geometry(TorusGeometry(0.062, 0.006, radial_segments=16, tubular_segments=48), "drip_pan_ring")
    burner_head_mesh = mesh_from_geometry(TorusGeometry(0.043, 0.004, radial_segments=12, tubular_segments=40), "burner_head_ports")
    for i, (x, y) in enumerate(burner_positions):
        hob.visual(
            Cylinder(radius=0.070, length=0.004),
            origin=Origin(xyz=(x, y, 0.020)),
            material=stainless,
            name=f"burner_pan_{i}",
        )
        hob.visual(
            drip_pan_mesh,
            origin=Origin(xyz=(x, y, 0.023)),
            material=stainless,
            name=f"pan_lip_{i}",
        )
        hob.visual(
            burner_head_mesh,
            origin=Origin(xyz=(x, y, 0.026)),
            material=burner_dark,
            name=f"burner_ring_{i}",
        )
        hob.visual(
            Cylinder(radius=0.031, length=0.008),
            origin=Origin(xyz=(x, y, 0.028)),
            material=burner_dark,
            name=f"burner_cap_{i}",
        )
        hob.visual(
            Cylinder(radius=0.034, length=0.008),
            origin=Origin(xyz=(x, y, 0.022)),
            material=burner_dark,
            name=f"burner_core_{i}",
        )
        hob.visual(
            Box((0.160, 0.014, 0.012)),
            origin=Origin(xyz=(x, y, 0.038)),
            material=cast_iron,
            name=f"grate_bar_x_{i}",
        )
        hob.visual(
            Box((0.014, 0.160, 0.012)),
            origin=Origin(xyz=(x, y, 0.038)),
            material=cast_iron,
            name=f"grate_bar_y_{i}",
        )
        for j, (dx, dy) in enumerate(((0.070, 0.0), (-0.070, 0.0), (0.0, 0.070), (0.0, -0.070))):
            hob.visual(
                Box((0.019, 0.019, 0.020)),
                origin=Origin(xyz=(x + dx, y + dy, 0.027)),
                material=cast_iron,
                name=f"grate_foot_{i}_{j}",
            )

    # Hinge bracket and concealed ignition markings on the rail top.
    hob.visual(
        Box((0.250, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, -0.252, 0.0665)),
        material=stainless,
        name="cover_hinge_leaf",
    )
    for x, name in [(-0.108, "cover_hinge_bracket_0"), (0.108, "cover_hinge_bracket_1")]:
        hob.visual(
            Box((0.018, 0.018, 0.010)),
            origin=Origin(xyz=(x, -0.252, 0.070)),
            material=stainless,
            name=name,
        )
    for x, name in [(-0.040, "ignition_mark_0"), (0.040, "ignition_mark_1")]:
        hob.visual(
            Box((0.026, 0.001, 0.003)),
            origin=Origin(xyz=(x, -0.306, 0.0663)),
            material=red_mark,
            name=name,
        )

    # Four front-face knob tick marks.
    knob_xs = (-0.285, -0.095, 0.095, 0.285)
    for i, x in enumerate(knob_xs):
        hob.visual(
            Box((0.030, 0.001, 0.003)),
            origin=Origin(xyz=(x, -0.3255, 0.058)),
            material=white_mark,
            name=f"knob_tick_{i}",
        )

    # Two spring-loaded ignition push buttons live under the small safety cover.
    for i, x in enumerate((-0.040, 0.040)):
        switch = model.part(f"ignition_switch_{i}")
        switch.visual(
            Box((0.045, 0.022, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=red_mark,
            name="switch_cap",
        )
        model.articulation(
            f"hob_to_ignition_switch_{i}",
            ArticulationType.PRISMATIC,
            parent=hob,
            child=switch,
            origin=Origin(xyz=(x, -0.286, 0.065)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.04, lower=0.0, upper=0.004),
        )

    # Smoked translucent hinged cover over the ignition switches.
    cover = model.part("safety_cover")
    cover.visual(
        Box((0.235, 0.065, 0.004)),
        origin=Origin(xyz=(0.0, -0.0325, 0.0005)),
        material=amber_clear,
        name="cover_panel",
    )
    cover.visual(
        Cylinder(radius=0.004, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="cover_hinge_barrel",
    )
    model.articulation(
        "hob_to_safety_cover",
        ArticulationType.REVOLUTE,
        parent=hob,
        child=cover,
        origin=Origin(xyz=(0.0, -0.252, 0.072)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.8, lower=0.0, upper=1.65),
    )

    # Four continuously rotating front-facing burner knobs.
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.050,
            0.024,
            body_style="skirted",
            top_diameter=0.038,
            skirt=KnobSkirt(0.058, 0.006, flare=0.08, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            bore=KnobBore(style="d_shaft", diameter=0.007, flat_depth=0.001),
        ),
        "front_burner_knob",
    )
    for i, x in enumerate(knob_xs):
        knob = model.part(f"burner_knob_{i}")
        knob.visual(
            Cylinder(radius=0.006, length=0.026),
            origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name="shaft",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, -0.038, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="knob_cap",
        )
        model.articulation(
            f"hob_to_burner_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=hob,
            child=knob,
            origin=Origin(xyz=(x, -0.325, 0.038)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hob = object_model.get_part("hob")
    cover = object_model.get_part("safety_cover")
    cover_joint = object_model.get_articulation("hob_to_safety_cover")

    knob_joints = [
        object_model.get_articulation(f"hob_to_burner_knob_{i}") for i in range(4)
    ]
    ctx.check(
        "four continuous burner knobs",
        len(knob_joints) == 4
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in knob_joints),
        details="Expected four continuous front-facing burner knob joints.",
    )
    ctx.check(
        "knob shafts face forward",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, -1.0, 0.0) for j in knob_joints),
        details="Burner knob axes should be normal to the front rail.",
    )

    ctx.expect_gap(
        cover,
        hob,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="front_rail",
        min_gap=0.002,
        name="closed safety cover clears rail top",
    )
    ctx.expect_overlap(
        cover,
        "ignition_switch_0",
        axes="xy",
        elem_a="cover_panel",
        elem_b="switch_cap",
        min_overlap=0.010,
        name="safety cover spans ignition switches",
    )
    ctx.expect_gap(
        cover,
        "ignition_switch_0",
        axis="z",
        positive_elem="cover_panel",
        negative_elem="switch_cap",
        min_gap=0.0005,
        max_gap=0.004,
        name="cover has close clearance above switch caps",
    )
    for i in range(4):
        ctx.expect_gap(
            hob,
            f"burner_knob_{i}",
            axis="y",
            positive_elem="front_rail",
            negative_elem="shaft",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"knob shaft {i} seats on front rail",
        )

    closed_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_joint: 1.20}):
        opened_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "safety cover opens upward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.035,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
