from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
    mesh_from_geometry,
)


TOP_Z = 0.040
HINGE_Y = 0.262
HINGE_Z = 0.095


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    return shape.edges("|Z").fillet(radius)


def _annular_disc(outer_diameter: float, inner_diameter: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_diameter / 2.0)
        .circle(inner_diameter / 2.0)
        .extrude(height)
    )


def _solid_disc(diameter: float, height: float, bevel: float = 0.001) -> cq.Workplane:
    return cq.Workplane("XY").circle(diameter / 2.0).extrude(height)


def _translated_box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(xyz)


def _union_all(solids: list[cq.Workplane]) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _grate_geometry() -> cq.Workplane:
    """One connected cast-iron pan-support grate with feet reaching the glass top."""
    zc = TOP_Z + 0.034
    bar_h = 0.018
    bar_w = 0.018
    x_span = 0.780
    y_span = 0.355
    solids: list[cq.Workplane] = []

    # Perimeter rectangle.
    solids.append(_translated_box((x_span, bar_w, bar_h), (0.0, -y_span / 2.0, zc)))
    solids.append(_translated_box((x_span, bar_w, bar_h), (0.0, y_span / 2.0, zc)))
    solids.append(_translated_box((bar_w, y_span, bar_h), (-x_span / 2.0, 0.0, zc)))
    solids.append(_translated_box((bar_w, y_span, bar_h), (x_span / 2.0, 0.0, zc)))

    # Continuous cross members tying the five burner positions together.
    for x in (-0.185, 0.0, 0.185):
        solids.append(_translated_box((bar_w, y_span, bar_h), (x, 0.0, zc)))
    for y in (-0.105, 0.025, 0.145):
        solids.append(_translated_box((x_span, bar_w, bar_h), (0.0, y, zc)))

    # Short raised pot fingers around each burner; every finger overlaps a main rail.
    burner_supports = [
        (-0.265, 0.105, 0.070),
        (0.265, 0.105, 0.070),
        (-0.260, -0.105, 0.066),
        (0.260, -0.105, 0.066),
        (0.0, 0.000, 0.092),
    ]
    for x, y, span in burner_supports:
        solids.append(_translated_box((span, 0.014, bar_h), (x, y, zc + 0.002)))
        solids.append(_translated_box((0.014, span, bar_h), (x, y, zc + 0.002)))

    # Feet embed very slightly into the glass, so the visual assembly reads as mounted.
    foot_h = zc - TOP_Z
    for x in (-0.365, -0.185, 0.0, 0.185, 0.365):
        for y in (-0.158, 0.158):
            solids.append(_translated_box((0.024, 0.024, foot_h + 0.004), (x, y, TOP_Z + foot_h / 2.0 - 0.002)))

    return _union_all(solids)


def _knob_mesh(name: str):
    knob = KnobGeometry(
        0.050,
        0.026,
        body_style="skirted",
        top_diameter=0.038,
        crown_radius=0.0012,
        edge_radius=0.0010,
        skirt=KnobSkirt(0.058, 0.006, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=20, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
        bore=KnobBore(style="d_shaft", diameter=0.007, flat_depth=0.001),
        center=False,
    )
    return mesh_from_geometry(knob, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_gas_hob")

    glass = model.material("black_ceramic_glass", rgba=(0.010, 0.012, 0.014, 1.0))
    smoked_glass = model.material("smoked_safety_glass", rgba=(0.50, 0.72, 0.82, 0.34))
    cast_iron = model.material("matte_cast_iron", rgba=(0.015, 0.014, 0.013, 1.0))
    burner_metal = model.material("brushed_burner_metal", rgba=(0.62, 0.60, 0.55, 1.0))
    black_enamel = model.material("black_enamel_caps", rgba=(0.025, 0.023, 0.021, 1.0))
    stainless = model.material("brushed_stainless_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    white_print = model.material("white_control_print", rgba=(0.92, 0.92, 0.86, 1.0))

    hob = model.part("hob")
    hob.visual(
        Box((0.880, 0.560, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=glass,
        name="glass_top",
    )
    hob.visual(Box((0.880, 0.012, 0.008)), origin=Origin(xyz=(0.0, -0.274, TOP_Z + 0.003)), material=stainless, name="front_trim")
    hob.visual(Box((0.880, 0.012, 0.008)), origin=Origin(xyz=(0.0, 0.274, TOP_Z + 0.003)), material=stainless, name="rear_trim")
    hob.visual(Box((0.012, 0.560, 0.008)), origin=Origin(xyz=(-0.434, 0.0, TOP_Z + 0.003)), material=stainless, name="side_trim_0")
    hob.visual(Box((0.012, 0.560, 0.008)), origin=Origin(xyz=(0.434, 0.0, TOP_Z + 0.003)), material=stainless, name="side_trim_1")

    # Recessed stainless burner bowls, black burner caps, and metal gas rings.
    burner_specs = [
        ("rear_burner_0", -0.265, 0.105, 0.116, 0.065, 0.045),
        ("rear_burner_1", 0.265, 0.105, 0.116, 0.065, 0.045),
        ("front_burner_0", -0.260, -0.105, 0.104, 0.060, 0.040),
        ("front_burner_1", 0.260, -0.105, 0.104, 0.060, 0.040),
    ]
    for name, x, y, bowl_d, ring_d, cap_d in burner_specs:
        hob.visual(
            Cylinder(radius=bowl_d / 2.0, length=0.006),
            origin=Origin(xyz=(x, y, TOP_Z + 0.002)),
            material=burner_metal,
            name=f"{name}_bowl",
        )
        hob.visual(
            Cylinder(radius=ring_d / 2.0, length=0.008),
            origin=Origin(xyz=(x, y, TOP_Z + 0.008)),
            material=burner_metal,
            name=f"{name}_gas_ring",
        )
        hob.visual(
            Cylinder(radius=cap_d / 2.0, length=0.010),
            origin=Origin(xyz=(x, y, TOP_Z + 0.016)),
            material=black_enamel,
            name=f"{name}_cap",
        )

    # Larger central wok burner with concentric gas rings and a raised black cap.
    hob.visual(
        Cylinder(radius=0.085, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z + 0.0025)),
        material=burner_metal,
        name="wok_bowl",
    )
    hob.visual(
        Cylinder(radius=0.067, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z + 0.009)),
        material=burner_metal,
        name="wok_outer_ring",
    )
    hob.visual(
        Cylinder(radius=0.039, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z + 0.020)),
        material=burner_metal,
        name="wok_inner_ring",
    )
    hob.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z + 0.030)),
        material=black_enamel,
        name="wok_cap",
    )

    # A connected cast-iron grate built from overlapping members and feet.
    grate_z = TOP_Z + 0.034
    grate_h = 0.018
    grate_w = 0.018
    x_span = 0.780
    y_span = 0.355
    bar_specs = [
        ((x_span, grate_w, grate_h), (0.0, -y_span / 2.0, grate_z), "grate_front_rail"),
        ((x_span, grate_w, grate_h), (0.0, y_span / 2.0, grate_z), "grate_rear_rail"),
        ((grate_w, y_span, grate_h), (-x_span / 2.0, 0.0, grate_z), "grate_side_rail_0"),
        ((grate_w, y_span, grate_h), (x_span / 2.0, 0.0, grate_z), "grate_side_rail_1"),
    ]
    for x in (-0.185, 0.0, 0.185):
        bar_specs.append(((grate_w, y_span, grate_h), (x, 0.0, grate_z), f"grate_long_rail_{x:.3f}"))
    for y in (-0.105, 0.025, 0.145):
        bar_specs.append(((x_span, grate_w, grate_h), (0.0, y, grate_z), f"grate_cross_rail_{y:.3f}"))
    for i, (size, xyz, name) in enumerate(bar_specs):
        hob.visual(Box(size), origin=Origin(xyz=xyz), material=cast_iron, name=name)
    foot_h = grate_z - TOP_Z
    foot_i = 0
    for x in (-0.365, -0.185, 0.0, 0.185, 0.365):
        for y in (-0.158, 0.158):
            hob.visual(
                Box((0.024, 0.024, foot_h + 0.004)),
                origin=Origin(xyz=(x, y, TOP_Z + foot_h / 2.0 - 0.002)),
                material=cast_iron,
                name=f"grate_foot_{foot_i}",
            )
            foot_i += 1

    # Small printed control indices on the front glass, one for each knob.
    knob_xs = (-0.320, -0.160, 0.0, 0.160, 0.320)
    for i, x in enumerate(knob_xs):
        hob.visual(Box((0.004, 0.026, 0.0012)), origin=Origin(xyz=(x, -0.177, TOP_Z + 0.0005)), material=white_print, name=f"knob_index_{i}")
        hob.visual(Cylinder(radius=0.0035, length=0.0012), origin=Origin(xyz=(x - 0.026, -0.178, TOP_Z + 0.0005), rpy=(0.0, 0.0, 0.0)), material=white_print, name=f"low_dot_{i}")
        hob.visual(Cylinder(radius=0.0035, length=0.0012), origin=Origin(xyz=(x + 0.026, -0.178, TOP_Z + 0.0005), rpy=(0.0, 0.0, 0.0)), material=white_print, name=f"high_dot_{i}")

    # Rear hinge hardware for the splash lid.  The pin is intentionally captured
    # inside the lid barrel and is scoped in run_tests as a real hinge overlap.
    hob.visual(
        Cylinder(radius=0.0032, length=0.820),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_pin",
    )
    for i, x in enumerate((-0.365, 0.365)):
        hob.visual(
            Cylinder(radius=0.010, length=0.070),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name=f"outer_hinge_barrel_{i}",
        )
        hob.visual(
            Box((0.082, 0.020, HINGE_Z - TOP_Z + 0.006)),
            origin=Origin(xyz=(x, HINGE_Y + 0.002, (HINGE_Z + TOP_Z) / 2.0 - 0.003)),
            material=stainless,
            name=f"hinge_stand_{i}",
        )

    # Five continuously turning front knobs, each on its own short vertical axis.
    for i, x in enumerate(knob_xs):
        knob = model.part(f"knob_{i}")
        knob.visual(Cylinder(radius=0.029, length=0.008), origin=Origin(xyz=(0.0, 0.0, 0.004)), material=stainless, name="knob_skirt")
        knob.visual(Cylinder(radius=0.023, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.016)), material=stainless, name="knob_body")
        knob.visual(Cylinder(radius=0.018, length=0.0025), origin=Origin(xyz=(0.0, 0.0, 0.030)), material=black_enamel, name="knob_top")
        knob.visual(Box((0.004, 0.030, 0.002)), origin=Origin(xyz=(0.0, 0.006, 0.032)), material=white_print, name="knob_pointer")
        model.articulation(
            f"hob_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=hob,
            child=knob,
            origin=Origin(xyz=(x, -0.218, TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.35, velocity=8.0),
        )

    lid = model.part("splash_lid")
    lid.visual(
        Cylinder(radius=0.007, length=0.600),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="lid_barrel",
    )
    lid.visual(
        Box((0.590, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.012, 0.001)),
        material=stainless,
        name="lid_clamp",
    )
    lid.visual(
        Box((0.785, 0.205, 0.006)),
        origin=Origin(xyz=(0.0, -0.118, 0.004)),
        material=smoked_glass,
        name="glass_panel",
    )
    lid.visual(
        Box((0.785, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.220, 0.004)),
        material=stainless,
        name="front_glass_edge",
    )
    model.articulation(
        "hob_to_splash_lid",
        ArticulationType.REVOLUTE,
        parent=hob,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hob = object_model.get_part("hob")
    lid = object_model.get_part("splash_lid")
    lid_hinge = object_model.get_articulation("hob_to_splash_lid")

    ctx.allow_overlap(
        hob,
        lid,
        elem_a="hinge_pin",
        elem_b="lid_barrel",
        reason="The slim hinge pin is intentionally captured inside the lid barrel.",
    )
    ctx.expect_within(
        hob,
        lid,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="lid_barrel",
        margin=0.002,
        name="hinge pin is centered inside lid barrel",
    )
    ctx.expect_overlap(
        hob,
        lid,
        axes="x",
        elem_a="hinge_pin",
        elem_b="lid_barrel",
        min_overlap=0.55,
        name="hinge pin spans the rotating glass barrel",
    )

    for i in range(5):
        knob = object_model.get_part(f"knob_{i}")
        joint = object_model.get_articulation(f"hob_to_knob_{i}")
        ctx.expect_gap(
            knob,
            hob,
            axis="z",
            positive_elem="knob_skirt",
            negative_elem="glass_top",
            max_gap=0.0015,
            max_penetration=0.000001,
            name=f"front knob {i} sits on the glass top",
        )
        with ctx.pose({joint: 2.4}):
            ctx.expect_gap(
                knob,
                hob,
                axis="z",
                positive_elem="knob_skirt",
                negative_elem="glass_top",
                max_gap=0.0015,
                max_penetration=0.000001,
                name=f"front knob {i} still seats while rotated",
            )

    with ctx.pose({lid_hinge: 1.35}):
        ctx.expect_gap(
            lid,
            hob,
            axis="z",
            positive_elem="glass_panel",
            negative_elem="glass_top",
            min_gap=0.055,
            name="raised splash glass clears the cooking surface",
        )

    return ctx.report()


object_model = build_object_model()
