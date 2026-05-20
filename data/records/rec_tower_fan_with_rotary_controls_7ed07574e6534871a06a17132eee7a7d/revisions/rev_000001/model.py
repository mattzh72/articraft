from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_tower_fan")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.045, 0.048, 0.052, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.55, 0.58, 0.60, 1.0))
    translucent_blue = model.material("translucent_blue", rgba=(0.18, 0.42, 0.58, 0.55))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.270, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=satin_black,
        name="weighted_disc",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=dark_plastic,
        name="base_collar",
    )
    base.visual(
        Cylinder(radius=0.122, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=graphite,
        name="trim_ring",
    )

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.052, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_plastic,
        name="pivot_stem",
    )
    tower.visual(
        Cylinder(radius=0.084, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_black,
        name="tower_skirt",
    )
    tower.visual(
        Box((0.190, 0.160, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=dark_plastic,
        name="bottom_cap",
    )
    tower.visual(
        Box((0.190, 0.160, 0.130)),
        origin=Origin(xyz=(0.0, 0.0, 1.065)),
        material=dark_plastic,
        name="top_cap",
    )
    tower.visual(
        Box((0.030, 0.160, 0.910)),
        origin=Origin(xyz=(-0.087, 0.0, 0.605)),
        material=dark_plastic,
        name="side_rail_0",
    )
    tower.visual(
        Box((0.030, 0.160, 0.910)),
        origin=Origin(xyz=(0.087, 0.0, 0.605)),
        material=dark_plastic,
        name="side_rail_1",
    )
    tower.visual(
        Box((0.154, 0.018, 0.900)),
        origin=Origin(xyz=(0.0, 0.074, 0.600)),
        material=dark_plastic,
        name="rear_spine",
    )
    for i, (x, y) in enumerate(
        ((-0.085, -0.063), (0.085, -0.063), (-0.085, 0.063), (0.085, 0.063))
    ):
        tower.visual(
            Cylinder(radius=0.017, length=0.910),
            origin=Origin(xyz=(x, y, 0.605)),
            material=dark_plastic,
            name=f"corner_post_{i}",
        )

    front_grille = SlotPatternPanelGeometry(
        (0.142, 0.810),
        0.006,
        slot_size=(0.062, 0.008),
        pitch=(0.017, 0.078),
        frame=0.010,
        corner_radius=0.012,
        slot_angle_deg=88.0,
        stagger=True,
    )
    tower.visual(
        mesh_from_geometry(front_grille, "front_slot_grille"),
        origin=Origin(xyz=(0.0, -0.079, 0.600), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="front_grille",
    )
    tower.visual(
        Cylinder(radius=0.066, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 1.128)),
        material=soft_gray,
        name="dial_recess_floor",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(0.062, 0.004), "dial_recess_rim"),
        origin=Origin(xyz=(0.0, 0.0, 1.132)),
        material=graphite,
        name="dial_recess_rim",
    )

    rotor = model.part("rotor")
    blower = BlowerWheelGeometry(
        outer_radius=0.052,
        inner_radius=0.028,
        width=0.760,
        blade_count=28,
        blade_thickness=0.0022,
        blade_sweep_deg=28.0,
        backplate=True,
        shroud=True,
    )
    rotor.visual(
        mesh_from_geometry(blower, "vertical_blower_wheel"),
        origin=Origin(),
        material=translucent_blue,
        name="blower_wheel",
    )
    rotor.visual(
        Cylinder(radius=0.010, length=0.860),
        origin=Origin(),
        material=graphite,
        name="rotor_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.030, length=0.080),
        origin=Origin(),
        material=graphite,
        name="rotor_hub",
    )

    dial = model.part("dial")
    selector = KnobGeometry(
        0.104,
        0.032,
        body_style="skirted",
        top_diameter=0.086,
        skirt=KnobSkirt(0.110, 0.006, flare=0.06, chamfer=0.0012),
        grip=KnobGrip(style="fluted", count=24, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0009),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(selector, "selector_dial"),
        origin=Origin(),
        material=soft_gray,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=graphite,
        name="dial_shaft",
    )

    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "tower_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.7, velocity=55.0),
    )
    model.articulation(
        "tower_to_dial",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 1.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    rotor = object_model.get_part("rotor")
    dial = object_model.get_part("dial")
    oscillation = object_model.get_articulation("base_to_tower")
    rotor_spin = object_model.get_articulation("tower_to_rotor")
    dial_spin = object_model.get_articulation("tower_to_dial")

    ctx.allow_overlap(
        rotor,
        tower,
        elem_a="rotor_shaft",
        elem_b="bottom_cap",
        reason="The hidden rotor shaft is intentionally captured in the simplified lower bearing cap.",
    )
    ctx.allow_overlap(
        rotor,
        tower,
        elem_a="rotor_shaft",
        elem_b="top_cap",
        reason="The hidden rotor shaft is intentionally captured in the simplified upper bearing cap.",
    )
    ctx.allow_overlap(
        dial,
        tower,
        elem_a="dial_shaft",
        elem_b="top_cap",
        reason="The selector dial shaft seats through the simplified top panel into its bearing hole.",
    )

    ctx.check(
        "tower oscillates on a limited vertical revolute joint",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and oscillation.axis == (0.0, 0.0, 1.0)
        and oscillation.motion_limits.lower < 0.0
        and oscillation.motion_limits.upper > 0.0,
    )
    ctx.check(
        "rotor and selector dial are continuous rotary controls",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS
        and dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and rotor_spin.axis == (0.0, 0.0, 1.0)
        and dial_spin.axis == (0.0, 0.0, 1.0),
    )
    ctx.expect_gap(
        tower,
        base,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem="tower_skirt",
        negative_elem="base_collar",
        name="oscillating tower skirt seats on the base collar",
    )
    ctx.expect_within(
        rotor,
        tower,
        axes="xy",
        margin=0.0,
        inner_elem="blower_wheel",
        name="internal blower wheel stays inside the tower footprint",
    )
    ctx.expect_within(
        rotor,
        tower,
        axes="xy",
        margin=0.002,
        inner_elem="rotor_shaft",
        outer_elem="dial_recess_floor",
        name="rotor shaft stays on the tower centerline",
    )
    ctx.expect_overlap(
        rotor,
        tower,
        axes="z",
        min_overlap=0.020,
        elem_a="rotor_shaft",
        elem_b="bottom_cap",
        name="lower bearing captures the rotor shaft",
    )
    ctx.expect_overlap(
        rotor,
        tower,
        axes="z",
        min_overlap=0.010,
        elem_a="rotor_shaft",
        elem_b="top_cap",
        name="upper bearing captures the rotor shaft",
    )
    ctx.expect_within(
        dial,
        tower,
        axes="xy",
        margin=0.002,
        inner_elem="dial_shaft",
        outer_elem="dial_recess_floor",
        name="selector shaft is centered in the recessed top dial",
    )
    ctx.expect_overlap(
        dial,
        tower,
        axes="z",
        min_overlap=0.020,
        elem_a="dial_shaft",
        elem_b="top_cap",
        name="selector shaft is retained in the top panel",
    )

    return ctx.report()


object_model = build_object_model()
