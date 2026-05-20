from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


ENAMEL = Material("white_enamel", rgba=(0.92, 0.91, 0.86, 1.0))
DARK_ENAMEL = Material("black_enamel", rgba=(0.02, 0.022, 0.024, 1.0))
CAST_IRON = Material("cast_iron", rgba=(0.005, 0.005, 0.006, 1.0))
BRASS = Material("brass_ports", rgba=(0.80, 0.58, 0.22, 1.0))
STEEL = Material("brushed_steel", rgba=(0.60, 0.62, 0.62, 1.0))
KNOB_BLACK = Material("satin_black_knob", rgba=(0.025, 0.025, 0.028, 1.0))
MARKING = Material("white_control_marking", rgba=(0.96, 0.96, 0.90, 1.0))
GLASS = Material("smoky_blue_glass", rgba=(0.48, 0.72, 0.88, 0.38))
HINGE_METAL = Material("polished_hinge_metal", rgba=(0.70, 0.72, 0.70, 1.0))


def _add_burner(stove, x: float, y: float, index: int, ring_mesh) -> None:
    """Add one supported gas burner, cap, and cast-iron grate to the root part."""
    stove.visual(
        Cylinder(radius=0.052, length=0.014),
        origin=Origin(xyz=(x, y, 0.065)),
        material=DARK_ENAMEL,
        name=f"burner_cup_{index}",
    )
    stove.visual(
        ring_mesh,
        origin=Origin(xyz=(x, y, 0.074)),
        material=BRASS,
        name=f"burner_ring_{index}",
    )
    stove.visual(
        Cylinder(radius=0.033, length=0.010),
        origin=Origin(xyz=(x, y, 0.081)),
        material=CAST_IRON,
        name=f"burner_cap_{index}",
    )

    # The grate is one visible supported structure: the bars overlap each other
    # and the four small feet touch the cooktop deck.
    z_bar = 0.096
    stove.visual(
        Box((0.150, 0.012, 0.012)),
        origin=Origin(xyz=(x, y, z_bar)),
        material=CAST_IRON,
        name=f"grate_cross_x_{index}",
    )
    stove.visual(
        Box((0.012, 0.150, 0.012)),
        origin=Origin(xyz=(x, y, z_bar)),
        material=CAST_IRON,
        name=f"grate_cross_y_{index}",
    )
    for offset_y, suffix in ((-0.075, "front"), (0.075, "rear")):
        stove.visual(
            Box((0.160, 0.010, 0.012)),
            origin=Origin(xyz=(x, y + offset_y, z_bar)),
            material=CAST_IRON,
            name=f"grate_{suffix}_bar_{index}",
        )
    for offset_x, suffix in ((-0.075, "side_0"), (0.075, "side_1")):
        stove.visual(
            Box((0.010, 0.160, 0.012)),
            origin=Origin(xyz=(x + offset_x, y, z_bar)),
            material=CAST_IRON,
            name=f"grate_{suffix}_bar_{index}",
        )
    for fx in (-0.065, 0.065):
        for fy in (-0.065, 0.065):
            stove.visual(
                Box((0.014, 0.014, 0.034)),
                origin=Origin(xyz=(x + fx, y + fy, 0.075)),
                material=CAST_IRON,
                name=f"grate_foot_{index}_{'p' if fx > 0 else 'n'}{'p' if fy > 0 else 'n'}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_four_burner_gas_stove")

    stove = model.part("stove")
    stove.visual(
        Box((0.620, 0.540, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=ENAMEL,
        name="body_shell",
    )
    stove.visual(
        Box((0.560, 0.405, 0.008)),
        origin=Origin(xyz=(0.0, 0.035, 0.054)),
        material=ENAMEL,
        name="cooktop_deck",
    )
    stove.visual(
        Box((0.585, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.176, 0.062)),
        material=ENAMEL,
        name="front_lip",
    )
    stove.visual(
        Box((0.585, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.246, 0.062)),
        material=ENAMEL,
        name="rear_lip",
    )
    for side_x, suffix in ((-0.286, "side_0"), (0.286, "side_1")):
        stove.visual(
            Box((0.018, 0.405, 0.014)),
            origin=Origin(xyz=(side_x, 0.035, 0.062)),
            material=ENAMEL,
            name=f"{suffix}_lip",
        )

    stove.visual(
        Box((0.600, 0.016, 0.090)),
        origin=Origin(xyz=(0.0, -0.278, 0.045)),
        material=STEEL,
        name="front_panel",
    )

    ring_mesh = mesh_from_geometry(TorusGeometry(0.034, 0.004, radial_segments=16, tubular_segments=36), "gas_port_ring")
    burner_positions = [
        (-0.145, -0.060),
        (0.145, -0.060),
        (-0.145, 0.120),
        (0.145, 0.120),
    ]
    for i, (x, y) in enumerate(burner_positions):
        _add_burner(stove, x, y, i, ring_mesh)

    # Rear hinge hardware fixed to the stove body.
    stove.visual(
        Box((0.575, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.258, 0.117)),
        material=HINGE_METAL,
        name="hinge_leaf",
    )
    stove.visual(
        Cylinder(radius=0.006, length=0.555),
        origin=Origin(xyz=(0.0, 0.248, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=HINGE_METAL,
        name="hinge_barrel",
    )
    for x in (-0.270, 0.270):
        stove.visual(
            Box((0.026, 0.018, 0.056)),
            origin=Origin(xyz=(x, 0.254, 0.097)),
            material=HINGE_METAL,
            name=f"hinge_bracket_{'0' if x < 0 else '1'}",
        )

    # Four rotary controls on the front edge with collars and tick markings.
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.026,
            body_style="skirted",
            top_diameter=0.039,
            skirt=KnobSkirt(0.060, 0.006, flare=0.08, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0013),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0009, angle_deg=90.0),
            center=False,
        ),
        "stove_control_knob",
    )

    knob_xs = (-0.225, -0.075, 0.075, 0.225)
    knob_parts = []
    for i, x in enumerate(knob_xs):
        stove.visual(
            Cylinder(radius=0.033, length=0.004),
            origin=Origin(xyz=(x, -0.284, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=STEEL,
            name=f"knob_collar_{i}",
        )
        stove.visual(
            Box((0.006, 0.002, 0.018)),
            origin=Origin(xyz=(x, -0.287, 0.082)),
            material=MARKING,
            name=f"control_tick_{i}",
        )

        knob = model.part(f"knob_{i}")
        knob.visual(
            knob_mesh,
            origin=Origin(),
            material=KNOB_BLACK,
            name="knob_cap",
        )
        knob_parts.append(knob)
        model.articulation(
            f"knob_{i}_axis",
            ArticulationType.CONTINUOUS,
            parent=stove,
            child=knob,
            origin=Origin(xyz=(x, -0.286, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.35, velocity=8.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    cover = model.part("glass_cover")
    cover.visual(
        Box((0.558, 0.392, 0.006)),
        origin=Origin(xyz=(0.0, -0.196, 0.000)),
        material=GLASS,
        name="glass_panel",
    )
    cover.visual(
        Box((0.568, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.008, -0.004)),
        material=HINGE_METAL,
        name="glass_clamp",
    )
    cover.visual(
        Cylinder(radius=0.006, length=0.555),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=HINGE_METAL,
        name="moving_barrel",
    )
    cover.visual(
        Box((0.030, 0.014, 0.018)),
        origin=Origin(xyz=(-0.260, -0.006, -0.004)),
        material=HINGE_METAL,
        name="end_clamp_0",
    )
    cover.visual(
        Box((0.030, 0.014, 0.018)),
        origin=Origin(xyz=(0.260, -0.006, -0.004)),
        material=HINGE_METAL,
        name="end_clamp_1",
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=stove,
        child=cover,
        origin=Origin(xyz=(0.0, 0.236, 0.125)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.6, lower=0.0, upper=1.92),
        motion_properties=MotionProperties(damping=0.06, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stove = object_model.get_part("stove")
    cover = object_model.get_part("glass_cover")
    hinge = object_model.get_articulation("cover_hinge")

    knob_joints = [object_model.get_articulation(f"knob_{i}_axis") for i in range(4)]
    ctx.check(
        "four continuously rotating knobs",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in knob_joints),
        details=f"joint types={[j.articulation_type for j in knob_joints]}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            cover,
            stove,
            axes="xy",
            elem_a="glass_panel",
            elem_b="cooktop_deck",
            min_overlap=0.33,
            name="closed glass spans burner field",
        )
        ctx.expect_gap(
            cover,
            stove,
            axis="z",
            positive_elem="glass_panel",
            negative_elem="grate_cross_x_0",
            min_gap=0.015,
            max_gap=0.035,
            name="closed glass clears the grates",
        )

    closed_box = ctx.part_element_world_aabb(cover, elem="glass_panel")
    with ctx.pose({hinge: 1.75}):
        open_box = ctx.part_element_world_aabb(cover, elem="glass_panel")
    closed_top = closed_box[1][2] if closed_box is not None else None
    open_top = open_box[1][2] if open_box is not None else None
    ctx.check(
        "cover folds upward on rear hinge",
        closed_top is not None and open_top is not None and open_top > closed_top + 0.25,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )

    for i in range(4):
        knob = object_model.get_part(f"knob_{i}")
        ctx.expect_gap(
            stove,
            knob,
            axis="y",
            positive_elem=f"knob_collar_{i}",
            negative_elem="knob_cap",
            max_gap=0.002,
            max_penetration=0.0,
            name=f"knob_{i} seats on front collar",
        )

    return ctx.report()


object_model = build_object_model()
