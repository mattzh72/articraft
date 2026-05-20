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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


COOKTOP_WIDTH = 0.76
COOKTOP_DEPTH = 0.52
BODY_HEIGHT = 0.055
GLASS_THICKNESS = 0.006
CONTROL_DECK_DEPTH = 0.14
CONTROL_DECK_THICKNESS = 0.014
GLASS_DEPTH = COOKTOP_DEPTH - CONTROL_DECK_DEPTH
CONTROL_TOP_Z = BODY_HEIGHT + CONTROL_DECK_THICKNESS
SHIELD_BARREL_RADIUS = 0.004
SHIELD_DEPTH = 0.062
SHIELD_THICKNESS = 0.0035
SHIELD_REST_TILT = 0.38
SHIELD_WIDTH = 0.69


def _shield_panel_origin(depth: float, thickness: float, tilt: float) -> Origin:
    """Place a flat plate so its lower front edge rides the hinge line at q=0."""
    shift_y = depth * 0.5 * math.cos(tilt) - thickness * 0.5 * math.sin(tilt)
    shift_z = depth * 0.5 * math.sin(tilt) + thickness * 0.5 * math.cos(tilt)
    return Origin(xyz=(0.0, shift_y, shift_z), rpy=(tilt, 0.0, 0.0))


def _add_zone(body, *, x: float, y: float, radius: float, inner_radius: float, index: int, ring_material, core_material) -> None:
    body.visual(
        Cylinder(radius=radius, length=0.0009),
        origin=Origin(xyz=(x, y, BODY_HEIGHT + GLASS_THICKNESS - 0.00045)),
        material=ring_material,
        name=f"zone_{index}_ring",
    )
    body.visual(
        Cylinder(radius=inner_radius, length=0.0007),
        origin=Origin(xyz=(x, y, BODY_HEIGHT + GLASS_THICKNESS - 0.00035)),
        material=core_material,
        name=f"zone_{index}_core",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radiant_electric_cooktop")

    shell = model.material("shell_stainless", rgba=(0.74, 0.75, 0.76, 1.0))
    trim = model.material("trim_dark", rgba=(0.53, 0.55, 0.58, 1.0))
    glass = model.material("glass_black", rgba=(0.08, 0.08, 0.09, 1.0))
    zone_ring = model.material("zone_ring", rgba=(0.34, 0.24, 0.22, 1.0))
    zone_core = model.material("zone_core", rgba=(0.18, 0.11, 0.10, 1.0))
    knob_finish = model.material("knob_black", rgba=(0.14, 0.14, 0.15, 1.0))
    knob_shaft_finish = model.material("shaft_dark", rgba=(0.42, 0.43, 0.45, 1.0))
    shield_smoke = model.material("shield_smoke", rgba=(0.22, 0.26, 0.30, 0.42))

    body = model.part("body")
    body.visual(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
        material=shell,
        name="body_shell",
    )
    body.visual(
        Box((COOKTOP_WIDTH - 0.02, CONTROL_DECK_DEPTH, CONTROL_DECK_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -COOKTOP_DEPTH * 0.5 + CONTROL_DECK_DEPTH * 0.5,
                BODY_HEIGHT + CONTROL_DECK_THICKNESS * 0.5,
            )
        ),
        material=trim,
        name="control_deck",
    )
    body.visual(
        Box((COOKTOP_WIDTH - 0.02, GLASS_DEPTH, GLASS_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -COOKTOP_DEPTH * 0.5 + CONTROL_DECK_DEPTH + GLASS_DEPTH * 0.5,
                BODY_HEIGHT + GLASS_THICKNESS * 0.5,
            )
        ),
        material=glass,
        name="glass_top",
    )
    body.visual(
        Box((COOKTOP_WIDTH - 0.02, 0.014, CONTROL_DECK_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -COOKTOP_DEPTH * 0.5 + 0.007,
                BODY_HEIGHT + CONTROL_DECK_THICKNESS * 0.5,
            )
        ),
        material=trim,
        name="front_hinge_rail",
    )

    for index, knob_x in enumerate((-0.18, -0.06, 0.06, 0.18)):
        body.visual(
            Cylinder(radius=0.014, length=0.0022),
            origin=Origin(
                xyz=(knob_x, -0.154, BODY_HEIGHT + CONTROL_DECK_THICKNESS + 0.0011),
            ),
            material=trim,
            name=f"knob_seat_{index}",
        )

    _add_zone(body, x=-0.19, y=0.14, radius=0.082, inner_radius=0.062, index=0, ring_material=zone_ring, core_material=zone_core)
    _add_zone(body, x=0.19, y=0.14, radius=0.094, inner_radius=0.070, index=1, ring_material=zone_ring, core_material=zone_core)
    _add_zone(body, x=-0.19, y=-0.02, radius=0.090, inner_radius=0.067, index=2, ring_material=zone_ring, core_material=zone_core)
    _add_zone(body, x=0.19, y=-0.02, radius=0.076, inner_radius=0.056, index=3, ring_material=zone_ring, core_material=zone_core)

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.024,
            body_style="skirted",
            top_diameter=0.030,
            skirt=KnobSkirt(0.048, 0.0048, flare=0.05),
            grip=KnobGrip(style="fluted", count=14, depth=0.0011),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            bore=KnobBore(style="d_shaft", diameter=0.0065, flat_depth=0.0012),
            center=False,
        ),
        "cooktop_knob",
    )

    shield = model.part("shield")
    shield.visual(
        Box((SHIELD_WIDTH, SHIELD_DEPTH, SHIELD_THICKNESS)),
        origin=_shield_panel_origin(SHIELD_DEPTH, SHIELD_THICKNESS, SHIELD_REST_TILT),
        material=shield_smoke,
        name="shield_panel",
    )
    shield.visual(
        Cylinder(radius=SHIELD_BARREL_RADIUS, length=SHIELD_WIDTH - 0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim,
        name="shield_barrel",
    )

    model.articulation(
        "body_to_shield",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shield,
        origin=Origin(
            xyz=(
                0.0,
                -COOKTOP_DEPTH * 0.5 + 0.007,
                CONTROL_TOP_Z + SHIELD_BARREL_RADIUS,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.0),
    )

    for index, knob_x in enumerate((-0.18, -0.06, 0.06, 0.18)):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.0045, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=knob_shaft_finish,
            name="shaft",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=knob_finish,
            name="knob_cap",
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(
                xyz=(
                    knob_x,
                    -0.154,
                    CONTROL_TOP_Z,
                )
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.2, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shield = object_model.get_part("shield")
    knob_0 = object_model.get_part("knob_0")
    knob_3 = object_model.get_part("knob_3")
    shield_hinge = object_model.get_articulation("body_to_shield")

    ctx.expect_gap(
        knob_0,
        shield,
        axis="y",
        min_gap=0.010,
        positive_elem="knob_cap",
        negative_elem="shield_panel",
        name="shield_sits_in_front_of_left_knob",
    )
    ctx.expect_gap(
        knob_3,
        shield,
        axis="y",
        min_gap=0.010,
        positive_elem="knob_cap",
        negative_elem="shield_panel",
        name="shield_sits_in_front_of_right_knob",
    )
    ctx.expect_overlap(
        shield,
        knob_0,
        axes="x",
        min_overlap=0.035,
        elem_a="shield_panel",
        elem_b="knob_cap",
        name="shield_spans_left_knob_width",
    )
    ctx.expect_overlap(
        shield,
        knob_3,
        axes="x",
        min_overlap=0.035,
        elem_a="shield_panel",
        elem_b="knob_cap",
        name="shield_spans_right_knob_width",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(shield, elem="shield_panel")
    with ctx.pose({shield_hinge: 1.0}):
        open_panel_aabb = ctx.part_element_world_aabb(shield, elem="shield_panel")

    closed_top_z = None if closed_panel_aabb is None else float(closed_panel_aabb[1][2])
    open_top_z = None if open_panel_aabb is None else float(open_panel_aabb[1][2])
    ctx.check(
        "shield_flips_upward",
        closed_top_z is not None and open_top_z is not None and open_top_z > closed_top_z + 0.028,
        details=f"closed_top_z={closed_top_z!r}, open_top_z={open_top_z!r}",
    )

    return ctx.report()


object_model = build_object_model()
