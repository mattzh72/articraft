from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_rotary_isolator_mixer")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    charcoal = model.material("charcoal_panel", rgba=(0.07, 0.075, 0.078, 1.0))
    dark_slot = model.material("dark_slot", rgba=(0.0, 0.0, 0.0, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.48, 0.50, 0.52, 1.0))
    white = model.material("white_markings", rgba=(0.92, 0.90, 0.84, 1.0))
    rubber = model.material("rubber_black", rgba=(0.025, 0.023, 0.021, 1.0))

    width = 0.56
    depth = 0.32
    body_h = 0.064
    top_t = 0.004
    top_z = body_h + top_t
    front_y = -depth / 2.0
    front_panel_front_y = front_y - 0.004
    slot_front_y = front_panel_front_y - 0.00090

    housing = model.part("housing")
    housing.visual(
        Box((width, depth, body_h)),
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
        material=matte_black,
        name="shallow_body",
    )
    housing.visual(
        Box((0.525, 0.275, top_t)),
        origin=Origin(xyz=(0.0, 0.012, body_h + top_t / 2.0)),
        material=charcoal,
        name="top_panel",
    )
    housing.visual(
        Box((0.545, 0.004, 0.052)),
        origin=Origin(xyz=(0.0, front_y - 0.002, 0.030)),
        material=brushed,
        name="front_face",
    )

    # Four broad isolation control stations on the top panel.
    knob_xs = (-0.195, -0.065, 0.065, 0.195)
    knob_y = 0.055
    ring_t = 0.002
    ring_top_z = top_z + ring_t
    for i, x in enumerate(knob_xs):
        housing.visual(
            Cylinder(radius=0.039, length=ring_t),
            origin=Origin(xyz=(x, knob_y, top_z + ring_t / 2.0)),
            material=brushed,
            name=f"knob_ring_{i}",
        )
        housing.visual(
            Box((0.004, 0.018, 0.0012)),
            origin=Origin(xyz=(x, knob_y + 0.048, top_z + 0.0004)),
            material=white,
            name=f"top_tick_{i}",
        )

    # Front-panel fader slots with end stops.  The dark plates are slightly
    # seated into the front face so the housing remains one manufactured part.
    fader_xs = (-0.225, -0.135, -0.045, 0.045, 0.135, 0.225)
    slot_z = 0.034
    slot_h = 0.055
    for i, x in enumerate(fader_xs):
        housing.visual(
            Box((0.014, 0.001, slot_h)),
            origin=Origin(xyz=(x, slot_front_y + 0.0005, slot_z)),
            material=dark_slot,
            name=f"fader_slot_{i}",
        )
        housing.visual(
            Box((0.030, 0.001, 0.003)),
            origin=Origin(xyz=(x, slot_front_y + 0.00045, slot_z - slot_h / 2.0)),
            material=dark_slot,
            name=f"slot_stop_{i}_0",
        )
        housing.visual(
            Box((0.030, 0.001, 0.003)),
            origin=Origin(xyz=(x, slot_front_y + 0.00045, slot_z + slot_h / 2.0)),
            material=dark_slot,
            name=f"slot_stop_{i}_1",
        )

    # Small rubber feet under the shallow desktop chassis.
    for i, (x, y) in enumerate(
        ((-0.235, -0.115), (0.235, -0.115), (-0.235, 0.125), (0.235, 0.125))
    ):
        housing.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(xyz=(x, y, -0.002)),
            material=rubber,
            name=f"foot_{i}",
        )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.064,
            0.028,
            body_style="skirted",
            base_diameter=0.070,
            top_diameter=0.054,
            edge_radius=0.001,
            skirt=KnobSkirt(0.073, 0.006, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="ribbed", count=32, depth=0.0014, width=0.0018),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "large_isolation_knob",
    )
    for i, x in enumerate(knob_xs):
        knob = model.part(f"knob_{i}")
        knob.visual(knob_mesh, material=matte_black, name="knob_cap")
        model.articulation(
            f"housing_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(x, knob_y, ring_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=6.0),
        )

    # Each front-face fader rides vertically in its slot.  The child frame is on
    # the slim stem, while the thumb cap protrudes toward the DJ.
    stem_half_y = 0.009
    fader_y = slot_front_y - stem_half_y
    lower_z = 0.022
    fader_travel = 0.028
    for i, x in enumerate(fader_xs):
        fader = model.part(f"fader_{i}")
        fader.visual(
            Box((0.011, 0.018, 0.030)),
            origin=Origin(),
            material=dark_slot,
            name="fader_stem",
        )
        fader.visual(
            Box((0.044, 0.020, 0.018)),
            origin=Origin(xyz=(0.0, -0.012, 0.0)),
            material=charcoal,
            name="fader_cap",
        )
        fader.visual(
            Box((0.034, 0.0022, 0.003)),
            origin=Origin(xyz=(0.0, -0.0231, -0.0045)),
            material=brushed,
            name="cap_ridge_0",
        )
        fader.visual(
            Box((0.034, 0.0022, 0.003)),
            origin=Origin(xyz=(0.0, -0.0231, 0.0045)),
            material=brushed,
            name="cap_ridge_1",
        )
        model.articulation(
            f"housing_to_fader_{i}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(x, fader_y, lower_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                lower=0.0, upper=fader_travel, effort=8.0, velocity=0.18
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")

    for i in range(4):
        knob = object_model.get_part(f"knob_{i}")
        joint = object_model.get_articulation(f"housing_to_knob_{i}")
        ctx.expect_contact(
            knob,
            housing,
            elem_a="knob_cap",
            elem_b=f"knob_ring_{i}",
            contact_tol=0.0005,
            name=f"knob_{i} seated on top ring",
        )
        rest = ctx.part_world_position(knob)
        with ctx.pose({joint: 1.75}):
            turned = ctx.part_world_position(knob)
            ctx.expect_contact(
                knob,
                housing,
                elem_a="knob_cap",
                elem_b=f"knob_ring_{i}",
                contact_tol=0.0005,
                name=f"knob_{i} stays seated while spun",
            )
        ctx.check(
            f"knob_{i} spins in place",
            rest is not None
            and turned is not None
            and abs(rest[0] - turned[0]) < 1e-6
            and abs(rest[1] - turned[1]) < 1e-6
            and abs(rest[2] - turned[2]) < 1e-6,
            details=f"rest={rest}, turned={turned}",
        )

    for i in range(6):
        fader = object_model.get_part(f"fader_{i}")
        joint = object_model.get_articulation(f"housing_to_fader_{i}")
        ctx.expect_contact(
            fader,
            housing,
            elem_a="fader_stem",
            elem_b=f"fader_slot_{i}",
            contact_tol=0.0005,
            name=f"fader_{i} stem rides in slot",
        )
        ctx.expect_within(
            fader,
            housing,
            axes="x",
            inner_elem="fader_stem",
            outer_elem=f"fader_slot_{i}",
            margin=0.0005,
            name=f"fader_{i} stem centered in slot",
        )
        rest = ctx.part_world_position(fader)
        with ctx.pose({joint: 0.028}):
            ctx.expect_within(
                fader,
                housing,
                axes="x",
                inner_elem="fader_stem",
                outer_elem=f"fader_slot_{i}",
                margin=0.0005,
                name=f"fader_{i} remains in slot at top",
            )
            ctx.expect_overlap(
                fader,
                housing,
                axes="z",
                elem_a="fader_stem",
                elem_b=f"fader_slot_{i}",
                min_overlap=0.020,
                name=f"fader_{i} retains vertical guide engagement",
            )
            raised = ctx.part_world_position(fader)
        ctx.check(
            f"fader_{i} travels upward",
            rest is not None and raised is not None and raised[2] > rest[2] + 0.025,
            details=f"rest={rest}, raised={raised}",
        )

    return ctx.report()


object_model = build_object_model()
