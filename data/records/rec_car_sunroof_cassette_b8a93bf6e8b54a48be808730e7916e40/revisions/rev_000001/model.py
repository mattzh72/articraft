from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_L = 1.70
OUTER_W = 0.92
FRAME_H = 0.055
OPEN_L = 1.05
OPEN_W = 0.62
OPEN_X = -0.20
RAIL_Y = 0.365
RAIL_L = 1.50
RAIL_W = 0.090
RAIL_H = 0.018
SLOT_W = 0.045
SLOT_CUT_H = 0.018
SLOT_FLOOR_H = 0.003
HINGE_X = OPEN_X - OPEN_L / 2.0
HINGE_Z = 0.055
SLIDE_TRAVEL = 0.42
PANEL_L = 1.04
PANEL_W = 0.66
PANEL_T = 0.020


def _cassette_shell() -> cq.Workplane:
    """One connected cassette frame with a through roof opening and two rail grooves."""

    frame = cq.Workplane("XY").box(OUTER_L, OUTER_W, FRAME_H)
    opening = cq.Workplane("XY").box(OPEN_L, OPEN_W, FRAME_H * 3.0).translate(
        (OPEN_X, 0.0, 0.0)
    )
    frame = frame.cut(opening)

    rail_z = FRAME_H / 2.0 + RAIL_H / 2.0
    slot_top = FRAME_H / 2.0 + RAIL_H
    for y in (-RAIL_Y, RAIL_Y):
        rail = cq.Workplane("XY").box(RAIL_L, RAIL_W, RAIL_H).translate(
            (0.0, y, rail_z)
        )
        frame = frame.union(rail)

        slot_cut = cq.Workplane("XY").box(RAIL_L, SLOT_W, SLOT_CUT_H).translate(
            (0.0, y, slot_top - SLOT_CUT_H / 2.0 + 0.002)
        )
        frame = frame.cut(slot_cut)

    return frame


def _glass_slab() -> cq.Workplane:
    """Rounded smoked-glass slab with its front edge just behind the hinge axis."""

    return (
        cq.Workplane("XY")
        .box(PANEL_L, PANEL_W, PANEL_T)
        .edges("|Z")
        .fillet(0.045)
        .translate((PANEL_L / 2.0 + 0.015, 0.0, 0.008 + PANEL_T / 2.0))
    )


def _hinge_barrel(y_center: float) -> cq.Workplane:
    """A short hollow hinge knuckle around the carriage pin."""

    length = 0.110
    barrel = (
        cq.Workplane("XY")
        .circle(0.017)
        .circle(0.0085)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((0.0, y_center, 0.0))
    )
    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vent_slide_sunroof_cassette")

    dark_metal = model.material("dark_anodized_aluminum", rgba=(0.045, 0.048, 0.052, 1.0))
    black_plastic = model.material("black_track_plastic", rgba=(0.008, 0.009, 0.010, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.045, 0.120, 0.150, 0.38))
    ceramic_black = model.material("ceramic_black_frit", rgba=(0.002, 0.002, 0.002, 0.92))
    zinc = model.material("zinc_plated_steel", rgba=(0.58, 0.60, 0.58, 1.0))

    cassette = model.part("cassette")
    cassette.visual(
        mesh_from_cadquery(_cassette_shell(), "cassette_frame"),
        material=dark_metal,
        name="frame_shell",
    )

    slot_floor_z = FRAME_H / 2.0 + RAIL_H - SLOT_CUT_H + SLOT_FLOOR_H / 2.0
    for idx, y in enumerate((-RAIL_Y, RAIL_Y)):
        cassette.visual(
            Box((RAIL_L - 0.020, SLOT_W - 0.006, SLOT_FLOOR_H)),
            origin=Origin(xyz=(0.0, y, slot_floor_z)),
            material=black_plastic,
            name=f"rail_slot_{idx}",
        )

    # Shallow drain pockets at the cassette corners give the frame real sunroof
    # cassette character while remaining physically seated on the shell.
    for idx, (x, y) in enumerate(
        (
            (OPEN_X - OPEN_L / 2.0 - 0.050, -0.405),
            (OPEN_X - OPEN_L / 2.0 - 0.050, 0.405),
            (OPEN_X + OPEN_L / 2.0 + 0.070, -0.405),
            (OPEN_X + OPEN_L / 2.0 + 0.070, 0.405),
        )
    ):
        cassette.visual(
            Box((0.080, 0.055, 0.004)),
            origin=Origin(xyz=(x, y, FRAME_H / 2.0 + 0.002)),
            material=black_plastic,
            name=f"drain_pocket_{idx}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.0085, length=0.750),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="hinge_pin",
    )

    for idx, y in enumerate((-RAIL_Y, RAIL_Y)):
        carriage.visual(
            Box((0.480, 0.026, 0.009)),
            origin=Origin(xyz=(0.300, y, -0.020)),
            material=black_plastic,
            name=f"shoe_{idx}",
        )
        carriage.visual(
            Box((0.085, 0.024, 0.024)),
            origin=Origin(xyz=(0.035, y, -0.007)),
            material=zinc,
            name=f"front_bracket_{idx}",
        )

    glass_panel = model.part("glass_panel")
    glass_panel.visual(
        mesh_from_cadquery(_glass_slab(), "smoked_glass_panel"),
        material=smoked_glass,
        name="glass_slab",
    )
    glass_panel.visual(
        Box((0.090, PANEL_W - 0.070, 0.003)),
        origin=Origin(xyz=(0.060, 0.0, 0.0285)),
        material=ceramic_black,
        name="front_frit",
    )
    glass_panel.visual(
        Box((0.085, PANEL_W - 0.070, 0.003)),
        origin=Origin(xyz=(PANEL_L + 0.005, 0.0, 0.0285)),
        material=ceramic_black,
        name="rear_frit",
    )
    for idx, y in enumerate((-(PANEL_W / 2.0 - 0.020), PANEL_W / 2.0 - 0.020)):
        glass_panel.visual(
            Box((PANEL_L - 0.100, 0.032, 0.003)),
            origin=Origin(xyz=(0.545, y, 0.0285)),
            material=ceramic_black,
            name=f"side_frit_{idx}",
        )
    for idx, y in enumerate((-0.220, 0.0, 0.220)):
        glass_panel.visual(
            mesh_from_cadquery(_hinge_barrel(y), f"glass_hinge_barrel_{idx}"),
            material=ceramic_black,
            name=f"hinge_barrel_{idx}",
        )
        glass_panel.visual(
            Box((0.050, 0.070, 0.018)),
            origin=Origin(xyz=(0.025, y, 0.019)),
            material=ceramic_black,
            name=f"hinge_strap_{idx}",
        )

    model.articulation(
        "cassette_to_carriage",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=carriage,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "carriage_to_glass",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=glass_panel,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=0.23),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette = object_model.get_part("cassette")
    carriage = object_model.get_part("carriage")
    glass = object_model.get_part("glass_panel")
    slide = object_model.get_articulation("cassette_to_carriage")
    tilt = object_model.get_articulation("carriage_to_glass")

    for idx in (0, 1, 2):
        ctx.allow_overlap(
            carriage,
            glass,
            elem_a="hinge_pin",
            elem_b=f"hinge_barrel_{idx}",
            reason=(
                "The simplified hinge barrel is intentionally modeled as a "
                "captured bushing around the front hinge pin."
            ),
        )
        ctx.expect_overlap(
            carriage,
            glass,
            axes="xyz",
            elem_a="hinge_pin",
            elem_b=f"hinge_barrel_{idx}",
            min_overlap=0.010,
            name=f"hinge_barrel_{idx} captures the front pin",
        )

    for idx in (0, 1):
        ctx.expect_within(
            carriage,
            cassette,
            axes="xy",
            inner_elem=f"shoe_{idx}",
            outer_elem=f"rail_slot_{idx}",
            margin=0.004,
            name=f"shoe_{idx} is captured in its rail slot",
        )
        ctx.expect_gap(
            carriage,
            cassette,
            axis="z",
            positive_elem=f"shoe_{idx}",
            negative_elem=f"rail_slot_{idx}",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"shoe_{idx} rides on the slot floor",
        )

    ctx.expect_gap(
        glass,
        cassette,
        axis="z",
        positive_elem="glass_slab",
        negative_elem="frame_shell",
        min_gap=0.008,
        max_gap=0.030,
        name="closed glass sits proud of the cassette frame",
    )

    closed_pos = ctx.part_world_position(glass)
    closed_glass = ctx.part_element_world_aabb(glass, elem="glass_slab")
    closed_rear_z = closed_glass[1][2] if closed_glass is not None else None

    with ctx.pose({slide: SLIDE_TRAVEL}):
        slid_pos = ctx.part_world_position(glass)
        for idx in (0, 1):
            ctx.expect_within(
                carriage,
                cassette,
                axes="xy",
                inner_elem=f"shoe_{idx}",
                outer_elem=f"rail_slot_{idx}",
                margin=0.004,
                name=f"shoe_{idx} remains retained after rearward slide",
            )
            ctx.expect_overlap(
                carriage,
                cassette,
                axes="x",
                elem_a=f"shoe_{idx}",
                elem_b=f"rail_slot_{idx}",
                min_overlap=0.20,
                name=f"shoe_{idx} keeps insertion length at full slide",
            )

    ctx.check(
        "glass translates rearward on the cassette rails",
        closed_pos is not None
        and slid_pos is not None
        and slid_pos[0] > closed_pos[0] + SLIDE_TRAVEL * 0.90,
        details=f"closed={closed_pos}, slid={slid_pos}",
    )

    with ctx.pose({tilt: 0.23}):
        vented_glass = ctx.part_element_world_aabb(glass, elem="glass_slab")
        vented_rear_z = vented_glass[1][2] if vented_glass is not None else None

    ctx.check(
        "front hinge vents the glass upward",
        closed_rear_z is not None
        and vented_rear_z is not None
        and vented_rear_z > closed_rear_z + 0.12,
        details=f"closed_max_z={closed_rear_z}, vented_max_z={vented_rear_z}",
    )

    return ctx.report()


object_model = build_object_model()
