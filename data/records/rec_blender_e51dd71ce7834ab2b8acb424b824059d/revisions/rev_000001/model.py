from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.260
BASE_DEPTH = 0.290
BASE_HEIGHT = 0.340
DECK_PAD_SIZE = 0.124
DECK_PAD_THICKNESS = 0.006

PITCHER_HEIGHT = 0.285
PITCHER_BASE_WIDTH = 0.118
PITCHER_BASE_DEPTH = 0.100
PITCHER_TOP_WIDTH = 0.165
PITCHER_TOP_DEPTH = 0.145
PITCHER_WALL = 0.004

HOOD_WIDTH = 0.242
HOOD_DEPTH = 0.208
HOOD_SIDE_HEIGHT = 0.310
HOOD_PANEL_THICKNESS = 0.004
HOOD_HINGE_Y = 0.118
HOOD_HINGE_Z = 0.665


def _base_body_shape():
    plinth = cq.Workplane("XY").box(
        BASE_WIDTH,
        BASE_DEPTH,
        0.030,
        centered=(True, True, False),
    )
    tapered_body = (
        cq.Workplane("XY")
        .workplane(offset=0.026)
        .rect(0.246, 0.278)
        .workplane(offset=BASE_HEIGHT - 0.026)
        .rect(0.206, 0.226)
        .loft(combine=True)
    )
    return plinth.union(tapered_body)


def _pitcher_shell_shape():
    outer = (
        cq.Workplane("XY")
        .rect(PITCHER_BASE_WIDTH, PITCHER_BASE_DEPTH)
        .workplane(offset=PITCHER_HEIGHT)
        .rect(PITCHER_TOP_WIDTH, PITCHER_TOP_DEPTH)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=PITCHER_WALL)
        .rect(PITCHER_BASE_WIDTH - 2.0 * PITCHER_WALL, PITCHER_BASE_DEPTH - 2.0 * PITCHER_WALL)
        .workplane(offset=PITCHER_HEIGHT + 0.008 - PITCHER_WALL)
        .rect(PITCHER_TOP_WIDTH - 2.0 * PITCHER_WALL, PITCHER_TOP_DEPTH - 2.0 * PITCHER_WALL)
        .loft(combine=True)
    )
    spout_cut = cq.Workplane("XY").box(
        0.034,
        0.030,
        0.030,
        centered=(True, True, False),
    ).translate((0.0, 0.076, PITCHER_HEIGHT - 0.020))
    return outer.cut(inner).cut(spout_cut)


def _pitcher_handle_shape():
    grip = cq.Workplane("XY").box(
        0.028,
        0.022,
        0.150,
        centered=(True, True, False),
    ).translate((0.0, -0.083, 0.070))
    upper_mount = cq.Workplane("XY").box(
        0.030,
        0.040,
        0.022,
        centered=(True, True, False),
    ).translate((0.0, -0.068, 0.206))
    lower_mount = cq.Workplane("XY").box(
        0.026,
        0.036,
        0.020,
        centered=(True, True, False),
    ).translate((0.0, -0.064, 0.060))
    return grip.union(upper_mount).union(lower_mount)


def _speed_knob_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.022,
            body_style="skirted",
            top_diameter=0.038,
            skirt=KnobSkirt(0.054, 0.006, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            center=False,
        ),
        "speed_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="food_service_blender")

    body_dark = model.material("body_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    control_panel = model.material("control_panel", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    glass = model.material("glass", rgba=(0.74, 0.84, 0.90, 0.34))
    hood_smoke = model.material("hood_smoke", rgba=(0.62, 0.70, 0.78, 0.22))
    lid_black = model.material("lid_black", rgba=(0.11, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.81, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "blender_base"),
        material=body_dark,
        name="base_body",
    )
    base.visual(
        Box((DECK_PAD_SIZE, DECK_PAD_SIZE, DECK_PAD_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + DECK_PAD_THICKNESS / 2.0)),
        material=rubber,
        name="deck_pad",
    )
    base.visual(
        Box((0.116, 0.024, 0.108)),
        origin=Origin(xyz=(0.0, -0.129, 0.168)),
        material=control_panel,
        name="control_panel",
    )
    base.visual(
        Box((0.032, 0.018, 0.334)),
        origin=Origin(xyz=(-0.102, 0.111, 0.497)),
        material=body_dark,
        name="rear_post_0",
    )
    base.visual(
        Box((0.032, 0.018, 0.334)),
        origin=Origin(xyz=(0.102, 0.111, 0.497)),
        material=body_dark,
        name="rear_post_1",
    )
    base.visual(
        Box((0.188, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, 0.108, 0.502)),
        material=body_dark,
        name="rear_crossbar",
    )
    base.visual(
        Box((0.034, 0.022, 0.040)),
        origin=Origin(xyz=(-0.084, HOOD_HINGE_Y, HOOD_HINGE_Z - 0.020)),
        material=body_dark,
        name="hinge_block_0",
    )
    base.visual(
        Box((0.034, 0.022, 0.040)),
        origin=Origin(xyz=(0.084, HOOD_HINGE_Y, HOOD_HINGE_Z - 0.020)),
        material=body_dark,
        name="hinge_block_1",
    )

    pitcher = model.part("pitcher")
    pitcher.visual(
        Cylinder(radius=0.041, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=lid_black,
        name="pitcher_collar",
    )
    pitcher.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=steel,
        name="spindle_mount",
    )
    pitcher.visual(
        mesh_from_cadquery(_pitcher_shell_shape(), "pitcher_shell"),
        material=glass,
        name="pitcher_shell",
    )
    pitcher.visual(
        mesh_from_cadquery(_pitcher_handle_shape(), "pitcher_handle"),
        material=lid_black,
        name="pitcher_handle",
    )
    pitcher.visual(
        Box((0.158, 0.138, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, PITCHER_HEIGHT + 0.004)),
        material=lid_black,
        name="lid",
    )
    pitcher.visual(
        Box((0.046, 0.046, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, PITCHER_HEIGHT + 0.016)),
        material=lid_black,
        name="lid_cap",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.012, length=0.010),
        material=steel,
        name="blade_hub",
    )
    blade.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=steel,
        name="blade_shaft",
    )
    blade.visual(
        Box((0.074, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.004), rpy=(0.32, 0.0, 0.08)),
        material=steel,
        name="blade_0",
    )
    blade.visual(
        Box((0.074, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.004), rpy=(-0.32, 0.0, math.pi / 2.0 + 0.08)),
        material=steel,
        name="blade_1",
    )

    hood = model.part("hood")
    hood.visual(
        Box((HOOD_WIDTH, HOOD_DEPTH, HOOD_PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, -HOOD_DEPTH / 2.0, -0.018)),
        material=hood_smoke,
        name="top_panel",
    )
    hood.visual(
        Box((HOOD_PANEL_THICKNESS, HOOD_DEPTH, HOOD_SIDE_HEIGHT)),
        origin=Origin(xyz=(-HOOD_WIDTH / 2.0, -HOOD_DEPTH / 2.0, -HOOD_SIDE_HEIGHT / 2.0)),
        material=hood_smoke,
        name="side_panel_0",
    )
    hood.visual(
        Box((HOOD_PANEL_THICKNESS, HOOD_DEPTH, HOOD_SIDE_HEIGHT)),
        origin=Origin(xyz=(HOOD_WIDTH / 2.0, -HOOD_DEPTH / 2.0, -HOOD_SIDE_HEIGHT / 2.0)),
        material=hood_smoke,
        name="side_panel_1",
    )
    hood.visual(
        Box((HOOD_WIDTH, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, -0.029, -0.020)),
        material=hood_smoke,
        name="rear_spine",
    )

    knob = model.part("speed_knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="shaft",
    )
    knob.visual(
        _speed_knob_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_dark,
        name="knob_cap",
    )

    model.articulation(
        "base_to_pitcher",
        ArticulationType.FIXED,
        parent=base,
        child=pitcher,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + DECK_PAD_THICKNESS)),
    )
    model.articulation(
        "pitcher_to_blade",
        ArticulationType.CONTINUOUS,
        parent=pitcher,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=30.0),
    )
    model.articulation(
        "base_to_hood",
        ArticulationType.REVOLUTE,
        parent=base,
        child=hood,
        origin=Origin(xyz=(0.0, HOOD_HINGE_Y, HOOD_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.9),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.0, -0.141, 0.168), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pitcher = object_model.get_part("pitcher")
    blade = object_model.get_part("blade")
    hood = object_model.get_part("hood")
    knob = object_model.get_part("speed_knob")

    hood_hinge = object_model.get_articulation("base_to_hood")
    blade_spin = object_model.get_articulation("pitcher_to_blade")
    knob_spin = object_model.get_articulation("base_to_speed_knob")

    ctx.expect_contact(
        pitcher,
        base,
        elem_a="pitcher_collar",
        elem_b="deck_pad",
        name="pitcher seats on the rubber deck pad",
    )
    ctx.expect_within(
        blade,
        pitcher,
        axes="xy",
        margin=0.012,
        inner_elem="blade_0",
        outer_elem="pitcher_shell",
        name="blade span stays inside the pitcher footprint",
    )

    with ctx.pose({hood_hinge: 0.0}):
        ctx.expect_gap(
            hood,
            pitcher,
            axis="z",
            min_gap=0.001,
            max_gap=0.030,
            positive_elem="top_panel",
            negative_elem="lid",
            name="closed hood clears the pitcher lid",
        )
        ctx.expect_overlap(
            hood,
            pitcher,
            axes="xy",
            min_overlap=0.120,
            elem_a="top_panel",
            elem_b="pitcher_shell",
            name="closed hood covers the pitcher footprint",
        )

    rest_hood_aabb = ctx.part_world_aabb(hood)
    with ctx.pose({hood_hinge: 1.65}):
        open_hood_aabb = ctx.part_world_aabb(hood)

    ctx.check(
        "hood lifts upward when opened",
        rest_hood_aabb is not None
        and open_hood_aabb is not None
        and open_hood_aabb[1][2] > rest_hood_aabb[1][2] + 0.12,
        details=f"rest={rest_hood_aabb}, open={open_hood_aabb}",
    )
    ctx.check(
        "blade uses a continuous spin joint",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and blade_spin.motion_limits is not None
        and blade_spin.motion_limits.lower is None
        and blade_spin.motion_limits.upper is None,
        details=str(blade_spin.motion_limits),
    )
    ctx.check(
        "speed knob uses a continuous spin joint",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.motion_limits is not None
        and knob_spin.motion_limits.lower is None
        and knob_spin.motion_limits.upper is None,
        details=str(knob_spin.motion_limits),
    )
    ctx.check(
        "hood hinge opens upward from the rear support line",
        hood_hinge.motion_limits is not None
        and hood_hinge.motion_limits.lower == 0.0
        and hood_hinge.motion_limits.upper is not None
        and hood_hinge.motion_limits.upper > 1.5,
        details=str(hood_hinge.motion_limits),
    )
    ctx.check(
        "knob stays forward of the base body",
        ctx.part_world_position(knob) is not None
        and ctx.part_world_position(base) is not None
        and ctx.part_world_position(knob)[1] < ctx.part_world_position(base)[1] - 0.10,
        details=f"base={ctx.part_world_position(base)}, knob={ctx.part_world_position(knob)}",
    )

    return ctx.report()


object_model = build_object_model()
