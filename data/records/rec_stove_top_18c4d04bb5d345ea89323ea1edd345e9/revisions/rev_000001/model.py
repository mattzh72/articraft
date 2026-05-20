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


WIDTH = 0.76
DEPTH = 0.64
HEIGHT = 0.90
FRONT_Y = -DEPTH / 2.0


def _annulus_mesh(outer_radius: float, inner_radius: float, thickness: float, name: str):
    ring = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(thickness)
    return mesh_from_cadquery(ring, name, tolerance=0.0008, angular_tolerance=0.08)


def _cabinet_body_mesh():
    body = cq.Workplane("XY").box(WIDTH, DEPTH, HEIGHT).translate((0.0, 0.0, HEIGHT / 2.0))
    oven_cutout = (
        cq.Workplane("XY")
        .box(0.615, 0.16, 0.535)
        .translate((0.0, FRONT_Y - 0.035, 0.365))
    )
    lower_toe_relief = (
        cq.Workplane("XY")
        .box(0.58, 0.10, 0.070)
        .translate((0.0, FRONT_Y - 0.030, 0.045))
    )
    body = body.cut(oven_cutout).cut(lower_toe_relief)
    return mesh_from_cadquery(body, "enamel_cabinet_body", tolerance=0.0012, angular_tolerance=0.10)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smooth_top_electric_stove")

    enamel = model.material("warm_white_enamel", rgba=(0.88, 0.86, 0.80, 1.0))
    dark_glass = model.material("black_ceramic_glass", rgba=(0.015, 0.016, 0.018, 1.0))
    smoked_glass = model.material("smoked_oven_window", rgba=(0.02, 0.025, 0.030, 1.0))
    charcoal = model.material("charcoal_trim", rgba=(0.05, 0.052, 0.052, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.60, 0.56, 1.0))
    red_zone = model.material("muted_radiant_red", rgba=(0.82, 0.10, 0.04, 0.90))
    grey_mark = model.material("etched_grey_markings", rgba=(0.34, 0.34, 0.34, 1.0))
    button_mat = model.material("satin_black_buttons", rgba=(0.025, 0.025, 0.024, 1.0))

    cabinet = model.part("cabinet")

    # Main freestanding range body: a single boxy enamel shell with a dark
    # recessed plinth and broad lower oven bay.
    cabinet.visual(
        _cabinet_body_mesh(),
        material=enamel,
        name="enamel_body",
    )
    cabinet.visual(
        Box((WIDTH + 0.018, DEPTH + 0.018, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT + 0.0150)),
        material=charcoal,
        name="top_lip",
    )
    cabinet.visual(
        Box((0.70, 0.56, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT + 0.041)),
        material=dark_glass,
        name="ceramic_cooktop",
    )
    cabinet.visual(
        Box((0.72, 0.022, 0.11)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.009, 0.735)),
        material=brushed,
        name="front_control_panel",
    )
    cabinet.visual(
        Box((0.72, 0.018, 0.075)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.008, 0.047)),
        material=charcoal,
        name="toe_kick",
    )

    # Oven opening shadow and perimeter reveal behind the hinged door.
    cabinet.visual(
        Box((0.62, 0.006, 0.50)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.032, 0.365)),
        material=charcoal,
        name="oven_shadow_recess",
    )
    cabinet.visual(
        Box((0.66, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.001, 0.630)),
        material=brushed,
        name="oven_upper_reveal",
    )
    cabinet.visual(
        Box((0.035, 0.008, 0.54)),
        origin=Origin(xyz=(-0.347, FRONT_Y + 0.001, 0.365)),
        material=brushed,
        name="oven_side_reveal_0",
    )
    cabinet.visual(
        Box((0.035, 0.008, 0.54)),
        origin=Origin(xyz=(0.347, FRONT_Y + 0.001, 0.365)),
        material=brushed,
        name="oven_side_reveal_1",
    )

    # Four radiant zones under the ceramic glass.  The annular outlines make the
    # smooth top read as electric radiant rather than gas or induction.
    ring_mesh_large = _annulus_mesh(0.092, 0.085, 0.0016, "large_radiant_ring")
    ring_mesh_small = _annulus_mesh(0.073, 0.067, 0.0016, "small_radiant_ring")
    inner_ring_mesh = _annulus_mesh(0.048, 0.043, 0.0016, "inner_radiant_ring")
    for name, x, y, large in (
        ("zone_0", -0.19, 0.16, True),
        ("zone_1", 0.18, 0.16, False),
        ("zone_2", -0.17, -0.13, False),
        ("zone_3", 0.20, -0.12, True),
    ):
        mesh = ring_mesh_large if large else ring_mesh_small
        cabinet.visual(
            mesh,
            origin=Origin(xyz=(x, y, HEIGHT + 0.0492)),
            material=grey_mark,
            name=f"{name}_outline",
        )
        cabinet.visual(
            inner_ring_mesh,
            origin=Origin(xyz=(x, y, HEIGHT + 0.0495)),
            material=red_zone,
            name=f"{name}_glow",
        )
        cabinet.visual(
            Cylinder(radius=0.010, length=0.0016),
            origin=Origin(xyz=(x, y, HEIGHT + 0.0508)),
            material=red_zone,
            name=f"{name}_center_dot",
        )

    # Simple printed legends above the controls, kept as raised/thin marks so
    # they remain part of the fixed control fascia.
    for i, x in enumerate((-0.175, -0.070, 0.080, 0.195)):
        cabinet.visual(
            Box((0.036, 0.0025, 0.006)),
            origin=Origin(xyz=(x, FRONT_Y - 0.0205, 0.782)),
            material=charcoal,
            name=f"control_tick_{i}",
        )

    # Two long push buttons left of center.  Their part frame is at the rear
    # seating plane; positive prismatic travel moves into the appliance.
    for i, x in enumerate((-0.165, -0.060)):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.088, 0.022, 0.030)),
            origin=Origin(xyz=(0.0, -0.011, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.070, 0.004, 0.004)),
            origin=Origin(xyz=(0.0, -0.0235, 0.006)),
            material=grey_mark,
            name="button_highlight",
        )
        model.articulation(
            f"cabinet_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y - 0.020, 0.735)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=0.016),
        )

    # Two skirted range knobs right of center, with their local Z axes turned to
    # face out of the front panel.  They are continuous rotary controls.
    knob_geometry = KnobGeometry(
        0.056,
        0.034,
        body_style="skirted",
        top_diameter=0.040,
        skirt=KnobSkirt(0.066, 0.006, flare=0.06, chamfer=0.0012),
        grip=KnobGrip(style="fluted", count=20, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        bore=KnobBore(style="d_shaft", diameter=0.008, flat_depth=0.001),
        center=False,
    )
    knob_mesh = mesh_from_geometry(knob_geometry, "skirted_front_knob")
    for i, x in enumerate((0.105, 0.215)):
        knob = model.part(f"knob_{i}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="knob_cap",
        )
        model.articulation(
            f"cabinet_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=knob,
            origin=Origin(xyz=(x, FRONT_Y - 0.020, 0.735)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=5.0),
        )

    # Bottom-hinged oven door.  The child frame sits on the hinge line, and the
    # door geometry extends upward in local +Z and forward in local -Y.
    door = model.part("oven_door")
    door.visual(
        Box((0.64, 0.034, 0.50)),
        origin=Origin(xyz=(0.0, -0.017, 0.25)),
        material=enamel,
        name="door_panel",
    )
    door.visual(
        Box((0.42, 0.006, 0.205)),
        origin=Origin(xyz=(0.0, -0.0365, 0.290)),
        material=smoked_glass,
        name="window_glass",
    )
    door.visual(
        Box((0.47, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, -0.0385, 0.430)),
        material=brushed,
        name="window_top_trim",
    )
    door.visual(
        Box((0.47, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, -0.0385, 0.150)),
        material=brushed,
        name="window_bottom_trim",
    )
    door.visual(
        Box((0.026, 0.010, 0.225)),
        origin=Origin(xyz=(-0.235, -0.0385, 0.290)),
        material=brushed,
        name="window_side_trim_0",
    )
    door.visual(
        Box((0.026, 0.010, 0.225)),
        origin=Origin(xyz=(0.235, -0.0385, 0.290)),
        material=brushed,
        name="window_side_trim_1",
    )
    door.visual(
        Cylinder(radius=0.016, length=0.50),
        origin=Origin(xyz=(0.0, -0.073, 0.470), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="door_handle_bar",
    )
    door.visual(
        Box((0.035, 0.052, 0.030)),
        origin=Origin(xyz=(-0.215, -0.049, 0.470)),
        material=brushed,
        name="door_handle_mount_0",
    )
    door.visual(
        Box((0.035, 0.052, 0.030)),
        origin=Origin(xyz=(0.215, -0.049, 0.470)),
        material=brushed,
        name="door_handle_mount_1",
    )
    model.articulation(
        "cabinet_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, FRONT_Y, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.65, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("oven_door")
    door_joint = object_model.get_articulation("cabinet_to_oven_door")

    # Closed oven door is broad, mounted on the lower front edge, and swings
    # outward/downward at the upper-limit pose.
    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            min_gap=-0.010,
            max_gap=0.004,
            positive_elem="enamel_body",
            negative_elem="door_panel",
            name="closed door sits on front face",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.45,
            elem_a="door_panel",
            elem_b="oven_shadow_recess",
            name="oven door covers broad oven bay",
        )
        closed_top = ctx.part_element_world_aabb(door, elem="door_handle_bar")

    with ctx.pose({door_joint: 1.05}):
        open_top = ctx.part_element_world_aabb(door, elem="door_handle_bar")
    ctx.check(
        "oven door opens downward and outward",
        closed_top is not None
        and open_top is not None
        and open_top[0][1] < closed_top[0][1] - 0.08
        and open_top[1][2] < closed_top[1][2] - 0.10,
        details=f"closed={closed_top}, open={open_top}",
    )

    for i in range(2):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"cabinet_to_button_{i}")
        ctx.check(
            f"button_{i} has short inward prismatic travel",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.motion_limits is not None
            and 0.010 <= joint.motion_limits.upper <= 0.025
            and joint.axis == (0.0, 1.0, 0.0),
        )
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: joint.motion_limits.upper}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} moves into the front panel",
            rest is not None and pressed is not None and pressed[1] > rest[1] + 0.010,
            details=f"rest={rest}, pressed={pressed}",
        )

    for i in range(2):
        knob_joint = object_model.get_articulation(f"cabinet_to_knob_{i}")
        ctx.check(
            f"knob_{i} is a continuous front-axis rotary control",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS
            and knob_joint.axis == (0.0, -1.0, 0.0),
        )

    return ctx.report()


object_model = build_object_model()
