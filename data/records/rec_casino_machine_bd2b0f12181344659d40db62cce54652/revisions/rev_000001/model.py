from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _button_sleeve_mesh():
    """A real hollow collar: the red plunger stem slides through the bore."""
    sleeve = (
        cq.Workplane("XY")
        .circle(0.066)
        .circle(0.034)
        .extrude(0.067)
    )
    return mesh_from_cadquery(sleeve, "spin_button_guide_sleeve", tolerance=0.0005)


def _tile_y(radius: float, z_offset: float, thickness: float) -> float:
    """Place flat reel symbol tiles so each tile locally bites into the drum skin."""
    surface = -math.sqrt(max(radius * radius - z_offset * z_offset, 0.0))
    return surface - 0.5 * thickness + 0.003


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_five_reel_casino_cabinet")

    black = model.material("satin_black", rgba=(0.015, 0.014, 0.017, 1.0))
    graphite = model.material("graphite_metal", rgba=(0.10, 0.105, 0.115, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.72, 0.68, 1.0))
    glass = model.material("smoked_glass", rgba=(0.08, 0.18, 0.26, 0.38))
    reel_plastic = model.material("warm_reel_plastic", rgba=(0.94, 0.89, 0.72, 1.0))
    red = model.material("translucent_red", rgba=(0.95, 0.03, 0.02, 1.0))
    amber = model.material("amber_led", rgba=(1.0, 0.62, 0.05, 1.0))
    blue_led = model.material("blue_led", rgba=(0.06, 0.30, 0.95, 1.0))
    green = model.material("green_symbol", rgba=(0.08, 0.72, 0.18, 1.0))
    gold = model.material("gold_symbol", rgba=(0.95, 0.76, 0.18, 1.0))
    violet = model.material("violet_symbol", rgba=(0.58, 0.20, 0.95, 1.0))
    white = model.material("white_symbol", rgba=(0.96, 0.95, 0.90, 1.0))

    cabinet = model.part("cabinet")

    # Main carcass: a broad, glossy machine body with a rear service opening.
    cabinet.visual(Box((1.20, 0.76, 0.20)), origin=Origin(xyz=(0.0, 0.0, 0.10)), material=graphite, name="base_plinth")
    cabinet.visual(Box((0.085, 0.76, 1.38)), origin=Origin(xyz=(-0.585, 0.0, 0.85)), material=black, name="side_column_0")
    cabinet.visual(Box((0.085, 0.76, 1.38)), origin=Origin(xyz=(0.585, 0.0, 0.85)), material=black, name="side_column_1")
    cabinet.visual(Box((1.20, 0.76, 0.12)), origin=Origin(xyz=(0.0, 0.0, 1.59)), material=black, name="top_cap")
    cabinet.visual(Box((1.13, 0.13, 0.56)), origin=Origin(xyz=(0.0, -0.335, 0.43)), material=graphite, name="lower_front_panel")
    cabinet.visual(Box((1.16, 0.34, 0.080)), origin=Origin(xyz=(0.0, -0.455, 0.620)), material=black, name="control_deck")
    cabinet.visual(Box((1.16, 0.020, 0.46)), origin=Origin(xyz=(0.0, 0.030, 1.14)), material=black, name="reel_shadow_box")

    # Front display face with a real framed opening rather than a solid panel.
    display_bezel = BezelGeometry(
        (0.995, 0.430),
        (1.145, 0.610),
        0.040,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.030,
        outer_corner_radius=0.050,
    )
    cabinet.visual(
        mesh_from_geometry(display_bezel, "broad_display_bezel"),
        origin=Origin(xyz=(0.0, -0.370, 1.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="display_bezel",
    )
    cabinet.visual(Box((1.000, 0.006, 0.420)), origin=Origin(xyz=(0.0, -0.404, 1.150)), material=glass, name="display_glass")
    for idx, x in enumerate((-0.30, -0.10, 0.10, 0.30)):
        cabinet.visual(Box((0.014, 0.018, 0.440)), origin=Origin(xyz=(x, -0.398, 1.150)), material=chrome, name=f"reel_separator_{idx}")

    # Marquee and credit/readout strips on the broad face.
    cabinet.visual(Box((0.92, 0.030, 0.085)), origin=Origin(xyz=(0.0, -0.392, 1.475)), material=blue_led, name="top_marquee")
    cabinet.visual(Box((0.42, 0.010, 0.050)), origin=Origin(xyz=(-0.30, -0.618, 0.685)), material=amber, name="credit_display")

    # Rear frame around the hinged maintenance panel.
    cabinet.visual(Box((1.13, 0.040, 0.065)), origin=Origin(xyz=(0.0, 0.360, 1.475)), material=graphite, name="rear_top_rail")
    cabinet.visual(Box((1.13, 0.040, 0.065)), origin=Origin(xyz=(0.0, 0.360, 0.365)), material=graphite, name="rear_bottom_rail")
    cabinet.visual(Box((0.055, 0.040, 1.08)), origin=Origin(xyz=(-0.555, 0.360, 0.920)), material=graphite, name="rear_edge_0")
    cabinet.visual(Box((0.055, 0.040, 1.08)), origin=Origin(xyz=(0.555, 0.360, 0.920)), material=graphite, name="rear_edge_1")
    cabinet.visual(Cylinder(0.014, 1.08), origin=Origin(xyz=(-0.618, 0.392, 0.920)), material=chrome, name="rear_hinge_pin")

    # The static guide sleeve is a hollow ring proud of the shallow deck.
    cabinet.visual(
        _button_sleeve_mesh(),
        origin=Origin(xyz=(0.380, -0.520, 0.655)),
        material=chrome,
        name="spin_button_sleeve",
    )

    # Five independently rotating reels.  Each part carries its shaft segment,
    # drum, side flanges, and symbol tiles so the whole reel spins together.
    reel_radius = 0.180
    reel_width = 0.150
    tile_thickness = 0.010
    tile_mats: tuple[Material, ...] = (red, green, gold, violet, white)
    reel_xs = (-0.400, -0.200, 0.0, 0.200, 0.400)
    for i, x in enumerate(reel_xs):
        for side, sx in enumerate((x - 0.096, x + 0.096)):
            cabinet.visual(
                Box((0.016, 0.040, 0.028)),
                origin=Origin(xyz=(sx, -0.200, 1.122)),
                material=chrome,
                name=f"reel_{i}_bearing_{side}",
            )
            cabinet.visual(
                Box((0.016, 0.240, 0.020)),
                origin=Origin(xyz=(sx, -0.085, 1.108)),
                material=graphite,
                name=f"reel_{i}_bearing_arm_{side}",
            )
        reel = model.part(f"reel_{i}")
        reel.visual(
            Cylinder(reel_radius, reel_width),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=reel_plastic,
            name="drum",
        )
        reel.visual(
            Cylinder(0.014, 0.190),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="shaft",
        )
        reel.visual(
            Cylinder(reel_radius + 0.006, 0.012),
            origin=Origin(xyz=(-0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="flange_0",
        )
        reel.visual(
            Cylinder(reel_radius + 0.006, 0.012),
            origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="flange_1",
        )
        for j, zoff in enumerate((-0.105, 0.0, 0.105)):
            reel.visual(
                Box((0.118, tile_thickness, 0.064)),
                origin=Origin(xyz=(0.0, _tile_y(reel_radius, zoff, tile_thickness), zoff)),
                material=tile_mats[(i + j) % len(tile_mats)],
                name=f"symbol_tile_{j}",
            )
        model.articulation(
            f"cabinet_to_reel_{i}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=reel,
            origin=Origin(xyz=(x, -0.200, 1.150)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=20.0),
            motion_properties=MotionProperties(damping=0.015, friction=0.0),
        )

    # Spring-loaded spin button.  The child frame is at the underside of the cap
    # at the unpressed position; positive motion translates the plunger downward.
    spin_button = model.part("spin_button")
    spin_button.visual(Cylinder(0.055, 0.028), origin=Origin(xyz=(0.0, 0.0, 0.014)), material=red, name="cap")
    spin_button.visual(Cylinder(0.025, 0.050), origin=Origin(xyz=(0.0, 0.0, -0.025)), material=chrome, name="stem")
    model.articulation(
        "cabinet_to_spin_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=spin_button,
        origin=Origin(xyz=(0.380, -0.520, 0.722)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.12, lower=0.0, upper=0.012),
    )

    # Rear maintenance panel, hinged along one vertical cabinet edge.
    panel = model.part("rear_panel")
    panel.visual(Box((1.050, 0.035, 1.050)), origin=Origin(xyz=(0.538, 0.0, 0.0)), material=graphite, name="panel_slab")
    panel.visual(Box((0.020, 0.024, 1.050)), origin=Origin(xyz=(0.005, 0.0, 0.0)), material=chrome, name="hinge_leaf")
    panel.visual(Box((0.070, 0.030, 0.160)), origin=Origin(xyz=(0.950, 0.030, 0.050)), material=black, name="pull_handle")
    rear_vent = SlotPatternPanelGeometry(
        (0.480, 0.300),
        0.004,
        slot_size=(0.060, 0.010),
        pitch=(0.085, 0.035),
        frame=0.020,
        corner_radius=0.012,
        stagger=True,
    )
    panel.visual(
        mesh_from_geometry(rear_vent, "rear_panel_slotted_vent"),
        origin=Origin(xyz=(0.550, 0.019, 0.120), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="service_vent",
    )
    model.articulation(
        "cabinet_to_rear_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=panel,
        origin=Origin(xyz=(-0.600, 0.405, 0.920)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "cabinet",
        "rear_panel",
        elem_a="rear_hinge_pin",
        elem_b="hinge_leaf",
        reason="The maintenance-door hinge leaf is intentionally wrapped into the fixed hinge pin clearance.",
    )
    ctx.expect_gap(
        "rear_panel",
        "cabinet",
        axis="x",
        positive_elem="hinge_leaf",
        negative_elem="rear_hinge_pin",
        max_gap=0.001,
        max_penetration=0.002,
        name="rear hinge leaf is seated on the hinge pin",
    )

    reel_joints = [object_model.get_articulation(f"cabinet_to_reel_{i}") for i in range(5)]
    for i, joint in enumerate(reel_joints):
        ctx.check(
            f"reel {i} is continuous on a horizontal shaft",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    button = object_model.get_part("spin_button")
    button_joint = object_model.get_articulation("cabinet_to_spin_button")
    rest_button_pos = ctx.part_world_position(button)
    ctx.expect_within(
        button,
        "cabinet",
        axes="xy",
        inner_elem="stem",
        outer_elem="spin_button_sleeve",
        margin=0.0,
        name="button stem is guided by sleeve footprint",
    )
    with ctx.pose({button_joint: 0.012}):
        pressed_button_pos = ctx.part_world_position(button)
        ctx.expect_within(
            button,
            "cabinet",
            axes="xy",
            inner_elem="stem",
            outer_elem="spin_button_sleeve",
            margin=0.0,
            name="pressed stem remains in sleeve footprint",
        )
    ctx.check(
        "spin button depresses downward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.010,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    panel = object_model.get_part("rear_panel")
    panel_joint = object_model.get_articulation("cabinet_to_rear_panel")
    closed_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_joint: 1.25}):
        open_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "rear panel swings outward behind cabinet",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][1] > closed_aabb[1][1] + 0.25,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
