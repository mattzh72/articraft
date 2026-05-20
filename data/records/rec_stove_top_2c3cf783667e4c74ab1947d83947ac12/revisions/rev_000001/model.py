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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_six_burner_range")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_steel = model.material("blackened_cast_iron", rgba=(0.025, 0.024, 0.022, 1.0))
    enamel_black = model.material("gloss_black_enamel", rgba=(0.005, 0.006, 0.007, 1.0))
    graphite = model.material("graphite_control_faces", rgba=(0.18, 0.18, 0.17, 1.0))
    glass = model.material("smoked_oven_glass", rgba=(0.02, 0.035, 0.045, 0.82))
    warm_steel = model.material("warm_knob_metal", rgba=(0.80, 0.77, 0.70, 1.0))

    body = model.part("range_body")

    # Main professional range carcass and the deliberately deep projecting
    # control rail.  The broad rail is the semantic anchor for the controls.
    body.visual(
        Box((0.96, 0.68, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=stainless,
        name="lower_body",
    )
    body.visual(
        Box((0.99, 0.13, 0.17)),
        origin=Origin(xyz=(0.0, 0.405, 0.695)),
        material=stainless,
        name="deep_front_rail",
    )
    body.visual(
        Box((0.99, 0.76, 0.035)),
        origin=Origin(xyz=(0.0, -0.01, 0.7975)),
        material=stainless,
        name="cooking_deck",
    )
    body.visual(
        Box((0.94, 0.05, 0.11)),
        origin=Origin(xyz=(0.0, -0.365, 0.852)),
        material=stainless,
        name="low_back_splash",
    )
    body.visual(
        Box((0.91, 0.12, 0.07)),
        origin=Origin(xyz=(0.0, 0.285, 0.035)),
        material=enamel_black,
        name="recessed_toe_kick",
    )

    # Six visible burners laid out in two rows of three, with black caps and
    # stainless burner rings sitting directly on the cooktop.
    burner_xs = (-0.31, 0.0, 0.31)
    burner_ys = (-0.19, 0.12)
    for row, y in enumerate(burner_ys):
        for col, x in enumerate(burner_xs):
            idx = row * 3 + col
            body.visual(
                Cylinder(radius=0.076, length=0.006),
                origin=Origin(xyz=(x, y, 0.818)),
                material=stainless,
                name=f"burner_ring_{idx}",
            )
            body.visual(
                Cylinder(radius=0.052, length=0.012),
                origin=Origin(xyz=(x, y, 0.824)),
                material=enamel_black,
                name=f"burner_cap_{idx}",
            )

    # A connected cast-iron grate grid with feet down to the cooktop so the
    # raised bars read as supported, not floating.
    grate_z = 0.837
    for i, x in enumerate((-0.45, -0.155, 0.155, 0.45)):
        body.visual(
            Box((0.025, 0.58, 0.018)),
            origin=Origin(xyz=(x, -0.035, grate_z)),
            material=dark_steel,
            name=f"grate_long_bar_{i}",
        )
    for i, y in enumerate((-0.305, -0.035, 0.235)):
        body.visual(
            Box((0.91, 0.025, 0.018)),
            origin=Origin(xyz=(0.0, y, grate_z)),
            material=dark_steel,
            name=f"grate_cross_bar_{i}",
        )
    for i, x in enumerate((-0.45, -0.155, 0.155, 0.45)):
        for j, y in enumerate((-0.305, 0.235)):
            body.visual(
                Box((0.034, 0.034, 0.024)),
                origin=Origin(xyz=(x, y, 0.827)),
                material=dark_steel,
                name=f"grate_foot_{i}_{j}",
            )

    # Body-side hinge knuckles and support tabs for the oven door hinge.
    for side, x in enumerate((-0.435, 0.435)):
        body.visual(
            Box((0.075, 0.020, 0.052)),
            origin=Origin(xyz=(x, 0.348, 0.095)),
            material=dark_steel,
            name=f"door_hinge_tab_{side}",
        )
        body.visual(
            Cylinder(radius=0.020, length=0.060),
            origin=Origin(xyz=(x, 0.360, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"door_hinge_knuckle_{side}",
        )

    # Control bushings mounted into the rail; the moving knobs start just in
    # front of these collars.
    knob_xs = (-0.43, -0.225, -0.075, 0.075, 0.225, 0.43)
    rail_front_y = 0.470
    bushing_front_y = rail_front_y + 0.008
    knob_z = 0.705
    for i, x in enumerate(knob_xs):
        radius = 0.026 if i in (0, 5) else 0.031
        body.visual(
            Cylinder(radius=radius, length=0.008),
            origin=Origin(
                xyz=(x, rail_front_y + 0.004, knob_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=graphite,
            name=f"knob_bushing_{i}",
        )

    button_xs = (-0.075, 0.075)
    button_z = 0.635
    button_plate_front_y = rail_front_y + 0.005
    for i, x in enumerate(button_xs):
        body.visual(
            Box((0.052, 0.010, 0.033)),
            origin=Origin(xyz=(x, rail_front_y + 0.005, button_z)),
            material=graphite,
            name=f"button_plate_{i}",
        )

    # Continuous rotary knobs.  The center four are taller, while the two far
    # end knobs are intentionally shorter and smaller as requested.
    center_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.058,
            0.043,
            body_style="skirted",
            top_diameter=0.041,
            skirt=KnobSkirt(0.067, 0.008, flare=0.06, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=20, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            bore=KnobBore(style="d_shaft", diameter=0.007, flat_depth=0.001),
            center=False,
        ),
        "center_range_knob",
    )
    end_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.045,
            0.029,
            body_style="skirted",
            top_diameter=0.033,
            skirt=KnobSkirt(0.052, 0.006, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            center=False,
        ),
        "short_end_range_knob",
    )

    knob_parts = []
    for i, x in enumerate(knob_xs):
        knob = model.part(f"knob_{i}")
        knob.visual(
            end_knob_mesh if i in (0, 5) else center_knob_mesh,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=warm_steel,
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name="knob_rear_stem",
        )
        model.articulation(
            f"range_body_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, bushing_front_y + 0.0020, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=8.0),
        )
        knob_parts.append(knob)

    # Two short push-buttons directly under the middle pair of knobs.  Their
    # positive prismatic travel moves inward along the front-to-back axis.
    for i, x in enumerate(button_xs):
        button = model.part(f"button_{i}")
        button.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=warm_steel,
            name="button_cap",
        )
        model.articulation(
            f"range_body_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, button_plate_front_y + 0.0005, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.25, lower=0.0, upper=0.012),
        )

    # Lower oven door spanning most of the range width.  The child frame is on
    # the bottom hinge axis so positive revolute travel swings the door down.
    door = model.part("oven_door")
    door.visual(
        Box((0.78, 0.052, 0.50)),
        origin=Origin(xyz=(0.0, 0.006, 0.250)),
        material=stainless,
        name="door_slab",
    )
    door.visual(
        Box((0.52, 0.006, 0.235)),
        origin=Origin(xyz=(0.0, 0.036, 0.285)),
        material=glass,
        name="glass_window",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="door_hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.62),
        origin=Origin(xyz=(0.0, 0.078, 0.420), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="oven_handle_bar",
    )
    for i, x in enumerate((-0.25, 0.25)):
        door.visual(
            Box((0.045, 0.052, 0.040)),
            origin=Origin(xyz=(x, 0.054, 0.420)),
            material=dark_steel,
            name=f"handle_standoff_{i}",
        )

    model.articulation(
        "range_body_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, 0.360, 0.095)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    knobs = [object_model.get_part(f"knob_{i}") for i in range(6)]
    knob_joints = [object_model.get_articulation(f"range_body_to_knob_{i}") for i in range(6)]
    button_joints = [object_model.get_articulation(f"range_body_to_button_{i}") for i in range(2)]
    door = object_model.get_part("oven_door")
    door_joint = object_model.get_articulation("range_body_to_oven_door")

    ctx.check(
        "six continuous front-to-back rotary knobs",
        len(knob_joints) == 6
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in knob_joints)
        and all(tuple(j.axis) == (0.0, 1.0, 0.0) for j in knob_joints),
        details=f"knob joints={knob_joints}",
    )
    for knob in knobs:
        ctx.expect_gap(
            knob,
            "range_body",
            axis="y",
            max_gap=0.004,
            max_penetration=0.0,
            elem_a="knob_cap",
            name=f"{knob.name} seats just proud of the rail bushing",
        )

    rest_button_positions = [ctx.part_world_position(object_model.get_part(f"button_{i}")) for i in range(2)]
    with ctx.pose({button_joints[0]: 0.012, button_joints[1]: 0.012}):
        pressed_button_positions = [
            ctx.part_world_position(object_model.get_part(f"button_{i}")) for i in range(2)
        ]
    ctx.check(
        "push buttons plunge inward",
        all(
            rest is not None and pressed is not None and pressed[1] < rest[1] - 0.010
            for rest, pressed in zip(rest_button_positions, pressed_button_positions)
        ),
        details=f"rest={rest_button_positions}, pressed={pressed_button_positions}",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.35}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "oven door rotates downward from bottom hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] < closed_aabb[1][2] - 0.16
        and open_aabb[1][1] > closed_aabb[1][1] + 0.18,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
