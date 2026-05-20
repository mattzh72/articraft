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
    model = ArticulatedObject(name="retrofit_wall_thermostat")

    bakelite = Material("aged_bakelite", rgba=(0.19, 0.16, 0.12, 1.0))
    cream = Material("aged_cream_face", rgba=(0.77, 0.70, 0.58, 1.0))
    brass = Material("brass_hardware", rgba=(0.70, 0.52, 0.24, 1.0))
    dark_metal = Material("dark_oxide_screws", rgba=(0.06, 0.055, 0.05, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    red = Material("warm_red_mark", rgba=(0.72, 0.10, 0.06, 1.0))
    blue = Material("cool_blue_mark", rgba=(0.05, 0.16, 0.55, 1.0))

    body = model.part("body")

    # Root structure: a retrofit adapter plate on the wall, an older shallow
    # Bakelite cover, metal side adapters, reinforcements, hinge brackets, and
    # the fixed center shaft/retainer that carries the dial.
    body.visual(
        Box((0.190, 0.012, 0.245)),
        origin=Origin(xyz=(0.0, 0.006, 0.125)),
        material=cream,
        name="wall_backplate",
    )
    body.visual(
        Box((0.168, 0.012, 0.218)),
        origin=Origin(xyz=(0.0, 0.017, 0.125)),
        material=brass,
        name="adapter_plate",
    )
    body.visual(
        Box((0.148, 0.028, 0.188)),
        origin=Origin(xyz=(0.0, 0.033, 0.125)),
        material=bakelite,
        name="main_cover",
    )
    body.visual(
        Box((0.164, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.037, 0.224)),
        material=brass,
        name="top_adapter_rail",
    )
    body.visual(
        Box((0.164, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.037, 0.026)),
        material=brass,
        name="bottom_adapter_rail",
    )
    for i, x in enumerate((-0.086, 0.086)):
        body.visual(
            Box((0.018, 0.017, 0.220)),
            origin=Origin(xyz=(x, 0.025, 0.125)),
            material=brass,
            name=f"side_adapter_{i}",
        )
    for i, x in enumerate((-0.064, 0.064)):
        body.visual(
            Box((0.010, 0.020, 0.176)),
            origin=Origin(xyz=(x, 0.046, 0.125)),
            material=bakelite,
            name=f"front_reinforcement_{i}",
        )

    # Circular scale plate and center shaft assembly.
    body.visual(
        Cylinder(radius=0.064, length=0.005),
        origin=Origin(xyz=(0.0, 0.049, 0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="scale_bezel",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.006),
        origin=Origin(xyz=(0.0, 0.052, 0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="shaft_boss",
    )
    body.visual(
        Cylinder(radius=0.0048, length=0.040),
        origin=Origin(xyz=(0.0, 0.066, 0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="center_shaft",
    )
    body.visual(
        Cylinder(radius=0.0095, length=0.006),
        origin=Origin(xyz=(0.0, 0.084, 0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="front_retainer",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.002),
        origin=Origin(xyz=(0.0, 0.088, 0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="retainer_screw",
    )

    # Old thermostat temperature ticks on the fixed scale.
    for i, angle_deg in enumerate(range(-120, 121, 20)):
        angle = math.radians(angle_deg)
        radius = 0.055
        x = math.sin(angle) * radius
        z = 0.125 + math.cos(angle) * radius
        tick_len = 0.013 if angle_deg % 40 == 0 else 0.008
        body.visual(
            Box((0.0023, 0.0014, tick_len)),
            origin=Origin(xyz=(x, 0.052, z), rpy=(0.0, angle, 0.0)),
            material=dark_metal,
            name=f"scale_tick_{i}",
        )
    body.visual(
        Box((0.018, 0.0016, 0.006)),
        origin=Origin(xyz=(-0.044, 0.052, 0.168), rpy=(0.0, -0.65, 0.0)),
        material=blue,
        name="cool_mark",
    )
    body.visual(
        Box((0.018, 0.0016, 0.006)),
        origin=Origin(xyz=(0.044, 0.052, 0.168), rpy=(0.0, 0.65, 0.0)),
        material=red,
        name="warm_mark",
    )

    # Pragmatic fasteners: adapter bolts and cover screws are seated in the
    # plates rather than floating as decorative dots.
    for i, (x, z) in enumerate(
        ((-0.072, 0.035), (0.072, 0.035), (-0.072, 0.215), (0.072, 0.215))
    ):
        body.visual(
            Cylinder(radius=0.0045, length=0.003),
            origin=Origin(xyz=(x, 0.026, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"adapter_bolt_{i}",
        )
    for i, (x, z) in enumerate(
        ((-0.053, 0.052), (0.053, 0.052), (-0.053, 0.198), (0.053, 0.198))
    ):
        body.visual(
            Cylinder(radius=0.0036, length=0.007),
            origin=Origin(xyz=(x, 0.050, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"cover_screw_{i}",
        )

    # Fixed hinge supports for the two service hatches.  Each has side knuckles
    # and a small leaf that lands on the reinforced front cover.
    body.visual(
        Cylinder(radius=0.0037, length=0.014),
        origin=Origin(xyz=(-0.033, 0.051, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hatch_knuckle_0_0",
    )
    body.visual(
        Box((0.018, 0.006, 0.009)),
        origin=Origin(xyz=(-0.033, 0.047, 0.036)),
        material=brass,
        name="hatch_leaf_0_0",
    )
    body.visual(
        Cylinder(radius=0.0037, length=0.014),
        origin=Origin(xyz=(0.033, 0.051, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hatch_knuckle_0_1",
    )
    body.visual(
        Box((0.018, 0.006, 0.009)),
        origin=Origin(xyz=(0.033, 0.047, 0.036)),
        material=brass,
        name="hatch_leaf_0_1",
    )
    body.visual(
        Cylinder(radius=0.0037, length=0.014),
        origin=Origin(xyz=(-0.033, 0.051, 0.214), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hatch_knuckle_1_0",
    )
    body.visual(
        Box((0.018, 0.006, 0.009)),
        origin=Origin(xyz=(-0.033, 0.047, 0.214)),
        material=brass,
        name="hatch_leaf_1_0",
    )
    body.visual(
        Cylinder(radius=0.0037, length=0.014),
        origin=Origin(xyz=(0.033, 0.051, 0.214), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hatch_knuckle_1_1",
    )
    body.visual(
        Box((0.018, 0.006, 0.009)),
        origin=Origin(xyz=(0.033, 0.047, 0.214)),
        material=brass,
        name="hatch_leaf_1_1",
    )

    # A small bottom row of legacy screw terminals sits under the lower hatch.
    for i, x in enumerate((-0.032, -0.016, 0.0, 0.016, 0.032)):
        body.visual(
            Box((0.010, 0.006, 0.006)),
            origin=Origin(xyz=(x, 0.049, 0.020)),
            material=dark_metal,
            name=f"terminal_block_{i}",
        )
        body.visual(
            Cylinder(radius=0.0022, length=0.002),
            origin=Origin(xyz=(x, 0.053, 0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"terminal_screw_{i}",
        )

    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.094,
            0.026,
            body_style="skirted",
            top_diameter=0.070,
            base_diameter=0.098,
            edge_radius=0.0015,
            skirt=KnobSkirt(0.102, 0.0045, flare=0.05, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=36, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="engraved", angle_deg=0.0, depth=0.0008),
            bore=KnobBore(style="round", diameter=0.014),
            center=False,
        ),
        "thermostat_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="dial_cap",
    )
    dial.visual(
        Box((0.006, 0.002, 0.038)),
        origin=Origin(xyz=(0.0, 0.0265, 0.026)),
        material=dark_metal,
        name="raised_pointer",
    )
    dial.visual(
        Cylinder(radius=0.011, length=0.003),
        origin=Origin(xyz=(0.0, 0.0275, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="dial_hub_washer",
    )

    lower_hatch = model.part("lower_hatch")
    lower_hatch.visual(
        Box((0.084, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, -0.004, 0.020)),
        material=bakelite,
        name="hatch_panel",
    )
    lower_hatch.visual(
        Cylinder(radius=0.0035, length=0.044),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hatch_barrel",
    )
    lower_hatch.visual(
        Box((0.046, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.002, 0.004)),
        material=brass,
        name="hatch_leaf",
    )
    lower_hatch.visual(
        Box((0.030, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.001, 0.030)),
        material=rubber,
        name="finger_pull",
    )

    upper_hatch = model.part("upper_hatch")
    upper_hatch.visual(
        Box((0.084, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, -0.004, -0.020)),
        material=bakelite,
        name="hatch_panel",
    )
    upper_hatch.visual(
        Cylinder(radius=0.0035, length=0.044),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hatch_barrel",
    )
    upper_hatch.visual(
        Box((0.046, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.002, -0.004)),
        material=brass,
        name="hatch_leaf",
    )
    upper_hatch.visual(
        Box((0.030, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.001, -0.030)),
        material=rubber,
        name="finger_pull",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.053, 0.125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.0, lower=-2.1, upper=2.1),
    )
    model.articulation(
        "body_to_lower_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_hatch,
        origin=Origin(xyz=(0.0, 0.051, 0.036)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=1.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_upper_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_hatch,
        origin=Origin(xyz=(0.0, 0.051, 0.214)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=1.5, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    lower_hatch = object_model.get_part("lower_hatch")
    upper_hatch = object_model.get_part("upper_hatch")
    dial_joint = object_model.get_articulation("body_to_dial")
    lower_joint = object_model.get_articulation("body_to_lower_hatch")
    upper_joint = object_model.get_articulation("body_to_upper_hatch")

    ctx.expect_within(
        dial,
        body,
        axes="xz",
        inner_elem="dial_hub_washer",
        outer_elem="scale_bezel",
        margin=0.003,
        name="dial hub remains centered on fixed bezel",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="y",
        elem_a="dial_hub_washer",
        elem_b="center_shaft",
        min_overlap=0.002,
        name="dial washer is retained by center shaft line",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="y",
        positive_elem="dial_cap",
        negative_elem="scale_bezel",
        min_gap=0.001,
        name="dial clears fixed scale bezel",
    )
    ctx.expect_gap(
        lower_hatch,
        body,
        axis="x",
        positive_elem="hatch_barrel",
        negative_elem="hatch_knuckle_0_0",
        min_gap=0.002,
        max_gap=0.006,
        name="lower hatch barrel clears left knuckle",
    )
    ctx.expect_gap(
        body,
        lower_hatch,
        axis="x",
        positive_elem="hatch_knuckle_0_1",
        negative_elem="hatch_barrel",
        min_gap=0.002,
        max_gap=0.006,
        name="lower hatch barrel clears right knuckle",
    )

    closed_lower = ctx.part_element_world_aabb(lower_hatch, elem="hatch_panel")
    with ctx.pose({lower_joint: 1.0}):
        opened_lower = ctx.part_element_world_aabb(lower_hatch, elem="hatch_panel")
    closed_lower_y = None if closed_lower is None else (closed_lower[0][1] + closed_lower[1][1]) / 2.0
    opened_lower_y = None if opened_lower is None else (opened_lower[0][1] + opened_lower[1][1]) / 2.0
    ctx.check(
        "lower hatch opens outward",
        closed_lower_y is not None and opened_lower_y is not None and opened_lower_y > closed_lower_y + 0.005,
        details=f"closed_y={closed_lower_y}, opened_y={opened_lower_y}",
    )

    ctx.expect_gap(
        upper_hatch,
        body,
        axis="x",
        positive_elem="hatch_barrel",
        negative_elem="hatch_knuckle_1_0",
        min_gap=0.002,
        max_gap=0.006,
        name="upper hatch barrel clears left knuckle",
    )
    ctx.expect_gap(
        body,
        upper_hatch,
        axis="x",
        positive_elem="hatch_knuckle_1_1",
        negative_elem="hatch_barrel",
        min_gap=0.002,
        max_gap=0.006,
        name="upper hatch barrel clears right knuckle",
    )

    closed_upper = ctx.part_element_world_aabb(upper_hatch, elem="hatch_panel")
    with ctx.pose({upper_joint: 1.0}):
        opened_upper = ctx.part_element_world_aabb(upper_hatch, elem="hatch_panel")
    closed_upper_y = None if closed_upper is None else (closed_upper[0][1] + closed_upper[1][1]) / 2.0
    opened_upper_y = None if opened_upper is None else (opened_upper[0][1] + opened_upper[1][1]) / 2.0
    ctx.check(
        "upper hatch opens outward",
        closed_upper_y is not None and opened_upper_y is not None and opened_upper_y > closed_upper_y + 0.005,
        details=f"closed_y={closed_upper_y}, opened_y={opened_upper_y}",
    )

    with ctx.pose({dial_joint: 1.0}):
        ctx.expect_within(
            dial,
            body,
            axes="xz",
            inner_elem="dial_hub_washer",
            outer_elem="scale_bezel",
            margin=0.003,
            name="rotated dial stays concentric with shaft",
        )

    return ctx.report()


object_model = build_object_model()
