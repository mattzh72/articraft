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
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PLINTH_W = 0.48
PLINTH_D = 0.37
PLINTH_H = 0.060
TOP_Z = 0.063
PLATTER_CENTER = (-0.075, 0.025)
PLATTER_RADIUS = 0.148
TONEARM_PIVOT = (0.150, 0.105, 0.105)


def _rounded_plinth_shell() -> object:
    """Low, softly rounded plinth with a refined consumer-electronics edge."""
    return (
        cq.Workplane("XY")
        .box(PLINTH_W, PLINTH_D, PLINTH_H)
        .edges("|Z")
        .fillet(0.020)
        .edges(">Z")
        .fillet(0.004)
        .edges("<Z")
        .fillet(0.003)
        .translate((0.0, 0.0, PLINTH_H / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refined_hifi_turntable")

    piano_black = model.material("piano_black", rgba=(0.004, 0.005, 0.006, 1.0))
    satin_black = model.material("satin_black", rgba=(0.018, 0.019, 0.020, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.002, 0.002, 0.002, 1.0))
    vinyl_black = model.material("vinyl_black", rgba=(0.006, 0.006, 0.007, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.080, 0.083, 0.086, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    chrome = model.material("chrome", rgba=(0.88, 0.90, 0.92, 1.0))
    label_paper = model.material("warm_record_label", rgba=(0.86, 0.74, 0.42, 1.0))
    groove_shadow = model.material("groove_shadow", rgba=(0.0, 0.0, 0.0, 1.0))
    white_mark = model.material("white_mark", rgba=(0.92, 0.92, 0.88, 1.0))
    red_lens = model.material("red_lens", rgba=(0.95, 0.05, 0.02, 1.0))
    blue_lens = model.material("blue_lens", rgba=(0.04, 0.18, 0.75, 1.0))
    cartridge_red = model.material("cartridge_red", rgba=(0.45, 0.02, 0.015, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        mesh_from_cadquery(_rounded_plinth_shell(), "rounded_plinth_shell", tolerance=0.0008),
        material=piano_black,
        name="plinth_shell",
    )
    plinth.visual(
        Box((0.430, 0.315, 0.003)),
        origin=Origin(xyz=(-0.010, 0.010, 0.0615)),
        material=satin_black,
        name="top_panel",
    )
    # Four broad compliant feet make the low plinth read as a real deck rather than a slab.
    for idx, (x, y) in enumerate(
        (
            (-0.195, -0.140),
            (0.195, -0.140),
            (-0.195, 0.140),
            (0.195, 0.140),
        )
    ):
        plinth.visual(
            Cylinder(radius=0.025, length=0.012),
            origin=Origin(xyz=(x, y, -0.0058)),
            material=rubber_black,
            name=f"foot_{idx}",
        )

    # Static hi-fi details: front badges/lenses, rear dust-cover hinge blocks, and tonearm bearing.
    plinth.visual(
        Box((0.070, 0.002, 0.010)),
        origin=Origin(xyz=(-0.145, -0.186, 0.036)),
        material=brushed_aluminum,
        name="front_badge",
    )
    plinth.visual(
        Box((0.010, 0.002, 0.006)),
        origin=Origin(xyz=(0.123, -0.186, 0.038)),
        material=red_lens,
        name="power_lens",
    )
    plinth.visual(
        Box((0.010, 0.002, 0.006)),
        origin=Origin(xyz=(0.141, -0.186, 0.038)),
        material=blue_lens,
        name="speed_lens",
    )
    for idx, x in enumerate((-0.145, 0.145)):
        plinth.visual(
            Box((0.050, 0.018, 0.012)),
            origin=Origin(xyz=(x, 0.176, 0.063)),
            material=dark_metal,
            name=f"hinge_block_{idx}",
        )

    pivot_x, pivot_y, pivot_z = TONEARM_PIVOT
    plinth.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(pivot_x, pivot_y, TOP_Z + 0.003)),
        material=brushed_aluminum,
        name="bearing_flange",
    )
    plinth.visual(
        Cylinder(radius=0.021, length=pivot_z - TOP_Z),
        origin=Origin(xyz=(pivot_x, pivot_y, (pivot_z + TOP_Z) / 2.0)),
        material=dark_metal,
        name="bearing_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(xyz=(0.095, -0.063, TOP_Z + 0.020)),
        material=dark_metal,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.032, 0.012, 0.006)),
        origin=Origin(xyz=(0.095, -0.063, TOP_Z + 0.042)),
        material=brushed_aluminum,
        name="arm_rest_clip",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=PLATTER_RADIUS, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=brushed_aluminum,
        name="main_platter",
    )
    platter.visual(
        Cylinder(radius=0.127, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0305)),
        material=rubber_black,
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.146, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0340)),
        material=vinyl_black,
        name="vinyl_record",
    )
    platter.visual(
        Cylinder(radius=0.035, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.03625)),
        material=label_paper,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0430)),
        material=chrome,
        name="center_spindle",
    )
    # Subtle groove rings in the vinyl surface.
    for idx, radius in enumerate((0.055, 0.078, 0.101, 0.123, 0.138)):
        platter.visual(
            mesh_from_geometry(TorusGeometry(radius=radius, tube=0.00032), f"record_groove_{idx}"),
            origin=Origin(xyz=(0.0, 0.0, 0.03565)),
            material=groove_shadow,
            name=f"record_groove_{idx}",
        )
    # Small bright strobe marks around the rotating platter rim.
    for idx in range(32):
        angle = 2.0 * math.pi * idx / 32.0
        r = PLATTER_RADIUS + 0.001
        platter.visual(
            Box((0.0045, 0.0013, 0.004)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.017),
                rpy=(0.0, 0.0, angle),
            ),
            material=chrome,
            name=f"strobe_mark_{idx}",
        )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brushed_aluminum,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.0040, length=0.185),
        origin=Origin(xyz=(0.104, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.040, 0.022, 0.008)),
        origin=Origin(xyz=(0.202, 0.0, 0.010)),
        material=dark_metal,
        name="headshell",
    )
    tonearm.visual(
        Box((0.021, 0.016, 0.007)),
        origin=Origin(xyz=(0.210, 0.0, 0.0045)),
        material=cartridge_red,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0007, length=0.009),
        origin=Origin(xyz=(0.213, 0.0, 0.0002)),
        material=chrome,
        name="stylus_tip",
    )
    tonearm.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(-0.026, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="counterweight",
    )
    tonearm.visual(
        Cylinder(radius=0.006, length=0.035),
        origin=Origin(xyz=(-0.047, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="weight_stub",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.016,
                body_style="faceted",
                base_diameter=0.041,
                top_diameter=0.030,
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=18, depth=0.0007),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_dial_knob",
        ),
        material=dark_metal,
        name="dial_body",
    )
    speed_dial.visual(
        Box((0.014, 0.002, 0.0012)),
        origin=Origin(xyz=(0.009, 0.0, 0.0162)),
        material=white_mark,
        name="dial_marker",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_black,
        name="button_cap",
    )
    power_button.visual(
        Cylinder(radius=0.009, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0084)),
        material=red_lens,
        name="button_inlay",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(PLATTER_CENTER[0], PLATTER_CENTER[1], TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=33.33),
        motion_properties=MotionProperties(damping=0.02, friction=0.002),
    )
    model.articulation(
        "tonearm_sweep",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=TONEARM_PIVOT, rpy=(0.0, 0.0, -1.88)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=0.8, lower=0.0, upper=0.82),
        motion_properties=MotionProperties(damping=0.08, friction=0.01),
    )
    model.articulation(
        "speed_select",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=speed_dial,
        origin=Origin(xyz=(0.170, -0.095, TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=1.2, lower=-0.55, upper=0.55),
        motion_properties=MotionProperties(damping=0.04, friction=0.02),
    )
    model.articulation(
        "power_press",
        ArticulationType.PRISMATIC,
        parent=plinth,
        child=power_button,
        origin=Origin(xyz=(0.118, -0.095, TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=-0.004, upper=0.0),
        motion_properties=MotionProperties(damping=0.2, friction=0.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    speed_dial = object_model.get_part("speed_dial")
    power_button = object_model.get_part("power_button")

    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_sweep = object_model.get_articulation("tonearm_sweep")

    ctx.check(
        "platter uses continuous rotation",
        platter_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={platter_spin.articulation_type}",
    )
    ctx.check(
        "tonearm sweep has realistic limited travel",
        tonearm_sweep.motion_limits is not None
        and tonearm_sweep.motion_limits.lower == 0.0
        and 0.70 <= tonearm_sweep.motion_limits.upper <= 0.95,
        details=f"limits={tonearm_sweep.motion_limits}",
    )

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="main_platter",
        negative_elem="top_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="platter sits on the top deck",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="main_platter",
        elem_b="top_panel",
        min_overlap=0.25,
        name="platter is centered over the deck panel",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="pivot_collar",
        negative_elem="bearing_pedestal",
        max_gap=0.001,
        max_penetration=0.0,
        name="tonearm collar rests on bearing",
    )
    ctx.expect_overlap(
        tonearm,
        plinth,
        axes="xy",
        elem_a="pivot_collar",
        elem_b="bearing_pedestal",
        min_overlap=0.025,
        name="tonearm pivot is concentric with bearing",
    )
    ctx.expect_gap(
        speed_dial,
        plinth,
        axis="z",
        positive_elem="dial_body",
        negative_elem="top_panel",
        max_gap=0.002,
        max_penetration=0.0,
        name="speed dial is surface mounted",
    )
    ctx.expect_gap(
        power_button,
        plinth,
        axis="z",
        positive_elem="button_cap",
        negative_elem="top_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="power button is surface mounted",
    )

    def elem_center(part, elem_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    def elem_min_z(part, elem_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lo, _ = aabb
        return lo[2]

    rest_tip = elem_center(tonearm, "stylus_tip")
    rest_dist = (
        math.hypot(rest_tip[0] - PLATTER_CENTER[0], rest_tip[1] - PLATTER_CENTER[1])
        if rest_tip is not None
        else None
    )
    with ctx.pose({tonearm_sweep: tonearm_sweep.motion_limits.upper}):
        play_tip = elem_center(tonearm, "stylus_tip")
        play_tip_min_z = elem_min_z(tonearm, "stylus_tip")

    play_dist = (
        math.hypot(play_tip[0] - PLATTER_CENTER[0], play_tip[1] - PLATTER_CENTER[1])
        if play_tip is not None
        else None
    )
    ctx.check(
        "tonearm sweeps from rest toward record",
        rest_dist is not None
        and play_dist is not None
        and rest_dist > PLATTER_RADIUS + 0.020
        and 0.025 < play_dist < 0.085
        and play_dist < rest_dist - 0.09,
        details=f"rest_dist={rest_dist}, play_dist={play_dist}",
    )
    ctx.check(
        "stylus clears the record surface",
        play_tip_min_z is not None and play_tip_min_z > TOP_Z + 0.036,
        details=f"stylus_min_z={play_tip_min_z}",
    )

    return ctx.report()


object_model = build_object_model()
