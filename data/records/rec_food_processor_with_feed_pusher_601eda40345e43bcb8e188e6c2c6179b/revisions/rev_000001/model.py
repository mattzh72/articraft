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
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_cylinder_mesh(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Thin cylindrical wall with open bore and annular top/bottom rims."""
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, height)],
        [(inner_radius, 0.0), (inner_radius, height)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _base_shell() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.28, 0.22, 0.105).translate((0.0, 0.0, 0.0525))
    return body.edges("|Z").fillet(0.018)


def _lid_shell(chute_x: float, chute_y: float) -> cq.Workplane:
    outer_radius = 0.112
    height = 0.015
    feed_inner_radius = 0.017
    lid = cq.Workplane("XY").circle(outer_radius).extrude(height)
    cutter = (
        cq.Workplane("XY")
        .center(chute_x, chute_y)
        .circle(feed_inner_radius)
        .extrude(height + 0.010)
        .translate((0.0, 0.0, -0.005))
    )
    return lid.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_food_processor")

    off_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = model.material("dark_graphite", rgba=(0.03, 0.035, 0.04, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    clear = model.material("clear_slightly_blue_plastic", rgba=(0.72, 0.91, 1.0, 0.38))
    smoked_clear = model.material("smoked_clear_lid", rgba=(0.62, 0.74, 0.82, 0.46))
    steel = model.material("brushed_stainless_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    button_blue = model.material("soft_blue_button", rgba=(0.18, 0.35, 0.72, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shell(), "rounded_motor_base", tolerance=0.001),
        material=off_white,
        name="rounded_motor_base",
    )
    base.visual(
        Box((0.145, 0.004, 0.064)),
        origin=Origin(xyz=(0.0, -0.112, 0.055)),
        material=dark,
        name="front_control_panel",
    )
    base.visual(
        Cylinder(radius=0.044, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=charcoal,
        name="bowl_socket_ring",
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.055, 0.000),
            (0.082, 0.018),
            (0.099, 0.116),
            (0.106, 0.142),
        ],
        [
            (0.028, 0.012),
            (0.074, 0.025),
            (0.092, 0.116),
            (0.098, 0.136),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "thin_wall_clear_bowl"),
        material=clear,
        name="thin_wall_bowl",
    )
    bowl.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=charcoal,
        name="central_spindle",
    )
    bowl.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=charcoal,
        name="spindle_boss",
    )

    lid = model.part("lid")
    chute_x = 0.046
    chute_y = -0.018
    lid.visual(
        mesh_from_cadquery(_lid_shell(chute_x, chute_y), "thin_lid_with_feed_hole"),
        material=smoked_clear,
        name="thin_lid",
    )
    lid.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.024, 0.017, 0.102), "hollow_feed_tube"),
        origin=Origin(xyz=(chute_x, chute_y, 0.015)),
        material=smoked_clear,
        name="hollow_feed_tube",
    )
    lid.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.030, 0.017, 0.008), "feed_tube_lip"),
        origin=Origin(xyz=(chute_x, chute_y, 0.112)),
        material=smoked_clear,
        name="feed_tube_lip",
    )

    pusher = model.part("feed_pusher")
    pusher.visual(
        Cylinder(radius=0.014, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, -0.0525)),
        material=charcoal,
        name="pusher_shaft",
    )
    pusher.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark,
        name="pusher_cap",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.021, 0.010, 0.018), "blade_hub_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="blade_hub_ring",
    )
    blade.visual(
        Box((0.060, 0.017, 0.004)),
        origin=Origin(xyz=(0.049, 0.0, 0.004), rpy=(0.0, 0.0, 0.12)),
        material=steel,
        name="upper_cutting_wing",
    )
    blade.visual(
        Box((0.060, 0.017, 0.004)),
        origin=Origin(xyz=(-0.049, 0.0, -0.004), rpy=(0.0, 0.0, 0.12)),
        material=steel,
        name="lower_cutting_wing",
    )

    timer_knob = model.part("timer_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.048,
            0.024,
            body_style="skirted",
            top_diameter=0.040,
            edge_radius=0.001,
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        ),
        "timer_knob_cap",
    )
    timer_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="knob_cap",
    )

    for index, x in enumerate((0.034, 0.076)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.028, 0.012, 0.019)),
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
            material=button_blue,
            name="button_cap",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, -0.1140, 0.054)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.FIXED,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
    )
    model.articulation(
        "lid_to_feed_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(chute_x, chute_y, 0.117)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.15, lower=0.0, upper=0.055),
    )
    model.articulation(
        "bowl_to_blade",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )
    model.articulation(
        "base_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=timer_knob,
        origin=Origin(xyz=(-0.050, -0.1140, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lid = object_model.get_part("lid")
    pusher = object_model.get_part("feed_pusher")
    bowl = object_model.get_part("bowl")
    blade = object_model.get_part("blade")
    knob = object_model.get_part("timer_knob")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    pusher_slide = object_model.get_articulation("lid_to_feed_pusher")
    button_slide_0 = object_model.get_articulation("base_to_button_0")
    button_slide_1 = object_model.get_articulation("base_to_button_1")
    blade_spin = object_model.get_articulation("bowl_to_blade")
    knob_spin = object_model.get_articulation("base_to_timer_knob")

    ctx.check(
        "primary controls are articulated",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and pusher_slide.articulation_type == ArticulationType.PRISMATIC
        and button_slide_0.articulation_type == ArticulationType.PRISMATIC
        and button_slide_1.articulation_type == ArticulationType.PRISMATIC,
        details="Expected continuous blade/knob, prismatic feed pusher and independent buttons.",
    )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="hollow_feed_tube",
        margin=0.000,
        name="feed pusher shaft stays inside hollow tube bore in plan",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_shaft",
        elem_b="hollow_feed_tube",
        min_overlap=0.080,
        name="feed pusher remains deeply inserted at rest",
    )
    with ctx.pose({pusher_slide: 0.055}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="hollow_feed_tube",
            margin=0.000,
            name="raised pusher remains guided by the tube",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="hollow_feed_tube",
            min_overlap=0.035,
            name="raised pusher retains insertion",
        )

    ctx.expect_within(
        blade,
        bowl,
        axes="xy",
        inner_elem="upper_cutting_wing",
        outer_elem="thin_wall_bowl",
        margin=0.002,
        name="blade fits inside clear bowl cavity footprint",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_slide_0: 0.006}):
        pressed_button_0 = ctx.part_world_position(button_0)
        same_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "first mode button presses independently inward",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and pressed_button_0[1] > rest_button_0[1] + 0.004
        and rest_button_1 is not None
        and same_button_1 is not None
        and abs(same_button_1[1] - rest_button_1[1]) < 0.0005,
        details=f"button0 rest={rest_button_0}, pressed={pressed_button_0}; button1 rest={rest_button_1}, posed={same_button_1}",
    )

    ctx.expect_gap(
        object_model.get_part("base"),
        knob,
        axis="y",
        max_penetration=0.000,
        positive_elem="front_control_panel",
        negative_elem="knob_cap",
        name="timer knob is seated proud of front panel without penetrating it",
    )

    return ctx.report()


object_model = build_object_model()
