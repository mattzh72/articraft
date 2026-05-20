from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    sx, sy, sz = size
    body = cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)
    return mesh_from_cadquery(body, name, tolerance=0.0012, angular_tolerance=0.12)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prep_processor_on_counter_scale")

    warm_white = model.material("warm_white", rgba=(0.86, 0.83, 0.76, 1.0))
    black = model.material("black_soft_touch", rgba=(0.025, 0.025, 0.028, 1.0))
    dark = model.material("dark_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.73, 0.70, 1.0))
    clear = model.material("clear_safety_plastic", rgba=(0.68, 0.88, 1.0, 0.33))
    clear_smoke = model.material("smoked_clear_plastic", rgba=(0.40, 0.57, 0.68, 0.38))
    grey = model.material("medium_grey", rgba=(0.36, 0.37, 0.36, 1.0))
    green = model.material("green_indicator", rgba=(0.0, 0.55, 0.18, 1.0))
    red = model.material("red_pointer", rgba=(0.88, 0.10, 0.06, 1.0))
    display_glass = model.material("display_glass", rgba=(0.04, 0.10, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        _rounded_box_mesh((0.42, 0.36, 0.12), 0.032, "rounded_scale_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=warm_white,
        name="rounded_scale_base",
    )
    base.visual(
        _rounded_box_mesh((0.355, 0.295, 0.008), 0.022, "stainless_scale_deck"),
        origin=Origin(xyz=(0.0, 0.012, 0.124)),
        material=stainless,
        name="scale_deck",
    )
    base_ring = LatheGeometry.from_shell_profiles(
        [(0.082, 0.126), (0.126, 0.126), (0.126, 0.150), (0.118, 0.154)],
        [(0.072, 0.130), (0.106, 0.132), (0.108, 0.146), (0.100, 0.150)],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=6,
    )
    base.visual(
        mesh_from_geometry(base_ring, "bayonet_socket_ring"),
        material=dark,
        name="bayonet_socket_ring",
    )
    for i in range(3):
        a = i * 2.0 * math.pi / 3.0 + 0.30
        base.visual(
            Box((0.034, 0.012, 0.006)),
            origin=Origin(
                xyz=(0.112 * math.cos(a), 0.112 * math.sin(a), 0.154),
                rpy=(0.0, 0.0, a),
            ),
            material=stainless,
            name=f"twist_slot_{i}",
        )

    base.visual(
        Box((0.270, 0.010, 0.072)),
        origin=Origin(xyz=(-0.025, -0.184, 0.066)),
        material=black,
        name="front_control_panel",
    )
    base.visual(
        Box((0.075, 0.003, 0.026)),
        origin=Origin(xyz=(0.077, -0.190, 0.077)),
        material=display_glass,
        name="scale_display",
    )
    for i, x in enumerate((-0.127, -0.113, -0.099, -0.085, -0.071)):
        base.visual(
            Box((0.005, 0.003, 0.012)),
            origin=Origin(xyz=(x, -0.184, 0.100), rpy=(0.0, 0.0, -0.35 + 0.17 * i)),
            material=stainless,
            name=f"timer_tick_{i}",
        )
    base.visual(
        Box((0.006, 0.066, 0.072)),
        origin=Origin(xyz=(0.213, -0.055, 0.074)),
        material=black,
        name="rocker_recess",
    )
    for i, x in enumerate((-0.155, 0.155)):
        for j, y in enumerate((-0.125, 0.125)):
            base.visual(
                Cylinder(radius=0.018, length=0.010),
                origin=Origin(xyz=(x, y, 0.005)),
                material=grey,
                name=f"rubber_foot_{i}_{j}",
            )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.070, 0.004),
            (0.112, 0.035),
            (0.136, 0.215),
            (0.137, 0.270),
            (0.146, 0.286),
        ],
        [
            (0.050, 0.015),
            (0.094, 0.046),
            (0.118, 0.220),
            (0.124, 0.269),
            (0.131, 0.282),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "clear_bowl_shell"),
        material=clear,
        name="clear_bowl_shell",
    )
    lid_rim = LatheGeometry.from_shell_profiles(
        [(0.090, 0.276), (0.148, 0.277), (0.149, 0.294), (0.139, 0.300)],
        [(0.070, 0.282), (0.106, 0.282), (0.111, 0.293), (0.101, 0.296)],
        segments=80,
        start_cap="flat",
        end_cap="round",
        lip_samples=6,
    )
    bowl.visual(mesh_from_geometry(lid_rim, "open_lid_rim"), material=clear_smoke, name="open_lid_rim")

    tube_x, tube_y = -0.045, 0.046
    tube_z = 0.286 + 0.115
    bowl.visual(
        Box((0.010, 0.106, 0.230)),
        origin=Origin(xyz=(tube_x - 0.033, tube_y, tube_z)),
        material=clear,
        name="tube_x_wall_0",
    )
    bowl.visual(
        Box((0.010, 0.106, 0.230)),
        origin=Origin(xyz=(tube_x + 0.033, tube_y, tube_z)),
        material=clear,
        name="tube_x_wall_1",
    )
    bowl.visual(
        Box((0.056, 0.010, 0.230)),
        origin=Origin(xyz=(tube_x, tube_y - 0.048, tube_z)),
        material=clear,
        name="tube_y_wall_0",
    )
    bowl.visual(
        Box((0.056, 0.010, 0.230)),
        origin=Origin(xyz=(tube_x, tube_y + 0.048, tube_z)),
        material=clear,
        name="tube_y_wall_1",
    )
    bowl.visual(
        Cylinder(radius=0.026, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=clear_smoke,
        name="center_bearing",
    )
    for i in range(3):
        a = i * 2.0 * math.pi / 3.0
        bowl.visual(
            Box((0.055, 0.010, 0.010)),
            origin=Origin(
                xyz=(0.045 * math.cos(a), 0.045 * math.sin(a), 0.030),
                rpy=(0.0, 0.0, a),
            ),
            material=clear_smoke,
            name=f"bearing_spoke_{i}",
        )
    for z in (0.085, 0.225):
        bowl.visual(
            Box((0.060, 0.018, 0.018)),
            origin=Origin(xyz=(0.147, 0.0, z)),
            material=clear,
            name=f"handle_boss_{int(z * 1000)}",
        )
    bowl.visual(
        Box((0.026, 0.030, 0.160)),
        origin=Origin(xyz=(0.176, 0.0, 0.155)),
        material=clear,
        name="bowl_handle",
    )
    for i in range(3):
        a = i * 2.0 * math.pi / 3.0 - 0.12
        bowl.visual(
            Box((0.044, 0.014, 0.010)),
            origin=Origin(
                xyz=(0.104 * math.cos(a), 0.104 * math.sin(a), 0.009),
                rpy=(0.0, 0.0, a),
            ),
            material=clear_smoke,
            name=f"bowl_lock_lug_{i}",
        )

    basket = model.part("grating_basket")
    basket_wall = LatheGeometry.from_shell_profiles(
        [(0.074, 0.000), (0.094, 0.018), (0.097, 0.110), (0.090, 0.126)],
        [(0.061, 0.006), (0.076, 0.022), (0.083, 0.105), (0.080, 0.120)],
        segments=80,
        start_cap="round",
        end_cap="round",
        lip_samples=5,
    )
    basket.visual(mesh_from_geometry(basket_wall, "perforated_basket_wall"), material=stainless, name="basket_wall")
    basket.visual(
        Cylinder(radius=0.079, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=stainless,
        name="grating_disk",
    )
    basket.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=grey,
        name="drive_hub",
    )
    for i in range(18):
        a = i * 2.0 * math.pi / 18.0
        basket.visual(
            Box((0.004, 0.006, 0.070)),
            origin=Origin(
                xyz=(0.093 * math.cos(a), 0.093 * math.sin(a), 0.067),
                rpy=(0.0, 0.0, a),
            ),
            material=dark,
            name=f"basket_slot_shadow_{i}",
        )
    for i in range(12):
        a = i * 2.0 * math.pi / 12.0 + 0.18
        r = 0.040 + 0.020 * (i % 2)
        basket.visual(
            Box((0.034, 0.003, 0.005)),
            origin=Origin(
                xyz=(r * math.cos(a), r * math.sin(a), 0.010),
                rpy=(0.0, 0.0, a + 0.55),
            ),
            material=dark,
            name=f"grater_tooth_{i}",
        )

    pusher = model.part("pusher")
    pusher.visual(
        _rounded_box_mesh((0.046, 0.076, 0.230), 0.010, "pusher_body_mesh"),
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        material=grey,
        name="pusher_body",
    )
    pusher.visual(
        Box((0.008, 0.012, 0.190)),
        origin=Origin(xyz=(-0.024, 0.0, -0.115)),
        material=grey,
        name="pusher_side_guide_0",
    )
    pusher.visual(
        Box((0.008, 0.012, 0.190)),
        origin=Origin(xyz=(0.024, 0.0, -0.115)),
        material=grey,
        name="pusher_side_guide_1",
    )
    pusher.visual(
        Box((0.018, 0.010, 0.190)),
        origin=Origin(xyz=(0.0, -0.038, -0.115)),
        material=grey,
        name="pusher_front_guide",
    )
    pusher.visual(
        Box((0.018, 0.010, 0.190)),
        origin=Origin(xyz=(0.0, 0.038, -0.115)),
        material=grey,
        name="pusher_rear_guide",
    )
    pusher.visual(
        _rounded_box_mesh((0.092, 0.122, 0.026), 0.014, "pusher_cap_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark,
        name="pusher_cap",
    )

    timer_dial = model.part("timer_dial")
    timer_knob = KnobGeometry(
        0.062,
        0.030,
        body_style="skirted",
        top_diameter=0.047,
        skirt=KnobSkirt(0.070, 0.006, flare=0.06, chamfer=0.0015),
        grip=KnobGrip(style="fluted", count=20, depth=0.0015),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        center=True,
    )
    timer_dial.visual(
        mesh_from_geometry(timer_knob, "timer_dial_knob"),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grey,
        name="timer_knob",
    )
    timer_dial.visual(
        Box((0.006, 0.003, 0.026)),
        origin=Origin(xyz=(0.0, -0.031, 0.012)),
        material=red,
        name="timer_pointer",
    )

    rocker = model.part("power_rocker")
    rocker.visual(
        Box((0.014, 0.048, 0.056)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=black,
        name="rocker_paddle",
    )
    rocker.visual(
        Box((0.003, 0.030, 0.006)),
        origin=Origin(xyz=(0.015, 0.0, 0.014)),
        material=green,
        name="power_mark",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2),
    )
    model.articulation(
        "bowl_to_basket",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=25.0),
    )
    model.articulation(
        "bowl_to_pusher",
        ArticulationType.PRISMATIC,
        parent=bowl,
        child=pusher,
        origin=Origin(xyz=(tube_x, tube_y, 0.533)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.095),
    )
    model.articulation(
        "base_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=timer_dial,
        origin=Origin(xyz=(-0.102, -0.195, 0.066)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )
    model.articulation(
        "base_to_rocker",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(0.216, -0.055, 0.074)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.22, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bowl = object_model.get_part("bowl")
    basket = object_model.get_part("grating_basket")
    pusher = object_model.get_part("pusher")
    timer = object_model.get_part("timer_dial")
    rocker = object_model.get_part("power_rocker")
    base = object_model.get_part("base")

    bowl_joint = object_model.get_articulation("base_to_bowl")
    basket_joint = object_model.get_articulation("bowl_to_basket")
    pusher_joint = object_model.get_articulation("bowl_to_pusher")
    timer_joint = object_model.get_articulation("base_to_timer_dial")
    rocker_joint = object_model.get_articulation("base_to_rocker")

    ctx.check(
        "requested articulation types",
        bowl_joint.articulation_type == ArticulationType.CONTINUOUS
        and basket_joint.articulation_type == ArticulationType.CONTINUOUS
        and timer_joint.articulation_type == ArticulationType.CONTINUOUS
        and pusher_joint.articulation_type == ArticulationType.PRISMATIC
        and rocker_joint.articulation_type == ArticulationType.REVOLUTE,
        details="processor requires continuous bowl, basket and dial; prismatic pusher; hinged rocker",
    )
    ctx.expect_within(
        basket,
        bowl,
        axes="xy",
        inner_elem="basket_wall",
        outer_elem="clear_bowl_shell",
        margin=0.002,
        name="basket sits inside clear bowl footprint",
    )
    ctx.expect_gap(
        pusher,
        basket,
        axis="z",
        min_gap=0.060,
        positive_elem="pusher_body",
        negative_elem="basket_wall",
        name="feed tube remains open above basket at rest",
    )
    ctx.expect_gap(
        pusher,
        bowl,
        axis="x",
        min_gap=0.002,
        positive_elem="pusher_body",
        negative_elem="tube_x_wall_0",
        name="pusher clears one side of hollow tube",
    )
    ctx.expect_gap(
        bowl,
        pusher,
        axis="x",
        min_gap=0.002,
        positive_elem="tube_x_wall_1",
        negative_elem="pusher_body",
        name="pusher clears opposite side of hollow tube",
    )
    ctx.expect_gap(
        pusher,
        bowl,
        axis="y",
        min_gap=0.002,
        positive_elem="pusher_body",
        negative_elem="tube_y_wall_0",
        name="pusher clears front wall of hollow tube",
    )
    ctx.expect_gap(
        bowl,
        pusher,
        axis="y",
        min_gap=0.002,
        positive_elem="tube_y_wall_1",
        negative_elem="pusher_body",
        name="pusher clears rear wall of hollow tube",
    )
    ctx.expect_overlap(
        pusher,
        bowl,
        axes="z",
        elem_a="pusher_body",
        elem_b="tube_x_wall_0",
        min_overlap=0.180,
        name="pusher remains inserted through feed tube",
    )
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.010,
        max_penetration=0.006,
        positive_elem="clear_bowl_shell",
        negative_elem="bayonet_socket_ring",
        name="bowl is seated over twist lock coupling",
    )
    ctx.expect_gap(
        base,
        timer,
        axis="y",
        min_gap=0.0,
        max_gap=0.012,
        positive_elem="front_control_panel",
        negative_elem="timer_knob",
        name="timer dial is mounted on front panel",
    )
    rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.095, rocker_joint: 0.18, timer_joint: 1.2, basket_joint: 2.4}):
        lowered = ctx.part_world_position(pusher)
        ctx.expect_gap(
            pusher,
            bowl,
            axis="x",
            min_gap=0.002,
            positive_elem="pusher_body",
            negative_elem="tube_x_wall_0",
            name="lowered pusher clears tube side",
        )
    ctx.check(
        "pusher slides downward",
        rest is not None and lowered is not None and lowered[2] < rest[2] - 0.080,
        details=f"rest={rest}, lowered={lowered}",
    )

    return ctx.report()


object_model = build_object_model()
