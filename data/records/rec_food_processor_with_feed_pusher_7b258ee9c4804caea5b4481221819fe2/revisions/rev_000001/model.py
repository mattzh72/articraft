from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
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


BOWL_TOP_Z = 0.310
LID_THICKNESS = 0.014
FEED_TUBE_CENTER = (0.0, 0.055)
FEED_TUBE_HEIGHT = 0.180
FEED_TUBE_BASE_Z = 0.014


def _circle_profile(radius: float, *, segments: int = 96, offset=(0.0, 0.0)) -> list[tuple[float, float]]:
    ox, oy = offset
    return [
        (
            ox + radius * math.cos(2.0 * math.pi * index / segments),
            oy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _make_base_mesh():
    # A softly radiused stainless housing reads as a heavy die-cast motor base,
    # while the separate black control fascia and metal bowl socket stay simple.
    shape = cq.Workplane("XY").box(0.360, 0.300, 0.220)
    shape = shape.edges("|Z").fillet(0.032)
    shape = shape.edges(">Z").fillet(0.010)
    return mesh_from_cadquery(shape, "motor_base_body", tolerance=0.002, angular_tolerance=0.15)


def _make_bowl_shell_mesh():
    outer = [
        (0.070, 0.000),
        (0.118, 0.030),
        (0.140, 0.155),
        (0.148, 0.292),
        (0.154, 0.310),
    ]
    inner = [
        (0.046, 0.016),
        (0.096, 0.042),
        (0.126, 0.158),
        (0.137, 0.286),
        (0.142, 0.302),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=96,
            start_cap="round",
            end_cap="round",
            lip_samples=10,
        ),
        "transparent_work_bowl_shell",
    )


def _make_lid_plate_mesh():
    tube_hole = _translate_profile(
        rounded_rect_profile(0.062, 0.042, 0.010, corner_segments=8),
        FEED_TUBE_CENTER[0],
        FEED_TUBE_CENTER[1],
    )
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.158, segments=128),
            [tube_hole],
            LID_THICKNESS,
            center=False,
            cap=True,
        ),
        "clear_lid_with_feed_opening",
    )


def _make_outer_rim_mesh():
    return mesh_from_geometry(
        LatheGeometry(
            [(0.140, 0.000), (0.160, 0.000), (0.160, 0.010), (0.140, 0.010)],
            segments=96,
            closed=True,
        ),
        "clear_lid_outer_rim",
    )


def _make_feed_tube_mesh():
    outer = rounded_rect_profile(0.084, 0.064, 0.014, corner_segments=8)
    inner = rounded_rect_profile(0.058, 0.038, 0.009, corner_segments=8)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            FEED_TUBE_HEIGHT,
            center=False,
            cap=True,
        ),
        "hollow_feed_tube",
    )


def _make_pusher_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.048, 0.030, 0.007, corner_segments=8),
            0.195,
            cap=True,
        ).rotate_x(math.pi),
        "feed_pusher_body",
    )


def _make_cutter_basket_mesh():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.038, 0.030), (0.090, 0.055), (0.105, 0.120), (0.110, 0.135)],
            [(0.026, 0.042), (0.074, 0.066), (0.091, 0.116), (0.096, 0.128)],
            segments=72,
            start_cap="round",
            end_cap="round",
            lip_samples=6,
        ),
        "perforated_cutter_basket_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_food_processor")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.35, 0.36, 0.36, 1.0))
    clear_poly = model.material("clear_polycarbonate", rgba=(0.72, 0.88, 1.00, 0.34))
    black = model.material("black_control_panel", rgba=(0.015, 0.017, 0.018, 1.0))
    rubber = model.material("black_rubber", rgba=(0.045, 0.045, 0.045, 1.0))
    white_mark = model.material("white_markings", rgba=(0.94, 0.94, 0.90, 1.0))
    button_green = model.material("start_green", rgba=(0.06, 0.42, 0.16, 1.0))
    safety_gray = model.material("latch_gray", rgba=(0.21, 0.23, 0.25, 1.0))
    blade_steel = model.material("sharp_blade_steel", rgba=(0.86, 0.88, 0.86, 1.0))

    base = model.part("motor_base")
    base.visual(_make_base_mesh(), material=stainless, name="rounded_body")
    base.visual(
        Box((0.168, 0.008, 0.088)),
        origin=Origin(xyz=(0.000, -0.154, 0.105)),
        material=black,
        name="front_fascia",
    )
    base.visual(
        Cylinder(radius=0.086, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.113)),
        material=dark_stainless,
        name="bowl_socket",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.000, 0.000, 0.125)),
        material=stainless,
        name="drive_coupler",
    )
    base.visual(
        Box((0.320, 0.260, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, -0.110)),
        material=rubber,
        name="rubber_foot_pad",
    )
    base.inertial = Inertial.from_geometry(Box((0.360, 0.300, 0.220)), mass=5.8)

    bowl = model.part("work_bowl")
    bowl.visual(_make_bowl_shell_mesh(), origin=Origin(), material=clear_poly, name="bowl_shell")
    for index, side in enumerate((-1.0, 1.0)):
        bowl.visual(
            Box((0.070, 0.076, 0.040)),
            origin=Origin(xyz=(side * 0.174, 0.000, 0.225)),
            material=safety_gray,
            name=f"side_socket_{index}",
        )
    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.000, 0.000, 0.117)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.302, 0.188, LID_THICKNESS)),
        origin=Origin(xyz=(0.000, -0.059, LID_THICKNESS * 0.5)),
        material=clear_poly,
        name="lid_front_panel",
    )
    lid.visual(
        Box((0.302, 0.078, LID_THICKNESS)),
        origin=Origin(xyz=(0.000, 0.114, LID_THICKNESS * 0.5)),
        material=clear_poly,
        name="lid_rear_panel",
    )
    lid.visual(
        Box((0.122, 0.046, LID_THICKNESS)),
        origin=Origin(xyz=(-0.093, 0.055, LID_THICKNESS * 0.5)),
        material=clear_poly,
        name="lid_side_panel_0",
    )
    lid.visual(
        Box((0.122, 0.046, LID_THICKNESS)),
        origin=Origin(xyz=(0.093, 0.055, LID_THICKNESS * 0.5)),
        material=clear_poly,
        name="lid_side_panel_1",
    )
    lid.visual(
        _make_outer_rim_mesh(),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        material=clear_poly,
        name="raised_outer_rim",
    )
    lid.visual(
        Box((0.084, 0.010, FEED_TUBE_HEIGHT)),
        origin=Origin(xyz=(FEED_TUBE_CENTER[0], FEED_TUBE_CENTER[1] - 0.027, FEED_TUBE_BASE_Z + FEED_TUBE_HEIGHT * 0.5)),
        material=clear_poly,
        name="feed_tube_front_wall",
    )
    lid.visual(
        Box((0.084, 0.010, FEED_TUBE_HEIGHT)),
        origin=Origin(xyz=(FEED_TUBE_CENTER[0], FEED_TUBE_CENTER[1] + 0.027, FEED_TUBE_BASE_Z + FEED_TUBE_HEIGHT * 0.5)),
        material=clear_poly,
        name="feed_tube_rear_wall",
    )
    lid.visual(
        Box((0.010, 0.064, FEED_TUBE_HEIGHT)),
        origin=Origin(xyz=(FEED_TUBE_CENTER[0] - 0.037, FEED_TUBE_CENTER[1], FEED_TUBE_BASE_Z + FEED_TUBE_HEIGHT * 0.5)),
        material=clear_poly,
        name="feed_tube_side_wall_0",
    )
    lid.visual(
        Box((0.010, 0.064, FEED_TUBE_HEIGHT)),
        origin=Origin(xyz=(FEED_TUBE_CENTER[0] + 0.037, FEED_TUBE_CENTER[1], FEED_TUBE_BASE_Z + FEED_TUBE_HEIGHT * 0.5)),
        material=clear_poly,
        name="feed_tube_side_wall_1",
    )
    lid.visual(
        Box((0.120, 0.026, 0.012)),
        origin=Origin(xyz=(0.000, -0.116, 0.020)),
        material=safety_gray,
        name="front_latch_bar",
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.FLOATING,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.000, 0.000, BOWL_TOP_Z)),
    )

    pusher = model.part("pusher")
    pusher.visual(_make_pusher_mesh(), origin=Origin(), material=safety_gray, name="sliding_plunger")
    pusher.visual(
        Box((0.096, 0.074, 0.024)),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=safety_gray,
        name="wide_pusher_cap",
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(
            xyz=(
                FEED_TUBE_CENTER[0],
                FEED_TUBE_CENTER[1],
                FEED_TUBE_BASE_Z + FEED_TUBE_HEIGHT,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.18, lower=0.0, upper=0.120),
    )

    cutter = model.part("cutter_basket")
    cutter.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=Origin(xyz=(0.000, 0.000, 0.080)),
        material=dark_stainless,
        name="drive_shaft",
    )
    cutter.visual(_make_cutter_basket_mesh(), origin=Origin(), material=blade_steel, name="basket_shell")
    cutter.visual(
        Box((0.120, 0.020, 0.006)),
        origin=Origin(xyz=(0.030, 0.000, 0.074), rpy=(0.0, 0.0, 0.10)),
        material=blade_steel,
        name="swept_blade_0",
    )
    cutter.visual(
        Box((0.120, 0.020, 0.006)),
        origin=Origin(xyz=(-0.030, 0.000, 0.083), rpy=(0.0, 0.0, math.pi + 0.10)),
        material=blade_steel,
        name="swept_blade_1",
    )
    model.articulation(
        "base_to_cutter",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cutter,
        origin=Origin(xyz=(0.000, 0.000, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    for index, side in enumerate((-1.0, 1.0)):
        arm = model.part(f"side_clamp_{index}")
        inward = -side
        arm.visual(
            Cylinder(radius=0.015, length=0.066),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=safety_gray,
            name="pivot_barrel",
        )
        arm.visual(
            Box((0.020, 0.030, 0.124)),
            origin=Origin(xyz=(0.000, 0.000, 0.064)),
            material=safety_gray,
            name="upright_arm",
        )
        arm.visual(
            Box((0.074, 0.030, 0.020)),
            origin=Origin(xyz=(inward * 0.040, 0.000, 0.121)),
            material=safety_gray,
            name="lid_hook",
        )
        arm.visual(
            Box((0.028, 0.034, 0.020)),
            origin=Origin(xyz=(inward * 0.004, 0.000, -0.020)),
            material=safety_gray,
            name="lower_thumb_tab",
        )
        model.articulation(
            f"bowl_to_side_clamp_{index}",
            ArticulationType.REVOLUTE,
            parent=bowl,
            child=arm,
            origin=Origin(xyz=(side * 0.174, 0.000, 0.225)),
            axis=(0.0, side, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.10),
        )

    dial = model.part("timer_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.064,
                0.030,
                body_style="skirted",
                top_diameter=0.048,
                edge_radius=0.0015,
                skirt=KnobSkirt(0.074, 0.006, flare=0.06, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=24, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "front_timer_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="knurled_dial",
    )
    dial.visual(
        Box((0.004, 0.006, 0.020)),
        origin=Origin(xyz=(0.000, -0.031, 0.020)),
        material=white_mark,
        name="pointer_mark",
    )
    model.articulation(
        "base_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(-0.048, -0.158, 0.110)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    start = model.part("start_button")
    start.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.000, -0.007, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_green,
        name="button_cap",
    )
    model.articulation(
        "base_to_start_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=start,
        origin=Origin(xyz=(0.056, -0.158, 0.086)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.05, lower=0.0, upper=0.008),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("motor_base")
    bowl = object_model.get_part("work_bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    cutter = object_model.get_part("cutter_basket")
    clamp_0 = object_model.get_part("side_clamp_0")
    clamp_1 = object_model.get_part("side_clamp_1")
    dial = object_model.get_part("timer_dial")
    button = object_model.get_part("start_button")

    pusher_slide = object_model.get_articulation("lid_to_pusher")
    lid_release = object_model.get_articulation("bowl_to_lid")
    cutter_spin = object_model.get_articulation("base_to_cutter")
    dial_spin = object_model.get_articulation("base_to_timer_dial")
    button_push = object_model.get_articulation("base_to_start_button")
    clamp_joint_0 = object_model.get_articulation("bowl_to_side_clamp_0")
    clamp_joint_1 = object_model.get_articulation("bowl_to_side_clamp_1")

    for index in (0, 1):
        ctx.allow_overlap(
            f"side_clamp_{index}",
            "work_bowl",
            elem_a="pivot_barrel",
            elem_b=f"side_socket_{index}",
            reason="The clamp pivot barrel is intentionally captured inside the bowl-shoulder socket.",
        )
        ctx.allow_overlap(
            f"side_clamp_{index}",
            "work_bowl",
            elem_a="upright_arm",
            elem_b=f"side_socket_{index}",
            reason="The lower root of the clamp arm is locally embedded in the pivot shoulder boss.",
        )
        ctx.allow_overlap(
            f"side_clamp_{index}",
            "work_bowl",
            elem_a="lower_thumb_tab",
            elem_b=f"side_socket_{index}",
            reason="The thumb-tab root shares the same compact pivot shoulder boss as the clamp arm.",
        )
        ctx.expect_within(
            f"side_clamp_{index}",
            "work_bowl",
            axes="yz",
            inner_elem="pivot_barrel",
            outer_elem=f"side_socket_{index}",
            margin=0.002,
            name=f"clamp_{index}_pivot_captured_in_socket",
        )
        ctx.expect_overlap(
            f"side_clamp_{index}",
            "work_bowl",
            axes="yz",
            elem_a="pivot_barrel",
            elem_b=f"side_socket_{index}",
            min_overlap=0.025,
            name=f"clamp_{index}_pivot_has_bearing_engagement",
        )
        ctx.expect_overlap(
            f"side_clamp_{index}",
            "work_bowl",
            axes="yz",
            elem_a="upright_arm",
            elem_b=f"side_socket_{index}",
            min_overlap=0.015,
            name=f"clamp_{index}_arm_root_enters_shoulder_boss",
        )
        ctx.expect_overlap(
            f"side_clamp_{index}",
            "work_bowl",
            axes="yz",
            elem_a="lower_thumb_tab",
            elem_b=f"side_socket_{index}",
            min_overlap=0.009,
            name=f"clamp_{index}_thumb_tab_root_enters_boss",
        )

    ctx.check(
        "lid_is_separate_removable_part",
        lid_release.articulation_type == ArticulationType.FLOATING,
        details=f"type={lid_release.articulation_type}",
    )
    ctx.check(
        "cutter_rotates_continuously_on_vertical_axis",
        cutter_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(cutter_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={cutter_spin.articulation_type} axis={cutter_spin.axis}",
    )
    ctx.check(
        "timer_dial_is_continuous_front_control",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(dial_spin.axis) == (0.0, -1.0, 0.0),
        details=f"type={dial_spin.articulation_type} axis={dial_spin.axis}",
    )
    ctx.check(
        "pusher_has_downward_prismatic_travel",
        pusher_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(pusher_slide.axis) == (0.0, 0.0, -1.0)
        and pusher_slide.motion_limits is not None
        and pusher_slide.motion_limits.upper is not None
        and pusher_slide.motion_limits.upper >= 0.10,
        details=f"type={pusher_slide.articulation_type} axis={pusher_slide.axis} limits={pusher_slide.motion_limits}",
    )
    ctx.check(
        "start_button_has_short_prismatic_press",
        button_push.articulation_type == ArticulationType.PRISMATIC
        and tuple(button_push.axis) == (0.0, 1.0, 0.0)
        and button_push.motion_limits is not None
        and 0.006 <= (button_push.motion_limits.upper or 0.0) <= 0.010,
        details=f"type={button_push.articulation_type} axis={button_push.axis} limits={button_push.motion_limits}",
    )

    tube_outer_x = 0.084
    tube_outer_y = 0.064
    plunger_box = ctx.part_element_world_aabb(pusher, elem="sliding_plunger")
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "pusher_body_fits_inside_feed_tube_opening",
        plunger_box is not None
        and lid_pos is not None
        and (plunger_box[1][0] - plunger_box[0][0]) < tube_outer_x - 0.020
        and (plunger_box[1][1] - plunger_box[0][1]) < tube_outer_y - 0.020,
        details=f"plunger_aabb={plunger_box}",
    )

    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.100}):
        pressed_pusher = ctx.part_world_position(pusher)
    ctx.check(
        "pusher_slides_down_through_feed_tube",
        rest_pusher is not None and pressed_pusher is not None and pressed_pusher[2] < rest_pusher[2] - 0.080,
        details=f"rest={rest_pusher}, pressed={pressed_pusher}",
    )

    rest_button = ctx.part_world_position(button)
    with ctx.pose({button_push: 0.008}):
        pressed_button = ctx.part_world_position(button)
    ctx.check(
        "start_button_moves_inward",
        rest_button is not None and pressed_button is not None and pressed_button[1] > rest_button[1] + 0.006,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    def _x_center(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0]) if aabb is not None else None

    hook0_rest = ctx.part_element_world_aabb(clamp_0, elem="lid_hook")
    hook1_rest = ctx.part_element_world_aabb(clamp_1, elem="lid_hook")
    with ctx.pose({clamp_joint_0: 1.0, clamp_joint_1: 1.0}):
        hook0_open = ctx.part_element_world_aabb(clamp_0, elem="lid_hook")
        hook1_open = ctx.part_element_world_aabb(clamp_1, elem="lid_hook")
    h0r, h0o = _x_center(hook0_rest), _x_center(hook0_open)
    h1r, h1o = _x_center(hook1_rest), _x_center(hook1_open)
    ctx.check(
        "clamp_0_swings_outward_from_lid",
        h0r is not None and h0o is not None and h0o < h0r - 0.035,
        details=f"rest={h0r}, open={h0o}",
    )
    ctx.check(
        "clamp_1_swings_outward_from_lid",
        h1r is not None and h1o is not None and h1o > h1r + 0.035,
        details=f"rest={h1r}, open={h1o}",
    )

    ctx.expect_gap(
        lid,
        bowl,
        axis="z",
        max_gap=0.002,
        max_penetration=0.004,
        name="removable_lid_sits_on_bowl_rim",
    )
    ctx.expect_overlap(bowl, cutter, axes="xy", min_overlap=0.060, name="cutter_basket_centered_in_open_bowl")
    ctx.expect_gap(cutter, base, axis="z", max_gap=0.002, max_penetration=0.0001, name="cutter_shaft_seats_on_drive_coupler")

    return ctx.report()


object_model = build_object_model()
