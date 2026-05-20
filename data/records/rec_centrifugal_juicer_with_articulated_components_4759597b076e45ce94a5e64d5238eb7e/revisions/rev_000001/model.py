from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)


BASE_H = 0.44
HINGE_Y = 0.150
HINGE_Z = 0.560
LID_CENTER_Y = -0.150
CHUTE_X = 0.085
CHUTE_Y = -0.125
CHUTE_BASE_Z = 0.105
CHUTE_H = 0.300


def _profile_loop(width: float, depth: float, z: float, segments: int = 48):
    return [(x, y, z) for x, y in superellipse_profile(width, depth, 3.1, segments=segments)]


def _lathe_shell(outer_profile, inner_profile, name: str):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_household_juicer")

    matte_white = Material("warm_matte_white", (0.90, 0.88, 0.82, 1.0))
    satin_grey = Material("satin_graphite", (0.16, 0.17, 0.18, 1.0))
    dark_panel = Material("gloss_black_panel", (0.02, 0.025, 0.03, 1.0))
    clear_smoke = Material("clear_smoked_polycarbonate", (0.63, 0.82, 0.95, 0.34))
    clear_blue = Material("clear_blue_lid", (0.60, 0.86, 1.0, 0.38))
    steel = Material("brushed_stainless_steel", (0.72, 0.74, 0.72, 1.0))
    plunger_white = Material("white_food_safe_plastic", (0.96, 0.96, 0.92, 1.0))
    green = Material("green_power_button", (0.05, 0.55, 0.22, 1.0))
    amber = Material("amber_pulse_button", (0.95, 0.55, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _profile_loop(0.350, 0.290, 0.000),
                    _profile_loop(0.325, 0.275, 0.110),
                    _profile_loop(0.285, 0.245, 0.320),
                    _profile_loop(0.245, 0.215, BASE_H),
                ]
            ),
            "tapered_base_body",
        ),
        material=matte_white,
        name="tapered_body",
    )
    body.visual(
        Box((0.175, 0.012, 0.185)),
        origin=Origin(xyz=(0.0, -0.132, 0.240), rpy=(0.0, 0.0, 0.0)),
        material=dark_panel,
        name="front_panel",
    )
    body.visual(
        Cylinder(0.018, 0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=satin_grey,
        name="drive_coupler",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(0.118, 0.006, radial_segments=16, tubular_segments=64), "clear_bowl_lower_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        material=clear_smoke,
        name="bowl_lower_ring",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(0.118, 0.006, radial_segments=16, tubular_segments=64), "clear_bowl_upper_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.558)),
        material=clear_smoke,
        name="bowl_upper_ring",
    )
    for idx, (x, y) in enumerate(((0.125, 0.0), (-0.125, 0.0), (0.0, 0.125), (0.0, -0.125))):
        body.visual(
            Box((0.012, 0.012, 0.118)),
            origin=Origin(xyz=(x, y, 0.499)),
            material=clear_smoke,
            name=f"bowl_post_{idx}",
        )
    body.visual(
        Cylinder(0.020, 0.055),
        origin=Origin(xyz=(-0.085, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_grey,
        name="hinge_barrel_0",
    )
    body.visual(
        Cylinder(0.020, 0.055),
        origin=Origin(xyz=(0.085, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_grey,
        name="hinge_barrel_1",
    )
    body.visual(
        Box((0.052, 0.030, 0.130)),
        origin=Origin(xyz=(-0.085, HINGE_Y - 0.035, 0.500)),
        material=satin_grey,
        name="hinge_bracket_0",
    )
    body.visual(
        Box((0.052, 0.030, 0.130)),
        origin=Origin(xyz=(0.085, HINGE_Y - 0.035, 0.500)),
        material=satin_grey,
        name="hinge_bracket_1",
    )
    body.visual(
        Box((0.026, 0.045, 0.048)),
        origin=Origin(xyz=(0.172, 0.115, 0.500)),
        material=satin_grey,
        name="latch_pivot_boss",
    )
    body.visual(
        Box((0.075, 0.070, 0.140)),
        origin=Origin(xyz=(0.135, 0.115, 0.475)),
        material=satin_grey,
        name="latch_mount_rib",
    )

    filter_part = model.part("filter")
    filter_part.visual(
        _lathe_shell(
            [(0.024, 0.010), (0.060, 0.025), (0.096, 0.065), (0.102, 0.080)],
            [(0.017, 0.016), (0.047, 0.027), (0.084, 0.059), (0.089, 0.070)],
            "conical_filter_basket",
        ),
        material=steel,
        name="basket_shell",
    )
    filter_part.visual(
        Cylinder(0.019, 0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_grey,
        name="hub",
    )

    lid = model.part("lid")
    lid.visual(
        _lathe_shell(
            [
                (0.140, 0.000),
                (0.137, 0.018),
                (0.128, 0.050),
                (0.090, 0.082),
                (0.035, 0.094),
            ],
            [
                (0.123, 0.008),
                (0.119, 0.026),
                (0.108, 0.048),
                (0.076, 0.070),
                (0.030, 0.079),
            ],
            "clear_upper_lid_shell",
        ),
        origin=Origin(xyz=(0.0, LID_CENTER_Y, 0.000)),
        material=clear_blue,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(0.018, 0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_blue,
        name="hinge_knuckle",
    )
    lid.visual(
        Box((0.072, 0.052, 0.014)),
        origin=Origin(xyz=(0.0, -0.026, 0.018)),
        material=clear_blue,
        name="hinge_tab",
    )
    lid.visual(
        _lathe_shell(
            [(0.050, 0.000), (0.052, 0.012), (0.050, 0.024)],
            [(0.030, 0.004), (0.030, 0.012), (0.030, 0.020)],
            "chute_base_collar",
        ),
        origin=Origin(xyz=(CHUTE_X, CHUTE_Y, CHUTE_BASE_Z - 0.020)),
        material=clear_blue,
        name="chute_collar",
    )
    lid.visual(
        _lathe_shell(
            [(0.036, 0.000), (0.037, CHUTE_H * 0.70), (0.034, CHUTE_H)],
            [(0.025, 0.006), (0.026, CHUTE_H * 0.70), (0.024, CHUTE_H - 0.006)],
            "offset_feed_chute",
        ),
        origin=Origin(xyz=(CHUTE_X, CHUTE_Y, CHUTE_BASE_Z)),
        material=clear_blue,
        name="feed_chute",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(0.018, 0.300),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=plunger_white,
        name="shaft",
    )
    pusher.visual(
        Cylinder(0.045, 0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=plunger_white,
        name="top_cap",
    )
    pusher.visual(
        Box((0.090, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=plunger_white,
        name="grip_bar",
    )

    button_specs = [
        ("top_button", -0.045, 0.296, green),
        ("middle_button", 0.000, 0.240, satin_grey),
        ("bottom_button", 0.045, 0.184, amber),
    ]
    for part_name, x, z, mat in button_specs:
        button = model.part(part_name)
        button.visual(
            Cylinder(0.014, 0.014),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(0.009, 0.006),
            origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_panel,
            name="plunger_stem",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.139, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(0.017, 0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_grey,
        name="pivot_pin",
    )
    latch.visual(
        Box((0.022, 0.020, 0.160)),
        origin=Origin(xyz=(0.024, 0.0, 0.080)),
        material=satin_grey,
        name="upright_arm",
    )
    latch.visual(
        Box((0.100, 0.024, 0.018)),
        origin=Origin(xyz=(-0.025, 0.0, 0.164)),
        material=satin_grey,
        name="top_hook",
    )

    model.articulation(
        "body_to_filter",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=filter_part,
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=40.0),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(CHUTE_X, CHUTE_Y, CHUTE_BASE_Z + CHUTE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=0.130),
    )
    model.articulation(
        "body_to_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(0.174, 0.115, 0.500)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-1.10, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    filter_part = object_model.get_part("filter")
    latch = object_model.get_part("latch")

    filter_joint = object_model.get_articulation("body_to_filter")
    lid_joint = object_model.get_articulation("body_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_pusher")

    ctx.allow_overlap(
        body,
        filter_part,
        elem_a="drive_coupler",
        elem_b="hub",
        reason="The spinning filter hub is intentionally seated on the fixed drive coupler.",
    )
    ctx.expect_overlap(
        body,
        filter_part,
        axes="xy",
        elem_a="drive_coupler",
        elem_b="hub",
        min_overlap=0.025,
        name="filter hub is centered on drive coupler",
    )
    ctx.expect_overlap(
        body,
        filter_part,
        axes="z",
        elem_a="drive_coupler",
        elem_b="hub",
        min_overlap=0.008,
        name="filter hub remains seated vertically",
    )

    for button_name in ("top_button", "middle_button", "bottom_button"):
        button = object_model.get_part(button_name)
        ctx.allow_overlap(
            body,
            button,
            elem_a="front_panel",
            elem_b="plunger_stem",
            reason="Each button has a short plunger stem intentionally seated into the control panel.",
        )
        ctx.expect_overlap(
            body,
            button,
            axes="xz",
            elem_a="front_panel",
            elem_b="plunger_stem",
            min_overlap=0.010,
            name=f"{button_name} stem aligns with control panel opening",
        )
        ctx.expect_gap(
            body,
            button,
            axis="y",
            positive_elem="front_panel",
            negative_elem="plunger_stem",
            max_penetration=0.006,
            name=f"{button_name} stem is only shallowly seated",
        )

    ctx.allow_overlap(
        body,
        latch,
        elem_a="latch_pivot_boss",
        elem_b="pivot_pin",
        reason="The latch pivot pin is intentionally captured by the side boss.",
    )
    ctx.allow_overlap(
        body,
        latch,
        elem_a="latch_mount_rib",
        elem_b="pivot_pin",
        reason="The pivot pin passes through the reinforced side latch mount.",
    )
    ctx.expect_overlap(
        body,
        latch,
        axes="yz",
        elem_a="latch_pivot_boss",
        elem_b="pivot_pin",
        min_overlap=0.020,
        name="latch pin is captured by side pivot boss",
    )
    ctx.expect_overlap(
        body,
        latch,
        axes="yz",
        elem_a="latch_mount_rib",
        elem_b="pivot_pin",
        min_overlap=0.020,
        name="latch pin passes through reinforced mount",
    )

    ctx.check(
        "filter uses continuous vertical spin",
        filter_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(filter_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={filter_joint.articulation_type}, axis={filter_joint.axis}",
    )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="shaft",
        outer_elem="feed_chute",
        margin=0.001,
        name="pusher shaft is centered inside offset chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="shaft",
        elem_b="feed_chute",
        min_overlap=0.250,
        name="pusher remains inserted in chute at rest",
    )

    rest_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.130}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="shaft",
            outer_elem="feed_chute",
            margin=0.001,
            name="raised pusher stays guided by chute",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="shaft",
            elem_b="feed_chute",
            min_overlap=0.120,
            name="raised pusher retains insertion",
        )
        raised_pos = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: 0.90}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "rear hinge lifts the lid",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.045,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    ctx.expect_overlap(
        latch,
        lid,
        axes="xy",
        elem_a="top_hook",
        elem_b="lid_shell",
        min_overlap=0.015,
        name="latch hook lies over lid edge",
    )

    for joint_name in ("body_to_top_button", "body_to_middle_button", "body_to_bottom_button"):
        joint = object_model.get_articulation(joint_name)
        part = object_model.get_part(joint.child)
        start = ctx.part_world_position(part)
        with ctx.pose({joint: 0.006}):
            pressed = ctx.part_world_position(part)
        ctx.check(
            f"{joint.child} plunges inward",
            start is not None and pressed is not None and pressed[1] > start[1] + 0.004,
            details=f"start={start}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
