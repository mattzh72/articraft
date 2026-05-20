from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _offset_profile(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(diameter: float, *, segments: int = 64) -> list[tuple[float, float]]:
    radius = diameter * 0.5
    return [
        (radius * math.cos(math.tau * i / segments), radius * math.sin(math.tau * i / segments))
        for i in range(segments)
    ]


def _rounded_z_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_food_processor")

    brushed_silver = model.material("brushed_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    deep_black = model.material("deep_black", rgba=(0.015, 0.016, 0.018, 1.0))
    warm_clear = model.material("warm_clear", rgba=(0.72, 0.92, 1.0, 0.34))
    smoky_clear = model.material("smoky_clear", rgba=(0.46, 0.62, 0.70, 0.42))
    dark_plastic = model.material("dark_plastic", rgba=(0.08, 0.085, 0.095, 1.0))
    button_plastic = model.material("button_plastic", rgba=(0.15, 0.17, 0.19, 1.0))
    button_blue = model.material("button_blue", rgba=(0.05, 0.22, 0.42, 1.0))
    white_mark = model.material("white_mark", rgba=(0.90, 0.94, 0.96, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.86, 0.88, 0.90, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.025, 0.025, 0.027, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(rounded_rect_profile(0.405, 0.305, 0.070, corner_segments=14), 0.052),
            "base_foot",
        ),
        material=deep_black,
        name="base_foot",
    )
    base.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _rounded_z_section(0.365, 0.270, 0.060, 0.030),
                    _rounded_z_section(0.350, 0.250, 0.075, 0.105),
                    _rounded_z_section(0.300, 0.220, 0.070, 0.178),
                    _rounded_z_section(0.240, 0.180, 0.055, 0.197),
                ]
            ),
            "rounded_base_body",
        ),
        material=brushed_silver,
        name="rounded_body",
    )
    base.visual(
        Box((0.018, 0.232, 0.118)),
        origin=Origin(xyz=(-0.187, 0.0, 0.104)),
        material=deep_black,
        name="control_panel",
    )
    base.visual(
        Cylinder(radius=0.098, length=0.012),
        origin=Origin(xyz=(0.035, 0.0, 0.201)),
        material=gasket_black,
        name="bowl_seat",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.050, 0.000),
                    (0.094, 0.012),
                    (0.114, 0.070),
                    (0.115, 0.166),
                    (0.122, 0.196),
                    (0.125, 0.205),
                ],
                [
                    (0.000, 0.014),
                    (0.080, 0.024),
                    (0.102, 0.074),
                    (0.104, 0.180),
                    (0.110, 0.196),
                ],
                segments=72,
                start_cap="flat",
                end_cap="round",
                lip_samples=8,
            ),
            "clear_bowl_shell",
        ),
        material=warm_clear,
        name="bowl_shell",
    )
    bowl.visual(
        Box((0.050, 0.062, 0.024)),
        origin=Origin(xyz=(0.0, 0.126, 0.150)),
        material=warm_clear,
        name="upper_handle_mount",
    )
    bowl.visual(
        Box((0.050, 0.062, 0.024)),
        origin=Origin(xyz=(0.0, 0.126, 0.072)),
        material=warm_clear,
        name="lower_handle_mount",
    )
    bowl.visual(
        Cylinder(radius=0.014, length=0.116),
        origin=Origin(xyz=(0.0, 0.162, 0.111)),
        material=warm_clear,
        name="side_handle_grip",
    )
    bowl.visual(
        Cylinder(radius=0.006, length=0.135),
        origin=Origin(xyz=(0.0, 0.0, 0.0735)),
        material=dark_plastic,
        name="vertical_shaft",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.035, 0.0, 0.205)),
    )

    lid = model.part("lid")
    lid_outer = _circle_profile(0.252, segments=88)
    lid_holes = [
        _offset_profile(rounded_rect_profile(0.052, 0.072, 0.014, corner_segments=8), -0.035, 0.0),
        _offset_profile(_circle_profile(0.034, segments=48), 0.035, 0.030),
    ]
    lid.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(lid_outer, lid_holes, 0.018, center=False),
            "clear_lid_cover",
        ),
        material=smoky_clear,
        name="lid_cover",
    )
    lid.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.068, 0.088, 0.018, corner_segments=8),
                [rounded_rect_profile(0.052, 0.072, 0.014, corner_segments=8)],
                0.145,
                center=False,
            ),
            "large_feed_chute",
        ),
        origin=Origin(xyz=(-0.035, 0.0, 0.018)),
        material=smoky_clear,
        name="large_feed_chute",
    )
    lid.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.023, 0.000), (0.023, 0.125)],
                [(0.016, 0.000), (0.016, 0.125)],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
            "small_ingredient_tube",
        ),
        origin=Origin(xyz=(0.035, 0.030, 0.018)),
        material=smoky_clear,
        name="small_ingredient_tube",
    )
    lid.visual(
        Box((0.022, 0.032, 0.070)),
        origin=Origin(xyz=(0.004, 0.018, 0.084)),
        material=smoky_clear,
        name="tube_bridge",
    )
    lid.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.127, 0.000), (0.127, 0.008)],
                [(0.116, 0.000), (0.116, 0.008)],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
            "lid_gasket",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=gasket_black,
        name="lid_gasket",
    )

    model.articulation(
        "bowl_to_lid",
        ArticulationType.FIXED,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
    )

    main_pusher = model.part("main_pusher")
    main_pusher.visual(
        Box((0.036, 0.057, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, -0.0525)),
        material=button_plastic,
        name="main_pusher_insert",
    )
    main_pusher.visual(
        Box((0.042, 0.063, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=button_blue,
        name="main_pusher_pad",
    )
    main_pusher.visual(
        Box((0.030, 0.045, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0305)),
        material=button_blue,
        name="main_pusher_grip",
    )
    model.articulation(
        "lid_to_main_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=main_pusher,
        origin=Origin(xyz=(-0.035, 0.0, 0.163)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.20, lower=0.0, upper=0.035),
    )

    small_pusher = model.part("small_pusher")
    small_pusher.visual(
        Cylinder(radius=0.0164, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, -0.0475)),
        material=button_plastic,
        name="small_pusher_insert",
    )
    small_pusher.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=button_blue,
        name="small_pusher_pad",
    )
    model.articulation(
        "lid_to_small_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=small_pusher,
        origin=Origin(xyz=(0.035, 0.030, 0.143)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.18, lower=0.0, upper=0.030),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.019, 0.000), (0.019, 0.058)],
                [(0.0055, 0.000), (0.0055, 0.058)],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
            "blade_hub",
        ),
        material=dark_plastic,
        name="blade_hub",
    )
    blade_carrier.visual(
        Box((0.092, 0.022, 0.004)),
        origin=Origin(xyz=(0.052, 0.006, 0.034), rpy=(0.0, 0.0, math.radians(13.0))),
        material=blade_metal,
        name="cutting_blade_0",
    )
    blade_carrier.visual(
        Box((0.092, 0.022, 0.004)),
        origin=Origin(xyz=(-0.052, -0.006, 0.042), rpy=(0.0, 0.0, math.radians(193.0))),
        material=blade_metal,
        name="cutting_blade_1",
    )
    model.articulation(
        "bowl_to_blade_carrier",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=blade_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=80.0),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.072,
                0.030,
                body_style="skirted",
                top_diameter=0.058,
                edge_radius=0.002,
                grip=KnobGrip(style="fluted", count=28, depth=0.0015),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "selector_dial",
        ),
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="selector_dial_cap",
    )
    model.articulation(
        "base_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=selector_dial,
        origin=Origin(xyz=(-0.196, 0.0, 0.128)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    button_positions = [(-0.069, 0.073), (-0.023, 0.073), (0.023, 0.073), (0.069, 0.073)]
    for idx, (y_pos, z_pos) in enumerate(button_positions):
        button = model.part(f"program_button_{idx}")
        button.visual(
            Box((0.014, 0.034, 0.018)),
            origin=Origin(xyz=(-0.007, 0.0, 0.0)),
            material=button_plastic,
            name="button_cap",
        )
        button.visual(
            Box((0.0025, 0.020, 0.002)),
            origin=Origin(xyz=(-0.0145, 0.0, 0.006)),
            material=white_mark,
            name="button_mark",
        )
        model.articulation(
            f"base_to_program_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(-0.196, y_pos, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.06, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.allow_overlap(
        "lid",
        "main_pusher",
        elem_a="large_feed_chute",
        elem_b="main_pusher_insert",
        reason=(
            "The main pusher is intentionally represented as a retained sliding member "
            "inside the clear feed-chute sleeve."
        ),
    )
    ctx.allow_overlap(
        "bowl",
        "blade_carrier",
        elem_a="vertical_shaft",
        elem_b="blade_hub",
        reason=(
            "The blade hub is intentionally captured on the vertical drive shaft with a "
            "tiny proxy interference so the rotating carrier stays mechanically seated."
        ),
    )
    ctx.allow_overlap(
        "lid",
        "small_pusher",
        elem_a="small_ingredient_tube",
        elem_b="small_pusher_insert",
        reason=(
            "The secondary pusher is intentionally captured by a slight sliding fit "
            "inside its small ingredient tube."
        ),
    )
    main_slide = object_model.get_articulation("lid_to_main_pusher")
    small_slide = object_model.get_articulation("lid_to_small_pusher")
    blade_spin = object_model.get_articulation("bowl_to_blade_carrier")
    dial_spin = object_model.get_articulation("base_to_selector_dial")

    ctx.check(
        "main pusher has prismatic travel",
        main_slide.articulation_type == ArticulationType.PRISMATIC,
        details=str(main_slide.articulation_type),
    )
    ctx.check(
        "small pusher has prismatic travel",
        small_slide.articulation_type == ArticulationType.PRISMATIC,
        details=str(small_slide.articulation_type),
    )
    ctx.check(
        "blade carrier spins continuously",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=str(blade_spin.articulation_type),
    )
    ctx.check(
        "selector dial spins continuously",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=str(dial_spin.articulation_type),
    )
    for idx in range(4):
        button_joint = object_model.get_articulation(f"base_to_program_button_{idx}")
        ctx.check(
            f"program button {idx} is an independent prismatic button",
            button_joint.articulation_type == ArticulationType.PRISMATIC,
            details=str(button_joint.articulation_type),
        )

    ctx.expect_within(
        "main_pusher",
        "lid",
        axes="xy",
        inner_elem="main_pusher_insert",
        outer_elem="large_feed_chute",
        margin=0.001,
        name="main pusher stays inside large chute footprint",
    )
    ctx.expect_within(
        "small_pusher",
        "lid",
        axes="xy",
        inner_elem="small_pusher_insert",
        outer_elem="small_ingredient_tube",
        margin=0.001,
        name="small pusher stays inside ingredient tube footprint",
    )
    ctx.expect_overlap(
        "main_pusher",
        "lid",
        axes="z",
        elem_a="main_pusher_insert",
        elem_b="large_feed_chute",
        min_overlap=0.095,
        name="main pusher remains engaged in large chute",
    )
    ctx.expect_overlap(
        "small_pusher",
        "lid",
        axes="z",
        elem_a="small_pusher_insert",
        elem_b="small_ingredient_tube",
        min_overlap=0.080,
        name="small pusher remains engaged in ingredient tube",
    )
    ctx.expect_within(
        "bowl",
        "blade_carrier",
        axes="xy",
        inner_elem="vertical_shaft",
        outer_elem="blade_hub",
        margin=0.001,
        name="drive shaft stays centered inside blade hub",
    )
    ctx.expect_overlap(
        "bowl",
        "blade_carrier",
        axes="z",
        elem_a="vertical_shaft",
        elem_b="blade_hub",
        min_overlap=0.045,
        name="blade hub stays seated on drive shaft",
    )

    rest_main = ctx.part_world_position("main_pusher")
    rest_small = ctx.part_world_position("small_pusher")
    with ctx.pose({main_slide: 0.035, small_slide: 0.030}):
        ctx.expect_within(
            "main_pusher",
            "lid",
            axes="xy",
            inner_elem="main_pusher_insert",
            outer_elem="large_feed_chute",
            margin=0.001,
            name="pressed main pusher stays centered in chute",
        )
        ctx.expect_within(
            "small_pusher",
            "lid",
            axes="xy",
            inner_elem="small_pusher_insert",
            outer_elem="small_ingredient_tube",
            margin=0.001,
            name="pressed small pusher stays centered in tube",
        )
        pressed_main = ctx.part_world_position("main_pusher")
        pressed_small = ctx.part_world_position("small_pusher")
    ctx.check(
        "main pusher slides downward",
        rest_main is not None and pressed_main is not None and pressed_main[2] < rest_main[2] - 0.025,
        details=f"rest={rest_main}, pressed={pressed_main}",
    )
    ctx.check(
        "small pusher slides downward",
        rest_small is not None and pressed_small is not None and pressed_small[2] < rest_small[2] - 0.020,
        details=f"rest={rest_small}, pressed={pressed_small}",
    )

    button_rest = ctx.part_world_position("program_button_0")
    button_joint = object_model.get_articulation("base_to_program_button_0")
    with ctx.pose({button_joint: 0.006}):
        button_pressed = ctx.part_world_position("program_button_0")
    ctx.check(
        "program buttons press inward toward panel",
        button_rest is not None and button_pressed is not None and button_pressed[0] > button_rest[0] + 0.004,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
