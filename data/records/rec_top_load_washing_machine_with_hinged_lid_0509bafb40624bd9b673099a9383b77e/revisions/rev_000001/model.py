from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _shift_profile(profile, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _ring_band(*, outer_radius: float, inner_radius: float, z_center: float, height: float):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=64)
    inner = CylinderGeometry(radius=inner_radius, height=height + 0.004, radial_segments=64)
    from sdk import boolean_difference

    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_top_load_washer")

    white_enamel = model.material("white_enamel", rgba=(0.92, 0.94, 0.95, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.035, 0.040, 0.045, 1.0))
    dark_panel = model.material("charcoal_panel", rgba=(0.10, 0.12, 0.14, 1.0))
    soft_gray = model.material("soft_gray_plastic", rgba=(0.68, 0.72, 0.74, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.70, 0.73, 0.75, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.26, 1.0))
    blue_trim = model.material("button_blue_gray", rgba=(0.37, 0.48, 0.55, 1.0))

    # Narrow laundry-room proportions: roughly 56 cm wide, 62 cm deep, and just over 1 m
    # tall including the compact rear control pod.
    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.56, 0.035, 0.82)),
        origin=Origin(xyz=(0.0, -0.3025, 0.41)),
        material=white_enamel,
        name="front_panel",
    )
    cabinet.visual(
        Box((0.56, 0.035, 0.82)),
        origin=Origin(xyz=(0.0, 0.3025, 0.41)),
        material=white_enamel,
        name="rear_panel",
    )
    cabinet.visual(
        Box((0.035, 0.62, 0.82)),
        origin=Origin(xyz=(-0.2625, 0.0, 0.41)),
        material=white_enamel,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((0.035, 0.62, 0.82)),
        origin=Origin(xyz=(0.2625, 0.0, 0.41)),
        material=white_enamel,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((0.54, 0.60, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=soft_gray,
        name="toe_plinth",
    )

    top_deck_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.56, 0.62, 0.045, corner_segments=10),
        [_shift_profile(superellipse_profile(0.45, 0.45, exponent=2.0, segments=80), dy=-0.055)],
        0.050,
        center=False,
    )
    cabinet.visual(
        _save_mesh(top_deck_geom, "washer_top_deck"),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=white_enamel,
        name="top_deck_opening",
    )

    # A visibly hollow stationary tub below the lid opening.
    tub_geom = LatheGeometry.from_shell_profiles(
        [
            (0.130, 0.185),
            (0.205, 0.225),
            (0.240, 0.770),
            (0.238, 0.822),
        ],
        [
            (0.030, 0.205),
            (0.176, 0.245),
            (0.212, 0.750),
            (0.214, 0.805),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    tub_geom.merge(TorusGeometry(radius=0.226, tube=0.010, radial_segments=18, tubular_segments=88).translate(0.0, 0.0, 0.812))
    cabinet.visual(
        _save_mesh(tub_geom, "stationary_tub_shell"),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=shadow,
        name="tub_shell",
    )

    # Compact rear pod and control face, carried by the cabinet root.
    cabinet.visual(
        Box((0.54, 0.120, 0.180)),
        origin=Origin(xyz=(0.0, 0.255, 0.930)),
        material=white_enamel,
        name="rear_control_pod",
    )
    cabinet.visual(
        Box((0.50, 0.006, 0.115)),
        origin=Origin(xyz=(0.0, 0.192, 0.935)),
        material=dark_panel,
        name="control_panel_face",
    )
    cabinet.visual(
        Box((0.54, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.195, 0.845)),
        material=white_enamel,
        name="pod_lower_blend",
    )

    # Static hinge-side brackets for the top lid.
    cabinet.visual(
        Cylinder(radius=0.008, length=0.39),
        origin=Origin(xyz=(0.0, 0.165, 0.902), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_hinge_pin",
    )
    cabinet.visual(
        Box((0.055, 0.025, 0.030)),
        origin=Origin(xyz=(-0.145, 0.173, 0.882)),
        material=dark_metal,
        name="hinge_bracket_0",
    )
    cabinet.visual(
        Box((0.055, 0.025, 0.030)),
        origin=Origin(xyz=(0.145, 0.173, 0.882)),
        material=dark_metal,
        name="hinge_bracket_1",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.475, 0.430, 0.026)),
        # The lid part frame is the rear hinge line; the compact panel extends
        # toward the front along local -Y.
        origin=Origin(xyz=(0.0, -0.215, 0.017)),
        material=white_enamel,
        name="lid_panel",
    )
    lid.visual(
        Box((0.395, 0.350, 0.006)),
        origin=Origin(xyz=(0.0, -0.215, 0.032)),
        material=soft_gray,
        name="recessed_lid_inset",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.23),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="lid_hinge_barrel",
    )

    basket = model.part("basket")
    basket_geom = LatheGeometry.from_shell_profiles(
        [
            (0.055, 0.000),
            (0.170, 0.034),
            (0.202, 0.515),
            (0.198, 0.580),
        ],
        [
            (0.016, 0.020),
            (0.148, 0.052),
            (0.184, 0.500),
            (0.184, 0.565),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )
    basket_geom.merge(TorusGeometry(radius=0.196, tube=0.007, radial_segments=18, tubular_segments=88).translate(0.0, 0.0, 0.580))
    basket.visual(
        _save_mesh(basket_geom, "perforated_basket_shell"),
        material=stainless,
        name="basket_shell",
    )

    agitator_geom = LatheGeometry(
        [
            (0.000, 0.020),
            (0.055, 0.022),
            (0.077, 0.070),
            (0.056, 0.130),
            (0.046, 0.405),
            (0.038, 0.515),
            (0.000, 0.525),
        ],
        segments=64,
    )
    basket.visual(
        _save_mesh(agitator_geom, "center_agitator"),
        material=white_enamel,
        name="center_agitator",
    )
    for fin_index in range(4):
        angle = fin_index * math.pi / 2.0
        basket.visual(
            Box((0.105, 0.020, 0.300)),
            origin=Origin(xyz=(0.054, 0.0, 0.235), rpy=(0.0, 0.0, angle)),
            material=white_enamel,
            name=f"agitator_fin_{fin_index}",
        )
    basket.visual(
        Cylinder(radius=0.042, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=soft_gray,
        name="softener_cup_rim",
    )
    basket.visual(
        Box((0.014, 0.007, 0.012)),
        origin=Origin(xyz=(-0.040, -0.019, 0.535)),
        material=dark_metal,
        name="cup_hinge_ear_0",
    )
    basket.visual(
        Box((0.014, 0.007, 0.012)),
        origin=Origin(xyz=(-0.040, 0.019, 0.535)),
        material=dark_metal,
        name="cup_hinge_ear_1",
    )

    softener_lid = model.part("softener_lid")
    softener_lid.visual(
        Cylinder(radius=0.038, length=0.008),
        origin=Origin(xyz=(0.035, 0.0, 0.006)),
        material=soft_gray,
        name="cup_lid_disc",
    )
    softener_lid.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="cup_lid_hinge_barrel",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        _save_mesh(
            KnobGeometry(
                0.070,
                0.030,
                body_style="skirted",
                top_diameter=0.055,
                grip=KnobGrip(style="fluted", count=20, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "selector_knob_mesh",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_gray,
        name="knob_cap",
    )

    button_xs = (-0.020, 0.065, 0.150)
    buttons = []
    for index, x in enumerate(button_xs):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.052, 0.018, 0.028)),
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=blue_trim,
            name="button_cap",
        )
        button.visual(
            Box((0.036, 0.012, 0.020)),
            origin=Origin(xyz=(0.0, -0.012, 0.0)),
            material=dark_metal,
            name="button_stem",
        )
        buttons.append((button, x))

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.165, 0.902)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "cabinet_to_basket",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(0.0, -0.055, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=18.0),
    )
    model.articulation(
        "basket_to_softener_lid",
        ArticulationType.REVOLUTE,
        parent=basket,
        child=softener_lid,
        origin=Origin(xyz=(-0.040, 0.0, 0.535)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.20, velocity=2.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "pod_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(-0.145, 0.189, 0.985)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.30, velocity=8.0),
    )
    for index, (button, x) in enumerate(buttons):
        model.articulation(
            f"pod_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, 0.189, 0.972)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.08, lower=0.0, upper=0.012),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lid = object_model.get_part("lid")
    cabinet = object_model.get_part("cabinet")
    basket = object_model.get_part("basket")
    softener_lid = object_model.get_part("softener_lid")
    selector_knob = object_model.get_part("selector_knob")
    button_0 = object_model.get_part("button_0")

    lid_joint = object_model.get_articulation("cabinet_to_lid")
    basket_joint = object_model.get_articulation("cabinet_to_basket")
    knob_joint = object_model.get_articulation("pod_to_selector_knob")
    button_joint = object_model.get_articulation("pod_to_button_0")
    softener_joint = object_model.get_articulation("basket_to_softener_lid")

    ctx.check("basket is continuous spin", basket_joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("selector knob is continuous spin", knob_joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("buttons are prismatic", button_joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("softener lid has hinge", softener_joint.articulation_type == ArticulationType.REVOLUTE)

    ctx.allow_overlap(
        cabinet,
        lid,
        elem_a="rear_hinge_pin",
        elem_b="lid_hinge_barrel",
        reason="The visible lid hinge barrel intentionally captures the rear metal pin.",
    )
    ctx.expect_overlap(
        lid,
        cabinet,
        axes="x",
        min_overlap=0.18,
        elem_a="lid_hinge_barrel",
        elem_b="rear_hinge_pin",
        name="lid hinge barrel shares the rear pin span",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.20}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "top lid rotates upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    button_rest = ctx.part_world_position(button_0)
    with ctx.pose({button_joint: 0.012}):
        button_pressed = ctx.part_world_position(button_0)
    ctx.check(
        "push button travels inward",
        button_rest is not None and button_pressed is not None and button_pressed[1] > button_rest[1] + 0.010,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    cup_rest = ctx.part_world_aabb(softener_lid)
    with ctx.pose({softener_joint: 1.10}):
        cup_open = ctx.part_world_aabb(softener_lid)
    ctx.check(
        "softener cup lid flips upward",
        cup_rest is not None and cup_open is not None and cup_open[1][2] > cup_rest[1][2] + 0.015,
        details=f"rest={cup_rest}, open={cup_open}",
    )

    with ctx.pose({basket_joint: math.pi / 2.0}):
        ctx.expect_origin_distance(
            basket,
            selector_knob,
            axes="xy",
            min_dist=0.20,
            name="basket spin remains centered under top opening",
        )

    return ctx.report()


object_model = build_object_model()
