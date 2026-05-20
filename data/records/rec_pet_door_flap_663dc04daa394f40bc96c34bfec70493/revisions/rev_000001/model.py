from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_pet_flap")

    white_plastic = model.material("warm_white_plastic", rgba=(0.88, 0.86, 0.78, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    smoked_clear = model.material("smoked_clear_polycarbonate", rgba=(0.50, 0.65, 0.78, 0.38))
    clear_panel_mat = model.material("clear_rigid_panel", rgba=(0.74, 0.90, 1.0, 0.34))
    red_lock = model.material("red_lock_slider", rgba=(0.80, 0.06, 0.03, 1.0))
    pin_metal = model.material("brushed_pin_metal", rgba=(0.55, 0.56, 0.55, 1.0))

    opening_w = 0.240
    opening_h = 0.300
    outer_w = 0.360
    outer_h = 0.440

    frame = model.part("frame")
    frame.visual(
        Box((outer_w, 0.060, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, opening_h / 2.0 + 0.035)),
        material=white_plastic,
        name="top_rail",
    )
    frame.visual(
        Box((outer_w, 0.060, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -opening_h / 2.0 - 0.035)),
        material=white_plastic,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.060, 0.060, opening_h + 0.020)),
        origin=Origin(xyz=(-opening_w / 2.0 - 0.030, 0.0, 0.0)),
        material=white_plastic,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.060, 0.060, opening_h + 0.020)),
        origin=Origin(xyz=(opening_w / 2.0 + 0.030, 0.0, 0.0)),
        material=white_plastic,
        name="side_rail_1",
    )
    frame.visual(
        Box((0.270, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.030, 0.141)),
        material=black_rubber,
        name="upper_gasket",
    )
    frame.visual(
        Box((0.270, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.030, -0.141)),
        material=black_rubber,
        name="lower_gasket",
    )
    frame.visual(
        Box((0.018, 0.006, 0.282)),
        origin=Origin(xyz=(-0.129, -0.030, 0.0)),
        material=black_rubber,
        name="side_gasket_0",
    )
    frame.visual(
        Box((0.018, 0.006, 0.282)),
        origin=Origin(xyz=(0.129, -0.030, 0.0)),
        material=black_rubber,
        name="side_gasket_1",
    )

    # Exterior screw caps and the lower lock slot are seated into the molded trim.
    for i, (x, z) in enumerate(
        (
            (-0.145, 0.170),
            (0.145, 0.170),
            (-0.145, -0.170),
            (0.145, -0.170),
        )
    ):
        frame.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, -0.033, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=white_plastic,
            name=f"screw_cap_{i}",
        )
    frame.visual(
        Box((0.150, 0.002, 0.032)),
        origin=Origin(xyz=(0.0, -0.031, -0.188)),
        material=black_rubber,
        name="lock_slot",
    )

    # The interior flap hinge sits along the upper edge of the opening.
    frame.visual(
        Box((0.024, 0.030, 0.030)),
        origin=Origin(xyz=(-0.126, 0.000, 0.150)),
        material=white_plastic,
        name="flap_hinge_boss_0",
    )
    frame.visual(
        Box((0.024, 0.030, 0.030)),
        origin=Origin(xyz=(0.126, 0.000, 0.150)),
        material=white_plastic,
        name="flap_hinge_boss_1",
    )

    # The exterior hood has a fixed top pin and two molded stand-off brackets.
    frame.visual(
        Cylinder(radius=0.0055, length=0.318),
        origin=Origin(xyz=(0.0, -0.045, 0.222), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_metal,
        name="frame_hood_pin",
    )
    frame.visual(
        Box((0.035, 0.022, 0.038)),
        origin=Origin(xyz=(-0.146, -0.037, 0.210)),
        material=white_plastic,
        name="hood_pin_bracket_0",
    )
    frame.visual(
        Box((0.035, 0.022, 0.038)),
        origin=Origin(xyz=(0.146, -0.037, 0.210)),
        material=white_plastic,
        name="hood_pin_bracket_1",
    )

    flap = model.part("flap")
    panel_h = 0.265
    panel_w = 0.214
    panel_t = 0.006
    panel_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(panel_w, panel_h, 0.018, corner_segments=8),
            panel_t,
            center=True,
        ),
        "clear_flap_panel",
    )
    flap.visual(
        panel_mesh,
        origin=Origin(xyz=(0.0, 0.000, -panel_h / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clear_panel_mat,
        name="clear_panel",
    )
    flap.visual(
        Cylinder(radius=0.008, length=0.210),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_panel_mat,
        name="flap_hinge_sleeve",
    )
    flap.visual(
        Box((0.205, 0.009, 0.017)),
        origin=Origin(xyz=(0.0, 0.000, -panel_h + 0.010)),
        material=black_rubber,
        name="magnetic_bottom_strip",
    )
    flap.visual(
        Box((0.008, 0.008, 0.218)),
        origin=Origin(xyz=(-panel_w / 2.0 + 0.004, 0.000, -0.132)),
        material=black_rubber,
        name="side_seal_0",
    )
    flap.visual(
        Box((0.008, 0.008, 0.218)),
        origin=Origin(xyz=(panel_w / 2.0 - 0.004, 0.000, -0.132)),
        material=black_rubber,
        name="side_seal_1",
    )

    weather_hood = model.part("weather_hood")
    weather_hood.visual(
        Cylinder(radius=0.0090, length=0.245),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="hood_hinge_clip",
    )
    weather_hood.visual(
        Box((0.305, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, -0.024, -0.008)),
        material=smoked_clear,
        name="hood_top_lip",
    )
    weather_hood.visual(
        Box((0.300, 0.006, 0.158)),
        origin=Origin(xyz=(0.0, -0.065, -0.076), rpy=(-0.35, 0.0, 0.0)),
        material=smoked_clear,
        name="hood_face",
    )
    weather_hood.visual(
        Box((0.009, 0.074, 0.150)),
        origin=Origin(xyz=(-0.153, -0.060, -0.074), rpy=(-0.35, 0.0, 0.0)),
        material=smoked_clear,
        name="hood_side_0",
    )
    weather_hood.visual(
        Box((0.009, 0.074, 0.150)),
        origin=Origin(xyz=(0.153, -0.060, -0.074), rpy=(-0.35, 0.0, 0.0)),
        material=smoked_clear,
        name="hood_side_1",
    )

    lock_slider = model.part("lock_slider")
    lock_slider.visual(
        Box((0.046, 0.014, 0.026)),
        origin=Origin(),
        material=red_lock,
        name="lock_tab",
    )
    lock_slider.visual(
        Box((0.012, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, 0.000)),
        material=black_rubber,
        name="lock_grip_inset",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.000, opening_h / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "frame_to_weather_hood",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=weather_hood,
        origin=Origin(xyz=(0.0, -0.045, 0.222)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "frame_to_lock_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lock_slider,
        origin=Origin(xyz=(0.0, -0.037, -0.188)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.15, lower=-0.040, upper=0.040),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    hood = object_model.get_part("weather_hood")
    lock = object_model.get_part("lock_slider")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    hood_hinge = object_model.get_articulation("frame_to_weather_hood")
    lock_slide = object_model.get_articulation("frame_to_lock_slider")

    ctx.allow_overlap(
        frame,
        hood,
        elem_a="frame_hood_pin",
        elem_b="hood_hinge_clip",
        reason="The exterior hood clip intentionally wraps the fixed top hinge pin.",
    )
    ctx.expect_overlap(
        hood,
        frame,
        axes="x",
        elem_a="hood_hinge_clip",
        elem_b="frame_hood_pin",
        min_overlap=0.20,
        name="weather hood clip remains captured on the top hinge pin",
    )

    ctx.allow_overlap(
        frame,
        flap,
        elem_a="top_rail",
        elem_b="flap_hinge_sleeve",
        reason="The clear flap hinge sleeve is intentionally seated in the upper frame edge.",
    )
    ctx.expect_overlap(
        flap,
        frame,
        axes="x",
        elem_a="flap_hinge_sleeve",
        elem_b="top_rail",
        min_overlap=0.18,
        name="flap hinge sleeve spans the upper opening edge",
    )

    panel_aabb = ctx.part_element_world_aabb(flap, elem="clear_panel")
    if panel_aabb is None:
        ctx.fail("clear panel has measurable bounds", "clear_panel AABB was unavailable")
    else:
        panel_min, panel_max = panel_aabb
        ctx.check(
            "clear panel fits inside the frame opening",
            panel_min[0] > -0.118
            and panel_max[0] < 0.118
            and panel_min[2] > -0.145
            and panel_max[2] <= 0.151,
            details=f"panel_min={panel_min}, panel_max={panel_max}",
        )
        ctx.check(
            "flap hangs from the upper frame edge",
            abs(panel_max[2] - 0.150) < 0.004,
            details=f"panel top z={panel_max[2]}",
        )

    closed_hood_aabb = ctx.part_world_aabb(hood)
    with ctx.pose({hood_hinge: 1.05}):
        lifted_hood_aabb = ctx.part_world_aabb(hood)
    if closed_hood_aabb is None or lifted_hood_aabb is None:
        ctx.fail("weather hood lifting pose is measurable", "hood AABB was unavailable")
    else:
        closed_min, _ = closed_hood_aabb
        lifted_min, _ = lifted_hood_aabb
        ctx.check(
            "weather hood rotates upward on its top hinge",
            lifted_min[2] > closed_min[2] + 0.035,
            details=f"closed_min_z={closed_min[2]}, lifted_min_z={lifted_min[2]}",
        )

    closed_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 0.80}):
        swung_flap_aabb = ctx.part_world_aabb(flap)
    if closed_flap_aabb is None or swung_flap_aabb is None:
        ctx.fail("flap swing pose is measurable", "flap AABB was unavailable")
    else:
        _, closed_max = closed_flap_aabb
        _, swung_max = swung_flap_aabb
        ctx.check(
            "pet flap swings through the opening on a horizontal upper hinge",
            swung_max[1] > closed_max[1] + 0.11,
            details=f"closed_max_y={closed_max[1]}, swung_max_y={swung_max[1]}",
        )

    rest_lock_pos = ctx.part_world_position(lock)
    with ctx.pose({lock_slide: 0.040}):
        shifted_lock_pos = ctx.part_world_position(lock)
    ctx.check(
        "locking slider travels across the lower rail",
        rest_lock_pos is not None
        and shifted_lock_pos is not None
        and shifted_lock_pos[0] > rest_lock_pos[0] + 0.035,
        details=f"rest={rest_lock_pos}, shifted={shifted_lock_pos}",
    )

    return ctx.report()


object_model = build_object_model()
