from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _body_shell_mesh():
    outer_profile = [
        (0.040, 0.000),
        (0.052, 0.010),
        (0.071, 0.040),
        (0.086, 0.095),
        (0.083, 0.142),
        (0.072, 0.186),
        (0.060, 0.212),
        (0.054, 0.226),
    ]
    inner_profile = [
        (0.000, 0.008),
        (0.028, 0.014),
        (0.053, 0.042),
        (0.072, 0.097),
        (0.069, 0.141),
        (0.058, 0.184),
        (0.047, 0.208),
        (0.042, 0.218),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )


def _lid_shell_mesh():
    outer_profile = [
        (0.000, 0.016),
        (0.020, 0.015),
        (0.038, 0.012),
        (0.050, 0.006),
        (0.055, 0.001),
    ]
    inner_profile = [
        (0.000, 0.010),
        (0.018, 0.010),
        (0.035, 0.008),
        (0.047, 0.004),
        (0.052, 0.001),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _spout_mesh():
    return sweep_profile_along_spline(
        [
            (0.056, 0.000, 0.168),
            (0.074, 0.000, 0.174),
            (0.095, 0.000, 0.170),
        ],
        profile=rounded_rect_profile(0.034, 0.018, radius=0.006, corner_segments=6),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _handle_mesh():
    return sweep_profile_along_spline(
        [
            (-0.060, 0.000, 0.188),
            (-0.104, 0.000, 0.210),
            (-0.122, 0.000, 0.154),
            (-0.116, 0.000, 0.090),
            (-0.072, 0.000, 0.040),
        ],
        profile=rounded_rect_profile(0.024, 0.015, radius=0.005, corner_segments=6),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cordless_electric_kettle")

    body_metal = model.material("body_metal", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    base_black = model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    base_trim = model.material("base_trim", rgba=(0.21, 0.22, 0.24, 1.0))
    control_grey = model.material("control_grey", rgba=(0.27, 0.29, 0.31, 1.0))
    switch_red = model.material("switch_red", rgba=(0.76, 0.19, 0.15, 1.0))

    heating_base = model.part("heating_base")
    heating_base.visual(
        Cylinder(radius=0.110, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=base_black,
        name="base_disc",
    )
    heating_base.visual(
        Cylinder(radius=0.088, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=base_trim,
        name="base_seat",
    )
    heating_base.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=base_trim,
        name="contact_pad",
    )
    heating_base.visual(
        Cylinder(radius=0.025, length=0.004),
        origin=Origin(xyz=(0.090, 0.0, 0.024)),
        material=control_grey,
        name="dial_bezel",
    )

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_shell_mesh(), "kettle_body_shell"),
        material=body_metal,
        name="body_shell",
    )
    body.visual(
        mesh_from_geometry(_spout_mesh(), "kettle_spout"),
        material=body_metal,
        name="spout",
    )
    body.visual(
        Box((0.014, 0.016, 0.020)),
        origin=Origin(xyz=(-0.062, -0.018, 0.217)),
        material=dark_plastic,
        name="hinge_mount_0",
    )
    body.visual(
        Box((0.014, 0.016, 0.020)),
        origin=Origin(xyz=(-0.062, 0.018, 0.217)),
        material=dark_plastic,
        name="hinge_mount_1",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.015),
        origin=Origin(xyz=(-0.064, -0.018, 0.227), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="hinge_knuckle_0",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.015),
        origin=Origin(xyz=(-0.064, 0.018, 0.227), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="hinge_knuckle_1",
    )

    body.visual(
        mesh_from_geometry(_handle_mesh(), "kettle_handle"),
        material=dark_plastic,
        name="handle_grip",
    )
    body.visual(
        Box((0.026, 0.026, 0.016)),
        origin=Origin(xyz=(-0.098, 0.0, 0.209)),
        material=dark_plastic,
        name="handle_top",
    )
    body.visual(
        Box((0.016, 0.028, 0.028)),
        origin=Origin(xyz=(-0.091, 0.0, 0.050)),
        material=dark_plastic,
        name="handle_base",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_lid_shell_mesh(), "kettle_lid_shell"),
        origin=Origin(xyz=(0.055, 0.0, 0.000)),
        material=body_metal,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="hinge_knuckle",
    )
    lid.visual(
        Box((0.016, 0.030, 0.008)),
        origin=Origin(xyz=(0.030, 0.0, 0.014)),
        material=dark_plastic,
        name="lid_pad",
    )

    temperature_dial = model.part("temperature_dial")
    temperature_dial.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=control_grey,
        name="dial_body",
    )
    temperature_dial.visual(
        Box((0.004, 0.018, 0.003)),
        origin=Origin(xyz=(0.013, 0.0, 0.011)),
        material=body_metal,
        name="dial_marker",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.022, 0.015, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=control_grey,
        name="button_cap",
    )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="switch_pivot",
    )
    power_switch.visual(
        Box((0.010, 0.010, 0.026)),
        origin=Origin(xyz=(-0.006, 0.0, 0.013)),
        material=switch_red,
        name="switch_paddle",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.PRISMATIC,
        parent=heating_base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.30,
            lower=0.0,
            upper=0.180,
        ),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.064, 0.0, 0.227)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "base_to_temperature_dial",
        ArticulationType.CONTINUOUS,
        parent=heating_base,
        child=temperature_dial,
        origin=Origin(xyz=(0.090, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )
    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(-0.098, 0.0, 0.221)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=0.004,
        ),
    )
    model.articulation(
        "body_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_switch,
        origin=Origin(xyz=(-0.102, 0.016, 0.048)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=2.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    heating_base = object_model.get_part("heating_base")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    release_button = object_model.get_part("release_button")
    power_switch = object_model.get_part("power_switch")
    temperature_dial = object_model.get_part("temperature_dial")

    lift = object_model.get_articulation("base_to_body")
    lid_hinge = object_model.get_articulation("body_to_lid")
    button_slide = object_model.get_articulation("body_to_release_button")
    switch_joint = object_model.get_articulation("body_to_power_switch")
    dial_joint = object_model.get_articulation("base_to_temperature_dial")

    ctx.expect_gap(
        body,
        heating_base,
        axis="z",
        max_gap=0.004,
        max_penetration=1e-5,
        name="kettle body seats just above the heating base",
    )
    ctx.expect_overlap(
        body,
        heating_base,
        axes="xy",
        min_overlap=0.140,
        name="kettle body stays centered over the heating base",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="body_shell",
        max_gap=0.010,
        max_penetration=0.0,
        name="closed lid sits near the body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="body_shell",
        min_overlap=0.080,
        name="closed lid covers the top opening",
    )

    rest_body_pos = ctx.part_world_position(body)
    lift_upper = lift.motion_limits.upper if lift.motion_limits is not None else None
    if rest_body_pos is not None and lift_upper is not None:
        with ctx.pose({lift: lift_upper}):
            lifted_body_pos = ctx.part_world_position(body)
            ctx.expect_gap(
                body,
                heating_base,
                axis="z",
                min_gap=0.150,
                name="lifted kettle clears the heating base",
            )
        ctx.check(
            "kettle body lifts vertically off the base",
            lifted_body_pos is not None and lifted_body_pos[2] > rest_body_pos[2] + 0.150,
            details=f"rest={rest_body_pos}, lifted={lifted_body_pos}",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if closed_lid_aabb is not None and lid_upper is not None:
        with ctx.pose({lid_hinge: lid_upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
            ctx.check(
                "lid opens upward above the vessel opening",
                open_lid_aabb is not None
                and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.060,
                details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
            )

    button_rest = ctx.part_world_position(release_button)
    button_upper = button_slide.motion_limits.upper if button_slide.motion_limits is not None else None
    if button_rest is not None and button_upper is not None:
        with ctx.pose({button_slide: button_upper}):
            button_pressed = ctx.part_world_position(release_button)
        ctx.check(
            "lid release button depresses into the handle top",
            button_pressed is not None and button_pressed[2] < button_rest[2] - 0.003,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    switch_lower = switch_joint.motion_limits.lower if switch_joint.motion_limits is not None else None
    switch_upper = switch_joint.motion_limits.upper if switch_joint.motion_limits is not None else None
    if switch_lower is not None and switch_upper is not None:
        with ctx.pose({switch_joint: switch_lower}):
            switch_low_aabb = ctx.part_element_world_aabb(power_switch, elem="switch_paddle")
        with ctx.pose({switch_joint: switch_upper}):
            switch_high_aabb = ctx.part_element_world_aabb(power_switch, elem="switch_paddle")
        ctx.check(
            "power switch toggles through a short arc",
            switch_low_aabb is not None
            and switch_high_aabb is not None
            and switch_high_aabb[1][0] < switch_low_aabb[1][0] - 0.004,
            details=f"low={switch_low_aabb}, high={switch_high_aabb}",
        )

    dial_rest = ctx.part_element_world_aabb(temperature_dial, elem="dial_marker")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_quarter_turn = ctx.part_element_world_aabb(temperature_dial, elem="dial_marker")
    ctx.check(
        "temperature dial rotates about its center axis",
        dial_rest is not None
        and dial_quarter_turn is not None
        and dial_quarter_turn[1][1] > dial_rest[1][1] + 0.004,
        details=f"rest={dial_rest}, quarter_turn={dial_quarter_turn}",
    )

    return ctx.report()


object_model = build_object_model()
