from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(cx + x, cy + y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y_center + y, z_center + z)
        for z, y in rounded_rect_profile(height, width, radius)
    ]


def _build_balloon_whisk():
    whisk_geom = CylinderGeometry(radius=0.006, height=0.046).translate(0.0, 0.0, -0.023)
    whisk_geom.merge(CylinderGeometry(radius=0.010, height=0.024).translate(0.0, 0.0, -0.050))
    whisk_geom.merge(CylinderGeometry(radius=0.014, height=0.018).translate(0.0, 0.0, -0.071))

    loop_count = 10
    for index in range(loop_count):
        angle = index * math.pi / loop_count
        c = math.cos(angle)
        s = math.sin(angle)
        whisk_geom.merge(
            tube_from_spline_points(
                [
                    (0.010 * c, 0.010 * s, -0.046),
                    (0.024 * c, 0.024 * s, -0.062),
                    (0.040 * c, 0.040 * s, -0.086),
                    (0.050 * c, 0.050 * s, -0.108),
                    (0.0, 0.0, -0.132),
                    (-0.050 * c, -0.050 * s, -0.108),
                    (-0.040 * c, -0.040 * s, -0.086),
                    (-0.024 * c, -0.024 * s, -0.062),
                    (-0.010 * c, -0.010 * s, -0.046),
                ],
                radius=0.00145,
                samples_per_segment=18,
                radial_segments=14,
            )
        )

    return whisk_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stand_mixer")

    enamel = model.material("gloss_enamel_red", rgba=(0.80, 0.10, 0.12, 1.0))
    steel = model.material("bright_steel", rgba=(0.89, 0.90, 0.92, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.11, 0.11, 0.12, 1.0))
    soft_dark = model.material("soft_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    base = model.part("base")

    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.37, 0.23, 0.055), 0.035),
            "base_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=enamel,
        name="base_plate",
    )

    pedestal_sections = [
        _xy_section(0.12, 0.16, 0.035, 0.035, cx=-0.105),
        _xy_section(0.10, 0.14, 0.030, 0.150, cx=-0.103),
        _xy_section(0.085, 0.120, 0.026, 0.255, cx=-0.102),
        _xy_section(0.074, 0.102, 0.022, 0.305, cx=-0.103),
    ]
    base.visual(
        mesh_from_geometry(section_loft(pedestal_sections), "pedestal_shell"),
        material=enamel,
        name="pedestal_shell",
    )

    base.visual(
        Box((0.17, 0.014, 0.010)),
        origin=Origin(xyz=(0.070, -0.050, 0.040)),
        material=soft_dark,
        name="left_track",
    )
    base.visual(
        Box((0.17, 0.014, 0.010)),
        origin=Origin(xyz=(0.070, 0.050, 0.040)),
        material=soft_dark,
        name="right_track",
    )

    base.visual(
        Box((0.040, 0.018, 0.046)),
        origin=Origin(xyz=(-0.092, 0.089, 0.172)),
        material=enamel,
        name="dial_mount_pad",
    )
    base.visual(
        Box((0.034, 0.018, 0.032)),
        origin=Origin(xyz=(-0.112, -0.089, 0.232)),
        material=enamel,
        name="button_mount_pad",
    )
    base.visual(
        Box((0.028, 0.014, 0.030)),
        origin=Origin(xyz=(-0.092, 0.073, 0.172)),
        material=enamel,
        name="dial_mount_bridge",
    )
    base.visual(
        Box((0.026, 0.040, 0.028)),
        origin=Origin(xyz=(-0.112, -0.073, 0.232)),
        material=enamel,
        name="button_mount_bridge",
    )

    base.visual(
        Box((0.032, 0.024, 0.040)),
        origin=Origin(xyz=(-0.105, -0.050, 0.305)),
        material=enamel,
        name="left_hinge_post",
    )
    base.visual(
        Box((0.032, 0.024, 0.040)),
        origin=Origin(xyz=(-0.105, 0.050, 0.305)),
        material=enamel,
        name="right_hinge_post",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(-0.105, -0.050, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="left_hinge_cheek",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(-0.105, 0.050, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="right_hinge_cheek",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.37, 0.23, 0.35)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.028, 0.000),
                    (0.058, 0.014),
                    (0.096, 0.056),
                    (0.108, 0.118),
                    (0.108, 0.145),
                    (0.112, 0.152),
                ],
                [
                    (0.000, 0.008),
                    (0.050, 0.016),
                    (0.089, 0.056),
                    (0.099, 0.118),
                    (0.099, 0.147),
                ],
                segments=56,
            ),
            "mixing_bowl",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=steel,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.028, length=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.0765)),
        material=steel,
        name="pedestal_post",
    )
    bowl.visual(
        Box((0.130, 0.155, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=steel,
        name="carriage_plate",
    )
    bowl.visual(
        Box((0.110, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.050, 0.052)),
        material=soft_dark,
        name="left_runner",
    )
    bowl.visual(
        Box((0.110, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.050, 0.052)),
        material=soft_dark,
        name="right_runner",
    )
    bowl.inertial = Inertial.from_geometry(
        Box((0.14, 0.16, 0.24)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    model.articulation(
        "base_to_bowl_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.060),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.017, length=0.074),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )
    head.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _yz_section(0.060, 0.082, 0.018, 0.025, z_center=0.052),
                    _yz_section(0.128, 0.168, 0.040, 0.065, z_center=0.070),
                    _yz_section(0.158, 0.190, 0.048, 0.170, z_center=0.064),
                    _yz_section(0.130, 0.150, 0.038, 0.270, z_center=0.044),
                    _yz_section(0.055, 0.058, 0.016, 0.340, z_center=0.012),
                ]
            ),
            "head_shell",
        ),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Box((0.036, 0.050, 0.032)),
        origin=Origin(xyz=(0.018, 0.0, 0.022)),
        material=enamel,
        name="rear_fairing",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.205, 0.0, -0.056)),
        material=dark_trim,
        name="nose_housing",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.35, 0.17, 0.22)),
        mass=5.8,
        origin=Origin(xyz=(0.165, 0.0, 0.02)),
    )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.105, 0.0, 0.325)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    whisk = model.part("whisk")
    whisk.visual(
        mesh_from_geometry(_build_balloon_whisk(), "balloon_whisk"),
        material=steel,
        name="whisk_shell",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.14),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
    )

    model.articulation(
        "head_to_whisk_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.205, 0.0, -0.081)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="dial_body",
    )
    speed_dial.visual(
        Box((0.008, 0.004, 0.012)),
        origin=Origin(xyz=(0.013, 0.014, 0.0)),
        material=dark_trim,
        name="dial_pointer",
    )
    speed_dial.inertial = Inertial.from_geometry(
        Box((0.036, 0.016, 0.036)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
    )

    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(-0.092, 0.098, 0.172)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=4.0,
            lower=-math.radians(120.0),
            upper=math.radians(120.0),
        ),
    )

    lock_button = model.part("lock_button")
    lock_button.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="button_cap",
    )
    lock_button.inertial = Inertial.from_geometry(
        Box((0.020, 0.012, 0.020)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
    )

    model.articulation(
        "base_to_lock_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_button,
        origin=Origin(xyz=(-0.112, -0.098, 0.232)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.05, lower=0.0, upper=0.0035),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    whisk = object_model.get_part("whisk")
    speed_dial = object_model.get_part("speed_dial")
    lock_button = object_model.get_part("lock_button")

    bowl_slide = object_model.get_articulation("base_to_bowl_slide")
    head_tilt = object_model.get_articulation("base_to_head_tilt")
    button_slide = object_model.get_articulation("base_to_lock_button")

    bowl_slide_upper = 0.060
    head_tilt_upper = math.radians(62.0)
    button_slide_upper = 0.0035

    ctx.expect_contact(
        bowl,
        base,
        elem_a="left_runner",
        elem_b="left_track",
        name="left carriage runner sits on left slide track",
    )
    ctx.expect_contact(
        bowl,
        base,
        elem_a="right_runner",
        elem_b="right_track",
        name="right carriage runner sits on right slide track",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="x",
        elem_a="left_runner",
        elem_b="left_track",
        min_overlap=0.10,
        name="left runner has strong insertion on its track at rest",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="x",
        elem_a="right_runner",
        elem_b="right_track",
        min_overlap=0.10,
        name="right runner has strong insertion on its track at rest",
    )
    ctx.expect_within(
        whisk,
        bowl,
        axes="xy",
        inner_elem="whisk_shell",
        outer_elem="bowl_shell",
        margin=0.0,
        name="balloon whisk footprint stays inside the bowl opening",
    )
    ctx.expect_contact(
        speed_dial,
        base,
        elem_a="dial_body",
        elem_b="dial_mount_pad",
        name="speed dial seats against its base boss",
    )
    ctx.expect_contact(
        lock_button,
        base,
        elem_a="button_cap",
        elem_b="button_mount_pad",
        name="lock button sits against its guide pad",
    )

    bowl_rest_pos = ctx.part_world_position(bowl)
    whisk_rest_pos = ctx.part_world_position(whisk)
    button_rest_pos = ctx.part_world_position(lock_button)

    with ctx.pose({bowl_slide: bowl_slide_upper}):
        ctx.expect_overlap(
            bowl,
            base,
            axes="x",
            elem_a="left_runner",
            elem_b="left_track",
            min_overlap=0.05,
            name="left runner stays retained when the bowl carriage is extended",
        )
        ctx.expect_overlap(
            bowl,
            base,
            axes="x",
            elem_a="right_runner",
            elem_b="right_track",
            min_overlap=0.05,
            name="right runner stays retained when the bowl carriage is extended",
        )
        bowl_extended_pos = ctx.part_world_position(bowl)

    with ctx.pose({head_tilt: head_tilt_upper}):
        whisk_open_pos = ctx.part_world_position(whisk)

    with ctx.pose({button_slide: button_slide_upper}):
        button_pressed_pos = ctx.part_world_position(lock_button)

    ctx.check(
        "bowl carriage slides forward from the base",
        bowl_rest_pos is not None
        and bowl_extended_pos is not None
        and bowl_extended_pos[0] > bowl_rest_pos[0] + 0.04,
        details=f"rest={bowl_rest_pos}, extended={bowl_extended_pos}",
    )
    ctx.check(
        "tilt head lifts the whisk clear upward",
        whisk_rest_pos is not None
        and whisk_open_pos is not None
        and whisk_open_pos[2] > whisk_rest_pos[2] + 0.08,
        details=f"rest={whisk_rest_pos}, open={whisk_open_pos}",
    )
    ctx.check(
        "lock button presses inward",
        button_rest_pos is not None
        and button_pressed_pos is not None
        and button_pressed_pos[1] > button_rest_pos[1] + 0.002,
        details=f"rest={button_rest_pos}, pressed={button_pressed_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
