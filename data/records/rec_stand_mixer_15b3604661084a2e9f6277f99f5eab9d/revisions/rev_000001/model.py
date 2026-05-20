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


def _yz_section(
    width_y: float,
    height_z: float,
    radius: float,
    x: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y_center + py, z_center + pz) for py, pz in rounded_rect_profile(width_y, height_z, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bright_stand_mixer")

    shell_red = model.material("shell_red", rgba=(0.86, 0.13, 0.16, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.79, 0.81, 0.84, 1.0))
    bowl_steel = model.material("bowl_steel", rgba=(0.92, 0.93, 0.95, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.21, 1.0))

    base = model.part("base")

    foot_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.36, 0.22, 0.055), 0.026),
        "mixer_foot",
    )
    base.visual(
        foot_mesh,
        origin=Origin(xyz=(0.03, 0.0, 0.013)),
        material=shell_red,
        name="foot_shell",
    )

    pedestal_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.28, 0.17, 0.045), 0.032),
        "mixer_pedestal",
    )
    base.visual(
        pedestal_mesh,
        origin=Origin(xyz=(-0.005, 0.0, 0.042)),
        material=shell_red,
        name="pedestal_shell",
    )

    column_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.130, 0.120, 0.038, 0.0, z_center=0.060),
                _yz_section(0.115, 0.150, 0.034, 0.028, z_center=0.150),
                _yz_section(0.094, 0.132, 0.028, 0.050, z_center=0.252),
            ]
        ),
        "mixer_column",
    )
    base.visual(
        column_mesh,
        origin=Origin(xyz=(-0.170, 0.0, 0.0)),
        material=shell_red,
        name="rear_column",
    )
    base.visual(
        Box((0.070, 0.118, 0.044)),
        origin=Origin(xyz=(-0.133, 0.0, 0.060)),
        material=shell_red,
        name="column_fairing",
    )
    base.visual(
        Box((0.040, 0.110, 0.060)),
        origin=Origin(xyz=(-0.145, 0.0, 0.288)),
        material=shell_red,
        name="hinge_spine",
    )
    base.visual(
        Box((0.022, 0.018, 0.064)),
        origin=Origin(xyz=(-0.119, -0.049, 0.320)),
        material=trim_silver,
        name="right_hinge_ear",
    )
    base.visual(
        Box((0.022, 0.018, 0.064)),
        origin=Origin(xyz=(-0.119, 0.049, 0.320)),
        material=trim_silver,
        name="left_hinge_ear",
    )

    base.visual(
        Box((0.22, 0.15, 0.010)),
        origin=Origin(xyz=(0.065, 0.0, 0.063)),
        material=shell_red,
        name="carriage_deck",
    )
    base.visual(
        Box((0.19, 0.026, 0.008)),
        origin=Origin(xyz=(0.070, -0.050, 0.072)),
        material=trim_silver,
        name="right_slide_rail",
    )
    base.visual(
        Box((0.19, 0.026, 0.008)),
        origin=Origin(xyz=(0.070, 0.050, 0.072)),
        material=trim_silver,
        name="left_slide_rail",
    )
    base.visual(
        Box((0.055, 0.12, 0.010)),
        origin=Origin(xyz=(-0.025, 0.0, 0.073)),
        material=trim_silver,
        name="rear_carriage_stop",
    )
    base.visual(
        Box((0.040, 0.022, 0.050)),
        origin=Origin(xyz=(-0.112, -0.070, 0.172)),
        material=shell_red,
        name="speed_pod",
    )
    base.visual(
        Box((0.032, 0.024, 0.024)),
        origin=Origin(xyz=(-0.118, -0.072, 0.116)),
        material=shell_red,
        name="lock_housing",
    )
    base.visual(
        Box((0.030, 0.014, 0.078)),
        origin=Origin(xyz=(-0.116, -0.058, 0.121)),
        material=shell_red,
        name="control_panel_rib",
    )
    base.visual(
        Box((0.028, 0.016, 0.022)),
        origin=Origin(xyz=(-0.118, -0.068, 0.136)),
        material=shell_red,
        name="lock_bridge",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.40, 0.24, 0.34)),
        mass=8.0,
        origin=Origin(xyz=(0.01, 0.0, 0.17)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.185, 0.15, 0.012)),
        origin=Origin(xyz=(0.080, 0.0, 0.016)),
        material=trim_silver,
        name="slide_plate",
    )
    carriage.visual(
        Box((0.140, 0.030, 0.010)),
        origin=Origin(xyz=(0.070, -0.050, 0.005)),
        material=dark_trim,
        name="right_runner",
    )
    carriage.visual(
        Box((0.140, 0.030, 0.010)),
        origin=Origin(xyz=(0.070, 0.050, 0.005)),
        material=dark_trim,
        name="left_runner",
    )
    carriage.visual(
        Box((0.030, 0.120, 0.020)),
        origin=Origin(xyz=(-0.005, 0.0, 0.012)),
        material=trim_silver,
        name="rear_carriage_block",
    )
    carriage.visual(
        Cylinder(radius=0.074, length=0.012),
        origin=Origin(xyz=(0.115, 0.0, 0.028)),
        material=trim_silver,
        name="bowl_seat",
    )
    carriage.visual(
        Box((0.065, 0.018, 0.024)),
        origin=Origin(xyz=(0.070, -0.048, 0.022)),
        material=trim_silver,
        name="right_seat_rib",
    )
    carriage.visual(
        Box((0.065, 0.018, 0.024)),
        origin=Origin(xyz=(0.070, 0.048, 0.022)),
        material=trim_silver,
        name="left_seat_rib",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.20, 0.16, 0.05)),
        mass=1.2,
        origin=Origin(xyz=(0.08, 0.0, 0.025)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.030, 0.0, 0.076)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.14, lower=0.0, upper=0.055),
    )

    bowl = model.part("bowl")
    bowl_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.020, 0.0), (0.052, 0.010), (0.095, 0.052), (0.108, 0.125), (0.110, 0.168)],
            [(0.0, 0.006), (0.045, 0.014), (0.093, 0.055), (0.101, 0.162)],
            segments=64,
        ),
        "mixer_bowl_shell",
    )
    bowl.visual(bowl_mesh, material=bowl_steel, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.168),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.115, 0.0, 0.034)),
    )

    head = model.part("head")
    head.visual(
        Box((0.024, 0.080, 0.040)),
        material=trim_silver,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.055, 0.078, 0.034)),
        origin=Origin(xyz=(0.022, 0.0, 0.017)),
        material=shell_red,
        name="rear_neck",
    )
    head_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.090, 0.118, 0.026, 0.0, z_center=0.065),
                _yz_section(0.150, 0.170, 0.050, 0.12, z_center=0.076),
                _yz_section(0.132, 0.152, 0.040, 0.23, z_center=0.076),
                _yz_section(0.102, 0.122, 0.030, 0.312, z_center=0.070),
            ]
        ),
        "mixer_head_shell",
    )
    head.visual(head_mesh, material=shell_red, name="head_shell")
    head.visual(
        Cylinder(radius=0.028, length=0.026),
        origin=Origin(xyz=(0.240, 0.0, 0.000)),
        material=trim_silver,
        name="hub_housing",
    )
    head.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.240, 0.0, -0.009)),
        material=dark_trim,
        name="hub_socket",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.35, 0.18, 0.18)),
        mass=4.5,
        origin=Origin(xyz=(0.16, 0.0, 0.02)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.104, 0.0, 0.320)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(60.0),
        ),
    )

    whisk = model.part("whisk")
    whisk.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=bowl_steel,
        name="drive_shaft",
    )
    whisk.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=bowl_steel,
        name="upper_ferrule",
    )
    whisk.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.067)),
        material=bowl_steel,
        name="lower_ferrule",
    )

    for loop_index in range(5):
        angle = loop_index * math.pi / 5.0
        c = math.cos(angle)
        s = math.sin(angle)
        loop_mesh = mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.009 * c, 0.009 * s, -0.058),
                    (0.027 * c, 0.027 * s, -0.082),
                    (0.042 * c, 0.042 * s, -0.118),
                    (0.050 * c, 0.050 * s, -0.149),
                    (0.0, 0.0, -0.176),
                    (-0.050 * c, -0.050 * s, -0.149),
                    (-0.042 * c, -0.042 * s, -0.118),
                    (-0.027 * c, -0.027 * s, -0.082),
                    (-0.009 * c, -0.009 * s, -0.058),
                ],
                radius=0.0017,
                samples_per_segment=16,
                radial_segments=18,
            ),
            f"whisk_loop_{loop_index}",
        )
        whisk.visual(loop_mesh, material=bowl_steel, name=f"wire_loop_{loop_index}")

    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.06, length=0.19),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
    )

    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.240, 0.0, -0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9.0, velocity=24.0),
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material=dark_trim,
        name="lever_hub",
    )
    speed_lever.visual(
        Box((0.050, 0.012, 0.012)),
        origin=Origin(xyz=(0.024, -0.010, -0.015)),
        material=dark_trim,
        name="lever_arm",
    )
    speed_lever.visual(
        Box((0.014, 0.014, 0.016)),
        origin=Origin(xyz=(0.050, -0.010, -0.020)),
        material=trim_silver,
        name="lever_tip",
    )
    speed_lever.inertial = Inertial.from_geometry(
        Box((0.070, 0.018, 0.028)),
        mass=0.08,
        origin=Origin(xyz=(0.028, -0.010, -0.012)),
    )

    model.articulation(
        "base_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(-0.112, -0.081, 0.172)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    lock_button = model.part("lock_button")
    lock_button.visual(
        Box((0.022, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=trim_silver,
        name="button_cap",
    )
    lock_button.visual(
        Box((0.014, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=dark_trim,
        name="button_stem",
    )
    lock_button.inertial = Inertial.from_geometry(
        Box((0.024, 0.020, 0.020)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
    )

    model.articulation(
        "base_to_lock_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_button,
        origin=Origin(xyz=(-0.118, -0.084, 0.116)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    bowl = object_model.get_part("bowl")
    whisk = object_model.get_part("whisk")
    speed_lever = object_model.get_part("speed_lever")
    lock_button = object_model.get_part("lock_button")

    carriage_slide = object_model.get_articulation("base_to_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    lever_joint = object_model.get_articulation("base_to_speed_lever")
    button_joint = object_model.get_articulation("base_to_lock_button")

    ctx.allow_overlap(
        base,
        lock_button,
        elem_a="lock_housing",
        elem_b="button_stem",
        reason="The lock button stem is intentionally represented as translating inside a simplified lock housing sleeve proxy.",
    )

    ctx.expect_within(
        carriage,
        base,
        axes="y",
        inner_elem="slide_plate",
        outer_elem="carriage_deck",
        margin=0.001,
        name="carriage stays centered on the deck",
    )
    ctx.expect_overlap(
        whisk,
        bowl,
        axes="xy",
        min_overlap=0.08,
        name="whisk sits over the bowl footprint",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: carriage_slide.motion_limits.upper}):
        carriage_extended = ctx.part_world_position(carriage)
    ctx.check(
        "carriage slides forward",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.04,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    whisk_rest = ctx.part_world_position(whisk)
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        whisk_raised = ctx.part_world_position(whisk)
    ctx.check(
        "head tilt raises the whisk",
        whisk_rest is not None
        and whisk_raised is not None
        and whisk_raised[2] > whisk_rest[2] + 0.10,
        details=f"rest={whisk_rest}, raised={whisk_raised}",
    )

    lever_rest = ctx.part_element_world_aabb(speed_lever, elem="lever_tip")
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        lever_fast = ctx.part_element_world_aabb(speed_lever, elem="lever_tip")
    ctx.check(
        "speed lever rotates through a visible arc",
        lever_rest is not None
        and lever_fast is not None
        and (
            (
                (
                    ((lever_fast[0][0] + lever_fast[1][0]) * 0.5)
                    - ((lever_rest[0][0] + lever_rest[1][0]) * 0.5)
                )
                ** 2
                + (
                    ((lever_fast[0][2] + lever_fast[1][2]) * 0.5)
                    - ((lever_rest[0][2] + lever_rest[1][2]) * 0.5)
                )
                ** 2
            )
            ** 0.5
            > 0.012
        ),
        details=f"rest={lever_rest}, fast={lever_fast}",
    )

    button_rest = ctx.part_world_position(lock_button)
    with ctx.pose({button_joint: button_joint.motion_limits.upper}):
        button_pressed = ctx.part_world_position(lock_button)
    ctx.check(
        "lock button presses inward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[1] > button_rest[1] + 0.004,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
