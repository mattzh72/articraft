from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
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
    x_shift: float = 0.0,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_shift, y + y_shift, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
    ]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_shift)
        for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)
    ]


def _make_base_shell():
    base_shell = BoxGeometry((0.34, 0.22, 0.028)).translate(0.03, 0.0, 0.014)

    pedestal = repair_loft(
        section_loft(
            [
                _xy_section(0.19, 0.16, 0.050, 0.028, x_shift=-0.075),
                _xy_section(0.15, 0.14, 0.040, 0.110, x_shift=-0.078),
                _xy_section(0.12, 0.12, 0.034, 0.210, x_shift=-0.082),
                _xy_section(0.10, 0.12, 0.028, 0.315, x_shift=-0.082),
            ]
        ),
        repair="mesh",
    )
    base_shell.merge(pedestal)

    hinge_block = BoxGeometry((0.052, 0.126, 0.046)).translate(-0.082, 0.0, 0.338)
    base_shell.merge(hinge_block)

    rear_brace = BoxGeometry((0.050, 0.120, 0.070)).translate(-0.100, 0.0, 0.278)
    base_shell.merge(rear_brace)

    speed_pod = BoxGeometry((0.024, 0.020, 0.026)).translate(-0.010, 0.056, 0.176)
    base_shell.merge(speed_pod)

    return base_shell


def _make_head_shell():
    return repair_loft(
        section_loft(
            [
                _yz_section(0.112, 0.100, 0.030, 0.030, z_shift=0.070),
                _yz_section(0.154, 0.150, 0.044, 0.120, z_shift=0.058),
                _yz_section(0.160, 0.146, 0.042, 0.212, z_shift=0.034),
                _yz_section(0.112, 0.090, 0.028, 0.302, z_shift=0.004),
            ]
        ),
        repair="mesh",
    )


def _make_dough_hook():
    hook_geom = CylinderGeometry(radius=0.007, height=0.056, radial_segments=24).translate(
        0.0, 0.0, -0.028
    )
    hook_geom.merge(
        CylinderGeometry(radius=0.011, height=0.024, radial_segments=24).translate(
            0.0, 0.0, -0.058
        )
    )
    hook_geom.merge(
        CylinderGeometry(radius=0.013, height=0.018, radial_segments=24).translate(
            0.0, 0.0, -0.079
        )
    )

    hook_points: list[tuple[float, float, float]] = [(0.0, 0.008, -0.067)]
    for i in range(16):
        t = i / 15.0
        angle = 0.55 * math.pi + 2.35 * math.pi * t
        radius = 0.014 + 0.034 * t
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = -0.072 - 0.078 * t
        hook_points.append((x, y, z))
    hook_points.extend(
        [
            (0.026, -0.030, -0.150),
            (0.010, -0.024, -0.164),
            (-0.006, -0.008, -0.166),
        ]
    )

    hook_geom.merge(
        tube_from_spline_points(
            hook_points,
            radius=0.0058,
            samples_per_segment=16,
            radial_segments=20,
            cap_ends=True,
        )
    )
    return hook_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flagship_stand_mixer")

    body_enamel = model.material("body_enamel", rgba=(0.89, 0.87, 0.82, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.90, 0.91, 0.93, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_make_base_shell(), "base_shell"),
        material=body_enamel,
        name="base_shell",
    )
    base.visual(
        Box((0.140, 0.026, 0.008)),
        origin=Origin(xyz=(0.070, 0.055, 0.032)),
        material=trim_dark,
        name="right_track",
    )
    base.visual(
        Box((0.140, 0.026, 0.008)),
        origin=Origin(xyz=(0.070, -0.055, 0.032)),
        material=trim_dark,
        name="left_track",
    )
    base.visual(
        Box((0.012, 0.024, 0.020)),
        origin=Origin(xyz=(-0.131, -0.046, 0.305)),
        material=trim_dark,
        name="lock_guide",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.35, 0.23, 0.37)),
        mass=8.0,
        origin=Origin(xyz=(0.005, 0.0, 0.185)),
    )

    carriage = model.part("bowl_carriage")
    carriage.visual(
        Box((0.120, 0.022, 0.010)),
        origin=Origin(xyz=(0.060, 0.055, 0.005)),
        material=trim_dark,
        name="right_runner",
    )
    carriage.visual(
        Box((0.120, 0.022, 0.010)),
        origin=Origin(xyz=(0.060, -0.055, 0.005)),
        material=trim_dark,
        name="left_runner",
    )
    carriage.visual(
        Box((0.090, 0.132, 0.012)),
        origin=Origin(xyz=(0.105, 0.0, 0.016)),
        material=body_enamel,
        name="cradle_bridge",
    )
    carriage.visual(
        Cylinder(radius=0.078, length=0.010),
        origin=Origin(xyz=(0.115, 0.0, 0.027)),
        material=steel,
        name="bowl_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.20, 0.15, 0.05)),
        mass=1.2,
        origin=Origin(xyz=(0.100, 0.0, 0.020)),
    )

    bowl = model.part("bowl")
    bowl_geom = LatheGeometry.from_shell_profiles(
        [
            (0.034, 0.000),
            (0.046, 0.010),
            (0.096, 0.048),
            (0.118, 0.118),
            (0.124, 0.144),
        ],
        [
            (0.000, 0.006),
            (0.040, 0.016),
            (0.090, 0.050),
            (0.112, 0.118),
            (0.118, 0.140),
        ],
        segments=64,
    )
    bowl.visual(
        mesh_from_geometry(bowl_geom, "mixing_bowl"),
        material=steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.124, length=0.144),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_make_head_shell(), "head_shell"),
        material=body_enamel,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.112),
        origin=Origin(xyz=(0.008, 0.0, 0.023), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_enamel,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.060, 0.090, 0.048)),
        origin=Origin(xyz=(0.044, 0.0, 0.044)),
        material=body_enamel,
        name="rear_bridge",
    )
    head.visual(
        Box((0.088, 0.076, 0.060)),
        origin=Origin(xyz=(0.180, 0.0, -0.006)),
        material=body_enamel,
        name="gearcase_blend",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.064),
        origin=Origin(xyz=(0.232, 0.0, -0.052)),
        material=body_enamel,
        name="tool_hub",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.33, 0.18, 0.16)),
        mass=4.5,
        origin=Origin(xyz=(0.160, 0.0, 0.035)),
    )

    agitator = model.part("agitator")
    agitator.visual(
        mesh_from_geometry(_make_dough_hook(), "spiral_dough_hook"),
        material=steel,
        name="dough_hook",
    )
    agitator.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.172),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="dial_body",
    )
    speed_control.visual(
        Box((0.010, 0.008, 0.026)),
        origin=Origin(xyz=(0.014, 0.0, 0.010)),
        material=steel,
        name="dial_tab",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.040, 0.014, 0.040)),
        mass=0.06,
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Cylinder(radius=0.007, length=0.024),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="lock_plunger",
    )
    head_lock.visual(
        Box((0.010, 0.022, 0.016)),
        origin=Origin(xyz=(-0.029, 0.0, 0.0)),
        material=trim_dark,
        name="lock_thumb",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.040, 0.024, 0.018)),
        mass=0.04,
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.020, 0.0, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.10, lower=0.0, upper=0.045),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.115, 0.0, 0.032)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.082, 0.0, 0.360)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    model.articulation(
        "head_to_agitator",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=agitator,
        origin=Origin(xyz=(0.232, 0.0, -0.084)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )

    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.005, 0.072, 0.176)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=math.radians(-42.0),
            upper=math.radians(42.0),
        ),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.137, -0.046, 0.305)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.05, lower=0.0, upper=0.012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    carriage = object_model.get_part("bowl_carriage")
    agitator = object_model.get_part("agitator")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    tool_drive = object_model.get_articulation("head_to_agitator")
    speed_joint = object_model.get_articulation("base_to_speed_control")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "canonical articulation stack is present",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and tool_drive.articulation_type == ArticulationType.CONTINUOUS
        and speed_joint.articulation_type == ArticulationType.REVOLUTE
        and lock_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"types={(bowl_slide.articulation_type, head_tilt.articulation_type, tool_drive.articulation_type, speed_joint.articulation_type, lock_joint.articulation_type)}"
        ),
    )

    with ctx.pose({head_tilt: 0.0, bowl_slide: 0.0}):
        ctx.expect_overlap(
            agitator,
            bowl,
            axes="xy",
            min_overlap=0.055,
            name="closed head centers dough hook over bowl",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem="left_runner",
            negative_elem="left_track",
            max_penetration=0.0001,
            max_gap=0.001,
            name="carriage runners stay seated on mixer tracks",
        )

    with ctx.pose({head_tilt: math.radians(62.0)}):
        ctx.expect_gap(
            agitator,
            bowl,
            axis="z",
            min_gap=0.030,
            name="tilted head lifts dough hook clear of bowl",
        )

    carriage_rest = ctx.part_world_position(carriage)
    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: 0.045}):
        carriage_extended = ctx.part_world_position(carriage)
        bowl_extended = ctx.part_world_position(bowl)

    ctx.check(
        "bowl carriage slides forward for loading",
        carriage_rest is not None
        and carriage_extended is not None
        and bowl_rest is not None
        and bowl_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.030
        and bowl_extended[0] > bowl_rest[0] + 0.030,
        details=(
            f"carriage_rest={carriage_rest}, carriage_extended={carriage_extended}, "
            f"bowl_rest={bowl_rest}, bowl_extended={bowl_extended}"
        ),
    )

    dial_rest = ctx.part_element_world_aabb(speed_control, elem="dial_tab")
    with ctx.pose({speed_joint: math.radians(35.0)}):
        dial_turned = ctx.part_element_world_aabb(speed_control, elem="dial_tab")
    ctx.check(
        "speed control rotates its selector tab",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_turned[1][2] - dial_rest[1][2]) > 0.008,
        details=f"dial_rest={dial_rest}, dial_turned={dial_turned}",
    )

    lock_rest = ctx.part_world_position(head_lock)
    with ctx.pose({lock_joint: 0.012}):
        lock_extended = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock plunger extends outward from the base",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[0] < lock_rest[0] - 0.008,
        details=f"lock_rest={lock_rest}, lock_extended={lock_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
