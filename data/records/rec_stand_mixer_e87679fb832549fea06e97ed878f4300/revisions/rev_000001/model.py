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
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries):
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    corner: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    radius = min(corner, width * 0.49, height * 0.49)
    return [
        (x_pos, y_pos, z_pos + z_center)
        for y_pos, z_pos in rounded_rect_profile(
            width,
            height,
            radius,
            corner_segments=8,
        )
    ]


def _build_base_shell_mesh():
    shell = section_loft(
        [
            _yz_section(-0.175, width=0.150, height=0.292, corner=0.050, z_center=0.146),
            _yz_section(-0.095, width=0.176, height=0.328, corner=0.060, z_center=0.164),
            _yz_section(-0.020, width=0.206, height=0.232, corner=0.055, z_center=0.116),
            _yz_section(0.040, width=0.224, height=0.146, corner=0.040, z_center=0.073),
            _yz_section(0.092, width=0.198, height=0.084, corner=0.026, z_center=0.042),
        ]
    )
    hinge_barrel = (
        CylinderGeometry(radius=0.027, height=0.158, radial_segments=32)
        .rotate_x(math.pi / 2.0)
        .translate(-0.086, 0.0, 0.274)
    )
    front_deck = BoxGeometry((0.092, 0.132, 0.018)).translate(0.050, 0.0, 0.030)
    front_tongue = BoxGeometry((0.040, 0.108, 0.020)).translate(0.116, 0.0, 0.028)
    carriage_stop = BoxGeometry((0.010, 0.108, 0.036)).translate(0.137, 0.0, 0.046)
    hinge_pad = BoxGeometry((0.040, 0.110, 0.045)).translate(-0.084, 0.0, 0.3205)
    speed_plinth = BoxGeometry((0.040, 0.024, 0.052)).translate(-0.050, 0.100, 0.180)
    return _merge_geometries(
        shell,
        hinge_barrel,
        front_deck,
        front_tongue,
        carriage_stop,
        hinge_pad,
        speed_plinth,
    )


def _build_head_shell_mesh():
    shell = section_loft(
        [
            _yz_section(0.028, width=0.104, height=0.104, corner=0.030, z_center=0.046),
            _yz_section(0.128, width=0.154, height=0.150, corner=0.046, z_center=0.042),
            _yz_section(0.222, width=0.132, height=0.126, corner=0.040, z_center=0.024),
            _yz_section(0.312, width=0.090, height=0.088, corner=0.026, z_center=0.006),
        ]
    )
    rear_trunnion = (
        CylinderGeometry(radius=0.022, height=0.104, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.012, 0.0, 0.040)
    )
    drive_boss = CylinderGeometry(radius=0.031, height=0.060, radial_segments=28).translate(
        0.286,
        0.0,
        -0.022,
    )
    return _merge_geometries(shell, rear_trunnion, drive_boss)


def _build_bowl_frame_mesh():
    platform = BoxGeometry((0.160, 0.140, 0.012)).translate(0.018, 0.0, 0.006)
    rear_carriage = BoxGeometry((0.060, 0.110, 0.024)).translate(-0.038, 0.0, 0.018)
    pedestal = CylinderGeometry(radius=0.052, height=0.024, radial_segments=32).translate(
        0.000,
        0.0,
        0.024,
    )
    return _merge_geometries(platform, rear_carriage, pedestal)


def _build_bowl_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.030, 0.000),
            (0.050, 0.008),
            (0.090, 0.028),
            (0.108, 0.078),
            (0.112, 0.146),
            (0.110, 0.154),
        ],
        [
            (0.000, 0.004),
            (0.024, 0.014),
            (0.082, 0.030),
            (0.098, 0.078),
            (0.100, 0.147),
        ],
        segments=56,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    ).translate(0.0, 0.0, 0.036)


def _build_whisk_mesh():
    whisk_geometries = [
        CylinderGeometry(radius=0.0055, height=0.083, radial_segments=20).translate(0.0, 0.0, -0.0105),
        CylinderGeometry(radius=0.0095, height=0.024, radial_segments=24).translate(0.0, 0.0, -0.050),
        CylinderGeometry(radius=0.0130, height=0.014, radial_segments=24).translate(0.0, 0.0, -0.068),
    ]
    loop_count = 10
    for loop_index in range(loop_count):
        angle = loop_index * math.pi / loop_count
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        loop_points = [
            (0.010 * cos_a, 0.010 * sin_a, -0.060),
            (0.018 * cos_a, 0.018 * sin_a, -0.078),
            (0.030 * cos_a, 0.030 * sin_a, -0.104),
            (0.043 * cos_a, 0.043 * sin_a, -0.130),
            (0.000, 0.000, -0.148),
            (-0.043 * cos_a, -0.043 * sin_a, -0.130),
            (-0.030 * cos_a, -0.030 * sin_a, -0.104),
            (-0.018 * cos_a, -0.018 * sin_a, -0.078),
            (-0.010 * cos_a, -0.010 * sin_a, -0.060),
        ]
        whisk_geometries.append(
            tube_from_spline_points(
                loop_points,
                radius=0.0016,
                samples_per_segment=12,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return _merge_geometries(*whisk_geometries)


def _build_speed_dial_mesh():
    dial = (
        CylinderGeometry(radius=0.018, height=0.010, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.005, 0.0)
    )
    pointer = BoxGeometry((0.006, 0.012, 0.022)).translate(0.0, 0.010, 0.006)
    stem = (
        CylinderGeometry(radius=0.007, height=0.008, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.004, 0.0)
    )
    return _merge_geometries(stem, dial, pointer)


def _build_head_lock_mesh():
    stem = BoxGeometry((0.024, 0.010, 0.008)).translate(0.012, 0.005, 0.0)
    grip = (
        CylinderGeometry(radius=0.008, height=0.012, radial_segments=24)
        .rotate_x(math.pi / 2.0)
        .translate(0.024, 0.006, 0.0)
    )
    return _merge_geometries(stem, grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_stand_mixer")

    body_cream = model.material("body_cream", rgba=(0.94, 0.90, 0.82, 1.0))
    bowl_turquoise = model.material("bowl_turquoise", rgba=(0.20, 0.77, 0.79, 1.0))
    polished_metal = model.material("polished_metal", rgba=(0.82, 0.84, 0.87, 1.0))
    dark_bakelite = model.material("dark_bakelite", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        _save_mesh("base_shell", _build_base_shell_mesh()),
        material=body_cream,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.280, 0.230, 0.330)),
        mass=10.0,
        origin=Origin(xyz=(-0.035, 0.0, 0.165)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        _save_mesh("bowl_carriage_frame", _build_bowl_frame_mesh()),
        material=polished_metal,
        name="carriage_frame",
    )
    bowl_carriage.visual(
        _save_mesh("mixer_bowl_shell", _build_bowl_shell_mesh()),
        material=bowl_turquoise,
        name="bowl_shell",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.170),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    head = model.part("head")
    head.visual(
        _save_mesh("head_shell", _build_head_shell_mesh()),
        material=body_cream,
        name="head_shell",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.340, 0.160, 0.160)),
        mass=4.8,
        origin=Origin(xyz=(0.165, 0.0, 0.040)),
    )

    whisk = model.part("whisk")
    whisk.visual(
        _save_mesh("balloon_whisk", _build_whisk_mesh()),
        material=polished_metal,
        name="whisk_shell",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.155),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        _save_mesh("speed_dial", _build_speed_dial_mesh()),
        material=dark_bakelite,
        name="speed_dial",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.040, 0.018, 0.040)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        _save_mesh("head_lock_slider", _build_head_lock_mesh()),
        material=dark_bakelite,
        name="head_lock_slider",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.032, 0.016, 0.018)),
        mass=0.04,
        origin=Origin(xyz=(0.016, 0.008, 0.0)),
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.210, 0.0, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.15, lower=0.0, upper=0.070),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.086, 0.0, 0.325)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.286, 0.0, -0.083)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=18.0),
    )
    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.050, 0.112, 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-0.90,
            upper=0.90,
        ),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.115, 0.089981, 0.116)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl_carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    speed_dial = object_model.get_articulation("base_to_speed_control")
    lock_slider = object_model.get_articulation("base_to_head_lock")

    with ctx.pose({bowl_slide: 0.0, head_tilt: 0.0}):
        ctx.expect_overlap(
            whisk,
            bowl_carriage,
            axes="xy",
            elem_a="whisk_shell",
            elem_b="bowl_shell",
            min_overlap=0.070,
            name="whisk is centered over the bowl at rest",
        )
        ctx.expect_overlap(
            bowl_carriage,
            base,
            axes="y",
            elem_a="carriage_frame",
            elem_b="base_shell",
            min_overlap=0.120,
            name="bowl carriage stays aligned with the base front",
        )
        ctx.expect_overlap(
            speed_control,
            base,
            axes="xz",
            elem_a="speed_dial",
            elem_b="base_shell",
            min_overlap=0.010,
            name="speed dial is mounted on the base flank",
        )
        ctx.expect_overlap(
            head_lock,
            base,
            axes="xz",
            elem_a="head_lock_slider",
            elem_b="base_shell",
            min_overlap=0.008,
            name="head lock slider is mounted on the base flank",
        )

    bowl_rest = ctx.part_world_position(bowl_carriage)
    whisk_rest = ctx.part_world_position(whisk)
    lock_rest = ctx.part_world_position(head_lock)

    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        bowl_extended = ctx.part_world_position(bowl_carriage)

    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        whisk_raised = ctx.part_world_position(whisk)

    with ctx.pose({lock_slider: lock_slider.motion_limits.upper}):
        lock_extended = ctx.part_world_position(head_lock)

    ctx.check(
        "bowl carriage slides forward",
        bowl_rest is not None and bowl_extended is not None and bowl_extended[0] > bowl_rest[0] + 0.05,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )
    ctx.check(
        "head tilt lifts the whisk",
        whisk_rest is not None and whisk_raised is not None and whisk_raised[2] > whisk_rest[2] + 0.10,
        details=f"rest={whisk_rest}, raised={whisk_raised}",
    )
    ctx.check(
        "head lock slider moves forward",
        lock_rest is not None and lock_extended is not None and lock_extended[0] > lock_rest[0] + 0.01,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )
    ctx.check(
        "speed dial remains on the right side of the base",
        ctx.part_world_position(speed_control) is not None
        and ctx.part_world_position(speed_control)[1] > 0.07,
        details=f"speed_pos={ctx.part_world_position(speed_control)}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
