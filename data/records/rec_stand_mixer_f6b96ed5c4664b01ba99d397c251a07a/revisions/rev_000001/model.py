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
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + center_x, y + center_y, z)
        for x, y in rounded_rect_profile(width, depth, radius)
    ]


def _yz_section(
    width_y: float,
    height_z: float,
    radius: float,
    x: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for z, y in rounded_rect_profile(height_z, width_y, radius)
    ]


def _tube_shell_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 48,
):
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _bowl_shell_geometry():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0, 0.0),
            (0.026, 0.010),
            (0.070, 0.048),
            (0.094, 0.108),
            (0.102, 0.156),
            (0.108, 0.166),
        ],
        [
            (0.0, 0.008),
            (0.022, 0.014),
            (0.062, 0.048),
            (0.088, 0.108),
            (0.096, 0.160),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _spiral_hook_geometry():
    hook = CylinderGeometry(radius=0.010, height=0.052, radial_segments=24).translate(
        0.0,
        0.0,
        -0.026,
    )

    points: list[tuple[float, float, float]] = [(0.0, 0.0, -0.004), (0.012, 0.0, -0.010)]
    turns = 1.15
    for index in range(24):
        t = index / 23.0
        angle = (turns * math.tau * t) - 0.4
        radius = 0.017 + (0.015 * t)
        z = -0.014 - (0.052 * t)
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    points.extend(
        [
            (0.020, -0.018, -0.070),
            (0.010, -0.015, -0.075),
            (0.004, -0.010, -0.078),
        ]
    )

    helix = tube_from_spline_points(
        points,
        radius=0.0075,
        samples_per_segment=8,
        radial_segments=18,
        cap_ends=True,
    )
    hook.merge(helix)
    return hook


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pastry_stand_mixer")

    enamel = model.material("enamel", rgba=(0.88, 0.86, 0.80, 1.0))
    trim = model.material("trim", rgba=(0.73, 0.74, 0.77, 1.0))
    bowl_metal = model.material("bowl_metal", rgba=(0.92, 0.93, 0.94, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")

    foot_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.40, 0.28, 0.055), 0.055),
        "mixer_foot_shell",
    )
    base.visual(
        foot_mesh,
        origin=Origin(xyz=(0.020, 0.0, 0.0275)),
        material=enamel,
        name="foot_shell",
    )

    pedestal_mesh = mesh_from_geometry(
        repair_loft(
            section_loft(
                [
                    _xy_section(0.18, 0.17, 0.040, 0.055, center_x=-0.092),
                    _xy_section(0.13, 0.15, 0.032, 0.180, center_x=-0.095),
                    _xy_section(0.10, 0.12, 0.026, 0.305, center_x=-0.108),
                    _xy_section(0.060, 0.090, 0.018, 0.327, center_x=-0.116),
                ]
            )
        ),
        "mixer_pedestal",
    )
    base.visual(pedestal_mesh, material=enamel, name="pedestal_shell")

    base.visual(
        Box((0.090, 0.190, 0.020)),
        origin=Origin(xyz=(-0.010, 0.0, 0.065)),
        material=enamel,
        name="column_bridge",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.230),
        origin=Origin(xyz=(0.030, -0.078, 0.170)),
        material=trim,
        name="left_column",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.230),
        origin=Origin(xyz=(0.030, 0.078, 0.170)),
        material=trim,
        name="right_column",
    )
    base.visual(
        Box((0.070, 0.012, 0.034)),
        origin=Origin(xyz=(-0.060, -0.063, 0.255)),
        material=trim,
        name="lock_deck",
    )
    base.visual(
        Box((0.052, 0.010, 0.006)),
        origin=Origin(xyz=(-0.064, -0.071, 0.266)),
        material=trim,
        name="lock_guide_left",
    )
    base.visual(
        Box((0.052, 0.010, 0.006)),
        origin=Origin(xyz=(-0.064, -0.071, 0.244)),
        material=trim,
        name="lock_guide_right",
    )
    base.visual(
        Box((0.036, 0.012, 0.040)),
        origin=Origin(xyz=(-0.050, 0.088, 0.165)),
        material=trim,
        name="selector_mount",
    )
    base.visual(
        Box((0.028, 0.028, 0.028)),
        origin=Origin(xyz=(-0.050, 0.074, 0.156)),
        material=trim,
        name="selector_bracket",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(-0.110, -0.041, 0.364), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="left_hinge_ear",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(-0.110, 0.041, 0.364), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="right_hinge_ear",
    )
    base.visual(
        Box((0.032, 0.020, 0.048)),
        origin=Origin(xyz=(-0.110, -0.041, 0.338)),
        material=trim,
        name="left_hinge_bracket",
    )
    base.visual(
        Box((0.032, 0.020, 0.048)),
        origin=Origin(xyz=(-0.110, 0.041, 0.338)),
        material=trim,
        name="right_hinge_bracket",
    )

    carriage = model.part("bowl_carriage")
    sleeve_mesh = mesh_from_geometry(
        _tube_shell_geometry(outer_radius=0.022, inner_radius=0.0165, length=0.160),
        "carriage_sleeve",
    )
    bowl_mesh = mesh_from_geometry(_bowl_shell_geometry(), "mixing_bowl_shell")
    carriage.visual(
        Box((0.020, 0.170, 0.030)),
        origin=Origin(xyz=(-0.025, 0.0, -0.005)),
        material=enamel,
        name="carriage_crossbeam",
    )
    carriage.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.0, -0.078, 0.055)),
        material=trim,
        name="left_sleeve",
    )
    carriage.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.078, 0.055)),
        material=trim,
        name="right_sleeve",
    )
    carriage.visual(
        Box((0.150, 0.024, 0.020)),
        origin=Origin(xyz=(0.055, -0.050, 0.015)),
        material=enamel,
        name="left_arm",
    )
    carriage.visual(
        Box((0.150, 0.024, 0.020)),
        origin=Origin(xyz=(0.055, 0.050, 0.015)),
        material=enamel,
        name="right_arm",
    )
    carriage.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.115, 0.0, 0.021)),
        material=trim,
        name="bowl_support",
    )
    carriage.visual(
        bowl_mesh,
        origin=Origin(xyz=(0.115, 0.0, 0.030)),
        material=bowl_metal,
        name="bowl_shell",
    )

    head = model.part("head")
    head_shell = mesh_from_geometry(
        repair_loft(
            section_loft(
                [
                    _yz_section(0.105, 0.095, 0.026, 0.055, z_center=0.012),
                    _yz_section(0.170, 0.148, 0.045, 0.185, z_center=0.036),
                    _yz_section(0.132, 0.102, 0.032, 0.320, z_center=0.032),
                ]
            )
        ),
        "tilt_head_shell",
    )
    head.visual(
        Cylinder(radius=0.021, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="head_hinge_barrel",
    )
    head.visual(
        Box((0.068, 0.046, 0.044)),
        origin=Origin(xyz=(0.034, 0.0, 0.010)),
        material=enamel,
        name="rear_spine",
    )
    head.visual(head_shell, material=enamel, name="head_shell")
    head.visual(
        Cylinder(radius=0.034, length=0.050),
        origin=Origin(xyz=(0.225, 0.0, -0.055)),
        material=trim,
        name="head_collar",
    )

    hook = model.part("spiral_hook")
    hook.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(),
        material=dark_metal,
        name="hook_ferrule",
    )
    hook.visual(
        mesh_from_geometry(_spiral_hook_geometry(), "spiral_hook_mesh"),
        material=dark_metal,
        name="hook_body",
    )

    selector = model.part("speed_selector")
    selector.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="selector_dial",
    )
    selector.visual(
        Box((0.006, 0.008, 0.022)),
        origin=Origin(xyz=(0.0, 0.010, 0.012)),
        material=black,
        name="selector_grip",
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.034, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=black,
        name="lock_slider",
    )
    head_lock.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.008, 0.0, 0.016)),
        material=black,
        name="lock_tab",
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.030, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.08, lower=0.0, upper=0.028),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.110, 0.0, 0.364)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )
    model.articulation(
        "head_to_hook",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=hook,
        origin=Origin(xyz=(0.225, 0.0, -0.088)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=22.0),
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=selector,
        origin=Origin(xyz=(-0.050, 0.100, 0.165)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=4.0,
            lower=-0.9,
            upper=0.9,
        ),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.068, -0.077, 0.255)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.06,
            lower=0.0,
            upper=0.016,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    hook = object_model.get_part("spiral_hook")
    selector = object_model.get_part("speed_selector")
    head_lock = object_model.get_part("head_lock")

    carriage_joint = object_model.get_articulation("base_to_bowl_carriage")
    head_joint = object_model.get_articulation("base_to_head")
    hook_joint = object_model.get_articulation("head_to_hook")
    selector_joint = object_model.get_articulation("base_to_speed_selector")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "bowl carriage uses a prismatic lift",
        carriage_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={carriage_joint.articulation_type}",
    )
    ctx.check(
        "tilt head uses a revolute rear hinge",
        head_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={head_joint.articulation_type}",
    )
    ctx.check(
        "spiral hook spins on a continuous tool joint",
        hook_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={hook_joint.articulation_type}",
    )
    ctx.check(
        "speed selector is rotary",
        selector_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={selector_joint.articulation_type}",
    )
    ctx.check(
        "head lock slides linearly",
        lock_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={lock_joint.articulation_type}",
    )

    ctx.expect_overlap(
        base,
        carriage,
        axes="z",
        elem_a="left_column",
        elem_b="left_sleeve",
        min_overlap=0.12,
        name="left guide sleeve stays engaged at rest",
    )
    ctx.expect_overlap(
        base,
        carriage,
        axes="z",
        elem_a="right_column",
        elem_b="right_sleeve",
        min_overlap=0.12,
        name="right guide sleeve stays engaged at rest",
    )
    ctx.expect_overlap(
        hook,
        carriage,
        axes="xy",
        elem_a="hook_body",
        elem_b="bowl_shell",
        min_overlap=0.05,
        name="spiral hook sits inside the bowl footprint",
    )
    ctx.expect_gap(
        hook,
        carriage,
        axis="z",
        positive_elem="hook_body",
        negative_elem="bowl_support",
        min_gap=0.030,
        name="hook clears the bowl support at rest",
    )

    carriage_upper = carriage_joint.motion_limits.upper
    head_upper = head_joint.motion_limits.upper
    lock_upper = lock_joint.motion_limits.upper

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_hook_pos = ctx.part_world_position(hook)
    rest_lock_pos = ctx.part_world_position(head_lock)

    with ctx.pose({carriage_joint: carriage_upper}):
        raised_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            base,
            carriage,
            axes="z",
            elem_a="left_column",
            elem_b="left_sleeve",
            min_overlap=0.09,
            name="left guide sleeve retains insertion when raised",
        )
        ctx.expect_overlap(
            base,
            carriage,
            axes="z",
            elem_a="right_column",
            elem_b="right_sleeve",
            min_overlap=0.09,
            name="right guide sleeve retains insertion when raised",
        )
        ctx.expect_gap(
            hook,
            carriage,
            axis="z",
            positive_elem="hook_body",
            negative_elem="bowl_support",
            min_gap=0.010,
            name="raised bowl still clears the hook from the support deck",
        )

    with ctx.pose({head_joint: head_upper}):
        opened_hook_pos = ctx.part_world_position(hook)
        ctx.expect_gap(
            hook,
            carriage,
            axis="z",
            positive_elem="hook_body",
            negative_elem="bowl_shell",
            min_gap=0.020,
            name="tilted head lifts the hook above the bowl rim",
        )

    with ctx.pose({lock_joint: lock_upper}):
        unlocked_lock_pos = ctx.part_world_position(head_lock)

    ctx.check(
        "bowl carriage lifts upward",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.020,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )
    ctx.check(
        "tilted head raises the hook",
        rest_hook_pos is not None
        and opened_hook_pos is not None
        and opened_hook_pos[2] > rest_hook_pos[2] + 0.090,
        details=f"rest={rest_hook_pos}, opened={opened_hook_pos}",
    )
    ctx.check(
        "head lock slider translates along the base",
        rest_lock_pos is not None
        and unlocked_lock_pos is not None
        and unlocked_lock_pos[0] > rest_lock_pos[0] + 0.010,
        details=f"rest={rest_lock_pos}, unlocked={unlocked_lock_pos}",
    )
    ctx.check(
        "speed selector stays mounted on the base",
        ctx.expect_contact(
            selector,
            base,
            elem_a="selector_dial",
            elem_b="selector_mount",
            name="selector dial contacts its base mount",
        ),
        details="selector dial should seat against the mount pad",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
