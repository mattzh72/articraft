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
    *,
    x_pos: float,
    width_y: float,
    height_z: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_center + z_pos)
        for (y_pos, z_pos) in rounded_rect_profile(width_y, height_z, radius)
    ]


def _base_shell_mesh():
    return section_loft(
        [
            _yz_section(x_pos=-0.130, width_y=0.124, height_z=0.236, radius=0.030, z_center=0.118),
            _yz_section(x_pos=-0.070, width_y=0.126, height_z=0.272, radius=0.036, z_center=0.136),
            _yz_section(x_pos=-0.015, width_y=0.112, height_z=0.156, radius=0.028, z_center=0.078),
            _yz_section(x_pos=0.055, width_y=0.166, height_z=0.070, radius=0.026, z_center=0.035),
            _yz_section(x_pos=0.110, width_y=0.190, height_z=0.038, radius=0.016, z_center=0.019),
        ]
    )


def _head_shell_mesh():
    return section_loft(
        [
            _yz_section(x_pos=0.036, width_y=0.050, height_z=0.046, radius=0.016, z_center=0.030),
            _yz_section(x_pos=0.096, width_y=0.110, height_z=0.082, radius=0.030, z_center=0.028),
            _yz_section(x_pos=0.170, width_y=0.152, height_z=0.104, radius=0.044, z_center=0.018),
            _yz_section(x_pos=0.245, width_y=0.132, height_z=0.086, radius=0.034, z_center=0.006),
            _yz_section(x_pos=0.300, width_y=0.088, height_z=0.060, radius=0.020, z_center=-0.006),
        ]
    )


def _bowl_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.026, 0.000),
            (0.054, 0.010),
            (0.086, 0.040),
            (0.104, 0.090),
            (0.110, 0.118),
        ],
        [
            (0.000, 0.004),
            (0.040, 0.012),
            (0.076, 0.040),
            (0.096, 0.088),
            (0.102, 0.114),
        ],
        segments=56,
    )


def _dough_hook_curve_mesh():
    return tube_from_spline_points(
        [
            (0.000, 0.000, -0.044),
            (0.010, 0.000, -0.058),
            (0.024, 0.000, -0.078),
            (0.031, 0.000, -0.100),
            (0.026, 0.000, -0.116),
            (0.014, 0.000, -0.126),
            (-0.002, 0.000, -0.132),
        ],
        radius=0.0068,
        samples_per_segment=18,
        radial_segments=22,
        cap_ends=True,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_stand_mixer")

    shell_white = model.material("shell_white", rgba=(0.95, 0.95, 0.93, 1.0))
    steel = model.material("steel", rgba=(0.84, 0.86, 0.88, 1.0))
    dark = model.material("dark", rgba=(0.17, 0.18, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_base_shell_mesh(), "base_shell"),
        material=shell_white,
        name="base_shell",
    )
    base.visual(
        Box((0.010, 0.080, 0.078)),
        origin=Origin(xyz=(0.105, 0.0, 0.039)),
        material=shell_white,
        name="carriage_guide",
    )
    base.visual(
        Box((0.032, 0.070, 0.012)),
        origin=Origin(xyz=(-0.094, 0.0, 0.283)),
        material=shell_white,
        name="head_saddle",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(-0.010, 0.087, 0.122), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="selector_pod",
    )
    base.visual(
        Box((0.030, 0.036, 0.020)),
        origin=Origin(xyz=(-0.022, 0.060, 0.122)),
        material=dark,
        name="selector_stem",
    )
    base.visual(
        Box((0.016, 0.014, 0.020)),
        origin=Origin(xyz=(-0.118, -0.056, 0.214)),
        material=dark,
        name="lock_guide",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.30)),
        mass=9.0,
        origin=Origin(xyz=(-0.010, 0.0, 0.150)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.024, 0.096, 0.082)),
        origin=Origin(xyz=(0.012, 0.0, 0.041)),
        material=shell_white,
        name="carriage_spine",
    )
    bowl_carriage.visual(
        Box((0.036, 0.056, 0.020)),
        origin=Origin(xyz=(0.042, 0.0, 0.030)),
        material=shell_white,
        name="carriage_arm",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.060, 0.0, 0.038)),
        material=shell_white,
        name="carriage_post",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.060, 0.0, 0.064)),
        material=shell_white,
        name="bowl_mount",
    )
    bowl_carriage.visual(
        mesh_from_geometry(_bowl_shell_mesh(), "mixing_bowl"),
        origin=Origin(xyz=(0.060, 0.0, 0.068)),
        material=steel,
        name="bowl_shell",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.24, 0.22, 0.20)),
        mass=2.1,
        origin=Origin(xyz=(0.060, 0.0, 0.110)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_head_shell_mesh(), "head_shell"),
        material=shell_white,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.062),
        origin=Origin(xyz=(0.006, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_white,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.026, 0.050, 0.024)),
        origin=Origin(xyz=(0.027, 0.0, 0.022)),
        material=shell_white,
        name="hinge_fairing",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.056),
        origin=Origin(xyz=(0.258, 0.0, -0.022)),
        material=shell_white,
        name="tool_socket",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.30, 0.17, 0.14)),
        mass=4.8,
        origin=Origin(xyz=(0.165, 0.0, 0.012)),
    )

    dough_hook = model.part("dough_hook")
    dough_hook.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=steel,
        name="hook_shaft",
    )
    dough_hook.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=steel,
        name="hook_collar",
    )
    dough_hook.visual(
        mesh_from_geometry(_dough_hook_curve_mesh(), "dough_hook_curve"),
        material=steel,
        name="hook_curve",
    )
    dough_hook.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.160),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="selector_axle",
    )
    speed_selector.visual(
        Box((0.034, 0.010, 0.012)),
        origin=Origin(xyz=(0.019, 0.008, 0.002)),
        material=dark,
        name="selector_paddle",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.040, 0.024, 0.016)),
        mass=0.08,
        origin=Origin(xyz=(0.018, 0.006, 0.001)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.028, 0.012, 0.012)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=dark,
        name="lock_slider",
    )
    head_lock.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=dark,
        name="lock_tab",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.030, 0.016, 0.020)),
        mass=0.07,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.110, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.12,
            lower=0.0,
            upper=0.024,
        ),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.100, 0.0, 0.305)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "head_to_dough_hook",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=dough_hook,
        origin=Origin(xyz=(0.258, 0.0, -0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=18.0),
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(-0.010, 0.105, 0.122)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=0.50,
        ),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.110, -0.056, 0.214)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.04,
            lower=0.0,
            upper=0.012,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl_carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    dough_hook = object_model.get_part("dough_hook")
    speed_selector = object_model.get_part("speed_selector")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    selector_turn = object_model.get_articulation("base_to_speed_selector")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    ctx.expect_gap(
        head,
        bowl_carriage,
        axis="z",
        min_gap=0.030,
        max_gap=0.080,
        positive_elem="head_shell",
        negative_elem="bowl_shell",
        name="head clears the bowl rim at rest",
    )
    ctx.expect_contact(
        bowl_carriage,
        base,
        elem_a="carriage_spine",
        elem_b="carriage_guide",
        name="bowl carriage rides on the front guide",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="hinge_barrel",
        elem_b="head_saddle",
        name="tilt head rests on the rear saddle",
    )
    ctx.expect_contact(
        dough_hook,
        head,
        elem_a="hook_shaft",
        elem_b="tool_socket",
        name="dough hook seats in the drive socket",
    )
    ctx.expect_contact(
        speed_selector,
        base,
        elem_a="selector_axle",
        elem_b="selector_pod",
        name="speed selector is mounted on the side pod",
    )
    ctx.expect_contact(
        head_lock,
        base,
        elem_a="lock_slider",
        elem_b="lock_guide",
        name="head lock starts seated in its guide",
    )

    carriage_rest = ctx.part_world_position(bowl_carriage)
    with ctx.pose({bowl_slide: 0.024}):
        carriage_raised = ctx.part_world_position(bowl_carriage)
        ctx.expect_contact(
            bowl_carriage,
            base,
            elem_a="carriage_spine",
            elem_b="carriage_guide",
            name="raised bowl carriage remains captured by the guide",
        )
    ctx.check(
        "bowl carriage raises upward",
        carriage_rest is not None
        and carriage_raised is not None
        and carriage_raised[2] > carriage_rest[2] + 0.018,
        details=f"rest={carriage_rest}, raised={carriage_raised}",
    )

    hook_rest = ctx.part_world_position(dough_hook)
    with ctx.pose({head_tilt: math.radians(55.0)}):
        hook_open = ctx.part_world_position(dough_hook)
        ctx.expect_gap(
            dough_hook,
            bowl_carriage,
            axis="z",
            min_gap=0.025,
            positive_elem="hook_curve",
            negative_elem="bowl_shell",
            name="tilted head lifts the dough hook clear of the bowl",
        )
    ctx.check(
        "head tilts upward and back",
        hook_rest is not None
        and hook_open is not None
        and hook_open[2] > hook_rest[2] + 0.10
        and hook_open[0] < hook_rest[0] - 0.025,
        details=f"hook_rest={hook_rest}, hook_open={hook_open}",
    )

    selector_rest = _aabb_center(ctx.part_element_world_aabb(speed_selector, elem="selector_paddle"))
    with ctx.pose({selector_turn: 0.40}):
        selector_open = _aabb_center(ctx.part_element_world_aabb(speed_selector, elem="selector_paddle"))
    ctx.check(
        "speed selector swings upward",
        selector_rest is not None
        and selector_open is not None
        and selector_open[2] > selector_rest[2] + 0.006,
        details=f"selector_rest={selector_rest}, selector_open={selector_open}",
    )

    lock_rest = ctx.part_world_position(head_lock)
    with ctx.pose({lock_slide: 0.012}):
        lock_open = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock slides forward",
        lock_rest is not None
        and lock_open is not None
        and lock_open[0] > lock_rest[0] + 0.010,
        details=f"lock_rest={lock_rest}, lock_open={lock_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
