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
    *,
    z: float,
    x_shift: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_shift, y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _yz_section(
    width_y: float,
    height_z: float,
    radius: float,
    *,
    x: float,
    z_shift: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_shift)
        for y, z in rounded_rect_profile(
            width_y,
            height_z,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soft_stand_mixer")

    enamel = model.material("enamel", rgba=(0.92, 0.86, 0.72, 1.0))
    trim = model.material("trim", rgba=(0.72, 0.74, 0.78, 1.0))
    steel = model.material("steel", rgba=(0.80, 0.82, 0.84, 1.0))
    dark = model.material("dark", rgba=(0.20, 0.20, 0.22, 1.0))

    base = model.part("base")
    base_foot = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.36, 0.22, 0.055, corner_segments=8),
        0.044,
    )
    base.visual(
        mesh_from_geometry(base_foot, "base_foot"),
        material=enamel,
        name="base_foot",
    )

    base_column = section_loft(
        [
            _xy_section(0.17, 0.12, 0.040, z=0.036, x_shift=-0.072),
            _xy_section(0.14, 0.11, 0.038, z=0.15, x_shift=-0.068),
            _xy_section(0.11, 0.095, 0.032, z=0.255, x_shift=-0.070),
            _xy_section(0.074, 0.078, 0.024, z=0.300, x_shift=-0.090),
        ]
    )
    base.visual(
        mesh_from_geometry(base_column, "base_column"),
        material=enamel,
        name="column_shell",
    )
    base.visual(
        Box((0.18, 0.11, 0.016)),
        origin=Origin(xyz=(0.075, 0.0, 0.052)),
        material=enamel,
        name="track_deck",
    )
    base.visual(
        Box((0.060, 0.050, 0.028)),
        origin=Origin(xyz=(-0.108, 0.0, 0.314)),
        material=enamel,
        name="hinge_saddle",
    )
    base.visual(
        Box((0.014, 0.012, 0.048)),
        origin=Origin(xyz=(-0.118, -0.028, 0.328)),
        material=trim,
        name="left_hinge_rib",
    )
    base.visual(
        Box((0.014, 0.012, 0.048)),
        origin=Origin(xyz=(-0.118, 0.028, 0.328)),
        material=trim,
        name="right_hinge_rib",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(-0.118, -0.028, 0.370), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="left_hinge_lug",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(-0.118, 0.028, 0.370), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="right_hinge_lug",
    )
    base.visual(
        Box((0.028, 0.032, 0.028)),
        origin=Origin(xyz=(-0.144, 0.0, 0.162)),
        material=trim,
        name="lock_guide_block",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.22, 0.34)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.11, 0.085, 0.018)),
        origin=Origin(xyz=(0.055, 0.0, 0.009)),
        material=trim,
        name="carriage_runner",
    )
    carriage.visual(
        Box((0.085, 0.055, 0.030)),
        origin=Origin(xyz=(0.102, 0.0, 0.040), rpy=(0.0, -0.55, 0.0)),
        material=enamel,
        name="carriage_brace",
    )
    carriage.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.128, 0.0, 0.061)),
        material=enamel,
        name="carriage_pedestal",
    )
    carriage.visual(
        Cylinder(radius=0.069, length=0.014),
        origin=Origin(xyz=(0.145, 0.0, 0.092)),
        material=trim,
        name="carriage_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.22, 0.10, 0.11)),
        mass=2.3,
        origin=Origin(xyz=(0.105, 0.0, 0.055)),
    )

    bowl = model.part("bowl")
    bowl_outer = [
        (0.043, 0.000),
        (0.052, 0.008),
        (0.070, 0.030),
        (0.098, 0.082),
        (0.113, 0.122),
        (0.119, 0.136),
        (0.123, 0.139),
    ]
    bowl_inner = [
        (0.038, 0.010),
        (0.046, 0.017),
        (0.064, 0.036),
        (0.091, 0.086),
        (0.105, 0.124),
        (0.110, 0.131),
    ]
    bowl_shell = LatheGeometry.from_shell_profiles(
        bowl_outer,
        bowl_inner,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "mixer_bowl"),
        material=steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.123, length=0.139),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    head = model.part("head")
    head_shell = section_loft(
        [
            _yz_section(0.094, 0.112, 0.030, x=0.060, z_shift=0.018),
            _yz_section(0.124, 0.146, 0.045, x=0.155, z_shift=0.024),
            _yz_section(0.112, 0.122, 0.038, x=0.245, z_shift=0.010),
            _yz_section(0.074, 0.084, 0.026, x=0.335, z_shift=-0.004),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "mixer_head"),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="head_hinge_barrel",
    )
    head.visual(
        Box((0.056, 0.032, 0.036)),
        origin=Origin(xyz=(0.032, 0.0, 0.028)),
        material=enamel,
        name="rear_neck",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.032),
        origin=Origin(xyz=(0.245, 0.0, -0.052)),
        material=trim,
        name="drive_collar",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.31, 0.15, 0.16)),
        mass=7.0,
        origin=Origin(xyz=(0.145, 0.0, 0.018)),
    )

    hook = model.part("hook")
    hook_geom = CylinderGeometry(radius=0.008, height=0.036, radial_segments=24).translate(
        0.0,
        0.0,
        -0.018,
    )
    spiral_path = [(0.0, 0.0, -0.004), (0.009, 0.0, -0.014)]
    turns = 1.15
    steps = 24
    for i in range(steps + 1):
        t = i / steps
        angle = turns * math.tau * t
        radius = 0.016 + 0.009 * t
        z = -0.016 - 0.058 * t
        spiral_path.append((radius * math.cos(angle), radius * math.sin(angle), z))
    spiral_path.extend(
        [
            (0.016, -0.012, -0.076),
            (0.009, -0.008, -0.086),
            (0.002, -0.003, -0.092),
        ]
    )
    hook_geom.merge(
        tube_from_spline_points(
            spiral_path,
            radius=0.006,
            samples_per_segment=10,
            radial_segments=18,
        )
    )
    hook.visual(
        mesh_from_geometry(hook_geom, "spiral_hook"),
        material=dark,
        name="hook_assembly",
    )
    hook.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.18),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=trim,
        name="selector_body",
    )
    speed_selector.visual(
        Box((0.036, 0.010, 0.006)),
        origin=Origin(xyz=(0.018, 0.0, 0.012)),
        material=dark,
        name="selector_paddle",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.042, 0.042, 0.022)),
        mass=0.08,
        origin=Origin(xyz=(0.010, 0.0, 0.010)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.014, 0.020, 0.008)),
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
        material=dark,
        name="lock_slider",
    )
    head_lock.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(-0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lock_knob",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.040, 0.024, 0.024)),
        mass=0.05,
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.025, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.10,
            lower=0.0,
            upper=0.050,
        ),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.145, 0.0, 0.099)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.118, 0.0, 0.370)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_hook",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=hook,
        origin=Origin(xyz=(0.245, 0.0, -0.068)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=25.0),
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(0.030, 0.072, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=2.4,
        ),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.158, 0.0, 0.162)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.015,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    hook = object_model.get_part("hook")
    speed_selector = object_model.get_part("speed_selector")
    head_lock = object_model.get_part("head_lock")

    carriage_joint = object_model.get_articulation("base_to_carriage")
    bowl_joint = object_model.get_articulation("carriage_to_bowl")
    head_joint = object_model.get_articulation("base_to_head")
    hook_joint = object_model.get_articulation("head_to_hook")
    selector_joint = object_model.get_articulation("base_to_speed_selector")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "bowl carriage is prismatic",
        carriage_joint.articulation_type == ArticulationType.PRISMATIC,
        details=str(carriage_joint.articulation_type),
    )
    ctx.check(
        "bowl mount stays fixed on carriage",
        bowl_joint.articulation_type == ArticulationType.FIXED,
        details=str(bowl_joint.articulation_type),
    )
    ctx.check(
        "head tilt is rear revolute hinge",
        head_joint.articulation_type == ArticulationType.REVOLUTE
        and head_joint.motion_limits is not None
        and head_joint.motion_limits.upper is not None
        and head_joint.motion_limits.upper > math.radians(45.0),
        details=str(head_joint.motion_limits),
    )
    ctx.check(
        "hook spins continuously",
        hook_joint.articulation_type == ArticulationType.CONTINUOUS
        and hook_joint.motion_limits is not None
        and hook_joint.motion_limits.lower is None
        and hook_joint.motion_limits.upper is None,
        details=str(hook_joint.motion_limits),
    )
    ctx.check(
        "speed selector rotates on revolute joint",
        selector_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(selector_joint.articulation_type),
    )
    ctx.check(
        "head lock slides prismatically",
        lock_joint.articulation_type == ArticulationType.PRISMATIC,
        details=str(lock_joint.articulation_type),
    )

    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="carriage_runner",
        elem_b="track_deck",
        min_overlap=0.10,
        name="carriage runner overlaps track at rest",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="y",
        inner_elem="carriage_runner",
        outer_elem="track_deck",
        margin=0.001,
        name="carriage runner stays within track width",
    )
    ctx.expect_overlap(
        hook,
        bowl,
        axes="xy",
        elem_a="hook_assembly",
        elem_b="bowl_shell",
        min_overlap=0.04,
        name="hook sits over bowl footprint",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        positive_elem="head_shell",
        negative_elem="bowl_shell",
        min_gap=0.02,
        name="head shell clears bowl rim",
    )

    rest_bowl_pos = ctx.part_world_position(bowl)
    carriage_upper = (
        carriage_joint.motion_limits.upper if carriage_joint.motion_limits is not None else None
    )
    if carriage_upper is not None:
        with ctx.pose({carriage_joint: carriage_upper}):
            ctx.expect_overlap(
                carriage,
                base,
                axes="x",
                elem_a="carriage_runner",
                elem_b="track_deck",
                min_overlap=0.06,
                name="carriage retains insertion when extended",
            )
            ctx.expect_within(
                carriage,
                base,
                axes="y",
                inner_elem="carriage_runner",
                outer_elem="track_deck",
                margin=0.001,
                name="extended carriage stays on track width",
            )
            extended_bowl_pos = ctx.part_world_position(bowl)
        ctx.check(
            "bowl carriage extends forward",
            rest_bowl_pos is not None
            and extended_bowl_pos is not None
            and extended_bowl_pos[0] > rest_bowl_pos[0] + 0.045,
            details=f"rest={rest_bowl_pos}, extended={extended_bowl_pos}",
        )

    rest_hook_pos = ctx.part_world_position(hook)
    head_upper = head_joint.motion_limits.upper if head_joint.motion_limits is not None else None
    if head_upper is not None:
        with ctx.pose({head_joint: head_upper}):
            open_hook_pos = ctx.part_world_position(hook)
        ctx.check(
            "tilting head raises hook",
            rest_hook_pos is not None
            and open_hook_pos is not None
            and open_hook_pos[2] > rest_hook_pos[2] + 0.06,
            details=f"rest={rest_hook_pos}, open={open_hook_pos}",
        )

    selector_rest = _aabb_center(ctx.part_element_world_aabb(speed_selector, elem="selector_paddle"))
    selector_upper = (
        selector_joint.motion_limits.upper if selector_joint.motion_limits is not None else None
    )
    if selector_upper is not None:
        with ctx.pose({selector_joint: selector_upper}):
            selector_turned = _aabb_center(
                ctx.part_element_world_aabb(speed_selector, elem="selector_paddle")
            )
        ctx.check(
            "speed selector paddle rotates visibly",
            selector_rest is not None
            and selector_turned is not None
            and (
                abs(selector_turned[0] - selector_rest[0]) > 0.01
                or abs(selector_turned[1] - selector_rest[1]) > 0.01
            ),
            details=f"rest={selector_rest}, turned={selector_turned}",
        )

    lock_rest = ctx.part_world_position(head_lock)
    lock_upper = lock_joint.motion_limits.upper if lock_joint.motion_limits is not None else None
    if lock_upper is not None:
        with ctx.pose({lock_joint: lock_upper}):
            lock_pulled = ctx.part_world_position(head_lock)
        ctx.check(
            "head lock pulls rearward",
            lock_rest is not None
            and lock_pulled is not None
            and lock_pulled[0] < lock_rest[0] - 0.010,
            details=f"rest={lock_rest}, pulled={lock_pulled}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
