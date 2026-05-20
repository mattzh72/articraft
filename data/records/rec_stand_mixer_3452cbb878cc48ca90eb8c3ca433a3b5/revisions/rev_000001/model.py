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
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _build_whisk_geometry() -> object:
    whisk_geom = CylinderGeometry(radius=0.0048, height=0.026).translate(0.0, 0.0, -0.013)
    whisk_geom.merge(CylinderGeometry(radius=0.0080, height=0.024).translate(0.0, 0.0, -0.028))
    whisk_geom.merge(CylinderGeometry(radius=0.0120, height=0.018).translate(0.0, 0.0, -0.046))
    whisk_geom.merge(CylinderGeometry(radius=0.0100, height=0.016).translate(0.0, 0.0, -0.141))

    loop_count = 8
    for index in range(loop_count):
        angle = (math.pi * index) / loop_count
        c = math.cos(angle)
        s = math.sin(angle)
        wire = tube_from_spline_points(
            [
                (0.010 * c, 0.010 * s, -0.042),
                (0.018 * c, 0.018 * s, -0.060),
                (0.028 * c, 0.028 * s, -0.086),
                (0.036 * c, 0.036 * s, -0.112),
                (0.0, 0.0, -0.141),
                (-0.036 * c, -0.036 * s, -0.112),
                (-0.028 * c, -0.028 * s, -0.086),
                (-0.018 * c, -0.018 * s, -0.060),
                (-0.010 * c, -0.010 * s, -0.042),
            ],
            radius=0.0015,
            samples_per_segment=14,
            radial_segments=12,
            cap_ends=True,
        )
        whisk_geom.merge(wire)

    return whisk_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bakery_corner_stand_mixer")

    painted_body = model.material("painted_body", rgba=(0.78, 0.16, 0.14, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.86, 0.87, 0.89, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    base = model.part("base")

    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.34, 0.23, 0.045, corner_segments=10), 0.044),
        "mixer_base_plate",
    )
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.060, 0.0, 0.022)),
        material=painted_body,
        name="base_plate",
    )

    pedestal_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.128, 0.158, 0.030, 0.034, x_shift=-0.040),
                _xy_section(0.112, 0.138, 0.028, 0.180, x_shift=-0.046),
                _xy_section(0.090, 0.114, 0.024, 0.276, x_shift=-0.054),
                _xy_section(0.074, 0.098, 0.020, 0.318, x_shift=-0.058),
            ]
        ),
        "mixer_pedestal_shell",
    )
    base.visual(pedestal_mesh, material=painted_body, name="pedestal_shell")
    base.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(-0.048, 0.041, 0.352), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_body,
        name="left_hinge_ear",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(-0.048, -0.041, 0.352), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_body,
        name="right_hinge_ear",
    )
    base.visual(
        Box((0.034, 0.024, 0.028)),
        origin=Origin(xyz=(-0.050, 0.041, 0.330)),
        material=painted_body,
        name="left_hinge_pedestal",
    )
    base.visual(
        Box((0.034, 0.024, 0.028)),
        origin=Origin(xyz=(-0.050, -0.041, 0.330)),
        material=painted_body,
        name="right_hinge_pedestal",
    )
    base.visual(
        Box((0.110, 0.024, 0.028)),
        origin=Origin(xyz=(0.072, 0.044, 0.052)),
        material=dark_trim,
        name="left_guide_rail",
    )
    base.visual(
        Box((0.110, 0.024, 0.028)),
        origin=Origin(xyz=(0.072, -0.044, 0.052)),
        material=dark_trim,
        name="right_guide_rail",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.010, -0.072, 0.168), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="speed_socket",
    )
    base.visual(
        Box((0.026, 0.024, 0.038)),
        origin=Origin(xyz=(-0.082, -0.069, 0.254)),
        material=dark_trim,
        name="head_lock_mount",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.23, 0.37)),
        mass=11.5,
        origin=Origin(xyz=(0.040, 0.0, 0.185)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.068, 0.074, 0.018)),
        origin=Origin(xyz=(0.026, 0.0, 0.050)),
        material=dark_trim,
        name="carriage_block",
    )
    bowl_carriage.visual(
        Box((0.026, 0.018, 0.046)),
        origin=Origin(xyz=(0.012, 0.028, 0.064)),
        material=dark_trim,
        name="left_carriage_cheek",
    )
    bowl_carriage.visual(
        Box((0.026, 0.018, 0.046)),
        origin=Origin(xyz=(0.012, -0.028, 0.064)),
        material=dark_trim,
        name="right_carriage_cheek",
    )
    bowl_carriage.visual(
        Box((0.120, 0.034, 0.020)),
        origin=Origin(xyz=(0.060, 0.0, 0.060)),
        material=dark_trim,
        name="support_arm",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.112, 0.0, 0.072)),
        material=satin_steel,
        name="bowl_platter",
    )
    bowl_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.022, 0.000),
                (0.042, 0.006),
                (0.079, 0.030),
                (0.105, 0.073),
                (0.118, 0.116),
                (0.115, 0.136),
            ],
            [
                (0.000, 0.004),
                (0.034, 0.012),
                (0.072, 0.034),
                (0.099, 0.076),
                (0.111, 0.129),
            ],
            segments=64,
            end_cap="round",
            lip_samples=10,
        ),
        "mixing_bowl_shell",
    )
    bowl_carriage.visual(
        bowl_mesh,
        origin=Origin(xyz=(0.112, 0.0, 0.080)),
        material=steel,
        name="bowl_shell",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.112, 0.0, 0.082)),
        material=satin_steel,
        name="bowl_foot",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.150),
        mass=1.8,
        origin=Origin(xyz=(0.112, 0.0, 0.116)),
    )

    head = model.part("head")
    head_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.094, 0.084, 0.022, 0.060, z_shift=0.022),
                _yz_section(0.154, 0.154, 0.040, 0.154, z_shift=0.004),
                _yz_section(0.148, 0.142, 0.036, 0.238, z_shift=0.000),
                _yz_section(0.108, 0.092, 0.026, 0.302, z_shift=0.008),
            ]
        ),
        "tilt_head_shell",
    )
    head.visual(head_mesh, material=painted_body, name="head_shell")
    head.visual(
        Cylinder(radius=0.024, length=0.058),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_body,
        name="rear_hinge_lug",
    )
    head.visual(
        Box((0.080, 0.050, 0.018)),
        origin=Origin(xyz=(0.040, 0.0, 0.028)),
        material=painted_body,
        name="upper_hinge_bridge",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.056),
        origin=Origin(xyz=(0.220, 0.0, -0.088)),
        material=satin_steel,
        name="planetary_socket",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.29, 0.16, 0.18)),
        mass=5.0,
        origin=Origin(xyz=(0.145, 0.0, -0.030)),
    )

    whisk = model.part("whisk")
    whisk.visual(
        mesh_from_geometry(_build_whisk_geometry(), "wire_whisk"),
        material=steel,
        name="whisk_assembly",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.200),
        mass=0.28,
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="speed_pivot",
    )
    speed_control.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="speed_knob",
    )
    speed_control.visual(
        Box((0.020, 0.008, 0.010)),
        origin=Origin(xyz=(0.012, -0.022, 0.006)),
        material=dark_trim,
        name="speed_grip",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.032, 0.026, 0.018)),
        mass=0.06,
        origin=Origin(xyz=(0.008, -0.016, 0.004)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
        material=dark_trim,
        name="lock_button",
    )
    head_lock.visual(
        Box((0.010, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, 0.001, 0.0)),
        material=dark_trim,
        name="lock_stem",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.014)),
        mass=0.03,
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.060, 0.0, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.10, lower=0.0, upper=0.045),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.048, 0.0, 0.352)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.220, 0.0, -0.116)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9.0, velocity=26.0),
    )
    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(0.010, -0.081, 0.168)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.5,
            lower=math.radians(-18.0),
            upper=math.radians(34.0),
        ),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.082, -0.081, 0.254)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.006),
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

    bowl_joint = object_model.get_articulation("base_to_bowl")
    head_joint = object_model.get_articulation("base_to_head")
    whisk_joint = object_model.get_articulation("head_to_whisk")
    speed_joint = object_model.get_articulation("base_to_speed_control")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    ctx.check(
        "bowl carriage uses a forward slide",
        bowl_joint.articulation_type == ArticulationType.PRISMATIC and bowl_joint.axis == (1.0, 0.0, 0.0),
        details=f"type={bowl_joint.articulation_type}, axis={bowl_joint.axis}",
    )
    ctx.check(
        "head uses a rear tilt hinge",
        head_joint.articulation_type == ArticulationType.REVOLUTE and head_joint.axis == (0.0, -1.0, 0.0),
        details=f"type={head_joint.articulation_type}, axis={head_joint.axis}",
    )
    ctx.check(
        "whisk spins continuously beneath the head",
        whisk_joint.articulation_type == ArticulationType.CONTINUOUS and whisk_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={whisk_joint.articulation_type}, axis={whisk_joint.axis}",
    )
    ctx.check(
        "speed control is a small rotary control",
        speed_joint.articulation_type == ArticulationType.REVOLUTE and speed_joint.axis == (0.0, -1.0, 0.0),
        details=f"type={speed_joint.articulation_type}, axis={speed_joint.axis}",
    )
    ctx.check(
        "head lock is a short sliding control",
        lock_joint.articulation_type == ArticulationType.PRISMATIC
        and lock_joint.axis == (0.0, -1.0, 0.0)
        and lock_joint.motion_limits is not None
        and lock_joint.motion_limits.upper is not None
        and lock_joint.motion_limits.upper <= 0.010,
        details=f"type={lock_joint.articulation_type}, axis={lock_joint.axis}, limits={lock_joint.motion_limits}",
    )

    ctx.expect_contact(whisk, head, name="whisk remains mounted to the planetary socket")
    ctx.expect_contact(head, base, name="tilt head remains captured by the rear hinge")
    ctx.expect_contact(bowl_carriage, base, name="bowl carriage stays seated on the base guides")
    ctx.allow_overlap(
        base,
        head_lock,
        elem_a="head_lock_mount",
        elem_b="lock_stem",
        reason="The lock stem is intentionally represented as sliding inside a simplified solid guide block.",
    )
    ctx.expect_within(
        head_lock,
        base,
        axes="xz",
        inner_elem="lock_stem",
        outer_elem="head_lock_mount",
        margin=0.0,
        name="head lock stem stays aligned inside the guide in section",
    )

    rest_bowl_pos = ctx.part_world_position(bowl_carriage)
    rest_whisk_pos = ctx.part_world_position(whisk)
    rest_speed_center = _aabb_center(ctx.part_element_world_aabb(speed_control, elem="speed_grip"))
    rest_lock_center = _aabb_center(ctx.part_element_world_aabb(head_lock, elem="lock_button"))

    bowl_upper = bowl_joint.motion_limits.upper if bowl_joint.motion_limits is not None else None
    head_upper = head_joint.motion_limits.upper if head_joint.motion_limits is not None else None
    speed_upper = speed_joint.motion_limits.upper if speed_joint.motion_limits is not None else None
    lock_upper = lock_joint.motion_limits.upper if lock_joint.motion_limits is not None else None

    with ctx.pose({bowl_joint: bowl_upper or 0.0}):
        extended_bowl_pos = ctx.part_world_position(bowl_carriage)
        ctx.expect_contact(bowl_carriage, base, name="extended bowl carriage stays guided by the rails")

    ctx.check(
        "bowl carriage advances toward the operator",
        rest_bowl_pos is not None and extended_bowl_pos is not None and extended_bowl_pos[0] > rest_bowl_pos[0] + 0.03,
        details=f"rest={rest_bowl_pos}, extended={extended_bowl_pos}",
    )

    with ctx.pose({head_joint: head_upper or 0.0}):
        tilted_whisk_pos = ctx.part_world_position(whisk)
        ctx.expect_contact(whisk, head, name="tilted head keeps the whisk attached")
        ctx.expect_contact(head, base, name="tilted head still bears on the hinge hardware")

    ctx.check(
        "tilt head lifts the whisk clear of the bowl",
        rest_whisk_pos is not None and tilted_whisk_pos is not None and tilted_whisk_pos[2] > rest_whisk_pos[2] + 0.20,
        details=f"rest={rest_whisk_pos}, tilted={tilted_whisk_pos}",
    )

    with ctx.pose({speed_joint: speed_upper or 0.0}):
        rotated_speed_center = _aabb_center(ctx.part_element_world_aabb(speed_control, elem="speed_grip"))

    ctx.check(
        "speed control grip rotates upward through its range",
        rest_speed_center is not None
        and rotated_speed_center is not None
        and rotated_speed_center[2] > rest_speed_center[2] + 0.004,
        details=f"rest={rest_speed_center}, rotated={rotated_speed_center}",
    )

    with ctx.pose({lock_joint: lock_upper or 0.0}):
        slid_lock_center = _aabb_center(ctx.part_element_world_aabb(head_lock, elem="lock_button"))
        ctx.expect_within(
            head_lock,
            base,
            axes="xz",
            inner_elem="lock_stem",
            outer_elem="head_lock_mount",
            margin=0.0,
            name="extended head lock stem stays aligned in the guide in section",
        )
        ctx.expect_overlap(
            head_lock,
            base,
            axes="y",
            elem_a="lock_stem",
            elem_b="head_lock_mount",
            min_overlap=0.002,
            name="extended head lock retains insertion in the guide",
        )

    ctx.check(
        "head lock button slides rearward on a short stroke",
        rest_lock_center is not None
        and slid_lock_center is not None
        and slid_lock_center[1] < rest_lock_center[1] - 0.004,
        details=f"rest={rest_lock_center}, slid={slid_lock_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
