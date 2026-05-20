from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_BEARING_PLANE_Z = 0.0
RAM_MOUNT_Z = -0.111
RAM_STROKE = 0.45


def _box_at(
    size: tuple[float, float, float],
    *,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    fillet: float | None = None,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if fillet is not None and fillet > 0.0:
        shape = shape.edges("|Z").fillet(fillet)
    return shape.translate(center)


def _cylinder_at(
    radius: float,
    length: float,
    *,
    z_min: float,
    inner_radius: float | None = None,
) -> cq.Workplane:
    profile = cq.Workplane("XY").circle(radius)
    if inner_radius is not None:
        profile = profile.circle(inner_radius)
    return profile.extrude(length).translate((0.0, 0.0, z_min))


def _make_top_support_shape() -> cq.Workplane:
    bearing_ring = _cylinder_at(0.160, 0.030, z_min=SUPPORT_BEARING_PLANE_Z, inner_radius=0.095)
    drive_housing = _cylinder_at(0.100, 0.090, z_min=0.030)
    left_beam = _box_at((0.620, 0.090, 0.100), center=(0.0, 0.140, 0.095), fillet=0.008)
    right_beam = _box_at((0.620, 0.090, 0.100), center=(0.0, -0.140, 0.095), fillet=0.008)
    front_brace = _box_at((0.060, 0.300, 0.070), center=(0.280, 0.0, 0.085), fillet=0.006)
    rear_brace = _box_at((0.060, 0.300, 0.070), center=(-0.280, 0.0, 0.085), fillet=0.006)
    top_plate = _box_at((0.720, 0.420, 0.030), center=(0.0, 0.0, 0.160), fillet=0.012)
    center_cap = _box_at((0.180, 0.130, 0.022), center=(0.0, 0.0, 0.186), fillet=0.006)

    support = (
        bearing_ring.union(drive_housing)
        .union(left_beam)
        .union(right_beam)
        .union(front_brace)
        .union(rear_brace)
        .union(top_plate)
        .union(center_cap)
    )
    return support


def _make_rotary_platform_shape() -> cq.Workplane:
    thrust_flange = _cylinder_at(0.155, 0.014, z_min=-0.014, inner_radius=0.078)
    deck = _cylinder_at(0.215, 0.028, z_min=-0.042)
    central_housing = _cylinder_at(0.090, 0.055, z_min=-0.097)
    ram_mount = _cylinder_at(0.105, 0.014, z_min=RAM_MOUNT_Z)
    service_lug = _box_at((0.090, 0.050, 0.022), center=(0.158, 0.0, -0.025), fillet=0.004)

    platform = thrust_flange.union(deck).union(central_housing).union(ram_mount).union(service_lug)

    rib_seed = _box_at((0.160, 0.045, 0.055), center=(0.100, 0.0, -0.0695), fillet=0.003)
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        rib = rib_seed.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        platform = platform.union(rib)

    return platform


def _make_vertical_ram_shape() -> cq.Workplane:
    top_head = _cylinder_at(0.082, 0.014, z_min=-0.014)
    neck = _cylinder_at(0.055, 0.070, z_min=-0.084)
    column = _cylinder_at(0.045, 0.280, z_min=-0.364)
    lower_body = _cylinder_at(0.058, 0.110, z_min=-0.474)
    saddle = _box_at((0.180, 0.120, 0.020), center=(0.0, 0.0, -0.484), fillet=0.010)
    ear_left = _box_at((0.025, 0.100, 0.080), center=(0.045, 0.0, -0.434), fillet=0.004)
    ear_right = _box_at((0.025, 0.100, 0.080), center=(-0.045, 0.0, -0.434), fillet=0.004)
    cross_web = _box_at((0.100, 0.025, 0.080), center=(0.0, 0.0, -0.434), fillet=0.004)

    ram = (
        top_head.union(neck)
        .union(column)
        .union(lower_body)
        .union(saddle)
        .union(ear_left)
        .union(ear_right)
        .union(cross_web)
    )
    return ram


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_rotary_lift")

    model.material("support_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("platform_steel", rgba=(0.47, 0.49, 0.52, 1.0))
    model.material("ram_steel", rgba=(0.58, 0.60, 0.63, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        mesh_from_cadquery(_make_top_support_shape(), "top_support"),
        material="support_steel",
        name="support_shell",
    )

    rotary_platform = model.part("rotary_platform")
    rotary_platform.visual(
        mesh_from_cadquery(_make_rotary_platform_shape(), "rotary_platform"),
        material="platform_steel",
        name="platform_shell",
    )

    vertical_ram = model.part("vertical_ram")
    vertical_ram.visual(
        mesh_from_cadquery(_make_vertical_ram_shape(), "vertical_ram"),
        material="ram_steel",
        name="ram_shell",
    )

    model.articulation(
        "support_to_platform",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=rotary_platform,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_BEARING_PLANE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=120.0,
            velocity=1.0,
        ),
    )
    model.articulation(
        "platform_to_ram",
        ArticulationType.PRISMATIC,
        parent=rotary_platform,
        child=vertical_ram,
        origin=Origin(xyz=(0.0, 0.0, RAM_MOUNT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=RAM_STROKE,
            effort=2500.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    rotary_platform = object_model.get_part("rotary_platform")
    vertical_ram = object_model.get_part("vertical_ram")
    platform_joint = object_model.get_articulation("support_to_platform")
    ram_joint = object_model.get_articulation("platform_to_ram")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "platform_joint_is_vertical_revolute",
        platform_joint.joint_type == ArticulationType.REVOLUTE and tuple(platform_joint.axis) == (0.0, 0.0, 1.0),
        details=f"joint_type={platform_joint.joint_type}, axis={platform_joint.axis}",
    )
    ctx.check(
        "ram_joint_is_downward_prismatic",
        ram_joint.joint_type == ArticulationType.PRISMATIC and tuple(ram_joint.axis) == (0.0, 0.0, -1.0),
        details=f"joint_type={ram_joint.joint_type}, axis={ram_joint.axis}",
    )

    ctx.expect_contact(
        rotary_platform,
        top_support,
        name="rotary_platform_is_carried_by_top_support",
    )
    ctx.expect_overlap(
        rotary_platform,
        top_support,
        axes="xy",
        min_overlap=0.28,
        name="platform_bearing_stack_has_plan_overlap",
    )
    ctx.expect_contact(
        vertical_ram,
        rotary_platform,
        name="ram_hangs_from_platform_at_retracted_pose",
    )

    with ctx.pose({platform_joint: 1.2}):
        ctx.expect_contact(
            rotary_platform,
            top_support,
            name="platform_remains_supported_while_rotated",
        )

    ram_rest_pos = ctx.part_world_position(vertical_ram)
    with ctx.pose({ram_joint: RAM_STROKE}):
        ram_extended_pos = ctx.part_world_position(vertical_ram)
        ctx.expect_gap(
            rotary_platform,
            vertical_ram,
            axis="z",
            min_gap=RAM_STROKE - 0.01,
            max_gap=RAM_STROKE + 0.01,
            name="ram_opens_a_vertical_gap_when_extended",
        )

    ctx.check(
        "positive_prismatic_motion_moves_ram_downward",
        ram_rest_pos is not None
        and ram_extended_pos is not None
        and ram_extended_pos[2] < ram_rest_pos[2] - (RAM_STROKE - 0.01),
        details=f"rest={ram_rest_pos}, extended={ram_extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
