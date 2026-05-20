from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    CapsuleGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

WHEEL_CENTER_Z = 1.52
WHEEL_RADIUS = 1.05
RIM_TUBE_RADIUS = 0.028
HUB_RADIUS = 0.18
HUB_LENGTH = 0.24
HUB_FLANGE_RADIUS = 0.23
HUB_FLANGE_LENGTH = 0.04
SPOKE_RADIUS = 0.012
SPOKE_COUNT = 16

CABIN_COUNT = 8
PIVOT_RADIUS = WHEEL_RADIUS + RIM_TUBE_RADIUS + 0.045
MOUNT_RADIUS = 0.014
MOUNT_LENGTH = 0.09
PIVOT_PIN_RADIUS = 0.014
PIVOT_PIN_LENGTH = 0.12
HANGER_LENGTH = 0.075
HANGER_RADIUS = 0.014
CABIN_RADIUS = 0.07
CABIN_CYL_LENGTH = 0.18
PIVOT_CLEARANCE = PIVOT_PIN_RADIUS
CABIN_CENTER_OFFSET = PIVOT_CLEARANCE + HANGER_LENGTH + CABIN_RADIUS - 0.002
BRACKET_RADIUS = 0.012


def _segment_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("segment length must be positive")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    midpoint = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return Origin(xyz=midpoint, rpy=(0.0, pitch, yaw)), length


def _add_segment(part, name: str, start: tuple[float, float, float], end: tuple[float, float, float], radius: float, material) -> None:
    origin, length = _segment_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observation_wheel", assets=ASSETS)

    support_mat = model.material("support_steel", rgba=(0.22, 0.24, 0.28, 1.0))
    wheel_mat = model.material("wheel_steel", rgba=(0.72, 0.75, 0.79, 1.0))
    accent_mat = model.material("accent_blue", rgba=(0.17, 0.40, 0.74, 1.0))
    cabin_mat = model.material("cabin_white", rgba=(0.94, 0.96, 0.98, 1.0))

    rim_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=WHEEL_RADIUS,
            tube=RIM_TUBE_RADIUS,
            radial_segments=18,
            tubular_segments=84,
        ),
        ASSETS.mesh_path("observation_wheel_rim.obj"),
    )
    capsule_geom = CapsuleGeometry(
        radius=CABIN_RADIUS,
        length=CABIN_CYL_LENGTH,
        radial_segments=24,
        height_segments=10,
    )
    capsule_geom.rotate_y(math.pi / 2.0)
    capsule_mesh = mesh_from_geometry(
        capsule_geom,
        ASSETS.mesh_path("observation_wheel_capsule.obj"),
    )

    support = model.part("support")
    support.visual(
        Box((0.64, 1.90, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=support_mat,
        name="base_plinth",
    )
    _add_segment(
        support,
        "left_leg",
        (-0.26, 0.0, 0.10),
        (-0.16, 0.0, 1.10),
        radius=0.05,
        material=support_mat,
    )
    _add_segment(
        support,
        "right_leg",
        (0.26, 0.0, 0.10),
        (0.16, 0.0, 1.10),
        radius=0.05,
        material=support_mat,
    )
    support.visual(
        Box((0.08, 0.18, 0.62)),
        origin=Origin(xyz=(-0.16, 0.0, 1.21)),
        material=support_mat,
        name="left_bearing_column",
    )
    support.visual(
        Box((0.08, 0.18, 0.62)),
        origin=Origin(xyz=(0.16, 0.0, 1.21)),
        material=support_mat,
        name="right_bearing_column",
    )
    support.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(-0.16, 0.0, WHEEL_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_mat,
        name="left_bearing_housing",
    )
    support.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(0.16, 0.0, WHEEL_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_mat,
        name="right_bearing_housing",
    )

    wheel = model.part("wheel")
    wheel.visual(
        rim_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_mat,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_mat,
        name="hub_drum",
    )
    wheel.visual(
        Cylinder(radius=HUB_FLANGE_RADIUS, length=HUB_FLANGE_LENGTH),
        origin=Origin(xyz=(-0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_mat,
        name="hub_flange_left",
    )
    wheel.visual(
        Cylinder(radius=HUB_FLANGE_RADIUS, length=HUB_FLANGE_LENGTH),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_mat,
        name="hub_flange_right",
    )

    spoke_inner = HUB_RADIUS - 0.004
    spoke_outer = WHEEL_RADIUS - RIM_TUBE_RADIUS + 0.008
    for i in range(SPOKE_COUNT):
        theta = 2.0 * math.pi * i / SPOKE_COUNT
        start = (0.0, spoke_inner * math.cos(theta), spoke_inner * math.sin(theta))
        end = (0.0, spoke_outer * math.cos(theta), spoke_outer * math.sin(theta))
        _add_segment(wheel, f"spoke_{i}", start, end, radius=SPOKE_RADIUS, material=wheel_mat)

    for i in range(CABIN_COUNT):
        theta = 2.0 * math.pi * i / CABIN_COUNT
        pivot = (0.0, PIVOT_RADIUS * math.cos(theta), PIVOT_RADIUS * math.sin(theta))
        bracket_start = (
            0.0,
            (WHEEL_RADIUS + RIM_TUBE_RADIUS - 0.004) * math.cos(theta),
            (WHEEL_RADIUS + RIM_TUBE_RADIUS - 0.004) * math.sin(theta),
        )
        bracket_end = (
            0.0,
            (PIVOT_RADIUS - MOUNT_RADIUS) * math.cos(theta),
            (PIVOT_RADIUS - MOUNT_RADIUS) * math.sin(theta),
        )
        _add_segment(
            wheel,
            f"bracket_{i}",
            bracket_start,
            bracket_end,
            radius=BRACKET_RADIUS,
            material=accent_mat,
        )
        wheel.visual(
            Cylinder(radius=MOUNT_RADIUS, length=MOUNT_LENGTH),
            origin=Origin(xyz=pivot, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=accent_mat,
            name=f"mount_{i}",
        )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.8),
    )

    for i in range(CABIN_COUNT):
        theta = 2.0 * math.pi * i / CABIN_COUNT
        pivot = (0.0, PIVOT_RADIUS * math.cos(theta), PIVOT_RADIUS * math.sin(theta))
        cabin = model.part(f"cabin_{i}")
        cabin.visual(
            Cylinder(radius=PIVOT_PIN_RADIUS, length=PIVOT_PIN_LENGTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=accent_mat,
            name="pivot_pin",
        )
        cabin.visual(
            Cylinder(radius=HANGER_RADIUS, length=HANGER_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, PIVOT_CLEARANCE + HANGER_LENGTH * 0.5)),
            material=support_mat,
            name="hanger_arm",
        )
        cabin.visual(
            capsule_mesh,
            origin=Origin(xyz=(0.0, 0.0, CABIN_CENTER_OFFSET)),
            material=cabin_mat,
            name="capsule_shell",
        )
        model.articulation(
            f"wheel_to_cabin_{i}",
            ArticulationType.REVOLUTE,
            parent=wheel,
            child=cabin,
            origin=Origin(xyz=pivot, rpy=(theta - math.pi / 2.0, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=24.0,
                velocity=2.0,
                lower=-math.pi,
                upper=math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support = object_model.get_part("support")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("support_to_wheel")
    left_flange = wheel.get_visual("hub_flange_left")
    right_flange = wheel.get_visual("hub_flange_right")
    left_bearing = support.get_visual("left_bearing_housing")
    right_bearing = support.get_visual("right_bearing_housing")
    cabins = [object_model.get_part(f"cabin_{i}") for i in range(CABIN_COUNT)]
    cabin_joints = [object_model.get_articulation(f"wheel_to_cabin_{i}") for i in range(CABIN_COUNT)]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    for i, cabin in enumerate(cabins):
        ctx.allow_overlap(
            cabin,
            wheel,
            elem_a=cabin.get_visual("pivot_pin"),
            elem_b=wheel.get_visual(f"mount_{i}"),
            reason="cabin pivot pin nests inside the wheel-side hanger bracket",
        )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "wheel_joint_is_horizontal_continuous_axis",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (1.0, 0.0, 0.0),
        details=f"expected a continuous x-axis wheel joint, got type={wheel_spin.articulation_type} axis={wheel_spin.axis}",
    )
    ctx.expect_contact(wheel, support, elem_a=left_flange, elem_b=left_bearing, name="left_flange_seats_in_left_bearing")
    ctx.expect_contact(wheel, support, elem_a=right_flange, elem_b=right_bearing, name="right_flange_seats_in_right_bearing")

    wheel_aabb = ctx.part_world_aabb(wheel)
    support_aabb = ctx.part_world_aabb(support)
    wheel_pos = ctx.part_world_position(wheel)
    if wheel_aabb is None or support_aabb is None or wheel_pos is None:
        ctx.fail("core_parts_have_measurable_bounds", "support and wheel need measurable bounds and positions")
        return ctx.report()

    wheel_height = wheel_aabb[1][2] - wheel_aabb[0][2]
    wheel_span_y = wheel_aabb[1][1] - wheel_aabb[0][1]
    support_height = support_aabb[1][2] - support_aabb[0][2]
    ctx.check(
        "wheel_reads_as_large_vertical_ring",
        wheel_height > 2.0 and wheel_span_y > 2.0,
        details=f"wheel bbox too small or not ring-like enough: height={wheel_height:.3f}, y_span={wheel_span_y:.3f}",
    )
    ctx.check(
        "support_reaches_axle_height",
        support_height > 1.4 and support_aabb[1][2] > WHEEL_CENTER_Z - 0.2,
        details=f"support should climb near the axle, got height={support_height:.3f} top_z={support_aabb[1][2]:.3f}",
    )
    ctx.check(
        "wheel_origin_is_centered_at_axle",
        abs(wheel_pos[0]) < 1e-6 and abs(wheel_pos[1]) < 1e-6 and abs(wheel_pos[2] - WHEEL_CENTER_Z) < 1e-6,
        details=f"wheel origin should sit at axle center, got {wheel_pos}",
    )

    for i, (cabin, joint) in enumerate(zip(cabins, cabin_joints)):
        pivot_pin = cabin.get_visual("pivot_pin")
        shell = cabin.get_visual("capsule_shell")
        mount = wheel.get_visual(f"mount_{i}")
        ctx.check(
            f"cabin_{i}_joint_is_horizontal_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"expected revolute x-axis hanger joint, got type={joint.articulation_type} axis={joint.axis}",
        )
        ctx.expect_origin_distance(
            cabin,
            wheel,
            axes="yz",
            min_dist=PIVOT_RADIUS - 0.005,
            max_dist=PIVOT_RADIUS + 0.005,
            name=f"cabin_{i}_pivot_sits_outside_rim_radius",
        )
        ctx.expect_overlap(
            cabin,
            wheel,
            axes="x",
            min_overlap=0.08,
            elem_a=pivot_pin,
            elem_b=mount,
            name=f"cabin_{i}_pivot_pin_seats_in_mount",
        )
        shell_center = _aabb_center(ctx.part_element_world_aabb(cabin, elem=shell))
        if shell_center is None:
            ctx.fail(f"cabin_{i}_shell_has_bounds", "capsule shell needs measurable geometry")
            continue
        shell_radius = math.hypot(shell_center[1], shell_center[2] - WHEEL_CENTER_Z)
        ctx.check(
            f"cabin_{i}_shell_stays_outside_rim",
            shell_radius > WHEEL_RADIUS + 0.10,
            details=f"cabin shell center should stay outside the rim; got radial distance {shell_radius:.3f}",
        )

    cabin_0 = cabins[0]
    cabin_0_joint = cabin_joints[0]
    rest_pivot = ctx.part_world_position(cabin_0)
    rest_shell_center = _aabb_center(ctx.part_element_world_aabb(cabin_0, elem=cabin_0.get_visual("capsule_shell")))
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        spun_pivot = ctx.part_world_position(cabin_0)
    ctx.check(
        "wheel_spin_moves_cabin_0_around_axle",
        rest_pivot is not None
        and spun_pivot is not None
        and abs(spun_pivot[1]) < 0.03
        and spun_pivot[2] > WHEEL_CENTER_Z + PIVOT_RADIUS - 0.03,
        details=f"expected cabin_0 pivot to sweep from +Y up toward +Z, got rest={rest_pivot}, spun={spun_pivot}",
    )
    with ctx.pose({cabin_0_joint: 0.6}):
        posed_shell_center = _aabb_center(
            ctx.part_element_world_aabb(cabin_0, elem=cabin_0.get_visual("capsule_shell"))
        )
    ctx.check(
        "cabin_0_pivots_independently_on_hanger_axis",
        rest_shell_center is not None
        and posed_shell_center is not None
        and posed_shell_center[2] > rest_shell_center[2] + 0.05
        and posed_shell_center[1] < rest_shell_center[1] - 0.02,
        details=f"expected cabin shell center to move upward and tangentially when pivoting, got rest={rest_shell_center}, posed={posed_shell_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
