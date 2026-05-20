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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _segment_length(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _segment_midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _segment_rpy(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _hex_profile(radius: float) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(i * math.pi / 3.0), radius * math.sin(i * math.pi / 3.0))
        for i in range(6)
    ]


def _point_lerp(
    a: tuple[float, float, float], b: tuple[float, float, float], t: float
) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_hose_bib_faucet")

    brass = model.material("brass", rgba=(0.70, 0.60, 0.35, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.20, 0.20, 0.22, 1.0))

    wall_mount = model.part("wall_mount")
    wall_mount.inertial = Inertial.from_geometry(
        Box((0.15, 0.10, 0.11)),
        mass=1.6,
        origin=Origin(xyz=(0.055, 0.0, -0.005)),
    )

    wall_mount.visual(
        Cylinder(radius=0.038, length=0.004),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="wall_plate",
    )
    wall_mount.visual(
        Cylinder(radius=0.0135, length=0.024),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="inlet_shank",
    )

    hex_nut = mesh_from_geometry(
        ExtrudeGeometry.centered(_hex_profile(0.0205), 0.012),
        "packing_nut",
    )
    wall_mount.visual(
        hex_nut,
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="packing_nut",
    )
    wall_mount.visual(
        Cylinder(radius=0.0155, length=0.018),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="shank_boss",
    )
    wall_mount.visual(
        Cylinder(radius=0.0165, length=0.058),
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="valve_body",
    )
    wall_mount.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.071, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="front_collar",
    )
    wall_mount.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.032, 0.0, 0.026)),
        material=brass,
        name="bonnet",
    )
    wall_mount.visual(
        Cylinder(radius=0.0095, length=0.008),
        origin=Origin(xyz=(0.032, 0.0, 0.043)),
        material=brass,
        name="stem_cap",
    )

    spout_path = [
        (0.055, 0.0, -0.006),
        (0.079, 0.0, -0.010),
        (0.097, 0.0, -0.022),
        (0.113, 0.0, -0.040),
        (0.128, 0.0, -0.058),
    ]
    spout_shell = mesh_from_geometry(
        tube_from_spline_points(
            spout_path,
            radius=0.0108,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=False,
        ),
        "spout_shell",
    )
    wall_mount.visual(spout_shell, material=brass, name="spout_shell")

    thread_start = (0.110, 0.0, -0.037)
    thread_end = (0.132, 0.0, -0.061)
    thread_sleeve = mesh_from_geometry(
        tube_from_spline_points(
            [thread_start, thread_end],
            radius=0.0142,
            samples_per_segment=4,
            radial_segments=20,
            cap_ends=False,
        ),
        "hose_thread_sleeve",
    )
    wall_mount.visual(thread_sleeve, material=brass, name="hose_thread_sleeve")

    outlet_length = _segment_length(thread_start, thread_end)
    outlet_rpy = _segment_rpy(thread_start, thread_end)
    for idx, t in enumerate((0.28, 0.48, 0.68, 0.86), start=1):
        ridge_center = _point_lerp(thread_start, thread_end, t)
        wall_mount.visual(
            Cylinder(radius=0.0150, length=0.0022),
            origin=Origin(xyz=ridge_center, rpy=outlet_rpy),
            material=brass,
            name=f"thread_ridge_{idx}",
        )
    wall_mount.visual(
        Cylinder(radius=0.0148, length=0.0045),
        origin=Origin(
            xyz=_point_lerp(thread_start, thread_end, 0.10),
            rpy=outlet_rpy,
        ),
        material=brass,
        name="hose_seat",
    )
    wall_mount.visual(
        Cylinder(radius=0.0080, length=0.018),
        origin=Origin(
            xyz=_point_lerp(thread_start, thread_end, 0.60),
            rpy=outlet_rpy,
        ),
        material=dark_handle,
        name="spout_bore_shadow",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.085, 0.095, 0.035)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )
    handle.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_handle,
        name="handle_stem",
    )
    handle.visual(
        Cylinder(radius=0.0135, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=dark_handle,
        name="hub",
    )

    x_spoke_a = (-0.034, 0.0, 0.027)
    x_spoke_b = (0.034, 0.0, 0.027)
    handle.visual(
        Cylinder(radius=0.0032, length=_segment_length(x_spoke_a, x_spoke_b)),
        origin=Origin(xyz=_segment_midpoint(x_spoke_a, x_spoke_b), rpy=_segment_rpy(x_spoke_a, x_spoke_b)),
        material=dark_handle,
        name="spoke_x",
    )
    y_spoke_a = (0.0, -0.043, 0.027)
    y_spoke_b = (0.0, 0.043, 0.027)
    handle.visual(
        Cylinder(radius=0.0032, length=_segment_length(y_spoke_a, y_spoke_b)),
        origin=Origin(xyz=_segment_midpoint(y_spoke_a, y_spoke_b), rpy=_segment_rpy(y_spoke_a, y_spoke_b)),
        material=dark_handle,
        name="spoke_y",
    )

    handwheel_rim = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.043, 0.027),
                (0.020, 0.037, 0.027),
                (0.034, 0.0, 0.027),
                (0.020, -0.037, 0.027),
                (0.0, -0.043, 0.027),
                (-0.020, -0.037, 0.027),
                (-0.034, 0.0, 0.027),
                (-0.020, 0.037, 0.027),
            ],
            radius=0.0036,
            samples_per_segment=14,
            radial_segments=18,
            closed_spline=True,
            cap_ends=False,
        ),
        "handwheel_rim",
    )
    handle.visual(handwheel_rim, material=dark_handle, name="handwheel_rim")

    model.articulation(
        "handle_turn",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=handle,
        origin=Origin(xyz=(0.032, 0.0, 0.047)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=6.0,
            lower=0.0,
            upper=7.2,
        ),
    )

    return model


def _aabb_dims(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return (
        maxs[0] - mins[0],
        maxs[1] - mins[1],
        maxs[2] - mins[2],
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_mount = object_model.get_part("wall_mount")
    handle = object_model.get_part("handle")
    handle_turn = object_model.get_articulation("handle_turn")

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

    ctx.expect_contact(
        handle,
        wall_mount,
        elem_a="handle_stem",
        elem_b="stem_cap",
        contact_tol=0.0005,
        name="handle stem seats on valve stem cap",
    )
    ctx.expect_origin_gap(
        handle,
        wall_mount,
        axis="z",
        min_gap=0.040,
        max_gap=0.055,
        name="handle sits above the wall mount body",
    )
    ctx.expect_overlap(
        handle,
        wall_mount,
        axes="xy",
        min_overlap=0.008,
        elem_a="handle_stem",
        elem_b="bonnet",
        name="handle stem stays centered over bonnet",
    )
    ctx.check(
        "handle joint uses vertical valve axis",
        tuple(round(v, 6) for v in handle_turn.axis) == (0.0, 0.0, 1.0),
        details=f"axis={handle_turn.axis}",
    )

    rim_rest = ctx.part_element_world_aabb(handle, elem="handwheel_rim")
    with ctx.pose({handle_turn: math.pi / 2.0}):
        rim_turned = ctx.part_element_world_aabb(handle, elem="handwheel_rim")

    rest_dims = _aabb_dims(rim_rest)
    turned_dims = _aabb_dims(rim_turned)
    ctx.check(
        "oval handwheel visibly rotates in plan",
        rest_dims is not None
        and turned_dims is not None
        and rest_dims[1] > rest_dims[0] + 0.008
        and turned_dims[0] > turned_dims[1] + 0.008,
        details=f"rest_dims={rest_dims}, turned_dims={turned_dims}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
