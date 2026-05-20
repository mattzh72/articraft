from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = atan2(dy, dx)
    pitch = atan2(hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_rear_wheel_visuals(
    part,
    mesh_prefix: str,
    *,
    side_sign: float,
    tire_radius: float,
    tire_tube: float,
    tire_width: float,
    wheel_metal,
    rubber,
) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        TorusGeometry(tire_radius - tire_tube, tire_tube, radial_segments=20, tubular_segments=56).rotate_y(pi / 2.0),
    )
    handrim_offset = side_sign * 0.022
    handrim_radius = tire_radius - 0.058
    handrim_mesh = _save_mesh(
        f"{mesh_prefix}_handrim",
        TorusGeometry(handrim_radius, 0.006, radial_segments=14, tubular_segments=48).rotate_y(pi / 2.0),
    )

    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=tire_radius - tire_tube * 0.55, length=0.004),
        origin=Origin(xyz=(tire_width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="outboard_rim",
    )
    part.visual(
        Cylinder(radius=tire_radius - tire_tube * 0.55, length=0.004),
        origin=Origin(xyz=(-tire_width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="inboard_rim",
    )
    part.visual(
        Cylinder(radius=0.035, length=0.044),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="axle_sleeve",
    )
    part.visual(
        handrim_mesh,
        origin=Origin(xyz=(handrim_offset, 0.0, 0.0)),
        material=wheel_metal,
        name="handrim",
    )

    spoke_radius = 0.0038
    for spoke_index in range(8):
        angle = 2.0 * pi * spoke_index / 8.0
        hub_point = (0.0, cos(angle) * 0.034, sin(angle) * 0.034)
        rim_point = (0.0, cos(angle + 0.18) * (tire_radius - 0.040), sin(angle + 0.18) * (tire_radius - 0.040))
        _add_member(part, hub_point, rim_point, spoke_radius, wheel_metal, name=f"spoke_{spoke_index}")

    for bracket_index, angle in enumerate((pi / 4.0, 3.0 * pi / 4.0, 5.0 * pi / 4.0, 7.0 * pi / 4.0)):
        rim_mount = (tire_width * 0.16 * side_sign, cos(angle) * (handrim_radius - 0.014), sin(angle) * (handrim_radius - 0.014))
        handrim_mount = (handrim_offset, cos(angle) * handrim_radius, sin(angle) * handrim_radius)
        _add_member(
            part,
            rim_mount,
            handrim_mount,
            0.0028,
            wheel_metal,
            name=f"handrim_bracket_{bracket_index}",
        )


def _add_caster_fork_visuals(part, material) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=material,
        name="swivel_stem",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=material,
        name="thrust_collar",
    )
    part.visual(
        Box((0.056, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, -0.002, -0.025)),
        material=material,
        name="yoke_head",
    )
    for side_sign in (-1.0, 1.0):
        _add_member(
            part,
            (side_sign * 0.018, -0.010, -0.036),
            (side_sign * 0.024, -0.040, -0.060),
            0.004,
            material,
            name=f"gusset_{'left' if side_sign > 0.0 else 'right'}",
        )
        part.visual(
            Box((0.008, 0.060, 0.072)),
            origin=Origin(xyz=(side_sign * 0.024, -0.050, -0.074)),
            material=material,
            name=f"side_plate_{'left' if side_sign > 0.0 else 'right'}",
        )


def _add_caster_wheel_visuals(part, mesh_prefix: str, *, wheel_metal, rubber) -> None:
    tire_radius = 0.065
    tire_tube = 0.012
    tire_width = 0.026
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        TorusGeometry(tire_radius - tire_tube, tire_tube, radial_segments=18, tubular_segments=40).rotate_y(pi / 2.0),
    )
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=tire_radius - tire_tube * 0.65, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="rim_shell",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="hub",
    )


def _add_caster_headtube_visuals(part, material) -> None:
    part.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=material,
        name="headtube",
    )
    part.visual(
        Box((0.022, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.016, 0.007)),
        material=material,
        name="mount_saddle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    frame_paint = model.material("frame_paint", rgba=(0.70, 0.73, 0.76, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.24, 0.25, 0.27, 1.0))
    seat_fabric = model.material("seat_fabric", rgba=(0.10, 0.11, 0.14, 1.0))
    arm_pad = model.material("arm_pad", rgba=(0.12, 0.12, 0.13, 1.0))
    footplate = model.material("footplate", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.82, 0.84, 0.86, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.68, 0.72, 1.00)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.10, 0.50)),
    )

    left_nodes = {
        "push": (0.215, -0.165, 0.945),
        "back_top": (0.215, -0.125, 0.835),
        "back_lower": (0.215, -0.115, 0.525),
        "axle": (0.275, -0.070, 0.310),
        "side_front": (0.225, 0.275, 0.505),
        "caster_socket": (0.245, 0.345, 0.145),
        "hanger_top": (0.135, 0.305, 0.305),
        "hanger_mid": (0.108, 0.355, 0.170),
        "foot_rear": (0.098, 0.370, 0.080),
        "arm_front": (0.245, 0.200, 0.655),
        "arm_back": (0.245, -0.085, 0.655),
    }
    right_nodes = {name: (-x, y, z) for name, (x, y, z) in left_nodes.items()}

    for side_nodes in (left_nodes, right_nodes):
        caster_axis = (
            side_nodes["caster_socket"][0],
            side_nodes["caster_socket"][1] + 0.037,
            side_nodes["caster_socket"][2] + 0.030,
        )
        _add_member(frame, side_nodes["push"], side_nodes["back_top"], 0.014, frame_paint)
        _add_member(frame, side_nodes["back_top"], side_nodes["back_lower"], 0.015, frame_paint)
        _add_member(frame, side_nodes["back_lower"], side_nodes["axle"], 0.015, frame_paint)
        _add_member(frame, side_nodes["back_lower"], side_nodes["side_front"], 0.016, frame_paint)
        _add_member(frame, side_nodes["side_front"], side_nodes["caster_socket"], 0.015, frame_paint)
        for brace_side in (-1.0, 1.0):
            _add_member(
                frame,
                side_nodes["caster_socket"],
                (
                    caster_axis[0] + brace_side * 0.020,
                    caster_axis[1] - 0.016,
                    caster_axis[2] + 0.009,
                ),
                0.0065,
                dark_frame,
            )
        _add_member(frame, side_nodes["side_front"], side_nodes["hanger_top"], 0.013, frame_paint)
        _add_member(frame, side_nodes["hanger_top"], side_nodes["hanger_mid"], 0.013, frame_paint)
        _add_member(frame, side_nodes["hanger_mid"], side_nodes["foot_rear"], 0.013, frame_paint)
        _add_member(frame, side_nodes["arm_back"], side_nodes["arm_front"], 0.012, dark_frame)
        _add_member(frame, side_nodes["back_lower"], side_nodes["arm_back"], 0.010, dark_frame)
        _add_member(frame, side_nodes["side_front"], side_nodes["arm_front"], 0.010, dark_frame)
        frame.visual(
            Cylinder(radius=0.018, length=0.062),
            origin=Origin(
                xyz=(side_nodes["axle"][0], side_nodes["axle"][1], side_nodes["axle"][2]),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=dark_frame,
        )

    _add_member(frame, left_nodes["back_top"], right_nodes["back_top"], 0.012, frame_paint)
    _add_member(frame, left_nodes["back_lower"], right_nodes["back_lower"], 0.012, frame_paint)
    _add_member(frame, left_nodes["back_lower"], right_nodes["side_front"], 0.012, frame_paint)
    _add_member(frame, right_nodes["back_lower"], left_nodes["side_front"], 0.012, frame_paint)
    _add_member(frame, left_nodes["hanger_mid"], right_nodes["hanger_mid"], 0.012, frame_paint)
    _add_member(frame, left_nodes["foot_rear"], right_nodes["foot_rear"], 0.010, frame_paint)

    frame.visual(
        Box((0.452, 0.420, 0.020)),
        origin=Origin(xyz=(0.0, 0.090, 0.505)),
        material=seat_fabric,
        name="seat_sling",
    )
    frame.visual(
        Box((0.440, 0.022, 0.360)),
        origin=Origin(xyz=(0.0, -0.125, 0.695)),
        material=seat_fabric,
        name="backrest_sling",
    )
    frame.visual(
        Box((0.048, 0.290, 0.028)),
        origin=Origin(xyz=(0.268, 0.058, 0.655)),
        material=arm_pad,
        name="left_arm_pad",
    )
    frame.visual(
        Box((0.048, 0.290, 0.028)),
        origin=Origin(xyz=(-0.268, 0.058, 0.655)),
        material=arm_pad,
        name="right_arm_pad",
    )
    frame.visual(
        Box((0.150, 0.110, 0.018)),
        origin=Origin(xyz=(0.095, 0.425, 0.071)),
        material=footplate,
        name="left_footrest_platform",
    )
    frame.visual(
        Box((0.150, 0.110, 0.018)),
        origin=Origin(xyz=(-0.095, 0.425, 0.071)),
        material=footplate,
        name="right_footrest_platform",
    )
    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.032),
        mass=2.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        rear_left_wheel,
        "rear_left_wheel",
        side_sign=1.0,
        tire_radius=0.305,
        tire_tube=0.023,
        tire_width=0.032,
        wheel_metal=wheel_metal,
        rubber=rubber,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.032),
        mass=2.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        rear_right_wheel,
        "rear_right_wheel",
        side_sign=-1.0,
        tire_radius=0.305,
        tire_tube=0.023,
        tire_width=0.032,
        wheel_metal=wheel_metal,
        rubber=rubber,
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.060, 0.120, 0.140)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.030, -0.070)),
    )
    _add_caster_fork_visuals(left_caster_fork, dark_frame)

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.060, 0.120, 0.140)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.030, -0.070)),
    )
    _add_caster_fork_visuals(right_caster_fork, dark_frame)

    left_caster_headtube = model.part("left_caster_headtube")
    left_caster_headtube.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.026)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )
    _add_caster_headtube_visuals(left_caster_headtube, dark_frame)

    right_caster_headtube = model.part("right_caster_headtube")
    right_caster_headtube.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.026)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )
    _add_caster_headtube_visuals(right_caster_headtube, dark_frame)

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.026),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel_visuals(left_caster_wheel, "left_caster_wheel", wheel_metal=wheel_metal, rubber=rubber)

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.026),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel_visuals(right_caster_wheel, "right_caster_wheel", wheel_metal=wheel_metal, rubber=rubber)

    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.328, -0.070, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=22.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.328, -0.070, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=22.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=left_caster_headtube,
        child=left_caster_fork,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=right_caster_headtube,
        child=right_caster_fork,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, -0.062, -0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, -0.062, -0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "left_caster_headtube_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=left_caster_headtube,
        origin=Origin(xyz=(0.245, 0.382, 0.176)),
    )
    model.articulation(
        "right_caster_headtube_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=right_caster_headtube,
        origin=Origin(xyz=(-0.245, 0.382, 0.176)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    left_caster_headtube = object_model.get_part("left_caster_headtube")
    right_caster_headtube = object_model.get_part("right_caster_headtube")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")

    ctx.allow_overlap(
        left_caster_headtube,
        left_caster_fork,
        elem_a="headtube",
        elem_b="swivel_stem",
        reason="The caster swivel stem is intentionally represented as passing through the solid headtube proxy.",
    )
    ctx.allow_overlap(
        frame,
        left_caster_headtube,
        reason="The left caster headtube is simplified as a welded saddle mount that slightly interpenetrates the tubular frame brace.",
    )
    ctx.allow_overlap(
        right_caster_headtube,
        right_caster_fork,
        elem_a="headtube",
        elem_b="swivel_stem",
        reason="The caster swivel stem is intentionally represented as passing through the solid headtube proxy.",
    )
    ctx.allow_overlap(
        frame,
        right_caster_headtube,
        reason="The right caster headtube is simplified as a welded saddle mount that slightly interpenetrates the tubular frame brace.",
    )

    ctx.expect_contact(
        left_caster_wheel,
        left_caster_fork,
        name="left caster wheel stays supported by the fork",
    )
    ctx.expect_contact(
        right_caster_wheel,
        right_caster_fork,
        name="right caster wheel stays supported by the fork",
    )
    ctx.expect_contact(
        rear_left_wheel,
        frame,
        name="left rear wheel remains seated on the axle mount",
    )
    ctx.expect_contact(
        rear_right_wheel,
        frame,
        name="right rear wheel remains seated on the axle mount",
    )

    with ctx.pose({left_caster_swivel: pi / 4.0, right_caster_swivel: -pi / 4.0}):
        ctx.expect_contact(
            left_caster_wheel,
            left_caster_fork,
            name="left caster wheel remains supported while swiveled",
        )
        ctx.expect_contact(
            right_caster_wheel,
            right_caster_fork,
            name="right caster wheel remains supported while swiveled",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
