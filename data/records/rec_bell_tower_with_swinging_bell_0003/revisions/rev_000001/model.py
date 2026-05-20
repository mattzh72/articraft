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
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, name: str, a, b, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _gusset_geometry(
    span: float,
    height: float,
    thickness: float,
    *,
    plane: str = "x",
    downward: bool = False,
):
    geom = ExtrudeGeometry.centered(
        [(0.0, 0.0), (span, 0.0), (0.0, height)],
        thickness,
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    if plane == "y":
        geom.rotate_z(math.pi / 2.0)
    if downward:
        geom.rotate_y(math.pi)
    return geom


def _bell_shell_geometry():
    return LatheGeometry.from_shell_profiles(
        [
            (0.168, -0.250),
            (0.165, -0.220),
            (0.156, -0.180),
            (0.145, -0.130),
            (0.130, -0.080),
            (0.118, -0.030),
            (0.118, 0.000),
            (0.095, 0.018),
            (0.060, 0.032),
            (0.020, 0.040),
        ],
        [
            (0.145, -0.243),
            (0.142, -0.220),
            (0.136, -0.180),
            (0.126, -0.130),
            (0.113, -0.080),
            (0.103, -0.030),
            (0.103, 0.000),
            (0.078, 0.018),
            (0.038, 0.030),
            (0.000, 0.038),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _yoke_arm_geometry(y_pos: float):
    return sweep_profile_along_spline(
        [
            (0.000, y_pos, 0.000),
            (0.028, y_pos, -0.026),
            (0.046, y_pos, -0.064),
            (0.032, y_pos, -0.102),
            (0.000, y_pos, -0.124),
        ],
        profile=rounded_rect_profile(0.046, 0.074, radius=0.010, corner_segments=6),
        samples_per_segment=14,
        cap_profile=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bell_frame_tower", assets=ASSETS)

    frame_steel = model.material("frame_steel", rgba=(0.24, 0.27, 0.29, 1.0))
    gusset_steel = model.material("gusset_steel", rgba=(0.21, 0.23, 0.25, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    weathered_bronze = model.material("weathered_bronze", rgba=(0.60, 0.45, 0.20, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.68, 0.70, 0.72, 1.0))

    base_x_gusset = _save_mesh(
        "tower_base_x_gusset.obj",
        _gusset_geometry(0.110, 0.130, 0.010, plane="x", downward=False),
    )
    base_y_gusset = _save_mesh(
        "tower_base_y_gusset.obj",
        _gusset_geometry(0.110, 0.130, 0.010, plane="y", downward=False),
    )
    top_x_gusset = _save_mesh(
        "tower_top_x_gusset.obj",
        _gusset_geometry(0.110, 0.130, 0.010, plane="x", downward=True),
    )
    top_y_gusset = _save_mesh(
        "tower_top_y_gusset.obj",
        _gusset_geometry(0.110, 0.130, 0.010, plane="y", downward=True),
    )
    bell_shell_mesh = _save_mesh("tower_bell_shell.obj", _bell_shell_geometry())
    yoke_arm_front_mesh = _save_mesh("tower_yoke_arm_front.obj", _yoke_arm_geometry(0.310))
    yoke_arm_back_mesh = _save_mesh("tower_yoke_arm_back.obj", _yoke_arm_geometry(-0.310))

    frame = model.part("frame")
    frame.visual(
        Box((0.94, 0.94, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=frame_steel,
        name="baseplate",
    )

    post_radius = 0.026
    tower_half_span = 0.360
    top_z = 1.200
    post_bottom = 0.035
    post_length = top_z - post_bottom
    for post_name, x_pos, y_pos in [
        ("post_front_right", tower_half_span, tower_half_span),
        ("post_front_left", -tower_half_span, tower_half_span),
        ("post_back_left", -tower_half_span, -tower_half_span),
        ("post_back_right", tower_half_span, -tower_half_span),
    ]:
        frame.visual(
            Cylinder(radius=post_radius, length=post_length),
            origin=Origin(xyz=(x_pos, y_pos, post_bottom + 0.5 * post_length)),
            material=frame_steel,
            name=post_name,
        )

    frame.visual(
        Cylinder(radius=0.022, length=0.720),
        origin=Origin(xyz=(0.0, tower_half_span, top_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_steel,
        name="front_top_rail",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.720),
        origin=Origin(xyz=(0.0, -tower_half_span, top_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_steel,
        name="back_top_rail",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.720),
        origin=Origin(xyz=(tower_half_span, 0.0, top_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_steel,
        name="right_top_rail",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.720),
        origin=Origin(xyz=(-tower_half_span, 0.0, top_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_steel,
        name="left_top_rail",
    )
    frame.visual(
        Box((0.110, 0.692, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, top_z + 0.085)),
        material=frame_steel,
        name="top_beam",
    )
    frame.visual(
        Box((0.085, 0.024, 0.140)),
        origin=Origin(xyz=(0.0, 0.347, top_z - 0.030)),
        material=cast_iron,
        name="front_pivot_plate",
    )
    frame.visual(
        Box((0.085, 0.024, 0.140)),
        origin=Origin(xyz=(0.0, -0.347, top_z - 0.030)),
        material=cast_iron,
        name="back_pivot_plate",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.0, 0.372, top_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="front_bolt_head",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.0, -0.372, top_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="back_bolt_head",
    )

    brace_radius = 0.012
    brace_low = 0.100
    brace_high = 1.040
    _add_member(
        frame,
        "front_brace_rising",
        (-tower_half_span, tower_half_span, brace_low),
        (tower_half_span, tower_half_span, brace_high),
        brace_radius,
        frame_steel,
    )
    _add_member(
        frame,
        "front_brace_falling",
        (tower_half_span, tower_half_span, brace_low),
        (-tower_half_span, tower_half_span, brace_high),
        brace_radius,
        frame_steel,
    )
    _add_member(
        frame,
        "back_brace_rising",
        (-tower_half_span, -tower_half_span, brace_low),
        (tower_half_span, -tower_half_span, brace_high),
        brace_radius,
        frame_steel,
    )
    _add_member(
        frame,
        "back_brace_falling",
        (tower_half_span, -tower_half_span, brace_low),
        (-tower_half_span, -tower_half_span, brace_high),
        brace_radius,
        frame_steel,
    )
    _add_member(
        frame,
        "right_brace_rising",
        (tower_half_span, -tower_half_span, brace_low),
        (tower_half_span, tower_half_span, brace_high),
        brace_radius,
        frame_steel,
    )
    _add_member(
        frame,
        "right_brace_falling",
        (tower_half_span, tower_half_span, brace_low),
        (tower_half_span, -tower_half_span, brace_high),
        brace_radius,
        frame_steel,
    )
    _add_member(
        frame,
        "left_brace_rising",
        (-tower_half_span, -tower_half_span, brace_low),
        (-tower_half_span, tower_half_span, brace_high),
        brace_radius,
        frame_steel,
    )
    _add_member(
        frame,
        "left_brace_falling",
        (-tower_half_span, tower_half_span, brace_low),
        (-tower_half_span, -tower_half_span, brace_high),
        brace_radius,
        frame_steel,
    )

    for label, x_pos, y_pos, x_yaw, y_yaw, top_x_yaw, top_y_yaw in [
        ("front_right", tower_half_span, tower_half_span, math.pi, math.pi, 0.0, math.pi),
        ("front_left", -tower_half_span, tower_half_span, 0.0, math.pi, math.pi, math.pi),
        ("back_left", -tower_half_span, -tower_half_span, 0.0, 0.0, math.pi, 0.0),
        ("back_right", tower_half_span, -tower_half_span, math.pi, 0.0, 0.0, 0.0),
    ]:
        frame.visual(
            base_x_gusset,
            origin=Origin(xyz=(x_pos, y_pos, post_bottom), rpy=(0.0, 0.0, x_yaw)),
            material=gusset_steel,
            name=f"{label}_base_gusset_x",
        )
        frame.visual(
            base_y_gusset,
            origin=Origin(xyz=(x_pos, y_pos, post_bottom), rpy=(0.0, 0.0, y_yaw)),
            material=gusset_steel,
            name=f"{label}_base_gusset_y",
        )
        frame.visual(
            top_x_gusset,
            origin=Origin(xyz=(x_pos, y_pos, top_z), rpy=(0.0, 0.0, top_x_yaw)),
            material=gusset_steel,
            name=f"{label}_top_gusset_x",
        )
        frame.visual(
            top_y_gusset,
            origin=Origin(xyz=(x_pos, y_pos, top_z), rpy=(0.0, 0.0, top_y_yaw)),
            material=gusset_steel,
            name=f"{label}_top_gusset_y",
        )

    frame.inertial = Inertial.from_geometry(
        Box((0.94, 0.94, 1.235)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, 0.6175)),
    )

    bell = model.part("bell_assembly")
    bell.visual(
        Cylinder(radius=0.016, length=0.570),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="pivot_journal",
    )
    bell.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.0, 0.310, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="front_yoke_boss",
    )
    bell.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.0, -0.310, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="back_yoke_boss",
    )
    bell.visual(yoke_arm_front_mesh, material=cast_iron, name="front_yoke_arm")
    bell.visual(yoke_arm_back_mesh, material=cast_iron, name="back_yoke_arm")
    bell.visual(
        Box((0.070, 0.620, 0.050)),
        origin=Origin(xyz=(0.018, 0.0, -0.100)),
        material=cast_iron,
        name="yoke_crossbar",
    )
    bell.visual(
        Box((0.080, 0.120, 0.070)),
        origin=Origin(xyz=(0.000, 0.0, -0.072)),
        material=cast_iron,
        name="crown_block",
    )
    bell.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.000, 0.0, -0.070)),
        material=cast_iron,
        name="crown_boss",
    )
    bell.visual(
        bell_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=weathered_bronze,
        name="bell_shell",
    )
    bell.inertial = Inertial.from_geometry(
        Box((0.360, 0.700, 0.360)),
        mass=44.0,
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=280.0,
            velocity=1.6,
            lower=-0.42,
            upper=0.42,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    bell = object_model.get_part("bell_assembly")
    bell_swing = object_model.get_articulation("bell_swing")
    baseplate = frame.get_visual("baseplate")
    top_beam = frame.get_visual("top_beam")
    front_top_rail = frame.get_visual("front_top_rail")
    right_top_rail = frame.get_visual("right_top_rail")
    front_pivot_plate = frame.get_visual("front_pivot_plate")
    back_pivot_plate = frame.get_visual("back_pivot_plate")
    left_top_rail = frame.get_visual("left_top_rail")
    front_brace_rising = frame.get_visual("front_brace_rising")
    front_brace_falling = frame.get_visual("front_brace_falling")
    right_brace_rising = frame.get_visual("right_brace_rising")
    right_brace_falling = frame.get_visual("right_brace_falling")
    front_right_base_gusset_x = frame.get_visual("front_right_base_gusset_x")
    front_right_top_gusset_x = frame.get_visual("front_right_top_gusset_x")
    front_right_top_gusset_y = frame.get_visual("front_right_top_gusset_y")
    bell_shell = bell.get_visual("bell_shell")
    front_yoke_boss = bell.get_visual("front_yoke_boss")
    back_yoke_boss = bell.get_visual("back_yoke_boss")
    yoke_crossbar = bell.get_visual("yoke_crossbar")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.045)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(bell, frame, axes="xy", max_dist=0.001)
    ctx.expect_overlap(frame, bell, axes="xy", elem_a=top_beam, elem_b=yoke_crossbar, min_overlap=0.02)
    ctx.expect_contact(frame, bell, elem_a=front_pivot_plate, elem_b=front_yoke_boss)
    ctx.expect_contact(frame, bell, elem_a=back_pivot_plate, elem_b=back_yoke_boss)
    ctx.expect_overlap(frame, bell, axes="xz", elem_a=front_pivot_plate, elem_b=front_yoke_boss, min_overlap=0.015)
    ctx.expect_overlap(frame, bell, axes="xz", elem_a=back_pivot_plate, elem_b=back_yoke_boss, min_overlap=0.015)
    ctx.expect_contact(frame, frame, elem_a=front_brace_rising, elem_b=front_brace_falling)
    ctx.expect_contact(frame, frame, elem_a=right_brace_rising, elem_b=right_brace_falling)
    ctx.expect_contact(frame, frame, elem_a=front_right_base_gusset_x, elem_b=baseplate)
    ctx.expect_contact(frame, frame, elem_a=front_right_top_gusset_x, elem_b=right_top_rail)
    ctx.expect_contact(frame, frame, elem_a=front_right_top_gusset_y, elem_b=front_top_rail)
    ctx.expect_gap(
        frame,
        bell,
        axis="z",
        positive_elem=top_beam,
        negative_elem=bell_shell,
        min_gap=0.070,
        max_gap=0.130,
    )
    ctx.expect_within(bell, frame, axes="xy", inner_elem=bell_shell, outer_elem=baseplate)

    with ctx.pose({bell_swing: 0.32}):
        ctx.expect_within(bell, frame, axes="xy", inner_elem=bell_shell, outer_elem=baseplate)
        ctx.expect_gap(
            frame,
            bell,
            axis="x",
            positive_elem=right_top_rail,
            negative_elem=bell_shell,
            min_gap=0.012,
        )

    with ctx.pose({bell_swing: -0.32}):
        ctx.expect_within(bell, frame, axes="xy", inner_elem=bell_shell, outer_elem=baseplate)
        ctx.expect_gap(
            bell,
            frame,
            axis="x",
            positive_elem=bell_shell,
            negative_elem=left_top_rail,
            min_gap=0.012,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
