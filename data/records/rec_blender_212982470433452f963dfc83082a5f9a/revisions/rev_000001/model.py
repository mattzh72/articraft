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
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _rect_loop(
    width: float,
    depth: float,
    z: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float, float]]:
    cx, cy = center
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (cx - half_w, cy - half_d, z),
        (cx + half_w, cy - half_d, z),
        (cx + half_w, cy + half_d, z),
        (cx - half_w, cy + half_d, z),
    ]


def _circle_loop_y(
    radius: float,
    y: float,
    *,
    segments: int = 28,
) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            y,
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _append_loop(
    geometry: MeshGeometry,
    loop: list[tuple[float, float, float]],
) -> list[int]:
    return [geometry.add_vertex(*point) for point in loop]


def _bridge_loops(
    geometry: MeshGeometry,
    loop_a: list[int],
    loop_b: list[int],
    *,
    reverse: bool = False,
) -> None:
    count = len(loop_a)
    for index in range(count):
        a0 = loop_a[index]
        a1 = loop_a[(index + 1) % count]
        b1 = loop_b[(index + 1) % count]
        b0 = loop_b[index]
        if reverse:
            _add_quad(geometry, a0, b0, b1, a1)
        else:
            _add_quad(geometry, a0, a1, b1, b0)


def _build_shell_from_loops(
    *,
    outer_bottom: list[tuple[float, float, float]],
    outer_top: list[tuple[float, float, float]],
    inner_top: list[tuple[float, float, float]],
    inner_bottom: list[tuple[float, float, float]],
) -> MeshGeometry:
    geometry = MeshGeometry()
    outer_bottom_ids = _append_loop(geometry, outer_bottom)
    outer_top_ids = _append_loop(geometry, outer_top)
    inner_top_ids = _append_loop(geometry, inner_top)
    inner_bottom_ids = _append_loop(geometry, inner_bottom)

    _bridge_loops(geometry, outer_bottom_ids, outer_top_ids)
    _bridge_loops(geometry, inner_top_ids, inner_bottom_ids, reverse=True)
    _bridge_loops(geometry, outer_top_ids, inner_top_ids)
    _bridge_loops(geometry, inner_bottom_ids, outer_bottom_ids)
    return geometry


def _helix_points(
    *,
    y_start: float,
    y_end: float,
    turns: float,
    radius_start: float,
    radius_end: float,
    phase: float = 0.0,
    samples_per_turn: int = 18,
) -> list[tuple[float, float, float]]:
    samples = max(8, int(math.ceil(abs(turns) * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        radius = radius_start + (radius_end - radius_start) * t
        angle = phase + (turns * math.tau * t)
        y = y_start + (y_end - y_start) * t
        points.append((radius * math.cos(angle), y, radius * math.sin(angle)))
    return points


def _build_auger_mesh() -> MeshGeometry:
    geometry = MeshGeometry()

    geometry.merge(
        CylinderGeometry(radius=0.018, height=0.182, radial_segments=40).rotate_x(-math.pi / 2.0)
    )
    geometry.merge(
        CylinderGeometry(radius=0.022, height=0.026, radial_segments=32)
        .rotate_x(-math.pi / 2.0)
        .translate(0.0, -0.077, 0.0)
    )
    geometry.merge(
        ConeGeometry(radius=0.020, height=0.036, radial_segments=36)
        .rotate_x(-math.pi / 2.0)
        .translate(0.0, 0.093, 0.0)
    )
    geometry.merge(
        CylinderGeometry(radius=0.0145, height=0.024, radial_segments=28)
        .rotate_x(-math.pi / 2.0)
        .translate(0.0, 0.050, 0.0)
    )

    rear_phase = 0.25
    rear_flight = tube_from_spline_points(
        _helix_points(
            y_start=-0.088,
            y_end=-0.006,
            turns=1.12,
            radius_start=0.0265,
            radius_end=0.0295,
            phase=rear_phase,
            samples_per_turn=22,
        ),
        radius=0.0115,
        samples_per_segment=4,
        radial_segments=16,
        cap_ends=True,
    )
    main_flight = tube_from_spline_points(
        _helix_points(
            y_start=-0.014,
            y_end=0.085,
            turns=1.72,
            radius_start=0.0275,
            radius_end=0.021,
            phase=rear_phase + (1.12 * math.tau),
            samples_per_turn=24,
        ),
        radius=0.010,
        samples_per_segment=4,
        radial_segments=16,
        cap_ends=True,
    )

    geometry.merge(rear_flight)
    geometry.merge(main_flight)
    return geometry


def _build_chamber_shell_mesh() -> MeshGeometry:
    return _build_shell_from_loops(
        outer_bottom=_circle_loop_y(0.050, -0.112, segments=40),
        outer_top=_circle_loop_y(0.047, 0.112, segments=40),
        inner_top=_circle_loop_y(0.0405, 0.112, segments=40),
        inner_bottom=_circle_loop_y(0.0430, -0.112, segments=40),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cold_press_slow_juicer")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.22, 0.24, 0.27, 0.62))
    matte_black = model.material("matte_black", rgba=(0.09, 0.10, 0.11, 1.0))
    auger_ivory = model.material("auger_ivory", rgba=(0.95, 0.94, 0.86, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.36, 0.29, 0.38)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.03, 0.19)),
    )

    base_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.36, 0.25, 0.030, corner_segments=8),
        0.155,
    )
    body.visual(
        _save_mesh("juicer_base_shell", base_shell),
        material=body_white,
        name="base_shell",
    )

    upper_shoulder = section_loft(
        [
            _rect_loop(0.24, 0.100, 0.155, center=(0.0, -0.070)),
            _rect_loop(0.215, 0.090, 0.182, center=(0.0, -0.067)),
            _rect_loop(0.180, 0.060, 0.205, center=(0.0, -0.075)),
        ]
    )
    body.visual(
        _save_mesh("juicer_upper_shoulder", upper_shoulder),
        material=body_white,
        name="upper_shoulder",
    )

    for foot_name, x_pos, y_pos in (
        ("rear_left_foot", -0.115, -0.078),
        ("rear_right_foot", 0.115, -0.078),
        ("front_left_foot", -0.115, 0.078),
        ("front_right_foot", 0.115, 0.078),
    ):
        body.visual(
            Box((0.050, 0.034, 0.018)),
            origin=Origin(xyz=(x_pos, y_pos, 0.009)),
            material=rubber,
            name=foot_name,
        )

    feed_chute_shell = _build_shell_from_loops(
        outer_bottom=_rect_loop(0.086, 0.048, 0.266, center=(0.0, -0.008)),
        outer_top=_rect_loop(0.102, 0.066, 0.273, center=(0.0, 0.012)),
        inner_top=_rect_loop(0.074, 0.041, 0.273, center=(0.0, 0.012)),
        inner_bottom=_rect_loop(0.060, 0.026, 0.266, center=(0.0, -0.008)),
    )
    body.visual(
        _save_mesh("juicer_feed_chute_shell", feed_chute_shell),
        material=smoked_clear,
        name="feed_chute_shell",
    )

    hopper_funnel = _build_shell_from_loops(
        outer_bottom=_rect_loop(0.102, 0.066, 0.273, center=(0.0, 0.012)),
        outer_top=_rect_loop(0.196, 0.146, 0.360, center=(0.0, -0.004)),
        inner_top=_rect_loop(0.168, 0.118, 0.360, center=(0.0, -0.004)),
        inner_bottom=_rect_loop(0.074, 0.041, 0.273, center=(0.0, 0.012)),
    )
    body.visual(
        _save_mesh("juicer_hopper_funnel", hopper_funnel),
        material=smoked_clear,
        name="hopper_funnel",
    )

    body.visual(
        _save_mesh("juicer_auger_chamber_shell", _build_chamber_shell_mesh()),
        origin=Origin(xyz=(0.0, 0.095, 0.220)),
        material=smoked_clear,
        name="auger_chamber_shell",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.0, -0.003, 0.220), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="drive_socket",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.040),
        origin=Origin(xyz=(0.0, -0.030, 0.220), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=body_white,
        name="motor_collar",
    )

    body.visual(
        Cylinder(radius=0.0075, length=0.040),
        origin=Origin(xyz=(-0.060, -0.075, 0.373), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.040),
        origin=Origin(xyz=(0.060, -0.075, 0.373), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="right_hinge_barrel",
    )
    body.visual(
        Box((0.046, 0.032, 0.172)),
        origin=Origin(xyz=(-0.062, -0.089, 0.282)),
        material=matte_black,
        name="left_hinge_tower",
    )
    body.visual(
        Box((0.046, 0.032, 0.172)),
        origin=Origin(xyz=(0.062, -0.089, 0.282)),
        material=matte_black,
        name="right_hinge_tower",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.20, 0.16, 0.04)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.084, -0.004)),
    )
    lid.visual(
        Box((0.198, 0.144, 0.010)),
        origin=Origin(xyz=(0.0, 0.084, -0.008)),
        material=smoked_clear,
        name="lid_panel",
    )
    lid.visual(
        Box((0.090, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.150, -0.007)),
        material=matte_black,
        name="lid_grip",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.026, 0.020, 0.012)),
        origin=Origin(xyz=(-0.020, 0.011, -0.006)),
        material=matte_black,
        name="left_hinge_leaf",
    )
    lid.visual(
        Box((0.026, 0.020, 0.012)),
        origin=Origin(xyz=(0.020, 0.011, -0.006)),
        material=matte_black,
        name="right_hinge_leaf",
    )

    auger = model.part("auger")
    auger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.200),
        mass=0.48,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )
    auger.visual(
        _save_mesh("juicer_auger_assembly", _build_auger_mesh()),
        material=auger_ivory,
        name="auger_assembly",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.075, 0.373)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "body_to_auger",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=auger,
        origin=Origin(xyz=(0.0, 0.095, 0.220)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=3.0,
        ),
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

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    auger = object_model.get_part("auger")
    lid_hinge = object_model.get_articulation("body_to_lid")
    auger_spin = object_model.get_articulation("body_to_auger")

    ctx.check(
        "lid hinge uses rear-edge revolute axis",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.axis == (1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > 1.0,
        details=f"axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "auger spins on horizontal continuous axis",
        auger_spin.articulation_type == ArticulationType.CONTINUOUS
        and auger_spin.axis == (0.0, 1.0, 0.0)
        and auger_spin.motion_limits is not None
        and auger_spin.motion_limits.lower is None
        and auger_spin.motion_limits.upper is None,
        details=f"axis={auger_spin.axis}, limits={auger_spin.motion_limits}",
    )

    with ctx.pose({lid_hinge: 0.0, auger_spin: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="hopper_funnel",
            min_overlap=0.12,
            name="closed lid covers hopper opening",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="hopper_funnel",
            min_gap=-0.001,
            max_gap=0.008,
            name="closed lid sits close above hopper rim",
        )
        ctx.expect_within(
            auger,
            body,
            axes="xz",
            inner_elem="auger_assembly",
            outer_elem="auger_chamber_shell",
            margin=0.003,
            name="auger stays inside chamber diameter",
        )
        ctx.expect_overlap(
            auger,
            body,
            axes="y",
            elem_a="auger_assembly",
            elem_b="auger_chamber_shell",
            min_overlap=0.16,
            name="auger spans the chamber length",
        )
        ctx.expect_contact(
            auger,
            body,
            elem_a="auger_assembly",
            elem_b="drive_socket",
            contact_tol=0.0015,
            name="auger is supported by rear drive socket",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))

    grip_closed = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_grip"))
    with ctx.pose({lid_hinge: math.radians(65.0)}):
        grip_open = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_grip"))

    ctx.check(
        "lid grip rises when opened",
        grip_closed is not None
        and grip_open is not None
        and grip_open[2] > grip_closed[2] + 0.07
        and grip_open[1] < grip_closed[1] - 0.03,
        details=f"closed={grip_closed}, open={grip_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
