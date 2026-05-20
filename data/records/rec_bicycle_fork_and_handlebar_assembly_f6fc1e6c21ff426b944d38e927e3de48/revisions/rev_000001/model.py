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
    CapsuleGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


HEAD_TILT = math.radians(17.5)
STEER_AXIS = (-math.sin(HEAD_TILT), 0.0, math.cos(HEAD_TILT))
STEERER_RADIUS = 0.0143
HEAD_TUBE_INNER = 0.0148
HEAD_TUBE_OUTER = 0.0290


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in vector))
    return tuple(component / length for component in vector)


def _rpy_from_z(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    dx, dy, dz = _normalize(direction)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _axis_point(distance: float) -> tuple[float, float, float]:
    ax, ay, az = STEER_AXIS
    return (ax * distance, ay * distance, az * distance)


def _shell_tube(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 56,
):
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
    )


def _orient_mesh(geometry, center: tuple[float, float, float], direction: tuple[float, float, float]):
    _, pitch, yaw = _rpy_from_z(direction)
    return geometry.copy().rotate_y(pitch).rotate_z(yaw).translate(*center)


def _mirror_points(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_commuter_fork_stem_handlebar")

    painted_steel = model.material("painted_steel", rgba=(0.19, 0.21, 0.24, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    silver_steel = model.material("silver_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))

    frame_stub = model.part("frame_stub")
    frame_stub.inertial = Inertial.from_geometry(
        Box((0.24, 0.10, 0.34)),
        mass=1.6,
        origin=Origin(xyz=(-0.03, 0.0, 0.10)),
    )

    head_tube_geom = _orient_mesh(
        _shell_tube(
            outer_radius=HEAD_TUBE_OUTER,
            inner_radius=HEAD_TUBE_INNER,
            length=0.160,
        ),
        _axis_point(0.080),
        STEER_AXIS,
    )
    head_tube_geom.merge(
        _orient_mesh(
            _shell_tube(
                outer_radius=0.0325,
                inner_radius=HEAD_TUBE_INNER,
                length=0.020,
            ),
            _axis_point(0.010),
            STEER_AXIS,
        )
    )
    head_tube_geom.merge(
        _orient_mesh(
            _shell_tube(
                outer_radius=0.0320,
                inner_radius=HEAD_TUBE_INNER,
                length=0.020,
            ),
            _axis_point(0.150),
            STEER_AXIS,
        )
    )
    frame_stub.visual(
        mesh_from_geometry(head_tube_geom, "frame_stub_head_tube"),
        material=satin_black,
        name="head_tube_shell",
    )

    frame_tubes_geom = tube_from_spline_points(
        [
            (-0.080, 0.0, 0.032),
            (-0.126, 0.0, -0.004),
            (-0.182, 0.0, -0.050),
            (-0.238, 0.0, -0.092),
        ],
        radius=0.018,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    frame_tubes_geom.merge(
        tube_from_spline_points(
            [
                (-0.078, 0.0, 0.128),
                (-0.120, 0.0, 0.138),
                (-0.168, 0.0, 0.150),
                (-0.214, 0.0, 0.166),
            ],
            radius=0.013,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        )
    )
    frame_tubes_geom.merge(BoxGeometry((0.026, 0.034, 0.040)).translate(-0.082, 0.0, 0.040))
    frame_tubes_geom.merge(BoxGeometry((0.022, 0.028, 0.026)).translate(-0.078, 0.0, 0.126))
    frame_stub.visual(
        mesh_from_geometry(frame_tubes_geom, "frame_stub_tubes"),
        material=painted_steel,
        name="frame_tubes",
    )

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.16, 0.13, 0.70)),
        mass=1.2,
        origin=Origin(xyz=(0.01, 0.0, -0.08)),
    )

    fork.visual(
        Cylinder(radius=STEERER_RADIUS, length=0.300),
        origin=Origin(
            xyz=_axis_point(0.130),
            rpy=_rpy_from_z(STEER_AXIS),
        ),
        material=silver_steel,
        name="steerer",
    )

    left_blade_top = (0.016, 0.037, -0.060)
    left_blade_mid = (0.028, 0.043, -0.228)
    left_blade_tip = (0.046, 0.050, -0.388)
    right_blade_top = (left_blade_top[0], -left_blade_top[1], left_blade_top[2])
    right_blade_mid = (left_blade_mid[0], -left_blade_mid[1], left_blade_mid[2])
    right_blade_tip = (left_blade_tip[0], -left_blade_tip[1], left_blade_tip[2])

    fork_geom = _orient_mesh(
        CapsuleGeometry(radius=0.015, length=0.080, radial_segments=28, height_segments=8),
        (0.018, 0.0, -0.050),
        (0.0, 1.0, 0.0),
    )
    fork_geom.merge(
        _orient_mesh(
            CylinderGeometry(radius=0.0165, height=0.020, radial_segments=28),
            _axis_point(-0.022),
            STEER_AXIS,
        )
    )
    fork_geom.merge(
        _orient_mesh(
            _shell_tube(
                outer_radius=0.0325,
                inner_radius=STEERER_RADIUS,
                length=0.008,
                segments=40,
            ),
            _axis_point(-0.004),
            STEER_AXIS,
        )
    )

    for start, mid, tip, side in (
        (left_blade_top, left_blade_mid, left_blade_tip, 1.0),
        (right_blade_top, right_blade_mid, right_blade_tip, -1.0),
    ):
        upper_dir = (mid[0] - start[0], mid[1] - start[1], mid[2] - start[2])
        lower_dir = (tip[0] - mid[0], tip[1] - mid[1], tip[2] - mid[2])
        upper_center = tuple((a + b) * 0.5 for a, b in zip(start, mid))
        lower_center = tuple((a + b) * 0.5 for a, b in zip(mid, tip))
        upper_len = math.dist(start, mid) + 0.006
        lower_len = math.dist(mid, tip) + 0.006

        fork_geom.merge(
            _orient_mesh(
                CylinderGeometry(radius=0.0092, height=upper_len, radial_segments=28),
                upper_center,
                upper_dir,
            )
        )
        fork_geom.merge(
            _orient_mesh(
                CylinderGeometry(radius=0.0080, height=lower_len, radial_segments=28),
                lower_center,
                lower_dir,
            )
        )
        fork_geom.merge(
            BoxGeometry((0.008, 0.026, 0.030)).translate(tip[0], tip[1], tip[2] - 0.008)
        )
        fork_geom.merge(
            BoxGeometry((0.004, 0.010, 0.018)).translate(
                tip[0] - 0.011,
                tip[1] + side * 0.010,
                tip[2] + 0.002,
            )
        )
        fork_geom.merge(
            _orient_mesh(
                CylinderGeometry(radius=0.0042, height=0.006, radial_segments=18),
                (tip[0] - 0.013, tip[1] + side * 0.010, tip[2] + 0.002),
                (1.0, 0.0, 0.0),
            )
        )

    fork.visual(
        mesh_from_geometry(fork_geom, "fork_structure"),
        material=painted_steel,
        name="fork_structure",
    )

    stem = model.part("stem")
    stem.inertial = Inertial.from_geometry(
        Box((0.20, 0.06, 0.13)),
        mass=0.5,
        origin=Origin(xyz=(0.074, 0.0, 0.050)),
    )

    stem_body_geom = _orient_mesh(
        _shell_tube(
            outer_radius=0.0225,
            inner_radius=STEERER_RADIUS + 0.0009,
            length=0.042,
        ),
        (0.0, 0.0, 0.0),
        STEER_AXIS,
    )
    stem_body_geom.merge(
        tube_from_spline_points(
            [
                (0.010, 0.0, 0.008),
                (0.042, 0.0, 0.025),
                (0.086, 0.0, 0.046),
                (0.120, 0.0, 0.060),
            ],
            radius=0.0125,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        )
    )
    stem_body_geom.merge(BoxGeometry((0.028, 0.028, 0.018)).translate(0.110, 0.0, 0.060))
    stem.visual(
        mesh_from_geometry(stem_body_geom, "stem_body"),
        material=satin_black,
        name="steerer_clamp_body",
    )

    bar_clamp_center = (0.148, 0.0, 0.086)
    bar_collar_geom = _orient_mesh(
        _shell_tube(
            outer_radius=0.0265,
            inner_radius=0.0175,
            length=0.056,
        ),
        bar_clamp_center,
        (0.0, 1.0, 0.0),
    )
    bar_collar_geom.merge(
        BoxGeometry((0.028, 0.036, 0.022)).translate(0.135, 0.0, 0.069)
    )
    bar_collar_geom.merge(
        _orient_mesh(
            CylinderGeometry(radius=0.0048, height=0.020, radial_segments=18),
            (0.164, 0.014, 0.086),
            (1.0, 0.0, 0.0),
        )
    )
    bar_collar_geom.merge(
        _orient_mesh(
            CylinderGeometry(radius=0.0048, height=0.020, radial_segments=18),
            (0.164, -0.014, 0.086),
            (1.0, 0.0, 0.0),
        )
    )
    stem.visual(
        mesh_from_geometry(bar_collar_geom, "stem_bar_collar"),
        material=satin_black,
        name="bar_collar",
    )

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.18, 0.70, 0.10)),
        mass=0.45,
        origin=Origin(xyz=(-0.035, 0.0, -0.004)),
    )

    left_bar_points = [
        (-0.008, -0.070, 0.000),
        (-0.014, -0.120, -0.001),
        (-0.029, -0.212, -0.005),
        (-0.050, -0.286, -0.010),
        (-0.072, -0.335, -0.014),
    ]
    right_bar_points = _mirror_points(left_bar_points)
    bar_geom = tube_from_spline_points(
        left_bar_points,
        radius=0.0111,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    bar_geom.merge(
        tube_from_spline_points(
            right_bar_points,
            radius=0.0111,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        )
    )
    handlebar.visual(
        mesh_from_geometry(
            bar_geom,
            "handlebar_bar_tube",
        ),
        material=painted_steel,
        name="bar_tube",
    )

    center_bulge_geom = _orient_mesh(
        CylinderGeometry(radius=0.0148, height=0.150, radial_segments=28),
        (0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    )
    handlebar.visual(
        mesh_from_geometry(center_bulge_geom, "handlebar_center_bulge"),
        material=satin_black,
        name="center_bulge",
    )

    left_grip_points = [
        (-0.030, 0.220, -0.005),
        (-0.050, 0.286, -0.010),
        (-0.072, 0.335, -0.014),
    ]
    right_grip_points = _mirror_points(left_grip_points)
    grip_geom = tube_from_spline_points(
        left_grip_points,
        radius=0.0170,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    grip_geom.merge(
        tube_from_spline_points(
            right_grip_points,
            radius=0.0170,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
    )

    left_end_dir = tuple(b - a for a, b in zip(left_grip_points[-2], left_grip_points[-1]))
    right_end_dir = tuple(b - a for a, b in zip(right_grip_points[-2], right_grip_points[-1]))
    left_end = left_grip_points[-1]
    right_end = right_grip_points[-1]
    left_flare_center = tuple(left_end[i] - _normalize(left_end_dir)[i] * 0.016 for i in range(3))
    right_flare_center = tuple(right_end[i] - _normalize(right_end_dir)[i] * 0.016 for i in range(3))

    grip_geom.merge(
        _orient_mesh(
            ConeGeometry(radius=0.021, height=0.034, radial_segments=22, closed=True),
            left_flare_center,
            tuple(-component for component in left_end_dir),
        )
    )
    grip_geom.merge(
        _orient_mesh(
            ConeGeometry(radius=0.021, height=0.034, radial_segments=22, closed=True),
            right_flare_center,
            tuple(-component for component in right_end_dir),
        )
    )
    grip_geom.merge(SphereGeometry(radius=0.012, width_segments=18, height_segments=14).translate(*left_end))
    grip_geom.merge(SphereGeometry(radius=0.012, width_segments=18, height_segments=14).translate(*right_end))
    handlebar.visual(
        mesh_from_geometry(grip_geom, "handlebar_grips"),
        material=rubber_black,
        name="grips",
    )

    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=frame_stub,
        child=fork,
        origin=Origin(),
        axis=STEER_AXIS,
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=2.6,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "stem_mount",
        ArticulationType.FIXED,
        parent=fork,
        child=stem,
        origin=Origin(xyz=_axis_point(0.220)),
    )
    model.articulation(
        "bar_mount",
        ArticulationType.FIXED,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=bar_clamp_center),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame_stub = object_model.get_part("frame_stub")
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    steering = object_model.get_articulation("steering")

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
    ctx.allow_overlap(
        fork,
        stem,
        elem_a="steerer",
        elem_b="steerer_clamp_body",
        reason="Threadless stem clamp intentionally closes around the steerer tube.",
    )
    ctx.allow_overlap(
        handlebar,
        stem,
        elem_a="center_bulge",
        elem_b="bar_collar",
        reason="The bar center section is intentionally clamped within the stem collar.",
    )
    ctx.allow_overlap(
        fork,
        frame_stub,
        elem_a="fork_structure",
        elem_b="head_tube_shell",
        reason="The lower headset cover and fork crown race are modeled as nested shells without separate bearing internals.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        fork,
        frame_stub,
        name="fork crown race contacts head tube lower cup",
    )
    ctx.expect_origin_gap(
        stem,
        frame_stub,
        axis="z",
        min_gap=0.14,
        max_gap=0.24,
        name="stem sits above headset stub",
    )
    ctx.expect_origin_gap(
        handlebar,
        frame_stub,
        axis="x",
        min_gap=0.06,
        max_gap=0.16,
        name="riser stem carries bar forward",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="yz",
        elem_a="center_bulge",
        elem_b="bar_collar",
        min_overlap=0.020,
        name="bar bulge stays inside stem clamp footprint",
    )
    ctx.expect_overlap(
        fork,
        stem,
        axes="xz",
        elem_a="steerer",
        elem_b="steerer_clamp_body",
        min_overlap=0.018,
        name="stem clamp remains centered on steerer",
    )

    rest_pos = ctx.part_world_position(handlebar)
    assert rest_pos is not None
    with ctx.pose({steering: 0.85}):
        left_pos = ctx.part_world_position(handlebar)
        assert left_pos is not None
        assert left_pos[1] > rest_pos[1] + 0.055
        ctx.expect_origin_distance(handlebar, frame_stub, axes="xy", min_dist=0.07)
    with ctx.pose({steering: -0.85}):
        right_pos = ctx.part_world_position(handlebar)
        assert right_pos is not None
        assert right_pos[1] < rest_pos[1] - 0.055
        ctx.expect_origin_distance(handlebar, frame_stub, axes="xy", min_dist=0.07)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
