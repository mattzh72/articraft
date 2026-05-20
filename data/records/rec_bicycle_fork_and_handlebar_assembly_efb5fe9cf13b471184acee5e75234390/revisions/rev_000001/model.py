from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


CARBON = Material("satin_carbon_black", rgba=(0.01, 0.011, 0.012, 1.0))
DARK_METAL = Material("dark_anodized_metal", rgba=(0.035, 0.037, 0.040, 1.0))
GUNMETAL = Material("gunmetal", rgba=(0.22, 0.23, 0.24, 1.0))
BRUSHED = Material("brushed_silver", rgba=(0.72, 0.70, 0.66, 1.0))
RUBBER = Material("matte_black_bar_tape", rgba=(0.0, 0.0, 0.0, 1.0))


def _v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _v_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _v_norm(a):
    length = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
    if length < 1.0e-9:
        return (1.0, 0.0, 0.0)
    return (a[0] / length, a[1] / length, a[2] / length)


def _bezier(p0, p1, p2, p3, t):
    u = 1.0 - t
    return (
        u * u * u * p0[0]
        + 3.0 * u * u * t * p1[0]
        + 3.0 * u * t * t * p2[0]
        + t * t * t * p3[0],
        u * u * u * p0[1]
        + 3.0 * u * u * t * p1[1]
        + 3.0 * u * t * t * p2[1]
        + t * t * t * p3[1],
        u * u * u * p0[2]
        + 3.0 * u * u * t * p1[2]
        + 3.0 * u * t * t * p2[2]
        + t * t * t * p3[2],
    )


def _fork_blade_mesh(side: float) -> MeshGeometry:
    """Curved, tapered, oval-section fork blade for one side of the fork."""
    controls = (
        (0.004, side * 0.036, -0.022),
        (0.020, side * 0.038, -0.130),
        (0.054, side * 0.047, -0.285),
        (0.068, side * 0.052, -0.370),
    )
    rings = 20
    segments = 18
    centers = [_bezier(*controls, i / (rings - 1)) for i in range(rings)]

    geom = MeshGeometry()
    ring_ids: list[list[int]] = []
    lateral = (0.0, 1.0, 0.0)
    for i, center in enumerate(centers):
        t = i / (rings - 1)
        if i == 0:
            tangent = _v_sub(centers[1], centers[0])
        elif i == rings - 1:
            tangent = _v_sub(centers[-1], centers[-2])
        else:
            tangent = _v_sub(centers[i + 1], centers[i - 1])
        tangent = _v_norm(tangent)
        fore_aft = _v_norm(_v_cross(lateral, tangent))

        # Road fork blades are deeper fore-aft near the crown and flatten at the dropout.
        r_lateral = 0.0085 * (1.0 - t) + 0.0044 * t
        r_fore_aft = 0.0155 * (1.0 - t) + 0.0060 * t

        ids = []
        for j in range(segments):
            theta = 2.0 * math.pi * j / segments
            offset = _v_add(
                _v_mul(lateral, math.cos(theta) * r_lateral),
                _v_mul(fore_aft, math.sin(theta) * r_fore_aft),
            )
            ids.append(geom.add_vertex(*_v_add(center, offset)))
        ring_ids.append(ids)

    for i in range(rings - 1):
        for j in range(segments):
            a = ring_ids[i][j]
            b = ring_ids[i][(j + 1) % segments]
            c = ring_ids[i + 1][j]
            d = ring_ids[i + 1][(j + 1) % segments]
            geom.add_face(a, c, b)
            geom.add_face(b, c, d)

    start_center = geom.add_vertex(*centers[0])
    end_center = geom.add_vertex(*centers[-1])
    for j in range(segments):
        geom.add_face(start_center, ring_ids[0][(j + 1) % segments], ring_ids[0][j])
        geom.add_face(end_center, ring_ids[-1][j], ring_ids[-1][(j + 1) % segments])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rigid_road_bike_fork")
    model.materials.extend([CARBON, DARK_METAL, GUNMETAL, BRUSHED, RUBBER])

    # A short stationary head-tube shell gives the steering assembly a real bearing
    # reference while leaving clearance around the rotating steerer.
    head_tube = model.part("head_tube")
    head_shell = LatheGeometry.from_shell_profiles(
        [(0.033, 0.105), (0.033, 0.390)],
        [(0.0205, 0.105), (0.0205, 0.390)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    head_tube.visual(
        mesh_from_geometry(head_shell, "head_tube_shell"),
        material=GUNMETAL,
        name="head_shell",
    )

    steerer = model.part("steerer")
    steerer_profile = [
        (0.0, 0.000),
        (0.0190, 0.000),
        (0.0168, 0.120),
        (0.0142, 0.550),
        (0.0, 0.550),
    ]
    steerer.visual(
        mesh_from_geometry(
            LatheGeometry(steerer_profile, segments=72, closed=True),
            "tapered_steerer",
        ),
        material=DARK_METAL,
        name="steerer_taper",
    )
    steerer.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.396)),
        material=BRUSHED,
        name="top_cap",
    )
    steerer.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=BRUSHED,
        name="lower_bearing_race",
    )
    steerer.visual(
        Cylinder(radius=0.023, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=BRUSHED,
        name="steerer_end_cap",
    )

    steer_joint = model.articulation(
        "steering_axis",
        ArticulationType.CONTINUOUS,
        parent=head_tube,
        child=steerer,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.5),
    )

    fork = model.part("fork")
    fork.visual(
        mesh_from_geometry(_fork_blade_mesh(-1.0), "fork_blade_0"),
        material=CARBON,
        name="blade_0",
    )
    fork.visual(
        mesh_from_geometry(_fork_blade_mesh(1.0), "fork_blade_1"),
        material=CARBON,
        name="blade_1",
    )
    fork.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.006, -0.048, -0.042),
                    (0.020, -0.020, -0.026),
                    (0.020, 0.020, -0.026),
                    (0.006, 0.048, -0.042),
                ],
                radius=0.018,
                samples_per_segment=12,
                radial_segments=24,
                cap_ends=True,
            ),
            "arched_crown_bridge",
        ),
        material=CARBON,
        name="crown_bridge",
    )
    fork.visual(
        Cylinder(radius=0.025, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=CARBON,
        name="crown_socket",
    )
    for idx, side in enumerate((-1.0, 1.0)):
        fork.visual(
            Box((0.050, 0.008, 0.038)),
            origin=Origin(xyz=(0.070, side * 0.054, -0.374)),
            material=CARBON,
            name=f"dropout_{idx}",
        )
        fork.visual(
            Cylinder(radius=0.0075, length=0.0025),
            origin=Origin(
                xyz=(0.083, side * 0.059, -0.374),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=BRUSHED,
            name=f"axle_seat_{idx}",
        )

    model.articulation(
        "steerer_to_fork",
        ArticulationType.FIXED,
        parent=steerer,
        child=fork,
        origin=Origin(),
    )

    stem = model.part("stem")
    stem_collar = LatheGeometry.from_shell_profiles(
        [(0.029, 0.440), (0.029, 0.520)],
        [(0.0142, 0.440), (0.0142, 0.520)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    stem.visual(
        mesh_from_geometry(stem_collar, "stem_steerer_clamp"),
        material=DARK_METAL,
        name="steerer_clamp",
    )
    stem.visual(
        Box((0.236, 0.034, 0.028)),
        origin=Origin(xyz=(0.144, 0.0, 0.490)),
        material=DARK_METAL,
        name="stem_body",
    )
    stem.visual(
        Box((0.027, 0.118, 0.060)),
        origin=Origin(xyz=(0.2625, 0.0, 0.515)),
        material=DARK_METAL,
        name="bar_cradle",
    )

    model.articulation(
        "steerer_to_stem",
        ArticulationType.FIXED,
        parent=steerer,
        child=stem,
        origin=Origin(),
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.0122, length=0.170),
        origin=Origin(xyz=(0.285, 0.0, 0.515), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=RUBBER,
        name="center_bar",
    )
    for idx, side in enumerate((-1.0, 1.0)):
        side_drop = tube_from_spline_points(
            [
                (0.285, side * 0.078, 0.515),
                (0.285, side * 0.140, 0.515),
                (0.285, side * 0.185, 0.515),
                (0.335, side * 0.205, 0.505),
                (0.365, side * 0.215, 0.465),
                (0.360, side * 0.225, 0.405),
                (0.315, side * 0.230, 0.365),
            ],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=24,
            cap_ends=True,
        )
        handlebar.visual(
            mesh_from_geometry(side_drop, f"drop_section_{idx}"),
            material=RUBBER,
            name=f"drop_{idx}",
        )

    model.articulation(
        "stem_to_handlebar",
        ArticulationType.FIXED,
        parent=stem,
        child=handlebar,
        origin=Origin(),
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Box((0.007, 0.112, 0.056)),
        origin=Origin(xyz=(0.2985, 0.0, 0.515)),
        material=DARK_METAL,
        name="plate",
    )
    for idx, y in enumerate((-0.036, 0.036)):
        faceplate.visual(
            Cylinder(radius=0.0065, length=0.010),
            origin=Origin(xyz=(0.305, y, 0.515), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=BRUSHED,
            name=f"bolt_{idx}",
        )

    model.articulation(
        "stem_to_faceplate",
        ArticulationType.FIXED,
        parent=stem,
        child=faceplate,
        origin=Origin(),
    )

    # Keep a reference in metadata for tests and downstream inspection.
    model.meta["primary_articulation"] = steer_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    steerer = object_model.get_part("steerer")
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    faceplate = object_model.get_part("faceplate")
    steering_axis = object_model.get_articulation("steering_axis")

    ctx.allow_overlap(
        stem,
        steerer,
        elem_a="steerer_clamp",
        elem_b="steerer_taper",
        reason="The threadless stem clamp is modeled with a slight compression fit around the steerer.",
    )
    ctx.allow_overlap(
        stem,
        handlebar,
        elem_a="bar_cradle",
        elem_b="center_bar",
        reason="The rear bar cradle intentionally compresses the handlebar at the clamp.",
    )
    ctx.allow_overlap(
        faceplate,
        handlebar,
        elem_a="plate",
        elem_b="center_bar",
        reason="The faceplate intentionally compresses the handlebar at the clamp.",
    )

    ctx.check(
        "steering uses continuous steerer-axis rotation",
        steering_axis.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in steering_axis.axis) == (0.0, 0.0, 1.0),
        details=f"type={steering_axis.articulation_type}, axis={steering_axis.axis}",
    )
    ctx.expect_within(
        steerer,
        head_tube,
        axes="xy",
        inner_elem="steerer_taper",
        outer_elem="head_shell",
        margin=0.0,
        name="tapered steerer passes through the head-tube bearing bore",
    )
    ctx.expect_overlap(
        steerer,
        head_tube,
        axes="z",
        elem_a="steerer_taper",
        elem_b="head_shell",
        min_overlap=0.24,
        name="steerer remains engaged through the head tube",
    )
    ctx.expect_contact(
        fork,
        steerer,
        elem_a="crown_socket",
        elem_b="steerer_taper",
        contact_tol=0.002,
        name="fork crown seats against the steerer base",
    )
    ctx.expect_within(
        steerer,
        stem,
        axes="xy",
        inner_elem="steerer_taper",
        outer_elem="steerer_clamp",
        margin=0.0,
        name="stem clamp surrounds the steerer",
    )
    ctx.expect_overlap(
        steerer,
        stem,
        axes="z",
        elem_a="steerer_taper",
        elem_b="steerer_clamp",
        min_overlap=0.070,
        name="stem clamp grips a realistic steerer length",
    )
    ctx.expect_gap(
        handlebar,
        stem,
        axis="x",
        positive_elem="center_bar",
        negative_elem="bar_cradle",
        max_penetration=0.006,
        max_gap=0.004,
        name="rear cradle lightly compresses the handlebar center",
    )
    ctx.expect_gap(
        faceplate,
        handlebar,
        axis="x",
        positive_elem="plate",
        negative_elem="center_bar",
        max_penetration=0.006,
        max_gap=0.004,
        name="faceplate lightly compresses the handlebar center",
    )

    def _center_from_aabb(box):
        if box is None:
            return None
        lower, upper = box
        return (
            0.5 * (lower[0] + upper[0]),
            0.5 * (lower[1] + upper[1]),
            0.5 * (lower[2] + upper[2]),
        )

    rest_bar_box = ctx.part_element_world_aabb(handlebar, elem="center_bar")
    rest_center = _center_from_aabb(rest_bar_box)
    with ctx.pose({steering_axis: 0.70}):
        turned_bar_box = ctx.part_element_world_aabb(handlebar, elem="center_bar")
        turned_center = _center_from_aabb(turned_bar_box)
    rest_radius = None if rest_center is None else math.hypot(rest_center[0], rest_center[1])
    turned_radius = None if turned_center is None else math.hypot(turned_center[0], turned_center[1])
    ctx.check(
        "handlebar sweeps naturally around the steerer axis",
        rest_center is not None
        and turned_center is not None
        and turned_center[1] > rest_center[1] + 0.12
        and abs(rest_radius - turned_radius) < 0.025,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
