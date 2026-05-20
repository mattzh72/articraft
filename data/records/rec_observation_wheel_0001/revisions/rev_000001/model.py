from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _box_geom(size: tuple[float, float, float], center: tuple[float, float, float]):
    return BoxGeometry(size).translate(*center)


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    radial_segments: int = 18,
):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments, closed=True)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    geom.rotate_y(pitch)
    geom.rotate_z(yaw)
    geom.translate(
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    return geom


def _torus_along_x(radius: float, tube: float, x_offset: float):
    return (
        TorusGeometry(radius, tube, radial_segments=20, tubular_segments=72)
        .rotate_y(math.pi / 2.0)
        .translate(x_offset, 0.0, 0.0)
    )


def _build_support_geometry(hub_height: float):
    frame = tube_from_spline_points(
        [(-0.78, -2.0, 0.28), (-0.72, -0.95, 1.7), (-0.42, 0.0, hub_height)],
        radius=0.10,
        samples_per_segment=16,
        radial_segments=18,
    )
    frame.merge(
        tube_from_spline_points(
            [(-0.78, 2.0, 0.28), (-0.72, 0.95, 1.7), (-0.42, 0.0, hub_height)],
            radius=0.10,
            samples_per_segment=16,
            radial_segments=18,
        )
    )
    frame.merge(
        tube_from_spline_points(
            [(0.78, -2.0, 0.28), (0.72, -0.95, 1.7), (0.42, 0.0, hub_height)],
            radius=0.10,
            samples_per_segment=16,
            radial_segments=18,
        )
    )
    frame.merge(
        tube_from_spline_points(
            [(0.78, 2.0, 0.28), (0.72, 0.95, 1.7), (0.42, 0.0, hub_height)],
            radius=0.10,
            samples_per_segment=16,
            radial_segments=18,
        )
    )

    for x in (-0.78, 0.78):
        frame.merge(_cylinder_between((x, -1.7, 0.42), (x, 1.7, 0.42), 0.06))
        frame.merge(_cylinder_between((x, -1.45, 1.48), (x, 1.45, 1.48), 0.05))
        top_x = -0.42 if x < 0.0 else 0.42
        frame.merge(_cylinder_between((x, -1.25, 1.02), (top_x, 0.0, hub_height - 0.16), 0.045))
        frame.merge(_cylinder_between((x, 1.25, 1.02), (top_x, 0.0, hub_height - 0.16), 0.045))

    frame.merge(_cylinder_between((-0.78, -2.08, 0.34), (0.78, -2.08, 0.34), 0.08))
    frame.merge(_cylinder_between((-0.78, 2.08, 0.34), (0.78, 2.08, 0.34), 0.08))
    frame.merge(_cylinder_between((-0.78, -0.92, 0.54), (0.78, -0.92, 0.54), 0.055))
    frame.merge(_cylinder_between((-0.78, 0.92, 0.54), (0.78, 0.92, 0.54), 0.055))

    frame.merge(_cylinder_between((-0.86, 0.0, hub_height), (-0.22, 0.0, hub_height), 0.12))
    frame.merge(_cylinder_between((0.22, 0.0, hub_height), (0.86, 0.0, hub_height), 0.12))
    frame.merge(_cylinder_between((-0.22, 0.0, hub_height), (0.22, 0.0, hub_height), 0.055))

    frame.merge(_box_geom((0.16, 0.22, 0.34), (-0.86, 0.0, hub_height - 0.04)))
    frame.merge(_box_geom((0.16, 0.22, 0.34), (0.86, 0.0, hub_height - 0.04)))
    return frame


def _build_wheel_geometry(
    wheel_radius: float,
    half_depth: float,
):
    wheel = _torus_along_x(wheel_radius, 0.08, -half_depth)
    wheel.merge(_torus_along_x(wheel_radius, 0.08, half_depth))
    wheel.merge(_torus_along_x(wheel_radius - 0.72, 0.04, -0.12))
    wheel.merge(_torus_along_x(wheel_radius - 0.72, 0.04, 0.12))

    hub = CylinderGeometry(0.19, 0.46, radial_segments=24, closed=True).rotate_y(math.pi / 2.0)
    wheel.merge(hub)
    wheel.merge(
        CylinderGeometry(0.27, 0.05, radial_segments=24, closed=True)
        .rotate_y(math.pi / 2.0)
        .translate(-0.20, 0.0, 0.0)
    )
    wheel.merge(
        CylinderGeometry(0.27, 0.05, radial_segments=24, closed=True)
        .rotate_y(math.pi / 2.0)
        .translate(0.20, 0.0, 0.0)
    )

    spoke_count = 24
    inner_ring = wheel_radius - 0.72
    for i in range(spoke_count):
        angle = math.tau * i / spoke_count
        next_angle = angle + math.pi / spoke_count
        c = math.cos(angle)
        s = math.sin(angle)
        cn = math.cos(next_angle)
        sn = math.sin(next_angle)

        wheel.merge(
            _cylinder_between(
                (-0.12, 0.30 * c, 0.30 * s), (-0.12, inner_ring * c, inner_ring * s), 0.024
            )
        )
        wheel.merge(
            _cylinder_between(
                (0.12, 0.30 * c, 0.30 * s), (0.12, inner_ring * c, inner_ring * s), 0.024
            )
        )
        wheel.merge(
            _cylinder_between(
                (-0.18, inner_ring * c, inner_ring * s),
                (-half_depth, (wheel_radius - 0.14) * cn, (wheel_radius - 0.14) * sn),
                0.020,
            )
        )
        wheel.merge(
            _cylinder_between(
                (0.18, inner_ring * c, inner_ring * s),
                (half_depth, (wheel_radius - 0.14) * cn, (wheel_radius - 0.14) * sn),
                0.020,
            )
        )
        if i % 2 == 0:
            wheel.merge(
                _cylinder_between(
                    (-0.10, 0.34 * c, 0.34 * s),
                    (0.10, inner_ring * cn, inner_ring * sn),
                    0.016,
                )
            )
            wheel.merge(
                _cylinder_between(
                    (0.10, 0.34 * c, 0.34 * s),
                    (-0.10, inner_ring * cn, inner_ring * sn),
                    0.016,
                )
            )

    cabin_count = 12
    for i in range(cabin_count * 2):
        angle = math.tau * i / (cabin_count * 2)
        y = wheel_radius * math.cos(angle)
        z = wheel_radius * math.sin(angle)
        wheel.merge(_cylinder_between((-half_depth, y, z), (half_depth, y, z), 0.028))

    for i in range(cabin_count):
        angle = math.tau * i / cabin_count
        anchor_y = (wheel_radius + 0.02) * math.cos(angle)
        anchor_z = (wheel_radius + 0.02) * math.sin(angle)
        cabin_center = (0.0, anchor_y, anchor_z - 0.34)

        wheel.merge(_box_geom((0.52, 0.30, 0.30), cabin_center))
        wheel.merge(_box_geom((0.58, 0.36, 0.08), (0.0, anchor_y, anchor_z - 0.17)))
        wheel.merge(_box_geom((0.54, 0.32, 0.05), (0.0, anchor_y, anchor_z - 0.51)))
        wheel.merge(_box_geom((0.48, 0.18, 0.18), (0.0, anchor_y, anchor_z - 0.33)))

        wheel.merge(
            _cylinder_between(
                (-0.18, anchor_y, anchor_z), (-0.18, anchor_y, anchor_z - 0.17), 0.018
            )
        )
        wheel.merge(
            _cylinder_between((0.18, anchor_y, anchor_z), (0.18, anchor_y, anchor_z - 0.17), 0.018)
        )
        wheel.merge(
            _cylinder_between(
                (-0.18, anchor_y, anchor_z - 0.17), (0.18, anchor_y, anchor_z - 0.17), 0.016
            )
        )

    return wheel


def build_object_model() -> ArticulatedObject:
    hub_height = 3.75
    wheel_radius = 2.82
    half_depth = 0.22

    model = ArticulatedObject(name="observation_wheel", assets=ASSETS)

    support = model.part("support")
    support.visual(
        Box((2.0, 5.0, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )
    support.visual(
        Box((1.30, 1.75, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
    )
    support.visual(
        Box((0.92, 0.72, 0.16)),
        origin=Origin(xyz=(0.0, -1.25, 0.36)),
    )
    support.visual(
        Box((0.92, 0.72, 0.16)),
        origin=Origin(xyz=(0.0, 1.25, 0.36)),
    )
    support_frame = mesh_from_geometry(
        _build_support_geometry(hub_height), ASSETS.mesh_path("support_frame.obj")
    )
    support.visual(support_frame, origin=Origin(xyz=(0.0, 0.0, 0.0)))
    support.inertial = Inertial.from_geometry(
        Box((2.0, 5.0, 0.28)),
        mass=4800.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    wheel = model.part("wheel")
    wheel_mesh = mesh_from_geometry(
        _build_wheel_geometry(wheel_radius, half_depth),
        ASSETS.mesh_path("wheel_assembly.obj"),
    )
    wheel.visual(wheel_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0)))
    wheel.inertial = Inertial.from_geometry(
        Box((0.70, 6.20, 6.70)),
        mass=2600.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="support",
        child="wheel",
        origin=Origin(xyz=(0.0, 0.0, hub_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(
        tol=0.02,
        reason="The wheel turns around a visible fixed axle and bearing pack, so the closest support geometry sits at the engineered hub clearance.",
    )
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "support",
        "wheel",
        reason="The rotating hub sleeves the fixed axle, and convex collision generation is conservative around the coaxial bearing interface.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("wheel", "support", axes="xy", min_overlap=0.65)
    ctx.expect_origin_distance("wheel", "support", axes="xy", max_dist=0.05)

    with ctx.pose(wheel_spin=math.pi / 4.0):
        ctx.expect_aabb_overlap("wheel", "support", axes="xy", min_overlap=0.65)
        ctx.expect_origin_distance("wheel", "support", axes="xy", max_dist=0.05)

    with ctx.pose(wheel_spin=math.pi / 2.0):
        ctx.expect_aabb_overlap("wheel", "support", axes="xy", min_overlap=0.65)
        ctx.expect_origin_distance("wheel", "support", axes="xy", max_dist=0.05)

    with ctx.pose(wheel_spin=math.pi):
        ctx.expect_aabb_overlap("wheel", "support", axes="xy", min_overlap=0.65)
        ctx.expect_origin_distance("wheel", "support", axes="xy", max_dist=0.05)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
