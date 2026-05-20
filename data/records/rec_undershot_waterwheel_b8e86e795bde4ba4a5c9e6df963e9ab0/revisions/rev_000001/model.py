from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_undershot_waterwheel")

    painted = model.material("satin_graphite_paint", rgba=(0.10, 0.12, 0.13, 1.0))
    dark_paint = model.material("deep_blue_black_paint", rgba=(0.03, 0.055, 0.075, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    polymer = model.material("warm_grey_polymer", rgba=(0.23, 0.24, 0.24, 1.0))
    elastomer = model.material("matte_black_elastomer", rgba=(0.012, 0.012, 0.010, 1.0))
    water = model.material("clear_channel_water", rgba=(0.10, 0.32, 0.46, 0.42))

    axle_z = 0.72
    frame = model.part("frame")

    # Low, broad base and open water trough.  The trough sits directly under the
    # undershot blades and ties both side frames into one supported assembly.
    frame.visual(
        Box((1.28, 0.74, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_paint,
        name="base_plinth",
    )
    frame.visual(
        Box((1.04, 0.44, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=painted,
        name="trough_floor",
    )
    for x, name in ((-0.275, "trough_wall_0"), (0.275, "trough_wall_1")):
        frame.visual(
            Box((0.034, 0.44, 0.096)),
            origin=Origin(xyz=(x, 0.0, 0.127)),
            material=painted,
            name=name,
        )
    frame.visual(
        Box((0.50, 0.355, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=water,
        name="water_sheet",
    )
    for x in (-0.49, 0.49):
        for y in (-0.29, 0.29):
            frame.visual(
                Box((0.18, 0.10, 0.018)),
                origin=Origin(xyz=(x, y, -0.005)),
                material=elastomer,
                name=f"foot_{'n' if y < 0 else 'p'}_{'l' if x < 0 else 'r'}",
            )

    # Two continuous, filleted side frames plus cross tubes.  The lower bridge
    # under each bearing slightly intersects the bearing ring, making the load
    # path explicit without crossing the spinning axle.
    side_frame_meshes = []
    for x, suffix in ((-0.52, "0"), (0.52, "1")):
        side_geom = wire_from_points(
            [
                (x, -0.320, 0.088),
                (x, -0.090, axle_z - 0.092),
                (x, 0.090, axle_z - 0.092),
                (x, 0.320, 0.088),
            ],
            radius=0.018,
            radial_segments=16,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=0.048,
            corner_segments=10,
        )
        side_frame_meshes.append(mesh_from_geometry(side_geom, f"side_frame_{suffix}"))
        frame.visual(
            side_frame_meshes[-1],
            material=painted,
            name=f"side_frame_{suffix}",
        )

    for y, name in ((-0.320, "rear_cross_tube"), (0.320, "front_cross_tube")):
        frame.visual(
            Cylinder(radius=0.015, length=1.08),
            origin=Origin(xyz=(0.0, y, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name=name,
        )

    bearing_mesh = mesh_from_geometry(
        TorusGeometry(0.066, 0.020, radial_segments=24, tubular_segments=48).rotate_y(math.pi / 2.0),
        "bearing_outer",
    )
    seal_mesh = mesh_from_geometry(
        TorusGeometry(0.045, 0.008, radial_segments=20, tubular_segments=40).rotate_y(math.pi / 2.0),
        "bearing_seal",
    )
    liner_mesh = mesh_from_geometry(
        TorusGeometry(0.034, 0.010, radial_segments=20, tubular_segments=40).rotate_y(math.pi / 2.0),
        "bearing_liner",
    )
    for x, suffix in ((-0.52, "0"), (0.52, "1")):
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(x, 0.0, axle_z)),
            material=aluminum,
            name=f"bearing_{suffix}",
        )
        frame.visual(
            seal_mesh,
            origin=Origin(xyz=(x, 0.0, axle_z)),
            material=elastomer,
            name=f"seal_{suffix}",
        )
        frame.visual(
            liner_mesh,
            origin=Origin(xyz=(x, 0.0, axle_z)),
            material=polymer,
            name=f"bearing_liner_{suffix}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.026, length=1.18),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.082, length=0.50),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="hub_drum",
    )
    for x, suffix in ((-0.262, "0"), (0.262, "1")):
        wheel.visual(
            Cylinder(radius=0.102, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=elastomer,
            name=f"hub_gasket_{suffix}",
        )

    rim_mesh = mesh_from_geometry(
        TorusGeometry(0.515, 0.023, radial_segments=32, tubular_segments=96).rotate_y(math.pi / 2.0),
        "wheel_outer_rim",
    )
    inner_rim_mesh = mesh_from_geometry(
        TorusGeometry(0.330, 0.014, radial_segments=28, tubular_segments=80).rotate_y(math.pi / 2.0),
        "wheel_inner_rim",
    )
    for x, suffix in ((-0.180, "0"), (0.180, "1")):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=painted,
            name=f"outer_rim_{suffix}",
        )
        wheel.visual(
            inner_rim_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=painted,
            name=f"inner_rim_{suffix}",
        )

    spoke_count = 10
    spoke_center_r = 0.292
    for side_x, side_name in ((-0.160, "0"), (0.160, "1")):
        for i in range(spoke_count):
            theta = 2.0 * math.pi * i / spoke_count
            wheel.visual(
                Box((0.034, 0.026, 0.435)),
                origin=Origin(
                    xyz=(side_x, spoke_center_r * math.sin(theta), spoke_center_r * math.cos(theta)),
                    rpy=(-theta, 0.0, 0.0),
                ),
                material=painted,
                name=f"spoke_{side_name}_{i}",
            )

    # Radial undershot paddle boards: each spans the wheel width and overlaps
    # the two outer rims.  Small elastomer strips on the outer edge read as a
    # replaceable, quiet-running wear surface.
    paddle_count = 16
    for i in range(paddle_count):
        theta = 2.0 * math.pi * i / paddle_count
        wheel.visual(
            Box((0.460, 0.026, 0.135)),
            origin=Origin(
                xyz=(0.0, 0.555 * math.sin(theta), 0.555 * math.cos(theta)),
                rpy=(-theta, 0.0, 0.0),
            ),
            material=polymer,
            name=f"paddle_{i}",
        )
        wheel.visual(
            Box((0.430, 0.030, 0.020)),
            origin=Origin(
                xyz=(0.0, 0.626 * math.sin(theta), 0.626 * math.cos(theta)),
                rpy=(-theta, 0.0, 0.0),
            ),
            material=elastomer,
            name=f"tip_strip_{i}",
        )

    model.articulation(
        "axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.08, friction=0.015),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    axle = object_model.get_articulation("axle")

    ctx.check(
        "wheel spins on x axle",
        axle.axis == (1.0, 0.0, 0.0) and axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"axis={axle.axis}, type={axle.articulation_type}",
    )
    for suffix in ("0", "1"):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=f"bearing_liner_{suffix}",
            elem_b="axle_shaft",
            reason="The low-friction bearing liner is modeled with a tiny interference fit around the centered rotating axle.",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="x",
            elem_a="axle_shaft",
            elem_b=f"bearing_liner_{suffix}",
            min_overlap=0.016,
            name=f"axle retained by bearing liner {suffix}",
        )
        ctx.expect_within(
            wheel,
            frame,
            axes="yz",
            inner_elem="axle_shaft",
            outer_elem=f"bearing_liner_{suffix}",
            margin=0.0,
            name=f"axle centered in liner {suffix}",
        )
    ctx.expect_within(
        wheel,
        frame,
        axes="yz",
        inner_elem="axle_shaft",
        outer_elem="bearing_0",
        margin=0.0,
        name="axle centered in bearing 0",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="yz",
        inner_elem="axle_shaft",
        outer_elem="bearing_1",
        margin=0.0,
        name="axle centered in bearing 1",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="x",
        elem_a="axle_shaft",
        elem_b="bearing_0",
        min_overlap=0.030,
        name="axle passes bearing 0",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="x",
        elem_a="axle_shaft",
        elem_b="bearing_1",
        min_overlap=0.030,
        name="axle passes bearing 1",
    )

    closed_aabb = ctx.part_element_world_aabb(wheel, elem="paddle_0")
    with ctx.pose({axle: math.pi / 2.0}):
        rotated_aabb = ctx.part_element_world_aabb(wheel, elem="paddle_0")

    closed_center = None
    rotated_center = None
    if closed_aabb is not None and rotated_aabb is not None:
        closed_center = tuple((closed_aabb[0][i] + closed_aabb[1][i]) * 0.5 for i in range(3))
        rotated_center = tuple((rotated_aabb[0][i] + rotated_aabb[1][i]) * 0.5 for i in range(3))
    ctx.check(
        "paddles rotate with wheel",
        closed_center is not None
        and rotated_center is not None
        and closed_center[2] > rotated_center[2] + 0.45
        and rotated_center[1] < -0.45,
        details=f"closed={closed_center}, rotated={rotated_center}",
    )

    return ctx.report()


object_model = build_object_model()
