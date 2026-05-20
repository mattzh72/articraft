from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


AXLE_Z = 1.15
AXLE_X = 0.0
AXLE_Y = 0.0
WHEEL_RADIUS = 0.82
BUCKET_OUTER_RADIUS = 0.93
BUCKET_COUNT = 18
WHEEL_WIDTH = 0.34


def _curved_bucket_geometry(
    *,
    inner_radius: float,
    outer_radius: float,
    start_angle: float,
    angle_span: float,
    width: float,
    skew_angle: float,
    segments: int = 5,
) -> MeshGeometry:
    """Closed, slightly swept annular-sector bucket in wheel-local coordinates."""

    geom = MeshGeometry()
    xs = (-width / 2.0, width / 2.0)
    verts: dict[tuple[int, int, int], int] = {}

    def add_vertex(side: int, layer: int, i: int) -> None:
        fraction = i / segments
        base_angle = start_angle + (angle_span * fraction)
        radius = inner_radius if layer == 0 else outer_radius
        # Offset the inner edge so the solid reads as a curved scoop rather than
        # a flat radial paddle.
        angle = base_angle + (skew_angle if layer == 0 else 0.0)
        verts[(side, layer, i)] = geom.add_vertex(
            xs[side],
            radius * math.cos(angle),
            radius * math.sin(angle),
        )

    def face(a: tuple[int, int, int], b: tuple[int, int, int], c: tuple[int, int, int]) -> None:
        geom.add_face(verts[a], verts[b], verts[c])

    def quad(
        a: tuple[int, int, int],
        b: tuple[int, int, int],
        c: tuple[int, int, int],
        d: tuple[int, int, int],
    ) -> None:
        face(a, b, c)
        face(a, c, d)

    for side in (0, 1):
        for layer in (0, 1):
            for i in range(segments + 1):
                add_vertex(side, layer, i)

    for i in range(segments):
        # Side sheets, visible at the wheel faces.
        quad((0, 0, i), (0, 1, i), (0, 1, i + 1), (0, 0, i + 1))
        quad((1, 1, i), (1, 0, i), (1, 0, i + 1), (1, 1, i + 1))
        # Curved inner and outer bucket boards.
        quad((0, 0, i), (0, 0, i + 1), (1, 0, i + 1), (1, 0, i))
        quad((0, 1, i + 1), (0, 1, i), (1, 1, i), (1, 1, i + 1))

    # End grain caps.
    quad((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0))
    quad((0, 1, segments), (1, 1, segments), (1, 0, segments), (0, 0, segments))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_timber_waterwheel")

    weathered_oak = model.material("weathered_oak", rgba=(0.53, 0.34, 0.17, 1.0))
    dark_endgrain = model.material("dark_endgrain", rgba=(0.23, 0.14, 0.08, 1.0))
    hatch_iron = model.material("dark_hinged_iron", rgba=(0.08, 0.075, 0.065, 1.0))
    blue_water = model.material("thin_water", rgba=(0.22, 0.50, 0.88, 0.62))

    frame = model.part("timber_frame")

    # Ground sills and cross ties.
    for index, x in enumerate((-0.55, 0.55)):
        frame.visual(
            Box((0.18, 2.10, 0.12)),
            origin=Origin(xyz=(x, 0.0, 0.06)),
            material=weathered_oak,
            name=f"sill_{index}",
        )

    for index, y in enumerate((-0.82, 0.82)):
        frame.visual(
            Box((1.35, 0.16, 0.14)),
            origin=Origin(xyz=(0.0, y, 0.13)),
            material=weathered_oak,
            name=f"cross_sill_{index}",
        )

    # Four upright timber posts and side rails supporting the bearing boxes.
    post_index = 0
    for x in (-0.55, 0.55):
        for y in (-0.48, 0.48):
            frame.visual(
                Box((0.17, 0.17, 2.15)),
                origin=Origin(xyz=(x, y, 1.10)),
                material=weathered_oak,
                name=f"post_{post_index}",
            )
            post_index += 1

    for index, x in enumerate((-0.55, 0.55)):
        for segment, y in enumerate((-0.42, 0.42)):
            frame.visual(
                Box((0.18, 0.40, 0.17)),
                origin=Origin(xyz=(x, y, AXLE_Z)),
                material=weathered_oak,
                name=f"side_rail_{index}_{segment}",
            )
        frame.visual(
            Box((0.18, 1.04, 0.14)),
            origin=Origin(xyz=(x, 0.0, 0.58)),
            material=weathered_oak,
            name=f"lower_rail_{index}",
        )

    frame.visual(
        Box((1.36, 1.12, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 2.16)),
        material=weathered_oak,
        name="top_tie",
    )

    # Bearing support boxes: the right-hand one carries the service hatch.
    for index, x in enumerate((-0.55, 0.55)):
        frame.visual(
            Box((0.24, 0.70, 0.55)),
            origin=Origin(xyz=(x, 0.0, AXLE_Z)),
            material=dark_endgrain,
            name=f"bearing_box_{index}",
        )

    # Short overshot flume, tied into the top timber and aimed just above the wheel.
    frame.visual(
        Box((0.34, 0.88, 0.055)),
        origin=Origin(xyz=(0.0, -0.50, 2.15)),
        material=weathered_oak,
        name="flume_floor",
    )
    for index, x in enumerate((-0.19, 0.19)):
        frame.visual(
            Box((0.055, 0.88, 0.18)),
            origin=Origin(xyz=(x, -0.50, 2.215)),
            material=weathered_oak,
            name=f"flume_side_{index}",
        )
    frame.visual(
        Box((0.40, 0.70, 0.018)),
        origin=Origin(xyz=(0.0, -0.50, 2.21)),
        material=blue_water,
        name="water_ribbon",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                WHEEL_RADIUS,
                0.29,
                rim=WheelRim(inner_radius=0.66, flange_height=0.018, flange_thickness=0.010),
                hub=WheelHub(radius=0.13, width=0.31, cap_style="flat"),
                face=WheelFace(dish_depth=0.012, front_inset=0.004, rear_inset=0.004),
                spokes=WheelSpokes(style="straight", count=12, thickness=0.025, window_radius=0.035),
                bore=WheelBore(style="round", diameter=0.10),
            ),
            "waterwheel_spoked_rim",
        ),
        material=weathered_oak,
        name="spoked_rim",
    )
    wheel.visual(
        Cylinder(radius=0.055, length=1.18),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hatch_iron,
        name="axle",
    )

    bucket_span = (2.0 * math.pi / BUCKET_COUNT) * 0.64
    for index in range(BUCKET_COUNT):
        theta = (2.0 * math.pi * index / BUCKET_COUNT) - (bucket_span / 2.0)
        bucket_mesh = _curved_bucket_geometry(
            inner_radius=0.76,
            outer_radius=BUCKET_OUTER_RADIUS,
            start_angle=theta,
            angle_span=bucket_span,
            width=WHEEL_WIDTH,
            skew_angle=0.08,
        )
        wheel.visual(
            mesh_from_geometry(bucket_mesh, f"curved_bucket_{index:02d}"),
            material=weathered_oak,
            name=f"bucket_{index:02d}",
        )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(AXLE_X, AXLE_Y, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5),
    )

    hatch = model.part("service_hatch")
    # The child frame lies on the small vertical hinge pin just proud of the
    # outer face of the positive-X bearing box.
    hatch.visual(
        Box((0.030, 0.24, 0.24)),
        origin=Origin(xyz=(-0.015, 0.12, 0.0)),
        material=hatch_iron,
        name="panel",
    )
    hatch.visual(
        Cylinder(radius=0.018, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hatch_iron,
        name="hinge_barrel",
    )
    hatch.visual(
        Box((0.018, 0.055, 0.075)),
        origin=Origin(xyz=(0.009, 0.18, 0.0)),
        material=dark_endgrain,
        name="pull_lug",
    )

    model.articulation(
        "frame_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hatch,
        origin=Origin(xyz=(0.70, -0.32, 1.00)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("timber_frame")
    wheel = object_model.get_part("wheel")
    hatch = object_model.get_part("service_hatch")
    wheel_joint = object_model.get_articulation("frame_to_wheel")
    hatch_joint = object_model.get_articulation("frame_to_service_hatch")

    for bearing_name in ("bearing_box_0", "bearing_box_1"):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=bearing_name,
            elem_b="axle",
            reason="The iron axle journal is intentionally captured inside the solid timber bearing support proxy.",
        )
        ctx.expect_within(
            wheel,
            frame,
            axes="yz",
            inner_elem="axle",
            outer_elem=bearing_name,
            margin=0.0,
            name=f"axle centered through {bearing_name}",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="x",
            elem_a="axle",
            elem_b=bearing_name,
            min_overlap=0.08,
            name=f"axle retained by {bearing_name}",
        )

    ctx.expect_gap(
        hatch,
        frame,
        axis="x",
        positive_elem="panel",
        negative_elem="bearing_box_1",
        max_gap=0.002,
        max_penetration=0.001,
        name="service hatch sits flush on bearing box side",
    )

    wheel_aabb = ctx.part_world_aabb(wheel)
    ctx.check("wheel_has_large_overshot_diameter", wheel_aabb is not None, "Expected wheel AABB.")
    if wheel_aabb is not None:
        mins, maxs = wheel_aabb
        diameter = max(float(maxs[1] - mins[1]), float(maxs[2] - mins[2]))
        width = float(maxs[0] - mins[0])
        ctx.check("wheel_diameter_is_plausible", 1.75 <= diameter <= 1.95, f"diameter={diameter:.3f}")
        ctx.check("wheel_has_side_width", 0.32 <= width <= 1.25, f"width={width:.3f}")

    bucket_visuals = [visual for visual in wheel.visuals if visual.name and visual.name.startswith("bucket_")]
    ctx.check(
        "curved_buckets_ring_present",
        len(bucket_visuals) == BUCKET_COUNT,
        f"expected {BUCKET_COUNT} curved buckets, found {len(bucket_visuals)}",
    )

    ctx.check(
        "wheel_axis_is_horizontal",
        tuple(wheel_joint.axis) == (1.0, 0.0, 0.0),
        f"axis={getattr(wheel_joint, 'axis', None)!r}",
    )

    rest_aabb = ctx.part_world_aabb(hatch)
    with ctx.pose({hatch_joint: 1.20}):
        open_aabb = ctx.part_world_aabb(hatch)
    ctx.check(
        "service_hatch_opens_outward",
        rest_aabb is not None
        and open_aabb is not None
        and float(open_aabb[1][0]) > float(rest_aabb[1][0]) + 0.08,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
