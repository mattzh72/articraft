from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 48,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (cx + radius * cos(2.0 * pi * i / segments), cy + radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_twin_post_lift_module")

    painted_base = model.material("powder_coated_black", rgba=(0.035, 0.037, 0.040, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.64, 0.06, 1.0))
    bolt_black = model.material("blackened_bolts", rgba=(0.015, 0.015, 0.016, 1.0))
    bronze = model.material("bronze_bushings", rgba=(0.72, 0.46, 0.22, 1.0))

    post_x = 0.28
    post_radius = 0.032
    post_height = 1.12
    base_top_z = 0.10

    frame = model.part("frame")

    # Heavy rounded foot plate.
    base_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.88, 0.38, 0.055, corner_segments=10),
        [],
        height=0.10,
        center=True,
    )
    frame.visual(
        _mesh("heavy_foot", base_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=painted_base,
        name="heavy_foot",
    )

    # Welded/bolted circular bosses tie the round posts into the foot.
    for i, x in enumerate((-post_x, post_x)):
        frame.visual(
            Cylinder(radius=0.077, length=0.040),
            origin=Origin(xyz=(x, 0.0, 0.117)),
            material=dark_steel,
            name=f"post_flange_{i}",
        )
        frame.visual(
            Cylinder(radius=post_radius, length=post_height),
            origin=Origin(xyz=(x, 0.0, base_top_z + post_height * 0.5)),
            material=polished_steel,
            name=f"round_post_{i}",
        )
        frame.visual(
            Cylinder(radius=0.047, length=0.030),
            origin=Origin(xyz=(x, 0.0, base_top_z + post_height + 0.010)),
            material=dark_steel,
            name=f"post_cap_{i}",
        )

    # Upper tie bar bridges the two column tops and keeps the columns rigid.
    frame.visual(
        Box((0.70, 0.12, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, base_top_z + post_height + 0.040)),
        material=dark_steel,
        name="upper_tie_bar",
    )
    frame.visual(
        Box((0.26, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, -0.090, base_top_z + post_height + 0.040)),
        material=painted_base,
        name="front_tie_web",
    )

    # Anchor bolts on the foot help the base read as heavy grounded hardware.
    for i, (x, y) in enumerate(((-0.35, -0.13), (-0.35, 0.13), (0.35, -0.13), (0.35, 0.13))):
        frame.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(x, y, 0.104)),
            material=bolt_black,
            name=f"anchor_bolt_{i}",
        )

    carriage = model.part("carriage")

    # A single carriage plate: rounded slab with two through-holes for the columns.
    hole_radius = 0.047
    carriage_plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.82, 0.25, 0.035, corner_segments=8),
        [
            _circle_profile(hole_radius, center=(-post_x, 0.0), segments=56),
            _circle_profile(hole_radius, center=(post_x, 0.0), segments=56),
        ],
        height=0.110,
        center=True,
    )
    carriage.visual(
        _mesh("carriage_plate", carriage_plate),
        material=safety_yellow,
        name="carriage_plate",
    )

    # Taller bushings make the plate visibly wrap both round posts while leaving
    # running clearance around each fixed column.
    bushing_outer = 0.073
    bushing_inner = 0.043
    bushing_geom = ExtrudeWithHolesGeometry(
        _circle_profile(bushing_outer, segments=64),
        [_circle_profile(bushing_inner, segments=64)],
        height=0.220,
        center=True,
    )
    for i, x in enumerate((-post_x, post_x)):
        carriage.visual(
            _mesh(f"guide_bushing_{i}", bushing_geom),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=bronze,
            name=f"guide_bushing_{i}",
        )

    # Front load plate and welded ribs make the moving member read as one wide
    # carriage rather than two independent sliding collars.
    carriage.visual(
        Box((0.72, 0.035, 0.220)),
        origin=Origin(xyz=(0.0, -0.142, 0.0)),
        material=safety_yellow,
        name="front_load_plate",
    )
    carriage.visual(
        Box((0.060, 0.165, 0.090)),
        origin=Origin(xyz=(-0.14, -0.060, 0.0)),
        material=safety_yellow,
        name="front_rib_0",
    )
    carriage.visual(
        Box((0.060, 0.165, 0.090)),
        origin=Origin(xyz=(0.14, -0.060, 0.0)),
        material=safety_yellow,
        name="front_rib_1",
    )
    carriage.visual(
        Box((0.40, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.164, -0.060)),
        material=dark_steel,
        name="bolt_strip",
    )
    for i, x in enumerate((-0.26, -0.13, 0.13, 0.26)):
        carriage.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(x, -0.1645, 0.055), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bolt_black,
            name=f"face_bolt_{i}",
        )

    model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1600.0, velocity=0.18, lower=0.0, upper=0.72),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("column_lift")

    for i in range(2):
        ctx.allow_overlap(
            carriage,
            frame,
            elem_a=f"guide_bushing_{i}",
            elem_b=f"round_post_{i}",
            reason="Each bronze guide bushing is intentionally modeled as the sliding wrap around its fixed round column.",
        )
        ctx.allow_overlap(
            carriage,
            frame,
            elem_a="carriage_plate",
            elem_b=f"round_post_{i}",
            reason="The carriage plate through-hole is represented as a post-wrapping guide opening around the column.",
        )

    ctx.check(
        "single lifting joint is prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint_type={lift.articulation_type}",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="x",
        min_overlap=0.50,
        elem_a="carriage_plate",
        elem_b="upper_tie_bar",
        name="carriage spans the twin-post frame width",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        min_overlap=0.18,
        elem_a="guide_bushing_0",
        elem_b="round_post_0",
        name="first guide bushing wraps a post in height",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        min_overlap=0.18,
        elem_a="guide_bushing_1",
        elem_b="round_post_1",
        name="second guide bushing wraps a post in height",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        min_overlap=0.10,
        elem_a="carriage_plate",
        elem_b="round_post_0",
        name="carriage plate surrounds first post through its thickness",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        min_overlap=0.10,
        elem_a="carriage_plate",
        elem_b="round_post_1",
        name="carriage plate surrounds second post through its thickness",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.72}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            min_gap=0.035,
            positive_elem="upper_tie_bar",
            negative_elem="front_load_plate",
            name="raised carriage remains below the upper tie bar",
        )

    ctx.check(
        "prismatic joint lifts the carriage upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
