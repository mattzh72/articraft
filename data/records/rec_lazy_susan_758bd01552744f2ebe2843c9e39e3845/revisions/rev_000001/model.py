from __future__ import annotations

from math import cos, pi, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _signed_area(profile: list[tuple[float, float]]) -> float:
    area = 0.0
    for index, (x0, y0) in enumerate(profile):
        x1, y1 = profile[(index + 1) % len(profile)]
        area += x0 * y1 - x1 * y0
    return 0.5 * area


def _kidney_profile(
    *,
    radius: float = 0.365,
    cutout_start_deg: float = 8.0,
    cutout_end_deg: float = 82.0,
    outer_segments: int = 72,
    cutout_segments: int = 22,
) -> list[tuple[float, float]]:
    """Round lazy-susan shelf outline with a concave door-side bite.

    The missing concave bite sits in the +X/+Y quadrant at the rest pose, so the
    shelves read as kidney shaped rather than as plain circular disks.
    """

    start = radians(cutout_end_deg)
    stop = radians(cutout_start_deg) + 2.0 * pi
    profile: list[tuple[float, float]] = []
    for i in range(outer_segments + 1):
        t = i / outer_segments
        angle = start + (stop - start) * t
        profile.append((radius * cos(angle), radius * sin(angle)))

    p0 = profile[-1]
    p1 = profile[0]
    control = (radius * 0.27, radius * 0.27)
    for i in range(1, cutout_segments):
        t = i / cutout_segments
        x = (1.0 - t) ** 2 * p0[0] + 2.0 * (1.0 - t) * t * control[0] + t**2 * p1[0]
        y = (1.0 - t) ** 2 * p0[1] + 2.0 * (1.0 - t) * t * control[1] + t**2 * p1[1]
        profile.append((x, y))

    if _signed_area(profile) < 0.0:
        profile.reverse()
    return profile


def _scaled_profile(profile: list[tuple[float, float]], factor: float) -> list[tuple[float, float]]:
    scaled = [(x * factor, y * factor) for x, y in profile]
    if _signed_area(scaled) < 0.0:
        scaled.reverse()
    return scaled


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_cabinet_lazy_susan")

    cabinet_wood = model.material("cabinet_wood", rgba=(0.55, 0.36, 0.18, 1.0))
    wood_edge = model.material("wood_edge", rgba=(0.39, 0.23, 0.11, 1.0))
    shelf_plastic = model.material("shelf_plastic", rgba=(0.92, 0.89, 0.80, 1.0))
    shelf_edge = model.material("shelf_edge", rgba=(0.82, 0.78, 0.66, 1.0))
    steel = model.material("brushed_steel", rgba=(0.63, 0.64, 0.62, 1.0))
    dark = model.material("dark_bearing", rgba=(0.08, 0.08, 0.075, 1.0))

    shelf_outline = _kidney_profile()
    inner_lip_outline = _scaled_profile(shelf_outline, 0.920)
    shelf_plate = _mesh(
        ExtrudeGeometry(shelf_outline, 0.030, center=True),
        "kidney_shelf_plate",
    )
    shelf_lip = _mesh(
        ExtrudeWithHolesGeometry(
            shelf_outline,
            [inner_lip_outline],
            0.026,
            center=True,
        ),
        "kidney_shelf_raised_lip",
    )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.90, 0.90, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cabinet_wood,
        name="floor_panel",
    )
    cabinet.visual(
        Box((0.90, 0.90, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.7825)),
        material=cabinet_wood,
        name="top_panel",
    )
    cabinet.visual(
        Box((0.035, 0.90, 0.80)),
        origin=Origin(xyz=(-0.4675, 0.0, 0.40)),
        material=cabinet_wood,
        name="side_wall_x",
    )
    cabinet.visual(
        Box((0.90, 0.035, 0.80)),
        origin=Origin(xyz=(0.0, -0.4675, 0.40)),
        material=cabinet_wood,
        name="side_wall_y",
    )
    cabinet.visual(
        Box((0.18, 0.035, 0.80)),
        origin=Origin(xyz=(0.43, 0.43, 0.40), rpy=(0.0, 0.0, pi / 4.0)),
        material=wood_edge,
        name="diagonal_face_frame",
    )
    cabinet.visual(
        Cylinder(radius=0.060, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=dark,
        name="lower_bearing_pad",
    )
    cabinet.visual(
        _mesh(TorusGeometry(0.045, 0.008, radial_segments=48, tubular_segments=12), "lower_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=steel,
        name="lower_bearing_ring",
    )
    cabinet.visual(
        _mesh(TorusGeometry(0.045, 0.008, radial_segments=48, tubular_segments=12), "upper_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.757)),
        material=steel,
        name="upper_bearing_ring",
    )

    carousel = model.part("carousel")
    carousel.visual(
        Cylinder(radius=0.025, length=0.712),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=steel,
        name="center_shaft",
    )

    carousel.visual(
        shelf_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=shelf_plastic,
        name="lower_shelf",
    )
    carousel.visual(
        shelf_lip,
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        material=shelf_edge,
        name="lower_raised_lip",
    )
    carousel.visual(
        Cylinder(radius=0.066, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.216)),
        material=steel,
        name="lower_shaft_collar",
    )
    carousel.visual(
        Box((0.185, 0.026, 0.026)),
        origin=Origin(xyz=(-0.110, 0.0, 0.245)),
        material=steel,
        name="lower_clip_bar_x",
    )
    carousel.visual(
        Box((0.026, 0.185, 0.026)),
        origin=Origin(xyz=(0.0, -0.110, 0.245)),
        material=steel,
        name="lower_clip_bar_y",
    )

    carousel.visual(
        shelf_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        material=shelf_plastic,
        name="upper_shelf",
    )
    carousel.visual(
        shelf_lip,
        origin=Origin(xyz=(0.0, 0.0, 0.578)),
        material=shelf_edge,
        name="upper_raised_lip",
    )
    carousel.visual(
        Cylinder(radius=0.066, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.556)),
        material=steel,
        name="upper_shaft_collar",
    )
    carousel.visual(
        Box((0.185, 0.026, 0.026)),
        origin=Origin(xyz=(-0.110, 0.0, 0.585)),
        material=steel,
        name="upper_clip_bar_x",
    )
    carousel.visual(
        Box((0.026, 0.185, 0.026)),
        origin=Origin(xyz=(0.0, -0.110, 0.585)),
        material=steel,
        name="upper_clip_bar_y",
    )

    model.articulation(
        "cabinet_to_carousel",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=carousel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    carousel = object_model.get_part("carousel")
    spin = object_model.get_articulation("cabinet_to_carousel")

    ctx.check(
        "shelf assembly has continuous rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_gap(
        carousel,
        cabinet,
        axis="z",
        positive_elem="lower_shelf",
        negative_elem="floor_panel",
        min_gap=0.14,
        name="bottom shelf clears cabinet floor",
    )
    ctx.expect_gap(
        cabinet,
        carousel,
        axis="z",
        positive_elem="top_panel",
        negative_elem="upper_raised_lip",
        min_gap=0.16,
        name="top shelf clears cabinet top",
    )
    ctx.expect_within(
        carousel,
        cabinet,
        axes="xy",
        inner_elem="center_shaft",
        outer_elem="floor_panel",
        margin=0.0,
        name="center shaft stays inside cabinet footprint",
    )

    for shelf_name in ("lower_shelf", "upper_shelf"):
        shelf_aabb = ctx.part_element_world_aabb(carousel, elem=shelf_name)
        shaft_aabb = ctx.part_element_world_aabb(carousel, elem="center_shaft")
        ok = False
        details = f"{shelf_name}={shelf_aabb}, shaft={shaft_aabb}"
        if shelf_aabb is not None and shaft_aabb is not None:
            ok = (
                shaft_aabb[0][2] <= shelf_aabb[0][2] + 0.001
                and shaft_aabb[1][2] >= shelf_aabb[1][2] - 0.001
                and shaft_aabb[0][0] < shelf_aabb[1][0]
                and shaft_aabb[1][0] > shelf_aabb[0][0]
                and shaft_aabb[0][1] < shelf_aabb[1][1]
                and shaft_aabb[1][1] > shelf_aabb[0][1]
            )
        ctx.check(f"{shelf_name} is clipped through center shaft", ok, details=details)

    with ctx.pose({spin: pi / 2.0}):
        ctx.expect_gap(
            carousel,
            cabinet,
            axis="x",
            positive_elem="lower_shelf",
            negative_elem="side_wall_x",
            min_gap=0.04,
            name="rotated lower shelf clears x wall",
        )
        ctx.expect_gap(
            carousel,
            cabinet,
            axis="y",
            positive_elem="lower_shelf",
            negative_elem="side_wall_y",
            min_gap=0.04,
            name="rotated lower shelf clears y wall",
        )
        ctx.expect_gap(
            carousel,
            cabinet,
            axis="x",
            positive_elem="upper_shelf",
            negative_elem="side_wall_x",
            min_gap=0.04,
            name="rotated upper shelf clears x wall",
        )
        ctx.expect_gap(
            carousel,
            cabinet,
            axis="y",
            positive_elem="upper_shelf",
            negative_elem="side_wall_y",
            min_gap=0.04,
            name="rotated upper shelf clears y wall",
        )

    return ctx.report()


object_model = build_object_model()
