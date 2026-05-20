from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_DISC_RADIUS = 0.220
BASE_DISC_THICKNESS = 0.026

OUTER_BEARING_RADIUS = 0.082
OUTER_BEARING_HEIGHT = 0.008
OUTER_BEARING_CENTER_Z = BASE_DISC_THICKNESS + OUTER_BEARING_HEIGHT / 2.0

CENTER_PEDESTAL_RADIUS = 0.034
CENTER_PEDESTAL_HEIGHT = 0.040
CENTER_PEDESTAL_BOTTOM_Z = BASE_DISC_THICKNESS + OUTER_BEARING_HEIGHT
CENTER_PEDESTAL_CENTER_Z = CENTER_PEDESTAL_BOTTOM_Z + CENTER_PEDESTAL_HEIGHT / 2.0

CENTER_BEARING_CAP_RADIUS = 0.038
CENTER_BEARING_CAP_HEIGHT = 0.006
CENTER_BEARING_CAP_BOTTOM_Z = CENTER_PEDESTAL_BOTTOM_Z + CENTER_PEDESTAL_HEIGHT
CENTER_BEARING_CAP_CENTER_Z = CENTER_BEARING_CAP_BOTTOM_Z + CENTER_BEARING_CAP_HEIGHT / 2.0

OUTER_RING_OUTER_RADIUS = 0.225
OUTER_RING_INNER_RADIUS = 0.100
OUTER_RING_SURFACE_HEIGHT = 0.018
OUTER_RING_SURFACE_BOTTOM_Z = 0.038
OUTER_RING_SURFACE_CENTER_Z = OUTER_RING_SURFACE_BOTTOM_Z + OUTER_RING_SURFACE_HEIGHT / 2.0
OUTER_RING_LIP_HEIGHT = 0.010
OUTER_RING_LIP_CENTER_Z = OUTER_RING_SURFACE_BOTTOM_Z + OUTER_RING_SURFACE_HEIGHT + OUTER_RING_LIP_HEIGHT / 2.0
OUTER_RING_OUTER_LIP_INNER_RADIUS = 0.213
OUTER_RING_INNER_LIP_OUTER_RADIUS = 0.112
OUTER_RING_SLEEVE_OUTER_RADIUS = 0.108
OUTER_RING_SLEEVE_INNER_RADIUS = 0.084
OUTER_RING_SLEEVE_HEIGHT = 0.016
OUTER_RING_SLEEVE_BOTTOM_Z = 0.024
OUTER_RING_SLEEVE_CENTER_Z = OUTER_RING_SLEEVE_BOTTOM_Z + OUTER_RING_SLEEVE_HEIGHT / 2.0

CENTER_PLATTER_RADIUS = 0.082
CENTER_PLATTER_HEIGHT = 0.018
CENTER_PLATTER_BOTTOM_Z = 0.084
CENTER_PLATTER_CENTER_Z = CENTER_PLATTER_BOTTOM_Z + CENTER_PLATTER_HEIGHT / 2.0
CENTER_PLATTER_LIP_INNER_RADIUS = 0.074
CENTER_PLATTER_LIP_HEIGHT = 0.010
CENTER_PLATTER_LIP_CENTER_Z = CENTER_PLATTER_BOTTOM_Z + CENTER_PLATTER_HEIGHT + CENTER_PLATTER_LIP_HEIGHT / 2.0
CENTER_PLATTER_SLEEVE_OUTER_RADIUS = 0.050
CENTER_PLATTER_SLEEVE_INNER_RADIUS = 0.040
CENTER_PLATTER_SLEEVE_HEIGHT = 0.022
CENTER_PLATTER_SLEEVE_BOTTOM_Z = 0.062
CENTER_PLATTER_SLEEVE_CENTER_Z = CENTER_PLATTER_SLEEVE_BOTTOM_Z + CENTER_PLATTER_SLEEVE_HEIGHT / 2.0
CENTER_PLATTER_BEARING_PLATE_RADIUS = 0.050
CENTER_PLATTER_BEARING_PLATE_HEIGHT = 0.004
CENTER_PLATTER_BEARING_PLATE_CENTER_Z = CENTER_BEARING_CAP_BOTTOM_Z + CENTER_BEARING_CAP_HEIGHT + CENTER_PLATTER_BEARING_PLATE_HEIGHT / 2.0


def circular_profile(radius: float, *, segments: int = 72, clockwise: bool = False) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]
    return list(reversed(points)) if clockwise else points


def annulus_mesh(name: str, outer_radius: float, inner_radius: float, height: float, *, segments: int = 72):
    geometry = ExtrudeWithHolesGeometry(
        circular_profile(outer_radius, segments=segments),
        [circular_profile(inner_radius, segments=segments, clockwise=True)],
        height,
        center=True,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_lazy_susan")

    dark_walnut = model.material("dark_walnut", rgba=(0.31, 0.22, 0.15, 1.0))
    maple = model.material("maple", rgba=(0.73, 0.60, 0.43, 1.0))
    oiled_oak = model.material("oiled_oak", rgba=(0.63, 0.49, 0.31, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.67, 0.36, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_DISC_RADIUS, length=BASE_DISC_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_THICKNESS / 2.0)),
        material=dark_walnut,
        name="elem_base_disc",
    )
    base.visual(
        Cylinder(radius=OUTER_BEARING_RADIUS, length=OUTER_BEARING_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, OUTER_BEARING_CENTER_Z)),
        material=brushed_steel,
        name="elem_outer_bearing_collar",
    )
    base.visual(
        Cylinder(radius=CENTER_PEDESTAL_RADIUS, length=CENTER_PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CENTER_PEDESTAL_CENTER_Z)),
        material=dark_walnut,
        name="elem_center_pedestal",
    )
    base.visual(
        Cylinder(radius=CENTER_BEARING_CAP_RADIUS, length=CENTER_BEARING_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CENTER_BEARING_CAP_CENTER_Z)),
        material=brushed_steel,
        name="elem_center_bearing_cap",
    )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        annulus_mesh(
            "outer_ring_surface",
            OUTER_RING_OUTER_RADIUS,
            OUTER_RING_INNER_RADIUS,
            OUTER_RING_SURFACE_HEIGHT,
            segments=96,
        ),
        origin=Origin(xyz=(0.0, 0.0, OUTER_RING_SURFACE_CENTER_Z - OUTER_BEARING_CENTER_Z)),
        material=oiled_oak,
        name="elem_outer_ring_surface",
    )
    outer_ring.visual(
        annulus_mesh(
            "outer_ring_outer_lip",
            OUTER_RING_OUTER_RADIUS,
            OUTER_RING_OUTER_LIP_INNER_RADIUS,
            OUTER_RING_LIP_HEIGHT,
            segments=96,
        ),
        origin=Origin(xyz=(0.0, 0.0, OUTER_RING_LIP_CENTER_Z - OUTER_BEARING_CENTER_Z)),
        material=oiled_oak,
        name="elem_outer_ring_outer_lip",
    )
    outer_ring.visual(
        annulus_mesh(
            "outer_ring_inner_lip",
            OUTER_RING_INNER_LIP_OUTER_RADIUS,
            OUTER_RING_INNER_RADIUS,
            OUTER_RING_LIP_HEIGHT,
            segments=96,
        ),
        origin=Origin(xyz=(0.0, 0.0, OUTER_RING_LIP_CENTER_Z - OUTER_BEARING_CENTER_Z)),
        material=oiled_oak,
        name="elem_outer_ring_inner_lip",
    )
    outer_ring.visual(
        annulus_mesh(
            "outer_ring_bearing_sleeve",
            OUTER_RING_SLEEVE_OUTER_RADIUS,
            OUTER_RING_SLEEVE_INNER_RADIUS,
            OUTER_RING_SLEEVE_HEIGHT,
            segments=80,
        ),
        origin=Origin(xyz=(0.0, 0.0, OUTER_RING_SLEEVE_CENTER_Z - OUTER_BEARING_CENTER_Z)),
        material=brushed_steel,
        name="elem_outer_ring_bearing_sleeve",
    )
    outer_ring.visual(
        Box((0.036, 0.010, 0.004)),
        origin=Origin(
            xyz=(
                0.178,
                0.0,
                OUTER_RING_SURFACE_BOTTOM_Z
                + OUTER_RING_SURFACE_HEIGHT
                + 0.002
                - OUTER_BEARING_CENTER_Z,
            )
        ),
        material=brass,
        name="elem_outer_ring_marker",
    )

    center_platter = model.part("center_platter")
    center_platter.visual(
        Cylinder(radius=CENTER_PLATTER_RADIUS, length=CENTER_PLATTER_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CENTER_PLATTER_CENTER_Z - CENTER_BEARING_CAP_CENTER_Z)),
        material=maple,
        name="elem_center_platter_disc",
    )
    center_platter.visual(
        annulus_mesh(
            "center_platter_lip",
            CENTER_PLATTER_RADIUS,
            CENTER_PLATTER_LIP_INNER_RADIUS,
            CENTER_PLATTER_LIP_HEIGHT,
            segments=80,
        ),
        origin=Origin(xyz=(0.0, 0.0, CENTER_PLATTER_LIP_CENTER_Z - CENTER_BEARING_CAP_CENTER_Z)),
        material=maple,
        name="elem_center_platter_lip",
    )
    center_platter.visual(
        annulus_mesh(
            "center_platter_bearing_sleeve",
            CENTER_PLATTER_SLEEVE_OUTER_RADIUS,
            CENTER_PLATTER_SLEEVE_INNER_RADIUS,
            CENTER_PLATTER_SLEEVE_HEIGHT,
            segments=72,
        ),
        origin=Origin(xyz=(0.0, 0.0, CENTER_PLATTER_SLEEVE_CENTER_Z - CENTER_BEARING_CAP_CENTER_Z)),
        material=brushed_steel,
        name="elem_center_platter_bearing_sleeve",
    )
    center_platter.visual(
        Cylinder(radius=CENTER_PLATTER_BEARING_PLATE_RADIUS, length=CENTER_PLATTER_BEARING_PLATE_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, CENTER_PLATTER_BEARING_PLATE_CENTER_Z - CENTER_BEARING_CAP_CENTER_Z)
        ),
        material=brushed_steel,
        name="elem_center_platter_bearing_plate",
    )
    center_platter.visual(
        Box((0.012, 0.028, 0.004)),
        origin=Origin(
            xyz=(
                0.0,
                0.056,
                CENTER_PLATTER_BOTTOM_Z + CENTER_PLATTER_HEIGHT + 0.002 - CENTER_BEARING_CAP_CENTER_Z,
            )
        ),
        material=brass,
        name="elem_center_platter_marker",
    )

    model.articulation(
        "base_to_outer_ring",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, OUTER_BEARING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "base_to_center_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=center_platter,
        origin=Origin(xyz=(0.0, 0.0, CENTER_BEARING_CAP_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
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

    base = object_model.get_part("base")
    outer_ring = object_model.get_part("outer_ring")
    center_platter = object_model.get_part("center_platter")
    outer_joint = object_model.get_articulation("base_to_outer_ring")
    center_joint = object_model.get_articulation("base_to_center_platter")

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    ctx.check(
        "outer ring uses a vertical axis",
        tuple(outer_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={outer_joint.axis}",
    )
    ctx.check(
        "center platter uses a vertical axis",
        tuple(center_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={center_joint.axis}",
    )
    ctx.expect_origin_distance(
        outer_ring,
        base,
        axes="xy",
        max_dist=0.001,
        name="outer ring stays centered on the base",
    )
    ctx.expect_origin_distance(
        center_platter,
        base,
        axes="xy",
        max_dist=0.001,
        name="center platter stays centered on the base",
    )
    ctx.expect_gap(
        outer_ring,
        base,
        axis="z",
        min_gap=0.010,
        max_gap=0.015,
        positive_elem="elem_outer_ring_surface",
        negative_elem="elem_base_disc",
        name="outer ring rides just above the lower support disc",
    )
    ctx.expect_gap(
        center_platter,
        outer_ring,
        axis="z",
        min_gap=0.015,
        max_gap=0.025,
        positive_elem="elem_center_platter_disc",
        negative_elem="elem_outer_ring_outer_lip",
        name="center platter is visibly stacked above the outer ring",
    )
    ctx.expect_overlap(
        outer_ring,
        base,
        axes="xy",
        min_overlap=0.35,
        elem_a="elem_outer_ring_surface",
        elem_b="elem_base_disc",
        name="outer ring remains broadly supported over the lower disc footprint",
    )

    outer_marker_rest = elem_center("outer_ring", "elem_outer_ring_marker")
    center_marker_rest = elem_center("center_platter", "elem_center_platter_marker")

    with ctx.pose({outer_joint: math.pi / 2.0}):
        outer_marker_turned = elem_center("outer_ring", "elem_outer_ring_marker")
        center_marker_during_outer_turn = elem_center("center_platter", "elem_center_platter_marker")

    with ctx.pose({center_joint: math.pi / 2.0}):
        outer_marker_during_center_turn = elem_center("outer_ring", "elem_outer_ring_marker")
        center_marker_turned = elem_center("center_platter", "elem_center_platter_marker")

    ctx.check(
        "outer ring rotates independently",
        outer_marker_rest is not None
        and outer_marker_turned is not None
        and outer_marker_rest[0] > 0.15
        and abs(outer_marker_turned[0]) < 0.02
        and outer_marker_turned[1] > 0.15,
        details=f"rest={outer_marker_rest}, turned={outer_marker_turned}",
    )
    ctx.check(
        "center platter remains still while only the outer ring turns",
        center_marker_rest is not None
        and center_marker_during_outer_turn is not None
        and abs(center_marker_rest[0] - center_marker_during_outer_turn[0]) < 0.001
        and abs(center_marker_rest[1] - center_marker_during_outer_turn[1]) < 0.001,
        details=f"rest={center_marker_rest}, during_outer_turn={center_marker_during_outer_turn}",
    )
    ctx.check(
        "center platter rotates independently",
        center_marker_rest is not None
        and center_marker_turned is not None
        and center_marker_rest[1] > 0.04
        and center_marker_turned[0] < -0.04
        and abs(center_marker_turned[1]) < 0.02,
        details=f"rest={center_marker_rest}, turned={center_marker_turned}",
    )
    ctx.check(
        "outer ring remains still while only the center platter turns",
        outer_marker_rest is not None
        and outer_marker_during_center_turn is not None
        and abs(outer_marker_rest[0] - outer_marker_during_center_turn[0]) < 0.001
        and abs(outer_marker_rest[1] - outer_marker_during_center_turn[1]) < 0.001,
        details=f"rest={outer_marker_rest}, during_center_turn={outer_marker_during_center_turn}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
