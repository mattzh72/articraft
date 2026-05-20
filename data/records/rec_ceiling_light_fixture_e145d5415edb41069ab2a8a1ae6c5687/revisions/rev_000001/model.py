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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _helix_geometry(*, phase: float, handedness: float = 1.0) -> MeshGeometry:
    """A slightly raised braided strand on the pendant cord."""

    points = []
    cord_radius = 0.0053
    z_top = -0.050
    z_bottom = -0.715
    turns = 5.5
    steps = 58
    for i in range(steps + 1):
        t = i / steps
        angle = phase + handedness * 2.0 * math.pi * turns * t
        points.append(
            (
                cord_radius * math.cos(angle),
                cord_radius * math.sin(angle),
                z_top + (z_bottom - z_top) * t,
            )
        )
    return tube_from_spline_points(
        points,
        radius=0.00055,
        samples_per_segment=3,
        radial_segments=8,
        cap_ends=True,
    )


def _canopy_geometry() -> MeshGeometry:
    """A shallow circular ceiling canopy with a soft rolled lower edge."""

    return LatheGeometry(
        [
            (0.000, 0.082),
            (0.085, 0.082),
            (0.106, 0.076),
            (0.111, 0.066),
            (0.105, 0.052),
            (0.092, 0.041),
            (0.022, 0.041),
            (0.000, 0.046),
        ],
        segments=72,
        closed=True,
    )


def _shade_shell_geometry() -> MeshGeometry:
    """Thin-walled hemispherical shade, open at the lower rim."""

    outer = []
    inner = []
    radius = 0.180
    center_z = -0.910
    wall = 0.004
    # Start just below the small top fitter hole and end at the open equator.
    for i in range(15):
        theta = 0.19 + (math.pi / 2.0 - 0.19) * i / 14
        outer_r = radius * math.sin(theta)
        outer_z = center_z + radius * math.cos(theta)
        inner_r = outer_r - wall
        inner_z = center_z + (radius - wall) * math.cos(theta)
        outer.append((outer_r, outer_z))
        inner.append((inner_r, inner_z))

    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pendant_ceiling_light")

    satin_white = Material("satin_white_powdercoat", rgba=(0.88, 0.86, 0.80, 1.0))
    dark_metal = Material("dark_oiled_metal", rgba=(0.03, 0.028, 0.024, 1.0))
    black_fabric = Material("black_woven_fabric", rgba=(0.006, 0.006, 0.007, 1.0))
    grey_thread = Material("charcoal_fabric_thread", rgba=(0.10, 0.10, 0.095, 1.0))
    brass = Material("warm_brushed_brass", rgba=(0.82, 0.58, 0.30, 1.0))
    warm_glass = Material("warm_frosted_glass", rgba=(1.0, 0.88, 0.55, 0.62))

    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_geometry(_canopy_geometry(), "canopy_saucer"),
        material=satin_white,
        name="canopy_saucer",
    )
    # Two downward ears form the fixed half of the swivel yoke.
    canopy.visual(
        Box((0.052, 0.007, 0.056)),
        origin=Origin(xyz=(0.0, 0.024, 0.016)),
        material=satin_white,
        name="yoke_cheek_pos",
    )
    canopy.visual(
        Box((0.052, 0.007, 0.056)),
        origin=Origin(xyz=(0.0, -0.024, 0.016)),
        material=satin_white,
        name="yoke_cheek_neg",
    )
    canopy.visual(
        Cylinder(radius=0.0055, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="yoke_pin",
    )

    pendant = model.part("pendant")
    pendant.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_barrel",
    )
    pendant.visual(
        Cylinder(radius=0.007, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=dark_metal,
        name="strain_relief",
    )
    pendant.visual(
        Cylinder(radius=0.0046, length=0.675),
        origin=Origin(xyz=(0.0, 0.0, -0.3825)),
        material=black_fabric,
        name="fabric_cord",
    )
    for idx, (phase, handedness) in enumerate(
        (
            (0.0, 1.0),
            (math.pi, 1.0),
            (math.pi / 2.0, -1.0),
            (3.0 * math.pi / 2.0, -1.0),
        )
    ):
        pendant.visual(
            mesh_from_geometry(_helix_geometry(phase=phase, handedness=handedness), f"cord_braid_{idx}"),
            material=grey_thread if idx % 2 == 0 else black_fabric,
            name=f"cord_braid_{idx}",
        )
    pendant.visual(
        Cylinder(radius=0.026, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.738)),
        material=brass,
        name="socket_cap",
    )
    pendant.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.735)),
        material=brass,
        name="shade_fitter_ring",
    )
    pendant.visual(
        mesh_from_geometry(_shade_shell_geometry(), "dome_shade_shell"),
        material=satin_white,
        name="dome_shade_shell",
    )
    pendant.visual(
        Cylinder(radius=0.018, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.778)),
        material=brass,
        name="bulb_base",
    )
    pendant.visual(
        Sphere(radius=0.043),
        origin=Origin(xyz=(0.0, 0.0, -0.833)),
        material=warm_glass,
        name="globe_bulb",
    )

    model.articulation(
        "canopy_to_pendant",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=pendant,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=-0.45, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    pendant = object_model.get_part("pendant")
    swivel = object_model.get_articulation("canopy_to_pendant")

    ctx.allow_overlap(
        canopy,
        pendant,
        elem_a="yoke_pin",
        elem_b="pivot_barrel",
        reason="The dark swivel pin is intentionally captured through the pendant pivot barrel.",
    )

    with ctx.pose({swivel: 0.0}):
        ctx.expect_overlap(
            canopy,
            pendant,
            axes="xz",
            elem_a="yoke_pin",
            elem_b="pivot_barrel",
            min_overlap=0.010,
            name="swivel pin passes through barrel",
        )
        ctx.expect_within(
            pendant,
            canopy,
            axes="y",
            inner_elem="pivot_barrel",
            outer_elem="yoke_cheek_pos",
            margin=0.045,
            name="pivot barrel is retained between yoke cheeks",
        )
        ctx.expect_gap(
            pendant,
            pendant,
            axis="z",
            positive_elem="pivot_barrel",
            negative_elem="fabric_cord",
            max_gap=0.040,
            max_penetration=0.010,
            name="cord is seated below the swivel barrel",
        )

    rest_aabb = ctx.part_element_world_aabb(pendant, elem="dome_shade_shell")
    with ctx.pose({swivel: 0.45}):
        swung_aabb = ctx.part_element_world_aabb(pendant, elem="dome_shade_shell")

    if rest_aabb is not None and swung_aabb is not None:
        rest_center_x = 0.5 * (rest_aabb[0][0] + rest_aabb[1][0])
        swung_center_x = 0.5 * (swung_aabb[0][0] + swung_aabb[1][0])
        ctx.check(
            "revolute swivel swings pendant from canopy",
            swung_center_x < rest_center_x - 0.20,
            details=f"rest shade x={rest_center_x:.3f}, swung shade x={swung_center_x:.3f}",
        )
    else:
        ctx.fail("shade aabb available for swivel check", "Could not measure dome_shade_shell AABB.")

    return ctx.report()


object_model = build_object_model()
