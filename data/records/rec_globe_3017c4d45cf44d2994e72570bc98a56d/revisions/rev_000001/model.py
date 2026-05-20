from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    Sphere,
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _oval_patch(width: float, height: float, thickness: float = 0.0016):
    """A thin oval decal mesh, centered with its normal along local +Z."""
    geom = CylinderGeometry(0.5, thickness, radial_segments=32, closed=True)
    geom.scale(width, height, 1.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_globe_on_meridian")

    brass = model.material("warm_brass", rgba=(0.74, 0.55, 0.25, 1.0))
    dark_brass = model.material("dark_brass", rgba=(0.45, 0.34, 0.18, 1.0))
    black = model.material("black_plastic", rgba=(0.02, 0.02, 0.018, 1.0))
    blue = model.material("ocean_blue", rgba=(0.08, 0.32, 0.72, 1.0))
    green = model.material("map_green", rgba=(0.12, 0.48, 0.20, 1.0))
    tan = model.material("map_tan", rgba=(0.74, 0.61, 0.36, 1.0))
    cream = model.material("cream_grid", rgba=(0.92, 0.86, 0.64, 1.0))

    globe_radius = 0.180
    meridian_radius = 0.222
    meridian_tube = 0.008

    pedestal = model.part("pedestal")

    # Weighted classroom-stand base and upright yoke.  The yoke cheeks stop just
    # outside the meridian trunnions, so the rotating frame has a visible bearing
    # seat without being fused to the stand.
    pedestal.visual(
        Cylinder(radius=0.205, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.600)),
        material=dark_brass,
        name="weighted_base",
    )
    pedestal.visual(
        Cylinder(radius=0.035, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        material=brass,
        name="upright_post",
    )
    pedestal.visual(
        Cylinder(radius=0.026, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, -0.275), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lower_yoke_bar",
    )
    for x, suffix in ((-0.286, "0"), (0.286, "1")):
        pedestal.visual(
            Box((0.038, 0.045, 0.335)),
            origin=Origin(xyz=(x, 0.0, -0.090)),
            material=brass,
            name=f"yoke_arm_{suffix}",
        )
        pedestal.visual(
            Cylinder(radius=0.055, length=0.042),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"bearing_boss_{suffix}",
        )

    meridian = model.part("meridian")
    meridian.visual(
        mesh_from_geometry(
            TorusGeometry(meridian_radius, meridian_tube, radial_segments=18, tubular_segments=96),
            "meridian_band",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="meridian_band",
    )
    # Side trunnions extend from the meridian ring to the yoke bearings.
    for x, suffix, sign in ((-0.2465, "0", -1.0), (0.2465, "1", 1.0)):
        meridian.visual(
            Cylinder(radius=0.018, length=0.037),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_brass,
            name=f"trunnion_{suffix}",
        )
        meridian.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(sign * 0.228, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"trunnion_flange_{suffix}",
        )
    # Polar sockets for the globe's axle pins.
    for z, suffix in ((0.212, "north"), (-0.212, "south")):
        meridian.visual(
            Cylinder(radius=0.024, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_brass,
            name=f"{suffix}_socket",
        )

    # Small degree marks on the front half of the meridian band.
    for i, angle_deg in enumerate((-60, -30, 0, 30, 60)):
        angle = math.radians(angle_deg)
        x = (meridian_radius + 0.010) * math.cos(angle)
        z = (meridian_radius + 0.010) * math.sin(angle)
        meridian.visual(
            Box((0.028, 0.006, 0.004)),
            origin=Origin(xyz=(x, -0.006, z), rpy=(0.0, -angle, 0.0)),
            material=cream,
            name=f"degree_mark_{i}",
        )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=globe_radius),
        origin=Origin(),
        material=blue,
        name="ocean_sphere",
    )
    # Polar axle pins are part of the spinning globe and seat against the fixed
    # meridian sockets without penetrating them.
    globe.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=dark_brass,
        name="north_pin",
    )
    globe.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        material=dark_brass,
        name="south_pin",
    )

    # Raised classroom-globe map treatment: cream great circles and latitude
    # rings sit slightly proud of the blue ocean, with simplified continent
    # decals on the surface.
    grid_torus = TorusGeometry(0.181, 0.0011, radial_segments=8, tubular_segments=96)
    globe.visual(
        mesh_from_geometry(grid_torus, "equator_grid"),
        origin=Origin(),
        material=cream,
        name="equator_grid",
    )
    globe.visual(
        mesh_from_geometry(
            TorusGeometry(0.181, 0.0010, radial_segments=8, tubular_segments=96),
            "prime_meridian_grid",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="prime_meridian_grid",
    )
    globe.visual(
        mesh_from_geometry(
            TorusGeometry(0.181, 0.0010, radial_segments=8, tubular_segments=96),
            "side_meridian_grid",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream,
        name="side_meridian_grid",
    )
    for i, z in enumerate((-0.105, -0.055, 0.055, 0.105)):
        radius = math.sqrt(0.181 * 0.181 - z * z)
        globe.visual(
            mesh_from_geometry(
                TorusGeometry(radius, 0.0008, radial_segments=8, tubular_segments=80),
                f"latitude_grid_{i}",
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=cream,
            name=f"latitude_grid_{i}",
        )

    globe.visual(
        mesh_from_geometry(_oval_patch(0.070, 0.040), "continent_0"),
        origin=Origin(xyz=(-0.055, 0.166, 0.045), rpy=(-math.pi / 2.0, 0.0, -0.30)),
        material=green,
        name="continent_0",
    )
    globe.visual(
        mesh_from_geometry(_oval_patch(0.048, 0.076), "continent_1"),
        origin=Origin(xyz=(0.045, 0.172, -0.030), rpy=(-math.pi / 2.0, 0.0, 0.45)),
        material=tan,
        name="continent_1",
    )
    globe.visual(
        mesh_from_geometry(_oval_patch(0.060, 0.032), "continent_2"),
        origin=Origin(xyz=(0.082, -0.148, 0.058), rpy=(math.pi / 2.0, 0.0, 0.20)),
        material=green,
        name="continent_2",
    )
    globe.visual(
        mesh_from_geometry(_oval_patch(0.040, 0.032), "continent_3"),
        origin=Origin(xyz=(-0.090, -0.149, -0.050), rpy=(math.pi / 2.0, 0.0, -0.55)),
        material=tan,
        name="continent_3",
    )

    knob_mesh_0 = mesh_from_geometry(
        KnobGeometry(
            0.058,
            0.036,
            body_style="lobed",
            top_diameter=0.044,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=16, depth=0.0010),
        ),
        "side_knob_0",
    )
    knob_mesh_1 = mesh_from_geometry(
        KnobGeometry(
            0.058,
            0.036,
            body_style="lobed",
            top_diameter=0.044,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=16, depth=0.0010),
        ),
        "side_knob_1",
    )
    knob_0 = model.part("side_knob_0")
    knob_0.visual(
        knob_mesh_0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="knob_cap",
    )
    knob_1 = model.part("side_knob_1")
    knob_1.visual(
        knob_mesh_1,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="knob_cap",
    )

    model.articulation(
        "meridian_tilt",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0),
    )
    model.articulation(
        "knob_0_spin",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=knob_0,
        origin=Origin(xyz=(-0.325, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )
    model.articulation(
        "knob_1_spin",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=knob_1,
        origin=Origin(xyz=(0.325, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    knob_0 = object_model.get_part("side_knob_0")
    knob_1 = object_model.get_part("side_knob_1")
    meridian_tilt = object_model.get_articulation("meridian_tilt")

    ctx.expect_within(
        globe,
        meridian,
        axes="xz",
        inner_elem="ocean_sphere",
        outer_elem="meridian_band",
        margin=0.050,
        name="globe sphere sits inside meridian ring",
    )
    ctx.expect_gap(
        meridian,
        globe,
        axis="z",
        positive_elem="north_socket",
        negative_elem="north_pin",
        min_gap=0.0,
        max_gap=0.003,
        name="north polar pin seats in socket",
    )
    ctx.expect_gap(
        globe,
        meridian,
        axis="z",
        positive_elem="south_pin",
        negative_elem="south_socket",
        min_gap=0.0,
        max_gap=0.003,
        name="south polar pin seats in socket",
    )
    ctx.expect_gap(
        knob_1,
        pedestal,
        axis="x",
        positive_elem="knob_cap",
        negative_elem="bearing_boss_1",
        min_gap=0.0,
        max_gap=0.003,
        name="positive side knob bears on yoke",
    )
    ctx.expect_gap(
        pedestal,
        knob_0,
        axis="x",
        positive_elem="bearing_boss_0",
        negative_elem="knob_cap",
        min_gap=0.0,
        max_gap=0.003,
        name="negative side knob bears on yoke",
    )

    rest_aabb = ctx.part_world_aabb(meridian)
    with ctx.pose({meridian_tilt: 0.55}):
        tilted_aabb = ctx.part_world_aabb(meridian)
        ctx.expect_contact(
            meridian,
            pedestal,
            elem_a="trunnion_1",
            elem_b="bearing_boss_1",
            name="tilted meridian remains on trunnion axis",
        )
    ctx.check(
        "meridian tilt changes vertical silhouette",
        rest_aabb is not None
        and tilted_aabb is not None
        and abs((tilted_aabb[1][1] - tilted_aabb[0][1]) - (rest_aabb[1][1] - rest_aabb[0][1])) > 0.04,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
