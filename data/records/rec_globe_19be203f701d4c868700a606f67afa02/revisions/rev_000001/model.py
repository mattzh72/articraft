from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TorusGeometry,
    mesh_from_geometry,
    TestContext,
    TestReport,
)


TRUNNION_Z = 0.72
GLOBE_RADIUS = 0.215
MERIDIAN_RADIUS = 0.248


def _continent_mesh(points: list[tuple[float, float]], name: str):
    """Raised flat decal for simple continent silhouettes on the globe front."""
    return mesh_from_geometry(
        ExtrudeGeometry(points, 0.008, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="illuminated_classroom_globe")

    brass = model.material("satin_brass", rgba=(0.72, 0.56, 0.30, 1.0))
    dark_brass = model.material("aged_brass", rgba=(0.38, 0.29, 0.15, 1.0))
    black = model.material("black_bakelite", rgba=(0.025, 0.022, 0.018, 1.0))
    chrome = model.material("polished_pivot", rgba=(0.78, 0.80, 0.78, 1.0))
    ocean = model.material("lit_blue_ocean", rgba=(0.15, 0.46, 0.82, 0.64))
    land = model.material("raised_land", rgba=(0.20, 0.58, 0.25, 1.0))
    graticule = model.material("cream_graticule", rgba=(0.92, 0.86, 0.62, 1.0))
    glow = model.material("warm_internal_glow", rgba=(1.0, 0.78, 0.25, 0.46))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.18, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_brass,
        name="round_base",
    )
    pedestal.visual(
        Cylinder(radius=0.043, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.248)),
        material=brass,
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=0.075, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
        material=dark_brass,
        name="top_collar",
    )
    pedestal.visual(
        Box((0.66, 0.085, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_Z - 0.300)),
        material=brass,
        name="yoke_frame",
    )
    pedestal.visual(
        Box((0.048, 0.085, 0.48)),
        origin=Origin(xyz=(-0.302, 0.0, TRUNNION_Z - 0.055)),
        material=brass,
        name="yoke_cheek_0",
    )
    pedestal.visual(
        Box((0.048, 0.085, 0.48)),
        origin=Origin(xyz=(0.302, 0.0, TRUNNION_Z - 0.055)),
        material=brass,
        name="yoke_cheek_1",
    )
    pedestal.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(-0.331, 0.0, TRUNNION_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="outer_boss_0",
    )
    pedestal.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.331, 0.0, TRUNNION_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="outer_boss_1",
    )

    meridian = model.part("meridian")
    meridian.visual(
        mesh_from_geometry(
            TorusGeometry(MERIDIAN_RADIUS, 0.0085, radial_segments=28, tubular_segments=96),
            "meridian_ring",
        ),
        # TorusGeometry is built in XY around local Z; rotate it into the XZ meridian plane.
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="meridian_ring",
    )
    meridian.visual(
        Cylinder(radius=0.021, length=0.034),
        origin=Origin(xyz=(MERIDIAN_RADIUS + 0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="trunnion_boss_0",
    )
    meridian.visual(
        Cylinder(radius=0.021, length=0.034),
        origin=Origin(xyz=(-(MERIDIAN_RADIUS + 0.001), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="trunnion_boss_1",
    )
    meridian.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, MERIDIAN_RADIUS), rpy=(0.0, 0.0, 0.0)),
        material=chrome,
        name="north_socket",
    )
    meridian.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -MERIDIAN_RADIUS), rpy=(0.0, 0.0, 0.0)),
        material=chrome,
        name="south_socket",
    )
    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        origin=Origin(),
        material=ocean,
        name="translucent_sphere",
    )
    globe.visual(
        Sphere(radius=0.082),
        origin=Origin(),
        material=glow,
        name="lamp_glow",
    )
    globe.visual(
        Cylinder(radius=0.010, length=0.468),
        origin=Origin(),
        material=chrome,
        name="polar_axle",
    )
    globe.visual(
        mesh_from_geometry(TorusGeometry(GLOBE_RADIUS + 0.0015, 0.0012, radial_segments=16, tubular_segments=96), "equator_line"),
        origin=Origin(),
        material=graticule,
        name="equator_line",
    )
    globe.visual(
        mesh_from_geometry(TorusGeometry(GLOBE_RADIUS + 0.0015, 0.0010, radial_segments=16, tubular_segments=96), "longitude_line_0"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graticule,
        name="longitude_line_0",
    )
    globe.visual(
        mesh_from_geometry(TorusGeometry(GLOBE_RADIUS + 0.0015, 0.0010, radial_segments=16, tubular_segments=96), "longitude_line_1"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graticule,
        name="longitude_line_1",
    )
    for idx, z in enumerate((-0.105, 0.105)):
        globe.visual(
            mesh_from_geometry(
                TorusGeometry(math.sqrt(GLOBE_RADIUS**2 - z**2), 0.0010, radial_segments=16, tubular_segments=80),
                f"latitude_line_{idx}",
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=graticule,
            name=f"latitude_line_{idx}",
        )

    continent_profiles = [
        [(-0.072, -0.015), (-0.052, 0.026), (-0.010, 0.042), (0.022, 0.020), (0.018, -0.020), (-0.020, -0.038)],
        [(0.045, 0.058), (0.080, 0.042), (0.092, 0.010), (0.056, -0.012), (0.028, 0.018)],
        [(-0.020, -0.085), (0.020, -0.070), (0.032, -0.112), (0.006, -0.150), (-0.026, -0.122)],
        [(-0.125, 0.030), (-0.092, 0.070), (-0.052, 0.052), (-0.075, 0.012)],
    ]
    continent_y = (-GLOBE_RADIUS - 0.001, -0.203, -0.190, -0.197)
    for idx, profile in enumerate(continent_profiles):
        globe.visual(
            _continent_mesh(profile, f"continent_patch_{idx}"),
            origin=Origin(xyz=(0.0, continent_y[idx], 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=land,
            name=f"continent_patch_{idx}",
        )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.078,
            0.038,
            body_style="lobed",
            top_diameter=0.068,
            base_diameter=0.054,
            crown_radius=0.003,
            grip=KnobGrip(style="ribbed", count=12, depth=0.0012),
            bore=KnobBore(style="round", diameter=0.010),
        ),
        "side_tilt_knob",
    )
    knob_0 = model.part("side_knob_0")
    knob_0.visual(
        knob_mesh,
        origin=Origin(xyz=(-0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lobed_grip",
    )
    knob_0.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="knob_shaft",
    )
    knob_0.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="seating_washer",
    )
    knob_1 = model.part("side_knob_1")
    knob_1.visual(
        knob_mesh,
        origin=Origin(xyz=(0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lobed_grip",
    )
    knob_1.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="knob_shaft",
    )
    knob_1.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="seating_washer",
    )

    model.articulation(
        "yoke_to_meridian",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "meridian_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0),
    )
    model.articulation(
        "yoke_to_side_knob_0",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=knob_0,
        origin=Origin(xyz=(-0.332, 0.0, TRUNNION_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )
    model.articulation(
        "yoke_to_side_knob_1",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=knob_1,
        origin=Origin(xyz=(0.332, 0.0, TRUNNION_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    knob_0 = object_model.get_part("side_knob_0")
    knob_1 = object_model.get_part("side_knob_1")
    tilt = object_model.get_articulation("yoke_to_meridian")
    spin = object_model.get_articulation("meridian_to_globe")
    knob_joint_0 = object_model.get_articulation("yoke_to_side_knob_0")
    knob_joint_1 = object_model.get_articulation("yoke_to_side_knob_1")

    ctx.check(
        "primary motions are present",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected tilting meridian, spinning globe, and two continuous knob joints.",
    )
    ctx.check(
        "joint axes match globe mechanism",
        tuple(round(v, 4) for v in tilt.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 4) for v in spin.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 4) for v in knob_joint_0.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 4) for v in knob_joint_1.axis) == (1.0, 0.0, 0.0),
        details="Meridian should tilt on horizontal trunnions and globe should spin on its polar axis.",
    )
    ctx.allow_overlap(
        pedestal,
        knob_0,
        elem_a="outer_boss_0",
        elem_b="knob_shaft",
        reason="The hand knob shaft intentionally passes through the yoke-side bushing collar.",
    )
    ctx.allow_overlap(
        pedestal,
        knob_1,
        elem_a="outer_boss_1",
        elem_b="knob_shaft",
        reason="The hand knob shaft intentionally passes through the yoke-side bushing collar.",
    )
    ctx.expect_within(
        meridian,
        pedestal,
        axes="x",
        inner_elem="meridian_ring",
        outer_elem="yoke_frame",
        margin=0.002,
        name="meridian ring is clipped between yoke cheeks",
    )
    ctx.expect_gap(
        globe,
        meridian,
        axis="z",
        positive_elem="polar_axle",
        negative_elem="south_socket",
        max_gap=0.004,
        max_penetration=0.003,
        name="lower polar axle seats near south pivot",
    )
    ctx.expect_gap(
        meridian,
        globe,
        axis="z",
        positive_elem="north_socket",
        negative_elem="polar_axle",
        max_gap=0.004,
        max_penetration=0.003,
        name="upper polar axle seats near north pivot",
    )
    ctx.expect_overlap(
        knob_0,
        pedestal,
        axes="yz",
        elem_a="knob_shaft",
        elem_b="yoke_cheek_0",
        min_overlap=0.010,
        name="first knob shaft aligns with yoke bore",
    )
    ctx.expect_overlap(
        knob_0,
        pedestal,
        axes="x",
        elem_a="knob_shaft",
        elem_b="outer_boss_0",
        min_overlap=0.006,
        name="first knob shaft is retained in bushing collar",
    )
    ctx.expect_overlap(
        knob_1,
        pedestal,
        axes="yz",
        elem_a="knob_shaft",
        elem_b="yoke_cheek_1",
        min_overlap=0.010,
        name="second knob shaft aligns with yoke bore",
    )
    ctx.expect_overlap(
        knob_1,
        pedestal,
        axes="x",
        elem_a="knob_shaft",
        elem_b="outer_boss_1",
        min_overlap=0.006,
        name="second knob shaft is retained in bushing collar",
    )
    with ctx.pose({tilt: 0.55}):
        ctx.expect_within(
            meridian,
            pedestal,
            axes="x",
            inner_elem="meridian_ring",
            outer_elem="yoke_frame",
            margin=0.002,
            name="tilted meridian remains captured between side trunnions",
        )

    return ctx.report()


object_model = build_object_model()
