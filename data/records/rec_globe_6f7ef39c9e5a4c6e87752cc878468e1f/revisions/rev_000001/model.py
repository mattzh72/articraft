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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_arc_points(
    *,
    center_x: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    samples: int,
    include_angles: tuple[float, ...] = (),
) -> list[tuple[float, float, float]]:
    angles = [start_angle + (end_angle - start_angle) * (i / samples) for i in range(samples + 1)]
    for angle in include_angles:
        if start_angle < angle < end_angle:
            angles.append(angle)
    angles = sorted(set(round(angle, 8) for angle in angles))
    return [
        (
            center_x + radius * math.cos(angle),
            0.0,
            radius * math.sin(angle),
        )
        for angle in angles
    ]


def _build_meridian_ring_mesh(
    *,
    ring_center_x: float,
    ring_radius: float,
    ring_tube_radius: float,
    trunnion_radius: float,
    trunnion_length: float,
    pivot_tip_center_z: float,
) :
    arc_points = _circle_arc_points(
        center_x=ring_center_x,
        radius=ring_radius,
        start_angle=0.55,
        end_angle=(2.0 * math.pi) - 0.55,
        samples=30,
        include_angles=(math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0),
    )
    ring_geom = tube_from_spline_points(
        arc_points,
        radius=ring_tube_radius,
        samples_per_segment=4,
        radial_segments=20,
        cap_ends=True,
    )
    ring_geom.merge(
        CylinderGeometry(radius=trunnion_radius, height=trunnion_length, radial_segments=24).rotate_x(math.pi / 2.0)
    )
    ring_geom.merge(
        CylinderGeometry(radius=0.011, height=0.018, radial_segments=24).rotate_x(math.pi / 2.0)
    )
    ring_geom.merge(
        CylinderGeometry(radius=0.0065, height=0.010, radial_segments=18).translate(
            ring_center_x,
            0.0,
            pivot_tip_center_z,
        )
    )
    ring_geom.merge(
        CylinderGeometry(radius=0.0065, height=0.010, radial_segments=18).translate(
            ring_center_x,
            0.0,
            -pivot_tip_center_z,
        )
    )
    return ring_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_globe")

    dark_iron = model.material("dark_iron", rgba=(0.14, 0.15, 0.16, 1.0))
    iron_highlight = model.material("iron_highlight", rgba=(0.24, 0.25, 0.27, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.66, 0.54, 0.29, 1.0))
    globe_blue = model.material("globe_blue", rgba=(0.29, 0.47, 0.66, 1.0))
    polar_metal = model.material("polar_metal", rgba=(0.72, 0.71, 0.68, 1.0))

    wall_support = model.part("wall_support")
    wall_support.visual(
        Box((0.022, 0.160, 0.240)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=dark_iron,
        name="wall_plate",
    )
    wall_support.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_highlight,
        name="wall_boss",
    )
    wall_support.visual(
        Cylinder(radius=0.014, length=0.118),
        origin=Origin(xyz=(0.107, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="support_arm",
    )
    wall_support.visual(
        Cylinder(radius=0.019, length=0.020),
        origin=Origin(xyz=(0.176, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_highlight,
        name="front_collar",
    )
    wall_support.visual(
        Box((0.018, 0.020, 0.060)),
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
        material=dark_iron,
        name="yoke_spine",
    )
    wall_support.visual(
        Box((0.024, 0.012, 0.086)),
        origin=Origin(xyz=(0.196, 0.017, 0.0)),
        material=dark_iron,
        name="upper_yoke_cheek",
    )
    wall_support.visual(
        Box((0.024, 0.012, 0.086)),
        origin=Origin(xyz=(0.196, -0.017, 0.0)),
        material=dark_iron,
        name="lower_yoke_cheek",
    )
    wall_support.inertial = Inertial.from_geometry(
        Box((0.220, 0.160, 0.240)),
        mass=6.5,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
    )

    ring_center_x = 0.152
    ring_radius = 0.152
    globe_radius = 0.130
    meridian_ring = model.part("meridian_ring")
    meridian_ring.visual(
        _save_mesh(
            "meridian_ring_assembly",
            _build_meridian_ring_mesh(
                ring_center_x=ring_center_x,
                ring_radius=ring_radius,
                ring_tube_radius=0.006,
                trunnion_radius=0.0055,
                trunnion_length=0.018,
                pivot_tip_center_z=0.150,
            ),
        ),
        material=aged_brass,
        name="ring_assembly",
    )
    meridian_ring.inertial = Inertial.from_geometry(
        Box((0.302, 0.026, 0.312)),
        mass=1.2,
        origin=Origin(xyz=(ring_center_x, 0.0, 0.0)),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=globe_radius),
        material=globe_blue,
        name="globe_shell",
    )
    globe.visual(
        Cylinder(radius=0.005, length=(2.0 * globe_radius) + 0.030),
        origin=Origin(),
        material=polar_metal,
        name="polar_axis",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=globe_radius),
        mass=2.4,
    )

    model.articulation(
        "support_to_ring_tilt",
        ArticulationType.REVOLUTE,
        parent=wall_support,
        child=meridian_ring,
        origin=Origin(xyz=(0.196, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "ring_to_globe_spin",
        ArticulationType.CONTINUOUS,
        parent=meridian_ring,
        child=globe,
        origin=Origin(xyz=(ring_center_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=6.0,
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

    support = object_model.get_part("wall_support")
    ring = object_model.get_part("meridian_ring")
    globe = object_model.get_part("globe")
    tilt = object_model.get_articulation("support_to_ring_tilt")
    spin = object_model.get_articulation("ring_to_globe_spin")

    ctx.check(
        "tilt joint uses a horizontal trunnion axis",
        tuple(round(value, 6) for value in tilt.axis) == (0.0, -1.0, 0.0),
        details=f"axis={tilt.axis}",
    )
    ctx.check(
        "globe spin uses the polar axis as a continuous joint",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    with ctx.pose({tilt: 0.0, spin: 0.0}):
        ctx.expect_origin_gap(
            globe,
            support,
            axis="x",
            min_gap=0.30,
            max_gap=0.36,
            name="globe projects out from the wall bracket",
        )
        ctx.expect_overlap(
            globe,
            ring,
            axes="xz",
            min_overlap=0.24,
            name="globe remains nested inside the meridian ring span",
        )
        ctx.expect_gap(
            support,
            ring,
            axis="y",
            positive_elem="upper_yoke_cheek",
            negative_elem="ring_assembly",
            min_gap=0.001,
            max_gap=0.004,
            name="upper cheek leaves trunnion running clearance",
        )
        ctx.expect_gap(
            ring,
            support,
            axis="y",
            positive_elem="ring_assembly",
            negative_elem="lower_yoke_cheek",
            min_gap=0.001,
            max_gap=0.004,
            name="lower cheek leaves trunnion running clearance",
        )

    rest_pos = ctx.part_world_position(globe)
    with ctx.pose({tilt: 0.45, spin: 0.0}):
        tilted_pos = ctx.part_world_position(globe)
    ctx.check(
        "positive tilt lifts the globe on the wall arm",
        rest_pos is not None and tilted_pos is not None and tilted_pos[2] > rest_pos[2] + 0.05,
        details=f"rest={rest_pos}, tilted={tilted_pos}",
    )

    with ctx.pose({tilt: 0.30, spin: 0.0}):
        spin_rest = ctx.part_world_position(globe)
    with ctx.pose({tilt: 0.30, spin: 1.20}):
        spin_turned = ctx.part_world_position(globe)
    spin_center_shift = None
    if spin_rest is not None and spin_turned is not None:
        spin_center_shift = math.dist(spin_rest, spin_turned)
    ctx.check(
        "globe spin occurs about its own center",
        spin_center_shift is not None and spin_center_shift <= 1e-6,
        details=f"rest={spin_rest}, spun={spin_turned}, shift={spin_center_shift}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
