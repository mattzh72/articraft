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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


GLOBE_CENTER_Z = 0.62


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_gimbal_globe")

    brass = model.material("brass", rgba=(0.78, 0.67, 0.35, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.58, 0.48, 0.24, 1.0))
    mahogany = model.material("mahogany", rgba=(0.35, 0.17, 0.09, 1.0))
    ebony = model.material("ebony", rgba=(0.11, 0.08, 0.05, 1.0))
    ocean_blue = model.material("ocean_blue", rgba=(0.15, 0.32, 0.56, 1.0))
    parchment = model.material("parchment", rgba=(0.86, 0.82, 0.70, 1.0))

    outer_ring_mesh = _mesh(
        "outer_guard_ring",
        TorusGeometry(radius=0.22, tube=0.014, radial_segments=20, tubular_segments=84),
    )
    inner_ring_mesh = _mesh(
        "inner_meridian_ring",
        TorusGeometry(radius=0.18, tube=0.010, radial_segments=18, tubular_segments=72),
    )
    equator_band_mesh = _mesh(
        "globe_equator_band",
        TorusGeometry(radius=0.139, tube=0.0034, radial_segments=12, tubular_segments=72),
    )
    meridian_band_mesh = _mesh(
        "globe_meridian_band",
        TorusGeometry(radius=0.139, tube=0.0028, radial_segments=12, tubular_segments=72),
    )
    support_arm_mesh = _mesh(
        "pedestal_support_arm",
        tube_from_spline_points(
            [
                (-0.055, 0.0, 0.398),
                (-0.100, 0.0, 0.432),
                (-0.135, 0.0, 0.452),
                (-0.155, 0.0, 0.468),
            ],
            radius=0.0125,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    pedestal = model.part("pedestal_cradle")
    pedestal.visual(
        Cylinder(radius=0.16, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=mahogany,
        name="base_disc",
    )
    pedestal.visual(
        Cylinder(radius=0.11, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=brass,
        name="base_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.042, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=aged_brass,
        name="column_shaft",
    )
    pedestal.visual(
        Cylinder(radius=0.046, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.392)),
        material=brass,
        name="crown_cap",
    )
    pedestal.visual(
        support_arm_mesh,
        material=aged_brass,
        name="support_arm_left",
    )
    pedestal.visual(
        support_arm_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi)),
        material=aged_brass,
        name="support_arm_right",
    )
    pedestal.visual(
        outer_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, GLOBE_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="outer_guard_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.018, length=0.03),
        origin=Origin(
            xyz=(0.205, 0.0, GLOBE_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=aged_brass,
        name="pivot_socket_pos",
    )
    pedestal.visual(
        Cylinder(radius=0.018, length=0.03),
        origin=Origin(
            xyz=(-0.205, 0.0, GLOBE_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=aged_brass,
        name="pivot_socket_neg",
    )
    pedestal.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(
            xyz=(0.231, 0.0, GLOBE_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="pivot_knob_pos",
    )
    pedestal.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(
            xyz=(-0.231, 0.0, GLOBE_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="pivot_knob_neg",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.50, 0.12, 0.86)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
    )

    inner_ring = model.part("inner_ring")
    inner_ring.visual(
        inner_ring_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="meridian_ring",
    )
    inner_ring.visual(
        Cylinder(radius=0.008, length=0.025),
        origin=Origin(xyz=(0.1775, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="trunnion_pos",
    )
    inner_ring.visual(
        Cylinder(radius=0.008, length=0.025),
        origin=Origin(xyz=(-0.1775, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="trunnion_neg",
    )
    inner_ring.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=aged_brass,
        name="north_socket",
    )
    inner_ring.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=aged_brass,
        name="south_socket",
    )
    inner_ring.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.193)),
        material=brass,
        name="north_pivot_cap",
    )
    inner_ring.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.193)),
        material=brass,
        name="south_pivot_cap",
    )
    inner_ring.inertial = Inertial.from_geometry(
        Box((0.40, 0.05, 0.40)),
        mass=1.4,
        origin=Origin(),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=0.14),
        material=ocean_blue,
        name="globe_surface",
    )
    globe.visual(
        Sphere(radius=0.135),
        material=parchment,
        name="map_tint",
    )
    globe.visual(
        Cylinder(radius=0.0065, length=0.32),
        material=brass,
        name="polar_spindle",
    )
    globe.visual(
        equator_band_mesh,
        material=parchment,
        name="equator_band",
    )
    globe.visual(
        meridian_band_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=parchment,
        name="prime_meridian_band",
    )
    globe.visual(
        meridian_band_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        material=parchment,
        name="secondary_meridian_band",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=0.14),
        mass=2.4,
        origin=Origin(),
    )

    model.articulation(
        "gimbal_tilt",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=inner_ring,
        origin=Origin(xyz=(0.0, 0.0, GLOBE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=inner_ring,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.4,
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

    pedestal = object_model.get_part("pedestal_cradle")
    inner_ring = object_model.get_part("inner_ring")
    globe = object_model.get_part("globe")
    gimbal_tilt = object_model.get_articulation("gimbal_tilt")
    globe_spin = object_model.get_articulation("globe_spin")

    def _aabb_extent(aabb, axis: str) -> float | None:
        if aabb is None:
            return None
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        return aabb[1][axis_index] - aabb[0][axis_index]

    def _aabb_center(aabb, axis: str) -> float | None:
        if aabb is None:
            return None
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])

    gimbal_limits = gimbal_tilt.motion_limits
    spin_limits = globe_spin.motion_limits
    ctx.check(
        "gimbal tilt uses horizontal side-pivot axis",
        gimbal_tilt.axis == (1.0, 0.0, 0.0)
        and gimbal_limits is not None
        and gimbal_limits.lower is not None
        and gimbal_limits.upper is not None
        and gimbal_limits.lower < 0.0 < gimbal_limits.upper,
        details=f"axis={gimbal_tilt.axis}, limits={gimbal_limits}",
    )
    ctx.check(
        "globe spin uses polar axis",
        globe_spin.axis == (0.0, 0.0, 1.0)
        and spin_limits is not None
        and spin_limits.lower is None
        and spin_limits.upper is None,
        details=f"axis={globe_spin.axis}, limits={spin_limits}",
    )

    with ctx.pose({gimbal_tilt: 0.0, globe_spin: 0.0}):
        ctx.expect_contact(
            globe,
            inner_ring,
            elem_a="polar_spindle",
            elem_b="north_socket",
            name="north polar pivot is seated in the inner ring",
        )
        ctx.expect_contact(
            globe,
            inner_ring,
            elem_a="polar_spindle",
            elem_b="south_socket",
            name="south polar pivot is seated in the inner ring",
        )
        ctx.expect_contact(
            inner_ring,
            pedestal,
            elem_a="trunnion_pos",
            elem_b="pivot_socket_pos",
            name="right trunnion is seated in the outer cradle",
        )
        ctx.expect_contact(
            inner_ring,
            pedestal,
            elem_a="trunnion_neg",
            elem_b="pivot_socket_neg",
            name="left trunnion is seated in the outer cradle",
        )
        ctx.expect_within(
            globe,
            inner_ring,
            axes="xz",
            inner_elem="globe_surface",
            outer_elem="meridian_ring",
            margin=0.0,
            name="globe fits inside the inner meridian ring",
        )
        ctx.expect_within(
            inner_ring,
            pedestal,
            axes="xz",
            inner_elem="meridian_ring",
            outer_elem="outer_guard_ring",
            margin=0.0,
            name="inner ring fits inside the outer guard ring",
        )

    prime_rest = ctx.part_element_world_aabb(globe, elem="prime_meridian_band")
    with ctx.pose({globe_spin: math.pi / 2.0}):
        prime_rotated = ctx.part_element_world_aabb(globe, elem="prime_meridian_band")
    ctx.check(
        "globe spin visibly rotates the meridian band around the polar axis",
        prime_rest is not None
        and prime_rotated is not None
        and (_aabb_extent(prime_rest, "x") or 0.0) > 0.20
        and (_aabb_extent(prime_rest, "y") or 0.0) < 0.02
        and (_aabb_extent(prime_rotated, "x") or 0.0) < 0.02
        and (_aabb_extent(prime_rotated, "y") or 0.0) > 0.20,
        details=f"rest={prime_rest}, rotated={prime_rotated}",
    )

    north_rest = ctx.part_element_world_aabb(inner_ring, elem="north_pivot_cap")
    with ctx.pose({gimbal_tilt: 0.50}):
        north_tilted = ctx.part_element_world_aabb(inner_ring, elem="north_pivot_cap")
        ctx.expect_contact(
            inner_ring,
            pedestal,
            elem_a="trunnion_pos",
            elem_b="pivot_socket_pos",
            name="right trunnion stays seated while the gimbal tilts",
        )
        ctx.expect_contact(
            inner_ring,
            pedestal,
            elem_a="trunnion_neg",
            elem_b="pivot_socket_neg",
            name="left trunnion stays seated while the gimbal tilts",
        )
    ctx.check(
        "gimbal tilt lifts the north pivot out of the cradle plane",
        north_rest is not None
        and north_tilted is not None
        and abs((_aabb_center(north_tilted, "y") or 0.0) - (_aabb_center(north_rest, "y") or 0.0)) > 0.08
        and (_aabb_center(north_tilted, "z") or 0.0) < (_aabb_center(north_rest, "z") or 0.0) - 0.01,
        details=f"rest={north_rest}, tilted={north_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
