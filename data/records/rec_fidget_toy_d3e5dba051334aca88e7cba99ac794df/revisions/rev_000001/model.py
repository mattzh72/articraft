from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_shell_mesh(
    name: str,
    *,
    inner_radius: float,
    outer_radius: float,
    height: float,
    edge_radius: float,
):
    half_height = height * 0.5
    outer_profile = [
        (outer_radius - edge_radius * 0.55, -half_height),
        (outer_radius, -half_height + edge_radius * 0.85),
        (outer_radius + edge_radius * 0.12, 0.0),
        (outer_radius, half_height - edge_radius * 0.85),
        (outer_radius - edge_radius * 0.55, half_height),
    ]
    inner_profile = [
        (inner_radius + edge_radius * 0.55, -half_height),
        (inner_radius, -half_height + edge_radius * 0.85),
        (inner_radius - edge_radius * 0.12, 0.0),
        (inner_radius, half_height - edge_radius * 0.85),
        (inner_radius + edge_radius * 0.55, half_height),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_fidget_ring_set")

    carrier_dark = model.material("carrier_dark", rgba=(0.10, 0.11, 0.13, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    warm_titanium = model.material("warm_titanium", rgba=(0.63, 0.55, 0.42, 1.0))
    magnet_black = model.material("magnet_black", rgba=(0.08, 0.09, 0.10, 1.0))

    carrier_radius = 0.053
    carrier_thickness = 0.0024
    ring_height = 0.0072
    ring_mount_z = carrier_thickness + (ring_height * 0.5)

    carrier = model.part("carrier")
    carrier.visual(
        Cylinder(radius=carrier_radius, length=carrier_thickness),
        origin=Origin(xyz=(0.0, 0.0, carrier_thickness * 0.5)),
        material=carrier_dark,
        name="carrier_plate",
    )
    carrier.visual(
        Cylinder(radius=0.0135, length=0.0042),
        origin=Origin(xyz=(0.0, 0.0, 0.0021)),
        material=carrier_dark,
        name="center_core",
    )
    carrier.visual(
        Cylinder(radius=0.0495, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0018)),
        material=carrier_dark,
        name="magnetic_backer",
    )
    carrier.inertial = Inertial.from_geometry(
        Cylinder(radius=carrier_radius, length=0.0042),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0021)),
    )

    inner_ring = model.part("inner_ring")
    inner_ring.visual(
        _ring_shell_mesh(
            "inner_ring_shell",
            inner_radius=0.018,
            outer_radius=0.034,
            height=ring_height,
            edge_radius=0.0012,
        ),
        material=satin_steel,
        name="inner_ring_shell",
    )
    inner_ring.visual(
        Cylinder(radius=0.0042, length=0.0018),
        origin=Origin(xyz=(0.025, 0.0, 0.0027)),
        material=magnet_black,
        name="inner_magnet",
    )
    inner_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=ring_height),
        mass=0.07,
    )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        _ring_shell_mesh(
            "outer_ring_shell",
            inner_radius=0.0375,
            outer_radius=0.052,
            height=ring_height,
            edge_radius=0.0014,
        ),
        material=warm_titanium,
        name="outer_ring_shell",
    )
    outer_ring.visual(
        Cylinder(radius=0.0046, length=0.0018),
        origin=Origin(xyz=(0.044, 0.0, 0.0027)),
        material=magnet_black,
        name="outer_magnet",
    )
    outer_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=ring_height),
        mass=0.10,
    )

    model.articulation(
        "carrier_to_inner_ring",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=inner_ring,
        origin=Origin(xyz=(0.0, 0.0, ring_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=18.0),
    )
    model.articulation(
        "carrier_to_outer_ring",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, ring_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=18.0),
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

    carrier = object_model.get_part("carrier")
    inner_ring = object_model.get_part("inner_ring")
    outer_ring = object_model.get_part("outer_ring")
    inner_spin = object_model.get_articulation("carrier_to_inner_ring")
    outer_spin = object_model.get_articulation("carrier_to_outer_ring")

    def elem_center(part, elem_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    ctx.expect_gap(
        inner_ring,
        carrier,
        axis="z",
        max_gap=0.0002,
        max_penetration=0.0,
        positive_elem="inner_ring_shell",
        negative_elem="carrier_plate",
        name="inner ring seats on the carrier",
    )
    ctx.expect_gap(
        outer_ring,
        carrier,
        axis="z",
        max_gap=0.0002,
        max_penetration=0.0,
        positive_elem="outer_ring_shell",
        negative_elem="carrier_plate",
        name="outer ring seats on the carrier",
    )
    ctx.expect_overlap(
        inner_ring,
        carrier,
        axes="xy",
        min_overlap=0.036,
        name="inner ring stays centered over the carrier",
    )
    ctx.expect_overlap(
        outer_ring,
        carrier,
        axes="xy",
        min_overlap=0.090,
        name="outer ring stays centered over the carrier",
    )

    ctx.check(
        "inner spin uses continuous coaxial motion",
        inner_spin.joint_type == ArticulationType.CONTINUOUS
        and inner_spin.axis == (0.0, 0.0, 1.0)
        and inner_spin.motion_limits is not None
        and inner_spin.motion_limits.lower is None
        and inner_spin.motion_limits.upper is None,
        details=(
            f"type={inner_spin.joint_type}, axis={inner_spin.axis}, "
            f"limits={inner_spin.motion_limits}"
        ),
    )
    ctx.check(
        "outer spin uses continuous coaxial motion",
        outer_spin.joint_type == ArticulationType.CONTINUOUS
        and outer_spin.axis == (0.0, 0.0, 1.0)
        and outer_spin.motion_limits is not None
        and outer_spin.motion_limits.lower is None
        and outer_spin.motion_limits.upper is None,
        details=(
            f"type={outer_spin.joint_type}, axis={outer_spin.axis}, "
            f"limits={outer_spin.motion_limits}"
        ),
    )

    inner_rest = elem_center(inner_ring, "inner_magnet")
    outer_rest = elem_center(outer_ring, "outer_magnet")

    with ctx.pose({inner_spin: math.pi / 2.0, outer_spin: 0.0}):
        inner_spun = elem_center(inner_ring, "inner_magnet")
        outer_static = elem_center(outer_ring, "outer_magnet")

    ctx.check(
        "inner ring rotates without dragging the outer ring",
        inner_rest is not None
        and outer_rest is not None
        and inner_spun is not None
        and outer_static is not None
        and abs(inner_spun[1]) > 0.020
        and abs(inner_spun[0]) < 0.008
        and abs(outer_static[0] - outer_rest[0]) < 0.001
        and abs(outer_static[1] - outer_rest[1]) < 0.001,
        details=(
            f"inner_rest={inner_rest}, inner_spun={inner_spun}, "
            f"outer_rest={outer_rest}, outer_static={outer_static}"
        ),
    )

    with ctx.pose({inner_spin: 0.0, outer_spin: -math.pi / 2.0}):
        inner_static = elem_center(inner_ring, "inner_magnet")
        outer_spun = elem_center(outer_ring, "outer_magnet")

    ctx.check(
        "outer ring rotates without dragging the inner ring",
        inner_rest is not None
        and outer_rest is not None
        and inner_static is not None
        and outer_spun is not None
        and abs(outer_spun[1]) > 0.036
        and abs(outer_spun[0]) < 0.010
        and abs(inner_static[0] - inner_rest[0]) < 0.001
        and abs(inner_static[1] - inner_rest[1]) < 0.001,
        details=(
            f"outer_rest={outer_rest}, outer_spun={outer_spun}, "
            f"inner_rest={inner_rest}, inner_static={inner_static}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
