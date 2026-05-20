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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spice_carousel_lazy_susan")

    brushed_steel = model.material("brushed_steel", rgba=(0.78, 0.79, 0.80, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.35, 0.37, 1.0))
    satin_black = model.material("satin_black", rgba=(0.14, 0.14, 0.15, 1.0))

    def annular_shell_mesh(
        *,
        outer_radius: float,
        inner_radius: float,
        z0: float,
        z1: float,
        name: str,
    ):
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(outer_radius, z0), (outer_radius, z1)],
                [(inner_radius, z0), (inner_radius, z1)],
                segments=64,
            ),
            name,
        )

    outer_rim_mesh = annular_shell_mesh(
        outer_radius=0.160,
        inner_radius=0.151,
        z0=0.024,
        z1=0.049,
        name="tray_outer_rim",
    )
    inner_ring_mesh = annular_shell_mesh(
        outer_radius=0.095,
        inner_radius=0.087,
        z0=0.024,
        z1=0.046,
        name="tray_inner_ring",
    )
    top_rail_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.156,
            tube=0.006,
            radial_segments=16,
            tubular_segments=72,
        ),
        "tray_top_rail",
    )
    base_ring_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.082,
            tube=0.007,
            radial_segments=16,
            tubular_segments=64,
        ),
        "tray_base_ring",
    )

    bearing_base = model.part("bearing_base")
    bearing_base.visual(
        Cylinder(radius=0.120, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_black,
        name="foot_disc",
    )
    bearing_base.visual(
        Cylinder(radius=0.060, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=dark_metal,
        name="bearing_housing",
    )
    bearing_base.visual(
        Cylinder(radius=0.072, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=brushed_steel,
        name="bearing_flange",
    )
    bearing_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.120, length=0.056),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    tray = model.part("tray")
    tray.visual(
        Cylinder(radius=0.165, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=brushed_steel,
        name="tray_floor",
    )
    tray.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_metal,
        name="hub_collar",
    )
    tray.visual(
        base_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=satin_black,
        name="underside_base_ring",
    )
    tray.visual(
        outer_rim_mesh,
        material=brushed_steel,
        name="outer_rim",
    )
    tray.visual(
        inner_ring_mesh,
        material=brushed_steel,
        name="inner_compartment_ring",
    )

    divider_center_radius = 0.119
    divider_length = 0.064
    divider_count = 8
    for index in range(divider_count):
        angle = (2.0 * math.pi * index) / divider_count
        tray.visual(
            Box((divider_length, 0.006, 0.031)),
            origin=Origin(
                xyz=(
                    divider_center_radius * math.cos(angle),
                    divider_center_radius * math.sin(angle),
                    0.0395,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"compartment_divider_{index}",
        )

    stanchion_radius = 0.153
    stanchion_count = 12
    for index in range(stanchion_count):
        angle = (2.0 * math.pi * index) / stanchion_count
        tray.visual(
            Cylinder(radius=0.0045, length=0.064),
            origin=Origin(
                xyz=(
                    stanchion_radius * math.cos(angle),
                    stanchion_radius * math.sin(angle),
                    0.056,
                )
            ),
            material=dark_metal,
            name=f"rail_stanchion_{index}",
        )

    tray.visual(
        top_rail_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=dark_metal,
        name="top_guard_rail",
    )
    tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.165, length=0.097),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.0485)),
    )

    model.articulation(
        "bearing_to_tray",
        ArticulationType.CONTINUOUS,
        parent=bearing_base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
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
    bearing_base = object_model.get_part("bearing_base")
    tray = object_model.get_part("tray")
    spin = object_model.get_articulation("bearing_to_tray")

    def aabb_center(aabb):
        if aabb is None:
            return None
        aabb_min, aabb_max = aabb
        return tuple((aabb_min[i] + aabb_max[i]) * 0.5 for i in range(3))

    ctx.expect_contact(
        tray,
        bearing_base,
        elem_a="hub_collar",
        elem_b="bearing_flange",
        contact_tol=5e-4,
        name="tray hub seats on bearing flange",
    )
    ctx.expect_overlap(
        tray,
        bearing_base,
        axes="xy",
        elem_a="hub_collar",
        elem_b="bearing_flange",
        min_overlap=0.050,
        name="tray hub remains centered over the bearing flange",
    )
    ctx.expect_gap(
        tray,
        bearing_base,
        axis="z",
        positive_elem="tray_floor",
        negative_elem="bearing_housing",
        min_gap=0.020,
        max_gap=0.035,
        name="tray floor clears the bearing housing",
    )

    divider_rest_center = aabb_center(
        ctx.part_element_world_aabb(tray, elem="compartment_divider_0")
    )
    with ctx.pose({spin: math.pi / 8.0}):
        ctx.expect_contact(
            tray,
            bearing_base,
            elem_a="hub_collar",
            elem_b="bearing_flange",
            contact_tol=5e-4,
            name="tray stays seated on the bearing while rotated",
        )
        divider_rotated_center = aabb_center(
            ctx.part_element_world_aabb(tray, elem="compartment_divider_0")
        )

    rest_radius = None
    rotated_radius = None
    if divider_rest_center is not None:
        rest_radius = math.hypot(divider_rest_center[0], divider_rest_center[1])
    if divider_rotated_center is not None:
        rotated_radius = math.hypot(divider_rotated_center[0], divider_rotated_center[1])

    ctx.check(
        "perimeter divider rotates around the vertical center axis",
        divider_rest_center is not None
        and divider_rotated_center is not None
        and abs(divider_rest_center[2] - divider_rotated_center[2]) < 1e-4
        and rest_radius is not None
        and rotated_radius is not None
        and abs(rest_radius - rotated_radius) < 0.002
        and (
            abs(divider_rest_center[0] - divider_rotated_center[0])
            + abs(divider_rest_center[1] - divider_rotated_center[1])
        )
        > 0.040,
        details=(
            f"rest_center={divider_rest_center}, rotated_center={divider_rotated_center}, "
            f"rest_radius={rest_radius}, rotated_radius={rotated_radius}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
