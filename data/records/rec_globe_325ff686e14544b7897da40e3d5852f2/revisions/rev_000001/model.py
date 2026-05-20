from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


GLOBE_RADIUS = 0.18
GRID_RADIUS = GLOBE_RADIUS + 0.0005
MERIDIAN_RADIUS = 0.225
MERIDIAN_TUBE = 0.009
CENTER_Z = 0.34
AXIAL_TILT = math.radians(23.5)


def _torus_mesh(radius: float, tube: float, name: str, *, z: float = 0.0, vertical: bool = False, yaw: float = 0.0):
    geom = TorusGeometry(radius, tube, radial_segments=20, tubular_segments=96)
    if vertical:
        geom.rotate_x(math.pi / 2.0)
    if yaw:
        geom.rotate_z(yaw)
    if z:
        geom.translate(0.0, 0.0, z)
    return mesh_from_geometry(geom, name)


def _land_patch_mesh(name: str, x: float, z: float, rx: float, rz: float):
    y = -math.sqrt(max(GLOBE_RADIUS * GLOBE_RADIUS - x * x - z * z, 0.0)) - 0.0015
    geom = SphereGeometry(1.0, width_segments=18, height_segments=8)
    geom.scale(rx, 0.003, rz).translate(x, y, z)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_tilted_world_globe")

    ocean = Material("deep_ocean_blue", rgba=(0.04, 0.22, 0.62, 1.0))
    land = Material("muted_land_green", rgba=(0.10, 0.46, 0.18, 1.0))
    grid = Material("engraved_grid_cream", rgba=(0.90, 0.82, 0.58, 1.0))
    brass = Material("brushed_brass", rgba=(0.78, 0.58, 0.22, 1.0))
    dark_wood = Material("dark_walnut", rgba=(0.18, 0.10, 0.045, 1.0))
    black = Material("black_felt", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.245, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_wood,
        name="round_plinth",
    )
    base.visual(
        Cylinder(radius=0.205, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=black,
        name="felt_foot",
    )
    base.visual(
        Cylinder(radius=0.073, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=brass,
        name="center_boss",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=brass,
        name="center_stem",
    )
    base.visual(
        Box((0.56, 0.034, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=brass,
        name="yoke_foot",
    )
    for idx, x in enumerate((-0.270, 0.270)):
        base.visual(
            Cylinder(radius=0.013, length=0.285),
            origin=Origin(xyz=(x, 0.0, 0.1765)),
            material=brass,
            name=f"side_post_{idx}",
        )
        base.visual(
            Cylinder(radius=0.026, length=0.050),
            origin=Origin(xyz=(x, 0.0, CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"side_bushing_{idx}",
        )

    meridian = model.part("meridian")
    meridian.visual(
        _torus_mesh(MERIDIAN_RADIUS, MERIDIAN_TUBE, "meridian_ring", vertical=True),
        material=brass,
        name="degree_ring",
    )
    for idx, x in enumerate((-0.247, 0.247)):
        meridian.visual(
            Cylinder(radius=0.007, length=0.050),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"side_pin_{idx}",
        )
    for idx, z in enumerate((-0.202, 0.202)):
        meridian.visual(
            Cylinder(radius=0.018, length=0.033),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brass,
            name=f"polar_socket_{idx}",
        )
    meridian.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=brass,
        name="north_cap",
    )
    meridian.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=brass,
        name="south_cap",
    )

    globe = model.part("globe")
    globe.visual(Sphere(radius=GLOBE_RADIUS), material=ocean, name="ocean_sphere")
    globe.visual(
        _torus_mesh(GRID_RADIUS, 0.0015, "equator_grid"),
        material=grid,
        name="equator",
    )
    for idx, z in enumerate((-0.12, -0.06, 0.06, 0.12)):
        latitude_radius = math.sqrt(max(GRID_RADIUS * GRID_RADIUS - z * z, 0.0))
        globe.visual(
            _torus_mesh(latitude_radius, 0.0012, f"latitude_grid_{idx}", z=z),
            material=grid,
            name=f"latitude_{idx}",
        )
    for idx, yaw in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)):
        globe.visual(
            _torus_mesh(GRID_RADIUS, 0.0012, f"longitude_grid_{idx}", vertical=True, yaw=yaw),
            material=grid,
            name=f"longitude_{idx}",
        )
    for idx, (x, z, rx, rz) in enumerate(
        (
            (-0.055, 0.070, 0.034, 0.050),
            (0.040, 0.045, 0.049, 0.030),
            (0.090, -0.035, 0.030, 0.055),
            (-0.075, -0.050, 0.044, 0.035),
            (0.000, -0.115, 0.035, 0.026),
        )
    ):
        globe.visual(
            _land_patch_mesh(f"land_patch_mesh_{idx}", x, z, rx, rz),
            material=land,
            name=f"land_{idx}",
        )
    for idx, z in enumerate((-0.195, 0.195)):
        globe.visual(
            Cylinder(radius=0.007, length=0.039),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brass,
            name=f"polar_pin_{idx}",
        )

    model.articulation(
        "base_to_meridian",
        ArticulationType.REVOLUTE,
        parent=base,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z), rpy=(AXIAL_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "meridian_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    meridian_joint = object_model.get_articulation("base_to_meridian")
    spin_joint = object_model.get_articulation("meridian_to_globe")

    for idx in (0, 1):
        ctx.allow_overlap(
            base,
            meridian,
            elem_a=f"side_bushing_{idx}",
            elem_b=f"side_pin_{idx}",
            reason="The meridian axle pin is intentionally captured inside the yoke bushing.",
        )
        ctx.expect_within(
            meridian,
            base,
            axes="yz",
            inner_elem=f"side_pin_{idx}",
            outer_elem=f"side_bushing_{idx}",
            margin=0.002,
            name=f"side pin {idx} centered in yoke bushing",
        )
        ctx.expect_overlap(
            meridian,
            base,
            axes="x",
            elem_a=f"side_pin_{idx}",
            elem_b=f"side_bushing_{idx}",
            min_overlap=0.018,
            name=f"side pin {idx} retained in bushing",
        )

    for idx in (0, 1):
        ctx.allow_overlap(
            meridian,
            globe,
            elem_a=f"polar_socket_{idx}",
            elem_b=f"polar_pin_{idx}",
            reason="The polar pin is intentionally seated in the meridian socket.",
        )
        ctx.expect_within(
            globe,
            meridian,
            axes="xy",
            inner_elem=f"polar_pin_{idx}",
            outer_elem=f"polar_socket_{idx}",
            margin=0.003,
            name=f"polar pin {idx} centered in socket",
        )
        ctx.expect_overlap(
            globe,
            meridian,
            axes="z",
            elem_a=f"polar_pin_{idx}",
            elem_b=f"polar_socket_{idx}",
            min_overlap=0.006,
            name=f"polar pin {idx} seated axially",
        )

    ctx.expect_gap(
        globe,
        base,
        axis="z",
        min_gap=0.10,
        positive_elem="ocean_sphere",
        negative_elem="round_plinth",
        name="globe clears the tabletop base",
    )
    ctx.expect_gap(
        meridian,
        base,
        axis="z",
        min_gap=0.04,
        positive_elem="degree_ring",
        negative_elem="round_plinth",
        name="tilted meridian clears round plinth",
    )

    ctx.check(
        "globe has polar spin joint",
        spin_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(spin_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin_joint.articulation_type}, axis={spin_joint.axis}",
    )
    ctx.check(
        "meridian pivots through side supports",
        meridian_joint.articulation_type == ArticulationType.REVOLUTE and tuple(meridian_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={meridian_joint.articulation_type}, axis={meridian_joint.axis}",
    )

    def _aabb_center(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_top = _aabb_center(meridian, "polar_socket_1")
    with ctx.pose({meridian_joint: 0.35}):
        tilted_top = _aabb_center(meridian, "polar_socket_1")
    ctx.check(
        "meridian rotation swings polar pivot",
        rest_top is not None
        and tilted_top is not None
        and abs(tilted_top[1] - rest_top[1]) > 0.04,
        details=f"rest={rest_top}, tilted={tilted_top}",
    )

    rest_land = _aabb_center(globe, "land_0")
    with ctx.pose({spin_joint: math.pi / 2.0}):
        spun_land = _aabb_center(globe, "land_0")
    ctx.check(
        "globe spin moves surface map",
        rest_land is not None
        and spun_land is not None
        and abs(spun_land[0] - rest_land[0]) > 0.03,
        details=f"rest={rest_land}, spun={spun_land}",
    )

    return ctx.report()


object_model = build_object_model()
