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
    DomeGeometry,
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
    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    model = ArticulatedObject(name="brick_lighthouse")

    brick = model.material("brick", rgba=(0.58, 0.24, 0.18, 1.0))
    stone = model.material("stone", rgba=(0.76, 0.74, 0.69, 1.0))
    lantern_metal = model.material("lantern_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    weathered_copper = model.material("weathered_copper", rgba=(0.30, 0.43, 0.40, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.72, 0.86, 0.94, 0.24))
    beacon_brass = model.material("beacon_brass", rgba=(0.83, 0.67, 0.28, 1.0))
    beacon_glass = model.material("beacon_glass", rgba=(0.94, 0.88, 0.66, 0.72))
    hatch_paint = model.material("hatch_paint", rgba=(0.28, 0.31, 0.26, 1.0))
    shadow_dark = model.material("shadow_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    tower_shell_mesh = _mesh(
        "tower_shell",
        LatheGeometry.from_shell_profiles(
            [
                (3.40, 0.00),
                (3.30, 1.40),
                (2.95, 5.40),
                (2.55, 9.40),
                (2.20, 12.20),
                (1.95, 14.42),
            ],
            [
                (2.85, 0.00),
                (2.77, 1.40),
                (2.48, 5.40),
                (2.16, 9.40),
                (1.87, 12.20),
                (1.64, 14.42),
            ],
            segments=88,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    lantern_shell_mesh = _mesh(
        "lantern_shell",
        LatheGeometry.from_shell_profiles(
            [
                (1.54, 0.00),
                (1.54, 1.95),
                (1.47, 2.22),
            ],
            [
                (1.42, 0.00),
                (1.42, 1.95),
                (1.36, 2.16),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    lantern_curb_mesh = _mesh(
        "lantern_curb",
        LatheGeometry.from_shell_profiles(
            [
                (1.70, 0.00),
                (1.70, 0.20),
            ],
            [
                (0.42, 0.00),
                (0.42, 0.20),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    gallery_ring_mesh = _mesh(
        "gallery_ring",
        TorusGeometry(radius=2.18, tube=0.10, radial_segments=20, tubular_segments=80),
    )
    cornice_ring_mesh = _mesh(
        "cornice_ring",
        TorusGeometry(radius=2.05, tube=0.08, radial_segments=20, tubular_segments=72),
    )
    bearing_ring_mesh = _mesh(
        "bearing_ring",
        LatheGeometry.from_shell_profiles(
            [
                (0.34, 0.00),
                (0.34, 0.08),
            ],
            [
                (0.135, 0.00),
                (0.135, 0.08),
            ],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    roof_mesh = _mesh(
        "lantern_roof",
        DomeGeometry(radius=1.18, radial_segments=48, height_segments=20, closed=True),
    )

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=3.75, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=stone,
        name="foundation_plinth",
    )
    tower.visual(
        Cylinder(radius=3.52, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.57)),
        material=stone,
        name="base_water_table",
    )
    tower.visual(
        tower_shell_mesh,
        material=brick,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=2.34, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 14.56)),
        material=stone,
        name="beacon_deck",
    )
    tower.visual(
        cornice_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 14.42)),
        material=stone,
        name="cornice_ring",
    )
    tower.visual(
        gallery_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 14.68)),
        material=lantern_metal,
        name="gallery_guard_ring",
    )
    tower.visual(
        lantern_curb_mesh,
        origin=Origin(xyz=(0.0, 0.0, 14.82)),
        material=stone,
        name="lantern_base_curb",
    )
    tower.visual(
        lantern_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 14.82)),
        material=lantern_glass,
        name="lantern_glass",
    )
    for index in range(8):
        angle = (math.tau * index) / 8.0
        tower.visual(
            Cylinder(radius=0.045, length=2.30),
            origin=Origin(
                xyz=(math.cos(angle) * 1.49, math.sin(angle) * 1.49, 16.02)
            ),
            material=lantern_metal,
            name=f"lantern_post_{index:02d}",
        )
    tower.visual(
        Cylinder(radius=1.57, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 17.12)),
        material=lantern_metal,
        name="roof_skirt",
    )
    tower.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, 17.12)),
        material=weathered_copper,
        name="lantern_roof",
    )
    tower.visual(
        Cylinder(radius=0.18, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 18.02)),
        material=lantern_metal,
        name="roof_vent",
    )
    tower.visual(
        Cylinder(radius=0.24, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 18.24)),
        material=lantern_metal,
        name="vent_cap",
    )
    tower.visual(
        bearing_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 14.76)),
        material=lantern_metal,
        name="beacon_bearing_ring",
    )
    tower.visual(
        Cylinder(radius=0.14, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 14.85)),
        material=lantern_metal,
        name="beacon_pedestal",
    )
    tower.visual(
        Box((0.20, 1.12, 0.96)),
        origin=Origin(xyz=(2.11, 0.0, 13.86)),
        material=stone,
        name="service_frame",
    )
    tower.visual(
        Box((0.10, 0.90, 0.72)),
        origin=Origin(xyz=(2.00, 0.0, 13.82)),
        material=shadow_dark,
        name="service_opening_shadow",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=3.75, length=18.35),
        mass=90000.0,
        origin=Origin(xyz=(0.0, 0.0, 9.175)),
    )

    beacon = model.part("beacon")
    beacon.visual(
        Cylinder(radius=0.10, length=1.88),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=lantern_metal,
        name="center_shaft",
    )
    beacon.visual(
        Cylinder(radius=0.14, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=lantern_metal,
        name="turntable_base",
    )
    beacon.visual(
        Cylinder(radius=0.26, length=0.94),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=beacon_brass,
        name="optic_core",
    )
    beacon.visual(
        Box((0.22, 0.88, 0.84)),
        origin=Origin(xyz=(0.36, 0.0, 0.86)),
        material=beacon_glass,
        name="lens_wing_east",
    )
    beacon.visual(
        Box((0.22, 0.88, 0.84)),
        origin=Origin(xyz=(-0.36, 0.0, 0.86)),
        material=beacon_glass,
        name="lens_wing_west",
    )
    beacon.visual(
        Box((0.88, 0.22, 0.84)),
        origin=Origin(xyz=(0.0, 0.36, 0.86)),
        material=beacon_glass,
        name="lens_wing_north",
    )
    beacon.visual(
        Box((0.88, 0.22, 0.84)),
        origin=Origin(xyz=(0.0, -0.36, 0.86)),
        material=beacon_glass,
        name="lens_wing_south",
    )
    beacon.visual(
        Box((0.28, 0.18, 0.26)),
        origin=Origin(xyz=(0.25, -0.11, 0.44)),
        material=lantern_metal,
        name="drive_motor_box",
    )
    beacon.visual(
        Cylinder(radius=0.18, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 1.68)),
        material=beacon_brass,
        name="top_cap",
    )
    beacon.inertial = Inertial.from_geometry(
        Box((1.10, 1.10, 1.95)),
        mass=450.0,
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
    )

    service_hatch = model.part("service_hatch")
    service_hatch.visual(
        Cylinder(radius=0.032, length=1.00),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lantern_metal,
        name="hinge_barrel",
    )
    service_hatch.visual(
        Box((0.044, 0.98, 0.84)),
        origin=Origin(xyz=(0.022, 0.0, -0.42)),
        material=hatch_paint,
        name="hatch_panel",
    )
    service_hatch.visual(
        Cylinder(radius=0.016, length=0.22),
        origin=Origin(xyz=(0.060, 0.0, -0.46), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lantern_metal,
        name="pull_handle",
    )
    service_hatch.inertial = Inertial.from_geometry(
        Box((0.09, 1.02, 0.88)),
        mass=40.0,
        origin=Origin(xyz=(0.035, 0.0, -0.42)),
    )

    model.articulation(
        "beacon_rotation",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 14.94)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.6),
    )
    model.articulation(
        "service_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=service_hatch,
        origin=Origin(xyz=(2.21, 0.0, 14.28)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=1.25,
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

    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    service_hatch = object_model.get_part("service_hatch")
    beacon_rotation = object_model.get_articulation("beacon_rotation")
    hatch_hinge = object_model.get_articulation("service_hatch_hinge")

    ctx.check(
        "beacon rotation is vertical continuous motion",
        beacon_rotation.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in beacon_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"type={beacon_rotation.joint_type}, axis={beacon_rotation.axis}",
    )
    ctx.check(
        "service hatch uses horizontal hinge below the lantern deck",
        hatch_hinge.joint_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in hatch_hinge.axis) == (0.0, -1.0, 0.0)
        and abs(hatch_hinge.origin.xyz[2] - 14.28) < 0.25,
        details=f"type={hatch_hinge.joint_type}, axis={hatch_hinge.axis}, origin={hatch_hinge.origin}",
    )

    ctx.expect_within(
        beacon,
        tower,
        axes="xy",
        outer_elem="lantern_glass",
        margin=0.05,
        name="beacon remains within the lantern enclosure footprint",
    )
    ctx.expect_contact(
        beacon,
        tower,
        elem_a="turntable_base",
        elem_b="beacon_pedestal",
        contact_tol=0.001,
        name="beacon turntable sits on the central pedestal",
    )
    ctx.expect_overlap(
        service_hatch,
        tower,
        axes="yz",
        elem_a="hatch_panel",
        elem_b="service_frame",
        min_overlap=0.70,
        name="closed service hatch covers its framed opening",
    )
    ctx.expect_gap(
        service_hatch,
        tower,
        axis="x",
        positive_elem="hatch_panel",
        negative_elem="service_frame",
        min_gap=-0.001,
        max_gap=0.07,
        name="closed service hatch sits close to the tower face",
    )

    rest_hatch_aabb = ctx.part_element_world_aabb(service_hatch, elem="hatch_panel")
    rest_beacon_pos = ctx.part_world_position(beacon)
    with ctx.pose({beacon_rotation: math.pi / 2.0, hatch_hinge: 1.0}):
        open_hatch_aabb = ctx.part_element_world_aabb(service_hatch, elem="hatch_panel")
        spun_beacon_pos = ctx.part_world_position(beacon)

    ctx.check(
        "service hatch opens outward",
        rest_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][0] > rest_hatch_aabb[1][0] + 0.25,
        details=f"rest={rest_hatch_aabb}, open={open_hatch_aabb}",
    )
    ctx.check(
        "beacon rotates in place on the center shaft",
        rest_beacon_pos is not None
        and spun_beacon_pos is not None
        and max(
            abs(spun_beacon_pos[i] - rest_beacon_pos[i]) for i in range(3)
        )
        < 1e-6,
        details=f"rest={rest_beacon_pos}, spun={spun_beacon_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
