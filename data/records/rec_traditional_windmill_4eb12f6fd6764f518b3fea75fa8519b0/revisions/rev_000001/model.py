from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    wire_from_points,
)
import cadquery as cq


CAP_Z = 1.75
HUB_X = 0.65
HUB_Z = 0.25


def _blade_point(x: float, radial: float, offset: float, angle: float) -> tuple[float, float, float]:
    """Point in the hub frame for one sail blade.

    The hub shaft is local +X; the sail plane is local YZ.  ``radial`` follows
    the blade centerline and ``offset`` is the local crosswise lattice width.
    """
    u_y = math.sin(angle)
    u_z = math.cos(angle)
    v_y = math.cos(angle)
    v_z = -math.sin(angle)
    return (x, radial * u_y + offset * v_y, radial * u_z + offset * v_z)


def _blade_lattice(angle: float):
    """One continuous tubular wooden lattice blade in the hub frame."""
    x = 0.055
    root_r = 0.065
    tip_r = 0.68
    root_w = 0.045
    tip_w = 0.19

    def side(r: float, side_sign: float) -> tuple[float, float, float]:
        t = (r - root_r) / (tip_r - root_r)
        width = root_w + t * (tip_w - root_w)
        return _blade_point(x, r, side_sign * width * 0.5, angle)

    def center(r: float) -> tuple[float, float, float]:
        return _blade_point(x, r, 0.0, angle)

    # The route deliberately forms the outer rails first, then a continuous
    # zig-zag of battens and diagonals.  It is one connected tube, so each
    # lattice blade reads as a supported assembly rather than separate sticks.
    path = [
        side(root_r, 1.0),
        side(tip_r, 1.0),
        side(tip_r, -1.0),
        side(root_r, -1.0),
        side(root_r, 1.0),
        center(root_r * 0.65),
        center(tip_r),
        side(tip_r, 1.0),
    ]

    for r in (0.22, 0.34, 0.46, 0.58):
        path.append(side(r, -1.0))
        path.append(side(r, 1.0))

    path.extend([center(tip_r), center(root_r * 0.55)])

    return wire_from_points(
        path,
        radius=0.010,
        radial_segments=12,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.006,
        corner_segments=5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("warm_stone", color=(0.62, 0.57, 0.48, 1.0))
    stone_shadow = model.material("stone_courses", color=(0.48, 0.45, 0.39, 1.0))
    thatch = model.material("weathered_thatch", color=(0.45, 0.31, 0.16, 1.0))
    painted_wood = model.material("painted_wood", color=(0.82, 0.76, 0.63, 1.0))
    blade_wood = model.material("sail_wood", color=(0.72, 0.50, 0.27, 1.0))
    dark_wood = model.material("dark_wood", color=(0.20, 0.12, 0.06, 1.0))
    metal = model.material("dark_iron", color=(0.08, 0.09, 0.09, 1.0))
    guard_paint = model.material("guard_iron", color=(0.12, 0.15, 0.16, 1.0))
    red_tip = model.material("painted_tip", color=(0.65, 0.08, 0.04, 1.0))

    # Root tower: an octagonal, tapering masonry tower with mounted door,
    # windows, and stone course bands.
    tower = model.part("tower")
    tower_body = (
        cq.Workplane("XY")
        .polygon(8, 0.74)
        .workplane(offset=1.62)
        .polygon(8, 0.56)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.08))
    )
    tower.visual(
        mesh_from_cadquery(tower_body, "tower_body", tolerance=0.002),
        material=stone,
        name="tower_body",
    )
    tower.visual(
        Cylinder(radius=0.44, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stone_shadow,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=0.315, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, CAP_Z - 0.03)),
        material=stone_shadow,
        name="top_turntable",
    )
    for i, (z, r) in enumerate(((0.55, 0.355), (1.05, 0.325), (1.48, 0.302))):
        tower.visual(
            Cylinder(radius=r, length=0.025),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=stone_shadow,
            name=f"course_band_{i}",
        )

    tower.visual(
        Box((0.024, 0.20, 0.44)),
        origin=Origin(xyz=(0.368, 0.0, 0.32)),
        material=dark_wood,
        name="front_door",
    )
    tower.visual(
        Box((0.026, 0.24, 0.035)),
        origin=Origin(xyz=(0.381, 0.0, 0.54)),
        material=painted_wood,
        name="door_lintel",
    )
    for i, z in enumerate((0.88, 1.27)):
        tower.visual(
            Box((0.018, 0.15, 0.18)),
            origin=Origin(xyz=(0.332 - 0.015 * i, 0.0, z)),
            material=dark_wood,
            name=f"front_window_{i}",
        )
        tower.visual(
            Box((0.020, 0.18, 0.026)),
            origin=Origin(xyz=(0.345 - 0.015 * i, 0.0, z + 0.095)),
            material=painted_wood,
            name=f"window_cap_{i}",
        )

    # Rotating cap: a turntable skirt, thatched cap roof, front nacelle,
    # stationary bearing, and a fixed guard ring/frame surrounding the rotor.
    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.34, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=painted_wood,
        name="cap_skirt",
    )
    cap_roof = ConeGeometry(radius=0.46, height=0.34, radial_segments=40).translate(
        0.0, 0.0, 0.27
    )
    cap.visual(
        mesh_from_geometry(cap_roof, "cap_roof"),
        material=thatch,
        name="cap_roof",
    )
    cap.visual(
        Box((0.52, 0.28, 0.22)),
        origin=Origin(xyz=(0.37, 0.0, 0.20)),
        material=painted_wood,
        name="front_nacelle",
    )
    cap.visual(
        Cylinder(radius=0.082, length=0.16),
        origin=Origin(xyz=(0.57, 0.0, HUB_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="bearing_barrel",
    )
    cap.visual(
        Box((0.07, 0.13, 0.11)),
        origin=Origin(xyz=(0.635, 0.17, HUB_Z)),
        material=painted_wood,
        name="guard_lug_0",
    )
    cap.visual(
        Box((0.07, 0.13, 0.11)),
        origin=Origin(xyz=(0.635, -0.17, HUB_Z)),
        material=painted_wood,
        name="guard_lug_1",
    )

    guard_ring = (
        TorusGeometry(radius=0.76, tube=0.014, radial_segments=18, tubular_segments=96)
        .rotate_y(math.pi / 2.0)
        .translate(HUB_X + 0.10, 0.0, HUB_Z)
    )
    cap.visual(
        mesh_from_geometry(guard_ring, "guard_ring"),
        material=guard_paint,
        name="guard_ring",
    )
    for sign, name in ((1.0, "guard_arm_0"), (-1.0, "guard_arm_1")):
        arm = wire_from_points(
            [
                (HUB_X - 0.02, sign * 0.18, HUB_Z),
                (HUB_X - 0.02, sign * 0.76, HUB_Z),
                (HUB_X + 0.10, sign * 0.76, HUB_Z),
            ],
            radius=0.012,
            radial_segments=12,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.012,
            corner_segments=6,
        )
        cap.visual(mesh_from_geometry(arm, name), material=guard_paint, name=name)

    # Rotating sail hub.  The hub frame is exactly on the wind shaft axis; all
    # geometry is forward of the stationary bearing so the contact is credible.
    hub = model.part("sail_hub")
    hub.visual(
        Cylinder(radius=0.083, length=0.10),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub_disk",
    )
    hub.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.112, 0.0, 0.0)),
        material=metal,
        name="hub_boss",
    )
    blade_specs = (
        ("blade_0", 0.0),
        ("blade_1", math.pi / 2.0),
        ("blade_2", math.pi),
        ("blade_3", 3.0 * math.pi / 2.0),
    )
    for blade_name, angle in blade_specs:
        hub.visual(
            mesh_from_geometry(_blade_lattice(angle), blade_name),
            material=blade_wood,
            name=blade_name,
        )
    hub.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=_blade_point(0.055, 0.69, 0.0, 0.0)),
        material=red_tip,
        name="blade_tip_marker",
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.5),
    )
    model.articulation(
        "cap_to_sail_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(HUB_X, 0.0, HUB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
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

    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    hub = object_model.get_part("sail_hub")
    yaw = object_model.get_articulation("tower_to_cap")
    spin = object_model.get_articulation("cap_to_sail_hub")

    ctx.check(
        "cap has continuous yaw joint",
        yaw.articulation_type == ArticulationType.CONTINUOUS and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "sail hub has continuous shaft joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        positive_elem="cap_skirt",
        negative_elem="top_turntable",
        max_gap=0.002,
        max_penetration=0.0,
        name="cap skirt sits on tower turntable",
    )
    ctx.expect_gap(
        hub,
        cap,
        axis="x",
        positive_elem="hub_disk",
        negative_elem="bearing_barrel",
        max_gap=0.003,
        max_penetration=0.0,
        name="rotating hub bears against fixed barrel",
    )
    ctx.expect_gap(
        cap,
        hub,
        axis="x",
        positive_elem="guard_ring",
        negative_elem="blade_0",
        min_gap=0.006,
        name="fixed guard ring clears rotating lattice",
    )
    ctx.expect_within(
        hub,
        cap,
        axes="yz",
        inner_elem="blade_tip_marker",
        outer_elem="guard_ring",
        margin=0.0,
        name="blade tip stays inside guard hoop outline",
    )

    rest_cap_pos = ctx.part_world_position(cap)
    with ctx.pose({yaw: 1.1}):
        yawed_cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap yaws about the fixed vertical tower axis",
        rest_cap_pos is not None
        and yawed_cap_pos is not None
        and abs(rest_cap_pos[0] - yawed_cap_pos[0]) < 1e-6
        and abs(rest_cap_pos[1] - yawed_cap_pos[1]) < 1e-6
        and abs(rest_cap_pos[2] - yawed_cap_pos[2]) < 1e-6,
        details=f"rest={rest_cap_pos}, yawed={yawed_cap_pos}",
    )

    rest_marker = ctx.part_element_world_aabb(hub, elem="blade_tip_marker")
    with ctx.pose({spin: math.pi / 2.0}):
        spun_marker = ctx.part_element_world_aabb(hub, elem="blade_tip_marker")
    if rest_marker is not None and spun_marker is not None:
        rest_center = tuple((rest_marker[0][i] + rest_marker[1][i]) * 0.5 for i in range(3))
        spun_center = tuple((spun_marker[0][i] + spun_marker[1][i]) * 0.5 for i in range(3))
    else:
        rest_center = None
        spun_center = None
    ctx.check(
        "hub spin rotates a blade tip around the shaft",
        rest_center is not None
        and spun_center is not None
        and spun_center[1] < rest_center[1] - 0.55
        and spun_center[2] < rest_center[2] - 0.55,
        details=f"rest={rest_center}, spun={spun_center}",
    )

    return ctx.report()


object_model = build_object_model()
