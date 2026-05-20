from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material: Material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_spotlight_shell() -> MeshGeometry:
    """Thin legacy can with raised lips, authored around +Y."""
    outer = [
        (0.205, -0.285),
        (0.218, -0.260),
        (0.225, -0.205),
        (0.222, 0.215),
        (0.235, 0.255),
        (0.238, 0.285),
    ]
    inner = [
        (0.168, -0.252),
        (0.181, -0.220),
        (0.188, 0.205),
        (0.190, 0.250),
    ]
    geom = LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )
    geom.rotate_x(-math.pi / 2.0)
    return geom


def _front_bezel_mesh() -> MeshGeometry:
    geom = TorusGeometry(radius=0.205, tube=0.018, radial_segments=20, tubular_segments=64)
    geom.rotate_x(-math.pi / 2.0)
    return geom


def _friction_washer_mesh() -> MeshGeometry:
    geom = TorusGeometry(radius=0.040, tube=0.010, radial_segments=16, tubular_segments=48)
    geom.rotate_y(math.pi / 2.0)
    return geom


def _add_bolt_grid(
    part,
    *,
    xs: tuple[float, ...],
    ys: tuple[float, ...],
    z: float,
    radius: float,
    height: float,
    material: Material,
    name_prefix: str,
) -> None:
    for ix, x in enumerate(xs):
        for iy, y in enumerate(ys):
            part.visual(
                Cylinder(radius=radius, length=height),
                origin=Origin(xyz=(x, y, z)),
                material=material,
                name=f"{name_prefix}_{ix}_{iy}",
            )


def _add_lock_knob(
    part,
    *,
    x: float,
    material: Material,
    accent: Material,
    name_prefix: str,
) -> None:
    sign = 1.0 if x >= 0.0 else -1.0
    part.visual(
        Cylinder(radius=0.058, length=0.040),
        origin=Origin(xyz=(x, 0.0, 1.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=f"{name_prefix}_wheel",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.100),
        origin=Origin(xyz=(x - sign * 0.065, 0.0, 1.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name=f"{name_prefix}_boss",
    )
    for index in range(12):
        angle = index * math.tau / 12.0
        y = math.cos(angle) * 0.054
        z = 1.120 + math.sin(angle) * 0.054
        part.visual(
            Box((0.045, 0.018, 0.026)),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, angle)),
            material=material,
            name=f"{name_prefix}_grip_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_studio_spotlight")

    black = model.material("black_wrinkle_paint", rgba=(0.030, 0.032, 0.034, 1.0))
    dark = model.material("oiled_dark_steel", rgba=(0.15, 0.15, 0.14, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    warm_steel = model.material("aged_galvanized_steel", rgba=(0.43, 0.41, 0.36, 1.0))
    brass = model.material("aged_brass", rgba=(0.78, 0.55, 0.24, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("fresnel_glass", rgba=(0.70, 0.86, 0.95, 0.36))
    red = model.material("red_service_tab", rgba=(0.70, 0.08, 0.06, 1.0))

    shell_mesh = mesh_from_geometry(_build_spotlight_shell(), "spotlight_hollow_can")
    bezel_mesh = mesh_from_geometry(_front_bezel_mesh(), "spotlight_front_bezel")
    washer_mesh = mesh_from_geometry(_friction_washer_mesh(), "spotlight_friction_washer")
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.700, 0.195, 0.505),
            span_width=0.520,
            trunnion_diameter=0.070,
            trunnion_center_z=0.360,
            base_thickness=0.055,
            corner_radius=0.018,
            center=False,
        ),
        "spotlight_trunnion_yoke",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.280, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=black,
        name="weighted_base",
    )
    stand.visual(
        Cylinder(radius=0.225, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=warm_steel,
        name="base_trim",
    )
    stand.visual(
        Cylinder(radius=0.052, length=0.735),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=dark,
        name="center_post",
    )
    stand.visual(
        Cylinder(radius=0.075, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        material=warm_steel,
        name="post_collar",
    )
    stand.visual(
        Box((0.330, 0.185, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        material=dark,
        name="yoke_adapter",
    )
    stand.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.760)),
        material=black,
        name="trunnion_yoke",
    )
    stand.visual(
        Box((0.660, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, -0.112, 0.835)),
        material=warm_steel,
        name="rear_bridge",
    )
    stand.visual(
        Box((0.060, 0.033, 0.335)),
        origin=Origin(xyz=(-0.315, -0.112, 0.985)),
        material=warm_steel,
        name="side_reinforcement_0",
    )
    stand.visual(
        Box((0.060, 0.033, 0.335)),
        origin=Origin(xyz=(0.315, -0.112, 0.985)),
        material=warm_steel,
        name="side_reinforcement_1",
    )
    _add_member(stand, (-0.160, -0.065, 0.785), (-0.285, -0.065, 1.035), 0.013, warm_steel, name="gusset_0")
    _add_member(stand, (0.160, -0.065, 0.785), (0.285, -0.065, 1.035), 0.013, warm_steel, name="gusset_1")
    _add_member(stand, (-0.160, 0.065, 0.785), (-0.285, 0.065, 1.035), 0.013, warm_steel, name="gusset_2")
    _add_member(stand, (0.160, 0.065, 0.785), (0.285, 0.065, 1.035), 0.013, warm_steel, name="gusset_3")
    _add_bolt_grid(
        stand,
        xs=(-0.115, 0.115),
        ys=(-0.062, 0.062),
        z=0.812,
        radius=0.011,
        height=0.015,
        material=brass,
        name_prefix="adapter_bolt",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        stand.visual(
            Cylinder(radius=0.015, length=0.018),
            origin=Origin(xyz=(math.cos(angle) * 0.185, math.sin(angle) * 0.185, 0.078)),
            material=brass,
            name=f"base_bolt_{index}",
        )
    stand.visual(
        washer_mesh,
        origin=Origin(xyz=(-0.360, 0.0, 1.120)),
        material=steel,
        name="friction_bushing_0",
    )
    stand.visual(
        washer_mesh,
        origin=Origin(xyz=(0.360, 0.0, 1.120)),
        material=steel,
        name="friction_bushing_1",
    )
    for index, x in enumerate((-0.292, 0.292)):
        stand.visual(
            Box((0.075, 0.042, 0.012)),
            origin=Origin(xyz=(x, 0.0, 1.084)),
            material=rubber,
            name=f"bearing_pad_{index}",
        )
    _add_lock_knob(stand, x=-0.445, material=rubber, accent=steel, name_prefix="lock_knob_0")
    _add_lock_knob(stand, x=0.445, material=rubber, accent=steel, name_prefix="lock_knob_1")
    stand.inertial = Inertial.from_geometry(
        Box((0.66, 0.56, 1.27)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.63)),
    )

    spotlight = model.part("spotlight")
    spotlight.visual(shell_mesh, material=black, name="can_shell")
    spotlight.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, 0.276, 0.0)),
        material=warm_steel,
        name="front_bezel",
    )
    spotlight.visual(
        Cylinder(radius=0.188, length=0.014),
        origin=Origin(xyz=(0.0, 0.270, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="fresnel_lens",
    )
    spotlight.visual(
        Cylinder(radius=0.188, length=0.026),
        origin=Origin(xyz=(0.0, -0.279, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="rear_cap",
    )
    spotlight.visual(
        Box((0.235, 0.016, 0.092)),
        origin=Origin(xyz=(0.0, -0.286, 0.040)),
        material=warm_steel,
        name="rear_service_plate",
    )
    spotlight.visual(
        Cylinder(radius=0.040, length=0.095),
        origin=Origin(xyz=(0.0, -0.335, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="cable_gland",
    )
    spotlight.visual(
        Cylinder(radius=0.030, length=0.650),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="trunnion_shaft",
    )
    for side, x in enumerate((-0.214, 0.214)):
        spotlight.visual(
            Cylinder(radius=0.075, length=0.030),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_steel,
            name=f"side_adapter_{side}",
        )
        spotlight.visual(
            Box((0.020, 0.145, 0.040)),
            origin=Origin(xyz=(x, 0.000, 0.078)),
            material=warm_steel,
            name=f"adapter_rib_top_{side}",
        )
        spotlight.visual(
            Box((0.020, 0.145, 0.040)),
            origin=Origin(xyz=(x, 0.000, -0.078)),
            material=warm_steel,
            name=f"adapter_rib_bottom_{side}",
        )
        for angle_index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
            spotlight.visual(
                Cylinder(radius=0.008, length=0.012),
                origin=Origin(
                    xyz=(x + (0.012 if x > 0 else -0.012), math.cos(angle) * 0.055, math.sin(angle) * 0.055),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=brass,
                name=f"adapter_bolt_{side}_{angle_index}",
            )
    for index, x in enumerate((-0.118, -0.060, 0.000, 0.060, 0.118)):
        spotlight.visual(
            Box((0.028, 0.020, 0.090)),
            origin=Origin(xyz=(x, -0.286, 0.108)),
            material=dark,
            name=f"cooling_fin_{index}",
        )
    for index, x in enumerate((-0.082, 0.0, 0.082)):
        spotlight.visual(
            Box((0.045, 0.008, 0.010)),
            origin=Origin(xyz=(x, -0.276, 0.052 + index * 0.018)),
            material=black,
            name=f"rear_vent_slot_{index}",
        )
    spotlight.inertial = Inertial.from_geometry(
        Cylinder(radius=0.240, length=0.620),
        mass=4.2,
        origin=Origin(),
    )

    service_hatch = model.part("service_hatch")
    service_hatch.visual(
        Box((0.250, 0.165, 0.014)),
        origin=Origin(xyz=(0.0, 0.105, 0.007)),
        material=warm_steel,
        name="hatch_panel",
    )
    service_hatch.visual(
        Cylinder(radius=0.015, length=0.126),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_knuckle",
    )
    service_hatch.visual(
        Box((0.040, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.176, 0.018)),
        material=red,
        name="latch_tab",
    )
    for index, x in enumerate((-0.092, 0.092)):
        service_hatch.visual(
            Cylinder(radius=0.008, length=0.009),
            origin=Origin(xyz=(x, 0.046, 0.0175)),
            material=brass,
            name=f"hatch_bolt_{index}",
        )
    service_hatch.inertial = Inertial.from_geometry(
        Box((0.25, 0.20, 0.04)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.10, 0.01)),
    )

    # Stationary hinge leaves and latch catch mounted on the can itself.
    for index, x in enumerate((-0.095, 0.095)):
        spotlight.visual(
            Box((0.070, 0.028, 0.076)),
            origin=Origin(xyz=(x, -0.112, 0.232)),
            material=warm_steel,
            name=f"hatch_hinge_leaf_{index}",
        )
        spotlight.visual(
            Cylinder(radius=0.015, length=0.064),
            origin=Origin(xyz=(x, -0.112, 0.260), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"hatch_hinge_socket_{index}",
        )
    spotlight.visual(
        Box((0.062, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, 0.087, 0.244)),
        material=dark,
        name="hatch_latch_catch",
    )

    tilt = model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=spotlight,
        origin=Origin(xyz=(0.0, 0.0, 1.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=0.8, lower=-0.65, upper=0.85),
    )
    hatch_hinge = model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=spotlight,
        child=service_hatch,
        origin=Origin(xyz=(0.0, -0.112, 0.260)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.4, lower=0.0, upper=1.25),
    )
    tilt.meta["description"] = "Friction-locked yoke tilt carried on the side trunnions."
    hatch_hinge.meta["description"] = "Small service hatch opens upward for lamp access."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    spotlight = object_model.get_part("spotlight")
    hatch = object_model.get_part("service_hatch")
    tilt = object_model.get_articulation("tilt")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.expect_origin_gap(
        spotlight,
        stand,
        axis="z",
        min_gap=1.10,
        max_gap=1.14,
        name="spotlight pivots at yoke height",
    )
    ctx.expect_overlap(
        spotlight,
        stand,
        axes="x",
        min_overlap=0.55,
        elem_a="trunnion_shaft",
        elem_b="trunnion_yoke",
        name="trunnion shaft spans both yoke cheeks",
    )
    ctx.expect_overlap(
        spotlight,
        stand,
        axes="yz",
        min_overlap=0.045,
        elem_a="trunnion_shaft",
        elem_b="friction_bushing_0",
        name="side friction bushing captures the shaft",
    )

    closed_aabb = ctx.part_world_aabb(hatch)
    with ctx.pose({tilt: 0.55}):
        tilted_aabb = ctx.part_world_aabb(spotlight)
    with ctx.pose({hatch_hinge: 1.0}):
        open_aabb = ctx.part_world_aabb(hatch)

    ctx.check(
        "tilt raises the front lens",
        tilted_aabb is not None and tilted_aabb[1][2] > 1.31,
        details=f"tilted_aabb={tilted_aabb}",
    )
    ctx.check(
        "service hatch opens upward",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.05,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
