from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


TAU = 2.0 * math.pi


def _paddle_geometry() -> MeshGeometry:
    """A broad, slightly swept and cambered ceiling-fan paddle blade."""

    x0 = 0.305
    length = 0.385
    n_edge = 12
    boundary: list[tuple[float, float, float]] = []

    def center_y(t: float) -> float:
        # Traditional blades have a subtle swept/scimitar outline.
        return 0.010 + 0.042 * t + 0.010 * math.sin(math.pi * t)

    def half_width(t: float) -> float:
        # Narrow at the root, wider at the rounded paddle tip.
        return 0.058 + 0.036 * (math.sin(0.5 * math.pi * t) ** 1.35)

    def center_z(t: float) -> float:
        # A small droop and pitch makes the blade read less like a flat board.
        return -0.023 - 0.013 * t

    # Lower/leading edge from root to tip.
    for i in range(n_edge + 1):
        t = i / n_edge
        x = x0 + length * t
        boundary.append((x, center_y(t) - half_width(t), center_z(t)))

    # Rounded paddle nose.
    tip_t = 1.0
    tip_x = x0 + length
    tip_y = center_y(tip_t)
    tip_w = half_width(tip_t)
    nose_r = 0.045
    for j in range(1, 9):
        a = -math.pi / 2.0 + j * math.pi / 8.0
        boundary.append((tip_x + nose_r * math.cos(a), tip_y + tip_w * math.sin(a), center_z(tip_t)))

    # Upper/trailing edge back to root.
    for i in range(n_edge, -1, -1):
        t = i / n_edge
        x = x0 + length * t
        boundary.append((x, center_y(t) + half_width(t), center_z(t)))

    thickness = 0.012
    geom = MeshGeometry()
    bottom: list[int] = []
    top: list[int] = []
    for x, y, z in boundary:
        bottom.append(geom.add_vertex(x, y, z - thickness * 0.50))
        top.append(geom.add_vertex(x, y, z + thickness * 0.50))

    center_x = sum(p[0] for p in boundary) / len(boundary)
    center_y_avg = sum(p[1] for p in boundary) / len(boundary)
    center_z_avg = sum(p[2] for p in boundary) / len(boundary)
    c_bot = geom.add_vertex(center_x, center_y_avg, center_z_avg - thickness * 0.50)
    c_top = geom.add_vertex(center_x, center_y_avg, center_z_avg + thickness * 0.50)

    n = len(boundary)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])
        geom.add_face(c_top, top[i], top[j])
        geom.add_face(c_bot, bottom[j], bottom[i])

    return geom


def _canopy_geometry() -> MeshGeometry:
    outer = (
        (0.045, 0.345),
        (0.070, 0.378),
        (0.113, 0.485),
        (0.107, 0.555),
    )
    inner = (
        (0.031, 0.347),
        (0.057, 0.382),
        (0.097, 0.484),
        (0.093, 0.542),
    )
    return LatheGeometry.from_shell_profiles(outer, inner, segments=64, start_cap="flat", end_cap="flat")


def _yawed_origin(radius: float, angle: float, z: float = 0.0) -> Origin:
    return Origin(xyz=(radius * math.cos(angle), radius * math.sin(angle), z), rpy=(0.0, 0.0, angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_blade_ceiling_fan")

    aged_bronze = model.material("aged_bronze", rgba=(0.38, 0.25, 0.13, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.18, 0.12, 0.07, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.64, 0.38, 0.17, 1.0))
    dark_wood = model.material("dark_wood_grain", rgba=(0.24, 0.13, 0.055, 1.0))
    screw_metal = model.material("slot_screw_heads", rgba=(0.08, 0.075, 0.065, 1.0))

    mount = model.part("mount")
    mount.visual(mesh_from_geometry(_canopy_geometry(), "bell_canopy"), material=aged_bronze, name="bell_canopy")
    mount.visual(Cylinder(radius=0.135, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.565)), material=aged_bronze, name="ceiling_plate")
    mount.visual(Sphere(radius=0.050), origin=Origin(xyz=(0.0, 0.0, 0.332)), material=dark_bronze, name="swivel_ball")
    mount.visual(mesh_from_geometry(TorusGeometry(radius=0.055, tube=0.009, radial_segments=20, tubular_segments=56), "socket_ring"), origin=Origin(xyz=(0.0, 0.0, 0.346)), material=aged_bronze, name="socket_ring")
    mount.visual(Cylinder(radius=0.020, length=0.235), origin=Origin(xyz=(0.0, 0.0, 0.205)), material=dark_bronze, name="downrod")

    mount.visual(Cylinder(radius=0.184, length=0.205), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=aged_bronze, name="motor_drum")
    mount.visual(mesh_from_geometry(TorusGeometry(radius=0.177, tube=0.008, radial_segments=18, tubular_segments=72), "upper_motor_bead"), origin=Origin(xyz=(0.0, 0.0, 0.104)), material=dark_bronze, name="upper_motor_bead")
    mount.visual(mesh_from_geometry(TorusGeometry(radius=0.177, tube=0.008, radial_segments=18, tubular_segments=72), "lower_motor_bead"), origin=Origin(xyz=(0.0, 0.0, -0.104)), material=dark_bronze, name="lower_motor_bead")
    mount.visual(Cylinder(radius=0.090, length=0.040), origin=Origin(xyz=(0.0, 0.0, -0.108)), material=dark_bronze, name="bearing_collar")
    mount.visual(Cylinder(radius=0.036, length=0.076), origin=Origin(xyz=(0.0, 0.0, -0.165)), material=dark_bronze, name="axle_stub")

    hub = model.part("hub")
    hub.visual(Cylinder(radius=0.128, length=0.070), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=aged_bronze, name="hub_shell")
    hub.visual(Cylinder(radius=0.083, length=0.030), origin=Origin(xyz=(0.0, 0.0, -0.048)), material=dark_bronze, name="lower_cap")
    hub.visual(mesh_from_geometry(TorusGeometry(radius=0.115, tube=0.006, radial_segments=16, tubular_segments=60), "hub_bead"), origin=Origin(xyz=(0.0, 0.0, 0.034)), material=dark_bronze, name="hub_bead")

    for i in range(5):
        angle = TAU * i / 5.0
        hub.visual(
            Box((0.075, 0.105, 0.040)),
            origin=_yawed_origin(0.145, angle, -0.004),
            material=aged_bronze,
            name=f"socket_{i}",
        )
        for j, side in enumerate((-1.0, 1.0)):
            local_x = 0.188
            local_y = side * 0.056
            hub.visual(
                Box((0.030, 0.022, 0.036)),
                origin=Origin(
                    xyz=(
                        local_x * math.cos(angle) - local_y * math.sin(angle),
                        local_x * math.sin(angle) + local_y * math.cos(angle),
                        -0.004,
                    ),
                    rpy=(0.0, 0.0, angle),
                ),
                material=aged_bronze,
                name=f"socket_ear_{i}_{j}",
            )
        hub.visual(
            Cylinder(radius=0.010, length=0.132),
            origin=Origin(
                xyz=(0.206 * math.cos(angle), 0.206 * math.sin(angle), -0.004),
                rpy=(math.pi / 2.0, 0.0, angle),
            ),
            material=dark_bronze,
            name=f"socket_pin_{i}",
        )

    axle = model.articulation(
        "axle",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=hub,
        origin=Origin(xyz=(0.0, 0.0, -0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=18.0),
    )
    axle.meta["description"] = "central motor axle for the rotating blade hub"

    for i in range(5):
        angle = TAU * i / 5.0
        blade = model.part(f"blade_{i}")
        blade.visual(
            Cylinder(radius=0.018, length=0.084),
            origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_bronze,
            name="root_barrel",
        )
        blade.visual(Box((0.205, 0.026, 0.014)), origin=Origin(xyz=(0.138, 0.0, -0.010)), material=dark_bronze, name="iron_arm")
        blade.visual(Box((0.048, 0.104, 0.013)), origin=Origin(xyz=(0.238, 0.0, -0.018)), material=dark_bronze, name="yoke_bridge")
        blade.visual(Box((0.132, 0.020, 0.010)), origin=Origin(xyz=(0.292, 0.047, -0.019)), material=dark_bronze, name="fork_arm_0")
        blade.visual(Box((0.132, 0.020, 0.010)), origin=Origin(xyz=(0.292, -0.047, -0.019)), material=dark_bronze, name="fork_arm_1")
        blade.visual(mesh_from_geometry(_paddle_geometry(), f"paddle_{i}"), material=warm_wood, name="paddle")
        blade.visual(Box((0.355, 0.010, 0.003)), origin=Origin(xyz=(0.505, 0.010, -0.017)), material=dark_wood, name="grain_line_0")
        blade.visual(Box((0.315, 0.008, 0.003)), origin=Origin(xyz=(0.530, 0.064, -0.019)), material=dark_wood, name="grain_line_1")
        for j, y in enumerate((-0.048, 0.048)):
            blade.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(0.342, y, -0.012)),
                material=screw_metal,
                name=f"screw_{j}",
            )

        model.articulation(
            f"blade_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=blade,
            origin=Origin(xyz=(0.188 * math.cos(angle), 0.188 * math.sin(angle), -0.004), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.24, upper=0.24),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    hub = object_model.get_part("hub")
    axle = object_model.get_articulation("axle")

    ctx.allow_overlap(
        mount,
        hub,
        elem_a="axle_stub",
        elem_b="hub_shell",
        reason="The central axle stub is intentionally captured inside the rotating hub shell.",
    )
    ctx.expect_within(
        mount,
        hub,
        axes="xy",
        inner_elem="axle_stub",
        outer_elem="hub_shell",
        margin=0.0,
        name="axle stub is centered within hub shell",
    )
    ctx.expect_overlap(
        mount,
        hub,
        axes="z",
        elem_a="axle_stub",
        elem_b="hub_shell",
        min_overlap=0.055,
        name="hub shell retains the axle stub",
    )

    blade_parts = [object_model.get_part(f"blade_{i}") for i in range(5)]
    blade_joints = [object_model.get_articulation(f"blade_pivot_{i}") for i in range(5)]
    for i, blade in enumerate(blade_parts):
        ctx.allow_overlap(
            hub,
            blade,
            elem_a=f"socket_pin_{i}",
            elem_b="root_barrel",
            reason="A dark hinge pin is intentionally captured inside the blade iron barrel.",
        )
        ctx.expect_overlap(
            hub,
            blade,
            axes="xyz",
            elem_a=f"socket_pin_{i}",
            elem_b="root_barrel",
            min_overlap=0.018,
            name=f"blade {i} hinge pin passes through its barrel",
        )

    ctx.check("five articulated blade assemblies", len(blade_parts) == 5 and len(blade_joints) == 5)
    ctx.check(
        "hub spins on a continuous central axle",
        axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"axle type={axle.articulation_type}",
    )

    with ctx.pose({axle: 0.0}):
        p0 = ctx.part_world_position(blade_parts[0])
    with ctx.pose({axle: TAU / 10.0}):
        p1 = ctx.part_world_position(blade_parts[0])
    ctx.check(
        "central axle rotates the blade circle",
        p0 is not None and p1 is not None and abs(p1[1] - p0[1]) > 0.05,
        details=f"rest={p0}, spun={p1}",
    )

    pivot = blade_joints[0]
    blade = blade_parts[0]
    with ctx.pose({pivot: 0.0}):
        rest_aabb = ctx.part_world_aabb(blade)
    with ctx.pose({pivot: 0.24}):
        pitched_aabb = ctx.part_world_aabb(blade)
    ctx.check(
        "blade iron pivots about its hub joint",
        rest_aabb is not None
        and pitched_aabb is not None
        and abs(pitched_aabb[0][2] - rest_aabb[0][2]) > 0.025,
        details=f"rest_aabb={rest_aabb}, pitched_aabb={pitched_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
