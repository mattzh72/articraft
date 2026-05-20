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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    dark_charcoal = model.material("dark_charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    soft_black = model.material("soft_black", rgba=(0.09, 0.10, 0.11, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.62, 0.66, 0.70, 1.0))

    base = model.part("base")
    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.28, 0.19, 0.040), 0.018),
        "fan_base_plate",
    )
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_charcoal,
        name="base_plate",
    )
    base.visual(
        Box((0.095, 0.105, 0.040)),
        origin=Origin(xyz=(-0.050, 0.0, 0.038)),
        material=graphite,
        name="rear_ballast",
    )
    base.visual(
        Box((0.052, 0.064, 0.056)),
        origin=Origin(xyz=(-0.032, 0.0, 0.066)),
        material=graphite,
        name="pedestal_core",
    )
    base.visual(
        Box((0.048, 0.164, 0.016)),
        origin=Origin(xyz=(-0.022, 0.0, 0.102)),
        material=graphite,
        name="yoke_bridge",
    )
    for side, y in (("left", 0.088), ("right", -0.088)):
        base.visual(
            Box((0.068, 0.012, 0.116)),
            origin=Origin(xyz=(-0.006, y, 0.082)),
            material=graphite,
            name=f"{side}_bracket_plate",
        )
        base.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.124), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_silver,
            name=f"{side}_pivot_collar",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.28, 0.20, 0.14)),
        mass=1.9,
        origin=Origin(xyz=(-0.020, 0.0, 0.070)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.013, length=0.164),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="pivot_barrel",
    )
    head.visual(
        Box((0.030, 0.060, 0.038)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=graphite,
        name="neck_block",
    )
    head.visual(
        Cylinder(radius=0.044, length=0.070),
        origin=Origin(xyz=(0.068, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="motor_housing",
    )
    head.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        material=graphite,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(0.105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_silver,
        name="axle_nose",
    )
    for ring_name, ring_x, major_radius, tube_radius in (
        ("rear_guard_ring", 0.106, 0.080, 0.0032),
        ("mid_guard_ring", 0.119, 0.079, 0.0027),
        ("front_guard_ring", 0.132, 0.081, 0.0032),
    ):
        ring_mesh = mesh_from_geometry(
            TorusGeometry(radius=major_radius, tube=tube_radius),
            ring_name,
        )
        head.visual(
            ring_mesh,
            origin=Origin(xyz=(ring_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_silver,
            name=ring_name,
        )

    for index, angle in enumerate((0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi)):
        y = math.cos(angle) * 0.079
        z = math.sin(angle) * 0.079
        _add_member(
            head,
            (0.106, y, z),
            (0.132, y, z),
            radius=0.0024,
            material=satin_silver,
            name=f"guard_depth_strut_{index}",
        )

    for index, angle in enumerate([k * math.pi / 3.0 for k in range(6)]):
        y0 = math.cos(angle) * 0.028
        z0 = math.sin(angle) * 0.028
        y1 = math.cos(angle) * 0.078
        z1 = math.sin(angle) * 0.078
        _add_member(
            head,
            (0.084, y0, z0),
            (0.106, y1, z1),
            radius=0.0030,
            material=satin_silver,
            name=f"guard_mount_strut_{index}",
        )

    head.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.134, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_silver,
        name="front_badge",
    )
    for index, angle in enumerate([k * math.pi / 4.0 for k in range(8)]):
        y = math.cos(angle) * 0.078
        z = math.sin(angle) * 0.078
        _add_member(
            head,
            (0.132, 0.0, 0.0),
            (0.132, y, z),
            radius=0.0019,
            material=satin_silver,
            name=f"front_spoke_{index}",
        )

    head.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.14)),
        mass=0.85,
        origin=Origin(xyz=(0.082, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_silver,
        name="hub",
    )
    rotor.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=satin_silver,
        name="hub_cap",
    )
    for blade_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            Box((0.006, 0.072, 0.026)),
            origin=Origin(
                xyz=(0.012, math.cos(angle) * 0.036, math.sin(angle) * 0.036),
                rpy=(angle, 0.26, 0.0),
            ),
            material=blade_gray,
            name=f"blade_{blade_index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Box((0.030, 0.150, 0.150)),
        mass=0.12,
        origin=Origin(),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=math.radians(-18.0),
            upper=math.radians(42.0),
        ),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.112, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=40.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head")

    ctx.expect_gap(
        head,
        base,
        axis="z",
        positive_elem="front_guard_ring",
        negative_elem="base_plate",
        min_gap=0.020,
        name="head clears the low base",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        outer_elem="front_guard_ring",
        margin=0.006,
        name="rotor remains within the front guard ring",
    )

    rest_aabb = ctx.part_element_world_aabb(head, elem="front_badge")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_aabb = ctx.part_element_world_aabb(head, elem="front_badge")

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_center_z = _aabb_center_z(rest_aabb)
    tilted_center_z = _aabb_center_z(tilted_aabb)
    ctx.check(
        "positive tilt raises the fan nose",
        rest_center_z is not None
        and tilted_center_z is not None
        and tilted_center_z > rest_center_z + 0.020,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
