from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _midpoint(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a, b) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _ring_band(outer_radius: float, inner_radius: float, width: float) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=width, radial_segments=72)
    inner = CylinderGeometry(radius=inner_radius, height=width + 0.006, radial_segments=72)
    return boolean_difference(outer, inner)


def _add_radial_box(
    part,
    *,
    size,
    y: float,
    radial: float,
    tangent: float = 0.0,
    angle: float,
    material,
    name: str | None = None,
) -> None:
    ca = math.cos(angle)
    sa = math.sin(angle)
    x_world = tangent * ca + radial * sa
    z_world = -tangent * sa + radial * ca
    part.visual(
        Box(size),
        origin=Origin(xyz=(x_world, y, z_world), rpy=(0.0, angle, 0.0)),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    weathered_wood = model.material("weathered_wood", rgba=(0.48, 0.31, 0.16, 1.0))
    dark_wood = model.material("dark_wet_wood", rgba=(0.26, 0.16, 0.08, 1.0))
    iron = model.material("dark_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.48, 0.50, 0.51, 1.0))
    water = model.material("water_in_trough", rgba=(0.22, 0.45, 0.72, 0.70))

    support = model.part("support_frame")

    # Ground sills and two timber support bents, tied into one rigid frame.
    support.visual(
        Box((2.25, 1.28, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_wood,
        name="stone_base",
    )
    for x in (-0.96, 0.96):
        support.visual(
            Box((0.15, 1.25, 0.12)),
            origin=Origin(xyz=(x, 0.0, 0.12)),
            material=weathered_wood,
            name=f"side_sill_{0 if x < 0 else 1}",
        )
    for y in (-0.43, 0.43):
        support.visual(
            Box((2.10, 0.13, 0.12)),
            origin=Origin(xyz=(0.0, y, 0.18)),
            material=weathered_wood,
            name=f"bent_sill_{0 if y < 0 else 1}",
        )
        support.visual(
            Box((1.26, 0.13, 0.10)),
            origin=Origin(xyz=(0.0, y, 1.42)),
            material=weathered_wood,
            name=f"bent_head_{0 if y < 0 else 1}",
        )
        _add_member(support, (-0.94, y, 0.20), (-0.22, y, 1.40), 0.035, weathered_wood)
        _add_member(support, (0.94, y, 0.20), (0.22, y, 1.40), 0.035, weathered_wood)
        _add_member(support, (-0.94, y, 0.20), (-0.94, y, 1.08), 0.030, weathered_wood)
        _add_member(support, (0.94, y, 0.20), (0.94, y, 1.08), 0.030, weathered_wood)
        _add_member(support, (-0.82, y, 0.72), (0.82, y, 0.72), 0.026, weathered_wood)

        # Split pillow-block bearing: four jaws leave a visible clearance hole for the axle.
        support.visual(
            Box((0.30, 0.13, 0.07)),
            origin=Origin(xyz=(0.0, y, 1.219)),
            material=iron,
            name=f"lower_bearing_{0 if y < 0 else 1}",
        )
        support.visual(
            Box((0.30, 0.13, 0.07)),
            origin=Origin(xyz=(0.0, y, 1.381)),
            material=iron,
            name=f"upper_bearing_{0 if y < 0 else 1}",
        )
        support.visual(
            Box((0.055, 0.13, 0.22)),
            origin=Origin(xyz=(-0.0745, y, 1.30)),
            material=iron,
            name=f"bearing_cheek_{0 if y < 0 else 2}",
        )
        support.visual(
            Box((0.055, 0.13, 0.22)),
            origin=Origin(xyz=(0.0745, y, 1.30)),
            material=iron,
            name=f"bearing_cheek_{1 if y < 0 else 3}",
        )

    # Overshot feed trough, raised above the top bucket path and carried by the rear posts.
    for y in (-0.43, 0.43):
        _add_member(support, (-0.96, y, 0.18), (-0.96, y, 2.26), 0.026, weathered_wood)
        _add_member(support, (-0.96, y, 2.22), (0.14, y, 2.22), 0.024, weathered_wood)
    _add_member(support, (-0.78, -0.43, 2.235), (-0.78, 0.43, 2.235), 0.022, weathered_wood)
    _add_member(support, (0.02, -0.43, 2.235), (0.02, 0.43, 2.235), 0.022, weathered_wood)
    support.visual(
        Box((1.12, 0.28, 0.04)),
        origin=Origin(xyz=(-0.40, 0.0, 2.255)),
        material=weathered_wood,
        name="feed_trough_floor",
    )
    support.visual(
        Box((1.12, 0.035, 0.13)),
        origin=Origin(xyz=(-0.40, -0.155, 2.315)),
        material=weathered_wood,
        name="feed_trough_side_0",
    )
    support.visual(
        Box((1.12, 0.035, 0.13)),
        origin=Origin(xyz=(-0.40, 0.155, 2.315)),
        material=weathered_wood,
        name="feed_trough_side_1",
    )
    support.visual(
        Box((0.72, 0.22, 0.012)),
        origin=Origin(xyz=(-0.22, 0.0, 2.281)),
        material=water,
        name="trough_water",
    )

    rotor = model.part("wheel_axle")

    # A continuous shaft links the main wheel to the outboard belt pulley.
    rotor.visual(
        Cylinder(radius=0.046, length=1.48),
        origin=Origin(xyz=(0.0, 0.10, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="axle",
    )
    rotor.visual(
        Cylinder(radius=0.145, length=0.38),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_wood,
        name="wheel_hub",
    )

    side_ring_mesh = mesh_from_geometry(_ring_band(0.82, 0.72, 0.045).rotate_x(math.pi / 2.0), "wood_side_ring")
    for y, suffix in ((-0.155, "0"), (0.155, "1")):
        rotor.visual(
            side_ring_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=weathered_wood,
            name=f"side_ring_{suffix}",
        )

    # Two side planes of spokes read as a traditional built-up timber wheel.
    for side_y in (-0.125, 0.125):
        for index in range(8):
            angle = index * math.tau / 8.0
            _add_radial_box(
                rotor,
                size=(0.052, 0.050, 0.68),
                y=side_y,
                radial=0.405,
                angle=angle,
                material=weathered_wood,
                name=f"spoke_{index}_{0 if side_y < 0 else 1}",
            )

    # Bucketed overshot rim: cupped timber pockets span between the side rings.
    for index in range(16):
        angle = index * math.tau / 16.0
        _add_radial_box(
            rotor,
            size=(0.25, 0.36, 0.030),
            y=0.0,
            radial=0.744,
            angle=angle,
            material=dark_wood,
            name=f"bucket_floor_{index}",
        )
        _add_radial_box(
            rotor,
            size=(0.038, 0.36, 0.125),
            y=0.0,
            radial=0.792,
            tangent=-0.105,
            angle=angle,
            material=dark_wood,
            name=f"bucket_back_{index}",
        )
        _add_radial_box(
            rotor,
            size=(0.034, 0.36, 0.110),
            y=0.0,
            radial=0.805,
            tangent=0.105,
            angle=angle,
            material=dark_wood,
            name=f"bucket_lip_{index}",
        )

    # Outboard belt pulley on the positive-Y axle end.  Shoulder and snap-clip keep it seated.
    rotor.visual(
        Cylinder(radius=0.074, length=0.035),
        origin=Origin(xyz=(0.0, 0.535, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="shaft_shoulder",
    )
    rotor.visual(
        Cylinder(radius=0.175, length=0.090),
        origin=Origin(xyz=(0.0, 0.625, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="drive_pulley",
    )
    rotor.visual(
        Cylinder(radius=0.215, length=0.025),
        origin=Origin(xyz=(0.0, 0.565, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="pulley_flange_0",
    )
    rotor.visual(
        Cylinder(radius=0.215, length=0.025),
        origin=Origin(xyz=(0.0, 0.675, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="pulley_flange_1",
    )
    rotor.visual(
        Cylinder(radius=0.086, length=0.026),
        origin=Origin(xyz=(0.0, 0.704, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="retaining_clip",
    )

    model.articulation(
        "axle_rotation",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rotor = object_model.get_part("wheel_axle")
    joint = object_model.get_articulation("axle_rotation")

    ctx.check(
        "main wheel has continuous axle rotation",
        joint.articulation_type == ArticulationType.CONTINUOUS and joint.child == "wheel_axle",
        details=f"type={joint.articulation_type}, child={joint.child}",
    )

    # The pulley, shaft shoulder, and retaining clip are authored on the same moving link
    # as the bucketed wheel, so they rotate with the axle rather than lagging on a second joint.
    axle_aabb = ctx.part_element_world_aabb(rotor, elem="axle")
    pulley_aabb = ctx.part_element_world_aabb(rotor, elem="drive_pulley")
    clip_aabb = ctx.part_element_world_aabb(rotor, elem="retaining_clip")
    flange_aabb = ctx.part_element_world_aabb(rotor, elem="pulley_flange_1")
    ctx.check(
        "outboard pulley is concentric with axle",
        axle_aabb is not None
        and pulley_aabb is not None
        and abs(_aabb_center(axle_aabb)[0] - _aabb_center(pulley_aabb)[0]) < 0.003
        and abs(_aabb_center(axle_aabb)[2] - _aabb_center(pulley_aabb)[2]) < 0.003,
        details=f"axle={axle_aabb}, pulley={pulley_aabb}",
    )
    ctx.check(
        "retaining clip sits outboard of pulley",
        clip_aabb is not None
        and flange_aabb is not None
        and clip_aabb[0][1] >= flange_aabb[1][1] - 0.006,
        details=f"clip={clip_aabb}, flange={flange_aabb}",
    )

    bucket_rest = ctx.part_element_world_aabb(rotor, elem="bucket_lip_0")
    with ctx.pose({joint: math.pi / 2.0}):
        bucket_quarter = ctx.part_element_world_aabb(rotor, elem="bucket_lip_0")
    ctx.check(
        "bucketed wheel turns about horizontal axle",
        bucket_rest is not None
        and bucket_quarter is not None
        and abs(_aabb_center(bucket_rest)[0] - _aabb_center(bucket_quarter)[0]) > 0.40
        and abs(_aabb_center(bucket_rest)[2] - _aabb_center(bucket_quarter)[2]) > 0.40,
        details=f"rest={bucket_rest}, quarter={bucket_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
