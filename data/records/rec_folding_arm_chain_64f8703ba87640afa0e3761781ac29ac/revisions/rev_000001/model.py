from __future__ import annotations

from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 40, dx: float = 0.0, dz: float = 0.0) -> list[tuple[float, float]]:
    return [
        (dx + radius * cos(tau * i / segments), dz + radius * sin(tau * i / segments))
        for i in range(segments)
    ]


def _capsule_profile(length: float, radius: float, *, segments: int = 18) -> list[tuple[float, float]]:
    """Closed 2-D outline in the link's X/Z plane, with joint centers at 0 and length."""
    pts: list[tuple[float, float]] = []
    for i in range(segments + 1):
        a = pi / 2.0 - pi * i / segments
        pts.append((length + radius * cos(a), radius * sin(a)))
    for i in range(segments + 1):
        a = -pi / 2.0 - pi * i / segments
        pts.append((radius * cos(a), radius * sin(a)))
    return pts


def _flat_link_geometry(length: float, *, radius: float, thickness: float, hole_radius: float):
    profile = _capsule_profile(length, radius)
    holes = [
        _circle_profile(hole_radius, dx=0.0, dz=0.0),
        _circle_profile(hole_radius, dx=length, dz=0.0),
    ]
    return ExtrudeWithHolesGeometry(profile, holes, thickness, center=True).rotate_x(pi / 2.0)


def _ring_geometry(*, outer_radius: float, inner_radius: float, thickness: float):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=48),
        [_circle_profile(inner_radius, segments=32)],
        thickness,
        center=True,
    ).rotate_x(pi / 2.0)


def _pin_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def _add_link_visuals(
    part,
    *,
    prefix: str,
    length: float,
    layer_y: float,
    plate_material,
    boss_material,
    radius: float = 0.028,
    thickness: float = 0.012,
    hole_radius: float = 0.013,
) -> None:
    part.visual(
        mesh_from_geometry(
            _flat_link_geometry(length, radius=radius, thickness=thickness, hole_radius=hole_radius),
            f"{prefix}_plate",
        ),
        origin=Origin(xyz=(0.0, layer_y, 0.0)),
        material=plate_material,
        name="plate",
    )
    ring = mesh_from_geometry(
        _ring_geometry(outer_radius=0.023, inner_radius=hole_radius, thickness=0.004),
        f"{prefix}_boss_ring",
    )
    for x, name in ((0.0, "boss_0"), (length, "boss_1")):
        part.visual(
            ring,
            origin=Origin(xyz=(x, layer_y - 0.006, 0.0)),
            material=boss_material,
            name=f"{name}_inner",
        )
        part.visual(
            ring,
            origin=Origin(xyz=(x, layer_y + 0.006, 0.0)),
            material=boss_material,
            name=f"{name}_outer",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_folding_arm_chain")

    painted_steel = model.material("painted_steel", rgba=(0.34, 0.38, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    mount_paint = model.material("mount_paint", rgba=(0.16, 0.19, 0.21, 1.0))

    # Dimensions are small machine-link proportions, roughly a compact folding
    # support arm bolted to a side plate.
    link_0_len = 0.260
    link_1_len = 0.220
    link_2_len = 0.185

    fixed_cheek = model.part("fixed_cheek")
    fixed_cheek.visual(
        Box((0.105, 0.082, 0.170)),
        origin=Origin(xyz=(-0.082, 0.0, 0.0)),
        material=mount_paint,
        name="mount_plate",
    )
    fixed_cheek.visual(
        Box((0.080, 0.014, 0.052)),
        origin=Origin(xyz=(-0.035, 0.026, 0.0)),
        material=mount_paint,
        name="cheek_lug_0",
    )
    fixed_cheek.visual(
        Box((0.080, 0.014, 0.052)),
        origin=Origin(xyz=(-0.035, -0.026, 0.0)),
        material=mount_paint,
        name="cheek_lug_1",
    )
    fixed_cheek.visual(
        Cylinder(radius=0.033, length=0.014),
        origin=_pin_origin(0.0, 0.026),
        material=mount_paint,
        name="cheek_boss_0",
    )
    fixed_cheek.visual(
        Cylinder(radius=0.033, length=0.014),
        origin=_pin_origin(0.0, -0.026),
        material=mount_paint,
        name="cheek_boss_1",
    )
    fixed_cheek.visual(
        Cylinder(radius=0.010, length=0.088),
        origin=_pin_origin(0.0),
        material=pin_steel,
        name="first_pin",
    )
    fixed_cheek.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_pin_origin(0.0, 0.043),
        material=pin_steel,
        name="first_pin_head_0",
    )
    fixed_cheek.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_pin_origin(0.0, -0.043),
        material=pin_steel,
        name="first_pin_head_1",
    )
    # Low-profile mounting screw heads on the fixed cheek, connected to the
    # plate visually as proud caps rather than free decorative dots.
    for z, name in ((0.055, "mount_screw_0"), (-0.055, "mount_screw_1")):
        fixed_cheek.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(-0.104, 0.0, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=pin_steel,
            name=name,
        )

    link_0 = model.part("link_0")
    _add_link_visuals(
        link_0,
        prefix="link_0",
        length=link_0_len,
        layer_y=0.0,
        plate_material=painted_steel,
        boss_material=dark_steel,
    )
    link_0.visual(
        Cylinder(radius=0.010, length=0.058),
        origin=_pin_origin(link_0_len, 0.016),
        material=pin_steel,
        name="second_pin",
    )
    link_0.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_pin_origin(link_0_len, -0.013),
        material=pin_steel,
        name="second_pin_head_0",
    )
    link_0.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_pin_origin(link_0_len, 0.045),
        material=pin_steel,
        name="second_pin_head_1",
    )

    link_1 = model.part("link_1")
    _add_link_visuals(
        link_1,
        prefix="link_1",
        length=link_1_len,
        layer_y=0.025,
        plate_material=painted_steel,
        boss_material=dark_steel,
    )
    link_1.visual(
        Cylinder(radius=0.010, length=0.058),
        origin=_pin_origin(link_1_len, 0.016),
        material=pin_steel,
        name="third_pin",
    )
    link_1.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_pin_origin(link_1_len, -0.013),
        material=pin_steel,
        name="third_pin_head_0",
    )
    link_1.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_pin_origin(link_1_len, 0.045),
        material=pin_steel,
        name="third_pin_head_1",
    )

    link_2 = model.part("link_2")
    _add_link_visuals(
        link_2,
        prefix="link_2",
        length=link_2_len,
        layer_y=0.0,
        plate_material=painted_steel,
        boss_material=dark_steel,
    )
    link_2.visual(
        Box((0.048, 0.048, 0.078)),
        origin=Origin(xyz=(link_2_len + 0.030, 0.0, 0.0)),
        material=black_rubber,
        name="end_pad",
    )
    link_2.visual(
        Box((0.024, 0.028, 0.040)),
        origin=Origin(xyz=(link_2_len + 0.004, 0.0, 0.0)),
        material=dark_steel,
        name="pad_backer",
    )

    model.articulation(
        "cheek_to_link_0",
        ArticulationType.REVOLUTE,
        parent=fixed_cheek,
        child=link_0,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-1.20, upper=1.15),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(link_0_len, 0.0, 0.0), rpy=(0.0, -0.72, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.4, lower=-1.45, upper=1.35),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(link_1_len, 0.0, 0.0), rpy=(0.0, 1.05, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.6, lower=-1.35, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    revolute_joints = [
        object_model.get_articulation("cheek_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
    ]
    ctx.check(
        "three parallel revolute pin joints",
        len(revolute_joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in revolute_joints)
        and all(tuple(j.axis or ()) == (0.0, 1.0, 0.0) for j in revolute_joints),
        details=f"joints={[j.name for j in revolute_joints]}",
    )

    fixed_cheek = object_model.get_part("fixed_cheek")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")

    ctx.allow_overlap(
        fixed_cheek,
        link_0,
        elem_a="first_pin",
        elem_b="plate",
        reason="The cheek pin is intentionally modeled as a captured shaft passing through the first link's clearance bore.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="second_pin",
        elem_b="plate",
        reason="The second pin is intentionally modeled as a captured shaft passing through the middle link's clearance bore.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        elem_a="third_pin",
        elem_b="plate",
        reason="The third pin is intentionally modeled as a captured shaft passing through the final link's clearance bore.",
    )

    ctx.expect_within(
        fixed_cheek,
        link_0,
        axes="xz",
        inner_elem="first_pin",
        outer_elem="plate",
        margin=0.002,
        name="first pin runs through the first link boss",
    )
    ctx.expect_within(
        link_0,
        link_1,
        axes="xz",
        inner_elem="second_pin",
        outer_elem="plate",
        margin=0.002,
        name="second pin runs through the middle link boss",
    )
    ctx.expect_within(
        link_1,
        link_2,
        axes="xz",
        inner_elem="third_pin",
        outer_elem="plate",
        margin=0.002,
        name="third pin runs through the final link boss",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="y",
        positive_elem="plate",
        negative_elem="plate",
        min_gap=0.006,
        max_gap=0.030,
        name="middle side plate is clear of first central link",
    )
    ctx.expect_gap(
        link_1,
        link_2,
        axis="y",
        positive_elem="plate",
        negative_elem="plate",
        min_gap=0.006,
        max_gap=0.030,
        name="middle side plate is clear of final central link",
    )

    pad_rest = ctx.part_element_world_aabb(link_2, elem="end_pad")
    joint_0, joint_1, joint_2 = revolute_joints
    with ctx.pose({joint_0: 0.45, joint_1: -0.55, joint_2: 0.60}):
        pad_moved = ctx.part_element_world_aabb(link_2, elem="end_pad")
    if pad_rest is not None and pad_moved is not None:
        rest_center_z = 0.5 * (pad_rest[0][2] + pad_rest[1][2])
        moved_center_z = 0.5 * (pad_moved[0][2] + pad_moved[1][2])
        ctx.check(
            "folding pose moves the end pad",
            abs(moved_center_z - rest_center_z) > 0.020,
            details=f"rest_z={rest_center_z:.3f}, moved_z={moved_center_z:.3f}",
        )
    else:
        ctx.fail("folding pose moves the end pad", "could not resolve end pad AABB")

    return ctx.report()


object_model = build_object_model()
