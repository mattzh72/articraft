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
    TestContext,
    TestReport,
)


STEEL = Material("powder_coated_dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
LINK_METAL = Material("satin_black_link_metal", rgba=(0.015, 0.017, 0.018, 1.0))
BUSHING = Material("oilite_bronze_bushings", rgba=(0.72, 0.48, 0.18, 1.0))
PLATFORM_MAT = Material("matte_graphite_platform", rgba=(0.12, 0.13, 0.14, 1.0))
BOLT_MAT = Material("dark_socket_head_bolts", rgba=(0.01, 0.01, 0.012, 1.0))


HINGE_AXIS = (0.0, 1.0, 0.0)
CYLINDER_ALONG_Y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _pitch_for_vector(start: tuple[float, float, float], end: tuple[float, float, float]) -> float:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    return math.atan2(-dz, dx)


def _point_along(
    start: tuple[float, float, float], end: tuple[float, float, float], distance_from_end: float
) -> tuple[float, float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0:
        return start
    t = max(0.0, min(1.0, (length - distance_from_end) / length))
    return (start[0] + dx * t, start[1] + dy * t, start[2] + dz * t)


def _add_bar(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    y_center: float = 0.0,
    y_width: float = 0.045,
    thickness: float = 0.034,
    material=LINK_METAL,
) -> None:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    if length <= 1e-6:
        return
    part.visual(
        Box((length, y_width, thickness)),
        origin=Origin(
            xyz=((start[0] + end[0]) / 2.0, y_center, (start[2] + end[2]) / 2.0),
            rpy=(0.0, _pitch_for_vector(start, end), 0.0),
        ),
        material=material,
        name=name,
    )


def _add_hinge_boss(part, name: str, xyz: tuple[float, float, float], *, y: float, length: float, radius: float) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(xyz[0], y, xyz[2]), rpy=CYLINDER_ALONG_Y.rpy),
        material=BUSHING,
        name=name,
    )


def _add_segment_link(
    part,
    distal: tuple[float, float, float],
    *,
    prefix: str,
    bar_material=LINK_METAL,
) -> None:
    """A central hinge eye at the proximal end and a forked clevis at the distal end."""
    start = (0.0, 0.0, 0.0)
    fork_root = _point_along(start, distal, 0.125)
    fork_neck = _point_along(start, distal, 0.155)

    _add_hinge_boss(part, f"{prefix}_proximal_eye", start, y=0.0, length=0.076, radius=0.045)
    _add_bar(
        part,
        f"{prefix}_spine",
        start,
        fork_root,
        y_center=0.0,
        y_width=0.044,
        thickness=0.034,
        material=bar_material,
    )
    _add_bar(
        part,
        f"{prefix}_fork_bridge",
        fork_neck,
        fork_root,
        y_center=0.0,
        y_width=0.142,
        thickness=0.038,
        material=bar_material,
    )
    _add_bar(
        part,
        f"{prefix}_fork_0",
        fork_root,
        distal,
        y_center=0.066,
        y_width=0.026,
        thickness=0.035,
        material=bar_material,
    )
    _add_bar(
        part,
        f"{prefix}_fork_1",
        fork_root,
        distal,
        y_center=-0.066,
        y_width=0.026,
        thickness=0.035,
        material=bar_material,
    )
    _add_hinge_boss(part, f"{prefix}_distal_lug_0", distal, y=0.053, length=0.030, radius=0.045)
    _add_hinge_boss(part, f"{prefix}_distal_lug_1", distal, y=-0.053, length=0.030, radius=0.045)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_fold_out_arm")

    support = model.part("top_support")
    support.visual(
        Box((0.36, 0.22, 0.035)),
        origin=Origin(xyz=(0.02, 0.0, 0.145)),
        material=STEEL,
        name="ceiling_plate",
    )
    support.visual(
        Box((0.13, 0.17, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
        material=STEEL,
        name="underside_saddle",
    )
    for idx, y in enumerate((-0.054, 0.054)):
        support.visual(
            Box((0.105, 0.024, 0.142)),
            origin=Origin(xyz=(0.0, y, 0.057)),
            material=STEEL,
            name=f"yoke_cheek_{idx}",
        )
        _add_hinge_boss(support, f"upper_bushing_{idx}", (0.0, 0.0, 0.0), y=y, length=0.032, radius=0.047)
    for idx, (x, y) in enumerate(((-0.12, -0.075), (-0.12, 0.075), (0.15, -0.075), (0.15, 0.075))):
        support.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x, y, 0.165)),
            material=BOLT_MAT,
            name=f"mount_bolt_{idx}",
        )

    link_0 = model.part("link_0")
    link_1 = model.part("link_1")
    link_2 = model.part("link_2")
    platform = model.part("platform_bracket")

    distal_0 = (0.42, 0.0, -0.23)
    distal_1 = (-0.37, 0.0, -0.24)
    distal_2 = (0.32, 0.0, -0.20)

    _add_segment_link(link_0, distal_0, prefix="link_0")
    _add_segment_link(link_1, distal_1, prefix="link_1")
    _add_segment_link(link_2, distal_2, prefix="link_2")

    _add_hinge_boss(platform, "platform_eye", (0.0, 0.0, 0.0), y=0.0, length=0.076, radius=0.043)
    platform.visual(
        Box((0.050, 0.048, 0.150)),
        origin=Origin(xyz=(0.018, 0.0, -0.070)),
        material=PLATFORM_MAT,
        name="drop_web",
    )
    platform.visual(
        Box((0.240, 0.180, 0.025)),
        origin=Origin(xyz=(0.112, 0.0, -0.145)),
        material=PLATFORM_MAT,
        name="small_shelf",
    )
    for idx, y in enumerate((-0.091, 0.091)):
        platform.visual(
            Box((0.240, 0.018, 0.055)),
            origin=Origin(xyz=(0.112, y, -0.110)),
            material=PLATFORM_MAT,
            name=f"shelf_lip_{idx}",
        )
    for idx, (x, y) in enumerate(((0.035, -0.045), (0.035, 0.045), (0.185, -0.045), (0.185, 0.045))):
        platform.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(x, y, -0.129)),
            material=BOLT_MAT,
            name=f"platform_bolt_{idx}",
        )

    hinge_limits = MotionLimits(effort=18.0, velocity=2.0, lower=-1.35, upper=1.35)
    end_limits = MotionLimits(effort=12.0, velocity=2.0, lower=-1.15, upper=1.15)
    model.articulation(
        "support_to_link_0",
        ArticulationType.REVOLUTE,
        parent=support,
        child=link_0,
        origin=Origin(),
        axis=HINGE_AXIS,
        motion_limits=end_limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=distal_0),
        axis=HINGE_AXIS,
        motion_limits=hinge_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=distal_1),
        axis=HINGE_AXIS,
        motion_limits=hinge_limits,
    )
    model.articulation(
        "link_2_to_platform",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=platform,
        origin=Origin(xyz=distal_2),
        axis=HINGE_AXIS,
        motion_limits=end_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    expected_joints = (
        "support_to_link_0",
        "link_0_to_link_1",
        "link_1_to_link_2",
        "link_2_to_platform",
    )
    joints = [object_model.get_articulation(name) for name in expected_joints]
    ctx.check(
        "four parallel revolute hinges",
        len(joints) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(round(v, 6) for v in j.axis) == HINGE_AXIS for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )

    support = object_model.get_part("top_support")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    platform = object_model.get_part("platform_bracket")

    ctx.expect_gap(
        support,
        link_0,
        axis="z",
        positive_elem="ceiling_plate",
        min_gap=0.020,
        max_gap=0.12,
        name="first link is carried below upper bracket",
    )
    ctx.expect_overlap(link_0, link_1, axes="xz", min_overlap=0.03, name="first and second hinge ends meet")
    ctx.expect_overlap(link_1, link_2, axes="xz", min_overlap=0.03, name="second and third hinge ends meet")
    ctx.expect_overlap(link_2, platform, axes="xz", min_overlap=0.03, name="last link carries platform bracket")

    rest_platform = ctx.part_world_position(platform)
    with ctx.pose({"support_to_link_0": 0.55, "link_0_to_link_1": -0.45, "link_1_to_link_2": 0.35}):
        moved_platform = ctx.part_world_position(platform)
    ctx.check(
        "fold-out chain moves the platform",
        rest_platform is not None
        and moved_platform is not None
        and abs(moved_platform[0] - rest_platform[0]) > 0.04,
        details=f"rest={rest_platform}, moved={moved_platform}",
    )

    return ctx.report()


object_model = build_object_model()
