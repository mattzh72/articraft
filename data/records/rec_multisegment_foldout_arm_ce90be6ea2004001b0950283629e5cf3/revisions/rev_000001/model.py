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


LINK_AXIS_RPY = (math.pi / 2.0, 0.0, 0.0)


def _y_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=LINK_AXIS_RPY),
        material=material,
        name=name,
    )


def _add_flat_link(
    part,
    *,
    length: float,
    y_offset: float,
    material: Material,
    boss_material: Material,
    web_height: float = 0.034,
    web_thickness: float = 0.014,
    boss_radius: float = 0.026,
) -> None:
    """Add a flat bar link whose local origin is the proximal pin axis."""
    part.visual(
        Box((length, web_thickness, web_height)),
        origin=Origin(xyz=(length / 2.0, y_offset, 0.0)),
        material=material,
        name="flat_web",
    )
    for x, boss_name in ((0.0, "proximal_boss"), (length, "distal_boss")):
        _y_cylinder(
            part,
            name=boss_name,
            radius=boss_radius,
            length=web_thickness,
            xyz=(x, y_offset, 0.0),
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_fold_out_arm")

    cheek_metal = Material("matte_black_cheek", rgba=(0.02, 0.025, 0.028, 1.0))
    link_metal = Material("blue_grey_anodized_links", rgba=(0.23, 0.31, 0.36, 1.0))
    dark_pin = Material("dark_pin_bosses", rgba=(0.04, 0.045, 0.05, 1.0))
    platform_metal = Material("brushed_platform", rgba=(0.55, 0.57, 0.55, 1.0))
    bolt_black = Material("black_bolt_heads", rgba=(0.01, 0.01, 0.012, 1.0))

    cheek = model.part("cheek")
    cheek.visual(
        Box((0.060, 0.095, 0.180)),
        origin=Origin(xyz=(-0.075, 0.0, 0.025)),
        material=cheek_metal,
        name="mount_plate",
    )
    cheek.visual(
        Box((0.074, 0.014, 0.045)),
        origin=Origin(xyz=(-0.030, 0.014, 0.0)),
        material=cheek_metal,
        name="fork_web_upper",
    )
    cheek.visual(
        Box((0.074, 0.014, 0.045)),
        origin=Origin(xyz=(-0.030, -0.014, 0.0)),
        material=cheek_metal,
        name="fork_web_lower",
    )
    _y_cylinder(
        cheek,
        name="fork_boss_upper",
        radius=0.031,
        length=0.014,
        xyz=(0.0, 0.014, 0.0),
        material=cheek_metal,
    )
    _y_cylinder(
        cheek,
        name="fork_boss_lower",
        radius=0.031,
        length=0.014,
        xyz=(0.0, -0.014, 0.0),
        material=cheek_metal,
    )
    cheek.visual(
        Box((0.030, 0.094, 0.026)),
        origin=Origin(xyz=(-0.065, 0.0, -0.058)),
        material=cheek_metal,
        name="lower_bridge",
    )
    for y in (-0.030, 0.030):
        cheek.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(-0.1065, y, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"mount_bolt_{0 if y < 0 else 1}",
        )

    link_0 = model.part("link_0")
    link_1 = model.part("link_1")
    link_2 = model.part("link_2")
    _add_flat_link(link_0, length=0.280, y_offset=0.000, material=link_metal, boss_material=dark_pin)
    _add_flat_link(link_1, length=0.240, y_offset=0.014, material=link_metal, boss_material=dark_pin)
    _add_flat_link(link_2, length=0.205, y_offset=0.000, material=link_metal, boss_material=dark_pin)

    platform = model.part("platform_bracket")
    _y_cylinder(
        platform,
        name="proximal_boss",
        radius=0.027,
        length=0.014,
        xyz=(0.0, 0.014, 0.0),
        material=platform_metal,
    )
    _y_cylinder(
        platform,
        name="proximal_pin_head",
        radius=0.018,
        length=0.005,
        xyz=(0.0, 0.023, 0.0),
        material=dark_pin,
    )
    platform.visual(
        Box((0.080, 0.014, 0.034)),
        origin=Origin(xyz=(0.040, 0.014, 0.0)),
        material=platform_metal,
        name="side_tab",
    )
    platform.visual(
        Box((0.032, 0.014, 0.060)),
        origin=Origin(xyz=(0.075, 0.014, 0.019)),
        material=platform_metal,
        name="riser_web",
    )
    platform.visual(
        Box((0.120, 0.082, 0.012)),
        origin=Origin(xyz=(0.108, 0.014, 0.052)),
        material=platform_metal,
        name="platform_plate",
    )
    platform.visual(
        Box((0.020, 0.082, 0.036)),
        origin=Origin(xyz=(0.158, 0.014, 0.032)),
        material=platform_metal,
        name="end_lip",
    )
    for i, x in enumerate((0.082, 0.132)):
        platform.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, -0.006, 0.060)),
            material=bolt_black,
            name=f"platform_bolt_{i}",
        )

    model.articulation(
        "cheek_to_link_0",
        ArticulationType.REVOLUTE,
        parent=cheek,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -0.24, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.85, upper=1.20),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.280, 0.0, 0.0), rpy=(0.0, 0.58, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.240, 0.0, 0.0), rpy=(0.0, -0.50, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "link_2_to_platform",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=platform,
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, 0.16, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-1.10, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cheek = object_model.get_part("cheek")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    platform = object_model.get_part("platform_bracket")
    joints = [
        object_model.get_articulation("cheek_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_platform"),
    ]

    ctx.check(
        "four parallel revolute hinge axes",
        len(joints) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"joints={[(j.name, j.articulation_type, j.axis) for j in joints]}",
    )

    ctx.expect_overlap(
        link_0,
        cheek,
        axes="xz",
        elem_a="proximal_boss",
        elem_b="fork_boss_upper",
        min_overlap=0.030,
        name="base pin bosses share the cheek axis",
    )
    ctx.expect_gap(
        cheek,
        link_0,
        axis="y",
        positive_elem="fork_boss_upper",
        negative_elem="proximal_boss",
        min_gap=0.0,
        max_gap=0.002,
        name="upper cheek boss supports the base pin",
    )
    ctx.expect_gap(
        link_0,
        cheek,
        axis="y",
        positive_elem="proximal_boss",
        negative_elem="fork_boss_lower",
        min_gap=0.0,
        max_gap=0.002,
        name="lower cheek boss supports the base pin",
    )

    hinge_pairs = (
        (link_0, link_1, "distal_boss", "proximal_boss", "first elbow bosses align"),
        (link_1, link_2, "distal_boss", "proximal_boss", "second elbow bosses align"),
        (link_2, platform, "distal_boss", "proximal_boss", "platform hinge bosses align"),
    )
    for parent, child, parent_elem, child_elem, check_name in hinge_pairs:
        ctx.expect_overlap(
            parent,
            child,
            axes="xz",
            elem_a=parent_elem,
            elem_b=child_elem,
            min_overlap=0.030,
            name=check_name,
        )

    ctx.expect_gap(
        link_1,
        link_0,
        axis="y",
        positive_elem="proximal_boss",
        negative_elem="distal_boss",
        min_gap=0.0,
        max_gap=0.002,
        name="first elbow side-by-side bosses clear",
    )
    ctx.expect_gap(
        link_1,
        link_2,
        axis="y",
        positive_elem="distal_boss",
        negative_elem="proximal_boss",
        min_gap=0.0,
        max_gap=0.002,
        name="second elbow side-by-side bosses clear",
    )
    ctx.expect_gap(
        platform,
        link_2,
        axis="y",
        positive_elem="proximal_boss",
        negative_elem="distal_boss",
        min_gap=0.0,
        max_gap=0.002,
        name="platform hinge side-by-side bosses clear",
    )

    rest_end = ctx.part_world_position(platform)
    with ctx.pose(
        {
            joints[0]: 0.55,
            joints[1]: -0.65,
            joints[2]: 0.50,
            joints[3]: -0.35,
        }
    ):
        folded_end = ctx.part_world_position(platform)
    ctx.check(
        "fold pose moves the end bracket",
        rest_end is not None
        and folded_end is not None
        and math.dist(rest_end, folded_end) > 0.08,
        details=f"rest={rest_end}, folded={folded_end}",
    )

    return ctx.report()


object_model = build_object_model()
