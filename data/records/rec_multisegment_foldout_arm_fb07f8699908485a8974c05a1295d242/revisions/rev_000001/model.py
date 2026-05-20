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


PIN_RPY = (math.pi / 2.0, 0.0, 0.0)


def _pin_origin(x: float, y: float, z: float = 0.0) -> Origin:
    """Cylinder helper: SDK cylinders are local-Z; these pins run across Y."""
    return Origin(xyz=(x, y, z), rpy=PIN_RPY)


def _add_joint_boss_pair(part, *, x: float, prefix: str, material: Material) -> None:
    """Two compact outer bosses on the fork cheeks of a link."""
    for index, y in enumerate((-0.0265, 0.0265)):
        part.visual(
            Cylinder(radius=0.030, length=0.017),
            origin=_pin_origin(x, y),
            material=material,
            name=f"{prefix}_boss_{index}",
        )


def _add_rigid_link(part, *, length: float, material: Material, dark: Material) -> None:
    """A slim single-plane link with a center lug and a forked distal cheek."""
    part.visual(
        Box((0.052, 0.024, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=material,
        name="proximal_lug",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.036),
        origin=_pin_origin(0.0, 0.0),
        material=dark,
        name="proximal_bushing",
    )

    bar_length = length - 0.055
    part.visual(
        Box((bar_length, 0.024, 0.022)),
        origin=Origin(xyz=(0.020 + bar_length / 2.0, 0.0, 0.0)),
        material=material,
        name="slim_bar",
    )
    part.visual(
        Box((0.050, 0.070, 0.016)),
        origin=Origin(xyz=(length - 0.065, 0.0, 0.014)),
        material=material,
        name="fork_bridge",
    )
    for name, y in (("distal_cheek_0", -0.0265), ("distal_cheek_1", 0.0265)):
        part.visual(
            Box((0.080, 0.017, 0.064)),
            origin=Origin(xyz=(length, y, 0.0)),
            material=material,
            name=name,
        )
    _add_joint_boss_pair(part, x=length, prefix="distal", material=dark)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_service_foldout_arm")

    spine_mat = Material("satin_black_spine", rgba=(0.02, 0.025, 0.028, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.62, 0.66, 0.67, 1.0))
    dark = Material("dark_pivot_bushings", rgba=(0.05, 0.055, 0.06, 1.0))
    platform_mat = Material("pale_platform_bracket", rgba=(0.82, 0.84, 0.78, 1.0))

    link0_len = 0.320
    link1_len = 0.260
    link2_len = 0.240

    spine = model.part("spine")
    spine.visual(
        Box((0.20, 0.15, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=spine_mat,
        name="ground_plate",
    )
    spine.visual(
        Box((0.050, 0.060, 0.640)),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=spine_mat,
        name="narrow_upright",
    )
    spine.visual(
        Box((0.080, 0.050, 0.070)),
        origin=Origin(xyz=(-0.020, 0.0, 0.640)),
        material=spine_mat,
        name="top_root_block",
    )
    for cheek_name, boss_name, y in (
        ("root_cheek_0", "root_boss_0", -0.0265),
        ("root_cheek_1", "root_boss_1", 0.0265),
    ):
        spine.visual(
            Box((0.116, 0.017, 0.078)),
            origin=Origin(xyz=(0.075, y, 0.640)),
            material=spine_mat,
            name=cheek_name,
        )
        spine.visual(
            Cylinder(radius=0.032, length=0.017),
            origin=_pin_origin(0.075, y, 0.640),
            material=dark,
            name=boss_name,
        )

    link_0 = model.part("link_0")
    _add_rigid_link(link_0, length=link0_len, material=aluminum, dark=dark)

    link_1 = model.part("link_1")
    _add_rigid_link(link_1, length=link1_len, material=aluminum, dark=dark)

    link_2 = model.part("link_2")
    _add_rigid_link(link_2, length=link2_len, material=aluminum, dark=dark)

    platform = model.part("platform_bracket")
    platform.visual(
        Box((0.052, 0.024, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=platform_mat,
        name="proximal_lug",
    )
    platform.visual(
        Cylinder(radius=0.022, length=0.036),
        origin=_pin_origin(0.0, 0.0),
        material=dark,
        name="proximal_bushing",
    )
    platform.visual(
        Box((0.095, 0.026, 0.024)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=platform_mat,
        name="short_neck",
    )
    platform.visual(
        Box((0.160, 0.105, 0.018)),
        origin=Origin(xyz=(0.145, 0.0, -0.002)),
        material=platform_mat,
        name="small_platform",
    )
    for index, y in enumerate((-0.0525, 0.0525)):
        platform.visual(
            Box((0.145, 0.012, 0.036)),
            origin=Origin(xyz=(0.145, y, 0.015)),
            material=platform_mat,
            name=f"side_lip_{index}",
        )

    model.articulation(
        "spine_to_link_0",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=link_0,
        origin=Origin(xyz=(0.075, 0.0, 0.640)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.80, upper=1.10),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(link0_len, 0.0, 0.0), rpy=(0.0, -0.85, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-1.00, upper=1.15),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(link1_len, 0.0, 0.0), rpy=(0.0, 1.40, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "link_2_to_platform",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=platform,
        origin=Origin(xyz=(link2_len, 0.0, 0.0), rpy=(0.0, -0.55, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [
        object_model.get_articulation("spine_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_platform"),
    ]
    ctx.check(
        "four serial revolute joints",
        len(joints) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "joint axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    spine = object_model.get_part("spine")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    platform = object_model.get_part("platform_bracket")

    ctx.expect_overlap(
        link_0,
        spine,
        axes="xz",
        elem_a="proximal_lug",
        elem_b="root_cheek_0",
        min_overlap=0.025,
        name="first lug sits inside spine fork",
    )
    ctx.expect_overlap(
        link_1,
        link_0,
        axes="xz",
        elem_a="proximal_lug",
        elem_b="distal_cheek_0",
        min_overlap=0.020,
        name="second lug sits inside first fork",
    )
    ctx.expect_overlap(
        link_2,
        link_1,
        axes="xz",
        elem_a="proximal_lug",
        elem_b="distal_cheek_0",
        min_overlap=0.020,
        name="third lug sits inside second fork",
    )
    ctx.expect_overlap(
        platform,
        link_2,
        axes="xz",
        elem_a="proximal_lug",
        elem_b="distal_cheek_0",
        min_overlap=0.020,
        name="platform lug sits inside terminal fork",
    )

    rest_platform = ctx.part_world_position(platform)
    with ctx.pose({"spine_to_link_0": 0.35, "link_0_to_link_1": -0.25, "link_1_to_link_2": 0.25}):
        moved_platform = ctx.part_world_position(platform)
    ctx.check(
        "serial arm changes platform pose",
        rest_platform is not None
        and moved_platform is not None
        and abs(moved_platform[0] - rest_platform[0]) > 0.025,
        details=f"rest={rest_platform}, moved={moved_platform}",
    )

    return ctx.report()


object_model = build_object_model()
