from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BAR_WIDTH = 0.032
BAR_THICKNESS = 0.010
BOSS_RADIUS = 0.024
BOSS_THICKNESS = 0.014


def _add_boxed_link(part, *, length: float, direction: float, layer_z: float, material, accent) -> None:
    """Add one compact rectangular link with integral round hinge bosses."""
    center_x = direction * length * 0.5
    distal_x = direction * length
    part.visual(
        Box((length, BAR_WIDTH, BAR_THICKNESS)),
        origin=Origin(xyz=(center_x, 0.0, layer_z)),
        material=material,
        name="link_box",
    )
    # Dark top inlay makes the member read as a boxed/tubular link instead of a plain bar.
    part.visual(
        Box((max(length - 2.2 * BOSS_RADIUS, 0.030), 0.014, 0.0015)),
        origin=Origin(
            xyz=(center_x, 0.0, layer_z + BAR_THICKNESS * 0.5 + 0.0005)
        ),
        material=accent,
        name="top_inlay",
    )
    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=BOSS_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, layer_z)),
        material=material,
        name="proximal_boss",
    )
    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=BOSS_THICKNESS),
        origin=Origin(xyz=(distal_x, 0.0, layer_z)),
        material=material,
        name="distal_boss",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_segment_fold_out_arm")

    satin_steel = model.material("satin_steel", rgba=(0.60, 0.64, 0.66, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.08, 0.23, 0.46, 1.0))
    dark_insert = model.material("black_recesses", rgba=(0.015, 0.017, 0.020, 1.0))
    screw_black = model.material("black_screws", rgba=(0.03, 0.03, 0.035, 1.0))

    base = model.part("base_plate")
    base.visual(
        Box((0.180, 0.120, 0.012)),
        origin=Origin(xyz=(-0.035, 0.0, 0.006)),
        material=satin_steel,
        name="plate",
    )
    base.visual(
        Box((0.072, 0.046, 0.010)),
        origin=Origin(xyz=(-0.020, 0.0, 0.017)),
        material=satin_steel,
        name="root_pad",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=satin_steel,
        name="root_boss",
    )
    for screw_index, (x, y) in enumerate(
        [(-0.100, -0.040), (-0.100, 0.040), (0.025, -0.040), (0.025, 0.040)]
    ):
        base.visual(
            Cylinder(radius=0.0075, length=0.003),
            origin=Origin(xyz=(x, y, 0.0135)),
            material=screw_black,
            name=f"screw_{screw_index}",
        )

    lengths = (0.160, 0.140, 0.120, 0.100)
    directions = (1.0, -1.0, 1.0, -1.0)
    layer_z = tuple(0.031 + i * BOSS_THICKNESS for i in range(4))
    links = [model.part(f"link_{i}") for i in range(4)]
    for link, length, direction, z in zip(links, lengths, directions, layer_z):
        _add_boxed_link(
            link,
            length=length,
            direction=direction,
            layer_z=z,
            material=blue_anodized,
            accent=dark_insert,
        )

    # The last link carries a small fixed platform bracket at its distal end.
    platform_center_x = directions[-1] * lengths[-1] - 0.026
    platform_z = layer_z[-1] + BOSS_THICKNESS * 0.5 + 0.003
    links[-1].visual(
        Box((0.076, 0.056, 0.008)),
        origin=Origin(xyz=(platform_center_x, 0.0, platform_z)),
        material=satin_steel,
        name="platform_plate",
    )
    links[-1].visual(
        Box((0.038, 0.032, 0.008)),
        origin=Origin(xyz=(directions[-1] * lengths[-1] - 0.006, 0.0, platform_z)),
        material=satin_steel,
        name="platform_neck",
    )
    for ear_index, y in enumerate((-0.025, 0.025)):
        links[-1].visual(
            Box((0.050, 0.006, 0.032)),
            origin=Origin(xyz=(platform_center_x, y, platform_z + 0.0195)),
            material=satin_steel,
            name=f"bracket_ear_{ear_index}",
        )
    for hole_index, y in enumerate((-0.016, 0.016)):
        links[-1].visual(
            Cylinder(radius=0.0055, length=0.002),
            origin=Origin(xyz=(platform_center_x - 0.006, y, platform_z + 0.005)),
            material=screw_black,
            name=f"platform_hole_{hole_index}",
        )

    joint_limits = (
        MotionLimits(effort=8.0, velocity=2.2, lower=-1.57, upper=1.57),
        MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=math.pi),
        MotionLimits(effort=8.0, velocity=2.2, lower=-math.pi, upper=0.0),
        MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=math.pi),
    )
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits[0],
    )
    for i in range(3):
        model.articulation(
            f"link_{i}_to_link_{i + 1}",
            ArticulationType.REVOLUTE,
            parent=links[i],
            child=links[i + 1],
            origin=Origin(xyz=(directions[i] * lengths[i], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=joint_limits[i + 1],
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("base_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_link_3"),
    ]
    links = [object_model.get_part(f"link_{i}") for i in range(4)]
    base = object_model.get_part("base_plate")

    ctx.check(
        "four serial revolute joints",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and [j.parent for j in joints] == ["base_plate", "link_0", "link_1", "link_2"]
        and [j.child for j in joints] == ["link_0", "link_1", "link_2", "link_3"],
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "all hinge axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.expect_gap(
        links[0],
        base,
        axis="z",
        positive_elem="proximal_boss",
        negative_elem="root_boss",
        max_gap=0.001,
        max_penetration=0.0,
        name="root hinge stack is seated",
    )
    for i in range(3):
        ctx.expect_gap(
            links[i + 1],
            links[i],
            axis="z",
            positive_elem="proximal_boss",
            negative_elem="distal_boss",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"hinge_{i + 1} stack is seated",
        )

    # At the zero pose, alternating link directions stack the four short links
    # tightly near the base while keeping small vertical running clearance.
    for i in range(3):
        ctx.expect_overlap(
            links[i],
            links[i + 1],
            axes="xy",
            elem_a="link_box",
            elem_b="link_box",
            min_overlap=0.025,
            name=f"folded links {i} and {i + 1} stack in footprint",
        )
        ctx.expect_gap(
            links[i + 1],
            links[i],
            axis="z",
            positive_elem="link_box",
            negative_elem="link_box",
            min_gap=0.002,
            max_gap=0.006,
            name=f"folded links {i} and {i + 1} have running clearance",
        )

    with ctx.pose(
        {
            joints[1]: math.pi,
            joints[2]: -math.pi,
            joints[3]: math.pi,
        }
    ):
        platform_aabb = ctx.part_element_world_aabb(links[3], elem="platform_plate")
        outward_ok = platform_aabb is not None and platform_aabb[1][0] > 0.50
        ctx.check(
            "platform bracket unfolds outward",
            outward_ok,
            details=f"platform_aabb={platform_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
