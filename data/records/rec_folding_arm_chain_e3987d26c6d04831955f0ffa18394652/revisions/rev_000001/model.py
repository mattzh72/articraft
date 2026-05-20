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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_task_light_arm")

    dark_metal = model.material("powder_coated_dark_grey", rgba=(0.18, 0.19, 0.20, 1.0))
    edge_metal = model.material("brushed_pin_metal", rgba=(0.66, 0.67, 0.64, 1.0))
    rubber = model.material("plain_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    # The mechanism folds in the X/Z plane; all pivot axes run along local Y.
    pin_axis_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    screw_axis_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    root = model.part("root_bracket")
    root.visual(
        Box((0.028, 0.150, 0.250)),
        origin=Origin(xyz=(-0.112, 0.0, -0.020)),
        material=dark_metal,
        name="wall_plate",
    )
    root.visual(
        Box((0.076, 0.080, 0.032)),
        origin=Origin(xyz=(-0.074, 0.0, -0.026)),
        material=dark_metal,
        name="root_bridge",
    )
    for side, y in enumerate((-0.032, 0.032)):
        root.visual(
            Box((0.085, 0.014, 0.074)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_metal,
            name=f"root_cheek_{side}",
        )
        root.visual(
            Cylinder(radius=0.027, length=0.006),
            origin=Origin(xyz=(0.0, y * 1.31, 0.0), rpy=pin_axis_y.rpy),
            material=edge_metal,
            name=f"root_pin_cap_{side}",
        )
    for row, z in enumerate((-0.080, 0.055)):
        for col, y in enumerate((-0.048, 0.048)):
            root.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(-0.095, y, z), rpy=screw_axis_x.rpy),
                material=edge_metal,
                name=f"mount_screw_{row}_{col}",
            )

    def add_link(part_name: str, length: float, *, distal_clevis: bool) -> object:
        link = model.part(part_name)
        rail_end = length - (0.078 if distal_clevis else 0.025)
        link.visual(
            Box((rail_end - 0.040, 0.030, 0.018)),
            origin=Origin(xyz=((rail_end + 0.040) / 2.0, 0.0, -0.032)),
            material=dark_metal,
            name="lower_rail",
        )
        link.visual(
            Box((0.060, 0.032, 0.028)),
            origin=Origin(xyz=(0.018, 0.0, 0.0)),
            material=dark_metal,
            name="proximal_tongue",
        )
        link.visual(
            Cylinder(radius=0.024, length=0.077),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=pin_axis_y.rpy),
            material=edge_metal,
            name="proximal_boss",
        )
        link.visual(
            Box((0.060, 0.030, 0.046)),
            origin=Origin(xyz=(0.030, 0.0, -0.016)),
            material=dark_metal,
            name="proximal_web",
        )

        if distal_clevis:
            link.visual(
                Box((0.055, 0.078, 0.022)),
                origin=Origin(xyz=(length - 0.062, 0.0, -0.025)),
                material=dark_metal,
                name="distal_bridge",
            )
            for side, y in enumerate((-0.032, 0.032)):
                link.visual(
                    Box((0.075, 0.014, 0.072)),
                    origin=Origin(xyz=(length, y, -0.004)),
                    material=dark_metal,
                    name=f"distal_cheek_{side}",
                )
                link.visual(
                    Cylinder(radius=0.026, length=0.006),
                    origin=Origin(xyz=(length, y * 1.31, 0.0), rpy=pin_axis_y.rpy),
                    material=edge_metal,
                    name=f"distal_pin_cap_{side}",
                )
        else:
            link.visual(
                Box((0.055, 0.044, 0.036)),
                origin=Origin(xyz=(length - 0.040, 0.0, -0.025)),
                material=dark_metal,
                name="tip_neck",
            )
            link.visual(
                Cylinder(radius=0.039, length=0.028),
                origin=Origin(xyz=(length - 0.006, 0.0, -0.026), rpy=screw_axis_x.rpy),
                material=rubber,
                name="tip_pad",
            )
        return link

    link_0 = add_link("link_0", 0.420, distal_clevis=True)
    link_1 = add_link("link_1", 0.380, distal_clevis=True)
    link_2 = add_link("link_2", 0.320, distal_clevis=False)

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=root,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.80, upper=1.40),
    )
    model.articulation(
        "middle_pivot",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.5, lower=-2.75, upper=1.20),
    )
    model.articulation(
        "tip_pivot",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-2.15, upper=1.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_bracket")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    root_pivot = object_model.get_articulation("root_pivot")
    middle_pivot = object_model.get_articulation("middle_pivot")
    tip_pivot = object_model.get_articulation("tip_pivot")

    ctx.check(
        "three revolute folding pivots",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    for joint in (root_pivot, middle_pivot, tip_pivot):
        ctx.check(
            f"{joint.name} axis is out of plane",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )

    def allow_captured_pin(parent, child, cheek_names, label: str) -> None:
        for cheek_name in cheek_names:
            ctx.allow_overlap(
                parent,
                child,
                elem_a=cheek_name,
                elem_b="proximal_boss",
                reason="The hinge pin is intentionally captured through the clevis cheek bore proxy.",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="yz",
                elem_a="proximal_boss",
                elem_b=cheek_name,
                min_overlap=0.010,
                name=f"{label} pin passes through {cheek_name}",
            )

    allow_captured_pin(root, link_0, ("root_cheek_0", "root_cheek_1"), "root")
    allow_captured_pin(link_0, link_1, ("distal_cheek_0", "distal_cheek_1"), "middle")
    allow_captured_pin(link_1, link_2, ("distal_cheek_0", "distal_cheek_1"), "tip")

    ctx.expect_within(
        link_0,
        root,
        axes="yz",
        inner_elem="proximal_boss",
        margin=0.002,
        name="root clevis captures first link boss",
    )
    ctx.expect_within(
        link_1,
        link_0,
        axes="yz",
        inner_elem="proximal_boss",
        margin=0.002,
        name="first clevis captures second link boss",
    )
    ctx.expect_within(
        link_2,
        link_1,
        axes="yz",
        inner_elem="proximal_boss",
        margin=0.002,
        name="second clevis captures third link boss",
    )

    extended_pos = ctx.part_world_position(link_2)
    ctx.check(
        "zero pose extends outward",
        extended_pos is not None and extended_pos[0] > 0.75 and abs(extended_pos[1]) < 1.0e-6,
        details=f"link_2_origin={extended_pos}",
    )

    with ctx.pose({root_pivot: 1.35, middle_pivot: -2.70, tip_pivot: -1.78}):
        folded_pos = ctx.part_world_position(link_2)
        folded_aabb = ctx.part_world_aabb(link_2)
        max_x = folded_aabb[1][0] if folded_aabb is not None else None
        ctx.check(
            "folded arm returns near root",
            folded_pos is not None
            and extended_pos is not None
            and folded_pos[0] < 0.20
            and max_x is not None
            and max_x < extended_pos[0] - 0.45,
            details=f"extended={extended_pos}, folded={folded_pos}, folded_max_x={max_x}",
        )

    return ctx.report()


object_model = build_object_model()
