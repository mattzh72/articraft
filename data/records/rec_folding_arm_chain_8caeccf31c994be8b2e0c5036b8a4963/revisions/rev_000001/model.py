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
    model = ArticulatedObject(name="compact_folding_arm_chain")

    black = model.material("black_anodized_aluminum", rgba=(0.015, 0.018, 0.020, 1.0))
    graphite = model.material("graphite_side_plates", rgba=(0.09, 0.10, 0.105, 1.0))
    steel = model.material("brushed_steel_pins", rgba=(0.70, 0.72, 0.70, 1.0))
    orange = model.material("orange_bearing_bushings", rgba=(0.95, 0.36, 0.06, 1.0))
    rubber = model.material("black_rubber_feet", rgba=(0.02, 0.018, 0.016, 1.0))

    cyl_y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))

    def y_cylinder_origin(x: float, y: float, z: float) -> Origin:
        return Origin(xyz=(x, y, z), rpy=cyl_y.rpy)

    def add_pin_faces(part, x: float, z: float, prefix: str, y: float = 0.010) -> None:
        for suffix, yy in (("pos", y), ("neg", -y)):
            part.visual(
                Cylinder(radius=0.017, length=0.004),
                origin=y_cylinder_origin(x, yy, z),
                material=steel,
                name=f"{prefix}_pin_face_{suffix}",
            )

    def add_central_x_link(part, length: float, *, prefix: str) -> None:
        part.visual(
            Box((length, 0.026, 0.041)),
            origin=Origin(xyz=(length / 2.0, 0.0, 0.0)),
            material=black,
            name=f"{prefix}_web",
        )
        for label, x in (("root", 0.0), ("end", length)):
            part.visual(
                Cylinder(radius=0.026, length=0.026),
                origin=y_cylinder_origin(x, 0.0, 0.0),
                material=black,
                name=f"{prefix}_{label}_boss",
            )
            part.visual(
                Cylinder(radius=0.019, length=0.018),
                origin=y_cylinder_origin(x, 0.0, 0.0),
                material=orange,
                name=f"{prefix}_{label}_bushing",
            )
            add_pin_faces(part, x, 0.0, f"{prefix}_{label}")

    def add_central_z_link(part, length: float, *, prefix: str) -> None:
        part.visual(
            Box((0.041, 0.026, length)),
            origin=Origin(xyz=(0.0, 0.0, length / 2.0)),
            material=black,
            name=f"{prefix}_web",
        )
        for label, z in (("root", 0.0), ("end", length)):
            part.visual(
                Cylinder(radius=0.026, length=0.026),
                origin=y_cylinder_origin(0.0, 0.0, z),
                material=black,
                name=f"{prefix}_{label}_boss",
            )
            part.visual(
                Cylinder(radius=0.019, length=0.018),
                origin=y_cylinder_origin(0.0, 0.0, z),
                material=orange,
                name=f"{prefix}_{label}_bushing",
            )
            add_pin_faces(part, 0.0, z, f"{prefix}_{label}")

    def add_outer_x_link(part, length: float, *, prefix: str) -> None:
        # This link is a boxed clevis: two outer side plates with top/bottom
        # bridges.  It nests around the central links without occupying their
        # middle y-layer.
        for suffix, y in (("pos", 0.031), ("neg", -0.031)):
            part.visual(
                Box((length, 0.012, 0.056)),
                origin=Origin(xyz=(-length / 2.0, y, 0.0)),
                material=graphite,
                name=f"{prefix}_side_plate_{suffix}",
            )
            for label, x in (("root", 0.0), ("end", -length)):
                part.visual(
                    Cylinder(radius=0.033, length=0.012),
                    origin=y_cylinder_origin(x, y, 0.0),
                    material=graphite,
                    name=f"{prefix}_{label}_lug_{suffix}",
                )

        bridge_len = length - 0.13
        for suffix, z in (("top", 0.032), ("bottom", -0.032)):
            part.visual(
                Box((bridge_len, 0.074, 0.010)),
                origin=Origin(xyz=(-length / 2.0, 0.0, z)),
                material=graphite,
                name=f"{prefix}_{suffix}_bridge",
            )

        for label, x in (("root", 0.0), ("end", -length)):
            for suffix, y in (("pos", 0.0415), ("neg", -0.0415)):
                part.visual(
                    Cylinder(radius=0.023, length=0.010),
                    origin=y_cylinder_origin(x, y, 0.0),
                    material=steel,
                    name=f"{prefix}_{label}_outer_pin_{suffix}",
                )
            for suffix, y in (("pos", 0.031), ("neg", -0.031)):
                part.visual(
                    Cylinder(radius=0.025, length=0.012),
                    origin=y_cylinder_origin(x, y, 0.0),
                    material=orange,
                    name=f"{prefix}_{label}_bushing_{suffix}",
                )

    base = model.part("base_bracket")
    base.visual(
        Box((0.28, 0.20, 0.024)),
        origin=Origin(xyz=(-0.040, 0.0, -0.212)),
        material=graphite,
        name="floor_plate",
    )
    for suffix, y in (("pos", 0.069), ("neg", -0.069)):
        base.visual(
            Box((0.055, 0.018, 0.200)),
            origin=Origin(xyz=(-0.030, y, -0.100)),
            material=graphite,
            name=f"side_post_{suffix}",
        )
        base.visual(
            Box((0.120, 0.018, 0.130)),
            origin=Origin(xyz=(0.0, 0.055 if y > 0 else -0.055, 0.0)),
            material=graphite,
            name=f"clevis_cheek_{suffix}",
        )
        base.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=y_cylinder_origin(0.0, 0.071 if y > 0 else -0.071, 0.0),
            material=steel,
            name=f"base_pin_cap_{suffix}",
        )
    for suffix, y in (("pos", 0.0295), ("neg", -0.0295)):
        base.visual(
            Cylinder(radius=0.020, length=0.033),
            origin=y_cylinder_origin(0.0, y, 0.0),
            material=steel,
            name=f"base_hinge_spacer_{suffix}",
        )
    for ix, x in enumerate((-0.135, 0.055)):
        for iy, y in enumerate((-0.065, 0.065)):
            base.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(x, y, -0.196)),
                material=steel,
                name=f"mount_bolt_{ix}_{iy}",
            )
            base.visual(
                Cylinder(radius=0.017, length=0.010),
                origin=Origin(xyz=(x, y, -0.229)),
                material=rubber,
                name=f"rubber_foot_{ix}_{iy}",
            )

    link_0 = model.part("root_link")
    add_central_x_link(link_0, 0.450, prefix="root")
    for suffix, y in (("pos", 0.019), ("neg", -0.019)):
        link_0.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=y_cylinder_origin(0.450, y, 0.0),
            material=steel,
            name=f"root_elbow_spacer_{suffix}",
        )

    link_1 = model.part("middle_link")
    add_outer_x_link(link_1, 0.390, prefix="middle")
    for suffix, y in (("pos", 0.019), ("neg", -0.019)):
        link_1.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=y_cylinder_origin(-0.390, y, 0.0),
            material=steel,
            name=f"middle_wrist_spacer_{suffix}",
        )

    link_2 = model.part("tip_link")
    add_central_z_link(link_2, 0.280, prefix="tip")

    model.articulation(
        "base_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.450, 0.0, 0.0), rpy=(0.0, math.pi / 4.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.2, lower=0.0, upper=3.0 * math.pi / 4.0),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(-0.390, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-math.pi / 2.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    link_0 = object_model.get_part("root_link")
    link_1 = object_model.get_part("middle_link")
    link_2 = object_model.get_part("tip_link")
    base_joint = object_model.get_articulation("base_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

    joints = (base_joint, elbow_joint, wrist_joint)
    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and base_joint.parent == "base_bracket"
        and base_joint.child == "root_link"
        and elbow_joint.parent == "root_link"
        and elbow_joint.child == "middle_link"
        and wrist_joint.parent == "middle_link"
        and wrist_joint.child == "tip_link",
        details=f"joints={[(j.name, j.articulation_type, j.parent, j.child) for j in joints]}",
    )
    ctx.check(
        "joint axes are parallel y axes",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    base_cheek_pos = base.get_visual("clevis_cheek_pos")
    root_root_boss = link_0.get_visual("root_root_boss")
    middle_side_plate_pos = link_1.get_visual("middle_side_plate_pos")
    root_end_boss = link_0.get_visual("root_end_boss")

    ctx.expect_gap(
        base,
        link_0,
        axis="y",
        positive_elem=base_cheek_pos,
        negative_elem=root_root_boss,
        min_gap=0.010,
        name="base clevis straddles root link",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="y",
        positive_elem=middle_side_plate_pos,
        negative_elem=root_end_boss,
        min_gap=0.006,
        name="middle clevis clears root boss",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="x",
        min_overlap=0.18,
        name="folded links pack in shared footprint",
    )

    folded_aabb = ctx.part_world_aabb(link_1)
    root_aabb = ctx.part_world_aabb(link_0)
    tip_aabb = ctx.part_world_aabb(link_2)
    if folded_aabb is not None and root_aabb is not None and tip_aabb is not None:
        folded_min_x = min(folded_aabb[0][0], root_aabb[0][0], tip_aabb[0][0])
        folded_max_x = max(folded_aabb[1][0], root_aabb[1][0], tip_aabb[1][0])
        folded_span_x = folded_max_x - folded_min_x
    else:
        folded_span_x = None
    ctx.check(
        "default pose is compact near bracket",
        folded_span_x is not None and folded_span_x < 0.58,
        details=f"folded_span_x={folded_span_x}",
    )

    with ctx.pose({elbow_joint: 3.0 * math.pi / 4.0, wrist_joint: -math.pi / 2.0}):
        extended_tip_aabb = ctx.part_world_aabb(link_2)
        root_extended_aabb = ctx.part_world_aabb(link_0)
        if extended_tip_aabb is not None and root_extended_aabb is not None:
            tip_reach = extended_tip_aabb[1][0]
            straight_z_span = max(extended_tip_aabb[1][2], root_extended_aabb[1][2]) - min(
                extended_tip_aabb[0][2], root_extended_aabb[0][2]
            )
        else:
            tip_reach = None
            straight_z_span = None
        ctx.check(
            "elbow and wrist extend into straight reach",
            tip_reach is not None and tip_reach > 1.05 and straight_z_span is not None and straight_z_span < 0.09,
            details=f"tip_reach={tip_reach}, straight_z_span={straight_z_span}",
        )

    return ctx.report()


object_model = build_object_model()
