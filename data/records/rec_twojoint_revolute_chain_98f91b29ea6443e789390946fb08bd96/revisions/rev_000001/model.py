from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overhead_utility_arm")

    powder_coat = Material("dark_powder_coated_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    edge_wear = Material("slightly_worn_edges", rgba=(0.23, 0.24, 0.25, 1.0))
    bolt_black = Material("black_oxide_fasteners", rgba=(0.015, 0.015, 0.014, 1.0))
    clamp_face = Material("matte_rubber_clamp_face", rgba=(0.02, 0.025, 0.025, 1.0))

    # The wall-plate frame is at the first revolute axis.  The plate itself
    # sits behind it against the wall; a boxy welded lug reaches forward to the
    # root link.  All dimensions are meter-scale and representative of a
    # compact overhead monitor/tool arm.
    wall = model.part("wall_plate")
    wall.visual(
        Box((0.040, 0.300, 0.420)),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=powder_coat,
        name="wall_backplate",
    )
    wall.visual(
        Box((0.050, 0.130, 0.090)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
        material=edge_wear,
        name="root_mount",
    )
    # Four visible lag-bolt heads embedded in the face of the backplate.
    for i, (y, z) in enumerate(
        ((-0.105, -0.155), (-0.105, 0.155), (0.105, -0.155), (0.105, 0.155))
    ):
        wall.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(-0.037, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"wall_bolt_{i}",
        )

    # The root link is intentionally short and boxy: a pivot lug, rectangular
    # tube, and elbow lug all lie in one horizontal working plane.
    root = model.part("root_link")
    root.visual(
        Box((0.060, 0.120, 0.075)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=edge_wear,
        name="inboard_lug",
    )
    root.visual(
        Box((0.310, 0.065, 0.050)),
        origin=Origin(xyz=(0.195, 0.0, 0.0)),
        material=powder_coat,
        name="short_tube",
    )
    root.visual(
        Box((0.070, 0.120, 0.075)),
        origin=Origin(xyz=(0.345, 0.0, 0.0)),
        material=edge_wear,
        name="elbow_mount",
    )
    for z, name in ((0.0405, "root_top_cap"), (-0.0405, "root_bottom_cap")):
        root.visual(
            Cylinder(radius=0.025, length=0.008),
            origin=Origin(xyz=(0.025, 0.0, z)),
            material=bolt_black,
            name=name,
        )

    # The distal link is longer and carries the terminal clamp plate as a
    # rigidly welded end effector.  Its local frame is at the elbow revolute
    # axis, so the whole link yaws in the same plane as the root link.
    distal = model.part("distal_link")
    distal.visual(
        Box((0.065, 0.115, 0.070)),
        origin=Origin(xyz=(0.0325, 0.0, 0.0)),
        material=edge_wear,
        name="elbow_lug",
    )
    distal.visual(
        Box((0.580, 0.060, 0.050)),
        origin=Origin(xyz=(0.345, 0.0, 0.0)),
        material=powder_coat,
        name="long_tube",
    )
    distal.visual(
        Box((0.075, 0.075, 0.075)),
        origin=Origin(xyz=(0.645, 0.0, 0.0)),
        material=edge_wear,
        name="clamp_adapter",
    )
    distal.visual(
        Box((0.045, 0.180, 0.140)),
        origin=Origin(xyz=(0.6975, 0.0, 0.0)),
        material=edge_wear,
        name="clamp_plate",
    )
    distal.visual(
        Box((0.006, 0.120, 0.030)),
        origin=Origin(xyz=(0.722, 0.0, 0.038)),
        material=clamp_face,
        name="upper_jaw_pad",
    )
    distal.visual(
        Box((0.006, 0.120, 0.030)),
        origin=Origin(xyz=(0.722, 0.0, -0.038)),
        material=clamp_face,
        name="lower_jaw_pad",
    )
    for i, (y, z) in enumerate(
        ((-0.060, -0.045), (-0.060, 0.045), (0.060, -0.045), (0.060, 0.045))
    ):
        distal.visual(
            Cylinder(radius=0.011, length=0.012),
            origin=Origin(xyz=(0.725, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"clamp_bolt_{i}",
        )
    for z, name in ((0.0365, "elbow_top_cap"), (-0.0365, "elbow_bottom_cap")):
        distal.visual(
            Cylinder(radius=0.025, length=0.008),
            origin=Origin(xyz=(0.0325, 0.0, z)),
            material=bolt_black,
            name=name,
        )

    joint_limits = MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.57)
    damped = MotionProperties(damping=0.18, friction=0.08)

    model.articulation(
        "wall_to_root",
        ArticulationType.REVOLUTE,
        parent=wall,
        child=root,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
        motion_properties=damped,
    )
    model.articulation(
        "root_to_distal",
        ArticulationType.REVOLUTE,
        parent=root,
        child=distal,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
        motion_properties=damped,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_plate")
    root = object_model.get_part("root_link")
    distal = object_model.get_part("distal_link")
    wall_joint = object_model.get_articulation("wall_to_root")
    elbow_joint = object_model.get_articulation("root_to_distal")

    ctx.check(
        "two serial revolute joints",
        wall_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.articulation_type == ArticulationType.REVOLUTE
        and wall_joint.axis == (0.0, 0.0, 1.0)
        and elbow_joint.axis == (0.0, 0.0, 1.0),
        details=f"wall_to_root={wall_joint.articulation_type}/{wall_joint.axis}, "
        f"root_to_distal={elbow_joint.articulation_type}/{elbow_joint.axis}",
    )

    with ctx.pose({wall_joint: 0.0, elbow_joint: 0.0}):
        ctx.expect_gap(
            root,
            wall,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="inboard_lug",
            negative_elem="root_mount",
            name="root lug bears on wall mount",
        )
        ctx.expect_overlap(
            root,
            wall,
            axes="yz",
            elem_a="inboard_lug",
            elem_b="root_mount",
            min_overlap=0.070,
            name="wall mount captures root lug footprint",
        )
        ctx.expect_gap(
            distal,
            root,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="elbow_lug",
            negative_elem="elbow_mount",
            name="distal lug bears on root elbow",
        )
        ctx.expect_overlap(
            distal,
            root,
            axes="yz",
            elem_a="elbow_lug",
            elem_b="elbow_mount",
            min_overlap=0.065,
            name="elbow mount captures distal lug footprint",
        )
        rest_elbow = ctx.part_world_position(distal)
        rest_clamp = ctx.part_element_world_aabb(distal, elem="clamp_plate")

    with ctx.pose({wall_joint: 1.0, elbow_joint: 0.0}):
        swung_elbow = ctx.part_world_position(distal)

    ctx.check(
        "root joint swings elbow in the horizontal plane",
        rest_elbow is not None
        and swung_elbow is not None
        and swung_elbow[1] > rest_elbow[1] + 0.25
        and abs(swung_elbow[2] - rest_elbow[2]) < 1.0e-6,
        details=f"rest={rest_elbow}, swung={swung_elbow}",
    )

    with ctx.pose({wall_joint: 0.0, elbow_joint: 1.0}):
        folded_clamp = ctx.part_element_world_aabb(distal, elem="clamp_plate")

    ctx.check(
        "distal joint swings terminal clamp in the same plane",
        rest_clamp is not None
        and folded_clamp is not None
        and folded_clamp[1][1] > rest_clamp[1][1] + 0.45
        and abs(folded_clamp[0][2] - rest_clamp[0][2]) < 1.0e-6,
        details=f"rest={rest_clamp}, folded={folded_clamp}",
    )

    return ctx.report()


object_model = build_object_model()
