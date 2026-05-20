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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_elbow_arm")

    dark_frame = Material("dark_powder_coated_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    link_blue = Material("satin_blue_link_casting", rgba=(0.08, 0.22, 0.45, 1.0))
    link_teal = Material("satin_teal_forearm", rgba=(0.08, 0.38, 0.40, 1.0))
    pin_steel = Material("brushed_pin_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    plate_black = Material("black_end_plate", rgba=(0.02, 0.02, 0.022, 1.0))

    rear_frame = model.part("rear_frame")
    # A grounded rear frame with a raised bridge carrying an open shoulder saddle.
    rear_frame.visual(
        Box((0.36, 0.38, 0.06)),
        origin=Origin(xyz=(-0.15, 0.0, 0.03)),
        material=dark_frame,
        name="base_foot",
    )
    rear_frame.visual(
        Box((0.10, 0.30, 1.16)),
        origin=Origin(xyz=(-0.23, 0.0, 0.58)),
        material=dark_frame,
        name="rear_backbone",
    )
    rear_frame.visual(
        Box((0.28, 0.28, 0.07)),
        origin=Origin(xyz=(-0.095, 0.0, 1.205)),
        material=dark_frame,
        name="top_bridge",
    )
    rear_frame.visual(
        Box((0.20, 0.035, 0.25)),
        origin=Origin(xyz=(0.02, 0.115, 1.05)),
        material=dark_frame,
        name="saddle_cheek_0",
    )
    rear_frame.visual(
        Box((0.20, 0.035, 0.25)),
        origin=Origin(xyz=(0.02, -0.115, 1.05)),
        material=dark_frame,
        name="saddle_cheek_1",
    )
    rear_frame.visual(
        Box((0.05, 0.265, 0.075)),
        origin=Origin(xyz=(-0.10, 0.0, 1.005)),
        material=dark_frame,
        name="saddle_rear_tie",
    )
    for i, y in enumerate((-0.115, 0.115)):
        # Diagonal bridge tubes make the saddle read as rear-backed, not floating.
        rear_frame.visual(
            Cylinder(radius=0.018, length=0.45),
            origin=Origin(
                xyz=(-0.115, y, 0.935),
                rpy=(0.0, math.atan2(0.19, 0.41), 0.0),
            ),
            material=dark_frame,
            name=f"diagonal_bridge_{i}",
        )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Box((0.47, 0.082, 0.055)),
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        material=link_blue,
        name="upper_web",
    )
    upper_link.visual(
        Cylinder(radius=0.074, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=link_blue,
        name="shoulder_boss",
    )
    upper_link.visual(
        Cylinder(radius=0.036, length=0.195),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="shoulder_pin_cap",
    )
    for i, y in enumerate((-0.058, 0.058)):
        upper_link.visual(
            Box((0.18, 0.038, 0.060)),
            origin=Origin(xyz=(0.555, y, 0.0)),
            material=link_blue,
            name=("elbow_fork_cheek_0", "elbow_fork_cheek_1")[i],
        )
        upper_link.visual(
            Cylinder(radius=0.070, length=0.038),
            origin=Origin(xyz=(0.62, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=link_blue,
            name=("elbow_boss_0", "elbow_boss_1")[i],
        )
    upper_link.visual(
        Box((0.34, 0.050, 0.040)),
        origin=Origin(xyz=(0.320, 0.0, 0.055)),
        material=link_blue,
        name="raised_back_bridge",
    )
    upper_link.visual(
        Box((0.045, 0.050, 0.095)),
        origin=Origin(xyz=(0.165, 0.0, 0.030)),
        material=link_blue,
        name="bridge_post_0",
    )
    upper_link.visual(
        Box((0.045, 0.050, 0.095)),
        origin=Origin(xyz=(0.475, 0.0, 0.030)),
        material=link_blue,
        name="bridge_post_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.058, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=link_teal,
        name="forearm_boss",
    )
    forearm.visual(
        Cylinder(radius=0.028, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="elbow_pin_cap",
    )
    forearm.visual(
        Box((0.43, 0.060, 0.048)),
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        material=link_teal,
        name="forearm_web",
    )
    forearm.visual(
        Box((0.045, 0.185, 0.185)),
        origin=Origin(xyz=(0.485, 0.0, 0.0)),
        material=plate_black,
        name="end_plate",
    )
    for row, z in enumerate((-0.062, 0.062)):
        for col, y in enumerate((-0.062, 0.062)):
            forearm.visual(
                Cylinder(radius=0.012, length=0.018),
                origin=Origin(xyz=(0.514, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=pin_steel,
                name=f"plate_bolt_{row}_{col}",
            )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=-0.65, upper=1.15),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(0.62, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-0.15, upper=2.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.check(
        "shoulder and elbow are parallel revolute joints",
        shoulder.axis == (0.0, -1.0, 0.0) and elbow.axis == (0.0, -1.0, 0.0),
        details=f"shoulder_axis={shoulder.axis}, elbow_axis={elbow.axis}",
    )
    for boss_name in ("elbow_boss_0", "elbow_boss_1"):
        ctx.allow_overlap(
            upper_link,
            forearm,
            elem_a=boss_name,
            elem_b="elbow_pin_cap",
            reason="The steel elbow pin is intentionally captured inside the fork boss bore.",
        )
        ctx.expect_overlap(
            upper_link,
            forearm,
            axes="y",
            elem_a=boss_name,
            elem_b="elbow_pin_cap",
            min_overlap=0.018,
            name=f"{boss_name} captures the elbow pin along its bore",
        )
    for cheek_name in ("elbow_fork_cheek_0", "elbow_fork_cheek_1"):
        ctx.allow_overlap(
            upper_link,
            forearm,
            elem_a=cheek_name,
            elem_b="elbow_pin_cap",
            reason="The simplified fork cheek is pierced by the elbow pin bore.",
        )
        ctx.expect_overlap(
            upper_link,
            forearm,
            axes="y",
            elem_a=cheek_name,
            elem_b="elbow_pin_cap",
            min_overlap=0.018,
            name=f"{cheek_name} is pinned through its cheek plate",
        )
    ctx.expect_gap(
        rear_frame,
        upper_link,
        axis="y",
        positive_elem="saddle_cheek_0",
        negative_elem="shoulder_boss",
        min_gap=0.035,
        max_gap=0.060,
        name="shoulder boss is centered inside the saddle gap",
    )
    ctx.expect_contact(
        rear_frame,
        upper_link,
        elem_a="saddle_cheek_0",
        elem_b="shoulder_pin_cap",
        contact_tol=0.0005,
        name="shoulder pin bears on one saddle cheek",
    )
    ctx.expect_contact(
        rear_frame,
        upper_link,
        elem_a="saddle_cheek_1",
        elem_b="shoulder_pin_cap",
        contact_tol=0.0005,
        name="shoulder pin bears on the opposite saddle cheek",
    )
    ctx.expect_gap(
        upper_link,
        forearm,
        axis="y",
        positive_elem="elbow_fork_cheek_1",
        negative_elem="forearm_boss",
        min_gap=0.006,
        max_gap=0.030,
        name="forearm lug clears the elbow fork cheek",
    )
    ctx.expect_overlap(
        forearm,
        forearm,
        axes="yz",
        elem_a="end_plate",
        elem_b="forearm_web",
        min_overlap=0.045,
        name="square end plate is carried by the forearm web",
    )

    rest_elbow_position = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.65}):
        raised_elbow_position = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder raises the elbow in the xz plane",
        rest_elbow_position is not None
        and raised_elbow_position is not None
        and raised_elbow_position[2] > rest_elbow_position[2] + 0.25
        and abs(raised_elbow_position[1] - rest_elbow_position[1]) < 0.002,
        details=f"rest={rest_elbow_position}, raised={raised_elbow_position}",
    )

    def _z_center(aabb):
        return 0.5 * (aabb[0][2] + aabb[1][2]) if aabb is not None else None

    rest_plate_aabb = ctx.part_element_world_aabb(forearm, elem="end_plate")
    with ctx.pose({elbow: 1.0}):
        bent_plate_aabb = ctx.part_element_world_aabb(forearm, elem="end_plate")
    rest_plate_z = _z_center(rest_plate_aabb)
    bent_plate_z = _z_center(bent_plate_aabb)
    ctx.check(
        "elbow bends the end plate in the same vertical plane",
        rest_plate_z is not None
        and bent_plate_z is not None
        and bent_plate_z > rest_plate_z + 0.25,
        details=f"rest_plate_z={rest_plate_z}, bent_plate_z={bent_plate_z}",
    )

    return ctx.report()


object_model = build_object_model()
