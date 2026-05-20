from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


AXLE_RPY = (-math.pi / 2.0, 0.0, 0.0)


def _trapezoid_web(length: float, base_width: float, tip_width: float, thickness: float) -> ExtrudeGeometry:
    """Flat tapered finger web, authored in local +X with width along Y."""
    return ExtrudeGeometry(
        [
            (0.006, -base_width / 2.0),
            (length, -tip_width / 2.0),
            (length, tip_width / 2.0),
            (0.006, base_width / 2.0),
        ],
        thickness,
        center=True,
    )


def _add_study_link(
    part,
    *,
    prefix: str,
    length: float,
    base_width: float,
    tip_width: float,
    thickness: float,
    base_barrel_radius: float,
    material: Material,
    hinge_material: Material,
    has_distal_fork: bool,
) -> None:
    """Add a compact robotic-finger link with a central root barrel and a tapered web."""
    web_end = length - (0.026 if has_distal_fork else 0.006)
    part.visual(
        mesh_from_geometry(
            _trapezoid_web(web_end, base_width, tip_width, thickness),
            f"{prefix}_web",
        ),
        material=material,
        name="tapered_web",
    )
    part.visual(
        Cylinder(radius=base_barrel_radius, length=0.024),
        origin=Origin(rpy=AXLE_RPY),
        material=hinge_material,
        name="base_barrel",
    )

    if has_distal_fork:
        lug_y = 0.015
        lug_len = 0.006
        fork_radius = max(0.0085, base_barrel_radius * 0.92)
        # A transverse bridge ties the side straps back into the tapered web but
        # stops short of the child barrel, leaving a visible clevis gap.
        part.visual(
            Box((0.012, 0.030, thickness * 0.78)),
            origin=Origin(xyz=(length - 0.031, 0.0, 0.0)),
            material=material,
            name="fork_bridge",
        )
        for index, y in enumerate((-lug_y, lug_y)):
            part.visual(
                Box((0.034, lug_len, thickness * 0.80)),
                origin=Origin(xyz=(length - 0.017, y, 0.0)),
                material=material,
                name=f"side_strap_{index}",
            )
            part.visual(
                Cylinder(radius=fork_radius, length=lug_len),
                origin=Origin(xyz=(length, y, 0.0), rpy=AXLE_RPY),
                material=hinge_material,
                name=f"distal_barrel_{index}",
            )
    else:
        part.visual(
            Cylinder(radius=max(0.0065, tip_width * 0.58), length=0.014),
            origin=Origin(xyz=(length, 0.0, 0.0), rpy=AXLE_RPY),
            material=material,
            name="rounded_tip",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parallel_finger_study")

    palm_mat = model.material("matte_black_palm", rgba=(0.035, 0.038, 0.042, 1.0))
    edge_mat = model.material("dark_anodized_edges", rgba=(0.12, 0.13, 0.14, 1.0))
    hinge_mat = model.material("brushed_steel_hinges", rgba=(0.68, 0.70, 0.72, 1.0))
    link_blue = model.material("blue_linkage", rgba=(0.12, 0.32, 0.82, 1.0))
    link_teal = model.material("teal_linkage", rgba=(0.10, 0.62, 0.72, 1.0))
    screw_mat = model.material("black_screw_heads", rgba=(0.005, 0.005, 0.006, 1.0))

    root_x = 0.080
    root_z = 0.052
    finger_ys = (-0.040, 0.040)

    palm = model.part("palm")
    palm.visual(
        Box((0.160, 0.170, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=palm_mat,
        name="central_plate",
    )
    palm.visual(
        Box((0.030, 0.136, 0.006)),
        origin=Origin(xyz=(0.043, 0.0, 0.027)),
        material=edge_mat,
        name="front_reinforcement",
    )

    for i, (sx, sy) in enumerate(((-0.050, -0.060), (-0.050, 0.060), (0.030, -0.060), (0.030, 0.060))):
        palm.visual(
            Cylinder(radius=0.0055, length=0.003),
            origin=Origin(xyz=(sx, sy, 0.0255)),
            material=screw_mat,
            name=f"screw_head_{i}",
        )

    # Two fixed root clevises are built into the same grounded palm plate.  Each
    # has separated side barrels so the moving proximal base barrel has visible
    # clearance instead of being hidden in an overlapping hinge proxy.
    for finger_index, y0 in enumerate(finger_ys):
        palm.visual(
            Box((0.046, 0.040, 0.010)),
            origin=Origin(xyz=(0.065, y0, 0.029)),
            material=edge_mat,
            name=f"root_pedestal_{finger_index}",
        )
        for side_index, side_y in enumerate((-0.015, 0.015)):
            palm.visual(
                Box((0.030, 0.006, 0.030)),
                origin=Origin(xyz=(root_x, y0 + side_y, 0.038)),
                material=edge_mat,
                name=f"root_cheek_{finger_index}_{side_index}",
            )
            palm.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(root_x, y0 + side_y, root_z), rpy=AXLE_RPY),
                material=hinge_mat,
                name=f"root_barrel_{finger_index}_{side_index}",
            )

    link_specs = (
        ("proximal", 0.083, 0.020, 0.016, 0.0110),
        ("middle", 0.066, 0.017, 0.013, 0.0098),
        ("distal", 0.050, 0.014, 0.008, 0.0085),
    )

    for finger_index, y0 in enumerate(finger_ys):
        link_mat = link_blue if finger_index == 0 else link_teal
        proximal = model.part(f"finger_{finger_index}_proximal")
        middle = model.part(f"finger_{finger_index}_middle")
        distal = model.part(f"finger_{finger_index}_distal")

        _add_study_link(
            proximal,
            prefix=f"finger_{finger_index}_proximal",
            length=link_specs[0][1],
            base_width=link_specs[0][2],
            tip_width=link_specs[0][3],
            thickness=0.014,
            base_barrel_radius=link_specs[0][4],
            material=link_mat,
            hinge_material=hinge_mat,
            has_distal_fork=True,
        )
        _add_study_link(
            middle,
            prefix=f"finger_{finger_index}_middle",
            length=link_specs[1][1],
            base_width=link_specs[1][2],
            tip_width=link_specs[1][3],
            thickness=0.012,
            base_barrel_radius=link_specs[1][4],
            material=link_mat,
            hinge_material=hinge_mat,
            has_distal_fork=True,
        )
        _add_study_link(
            distal,
            prefix=f"finger_{finger_index}_distal",
            length=link_specs[2][1],
            base_width=link_specs[2][2],
            tip_width=link_specs[2][3],
            thickness=0.010,
            base_barrel_radius=link_specs[2][4],
            material=link_mat,
            hinge_material=hinge_mat,
            has_distal_fork=False,
        )

        model.articulation(
            f"finger_{finger_index}_root_joint",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(root_x, y0, root_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=-0.12, upper=1.15),
        )
        model.articulation(
            f"finger_{finger_index}_middle_joint",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(link_specs[0][1], 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=4.0, lower=0.0, upper=1.25),
        )
        model.articulation(
            f"finger_{finger_index}_tip_joint",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(link_specs[1][1], 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=4.5, lower=0.0, upper=1.15),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "two independent three joint chains",
        len(revolute_joints) == 6,
        details=f"revolute joints={len(revolute_joints)}",
    )

    for finger_index in (0, 1):
        proximal = object_model.get_part(f"finger_{finger_index}_proximal")
        middle = object_model.get_part(f"finger_{finger_index}_middle")
        distal = object_model.get_part(f"finger_{finger_index}_distal")
        root_joint = object_model.get_articulation(f"finger_{finger_index}_root_joint")
        middle_joint = object_model.get_articulation(f"finger_{finger_index}_middle_joint")
        tip_joint = object_model.get_articulation(f"finger_{finger_index}_tip_joint")

        ctx.expect_overlap(
            proximal,
            middle,
            axes="xz",
            elem_a="distal_barrel_0",
            elem_b="base_barrel",
            min_overlap=0.005,
            name=f"finger {finger_index} middle joint barrels share an axle line",
        )
        ctx.expect_overlap(
            middle,
            distal,
            axes="xz",
            elem_a="distal_barrel_0",
            elem_b="base_barrel",
            min_overlap=0.004,
            name=f"finger {finger_index} tip joint barrels share an axle line",
        )

        rest_distal_pos = ctx.part_world_position(distal)
        other_distal = object_model.get_part(f"finger_{1 - finger_index}_distal")
        other_rest = ctx.part_world_position(other_distal)
        with ctx.pose({root_joint: 0.55, middle_joint: 0.35, tip_joint: 0.20}):
            flexed_distal_pos = ctx.part_world_position(distal)
            other_flexed = ctx.part_world_position(other_distal)
        ctx.check(
            f"finger {finger_index} flexes upward independently",
            rest_distal_pos is not None
            and flexed_distal_pos is not None
            and other_rest is not None
            and other_flexed is not None
            and flexed_distal_pos[2] > rest_distal_pos[2] + 0.020
            and abs(other_flexed[2] - other_rest[2]) < 0.001,
            details=f"rest={rest_distal_pos}, flexed={flexed_distal_pos}, other={other_rest}->{other_flexed}",
        )

    return ctx.report()


object_model = build_object_model()
