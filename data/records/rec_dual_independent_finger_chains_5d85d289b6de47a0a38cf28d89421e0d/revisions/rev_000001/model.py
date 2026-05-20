from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


PALM_FRONT_Y = 0.040
ROOT_Y = 0.062
ROOT_Z = 0.063
ROOT_XS = (-0.045, 0.045)

PROXIMAL_LEN = 0.062
MIDDLE_LEN = 0.052
DISTAL_LEN = 0.044

LINK_WIDTH = 0.023
LINK_THICKNESS = 0.017
KNUCKLE_RADIUS = 0.011
KNUCKLE_LEN = 0.026
YOKE_OUTER_WIDTH = 0.038
YOKE_PLATE_THICKNESS = 0.006
YOKE_PLATE_OFFSET = 0.016

PROXIMAL_KNUCKLE = "proximal_knuckle"
PROXIMAL_FORK_BRIDGE = "proximal_fork_bridge"
MIDDLE_KNUCKLE = "middle_knuckle"
MIDDLE_FORK_BRIDGE = "middle_fork_bridge"
DISTAL_KNUCKLE = "distal_knuckle"


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _add_cylinder_x(part, *, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_distal_yoke(part, *, length: float, material, prefix: str) -> None:
    """A compact U-bracket at the distal end of a phalanx link."""
    plate_size = (YOKE_PLATE_THICKNESS, 0.034, 0.026)
    for side, x in (("a", -YOKE_PLATE_OFFSET), ("b", YOKE_PLATE_OFFSET)):
        part.visual(
            Box(plate_size),
            origin=Origin(xyz=(x, length - 0.005, 0.0)),
            material=material,
            name=f"{prefix}_fork_{side}",
        )
    part.visual(
        Box((YOKE_OUTER_WIDTH, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, length - 0.023, 0.0)),
        material=material,
        name=f"{prefix}_fork_bridge",
    )


def _add_link(part, *, length: float, has_yoke: bool, link_material, metal, pad=None, prefix: str) -> None:
    knuckle_name = {
        "proximal": PROXIMAL_KNUCKLE,
        "middle": MIDDLE_KNUCKLE,
        "distal": DISTAL_KNUCKLE,
    }[prefix]
    _add_cylinder_x(
        part,
        radius=KNUCKLE_RADIUS,
        length=KNUCKLE_LEN,
        xyz=(0.0, 0.0, 0.0),
        material=metal,
        name=knuckle_name,
    )
    beam_len = length - (0.030 if has_yoke else 0.010)
    beam_center = 0.010 + beam_len / 2.0
    part.visual(
        Box((LINK_WIDTH, beam_len, LINK_THICKNESS)),
        origin=Origin(xyz=(0.0, beam_center, 0.0)),
        material=link_material,
        name=f"{prefix}_beam",
    )
    if has_yoke:
        _add_distal_yoke(part, length=length, material=link_material, prefix=prefix)
    else:
        part.visual(
            Box((LINK_WIDTH * 0.95, 0.022, LINK_THICKNESS * 1.15)),
            origin=Origin(xyz=(0.0, length + 0.003, -0.001)),
            material=pad,
            name=f"{prefix}_rubber_tip",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_palm_two_finger_chain")

    palm_mat = model.material("powder_coated_palm", rgba=(0.08, 0.09, 0.10, 1.0))
    bracket_mat = model.material("dark_mounts", rgba=(0.025, 0.027, 0.030, 1.0))
    link_mat = model.material("anodized_link", rgba=(0.18, 0.22, 0.27, 1.0))
    metal_mat = model.material("brushed_steel", rgba=(0.66, 0.67, 0.64, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.008, 0.008, 0.007, 1.0))

    palm = model.part("palm")
    palm_shell = (
        cq.Workplane("XY")
        .box(0.180, 0.080, 0.110)
        .edges()
        .fillet(0.006)
    )
    palm.visual(
        mesh_from_cadquery(palm_shell, "rounded_palm_box", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=palm_mat,
        name="palm_box",
    )

    # Two clearly separate root clevises mounted to the front face of the boxed palm.
    for root_index, root_x in enumerate(ROOT_XS):
        for side, x_offset in (("a", -YOKE_PLATE_OFFSET), ("b", YOKE_PLATE_OFFSET)):
            palm.visual(
                Box((YOKE_PLATE_THICKNESS, 0.042, 0.050)),
                origin=Origin(xyz=(root_x + x_offset, 0.057, ROOT_Z)),
                material=bracket_mat,
                name=f"root_{root_index}_cheek_{side}",
            )
        palm.visual(
            Box((YOKE_OUTER_WIDTH, 0.010, 0.030)),
            origin=Origin(xyz=(root_x, PALM_FRONT_Y + 0.002, ROOT_Z)),
            material=bracket_mat,
            name=f"root_{root_index}_mount_pad",
        )
        for side, x_offset in (("a", -YOKE_PLATE_OFFSET), ("b", YOKE_PLATE_OFFSET)):
            _add_cylinder_x(
                palm,
                radius=0.0045,
                length=YOKE_PLATE_THICKNESS,
                xyz=(root_x + x_offset, ROOT_Y, ROOT_Z),
                material=metal_mat,
                name=f"root_{root_index}_pin_head_{side}",
            )

    # Small countersunk screw heads make the body read as a compact boxed module.
    for x in (-0.072, 0.072):
        for z in (0.028, 0.092):
            palm.visual(
                Cylinder(radius=0.0055, length=0.003),
                origin=Origin(xyz=(x, PALM_FRONT_Y + 0.0015, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=metal_mat,
                name=f"front_screw_{x:+.3f}_{z:.3f}",
            )

    for finger_index, root_x in enumerate(ROOT_XS):
        proximal = model.part(f"finger_{finger_index}_proximal")
        middle = model.part(f"finger_{finger_index}_middle")
        distal = model.part(f"finger_{finger_index}_distal")

        _add_link(
            proximal,
            length=PROXIMAL_LEN,
            has_yoke=True,
            link_material=link_mat,
            metal=metal_mat,
            prefix="proximal",
        )
        _add_link(
            middle,
            length=MIDDLE_LEN,
            has_yoke=True,
            link_material=link_mat,
            metal=metal_mat,
            prefix="middle",
        )
        _add_link(
            distal,
            length=DISTAL_LEN,
            has_yoke=False,
            link_material=link_mat,
            metal=metal_mat,
            pad=rubber_mat,
            prefix="distal",
        )

        model.articulation(
            f"finger_{finger_index}_root",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(root_x, ROOT_Y, ROOT_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=0.95),
        )
        model.articulation(
            f"finger_{finger_index}_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, PROXIMAL_LEN, 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=3.5, lower=0.0, upper=0.85),
        )
        model.articulation(
            f"finger_{finger_index}_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, MIDDLE_LEN, 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=0.0, upper=0.70),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    articulations = object_model.articulations
    ctx.check(
        "two fingers have six independent revolute joints",
        len(articulations) == 6
        and all(a.articulation_type == ArticulationType.REVOLUTE for a in articulations)
        and all(getattr(a, "mimic", None) is None for a in articulations),
        details=f"articulations={[a.name for a in articulations]}",
    )

    for finger_index, root_x in enumerate(ROOT_XS):
        proximal = object_model.get_part(f"finger_{finger_index}_proximal")
        middle = object_model.get_part(f"finger_{finger_index}_middle")
        distal = object_model.get_part(f"finger_{finger_index}_distal")

        ctx.expect_within(
            proximal,
            "palm",
            axes="x",
            inner_elem=PROXIMAL_KNUCKLE,
            outer_elem=f"root_{finger_index}_mount_pad",
            margin=0.001,
            name=f"finger {finger_index} root knuckle sits in its own palm clevis",
        )
        ctx.expect_overlap(
            proximal,
            middle,
            axes="x",
            elem_a=PROXIMAL_FORK_BRIDGE,
            elem_b=MIDDLE_KNUCKLE,
            min_overlap=0.020,
            name=f"finger {finger_index} proximal yoke captures middle knuckle width",
        )
        ctx.expect_overlap(
            middle,
            distal,
            axes="x",
            elem_a=MIDDLE_FORK_BRIDGE,
            elem_b=DISTAL_KNUCKLE,
            min_overlap=0.020,
            name=f"finger {finger_index} middle yoke captures distal knuckle width",
        )

        root = object_model.get_articulation(f"finger_{finger_index}_root")
        mid = object_model.get_articulation(f"finger_{finger_index}_middle")
        dist = object_model.get_articulation(f"finger_{finger_index}_distal")
        rest_tip = ctx.part_world_position(distal)
        with ctx.pose({root: 0.75, mid: 0.55, dist: 0.35}):
            curled_tip = ctx.part_world_position(distal)
        ctx.check(
            f"finger {finger_index} curls downward when actuated",
            rest_tip is not None and curled_tip is not None and curled_tip[2] < rest_tip[2] - 0.020,
            details=f"rest={rest_tip}, curled={curled_tip}",
        )

    other_root = object_model.get_articulation("finger_1_root")
    other_mid = object_model.get_articulation("finger_1_middle")
    other_dist = object_model.get_articulation("finger_1_distal")
    stationary_before = ctx.part_world_position("finger_0_distal")
    with ctx.pose({other_root: 0.7, other_mid: 0.4, other_dist: 0.3}):
        stationary_after = ctx.part_world_position("finger_0_distal")
    ctx.check(
        "actuating one finger does not drive the other",
        stationary_before is not None
        and stationary_after is not None
        and max(abs(a - b) for a, b in zip(stationary_before, stationary_after)) < 1e-6,
        details=f"before={stationary_before}, after={stationary_after}",
    )

    return ctx.report()


object_model = build_object_model()
