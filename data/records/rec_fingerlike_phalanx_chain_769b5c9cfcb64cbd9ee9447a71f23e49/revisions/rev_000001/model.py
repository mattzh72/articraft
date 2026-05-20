from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


PIN_RPY = (-math.pi / 2.0, 0.0, 0.0)


def _loft_section(x_pos: float, width_y: float, height_z: float, *, segments: int = 32):
    # LoftGeometry expects each closed section to lie in the local XY plane with
    # a constant local Z.  Author height along local X and width along local Y,
    # then rotate the completed loft so local Z becomes the link's +X axis.
    return [(h, y, x_pos) for (h, y) in superellipse_profile(height_z, width_y, 3.1, segments=segments)]


def _tapered_core_mesh(
    length: float,
    proximal_width: float,
    distal_width: float,
    proximal_height: float,
    distal_height: float,
    name: str,
):
    geom = LoftGeometry(
        [
            _loft_section(0.014, proximal_width, proximal_height),
            _loft_section(length - 0.050, distal_width, distal_height),
        ],
        cap=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _add_pin_heads(part, *, x: float, y_outer: float, radius: float, material: Material, name: str, z: float = 0.0) -> None:
    # Two proud metal caps sit on the outside faces of the clevis cheeks.  They
    # read as a through-pin without forcing a separate overlapping pin link.
    part.visual(
        Cylinder(radius=radius, length=0.006),
        origin=Origin(xyz=(x, y_outer + 0.0028, z), rpy=PIN_RPY),
        material=material,
        name=f"{name}_pin_head_0",
    )
    part.visual(
        Cylinder(radius=radius, length=0.006),
        origin=Origin(xyz=(x, -y_outer - 0.0028, z), rpy=PIN_RPY),
        material=material,
        name=f"{name}_pin_head_1",
    )


def _add_pin_shaft(
    part,
    *,
    x: float,
    length: float,
    radius: float,
    material: Material,
    visual_name: str,
    z: float = 0.0,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, 0.0, z), rpy=PIN_RPY),
        material=material,
        name=visual_name,
    )


def _build_phalanx(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    total_width: float,
    proximal_height: float,
    distal_height: float,
    body_material: Material,
    cheek_material: Material,
    pin_material: Material,
    has_distal_fork: bool,
    fingertip_material: Material | None = None,
):
    part = model.part(name)

    inner_width = total_width * 0.50
    cheek_thickness = 0.012
    cheek_y = total_width * 0.42
    y_outer = cheek_y + cheek_thickness / 2.0

    # A rounded, tapered middle member gives each phalanx a finger-like
    # silhouette while leaving the hinge regions mechanically distinct.
    part.visual(
        _tapered_core_mesh(
            length,
            proximal_width=inner_width,
            distal_width=inner_width * 0.82,
            proximal_height=proximal_height * 0.70,
            distal_height=distal_height * 0.70,
            name=f"{name}_tapered_core",
        ),
        material=body_material,
        name="tapered_core",
    )

    # The proximal link tongue is the center plate captured between the parent
    # cheeks.  Its round lug is intentionally part of the moving phalanx.
    part.visual(
        Cylinder(radius=proximal_height * 0.39, length=inner_width * 0.86),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=PIN_RPY),
        material=body_material,
        name="proximal_lug",
    )
    part.visual(
        Box((0.036, inner_width * 0.82, proximal_height * 0.42)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=body_material,
        name="lug_neck",
    )

    if has_distal_fork:
        cheek_length = 0.074
        cheek_x = length - 0.010
        for idx, y in enumerate((cheek_y, -cheek_y)):
            part.visual(
                Box((cheek_length, cheek_thickness, distal_height)),
                origin=Origin(xyz=(cheek_x, y, 0.0)),
                material=cheek_material,
                name=f"distal_cheek_{idx}",
            )
        part.visual(
            Box((0.024, total_width, distal_height * 0.48)),
            origin=Origin(xyz=(length - 0.051, 0.0, 0.0)),
            material=cheek_material,
            name="distal_bridge",
        )
        _add_pin_heads(
            part,
            x=length,
            y_outer=y_outer,
            radius=distal_height * 0.34,
            material=pin_material,
            name="distal",
        )
        _add_pin_shaft(
            part,
            x=length,
            length=total_width + 0.014,
            radius=0.0055,
            material=pin_material,
            visual_name="distal_pin_shaft",
        )
    else:
        # The terminal phalanx finishes in a blunt soft pad rather than another
        # fork.  The pad overlaps its own core slightly so the part is one
        # supported assembly.
        part.visual(
            Box((0.038, inner_width * 0.75, distal_height * 0.44)),
            origin=Origin(xyz=(length - 0.030, 0.0, 0.0)),
            material=body_material,
            name="tip_neck",
        )
        part.visual(
            mesh_from_geometry(CapsuleGeometry(radius=distal_height * 0.58, length=0.030), f"{name}_tip_pad"),
            origin=Origin(xyz=(length + 0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fingertip_material or body_material,
            name="tip_pad",
        )

    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="finger_phalanx_chain")

    block_mat = model.material("anodized_knuckle_block", rgba=(0.12, 0.13, 0.15, 1.0))
    body_mat = model.material("warm_phalanx_composite", rgba=(0.78, 0.70, 0.58, 1.0))
    cheek_mat = model.material("dark_side_cheeks", rgba=(0.19, 0.20, 0.23, 1.0))
    pin_mat = model.material("brushed_pin_metal", rgba=(0.72, 0.72, 0.68, 1.0))
    pad_mat = model.material("matte_fingertip_pad", rgba=(0.22, 0.18, 0.16, 1.0))

    knuckle = model.part("knuckle")
    knuckle.visual(
        Box((0.135, 0.105, 0.070)),
        origin=Origin(xyz=(0.018, 0.0, 0.035)),
        material=block_mat,
        name="ground_block",
    )
    knuckle.visual(
        Box((0.160, 0.125, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.006)),
        material=block_mat,
        name="ground_foot",
    )
    joint_x = 0.075
    joint_z = 0.092
    for idx, y in enumerate((0.030, -0.030)):
        knuckle.visual(
            Box((0.065, 0.014, 0.052)),
            origin=Origin(xyz=(joint_x, y, joint_z)),
            material=cheek_mat,
            name=f"base_cheek_{idx}",
        )
    knuckle.visual(
        Box((0.026, 0.074, 0.028)),
        origin=Origin(xyz=(joint_x - 0.036, 0.0, joint_z - 0.018)),
        material=cheek_mat,
        name="base_cheek_bridge",
    )
    _add_pin_heads(
        knuckle,
        x=joint_x,
        y_outer=0.037,
        radius=0.017,
        material=pin_mat,
        name="base",
        z=joint_z,
    )
    _add_pin_shaft(
        knuckle,
        x=joint_x,
        length=0.084,
        radius=0.006,
        material=pin_mat,
        visual_name="base_pin_shaft",
        z=joint_z,
    )

    proximal = _build_phalanx(
        model,
        name="proximal_phalanx",
        length=0.160,
        total_width=0.060,
        proximal_height=0.048,
        distal_height=0.041,
        body_material=body_mat,
        cheek_material=cheek_mat,
        pin_material=pin_mat,
        has_distal_fork=True,
    )
    middle = _build_phalanx(
        model,
        name="middle_phalanx",
        length=0.125,
        total_width=0.054,
        proximal_height=0.041,
        distal_height=0.034,
        body_material=body_mat,
        cheek_material=cheek_mat,
        pin_material=pin_mat,
        has_distal_fork=True,
    )
    distal = _build_phalanx(
        model,
        name="distal_phalanx",
        length=0.095,
        total_width=0.047,
        proximal_height=0.034,
        distal_height=0.030,
        body_material=body_mat,
        cheek_material=cheek_mat,
        pin_material=pin_mat,
        has_distal_fork=False,
        fingertip_material=pad_mat,
    )

    bend_limits = MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.25)
    model.articulation(
        "knuckle_to_proximal",
        ArticulationType.REVOLUTE,
        parent=knuckle,
        child=proximal,
        origin=Origin(xyz=(joint_x, 0.0, joint_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=bend_limits,
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("knuckle_to_proximal"),
        object_model.get_articulation("proximal_to_middle"),
        object_model.get_articulation("middle_to_distal"),
    ]
    pin_reason = "A visible hinge shaft is intentionally captured inside the phalanx lug to model the pin joint."
    ctx.allow_overlap(
        "proximal_phalanx",
        "knuckle",
        elem_a="proximal_lug",
        elem_b="base_pin_shaft",
        reason=pin_reason,
    )
    ctx.allow_overlap(
        "middle_phalanx",
        "proximal_phalanx",
        elem_a="proximal_lug",
        elem_b="distal_pin_shaft",
        reason=pin_reason,
    )
    ctx.allow_overlap(
        "distal_phalanx",
        "middle_phalanx",
        elem_a="proximal_lug",
        elem_b="distal_pin_shaft",
        reason=pin_reason,
    )
    ctx.expect_overlap(
        "proximal_phalanx",
        "knuckle",
        axes="xz",
        elem_a="proximal_lug",
        elem_b="base_pin_shaft",
        min_overlap=0.005,
        name="base pin passes through proximal lug",
    )
    ctx.expect_overlap(
        "middle_phalanx",
        "proximal_phalanx",
        axes="xz",
        elem_a="proximal_lug",
        elem_b="distal_pin_shaft",
        min_overlap=0.005,
        name="proximal distal pin passes through middle lug",
    )
    ctx.expect_overlap(
        "distal_phalanx",
        "middle_phalanx",
        axes="xz",
        elem_a="proximal_lug",
        elem_b="distal_pin_shaft",
        min_overlap=0.005,
        name="middle distal pin passes through distal lug",
    )
    ctx.check(
        "three serial revolute joints",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=str([j.name for j in joints]),
    )

    distal = object_model.get_part("distal_phalanx")
    rest_tip = ctx.part_world_aabb(distal)
    with ctx.pose(
        {
            joints[0]: 0.65,
            joints[1]: 0.55,
            joints[2]: 0.45,
        }
    ):
        bent_tip = ctx.part_world_aabb(distal)
        bent_positions = [ctx.part_world_position(object_model.get_part(name)) for name in ("proximal_phalanx", "middle_phalanx", "distal_phalanx")]

    ctx.check(
        "positive joint motion curls the finger upward",
        rest_tip is not None
        and bent_tip is not None
        and bent_tip[1][2] > rest_tip[1][2] + 0.10
        and bent_tip[1][0] < rest_tip[1][0],
        details=f"rest={rest_tip}, bent={bent_tip}",
    )
    ctx.check(
        "bending remains in the central plane",
        all(pos is not None and abs(pos[1]) < 1e-6 for pos in bent_positions),
        details=str(bent_positions),
    )

    return ctx.report()


object_model = build_object_model()
