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
    Sphere,
    TestContext,
    TestReport,
)


FLEX_AXIS = (-1.0, 0.0, 0.0)
SPLAY_AXIS = (0.0, 0.0, 1.0)


def _cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder aligned along local X."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _finger_materials(model: ArticulatedObject) -> dict[str, Material]:
    return {
        "palm": model.material("warm_gunmetal", rgba=(0.42, 0.44, 0.46, 1.0)),
        "link": model.material("brushed_aluminum", rgba=(0.66, 0.68, 0.70, 1.0)),
        "joint": model.material("dark_bearing", rgba=(0.055, 0.060, 0.065, 1.0)),
        "rubber": model.material("matte_rubber", rgba=(0.015, 0.017, 0.018, 1.0)),
        "accent": model.material("blue_axis_mark", rgba=(0.05, 0.22, 0.85, 1.0)),
    }


def _add_distal_clevis(
    part,
    *,
    length: float,
    width: float,
    thickness: float,
    radius: float,
    z: float = 0.0,
    material: Material,
    joint_material: Material,
) -> tuple[float, float, float]:
    """Add two outside hinge barrels and the side straps that capture a child tongue."""
    central_len = width * 0.56
    outer_len = max(width * 0.18, 0.0024)
    side_gap = 0.0012
    outer_center = central_len / 2.0 + side_gap + outer_len / 2.0
    outer_span = 2.0 * outer_center + outer_len

    cyl, base_origin = _cyl_x(radius, outer_len)
    pin_cyl, pin_origin = _cyl_x(radius * 0.30, outer_span)
    part.visual(
        pin_cyl,
        origin=Origin(xyz=(0.0, length, z), rpy=pin_origin.rpy),
        material=joint_material,
        name="distal_pin",
    )
    for sign, suffix in ((-1.0, "a"), (1.0, "b")):
        part.visual(
            cyl,
            origin=Origin(
                xyz=(sign * outer_center, length, z),
                rpy=base_origin.rpy,
            ),
            material=joint_material,
            name=f"distal_barrel_{suffix}",
        )
        part.visual(
            Box((outer_len, radius * 1.55, thickness * 0.56)),
            origin=Origin(xyz=(sign * outer_center, length - radius * 0.70, z)),
            material=material,
            name=f"distal_strap_{suffix}",
        )

    part.visual(
        Box((outer_span, radius * 0.80, thickness * 0.45)),
        origin=Origin(xyz=(0.0, length - radius * 1.65, z)),
        material=material,
        name="distal_bridge",
    )
    return central_len, outer_len, outer_span


def _add_base_knuckle(
    part,
    *,
    length: float,
    width: float,
    thickness: float,
    hinge_z: float,
    material: Material,
    joint_material: Material,
    accent_material: Material,
) -> None:
    radius = thickness * 0.45
    disk_radius = width * 0.60

    part.visual(
        Cylinder(radius=disk_radius, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=joint_material,
        name="splay_turntable",
    )
    part.visual(
        Cylinder(radius=disk_radius * 0.60, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=accent_material,
        name="axis_cap",
    )
    part.visual(
        Box((width * 0.56, max(length - radius * 1.5, 0.006), thickness * 0.62)),
        origin=Origin(xyz=(0.0, (length - radius * 1.5) / 2.0, hinge_z)),
        material=material,
        name="base_saddle",
    )
    _add_distal_clevis(
        part,
        length=length,
        width=width,
        thickness=thickness,
        radius=radius,
        z=hinge_z,
        material=material,
        joint_material=joint_material,
    )


def _add_finger_link(
    part,
    *,
    length: float,
    width: float,
    thickness: float,
    has_distal_clevis: bool,
    is_tip: bool,
    material: Material,
    joint_material: Material,
    rubber_material: Material,
) -> None:
    radius = thickness * 0.45
    central_len = width * 0.56
    body_start = radius * 0.66
    body_end = length - (radius * 2.0 if has_distal_clevis else radius * 0.70)
    body_len = max(body_end - body_start, 0.004)

    cyl, cyl_origin = _cyl_x(radius, central_len)
    part.visual(
        cyl,
        origin=cyl_origin,
        material=joint_material,
        name="proximal_barrel",
    )
    part.visual(
        Box((central_len, body_len, thickness * 0.70)),
        origin=Origin(xyz=(0.0, body_start + body_len / 2.0, 0.0)),
        material=material,
        name="link_beam",
    )

    if has_distal_clevis:
        _add_distal_clevis(
            part,
            length=length,
            width=width,
            thickness=thickness,
            radius=radius,
            material=material,
            joint_material=joint_material,
        )
    if is_tip:
        part.visual(
            Sphere(radius=thickness * 0.46),
            origin=Origin(xyz=(0.0, length, 0.0)),
            material=rubber_material,
            name="soft_tip",
        )
        part.visual(
            Box((central_len * 0.72, length * 0.32, thickness * 0.18)),
            origin=Origin(xyz=(0.0, length * 0.78, -thickness * 0.38)),
            material=rubber_material,
            name="finger_pad",
        )


def _build_long_finger(
    model: ArticulatedObject,
    *,
    name: str,
    x: float,
    root_y: float,
    root_z: float,
    width: float,
    thickness: float,
    lengths: tuple[float, float, float],
    splay_limits: tuple[float, float],
    materials: dict[str, Material],
) -> None:
    base_len = thickness * 1.55
    hinge_z = 0.017

    base = model.part(f"{name}_base")
    _add_base_knuckle(
        base,
        length=base_len,
        width=width,
        thickness=thickness,
        hinge_z=hinge_z,
        material=materials["link"],
        joint_material=materials["joint"],
        accent_material=materials["accent"],
    )
    model.articulation(
        f"palm_to_{name}_base_splay",
        ArticulationType.REVOLUTE,
        parent="palm",
        child=base,
        origin=Origin(xyz=(x, root_y, root_z)),
        axis=SPLAY_AXIS,
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=splay_limits[0], upper=splay_limits[1]),
    )

    proximal = model.part(f"{name}_proximal")
    _add_finger_link(
        proximal,
        length=lengths[0],
        width=width,
        thickness=thickness,
        has_distal_clevis=True,
        is_tip=False,
        material=materials["link"],
        joint_material=materials["joint"],
        rubber_material=materials["rubber"],
    )
    model.articulation(
        f"{name}_base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(xyz=(0.0, base_len, hinge_z)),
        axis=FLEX_AXIS,
        motion_limits=MotionLimits(effort=1.6, velocity=4.0, lower=0.0, upper=1.12),
    )

    middle = model.part(f"{name}_middle")
    _add_finger_link(
        middle,
        length=lengths[1],
        width=width * 0.92,
        thickness=thickness * 0.92,
        has_distal_clevis=True,
        is_tip=False,
        material=materials["link"],
        joint_material=materials["joint"],
        rubber_material=materials["rubber"],
    )
    model.articulation(
        f"{name}_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(0.0, lengths[0], 0.0)),
        axis=FLEX_AXIS,
        motion_limits=MotionLimits(effort=1.2, velocity=4.5, lower=0.0, upper=1.35),
    )

    distal = model.part(f"{name}_distal")
    _add_finger_link(
        distal,
        length=lengths[2],
        width=width * 0.82,
        thickness=thickness * 0.82,
        has_distal_clevis=False,
        is_tip=True,
        material=materials["link"],
        joint_material=materials["joint"],
        rubber_material=materials["rubber"],
    )
    model.articulation(
        f"{name}_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(0.0, lengths[1], 0.0)),
        axis=FLEX_AXIS,
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=0.0, upper=1.05),
    )


def _build_thumb(
    model: ArticulatedObject,
    *,
    root_x: float,
    root_y: float,
    root_z: float,
    yaw: float,
    materials: dict[str, Material],
) -> None:
    width = 0.017
    thickness = 0.014
    base_len = 0.021
    lengths = (0.046, 0.034)
    hinge_z = 0.016

    base = model.part("thumb_base")
    _add_base_knuckle(
        base,
        length=base_len,
        width=width,
        thickness=thickness,
        hinge_z=hinge_z,
        material=materials["link"],
        joint_material=materials["joint"],
        accent_material=materials["accent"],
    )
    model.articulation(
        "palm_to_thumb_base_splay",
        ArticulationType.REVOLUTE,
        parent="palm",
        child=base,
        origin=Origin(xyz=(root_x, root_y, root_z), rpy=(0.0, 0.0, yaw)),
        axis=SPLAY_AXIS,
        motion_limits=MotionLimits(effort=1.4, velocity=3.0, lower=-0.55, upper=0.42),
    )

    proximal = model.part("thumb_proximal")
    _add_finger_link(
        proximal,
        length=lengths[0],
        width=width,
        thickness=thickness,
        has_distal_clevis=True,
        is_tip=False,
        material=materials["link"],
        joint_material=materials["joint"],
        rubber_material=materials["rubber"],
    )
    model.articulation(
        "thumb_base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(xyz=(0.0, base_len, hinge_z)),
        axis=FLEX_AXIS,
        motion_limits=MotionLimits(effort=1.3, velocity=4.0, lower=0.0, upper=1.05),
    )

    distal = model.part("thumb_distal")
    _add_finger_link(
        distal,
        length=lengths[1],
        width=width * 0.82,
        thickness=thickness * 0.84,
        has_distal_clevis=False,
        is_tip=True,
        material=materials["link"],
        joint_material=materials["joint"],
        rubber_material=materials["rubber"],
    )
    model.articulation(
        "thumb_proximal_to_distal",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=distal,
        origin=Origin(xyz=(0.0, lengths[0], 0.0)),
        axis=FLEX_AXIS,
        motion_limits=MotionLimits(effort=0.7, velocity=4.5, lower=0.0, upper=1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anthropomorphic_robotic_palm")
    materials = _finger_materials(model)

    palm = model.part("palm")
    palm.visual(
        Box((0.130, 0.112, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=materials["palm"],
        name="palm_block",
    )
    palm.visual(
        Box((0.074, 0.042, 0.032)),
        origin=Origin(xyz=(0.0, -0.074, 0.016)),
        material=materials["joint"],
        name="wrist_socket",
    )

    root_y = 0.058
    root_z = 0.028
    for i, x in enumerate((-0.045, -0.015, 0.015, 0.045)):
        palm.visual(
            Cylinder(radius=0.0115, length=0.006),
            origin=Origin(xyz=(x, root_y, 0.025)),
            material=materials["joint"],
            name=f"finger_socket_{i}",
        )

    palm.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(-0.065, -0.013, 0.025)),
        material=materials["joint"],
        name="thumb_socket",
    )
    palm.visual(
        Box((0.017, 0.044, 0.014)),
        origin=Origin(xyz=(-0.065, -0.010, 0.021)),
        material=materials["palm"],
        name="thumb_saddle",
    )

    long_fingers = (
        ("index", -0.045, 0.016, 0.014, (0.060, 0.043, 0.031), (-0.34, 0.24)),
        ("middle", -0.015, 0.018, 0.015, (0.067, 0.048, 0.034), (-0.20, 0.20)),
        ("ring", 0.015, 0.017, 0.0145, (0.063, 0.045, 0.032), (-0.24, 0.30)),
        ("little", 0.045, 0.014, 0.013, (0.052, 0.038, 0.027), (-0.26, 0.42)),
    )
    for name, x, width, thickness, lengths, limits in long_fingers:
        _build_long_finger(
            model,
            name=name,
            x=x,
            root_y=root_y,
            root_z=root_z,
            width=width,
            thickness=thickness,
            lengths=lengths,
            splay_limits=limits,
            materials=materials,
        )

    _build_thumb(
        model,
        root_x=-0.065,
        root_y=-0.013,
        root_z=root_z,
        yaw=math.radians(48.0),
        materials=materials,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    long_names = ("index", "middle", "ring", "little")

    ctx.check(
        "one palm plus nineteen articulated digit parts",
        len(object_model.parts) == 20,
        details=f"parts={len(object_model.parts)}",
    )
    ctx.check(
        "nineteen independent revolute joints",
        len(object_model.articulations) == 19
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={len(object_model.articulations)}",
    )

    for name in long_names:
        for parent_suffix, child_suffix in (
            ("base", "proximal"),
            ("proximal", "middle"),
            ("middle", "distal"),
        ):
            parent = f"{name}_{parent_suffix}"
            child = f"{name}_{child_suffix}"
            ctx.allow_overlap(
                parent,
                child,
                elem_a="distal_pin",
                elem_b="proximal_barrel",
                reason="A solid pin proxy is intentionally captured through the child knuckle barrel.",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="yz",
                elem_a="proximal_barrel",
                elem_b="distal_pin",
                min_overlap=0.002,
                name=f"{name} {child_suffix} barrel is retained on pin",
            )

        ctx.check(
            f"{name} has base and three serial links",
            all(object_model.get_part(f"{name}_{suffix}") is not None for suffix in ("base", "proximal", "middle", "distal")),
        )
        ctx.expect_overlap(
            f"{name}_proximal",
            f"{name}_base",
            axes="yz",
            elem_a="proximal_barrel",
            elem_b="distal_barrel_a",
            min_overlap=0.004,
            name=f"{name} proximal barrel shares first hinge line",
        )
        ctx.expect_overlap(
            f"{name}_middle",
            f"{name}_proximal",
            axes="yz",
            elem_a="proximal_barrel",
            elem_b="distal_barrel_a",
            min_overlap=0.004,
            name=f"{name} middle barrel shares second hinge line",
        )
        ctx.expect_overlap(
            f"{name}_distal",
            f"{name}_middle",
            axes="yz",
            elem_a="proximal_barrel",
            elem_b="distal_barrel_a",
            min_overlap=0.003,
            name=f"{name} distal barrel shares fingertip hinge line",
        )

    ctx.check(
        "thumb has base and two serial links",
        all(object_model.get_part(f"thumb_{suffix}") is not None for suffix in ("base", "proximal", "distal")),
    )
    for parent_suffix, child_suffix in (("base", "proximal"), ("proximal", "distal")):
        parent = f"thumb_{parent_suffix}"
        child = f"thumb_{child_suffix}"
        ctx.allow_overlap(
            parent,
            child,
            elem_a="distal_pin",
            elem_b="proximal_barrel",
            reason="A solid pin proxy is intentionally captured through the child thumb knuckle barrel.",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="yz",
            elem_a="proximal_barrel",
            elem_b="distal_pin",
            min_overlap=0.002,
            name=f"thumb {child_suffix} barrel is retained on pin",
        )

    splay_joint = object_model.get_articulation("palm_to_index_base_splay")
    flex_joint = object_model.get_articulation("index_base_to_proximal")
    index_tip = object_model.get_part("index_distal")

    rest_tip = ctx.part_world_position(index_tip)
    with ctx.pose({splay_joint: 0.22}):
        splayed_tip = ctx.part_world_position(index_tip)
    ctx.check(
        "index root splay moves the whole finger sideways",
        rest_tip is not None and splayed_tip is not None and abs(splayed_tip[0] - rest_tip[0]) > 0.010,
        details=f"rest={rest_tip}, splayed={splayed_tip}",
    )

    with ctx.pose({flex_joint: 0.75}):
        flexed_tip = ctx.part_world_position(index_tip)
    ctx.check(
        "index flex hinge curls the chain out of the palm plane",
        rest_tip is not None and flexed_tip is not None and flexed_tip[2] < rest_tip[2] - 0.010,
        details=f"rest={rest_tip}, flexed={flexed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
