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
    mesh_from_geometry,
    section_loft,
)


PIN_AXIS_RPY = (math.pi / 2.0, 0.0, 0.0)


def _rounded_section(x: float, width: float, height: float, segments: int = 18):
    """Superellipse-like section in the local YZ plane for tapered finger links."""
    exponent = 3.2
    pts = []
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        y = 0.5 * width * math.copysign(abs(c) ** (2.0 / exponent), c)
        z = 0.5 * height * math.copysign(abs(s) ** (2.0 / exponent), s)
        pts.append((x, y, z))
    return pts


def _phalanx_body_mesh(
    *,
    length: float,
    width0: float,
    width1: float,
    height0: float,
    height1: float,
    proximal_barrel: float,
    distal_neck: float,
    name: str,
):
    neck0_w = proximal_barrel * 0.92
    neck1_w = distal_neck * 0.92
    sections = [
        _rounded_section(0.006, neck0_w, height0 * 0.72),
        _rounded_section(0.026, width0, height0),
        _rounded_section(max(length - 0.032, 0.045), width1, height1),
        _rounded_section(length - 0.008, neck1_w, height1 * 0.74),
    ]
    return mesh_from_geometry(section_loft(sections), name)


def _add_cylinder_y(part, radius: float, length: float, xyz, material, name: str):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=PIN_AXIS_RPY),
        material=material,
        name=name,
    )


def _add_finger_link(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    width0: float,
    width1: float,
    height0: float,
    height1: float,
    proximal_barrel: float,
    distal_barrel: float,
    has_distal_fork: bool,
    metal: Material,
    pin: Material,
    rubber: Material,
):
    link = model.part(name)
    link.visual(
        _phalanx_body_mesh(
            length=length,
            width0=width0,
            width1=width1,
            height0=height0,
            height1=height1,
            proximal_barrel=proximal_barrel,
            distal_neck=distal_barrel if has_distal_fork else width1 * 0.70,
            name=f"{name}_taper",
        ),
        material=metal,
        name="tapered_body",
    )

    _add_cylinder_y(
        link,
        radius=height0 * 0.52,
        length=proximal_barrel,
        xyz=(0.0, 0.0, 0.0),
        material=pin,
        name="proximal_barrel",
    )

    if has_distal_fork:
        lug_len = 0.0065
        clearance = 0.0014
        lug_y = 0.5 * distal_barrel + clearance + 0.5 * lug_len
        lug_radius = height1 * 0.49
        for side, y in (("outer", lug_y), ("inner", -lug_y)):
            _add_cylinder_y(
                link,
                radius=lug_radius,
                length=lug_len,
                xyz=(length, y, 0.0),
                material=pin,
                name=f"distal_lug_{side}",
            )
        # A short crosshead web ties the split knuckles visibly back into the
        # tapered body, avoiding floating-looking hinge ears.
        for side, y in (("outer", lug_y), ("inner", -lug_y)):
            link.visual(
                Box((0.024, lug_len * 1.15, height1 * 0.45)),
                origin=Origin(xyz=(length - 0.012, y, 0.0)),
                material=metal,
                name=f"distal_web_{side}",
            )
    else:
        # The terminal pad is a compliant fingertip cap seated into the last
        # tapered phalanx rather than another revolute joint.
        link.visual(
            Box((0.018, width1 * 0.95, height1 * 0.82)),
            origin=Origin(xyz=(length - 0.001, 0.0, 0.0)),
            material=rubber,
            name="tip_pad",
        )

    # Raised dorsal rib gives each metal link a believable machined profile and
    # makes the taper easier to read.
    link.visual(
        Box((max(length - 0.060, 0.020), width1 * 0.22, height1 * 0.18)),
        origin=Origin(xyz=(length * 0.52, 0.0, height1 * 0.52)),
        material=metal,
        name="top_rib",
    )
    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_independent_finger_mechanism")

    palm_mat = model.material("matte_black_palm", rgba=(0.02, 0.025, 0.03, 1.0))
    metal_mat = model.material("brushed_titanium_links", rgba=(0.56, 0.62, 0.68, 1.0))
    pin_mat = model.material("dark_hardened_pin", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber_mat = model.material("black_rubber_tip", rgba=(0.005, 0.005, 0.004, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((0.125, 0.150, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=palm_mat,
        name="palm_block",
    )
    palm.visual(
        Box((0.050, 0.128, 0.010)),
        origin=Origin(xyz=(0.044, 0.0, 0.043)),
        material=palm_mat,
        name="front_mount_plate",
    )

    finger_y = (-0.034, 0.034)
    base_x = 0.062
    base_z = 0.064
    base_center_barrel = 0.0170
    base_lug_len = 0.0075
    base_clearance = 0.0016
    base_lug_offset = 0.5 * base_center_barrel + base_clearance + 0.5 * base_lug_len

    for idx, y0 in enumerate(finger_y):
        for suffix, sy in (("outer", base_lug_offset), ("inner", -base_lug_offset)):
            lug_y = y0 + sy
            palm.visual(
                Box((0.031, base_lug_len, 0.041)),
                origin=Origin(xyz=(base_x - 0.007, lug_y, 0.047)),
                material=palm_mat,
                name=f"finger_{idx}_base_cheek_{suffix}",
            )
            _add_cylinder_y(
                palm,
                radius=0.0140,
                length=base_lug_len,
                xyz=(base_x, lug_y, base_z),
                material=pin_mat,
                name=f"finger_{idx}_base_lug_{suffix}",
            )
        # Small exposed pin heads outside the clevis make the base pivot read as
        # a real pin joint while staying clear of the moving center barrel.
        for suffix, sy in (("cap_a", base_lug_offset + 0.0048), ("cap_b", -base_lug_offset - 0.0048)):
            _add_cylinder_y(
                palm,
                radius=0.0078,
                length=0.0032,
                xyz=(base_x, y0 + sy, base_z),
                material=pin_mat,
                name=f"finger_{idx}_base_{suffix}",
            )

    lengths = (0.112, 0.084, 0.064)
    widths = ((0.030, 0.024), (0.025, 0.020), (0.021, 0.016))
    heights = ((0.027, 0.023), (0.023, 0.019), (0.019, 0.015))
    proximal_barrels = (base_center_barrel, 0.0145, 0.0125)
    distal_barrels = (proximal_barrels[1], proximal_barrels[2], 0.0100)

    for idx, y0 in enumerate(finger_y):
        proximal = _add_finger_link(
            model,
            name=f"finger_{idx}_proximal",
            length=lengths[0],
            width0=widths[0][0],
            width1=widths[0][1],
            height0=heights[0][0],
            height1=heights[0][1],
            proximal_barrel=proximal_barrels[0],
            distal_barrel=distal_barrels[0],
            has_distal_fork=True,
            metal=metal_mat,
            pin=pin_mat,
            rubber=rubber_mat,
        )
        middle = _add_finger_link(
            model,
            name=f"finger_{idx}_middle",
            length=lengths[1],
            width0=widths[1][0],
            width1=widths[1][1],
            height0=heights[1][0],
            height1=heights[1][1],
            proximal_barrel=proximal_barrels[1],
            distal_barrel=distal_barrels[1],
            has_distal_fork=True,
            metal=metal_mat,
            pin=pin_mat,
            rubber=rubber_mat,
        )
        distal = _add_finger_link(
            model,
            name=f"finger_{idx}_distal",
            length=lengths[2],
            width0=widths[2][0],
            width1=widths[2][1],
            height0=heights[2][0],
            height1=heights[2][1],
            proximal_barrel=proximal_barrels[2],
            distal_barrel=distal_barrels[2],
            has_distal_fork=False,
            metal=metal_mat,
            pin=pin_mat,
            rubber=rubber_mat,
        )

        model.articulation(
            f"finger_{idx}_base_pivot",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(base_x, y0, base_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=3.5, lower=0.0, upper=1.25),
        )
        model.articulation(
            f"finger_{idx}_middle_pivot",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(lengths[0], 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=3.5, lower=0.0, upper=1.35),
        )
        model.articulation(
            f"finger_{idx}_tip_pivot",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(lengths[1], 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=3.5, velocity=3.5, lower=0.0, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = {joint.name: joint for joint in object_model.articulations}

    for idx in (0, 1):
        prefix = f"finger_{idx}"
        names = [f"{prefix}_base_pivot", f"{prefix}_middle_pivot", f"{prefix}_tip_pivot"]
        ctx.check(
            f"{prefix} has three revolute joints",
            all(joints[name].articulation_type == ArticulationType.REVOLUTE for name in names),
            details=str([joints[name].articulation_type for name in names]),
        )
        ctx.check(
            f"{prefix} joints are not mimicked",
            all(joints[name].mimic is None for name in names),
            details="Finger chain should be independently driven, not coupled by mimic joints.",
        )

    ctx.expect_origin_gap(
        "finger_1_proximal",
        "finger_0_proximal",
        axis="y",
        min_gap=0.060,
        name="base pivots are side by side",
    )

    finger0_tip = object_model.get_part("finger_0_distal")
    finger1_tip = object_model.get_part("finger_1_distal")
    rest0 = ctx.part_world_position(finger0_tip)
    rest1 = ctx.part_world_position(finger1_tip)
    with ctx.pose({"finger_0_base_pivot": 0.75, "finger_0_middle_pivot": 0.55}):
        curled0 = ctx.part_world_position(finger0_tip)
        unchanged1 = ctx.part_world_position(finger1_tip)

    ctx.check(
        "finger_0 curls independently",
        rest0 is not None
        and curled0 is not None
        and curled0[2] > rest0[2] + 0.060
        and curled0[0] < rest0[0] - 0.020,
        details=f"rest={rest0}, curled={curled0}",
    )
    ctx.check(
        "finger_1 stays fixed while finger_0 moves",
        rest1 is not None
        and unchanged1 is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest1, unchanged1)),
        details=f"rest={rest1}, after={unchanged1}",
    )

    return ctx.report()


object_model = build_object_model()
