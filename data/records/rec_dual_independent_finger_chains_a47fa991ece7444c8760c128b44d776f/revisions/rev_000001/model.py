from __future__ import annotations

from math import pi

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


PIN_RPY = (pi / 2.0, 0.0, 0.0)


def _pin(part, *, x: float, y: float, z: float, radius: float, length: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=PIN_RPY),
        material=material,
        name=name,
    )


def _add_phalanx(part, *, length: float, material, pin_material, pad_material=None) -> None:
    """Add a flat robotic finger link with a central proximal barrel."""
    body_start = 0.014
    body_end = max(body_start + 0.020, length - 0.016)
    body_len = body_end - body_start

    part.visual(
        Box((body_len, 0.024, 0.016)),
        origin=Origin(xyz=(body_start + body_len / 2.0, 0.0, 0.0)),
        material=material,
        name="link_body",
    )
    part.visual(
        Box((0.011, 0.020, 0.014)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=material,
        name="hub_bridge",
    )
    _pin(
        part,
        x=0.0,
        y=0.0,
        z=0.0,
        radius=0.0105,
        length=0.018,
        material=pin_material,
        name="proximal_barrel",
    )

    if pad_material is None:
        for side, y in enumerate((-0.0125, 0.0125)):
            part.visual(
                Box((0.024, 0.007, 0.020)),
                origin=Origin(xyz=(length - 0.006, y, 0.0)),
                material=material,
                name=f"distal_lug_{side}",
            )
            _pin(
                part,
                x=length,
                y=y,
                z=0.0,
                radius=0.0105,
                length=0.007,
                material=pin_material,
                name=f"distal_pin_{side}",
            )
    else:
        body_end = length - 0.003
        body_len = body_end - body_start
        # A second, longer center cover makes the distal link read as a continuous
        # short phalanx that disappears into the rubber pad.
        part.visual(
            Box((body_len, 0.020, 0.014)),
            origin=Origin(xyz=(body_start + body_len / 2.0, 0.0, 0.0)),
            material=material,
            name="distal_cover",
        )
        _pin(
            part,
            x=length,
            y=0.0,
            z=0.0,
            radius=0.0135,
            length=0.027,
            material=pad_material,
            name="fingertip_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_independent_finger_mechanism")

    palm_mat = model.material("anodized_palm", rgba=(0.06, 0.07, 0.08, 1.0))
    link_mat = model.material("brushed_link_blue", rgba=(0.18, 0.36, 0.62, 1.0))
    pin_mat = model.material("brushed_steel_pins", rgba=(0.72, 0.72, 0.68, 1.0))
    pad_mat = model.material("matte_rubber_pads", rgba=(0.015, 0.015, 0.014, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((0.140, 0.160, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=palm_mat,
        name="palm_block",
    )
    palm.visual(
        Box((0.050, 0.150, 0.010)),
        origin=Origin(xyz=(-0.018, 0.0, 0.060)),
        material=palm_mat,
        name="top_land",
    )

    finger_y = (-0.035, 0.035)
    base_x = 0.085
    base_z = 0.078
    link_lengths = (0.075, 0.065, 0.052)

    for idx, y0 in enumerate(finger_y):
        for side, yoff in enumerate((-0.0125, 0.0125)):
            palm.visual(
                Box((0.030, 0.007, 0.044)),
                origin=Origin(xyz=(base_x, y0 + yoff, base_z - 0.003)),
                material=palm_mat,
                name=f"finger_{idx}_base_lug_{side}",
            )
            _pin(
                palm,
                x=base_x,
                y=y0 + yoff,
                z=base_z,
                radius=0.0105,
                length=0.007,
                material=pin_mat,
                name=f"finger_{idx}_base_pin_{side}",
            )

        proximal = model.part(f"finger_{idx}_proximal")
        middle = model.part(f"finger_{idx}_middle")
        distal = model.part(f"finger_{idx}_distal")

        _add_phalanx(proximal, length=link_lengths[0], material=link_mat, pin_material=pin_mat)
        _add_phalanx(middle, length=link_lengths[1], material=link_mat, pin_material=pin_mat)
        _add_phalanx(
            distal,
            length=link_lengths[2],
            material=link_mat,
            pin_material=pin_mat,
            pad_material=pad_mat,
        )

        model.articulation(
            f"finger_{idx}_base",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(base_x, y0, base_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.05),
        )
        model.articulation(
            f"finger_{idx}_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(link_lengths[0], 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=2.2, lower=0.0, upper=1.25),
        )
        model.articulation(
            f"finger_{idx}_tip",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(link_lengths[1], 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.4, lower=0.0, upper=0.95),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joint_names = [
        f"finger_{idx}_{stage}"
        for idx in (0, 1)
        for stage in ("base", "middle", "tip")
    ]
    joints = [object_model.get_articulation(name) for name in joint_names]
    ctx.check(
        "two fingers have six revolute joints",
        len(joints) == 6 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "finger chains are mechanically uncoupled",
        all(j.mimic is None for j in joints),
        details="No joint uses mimic coupling; each chain has its own actuated joints.",
    )
    ctx.check(
        "all finger joints flex from straight rest",
        all(
            j.motion_limits is not None
            and j.motion_limits.lower == 0.0
            and j.motion_limits.upper is not None
            and j.motion_limits.upper >= 0.9
            for j in joints
        ),
        details="Each revolute joint has a nonzero positive flexion range.",
    )

    for idx in (0, 1):
        ctx.expect_overlap(
            "palm",
            f"finger_{idx}_proximal",
            axes="xz",
            elem_a=f"finger_{idx}_base_lug_0",
            elem_b="proximal_barrel",
            min_overlap=0.015,
            name=f"finger_{idx} base knuckle is captured by palm lug",
        )
        ctx.expect_overlap(
            f"finger_{idx}_proximal",
            f"finger_{idx}_middle",
            axes="xz",
            elem_a="distal_lug_0",
            elem_b="proximal_barrel",
            min_overlap=0.015,
            name=f"finger_{idx} middle knuckle aligns to proximal link",
        )
        ctx.expect_overlap(
            f"finger_{idx}_middle",
            f"finger_{idx}_distal",
            axes="xz",
            elem_a="distal_lug_0",
            elem_b="proximal_barrel",
            min_overlap=0.015,
            name=f"finger_{idx} distal knuckle aligns to middle link",
        )

    finger_0_tip = object_model.get_part("finger_0_distal")
    finger_1_tip = object_model.get_part("finger_1_distal")
    f0_rest = ctx.part_world_position(finger_0_tip)
    f1_rest = ctx.part_world_position(finger_1_tip)
    with ctx.pose({"finger_0_base": 0.25, "finger_0_middle": 0.25, "finger_0_tip": 0.15}):
        f0_flexed = ctx.part_world_position(finger_0_tip)
        f1_when_f0_moves = ctx.part_world_position(finger_1_tip)
    with ctx.pose({"finger_1_base": 0.25, "finger_1_middle": 0.25, "finger_1_tip": 0.15}):
        f0_when_f1_moves = ctx.part_world_position(finger_0_tip)
        f1_flexed = ctx.part_world_position(finger_1_tip)

    ctx.check(
        "finger_0 bends in its own xz plane",
        f0_rest is not None
        and f0_flexed is not None
        and f0_flexed[2] < f0_rest[2] - 0.015
        and abs(f0_flexed[1] - f0_rest[1]) < 1e-6,
        details=f"rest={f0_rest}, flexed={f0_flexed}",
    )
    ctx.check(
        "finger_1 bends in its own xz plane",
        f1_rest is not None
        and f1_flexed is not None
        and f1_flexed[2] < f1_rest[2] - 0.015
        and abs(f1_flexed[1] - f1_rest[1]) < 1e-6,
        details=f"rest={f1_rest}, flexed={f1_flexed}",
    )
    ctx.check(
        "finger_1 stays still when finger_0 moves",
        f1_rest is not None
        and f1_when_f0_moves is not None
        and all(abs(a - b) < 1e-8 for a, b in zip(f1_rest, f1_when_f0_moves)),
        details=f"rest={f1_rest}, during_finger_0_motion={f1_when_f0_moves}",
    )
    ctx.check(
        "finger_0 stays still when finger_1 moves",
        f0_rest is not None
        and f0_when_f1_moves is not None
        and all(abs(a - b) < 1e-8 for a, b in zip(f0_rest, f0_when_f1_moves)),
        details=f"rest={f0_rest}, during_finger_1_motion={f0_when_f1_moves}",
    )

    return ctx.report()


object_model = build_object_model()
