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


PIN_RPY = (-math.pi / 2.0, 0.0, 0.0)


def _cylinder_y(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=PIN_RPY),
        material=material,
        name=name,
    )


def _add_barrel(part, *, radius, length, material, name="proximal_barrel"):
    _cylinder_y(part, radius, length, (0.0, 0.0, 0.0), material, name)


def _add_yoke(part, *, x, ear_radius, ear_length, pin_length, material, pin_material, prefix):
    ear_y = 0.027
    for sign, suffix in ((1.0, "upper"), (-1.0, "lower")):
        _cylinder_y(
            part,
            ear_radius,
            ear_length,
            (x, sign * ear_y, 0.0),
            material,
            f"{prefix}_{suffix}_ear",
        )
        part.visual(
            Box((0.039, 0.012, 0.014)),
            origin=Origin(xyz=(x - 0.033, sign * 0.016, 0.0)),
            material=material,
            name=f"{prefix}_{suffix}_strap",
        )
        _cylinder_y(
            part,
            0.0065,
            0.002,
            (x, sign * (ear_y + 0.006), 0.0),
            pin_material,
            f"{prefix}_{suffix}_bushing",
        )

    _cylinder_y(part, 0.0045, pin_length, (x, 0.0, 0.0), pin_material, f"{prefix}_pin")


def _add_proximal_or_middle_link(
    part,
    *,
    length,
    body_width,
    body_height,
    barrel_radius,
    material,
    pin_material,
    name_prefix,
):
    _add_barrel(
        part,
        radius=barrel_radius,
        length=0.034,
        material=material,
        name="proximal_barrel",
    )
    part.visual(
        Box((length - 0.040, body_width, body_height)),
        origin=Origin(xyz=((length - 0.040) / 2.0 + 0.015, 0.0, 0.0)),
        material=material,
        name="tapered_web",
    )
    part.visual(
        Box((length - 0.060, body_width * 0.62, body_height * 0.45)),
        origin=Origin(xyz=((length - 0.060) / 2.0 + 0.025, 0.0, body_height * 0.43)),
        material=material,
        name="raised_tendon",
    )
    _add_yoke(
        part,
        x=length,
        ear_radius=0.017,
        ear_length=0.010,
        pin_length=0.064,
        material=material,
        pin_material=pin_material,
        prefix=name_prefix,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_root_digit")

    aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    polymer = model.material("warm_ivory_polymer", rgba=(0.86, 0.82, 0.72, 1.0))
    polymer_mid = model.material("slightly_darker_polymer", rgba=(0.78, 0.75, 0.66, 1.0))
    steel = model.material("dark_steel_pins", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    fork = model.part("fork")
    fork.visual(
        Box((0.080, 0.072, 0.034)),
        origin=Origin(xyz=(-0.058, 0.0, 0.0)),
        material=aluminum,
        name="mount_block",
    )
    fork.visual(
        Box((0.018, 0.080, 0.046)),
        origin=Origin(xyz=(-0.042, 0.0, 0.0)),
        material=aluminum,
        name="fork_bridge",
    )
    for sign, suffix in ((1.0, "upper"), (-1.0, "lower")):
        fork.visual(
            Box((0.058, 0.014, 0.054)),
            origin=Origin(xyz=(0.006, sign * 0.032, 0.0)),
            material=aluminum,
            name=f"{suffix}_cheek",
        )
        _cylinder_y(
            fork,
            0.027,
            0.014,
            (0.0, sign * 0.032, 0.0),
            aluminum,
            f"{suffix}_rounded_lug",
        )
        _cylinder_y(
            fork,
            0.0075,
            0.002,
            (0.0, sign * 0.040, 0.0),
            steel,
            f"{suffix}_screw_cap",
        )
    _cylinder_y(fork, 0.0048, 0.084, (0.0, 0.0, 0.0), steel, "root_pin")

    proximal = model.part("proximal")
    _add_proximal_or_middle_link(
        proximal,
        length=0.095,
        body_width=0.028,
        body_height=0.022,
        barrel_radius=0.018,
        material=polymer,
        pin_material=steel,
        name_prefix="middle_hinge",
    )

    middle = model.part("middle")
    _add_proximal_or_middle_link(
        middle,
        length=0.070,
        body_width=0.024,
        body_height=0.019,
        barrel_radius=0.016,
        material=polymer_mid,
        pin_material=steel,
        name_prefix="distal_hinge",
    )

    distal = model.part("distal")
    _add_barrel(
        distal,
        radius=0.0145,
        length=0.030,
        material=polymer_mid,
        name="proximal_barrel",
    )
    distal.visual(
        Box((0.046, 0.021, 0.017)),
        origin=Origin(xyz=(0.029, 0.0, 0.0)),
        material=polymer_mid,
        name="distal_web",
    )
    distal.visual(
        Box((0.030, 0.013, 0.007)),
        origin=Origin(xyz=(0.036, 0.0, 0.010)),
        material=polymer_mid,
        name="distal_tendon",
    )
    distal.visual(
        Cylinder(radius=0.016, length=0.027),
        origin=Origin(xyz=(0.063, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="tip_pad_shank",
    )
    distal.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.077, 0.0, 0.0)),
        material=rubber,
        name="tip_pad",
    )

    model.articulation(
        "root_knuckle",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "middle_knuckle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "distal_knuckle",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fork = object_model.get_part("fork")
    proximal = object_model.get_part("proximal")
    middle = object_model.get_part("middle")
    distal = object_model.get_part("distal")
    root_knuckle = object_model.get_articulation("root_knuckle")
    middle_knuckle = object_model.get_articulation("middle_knuckle")
    distal_knuckle = object_model.get_articulation("distal_knuckle")

    joints = (root_knuckle, middle_knuckle, distal_knuckle)
    ctx.check(
        "three revolute knuckles",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )

    ctx.allow_overlap(
        fork,
        proximal,
        elem_a="root_pin",
        elem_b="proximal_barrel",
        reason="The fork's steel pin intentionally passes through the proximal hinge barrel.",
    )
    ctx.expect_within(
        fork,
        proximal,
        axes="xz",
        inner_elem="root_pin",
        outer_elem="proximal_barrel",
        margin=0.001,
        name="root pin is centered inside proximal barrel",
    )
    ctx.expect_overlap(
        fork,
        proximal,
        axes="y",
        elem_a="root_pin",
        elem_b="proximal_barrel",
        min_overlap=0.030,
        name="root pin spans the captured barrel",
    )

    ctx.allow_overlap(
        proximal,
        middle,
        elem_a="middle_hinge_pin",
        elem_b="proximal_barrel",
        reason="The proximal link's pin intentionally captures the middle hinge barrel.",
    )
    ctx.expect_within(
        proximal,
        middle,
        axes="xz",
        inner_elem="middle_hinge_pin",
        outer_elem="proximal_barrel",
        margin=0.001,
        name="middle pin is centered inside middle barrel",
    )
    ctx.expect_overlap(
        proximal,
        middle,
        axes="y",
        elem_a="middle_hinge_pin",
        elem_b="proximal_barrel",
        min_overlap=0.028,
        name="middle pin spans the captured barrel",
    )

    ctx.allow_overlap(
        middle,
        distal,
        elem_a="distal_hinge_pin",
        elem_b="proximal_barrel",
        reason="The middle link's pin intentionally captures the distal hinge barrel.",
    )
    ctx.expect_within(
        middle,
        distal,
        axes="xz",
        inner_elem="distal_hinge_pin",
        outer_elem="proximal_barrel",
        margin=0.001,
        name="distal pin is centered inside distal barrel",
    )
    ctx.expect_overlap(
        middle,
        distal,
        axes="y",
        elem_a="distal_hinge_pin",
        elem_b="proximal_barrel",
        min_overlap=0.026,
        name="distal pin spans the captured barrel",
    )

    rest_tip = ctx.part_element_world_aabb(distal, elem="tip_pad")
    with ctx.pose({root_knuckle: 0.65, middle_knuckle: 0.70, distal_knuckle: 0.45}):
        curled_tip = ctx.part_element_world_aabb(distal, elem="tip_pad")
    ctx.check(
        "positive flexion curls tip downward",
        rest_tip is not None
        and curled_tip is not None
        and curled_tip[0][2] < rest_tip[0][2] - 0.030,
        details=f"rest_tip={rest_tip}, curled_tip={curled_tip}",
    )

    return ctx.report()


object_model = build_object_model()
