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
    model = ArticulatedObject(name="compact_articulated_digit")

    plate_mat = model.material("blue_anodized_plates", rgba=(0.05, 0.13, 0.28, 1.0))
    base_mat = model.material("charcoal_base", rgba=(0.10, 0.11, 0.12, 1.0))
    steel_mat = model.material("brushed_steel_pins", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_mat = model.material("black_rubber_feet", rgba=(0.01, 0.01, 0.012, 1.0))

    pin_axis = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))

    lug_radius = 0.018
    pin_radius = 0.0045
    web_width = 0.030
    web_thickness = 0.012
    outer_lug_width = 0.014
    center_lug_width = 0.020
    outer_lug_y = 0.029
    pin_length = 0.090
    pin_head_radius = 0.008
    pin_head_thickness = 0.004

    def add_pin(part, *, prefix: str = "") -> None:
        name_prefix = f"{prefix}_" if prefix else ""
        part.visual(
            Cylinder(radius=pin_radius, length=pin_length),
            origin=pin_axis,
            material=steel_mat,
            name=f"{name_prefix}pin_shaft",
        )
        part.visual(
            Cylinder(radius=pin_head_radius, length=pin_head_thickness),
            origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel_mat,
            name=f"{name_prefix}pin_head_0",
        )
        part.visual(
            Cylinder(radius=pin_head_radius, length=pin_head_thickness),
            origin=Origin(xyz=(0.0, -0.047, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel_mat,
            name=f"{name_prefix}pin_head_1",
        )

    def add_phalanx(part, length: float, *, distal_fork: bool, prefix: str) -> None:
        web_start = -0.004
        web_end = length - 0.030 if distal_fork else length - 0.010
        web_len = web_end - web_start
        part.visual(
            Box((web_len, web_width, web_thickness)),
            origin=Origin(xyz=((web_start + web_end) / 2.0, 0.0, 0.0)),
            material=plate_mat,
            name=f"{prefix}_web",
        )
        part.visual(
            Cylinder(radius=lug_radius, length=center_lug_width),
            origin=pin_axis,
            material=plate_mat,
            name=f"{prefix}_proximal_lug",
        )
        part.visual(
            Box((max(web_len - 0.030, 0.020), 0.010, 0.006)),
            origin=Origin(xyz=((web_start + web_end) / 2.0 + 0.006, 0.0, 0.009)),
            material=plate_mat,
            name=f"{prefix}_raised_rib",
        )
        add_pin(part, prefix=prefix)

        if distal_fork:
            for idx, sign in enumerate((-1.0, 1.0)):
                y = sign * outer_lug_y
                part.visual(
                    Box((0.038, 0.024, web_thickness)),
                    origin=Origin(xyz=(length - 0.029, sign * 0.0248, 0.0)),
                    material=plate_mat,
                    name=f"{prefix}_fork_arm_{idx}",
                )
                part.visual(
                    Cylinder(radius=lug_radius, length=outer_lug_width),
                    origin=Origin(xyz=(length, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                    material=plate_mat,
                    name=f"{prefix}_distal_lug_{idx}",
                )
        else:
            part.visual(
                Cylinder(radius=lug_radius * 0.82, length=center_lug_width),
                origin=Origin(xyz=(length, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=plate_mat,
                name=f"{prefix}_rounded_tip",
            )

    base = model.part("base_mount")
    base.visual(
        Box((0.120, 0.095, 0.014)),
        origin=Origin(xyz=(-0.035, 0.0, -0.054)),
        material=base_mat,
        name="base_plate",
    )
    base.visual(
        Box((0.060, 0.030, 0.035)),
        origin=Origin(xyz=(-0.018, 0.0, -0.0385)),
        material=base_mat,
        name="pedestal_block",
    )
    for idx, sign in enumerate((-1.0, 1.0)):
        y = sign * outer_lug_y
        base.visual(
            Box((0.042, 0.014, 0.048)),
            origin=Origin(xyz=(-0.012, y, -0.024)),
            material=base_mat,
            name=f"base_cheek_{idx}",
        )
        base.visual(
            Cylinder(radius=lug_radius, length=outer_lug_width),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=base_mat,
            name=f"base_boss_{idx}",
        )
    base.visual(
        Box((0.070, 0.075, 0.005)),
        origin=Origin(xyz=(-0.035, 0.0, -0.0635)),
        material=dark_mat,
        name="rubber_foot",
    )

    proximal = model.part("proximal_plate")
    middle = model.part("middle_plate")
    distal = model.part("distal_plate")

    proximal_len = 0.105
    middle_len = 0.082
    distal_len = 0.062

    add_phalanx(proximal, proximal_len, distal_fork=True, prefix="proximal")
    add_phalanx(middle, middle_len, distal_fork=True, prefix="middle")
    add_phalanx(distal, distal_len, distal_fork=False, prefix="distal")

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.10),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(proximal_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(middle_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_mount")
    proximal = object_model.get_part("proximal_plate")
    middle = object_model.get_part("middle_plate")
    distal = object_model.get_part("distal_plate")

    base_joint = object_model.get_articulation("base_to_proximal")
    proximal_joint = object_model.get_articulation("proximal_to_middle")
    middle_joint = object_model.get_articulation("middle_to_distal")

    ctx.check(
        "three revolute hinge joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )

    def allow_pin(parent, child, parent_elems, pin_elem, label: str) -> None:
        for elem in parent_elems:
            ctx.allow_overlap(
                parent,
                child,
                elem_a=elem,
                elem_b=pin_elem,
                reason="The steel pin is intentionally seated through the hinge boss bore.",
            )
            ctx.expect_overlap(
                parent,
                child,
                elem_a=elem,
                elem_b=pin_elem,
                axes="xyz",
                min_overlap=0.008,
                name=f"{label} pin passes through {elem}",
            )

    allow_pin(base, proximal, ("base_boss_0", "base_boss_1"), "proximal_pin_shaft", "base hinge")
    allow_pin(
        proximal,
        middle,
        ("proximal_distal_lug_0", "proximal_distal_lug_1"),
        "middle_pin_shaft",
        "middle hinge",
    )
    allow_pin(
        middle,
        distal,
        ("middle_distal_lug_0", "middle_distal_lug_1"),
        "distal_pin_shaft",
        "distal hinge",
    )

    rest_distal_pos = ctx.part_world_position(distal)
    with ctx.pose({base_joint: 0.65, proximal_joint: 0.85, middle_joint: 0.95}):
        curled_distal_pos = ctx.part_world_position(distal)

    ctx.check(
        "digit curls upward",
        rest_distal_pos is not None
        and curled_distal_pos is not None
        and curled_distal_pos[2] > rest_distal_pos[2] + 0.060,
        details=f"rest={rest_distal_pos}, curled={curled_distal_pos}",
    )

    return ctx.report()


object_model = build_object_model()
