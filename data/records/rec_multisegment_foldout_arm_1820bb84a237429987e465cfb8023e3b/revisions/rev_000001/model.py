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


LINK_LENGTH = 0.42
JOINT_HEIGHT = 0.160


def _add_boom_link(part, *, length: float, material, dark_material, include_end_plate: bool = False) -> None:
    """Slim rectangular boom segment whose local origin is the proximal pivot."""
    # The lugs stop exactly at the joint planes, while the beam overlaps them
    # slightly so the link reads as one welded/fabricated member.
    part.visual(
        Box((length - 0.11, 0.028, 0.026)),
        origin=Origin(xyz=(length * 0.5, 0.0, 0.0)),
        material=material,
        name="web",
    )
    part.visual(
        Box((0.075, 0.052, 0.034)),
        origin=Origin(xyz=(0.0375, 0.0, 0.0)),
        material=material,
        name="proximal_lug",
    )
    part.visual(
        Box((0.075, 0.052, 0.034)),
        origin=Origin(xyz=(length - 0.0375, 0.0, 0.0)),
        material=material,
        name="distal_lug",
    )

    # Narrow side straps give the otherwise simple bar a bracketed CAD look.
    for y, suffix in ((0.020, "side_0"), (-0.020, "side_1")):
        part.visual(
            Box((length - 0.13, 0.010, 0.038)),
            origin=Origin(xyz=(length * 0.5, y, 0.0)),
            material=dark_material,
            name=suffix,
        )

    for x, prefix in ((0.036, "proximal"), (length - 0.036, "distal")):
        part.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, 0.0, 0.019)),
            material=dark_material,
            name=f"{prefix}_boss",
        )
        for y, suffix in ((0.031, "cheek_0"), (-0.031, "cheek_1")):
            part.visual(
                Box((0.058, 0.010, 0.052)),
                origin=Origin(xyz=(x, y, 0.002)),
                material=material,
                name=f"{prefix}_{suffix}",
            )

    if include_end_plate:
        part.visual(
            Box((0.028, 0.090, 0.066)),
            origin=Origin(xyz=(length + 0.014, 0.0, 0.0)),
            material=dark_material,
            name="end_plate",
        )
        for y in (-0.026, 0.026):
            part.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(length + 0.029, y, 0.020), rpy=(0.0, pi / 2.0, 0.0)),
                material=material,
                name=f"end_bolt_{0 if y < 0 else 1}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_boom")

    painted = model.material("satin_yellow", rgba=(0.95, 0.63, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    base_gray = model.material("cast_gray", rgba=(0.33, 0.35, 0.37, 1.0))
    rubber = model.material("black_bolt", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.260, 0.170, 0.018)),
        origin=Origin(xyz=(-0.040, 0.0, 0.009)),
        material=base_gray,
        name="mount_plate",
    )
    base.visual(
        Box((0.070, 0.070, 0.116)),
        origin=Origin(xyz=(-0.035, 0.0, 0.075)),
        material=base_gray,
        name="pedestal",
    )
    base.visual(
        Box((0.060, 0.086, 0.058)),
        origin=Origin(xyz=(-0.030, 0.0, JOINT_HEIGHT)),
        material=base_gray,
        name="root_yoke",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.064),
        origin=Origin(xyz=(-0.033, 0.0, JOINT_HEIGHT)),
        material=dark_steel,
        name="root_pin_cap",
    )
    for x in (-0.130, 0.040):
        for y in (-0.060, 0.060):
            base.visual(
                Cylinder(radius=0.011, length=0.007),
                origin=Origin(xyz=(x, y, 0.021)),
                material=rubber,
                name=f"bolt_{len(base.visuals)}",
            )

    segments = []
    for index in range(4):
        segment = model.part(f"segment_{index}")
        _add_boom_link(
            segment,
            length=LINK_LENGTH,
            material=painted,
            dark_material=dark_steel,
            include_end_plate=index == 3,
        )
        segments.append(segment)

    limits = MotionLimits(effort=60.0, velocity=1.5, lower=-pi / 2.0, upper=pi / 2.0)
    model.articulation(
        "base_to_segment_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=segments[0],
        origin=Origin(xyz=(0.0, 0.0, JOINT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    for index in range(3):
        model.articulation(
            f"segment_{index}_to_segment_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=segments[index],
            child=segments[index + 1],
            origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-pi / 2.0, upper=pi / 2.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [object_model.get_articulation("base_to_segment_0")] + [
        object_model.get_articulation(f"segment_{i}_to_segment_{i + 1}") for i in range(3)
    ]
    ctx.check(
        "four repeated revolute pivots",
        len(joints) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )

    base = object_model.get_part("base")
    first = object_model.get_part("segment_0")
    ctx.expect_gap(
        first,
        base,
        axis="x",
        positive_elem="proximal_lug",
        negative_elem="root_yoke",
        min_gap=0.0,
        max_gap=0.001,
        name="root bracket sits just behind first link",
    )

    for index in range(3):
        parent = object_model.get_part(f"segment_{index}")
        child = object_model.get_part(f"segment_{index + 1}")
        ctx.expect_gap(
            child,
            parent,
            axis="x",
            positive_elem="proximal_lug",
            negative_elem="distal_lug",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"segment_{index} hinge faces meet segment_{index + 1}",
        )

    tip = object_model.get_part("segment_3")
    straight_tip = ctx.part_world_position(tip)
    with ctx.pose(
        {
            joints[0]: pi / 2.0,
            joints[1]: -pi / 2.0,
            joints[2]: pi / 2.0,
            joints[3]: -pi / 2.0,
        }
    ):
        folded_tip = ctx.part_world_position(tip)
    ctx.check(
        "alternating joints fold the boom sideways",
        straight_tip is not None
        and folded_tip is not None
        and abs(folded_tip[1] - straight_tip[1]) > 0.30,
        details=f"straight={straight_tip}, folded={folded_tip}",
    )

    return ctx.report()


object_model = build_object_model()
