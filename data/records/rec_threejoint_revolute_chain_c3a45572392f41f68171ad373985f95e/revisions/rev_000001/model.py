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


JOINT_AXIS = (0.0, -1.0, 0.0)
BASE_JOINT_X = -0.09
BASE_JOINT_Z = 0.125
LINK_LENGTHS = (0.205, 0.180, 0.155)


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def _add_axle_caps(part, *, x: float, z: float, prefix: str) -> None:
    for side, y in (("pos", 0.032), ("neg", -0.032)):
        part.visual(
            Cylinder(radius=0.017, length=0.004),
            origin=_y_cylinder_origin(x, y, z),
            material="brushed_pin",
            name=f"{prefix}_cap_{side}",
        )


def _add_fork(part, *, x: float, z: float, prefix: str) -> None:
    for side, y in (("pos", 0.026), ("neg", -0.026)):
        part.visual(
            Box((0.100, 0.008, 0.078)),
            origin=Origin(xyz=(x, y, z)),
            material="clear_polycarbonate",
            name=f"{prefix}_plate_{side}",
        )
    part.visual(
        Box((0.035, 0.060, 0.026)),
        origin=Origin(xyz=(x - 0.060, 0.0, z)),
        material="dark_anodized",
        name=f"{prefix}_hinge_block",
    )
    _add_axle_caps(part, x=x, z=z, prefix=prefix)


def _add_link(part, *, length: float, has_fork: bool, has_end_tab: bool = False) -> None:
    part.visual(
        Cylinder(radius=0.028, length=0.044),
        origin=_y_cylinder_origin(0.0, 0.0, 0.0),
        material="satin_aluminum",
        name="proximal_hub",
    )

    bar_end = length - (0.050 if has_fork else 0.020)
    bar_start = 0.025
    part.visual(
        Box((bar_end - bar_start, 0.022, 0.022)),
        origin=Origin(xyz=((bar_start + bar_end) / 2.0, 0.0, 0.0)),
        material="satin_aluminum",
        name="link_bar",
    )

    part.visual(
        Box((0.048, 0.016, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, -0.021)),
        material="dark_anodized",
        name="hub_web",
    )

    if has_fork:
        _add_fork(part, x=length, z=0.0, prefix="distal")

    if has_end_tab:
        part.visual(
            Box((0.055, 0.050, 0.018)),
            origin=Origin(xyz=(length - 0.004, 0.0, 0.0)),
            material="end_tab_red",
            name="end_tab",
        )
        part.visual(
            Cylinder(radius=0.014, length=0.054),
            origin=_y_cylinder_origin(length - 0.004, 0.0, 0.0),
            material="brushed_pin",
            name="tab_bushing",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_three_revolute_chain")
    model.material("matte_black", rgba=(0.025, 0.027, 0.030, 1.0))
    model.material("dark_anodized", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("satin_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("brushed_pin", rgba=(0.86, 0.84, 0.78, 1.0))
    model.material("clear_polycarbonate", rgba=(0.55, 0.82, 0.95, 0.34))
    model.material("rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    model.material("end_tab_red", rgba=(0.78, 0.12, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.300, 0.170, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material="matte_black",
        name="base_foot",
    )
    for x in (-0.110, 0.110):
        for y in (-0.060, 0.060):
            base.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(x, y, 0.003)),
                material="rubber",
                name=f"rubber_pad_{x}_{y}",
            )

    base.visual(
        Box((0.055, 0.064, 0.075)),
        origin=Origin(xyz=(-0.145, 0.0, 0.066)),
        material="dark_anodized",
        name="rear_pedestal",
    )
    base.visual(
        Box((0.025, 0.060, 0.040)),
        origin=Origin(xyz=(-0.135, 0.0, 0.095)),
        material="dark_anodized",
        name="base_hinge_block",
    )
    for side, y in (("pos", 0.026), ("neg", -0.026)):
        base.visual(
            Box((0.090, 0.008, 0.080)),
            origin=Origin(xyz=(BASE_JOINT_X, y, BASE_JOINT_Z)),
            material="clear_polycarbonate",
            name=f"joint_0_plate_{side}",
        )
    _add_axle_caps(base, x=BASE_JOINT_X, z=BASE_JOINT_Z, prefix="joint_0")

    link_0 = model.part("link_0")
    _add_link(link_0, length=LINK_LENGTHS[0], has_fork=True)

    link_1 = model.part("link_1")
    _add_link(link_1, length=LINK_LENGTHS[1], has_fork=True)

    link_2 = model.part("link_2")
    _add_link(link_2, length=LINK_LENGTHS[2], has_fork=False, has_end_tab=True)

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(BASE_JOINT_X, 0.0, BASE_JOINT_Z)),
        axis=JOINT_AXIS,
        motion_limits=MotionLimits(lower=-1.10, upper=1.25, effort=18.0, velocity=1.4),
    )
    model.articulation(
        "hinge_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, 0.0)),
        axis=JOINT_AXIS,
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=12.0, velocity=1.8),
    )
    model.articulation(
        "hinge_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        axis=JOINT_AXIS,
        motion_limits=MotionLimits(lower=-1.45, upper=1.45, effort=8.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    base_hinge = object_model.get_articulation("base_hinge")
    hinge_1 = object_model.get_articulation("hinge_1")
    hinge_2 = object_model.get_articulation("hinge_2")

    joints = (base_hinge, hinge_1, hinge_2)
    ctx.check(
        "three revolute hinges",
        len(joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "hinge axes are parallel",
        all(tuple(j.axis) == JOINT_AXIS for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    hinge_pairs = (
        (base, link_0, "joint_0_plate_pos", "proximal_hub", "base hinge"),
        (link_0, link_1, "distal_plate_pos", "proximal_hub", "middle hinge"),
        (link_1, link_2, "distal_plate_pos", "proximal_hub", "distal hinge"),
    )
    for parent, child, plate, hub, label in hinge_pairs:
        ctx.expect_overlap(
            parent,
            child,
            axes="xz",
            elem_a=plate,
            elem_b=hub,
            min_overlap=0.045,
            name=f"{label} plates surround hub",
        )
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            positive_elem=plate,
            negative_elem=hub,
            min_gap=0.0,
            max_gap=0.001,
            name=f"{label} side clearance",
        )

    rest_joint_1 = ctx.part_world_position(link_1)
    with ctx.pose({base_hinge: 0.60}):
        raised_joint_1 = ctx.part_world_position(link_1)
    ctx.check(
        "base hinge raises first span",
        rest_joint_1 is not None
        and raised_joint_1 is not None
        and raised_joint_1[2] > rest_joint_1[2] + 0.08,
        details=f"rest={rest_joint_1}, raised={raised_joint_1}",
    )

    return ctx.report()


object_model = build_object_model()
