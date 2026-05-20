from __future__ import annotations

from math import pi

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
)


LINK_MAT = Material("anodized_dark_blue", rgba=(0.08, 0.13, 0.20, 1.0))
BLOCK_MAT = Material("machined_hinge_blocks", rgba=(0.42, 0.45, 0.48, 1.0))
PIN_MAT = Material("brushed_steel_pins", rgba=(0.78, 0.76, 0.70, 1.0))
BASE_MAT = Material("matte_black_base", rgba=(0.02, 0.025, 0.03, 1.0))


AXIS_Y = Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _add_y_axis_cylinder(part, *, x: float, y: float, z: float, radius: float, length: float, name: str, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_distal_clevis(part, *, x: float, z: float, prefix: str) -> None:
    """A compact forked hinge block whose cheeks receive the next link's pin."""
    part.visual(
        Box((0.052, 0.014, 0.058)),
        origin=Origin(xyz=(x, 0.039, z)),
        material=BLOCK_MAT,
        name=f"{prefix}_cheek_pos",
    )
    part.visual(
        Box((0.052, 0.014, 0.058)),
        origin=Origin(xyz=(x, -0.039, z)),
        material=BLOCK_MAT,
        name=f"{prefix}_cheek_neg",
    )
    # The rear bridge makes the fork a single manufactured block without
    # obstructing the child hinge tongue at the pin center.
    part.visual(
        Box((0.052, 0.086, 0.036)),
        origin=Origin(xyz=(x - 0.048, 0.0, z)),
        material=BLOCK_MAT,
        name=f"{prefix}_bridge",
    )


def _add_proximal_pin_block(part, *, prefix: str) -> None:
    part.visual(
        Box((0.038, 0.044, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=BLOCK_MAT,
        name=f"{prefix}_knuckle",
    )
    _add_y_axis_cylinder(
        part,
        x=0.0,
        y=0.0,
        z=0.0,
        radius=0.010,
        length=0.104,
        material=PIN_MAT,
        name=f"{prefix}_pin",
    )


def _add_standard_link(part, *, prefix: str, length: float) -> None:
    _add_proximal_pin_block(part, prefix=prefix)
    part.visual(
        Box((length - 0.070, 0.034, 0.028)),
        origin=Origin(xyz=((length - 0.070) / 2.0 + 0.018, 0.0, 0.0)),
        material=LINK_MAT,
        name=f"{prefix}_bar",
    )
    _add_distal_clevis(part, x=length, z=0.0, prefix=f"{prefix}_distal")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_three_joint_revolute_chain")

    root_link = model.part("root_link")
    root_link.visual(
        Box((0.260, 0.150, 0.020)),
        origin=Origin(xyz=(0.065, 0.0, 0.010)),
        material=BASE_MAT,
        name="ground_plate",
    )
    root_link.visual(
        Box((0.050, 0.058, 0.122)),
        origin=Origin(xyz=(0.040, 0.0, 0.070)),
        material=BLOCK_MAT,
        name="root_pedestal",
    )
    root_link.visual(
        Box((0.092, 0.014, 0.126)),
        origin=Origin(xyz=(0.070, 0.046, 0.074)),
        material=BLOCK_MAT,
        name="bracket_rib_pos",
    )
    root_link.visual(
        Box((0.092, 0.014, 0.126)),
        origin=Origin(xyz=(0.070, -0.046, 0.074)),
        material=BLOCK_MAT,
        name="bracket_rib_neg",
    )
    root_link.visual(
        Box((0.180, 0.042, 0.032)),
        origin=Origin(xyz=(0.102, 0.0, 0.122)),
        material=LINK_MAT,
        name="root_bar",
    )
    _add_distal_clevis(root_link, x=0.210, z=0.122, prefix="joint_0")

    link_1 = model.part("link_1")
    _add_standard_link(link_1, prefix="link_1", length=0.205)

    link_2 = model.part("link_2")
    _add_standard_link(link_2, prefix="link_2", length=0.195)

    end_tab = model.part("end_tab")
    _add_proximal_pin_block(end_tab, prefix="end_tab")
    end_tab.visual(
        Box((0.105, 0.040, 0.022)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=LINK_MAT,
        name="tab_plate",
    )
    _add_y_axis_cylinder(
        end_tab,
        x=0.124,
        y=0.0,
        z=0.0,
        radius=0.021,
        length=0.040,
        material=LINK_MAT,
        name="rounded_tip",
    )

    limits = MotionLimits(effort=18.0, velocity=2.5, lower=-1.15, upper=1.15)
    model.articulation(
        "joint_0",
        ArticulationType.REVOLUTE,
        parent=root_link,
        child=link_1,
        origin=Origin(xyz=(0.210, 0.0, 0.122)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "joint_1",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "joint_2",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_tab,
        origin=Origin(xyz=(0.195, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root_link = object_model.get_part("root_link")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_tab = object_model.get_part("end_tab")
    joint_0 = object_model.get_articulation("joint_0")
    joint_1 = object_model.get_articulation("joint_1")
    joint_2 = object_model.get_articulation("joint_2")

    hinge_pairs = (
        (root_link, link_1, "joint_0", "link_1"),
        (link_1, link_2, "link_1_distal", "link_2"),
        (link_2, end_tab, "link_2_distal", "end_tab"),
    )
    for parent, child, clevis_prefix, child_prefix in hinge_pairs:
        for cheek in ("pos", "neg"):
            ctx.allow_overlap(
                parent,
                child,
                elem_a=f"{clevis_prefix}_cheek_{cheek}",
                elem_b=f"{child_prefix}_pin",
                reason="The steel hinge pin is intentionally captured through the simplified solid clevis cheek.",
            )
            ctx.expect_overlap(
                parent,
                child,
                axes="xyz",
                elem_a=f"{clevis_prefix}_cheek_{cheek}",
                elem_b=f"{child_prefix}_pin",
                min_overlap=0.006,
                name=f"{clevis_prefix} pin passes through {cheek} cheek",
            )

    ctx.check(
        "four rigid chain links",
        {part.name for part in object_model.parts} == {"root_link", "link_1", "link_2", "end_tab"},
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "three serial revolute joints",
        all(joint.articulation_type == ArticulationType.REVOLUTE for joint in (joint_0, joint_1, joint_2)),
        details="Expected joint_0, joint_1, and joint_2 to be revolute.",
    )
    ctx.check(
        "parallel hinge axes",
        all(tuple(joint.axis) == (0.0, 1.0, 0.0) for joint in (joint_0, joint_1, joint_2)),
        details=f"axes={[joint.axis for joint in (joint_0, joint_1, joint_2)]}",
    )

    rest_tip = ctx.part_world_position(end_tab)
    with ctx.pose({joint_0: 0.55, joint_1: -0.35, joint_2: 0.45}):
        posed_tip = ctx.part_world_position(end_tab)

    ctx.check(
        "end tab responds to serial joints",
        rest_tip is not None and posed_tip is not None and abs(posed_tip[2] - rest_tip[2]) > 0.030,
        details=f"rest={rest_tip}, posed={posed_tip}",
    )
    ctx.check(
        "chain remains in one hinge plane",
        posed_tip is not None and abs(posed_tip[1]) < 0.002,
        details=f"posed end-tab origin={posed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
