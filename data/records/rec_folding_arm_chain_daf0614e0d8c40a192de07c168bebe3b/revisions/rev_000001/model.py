from __future__ import annotations

from math import isclose, sqrt

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


LINK_0_LEN = 0.240
LINK_1_LEN = 0.230
LINK_2_LEN = 0.200


def _axis_y() -> Origin:
    """Rotate a cylinder's local Z axis onto the hinge Y axis."""
    return Origin(rpy=(1.5707963267948966, 0.0, 0.0))


def _add_central_link(part, *, length: float, material: str, prefix: str) -> None:
    plate_thickness = 0.018
    boss_radius = 0.024
    part.visual(
        Box((0.030, plate_thickness, length)),
        origin=Origin(xyz=(0.0, 0.0, -length / 2.0)),
        material=material,
        name=f"{prefix}_strap",
    )
    for name, z in (("upper_boss", 0.0), ("lower_boss", -length)):
        part.visual(
            Cylinder(radius=boss_radius, length=plate_thickness),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=_axis_y().rpy),
            material=material,
            name=f"{prefix}_{name}",
        )


def _add_outer_link(part, *, length: float, material: str, prefix: str) -> None:
    side_y = 0.022
    side_thickness = 0.010
    boss_radius = 0.023
    for side_name, y in (("front", -side_y), ("rear", side_y)):
        part.visual(
            Box((0.026, side_thickness, length)),
            origin=Origin(xyz=(0.0, y, -length / 2.0)),
            material=material,
            name=f"{prefix}_{side_name}_strap",
        )
        for boss_name, z in (("upper_boss", 0.0), ("lower_boss", -length)):
            part.visual(
                Cylinder(radius=boss_radius, length=side_thickness),
                origin=Origin(xyz=(0.0, y, z), rpy=_axis_y().rpy),
                material=material,
                name=f"{prefix}_{side_name}_{boss_name}",
            )
            part.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(0.0, y * 1.30, z), rpy=_axis_y().rpy),
                material="zinc_pin",
                name=f"{prefix}_{side_name}_{boss_name}_pin_head",
            )

    # The transverse spacer makes the twin side straps read as one rigid link,
    # while leaving the hinge eyes open for the neighboring central straps.
    part.visual(
        Box((0.022, 0.056, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -length / 2.0)),
        material=material,
        name=f"{prefix}_spacer",
    )
    for pin_name, z in (("upper_pin", 0.0), ("lower_pin", -length)):
        part.visual(
            Cylinder(radius=0.008, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=_axis_y().rpy),
            material="zinc_pin",
            name=f"{prefix}_{pin_name}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_folding_arm_chain")
    model.material("bracket_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("brushed_link", rgba=(0.63, 0.66, 0.68, 1.0))
    model.material("dark_bore", rgba=(0.045, 0.047, 0.050, 1.0))
    model.material("zinc_pin", rgba=(0.78, 0.76, 0.70, 1.0))
    model.material("rubber_pad", rgba=(0.025, 0.026, 0.025, 1.0))

    bracket = model.part("top_bracket")
    bracket.visual(
        Box((0.170, 0.100, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material="bracket_steel",
        name="mount_plate",
    )
    for y in (-0.020, 0.020):
        bracket.visual(
            Box((0.066, 0.012, 0.064)),
            origin=Origin(xyz=(0.0, y, 0.028)),
            material="bracket_steel",
            name=f"clevis_cheek_{'front' if y < 0 else 'rear'}",
        )
    for y in (-0.031, 0.031):
        bracket.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=_axis_y().rpy),
            material="zinc_pin",
            name=f"top_pin_head_{'front' if y < 0 else 'rear'}",
        )
    bracket.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(rpy=_axis_y().rpy),
        material="zinc_pin",
        name="top_pin",
    )
    for x in (-0.055, 0.055):
        for y in (-0.030, 0.030):
            bracket.visual(
                Cylinder(radius=0.006, length=0.003),
                origin=Origin(xyz=(x, y, 0.073)),
                material="dark_bore",
                name=f"mount_screw_{'n' if y > 0 else 's'}_{'e' if x > 0 else 'w'}",
            )

    link_0 = model.part("link_0")
    link_0.visual(
        Box((0.030, 0.018, LINK_0_LEN - 0.048)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_0_LEN / 2.0)),
        material="brushed_link",
        name="link0_strap",
    )
    link_0.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_axis_y().rpy),
        material="brushed_link",
        name="link0_upper_boss",
    )
    link_0.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -LINK_0_LEN), rpy=_axis_y().rpy),
        material="brushed_link",
        name="link0_lower_boss",
    )

    link_1 = model.part("link_1")
    for side_name, y in (("front", -0.022), ("rear", 0.022)):
        link_1.visual(
            Box((0.026, 0.010, LINK_1_LEN)),
            origin=Origin(xyz=(0.0, y, -LINK_1_LEN / 2.0)),
            material="brushed_link",
            name=f"link1_{side_name}_strap",
        )
    link_1.visual(
        Cylinder(radius=0.023, length=0.010),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=_axis_y().rpy),
        material="brushed_link",
        name="link1_front_upper_boss",
    )
    link_1.visual(
        Cylinder(radius=0.023, length=0.010),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=_axis_y().rpy),
        material="brushed_link",
        name="link1_rear_upper_boss",
    )
    link_1.visual(
        Cylinder(radius=0.023, length=0.010),
        origin=Origin(xyz=(0.0, -0.022, -LINK_1_LEN), rpy=_axis_y().rpy),
        material="brushed_link",
        name="link1_front_lower_boss",
    )
    link_1.visual(
        Cylinder(radius=0.023, length=0.010),
        origin=Origin(xyz=(0.0, 0.022, -LINK_1_LEN), rpy=_axis_y().rpy),
        material="brushed_link",
        name="link1_rear_lower_boss",
    )
    for boss_name, y, z in (
        ("front_upper", -0.0286, 0.0),
        ("rear_upper", 0.0286, 0.0),
        ("front_lower", -0.0286, -LINK_1_LEN),
        ("rear_lower", 0.0286, -LINK_1_LEN),
    ):
        link_1.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.0, y, z), rpy=_axis_y().rpy),
            material="zinc_pin",
            name=f"link1_{boss_name}_pin_head",
        )
    link_1.visual(
        Box((0.022, 0.056, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_1_LEN / 2.0)),
        material="brushed_link",
        name="link1_spacer",
    )
    link_1.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_axis_y().rpy),
        material="zinc_pin",
        name="link1_upper_pin",
    )
    link_1.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -LINK_1_LEN), rpy=_axis_y().rpy),
        material="zinc_pin",
        name="link1_lower_pin",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        Box((0.030, 0.018, LINK_2_LEN - 0.048)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_2_LEN / 2.0)),
        material="brushed_link",
        name="link2_strap",
    )
    link_2.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_axis_y().rpy),
        material="brushed_link",
        name="link2_upper_boss",
    )
    link_2.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -LINK_2_LEN), rpy=_axis_y().rpy),
        material="brushed_link",
        name="link2_lower_boss",
    )
    link_2.visual(
        Box((0.020, 0.018, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_2_LEN - 0.033)),
        material="brushed_link",
        name="pad_stem",
    )
    link_2.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -LINK_2_LEN - 0.062)),
        material="rubber_pad",
        name="end_pad",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=link_0,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=18.0, velocity=2.0),
    )
    model.articulation(
        "middle_hinge",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, -LINK_0_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.20, upper=2.20, effort=14.0, velocity=2.2),
    )
    model.articulation(
        "lower_hinge",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, 0.0, -LINK_1_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.20, upper=2.20, effort=10.0, velocity=2.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("top_bracket")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    joints = [
        object_model.get_articulation("top_hinge"),
        object_model.get_articulation("middle_hinge"),
        object_model.get_articulation("lower_hinge"),
    ]

    ctx.allow_overlap(
        bracket,
        link_0,
        elem_a="top_pin",
        elem_b="link0_upper_boss",
        reason="The top hinge pin is intentionally captured through the first link's upper pivot boss.",
    )
    ctx.allow_overlap(
        link_1,
        link_0,
        elem_a="link1_upper_pin",
        elem_b="link0_lower_boss",
        reason="The middle hinge pin is intentionally captured through the lower boss of the first link.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        elem_a="link1_lower_pin",
        elem_b="link2_upper_boss",
        reason="The lower hinge pin is intentionally captured through the upper boss of the final link.",
    )

    ctx.check(
        "three parallel revolute hinges",
        len(joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(_parallel(j.axis, joints[0].axis) for j in joints[1:]),
        details=f"axes={[j.axis for j in joints]}",
    )
    ctx.expect_gap(
        bracket,
        link_0,
        axis="z",
        positive_elem="mount_plate",
        negative_elem="link0_upper_boss",
        min_gap=0.025,
        name="first link is under the top support plate",
    )
    ctx.expect_within(
        bracket,
        link_0,
        axes="xz",
        inner_elem="top_pin",
        outer_elem="link0_upper_boss",
        margin=0.001,
        name="top pin passes through first link boss",
    )
    ctx.expect_within(
        link_1,
        link_0,
        axes="xz",
        inner_elem="link1_upper_pin",
        outer_elem="link0_lower_boss",
        margin=0.001,
        name="middle pin passes through first link lower boss",
    )
    ctx.expect_within(
        link_1,
        link_2,
        axes="xz",
        inner_elem="link1_lower_pin",
        outer_elem="link2_upper_boss",
        margin=0.001,
        name="lower pin passes through final link upper boss",
    )
    ctx.expect_origin_gap(link_0, link_1, axis="z", min_gap=0.20, name="second hinge hangs below first")
    ctx.expect_origin_gap(link_1, link_2, axis="z", min_gap=0.19, name="third hinge hangs below second")
    ctx.expect_overlap(
        link_1,
        link_2,
        axes="z",
        elem_a="link1_front_lower_boss",
        elem_b="link2_upper_boss",
        min_overlap=0.030,
        name="lower hinge eyes are co-located along the chain",
    )

    rest_tip = ctx.part_world_position(link_2)
    with ctx.pose({"top_hinge": 0.65, "middle_hinge": -0.45, "lower_hinge": 0.30}):
        folded_tip = ctx.part_world_position(link_2)
    ctx.check(
        "folding pose swings the hanging chain sideways",
        rest_tip is not None
        and folded_tip is not None
        and abs(folded_tip[0] - rest_tip[0]) > 0.10,
        details=f"rest={rest_tip}, folded={folded_tip}",
    )

    return ctx.report()


def _parallel(a: tuple[float, float, float], b: tuple[float, float, float]) -> bool:
    an = sqrt(sum(v * v for v in a))
    bn = sqrt(sum(v * v for v in b))
    if isclose(an, 0.0) or isclose(bn, 0.0):
        return False
    dot = sum(x * y for x, y in zip(a, b)) / (an * bn)
    return abs(dot) > 0.999


object_model = build_object_model()
