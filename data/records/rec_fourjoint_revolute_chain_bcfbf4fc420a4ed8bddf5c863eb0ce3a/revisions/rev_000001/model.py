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


AXIS_Y = (0.0, 1.0, 0.0)
PIN_RPY = (-pi / 2.0, 0.0, 0.0)


def _make_link(part, length: float, *, final: bool, material: Material, accent: Material) -> None:
    """Add one slim service-chain link in a frame whose origin is the proximal pin."""
    # Central tongue captured by the parent yoke.
    part.visual(
        Box((0.070, 0.036, 0.036)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=material,
        name="proximal_lug",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=PIN_RPY),
        material=accent,
        name="proximal_boss",
    )

    bar_start = 0.035
    bar_end = length - 0.030
    part.visual(
        Box((bar_end - bar_start, 0.018, 0.024)),
        origin=Origin(xyz=((bar_start + bar_end) / 2.0, 0.0, 0.0)),
        material=material,
        name="web_bar",
    )

    if final:
        # Compact service tab: a short flattened tongue with a small rounded eye.
        part.visual(
            Box((0.066, 0.040, 0.018)),
            origin=Origin(xyz=(length - 0.012, 0.0, 0.0)),
            material=material,
            name="end_tab",
        )
        part.visual(
            Cylinder(radius=0.016, length=0.042),
            origin=Origin(xyz=(length + 0.018, 0.0, 0.0), rpy=PIN_RPY),
            material=accent,
            name="tab_boss",
        )
        return

    # Distal clevis/yoke carried by this link for the next revolute joint.
    part.visual(
        Box((0.040, 0.055, 0.018)),
        origin=Origin(xyz=(length - 0.045, 0.0, 0.0)),
        material=material,
        name="distal_bridge",
    )
    part.visual(
        Box((0.050, 0.010, 0.050)),
        origin=Origin(xyz=(length, 0.023, 0.0)),
        material=material,
        name="distal_cheek_0",
    )
    part.visual(
        Box((0.050, 0.010, 0.050)),
        origin=Origin(xyz=(length, -0.023, 0.0)),
        material=material,
        name="distal_cheek_1",
    )
    for side, y in (("cap_0", 0.031), ("cap_1", -0.031)):
        part.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(length, y, 0.0), rpy=PIN_RPY),
            material=accent,
            name=f"distal_{side}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_service_four_joint_chain")

    dark_steel = model.material("dark_steel", rgba=(0.12, 0.14, 0.15, 1.0))
    brushed = model.material("brushed_link", rgba=(0.62, 0.66, 0.68, 1.0))
    pin_blue = model.material("pin_blue", rgba=(0.08, 0.22, 0.45, 1.0))
    safety_orange = model.material("service_orange", rgba=(0.95, 0.42, 0.08, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box((0.180, 0.085, 0.034)),
        origin=Origin(xyz=(-0.080, 0.0, 0.017)),
        material=dark_steel,
        name="ground_foot",
    )
    spine.visual(
        Box((0.034, 0.040, 0.330)),
        origin=Origin(xyz=(-0.090, 0.0, 0.182)),
        material=dark_steel,
        name="upright_spine",
    )
    spine.visual(
        Box((0.100, 0.055, 0.020)),
        origin=Origin(xyz=(-0.075, 0.0, 0.350)),
        material=dark_steel,
        name="shoulder_bridge",
    )
    spine.visual(
        Box((0.050, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, 0.023, 0.350)),
        material=dark_steel,
        name="shoulder_cheek_0",
    )
    spine.visual(
        Box((0.050, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, -0.023, 0.350)),
        material=dark_steel,
        name="shoulder_cheek_1",
    )
    for side, y in (("cap_0", 0.031), ("cap_1", -0.031)):
        spine.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.350), rpy=PIN_RPY),
            material=pin_blue,
            name=f"shoulder_{side}",
        )

    lengths = (0.240, 0.220, 0.195, 0.155)
    links = []
    for index, length in enumerate(lengths):
        link = model.part(f"link_{index}")
        _make_link(
            link,
            length,
            final=index == len(lengths) - 1,
            material=safety_orange if index == len(lengths) - 1 else brushed,
            accent=pin_blue,
        )
        links.append(link)

    joint_limits = MotionLimits(effort=18.0, velocity=2.5, lower=-1.35, upper=1.35)
    model.articulation(
        "joint_0",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=links[0],
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=AXIS_Y,
        motion_limits=joint_limits,
    )
    for index in range(1, len(links)):
        model.articulation(
            f"joint_{index}",
            ArticulationType.REVOLUTE,
            parent=links[index - 1],
            child=links[index],
            origin=Origin(xyz=(lengths[index - 1], 0.0, 0.0)),
            axis=AXIS_Y,
            motion_limits=joint_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [object_model.get_articulation(f"joint_{i}") for i in range(4)]
    expected_serial_pairs = [
        ("spine", "link_0"),
        ("link_0", "link_1"),
        ("link_1", "link_2"),
        ("link_2", "link_3"),
    ]
    ctx.check(
        "four serial parallel revolute joints",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(j.axis) == AXIS_Y for j in joints)
        and [(j.parent, j.child) for j in joints] == expected_serial_pairs,
        details=f"joints={[(j.name, j.articulation_type, j.axis, j.parent, j.child) for j in joints]}",
    )

    ctx.expect_contact(
        "link_0",
        "spine",
        elem_a="proximal_lug",
        elem_b="shoulder_cheek_0",
        name="joint 0 lug contacts first cheek",
    )
    ctx.expect_contact(
        "link_0",
        "spine",
        elem_a="proximal_lug",
        elem_b="shoulder_cheek_1",
        name="joint 0 lug contacts second cheek",
    )
    ctx.expect_overlap(
        "link_0",
        "spine",
        axes="xz",
        elem_a="proximal_lug",
        elem_b="shoulder_cheek_0",
        min_overlap=0.030,
        name="joint 0 cheek and lug share hinge station",
    )

    for index in range(1, 4):
        child = f"link_{index}"
        parent = f"link_{index - 1}"
        ctx.expect_contact(
            child,
            parent,
            elem_a="proximal_lug",
            elem_b="distal_cheek_0",
            name=f"joint {index} lug contacts first cheek",
        )
        ctx.expect_contact(
            child,
            parent,
            elem_a="proximal_lug",
            elem_b="distal_cheek_1",
            name=f"joint {index} lug contacts second cheek",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="xz",
            elem_a="proximal_lug",
            elem_b="distal_cheek_0",
            min_overlap=0.030,
            name=f"joint {index} cheek and lug share hinge station",
        )

    terminal = object_model.get_part("link_3")
    rest_pos = ctx.part_world_position(terminal)
    with ctx.pose({joints[1]: 0.60}):
        moved_pos = ctx.part_world_position(terminal)
    ctx.check(
        "middle joint moves the downstream chain",
        rest_pos is not None
        and moved_pos is not None
        and abs(moved_pos[2] - rest_pos[2]) > 0.08
        and moved_pos[0] < rest_pos[0] - 0.02,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
