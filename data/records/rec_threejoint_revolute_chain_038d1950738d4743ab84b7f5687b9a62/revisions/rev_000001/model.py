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


PIN_RPY = (-math.pi / 2.0, 0.0, 0.0)


def _add_y_cylinder(part, radius: float, length: float, xyz, material, name: str) -> None:
    """Add a cylinder whose axis is the hinge-chain Y pin axis."""

    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=PIN_RPY),
        material=material,
        name=name,
    )


def _add_boxed_link(part, *, length: float, material, pin_material, accent_material) -> None:
    """Box-section link with a center knuckle at its root and a fork at its tip."""

    # Root-side center knuckle: the child member captured between the parent's
    # two outside clevis barrels.
    part.visual(
        Cylinder(radius=0.023, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=PIN_RPY),
        material=pin_material,
        name="proximal_knuckle",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=PIN_RPY),
        material=pin_material,
        name="proximal_pin",
    )
    part.visual(
        Box((0.076, 0.052, 0.036)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=material,
        name="proximal_web",
    )

    # Rectangular boxed beam between hinge stations.  Four visible flanges and
    # two end webs keep the member reading as a rigid fabricated box, not a
    # single featureless bar.
    beam_start = 0.060
    beam_end = length - 0.066
    beam_len = beam_end - beam_start
    beam_x = (beam_start + beam_end) / 2.0
    part.visual(
        Box((beam_len, 0.030, 0.024)),
        origin=Origin(xyz=(beam_x, 0.0, 0.0)),
        material=material,
        name="central_web",
    )
    part.visual(
        Box((beam_len, 0.020, 0.044)),
        origin=Origin(xyz=(beam_x, 0.033, 0.0)),
        material=material,
        name="side_wall_0",
    )
    part.visual(
        Box((beam_len, 0.020, 0.044)),
        origin=Origin(xyz=(beam_x, -0.033, 0.0)),
        material=material,
        name="side_wall_1",
    )
    part.visual(
        Box((beam_len, 0.088, 0.010)),
        origin=Origin(xyz=(beam_x, 0.0, 0.027)),
        material=material,
        name="top_flange",
    )
    part.visual(
        Box((beam_len, 0.088, 0.010)),
        origin=Origin(xyz=(beam_x, 0.0, -0.027)),
        material=material,
        name="bottom_flange",
    )

    # Fork bridge and twin cheeks at the distal hinge.  The bridge is set back
    # from the pin axis, leaving real clearance for the next center knuckle.
    part.visual(
        Box((0.034, 0.118, 0.050)),
        origin=Origin(xyz=(length - 0.064, 0.0, 0.0)),
        material=material,
        name="distal_bridge",
    )
    part.visual(
        Box((0.086, 0.026, 0.018)),
        origin=Origin(xyz=(length - 0.030, 0.052, 0.032)),
        material=material,
        name="distal_cheek_0_upper",
    )
    part.visual(
        Box((0.086, 0.026, 0.018)),
        origin=Origin(xyz=(length - 0.030, 0.052, -0.032)),
        material=material,
        name="distal_cheek_0_lower",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(length, 0.052, 0.0), rpy=PIN_RPY),
        material=pin_material,
        name="distal_barrel_0",
    )
    part.visual(
        Box((0.086, 0.026, 0.018)),
        origin=Origin(xyz=(length - 0.030, -0.052, 0.032)),
        material=material,
        name="distal_cheek_1_upper",
    )
    part.visual(
        Box((0.086, 0.026, 0.018)),
        origin=Origin(xyz=(length - 0.030, -0.052, -0.032)),
        material=material,
        name="distal_cheek_1_lower",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(length, -0.052, 0.0), rpy=PIN_RPY),
        material=pin_material,
        name="distal_barrel_1",
    )

    # Thin bronze-colored washers at the center knuckle face make the coaxial
    # supported pin stations visually legible while staying within the link.
    part.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.0, 0.033, 0.0), rpy=PIN_RPY),
        material=accent_material,
        name="washer_0",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.0, -0.033, 0.0), rpy=PIN_RPY),
        material=accent_material,
        name="washer_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_three_link_hinge_chain")

    powder_blue = model.material("powder_blue", rgba=(0.16, 0.30, 0.44, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    bronze = model.material("bronze_bushings", rgba=(0.85, 0.55, 0.22, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    root = model.part("root_clevis")
    root.visual(
        Box((0.230, 0.230, 0.035)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0175)),
        material=powder_blue,
        name="base_plate",
    )
    root.visual(
        Box((0.050, 0.170, 0.230)),
        origin=Origin(xyz=(-0.074, 0.0, 0.150)),
        material=powder_blue,
        name="rear_box",
    )
    root.visual(
        Box((0.038, 0.134, 0.114)),
        origin=Origin(xyz=(-0.052, 0.0, 0.248)),
        material=powder_blue,
        name="back_bridge",
    )
    root.visual(
        Box((0.094, 0.030, 0.222)),
        origin=Origin(xyz=(0.0, 0.052, 0.146)),
        material=powder_blue,
        name="root_cheek_0_lower",
    )
    root.visual(
        Box((0.094, 0.030, 0.021)),
        origin=Origin(xyz=(0.0, 0.052, 0.3155)),
        material=powder_blue,
        name="root_cheek_0_upper",
    )
    root.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.0, 0.052, 0.280), rpy=PIN_RPY),
        material=pin_steel,
        name="root_barrel_0",
    )
    root.visual(
        Box((0.094, 0.030, 0.222)),
        origin=Origin(xyz=(0.0, -0.052, 0.146)),
        material=powder_blue,
        name="root_cheek_1_lower",
    )
    root.visual(
        Box((0.094, 0.030, 0.021)),
        origin=Origin(xyz=(0.0, -0.052, 0.3155)),
        material=powder_blue,
        name="root_cheek_1_upper",
    )
    root.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.0, -0.052, 0.280), rpy=PIN_RPY),
        material=pin_steel,
        name="root_barrel_1",
    )

    link_0 = model.part("link_0")
    _add_boxed_link(link_0, length=0.300, material=dark_steel, pin_material=pin_steel, accent_material=bronze)

    link_1 = model.part("link_1")
    _add_boxed_link(link_1, length=0.270, material=dark_steel, pin_material=pin_steel, accent_material=bronze)

    output_pad = model.part("output_pad")
    _add_y_cylinder(output_pad, 0.023, 0.062, (0.0, 0.0, 0.0), pin_steel, "proximal_knuckle")
    _add_y_cylinder(output_pad, 0.010, 0.150, (0.0, 0.0, 0.0), pin_steel, "proximal_pin")
    output_pad.visual(
        Box((0.165, 0.055, 0.036)),
        origin=Origin(xyz=(0.094, 0.0, 0.0)),
        material=dark_steel,
        name="pad_stem",
    )
    output_pad.visual(
        Box((0.105, 0.145, 0.040)),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=dark_steel,
        name="pad_plate",
    )
    output_pad.visual(
        Box((0.012, 0.130, 0.030)),
        origin=Origin(xyz=(0.2635, 0.0, 0.0)),
        material=rubber,
        name="rubber_face",
    )

    joint_limits = MotionLimits(effort=25.0, velocity=2.5, lower=-1.05, upper=1.05)
    model.articulation(
        "root_to_link_0",
        ArticulationType.REVOLUTE,
        parent=root,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_1_to_pad",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=output_pad,
        origin=Origin(xyz=(0.270, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.90, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_clevis")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    output_pad = object_model.get_part("output_pad")
    j0 = object_model.get_articulation("root_to_link_0")
    j1 = object_model.get_articulation("link_0_to_link_1")
    j2 = object_model.get_articulation("link_1_to_pad")

    pin_interfaces = (
        (root, link_0, ("root_barrel_0", "root_barrel_1")),
        (link_0, link_1, ("distal_barrel_0", "distal_barrel_1")),
        (link_1, output_pad, ("distal_barrel_0", "distal_barrel_1")),
    )
    for parent, child, barrels in pin_interfaces:
        for barrel in barrels:
            ctx.allow_overlap(
                parent,
                child,
                elem_a=barrel,
                elem_b="proximal_pin",
                reason="The steel hinge pin is intentionally captured inside the supported clevis barrel.",
            )
            ctx.expect_within(
                child,
                parent,
                axes="xz",
                inner_elem="proximal_pin",
                outer_elem=barrel,
                margin=0.0,
                name=f"{child.name} pin is concentric within {parent.name} {barrel}",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="y",
                elem_a="proximal_pin",
                elem_b=barrel,
                min_overlap=0.025,
                name=f"{child.name} pin is retained in {parent.name} {barrel}",
            )

    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "four rigid members in series",
        len(object_model.parts) == 4
        and [p.name for p in object_model.parts] == ["root_clevis", "link_0", "link_1", "output_pad"],
        details=f"parts={[p.name for p in object_model.parts]}",
    )
    ctx.check(
        "parallel pin axes",
        all(tuple(j.axis) == (0.0, 1.0, 0.0) for j in (j0, j1, j2)),
        details=f"axes={[j.axis for j in (j0, j1, j2)]}",
    )
    ctx.expect_gap(
        root,
        link_0,
        axis="y",
        min_gap=-0.001,
        max_gap=0.036,
        positive_elem="root_barrel_0",
        negative_elem="washer_0",
        name="root clevis captures first knuckle with side clearance",
    )
    ctx.expect_gap(
        link_0,
        link_1,
        axis="y",
        min_gap=-0.001,
        max_gap=0.036,
        positive_elem="distal_barrel_0",
        negative_elem="washer_0",
        name="first link fork captures second knuckle with side clearance",
    )
    ctx.expect_gap(
        link_1,
        output_pad,
        axis="y",
        min_gap=-0.001,
        max_gap=0.036,
        positive_elem="distal_barrel_0",
        negative_elem="proximal_knuckle",
        name="second link fork captures output pad knuckle",
    )

    rest_pad = ctx.part_world_position(output_pad)
    with ctx.pose({j0: 0.45, j1: -0.35, j2: 0.30}):
        moved_pad = ctx.part_world_position(output_pad)
    ctx.check(
        "serial chain moves output pad",
        rest_pad is not None
        and moved_pad is not None
        and abs(moved_pad[2] - rest_pad[2]) > 0.030
        and moved_pad[0] > rest_pad[0] - 0.080,
        details=f"rest={rest_pad}, moved={moved_pad}",
    )

    return ctx.report()


object_model = build_object_model()
