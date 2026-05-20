from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FIRST_LINK_LENGTH = 0.82


def _arched_plate_mesh(name: str, *, y_center: float):
    """One flat cheek of the arched first link, extruded in the joint-axis direction."""
    upper: list[tuple[float, float]] = []
    lower: list[tuple[float, float]] = []
    samples = 24
    for i in range(samples + 1):
        t = i / samples
        x = FIRST_LINK_LENGTH * t
        center_z = 0.145 * math.sin(math.pi * t)
        half_depth = 0.046 + 0.012 * math.sin(math.pi * t)
        upper.append((x, center_z + half_depth))
        lower.append((x, center_z - half_depth))

    profile = upper + list(reversed(lower))
    geom = ExtrudeGeometry(profile, 0.035, center=True)
    # ExtrudeGeometry builds in local XY and extrudes along local Z.  Rotate so
    # profile Y becomes world Z, and extrusion thickness becomes world Y.
    geom.rotate_x(math.pi / 2.0).translate(0.0, y_center, 0.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_mounted_transfer_arm")

    bracket_mat = model.material("graphite_painted_steel", rgba=(0.16, 0.18, 0.20, 1.0))
    arm_mat = model.material("safety_yellow_arm", rgba=(0.92, 0.64, 0.12, 1.0))
    dark_mat = model.material("blackened_pin_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    plate_mat = model.material("brushed_end_plate", rgba=(0.62, 0.64, 0.63, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        Box((0.48, 0.080, 0.70)),
        origin=Origin(xyz=(0.0, -0.190, 0.0)),
        material=bracket_mat,
        name="wall_plate",
    )
    root_bracket.visual(
        Box((0.31, 0.10, 0.34)),
        origin=Origin(xyz=(0.0, -0.125, 0.0)),
        material=bracket_mat,
        name="mounting_block",
    )
    root_bracket.visual(
        Box((0.09, 0.115, 0.60)),
        origin=Origin(xyz=(0.0, -0.132, 0.0)),
        material=bracket_mat,
        name="vertical_web",
    )
    root_bracket.visual(
        Box((0.36, 0.105, 0.070)),
        origin=Origin(xyz=(0.0, -0.128, 0.0)),
        material=bracket_mat,
        name="cross_web",
    )
    root_bracket.visual(
        Cylinder(radius=0.152, length=0.120),
        origin=Origin(xyz=(0.0, -0.135, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="root_bearing",
    )
    for index, (x, z) in enumerate(
        ((-0.165, -0.255), (0.165, -0.255), (-0.165, 0.255), (0.165, 0.255))
    ):
        root_bracket.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(xyz=(x, -0.142, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_mat,
            name=f"bolt_head_{index}",
        )

    first_link = model.part("first_link")
    first_link.visual(
        _arched_plate_mesh("arched_cheek_low_mesh", y_center=-0.0575),
        material=arm_mat,
        name="arched_cheek_low",
    )
    first_link.visual(
        _arched_plate_mesh("arched_cheek_high_mesh", y_center=0.0575),
        material=arm_mat,
        name="arched_cheek_high",
    )
    first_link.visual(
        Cylinder(radius=0.124, length=0.150),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_mat,
        name="root_hub",
    )
    first_link.visual(
        Cylinder(radius=0.142, length=0.035),
        origin=Origin(
            xyz=(FIRST_LINK_LENGTH, -0.0575, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=arm_mat,
        name="elbow_lug_low",
    )
    first_link.visual(
        Cylinder(radius=0.142, length=0.035),
        origin=Origin(
            xyz=(FIRST_LINK_LENGTH, 0.0575, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=arm_mat,
        name="elbow_lug_high",
    )

    terminal_link = model.part("terminal_link")
    terminal_link.visual(
        Cylinder(radius=0.085, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="elbow_hub",
    )
    terminal_link.visual(
        Box((0.560, 0.045, 0.055)),
        origin=Origin(xyz=(0.320, 0.0, 0.0)),
        material=arm_mat,
        name="terminal_bar",
    )
    terminal_link.visual(
        Box((0.040, 0.160, 0.130)),
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        material=plate_mat,
        name="tool_plate",
    )
    for index, z in enumerate((-0.038, 0.038)):
        terminal_link.visual(
            Cylinder(radius=0.015, length=0.014),
            origin=Origin(xyz=(0.647, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_mat,
            name=f"plate_fastener_{index}",
        )

    model.articulation(
        "root_joint",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=first_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=0.95, effort=180.0, velocity=1.4),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=terminal_link,
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0), rpy=(0.0, 0.64, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.15, upper=1.10, effort=95.0, velocity=1.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    first_link = object_model.get_part("first_link")
    terminal_link = object_model.get_part("terminal_link")
    root_joint = object_model.get_articulation("root_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

    ctx.check(
        "two serial revolute axes",
        root_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.articulation_type == ArticulationType.REVOLUTE
        and root_joint.parent == "root_bracket"
        and root_joint.child == "first_link"
        and elbow_joint.parent == "first_link"
        and elbow_joint.child == "terminal_link"
        and tuple(root_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(elbow_joint.axis) == (0.0, 1.0, 0.0),
        details=f"root={root_joint}, elbow={elbow_joint}",
    )

    ctx.expect_gap(
        first_link,
        root_bracket,
        axis="y",
        positive_elem="root_hub",
        negative_elem="root_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="root hub seats against bracket bearing",
    )
    ctx.expect_gap(
        terminal_link,
        first_link,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_lug_low",
        max_gap=0.001,
        max_penetration=0.0,
        name="terminal hub bears on lower elbow cheek",
    )
    ctx.expect_gap(
        first_link,
        terminal_link,
        axis="y",
        positive_elem="elbow_lug_high",
        negative_elem="elbow_hub",
        max_gap=0.001,
        max_penetration=0.0,
        name="terminal hub bears on upper elbow cheek",
    )
    ctx.expect_origin_gap(
        terminal_link,
        root_bracket,
        axis="x",
        min_gap=0.78,
        max_gap=0.86,
        name="elbow joint sits at the arched link end",
    )

    def center_of_aabb(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_elbow = ctx.part_world_position(terminal_link)
    with ctx.pose({root_joint: 0.55}):
        swept_elbow = ctx.part_world_position(terminal_link)
    ctx.check(
        "root joint sweeps the elbow in the reaching plane",
        rest_elbow is not None
        and swept_elbow is not None
        and swept_elbow[2] < rest_elbow[2] - 0.30
        and abs(swept_elbow[1] - rest_elbow[1]) < 0.002,
        details=f"rest={rest_elbow}, swept={swept_elbow}",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(terminal_link, elem="tool_plate")
    with ctx.pose({elbow_joint: 0.65}):
        bent_plate_aabb = ctx.part_element_world_aabb(terminal_link, elem="tool_plate")
    rest_plate = center_of_aabb(rest_plate_aabb) if rest_plate_aabb is not None else None
    bent_plate = center_of_aabb(bent_plate_aabb) if bent_plate_aabb is not None else None
    ctx.check(
        "elbow joint moves the terminal plate",
        rest_plate is not None
        and bent_plate is not None
        and bent_plate[2] < rest_plate[2] - 0.20
        and abs(bent_plate[1] - rest_plate[1]) < 0.002,
        details=f"rest_plate={rest_plate}, bent_plate={bent_plate}",
    )

    return ctx.report()


object_model = build_object_model()
