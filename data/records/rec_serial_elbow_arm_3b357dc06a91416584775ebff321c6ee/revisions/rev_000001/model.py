from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def make_root_bracket() -> cq.Workplane:
    plate = cq.Workplane("XZ").box(0.18, 0.32, 0.03).translate((0.0, -0.015, 0.0))
    boss = cq.Workplane("XZ").circle(0.07).extrude(0.05).translate((0.0, -0.05, 0.0))
    lower_foot = (
        cq.Workplane("XZ")
        .box(0.10, 0.08, 0.06)
        .translate((0.015, -0.03, -0.11))
    )
    support_web = (
        cq.Workplane("XZ")
        .moveTo(-0.02, -0.13)
        .lineTo(0.07, -0.13)
        .lineTo(0.08, -0.035)
        .threePointArc((0.035, -0.012), (-0.02, -0.02))
        .close()
        .extrude(0.028)
        .translate((0.0, -0.028, 0.0))
    )

    body = plate.union(boss).union(lower_foot).union(support_web)

    bolt_cutters = (
        cq.Workplane("XZ")
        .pushPoints(
            [
                (-0.055, 0.105),
                (0.055, 0.105),
                (-0.055, -0.105),
                (0.055, -0.105),
            ]
        )
        .circle(0.0085)
        .extrude(0.08)
        .translate((0.0, -0.06, 0.0))
    )
    return body.cut(bolt_cutters)


def make_first_link() -> cq.Workplane:
    root_collar = cq.Workplane("XZ").circle(0.064).extrude(0.05)
    elbow_collar = (
        cq.Workplane("XZ")
        .circle(0.076)
        .extrude(0.05)
        .translate((0.42, 0.0, 0.10))
    )
    arch_web = (
        cq.Workplane("XZ")
        .moveTo(0.028, 0.060)
        .threePointArc((0.17, 0.205), (0.385, 0.145))
        .lineTo(0.400, 0.090)
        .threePointArc((0.24, 0.015), (0.055, 0.006))
        .close()
        .extrude(0.05)
    )
    lightening_cut = (
        cq.Workplane("XZ")
        .moveTo(0.115, 0.102)
        .threePointArc((0.205, 0.144), (0.300, 0.116))
        .lineTo(0.298, 0.093)
        .threePointArc((0.210, 0.082), (0.130, 0.058))
        .close()
        .extrude(0.052)
        .translate((0.0, -0.001, 0.0))
    )
    return root_collar.union(elbow_collar).union(arch_web).cut(lightening_cut).translate(
        (0.0, 0.05, 0.0)
    )


def make_terminal_beam() -> cq.Workplane:
    elbow_collar = cq.Workplane("XZ").circle(0.060).extrude(0.04)
    beam = (
        cq.Workplane("XZ")
        .moveTo(0.028, 0.032)
        .lineTo(0.28, 0.022)
        .lineTo(0.35, 0.022)
        .lineTo(0.35, -0.022)
        .lineTo(0.28, -0.022)
        .lineTo(0.028, -0.032)
        .close()
        .extrude(0.04)
    )
    return elbow_collar.union(beam).translate((0.0, 0.04, 0.0))


def make_tool_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .box(0.060, 0.100, 0.014)
        .translate((0.357, 0.020, 0.0))
    )
    return (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, 0.028), (0.0, -0.028)])
        .hole(0.010)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_transfer_arm")

    bracket_mat = model.material("bracket_steel", rgba=(0.21, 0.23, 0.26, 1.0))
    arm_mat = model.material("arm_paint", rgba=(0.62, 0.64, 0.67, 1.0))
    plate_mat = model.material("plate_finish", rgba=(0.82, 0.82, 0.80, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(make_root_bracket(), "root_bracket_shell"),
        material=bracket_mat,
        name="root_bracket_shell",
    )
    root_bracket.inertial = Inertial.from_geometry(
        Box((0.18, 0.06, 0.32)),
        mass=12.0,
        origin=Origin(xyz=(0.0, -0.03, 0.0)),
    )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(make_first_link(), "first_link_shell"),
        material=arm_mat,
        name="first_link_shell",
    )
    first_link.inertial = Inertial.from_geometry(
        Box((0.46, 0.05, 0.22)),
        mass=4.2,
        origin=Origin(xyz=(0.23, 0.025, 0.10)),
    )

    terminal_link = model.part("terminal_link")
    terminal_link.visual(
        mesh_from_cadquery(make_terminal_beam(), "terminal_link_shell"),
        material=arm_mat,
        name="terminal_link_shell",
    )
    terminal_link.visual(
        mesh_from_cadquery(make_tool_plate(), "tool_plate"),
        material=plate_mat,
        name="tool_plate",
    )
    terminal_link.inertial = Inertial.from_geometry(
        Box((0.38, 0.04, 0.12)),
        mass=2.1,
        origin=Origin(xyz=(0.19, 0.02, 0.0)),
    )

    model.articulation(
        "root_joint",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.55, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-1.10,
            upper=1.05,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=terminal_link,
        origin=Origin(xyz=(0.42, 0.05, 0.10), rpy=(0.0, -1.15, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=85.0,
            velocity=1.5,
            lower=-1.20,
            upper=1.10,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    first_link = object_model.get_part("first_link")
    terminal_link = object_model.get_part("terminal_link")
    root_joint = object_model.get_articulation("root_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    tool_plate = terminal_link.get_visual("tool_plate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "parts_present",
        all(part is not None for part in (root_bracket, first_link, terminal_link))
        and tool_plate is not None,
        "Bracket, both arm links, and terminal plate should all be present.",
    )
    ctx.check(
        "serial_revolute_axes_in_plane",
        root_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.articulation_type == ArticulationType.REVOLUTE
        and root_joint.axis == (0.0, 1.0, 0.0)
        and elbow_joint.axis == (0.0, 1.0, 0.0)
        and object_model.get_part(root_joint.child).name == first_link.name
        and object_model.get_part(elbow_joint.parent).name == first_link.name
        and object_model.get_part(elbow_joint.child).name == terminal_link.name,
        "The transfer arm should be a two-joint serial chain with parallel Y-axis revolute joints.",
    )
    ctx.expect_contact(
        first_link,
        root_bracket,
        name="root_joint_bearing_contact",
    )
    ctx.expect_contact(
        terminal_link,
        first_link,
        name="elbow_knuckle_contact",
    )

    with ctx.pose({root_joint: 0.35, elbow_joint: -0.55}):
        ctx.expect_contact(
            first_link,
            root_bracket,
            name="root_joint_contact_in_reach_pose",
        )
        ctx.expect_contact(
            terminal_link,
            first_link,
            name="elbow_contact_in_reach_pose",
        )

    with ctx.pose({root_joint: -0.30, elbow_joint: 0.60}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
