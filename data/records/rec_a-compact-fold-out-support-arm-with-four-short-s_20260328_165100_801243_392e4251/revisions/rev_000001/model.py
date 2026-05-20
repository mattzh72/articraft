from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import hypot

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_support_arm")

    base_thickness = 0.006
    base_width = 0.045
    base_height = 0.090

    fork_length = 0.008
    tongue_length = 0.008
    outer_width = 0.020
    fork_gap = 0.011
    lug_width = (outer_width - fork_gap) / 2.0
    tongue_width = fork_gap
    bar_width = 0.013
    bar_height = 0.014
    base_hinge_offset = 0.014

    link_lengths = (0.068, 0.064, 0.060, 0.056)
    end_pad_length = 0.010

    bracket_ear_length = 0.008
    bracket_ear_height = 0.028
    bracket_plate_length = 0.038
    bracket_plate_width = 0.050
    bracket_plate_thickness = 0.004
    bracket_lip_thickness = 0.004
    bracket_lip_height = 0.014
    gusset_length = 0.018
    gusset_height = 0.014
    gusset_width = 0.020

    steel = model.material("steel", color=(0.28, 0.30, 0.33))
    zinc = model.material("zinc", color=(0.68, 0.70, 0.72))

    def add_fork(part, joint_x: float) -> None:
        x_center = joint_x + (fork_length / 2.0)
        y_offset = (fork_gap / 2.0) + (lug_width / 2.0)
        part.visual(
            Box((fork_length, lug_width, bar_height)),
            origin=Origin(xyz=(x_center, y_offset, 0.0)),
            material=steel,
            name="fork_upper",
        )
        part.visual(
            Box((fork_length, lug_width, bar_height)),
            origin=Origin(xyz=(x_center, -y_offset, 0.0)),
            material=steel,
            name="fork_lower",
        )

    def add_link(part, length: float, *, has_distal_fork: bool) -> None:
        part.visual(
            Box((tongue_length, tongue_width, bar_height)),
            origin=Origin(xyz=(tongue_length / 2.0, 0.0, 0.0)),
            material=steel,
            name="tongue",
        )

        core_length = length - tongue_length
        part.visual(
            Box((core_length, bar_width, bar_height)),
            origin=Origin(xyz=(tongue_length + (core_length / 2.0), 0.0, 0.0)),
            material=steel,
            name="core",
        )

        if has_distal_fork:
            add_fork(part, length)
        else:
            part.visual(
                Box((end_pad_length, outer_width, bar_height)),
                origin=Origin(xyz=(length + (end_pad_length / 2.0), 0.0, 0.0)),
                material=steel,
                name="end_pad",
            )

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((base_thickness, base_width, base_height)),
        origin=Origin(xyz=(-base_thickness / 2.0, 0.0, 0.0)),
        material=steel,
        name="plate",
    )
    base_plate.visual(
        Box((base_thickness * 0.75, base_width * 0.7, bar_height * 1.3)),
        origin=Origin(xyz=(-base_thickness * 0.55, 0.0, 0.0)),
        material=steel,
        name="hinge_reinforcement",
    )
    y_offset = (fork_gap / 2.0) + (lug_width / 2.0)
    base_plate.visual(
        Box((base_hinge_offset, lug_width, bar_height)),
        origin=Origin(xyz=(base_hinge_offset / 2.0, y_offset, 0.0)),
        material=steel,
        name="hinge_bridge_upper",
    )
    base_plate.visual(
        Box((base_hinge_offset, lug_width, bar_height)),
        origin=Origin(xyz=(base_hinge_offset / 2.0, -y_offset, 0.0)),
        material=steel,
        name="hinge_bridge_lower",
    )
    add_fork(base_plate, base_hinge_offset)

    link1 = model.part("link1")
    add_link(link1, link_lengths[0], has_distal_fork=True)

    link2 = model.part("link2")
    add_link(link2, link_lengths[1], has_distal_fork=True)

    link3 = model.part("link3")
    add_link(link3, link_lengths[2], has_distal_fork=True)

    link4 = model.part("link4")
    add_link(link4, link_lengths[3], has_distal_fork=False)

    bracket_shape = (
        cq.Workplane("XY")
        .box(0.004, 0.014, 0.002)
        .translate((0.002, 0.0, 0.008))
        .union(
            cq.Workplane("XY")
            .box(0.003, 0.012, 0.016)
            .translate((0.0055, 0.0, 0.016))
        )
        .union(
            cq.Workplane("XY")
            .box(0.028, bracket_plate_width * 0.90, 0.004)
            .edges("|Z")
            .fillet(0.0015)
            .translate((0.022, 0.0, 0.025))
        )
        .union(
            cq.Workplane("XY")
            .box(0.004, bracket_plate_width * 0.90, 0.014)
            .translate((0.034, 0.0, 0.018))
        )
        .union(
            cq.Workplane("XZ")
            .polyline(((0.007, 0.009), (0.007, 0.023), (0.020, 0.023)))
            .close()
            .extrude(0.006, both=True)
        )
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.006, 0.0), (0.006, 0.0)])
        .hole(0.0045)
    )

    platform_bracket = model.part("platform_bracket")
    platform_bracket.visual(
        mesh_from_cadquery(bracket_shape, "platform_bracket"),
        material=zinc,
        name="bracket_shell",
    )

    revolute_limits = MotionLimits(
        effort=15.0,
        velocity=2.0,
        lower=-0.75,
        upper=0.75,
    )

    model.articulation(
        "base_to_link1",
        ArticulationType.REVOLUTE,
        parent=base_plate,
        child=link1,
        origin=Origin(xyz=(base_hinge_offset, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(link_lengths[0], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(link_lengths[1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link3_to_link4",
        ArticulationType.REVOLUTE,
        parent=link3,
        child=link4,
        origin=Origin(xyz=(link_lengths[2], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link4_to_platform_bracket",
        ArticulationType.FIXED,
        parent=link4,
        child=platform_bracket,
        origin=Origin(xyz=(link_lengths[3] + end_pad_length, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")
    link4 = object_model.get_part("link4")
    platform_bracket = object_model.get_part("platform_bracket")

    link4_end_pad = link4.get_visual("end_pad")

    base_to_link1 = object_model.get_articulation("base_to_link1")
    link1_to_link2 = object_model.get_articulation("link1_to_link2")
    link2_to_link3 = object_model.get_articulation("link2_to_link3")
    link3_to_link4 = object_model.get_articulation("link3_to_link4")

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

    for part in (
        base_plate,
        link1,
        link2,
        link3,
        link4,
        platform_bracket,
    ):
        ctx.check(f"part present: {part.name}", part is not None)

    for joint in (
        base_to_link1,
        link1_to_link2,
        link2_to_link3,
        link3_to_link4,
    ):
        ctx.check(
            f"{joint.name} axis is parallel",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis was {joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} has fold range",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= -0.7
            and limits.upper >= 0.7,
            details=f"limits were {limits}",
        )

    ctx.expect_contact(base_plate, link1, name="base plate contacts link1")
    ctx.expect_contact(link1, link2, name="link1 contacts link2")
    ctx.expect_contact(link2, link3, name="link2 contacts link3")
    ctx.expect_contact(link3, link4, name="link3 contacts link4")
    ctx.expect_contact(link4, platform_bracket, name="link4 contacts bracket")

    ctx.expect_overlap(
        platform_bracket,
        link4,
        axes="y",
        min_overlap=0.012,
        name="platform bracket shares the link end mounting width",
    )
    ctx.expect_origin_gap(
        platform_bracket,
        link4,
        axis="x",
        min_gap=0.050,
        max_gap=0.070,
        name="platform bracket is mounted at the far end of link4",
    )

    straight_pose = {
        base_to_link1: 0.0,
        link1_to_link2: 0.0,
        link2_to_link3: 0.0,
        link3_to_link4: 0.0,
    }
    tucked_pose = {
        base_to_link1: 0.70,
        link1_to_link2: -0.70,
        link2_to_link3: 0.10,
        link3_to_link4: -0.70,
    }

    with ctx.pose(straight_pose):
        straight_pos = ctx.part_world_position(platform_bracket)
        straight_reach = (
            hypot(straight_pos[0], straight_pos[2]) if straight_pos is not None else None
        )

    with ctx.pose(tucked_pose):
        tucked_pos = ctx.part_world_position(platform_bracket)
        tucked_reach = (
            hypot(tucked_pos[0], tucked_pos[2]) if tucked_pos is not None else None
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in tucked pose")

    ctx.check(
        "arm unfolds into longer reach",
        straight_reach is not None
        and tucked_reach is not None
        and straight_reach > 0.25
        and tucked_reach < 0.25
        and tucked_reach < straight_reach - 0.02,
        details=(
            f"straight reach={straight_reach}, tucked reach={tucked_reach}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
