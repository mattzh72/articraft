from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="attic_hatch_folding_loft_ladder")

    wood = Material("varnished_wood", rgba=(0.55, 0.34, 0.18, 1.0))
    pale_ceiling = Material("painted_ceiling", rgba=(0.86, 0.84, 0.78, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_step = Material("dark_grip_steps", rgba=(0.08, 0.09, 0.09, 1.0))
    steel = Material("galvanized_steel", rgba=(0.48, 0.50, 0.50, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.012, 1.0))

    # Root frame: a ceiling-mounted rectangular hatch frame with a lowered
    # hinge line along one short edge.  The frame part is one connected assembly
    # of overlapping trim boards, joists, brackets, and hinge knuckles.
    frame = model.part("ceiling_frame")
    frame_z = 2.62
    frame.visual(Box((1.28, 0.070, 0.080)), origin=Origin(xyz=(0.56, -0.43, frame_z)), material=wood, name="side_rail_0")
    frame.visual(Box((1.28, 0.070, 0.080)), origin=Origin(xyz=(0.56, 0.43, frame_z)), material=wood, name="side_rail_1")
    frame.visual(Box((0.080, 0.93, 0.080)), origin=Origin(xyz=(-0.045, 0.0, frame_z)), material=wood, name="hinge_edge_rail")
    frame.visual(Box((0.080, 0.93, 0.080)), origin=Origin(xyz=(1.165, 0.0, frame_z)), material=wood, name="far_edge_rail")
    frame.visual(Box((1.42, 0.090, 0.028)), origin=Origin(xyz=(0.56, -0.505, 2.585)), material=pale_ceiling, name="ceiling_trim_0")
    frame.visual(Box((1.42, 0.090, 0.028)), origin=Origin(xyz=(0.56, 0.505, 2.585)), material=pale_ceiling, name="ceiling_trim_1")
    frame.visual(Box((0.110, 1.08, 0.028)), origin=Origin(xyz=(-0.125, 0.0, 2.585)), material=pale_ceiling, name="ceiling_trim_2")
    frame.visual(Box((0.110, 1.08, 0.028)), origin=Origin(xyz=(1.245, 0.0, 2.585)), material=pale_ceiling, name="ceiling_trim_3")
    for y in (-0.31, 0.31):
        suffix = "0" if y < 0 else "1"
        frame.visual(Box((0.075, 0.045, 0.145)), origin=Origin(xyz=(0.0, y, 2.545)), material=steel, name=f"hinge_cheek_{suffix}")
        frame.visual(
            Cylinder(radius=0.024, length=0.135),
            origin=Origin(xyz=(0.0, y, 2.520), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"frame_knuckle_{suffix}",
        )
    frame.visual(
        Cylinder(radius=0.008, length=0.650),
        origin=Origin(xyz=(0.0, 0.0, 2.520), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="frame_pin",
    )

    section_length = 0.88
    section_width = 0.48
    stile_offset = 0.215
    stile_thickness = 0.045
    stile_depth = 0.052

    def make_section(name: str, rung_positions: tuple[float, ...], *, top_barrel_x: float = 0.0, with_feet: bool = False):
        section = model.part(name)
        section.visual(
            Box((section_length - 0.100, stile_thickness, stile_depth)),
            origin=Origin(xyz=(section_length / 2.0, -stile_offset, 0.0)),
            material=aluminum,
            name="stile_0",
        )
        section.visual(
            Box((section_length - 0.100, stile_thickness, stile_depth)),
            origin=Origin(xyz=(section_length / 2.0, stile_offset, 0.0)),
            material=aluminum,
            name="stile_1",
        )

        for idx, x in enumerate(rung_positions):
            section.visual(
                Box((0.060, section_width, 0.040)),
                origin=Origin(xyz=(x, 0.0, -0.003)),
                material=dark_step,
                name=f"rung_{idx}",
            )

        # Crossbars at each end make the stile pair one continuous ladder
        # section and read as the bolted hinge ties at the folding knees.
        section.visual(
            Box((0.070, section_width, 0.044)),
            origin=Origin(xyz=(0.050, 0.0, 0.0)),
            material=aluminum,
            name="top_crossbar",
        )
        section.visual(
            Box((0.070, section_width, 0.044)),
            origin=Origin(xyz=(section_length - 0.075, 0.0, 0.0)),
            material=aluminum,
            name="bottom_crossbar",
        )
        section.visual(
            Cylinder(radius=0.023, length=0.300),
            origin=Origin(xyz=(top_barrel_x, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="top_hinge_barrel",
        )
        section.visual(
            Cylinder(radius=0.024, length=0.120),
            origin=Origin(xyz=(section_length - 0.020, -0.275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="bottom_knuckle_0",
        )
        section.visual(
            Cylinder(radius=0.024, length=0.120),
            origin=Origin(xyz=(section_length - 0.020, 0.275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="bottom_knuckle_1",
        )
        section.visual(
            Cylinder(radius=0.008, length=0.650),
            origin=Origin(xyz=(section_length, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="bottom_hinge_pin",
        )

        if with_feet:
            section.visual(
                Box((0.095, 0.075, 0.045)),
                origin=Origin(xyz=(section_length + 0.020, -stile_offset, -0.040)),
                material=rubber,
                name="foot_0",
            )
            section.visual(
                Box((0.095, 0.075, 0.045)),
                origin=Origin(xyz=(section_length + 0.020, stile_offset, -0.040)),
                material=rubber,
                name="foot_1",
            )
        return section

    upper = make_section("upper_section", (0.22, 0.47, 0.72), top_barrel_x=0.0)
    middle = make_section("middle_section", (0.20, 0.45, 0.70))
    lower = make_section("lower_section", (0.18, 0.42, 0.66), with_feet=True)

    deployed_pitch = math.radians(70.0)
    hinge_z = 2.520

    model.articulation(
        "frame_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, hinge_z), rpy=(0.0, deployed_pitch, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=1.10),
    )
    model.articulation(
        "upper_knee",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=middle,
        origin=Origin(xyz=(section_length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.5, lower=0.0, upper=2.75),
    )
    model.articulation(
        "middle_knee",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=lower,
        origin=Origin(xyz=(section_length, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.5, lower=0.0, upper=2.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("ceiling_frame")
    upper = object_model.get_part("upper_section")
    middle = object_model.get_part("middle_section")
    lower = object_model.get_part("lower_section")
    frame_hinge = object_model.get_articulation("frame_hinge")
    upper_knee = object_model.get_articulation("upper_knee")
    middle_knee = object_model.get_articulation("middle_knee")

    ctx.allow_overlap(
        frame,
        upper,
        elem_a="frame_pin",
        elem_b="top_hinge_barrel",
        reason="The ceiling-frame hinge pin is intentionally represented as a solid pin captured inside the upper section hinge barrel.",
    )
    ctx.allow_overlap(
        upper,
        middle,
        elem_a="bottom_hinge_pin",
        elem_b="top_hinge_barrel",
        reason="The first folding knee uses a captured steel pin through the middle section hinge barrel.",
    )
    ctx.allow_overlap(
        middle,
        lower,
        elem_a="bottom_hinge_pin",
        elem_b="top_hinge_barrel",
        reason="The second folding knee uses a captured steel pin through the lower section hinge barrel.",
    )

    for joint, parent_name, child_name, upper_limit in (
        (frame_hinge, "ceiling_frame", "upper_section", 1.10),
        (upper_knee, "upper_section", "middle_section", 2.75),
        (middle_knee, "middle_section", "lower_section", 2.75),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is a bounded revolute hinge",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.parent == parent_name
            and joint.child == child_name
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and abs(limits.upper - upper_limit) < 1e-6,
            details=f"type={joint.articulation_type}, parent={joint.parent}, child={joint.child}, limits={limits}",
        )

    with ctx.pose({frame_hinge: 0.0, upper_knee: 0.0, middle_knee: 0.0}):
        ctx.expect_within(
            frame,
            upper,
            axes="xz",
            inner_elem="frame_pin",
            outer_elem="top_hinge_barrel",
            margin=0.001,
            name="hinge pin sits inside upper barrel in cross section",
        )
        ctx.expect_overlap(
            frame,
            upper,
            axes="y",
            elem_a="frame_pin",
            elem_b="top_hinge_barrel",
            min_overlap=0.28,
            name="upper barrel remains captured on the frame pin",
        )
        ctx.expect_within(
            upper,
            middle,
            axes="xz",
            inner_elem="bottom_hinge_pin",
            outer_elem="top_hinge_barrel",
            margin=0.001,
            name="first knee pin sits inside middle barrel in cross section",
        )
        ctx.expect_overlap(
            upper,
            middle,
            axes="y",
            elem_a="bottom_hinge_pin",
            elem_b="top_hinge_barrel",
            min_overlap=0.28,
            name="first knee barrel remains captured on its pin",
        )
        ctx.expect_within(
            middle,
            lower,
            axes="xz",
            inner_elem="bottom_hinge_pin",
            outer_elem="top_hinge_barrel",
            margin=0.001,
            name="second knee pin sits inside lower barrel in cross section",
        )
        ctx.expect_overlap(
            middle,
            lower,
            axes="y",
            elem_a="bottom_hinge_pin",
            elem_b="top_hinge_barrel",
            min_overlap=0.28,
            name="second knee barrel remains captured on its pin",
        )

        rest_upper_bottom = ctx.part_element_world_aabb(upper, elem="bottom_crossbar")
        rest_middle_bottom = ctx.part_element_world_aabb(middle, elem="bottom_crossbar")
        rest_lower_foot = ctx.part_element_world_aabb(lower, elem="foot_0")

    with ctx.pose({frame_hinge: 0.90, upper_knee: 0.0, middle_knee: 0.0}):
        folded_upper_bottom = ctx.part_element_world_aabb(upper, elem="bottom_crossbar")

    with ctx.pose({frame_hinge: 0.0, upper_knee: 1.20, middle_knee: 0.0}):
        folded_middle_bottom = ctx.part_element_world_aabb(middle, elem="bottom_crossbar")

    with ctx.pose({frame_hinge: 0.0, upper_knee: 0.0, middle_knee: 1.20}):
        folded_lower_foot = ctx.part_element_world_aabb(lower, elem="foot_0")

    def center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    ctx.check(
        "upper section folds upward toward the ceiling",
        center_z(rest_upper_bottom) is not None
        and center_z(folded_upper_bottom) is not None
        and center_z(folded_upper_bottom) > center_z(rest_upper_bottom) + 0.35,
        details=f"rest={rest_upper_bottom}, folded={folded_upper_bottom}",
    )
    ctx.check(
        "first knee folds the middle section upward",
        center_z(rest_middle_bottom) is not None
        and center_z(folded_middle_bottom) is not None
        and center_z(folded_middle_bottom) > center_z(rest_middle_bottom) + 0.15,
        details=f"rest={rest_middle_bottom}, folded={folded_middle_bottom}",
    )
    ctx.check(
        "second knee folds the lower section upward",
        center_z(rest_lower_foot) is not None
        and center_z(folded_lower_foot) is not None
        and center_z(folded_lower_foot) > center_z(rest_lower_foot) + 0.35,
        details=f"rest={rest_lower_foot}, folded={folded_lower_foot}",
    )

    return ctx.report()


object_model = build_object_model()
