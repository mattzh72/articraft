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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_inspection_boom")

    model.material("powder_coat_yellow", rgba=(0.95, 0.72, 0.12, 1.0))
    model.material("dark_steel", rgba=(0.06, 0.07, 0.075, 1.0))
    model.material("rubber_black", rgba=(0.015, 0.015, 0.014, 1.0))
    model.material("brushed_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    model.material("glass_blue", rgba=(0.15, 0.45, 0.85, 0.72))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.30, 0.04)),
        origin=Origin(xyz=(0.10, 0.0, 0.02)),
        material="dark_steel",
        name="floor_plate",
    )
    base.visual(
        Box((0.13, 0.16, 0.47)),
        origin=Origin(xyz=(-0.055, 0.0, 0.255)),
        material="powder_coat_yellow",
        name="pedestal",
    )
    base.visual(
        Box((0.14, 0.025, 0.20)),
        origin=Origin(xyz=(0.0, 0.075, 0.55)),
        material="powder_coat_yellow",
        name="root_yoke_cheek_0",
    )
    base.visual(
        Box((0.14, 0.025, 0.20)),
        origin=Origin(xyz=(0.0, -0.075, 0.55)),
        material="powder_coat_yellow",
        name="root_yoke_cheek_1",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.0, 0.091, 0.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="root_boss_0",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.0, -0.091, 0.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="root_boss_1",
    )
    base.visual(
        Box((0.055, 0.070, 0.46)),
        origin=Origin(xyz=(0.22, 0.0, 0.25)),
        material="dark_steel",
        name="lower_stop_post",
    )
    base.visual(
        Box((0.080, 0.085, 0.020)),
        origin=Origin(xyz=(0.22, 0.0, 0.49)),
        material="rubber_black",
        name="lower_stop_pad",
    )
    base.visual(
        Box((0.055, 0.15, 0.035)),
        origin=Origin(xyz=(-0.018, 0.0, 0.660)),
        material="rubber_black",
        name="upper_stop_pad",
    )

    outer_boom = model.part("outer_boom")
    outer_boom.visual(
        Cylinder(radius=0.044, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="root_knuckle",
    )
    outer_boom.visual(
        Box((0.82, 0.100, 0.012)),
        origin=Origin(xyz=(0.45, 0.0, 0.044)),
        material="powder_coat_yellow",
        name="top_wall",
    )
    outer_boom.visual(
        Box((0.82, 0.100, 0.012)),
        origin=Origin(xyz=(0.45, 0.0, -0.044)),
        material="powder_coat_yellow",
        name="bottom_wall",
    )
    outer_boom.visual(
        Box((0.82, 0.012, 0.100)),
        origin=Origin(xyz=(0.45, 0.044, 0.0)),
        material="powder_coat_yellow",
        name="side_wall_0",
    )
    outer_boom.visual(
        Box((0.82, 0.012, 0.100)),
        origin=Origin(xyz=(0.45, -0.044, 0.0)),
        material="powder_coat_yellow",
        name="side_wall_1",
    )
    outer_boom.visual(
        Box((0.060, 0.116, 0.116)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material="dark_steel",
        name="root_collar",
    )
    outer_boom.visual(
        Box((0.035, 0.116, 0.012)),
        origin=Origin(xyz=(0.86, 0.0, 0.052)),
        material="dark_steel",
        name="mouth_stop_top",
    )
    outer_boom.visual(
        Box((0.035, 0.116, 0.012)),
        origin=Origin(xyz=(0.86, 0.0, -0.052)),
        material="dark_steel",
        name="mouth_stop_bottom",
    )
    outer_boom.visual(
        Box((0.035, 0.012, 0.116)),
        origin=Origin(xyz=(0.86, 0.052, 0.0)),
        material="dark_steel",
        name="mouth_stop_side_0",
    )
    outer_boom.visual(
        Box((0.035, 0.012, 0.116)),
        origin=Origin(xyz=(0.86, -0.052, 0.0)),
        material="dark_steel",
        name="mouth_stop_side_1",
    )
    outer_boom.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.72, 0.0, 0.060)),
        material="brushed_steel",
        name="slide_stop_screw",
    )

    extension = model.part("extension")
    extension.visual(
        Box((0.620, 0.052, 0.052)),
        origin=Origin(xyz=(-0.240, 0.0, 0.0)),
        material="brushed_steel",
        name="inner_beam",
    )
    extension.visual(
        Box((0.032, 0.076, 0.076)),
        origin=Origin(xyz=(-0.535, 0.0, 0.0)),
        material="dark_steel",
        name="hidden_stop_collar",
    )
    extension.visual(
        Box((0.160, 0.012, 0.030)),
        origin=Origin(xyz=(-0.250, 0.032, 0.0)),
        material="rubber_black",
        name="slide_wear_pad_0",
    )
    extension.visual(
        Box((0.160, 0.012, 0.030)),
        origin=Origin(xyz=(-0.250, -0.032, 0.0)),
        material="rubber_black",
        name="slide_wear_pad_1",
    )
    extension.visual(
        Box((0.160, 0.030, 0.012)),
        origin=Origin(xyz=(-0.250, 0.0, 0.032)),
        material="rubber_black",
        name="slide_wear_pad_2",
    )
    extension.visual(
        Box((0.160, 0.030, 0.012)),
        origin=Origin(xyz=(-0.250, 0.0, -0.032)),
        material="rubber_black",
        name="slide_wear_pad_3",
    )
    extension.visual(
        Box((0.030, 0.086, 0.086)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material="rubber_black",
        name="outer_stop_bumper",
    )
    extension.visual(
        Box((0.035, 0.082, 0.060)),
        origin=Origin(xyz=(0.0875, 0.0, 0.0)),
        material="dark_steel",
        name="end_fork_bridge",
    )
    extension.visual(
        Box((0.080, 0.014, 0.076)),
        origin=Origin(xyz=(0.14, 0.047, 0.0)),
        material="dark_steel",
        name="end_fork_cheek_0",
    )
    extension.visual(
        Box((0.080, 0.014, 0.076)),
        origin=Origin(xyz=(0.14, -0.047, 0.0)),
        material="dark_steel",
        name="end_fork_cheek_1",
    )
    extension.visual(
        Box((0.030, 0.030, 0.018)),
        origin=Origin(xyz=(0.14, 0.047, 0.045)),
        material="rubber_black",
        name="rotary_stop_0",
    )
    extension.visual(
        Box((0.030, 0.030, 0.018)),
        origin=Origin(xyz=(0.14, -0.047, 0.045)),
        material="rubber_black",
        name="rotary_stop_1",
    )

    end_head = model.part("end_head")
    end_head.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="rotary_collar",
    )
    end_head.visual(
        Box((0.025, 0.018, 0.022)),
        origin=Origin(xyz=(-0.006, 0.0, 0.039)),
        material="dark_steel",
        name="rotary_stop_tab",
    )
    end_head.visual(
        Box((0.090, 0.090, 0.060)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material="dark_steel",
        name="inspection_head_body",
    )
    end_head.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.122, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="glass_blue",
        name="lens_window",
    )
    end_head.visual(
        Box((0.038, 0.110, 0.012)),
        origin=Origin(xyz=(0.074, 0.0, -0.035)),
        material="rubber_black",
        name="protective_skid",
    )

    model.articulation(
        "root_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_boom,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.9, lower=0.0, upper=1.75),
    )
    model.articulation(
        "boom_slide",
        ArticulationType.PRISMATIC,
        parent=outer_boom,
        child=extension,
        origin=Origin(xyz=(0.86, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.45),
    )
    model.articulation(
        "end_rotary",
        ArticulationType.REVOLUTE,
        parent=extension,
        child=end_head,
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-2.6, upper=2.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer_boom = object_model.get_part("outer_boom")
    extension = object_model.get_part("extension")
    end_head = object_model.get_part("end_head")

    root_hinge = object_model.get_articulation("root_hinge")
    boom_slide = object_model.get_articulation("boom_slide")
    end_rotary = object_model.get_articulation("end_rotary")

    def is_type(joint, articulation_type) -> bool:
        actual = getattr(joint, "articulation_type", None)
        return actual == articulation_type or str(actual).lower().endswith(
            articulation_type.name.lower()
        )

    ctx.check(
        "three user mechanisms",
        len(object_model.articulations) == 3,
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check("root joint is revolute", is_type(root_hinge, ArticulationType.REVOLUTE))
    ctx.check("extension stage is prismatic", is_type(boom_slide, ArticulationType.PRISMATIC))
    ctx.check("end joint is stopped rotary", is_type(end_rotary, ArticulationType.REVOLUTE))

    ctx.check(
        "root hinge has stop limits",
        root_hinge.motion_limits is not None
        and root_hinge.motion_limits.lower == 0.0
        and 1.7 <= root_hinge.motion_limits.upper <= 1.8,
    )
    ctx.check(
        "slide has retained travel stops",
        boom_slide.motion_limits is not None
        and boom_slide.motion_limits.lower == 0.0
        and 0.40 <= boom_slide.motion_limits.upper <= 0.50,
    )
    ctx.check(
        "end rotary has finite stops",
        end_rotary.motion_limits is not None
        and end_rotary.motion_limits.lower < -2.0
        and end_rotary.motion_limits.upper > 2.0,
    )

    with ctx.pose({root_hinge: 0.0, boom_slide: 0.0, end_rotary: 0.0}):
        ctx.expect_contact(
            outer_boom,
            base,
            elem_a="bottom_wall",
            elem_b="lower_stop_pad",
            contact_tol=0.001,
            name="folded boom rests on lower stop",
        )
        ctx.expect_contact(
            extension,
            outer_boom,
            elem_a="slide_wear_pad_0",
            elem_b="side_wall_0",
            contact_tol=0.001,
            name="extension rides on sleeve wear pad",
        )
        ctx.expect_within(
            extension,
            outer_boom,
            axes="yz",
            inner_elem="inner_beam",
            margin=0.001,
            name="inner beam fits within the sleeve envelope",
        )
        ctx.expect_overlap(
            extension,
            outer_boom,
            axes="x",
            elem_a="inner_beam",
            elem_b="top_wall",
            min_overlap=0.50,
            name="retracted extension remains deeply inserted",
        )

    rest_extension_pos = ctx.part_world_position(extension)
    with ctx.pose({boom_slide: boom_slide.motion_limits.upper}):
        extended_extension_pos = ctx.part_world_position(extension)
        ctx.expect_overlap(
            extension,
            outer_boom,
            axes="x",
            elem_a="inner_beam",
            elem_b="top_wall",
            min_overlap=0.09,
            name="extended stage retains insertion",
        )
        ctx.expect_contact(
            extension,
            outer_boom,
            elem_a="hidden_stop_collar",
            elem_b="top_wall",
            contact_tol=0.001,
            name="pull-out stop collar stays captured in sleeve",
        )
    ctx.check(
        "extension moves outward",
        rest_extension_pos is not None
        and extended_extension_pos is not None
        and extended_extension_pos[0] > rest_extension_pos[0] + 0.40,
        details=f"rest={rest_extension_pos}, extended={extended_extension_pos}",
    )

    rest_fold_pos = ctx.part_world_position(extension)
    with ctx.pose({root_hinge: 1.0}):
        raised_fold_pos = ctx.part_world_position(extension)
    ctx.check(
        "root hinge folds boom upward",
        rest_fold_pos is not None
        and raised_fold_pos is not None
        and raised_fold_pos[2] > rest_fold_pos[2] + 0.50,
        details=f"rest={rest_fold_pos}, raised={raised_fold_pos}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_tab_center = aabb_center(ctx.part_element_world_aabb(end_head, elem="rotary_stop_tab"))
    with ctx.pose({end_rotary: math.pi / 2.0}):
        rotated_tab_center = aabb_center(
            ctx.part_element_world_aabb(end_head, elem="rotary_stop_tab")
        )
    ctx.check(
        "end rotary turns inspection head about boom axis",
        rest_tab_center is not None
        and rotated_tab_center is not None
        and rotated_tab_center[1] < rest_tab_center[1] - 0.025,
        details=f"rest={rest_tab_center}, rotated={rotated_tab_center}",
    )

    return ctx.report()


object_model = build_object_model()
