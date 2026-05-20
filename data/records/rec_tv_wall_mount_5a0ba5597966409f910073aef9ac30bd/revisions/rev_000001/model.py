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
    model = ArticulatedObject(name="screenless_tv_wall_mount")

    powder = model.material("black_powder_coat", rgba=(0.015, 0.016, 0.017, 1.0))
    dark = model.material("dark_gunmetal", rgba=(0.10, 0.105, 0.11, 1.0))
    edge = model.material("worn_edge_highlight", rgba=(0.22, 0.23, 0.235, 1.0))
    rubber = model.material("black_plastic_bushings", rgba=(0.0, 0.0, 0.0, 1.0))

    cyl_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    cyl_y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.035, 0.32, 0.58)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
        material=dark,
        name="plate_panel",
    )
    wall_plate.visual(
        Box((0.010, 0.34, 0.60)),
        origin=Origin(xyz=(-0.047, 0.0, 0.0)),
        material=powder,
        name="wall_shadow_backer",
    )
    for y in (-0.105, 0.105):
        for z in (-0.205, 0.205):
            wall_plate.visual(
                Cylinder(radius=0.017, length=0.008),
                origin=Origin(xyz=(-0.0035, y, z), rpy=cyl_x.rpy),
                material=rubber,
                name=f"lag_screw_{'pos' if y > 0 else 'neg'}_{'top' if z > 0 else 'bottom'}",
            )
    wall_plate.visual(
        Cylinder(radius=0.010, length=0.300),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=edge,
        name="wall_hinge_pin",
    )
    for z, name in ((0.105, "upper_wall_knuckle"), (-0.105, "lower_wall_knuckle")):
        wall_plate.visual(
            Box((0.070, 0.075, 0.052)),
            origin=Origin(xyz=(0.022, 0.0, z)),
            material=powder,
            name=f"{name}_web",
        )
        wall_plate.visual(
            Cylinder(radius=0.026, length=0.090),
            origin=Origin(xyz=(0.055, 0.0, z)),
            material=powder,
            name=name,
        )
        wall_plate.visual(
            Cylinder(radius=0.012, length=0.094),
            origin=Origin(xyz=(0.055, 0.0, z)),
            material=rubber,
            name=f"{name}_bore_shadow",
        )

    outer_arm = model.part("outer_arm")
    outer_len = 0.380
    outer_arm.visual(
        Cylinder(radius=0.022, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=powder,
        name="wall_pivot_barrel",
    )
    outer_arm.visual(
        Box((0.070, 0.112, 0.040)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=powder,
        name="rear_cross_block",
    )
    outer_arm.visual(
        Box((0.340, 0.022, 0.036)),
        origin=Origin(xyz=(0.180, 0.045, 0.0)),
        material=powder,
        name="outer_side_bar_0",
    )
    outer_arm.visual(
        Box((0.340, 0.022, 0.036)),
        origin=Origin(xyz=(0.180, -0.045, 0.0)),
        material=powder,
        name="outer_side_bar_1",
    )
    outer_arm.visual(
        Box((0.060, 0.112, 0.160)),
        origin=Origin(xyz=(outer_len - 0.060, 0.0, 0.0)),
        material=powder,
        name="elbow_cross_block",
    )
    outer_arm.visual(
        Cylinder(radius=0.010, length=0.230),
        origin=Origin(xyz=(outer_len, 0.0, 0.0)),
        material=edge,
        name="elbow_pin",
    )
    for z, name in ((0.070, "upper_elbow_knuckle"), (-0.070, "lower_elbow_knuckle")):
        outer_arm.visual(
            Cylinder(radius=0.024, length=0.070),
            origin=Origin(xyz=(outer_len, 0.0, z)),
            material=powder,
            name=name,
        )
        outer_arm.visual(
            Box((0.050, 0.055, 0.018)),
            origin=Origin(xyz=(outer_len - 0.025, 0.0, z)),
            material=powder,
            name=f"{name}_bridge",
        )

    inner_arm = model.part("inner_arm")
    inner_len = 0.320
    inner_arm.visual(
        Cylinder(radius=0.020, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=powder,
        name="elbow_center_barrel",
    )
    inner_arm.visual(
        Box((0.070, 0.095, 0.036)),
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        material=powder,
        name="elbow_bridge_block",
    )
    inner_arm.visual(
        Box((0.292, 0.020, 0.032)),
        origin=Origin(xyz=(0.155, 0.035, 0.0)),
        material=powder,
        name="inner_side_bar_0",
    )
    inner_arm.visual(
        Box((0.292, 0.020, 0.032)),
        origin=Origin(xyz=(0.155, -0.035, 0.0)),
        material=powder,
        name="inner_side_bar_1",
    )
    for y, name in ((0.052, "head_clevis_0"), (-0.052, "head_clevis_1")):
        inner_arm.visual(
            Box((0.074, 0.020, 0.150)),
            origin=Origin(xyz=(inner_len, y, 0.0)),
            material=powder,
            name=name,
        )
        inner_arm.visual(
            Cylinder(radius=0.018, length=0.022),
            origin=Origin(xyz=(inner_len, y, 0.0), rpy=cyl_y.rpy),
            material=edge,
            name=f"{name}_boss",
        )
    inner_arm.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(inner_len, 0.0, 0.0)),
        material=edge,
        name="swivel_pin",
    )
    for z, name in ((0.075, "upper_swivel_saddle"), (-0.075, "lower_swivel_saddle")):
        inner_arm.visual(
            Box((0.046, 0.116, 0.014)),
            origin=Origin(xyz=(inner_len, 0.0, z)),
            material=powder,
            name=name,
        )

    swivel_support = model.part("swivel_support")
    swivel_support.visual(
        Cylinder(radius=0.021, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=powder,
        name="swivel_post",
    )
    swivel_support.visual(
        Box((0.052, 0.035, 0.036)),
        origin=Origin(xyz=(0.047, 0.0, 0.0)),
        material=powder,
        name="post_neck",
    )
    swivel_support.visual(
        Box((0.032, 0.132, 0.036)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material=powder,
        name="rear_yoke_bridge",
    )
    swivel_support.visual(
        Cylinder(radius=0.010, length=0.145),
        origin=Origin(xyz=(0.115, 0.0, 0.0), rpy=cyl_y.rpy),
        material=edge,
        name="tilt_pin",
    )
    for y, name in ((0.055, "tilt_cheek_0"), (-0.055, "tilt_cheek_1")):
        swivel_support.visual(
            Box((0.055, 0.018, 0.036)),
            origin=Origin(xyz=(0.088, y, 0.0)),
            material=powder,
            name=f"{name}_side_tie",
        )
        swivel_support.visual(
            Box((0.045, 0.018, 0.116)),
            origin=Origin(xyz=(0.115, y, 0.0)),
            material=powder,
            name=name,
        )
        swivel_support.visual(
            Cylinder(radius=0.026, length=0.020),
            origin=Origin(xyz=(0.115, y, 0.0), rpy=cyl_y.rpy),
            material=edge,
            name=f"{name}_tilt_boss",
        )

    head_frame = model.part("head_frame")
    head_frame.visual(
        Cylinder(radius=0.020, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy),
        material=powder,
        name="tilt_barrel",
    )
    head_frame.visual(
        Box((0.080, 0.045, 0.040)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=powder,
        name="tilt_to_frame_spine",
    )
    head_frame.visual(
        Box((0.012, 0.190, 0.150)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=dark,
        name="vesa_back_plate",
    )
    head_frame.visual(
        Box((0.026, 0.286, 0.026)),
        origin=Origin(xyz=(0.105, 0.0, 0.098)),
        material=powder,
        name="top_mount_rail",
    )
    head_frame.visual(
        Box((0.026, 0.286, 0.026)),
        origin=Origin(xyz=(0.105, 0.0, -0.098)),
        material=powder,
        name="bottom_mount_rail",
    )
    head_frame.visual(
        Box((0.026, 0.026, 0.222)),
        origin=Origin(xyz=(0.105, 0.130, 0.0)),
        material=powder,
        name="side_mount_rail_0",
    )
    head_frame.visual(
        Box((0.026, 0.026, 0.222)),
        origin=Origin(xyz=(0.105, -0.130, 0.0)),
        material=powder,
        name="side_mount_rail_1",
    )
    head_frame.visual(
        Box((0.026, 0.240, 0.036)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=powder,
        name="center_cross_rail",
    )
    head_frame.visual(
        Box((0.026, 0.036, 0.196)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=powder,
        name="center_upright_rail",
    )
    for y in (-0.075, 0.075):
        for z in (-0.055, 0.055):
            head_frame.visual(
                Cylinder(radius=0.009, length=0.006),
                origin=Origin(xyz=(0.113, y, z), rpy=cyl_x.rpy),
                material=rubber,
                name=f"vesa_hole_{'pos' if y > 0 else 'neg'}_{'top' if z > 0 else 'bottom'}",
            )

    model.articulation(
        "wall_fold",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=outer_arm,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=outer_arm,
        child=inner_arm,
        origin=Origin(xyz=(outer_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=swivel_support,
        origin=Origin(xyz=(inner_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_support,
        child=head_frame,
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.35, upper=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_plate")
    outer = object_model.get_part("outer_arm")
    inner = object_model.get_part("inner_arm")
    swivel = object_model.get_part("swivel_support")
    head = object_model.get_part("head_frame")
    wall_fold = object_model.get_articulation("wall_fold")
    elbow_fold = object_model.get_articulation("elbow_fold")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "four revolute mechanisms",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details="The mount should have wall fold, elbow fold, head swivel, and head tilt revolutes.",
    )
    ctx.allow_overlap(
        wall,
        outer,
        elem_a="wall_hinge_pin",
        elem_b="wall_pivot_barrel",
        reason="The wall pivot uses a captured hinge pin passing through the arm barrel.",
    )
    ctx.allow_overlap(
        outer,
        inner,
        elem_a="elbow_pin",
        elem_b="elbow_center_barrel",
        reason="The elbow folding joint uses a captured pin through the inner arm barrel.",
    )
    ctx.allow_overlap(
        inner,
        swivel,
        elem_a="swivel_pin",
        elem_b="swivel_post",
        reason="The head swivel post is retained by a vertical pin through the clevis.",
    )
    ctx.allow_overlap(
        swivel,
        head,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        reason="The tilt barrel rotates around a horizontal pin carried by the head yoke.",
    )
    ctx.expect_overlap(
        wall,
        outer,
        axes="z",
        min_overlap=0.100,
        elem_a="wall_hinge_pin",
        elem_b="wall_pivot_barrel",
        name="wall hinge pin spans arm barrel",
    )
    ctx.expect_contact(
        wall,
        outer,
        elem_a="upper_wall_knuckle",
        elem_b="wall_pivot_barrel",
        contact_tol=0.002,
        name="wall knuckle seats against arm barrel",
    )
    ctx.expect_overlap(
        outer,
        inner,
        axes="z",
        min_overlap=0.065,
        elem_a="elbow_pin",
        elem_b="elbow_center_barrel",
        name="elbow pin spans inner barrel",
    )
    ctx.expect_contact(
        outer,
        inner,
        elem_a="upper_elbow_knuckle",
        elem_b="elbow_center_barrel",
        contact_tol=0.002,
        name="elbow knuckle seats against inner barrel",
    )
    ctx.expect_overlap(
        inner,
        swivel,
        axes="z",
        min_overlap=0.120,
        elem_a="swivel_pin",
        elem_b="swivel_post",
        name="swivel pin spans vertical post",
    )
    ctx.expect_overlap(
        inner,
        swivel,
        axes="z",
        min_overlap=0.110,
        elem_a="head_clevis_0",
        elem_b="swivel_post",
        name="swivel post captured by head clevis height",
    )
    ctx.expect_overlap(
        swivel,
        head,
        axes="y",
        min_overlap=0.085,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        name="tilt pin spans head barrel",
    )
    ctx.expect_contact(
        swivel,
        head,
        elem_a="tilt_cheek_0",
        elem_b="tilt_barrel",
        contact_tol=0.002,
        name="tilt barrel sits between yoke cheeks",
    )

    rest_head = ctx.part_world_position(head)
    with ctx.pose({wall_fold: 0.95, elbow_fold: -0.20, head_swivel: 0.65, head_tilt: 0.25}):
        posed_head = ctx.part_world_position(head)
        ctx.expect_origin_distance(
            outer,
            inner,
            axes="xy",
            min_dist=0.25,
            max_dist=0.45,
            name="folded arm keeps realistic elbow reach",
        )
    ctx.check(
        "fold and swivel move head laterally",
        rest_head is not None
        and posed_head is not None
        and abs(posed_head[1] - rest_head[1]) > 0.12,
        details=f"rest_head={rest_head}, posed_head={posed_head}",
    )

    return ctx.report()


object_model = build_object_model()
