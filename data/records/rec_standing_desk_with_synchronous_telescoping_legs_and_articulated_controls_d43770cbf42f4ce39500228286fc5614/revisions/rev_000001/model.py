from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_leg_conference_standing_desk")

    wood = model.material("warm_oak_laminate", rgba=(0.72, 0.50, 0.30, 1.0))
    edge = model.material("dark_edge_band", rgba=(0.10, 0.085, 0.07, 1.0))
    metal = model.material("black_powder_coated_steel", rgba=(0.025, 0.027, 0.030, 1.0))
    inner_metal = model.material("brushed_inner_steel", rgba=(0.52, 0.54, 0.56, 1.0))
    plastic = model.material("matte_black_plastic", rgba=(0.015, 0.015, 0.017, 1.0))
    white = model.material("white_indicator_paint", rgba=(0.92, 0.92, 0.86, 1.0))

    frame = model.part("desk_frame")

    # Broad conference table top with a darker edge band.
    frame.visual(
        Box((1.80, 0.90, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 1.135)),
        material=wood,
        name="top_panel",
    )
    frame.visual(
        Box((1.84, 0.025, 0.075)),
        origin=Origin(xyz=(0.0, -0.462, 1.132)),
        material=edge,
        name="front_edge_band",
    )
    frame.visual(
        Box((1.84, 0.025, 0.075)),
        origin=Origin(xyz=(0.0, 0.462, 1.132)),
        material=edge,
        name="rear_edge_band",
    )
    frame.visual(
        Box((0.025, 0.90, 0.075)),
        origin=Origin(xyz=(-0.912, 0.0, 1.132)),
        material=edge,
        name="end_edge_band_0",
    )
    frame.visual(
        Box((0.025, 0.90, 0.075)),
        origin=Origin(xyz=(0.912, 0.0, 1.132)),
        material=edge,
        name="end_edge_band_1",
    )

    # Heavy welded underframe below the slab: perimeter rails and a center tie.
    frame.visual(
        Box((1.52, 0.080, 0.090)),
        origin=Origin(xyz=(0.0, -0.330, 1.065)),
        material=metal,
        name="front_rail",
    )
    frame.visual(
        Box((1.52, 0.080, 0.090)),
        origin=Origin(xyz=(0.0, 0.330, 1.065)),
        material=metal,
        name="rear_rail",
    )
    frame.visual(
        Box((0.080, 0.620, 0.090)),
        origin=Origin(xyz=(-0.700, 0.0, 1.065)),
        material=metal,
        name="end_rail_0",
    )
    frame.visual(
        Box((0.080, 0.620, 0.090)),
        origin=Origin(xyz=(0.700, 0.0, 1.065)),
        material=metal,
        name="end_rail_1",
    )
    frame.visual(
        Box((0.090, 0.700, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 1.030)),
        material=metal,
        name="center_cross_tie",
    )

    # Side control pod clipped under the front edge of the desktop.
    frame.visual(
        Box((0.220, 0.105, 0.075)),
        origin=Origin(xyz=(0.550, -0.505, 1.055)),
        material=plastic,
        name="control_pod",
    )
    frame.visual(
        Box((0.245, 0.095, 0.018)),
        origin=Origin(xyz=(0.550, -0.465, 1.097)),
        material=plastic,
        name="pod_mount_flange",
    )

    # Four hollow square upper columns.  The walls intentionally overlap at the
    # corners so each sleeve reads as one welded tube while leaving a clear bore.
    column_positions = (
        (-0.700, -0.320),
        (0.700, -0.320),
        (-0.700, 0.320),
        (0.700, 0.320),
    )
    outer_size = 0.130
    wall = 0.022
    sleeve_height = 0.535
    sleeve_center_z = 0.7875

    for index, (x, y) in enumerate(column_positions):
        frame.visual(
            Box((outer_size, wall, sleeve_height)),
            origin=Origin(xyz=(x, y - outer_size / 2 + wall / 2, sleeve_center_z)),
            material=metal,
            name=f"column_{index}_front_wall",
        )
        frame.visual(
            Box((outer_size, wall, sleeve_height)),
            origin=Origin(xyz=(x, y + outer_size / 2 - wall / 2, sleeve_center_z)),
            material=metal,
            name=f"column_{index}_rear_wall",
        )
        frame.visual(
            Box((wall, outer_size, sleeve_height)),
            origin=Origin(xyz=(x - outer_size / 2 + wall / 2, y, sleeve_center_z)),
            material=metal,
            name=f"column_{index}_side_wall_0",
        )
        frame.visual(
            Box((wall, outer_size, sleeve_height)),
            origin=Origin(xyz=(x + outer_size / 2 - wall / 2, y, sleeve_center_z)),
            material=metal,
            name=f"column_{index}_side_wall_1",
        )
        frame.visual(
            Box((0.185, 0.185, 0.018)),
            origin=Origin(xyz=(x, y, 1.055)),
            material=metal,
            name=f"column_{index}_top_plate",
        )

    # Lower telescoping leg stages.  The joint is at each sleeve's lower lip;
    # the inner member extends upward inside the hollow tube for retained
    # insertion even when the prismatic slide is fully extended.
    for index, (x, y) in enumerate(column_positions):
        stage = model.part(f"leg_stage_{index}")
        stage.visual(
            Box((0.062, 0.062, 0.880)),
            origin=Origin(xyz=(0.0, 0.0, -0.050)),
            material=inner_metal,
            name="inner_member",
        )
        stage.visual(
            Box((0.012, 0.045, 0.090)),
            origin=Origin(xyz=(-0.037, 0.0, 0.245)),
            material=plastic,
            name="guide_shoe_x0",
        )
        stage.visual(
            Box((0.012, 0.045, 0.090)),
            origin=Origin(xyz=(0.037, 0.0, 0.245)),
            material=plastic,
            name="guide_shoe_x1",
        )
        stage.visual(
            Box((0.045, 0.012, 0.090)),
            origin=Origin(xyz=(0.0, -0.037, 0.155)),
            material=plastic,
            name="guide_shoe_y0",
        )
        stage.visual(
            Box((0.045, 0.012, 0.090)),
            origin=Origin(xyz=(0.0, 0.037, 0.155)),
            material=plastic,
            name="guide_shoe_y1",
        )
        stage.visual(
            Box((0.245, 0.165, 0.040)),
            origin=Origin(xyz=(0.0, 0.0, -0.515)),
            material=metal,
            name="foot_plate",
        )
        stage.visual(
            Box((0.062, 0.112, 0.045)),
            origin=Origin(xyz=(0.0, 0.0, -0.475)),
            material=metal,
            name="foot_neck",
        )
        model.articulation(
            f"column_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=stage,
            origin=Origin(xyz=(x, y, 0.535)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=220.0, velocity=0.08, lower=0.0, upper=0.300),
        )

    knob = model.part("control_knob")
    knob.visual(
        Cylinder(radius=0.014, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=metal,
        name="short_shaft",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.074,
            0.036,
            body_style="cylindrical",
            edge_radius=0.002,
            grip=KnobGrip(style="ribbed", count=24, depth=0.0016),
            center=False,
        ),
        "ribbed_side_control_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=plastic,
        name="knob_cap",
    )
    knob.visual(
        Box((0.006, 0.035, 0.004)),
        origin=Origin(xyz=(0.0, 0.015, 0.0845)),
        material=white,
        name="indicator_line",
    )
    model.articulation(
        "pod_to_knob",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=knob,
        origin=Origin(xyz=(0.550, -0.5575, 1.055), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("desk_frame")
    knob = object_model.get_part("control_knob")
    knob_joint = object_model.get_articulation("pod_to_knob")

    top_aabb = ctx.part_element_world_aabb(frame, elem="top_panel")
    ctx.check(
        "broad conference top proportions",
        top_aabb is not None
        and (top_aabb[1][0] - top_aabb[0][0]) > 1.65
        and (top_aabb[1][1] - top_aabb[0][1]) > 0.80
        and (top_aabb[1][2] - top_aabb[0][2]) < 0.09,
        details=f"top_aabb={top_aabb}",
    )

    ctx.check(
        "side knob is a revolute control",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is not None
        and knob_joint.motion_limits.upper is not None
        and knob_joint.motion_limits.lower <= -pi + 1e-6
        and knob_joint.motion_limits.upper >= pi - 1e-6,
        details=f"knob_joint={knob_joint}",
    )
    ctx.expect_contact(
        knob,
        frame,
        elem_a="short_shaft",
        elem_b="control_pod",
        contact_tol=0.001,
        name="knob shaft seats on side pod face",
    )
    with ctx.pose({knob_joint: pi / 2.0}):
        ctx.expect_contact(
            knob,
            frame,
            elem_a="short_shaft",
            elem_b="control_pod",
            contact_tol=0.001,
            name="rotated knob remains seated on pod face",
        )

    for index in range(4):
        stage = object_model.get_part(f"leg_stage_{index}")
        slide = object_model.get_articulation(f"column_{index}_slide")
        ctx.check(
            f"column {index} has vertical prismatic slide",
            slide.articulation_type == ArticulationType.PRISMATIC
            and tuple(slide.axis) == (0.0, 0.0, -1.0)
            and slide.motion_limits is not None
            and slide.motion_limits.lower == 0.0
            and slide.motion_limits.upper == 0.300,
            details=f"slide={slide}",
        )
        ctx.expect_contact(
            stage,
            frame,
            elem_a="guide_shoe_x1",
            elem_b=f"column_{index}_side_wall_1",
            contact_tol=0.001,
            name=f"column {index} guide shoe rides in sleeve",
        )
        ctx.expect_overlap(
            stage,
            frame,
            axes="z",
            elem_a="inner_member",
            elem_b=f"column_{index}_front_wall",
            min_overlap=0.30,
            name=f"column {index} inner member retained at rest",
        )
        rest_pos = ctx.part_world_position(stage)
        with ctx.pose({slide: 0.300}):
            ctx.expect_overlap(
                stage,
                frame,
                axes="z",
                elem_a="inner_member",
                elem_b=f"column_{index}_front_wall",
                min_overlap=0.08,
                name=f"column {index} inner member retained when extended",
            )
            extended_pos = ctx.part_world_position(stage)
        ctx.check(
            f"column {index} slide extends downward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] < rest_pos[2] - 0.25,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
