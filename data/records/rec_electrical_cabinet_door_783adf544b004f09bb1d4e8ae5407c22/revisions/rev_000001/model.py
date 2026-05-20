from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="switchroom_double_door")

    frame_paint = model.material("frame_paint", rgba=(0.22, 0.24, 0.26, 1.0))
    leaf_paint = model.material("leaf_paint", rgba=(0.72, 0.74, 0.76, 1.0))
    hardware = model.material("hardware", rgba=(0.60, 0.63, 0.66, 1.0))
    latch_black = model.material("latch_black", rgba=(0.18, 0.18, 0.19, 1.0))

    opening_width = 1.60
    opening_height = 2.10
    frame_face = 0.09
    frame_depth = 0.07
    side_gap = 0.003
    center_gap = 0.004
    head_gap = 0.003
    leaf_thickness = 0.045
    leaf_height = opening_height - 0.008 - head_gap
    leaf_width = (opening_width - (2.0 * side_gap) - center_gap) / 2.0
    outer_width = opening_width + (2.0 * frame_face)
    outer_height = opening_height + frame_face
    hinge_axis_y = -0.0305
    hinge_radius = 0.008
    hinge_axis_inset = 0.007
    hinge_assembly_length = 0.12
    frame_knuckle_length = 0.028
    leaf_knuckle_length = hinge_assembly_length - (2.0 * frame_knuckle_length)
    slab_center_y = 0.0305
    left_leaf_outer_x = -(opening_width * 0.5) + side_gap
    right_leaf_outer_x = (opening_width * 0.5) - side_gap
    left_hinge_x = left_leaf_outer_x + hinge_axis_inset
    right_hinge_x = right_leaf_outer_x - hinge_axis_inset
    leaf_bottom = 0.008

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((outer_width, frame_depth, outer_height)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, outer_height * 0.5)),
    )
    frame.visual(
        Box((frame_face, frame_depth, outer_height)),
        origin=Origin(xyz=(-(opening_width * 0.5) - (frame_face * 0.5), 0.0, outer_height * 0.5)),
        material=frame_paint,
        name="left_jamb",
    )
    frame.visual(
        Box((frame_face, frame_depth, outer_height)),
        origin=Origin(xyz=((opening_width * 0.5) + (frame_face * 0.5), 0.0, outer_height * 0.5)),
        material=frame_paint,
        name="right_jamb",
    )
    frame.visual(
        Box((outer_width, frame_depth, frame_face)),
        origin=Origin(xyz=(0.0, 0.0, opening_height + (frame_face * 0.5))),
        material=frame_paint,
        name="head",
    )

    left_leaf = model.part("left_leaf")
    left_leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, leaf_thickness, leaf_height)),
        mass=48.0,
        origin=Origin(xyz=((leaf_width * 0.5) - hinge_axis_inset, slab_center_y, leaf_height * 0.5)),
    )
    left_leaf.visual(
        Box((leaf_width, leaf_thickness, leaf_height)),
        origin=Origin(
            xyz=((leaf_width * 0.5) - hinge_axis_inset, slab_center_y, leaf_height * 0.5),
        ),
        material=leaf_paint,
        name="slab",
    )

    right_leaf = model.part("right_leaf")
    right_leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, leaf_thickness, leaf_height)),
        mass=48.0,
        origin=Origin(xyz=(-(leaf_width * 0.5) + hinge_axis_inset, slab_center_y, leaf_height * 0.5)),
    )
    right_leaf.visual(
        Box((leaf_width, leaf_thickness, leaf_height)),
        origin=Origin(
            xyz=(-(leaf_width * 0.5) + hinge_axis_inset, slab_center_y, leaf_height * 0.5),
        ),
        material=leaf_paint,
        name="slab",
    )

    hinge_assembly_bottoms = (0.23, leaf_bottom + leaf_height - 0.32)
    for index, assembly_bottom in enumerate(hinge_assembly_bottoms, start=1):
        hinge_center_z = (assembly_bottom - leaf_bottom) + (hinge_assembly_length * 0.5)
        lower_frame_center_z = assembly_bottom + (frame_knuckle_length * 0.5)
        upper_frame_center_z = assembly_bottom + hinge_assembly_length - (frame_knuckle_length * 0.5)
        frame_plate_width = 0.014
        frame_plate_thickness = 0.009
        frame_plate_y = hinge_axis_y + hinge_radius - (frame_plate_thickness * 0.5)
        left_frame_plate_x = left_hinge_x - (frame_plate_width * 0.5)
        right_frame_plate_x = right_hinge_x + (frame_plate_width * 0.5)
        leaf_plate_width = 0.020
        leaf_plate_thickness = 0.008
        leaf_plate_y = (leaf_plate_thickness * 0.5)
        left_leaf_plate_x = 0.006
        right_leaf_plate_x = -0.006

        frame.visual(
            Box((frame_plate_width, frame_plate_thickness, frame_knuckle_length)),
            origin=Origin(xyz=(left_frame_plate_x, frame_plate_y, lower_frame_center_z)),
            material=hardware,
            name=f"left_frame_leaf_{index}_lower",
        )
        frame.visual(
            Cylinder(radius=hinge_radius, length=frame_knuckle_length),
            origin=Origin(xyz=(left_hinge_x, hinge_axis_y, lower_frame_center_z)),
            material=hardware,
            name=f"left_frame_hinge_{index}_lower",
        )
        frame.visual(
            Box((frame_plate_width, frame_plate_thickness, frame_knuckle_length)),
            origin=Origin(xyz=(left_frame_plate_x, frame_plate_y, upper_frame_center_z)),
            material=hardware,
            name=f"left_frame_leaf_{index}_upper",
        )
        frame.visual(
            Cylinder(radius=hinge_radius, length=frame_knuckle_length),
            origin=Origin(xyz=(left_hinge_x, hinge_axis_y, upper_frame_center_z)),
            material=hardware,
            name=f"left_frame_hinge_{index}_upper",
        )
        frame.visual(
            Box((frame_plate_width, frame_plate_thickness, frame_knuckle_length)),
            origin=Origin(xyz=(right_frame_plate_x, frame_plate_y, lower_frame_center_z)),
            material=hardware,
            name=f"right_frame_leaf_{index}_lower",
        )
        frame.visual(
            Cylinder(radius=hinge_radius, length=frame_knuckle_length),
            origin=Origin(xyz=(right_hinge_x, hinge_axis_y, lower_frame_center_z)),
            material=hardware,
            name=f"right_frame_hinge_{index}_lower",
        )
        frame.visual(
            Box((frame_plate_width, frame_plate_thickness, frame_knuckle_length)),
            origin=Origin(xyz=(right_frame_plate_x, frame_plate_y, upper_frame_center_z)),
            material=hardware,
            name=f"right_frame_leaf_{index}_upper",
        )
        frame.visual(
            Cylinder(radius=hinge_radius, length=frame_knuckle_length),
            origin=Origin(xyz=(right_hinge_x, hinge_axis_y, upper_frame_center_z)),
            material=hardware,
            name=f"right_frame_hinge_{index}_upper",
        )
        left_leaf.visual(
            Box((leaf_plate_width, leaf_plate_thickness, leaf_knuckle_length)),
            origin=Origin(xyz=(left_leaf_plate_x, leaf_plate_y, hinge_center_z)),
            material=hardware,
            name=f"hinge_leaf_{index}",
        )
        left_leaf.visual(
            Cylinder(radius=hinge_radius, length=leaf_knuckle_length),
            origin=Origin(xyz=(0.0, 0.0, hinge_center_z)),
            material=hardware,
            name=f"hinge_knuckle_{index}",
        )
        right_leaf.visual(
            Box((leaf_plate_width, leaf_plate_thickness, leaf_knuckle_length)),
            origin=Origin(xyz=(right_leaf_plate_x, leaf_plate_y, hinge_center_z)),
            material=hardware,
            name=f"hinge_leaf_{index}",
        )
        right_leaf.visual(
            Cylinder(radius=hinge_radius, length=leaf_knuckle_length),
            origin=Origin(xyz=(0.0, 0.0, hinge_center_z)),
            material=hardware,
            name=f"hinge_knuckle_{index}",
        )

    latch_plate_y = slab_center_y - (leaf_thickness * 0.5) + 0.001
    right_leaf.visual(
        Box((0.050, 0.002, 0.260)),
        origin=Origin(
            xyz=(-(leaf_width - hinge_axis_inset) + 0.026, latch_plate_y, 1.12),
        ),
        material=hardware,
        name="draw_bolt_plate",
    )
    right_leaf.visual(
        Box((0.018, 0.004, 0.105)),
        origin=Origin(
            xyz=(-(leaf_width - hinge_axis_inset) + 0.026, latch_plate_y + 0.001, 1.12),
        ),
        material=latch_black,
        name="draw_bolt_pull",
    )

    model.articulation(
        "frame_to_left_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_leaf,
        origin=Origin(xyz=(left_hinge_x, hinge_axis_y, leaf_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.6),
    )
    model.articulation(
        "frame_to_right_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_leaf,
        origin=Origin(xyz=(right_hinge_x, hinge_axis_y, leaf_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    left_hinge = object_model.get_articulation("frame_to_left_leaf")
    right_hinge = object_model.get_articulation("frame_to_right_leaf")

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
        "left_hinge_axis_vertical",
        left_hinge.axis == (0.0, 0.0, 1.0),
        f"expected left hinge axis (0, 0, 1), got {left_hinge.axis}",
    )
    ctx.check(
        "right_hinge_axis_vertical",
        right_hinge.axis == (0.0, 0.0, -1.0),
        f"expected right hinge axis (0, 0, -1), got {right_hinge.axis}",
    )

    ctx.expect_gap(
        left_leaf,
        frame,
        axis="x",
        positive_elem="slab",
        negative_elem="left_jamb",
        min_gap=0.0025,
        max_gap=0.0035,
        name="left_leaf_jamb_clearance",
    )
    ctx.expect_gap(
        frame,
        right_leaf,
        axis="x",
        positive_elem="right_jamb",
        negative_elem="slab",
        min_gap=0.0025,
        max_gap=0.0035,
        name="right_leaf_jamb_clearance",
    )
    ctx.expect_gap(
        right_leaf,
        left_leaf,
        axis="x",
        positive_elem="slab",
        negative_elem="slab",
        min_gap=0.0035,
        max_gap=0.0045,
        name="center_meeting_gap",
    )
    ctx.expect_gap(
        frame,
        left_leaf,
        axis="z",
        positive_elem="head",
        negative_elem="slab",
        min_gap=0.0025,
        max_gap=0.0035,
        name="left_leaf_head_clearance",
    )
    ctx.expect_gap(
        frame,
        right_leaf,
        axis="z",
        positive_elem="head",
        negative_elem="slab",
        min_gap=0.0025,
        max_gap=0.0035,
        name="right_leaf_head_clearance",
    )
    ctx.expect_contact(
        left_leaf,
        frame,
        elem_a="hinge_knuckle_1",
        elem_b="left_frame_hinge_1_lower",
        contact_tol=1e-6,
        name="left_lower_hinge_contact",
    )
    ctx.expect_contact(
        right_leaf,
        frame,
        elem_a="hinge_knuckle_1",
        elem_b="right_frame_hinge_1_lower",
        contact_tol=1e-6,
        name="right_lower_hinge_contact",
    )

    with ctx.pose({left_hinge: 1.2, right_hinge: 1.2}):
        left_aabb = ctx.part_world_aabb(left_leaf)
        right_aabb = ctx.part_world_aabb(right_leaf)
        ctx.check(
            "left_leaf_opens_clear_of_centerline",
            left_aabb is not None and left_aabb[1][1] > 0.65,
            f"left leaf open-pose AABB was {left_aabb}",
        )
        ctx.check(
            "right_leaf_opens_clear_of_centerline",
            right_aabb is not None and right_aabb[1][1] > 0.65,
            f"right leaf open-pose AABB was {right_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
