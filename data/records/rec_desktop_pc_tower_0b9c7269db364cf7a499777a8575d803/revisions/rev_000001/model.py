from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_desktop_pc")

    aluminium = model.material("satin_aluminium", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_aluminium = model.material("anodized_black", rgba=(0.025, 0.028, 0.030, 1.0))
    smoked_glass = model.material("smoked_tempered_glass", rgba=(0.35, 0.50, 0.62, 0.34))
    black_mesh = model.material("black_powdercoat_mesh", rgba=(0.010, 0.012, 0.014, 1.0))
    drive_metal = model.material("drive_cage_galvanized", rgba=(0.45, 0.46, 0.45, 1.0))
    drive_label = model.material("drive_label_blue", rgba=(0.05, 0.16, 0.42, 1.0))

    def box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    width = 0.36
    depth = 0.48
    height = 0.44
    tube = 0.026

    frame = model.part("frame")

    # Rectangular aluminium tube skeleton: four posts plus top and bottom perimeter rails.
    for ix, x in enumerate((-width / 2, width / 2)):
        for iy, y in enumerate((-depth / 2, depth / 2)):
            box(frame, f"post_{ix}_{iy}", (tube, tube, height), (x, y, height / 2), aluminium)

    for z_name, z in (("lower", tube / 2), ("upper", height - tube / 2)):
        for y_name, y in (("front", -depth / 2), ("rear", depth / 2)):
            box(frame, f"{z_name}_{y_name}_crossbar", (width + tube, tube, tube), (0.0, y, z), aluminium)
        for x_name, x in (("side_0", -width / 2), ("side_1", width / 2)):
            box(frame, f"{z_name}_{x_name}_rail", (tube, depth + tube, tube), (x, 0.0, z), aluminium)

    # Front interior hinge post for the drive cage, tied into the front rails.
    drive_post_x = -width / 2 + 0.10
    drive_post_y = -depth / 2 + tube / 2
    box(frame, "drive_post", (0.020, 0.020, height - 2 * tube), (drive_post_x, drive_post_y, height / 2), aluminium)
    box(frame, "drive_post_top_gusset", (0.050, 0.018, 0.018), (drive_post_x + 0.012, drive_post_y, height - tube / 2), aluminium)
    box(frame, "drive_post_bottom_gusset", (0.050, 0.018, 0.018), (drive_post_x + 0.012, drive_post_y, tube / 2), aluminium)

    # Small feet under the open test-bench frame.
    for ix, x in enumerate((-width / 2, width / 2)):
        for iy, y in enumerate((-depth / 2, depth / 2)):
            box(frame, f"rubber_foot_{ix}_{iy}", (0.055, 0.055, 0.014), (x, y, -0.007), dark_aluminium)

    # Static hinge brackets mounted to the skeleton.  The moving leaves live on the articulated panels.
    side_hinge_x = width / 2 + 0.025
    side_hinge_y = depth / 2 + tube / 2
    for suffix, z in (("lower", 0.145), ("upper", 0.315)):
        box(frame, f"glass_{suffix}_hinge_block", (0.006, 0.008, 0.064), (side_hinge_x - 0.009, side_hinge_y + 0.004, z), dark_aluminium)
        cyl(frame, f"glass_{suffix}_fixed_knuckle", 0.0045, 0.040, (side_hinge_x - 0.003, side_hinge_y + 0.005, z), dark_aluminium)

    top_hinge_y = -depth / 2 - 0.018
    top_hinge_z = height + tube / 2 + 0.003
    for suffix, x in (("left", -0.105), ("right", 0.105)):
        box(frame, f"top_{suffix}_hinge_block", (0.066, 0.010, 0.018), (x, top_hinge_y, height + 0.008), dark_aluminium)
        cyl(
            frame,
            f"top_{suffix}_fixed_knuckle",
            0.004,
            0.052,
            (x, top_hinge_y - 0.002, top_hinge_z),
            dark_aluminium,
            rpy=(0.0, math.pi / 2.0, 0.0),
        )

    drive_hinge_x = drive_post_x + 0.020
    drive_hinge_y = drive_post_y + 0.017
    drive_hinge_z = 0.205
    box(frame, "drive_hinge_bracket", (0.018, 0.030, 0.185), (drive_hinge_x - 0.006, drive_hinge_y, drive_hinge_z), dark_aluminium)

    # Tempered-glass side panel: the part frame is the vertical hinge line.
    glass_panel = model.part("glass_panel")
    glass_depth = 0.430
    glass_height = 0.380
    glass_thick = 0.006
    box(glass_panel, "glass_sheet", (glass_thick, glass_depth, glass_height), (0.0, -glass_depth / 2, 0.0), smoked_glass)
    rail = 0.012
    box(glass_panel, "rear_rail", (0.010, rail, glass_height + 0.020), (0.002, -rail / 2, 0.0), dark_aluminium)
    box(glass_panel, "front_rail", (0.010, rail, glass_height + 0.020), (0.002, -glass_depth + rail / 2, 0.0), dark_aluminium)
    box(glass_panel, "top_rail", (0.010, glass_depth, rail), (0.002, -glass_depth / 2, glass_height / 2), dark_aluminium)
    box(glass_panel, "bottom_rail", (0.010, glass_depth, rail), (0.002, -glass_depth / 2, -glass_height / 2), dark_aluminium)
    for suffix, z in (("lower", -0.075), ("upper", 0.095)):
        box(glass_panel, f"{suffix}_moving_leaf", (0.004, 0.040, 0.052), (0.006, -0.020, z), dark_aluminium)
        cyl(glass_panel, f"{suffix}_moving_knuckle", 0.005, 0.056, (-0.003, 0.005, z), dark_aluminium)

    model.articulation(
        "frame_to_glass_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=glass_panel,
        origin=Origin(xyz=(side_hinge_x, side_hinge_y, height / 2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=8.0, velocity=1.4),
    )

    # Hinged top mesh panel: a perforated sheet in a stiff rectangular surround.
    top_panel = model.part("top_mesh_panel")
    top_width = 0.310
    top_depth = 0.420
    perforated = PerforatedPanelGeometry(
        (top_width, top_depth),
        0.005,
        hole_diameter=0.008,
        pitch=(0.016, 0.016),
        frame=0.018,
        corner_radius=0.004,
        stagger=True,
    )
    top_panel.visual(
        mesh_from_geometry(perforated, "top_perforated_mesh"),
        origin=Origin(xyz=(0.0, top_depth / 2, 0.0)),
        material=black_mesh,
        name="perforated_mesh",
    )
    box(top_panel, "front_frame", (top_width + 0.024, 0.014, 0.014), (0.0, 0.020, 0.001), dark_aluminium)
    box(top_panel, "rear_frame", (top_width + 0.024, 0.014, 0.014), (0.0, top_depth - 0.007, 0.001), dark_aluminium)
    box(top_panel, "side_frame_0", (0.014, top_depth, 0.014), (-top_width / 2, top_depth / 2, 0.001), dark_aluminium)
    box(top_panel, "side_frame_1", (0.014, top_depth, 0.014), (top_width / 2, top_depth / 2, 0.001), dark_aluminium)
    for suffix, x in (("left", -0.105), ("right", 0.105)):
        box(top_panel, f"{suffix}_moving_leaf", (0.054, 0.028, 0.004), (x, 0.014, -0.006), dark_aluminium)
        cyl(
            top_panel,
            f"{suffix}_moving_knuckle",
            0.0045,
            0.052,
            (x, -0.002, -0.001),
            dark_aluminium,
            rpy=(0.0, math.pi / 2.0, 0.0),
        )

    model.articulation(
        "frame_to_top_mesh_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=top_panel,
        origin=Origin(xyz=(0.0, top_hinge_y, top_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=6.0, velocity=1.2),
    )

    # Swing-out 3.5-inch drive cage.  It is modeled as an open rail cage with two mounted drives.
    drive_cage = model.part("drive_cage")
    cage_x0 = 0.014
    cage_x1 = 0.174
    cage_y0 = 0.014
    cage_y1 = 0.144
    cage_z = 0.180
    member = 0.012
    cage_cx = (cage_x0 + cage_x1) / 2
    cage_cy = (cage_y0 + cage_y1) / 2
    for x in (cage_x0, cage_x1):
        for y in (cage_y0, cage_y1):
            box(drive_cage, f"vertical_rail_{x:.3f}_{y:.3f}", (member, member, cage_z), (x, y, 0.0), drive_metal)
    for z in (-cage_z / 2, cage_z / 2):
        for y in (cage_y0, cage_y1):
            box(drive_cage, f"x_rail_{y:.3f}_{z:.3f}", (cage_x1 - cage_x0 + member, member, member), (cage_cx, y, z), drive_metal)
        for x in (cage_x0, cage_x1):
            box(drive_cage, f"y_rail_{x:.3f}_{z:.3f}", (member, cage_y1 - cage_y0 + member, member), (x, cage_cy, z), drive_metal)
    for z in (-0.038, 0.038):
        box(drive_cage, f"drive_shelf_{z:.3f}", (cage_x1 - cage_x0 + 0.004, cage_y1 - cage_y0 + 0.004, 0.005), (cage_cx, cage_cy, z), drive_metal)
        box(drive_cage, f"hard_drive_{z:.3f}", (0.132, 0.096, 0.024), (cage_cx + 0.008, cage_cy, z + 0.004), dark_aluminium)
        box(drive_cage, f"drive_label_{z:.3f}", (0.070, 0.040, 0.002), (cage_cx + 0.010, cage_cy - 0.020, z + 0.017), drive_label)
    cyl(drive_cage, "drive_moving_knuckle", 0.006, 0.170, (0.004, cage_y0, 0.0), drive_metal)
    box(drive_cage, "drive_moving_leaf", (0.020, 0.036, 0.150), (0.012, cage_y0 + 0.006, 0.0), drive_metal)

    model.articulation(
        "frame_to_drive_cage",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=drive_cage,
        origin=Origin(xyz=(drive_hinge_x, drive_hinge_y, drive_hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=5.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    glass = object_model.get_part("glass_panel")
    top = object_model.get_part("top_mesh_panel")
    drive = object_model.get_part("drive_cage")

    glass_joint = object_model.get_articulation("frame_to_glass_panel")
    top_joint = object_model.get_articulation("frame_to_top_mesh_panel")
    drive_joint = object_model.get_articulation("frame_to_drive_cage")

    for suffix in ("lower", "upper"):
        ctx.allow_overlap(
            frame,
            glass,
            elem_a=f"glass_{suffix}_fixed_knuckle",
            elem_b=f"{suffix}_moving_knuckle",
            reason="The glass door hinge knuckles are intentionally modeled as a captured pin/barrel fit at the rear frame.",
        )
    for suffix in ("left", "right"):
        ctx.allow_overlap(
            frame,
            top,
            elem_a=f"top_{suffix}_fixed_knuckle",
            elem_b=f"{suffix}_moving_knuckle",
            reason="The top mesh panel hinge barrels intentionally share the hinge-pin volume.",
        )
        ctx.allow_overlap(
            frame,
            top,
            elem_a=f"top_{suffix}_hinge_block",
            elem_b=f"{suffix}_moving_knuckle",
            reason="The fixed hinge block locally wraps the simplified top-panel barrel at the front edge.",
        )
    ctx.allow_overlap(
        frame,
        drive,
        elem_a="drive_hinge_bracket",
        elem_b="drive_moving_knuckle",
        reason="The swing-out drive cage hinge knuckle is captured by the fixed interior-post bracket.",
    )

    ctx.check(
        "three primary revolute mechanisms",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (glass_joint, top_joint, drive_joint)),
        details="glass side panel, top mesh panel, and drive cage should each be on a revolute hinge",
    )

    with ctx.pose({glass_joint: 0.0, top_joint: 0.0, drive_joint: 0.0}):
        ctx.expect_gap(
            glass,
            frame,
            axis="x",
            min_gap=0.006,
            positive_elem="glass_sheet",
            negative_elem="upper_side_1_rail",
            name="closed glass sheet sits outside side rail",
        )
        ctx.expect_gap(
            top,
            frame,
            axis="z",
            min_gap=0.006,
            positive_elem="perforated_mesh",
            negative_elem="upper_front_crossbar",
            name="top perforated mesh clears upper frame",
        )
        ctx.expect_overlap(top, frame, axes="x", min_overlap=0.25, name="top mesh spans the frame width")
        for suffix in ("lower", "upper"):
            ctx.expect_overlap(
                glass,
                frame,
                axes="z",
                min_overlap=0.035,
                elem_a=f"{suffix}_moving_knuckle",
                elem_b=f"glass_{suffix}_fixed_knuckle",
                name=f"glass {suffix} hinge has captured vertical engagement",
            )
        for suffix in ("left", "right"):
            ctx.expect_overlap(
                top,
                frame,
                axes="x",
                min_overlap=0.040,
                elem_a=f"{suffix}_moving_knuckle",
                elem_b=f"top_{suffix}_fixed_knuckle",
                name=f"top {suffix} hinge has captured pin engagement",
            )
            ctx.expect_overlap(
                top,
                frame,
                axes="x",
                min_overlap=0.040,
                elem_a=f"{suffix}_moving_knuckle",
                elem_b=f"top_{suffix}_hinge_block",
                name=f"top {suffix} hinge block wraps barrel",
            )
        ctx.expect_overlap(
            drive,
            frame,
            axes="z",
            min_overlap=0.120,
            elem_a="drive_moving_knuckle",
            elem_b="drive_hinge_bracket",
            name="drive cage hinge has tall captured engagement",
        )

        glass_rest_aabb = ctx.part_world_aabb(glass)
        top_rest_aabb = ctx.part_world_aabb(top)
        drive_rest_aabb = ctx.part_world_aabb(drive)

    with ctx.pose({glass_joint: 1.25}):
        glass_open_aabb = ctx.part_world_aabb(glass)
    with ctx.pose({top_joint: 1.0}):
        top_open_aabb = ctx.part_world_aabb(top)
    with ctx.pose({drive_joint: 0.75}):
        drive_open_aabb = ctx.part_world_aabb(drive)

    ctx.check(
        "glass side panel swings outward",
        glass_rest_aabb is not None
        and glass_open_aabb is not None
        and glass_open_aabb[1][0] > glass_rest_aabb[1][0] + 0.09,
        details=f"rest={glass_rest_aabb}, open={glass_open_aabb}",
    )
    ctx.check(
        "top mesh panel hinges upward",
        top_rest_aabb is not None
        and top_open_aabb is not None
        and top_open_aabb[1][2] > top_rest_aabb[1][2] + 0.18,
        details=f"rest={top_rest_aabb}, open={top_open_aabb}",
    )
    ctx.check(
        "drive cage swings toward service side",
        drive_rest_aabb is not None
        and drive_open_aabb is not None
        and drive_open_aabb[1][0] > drive_rest_aabb[1][0] + 0.035,
        details=f"rest={drive_rest_aabb}, open={drive_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
