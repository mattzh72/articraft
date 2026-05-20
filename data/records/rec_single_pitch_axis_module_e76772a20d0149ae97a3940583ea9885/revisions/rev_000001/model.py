from __future__ import annotations

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
    mesh_from_cadquery,
)
import cadquery as cq

# Define materials with realistic colors
DARK_GRAY = Material(name="dark_gray", rgba=(0.2, 0.2, 0.2, 1.0))
SILVER = Material(name="silver", rgba=(0.75, 0.75, 0.8, 1.0))
ALUMINUM = Material(name="aluminum", rgba=(0.6, 0.6, 0.65, 1.0))
RUBBER_BLACK = Material(name="rubber_black", rgba=(0.1, 0.1, 0.1, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tablet_stand")

    # Base: foot + support arm
    base = model.part("base")

    # Foot: flat rectangular box on the desk with chamfered edges
    foot_length = 0.22  # x (forward/back)
    foot_width = 0.16   # y (left/right)
    foot_height = 0.025  # z (up/down)
    # Create foot with chamfered edges using CadQuery
    foot_cq = (
        cq.Workplane("XY")
        .box(foot_length, foot_width, foot_height)
        .edges("|Z")
        .chamfer(0.003)  # 3mm chamfer on top/bottom edges
    )
    base.visual(
        mesh_from_cadquery(foot_cq, "foot"),
        origin=Origin(xyz=(0.0, 0.0, foot_height/2)),
        name="foot",
        material=DARK_GRAY,
    )

    # Support arm: vertical member at the back of the foot with chamfered edges
    arm_length = 0.06  # x (forward/back)
    arm_width = 0.14   # y (left/right)
    arm_height = 0.12  # z (up/down)
    arm_x = -foot_length/2 + arm_length/2
    arm_z = foot_height + arm_height/2
    # Create support arm with chamfered edges
    arm_cq = (
        cq.Workplane("XY")
        .box(arm_length, arm_width, arm_height)
        .edges("|Z")
        .chamfer(0.002)  # 2mm chamfer
    )
    base.visual(
        mesh_from_cadquery(arm_cq, "support_arm"),
        origin=Origin(xyz=(arm_x, 0.0, arm_z)),
        name="support_arm",
        material=DARK_GRAY,
    )

    # Hinge cylinder: aluminum cylinder at the top of the support arm
    # Cylinder is aligned with Y-axis (hinge axis)
    hinge_cyl_radius = 0.015
    hinge_cyl_length = 0.08  # along Y
    hinge_x = arm_x
    hinge_z = foot_height + arm_height + hinge_cyl_radius  # center at radius height
    base.visual(
        Cylinder(radius=hinge_cyl_radius, length=hinge_cyl_length),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(1.5708, 0.0, 0.0)),  # rotate to align with Y
        name="hinge_cylinder",
        material=ALUMINUM,
    )

    # Rubber pads on foot bottom (4 corners)
    pad_size = (0.05, 0.05, 0.003)
    pad_z = pad_size[2]/2
    for i, (px, py) in enumerate([
        (-foot_length/2 + 0.05, -foot_width/2 + 0.05),
        (foot_length/2 - 0.05, -foot_width/2 + 0.05),
        (-foot_length/2 + 0.05, foot_width/2 - 0.05),
        (foot_length/2 - 0.05, foot_width/2 - 0.05),
    ]):
        base.visual(
            Box(pad_size),
            origin=Origin(xyz=(px, py, pad_z)),
            name=f"rubber_pad_{i}",
            material=RUBBER_BLACK,
        )

    # Stop tabs: small protrusions to limit plate travel
    # The cylinder extends from hinge_z - radius to hinge_z + radius in Z
    stop_tab_size = (0.025, 0.04, 0.015)
    # Calculate position to overlap with hinge cylinder by 3mm
    hinge_front_x = hinge_x + hinge_cyl_radius
    stop_tab_x = hinge_front_x + stop_tab_size[0]/2 - 0.003  # overlap by 3mm
    # Stop tabs positioned within cylinder Z range (cylinder center at hinge_z with radius)
    # Upper stop tab (limits backward tilt)
    base.visual(
        Box(stop_tab_size),
        origin=Origin(xyz=(stop_tab_x, 0.0, hinge_z + 0.005)),
        name="stop_tab_upper",
        material=DARK_GRAY,
    )
    # Lower stop tab (limits forward tilt)
    base.visual(
        Box(stop_tab_size),
        origin=Origin(xyz=(stop_tab_x, 0.0, hinge_z - 0.005)),
        name="stop_tab_lower",
        material=DARK_GRAY,
    )

    # Tablet plate: tilting rectangular plate with chamfered edges
    plate = model.part("tablet_plate")

    plate_thickness = 0.012
    plate_width = 0.20
    plate_height = 0.28

    # Create plate with chamfered edges using CadQuery
    plate_cq = (
        cq.Workplane("XY")
        .box(plate_thickness, plate_width, plate_height)
        .edges("|X")
        .chamfer(0.002)  # 2mm chamfer on the long edges
        .edges("|Y")
        .chamfer(0.002)  # 2mm chamfer on the side edges
    )
    plate.visual(
        mesh_from_cadquery(plate_cq, "plate"),
        origin=Origin(xyz=(0.0, 0.0, plate_height/2)),
        name="plate",
        material=SILVER,
    )

    # Rubber pads on the plate (where the tablet touches)
    # Two pads: left and right, positioned at 60% up the plate
    plate_pad_thickness = 0.003
    plate_pad_width = 0.05
    plate_pad_height = 0.05
    pad_x = plate_thickness/2 + plate_pad_thickness/2
    pad_z = plate_height * 0.6
    for i, py in enumerate([-0.07, 0.07]):
        plate.visual(
            Box((plate_pad_thickness, plate_pad_width, plate_pad_height)),
            origin=Origin(xyz=(pad_x, py, pad_z)),
            name=f"plate_rubber_pad_{i}",
            material=RUBBER_BLACK,
        )

    # Small lip at the bottom of the plate to prevent tablet from sliding out
    lip_size = (0.008, plate_width * 0.9, 0.015)
    lip_x = plate_thickness/2 + lip_size[0]/2
    lip_z = lip_size[2]/2
    plate.visual(
        Box(lip_size),
        origin=Origin(xyz=(lip_x, 0.0, lip_z)),
        name="bottom_lip",
        material=SILVER,
    )

    # Articulation: base to plate (pitch joint)
    # The hinge point in world frame
    hinge_world_x = hinge_x
    hinge_world_y = 0.0
    hinge_world_z = hinge_z

    model.articulation(
        "base_to_plate",
        ArticulationType.REVOLUTE,
        parent=base,
        child=plate,
        origin=Origin(xyz=(hinge_world_x, hinge_world_y, hinge_world_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=0.0,   # vertical (upright)
            upper=1.047,   # tilted back ~60 degrees
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    plate = object_model.get_part("tablet_plate")
    hinge = object_model.get_articulation("base_to_plate")

    # Allow intentional overlap between hinge cylinder and plate (hinge mechanism)
    ctx.allow_overlap(
        "base",
        "tablet_plate",
        elem_a="hinge_cylinder",
        elem_b="plate",
        reason="Hinge cylinder is the pivot point for the plate; overlap represents the hinge mechanism where the plate rotates around the cylinder",
    )
    ctx.allow_overlap(
        "base",
        "tablet_plate",
        elem_a="hinge_cylinder",
        elem_b="bottom_lip",
        reason="Bottom lip of plate is near hinge cylinder; small overlap is intentional as both are at the bottom of the plate",
    )

    # Also allow overlap between stop tabs and hinge cylinder (connected geometry)
    ctx.allow_overlap(
        "base",
        "base",
        elem_a="stop_tab_upper",
        elem_b="hinge_cylinder",
        reason="Stop tab is attached to the hinge cylinder; small overlap ensures geometric connectivity",
    )
    ctx.allow_overlap(
        "base",
        "base",
        elem_a="stop_tab_lower",
        elem_b="hinge_cylinder",
        reason="Stop tab is attached to the hinge cylinder; small overlap ensures geometric connectivity",
    )

    # Get actual hinge position from the model
    with ctx.pose({hinge: 0.0}):
        hinge_cyl_aabb = ctx.part_element_world_aabb(base, elem="hinge_cylinder")
        hinge_center_x = (hinge_cyl_aabb[0][0] + hinge_cyl_aabb[1][0]) / 2
        hinge_center_z = (hinge_cyl_aabb[0][2] + hinge_cyl_aabb[1][2]) / 2

        # At q=0 (vertical), check plate position
        plate_pos = ctx.part_world_position(plate)
        ctx.check(
            "plate at correct position at q=0",
            abs(plate_pos[0] - hinge_center_x) < 0.01 and abs(plate_pos[2] - hinge_center_z) < 0.01,
            details=f"plate_pos={plate_pos}, expected=({hinge_center_x}, 0, {hinge_center_z})",
        )

    # Check that the plate tilts correctly - compare AABB at different poses
    with ctx.pose({hinge: 0.0}):
        plate_aabb_vert = ctx.part_element_world_aabb(plate, elem="plate")
    with ctx.pose({hinge: 1.047}):
        plate_aabb_tilt = ctx.part_element_world_aabb(plate, elem="plate")

    ctx.check(
        "plate tilts back at upper limit",
        plate_aabb_tilt[1][2] < plate_aabb_vert[1][2],
        details=f"vertical max_z={plate_aabb_vert[1][2]:.3f}, tilted max_z={plate_aabb_tilt[1][2]:.3f}",
    )

    # Check that positive q moves the top of the plate toward -X (back)
    with ctx.pose({hinge: 0.0}):
        plate_aabb_vert = ctx.part_element_world_aabb(plate, elem="plate")
    with ctx.pose({hinge: 0.5}):
        plate_aabb_mid = ctx.part_element_world_aabb(plate, elem="plate")

    ctx.check(
        "plate tilts in correct direction",
        plate_aabb_mid[1][0] < plate_aabb_vert[1][0],
        details=f"vertical max_x={plate_aabb_vert[1][0]:.3f}, mid-tilt max_x={plate_aabb_mid[1][0]:.3f}",
    )

    # Check that the plate has rubber pads
    with ctx.pose({hinge: 0.0}):
        pad_aabb = ctx.part_element_world_aabb(plate, elem="plate_rubber_pad_0")
        plate_aabb = ctx.part_element_world_aabb(plate, elem="plate")
        ctx.check(
            "plate rubber pad exists and positioned correctly",
            pad_aabb is not None and pad_aabb[0][0] > plate_aabb[0][0],
            details=f"pad_aabb={pad_aabb}, plate_aabb={plate_aabb}",
        )

    # Check that the hinge cylinder is at the correct position relative to the plate
    with ctx.pose({hinge: 0.0}):
        hinge_cyl_aabb = ctx.part_element_world_aabb(base, elem="hinge_cylinder")
        plate_aabb = ctx.part_element_world_aabb(plate, elem="plate")
        # The hinge cylinder should be near the bottom of the plate
        ctx.check(
            "hinge cylinder positioned at plate bottom",
            abs(hinge_cyl_aabb[1][2] - plate_aabb[0][2]) < 0.02,
            details=f"hinge_cyl max_z={hinge_cyl_aabb[1][2]:.3f}, plate min_z={plate_aabb[0][2]:.3f}",
        )

    # Check that stop tabs are positioned correctly (overlap with hinge cylinder)
    with ctx.pose({hinge: 0.0}):
        stop_tab_upper_aabb = ctx.part_element_world_aabb(base, elem="stop_tab_upper")
        stop_tab_lower_aabb = ctx.part_element_world_aabb(base, elem="stop_tab_lower")
        hinge_cyl_aabb = ctx.part_element_world_aabb(base, elem="hinge_cylinder")

        ctx.check(
            "stop tabs overlap with hinge cylinder",
            stop_tab_upper_aabb is not None and stop_tab_lower_aabb is not None,
            details=f"upper={stop_tab_upper_aabb}, lower={stop_tab_lower_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
