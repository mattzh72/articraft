from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


SCREEN_WIDTH = 1.02
SCREEN_HEIGHT = 0.63
SCREEN_DEPTH = 0.060
SLEEVE_TOP_Z = 0.330
HEIGHT_TRAVEL = 0.220
HINGE_LOCAL_Z = 0.420


def _rounded_button_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.034, 0.018, 0.0045),
            0.008,
            cap=True,
            center=True,
        ),
        "front_button_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_design_monitor")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.055, 0.060, 0.066, 1.0))
    soft_graphite = model.material("soft_graphite", rgba=(0.115, 0.122, 0.132, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.46, 0.48, 0.50, 1.0))
    black_glass = model.material("black_glass", rgba=(0.010, 0.013, 0.018, 0.72))
    button_rubber = model.material("button_rubber", rgba=(0.030, 0.032, 0.035, 1.0))
    label_grey = model.material("label_grey", rgba=(0.64, 0.66, 0.68, 1.0))
    power_green = model.material("power_green", rgba=(0.16, 0.88, 0.42, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((0.46, 0.32, 0.026)),
        origin=Origin(xyz=(0.0, 0.020, 0.013)),
        material=matte_black,
        name="weighted_plate",
    )
    base_plate.visual(
        Box((0.38, 0.24, 0.010)),
        origin=Origin(xyz=(0.0, 0.028, 0.031)),
        material=dark_graphite,
        name="top_dish",
    )
    base_plate.visual(
        Box((0.39, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.125, 0.006)),
        material=soft_graphite,
        name="front_rubber_foot",
    )
    base_plate.visual(
        Box((0.34, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.175, 0.006)),
        material=soft_graphite,
        name="rear_rubber_foot",
    )

    # A rectangular hollow lower sleeve: four connected walls leave a real
    # clearanced slot for the prismatic height column instead of hiding a solid
    # telescoping overlap.
    sleeve_height = SLEEVE_TOP_Z - 0.024
    sleeve_center_z = 0.024 + sleeve_height * 0.5
    base_plate.visual(
        Box((0.010, 0.074, sleeve_height)),
        origin=Origin(xyz=(-0.041, 0.080, sleeve_center_z)),
        material=satin_metal,
        name="sleeve_side_0",
    )
    base_plate.visual(
        Box((0.010, 0.074, sleeve_height)),
        origin=Origin(xyz=(0.041, 0.080, sleeve_center_z)),
        material=satin_metal,
        name="sleeve_side_1",
    )
    base_plate.visual(
        Box((0.092, 0.010, sleeve_height)),
        origin=Origin(xyz=(0.0, 0.038, sleeve_center_z)),
        material=satin_metal,
        name="sleeve_front_wall",
    )
    base_plate.visual(
        Box((0.092, 0.010, sleeve_height)),
        origin=Origin(xyz=(0.0, 0.122, sleeve_center_z)),
        material=satin_metal,
        name="sleeve_rear_wall",
    )
    base_plate.visual(
        Box((0.014, 0.104, 0.018)),
        origin=Origin(xyz=(-0.051, 0.080, SLEEVE_TOP_Z + 0.004)),
        material=dark_graphite,
        name="collar_side_0",
    )
    base_plate.visual(
        Box((0.014, 0.104, 0.018)),
        origin=Origin(xyz=(0.051, 0.080, SLEEVE_TOP_Z + 0.004)),
        material=dark_graphite,
        name="collar_side_1",
    )
    base_plate.visual(
        Box((0.116, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.026, SLEEVE_TOP_Z + 0.004)),
        material=dark_graphite,
        name="collar_front",
    )
    base_plate.visual(
        Box((0.116, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.134, SLEEVE_TOP_Z + 0.004)),
        material=dark_graphite,
        name="collar_rear",
    )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Box((0.058, 0.038, 0.640)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=satin_metal,
        name="inner_mast",
    )
    upper_column.visual(
        Box((0.007, 0.030, 0.045)),
        origin=Origin(xyz=(-0.0325, 0.0, -0.150)),
        material=soft_graphite,
        name="guide_pad_0",
    )
    upper_column.visual(
        Box((0.007, 0.030, 0.045)),
        origin=Origin(xyz=(0.0325, 0.0, -0.150)),
        material=soft_graphite,
        name="guide_pad_1",
    )
    upper_column.visual(
        Box((0.078, 0.052, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=dark_graphite,
        name="top_neck",
    )
    upper_column.visual(
        Box((0.220, 0.056, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, HINGE_LOCAL_Z - 0.068)),
        material=dark_graphite,
        name="yoke_bridge",
    )
    upper_column.visual(
        Box((0.030, 0.074, 0.112)),
        origin=Origin(xyz=(-0.095, 0.0, HINGE_LOCAL_Z)),
        material=dark_graphite,
        name="yoke_cheek_0",
    )
    upper_column.visual(
        Box((0.030, 0.074, 0.112)),
        origin=Origin(xyz=(0.095, 0.0, HINGE_LOCAL_Z)),
        material=dark_graphite,
        name="yoke_cheek_1",
    )
    upper_column.visual(
        Box((0.006, 0.006, 0.360)),
        origin=Origin(xyz=(0.031, -0.019, 0.060)),
        material=label_grey,
        name="height_scale",
    )

    height_slide = model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base_plate,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.080, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=HEIGHT_TRAVEL),
    )

    tilt_carrier = model.part("tilt_carrier")
    tilt_carrier.visual(
        Cylinder(radius=0.016, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="tilt_axle",
    )
    tilt_carrier.visual(
        Box((0.128, 0.010, 0.096)),
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=dark_graphite,
        name="rotation_plate",
    )
    tilt_carrier.visual(
        Box((0.048, 0.026, 0.034)),
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
        material=dark_graphite,
        name="axle_boss",
    )
    tilt_carrier.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.0, -0.027, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="turntable_face",
    )

    tilt_hinge = model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_column,
        child=tilt_carrier,
        origin=Origin(xyz=(0.0, 0.0, HINGE_LOCAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.35, upper=0.28),
    )

    display_shell = model.part("display_shell")
    display_shell.visual(
        Box((SCREEN_WIDTH, SCREEN_DEPTH, SCREEN_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.076, 0.0)),
        material=dark_graphite,
        name="rear_shell",
    )
    display_shell.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.865, 0.485),
                (0.995, 0.600),
                0.026,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.012,
                outer_corner_radius=0.022,
            ),
            "front_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.112, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="front_bezel",
    )
    display_shell.visual(
        Box((0.875, 0.004, 0.495)),
        origin=Origin(xyz=(0.0, -0.123, 0.030)),
        material=black_glass,
        name="screen_glass",
    )
    display_shell.visual(
        Box((0.545, 0.010, 0.044)),
        origin=Origin(xyz=(0.195, -0.126, -0.272)),
        material=matte_black,
        name="control_recess",
    )
    display_shell.visual(
        Cylinder(radius=0.062, length=0.018),
        origin=Origin(xyz=(0.0, -0.039, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_graphite,
        name="rear_rotation_hub",
    )
    display_shell.visual(
        Box((0.118, 0.012, 0.088)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=soft_graphite,
        name="vesa_block",
    )
    display_shell.visual(
        Box((0.012, 0.003, 0.004)),
        origin=Origin(xyz=(-0.055, -0.130, -0.274)),
        material=power_green,
        name="power_led",
    )

    rotation_joint = model.articulation(
        "view_rotation",
        ArticulationType.CONTINUOUS,
        parent=tilt_carrier,
        child=display_shell,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.6),
    )

    button_mesh = _rounded_button_mesh()
    button_x_positions = (0.085, 0.137, 0.189, 0.241, 0.293, 0.345)
    button_joints = []
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_rubber,
            name="button_cap",
        )
        joint = model.articulation(
            f"button_push_{index}",
            ArticulationType.PRISMATIC,
            parent=display_shell,
            child=button,
            origin=Origin(xyz=(x_pos, -0.135, -0.272)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.035, lower=0.0, upper=0.004),
        )
        button_joints.append(joint)

    # Keep these local names alive for readability and to make the intended
    # primary mechanisms obvious in generated model metadata.
    model.meta["primary_mechanisms"] = (
        height_slide.name,
        tilt_hinge.name,
        rotation_joint.name,
        tuple(j.name for j in button_joints),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    column = object_model.get_part("upper_column")
    display = object_model.get_part("display_shell")
    first_button = object_model.get_part("button_0")
    height_slide = object_model.get_articulation("height_slide")
    view_rotation = object_model.get_articulation("view_rotation")
    tilt_hinge = object_model.get_articulation("tilt_hinge")
    button_push = object_model.get_articulation("button_push_0")

    ctx.expect_within(
        column,
        base,
        axes="xy",
        inner_elem="inner_mast",
        margin=0.0,
        name="inner mast fits within lower sleeve footprint",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="sleeve_front_wall",
        min_overlap=0.16,
        name="collapsed height column remains deeply engaged",
    )

    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({height_slide: HEIGHT_TRAVEL}):
        extended_column_pos = ctx.part_world_position(column)
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="sleeve_front_wall",
            min_overlap=0.035,
            name="extended height column retains insertion",
        )
    ctx.check(
        "height slide raises display support",
        rest_column_pos is not None
        and extended_column_pos is not None
        and extended_column_pos[2] > rest_column_pos[2] + HEIGHT_TRAVEL * 0.90,
        details=f"rest={rest_column_pos}, extended={extended_column_pos}",
    )

    rest_display_aabb = ctx.part_world_aabb(display)
    with ctx.pose({view_rotation: math.pi / 2.0}):
        portrait_aabb = ctx.part_world_aabb(display)
    if rest_display_aabb is not None and portrait_aabb is not None:
        rest_mins, rest_maxs = rest_display_aabb
        portrait_mins, portrait_maxs = portrait_aabb
        rest_w = float(rest_maxs[0] - rest_mins[0])
        rest_h = float(rest_maxs[2] - rest_mins[2])
        portrait_w = float(portrait_maxs[0] - portrait_mins[0])
        portrait_h = float(portrait_maxs[2] - portrait_mins[2])
        ctx.check(
            "landscape rest and portrait rotation",
            rest_w > rest_h * 1.45 and portrait_h > portrait_w * 1.45,
            details=f"rest=({rest_w:.3f}, {rest_h:.3f}), portrait=({portrait_w:.3f}, {portrait_h:.3f})",
        )
    else:
        ctx.fail("display aabb available", "Expected rest and portrait display AABBs.")

    rest_display_pos = ctx.part_world_position(display)
    with ctx.pose({tilt_hinge: 0.25}):
        tilted_display_pos = ctx.part_world_position(display)
    ctx.check(
        "tilt hinge keeps display centered on stand head",
        rest_display_pos is not None
        and tilted_display_pos is not None
        and abs(tilted_display_pos[0] - rest_display_pos[0]) < 0.002,
        details=f"rest={rest_display_pos}, tilted={tilted_display_pos}",
    )

    ctx.expect_gap(
        display,
        first_button,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="control_recess",
        negative_elem="button_cap",
        name="front button cap is tucked against control recess",
    )
    rest_button_pos = ctx.part_world_position(first_button)
    with ctx.pose({button_push: 0.004}):
        pressed_button_pos = ctx.part_world_position(first_button)
    ctx.check(
        "front button pushes inward independently",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.003,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
