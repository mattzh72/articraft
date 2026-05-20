from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="folding_platform_hand_truck")

    frame_finish = model.material("frame_finish", rgba=(0.66, 0.69, 0.72, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.28, 0.30, 0.33, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.64, 0.66, 0.69, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.028, 0.030, 0.900)),
        origin=Origin(xyz=(0.0, 0.180, 0.420)),
        material=frame_finish,
        name="elem_left_rail",
    )
    frame.visual(
        Box((0.028, 0.030, 0.900)),
        origin=Origin(xyz=(0.0, -0.180, 0.420)),
        material=frame_finish,
        name="elem_right_rail",
    )
    frame.visual(
        Box((0.022, 0.330, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=dark_frame,
        name="elem_lower_crossbar",
    )
    frame.visual(
        Box((0.022, 0.330, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=dark_frame,
        name="elem_mid_crossbar",
    )
    frame.visual(
        Box((0.040, 0.330, 0.050)),
        origin=Origin(xyz=(0.010, 0.0, 0.880)),
        material=dark_frame,
        name="elem_top_bridge",
    )
    frame.visual(
        Box((0.030, 0.030, 0.100)),
        origin=Origin(xyz=(0.020, 0.140, 0.930)),
        material=frame_finish,
        name="elem_left_handle_post",
    )
    frame.visual(
        Box((0.030, 0.030, 0.100)),
        origin=Origin(xyz=(0.020, -0.140, 0.930)),
        material=frame_finish,
        name="elem_right_handle_post",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.280),
        origin=Origin(xyz=(0.040, 0.0, 0.980), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_frame,
        name="elem_handle_grip",
    )
    frame.visual(
        Box((0.100, 0.032, 0.180)),
        origin=Origin(xyz=(-0.020, 0.180, -0.005)),
        material=dark_frame,
        name="elem_left_wheel_bracket",
    )
    frame.visual(
        Box((0.100, 0.032, 0.180)),
        origin=Origin(xyz=(-0.020, -0.180, -0.005)),
        material=dark_frame,
        name="elem_right_wheel_bracket",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.407),
        origin=Origin(xyz=(-0.060, 0.0, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_frame,
        name="elem_axle",
    )
    frame.visual(
        Box((0.260, 0.360, 0.028)),
        origin=Origin(xyz=(0.130, 0.0, -0.068)),
        material=frame_finish,
        name="elem_toe_plate",
    )
    frame.visual(
        Box((0.020, 0.360, 0.050)),
        origin=Origin(xyz=(0.250, 0.0, -0.029)),
        material=frame_finish,
        name="elem_toe_lip",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.330),
        origin=Origin(xyz=(-0.030, 0.0, 0.005), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_frame,
        name="elem_shelf_hinge_rod",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.320, 0.500, 1.080)),
        mass=7.0,
        origin=Origin(xyz=(0.020, 0.0, 0.420)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=0.125, length=0.055),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="elem_left_tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="elem_left_hub",
    )
    left_wheel.visual(
        Box((0.014, 0.008, 0.020)),
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        material=hub_gray,
        name="elem_left_valve_stem",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.055),
        mass=0.9,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=0.125, length=0.055),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="elem_right_tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="elem_right_hub",
    )
    right_wheel.visual(
        Box((0.014, 0.008, 0.020)),
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        material=hub_gray,
        name="elem_right_valve_stem",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.055),
        mass=0.9,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    secondary_shelf = model.part("secondary_shelf")
    secondary_shelf.visual(
        Box((0.016, 0.280, 0.100)),
        origin=Origin(xyz=(0.014, 0.0, -0.050)),
        material=frame_finish,
        name="elem_shelf_panel",
    )
    secondary_shelf.visual(
        Box((0.016, 0.240, 0.010)),
        origin=Origin(xyz=(0.014, 0.0, -0.002)),
        material=dark_frame,
        name="elem_shelf_hinge_leaf",
    )
    secondary_shelf.visual(
        Box((0.020, 0.280, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, -0.100)),
        material=frame_finish,
        name="elem_shelf_lip",
    )
    secondary_shelf.inertial = Inertial.from_geometry(
        Box((0.110, 0.290, 0.020)),
        mass=0.7,
        origin=Origin(xyz=(0.050, 0.0, -0.050)),
    )

    model.articulation(
        "frame_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(-0.060, 0.232, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=28.0),
    )
    model.articulation(
        "frame_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(-0.060, -0.232, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=28.0),
    )
    model.articulation(
        "frame_to_secondary_shelf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=secondary_shelf,
        origin=Origin(xyz=(-0.030, 0.0, 0.005)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    secondary_shelf = object_model.get_part("secondary_shelf")

    left_wheel_joint = object_model.get_articulation("frame_to_left_wheel")
    right_wheel_joint = object_model.get_articulation("frame_to_right_wheel")
    shelf_joint = object_model.get_articulation("frame_to_secondary_shelf")

    ctx.check(
        "wheel joints are continuous on the axle axis",
        left_wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_wheel_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(right_wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=(
            f"left_type={left_wheel_joint.articulation_type}, "
            f"right_type={right_wheel_joint.articulation_type}, "
            f"left_axis={left_wheel_joint.axis}, right_axis={right_wheel_joint.axis}"
        ),
    )
    ctx.check(
        "secondary shelf hinge opens forward",
        shelf_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(shelf_joint.axis) == (0.0, -1.0, 0.0)
        and shelf_joint.motion_limits is not None
        and shelf_joint.motion_limits.lower == 0.0
        and shelf_joint.motion_limits.upper is not None
        and shelf_joint.motion_limits.upper >= 1.4,
        details=(
            f"type={shelf_joint.articulation_type}, axis={shelf_joint.axis}, "
            f"limits={shelf_joint.motion_limits}"
        ),
    )

    ctx.expect_gap(
        left_wheel,
        frame,
        axis="y",
        positive_elem="elem_left_tire",
        negative_elem="elem_left_wheel_bracket",
        min_gap=0.004,
        max_gap=0.015,
        name="left wheel clears the left frame bracket",
    )
    ctx.expect_gap(
        frame,
        right_wheel,
        axis="y",
        positive_elem="elem_right_wheel_bracket",
        negative_elem="elem_right_tire",
        min_gap=0.004,
        max_gap=0.015,
        name="right wheel clears the right frame bracket",
    )

    closed_valve_aabb = ctx.part_element_world_aabb(left_wheel, elem="elem_left_valve_stem")
    closed_shelf_aabb = ctx.part_element_world_aabb(secondary_shelf, elem="elem_shelf_panel")

    with ctx.pose({left_wheel_joint: -pi / 2.0, shelf_joint: 1.50}):
        ctx.expect_overlap(
            secondary_shelf,
            frame,
            axes="xy",
            elem_a="elem_shelf_panel",
            elem_b="elem_toe_plate",
            min_overlap=0.060,
            name="deployed shelf stays centered behind the toe plate footprint",
        )
        ctx.expect_gap(
            secondary_shelf,
            frame,
            axis="z",
            positive_elem="elem_shelf_panel",
            negative_elem="elem_toe_plate",
            min_gap=0.050,
            max_gap=0.090,
            name="deployed shelf sits above the main toe plate",
        )
        open_valve_aabb = ctx.part_element_world_aabb(left_wheel, elem="elem_left_valve_stem")
        open_shelf_aabb = ctx.part_element_world_aabb(secondary_shelf, elem="elem_shelf_panel")

    if closed_valve_aabb is None or open_valve_aabb is None:
        ctx.fail("left wheel valve stem probe available", "missing valve stem bounds")
    else:
        closed_valve_center_z = 0.5 * (
            closed_valve_aabb[0][2] + closed_valve_aabb[1][2]
        )
        open_valve_center_z = 0.5 * (open_valve_aabb[0][2] + open_valve_aabb[1][2])
        ctx.check(
            "left wheel rotation moves the valve stem around the axle",
            open_valve_center_z > closed_valve_center_z + 0.090,
            details=(
                f"closed_center_z={closed_valve_center_z:.4f}, "
                f"open_center_z={open_valve_center_z:.4f}"
            ),
        )

    if closed_shelf_aabb is None or open_shelf_aabb is None:
        ctx.fail("secondary shelf panel probe available", "missing shelf panel bounds")
    else:
        closed_panel_max_x = closed_shelf_aabb[1][0]
        open_panel_max_x = open_shelf_aabb[1][0]
        ctx.check(
            "secondary shelf swings forward from its stowed position",
            open_panel_max_x > closed_panel_max_x + 0.055,
            details=(
                f"closed_max_x={closed_panel_max_x:.4f}, "
                f"open_max_x={open_panel_max_x:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
