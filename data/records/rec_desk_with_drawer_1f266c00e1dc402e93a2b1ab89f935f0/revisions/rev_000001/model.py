from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="laboratory_workbench_desk")

    desk_depth = 0.78
    desk_width = 1.60
    desk_height = 0.92
    top_thickness = 0.06
    top_underside_z = desk_height - top_thickness

    pedestal_y_center = 0.59
    pedestal_width = 0.42
    drawer_joint_x = 0.26
    drawer_y_center = pedestal_y_center
    keyboard_joint_y = -0.18
    keyboard_joint_z = 0.80

    enamel_white = model.material("enamel_white", rgba=(0.82, 0.84, 0.86, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.58, 0.61, 0.65, 1.0))
    laminate_gray = model.material("laminate_gray", rgba=(0.43, 0.45, 0.46, 1.0))
    graphite = model.material("graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    satin_chrome = model.material("satin_chrome", rgba=(0.75, 0.77, 0.80, 1.0))

    frame = model.part("desk_frame")
    frame.visual(
        Box((desk_depth, desk_width, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, desk_height - top_thickness / 2.0)),
        material=laminate_gray,
        name="top_slab",
    )
    frame.visual(
        Box((0.72, 0.05, top_underside_z)),
        origin=Origin(xyz=(-0.01, -0.765, top_underside_z / 2.0)),
        material=enamel_white,
        name="left_end_panel",
    )
    frame.visual(
        Box((0.025, 1.14, 0.56)),
        origin=Origin(xyz=(-0.357, -0.175, 0.39)),
        material=enamel_white,
        name="back_modesty_panel",
    )
    frame.visual(
        Box((0.05, 1.14, 0.08)),
        origin=Origin(xyz=(0.27, -0.175, 0.82)),
        material=enamel_white,
        name="front_stretcher",
    )
    frame.visual(
        Box((0.72, 0.018, top_underside_z)),
        origin=Origin(xyz=(-0.01, 0.389, top_underside_z / 2.0)),
        material=enamel_white,
        name="pedestal_inner_side",
    )
    frame.visual(
        Box((0.72, 0.018, top_underside_z)),
        origin=Origin(xyz=(-0.01, 0.791, top_underside_z / 2.0)),
        material=enamel_white,
        name="pedestal_outer_side",
    )
    frame.visual(
        Box((0.018, pedestal_width, top_underside_z)),
        origin=Origin(xyz=(-0.351, pedestal_y_center, top_underside_z / 2.0)),
        material=enamel_white,
        name="pedestal_back_panel",
    )
    frame.visual(
        Box((0.66, 0.402, 0.02)),
        origin=Origin(xyz=(-0.03, pedestal_y_center, 0.01)),
        material=enamel_white,
        name="pedestal_bottom",
    )
    frame.visual(
        Box((0.66, 0.402, 0.02)),
        origin=Origin(xyz=(-0.03, pedestal_y_center, 0.85)),
        material=enamel_white,
        name="pedestal_top_stretcher",
    )
    frame.visual(
        Box((0.09, 0.402, 0.12)),
        origin=Origin(xyz=(0.255, pedestal_y_center, 0.06)),
        material=enamel_white,
        name="pedestal_toe_kick",
    )
    frame.visual(
        Box((0.04, 0.402, 0.022)),
        origin=Origin(xyz=(0.28, pedestal_y_center, 0.79)),
        material=enamel_white,
        name="pedestal_top_face_rail",
    )
    frame.visual(
        Box((0.04, 0.402, 0.018)),
        origin=Origin(xyz=(0.28, pedestal_y_center, 0.61)),
        material=enamel_white,
        name="pedestal_upper_divider",
    )
    frame.visual(
        Box((0.04, 0.402, 0.018)),
        origin=Origin(xyz=(0.28, pedestal_y_center, 0.39)),
        material=enamel_white,
        name="pedestal_lower_divider",
    )

    drawer_rail_specs = (
        ("top_drawer", 0.66),
        ("middle_drawer", 0.46),
        ("bottom_drawer", 0.23),
    )
    for drawer_prefix, rail_z in drawer_rail_specs:
        frame.visual(
            Box((0.54, 0.028, 0.018)),
            origin=Origin(xyz=(-0.05, 0.412, rail_z)),
            material=steel_gray,
            name=f"{drawer_prefix}_outer_left_rail",
        )
        frame.visual(
            Box((0.54, 0.028, 0.018)),
            origin=Origin(xyz=(-0.05, 0.768, rail_z)),
            material=steel_gray,
            name=f"{drawer_prefix}_outer_right_rail",
        )

    frame.visual(
        Box((0.52, 0.03, 0.04)),
        origin=Origin(xyz=(-0.05, -0.481, 0.84)),
        material=steel_gray,
        name="keyboard_left_runner",
    )
    frame.visual(
        Box((0.52, 0.03, 0.04)),
        origin=Origin(xyz=(-0.05, 0.121, 0.84)),
        material=steel_gray,
        name="keyboard_right_runner",
    )
    frame.inertial = Inertial.from_geometry(
        Box((desk_depth, desk_width, desk_height)),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, desk_height / 2.0)),
    )

    drawer_specs = (
        ("top_drawer", 0.70, 0.15, 0.10),
        ("middle_drawer", 0.50, 0.19, 0.14),
        ("bottom_drawer", 0.27, 0.23, 0.18),
    )
    for drawer_name, drawer_center_z, front_height, box_height in drawer_specs:
        drawer = model.part(drawer_name)
        drawer.visual(
            Box((0.022, 0.39, front_height)),
            origin=Origin(xyz=(0.095, 0.0, 0.0)),
            material=graphite,
            name="front_panel",
        )
        drawer.visual(
            Box((0.528, 0.012, box_height)),
            origin=Origin(xyz=(-0.18, -0.154, -0.01)),
            material=enamel_white,
            name="left_side",
        )
        drawer.visual(
            Box((0.528, 0.012, box_height)),
            origin=Origin(xyz=(-0.18, 0.154, -0.01)),
            material=enamel_white,
            name="right_side",
        )
        drawer.visual(
            Box((0.012, 0.296, box_height)),
            origin=Origin(xyz=(-0.438, 0.0, -0.01)),
            material=enamel_white,
            name="drawer_back",
        )
        drawer.visual(
            Box((0.516, 0.296, 0.012)),
            origin=Origin(xyz=(-0.18, 0.0, -0.01 - box_height / 2.0 + 0.006)),
            material=enamel_white,
            name="drawer_bottom",
        )
        drawer.visual(
            Box((0.44, 0.012, 0.012)),
            origin=Origin(xyz=(-0.20, -0.165, -0.056)),
            material=steel_gray,
            name="inner_left_rail",
        )
        drawer.visual(
            Box((0.44, 0.012, 0.012)),
            origin=Origin(xyz=(-0.20, 0.165, -0.056)),
            material=steel_gray,
            name="inner_right_rail",
        )
        drawer.visual(
            Box((0.02, 0.16, 0.012)),
            origin=Origin(xyz=(0.116, 0.0, 0.0)),
            material=satin_chrome,
            name="pull_bar",
        )
        drawer.visual(
            Cylinder(radius=0.01, length=0.02),
            origin=Origin(xyz=(0.116, 0.14, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_chrome,
            name="lock_cylinder",
        )
        drawer.inertial = Inertial.from_geometry(
            Box((0.55, 0.39, front_height)),
            mass=8.0 if drawer_name == "bottom_drawer" else 6.0,
            origin=Origin(xyz=(-0.14, 0.0, -0.01)),
        )
        model.articulation(
            f"desk_frame_to_{drawer_name}",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=drawer,
            origin=Origin(xyz=(drawer_joint_x, drawer_y_center, drawer_center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=140.0,
                velocity=0.35,
                lower=0.0,
                upper=0.24,
            ),
        )

    keyboard_shelf = model.part("keyboard_shelf")
    keyboard_shelf.visual(
        Box((0.50, 0.62, 0.02)),
        origin=Origin(xyz=(-0.10, 0.0, -0.055)),
        material=laminate_gray,
        name="shelf_board",
    )
    keyboard_shelf.visual(
        Box((0.022, 0.62, 0.06)),
        origin=Origin(xyz=(0.106, 0.0, -0.02)),
        material=graphite,
        name="shelf_front",
    )
    keyboard_shelf.visual(
        Box((0.46, 0.012, 0.048)),
        origin=Origin(xyz=(-0.10, -0.304, -0.048)),
        material=graphite,
        name="left_tray_side",
    )
    keyboard_shelf.visual(
        Box((0.46, 0.012, 0.048)),
        origin=Origin(xyz=(-0.10, 0.304, -0.048)),
        material=graphite,
        name="right_tray_side",
    )
    keyboard_shelf.visual(
        Box((0.44, 0.012, 0.044)),
        origin=Origin(xyz=(-0.18, -0.301, -0.0025)),
        material=steel_gray,
        name="left_inner_runner",
    )
    keyboard_shelf.visual(
        Box((0.44, 0.012, 0.044)),
        origin=Origin(xyz=(-0.18, 0.301, -0.0025)),
        material=steel_gray,
        name="right_inner_runner",
    )
    keyboard_shelf.inertial = Inertial.from_geometry(
        Box((0.50, 0.62, 0.08)),
        mass=5.5,
        origin=Origin(xyz=(-0.08, 0.0, -0.03)),
    )
    model.articulation(
        "desk_frame_to_keyboard_shelf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=keyboard_shelf,
        origin=Origin(xyz=(0.20, keyboard_joint_y, keyboard_joint_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("desk_frame")
    keyboard_shelf = object_model.get_part("keyboard_shelf")
    keyboard_joint = object_model.get_articulation("desk_frame_to_keyboard_shelf")

    drawer_names = ("top_drawer", "middle_drawer", "bottom_drawer")
    rail_prefix = {
        "top_drawer": "top_drawer",
        "middle_drawer": "middle_drawer",
        "bottom_drawer": "bottom_drawer",
    }

    for drawer_name in drawer_names:
        drawer = object_model.get_part(drawer_name)
        joint = object_model.get_articulation(f"desk_frame_to_{drawer_name}")
        upper = joint.motion_limits.upper if joint.motion_limits is not None else 0.0

        ctx.expect_gap(
            drawer,
            frame,
            axis="x",
            positive_elem="drawer_back",
            negative_elem="pedestal_back_panel",
            min_gap=0.04,
            max_gap=0.20,
            name=f"{drawer_name} clears the pedestal back when closed",
        )
        ctx.expect_overlap(
            drawer,
            frame,
            axes="x",
            elem_a="inner_left_rail",
            elem_b=f"{rail_prefix[drawer_name]}_outer_left_rail",
            min_overlap=0.30,
            name=f"{drawer_name} left guide rail is well engaged when closed",
        )
        ctx.expect_overlap(
            drawer,
            frame,
            axes="x",
            elem_a="inner_right_rail",
            elem_b=f"{rail_prefix[drawer_name]}_outer_right_rail",
            min_overlap=0.30,
            name=f"{drawer_name} right guide rail is well engaged when closed",
        )

        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: upper}):
            ctx.expect_overlap(
                drawer,
                frame,
                axes="x",
                elem_a="inner_left_rail",
                elem_b=f"{rail_prefix[drawer_name]}_outer_left_rail",
                min_overlap=0.10,
                name=f"{drawer_name} left guide rail retains insertion when extended",
            )
            ctx.expect_overlap(
                drawer,
                frame,
                axes="x",
                elem_a="inner_right_rail",
                elem_b=f"{rail_prefix[drawer_name]}_outer_right_rail",
                min_overlap=0.10,
                name=f"{drawer_name} right guide rail retains insertion when extended",
            )
            extended_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"{drawer_name} extends outward",
            closed_pos is not None
            and extended_pos is not None
            and extended_pos[0] > closed_pos[0] + 0.18,
            details=f"closed={closed_pos}, extended={extended_pos}",
        )

    keyboard_closed = ctx.part_world_position(keyboard_shelf)
    ctx.expect_gap(
        frame,
        keyboard_shelf,
        axis="z",
        positive_elem="top_slab",
        negative_elem="shelf_front",
        min_gap=0.04,
        max_gap=0.12,
        name="keyboard shelf stores beneath the thick work surface",
    )
    ctx.expect_overlap(
        keyboard_shelf,
        frame,
        axes="x",
        elem_a="left_inner_runner",
        elem_b="keyboard_left_runner",
        min_overlap=0.30,
        name="keyboard shelf left runner is engaged when closed",
    )
    ctx.expect_overlap(
        keyboard_shelf,
        frame,
        axes="x",
        elem_a="right_inner_runner",
        elem_b="keyboard_right_runner",
        min_overlap=0.30,
        name="keyboard shelf right runner is engaged when closed",
    )
    with ctx.pose({keyboard_joint: 0.24}):
        ctx.expect_overlap(
            keyboard_shelf,
            frame,
            axes="x",
            elem_a="left_inner_runner",
            elem_b="keyboard_left_runner",
            min_overlap=0.10,
            name="keyboard shelf left runner retains insertion when extended",
        )
        ctx.expect_overlap(
            keyboard_shelf,
            frame,
            axes="x",
            elem_a="right_inner_runner",
            elem_b="keyboard_right_runner",
            min_overlap=0.10,
            name="keyboard shelf right runner retains insertion when extended",
        )
        keyboard_extended = ctx.part_world_position(keyboard_shelf)
    ctx.check(
        "keyboard shelf extends forward",
        keyboard_closed is not None
        and keyboard_extended is not None
        and keyboard_extended[0] > keyboard_closed[0] + 0.18,
        details=f"closed={keyboard_closed}, extended={keyboard_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
