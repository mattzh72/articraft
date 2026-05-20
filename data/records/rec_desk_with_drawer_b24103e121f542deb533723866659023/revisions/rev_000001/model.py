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
    model = ArticulatedObject(name="executive_double_pedestal_desk")

    walnut = model.material("dark_walnut", color=(0.24, 0.12, 0.055, 1.0))
    endgrain = model.material("walnut_endgrain", color=(0.18, 0.085, 0.035, 1.0))
    leather = model.material("green_leather_inset", color=(0.035, 0.13, 0.07, 1.0))
    brass = model.material("aged_brass", color=(0.80, 0.57, 0.24, 1.0))
    steel = model.material("brushed_steel", color=(0.62, 0.64, 0.62, 1.0))
    shadow = model.material("black_shadow", color=(0.015, 0.013, 0.011, 1.0))
    folder_mat = model.material("manila_folders", color=(0.78, 0.63, 0.38, 1.0))

    frame = model.part("desk_frame")

    # Overall executive-desk proportions, in metres.
    desk_w = 1.65
    desk_d = 0.78
    top_th = 0.060
    desk_h = 0.760
    top_z = desk_h - top_th / 2.0

    ped_w = 0.43
    ped_d = 0.70
    ped_h = 0.68
    ped_z = 0.02 + ped_h / 2.0
    side_t = 0.025
    front_y = -0.36
    back_y = 0.34
    left_x = -0.56
    right_x = 0.56

    # Broad writing surface with a leather blotter inset.
    frame.visual(
        Box((desk_w + 0.07, desk_d + 0.04, top_th)),
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        material=walnut,
        name="broad_top",
    )
    frame.visual(
        Box((0.92, 0.48, 0.005)),
        origin=Origin(xyz=(0.0, -0.03, desk_h + 0.001)),
        material=leather,
        name="leather_inset",
    )
    frame.visual(
        Box((desk_w + 0.02, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, front_y - 0.032, desk_h - 0.045)),
        material=endgrain,
        name="front_top_edge",
    )

    def add_static_pedestal(prefix: str, cx: float) -> None:
        # Carcase: separate panels instead of solid blocks so the drawer cavities
        # read as real wooden casework.
        for side_name, sx in (("outer_side", cx - ped_w / 2.0 + side_t / 2.0), ("inner_side", cx + ped_w / 2.0 - side_t / 2.0)):
            frame.visual(
                Box((side_t, ped_d, ped_h)),
                origin=Origin(xyz=(sx, 0.0, ped_z)),
                material=walnut,
                name=f"{prefix}_{side_name}",
            )
        frame.visual(
            Box((ped_w, side_t, ped_h)),
            origin=Origin(xyz=(cx, back_y - side_t / 2.0, ped_z)),
            material=walnut,
            name=f"{prefix}_back_panel",
        )
        frame.visual(
            Box((ped_w, ped_d, 0.035)),
            origin=Origin(xyz=(cx, 0.0, 0.0175)),
            material=walnut,
            name=f"{prefix}_bottom_panel",
        )
        frame.visual(
            Box((ped_w - 0.035, 0.045, 0.080)),
            origin=Origin(xyz=(cx, front_y + 0.020, 0.040)),
            material=shadow,
            name=f"{prefix}_toe_recess",
        )

        # Front rails and dividers, leaving three distinct drawer openings.
        for idx, z in enumerate((0.688, 0.525, 0.325, 0.084)):
            h = 0.024 if idx != 3 else 0.030
            frame.visual(
                Box((ped_w, 0.028, h)),
                origin=Origin(xyz=(cx, front_y + 0.006, z)),
                material=endgrain,
                name=f"{prefix}_front_rail_{idx}",
            )

        # Individual fixed guide rails for the six drawers, two per drawer.
        for drawer_index, (z, drawer_height) in enumerate(((0.610, 0.150), (0.425, 0.180), (0.205, 0.230))):
            rail_z = z - drawer_height / 2.0 + 0.065
            for side_label, sign in (("neg", -1.0), ("pos", 1.0)):
                rail_x = cx + sign * (ped_w / 2.0 - side_t - 0.006)
                frame.visual(
                    Box((0.012, 0.560, 0.018)),
                    origin=Origin(xyz=(rail_x, -0.035, rail_z)),
                    material=steel,
                    name=f"{prefix}_{drawer_index}_guide_{side_label}",
                )

        # Raised side panels give the pedestal mass a furniture-like silhouette.
        for side_label, sx in (("outer_panel", cx - ped_w / 2.0 - 0.003), ("inner_panel", cx + ped_w / 2.0 + 0.003)):
            frame.visual(
                Box((0.010, 0.50, 0.40)),
                origin=Origin(xyz=(sx, 0.005, 0.385)),
                material=endgrain,
                name=f"{prefix}_{side_label}",
            )

    add_static_pedestal("left", left_x)
    add_static_pedestal("right", right_x)

    # Rear modesty panel spanning the kneehole, attached to the top and both
    # pedestals.
    frame.visual(
        Box((0.66, 0.026, 0.42)),
        origin=Origin(xyz=(0.0, back_y - side_t / 2.0, 0.49)),
        material=walnut,
        name="rear_modesty_panel",
    )
    frame.visual(
        Box((0.66, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, front_y - 0.005, 0.665)),
        material=endgrain,
        name="knee_front_apron",
    )

    drawer_front_y = -0.405
    drawer_specs = (
        (0, 0.610, 0.150, 0.30),
        (1, 0.425, 0.180, 0.32),
        (2, 0.205, 0.230, 0.36),
    )

    def add_drawer(prefix: str, cx: float, index: int, z: float, height: float, travel: float, *, file_tray: bool = False):
        drawer = model.part(f"{prefix}_drawer_{index}")
        front_w = 0.370
        tray_w = 0.320
        tray_depth = 0.560
        side_wall_t = 0.015
        tray_h = min(height - 0.040, 0.180)
        tray_z = -height / 2.0 + 0.020 + tray_h / 2.0

        if not file_tray:
            drawer.visual(
                Box((front_w, 0.025, height)),
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=walnut,
                name="front_panel",
            )
            # Recessed field and raised perimeter make each drawer front legible.
            drawer.visual(
                Box((front_w - 0.060, 0.006, height - 0.055)),
                origin=Origin(xyz=(0.0, -0.015, 0.0)),
                material=endgrain,
                name="front_recess",
            )
            for rail_name, rz in (("top_trim", height / 2.0 - 0.018), ("bottom_trim", -height / 2.0 + 0.018)):
                drawer.visual(
                    Box((front_w - 0.035, 0.012, 0.018)),
                    origin=Origin(xyz=(0.0, -0.018, rz)),
                    material=endgrain,
                    name=rail_name,
                )
            for rail_name, rx in (("side_trim_0", -front_w / 2.0 + 0.018), ("side_trim_1", front_w / 2.0 - 0.018)):
                drawer.visual(
                    Box((0.018, 0.012, height - 0.035)),
                    origin=Origin(xyz=(rx, -0.018, 0.0)),
                    material=endgrain,
                    name=rail_name,
                )
            drawer.visual(
                Box((0.160, 0.018, 0.018)),
                origin=Origin(xyz=(0.0, -0.036, 0.0)),
                material=brass,
                name="handle_bar",
            )
            for post_name, px in (("handle_post_0", -0.070), ("handle_post_1", 0.070)):
                drawer.visual(
                    Box((0.014, 0.025, 0.014)),
                    origin=Origin(xyz=(px, -0.023, 0.0)),
                    material=brass,
                    name=post_name,
                )

        # Drawer tray behind the face (or behind the hinged filing front).
        for side_name, sx in (("tray_side_0", -tray_w / 2.0 + side_wall_t / 2.0), ("tray_side_1", tray_w / 2.0 - side_wall_t / 2.0)):
            drawer.visual(
                Box((side_wall_t, tray_depth, tray_h)),
                origin=Origin(xyz=(sx, tray_depth / 2.0 + 0.010, tray_z)),
                material=walnut,
                name=side_name,
            )
        drawer.visual(
            Box((tray_w, 0.015, tray_h)),
            origin=Origin(xyz=(0.0, tray_depth + 0.010, tray_z)),
            material=walnut,
            name="tray_back",
        )
        drawer.visual(
            Box((tray_w, tray_depth, 0.014)),
            origin=Origin(xyz=(0.0, tray_depth / 2.0 + 0.010, -height / 2.0 + 0.027)),
            material=walnut,
            name="tray_bottom",
        )

        # Moving rail members attached to the tray sides.
        for rail_name, sx in (("side_rail_0", -0.168), ("side_rail_1", 0.168)):
            drawer.visual(
                Box((0.020, 0.540, 0.016)),
                origin=Origin(xyz=(sx, tray_depth / 2.0 + 0.010, -height / 2.0 + 0.065)),
                material=steel,
                name=rail_name,
            )

        if file_tray:
            # Hanging-file hardware and sample folders visible when the drawer
            # and drop front are opened.
            for rail_name, sx in (("file_rail_0", -tray_w / 2.0 + 0.016), ("file_rail_1", tray_w / 2.0 - 0.016)):
                drawer.visual(
                    Box((0.012, 0.465, 0.014)),
                    origin=Origin(xyz=(sx, 0.305, height / 2.0 - 0.045)),
                    material=steel,
                    name=rail_name,
                )
            for folder_index, fy in enumerate((0.170, 0.225, 0.280, 0.335)):
                drawer.visual(
                    Box((0.250, 0.006, 0.155)),
                    origin=Origin(xyz=(0.0, fy, -0.016 + 0.004 * (folder_index % 2))),
                    material=folder_mat,
                    name=f"file_folder_{folder_index}",
                )
            for bracket_name, sx in (("lock_bracket_0", -0.160), ("lock_bracket_1", 0.160)):
                drawer.visual(
                    Box((0.022, 0.025, 0.080)),
                    origin=Origin(xyz=(sx, 0.410, 0.055)),
                    material=steel,
                    name=bracket_name,
                )

        joint = model.articulation(
            f"{prefix}_drawer_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=drawer,
            origin=Origin(xyz=(cx, drawer_front_y, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=travel),
        )
        return drawer, joint

    drawers = {}
    for prefix, cx in (("left", left_x), ("right", right_x)):
        for index, z, height, travel in drawer_specs:
            drawer, joint = add_drawer(prefix, cx, index, z, height, travel, file_tray=(prefix == "right" and index == 2))
            drawers[(prefix, index)] = drawer

    # The lower right filing drawer has a hinged drop front.
    file_drawer = drawers[("right", 2)]
    file_front = model.part("file_front")
    file_front_h = 0.230
    file_front_w = 0.370
    file_front.visual(
        Box((file_front_w, 0.025, file_front_h)),
        origin=Origin(xyz=(0.0, 0.0, file_front_h / 2.0 - 0.020)),
        material=walnut,
        name="front_panel",
    )
    file_front.visual(
        Box((file_front_w - 0.060, 0.006, file_front_h - 0.055)),
        origin=Origin(xyz=(0.0, -0.015, file_front_h / 2.0 - 0.020)),
        material=endgrain,
        name="front_recess",
    )
    file_front.visual(
        Cylinder(radius=0.012, length=file_front_w - 0.050),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="bottom_hinge_barrel",
    )
    file_front.visual(
        Box((0.170, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.036, 0.085)),
        material=brass,
        name="handle_bar",
    )
    for post_name, px in (("handle_post_0", -0.075), ("handle_post_1", 0.075)):
        file_front.visual(
            Box((0.014, 0.025, 0.014)),
            origin=Origin(xyz=(px, -0.023, 0.085)),
            material=brass,
            name=post_name,
        )
    model.articulation(
        "file_front_hinge",
        ArticulationType.REVOLUTE,
        parent=file_drawer,
        child=file_front,
        origin=Origin(xyz=(0.0, 0.0, -file_front_h / 2.0 + 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.10),
    )

    # Revolute lock bar for hanging files, nested inside the filing drawer.
    lock_bar = model.part("lock_bar")
    lock_bar.visual(
        Box((0.285, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.205, 0.0)),
        material=brass,
        name="cross_bar",
    )
    for arm_name, sx in (("side_arm_0", -0.135), ("side_arm_1", 0.135)):
        lock_bar.visual(
            Box((0.014, 0.210, 0.014)),
            origin=Origin(xyz=(sx, -0.105, 0.0)),
            material=brass,
            name=arm_name,
        )
        lock_bar.visual(
            Box((0.028, 0.018, 0.018)),
            origin=Origin(xyz=(sx, 0.0, 0.0)),
            material=brass,
            name=f"pivot_{0 if sx < 0 else 1}",
        )
    model.articulation(
        "lock_bar_hinge",
        ArticulationType.REVOLUTE,
        parent=file_drawer,
        child=lock_bar,
        origin=Origin(xyz=(0.0, 0.410, file_front_h / 2.0 - 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("desk_frame")
    slide_names = [f"{side}_drawer_{idx}_slide" for side in ("left", "right") for idx in range(3)]
    ctx.check("six independent drawer slides", len(slide_names) == 6)
    for name in slide_names:
        joint = object_model.get_articulation(name)
        limits = joint.motion_limits
        ctx.check(
            f"{name} is a prismatic guide",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, -1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper >= 0.30,
        )

        drawer = object_model.get_part(name.replace("_slide", ""))
        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: limits.upper if limits is not None and limits.upper is not None else 0.0}):
            moved_pos = ctx.part_world_position(drawer)
            # The moving rail must remain engaged with its fixed guide at full extension.
            ped_side, idx_text = name.split("_drawer_")
            idx = int(idx_text.split("_")[0])
            ctx.expect_overlap(
                drawer,
                frame,
                axes="y",
                min_overlap=0.08,
                elem_a="side_rail_1",
                elem_b=f"{ped_side}_{idx}_guide_pos",
                name=f"{name} retains guide overlap",
            )
        ctx.check(
            f"{name} opens toward seated user",
            rest_pos is not None and moved_pos is not None and moved_pos[1] < rest_pos[1] - 0.25,
            details=f"rest={rest_pos}, moved={moved_pos}",
        )

    file_hinge = object_model.get_articulation("file_front_hinge")
    lock_hinge = object_model.get_articulation("lock_bar_hinge")
    ctx.check("filing front is hinged", file_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("file lock bar is revolute", lock_hinge.articulation_type == ArticulationType.REVOLUTE)

    file_front = object_model.get_part("file_front")
    lock_bar = object_model.get_part("lock_bar")
    closed_front = ctx.part_world_aabb(file_front)
    with ctx.pose({file_hinge: 1.10}):
        dropped_front = ctx.part_world_aabb(file_front)
    ctx.check(
        "filing front drops outward",
        closed_front is not None
        and dropped_front is not None
        and dropped_front[0][1] < closed_front[0][1] - 0.08,
        details=f"closed={closed_front}, dropped={dropped_front}",
    )

    closed_bar = ctx.part_world_aabb(lock_bar)
    with ctx.pose({lock_hinge: 1.25}):
        raised_bar = ctx.part_world_aabb(lock_bar)
    ctx.check(
        "lock bar lifts above hanging files",
        closed_bar is not None
        and raised_bar is not None
        and raised_bar[1][2] > closed_bar[1][2] + 0.10,
        details=f"closed={closed_bar}, raised={raised_bar}",
    )

    return ctx.report()


object_model = build_object_model()
