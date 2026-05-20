from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_l_desk")

    walnut = Material("warm_walnut", rgba=(0.55, 0.34, 0.16, 1.0))
    dark_walnut = Material("dark_edge_banding", rgba=(0.32, 0.18, 0.08, 1.0))
    drawer_wood = Material("drawer_walnut", rgba=(0.48, 0.29, 0.14, 1.0))
    inner_shadow = Material("dark_drawer_interior", rgba=(0.08, 0.07, 0.06, 1.0))
    satin_black = Material("satin_black_metal", rgba=(0.015, 0.016, 0.018, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.62, 0.63, 0.60, 1.0))

    frame = model.part("desk_frame")

    # Two rectangular worktop slabs form the L.  The return slab kisses the back
    # edge of the main slab over the shared corner run instead of being fused
    # into a single abstract plate.
    frame.visual(
        Box((1.60, 0.65, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.7275)),
        material=walnut,
        name="main_top",
    )
    frame.visual(
        Box((0.65, 1.25, 0.045)),
        origin=Origin(xyz=(0.475, 0.95, 0.7275)),
        material=walnut,
        name="return_top",
    )

    # Dark edge banding makes the two separate rectangular work surfaces read as
    # finished desk boards rather than plain blocks.
    frame.visual(Box((1.60, 0.014, 0.050)), origin=Origin(xyz=(0.0, -0.332, 0.7275)), material=dark_walnut, name="main_front_band")
    frame.visual(Box((1.60, 0.014, 0.050)), origin=Origin(xyz=(0.0, 0.332, 0.7275)), material=dark_walnut, name="main_back_band")
    frame.visual(Box((0.014, 0.65, 0.050)), origin=Origin(xyz=(-0.807, 0.0, 0.7275)), material=dark_walnut, name="main_left_band")
    frame.visual(Box((0.014, 0.65, 0.050)), origin=Origin(xyz=(0.807, 0.0, 0.7275)), material=dark_walnut, name="main_right_band")
    frame.visual(Box((0.65, 0.014, 0.050)), origin=Origin(xyz=(0.475, 1.582, 0.7275)), material=dark_walnut, name="return_end_band")
    frame.visual(Box((0.014, 1.25, 0.050)), origin=Origin(xyz=(0.143, 0.95, 0.7275)), material=dark_walnut, name="return_inner_band")
    frame.visual(Box((0.014, 1.25, 0.050)), origin=Origin(xyz=(0.807, 0.95, 0.7275)), material=dark_walnut, name="return_outer_band")

    # 45-degree metal plate under the inside corner: a visible mitred bracket
    # tying the two rectangular top surfaces together.
    frame.visual(
        Box((0.42, 0.070, 0.018)),
        origin=Origin(xyz=(0.275, 0.505, 0.696), rpy=(0.0, 0.0, -math.pi / 4.0)),
        material=brushed_steel,
        name="miter_bracket",
    )

    # Simple black steel support legs and aprons.
    for idx, (x, y) in enumerate(((-0.72, -0.265), (-0.72, 0.265), (0.205, 1.48), (0.745, 1.48))):
        frame.visual(
            Box((0.045, 0.045, 0.705)),
            origin=Origin(xyz=(x, y, 0.3525)),
            material=satin_black,
            name=f"leg_{idx}",
        )
    frame.visual(Box((1.05, 0.030, 0.055)), origin=Origin(xyz=(-0.245, 0.292, 0.675)), material=satin_black, name="main_rear_apron")
    frame.visual(Box((0.055, 1.00, 0.055)), origin=Origin(xyz=(0.185, 1.02, 0.675)), material=satin_black, name="return_inner_apron")
    frame.visual(Box((0.055, 1.15, 0.055)), origin=Origin(xyz=(0.765, 0.96, 0.675)), material=satin_black, name="return_outer_apron")

    # Right pedestal carcass.  The front is open except for thin rails between
    # the three drawer bays, so the separate sliding drawers are visible as real
    # moving boxes instead of painted lines.
    x_min, x_max = 0.330, 0.780
    y_front, y_back = -0.325, 0.255
    ped_x = (x_min + x_max) / 2.0
    ped_y = (y_front + y_back) / 2.0
    ped_h = 0.665
    ped_z = 0.040 + ped_h / 2.0
    frame.visual(Box((0.025, 0.580, ped_h)), origin=Origin(xyz=(x_min + 0.0125, ped_y, ped_z)), material=walnut, name="pedestal_side_0")
    frame.visual(Box((0.025, 0.580, ped_h)), origin=Origin(xyz=(x_max - 0.0125, ped_y, ped_z)), material=walnut, name="pedestal_side_1")
    frame.visual(Box((0.450, 0.025, ped_h)), origin=Origin(xyz=(ped_x, y_back - 0.0125, ped_z)), material=walnut, name="pedestal_back")
    frame.visual(Box((0.450, 0.580, 0.025)), origin=Origin(xyz=(ped_x, ped_y, 0.0525)), material=walnut, name="pedestal_bottom")
    frame.visual(Box((0.450, 0.580, 0.025)), origin=Origin(xyz=(ped_x, ped_y, 0.6925)), material=walnut, name="pedestal_top")
    for idx, z in enumerate((0.052, 0.267, 0.482, 0.692)):
        frame.visual(
            Box((0.450, 0.012, 0.018)),
            origin=Origin(xyz=(ped_x, y_front - 0.006, z)),
            material=dark_walnut,
            name=f"drawer_rail_{idx}",
        )

    drawer_centers = (0.160, 0.375, 0.590)
    for idx, z in enumerate(drawer_centers):
        frame.visual(Box((0.016, 0.460, 0.018)), origin=Origin(xyz=(0.370, -0.055, z - 0.045)), material=brushed_steel, name=f"drawer_{idx}_left_rail")
        frame.visual(Box((0.016, 0.460, 0.018)), origin=Origin(xyz=(0.740, -0.055, z - 0.045)), material=brushed_steel, name=f"drawer_{idx}_right_rail")
        frame.visual(Box((0.016, 0.460, 0.018)), origin=Origin(xyz=(0.358, -0.055, z - 0.045)), material=brushed_steel, name=f"drawer_{idx}_left_mount")
        frame.visual(Box((0.016, 0.460, 0.018)), origin=Origin(xyz=(0.752, -0.055, z - 0.045)), material=brushed_steel, name=f"drawer_{idx}_right_mount")

    # Fixed keyboard-tray rails hang from the underside of the main top.
    for idx, x in enumerate((-0.698, -0.002)):
        rail_x = (-0.695, -0.005)[idx]
        frame.visual(Box((0.020, 0.380, 0.018)), origin=Origin(xyz=(rail_x, -0.055, 0.672)), material=brushed_steel, name=f"tray_rail_{idx}")
        for h_idx, y in enumerate((-0.185, 0.075)):
            frame.visual(
                Box((0.040, 0.024, 0.024)),
                origin=Origin(xyz=(x, y, 0.693)),
                material=brushed_steel,
                name=f"tray_hanger_{idx}_{h_idx}",
            )

    for idx, z in enumerate(drawer_centers):
        drawer = model.part(f"drawer_{idx}")
        drawer.visual(Box((0.320, 0.480, 0.014)), origin=Origin(xyz=(0.0, 0.0, -0.073)), material=inner_shadow, name="bottom")
        drawer.visual(Box((0.014, 0.480, 0.145)), origin=Origin(xyz=(-0.160, 0.0, -0.005)), material=drawer_wood, name="side_0")
        drawer.visual(Box((0.014, 0.480, 0.145)), origin=Origin(xyz=(0.160, 0.0, -0.005)), material=drawer_wood, name="side_1")
        drawer.visual(Box((0.320, 0.014, 0.145)), origin=Origin(xyz=(0.0, 0.233, -0.005)), material=drawer_wood, name="back")
        drawer.visual(Box((0.390, 0.026, 0.185)), origin=Origin(xyz=(0.0, -0.253, 0.0)), material=drawer_wood, name="front")
        drawer.visual(Box((0.010, 0.480, 0.020)), origin=Origin(xyz=(-0.172, 0.0, -0.045)), material=brushed_steel, name="left_slide")
        drawer.visual(Box((0.010, 0.480, 0.020)), origin=Origin(xyz=(0.172, 0.0, -0.045)), material=brushed_steel, name="right_slide")
        drawer.visual(Cylinder(radius=0.006, length=0.042), origin=Origin(xyz=(-0.070, -0.284, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="handle_post_0")
        drawer.visual(Cylinder(radius=0.006, length=0.042), origin=Origin(xyz=(0.070, -0.284, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="handle_post_1")
        drawer.visual(Cylinder(radius=0.007, length=0.170), origin=Origin(xyz=(0.0, -0.303, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)), material=brushed_steel, name="handle_bar")
        model.articulation(
            f"frame_to_drawer_{idx}",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=drawer,
            origin=Origin(xyz=(ped_x, -0.060, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.380),
        )

    tray = model.part("keyboard_tray")
    tray.visual(Box((0.650, 0.282, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark_walnut, name="tray_board")
    tray.visual(Box((0.660, 0.025, 0.055)), origin=Origin(xyz=(0.0, -0.153, 0.005)), material=walnut, name="front_lip")
    tray.visual(Box((0.020, 0.340, 0.060)), origin=Origin(xyz=(-0.325, 0.0, 0.020)), material=brushed_steel, name="side_flange_0")
    tray.visual(Box((0.020, 0.340, 0.060)), origin=Origin(xyz=(0.325, 0.0, 0.020)), material=brushed_steel, name="side_flange_1")
    model.articulation(
        "frame_to_keyboard_tray",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(-0.350, -0.055, 0.620)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.320),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("desk_frame")

    for idx in range(3):
        drawer = object_model.get_part(f"drawer_{idx}")
        joint = object_model.get_articulation(f"frame_to_drawer_{idx}")
        rest_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            frame,
            axes="y",
            min_overlap=0.25,
            elem_a="left_slide",
            elem_b=f"drawer_{idx}_left_rail",
            name=f"drawer {idx} rail engaged closed",
        )
        ctx.expect_gap(
            drawer,
            frame,
            axis="x",
            min_gap=0.0,
            max_gap=0.002,
            positive_elem="left_slide",
            negative_elem=f"drawer_{idx}_left_rail",
            name=f"drawer {idx} left slide clearance",
        )
        with ctx.pose({joint: 0.380}):
            ctx.expect_overlap(
                drawer,
                frame,
                axes="y",
                min_overlap=0.07,
                elem_a="left_slide",
                elem_b=f"drawer_{idx}_left_rail",
                name=f"drawer {idx} rail retained extended",
            )
            extended_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"drawer {idx} slides toward user",
            rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.30,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    tray = object_model.get_part("keyboard_tray")
    tray_joint = object_model.get_articulation("frame_to_keyboard_tray")
    tray_rest = ctx.part_world_position(tray)
    ctx.expect_overlap(
        tray,
        frame,
        axes="y",
        min_overlap=0.25,
        elem_a="side_flange_0",
        elem_b="tray_rail_0",
        name="keyboard tray rail engaged closed",
    )
    with ctx.pose({tray_joint: 0.320}):
        ctx.expect_overlap(
            tray,
            frame,
            axes="y",
            min_overlap=0.035,
            elem_a="side_flange_0",
            elem_b="tray_rail_0",
            name="keyboard tray rail retained extended",
        )
        tray_extended = ctx.part_world_position(tray)
    ctx.check(
        "keyboard tray slides toward user",
        tray_rest is not None and tray_extended is not None and tray_extended[1] < tray_rest[1] - 0.25,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    return ctx.report()


object_model = build_object_model()
