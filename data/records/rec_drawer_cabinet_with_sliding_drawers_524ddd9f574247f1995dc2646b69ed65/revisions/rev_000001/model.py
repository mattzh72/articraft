from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="solid_wood_drawer_cabinet")

    walnut = model.material("warm_walnut", color=(0.46, 0.24, 0.11, 1.0))
    dark_walnut = model.material("dark_end_grain", color=(0.25, 0.12, 0.055, 1.0))
    birch = model.material("sealed_birch_ply", color=(0.74, 0.56, 0.34, 1.0))
    shadow = model.material("shadowed_reveal", color=(0.025, 0.022, 0.020, 1.0))
    metal = model.material("brushed_oil_rubbed_metal", color=(0.11, 0.10, 0.09, 1.0))
    slide_metal = model.material("galvanized_slide_steel", color=(0.58, 0.59, 0.56, 1.0))

    width = 0.94
    depth = 0.56
    height = 1.05
    panel_t = 0.025
    back_t = 0.018
    face_t = 0.022
    stile_w = 0.055
    rail_h = 0.022
    front_x = depth / 2.0 + face_t

    carcass = model.part("carcass")
    # Structural box: full-height side panels, top/bottom, and a set-in back panel.
    carcass.visual(
        Box((depth, panel_t, height)),
        origin=Origin(xyz=(0.0, width / 2.0 - panel_t / 2.0, height / 2.0)),
        material=walnut,
        name="side_panel_0",
    )
    carcass.visual(
        Box((depth, panel_t, height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + panel_t / 2.0, height / 2.0)),
        material=walnut,
        name="side_panel_1",
    )
    carcass.visual(
        Box((depth, width, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, height - panel_t / 2.0)),
        material=walnut,
        name="top_panel",
    )
    carcass.visual(
        Box((depth, width, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, panel_t / 2.0)),
        material=walnut,
        name="bottom_panel",
    )
    carcass.visual(
        Box((back_t, width - 0.010, height - 0.020)),
        origin=Origin(xyz=(-depth / 2.0 + back_t / 2.0, 0.0, height / 2.0)),
        material=dark_walnut,
        name="back_panel",
    )
    # Slightly recessed black toe-kick reads as the shadow under a heavy cabinet.
    carcass.visual(
        Box((0.080, width - 0.12, 0.050)),
        origin=Origin(xyz=(depth / 2.0 - 0.040, 0.0, 0.040)),
        material=shadow,
        name="toe_shadow",
    )

    # Proud face frame, built from stiles and rails just like real cabinetry.
    face_center_x = depth / 2.0 + face_t / 2.0
    carcass.visual(
        Box((face_t, stile_w, height)),
        origin=Origin(xyz=(face_center_x, width / 2.0 - stile_w / 2.0, height / 2.0)),
        material=dark_walnut,
        name="front_stile_0",
    )
    carcass.visual(
        Box((face_t, stile_w, height)),
        origin=Origin(xyz=(face_center_x, -width / 2.0 + stile_w / 2.0, height / 2.0)),
        material=dark_walnut,
        name="front_stile_1",
    )
    carcass.visual(
        Box((face_t, width, 0.050)),
        origin=Origin(xyz=(face_center_x, 0.0, height - 0.025)),
        material=dark_walnut,
        name="front_top_rail",
    )
    carcass.visual(
        Box((face_t, width, 0.060)),
        origin=Origin(xyz=(face_center_x, 0.0, 0.030)),
        material=dark_walnut,
        name="front_bottom_rail",
    )

    drawer_specs = [
        # name, center z, face height, travel
        ("drawer_0", 0.905, 0.170, 0.340),
        ("drawer_1", 0.710, 0.170, 0.340),
        ("drawer_2", 0.4775, 0.245, 0.350),
        ("drawer_3", 0.2075, 0.245, 0.350),
    ]
    gap_rail_z = [0.8075, 0.6125, 0.3425]
    for rail_i, z in enumerate(gap_rail_z):
        carcass.visual(
            Box((face_t, width, rail_h)),
            origin=Origin(xyz=(face_center_x, 0.0, z)),
            material=dark_walnut,
            name=f"front_mid_rail_{rail_i}",
        )
        carcass.visual(
            Box((depth - 0.050, width - 2.0 * panel_t + 0.006, 0.016)),
            origin=Origin(xyz=(-0.020, 0.0, z)),
            material=dark_walnut,
            name=f"dust_frame_{rail_i}",
        )

    drawer_front_w = 0.800
    tray_w = 0.800
    tray_depth = 0.500
    wall_t = 0.012
    drawer_bottom_t = 0.012
    # Drawer-mounted runners lightly bear on the fixed steel channels; this
    # gives every sliding drawer a real support path without interpenetration.
    runner_y = tray_w / 2.0 + 0.0145
    fixed_slide_y = width / 2.0 - panel_t - 0.007

    for idx, (drawer_name, z_center, face_h, travel) in enumerate(drawer_specs):
        # Fixed slide channels are part of the carcass and are let into the side panels.
        slide_z = z_center - face_h / 2.0 + 0.066
        for side, y_sign in enumerate((1.0, -1.0)):
            carcass.visual(
                Box((0.480, 0.018, 0.026)),
                origin=Origin(xyz=(0.015, y_sign * fixed_slide_y, slide_z)),
                material=slide_metal,
                name=f"fixed_slide_{side}_{idx}",
            )

        drawer = model.part(drawer_name)
        # Front slab with raised frame-and-panel detailing.
        drawer.visual(
            Box((0.035, drawer_front_w, face_h)),
            origin=Origin(xyz=(0.0175, 0.0, 0.0)),
            material=walnut,
            name="front_panel",
        )
        rail_face_z = face_h / 2.0 - 0.022
        drawer.visual(
            Box((0.008, drawer_front_w - 0.060, 0.024)),
            origin=Origin(xyz=(0.039, 0.0, rail_face_z)),
            material=dark_walnut,
            name="front_top_molding",
        )
        drawer.visual(
            Box((0.008, drawer_front_w - 0.060, 0.024)),
            origin=Origin(xyz=(0.039, 0.0, -rail_face_z)),
            material=dark_walnut,
            name="front_bottom_molding",
        )
        drawer.visual(
            Box((0.008, 0.026, face_h - 0.050)),
            origin=Origin(xyz=(0.039, drawer_front_w / 2.0 - 0.030, 0.0)),
            material=dark_walnut,
            name="front_side_molding_0",
        )
        drawer.visual(
            Box((0.008, 0.026, face_h - 0.050)),
            origin=Origin(xyz=(0.039, -drawer_front_w / 2.0 + 0.030, 0.0)),
            material=dark_walnut,
            name="front_side_molding_1",
        )
        drawer.visual(
            Box((0.006, drawer_front_w - 0.170, max(0.060, face_h - 0.110))),
            origin=Origin(xyz=(0.043, 0.0, 0.0)),
            material=walnut,
            name="raised_center_panel",
        )

        # A true drawer box extends into the cabinet behind the applied front.
        tray_h = face_h - 0.060
        tray_bottom_z = -face_h / 2.0 + 0.022
        drawer.visual(
            Box((tray_depth, tray_w, drawer_bottom_t)),
            origin=Origin(xyz=(-tray_depth / 2.0, 0.0, tray_bottom_z)),
            material=birch,
            name="tray_bottom",
        )
        side_z = tray_bottom_z + tray_h / 2.0
        drawer.visual(
            Box((tray_depth, wall_t, tray_h)),
            origin=Origin(xyz=(-tray_depth / 2.0, tray_w / 2.0 - wall_t / 2.0, side_z)),
            material=birch,
            name="tray_side_0",
        )
        drawer.visual(
            Box((tray_depth, wall_t, tray_h)),
            origin=Origin(xyz=(-tray_depth / 2.0, -tray_w / 2.0 + wall_t / 2.0, side_z)),
            material=birch,
            name="tray_side_1",
        )
        drawer.visual(
            Box((wall_t, tray_w, tray_h)),
            origin=Origin(xyz=(-tray_depth + wall_t / 2.0, 0.0, side_z)),
            material=birch,
            name="tray_back",
        )

        for side, y_sign in enumerate((1.0, -1.0)):
            drawer.visual(
                Box((0.420, 0.029, 0.022)),
                origin=Origin(xyz=(-0.245, y_sign * runner_y, tray_bottom_z + 0.044)),
                material=slide_metal,
                name=f"runner_{side}",
            )

        # Practical metal pull: a bar, two standoffs, mounting plates, and visible screw heads.
        handle_z = 0.0
        drawer.visual(
            Cylinder(radius=0.012, length=0.330),
            origin=Origin(xyz=(0.073, 0.0, handle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="pull_bar",
        )
        for post_i, y in enumerate((-0.135, 0.135)):
            drawer.visual(
                Cylinder(radius=0.007, length=0.048),
                origin=Origin(xyz=(0.053, y, handle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=metal,
                name=f"pull_post_{post_i}",
            )
            drawer.visual(
                Box((0.006, 0.046, 0.030)),
                origin=Origin(xyz=(0.039, y, handle_z)),
                material=metal,
                name=f"pull_plate_{post_i}",
            )
            drawer.visual(
                Cylinder(radius=0.004, length=0.004),
                origin=Origin(xyz=(0.044, y - 0.014, handle_z + 0.007), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=metal,
                name=f"pull_screw_{post_i}_0",
            )
            drawer.visual(
                Cylinder(radius=0.004, length=0.004),
                origin=Origin(xyz=(0.044, y + 0.014, handle_z - 0.007), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=metal,
                name=f"pull_screw_{post_i}_1",
            )

        model.articulation(
            f"carcass_to_{drawer_name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(front_x, 0.0, z_center)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=travel),
            motion_properties=MotionProperties(damping=10.0, friction=4.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    for idx in range(4):
        drawer = object_model.get_part(f"drawer_{idx}")
        joint = object_model.get_articulation(f"carcass_to_drawer_{idx}")
        upper = joint.motion_limits.upper or 0.0

        ctx.expect_within(
            drawer,
            carcass,
            axes="yz",
            inner_elem="front_panel",
            margin=0.015,
            name=f"drawer_{idx} front stays inside the face frame outline",
        )
        ctx.expect_overlap(
            drawer,
            carcass,
            axes="x",
            elem_a="tray_bottom",
            elem_b="side_panel_0",
            min_overlap=0.35,
            name=f"drawer_{idx} tray is deeply seated when closed",
        )
        ctx.expect_gap(
            carcass,
            drawer,
            axis="y",
            positive_elem=f"fixed_slide_0_{idx}",
            negative_elem="runner_0",
            max_gap=0.004,
            max_penetration=0.00001,
            name=f"drawer_{idx} positive-side slide has working clearance",
        )
        ctx.expect_gap(
            drawer,
            carcass,
            axis="y",
            positive_elem="runner_1",
            negative_elem=f"fixed_slide_1_{idx}",
            max_gap=0.004,
            max_penetration=0.00001,
            name=f"drawer_{idx} negative-side slide has working clearance",
        )

        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: upper}):
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="x",
                elem_a="tray_bottom",
                elem_b="side_panel_0",
                min_overlap=0.08,
                name=f"drawer_{idx} keeps retained insertion when extended",
            )
            extended_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer_{idx} extends outward on its slide",
            closed_pos is not None
            and extended_pos is not None
            and extended_pos[0] > closed_pos[0] + upper - 0.005,
            details=f"closed={closed_pos}, extended={extended_pos}, upper={upper}",
        )

    return ctx.report()


object_model = build_object_model()
