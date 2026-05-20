from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stackable_rolling_toolbox_base")

    graphite = model.material("graphite_rugged_polymer", rgba=(0.055, 0.060, 0.060, 1.0))
    dark = model.material("black_rubberized_trim", rgba=(0.012, 0.013, 0.012, 1.0))
    yellow = model.material("yellow_latch_plastic", rgba=(0.95, 0.70, 0.08, 1.0))
    steel = model.material("dark_zinc_pin", rgba=(0.35, 0.36, 0.34, 1.0))

    body = model.part("body")
    body_w = 0.72
    body_d = 0.42
    body_h = 0.40
    body_z = 0.08
    wall = 0.035
    bottom_thickness = 0.040

    outer_bin = (
        cq.Workplane("XY")
        .box(body_w, body_d, body_h, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.030)
    )
    inner_void = (
        cq.Workplane("XY")
        .box(body_w - 2.0 * wall, body_d - 2.0 * wall, body_h, centered=(True, True, False))
        .translate((0.0, 0.0, bottom_thickness))
    )
    lower_bin_shell = outer_bin.cut(inner_void)
    body.visual(
        mesh_from_cadquery(lower_bin_shell, "lower_bin_shell", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, body_z)),
        material=graphite,
        name="lower_bin_shell",
    )

    # Rugged molded ribs and stack feet are part of the lower shell and overlap the shell wall.
    for i, x in enumerate((-0.245, 0.245)):
        body.visual(
            Box((0.055, 0.030, 0.34)),
            origin=Origin(xyz=(x, -body_d / 2.0 - 0.006, body_z + 0.20)),
            material=dark,
            name=f"front_rib_{i}",
        )
    for i, x in enumerate((-0.260, 0.0, 0.260)):
        body.visual(
            Box((0.130, 0.035, 0.026)),
            origin=Origin(xyz=(x, -body_d / 2.0 + 0.015, body_z + 0.018)),
            material=dark,
            name=f"front_skid_{i}",
        )
    for i, x in enumerate((-0.220, 0.220)):
        body.visual(
            Box((0.060, 0.055, 0.026)),
            origin=Origin(xyz=(x, body_d / 2.0 - 0.035, body_z + body_h + 0.006)),
            material=dark,
            name=f"stack_socket_{i}",
        )

    # Rear axle and wheel brackets for the rolling base.
    body.visual(
        Cylinder(radius=0.012, length=0.700),
        origin=Origin(xyz=(0.0, 0.320, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_axle",
    )
    for i, x in enumerate((-0.220, 0.220)):
        body.visual(
            Box((0.040, 0.120, 0.050)),
            origin=Origin(xyz=(x, 0.270, 0.110)),
            material=dark,
            name=f"axle_bracket_{i}",
        )

    # Fixed twin rear rails and guide brackets for the telescoping handle.
    rail_xs = (-0.230, 0.230)
    rail_back_names = ("rear_rail_back_0", "rear_rail_back_1")
    for i, x in enumerate(rail_xs):
        body.visual(
            Box((0.060, 0.018, 0.460)),
            origin=Origin(xyz=(x, 0.312, 0.455)),
            material=dark,
            name=rail_back_names[i],
        )
        for j, dx in enumerate((-0.033, 0.033)):
            body.visual(
                Box((0.010, 0.045, 0.460)),
                origin=Origin(xyz=(x + dx, 0.335, 0.455)),
                material=dark,
                name=f"rear_rail_lip_{i}_{j}",
            )
        for j, z in enumerate((0.300, 0.675)):
            body.visual(
                Box((0.080, 0.125, 0.035)),
                origin=Origin(xyz=(x, 0.260, z)),
                material=dark,
                name=f"rail_mount_{i}_{j}",
            )

    hinge_y = body_d / 2.0 + 0.025
    hinge_z = body_z + body_h + 0.035
    body.visual(
        Box((0.700, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, body_d / 2.0 + 0.012, body_z + body_h + 0.010)),
        material=dark,
        name="rear_hinge_strip",
    )
    for i, x in enumerate((-0.300, 0.0, 0.300)):
        body.visual(
            Cylinder(radius=0.018, length=0.100),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"body_hinge_barrel_{i}",
        )
        body.visual(
            Box((0.105, 0.026, 0.010)),
            origin=Origin(xyz=(x, body_d / 2.0 + 0.004, body_z + body_h + 0.012)),
            material=dark,
            name=f"body_hinge_leaf_{i}",
        )

    # Side pivot pads for latch levers.
    latch_pivot_y = -0.055
    latch_pivot_z = body_z + body_h - 0.025
    latch_pivot_x = body_w / 2.0 + 0.036
    for i, sign in enumerate((1.0, -1.0)):
        body.visual(
            Cylinder(radius=0.024, length=0.018),
            origin=Origin(
                xyz=(sign * (body_w / 2.0 + 0.009), latch_pivot_y, latch_pivot_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark,
            name=f"latch_pivot_pad_{i}",
        )

    lid = model.part("lid")
    lid_w = 0.740
    lid_d = 0.415
    lid_h = 0.080
    lid_panel = (
        cq.Workplane("XY")
        .box(lid_w, lid_d, lid_h, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.024)
        .edges(">Z")
        .fillet(0.006)
    )
    lid.visual(
        mesh_from_cadquery(lid_panel, "lid_panel", tolerance=0.0015),
        origin=Origin(xyz=(0.0, -0.2325, 0.020)),
        material=graphite,
        name="lid_panel",
    )
    # Raised ribs and a recessed-looking top field read as a rugged polymer lid.
    lid.visual(
        Box((0.580, 0.230, 0.012)),
        origin=Origin(xyz=(0.0, -0.245, 0.061)),
        material=dark,
        name="top_recess_field",
    )
    for i, x in enumerate((-0.260, 0.260)):
        lid.visual(
            Box((0.050, 0.300, 0.022)),
            origin=Origin(xyz=(x, -0.240, 0.067)),
            material=graphite,
            name=f"lid_long_rib_{i}",
        )
    for i, y in enumerate((-0.350, -0.155)):
        lid.visual(
            Box((0.620, 0.035, 0.020)),
            origin=Origin(xyz=(0.0, y, 0.066)),
            material=graphite,
            name=f"lid_cross_rib_{i}",
        )
    for i, x in enumerate((-0.150, 0.150)):
        lid.visual(
            Cylinder(radius=0.017, length=0.120),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"lid_hinge_barrel_{i}",
        )
        lid.visual(
            Box((0.120, 0.030, 0.010)),
            origin=Origin(xyz=(x, -0.018, 0.004)),
            material=dark,
            name=f"lid_hinge_leaf_{i}",
        )
    for i, sign in enumerate((1.0, -1.0)):
        lid.visual(
            Box((0.018, 0.085, 0.040)),
            origin=Origin(xyz=(sign * 0.379, -0.290, 0.024)),
            material=yellow,
            name=f"lid_latch_catch_{i}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=1.65),
    )

    handle = model.part("handle")
    stanchion_names = ("handle_stanchion_0", "handle_stanchion_1")
    for i, x in enumerate(rail_xs):
        handle.visual(
            Box((0.024, 0.024, 0.680)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material=steel,
            name=stanchion_names[i],
        )
    handle.visual(
        Cylinder(radius=0.026, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.390), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="handle_grip",
    )
    handle.visual(
        Box((0.510, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
        material=steel,
        name="handle_cross_tie",
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.340, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.380),
    )

    # Yellow over-center latch levers bridge the lid/body seam on short side pivots.
    for i, sign in enumerate((1.0, -1.0)):
        latch = model.part(f"side_latch_{i}")
        latch.visual(
            Cylinder(radius=0.024, length=0.036),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=yellow,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.018, 0.070, 0.180)),
            origin=Origin(xyz=(sign * 0.018, 0.0, 0.055)),
            material=yellow,
            name="latch_bridge",
        )
        latch.visual(
            Box((0.028, 0.082, 0.035)),
            origin=Origin(xyz=(sign * 0.009, 0.0, 0.105)),
            material=yellow,
            name="upper_hook",
        )
        latch.visual(
            Box((0.030, 0.056, 0.028)),
            origin=Origin(xyz=(sign * 0.008, 0.0, -0.040)),
            material=yellow,
            name="lower_pull_tab",
        )
        model.articulation(
            f"body_to_side_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=latch,
            origin=Origin(xyz=(sign * latch_pivot_x, latch_pivot_y, latch_pivot_z)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.05),
        )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.066,
            0.055,
            rim=WheelRim(inner_radius=0.045, flange_height=0.006, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.024,
                width=0.040,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.032, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.003, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "wheel_hub",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.090,
            0.075,
            inner_radius=0.064,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.003),),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.007, radius=0.003),
        ),
        "wheel_tire",
    )
    for i, x in enumerate((-0.310, 0.310)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, origin=Origin(), material=dark, name="tire")
        wheel.visual(wheel_mesh, origin=Origin(), material=steel, name="hub")
        model.articulation(
            f"body_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x, 0.320, 0.090)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=15.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    handle_slide = object_model.get_articulation("body_to_handle")
    lid_hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="lower_bin_shell",
        min_gap=0.006,
        max_gap=0.025,
        name="closed lid has a clear polymer seam above the lower bin",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="lower_bin_shell",
        min_overlap=0.34,
        name="lid footprint covers the lower bin opening",
    )

    collapsed_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.380}):
        extended_pos = ctx.part_world_position(handle)
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="handle_stanchion_0",
            elem_b="rear_rail_back_0",
            min_overlap=0.25,
            name="extended handle remains captured in rear rail 0",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="handle_stanchion_1",
            elem_b="rear_rail_back_1",
            min_overlap=0.25,
            name="extended handle remains captured in rear rail 1",
        )
    ctx.check(
        "telescoping handle translates upward",
        collapsed_pos is not None and extended_pos is not None and extended_pos[2] > collapsed_pos[2] + 0.30,
        details=f"collapsed={collapsed_pos}, extended={extended_pos}",
    )

    with ctx.pose({lid_hinge: 1.20}):
        ctx.expect_origin_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            name="lid hinge opens the lid upward",
        )

    for i in (0, 1):
        latch = object_model.get_part(f"side_latch_{i}")
        ctx.expect_overlap(
            latch,
            body,
            axes="z",
            elem_a="latch_bridge",
            elem_b="lower_bin_shell",
            min_overlap=0.035,
            name=f"side latch {i} reaches down onto the body side",
        )
        ctx.expect_overlap(
            latch,
            lid,
            axes="z",
            elem_a="latch_bridge",
            elem_b=f"lid_latch_catch_{i}",
            min_overlap=0.025,
            name=f"side latch {i} bridges up to the lid catch",
        )

    for i in (0, 1):
        wheel = object_model.get_part(f"wheel_{i}")
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="rear_axle",
            elem_b="hub",
            reason="The steel axle intentionally passes through the wheel hub bore so the wheel is retained while spinning.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="x",
            elem_a="rear_axle",
            elem_b="hub",
            min_overlap=0.040,
            name=f"wheel {i} hub is captured on the axle",
        )

    return ctx.report()


object_model = build_object_model()
