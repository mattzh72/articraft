from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel_mesh(width: float, depth: float, thickness: float, radius: float, name: str):
    profile = rounded_rect_profile(width, depth, radius, corner_segments=10)
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehab_overbed_table")

    powder_white = model.material("powder_white", rgba=(0.86, 0.88, 0.87, 1.0))
    satin_chrome = model.material("satin_chrome", rgba=(0.62, 0.64, 0.66, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.03, 0.03, 0.035, 1.0))
    warm_laminate = model.material("warm_laminate", rgba=(0.78, 0.68, 0.50, 1.0))
    dark_edge = model.material("dark_edge", rgba=(0.10, 0.09, 0.075, 1.0))
    blue_release = model.material("blue_release", rgba=(0.08, 0.20, 0.72, 1.0))
    lock_red = model.material("lock_red", rgba=(0.76, 0.05, 0.04, 1.0))

    main_panel_mesh = _rounded_panel_mesh(0.66, 0.42, 0.030, 0.045, "main_top_panel")
    wing_panel_mesh = _rounded_panel_mesh(0.24, 0.42, 0.030, 0.040, "side_wing_panel")
    wheel_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.024,
            0.024,
            rim=WheelRim(inner_radius=0.014, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(radius=0.010, width=0.026, cap_style="domed"),
            face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.006),
        ),
        "caster_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.038,
            0.030,
            inner_radius=0.0245,
            tread=TireTread(style="ribbed", depth=0.002, count=18, land_ratio=0.6),
            sidewall=TireSidewall(style="rounded", bulge=0.03),
            shoulder=TireShoulder(width=0.003, radius=0.002),
        ),
        "caster_tire",
    )

    base = model.part("base")
    # U-shaped rolling base: two forward rails and a rear cross member, open at +Y.
    base.visual(Box((0.080, 0.92, 0.052)), origin=Origin(xyz=(-0.34, 0.05, 0.125)), material=powder_white, name="side_rail_0")
    base.visual(Box((0.080, 0.92, 0.052)), origin=Origin(xyz=(0.34, 0.05, 0.125)), material=powder_white, name="side_rail_1")
    base.visual(Box((0.76, 0.080, 0.052)), origin=Origin(xyz=(0.0, -0.37, 0.125)), material=powder_white, name="rear_crossbar")
    base.visual(Box((0.22, 0.12, 0.022)), origin=Origin(xyz=(0.0, -0.37, 0.172)), material=satin_chrome, name="mast_foot_plate")
    base.visual(Cylinder(radius=0.046, length=0.020), origin=Origin(xyz=(0.0, -0.37, 0.180)), material=satin_chrome, name="mast_socket_collar")

    # Rectangular telescoping outer sleeve, built as four walls so the sliding mast
    # is genuinely clearanced rather than hidden inside a solid proxy.
    sleeve_z = 0.360
    sleeve_height = 0.47
    base.visual(Box((0.014, 0.084, sleeve_height)), origin=Origin(xyz=(-0.049, -0.37, sleeve_z)), material=satin_chrome, name="sleeve_wall_0")
    base.visual(Box((0.014, 0.084, sleeve_height)), origin=Origin(xyz=(0.049, -0.37, sleeve_z)), material=satin_chrome, name="sleeve_wall_1")
    base.visual(Box((0.084, 0.014, sleeve_height)), origin=Origin(xyz=(0.0, -0.419, sleeve_z)), material=satin_chrome, name="sleeve_wall_2")
    base.visual(Box((0.084, 0.014, sleeve_height)), origin=Origin(xyz=(0.0, -0.321, sleeve_z)), material=satin_chrome, name="sleeve_wall_3")
    base.visual(Box((0.115, 0.030, 0.045)), origin=Origin(xyz=(0.0, -0.306, 0.560)), material=dark_graphite, name="height_lock_clamp")
    base.visual(Cylinder(radius=0.012, length=0.050), origin=Origin(xyz=(0.075, -0.306, 0.560), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_graphite, name="height_knob_stem")
    base.visual(Cylinder(radius=0.020, length=0.020), origin=Origin(xyz=(0.110, -0.306, 0.560), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_graphite, name="height_knob")

    caster_positions = [
        (-0.34, -0.37),
        (0.34, -0.37),
        (-0.34, 0.49),
        (0.34, 0.49),
    ]
    for i, (x, y) in enumerate(caster_positions):
        base.visual(Cylinder(radius=0.026, length=0.014), origin=Origin(xyz=(x, y, 0.108)), material=satin_chrome, name=f"caster_socket_{i}")

    mast = model.part("mast")
    mast.visual(Box((0.044, 0.044, 0.660)), origin=Origin(xyz=(0.0, 0.0, -0.030)), material=satin_chrome, name="inner_mast")
    mast.visual(Box((0.020, 0.036, 0.070)), origin=Origin(xyz=(-0.032, 0.0, -0.250)), material=dark_graphite, name="guide_pad_0")
    mast.visual(Box((0.020, 0.036, 0.070)), origin=Origin(xyz=(0.032, 0.0, -0.250)), material=dark_graphite, name="guide_pad_1")
    mast.visual(Box((0.036, 0.020, 0.070)), origin=Origin(xyz=(0.0, -0.032, -0.250)), material=dark_graphite, name="guide_pad_2")
    mast.visual(Box((0.036, 0.020, 0.070)), origin=Origin(xyz=(0.0, 0.032, -0.250)), material=dark_graphite, name="guide_pad_3")
    mast.visual(Box((0.066, 0.058, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.3175)), material=dark_graphite, name="top_cap")

    head = model.part("head")
    head.visual(Box((0.200, 0.110, 0.052)), origin=Origin(xyz=(0.0, 0.018, 0.026)), material=dark_graphite, name="head_block")
    head.visual(Box((0.880, 0.060, 0.035)), origin=Origin(xyz=(0.0, 0.070, 0.022)), material=dark_graphite, name="shared_support")
    head.visual(Box((0.075, 0.275, 0.022)), origin=Origin(xyz=(-0.235, 0.195, 0.018)), material=dark_graphite, name="wing_underarm")
    head.visual(Box((0.105, 0.250, 0.024)), origin=Origin(xyz=(0.090, 0.195, 0.018)), material=dark_graphite, name="main_underarm")
    head.visual(Cylinder(radius=0.013, length=0.740), origin=Origin(xyz=(0.020, -0.028, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)), material=satin_chrome, name="tilt_hinge_pin")
    head.visual(Box((0.030, 0.050, 0.050)), origin=Origin(xyz=(0.4395, 0.025, 0.042)), material=dark_graphite, name="handle_mount_lug")
    head.visual(Cylinder(radius=0.010, length=0.060), origin=Origin(xyz=(0.0, -0.018, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)), material=satin_chrome, name="mast_head_pin")

    main_top = model.part("main_top")
    main_top.visual(main_panel_mesh, origin=Origin(xyz=(0.0, 0.210, 0.018)), material=warm_laminate, name="main_panel")
    main_top.visual(Box((0.670, 0.018, 0.032)), origin=Origin(xyz=(0.0, 0.418, 0.018)), material=dark_edge, name="front_edge")
    main_top.visual(Box((0.670, 0.014, 0.030)), origin=Origin(xyz=(0.0, 0.006, 0.018)), material=dark_edge, name="rear_edge")
    main_top.visual(Box((0.018, 0.410, 0.030)), origin=Origin(xyz=(-0.329, 0.210, 0.018)), material=dark_edge, name="side_edge_0")
    main_top.visual(Box((0.018, 0.410, 0.030)), origin=Origin(xyz=(0.329, 0.210, 0.018)), material=dark_edge, name="side_edge_1")
    main_top.visual(Box((0.230, 0.045, 0.012)), origin=Origin(xyz=(0.0, 0.030, -0.002)), material=satin_chrome, name="hinge_leaf")

    side_wing = model.part("side_wing")
    side_wing.visual(wing_panel_mesh, origin=Origin(xyz=(-0.380, 0.210, 0.068)), material=warm_laminate, name="wing_panel")
    side_wing.visual(Box((0.250, 0.018, 0.032)), origin=Origin(xyz=(-0.380, 0.418, 0.068)), material=dark_edge, name="wing_front_edge")
    side_wing.visual(Box((0.250, 0.014, 0.030)), origin=Origin(xyz=(-0.380, 0.006, 0.068)), material=dark_edge, name="wing_rear_edge")
    side_wing.visual(Box((0.018, 0.410, 0.030)), origin=Origin(xyz=(-0.499, 0.210, 0.068)), material=dark_edge, name="wing_outer_edge")
    side_wing.visual(Box((0.014, 0.410, 0.030)), origin=Origin(xyz=(-0.262, 0.210, 0.068)), material=dark_edge, name="wing_seam_edge")
    side_wing.visual(Box((0.205, 0.070, 0.014)), origin=Origin(xyz=(-0.380, 0.060, 0.047)), material=satin_chrome, name="shared_head_foot")

    tilt_handle = model.part("tilt_handle")
    tilt_handle.visual(Cylinder(radius=0.016, length=0.038), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=blue_release, name="pivot_boss")
    tilt_handle.visual(Box((0.125, 0.022, 0.018)), origin=Origin(xyz=(0.070, 0.0, -0.008)), material=blue_release, name="release_lever")
    tilt_handle.visual(Cylinder(radius=0.014, length=0.050), origin=Origin(xyz=(0.140, 0.0, -0.008), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_graphite, name="finger_pad")

    caster_forks = []
    caster_wheels = []
    lock_tabs = []
    for i, (x, y) in enumerate(caster_positions):
        fork = model.part(f"caster_fork_{i}")
        fork.visual(Box((0.074, 0.052, 0.018)), origin=Origin(xyz=(0.0, 0.0, -0.009)), material=satin_chrome, name="fork_top_plate")
        fork.visual(Box((0.012, 0.048, 0.070)), origin=Origin(xyz=(-0.031, 0.0, -0.048)), material=satin_chrome, name="fork_cheek_0")
        fork.visual(Box((0.012, 0.048, 0.070)), origin=Origin(xyz=(0.031, 0.0, -0.048)), material=satin_chrome, name="fork_cheek_1")
        fork.visual(Cylinder(radius=0.006, length=0.010), origin=Origin(xyz=(-0.0215, 0.0, -0.075), rpy=(0.0, math.pi / 2.0, 0.0)), material=satin_chrome, name="axle_cap_0")
        fork.visual(Cylinder(radius=0.006, length=0.010), origin=Origin(xyz=(0.0215, 0.0, -0.075), rpy=(0.0, math.pi / 2.0, 0.0)), material=satin_chrome, name="axle_cap_1")
        caster_forks.append(fork)

        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(wheel_rim_mesh, material=satin_chrome, name="rim")
        wheel.visual(tire_mesh, material=black_rubber, name="tire")
        caster_wheels.append(wheel)

        tab = model.part(f"lock_tab_{i}")
        tab.visual(Cylinder(radius=0.005, length=0.050), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=lock_red, name="tab_pivot")
        tab.visual(Box((0.052, 0.038, 0.010)), origin=Origin(xyz=(0.0, -0.012, 0.002)), material=lock_red, name="brake_pedal")
        tab.visual(Box((0.040, 0.018, 0.012)), origin=Origin(xyz=(0.0, -0.025, 0.008)), material=black_rubber, name="grip_rib")
        lock_tabs.append(tab)

        model.articulation(
            f"base_to_caster_fork_{i}",
            ArticulationType.FIXED,
            parent=base,
            child=fork,
            origin=Origin(xyz=(x, y, 0.099)),
        )
        model.articulation(
            f"caster_wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.075)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )
        model.articulation(
            f"lock_tab_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=fork,
            child=tab,
            origin=Origin(xyz=(0.0, -0.0285, -0.025)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=-0.55, upper=0.65),
        )

    model.articulation(
        "mast_height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, -0.37, 0.560)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.12, lower=0.0, upper=0.22),
    )
    model.articulation(
        "mast_to_head",
        ArticulationType.FIXED,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
    )
    model.articulation(
        "main_top_tilt",
        ArticulationType.REVOLUTE,
        parent=head,
        child=main_top,
        origin=Origin(xyz=(0.080, -0.006, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=0.70),
    )
    model.articulation(
        "head_to_side_wing",
        ArticulationType.FIXED,
        parent=head,
        child=side_wing,
        origin=Origin(),
    )
    model.articulation(
        "tilt_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=head,
        child=tilt_handle,
        origin=Origin(xyz=(0.470, 0.025, 0.042)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-0.45, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    head = object_model.get_part("head")
    main_top = object_model.get_part("main_top")
    side_wing = object_model.get_part("side_wing")

    mast_slide = object_model.get_articulation("mast_height_slide")
    top_tilt = object_model.get_articulation("main_top_tilt")
    handle_pivot = object_model.get_articulation("tilt_handle_pivot")

    ctx.check(
        "four continuous caster wheels",
        all(
            object_model.get_articulation(f"caster_wheel_spin_{i}").articulation_type
            == ArticulationType.CONTINUOUS
            for i in range(4)
        ),
        details="Each caster wheel should spin continuously on its axle.",
    )
    ctx.check(
        "four lock tab pivots",
        all(
            object_model.get_articulation(f"lock_tab_pivot_{i}").articulation_type
            == ArticulationType.REVOLUTE
            for i in range(4)
        ),
        details="Each caster lock tab should be a separate revolute control.",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="sleeve_wall_0",
        margin=0.065,
        name="sliding mast remains centered in sleeve opening",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="sleeve_wall_0",
        min_overlap=0.18,
        name="lowered mast retains insertion in sleeve",
    )
    with ctx.pose({mast_slide: 0.22}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="sleeve_wall_0",
            min_overlap=0.15,
            name="raised mast still retained in sleeve",
        )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.22}):
        raised_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast raises table head",
        rest_mast_pos is not None
        and raised_mast_pos is not None
        and raised_mast_pos[2] > rest_mast_pos[2] + 0.20,
        details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
    )

    ctx.expect_gap(
        side_wing,
        head,
        axis="z",
        min_gap=0.0,
        max_gap=0.030,
        positive_elem="shared_head_foot",
        negative_elem="shared_support",
        name="side wing is seated on shared support head",
    )
    ctx.expect_overlap(
        side_wing,
        head,
        axes="xy",
        min_overlap=0.05,
        elem_a="shared_head_foot",
        elem_b="shared_support",
        name="side wing shares the same support head",
    )

    rest_top_aabb = ctx.part_element_world_aabb(main_top, elem="main_panel")
    with ctx.pose({top_tilt: 0.70}):
        tilted_top_aabb = ctx.part_element_world_aabb(main_top, elem="main_panel")
    ctx.check(
        "main top tilts upward about horizontal hinge",
        rest_top_aabb is not None
        and tilted_top_aabb is not None
        and tilted_top_aabb[1][2] > rest_top_aabb[1][2] + 0.12,
        details=f"rest={rest_top_aabb}, tilted={tilted_top_aabb}",
    )

    rest_handle_aabb = ctx.part_world_aabb(object_model.get_part("tilt_handle"))
    with ctx.pose({handle_pivot: 0.50}):
        moved_handle_aabb = ctx.part_world_aabb(object_model.get_part("tilt_handle"))
    ctx.check(
        "tilt release handle rotates on side pivot",
        rest_handle_aabb is not None
        and moved_handle_aabb is not None
        and abs(moved_handle_aabb[0][2] - rest_handle_aabb[0][2]) > 0.010,
        details=f"rest={rest_handle_aabb}, moved={moved_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
