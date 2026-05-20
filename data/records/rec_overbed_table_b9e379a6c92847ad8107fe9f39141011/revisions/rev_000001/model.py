from __future__ import annotations

import math

import cadquery as cq
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
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehab_overbed_table")

    white = model.material("warm_white_laminate", rgba=(0.92, 0.88, 0.78, 1.0))
    edge = model.material("soft_grey_edge_banding", rgba=(0.42, 0.43, 0.42, 1.0))
    seam = model.material("dark_split_seam", rgba=(0.06, 0.06, 0.055, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.66, 0.67, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    red = model.material("red_release_handle", rgba=(0.75, 0.04, 0.035, 1.0))
    dark = model.material("dark_hub_plastic", rgba=(0.08, 0.085, 0.09, 1.0))

    def box(part, name, size, xyz, material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def cyl(part, name, radius, length, xyz, rpy, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    # Root: a low U-shaped steel base with the open side facing +X.  The caster
    # fork brackets are part of the fixed base so the four wheels can spin
    # independently on their horizontal axles.
    base = model.part("base")
    box(base, "rear_cross_tube", (0.070, 0.660, 0.045), (-0.320, 0.0, 0.100), steel)
    box(base, "side_tube_0", (0.805, 0.055, 0.045), (0.045, -0.300, 0.100), steel)
    box(base, "side_tube_1", (0.805, 0.055, 0.045), (0.045, 0.300, 0.100), steel)

    # Rectangular outer mast sleeve: four walls leave a true clear inner bore for
    # the sliding mast instead of using a solid proxy.
    box(base, "sleeve_front_wall", (0.083, 0.010, 0.505), (-0.250, -0.238, 0.345), steel)
    box(base, "sleeve_rear_wall", (0.083, 0.010, 0.505), (-0.250, -0.322, 0.345), steel)
    box(base, "sleeve_side_wall_0", (0.010, 0.084, 0.505), (-0.197, -0.280, 0.345), steel)
    box(base, "sleeve_side_wall_1", (0.010, 0.084, 0.505), (-0.303, -0.280, 0.345), steel)
    box(base, "sleeve_foot_plate", (0.150, 0.130, 0.012), (-0.250, -0.280, 0.126), steel)
    box(base, "mast_guide_pad_x0", (0.0225, 0.026, 0.055), (-0.28675, -0.280, 0.465), dark)
    box(base, "mast_guide_pad_x1", (0.0225, 0.026, 0.055), (-0.21325, -0.280, 0.465), dark)
    box(base, "mast_guide_pad_y0", (0.026, 0.0115, 0.055), (-0.250, -0.24875, 0.405), dark)
    box(base, "mast_guide_pad_y1", (0.026, 0.0115, 0.055), (-0.250, -0.31125, 0.405), dark)
    box(base, "mast_gusset_x", (0.160, 0.020, 0.140), (-0.250, -0.300, 0.195), steel)
    box(base, "mast_gusset_y", (0.020, 0.150, 0.140), (-0.250, -0.280, 0.195), steel)

    caster_positions = [
        (-0.330, -0.300, "0"),
        (0.415, -0.300, "1"),
        (-0.330, 0.300, "2"),
        (0.415, 0.300, "3"),
    ]
    for x, y, idx in caster_positions:
        box(base, f"caster_stem_{idx}", (0.030, 0.030, 0.035), (x, y, 0.104), steel)
        box(base, f"caster_yoke_top_{idx}", (0.058, 0.078, 0.014), (x, y, 0.083), steel)
        box(base, f"caster_yoke_cheek_{idx}_0", (0.016, 0.006, 0.075), (x, y - 0.0195, 0.0395), steel)
        box(base, f"caster_yoke_cheek_{idx}_1", (0.016, 0.006, 0.075), (x, y + 0.0195, 0.0395), steel)

    # Height-adjustable inner mast.
    mast = model.part("mast")
    box(mast, "mast_tube", (0.052, 0.052, 0.680), (0.0, 0.0, 0.040), steel)
    box(mast, "height_scale_strip", (0.004, 0.054, 0.360), (0.028, 0.0, 0.155), dark)

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(-0.250, -0.280, 0.585)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.16, lower=0.0, upper=0.260),
    )

    # Head shares the mast attachment and carries both the main top hinge and the
    # separate reading wing bracket, making the smaller wing visibly supported by
    # the same head rather than floating beside the tray.
    head = model.part("head")
    # Hollow clamp collar around the mast.
    box(head, "clamp_front_wall", (0.086, 0.010, 0.078), (0.0, 0.039, 0.0), steel)
    box(head, "clamp_rear_wall", (0.086, 0.010, 0.078), (0.0, -0.039, 0.0), steel)
    box(head, "clamp_side_wall_0", (0.010, 0.088, 0.078), (0.048, 0.0, 0.0), steel)
    box(head, "clamp_side_wall_1", (0.010, 0.088, 0.078), (-0.048, 0.0, 0.0), steel)
    cyl(head, "clamp_knob", 0.018, 0.046, (0.070, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0), dark)

    # Cantilever from mast to the tray hinge line.
    box(head, "head_boom_x", (0.235, 0.044, 0.044), (0.1705, 0.0, -0.025), steel)
    box(head, "head_boom_y", (0.045, 0.130, 0.044), (0.265, 0.055, -0.025), steel)
    box(head, "main_hinge_rail", (0.850, 0.046, 0.032), (0.250, 0.048, -0.025), steel)
    cyl(
        head,
        "main_hinge_barrel",
        0.014,
        0.830,
        (0.250, 0.070, -0.002),
        (0.0, math.pi / 2.0, 0.0),
        steel,
    )

    # Short bracket from the tray edge to the secondary wing hinge.
    box(head, "reading_bracket_arm", (0.040, 0.305, 0.026), (0.665, 0.215, -0.022), steel)
    box(head, "reading_bracket_pad", (0.096, 0.044, 0.026), (0.665, 0.365, -0.022), steel)
    cyl(
        head,
        "reading_hinge_barrel",
        0.011,
        0.325,
        (0.685, 0.365, -0.001),
        (-math.pi / 2.0, 0.0, 0.0),
        steel,
    )
    box(head, "handle_mount_tab", (0.028, 0.046, 0.040), (0.666, 0.048, -0.047), steel)

    model.articulation(
        "mast_to_head",
        ArticulationType.FIXED,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
    )

    # Main split-top work surface.  Its frame is the hinge line; the tray extends
    # along local +Y so positive rotation about +X tilts the free edge upward.
    main_top = model.part("main_top")
    box(main_top, "top_panel", (0.780, 0.560, 0.032), (0.0, 0.300, 0.020), white)
    box(main_top, "hinge_leaf", (0.780, 0.045, 0.008), (0.0, 0.024, 0.005), edge)
    box(main_top, "front_lip", (0.780, 0.024, 0.032), (0.0, 0.585, 0.044), edge)
    box(main_top, "side_lip_0", (0.024, 0.535, 0.026), (-0.390, 0.310, 0.041), edge)
    box(main_top, "side_lip_1", (0.024, 0.535, 0.026), (0.390, 0.310, 0.041), edge)
    box(main_top, "split_seam", (0.010, 0.495, 0.006), (0.170, 0.325, 0.039), seam)

    model.articulation(
        "head_to_main_top",
        ArticulationType.REVOLUTE,
        parent=head,
        child=main_top,
        origin=Origin(xyz=(0.250, 0.070, 0.0105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=0.0, upper=0.85),
    )

    # Smaller reading wing beside the main panel.  Its hinge line is local Y; the
    # panel extends along +X and lifts on positive motion around -Y.
    reading_wing = model.part("reading_wing")
    box(reading_wing, "wing_panel", (0.280, 0.330, 0.026), (0.140, 0.0, 0.017), white)
    box(reading_wing, "wing_hinge_leaf", (0.045, 0.315, 0.007), (0.022, 0.0, 0.005), edge)
    box(reading_wing, "book_lip", (0.028, 0.300, 0.034), (0.270, 0.0, 0.043), edge)
    box(reading_wing, "wing_side_lip_0", (0.245, 0.020, 0.024), (0.145, -0.165, 0.038), edge)
    box(reading_wing, "wing_side_lip_1", (0.245, 0.020, 0.024), (0.145, 0.165, 0.038), edge)

    model.articulation(
        "head_to_reading_wing",
        ArticulationType.REVOLUTE,
        parent=head,
        child=reading_wing,
        origin=Origin(xyz=(0.685, 0.365, 0.0085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.9, lower=0.0, upper=1.05),
    )

    # Side tilt-release handle at the hinge head.
    tilt_handle = model.part("tilt_handle")
    cyl(tilt_handle, "handle_pivot", 0.018, 0.050, (0.0, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0), red)
    box(tilt_handle, "release_lever", (0.025, 0.032, 0.155), (0.0, -0.022, -0.080), red)
    cyl(tilt_handle, "handle_grip", 0.017, 0.075, (0.0, -0.030, -0.158), (0.0, math.pi / 2.0, 0.0), black)

    model.articulation(
        "head_to_tilt_handle",
        ArticulationType.REVOLUTE,
        parent=head,
        child=tilt_handle,
        origin=Origin(xyz=(0.705, 0.048, -0.047)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.2, lower=-0.35, upper=0.55),
    )

    # Caster wheels: small rubber tires with visible hubs.  The child frame is
    # rotated so each wheel's local +X spin axis lies on the base's side-to-side
    # axle direction.
    caster_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.026,
            0.026,
            rim=WheelRim(inner_radius=0.017, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(radius=0.012, width=0.020, cap_style="domed"),
            bore=WheelBore(style="round", diameter=0.006),
        ),
        "caster_wheel_hub",
    )
    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.036,
            0.034,
            inner_radius=0.026,
            tread=TireTread(style="ribbed", depth=0.002, count=14, land_ratio=0.62),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.003, radius=0.002),
        ),
        "caster_rubber_tire",
    )

    for x, y, idx in caster_positions:
        caster = model.part(f"caster_{idx}")
        caster.visual(caster_tire_mesh, material=black, name="tire")
        caster.visual(caster_wheel_mesh, material=dark, name="hub")
        model.articulation(
            f"base_to_caster_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, 0.036), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    head = object_model.get_part("head")
    main_top = object_model.get_part("main_top")
    reading_wing = object_model.get_part("reading_wing")
    tilt_handle = object_model.get_part("tilt_handle")

    mast_slide = object_model.get_articulation("base_to_mast")
    main_hinge = object_model.get_articulation("head_to_main_top")
    wing_hinge = object_model.get_articulation("head_to_reading_wing")
    handle_hinge = object_model.get_articulation("head_to_tilt_handle")
    caster_joints = [
        object_model.get_articulation("base_to_caster_0"),
        object_model.get_articulation("base_to_caster_1"),
        object_model.get_articulation("base_to_caster_2"),
        object_model.get_articulation("base_to_caster_3"),
    ]

    def z_max(aabb):
        return None if aabb is None else tuple(aabb[1])[2]

    def z_min(aabb):
        return None if aabb is None else tuple(aabb[0])[2]

    def y_max(aabb):
        return None if aabb is None else tuple(aabb[1])[1]

    ctx.check("four continuous caster wheel joints are present", len(caster_joints) == 4)

    # The mast is retained inside the hollow sleeve both collapsed and at the
    # height-adjustment upper stop.
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        min_overlap=0.25,
        elem_a="mast_tube",
        elem_b="sleeve_front_wall",
        name="collapsed mast remains deeply inserted in sleeve",
    )
    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.260}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            min_overlap=0.045,
            elem_a="mast_tube",
            elem_b="sleeve_front_wall",
            name="raised mast still has retained sleeve insertion",
        )
        raised_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast prismatic joint raises the table head",
        rest_mast_pos is not None
        and raised_mast_pos is not None
        and raised_mast_pos[2] > rest_mast_pos[2] + 0.20,
        details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
    )

    # Main top hinge is supported by the head and raises the tray surface.
    ctx.expect_overlap(
        main_top,
        head,
        axes="x",
        min_overlap=0.70,
        elem_a="hinge_leaf",
        elem_b="main_hinge_barrel",
        name="main top hinge spans across the support head",
    )
    ctx.expect_gap(
        main_top,
        head,
        axis="z",
        positive_elem="hinge_leaf",
        negative_elem="main_hinge_barrel",
        max_penetration=0.002,
        max_gap=0.002,
        name="main hinge leaf is seated on the barrel line",
    )
    rest_top_z = z_max(ctx.part_world_aabb(main_top))
    with ctx.pose({main_hinge: 0.75}):
        tilted_top_z = z_max(ctx.part_world_aabb(main_top))
    ctx.check(
        "main top hinge tilts the work surface upward",
        rest_top_z is not None and tilted_top_z is not None and tilted_top_z > rest_top_z + 0.18,
        details=f"rest_zmax={rest_top_z}, tilted_zmax={tilted_top_z}",
    )

    # The smaller wing is beside, not on top of, the main surface and is carried
    # by the same head through the short bracket at the tray edge.
    ctx.expect_gap(
        reading_wing,
        main_top,
        axis="x",
        min_gap=0.015,
        max_gap=0.080,
        positive_elem="wing_panel",
        negative_elem="top_panel",
        name="reading wing sits beside the main top with a clear side gap",
    )
    ctx.expect_overlap(
        reading_wing,
        head,
        axes="y",
        min_overlap=0.28,
        elem_a="wing_hinge_leaf",
        elem_b="reading_hinge_barrel",
        name="reading wing hinge is carried by the tray-edge bracket",
    )
    ctx.expect_gap(
        reading_wing,
        head,
        axis="z",
        positive_elem="wing_hinge_leaf",
        negative_elem="reading_hinge_barrel",
        max_penetration=0.002,
        max_gap=0.003,
        name="reading wing hinge leaf is seated on its short hinge",
    )
    rest_wing_z = z_max(ctx.part_world_aabb(reading_wing))
    with ctx.pose({wing_hinge: 0.90}):
        raised_wing_z = z_max(ctx.part_world_aabb(reading_wing))
    ctx.check(
        "reading wing independently rotates upward",
        rest_wing_z is not None and raised_wing_z is not None and raised_wing_z > rest_wing_z + 0.10,
        details=f"rest_zmax={rest_wing_z}, raised_zmax={raised_wing_z}",
    )

    rest_handle_y = y_max(ctx.part_world_aabb(tilt_handle))
    with ctx.pose({handle_hinge: 0.50}):
        released_handle_y = y_max(ctx.part_world_aabb(tilt_handle))
    ctx.check(
        "side tilt-release handle articulates at the head",
        rest_handle_y is not None
        and released_handle_y is not None
        and released_handle_y > rest_handle_y + 0.030,
        details=f"rest_ymax={rest_handle_y}, released_ymax={released_handle_y}",
    )

    return ctx.report()


object_model = build_object_model()
