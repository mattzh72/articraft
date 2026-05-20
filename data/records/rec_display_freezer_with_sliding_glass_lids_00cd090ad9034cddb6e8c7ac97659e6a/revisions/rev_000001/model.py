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


def _polygon_prism(points: list[tuple[float, float]], height: float, z: float) -> cq.Workplane:
    return cq.Workplane("XY").polyline(points).close().extrude(height).translate((0.0, 0.0, z))


def _cabinet_shell() -> cq.Workplane:
    """Insulated corner-freezer body: solid bottom with an open, angled-plan tub."""
    outer = [
        (-0.72, -0.46),
        (0.66, -0.54),
        (0.76, 0.40),
        (-0.56, 0.56),
    ]
    inner = [
        (-0.56, -0.29),
        (0.49, -0.35),
        (0.57, 0.26),
        (-0.43, 0.38),
    ]
    bottom_z = 0.17
    top_z = 0.83
    floor_z = 0.29
    outer_solid = _polygon_prism(outer, top_z - bottom_z, bottom_z)
    opening_cut = _polygon_prism(inner, top_z - floor_z + 0.04, floor_z - 0.01)
    return outer_solid.cut(opening_cut)


def _glass_lid_part(
    part,
    *,
    panel_x: float,
    panel_y: float,
    material_glass: Material,
    material_metal: Material,
    material_grip: Material,
) -> None:
    """Add a framed glass sliding lid, centered on the part frame."""
    frame_h = 0.032
    frame_w = 0.030
    glass_h = 0.012
    glass_inset = 0.015

    part.visual(
        Box((panel_x - 2.0 * glass_inset, panel_y - 2.0 * glass_inset, glass_h)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=material_glass,
        name="glass_pane",
    )
    part.visual(
        Box((panel_x, frame_w, frame_h)),
        origin=Origin(xyz=(0.0, panel_y / 2.0, 0.0)),
        material=material_metal,
        name="rear_frame",
    )
    part.visual(
        Box((panel_x, frame_w, frame_h)),
        origin=Origin(xyz=(0.0, -panel_y / 2.0, 0.0)),
        material=material_metal,
        name="front_frame",
    )
    part.visual(
        Box((frame_w, panel_y, frame_h)),
        origin=Origin(xyz=(panel_x / 2.0, 0.0, 0.0)),
        material=material_metal,
        name="end_frame_0",
    )
    part.visual(
        Box((frame_w, panel_y, frame_h)),
        origin=Origin(xyz=(-panel_x / 2.0, 0.0, 0.0)),
        material=material_metal,
        name="end_frame_1",
    )
    # A small raised pull handle mounted on the glass.  The posts touch both the
    # glass and the transverse grip so the lid reads as one manufactured part.
    handle_x = 0.18
    part.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(xyz=(handle_x, 0.0, 0.046), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material_grip,
        name="handle_grip",
    )
    for y, name in ((-0.070, "handle_post_0"), (0.070, "handle_post_1")):
        part.visual(
            Cylinder(radius=0.006, length=0.036),
            origin=Origin(xyz=(handle_x, y, 0.026)),
            material=material_grip,
            name=name,
        )


def _edge_bar(part, p0, p1, *, z: float, width: float, height: float, material: Material, name: str) -> None:
    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    part.visual(
        Box((length, width, height)),
        origin=Origin(xyz=((x0 + x1) / 2.0, (y0 + y1) / 2.0, z), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _add_caster_mount(part, xy: tuple[float, float], material: Material, name: str) -> None:
    x, y = xy
    part.visual(
        Box((0.145, 0.115, 0.014)),
        origin=Origin(xyz=(x, y, 0.163)),
        material=material,
        name=f"{name}_plate",
    )
    # Four visible bolt heads keep the mount from reading as a floating pad.
    for index, (dx, dy) in enumerate(((-0.045, -0.032), (0.045, -0.032), (0.045, 0.032), (-0.045, 0.032))):
        part.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x + dx, y + dy, 0.173)),
            material=material,
            name=f"{name}_bolt_{index}",
        )


def _add_caster_yoke(part, material_steel: Material) -> None:
    trail_y = -0.075
    wheel_z = -0.125
    part.visual(
        Cylinder(radius=0.041, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=material_steel,
        name="swivel_disc",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=material_steel,
        name="kingpin",
    )
    part.visual(
        Box((0.030, 0.090, 0.012)),
        origin=Origin(xyz=(0.0, trail_y / 2.0, -0.053)),
        material=material_steel,
        name="offset_arm",
    )
    part.visual(
        Box((0.095, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, trail_y, -0.058)),
        material=material_steel,
        name="fork_crown",
    )
    for x, name in ((-0.034, "fork_cheek_0"), (0.034, "fork_cheek_1")):
        part.visual(
            Box((0.013, 0.034, 0.104)),
            origin=Origin(xyz=(x, trail_y, -0.110)),
            material=material_steel,
            name=name,
        )
    part.visual(
        Cylinder(radius=0.0055, length=0.090),
        origin=Origin(xyz=(0.0, trail_y, wheel_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material_steel,
        name="axle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_display_freezer")

    insulated = model.material("warm_insulated_white", rgba=(0.86, 0.91, 0.93, 1.0))
    liner = model.material("white_plastic_liner", rgba=(0.96, 0.98, 0.98, 1.0))
    blue_trim = model.material("deep_blue_trim", rgba=(0.02, 0.12, 0.30, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.76, 1.0))
    dark_rail = model.material("black_rail_insert", rgba=(0.02, 0.025, 0.03, 1.0))
    glass = model.material("pale_blue_glass", rgba=(0.55, 0.86, 1.0, 0.36))
    grip = model.material("dark_plastic_grip", rgba=(0.015, 0.015, 0.018, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    steel = model.material("zinc_plated_steel", rgba=(0.52, 0.55, 0.56, 1.0))
    hub_mat = model.material("grey_wheel_hub", rgba=(0.34, 0.36, 0.37, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "angled_insulated_cabinet", tolerance=0.002),
        material=insulated,
        name="insulated_shell",
    )

    # Smooth liner floor visible through the top opening.
    liner_floor = _polygon_prism(
        [(-0.56, -0.29), (0.49, -0.35), (0.57, 0.26), (-0.43, 0.38)],
        0.012,
        0.292,
    )
    cabinet.visual(
        mesh_from_cadquery(liner_floor, "liner_floor", tolerance=0.002),
        material=liner,
        name="liner_floor",
    )

    outer_edges = [
        ((-0.72, -0.46), (0.66, -0.54), "lower_front_bumper"),
        ((0.66, -0.54), (0.76, 0.40), "lower_side_bumper"),
        ((0.76, 0.40), (-0.56, 0.56), "lower_rear_bumper"),
        ((-0.56, 0.56), (-0.72, -0.46), "lower_back_bumper"),
    ]
    for p0, p1, name in outer_edges:
        _edge_bar(cabinet, p0, p1, z=0.345, width=0.038, height=0.060, material=blue_trim, name=name)
    for p0, p1, name in outer_edges:
        _edge_bar(cabinet, p0, p1, z=0.835, width=0.052, height=0.045, material=blue_trim, name=name.replace("lower", "top"))

    # Two offset guide-rail levels: the rear/high set lets one glass slider pass
    # over the other without occupying the same height.
    panel_x = 0.72
    panel_y = 0.58
    lower_y = -0.030
    upper_y = 0.030
    lower_rail_top = 0.860
    upper_rail_top = 0.934
    rail_h = 0.030
    rail_w = 0.026
    cabinet.visual(
        Box((1.18, rail_w, rail_h)),
        origin=Origin(xyz=(0.0, lower_y - panel_y / 2.0 - 0.015, lower_rail_top - rail_h / 2.0)),
        material=dark_rail,
        name="lower_rail_front",
    )
    cabinet.visual(
        Box((1.18, rail_w, rail_h)),
        origin=Origin(xyz=(0.0, lower_y + panel_y / 2.0 + 0.015, lower_rail_top - rail_h / 2.0)),
        material=dark_rail,
        name="lower_rail_rear",
    )
    cabinet.visual(
        Box((1.18, rail_w, rail_h)),
        origin=Origin(xyz=(0.0, upper_y - panel_y / 2.0 - 0.015, upper_rail_top - rail_h / 2.0)),
        material=dark_rail,
        name="upper_rail_front",
    )
    cabinet.visual(
        Box((1.18, rail_w, rail_h)),
        origin=Origin(xyz=(0.0, upper_y + panel_y / 2.0 + 0.015, upper_rail_top - rail_h / 2.0)),
        material=dark_rail,
        name="upper_rail_rear",
    )
    for x in (-0.57, 0.57):
        for y in (lower_y - panel_y / 2.0 - 0.015, lower_y + panel_y / 2.0 + 0.015):
            cabinet.visual(
                Box((0.040, 0.040, lower_rail_top - 0.830)),
                origin=Origin(xyz=(x, y, 0.830 + (lower_rail_top - 0.830) / 2.0)),
                material=aluminum,
                name=f"lower_rail_post_{len(cabinet.visuals)}",
            )
        if x > 0.0:
            for y in (upper_y - panel_y / 2.0 - 0.015, upper_y + panel_y / 2.0 + 0.015):
                cabinet.visual(
                    Box((0.040, 0.040, upper_rail_top - 0.830)),
                    origin=Origin(xyz=(x, y, 0.830 + (upper_rail_top - 0.830) / 2.0)),
                    material=aluminum,
                    name=f"upper_rail_post_{len(cabinet.visuals)}",
                )

    caster_xy = {
        "caster_0": (-0.55, -0.34),
        "caster_1": (0.50, -0.41),
        "caster_2": (0.56, 0.30),
        "caster_3": (-0.46, 0.42),
    }
    for name, xy in caster_xy.items():
        _add_caster_mount(cabinet, xy, steel, name)

    lower_lid = model.part("glass_lid_0")
    _glass_lid_part(
        lower_lid,
        panel_x=panel_x,
        panel_y=panel_y,
        material_glass=glass,
        material_metal=aluminum,
        material_grip=grip,
    )
    upper_lid = model.part("glass_lid_1")
    _glass_lid_part(
        upper_lid,
        panel_x=panel_x,
        panel_y=panel_y,
        material_glass=glass,
        material_metal=aluminum,
        material_grip=grip,
    )

    lower_center_z = lower_rail_top + 0.016
    upper_center_z = upper_rail_top + 0.016
    model.articulation(
        "lid_0_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_lid,
        origin=Origin(xyz=(-0.34, lower_y, lower_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.45, lower=0.0, upper=0.36),
    )
    model.articulation(
        "lid_1_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_lid,
        origin=Origin(xyz=(0.34, upper_y, upper_center_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.45, lower=0.0, upper=0.36),
    )

    tire_meshes = []
    hub_meshes = []
    for index in range(4):
        tire_meshes.append(
            mesh_from_geometry(
                TireGeometry(
                    0.055,
                    0.036,
                    inner_radius=0.034,
                    tread=TireTread(style="block", depth=0.0035, count=16, land_ratio=0.58),
                    grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
                    sidewall=TireSidewall(style="rounded", bulge=0.05),
                    shoulder=TireShoulder(width=0.004, radius=0.002),
                ),
                f"caster_tire_{index}",
            )
        )
        hub_meshes.append(
            mesh_from_geometry(
                WheelGeometry(
                    0.036,
                    0.040,
                    rim=WheelRim(inner_radius=0.024, flange_height=0.004, flange_thickness=0.002),
                    hub=WheelHub(
                        radius=0.015,
                        width=0.030,
                        cap_style="flat",
                        bolt_pattern=BoltPattern(count=4, circle_diameter=0.018, hole_diameter=0.0025),
                    ),
                    face=WheelFace(dish_depth=0.002, front_inset=0.0015, rear_inset=0.0015),
                    spokes=WheelSpokes(style="straight", count=5, thickness=0.002, window_radius=0.006),
                    bore=WheelBore(style="round", diameter=0.016),
                ),
                f"caster_hub_{index}",
            )
        )

    for index, (name, xy) in enumerate(caster_xy.items()):
        swivel = model.part(name)
        _add_caster_yoke(swivel, steel)
        wheel = model.part(f"wheel_{index}")
        wheel.visual(tire_meshes[index], material=rubber, name="tire")
        wheel.visual(hub_meshes[index], material=hub_mat, name="hub")
        model.articulation(
            f"{name}_swivel",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=swivel,
            origin=Origin(xyz=(xy[0], xy[1], 0.156)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=5.0),
        )
        model.articulation(
            f"wheel_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=swivel,
            child=wheel,
            origin=Origin(xyz=(0.0, -0.075, -0.125)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid_0 = object_model.get_part("glass_lid_0")
    lid_1 = object_model.get_part("glass_lid_1")
    slide_0 = object_model.get_articulation("lid_0_slide")
    slide_1 = object_model.get_articulation("lid_1_slide")

    ctx.expect_gap(
        lid_0,
        cabinet,
        axis="z",
        positive_elem="front_frame",
        negative_elem="lower_rail_front",
        min_gap=0.0,
        max_gap=0.002,
        name="lower glass slider sits on lower front rail",
    )
    ctx.expect_gap(
        lid_1,
        cabinet,
        axis="z",
        positive_elem="rear_frame",
        negative_elem="upper_rail_rear",
        min_gap=0.0,
        max_gap=0.002,
        name="upper glass slider sits on raised rear rail",
    )
    ctx.expect_overlap(
        lid_0,
        cabinet,
        axes="x",
        elem_a="front_frame",
        elem_b="lower_rail_front",
        min_overlap=0.55,
        name="lower slider is retained by its guide rail",
    )
    ctx.expect_overlap(
        lid_1,
        cabinet,
        axes="x",
        elem_a="rear_frame",
        elem_b="upper_rail_rear",
        min_overlap=0.55,
        name="upper slider is retained by its raised guide rail",
    )

    lower_rest = ctx.part_world_position(lid_0)
    upper_rest = ctx.part_world_position(lid_1)
    with ctx.pose({slide_0: 0.32, slide_1: 0.32}):
        lower_moved = ctx.part_world_position(lid_0)
        upper_moved = ctx.part_world_position(lid_1)
        ctx.expect_overlap(
            lid_0,
            cabinet,
            axes="x",
            elem_a="front_frame",
            elem_b="lower_rail_front",
            min_overlap=0.38,
            name="lower slider remains on rail when opened",
        )
        ctx.expect_overlap(
            lid_1,
            cabinet,
            axes="x",
            elem_a="rear_frame",
            elem_b="upper_rail_rear",
            min_overlap=0.38,
            name="upper slider remains on rail when opened",
        )
    ctx.check(
        "sliders move in opposite directions",
        lower_rest is not None
        and upper_rest is not None
        and lower_moved is not None
        and upper_moved is not None
        and lower_moved[0] > lower_rest[0] + 0.25
        and upper_moved[0] < upper_rest[0] - 0.25,
        details=f"lower_rest={lower_rest}, lower_moved={lower_moved}, upper_rest={upper_rest}, upper_moved={upper_moved}",
    )

    for index in range(4):
        swivel = object_model.get_articulation(f"caster_{index}_swivel")
        spin = object_model.get_articulation(f"wheel_{index}_spin")
        ctx.check(
            f"caster {index} has vertical swivel",
            swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
            details=f"type={swivel.articulation_type}, axis={swivel.axis}",
        )
        ctx.check(
            f"wheel {index} spins on horizontal axle",
            spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )

    wheel_0 = object_model.get_part("wheel_0")
    wheel_0_rest = ctx.part_world_position(wheel_0)
    with ctx.pose({"caster_0_swivel": math.pi / 2.0}):
        wheel_0_swiveled = ctx.part_world_position(wheel_0)
    ctx.check(
        "caster wheel trails around swivel axis",
        wheel_0_rest is not None
        and wheel_0_swiveled is not None
        and abs(wheel_0_swiveled[0] - wheel_0_rest[0]) > 0.025,
        details=f"rest={wheel_0_rest}, swiveled={wheel_0_swiveled}",
    )

    return ctx.report()


object_model = build_object_model()
