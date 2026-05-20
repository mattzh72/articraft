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


def _rounded_box(size: tuple[float, float, float], fillet: float) -> cq.Workplane:
    """A meter-scale rounded rectangular solid for molded plastic pieces."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(fillet)


def _x_cylinder(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder geometry/origin pair for an axle-like visual along local X."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _z_box_visual(part, size, xyz, name, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), name=name, material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_chest")

    plastic = model.material("charcoal_ribbed_plastic", color=(0.10, 0.11, 0.12, 1.0))
    dark = model.material("black_rubber", color=(0.015, 0.014, 0.013, 1.0))
    graphite = model.material("dark_graphite", color=(0.22, 0.23, 0.23, 1.0))
    yellow = model.material("yellow_latch_plastic", color=(0.98, 0.73, 0.08, 1.0))
    steel = model.material("brushed_steel", color=(0.70, 0.72, 0.70, 1.0))
    clear_smoke = model.material("smoked_clear_lid", color=(0.34, 0.45, 0.50, 0.42))

    chest = model.part("chest")

    # Mid-size portable job-site organizer body: a thick molded trunk with rounded
    # shoulders, a shallow top tray, raised ribs, latches, wheel pockets, and rear
    # slide sockets for the trolley handle.
    body_shape = _rounded_box((0.62, 0.38, 0.36), 0.035)
    chest.visual(
        mesh_from_cadquery(body_shape, "molded_body", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=plastic,
        name="molded_body",
    )

    tray_shape = _rounded_box((0.58, 0.34, 0.08), 0.025)
    chest.visual(
        mesh_from_cadquery(tray_shape, "top_tray", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
        material=plastic,
        name="top_tray",
    )

    # Ribbing on the plastic shell.  The ribs are slightly sunk into the body so
    # they read as molded-in raised ribs and remain one connected shell assembly.
    for i, z in enumerate((0.20, 0.27, 0.34, 0.41)):
        _z_box_visual(chest, (0.52, 0.018, 0.024), (0.0, -0.195, z), f"front_rib_{i}", graphite)
        _z_box_visual(chest, (0.52, 0.018, 0.020), (0.0, 0.195, z), f"rear_rib_{i}", graphite)
    for i, x in enumerate((-0.20, -0.10, 0.0, 0.10, 0.20)):
        _z_box_visual(chest, (0.018, 0.022, 0.23), (x, -0.197, 0.315), f"front_uprib_{i}", graphite)
    for side, x in (("neg", -0.317), ("pos", 0.317)):
        for i, z in enumerate((0.22, 0.31, 0.40)):
            _z_box_visual(chest, (0.018, 0.26, 0.022), (x, 0.0, z), f"side_rib_{side}_{i}", graphite)

    # Top tray organizer pockets under the smoked flip lid.
    _z_box_visual(chest, (0.52, 0.018, 0.025), (0.0, -0.178, 0.558), "front_organizer_lip", graphite)
    _z_box_visual(chest, (0.52, 0.018, 0.025), (0.0, 0.015, 0.558), "organizer_rear_lip", graphite)
    _z_box_visual(chest, (0.018, 0.20, 0.020), (-0.255, -0.070, 0.555), "organizer_side_lip_0", graphite)
    _z_box_visual(chest, (0.018, 0.20, 0.020), (0.255, -0.070, 0.555), "organizer_side_lip_1", graphite)
    _z_box_visual(chest, (0.012, 0.17, 0.016), (-0.085, -0.072, 0.553), "organizer_divider_0", graphite)
    _z_box_visual(chest, (0.012, 0.17, 0.016), (0.085, -0.072, 0.553), "organizer_divider_1", graphite)

    # Front draw latches and a recessed carry grip.
    _z_box_visual(chest, (0.075, 0.030, 0.055), (-0.18, -0.195, 0.495), "front_latch_0", yellow)
    _z_box_visual(chest, (0.075, 0.030, 0.055), (0.18, -0.195, 0.495), "front_latch_1", yellow)
    _z_box_visual(chest, (0.18, 0.025, 0.035), (0.0, -0.214, 0.445), "front_grip", graphite)

    # Rear trolley-handle sleeves: broad molded channels bolted to the back.
    for i, x in enumerate((-0.20, 0.20)):
        _z_box_visual(chest, (0.052, 0.060, 0.345), (x, 0.220, 0.350), f"rear_sleeve_{i}", graphite)
        _z_box_visual(chest, (0.080, 0.030, 0.040), (x, 0.192, 0.535), f"sleeve_collar_{i}", graphite)
    _z_box_visual(chest, (0.48, 0.020, 0.040), (0.0, 0.187, 0.185), "rear_lower_bridge", graphite)

    # Wheel pockets, axle support plates, and front molded feet.
    for i, x in enumerate((-0.318, 0.318)):
        _z_box_visual(chest, (0.030, 0.110, 0.130), (x, 0.130, 0.082), f"wheel_fork_{i}", graphite)
    _z_box_visual(chest, (0.14, 0.08, 0.04), (-0.20, -0.145, 0.090), "front_foot_0", dark)
    _z_box_visual(chest, (0.14, 0.08, 0.04), (0.20, -0.145, 0.090), "front_foot_1", dark)

    # Sliding trolley handle: two steel tubes tied together by a black hand grip.
    trolley_handle = model.part("trolley_handle")
    for i, x in enumerate((-0.20, 0.20)):
        trolley_handle.visual(
            Cylinder(radius=0.014, length=0.54),
            origin=Origin(xyz=(x, 0.222, 0.450)),
            material=steel,
            name=f"rail_{i}",
        )
        trolley_handle.visual(
            Cylinder(radius=0.020, length=0.080),
            origin=Origin(xyz=(x, 0.222, 0.140)),
            material=graphite,
            name=f"lower_stop_{i}",
        )

    grip_geom, grip_origin = _x_cylinder(0.026, 0.47)
    trolley_handle.visual(
        grip_geom,
        origin=Origin(xyz=(0.0, 0.222, 0.720), rpy=grip_origin.rpy),
        material=dark,
        name="hand_grip",
    )

    model.articulation(
        "chest_to_trolley_handle",
        ArticulationType.PRISMATIC,
        parent=chest,
        child=trolley_handle,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.30, effort=90.0, velocity=0.35),
    )

    # Smoked front organizer lid with an exposed rear hinge axis.  The lid's
    # local frame is the hinge pin line; its panel extends toward the front (-Y).
    organizer_lid = model.part("organizer_lid")
    organizer_lid.visual(
        Box((0.50, 0.210, 0.012)),
        origin=Origin(xyz=(0.0, -0.119, -0.003)),
        material=clear_smoke,
        name="clear_panel",
    )
    organizer_lid.visual(
        Box((0.43, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.125, 0.007)),
        material=graphite,
        name="lid_front_rib",
    )
    organizer_lid.visual(
        Box((0.090, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.236, 0.012)),
        material=graphite,
        name="pull_tab",
    )
    lid_barrel_geom, lid_barrel_origin = _x_cylinder(0.010, 0.145)
    organizer_lid.visual(
        lid_barrel_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=lid_barrel_origin.rpy),
        material=graphite,
        name="center_knuckle",
    )
    organizer_lid.visual(
        Box((0.120, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.014, -0.002)),
        material=graphite,
        name="knuckle_web",
    )
    organizer_lid.visual(
        Box((0.130, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, -0.060, -0.006)),
        material=graphite,
        name="hinge_leaf",
    )

    # Parent-side hinge knuckles on the tray, clear of the rotating center knuckle.
    for i, x in enumerate((-0.165, 0.165)):
        _z_box_visual(chest, (0.145, 0.020, 0.016), (x, 0.036, 0.558), f"tray_hinge_leaf_{i}", graphite)
        geom, ori = _x_cylinder(0.009, 0.145)
        chest.visual(
            geom,
            origin=Origin(xyz=(x, 0.045, 0.575), rpy=ori.rpy),
            material=graphite,
            name=f"tray_knuckle_{i}",
        )

    model.articulation(
        "chest_to_organizer_lid",
        ArticulationType.REVOLUTE,
        parent=chest,
        child=organizer_lid,
        origin=Origin(xyz=(0.0, 0.045, 0.575)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=8.0, velocity=2.0),
    )

    # Two utility wheels, each a tire and a molded rim, spin continuously about
    # their axle axes.  The axle brackets are on the fixed chest part.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.070,
            0.064,
            inner_radius=0.046,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.005, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "utility_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.047,
            0.052,
            rim=WheelRim(inner_radius=0.032, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.018,
                width=0.035,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "gray_wheel_rim",
    )

    for i, x in enumerate((-0.365, 0.365)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, material=dark, name="tire")
        wheel.visual(rim_mesh, material=steel, name="rim")
        model.articulation(
            f"chest_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=chest,
            child=wheel,
            origin=Origin(xyz=(x, 0.130, 0.075)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chest = object_model.get_part("chest")
    handle = object_model.get_part("trolley_handle")
    lid = object_model.get_part("organizer_lid")
    handle_slide = object_model.get_articulation("chest_to_trolley_handle")
    lid_hinge = object_model.get_articulation("chest_to_organizer_lid")

    # The handle rails are intentionally represented as rods captured inside
    # solid molded rear sleeves.  Scope the allowance to the two sleeve/rod fits
    # and prove retained insertion at collapsed and extended poses.
    for sleeve, rail in (("rear_sleeve_0", "rail_0"), ("rear_sleeve_1", "rail_1")):
        ctx.allow_overlap(
            chest,
            handle,
            elem_a=sleeve,
            elem_b=rail,
            reason="Trolley handle rail is intentionally captured inside its molded rear slide sleeve.",
        )
        ctx.expect_within(
            handle,
            chest,
            axes="xy",
            inner_elem=rail,
            outer_elem=sleeve,
            margin=0.002,
            name=f"{rail} centered in {sleeve}",
        )
        ctx.expect_overlap(
            handle,
            chest,
            axes="z",
            elem_a=rail,
            elem_b=sleeve,
            min_overlap=0.25,
            name=f"{rail} collapsed insertion",
        )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.30}):
        for sleeve, rail in (("rear_sleeve_0", "rail_0"), ("rear_sleeve_1", "rail_1")):
            ctx.expect_within(
                handle,
                chest,
                axes="xy",
                inner_elem=rail,
                outer_elem=sleeve,
                margin=0.002,
                name=f"{rail} stays in sleeve when extended",
            )
            ctx.expect_overlap(
                handle,
                chest,
                axes="z",
                elem_a=rail,
                elem_b=sleeve,
                min_overlap=0.03,
                name=f"{rail} extended retained insertion",
            )
        raised_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "trolley handle slides upward",
        rest_handle_pos is not None
        and raised_handle_pos is not None
        and raised_handle_pos[2] > rest_handle_pos[2] + 0.25,
        details=f"rest={rest_handle_pos}, raised={raised_handle_pos}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(lid, chest, axes="xy", elem_a="clear_panel", elem_b="top_tray", min_overlap=0.16)
        ctx.expect_gap(
            lid,
            chest,
            axis="z",
            positive_elem="clear_panel",
            negative_elem="organizer_divider_0",
            min_gap=0.002,
            max_gap=0.025,
            name="closed lid clears organizer dividers",
        )
        closed_panel_aabb = ctx.part_element_world_aabb(lid, elem="clear_panel")
    with ctx.pose({lid_hinge: 1.10}):
        raised_panel_aabb = ctx.part_element_world_aabb(lid, elem="clear_panel")
    ctx.check(
        "organizer lid hinge opens upward",
        closed_panel_aabb is not None
        and raised_panel_aabb is not None
        and raised_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.07,
        details=f"closed_panel={closed_panel_aabb}, raised_panel={raised_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
