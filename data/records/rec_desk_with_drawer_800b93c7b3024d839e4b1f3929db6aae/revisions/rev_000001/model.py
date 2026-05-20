from __future__ import annotations

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
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_desk")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.020, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.075, 0.055, 0.040, 1.0))
    charcoal = model.material("charcoal_metal", rgba=(0.10, 0.105, 0.115, 1.0))
    rail_metal = model.material("brushed_rail", rgba=(0.44, 0.46, 0.48, 1.0))
    drawer_gray = model.material("drawer_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    accent_red = model.material("red_edge_light", rgba=(0.95, 0.04, 0.025, 1.0))

    desk = model.part("desk")

    # A real gaming desk is broad and shallow with softened corners and cable
    # grommets near the rear edge.  The top is a single continuous slab.
    desktop_width = 1.60
    desktop_depth = 0.75
    desktop_thickness = 0.045
    desktop_z = 0.740
    top_shape = (
        cq.Workplane("XY")
        .box(desktop_width, desktop_depth, desktop_thickness)
        .edges("|Z")
        .fillet(0.035)
        .pushPoints([(-0.48, 0.27), (0.32, 0.27)])
        .circle(0.035)
        .cutThruAll()
        .edges(">Z")
        .fillet(0.004)
    )
    desk.visual(
        mesh_from_cadquery(top_shape, "rounded_cable_desktop", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, desktop_z)),
        material=dark_wood,
        name="top_slab",
    )

    # Low red accent strip on the user's edge, seated into the top perimeter.
    desk.visual(
        Box((1.46, 0.012, 0.012)),
        origin=Origin(xyz=(-0.02, -0.379, desktop_z - 0.005)),
        material=accent_red,
        name="front_light_strip",
    )

    # Left steel legs and rear stretcher.  The right side is supported by the
    # pedestal, so the asymmetry matches a single-pedestal computer desk.
    for i, (x, y) in enumerate(((-0.70, -0.30), (-0.70, 0.30))):
        desk.visual(
            Box((0.055, 0.055, 0.705)),
            origin=Origin(xyz=(x, y, 0.365)),
            material=charcoal,
            name=f"left_leg_{i}",
        )
    desk.visual(
        Box((1.22, 0.038, 0.055)),
        origin=Origin(xyz=(-0.08, 0.335, 0.595)),
        material=charcoal,
        name="rear_stretcher",
    )
    desk.visual(
        Box((0.090, 0.030, 0.055)),
        origin=Origin(xyz=(-0.70, 0.335, 0.595)),
        material=charcoal,
        name="leg_stretcher_overlap",
    )

    # Single pedestal carcass: side panels, bottom/top caps, and a back panel
    # leave the front open for the sliding drawer.
    ped_x = 0.52
    ped_y = 0.02
    ped_w = 0.44
    ped_d = 0.60
    side_t = 0.026
    panel_t = 0.030
    ped_z_min = 0.060
    ped_h = 0.645
    ped_z = ped_z_min + ped_h / 2.0
    for side, x in (("side_0", ped_x - ped_w / 2 + side_t / 2), ("side_1", ped_x + ped_w / 2 - side_t / 2)):
        desk.visual(
            Box((side_t, ped_d, ped_h)),
            origin=Origin(xyz=(x, ped_y, ped_z)),
            material=matte_black,
            name=f"pedestal_{side}",
        )
    desk.visual(
        Box((ped_w, ped_d, panel_t)),
        origin=Origin(xyz=(ped_x, ped_y, ped_z_min + panel_t / 2)),
        material=matte_black,
        name="pedestal_bottom",
    )
    desk.visual(
        Box((ped_w, ped_d, panel_t)),
        origin=Origin(xyz=(ped_x, ped_y, ped_z_min + ped_h - panel_t / 2)),
        material=matte_black,
        name="pedestal_top",
    )
    desk.visual(
        Box((ped_w, panel_t, ped_h)),
        origin=Origin(xyz=(ped_x, ped_y + ped_d / 2 - panel_t / 2, ped_z)),
        material=matte_black,
        name="pedestal_back",
    )

    # Drawer rails are fixed to the pedestal sides and visibly guide the drawer.
    for side, x in (("rail_0", 0.326), ("rail_1", 0.714)):
        desk.visual(
            Box((0.018, 0.470, 0.025)),
            origin=Origin(xyz=(x, -0.030, 0.505)),
            material=rail_metal,
            name=f"drawer_{side}",
        )

    # Cable-tray slides and their hanger tabs sit under the front edge of the top.
    for side, x in (("rail_0", -0.630), ("rail_1", 0.230)):
        desk.visual(
            Box((0.026, 0.430, 0.026)),
            origin=Origin(xyz=(x, -0.190, 0.666)),
            material=rail_metal,
            name=f"cable_{side}",
        )
        for j, y in enumerate((-0.325, -0.055)):
            desk.visual(
                Box((0.026, 0.020, 0.070)),
                origin=Origin(xyz=(x, y, 0.700)),
                material=rail_metal,
                name=f"cable_hanger_{side}_{j}",
            )

    # Pull-out cable management tray: shallow U-channel pan with long hidden
    # runners so the tray remains captured when pulled forward.
    cable_tray = model.part("cable_tray")
    cable_tray.visual(
        Box((0.800, 0.250, 0.012)),
        origin=Origin(xyz=(0.0, 0.015, -0.035)),
        material=charcoal,
        name="tray_floor",
    )
    cable_tray.visual(
        Box((0.018, 0.250, 0.060)),
        origin=Origin(xyz=(-0.400, 0.015, -0.005)),
        material=charcoal,
        name="tray_side_0",
    )
    cable_tray.visual(
        Box((0.018, 0.250, 0.060)),
        origin=Origin(xyz=(0.400, 0.015, -0.005)),
        material=charcoal,
        name="tray_side_1",
    )
    cable_tray.visual(
        Box((0.800, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.110, -0.010)),
        material=charcoal,
        name="tray_front_lip",
    )
    cable_tray.visual(
        Box((0.800, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.140, -0.015)),
        material=charcoal,
        name="tray_back_lip",
    )
    for side, x in (("runner_0", -0.435), ("runner_1", 0.435)):
        cable_tray.visual(
            Box((0.020, 0.500, 0.020)),
            origin=Origin(xyz=(x, 0.045, 0.001)),
            material=rail_metal,
            name=f"tray_{side}",
        )
        web_x = -0.418 if x < 0 else 0.418
        cable_tray.visual(
            Box((0.034, 0.430, 0.012)),
            origin=Origin(xyz=(web_x, 0.045, -0.001)),
            material=charcoal,
            name=f"tray_web_{side}",
        )
    cable_tray.visual(
        Box((0.230, 0.024, 0.028)),
        origin=Origin(xyz=(0.0, -0.148, -0.006)),
        material=accent_red,
        name="tray_pull_grip",
    )
    for i, x in enumerate((-0.075, 0.075)):
        cable_tray.visual(
            Box((0.026, 0.040, 0.018)),
            origin=Origin(xyz=(x, -0.129, -0.006)),
            material=accent_red,
            name=f"tray_pull_stem_{i}",
        )

    model.articulation(
        "desk_to_cable_tray",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=cable_tray,
        origin=Origin(xyz=(-0.200, -0.220, 0.642)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.280),
    )

    # The pedestal drawer is a separate sliding box, with front slab, side walls,
    # an open top, a rear wall, side runners, and a horizontal pull.
    drawer = model.part("drawer")
    drawer.visual(
        Box((0.330, 0.430, 0.018)),
        origin=Origin(xyz=(0.0, -0.005, -0.115)),
        material=drawer_gray,
        name="drawer_floor",
    )
    for side, x in (("side_0", -0.174), ("side_1", 0.174)):
        drawer.visual(
            Box((0.018, 0.430, 0.190)),
            origin=Origin(xyz=(x, -0.005, -0.035)),
            material=drawer_gray,
            name=f"drawer_{side}",
        )
    drawer.visual(
        Box((0.330, 0.018, 0.190)),
        origin=Origin(xyz=(0.0, 0.210, -0.035)),
        material=drawer_gray,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.380, 0.030, 0.300)),
        origin=Origin(xyz=(0.0, -0.235, 0.000)),
        material=matte_black,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.240, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.258, 0.035)),
        material=accent_red,
        name="drawer_pull",
    )
    for side, x in (("runner_0", -0.188), ("runner_1", 0.188)):
        drawer.visual(
            Box((0.010, 0.530, 0.020)),
            origin=Origin(xyz=(x, -0.020, -0.012)),
            material=rail_metal,
            name=f"drawer_{side}",
        )
        web_x = -0.179 if x < 0 else 0.179
        drawer.visual(
            Box((0.010, 0.430, 0.012)),
            origin=Origin(xyz=(web_x, -0.020, -0.012)),
            material=drawer_gray,
            name=f"drawer_web_{side}",
        )

    model.articulation(
        "desk_to_drawer",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=drawer,
        origin=Origin(xyz=(0.520, -0.045, 0.490)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.28, lower=0.0, upper=0.300),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk = object_model.get_part("desk")
    cable_tray = object_model.get_part("cable_tray")
    drawer = object_model.get_part("drawer")
    cable_slide = object_model.get_articulation("desk_to_cable_tray")
    drawer_slide = object_model.get_articulation("desk_to_drawer")

    ctx.check(
        "two independent prismatic slides",
        cable_slide.articulation_type == ArticulationType.PRISMATIC
        and drawer_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"cable={cable_slide.articulation_type}, drawer={drawer_slide.articulation_type}",
    )

    ctx.expect_gap(
        desk,
        cable_tray,
        axis="z",
        positive_elem="top_slab",
        negative_elem="tray_side_0",
        min_gap=0.020,
        max_gap=0.080,
        name="cable tray hangs below desktop",
    )
    ctx.expect_overlap(
        cable_tray,
        desk,
        axes="xy",
        elem_a="tray_floor",
        elem_b="top_slab",
        min_overlap=0.15,
        name="cable tray stores under front work surface",
    )
    ctx.expect_overlap(
        cable_tray,
        desk,
        axes="y",
        elem_a="tray_runner_0",
        elem_b="cable_rail_0",
        min_overlap=0.18,
        name="closed cable tray runner is retained by rail",
    )

    cable_rest = ctx.part_world_position(cable_tray)
    with ctx.pose({cable_slide: 0.280}):
        cable_extended = ctx.part_world_position(cable_tray)
        ctx.expect_overlap(
            cable_tray,
            desk,
            axes="y",
            elem_a="tray_runner_0",
            elem_b="cable_rail_0",
            min_overlap=0.10,
            name="extended cable tray keeps runner engagement",
        )
    ctx.check(
        "cable tray slides toward user",
        cable_rest is not None
        and cable_extended is not None
        and cable_extended[1] < cable_rest[1] - 0.20,
        details=f"rest={cable_rest}, extended={cable_extended}",
    )

    ctx.expect_overlap(
        drawer,
        desk,
        axes="y",
        elem_a="drawer_runner_0",
        elem_b="drawer_rail_0",
        min_overlap=0.20,
        name="closed drawer runner is on pedestal rail",
    )
    ctx.expect_within(
        drawer,
        desk,
        axes="x",
        inner_elem="drawer_floor",
        outer_elem="pedestal_top",
        margin=0.010,
        name="drawer fits between pedestal side panels",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.300}):
        drawer_extended = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a="drawer_runner_0",
            elem_b="drawer_rail_0",
            min_overlap=0.12,
            name="extended drawer remains captured on rail",
        )
    ctx.check(
        "pedestal drawer slides outward",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] < drawer_rest[1] - 0.24,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    return ctx.report()


object_model = build_object_model()
