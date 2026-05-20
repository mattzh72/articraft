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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sff_pc_case")

    body_mat = Material("anodized_dark_aluminum", rgba=(0.045, 0.047, 0.050, 1.0))
    lid_mat = Material("slate_vented_aluminum", rgba=(0.070, 0.074, 0.080, 1.0))
    hinge_mat = Material("blackened_steel_hinges", rgba=(0.010, 0.010, 0.012, 1.0))
    port_mat = Material("black_port_cavities", rgba=(0.0, 0.0, 0.0, 1.0))
    foot_mat = Material("soft_rubber_feet", rgba=(0.012, 0.011, 0.010, 1.0))
    accent_mat = Material("brushed_edge_highlights", rgba=(0.32, 0.33, 0.35, 1.0))

    width = 0.360
    depth = 0.280
    case_height = 0.106
    wall = 0.008

    chassis = model.part("chassis")
    chassis.visual(
        Box((width, depth, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=body_mat,
        name="bottom_tray",
    )
    chassis.visual(
        Box((wall, depth, 0.102)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, 0.055)),
        material=body_mat,
        name="side_wall_0",
    )
    chassis.visual(
        Box((wall, depth, 0.102)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, 0.055)),
        material=body_mat,
        name="side_wall_1",
    )
    chassis.visual(
        Box((width, wall, 0.102)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, 0.055)),
        material=body_mat,
        name="rear_wall",
    )
    chassis.visual(
        Box((width, wall, 0.102)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, 0.055)),
        material=body_mat,
        name="front_wall",
    )
    chassis.visual(
        Box((width - 0.018, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.006, case_height + 0.002)),
        material=accent_mat,
        name="front_top_lip",
    )
    chassis.visual(
        Box((width - 0.018, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.006, case_height + 0.002)),
        material=accent_mat,
        name="rear_top_lip",
    )

    # Dark inset side ventilation fields read as the perforated air paths of a
    # slim living-room SFF case while remaining mounted to the side walls.
    for idx, x in enumerate((-width / 2.0 - 0.001, width / 2.0 + 0.001)):
        chassis.visual(
            Box((0.002, 0.150, 0.048)),
            origin=Origin(xyz=(x, 0.000, 0.058)),
            material=port_mat,
            name=f"side_vent_shadow_{idx}",
        )
        for slot_i in range(6):
            chassis.visual(
                Box((0.003, 0.100, 0.003)),
                origin=Origin(xyz=(x, -0.002, 0.039 + slot_i * 0.007)),
                material=accent_mat,
                name=f"side_vent_slit_{idx}_{slot_i}",
            )

    # Four low rubber feet keep the case visibly grounded and raise the slim
    # body slightly like a real media-center PC enclosure.
    for ix, x in enumerate((-0.135, 0.135)):
        for iy, y in enumerate((-0.095, 0.095)):
            chassis.visual(
                Cylinder(radius=0.015, length=0.008),
                origin=Origin(xyz=(x, y, -0.004)),
                material=foot_mat,
                name=f"foot_{ix}_{iy}",
            )

    # Two exposed hinge barrels mounted to the rear wall.  The top lid uses one
    # physical revolute joint along the common rear-edge hinge line, with two
    # separated hinge sets shown in the geometry.
    top_hinge_y = depth / 2.0 + 0.002
    top_hinge_z = case_height + 0.004
    for idx, hinge_x in enumerate((-0.085, 0.085)):
        chassis.visual(
            Cylinder(radius=0.0045, length=0.020),
            origin=Origin(xyz=(hinge_x, top_hinge_y, top_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_mat,
            name=f"top_hinge_body_knuckle_{idx}",
        )
        chassis.visual(
            Box((0.058, 0.004, 0.020)),
            origin=Origin(xyz=(hinge_x, depth / 2.0 + 0.0005, case_height - 0.006)),
            material=hinge_mat,
            name=f"top_hinge_body_leaf_{idx}",
        )

    # Fixed knuckles for the front I/O panel's vertical side hinge.
    io_hinge_x = -0.145
    io_hinge_y = -depth / 2.0 - 0.004
    io_hinge_z = 0.060
    chassis.visual(
        Box((0.012, 0.004, 0.074)),
        origin=Origin(xyz=(io_hinge_x - 0.006, -depth / 2.0 - 0.001, io_hinge_z)),
        material=hinge_mat,
        name="io_hinge_body_leaf",
    )
    for z in (io_hinge_z - 0.026, io_hinge_z + 0.026):
        chassis.visual(
            Cylinder(radius=0.0042, length=0.018),
            origin=Origin(xyz=(io_hinge_x, io_hinge_y, z)),
            material=hinge_mat,
            name=f"io_hinge_body_knuckle_{0 if z < io_hinge_z else 1}",
        )

    # The top cover is a thin removable aluminum lid with real slots rather
    # than a solid block.  Its local frame is on the rear hinge line.
    lid = model.part("top_lid")
    top_vent_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.368, 0.282),
            0.006,
            slot_size=(0.052, 0.006),
            pitch=(0.068, 0.018),
            frame=0.020,
            corner_radius=0.006,
            stagger=True,
        ),
        "top_lid_slot_panel",
    )
    lid.visual(
        top_vent_mesh,
        origin=Origin(xyz=(0.0, -0.141, 0.002)),
        material=lid_mat,
        name="vented_top",
    )
    lid.visual(
        Box((0.366, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.284, -0.004)),
        material=lid_mat,
        name="front_return_lip",
    )
    lid.visual(
        Box((0.006, 0.284, 0.014)),
        origin=Origin(xyz=(-0.184, -0.142, -0.004)),
        material=lid_mat,
        name="side_return_0",
    )
    lid.visual(
        Box((0.006, 0.284, 0.014)),
        origin=Origin(xyz=(0.184, -0.142, -0.004)),
        material=lid_mat,
        name="side_return_1",
    )
    lid.visual(
        Box((0.060, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, -0.284, 0.0045)),
        material=accent_mat,
        name="front_fingernail_bevel",
    )
    for idx, hinge_x in enumerate((-0.085, 0.085)):
        for side, offset in enumerate((-0.021, 0.021)):
            lid.visual(
                Cylinder(radius=0.0043, length=0.018),
                origin=Origin(xyz=(hinge_x + offset, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=hinge_mat,
                name=f"top_hinge_lid_knuckle_{idx}_{side}",
            )
        lid.visual(
            Box((0.058, 0.024, 0.003)),
            origin=Origin(xyz=(hinge_x, -0.015, 0.004)),
            material=hinge_mat,
            name=f"top_hinge_lid_leaf_{idx}",
        )

    model.articulation(
        "chassis_to_top_lid",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=lid,
        origin=Origin(xyz=(0.0, depth / 2.0, top_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    io_panel = model.part("io_panel")
    io_panel.visual(
        Box((0.118, 0.006, 0.064)),
        origin=Origin(xyz=(0.059, -0.003, 0.0)),
        material=lid_mat,
        name="io_panel_face",
    )
    io_panel.visual(
        Cylinder(radius=0.0039, length=0.026),
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
        material=hinge_mat,
        name="io_hinge_panel_knuckle",
    )
    io_panel.visual(
        Box((0.008, 0.004, 0.070)),
        origin=Origin(xyz=(0.004, -0.003, 0.0)),
        material=hinge_mat,
        name="io_hinge_panel_leaf",
    )
    # Visible I/O details: twin USB-A ports, one USB-C port, an audio jack, and
    # a small status light are all mounted flush in the swinging panel.
    for idx, x in enumerate((0.038, 0.064)):
        io_panel.visual(
            Box((0.018, 0.002, 0.008)),
            origin=Origin(xyz=(x, -0.007, 0.010)),
            material=port_mat,
            name=f"usb_a_port_{idx}",
        )
    io_panel.visual(
        Box((0.017, 0.002, 0.005)),
        origin=Origin(xyz=(0.091, -0.007, 0.010)),
        material=port_mat,
        name="usb_c_port",
    )
    io_panel.visual(
        Cylinder(radius=0.0042, length=0.002),
        origin=Origin(xyz=(0.037, -0.007, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=port_mat,
        name="audio_jack",
    )
    io_panel.visual(
        Cylinder(radius=0.0032, length=0.002),
        origin=Origin(xyz=(0.092, -0.007, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=Material("cool_white_led", rgba=(0.65, 0.85, 1.0, 1.0)),
        name="status_led_lens",
    )

    model.articulation(
        "chassis_to_io_panel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=io_panel,
        origin=Origin(xyz=(io_hinge_x, -depth / 2.0 - 0.002, io_hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.2, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    lid = object_model.get_part("top_lid")
    io_panel = object_model.get_part("io_panel")
    lid_hinge = object_model.get_articulation("chassis_to_top_lid")
    io_hinge = object_model.get_articulation("chassis_to_io_panel")

    with ctx.pose({lid_hinge: 0.0, io_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            chassis,
            axis="z",
            min_gap=0.0005,
            max_gap=0.004,
            positive_elem="vented_top",
            negative_elem="side_wall_0",
            name="closed top lid sits just above side rail",
        )
        ctx.expect_overlap(
            lid,
            chassis,
            axes="xy",
            elem_a="vented_top",
            elem_b="bottom_tray",
            min_overlap=0.24,
            name="top lid covers the compact chassis footprint",
        )
        ctx.expect_gap(
            chassis,
            io_panel,
            axis="y",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="front_wall",
            negative_elem="io_panel_face",
            name="closed io panel is flush ahead of front wall",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="front_return_lip")
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="front_return_lip")
    ctx.check(
        "top lid rotates upward from rear hinges",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.16,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_io_aabb = ctx.part_element_world_aabb(io_panel, elem="io_panel_face")
    with ctx.pose({io_hinge: 1.25}):
        open_io_aabb = ctx.part_element_world_aabb(io_panel, elem="io_panel_face")
    ctx.check(
        "front io panel swings outward on vertical side hinge",
        closed_io_aabb is not None
        and open_io_aabb is not None
        and open_io_aabb[0][1] < closed_io_aabb[0][1] - 0.050,
        details=f"closed={closed_io_aabb}, open={open_io_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
