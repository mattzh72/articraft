from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel")

    painted_steel = model.material("painted_steel", rgba=(0.74, 0.76, 0.74, 1.0))
    frame_paint = model.material("satin_frame", rgba=(0.34, 0.37, 0.38, 1.0))
    door_paint = model.material("powder_coated_door", rgba=(0.60, 0.63, 0.62, 1.0))
    dark_cavity = model.material("shadowed_service_bay", rgba=(0.025, 0.028, 0.030, 1.0))
    rubber = model.material("black_rubber", rgba=(0.012, 0.012, 0.012, 1.0))
    brushed_metal = model.material("brushed_stainless", rgba=(0.76, 0.74, 0.68, 1.0))
    latch_metal = model.material("dark_zinc_latch", rgba=(0.17, 0.18, 0.18, 1.0))
    label_yellow = model.material("service_label_yellow", rgba=(0.95, 0.72, 0.16, 1.0))

    # The root link represents the surrounding piece of equipment or wall.  It is
    # a real trim ring set into a larger skin rather than a freestanding door.
    housing = model.part("housing")

    skin_outer = rounded_rect_profile(0.90, 1.10, 0.025, corner_segments=8)
    skin_opening = rounded_rect_profile(0.66, 0.86, 0.020, corner_segments=8)
    skin_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(skin_outer, [skin_opening], 0.022, center=True),
        "equipment_skin",
    )
    housing.visual(
        skin_mesh,
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="equipment_skin",
    )

    service_bezel = BezelGeometry(
        (0.510, 0.710),
        (0.640, 0.840),
        0.052,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.016,
        outer_corner_radius=0.024,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
    )
    housing.visual(
        mesh_from_geometry(service_bezel, "service_bezel"),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="service_bezel",
    )

    # Recessed bay surfaces become visible when the door is open.
    housing.visual(
        Box((0.010, 0.064, 0.705)),
        origin=Origin(xyz=(-0.257, -0.038, 0.0)),
        material=dark_cavity,
        name="bay_left_wall",
    )
    housing.visual(
        Box((0.010, 0.064, 0.705)),
        origin=Origin(xyz=(0.257, -0.038, 0.0)),
        material=dark_cavity,
        name="bay_right_wall",
    )
    housing.visual(
        Box((0.514, 0.064, 0.010)),
        origin=Origin(xyz=(0.0, -0.038, 0.357)),
        material=dark_cavity,
        name="bay_top_wall",
    )
    housing.visual(
        Box((0.514, 0.064, 0.010)),
        origin=Origin(xyz=(0.0, -0.038, -0.357)),
        material=dark_cavity,
        name="bay_bottom_wall",
    )
    housing.visual(
        Box((0.524, 0.010, 0.714)),
        origin=Origin(xyz=(0.0, -0.073, 0.0)),
        material=dark_cavity,
        name="bay_back_panel",
    )

    hinge_x = -0.272
    hinge_y = 0.040
    hinge_radius = 0.009

    for idx, (z_center, length) in enumerate(((-0.265, 0.112), (0.0, 0.126), (0.265, 0.112))):
        housing.visual(
            Box((0.038, 0.006, length)),
            origin=Origin(xyz=(hinge_x - 0.021, hinge_y, z_center)),
            material=brushed_metal,
            name=f"fixed_hinge_leaf_{idx}",
        )
    housing.visual(
        Box((0.018, 0.028, 0.690)),
        origin=Origin(xyz=(hinge_x - 0.038, 0.027, 0.0)),
        material=frame_paint,
        name="hinge_standoff",
    )
    for idx, (z_center, length) in enumerate(((-0.265, 0.115), (0.0, 0.130), (0.265, 0.115))):
        housing.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(xyz=(hinge_x, hinge_y, z_center)),
            material=brushed_metal,
            name=f"fixed_knuckle_{idx}",
        )
    housing.visual(
        Cylinder(radius=0.0045, length=0.650),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        material=brushed_metal,
        name="hinge_pin",
    )
    for idx, z_center in enumerate((-0.3295, 0.3295)):
        housing.visual(
            Cylinder(radius=0.0105, length=0.014),
            origin=Origin(xyz=(hinge_x, hinge_y, z_center)),
            material=brushed_metal,
            name=f"hinge_pin_cap_{idx}",
        )

    # Flush fasteners on the frame make the panel read as bolted into equipment.
    for idx, (x, z) in enumerate(((-0.275, 0.365), (0.275, 0.365), (-0.275, -0.365), (0.275, -0.365))):
        housing.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, 0.016, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"frame_screw_{idx}",
        )

    housing.visual(
        Box((0.014, 0.026, 0.128)),
        origin=Origin(xyz=(0.270, 0.022, 0.050)),
        material=brushed_metal,
        name="latch_strike",
    )

    # The door link frame is on the vertical hinge line.  All door visuals extend
    # along +X from this frame, so +Z revolute motion opens the free edge outward.
    door = model.part("door")
    door_w = 0.488
    door_h = 0.688
    door_t = 0.026
    door_center_x = 0.272

    door_profile = rounded_rect_profile(door_w, door_h, 0.018, corner_segments=10)
    door_shell = ExtrudeGeometry.centered(door_profile, door_t)
    door_shell.rotate_x(math.pi / 2.0).translate(door_center_x, 0.0, 0.0)
    door.visual(
        mesh_from_geometry(door_shell, "door_panel"),
        material=door_paint,
        name="door_panel",
    )

    # Raised gasket and formed ribs are attached to the panel face.
    door.visual(
        Box((door_w - 0.070, 0.004, 0.010)),
        origin=Origin(xyz=(door_center_x, 0.015, door_h / 2.0 - 0.028)),
        material=rubber,
        name="top_gasket",
    )
    door.visual(
        Box((door_w - 0.070, 0.004, 0.010)),
        origin=Origin(xyz=(door_center_x, 0.015, -door_h / 2.0 + 0.028)),
        material=rubber,
        name="bottom_gasket",
    )
    door.visual(
        Box((0.010, 0.004, door_h - 0.080)),
        origin=Origin(xyz=(door_center_x - door_w / 2.0 + 0.030, 0.015, 0.0)),
        material=rubber,
        name="hinge_gasket",
    )
    door.visual(
        Box((0.010, 0.004, door_h - 0.080)),
        origin=Origin(xyz=(door_center_x + door_w / 2.0 - 0.030, 0.015, 0.0)),
        material=rubber,
        name="latch_gasket",
    )
    for idx, z in enumerate((-0.180, 0.180)):
        door.visual(
            Box((door_w - 0.120, 0.006, 0.018)),
            origin=Origin(xyz=(door_center_x, -0.016, z)),
            material=frame_paint,
            name=f"inner_rib_{idx}",
        )
    door.visual(
        Box((0.018, 0.006, door_h - 0.160)),
        origin=Origin(xyz=(door_center_x + 0.145, -0.016, 0.0)),
        material=frame_paint,
        name="inner_vertical_rib",
    )

    door.visual(
        Box((0.160, 0.006, 0.045)),
        origin=Origin(xyz=(door_center_x + 0.020, 0.0145, 0.215)),
        material=label_yellow,
        name="service_label",
    )
    for idx, x_offset in enumerate((-0.050, 0.0, 0.050)):
        door.visual(
            Box((0.026, 0.005, 0.034)),
            origin=Origin(xyz=(door_center_x + 0.020 + x_offset, 0.018, 0.215)),
            material=latch_metal,
            name=f"label_mark_{idx}",
        )

    for idx, (x, z) in enumerate(
        (
            (door_center_x - 0.175, 0.255),
            (door_center_x + 0.175, 0.255),
            (door_center_x - 0.175, -0.255),
            (door_center_x + 0.175, -0.255),
        )
    ):
        door.visual(
            Cylinder(radius=0.008, length=0.005),
            origin=Origin(xyz=(x, 0.0145, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"door_screw_{idx}",
        )

    for idx, z_center in enumerate((-0.135, 0.135)):
        door.visual(
            Box((0.046, 0.006, 0.118)),
            origin=Origin(xyz=(0.023, 0.000, z_center)),
            material=brushed_metal,
            name=f"moving_hinge_leaf_{idx}",
        )
    for idx, z_center in enumerate((-0.135, 0.135)):
        door.visual(
            Cylinder(radius=hinge_radius, length=0.120),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=brushed_metal,
            name=f"moving_knuckle_{idx}",
        )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=1.85),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="latch_hub",
    )
    latch.visual(
        Box((0.030, 0.012, 0.132)),
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material=latch_metal,
        name="handle_bar",
    )
    latch.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="handle_button",
    )
    latch.visual(
        Box((0.018, 0.006, 0.086)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=brushed_metal,
        name="latch_cam",
    )
    latch.visual(
        Cylinder(radius=0.0065, length=0.036),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="latch_spindle",
    )

    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(door_center_x + 0.150, door_t / 2.0, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    # Keep an explicit reference so static type checkers do not consider the
    # primary mechanism unused when this script is inspected standalone.
    assert door_hinge is not None
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    ctx.allow_overlap(
        latch,
        door,
        elem_a="latch_spindle",
        elem_b="door_panel",
        reason="The quarter-turn latch spindle intentionally passes through the simplified solid door leaf.",
    )
    for knuckle_name in ("moving_knuckle_0", "moving_knuckle_1"):
        ctx.allow_overlap(
            housing,
            door,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            reason="The exposed hinge pin is intentionally captured inside the moving hinge knuckle.",
        )
        ctx.expect_within(
            housing,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=knuckle_name,
            margin=0.0,
            name=f"{knuckle_name} surrounds the hinge pin",
        )
        ctx.expect_overlap(
            housing,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            min_overlap=0.100,
            name=f"{knuckle_name} has retained length on hinge pin",
        )
    ctx.expect_within(
        latch,
        door,
        axes="xz",
        inner_elem="latch_spindle",
        outer_elem="door_panel",
        margin=0.0,
        name="latch spindle is centered inside the door leaf footprint",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="y",
        elem_a="latch_spindle",
        elem_b="door_panel",
        min_overlap=0.020,
        name="latch spindle visibly passes through the door thickness",
    )

    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="door_panel",
        negative_elem="service_bezel",
        min_gap=0.006,
        max_gap=0.025,
        name="closed door sits just proud of the recessed frame",
    )

    door_box = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door leaf fits inside the visible service opening",
        door_box is not None
        and door_box[0][0] > -0.255
        and door_box[1][0] < 0.255
        and door_box[0][2] > -0.355
        and door_box[1][2] < 0.355,
        details=f"door_panel_aabb={door_box}",
    )

    ctx.expect_gap(
        latch,
        door,
        axis="y",
        positive_elem="latch_hub",
        negative_elem="door_panel",
        min_gap=0.0,
        max_gap=0.002,
        name="quarter-turn latch hub is seated on the door face",
    )

    closed_box = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.35}):
        open_box = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "hinged service door opens outward from the frame",
        closed_box is not None and open_box is not None and open_box[1][1] > closed_box[1][1] + 0.28,
        details=f"closed={closed_box}, open={open_box}",
    )

    rest_handle = ctx.part_element_world_aabb(latch, elem="handle_bar")
    with ctx.pose({latch_turn: math.pi / 2.0}):
        turned_handle = ctx.part_element_world_aabb(latch, elem="handle_bar")
    ctx.check(
        "quarter-turn latch rotates from vertical to horizontal",
        rest_handle is not None
        and turned_handle is not None
        and (turned_handle[1][0] - turned_handle[0][0]) > (rest_handle[1][0] - rest_handle[0][0]) + 0.07,
        details=f"rest={rest_handle}, turned={turned_handle}",
    )

    return ctx.report()


object_model = build_object_model()
