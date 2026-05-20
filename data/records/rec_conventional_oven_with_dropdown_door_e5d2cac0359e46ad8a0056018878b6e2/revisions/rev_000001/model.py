from __future__ import annotations

from math import pi

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
    mesh_from_cadquery,
)


def _toaster_body_shell() -> cq.Workplane:
    """Single-piece rectangular oven shell with a real front cavity opening."""
    width = 0.50
    depth = 0.34
    height = 0.28
    front_y = -depth / 2.0
    rear_wall = 0.026

    opening_w = 0.36
    opening_x = -0.055
    opening_bottom = 0.045
    opening_h = 0.205

    outer = cq.Workplane("XY").box(width, depth, height).translate((0.0, 0.0, height / 2.0))

    cavity_depth = depth - rear_wall + 0.050
    cavity = (
        cq.Workplane("XY")
        .box(opening_w, cavity_depth, opening_h)
        .translate(
            (
                opening_x,
                front_y - 0.025 + cavity_depth / 2.0,
                opening_bottom + opening_h / 2.0,
            )
        )
    )
    return outer.cut(cavity)


def _door_frame() -> cq.Workplane:
    """Thin drop-down door frame with a through window for the dark glass pane."""
    width = 0.390
    thickness = 0.018
    height = 0.220
    outer = cq.Workplane("XY").box(width, thickness, height).translate((0.0, 0.0, height / 2.0))
    window = cq.Workplane("XY").box(0.285, thickness * 3.0, 0.112).translate((0.0, 0.0, 0.120))
    return outer.cut(window)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_toaster_oven")

    stainless = Material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark = Material("black_enamel", rgba=(0.015, 0.016, 0.018, 1.0))
    glass = Material("smoky_glass", rgba=(0.06, 0.10, 0.12, 0.55))
    chrome = Material("chrome", rgba=(0.88, 0.90, 0.92, 1.0))
    tray_metal = Material("tray_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = Material("rubber_black", rgba=(0.005, 0.005, 0.004, 1.0))
    white = Material("white_marker", rgba=(0.95, 0.93, 0.86, 1.0))

    body_w = 0.50
    body_d = 0.34
    body_h = 0.28
    front_y = -body_d / 2.0
    opening_x = -0.055
    opening_bottom = 0.045

    housing = model.part("housing")
    # The housing is made from real wall panels rather than a solid proxy so the
    # door opening and crumb-tray cavity are physically open.
    housing.visual(
        Box((body_w, body_d, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=stainless,
        name="bottom_panel",
    )
    housing.visual(
        Box((body_w, body_d, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, body_h - 0.013)),
        material=stainless,
        name="top_panel",
    )
    housing.visual(
        Box((0.024, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + 0.012, 0.0, body_h / 2.0)),
        material=stainless,
        name="side_wall",
    )
    housing.visual(
        Box((0.124, body_d, body_h)),
        origin=Origin(xyz=(0.188, 0.0, body_h / 2.0)),
        material=stainless,
        name="control_wall",
    )
    housing.visual(
        Box((body_w, 0.028, body_h)),
        origin=Origin(xyz=(0.0, body_d / 2.0 - 0.014, body_h / 2.0)),
        material=stainless,
        name="rear_panel",
    )
    housing.visual(
        Box((0.35, 0.004, 0.190)),
        origin=Origin(xyz=(opening_x, body_d / 2.0 - 0.030, 0.145)),
        material=dark,
        name="dark_cavity_back",
    )
    housing.visual(
        Box((0.078, 0.004, 0.184)),
        origin=Origin(xyz=(0.202, front_y - 0.001, 0.158)),
        material=dark,
        name="control_panel",
    )

    # Flush dark ventilation slots are slightly seated in the top sheet metal.
    for idx, x in enumerate((-0.150, -0.100, -0.050, 0.000, 0.050, 0.100)):
        housing.visual(
            Box((0.012, 0.180, 0.0025)),
            origin=Origin(xyz=(x, 0.020, body_h + 0.0003)),
            material=dark,
            name=f"top_vent_{idx}",
        )

    # Rubber feet overlap the underside by a millimeter so they read as mounted.
    for idx, (x, y) in enumerate(
        ((-0.190, -0.110), (0.190, -0.110), (-0.190, 0.115), (0.190, 0.115))
    ):
        housing.visual(
            Cylinder(radius=0.018, length=0.015),
            origin=Origin(xyz=(x, y, -0.0065)),
            material=rubber,
            name=f"foot_{idx}",
        )

    hinge_y = front_y - 0.012
    hinge_z = 0.036
    housing_hinge_z = 0.024
    for name, x in (("hinge_barrel_0", opening_x - 0.155), ("hinge_barrel_1", opening_x + 0.155)):
        housing.visual(
            Cylinder(radius=0.012, length=0.072),
            origin=Origin(xyz=(x, hinge_y, housing_hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=chrome,
            name=name,
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_frame(), "drop_down_door_frame", tolerance=0.001),
        material=dark,
        name="door_frame",
    )
    door.visual(
        Box((0.305, 0.006, 0.128)),
        origin=Origin(xyz=(0.0, -0.002, 0.120)),
        material=glass,
        name="glass_pane",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="door_hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.315),
        origin=Origin(xyz=(0.0, -0.047, 0.177), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="handle_bar",
    )
    for name, x in (("handle_post_0", -0.130), ("handle_post_1", 0.130)):
        door.visual(
            Cylinder(radius=0.006, length=0.042),
            origin=Origin(xyz=(x, -0.028, 0.177), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=name,
        )

    door_joint = model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(opening_x, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.75),
    )

    crumb_tray = model.part("crumb_tray")
    tray_w = 0.340
    tray_len = 0.285
    tray_t = 0.006
    rim_h = 0.024
    rim_t = 0.010
    crumb_tray.visual(
        Box((tray_w, tray_len, tray_t)),
        origin=Origin(xyz=(0.0, tray_len / 2.0, 0.0)),
        material=tray_metal,
        name="tray_base",
    )
    crumb_tray.visual(
        Box((tray_w, rim_t, 0.032)),
        origin=Origin(xyz=(0.0, rim_t / 2.0, 0.018)),
        material=tray_metal,
        name="front_lip",
    )
    crumb_tray.visual(
        Box((0.095, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, -0.0005, 0.020)),
        material=dark,
        name="finger_pull",
    )
    crumb_tray.visual(
        Box((tray_w, rim_t, rim_h)),
        origin=Origin(xyz=(0.0, tray_len - rim_t / 2.0, 0.015)),
        material=tray_metal,
        name="rear_rim",
    )
    for name, x in (("side_rim_0", -(tray_w - rim_t) / 2.0), ("side_rim_1", (tray_w - rim_t) / 2.0)):
        crumb_tray.visual(
            Box((rim_t, tray_len, rim_h)),
            origin=Origin(xyz=(x, tray_len / 2.0, 0.015)),
            material=tray_metal,
            name=name,
        )

    tray_joint = model.articulation(
        "housing_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crumb_tray,
        origin=Origin(xyz=(opening_x, front_y + 0.018, opening_bottom + tray_t / 2.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.180),
    )

    for name, z in (("upper_dial", 0.195), ("lower_dial", 0.118)):
        dial = model.part(name)
        dial.visual(
            Cylinder(radius=0.025, length=0.022),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="dial_cap",
        )
        dial.visual(
            Box((0.006, 0.003, 0.029)),
            origin=Origin(xyz=(0.0, -0.0125, 0.006)),
            material=white,
            name="pointer_mark",
        )
        model.articulation(
            f"housing_to_{name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=dial,
            origin=Origin(xyz=(0.202, front_y - 0.011, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=2.5, lower=-2.7, upper=2.7),
        )

    # Keep joint objects referenced for tests through stable names.
    _ = (door_joint, tray_joint)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    tray = object_model.get_part("crumb_tray")
    door_joint = object_model.get_articulation("housing_to_door")
    tray_joint = object_model.get_articulation("housing_to_crumb_tray")

    ctx.expect_gap(
        housing,
        door,
        axis="y",
        positive_elem="control_panel",
        negative_elem="door_frame",
        max_gap=0.003,
        max_penetration=0.001,
        name="closed door sits just proud of front opening",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        elem_a="door_frame",
        min_overlap=0.17,
        name="door covers the rectangular oven opening",
    )
    ctx.expect_contact(
        tray,
        housing,
        elem_a="tray_base",
        elem_b="bottom_panel",
        contact_tol=0.002,
        name="crumb tray rides on the oven floor",
    )

    door_closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.35}):
        door_open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "drop-down door rotates downward and outward",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][2] < door_closed_aabb[1][2] - 0.10
        and door_open_aabb[0][1] < door_closed_aabb[0][1] - 0.08,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )

    tray_retracted_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: 0.180}):
        tray_extended_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            housing,
            axes="y",
            elem_a="tray_base",
            elem_b="bottom_panel",
            min_overlap=0.08,
            name="extended crumb tray remains retained in the cavity",
        )
    ctx.check(
        "crumb tray slides out through the front",
        tray_retracted_pos is not None
        and tray_extended_pos is not None
        and tray_extended_pos[1] < tray_retracted_pos[1] - 0.15,
        details=f"retracted={tray_retracted_pos}, extended={tray_extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
