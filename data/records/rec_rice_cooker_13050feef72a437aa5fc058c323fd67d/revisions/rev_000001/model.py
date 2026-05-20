from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_plate(width: float, height: float, depth: float, radius: float):
    """Rounded rectangular extrusion with its face in local XY and thickness +Z."""
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        depth,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_rice_cooker")

    warm_white = Material("warm_white_plastic", rgba=(0.92, 0.89, 0.82, 1.0))
    light_trim = Material("satin_light_gray", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_trim = Material("charcoal_panel", rgba=(0.10, 0.11, 0.12, 1.0))
    button_gray = Material("button_light_gray", rgba=(0.78, 0.80, 0.78, 1.0))
    black = Material("soft_black", rgba=(0.015, 0.015, 0.014, 1.0))
    label_white = Material("printed_white", rgba=(0.95, 0.95, 0.90, 1.0))

    body = model.part("body")

    # A compact household cooker: about 29 cm overall height and 29 cm diameter.
    # The lathed shell has a visibly inset lower skirt and a slightly rolled top.
    body_profile = [
        (0.000, 0.000),
        (0.122, 0.000),
        (0.128, 0.008),
        (0.128, 0.052),
        (0.139, 0.063),
        (0.142, 0.145),
        (0.138, 0.205),
        (0.130, 0.216),
        (0.000, 0.216),
    ]
    body.visual(
        mesh_from_geometry(LatheGeometry(body_profile, segments=72), "body_shell"),
        material=warm_white,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.1295, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=light_trim,
        name="lower_band",
    )
    body.visual(
        Cylinder(radius=0.116, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black,
        name="foot_ring",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.132, tube=0.006, radial_segments=24, tubular_segments=72), "top_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.216)),
        material=light_trim,
        name="top_rim",
    )

    # A dark rounded control panel is seated on the front.  The lighter upper pad
    # makes the separate latch zone clear before the latch button is added.
    body.visual(
        mesh_from_geometry(_rounded_plate(0.115, 0.112, 0.004, 0.013), "front_panel"),
        origin=Origin(xyz=(0.0, -0.142, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="front_panel",
    )
    body.visual(
        mesh_from_geometry(_rounded_plate(0.080, 0.034, 0.002, 0.009), "latch_zone"),
        origin=Origin(xyz=(0.0, -0.146, 0.166), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_trim,
        name="latch_zone",
    )

    # Rear hinge cheeks mounted into the cooker rim.  The moving lid barrel sits
    # between them, leaving a small visible clearance at each end.
    for x in (-0.076, 0.076):
        body.visual(
            Box((0.035, 0.055, 0.014)),
            origin=Origin(xyz=(x, 0.130, 0.213)),
            material=light_trim,
            name=f"hinge_mount_{0 if x < 0 else 1}",
        )
        body.visual(
            Box((0.027, 0.030, 0.032)),
            origin=Origin(xyz=(x, 0.158, 0.232)),
            material=light_trim,
            name=f"hinge_cheek_{0 if x < 0 else 1}",
        )

    lid = model.part("lid")
    dome = DomeGeometry(radius=0.142, radial_segments=72, height_segments=18)
    dome.scale(1.0, 1.0, 0.46).translate(0.0, -0.153, -0.010)
    lid.visual(
        mesh_from_geometry(dome, "lid_dome"),
        material=warm_white,
        name="lid_dome",
    )
    lid.visual(
        mesh_from_geometry(TorusGeometry(radius=0.141, tube=0.004, radial_segments=18, tubular_segments=72), "lid_rim"),
        origin=Origin(xyz=(0.0, -0.153, -0.006)),
        material=light_trim,
        name="lid_rim",
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.105),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_trim,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.112, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.008, -0.004)),
        material=warm_white,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, -0.153, 0.058)),
        material=black,
        name="steam_vent",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.153, 0.232)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_geometry(_rounded_plate(0.050, 0.020, 0.008, 0.007), "latch_button_cap"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_gray,
        name="button_cap",
    )
    latch_button.visual(
        Box((0.020, 0.0008, 0.003)),
        origin=Origin(xyz=(0.0, -0.0083, 0.0)),
        material=dark_trim,
        name="button_mark",
    )
    model.articulation(
        "body_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(0.0, -0.1480, 0.166)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.12, lower=0.0, upper=0.006),
    )

    for idx, x in enumerate((-0.026, 0.026)):
        menu_button = model.part(f"menu_button_{idx}")
        menu_button.visual(
            mesh_from_geometry(_rounded_plate(0.032, 0.018, 0.006, 0.006), f"menu_button_{idx}_cap"),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_gray,
            name="button_cap",
        )
        menu_button.visual(
            Box((0.014, 0.0007, 0.0025)),
            origin=Origin(xyz=(0.0, -0.0062, 0.0)),
            material=label_white,
            name="button_mark",
        )
        model.articulation(
            f"body_to_menu_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=menu_button,
            origin=Origin(xyz=(x, -0.1460, 0.092)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.10, lower=0.0, upper=0.0045),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch_button")
    menu_0 = object_model.get_part("menu_button_0")
    menu_1 = object_model.get_part("menu_button_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_slide = object_model.get_articulation("body_to_latch_button")
    menu_slide_0 = object_model.get_articulation("body_to_menu_button_0")
    menu_slide_1 = object_model.get_articulation("body_to_menu_button_1")

    ctx.expect_within(
        latch,
        body,
        axes="xz",
        inner_elem="button_cap",
        outer_elem="latch_zone",
        margin=0.003,
        name="latch button sits inside the latch zone footprint",
    )
    ctx.expect_gap(
        body,
        latch,
        axis="y",
        positive_elem="latch_zone",
        negative_elem="button_cap",
        min_gap=0.0,
        max_gap=0.001,
        name="latch button rests just proud of the latch zone",
    )
    for idx, button in enumerate((menu_0, menu_1)):
        ctx.expect_within(
            button,
            body,
            axes="xz",
            inner_elem="button_cap",
            outer_elem="front_panel",
            margin=0.004,
            name=f"menu button {idx} sits inside the control panel",
        )
        ctx.expect_gap(
            body,
            button,
            axis="y",
            positive_elem="front_panel",
            negative_elem="button_cap",
            min_gap=0.0,
            max_gap=0.002,
            name=f"menu button {idx} rests proud of the panel",
        )
    ctx.expect_origin_distance(
        menu_0,
        menu_1,
        axes="x",
        min_dist=0.045,
        max_dist=0.060,
        name="the two menu buttons are visibly separate controls",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_dome",
        elem_b="body_shell",
        min_overlap=0.20,
        name="closed lid covers the cylindrical cooker body",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.0}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge opens the dome upward at the rear rim",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.045,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    for name, part, joint, travel in (
        ("latch button", latch, latch_slide, 0.006),
        ("menu button 0", menu_0, menu_slide_0, 0.0045),
        ("menu button 1", menu_1, menu_slide_1, 0.0045),
    ):
        rest = ctx.part_world_position(part)
        with ctx.pose({joint: travel}):
            depressed = ctx.part_world_position(part)
        ctx.check(
            f"{name} translates inward independently",
            rest is not None and depressed is not None and depressed[1] > rest[1] + travel * 0.8,
            details=f"rest={rest}, depressed={depressed}",
        )

    return ctx.report()


object_model = build_object_model()
