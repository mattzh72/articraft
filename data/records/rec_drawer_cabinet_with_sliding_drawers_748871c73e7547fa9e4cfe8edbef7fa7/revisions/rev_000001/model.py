from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 1.20
BODY_D = 0.78
BODY_H = 1.45
PANEL_T = 0.06
FRONT_Y = -0.432
DRAWER_ZS = (0.33, 0.61, 0.89, 1.17)
DRAWER_TRAVEL = 0.52


def _add_cylinder_x(part, name, *, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_y(part, name, *, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refrigerated_sample_cabinet")

    insulated_white = model.material("insulated_white", rgba=(0.88, 0.92, 0.93, 1.0))
    inner_liner = model.material("cool_liner", rgba=(0.76, 0.86, 0.91, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.015, 0.017, 0.018, 1.0))
    steel = model.material("slide_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    blue_display = model.material("cold_blue_display", rgba=(0.05, 0.18, 0.32, 1.0))

    body = model.part("body")

    # Thick insulated cabinet shell, with a real rectangular top opening for
    # the rear access hatch rather than a solid placeholder slab.
    body.visual(
        Box((PANEL_T, BODY_D, BODY_H - 0.06)),
        origin=Origin(xyz=(-BODY_W / 2.0 + PANEL_T / 2.0, 0.0, 0.755)),
        material=insulated_white,
        name="side_wall_neg",
    )
    body.visual(
        Box((PANEL_T, BODY_D, BODY_H - 0.06)),
        origin=Origin(xyz=(BODY_W / 2.0 - PANEL_T / 2.0, 0.0, 0.755)),
        material=insulated_white,
        name="side_wall_pos",
    )
    body.visual(
        Box((BODY_W, PANEL_T, BODY_H - 0.06)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - PANEL_T / 2.0, 0.755)),
        material=insulated_white,
        name="rear_wall",
    )
    body.visual(
        Box((BODY_W, BODY_D, PANEL_T)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=insulated_white,
        name="bottom_panel",
    )
    body.visual(
        Box((BODY_W, 0.29, PANEL_T)),
        origin=Origin(xyz=(0.0, -0.245, BODY_H - PANEL_T / 2.0)),
        material=insulated_white,
        name="top_front_deck",
    )
    body.visual(
        Box((BODY_W, 0.14, PANEL_T)),
        origin=Origin(xyz=(0.0, 0.32, BODY_H - PANEL_T / 2.0)),
        material=insulated_white,
        name="top_rear_deck",
    )
    body.visual(
        Box((0.20, 0.35, PANEL_T)),
        origin=Origin(xyz=(-0.50, 0.075, BODY_H - PANEL_T / 2.0)),
        material=insulated_white,
        name="top_side_deck_neg",
    )
    body.visual(
        Box((0.20, 0.35, PANEL_T)),
        origin=Origin(xyz=(0.50, 0.075, BODY_H - PANEL_T / 2.0)),
        material=insulated_white,
        name="top_side_deck_pos",
    )

    # Drawer-bay front frame and soft black gaskets around the four cold
    # drawer openings.
    body.visual(
        Box((BODY_W, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, -0.367, 0.150)),
        material=insulated_white,
        name="front_bottom_sill",
    )
    body.visual(
        Box((BODY_W, 0.045, 0.080)),
        origin=Origin(xyz=(0.0, -0.367, 1.345)),
        material=insulated_white,
        name="front_top_rail",
    )
    for idx, z in enumerate((0.470, 0.750, 1.030)):
        body.visual(
            Box((BODY_W, 0.045, 0.030)),
            origin=Origin(xyz=(0.0, -0.367, z)),
            material=insulated_white,
            name=f"front_mullion_{idx}",
        )
    body.visual(
        Box((0.012, 0.012, 1.135)),
        origin=Origin(xyz=(-0.535, -0.3955, 0.750)),
        material=dark_rubber,
        name="gasket_side_neg",
    )
    body.visual(
        Box((0.012, 0.012, 1.135)),
        origin=Origin(xyz=(0.535, -0.3955, 0.750)),
        material=dark_rubber,
        name="gasket_side_pos",
    )
    for idx, z in enumerate((0.190, 0.470, 0.750, 1.030, 1.310)):
        body.visual(
            Box((1.080, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -0.3955, z)),
            material=dark_rubber,
            name=f"gasket_strip_{idx}",
        )

    # A pale blue liner visible behind the drawer stack suggests a refrigerated,
    # insulated interior without making the cabinet a solid block.
    body.visual(
        Box((0.96, 0.012, 1.10)),
        origin=Origin(xyz=(0.0, 0.324, 0.750)),
        material=inner_liner,
        name="rear_cold_liner",
    )

    # Rear service details for the refrigeration unit.
    for idx, z in enumerate((0.34, 0.40, 0.46, 0.52, 0.58)):
        body.visual(
            Box((0.44, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, 0.393, z)),
            material=steel,
            name=f"rear_vent_slat_{idx}",
        )
    body.visual(
        Box((0.30, 0.010, 0.045)),
        origin=Origin(xyz=(-0.30, 0.393, 1.280)),
        material=blue_display,
        name="temperature_display",
    )

    # Leveling feet overlap the underside slightly, as real adjustable feet
    # screw into the insulated base.
    for ix, x in enumerate((-0.49, 0.49)):
        for iy, y in enumerate((-0.30, 0.30)):
            body.visual(
                Cylinder(radius=0.035, length=0.070),
                origin=Origin(xyz=(x, y, 0.035)),
                material=dark_rubber,
                name=f"leveling_foot_{ix}_{iy}",
            )

    # Fixed members of the full-extension slides, mounted to the insulated
    # side walls. The moving members on each drawer touch these rails at the
    # bearing plane but do not interpenetrate.
    for i, z in enumerate(DRAWER_ZS):
        for side, x in (("neg", -0.52625), ("pos", 0.52625)):
            body.visual(
                Box((0.0275, 0.700, 0.035)),
                origin=Origin(xyz=(x, 0.000, z - 0.055)),
                material=steel,
                name=f"fixed_slide_{i}_{side}",
            )

    # Alternating hinge barrels and a leaf on the fixed body side of the rear
    # top access panel.
    body.visual(
        Box((0.86, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.260, 1.452)),
        material=steel,
        name="hinge_leaf",
    )
    for i, x in enumerate((-0.28, 0.0, 0.28)):
        _add_cylinder_x(
            body,
            f"hinge_barrel_{i}",
            radius=0.012,
            length=0.115,
            xyz=(x, 0.260, 1.465),
            material=steel,
        )

    # Four independent stainless drawers.
    for i, z in enumerate(DRAWER_ZS):
        drawer = model.part(f"drawer_{i}")
        drawer.visual(
            Box((1.040, 0.040, 0.250)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=stainless,
            name="front_panel",
        )
        drawer.visual(
            Box((0.980, 0.018, 0.160)),
            origin=Origin(xyz=(0.0, 0.035, -0.010)),
            material=stainless,
            name="front_return",
        )
        drawer.visual(
            Box((0.980, 0.600, 0.018)),
            origin=Origin(xyz=(0.0, 0.335, -0.095)),
            material=stainless,
            name="tray_floor",
        )
        drawer.visual(
            Box((0.025, 0.600, 0.160)),
            origin=Origin(xyz=(-0.480, 0.335, -0.020)),
            material=stainless,
            name="tray_side_neg",
        )
        drawer.visual(
            Box((0.025, 0.600, 0.160)),
            origin=Origin(xyz=(0.480, 0.335, -0.020)),
            material=stainless,
            name="tray_side_pos",
        )
        drawer.visual(
            Box((0.980, 0.025, 0.160)),
            origin=Origin(xyz=(0.0, 0.635, -0.020)),
            material=stainless,
            name="tray_rear_wall",
        )
        drawer.visual(
            Box((0.020, 0.760, 0.035)),
            origin=Origin(xyz=(-0.5025, 0.380, -0.055)),
            material=steel,
            name="moving_slide_neg",
        )
        drawer.visual(
            Box((0.020, 0.760, 0.035)),
            origin=Origin(xyz=(0.5025, 0.380, -0.055)),
            material=steel,
            name="moving_slide_pos",
        )
        drawer.visual(
            Box((0.34, 0.008, 0.045)),
            origin=Origin(xyz=(0.0, -0.024, 0.058)),
            material=blue_display,
            name="label_window",
        )
        _add_cylinder_x(
            drawer,
            "handle_bar",
            radius=0.012,
            length=0.460,
            xyz=(0.0, -0.060, -0.035),
            material=steel,
        )
        _add_cylinder_y(
            drawer,
            "handle_post_neg",
            radius=0.009,
            length=0.040,
            xyz=(-0.190, -0.040, -0.035),
            material=steel,
        )
        _add_cylinder_y(
            drawer,
            "handle_post_pos",
            radius=0.009,
            length=0.040,
            xyz=(0.190, -0.040, -0.035),
            material=steel,
        )

        model.articulation(
            f"drawer_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, FRONT_Y, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0, velocity=0.35, lower=0.0, upper=DRAWER_TRAVEL
            ),
        )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((0.860, 0.355, 0.035)),
        origin=Origin(xyz=(0.0, -0.1925, 0.0025)),
        material=insulated_white,
        name="panel_skin",
    )
    access_panel.visual(
        Box((0.780, 0.290, 0.010)),
        origin=Origin(xyz=(0.0, -0.185, 0.025)),
        material=stainless,
        name="stainless_cap",
    )
    access_panel.visual(
        Box((0.220, 0.035, 0.018)),
        origin=Origin(xyz=(0.0, -0.305, 0.039)),
        material=steel,
        name="lift_pull",
    )
    for i, x in enumerate((-0.14, 0.14)):
        access_panel.visual(
            Box((0.115, 0.030, 0.008)),
            origin=Origin(xyz=(x, -0.014, -0.006)),
            material=steel,
            name=f"panel_hinge_leaf_{i}",
        )
    for i, x in enumerate((-0.14, 0.14)):
        _add_cylinder_x(
            access_panel,
            f"panel_barrel_{i}",
            radius=0.012,
            length=0.115,
            xyz=(x, 0.0, 0.0),
            material=steel,
        )

    model.articulation(
        "access_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=access_panel,
        origin=Origin(xyz=(0.0, 0.260, 1.465)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    ctx.check(
        "four articulated drawers",
        all(object_model.get_part(f"drawer_{i}") is not None for i in range(4)),
        details="Expected four wide stainless drawer links.",
    )

    for i in range(4):
        drawer = object_model.get_part(f"drawer_{i}")
        slide = object_model.get_articulation(f"drawer_{i}_slide")
        ctx.check(
            f"drawer {i} uses prismatic slide",
            slide.articulation_type == ArticulationType.PRISMATIC,
            details=f"joint_type={slide.articulation_type}",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="moving_slide_pos",
            elem_b=f"fixed_slide_{i}_pos",
            contact_tol=0.0005,
            name=f"drawer {i} positive slide rides body rail",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="moving_slide_neg",
            elem_b=f"fixed_slide_{i}_neg",
            contact_tol=0.0005,
            name=f"drawer {i} negative slide rides body rail",
        )
        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: DRAWER_TRAVEL}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="moving_slide_pos",
                elem_b=f"fixed_slide_{i}_pos",
                min_overlap=0.120,
                name=f"drawer {i} positive slide retains insertion",
            )
            extended_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"drawer {i} extends toward front",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] < rest_pos[1] - 0.45,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    panel = object_model.get_part("access_panel")
    hinge = object_model.get_articulation("access_panel_hinge")
    ctx.check(
        "access panel has revolute hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint_type={hinge.articulation_type}",
    )
    ctx.expect_contact(
        panel,
        body,
        elem_a="panel_skin",
        elem_b="top_side_deck_pos",
        contact_tol=0.0005,
        name="closed access panel rests on top lip",
    )
    closed_aabb = ctx.part_element_world_aabb(panel, elem="lift_pull")
    with ctx.pose({hinge: 1.0}):
        open_aabb = ctx.part_element_world_aabb(panel, elem="lift_pull")
    ctx.check(
        "rear top access panel lifts upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
