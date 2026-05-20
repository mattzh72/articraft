from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_roof_vent_tower")

    matte_body = model.material("matte_warm_grey", rgba=(0.43, 0.45, 0.43, 1.0))
    satin_panel = model.material("satin_charcoal", rgba=(0.13, 0.14, 0.14, 1.0))
    satin_trim = model.material("satin_trim", rgba=(0.62, 0.64, 0.61, 1.0))
    dark_gasket = model.material("dark_gasket", rgba=(0.015, 0.016, 0.015, 1.0))
    dark_void = model.material("shadow_black", rgba=(0.025, 0.027, 0.026, 1.0))
    fastener = model.material("brushed_fastener", rgba=(0.72, 0.72, 0.68, 1.0))

    tower = model.part("tower")

    # Low roof curb and flashed base: broad, restrained, and visibly structural.
    tower.visual(
        Box((0.86, 0.72, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_panel,
        name="flashing_plate",
    )
    tower.visual(
        Box((0.66, 0.52, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=matte_body,
        name="raised_curb",
    )

    # Hollow rectangular tower body with a front framed outlet, built from
    # separate wall members so the opening is physically real rather than a
    # painted-on panel.
    outer_x = 0.56
    outer_y = 0.42
    body_h = 0.78
    wall = 0.045
    z_body = 0.39
    y_front = -outer_y / 2.0 + wall / 2.0
    y_back = outer_y / 2.0 - wall / 2.0
    x_side = outer_x / 2.0 - wall / 2.0

    tower.visual(
        Box((wall, outer_y, body_h)),
        origin=Origin(xyz=(-x_side, 0.0, z_body)),
        material=matte_body,
        name="side_wall_0",
    )
    tower.visual(
        Box((wall, outer_y, body_h)),
        origin=Origin(xyz=(x_side, 0.0, z_body)),
        material=matte_body,
        name="side_wall_1",
    )
    tower.visual(
        Box((outer_x, wall, body_h)),
        origin=Origin(xyz=(0.0, y_back, z_body)),
        material=matte_body,
        name="rear_wall",
    )

    outlet_w = 0.36
    outlet_h = 0.30
    outlet_z = 0.48
    outlet_min_z = outlet_z - outlet_h / 2.0
    outlet_max_z = outlet_z + outlet_h / 2.0
    jamb_w = (outer_x - outlet_w) / 2.0
    tower.visual(
        Box((jamb_w, wall, body_h)),
        origin=Origin(xyz=(-(outlet_w / 2.0 + jamb_w / 2.0), y_front, z_body)),
        material=matte_body,
        name="front_jamb_0",
    )
    tower.visual(
        Box((jamb_w, wall, body_h)),
        origin=Origin(xyz=((outlet_w / 2.0 + jamb_w / 2.0), y_front, z_body)),
        material=matte_body,
        name="front_jamb_1",
    )
    tower.visual(
        Box((outlet_w, wall, outlet_min_z)),
        origin=Origin(xyz=(0.0, y_front, outlet_min_z / 2.0)),
        material=matte_body,
        name="front_sill",
    )
    tower.visual(
        Box((outlet_w, wall, body_h - outlet_max_z)),
        origin=Origin(xyz=(0.0, y_front, (outlet_max_z + body_h) / 2.0)),
        material=matte_body,
        name="front_lintel",
    )

    # One-piece rounded trim ring around the outlet with a shallow recessed face.
    front_bezel = BezelGeometry(
        (0.36, 0.28),
        (0.48, 0.40),
        0.035,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.020,
        outer_corner_radius=0.030,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
    )
    tower.visual(
        mesh_from_geometry(front_bezel, "front_outlet_bezel"),
        origin=Origin(xyz=(0.0, -outer_y / 2.0 - 0.010, outlet_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_trim,
        name="front_outlet_bezel",
    )
    tower.visual(
        Box((0.47, 0.010, 0.31)),
        origin=Origin(xyz=(0.0, -outer_y / 2.0 + 0.080, outlet_z)),
        material=dark_void,
        name="recessed_shadow_baffle",
    )

    # Top outlet curb: four raised lips plus a black gasket, leaving a real open
    # rectangular outlet under the weather flap.
    rim_outer_x = 0.62
    rim_outer_y = 0.48
    rim_inner_x = 0.44
    rim_inner_y = 0.30
    rim_h = 0.080
    rim_z = body_h + rim_h / 2.0
    tower.visual(
        Box(((rim_outer_x - rim_inner_x) / 2.0, rim_outer_y, rim_h)),
        origin=Origin(xyz=(-(rim_inner_x / 2.0 + (rim_outer_x - rim_inner_x) / 4.0), 0.0, rim_z)),
        material=satin_trim,
        name="top_rim_0",
    )
    tower.visual(
        Box(((rim_outer_x - rim_inner_x) / 2.0, rim_outer_y, rim_h)),
        origin=Origin(xyz=((rim_inner_x / 2.0 + (rim_outer_x - rim_inner_x) / 4.0), 0.0, rim_z)),
        material=satin_trim,
        name="top_rim_1",
    )
    tower.visual(
        Box((rim_inner_x, (rim_outer_y - rim_inner_y) / 2.0, rim_h)),
        origin=Origin(xyz=(0.0, -(rim_inner_y / 2.0 + (rim_outer_y - rim_inner_y) / 4.0), rim_z)),
        material=satin_trim,
        name="top_rim_front",
    )
    tower.visual(
        Box((rim_inner_x, (rim_outer_y - rim_inner_y) / 2.0, rim_h)),
        origin=Origin(xyz=(0.0, (rim_inner_y / 2.0 + (rim_outer_y - rim_inner_y) / 4.0), rim_z)),
        material=satin_trim,
        name="top_rim_rear",
    )
    gasket_z = body_h + rim_h + 0.004
    tower.visual(
        Box((rim_outer_x, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -rim_outer_y / 2.0 + 0.006, gasket_z)),
        material=dark_gasket,
        name="front_gasket",
    )
    tower.visual(
        Box((rim_outer_x, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, rim_outer_y / 2.0 - 0.006, gasket_z)),
        material=dark_gasket,
        name="rear_gasket",
    )
    tower.visual(
        Box((0.012, rim_outer_y - 0.024, 0.008)),
        origin=Origin(xyz=(-rim_outer_x / 2.0 + 0.006, 0.0, gasket_z)),
        material=dark_gasket,
        name="side_gasket_0",
    )
    tower.visual(
        Box((0.012, rim_outer_y - 0.024, 0.008)),
        origin=Origin(xyz=(rim_outer_x / 2.0 - 0.006, 0.0, gasket_z)),
        material=dark_gasket,
        name="side_gasket_1",
    )

    # Restrained fasteners and alternating fixed hinge knuckles mounted on the
    # rear rim.  Cylinder primitives are rotated so their local Z axis becomes
    # the real hinge pin axis along X.
    hinge_y = rim_outer_y / 2.0 + 0.032
    hinge_z = body_h + rim_h + 0.015
    cyl_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    fixed_knuckle_x = (-0.212, 0.0, 0.212)
    for index, x in enumerate(fixed_knuckle_x):
        tower.visual(
            Box((0.092, 0.040, 0.006)),
            origin=Origin(xyz=(x, hinge_y - 0.018, hinge_z - 0.013)),
            material=satin_trim,
            name=f"fixed_leaf_{index}",
        )
        tower.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=cyl_x.rpy),
            material=satin_trim,
            name=f"fixed_knuckle_{index}",
        )
        tower.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(x, hinge_y - 0.028, hinge_z - 0.008)),
            material=fastener,
            name=f"fixed_screw_{index}",
        )
    tower.visual(
        Cylinder(radius=0.004, length=0.540),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=cyl_x.rpy),
        material=fastener,
        name="hinge_pin",
    )

    flap = model.part("flap")
    # Child frame is the hinge pin centerline.  At q=0 the flap is closed just
    # above the gasket, with a narrow reveal line all around the top frame.
    flap.visual(
        Box((0.66, 0.56, 0.018)),
        origin=Origin(xyz=(0.0, -0.306, 0.006)),
        material=satin_panel,
        name="flap_skin",
    )
    flap.visual(
        Box((0.66, 0.034, 0.060)),
        origin=Origin(xyz=(0.0, -0.565, -0.024)),
        material=satin_panel,
        name="front_drip_hem",
    )
    flap.visual(
        Box((0.024, 0.520, 0.050)),
        origin=Origin(xyz=(-0.342, -0.290, -0.020)),
        material=satin_panel,
        name="side_fold_0",
    )
    flap.visual(
        Box((0.024, 0.520, 0.050)),
        origin=Origin(xyz=(0.342, -0.290, -0.020)),
        material=satin_panel,
        name="side_fold_1",
    )
    # A subtle raised rib stiffens the sheet and makes the satin/matte material
    # break read as a manufactured premium cap rather than a plain slab.
    flap.visual(
        Box((0.52, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, -0.300, 0.019)),
        material=satin_trim,
        name="stiffening_rib",
    )
    for index, x in enumerate((-0.106, 0.106)):
        flap.visual(
            Box((0.092, 0.060, 0.012)),
            origin=Origin(xyz=(x, -0.030, 0.009)),
            material=satin_trim,
            name=f"moving_leaf_{index}",
        )
        flap.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=cyl_x.rpy),
            material=satin_trim,
            name=f"moving_knuckle_{index}",
        )
        flap.visual(
            Cylinder(radius=0.0055, length=0.003),
            origin=Origin(xyz=(x, -0.044, 0.0165)),
            material=fastener,
            name=f"moving_screw_{index}",
        )

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("tower_to_flap")

    for knuckle in ("moving_knuckle_0", "moving_knuckle_1"):
        ctx.allow_overlap(
            tower,
            flap,
            elem_a="hinge_pin",
            elem_b=knuckle,
            reason="A stainless hinge pin intentionally runs through the moving hinge knuckle bore.",
        )
        ctx.expect_overlap(
            tower,
            flap,
            axes="xyz",
            min_overlap=0.006,
            elem_a="hinge_pin",
            elem_b=knuckle,
            name=f"{knuckle} is captured on hinge pin",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            min_gap=0.001,
            max_gap=0.018,
            positive_elem="flap_skin",
            negative_elem="front_gasket",
            name="closed flap has tight gasket reveal",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="xy",
            min_overlap=0.08,
            elem_a="flap_skin",
            elem_b="top_rim_front",
            name="flap covers the framed top outlet",
        )

    closed_aabb = ctx.part_element_world_aabb(flap, elem="front_drip_hem")
    with ctx.pose({hinge: 0.95}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="front_drip_hem")
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            min_gap=0.055,
            positive_elem="front_drip_hem",
            negative_elem="front_gasket",
            name="opened flap clears the outlet frame",
        )
    ctx.check(
        "hinge lifts the front drip hem",
        closed_aabb is not None and open_aabb is not None and open_aabb[0][2] > closed_aabb[0][2] + 0.05,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
