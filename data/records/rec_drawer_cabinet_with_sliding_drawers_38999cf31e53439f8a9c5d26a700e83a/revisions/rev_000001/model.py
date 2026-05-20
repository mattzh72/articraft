from __future__ import annotations

from math import pi

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


OUTER_WIDTH = 1.00
OUTER_DEPTH = 0.60
OUTER_HEIGHT = 0.86
PANEL_THICKNESS = 0.018
TOP_BOTTOM_THICKNESS = 0.020
BACK_THICKNESS = 0.014
FRONT_PLANE_X = 0.0

INNER_WIDTH = OUTER_WIDTH - 2.0 * PANEL_THICKNESS
DRAWER_BOX_WIDTH = 0.84
DRAWER_BOX_DEPTH = 0.54
DRAWER_TRAVEL = 0.48


def _box(part, name: str, size, xyz, material: str) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(part, name: str, radius: float, length: float, xyz, rpy, material: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_drawer_visuals(
    drawer,
    *,
    front_height: float,
    box_height: float,
    handle_width: float,
    name_prefix: str,
) -> None:
    """Drawer geometry in a child frame whose origin is at the closed front plane."""
    front_width = 0.92
    front_thickness = 0.022
    bottom_z = -front_height / 2.0 + 0.035
    wall_center_z = bottom_z + 0.006 + box_height / 2.0
    rail_z = bottom_z + 0.055

    # Wide slab front; the box and slides extend into the cabinet along -X.
    _box(
        drawer,
        "front_slab",
        (front_thickness, front_width, front_height),
        (front_thickness / 2.0, 0.0, 0.0),
        "warm_white",
    )

    # Open-topped birch drawer box, visibly hollow behind the front.
    _box(
        drawer,
        "box_bottom",
        (DRAWER_BOX_DEPTH, DRAWER_BOX_WIDTH, 0.012),
        (-DRAWER_BOX_DEPTH / 2.0, 0.0, bottom_z),
        "birch_plywood",
    )
    for sign, side_name in ((1.0, "side_wall_0"), (-1.0, "side_wall_1")):
        _box(
            drawer,
            side_name,
            (DRAWER_BOX_DEPTH, 0.012, box_height),
            (-DRAWER_BOX_DEPTH / 2.0, sign * (DRAWER_BOX_WIDTH / 2.0 - 0.006), wall_center_z),
            "birch_plywood",
        )
    _box(
        drawer,
        "back_wall",
        (0.014, DRAWER_BOX_WIDTH, box_height),
        (-DRAWER_BOX_DEPTH + 0.007, 0.0, wall_center_z),
        "birch_plywood",
    )
    _box(
        drawer,
        "inner_front_rail",
        (0.018, DRAWER_BOX_WIDTH, box_height * 0.62),
        (-0.009, 0.0, wall_center_z - box_height * 0.10),
        "birch_plywood",
    )

    # Moving slide members are carried by small brackets from the drawer box sides.
    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        _box(
            drawer,
            f"moving_slide_{suffix}",
            (0.52, 0.012, 0.024),
            (-0.260, sign * 0.464, rail_z),
            "galvanized_steel",
        )
        _box(
            drawer,
            f"slide_bracket_{suffix}",
            (0.16, 0.038, 0.014),
            (-0.145, sign * 0.439, rail_z),
            "galvanized_steel",
        )

    # Dark recess pocket and a stainless horizontal bar pull, with posts that
    # physically tie the handle to the front slab.
    pocket_height = 0.060
    handle_z = front_height * 0.22
    _box(
        drawer,
        "handle_recess",
        (0.004, handle_width + 0.110, pocket_height),
        (front_thickness - 0.003, 0.0, handle_z),
        "shadow_black",
    )
    _cylinder(
        drawer,
        "bar_pull",
        0.008,
        handle_width,
        (front_thickness + 0.019, 0.0, handle_z),
        (pi / 2.0, 0.0, 0.0),
        "brushed_steel",
    )
    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        _box(
            drawer,
            f"handle_post_{suffix}",
            (0.030, 0.018, 0.028),
            (front_thickness + 0.010, sign * (handle_width / 2.0 - 0.030), handle_z),
            "brushed_steel",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="base_cabinet_drawer_stack")
    model.material("carcass_melamine", rgba=(0.78, 0.77, 0.72, 1.0))
    model.material("warm_white", rgba=(0.93, 0.91, 0.86, 1.0))
    model.material("birch_plywood", rgba=(0.86, 0.68, 0.42, 1.0))
    model.material("galvanized_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("brushed_steel", rgba=(0.80, 0.79, 0.74, 1.0))
    model.material("shadow_black", rgba=(0.015, 0.014, 0.012, 1.0))
    model.material("toe_kick_black", rgba=(0.04, 0.035, 0.03, 1.0))

    carcass = model.part("carcass")
    half_w = OUTER_WIDTH / 2.0

    # Wide rectangular carcass: side panels, top, bottom, back, and a recessed toe kick.
    for sign, side_name in ((1.0, "side_panel_0"), (-1.0, "side_panel_1")):
        _box(
            carcass,
            side_name,
            (OUTER_DEPTH, PANEL_THICKNESS, OUTER_HEIGHT),
            (-OUTER_DEPTH / 2.0, sign * (half_w - PANEL_THICKNESS / 2.0), OUTER_HEIGHT / 2.0),
            "carcass_melamine",
        )
    _box(
        carcass,
        "top_panel",
        (OUTER_DEPTH, OUTER_WIDTH, TOP_BOTTOM_THICKNESS),
        (-OUTER_DEPTH / 2.0, 0.0, OUTER_HEIGHT - TOP_BOTTOM_THICKNESS / 2.0),
        "carcass_melamine",
    )
    _box(
        carcass,
        "bottom_panel",
        (OUTER_DEPTH, OUTER_WIDTH, TOP_BOTTOM_THICKNESS),
        (-OUTER_DEPTH / 2.0, 0.0, TOP_BOTTOM_THICKNESS / 2.0),
        "carcass_melamine",
    )
    _box(
        carcass,
        "back_panel",
        (BACK_THICKNESS, OUTER_WIDTH, OUTER_HEIGHT),
        (-OUTER_DEPTH + BACK_THICKNESS / 2.0, 0.0, OUTER_HEIGHT / 2.0),
        "carcass_melamine",
    )
    _box(
        carcass,
        "toe_kick",
        (0.070, 0.86, 0.070),
        (-0.105, 0.0, 0.035),
        "toe_kick_black",
    )

    # Narrow dark reveal strips make the three independent drawer fronts read as separate.
    _box(carcass, "reveal_side_0", (0.004, 0.022, 0.740), (-0.006, 0.471, 0.450), "shadow_black")
    _box(carcass, "reveal_side_1", (0.004, 0.022, 0.740), (-0.006, -0.471, 0.450), "shadow_black")
    _box(carcass, "reveal_pot_gap", (0.004, 0.942, 0.010), (-0.006, 0.0, 0.405), "shadow_black")
    _box(carcass, "reveal_drawer_gap", (0.004, 0.942, 0.010), (-0.006, 0.0, 0.610), "shadow_black")
    _box(carcass, "reveal_top_gap", (0.004, 0.942, 0.012), (-0.006, 0.0, 0.818), "shadow_black")
    _box(carcass, "reveal_bottom_gap", (0.004, 0.942, 0.010), (-0.006, 0.0, 0.084), "shadow_black")

    drawer_specs = [
        ("pot_drawer", 0.245, 0.310, 0.230, 0.760),
        ("drawer_0", 0.505, 0.200, 0.120, 0.590),
        ("drawer_1", 0.710, 0.200, 0.120, 0.590),
    ]

    for name, z_center, front_h, box_h, handle_w in drawer_specs:
        rail_z_world = z_center + (-front_h / 2.0 + 0.035) + 0.055
        # Fixed cabinet members of the full-extension slides, attached to the inside faces.
        for sign, suffix in ((1.0, "0"), (-1.0, "1")):
            _box(
                carcass,
                f"{name}_fixed_slide_{suffix}",
                (0.52, 0.012, 0.028),
                (-0.260, sign * 0.476, rail_z_world),
                "galvanized_steel",
            )

        drawer = model.part(name)
        _add_drawer_visuals(
            drawer,
            front_height=front_h,
            box_height=box_h,
            handle_width=handle_w,
            name_prefix=name,
        )
        model.articulation(
            f"carcass_to_{name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(FRONT_PLANE_X, 0.0, z_center)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=DRAWER_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")
    for name in ("drawer_0", "drawer_1", "pot_drawer"):
        drawer = object_model.get_part(name)
        joint = object_model.get_articulation(f"carcass_to_{name}")
        ctx.expect_within(
            drawer,
            carcass,
            axes="y",
            inner_elem="box_bottom",
            outer_elem="bottom_panel",
            margin=0.010,
            name=f"{name} box stays between side panels",
        )
        ctx.expect_overlap(
            drawer,
            carcass,
            axes="x",
            elem_a="moving_slide_0",
            elem_b=f"{name}_fixed_slide_0",
            min_overlap=0.48,
            name=f"{name} closed slide engagement",
        )
        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: DRAWER_TRAVEL}):
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="x",
                elem_a="moving_slide_0",
                elem_b=f"{name}_fixed_slide_0",
                min_overlap=0.030,
                name=f"{name} full-extension slide retained",
            )
            open_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"{name} extends forward",
            closed_pos is not None and open_pos is not None and open_pos[0] > closed_pos[0] + 0.40,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    return ctx.report()


object_model = build_object_model()
