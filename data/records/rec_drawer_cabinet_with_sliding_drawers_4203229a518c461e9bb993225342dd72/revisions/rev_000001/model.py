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


CABINET_DEPTH = 0.58
CABINET_WIDTH = 0.76
CARCASS_HEIGHT = 0.86
DRAWER_FRONT_X = CABINET_DEPTH + 0.015
DRAWER_TRAVEL = 0.43


def _add_box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_oak_grain(part, prefix, face_height, material, *, width=0.62):
    """Shallow raised darker grain strips on the front face of a drawer."""
    offsets = (-0.32, -0.12, 0.10, 0.31)
    for i, frac in enumerate(offsets):
        z = frac * face_height
        strip_width = width * (0.86 if i % 2 else 1.0)
        _add_box(
            part,
            f"{prefix}_grain_{i}",
            (0.002, strip_width, 0.0035),
            (0.014, 0.0, z),
            material,
        )


def _add_bar_pull(part, prefix, z_offset, metal):
    """Horizontal brushed-metal drawer pull tied to the drawer face by two posts."""
    part.visual(
        Cylinder(radius=0.010, length=0.38),
        origin=Origin(xyz=(0.055, 0.0, z_offset), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name=f"{prefix}_handle_bar",
    )
    for side, y in (("lower", -0.16), ("upper", 0.16)):
        part.visual(
            Cylinder(radius=0.006, length=0.042),
            origin=Origin(xyz=(0.034, y, z_offset), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name=f"{prefix}_handle_post_{side}",
        )


def _add_drawer_body(part, prefix, face_height, body_height, oak, metal):
    """Open-top drawer box with side-mounted full-extension slide members."""
    body_front = -0.012
    body_back = -0.520
    body_len = body_front - body_back
    body_center_x = (body_front + body_back) / 2.0
    body_width = 0.62
    wall = 0.014
    bottom_thick = 0.012
    body_center_z = -0.018

    _add_box(
        part,
        f"{prefix}_front",
        (0.026, 0.70, face_height),
        (0.0, 0.0, 0.0),
        oak,
    )
    _add_oak_grain(part, prefix, face_height, "oak_grain")

    for side, y in (("side_0", -body_width / 2.0 + wall / 2.0), ("side_1", body_width / 2.0 - wall / 2.0)):
        _add_box(
            part,
            f"{prefix}_{side}",
            (body_len, wall, body_height),
            (body_center_x, y, body_center_z),
            oak,
        )
    _add_box(
        part,
        f"{prefix}_back",
        (wall, body_width, body_height),
        (body_back + wall / 2.0, 0.0, body_center_z),
        oak,
    )
    _add_box(
        part,
        f"{prefix}_bottom",
        (body_len, body_width, bottom_thick),
        (body_center_x, 0.0, body_center_z - body_height / 2.0 + bottom_thick / 2.0),
        oak,
    )

    # Moving members of the ball-bearing slides stay attached to the drawer box.
    for side, y in (("slide_0", -0.336), ("slide_1", 0.336)):
        _add_box(
            part,
            f"{prefix}_{side}",
            (0.55, 0.010, 0.026),
            (-0.275, y, body_center_z - 0.006),
            metal,
        )

    _add_bar_pull(part, prefix, 0.0, metal)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oak_base_cabinet")

    oak = model.material("oak", rgba=(0.72, 0.45, 0.22, 1.0))
    oak_dark = model.material("oak_grain", rgba=(0.36, 0.20, 0.09, 1.0))
    stone = model.material("stone_countertop", rgba=(0.46, 0.45, 0.42, 1.0))
    metal = model.material("brushed_slide_steel", rgba=(0.68, 0.68, 0.64, 1.0))
    shadow = model.material("shadow_black", rgba=(0.035, 0.030, 0.025, 1.0))

    carcass = model.part("carcass")

    # Oak cabinet carcass: sides, back, bottom deck, and front rails leave a true
    # open front for the drawers instead of a solid block.
    _add_box(
        carcass,
        "side_panel_0",
        (CABINET_DEPTH, 0.025, CARCASS_HEIGHT),
        (CABINET_DEPTH / 2.0, -CABINET_WIDTH / 2.0 + 0.0125, CARCASS_HEIGHT / 2.0),
        oak,
    )
    _add_box(
        carcass,
        "side_panel_1",
        (CABINET_DEPTH, 0.025, CARCASS_HEIGHT),
        (CABINET_DEPTH / 2.0, CABINET_WIDTH / 2.0 - 0.0125, CARCASS_HEIGHT / 2.0),
        oak,
    )
    _add_box(
        carcass,
        "back_panel",
        (0.025, CABINET_WIDTH, CARCASS_HEIGHT),
        (0.0125, 0.0, CARCASS_HEIGHT / 2.0),
        oak,
    )
    _add_box(carcass, "bottom_deck", (CABINET_DEPTH, 0.71, 0.026), (CABINET_DEPTH / 2.0, 0.0, 0.013), oak)
    _add_box(carcass, "top_front_rail", (0.035, 0.71, 0.040), (0.5625, 0.0, 0.840), oak)
    _add_box(carcass, "bottom_front_rail", (0.035, 0.71, 0.030), (0.5625, 0.0, 0.100), oak)
    _add_box(carcass, "drawer_rail_0", (0.024, 0.71, 0.018), (0.568, 0.0, 0.440), oak)
    _add_box(carcass, "drawer_rail_1", (0.024, 0.71, 0.018), (0.568, 0.0, 0.695), oak)
    _add_box(carcass, "toe_kick", (0.070, 0.68, 0.090), (0.500, 0.0, 0.055), shadow)

    # Countertop with a slight overhang and rounded-looking front nosing.
    _add_box(carcass, "counter_slab", (0.66, 0.84, 0.045), (0.300, 0.0, 0.8825), stone)
    carcass.visual(
        Cylinder(radius=0.018, length=0.84),
        origin=Origin(xyz=(0.612, 0.0, 0.866), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stone,
        name="counter_bullnose",
    )

    # Fixed cabinet-side slide rails.  Each pair is mounted to the inside of
    # the oak side panels; the drawer-mounted rails sit just inboard of them.
    for prefix, z in (("top", 0.750), ("middle", 0.555), ("bottom", 0.260)):
        for side, y in (("slide_0", -0.348), ("slide_1", 0.348)):
            _add_box(
                carcass,
                f"{prefix}_{side}",
                (0.54, 0.014, 0.030),
                (0.310, y, z - 0.024),
                metal,
            )

    drawers = (
        ("top_drawer", "top", 0.765, 0.130, 0.095),
        ("middle_drawer", "middle", 0.565, 0.245, 0.170),
        ("bottom_drawer", "bottom", 0.275, 0.320, 0.250),
    )

    for part_name, prefix, z, face_height, body_height in drawers:
        drawer = model.part(part_name)
        _add_drawer_body(drawer, prefix, face_height, body_height, oak, metal)
        model.articulation(
            f"{prefix}_slide",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(DRAWER_FRONT_X, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=DRAWER_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    for drawer_name, prefix, reveal_rail in (
        ("top_drawer", "top", "top_front_rail"),
        ("middle_drawer", "middle", "drawer_rail_1"),
        ("bottom_drawer", "bottom", "drawer_rail_0"),
    ):
        drawer = object_model.get_part(drawer_name)
        slide = object_model.get_articulation(f"{prefix}_slide")

        ctx.expect_gap(
            drawer,
            carcass,
            axis="x",
            min_gap=0.001,
            max_gap=0.010,
            positive_elem=f"{prefix}_front",
            negative_elem=reveal_rail,
            name=f"{prefix} front has a cabinet reveal",
        )
        ctx.expect_overlap(
            drawer,
            carcass,
            axes="x",
            min_overlap=0.45,
            elem_a=f"{prefix}_slide_1",
            elem_b=f"{prefix}_slide_1",
            name=f"{prefix} slide is deeply inserted when shut",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: DRAWER_TRAVEL}):
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="x",
                min_overlap=0.075,
                elem_a=f"{prefix}_slide_1",
                elem_b=f"{prefix}_slide_1",
                name=f"{prefix} full-extension slide remains retained",
            )
            extended_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"{prefix} drawer extends outward",
            rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.40,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
