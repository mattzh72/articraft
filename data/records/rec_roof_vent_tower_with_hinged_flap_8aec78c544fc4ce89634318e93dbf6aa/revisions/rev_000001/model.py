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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized_metal", rgba=(0.62, 0.66, 0.66, 1.0))
    dark_shadow = model.material("dark_duct_interior", rgba=(0.04, 0.045, 0.045, 1.0))
    hinge_steel = model.material("brushed_hinge_steel", rgba=(0.46, 0.48, 0.48, 1.0))
    weathered_roof = model.material("weathered_roof_flashing", rgba=(0.18, 0.20, 0.19, 1.0))

    housing = model.part("housing")

    # Real-world scale: a compact rooftop vent tower about waist height,
    # sitting on a low roof curb/flashing.
    base_z = 0.08
    body_h = 1.10
    body_top = base_z + body_h
    body_d = 0.38
    body_w = 0.52
    wall_t = 0.035

    outlet_w = 0.40
    outlet_bottom = 0.86
    outlet_top = 1.12
    outlet_h = outlet_top - outlet_bottom

    frame_depth = 0.070
    frame_bar = 0.060
    frame_front_x = body_d / 2.0 + frame_depth
    frame_top_z = outlet_top + frame_bar
    hinge_r = 0.018
    hinge_x = frame_front_x + hinge_r + 0.002
    hinge_z = frame_top_z + hinge_r + 0.002

    housing.visual(
        Box((0.82, 0.72, base_z)),
        origin=Origin(xyz=(0.0, 0.0, base_z / 2.0)),
        material=weathered_roof,
        name="roof_flashing",
    )
    housing.visual(
        Box((body_d, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, base_z + 0.017)),
        material=dark_shadow,
        name="duct_shadow",
    )

    # Rectangular hollow duct walls.  The front wall is broken around the outlet
    # so the opening reads as real void rather than a painted rectangle.
    housing.visual(
        Box((wall_t, body_w, body_h)),
        origin=Origin(xyz=(-body_d / 2.0 + wall_t / 2.0, 0.0, base_z + body_h / 2.0)),
        material=galvanized,
        name="rear_wall",
    )
    housing.visual(
        Box((body_d, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -body_w / 2.0 + wall_t / 2.0, base_z + body_h / 2.0)),
        material=galvanized,
        name="side_wall_0",
    )
    housing.visual(
        Box((body_d, wall_t, body_h)),
        origin=Origin(xyz=(0.0, body_w / 2.0 - wall_t / 2.0, base_z + body_h / 2.0)),
        material=galvanized,
        name="side_wall_1",
    )
    housing.visual(
        Box((wall_t, body_w, outlet_bottom - base_z)),
        origin=Origin(
            xyz=(body_d / 2.0 - wall_t / 2.0, 0.0, base_z + (outlet_bottom - base_z) / 2.0)
        ),
        material=galvanized,
        name="front_lower_wall",
    )
    housing.visual(
        Box((wall_t, body_w, body_top - outlet_top)),
        origin=Origin(
            xyz=(body_d / 2.0 - wall_t / 2.0, 0.0, outlet_top + (body_top - outlet_top) / 2.0)
        ),
        material=galvanized,
        name="front_header",
    )
    jamb_w = (body_w - outlet_w) / 2.0
    housing.visual(
        Box((wall_t, jamb_w + 0.006, outlet_h + 0.010)),
        origin=Origin(
            xyz=(
                body_d / 2.0 - wall_t / 2.0,
                -(outlet_w / 2.0 + jamb_w / 2.0),
                (outlet_bottom + outlet_top) / 2.0,
            )
        ),
        material=galvanized,
        name="front_jamb_0",
    )
    housing.visual(
        Box((wall_t, jamb_w + 0.006, outlet_h + 0.010)),
        origin=Origin(
            xyz=(
                body_d / 2.0 - wall_t / 2.0,
                outlet_w / 2.0 + jamb_w / 2.0,
                (outlet_bottom + outlet_top) / 2.0,
            )
        ),
        material=galvanized,
        name="front_jamb_1",
    )

    # Deep projecting outlet frame with visible top edge for the weather flap.
    frame_w = outlet_w + 2.0 * frame_bar
    frame_z_center = (outlet_bottom + outlet_top) / 2.0
    housing.visual(
        Box((frame_depth, frame_w, frame_bar)),
        origin=Origin(xyz=(body_d / 2.0 + frame_depth / 2.0, 0.0, outlet_top + frame_bar / 2.0)),
        material=galvanized,
        name="outlet_frame_top",
    )
    housing.visual(
        Box((frame_depth, frame_w, frame_bar)),
        origin=Origin(xyz=(body_d / 2.0 + frame_depth / 2.0, 0.0, outlet_bottom - frame_bar / 2.0)),
        material=galvanized,
        name="outlet_frame_bottom",
    )
    housing.visual(
        Box((frame_depth, frame_bar, outlet_h + 2.0 * frame_bar)),
        origin=Origin(
            xyz=(body_d / 2.0 + frame_depth / 2.0, -(outlet_w / 2.0 + frame_bar / 2.0), frame_z_center)
        ),
        material=galvanized,
        name="outlet_frame_side_0",
    )
    housing.visual(
        Box((frame_depth, frame_bar, outlet_h + 2.0 * frame_bar)),
        origin=Origin(
            xyz=(body_d / 2.0 + frame_depth / 2.0, outlet_w / 2.0 + frame_bar / 2.0, frame_z_center)
        ),
        material=galvanized,
        name="outlet_frame_side_1",
    )

    # Small hinge knuckles mounted to the top of the outlet frame.
    parent_knuckles = [(-0.190, 0.065), (0.0, 0.070), (0.190, 0.065)]
    for index, (y_pos, length) in enumerate(parent_knuckles):
        housing.visual(
            Cylinder(radius=hinge_r, length=length),
            origin=Origin(xyz=(hinge_x, y_pos, hinge_z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=hinge_steel,
            name=f"hinge_knuckle_{index}",
        )
        leaf_end_x = hinge_x - 0.004
        housing.visual(
            Box((leaf_end_x - frame_front_x + 0.004, length, 0.008)),
            origin=Origin(
                xyz=((frame_front_x + leaf_end_x) / 2.0, y_pos, frame_top_z + 0.004)
            ),
            material=hinge_steel,
            name=f"hinge_leaf_{index}",
        )
    housing.visual(
        Cylinder(radius=0.008, length=0.50),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="hinge_pin",
    )

    flap = model.part("flap")

    # The weather flap is intentionally short relative to the tall tower, but
    # formed as a deep tray-like panel with side cheeks and bottom lip to shed
    # rain away from the outlet.
    flap_w = 0.50
    flap_h = 0.34
    flap_depth = 0.105
    panel_t = 0.036
    top_clearance = 0.025

    flap.visual(
        Box((panel_t, flap_w, flap_h)),
        origin=Origin(xyz=(0.022, 0.0, -top_clearance - flap_h / 2.0)),
        material=galvanized,
        name="flap_panel",
    )
    flap.visual(
        Box((flap_depth, 0.020, flap_h)),
        origin=Origin(xyz=(flap_depth / 2.0, -flap_w / 2.0 + 0.010, -top_clearance - flap_h / 2.0)),
        material=galvanized,
        name="side_cheek_0",
    )
    flap.visual(
        Box((flap_depth, 0.020, flap_h)),
        origin=Origin(xyz=(flap_depth / 2.0, flap_w / 2.0 - 0.010, -top_clearance - flap_h / 2.0)),
        material=galvanized,
        name="side_cheek_1",
    )
    flap.visual(
        Box((flap_depth, flap_w, 0.026)),
        origin=Origin(xyz=(flap_depth / 2.0, 0.0, -top_clearance - flap_h + 0.013)),
        material=galvanized,
        name="bottom_lip",
    )

    child_knuckles = [(-0.100, 0.090), (0.100, 0.090)]
    for index, (y_pos, length) in enumerate(child_knuckles):
        flap.visual(
            Cylinder(radius=0.016, length=length),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=hinge_steel,
            name=f"flap_knuckle_{index}",
        )
        flap.visual(
            Box((0.058, length, 0.035)),
            origin=Origin(xyz=(0.031, y_pos, -0.030)),
            material=hinge_steel,
            name=f"flap_leaf_{index}",
        )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        # The closed flap hangs downward from its top edge.  A -Y axis makes
        # positive joint motion lift the free edge upward and out from the vent.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("housing_to_flap")

    ctx.check(
        "single revolute weather flap",
        hinge is not None
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and len(object_model.articulations) == 1,
        details=f"hinge={hinge}, articulations={len(object_model.articulations)}",
    )

    for knuckle_name in ("flap_knuckle_0", "flap_knuckle_1"):
        ctx.allow_overlap(
            housing,
            flap,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            reason="The fixed hinge pin is intentionally captured through the flap knuckle bore.",
        )
        ctx.expect_within(
            housing,
            flap,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem=knuckle_name,
            margin=0.001,
            name=f"{knuckle_name} surrounds hinge pin",
        )
        ctx.expect_overlap(
            housing,
            flap,
            axes="y",
            min_overlap=0.080,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            name=f"{knuckle_name} retained on hinge pin",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            housing,
            axis="x",
            min_gap=0.0,
            max_gap=0.030,
            positive_elem="flap_panel",
            negative_elem="outlet_frame_top",
            name="closed flap sits proud of outlet frame",
        )
        ctx.expect_overlap(
            flap,
            housing,
            axes="yz",
            min_overlap=0.040,
            elem_a="flap_panel",
            elem_b="outlet_frame_side_0",
            name="flap covers framed outlet face",
        )

    housing_aabb = ctx.part_element_world_aabb(housing, elem="side_wall_0")
    flap_aabb = ctx.part_world_aabb(flap)
    if housing_aabb is None or flap_aabb is None:
        ctx.fail("flap proportions are measurable", "missing AABB for tower body or flap")
    else:
        h_lo, h_hi = housing_aabb
        f_lo, f_hi = flap_aabb
        housing_depth = h_hi[0] - h_lo[0]
        housing_height = h_hi[2] - h_lo[2]
        flap_depth = f_hi[0] - f_lo[0]
        flap_height = f_hi[2] - f_lo[2]
        ctx.check(
            "flap is short and deep",
            flap_height < 0.38 * housing_height and flap_depth > 0.18 * housing_depth,
            details=(
                f"flap_height={flap_height:.3f}, housing_height={housing_height:.3f}, "
                f"flap_depth={flap_depth:.3f}, housing_depth={housing_depth:.3f}"
            ),
        )

    def element_xz_center(part, elem_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) / 2.0, (lo[2] + hi[2]) / 2.0)

    with ctx.pose({hinge: 0.0}):
        closed_lip = element_xz_center(flap, "bottom_lip")
    with ctx.pose({hinge: 1.0}):
        open_lip = element_xz_center(flap, "bottom_lip")
    ctx.check(
        "positive hinge opens flap outward and upward",
        closed_lip is not None
        and open_lip is not None
        and open_lip[0] > closed_lip[0] + 0.12
        and open_lip[1] > closed_lip[1] + 0.10,
        details=f"closed_lip={closed_lip}, open_lip={open_lip}",
    )

    return ctx.report()


object_model = build_object_model()
