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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = Material("galvanized_steel", rgba=(0.58, 0.62, 0.62, 1.0))
    bright_edge = Material("folded_frame_edge", rgba=(0.72, 0.75, 0.73, 1.0))
    dark_void = Material("dark_vent_interior", rgba=(0.03, 0.035, 0.035, 1.0))
    roof_membrane = Material("dark_roof_membrane", rgba=(0.08, 0.075, 0.07, 1.0))

    model.materials.extend([galvanized, bright_edge, dark_void, roof_membrane])

    housing = model.part("housing")

    # A low roof flashing/curb plate anchors the upright tower and makes the
    # object read as rooftop-mounted rather than a freestanding box.
    housing.visual(
        Box((0.78, 0.58, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=roof_membrane,
        name="roof_flashing",
    )

    wall = 0.035
    tower_w = 0.44
    tower_d = 0.30
    tower_h = 0.72
    zc = 0.030 + tower_h / 2.0

    # Thin sheet-metal duct walls leave the outlet visually hollow.
    housing.visual(
        Box((wall, tower_d, tower_h)),
        origin=Origin(xyz=(-tower_w / 2.0 + wall / 2.0, 0.0, zc)),
        material=galvanized,
        name="side_wall_0",
    )
    housing.visual(
        Box((wall, tower_d, tower_h)),
        origin=Origin(xyz=(tower_w / 2.0 - wall / 2.0, 0.0, zc)),
        material=galvanized,
        name="side_wall_1",
    )
    housing.visual(
        Box((tower_w, wall, tower_h)),
        origin=Origin(xyz=(0.0, tower_d / 2.0 - wall / 2.0, zc)),
        material=galvanized,
        name="rear_wall",
    )

    # Folded cap and base bands tie the side and rear walls into a single sheet
    # metal tower.
    housing.visual(
        Box((tower_w, tower_d, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.030 + tower_h - 0.020)),
        material=galvanized,
        name="top_cap",
    )
    housing.visual(
        Box((tower_w, tower_d, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=galvanized,
        name="base_band",
    )

    # Front outlet frame: a robust welded rectangular frame around the open
    # mouth of the duct.
    front_y = -tower_d / 2.0 - 0.003
    frame_depth = 0.055
    frame_w = 0.43
    frame_h = 0.36
    inner_w = 0.32
    inner_h = 0.25
    frame_cz = 0.485
    rail_h = (frame_h - inner_h) / 2.0
    side_w = (frame_w - inner_w) / 2.0

    housing.visual(
        Box((frame_w, frame_depth, rail_h)),
        origin=Origin(xyz=(0.0, front_y, frame_cz + inner_h / 2.0 + rail_h / 2.0)),
        material=bright_edge,
        name="outlet_top_frame",
    )
    housing.visual(
        Box((frame_w, frame_depth, rail_h)),
        origin=Origin(xyz=(0.0, front_y, frame_cz - inner_h / 2.0 - rail_h / 2.0)),
        material=bright_edge,
        name="outlet_bottom_frame",
    )
    housing.visual(
        Box((side_w, frame_depth, frame_h)),
        origin=Origin(xyz=(-inner_w / 2.0 - side_w / 2.0, front_y, frame_cz)),
        material=bright_edge,
        name="outlet_side_frame_0",
    )
    housing.visual(
        Box((side_w, frame_depth, frame_h)),
        origin=Origin(xyz=(inner_w / 2.0 + side_w / 2.0, front_y, frame_cz)),
        material=bright_edge,
        name="outlet_side_frame_1",
    )

    # A smaller proud stop lip sits around the opening for the flap to close
    # against.  It is fixed to the outlet frame and intentionally separate from
    # the moving flap.
    lip_depth = 0.018
    lip_y = -0.179
    lip_w = 0.36
    lip_h = 0.29
    clear_w = 0.31
    clear_h = 0.235
    lip_rail = (lip_h - clear_h) / 2.0
    lip_side = (lip_w - clear_w) / 2.0
    housing.visual(
        Box((lip_w, lip_depth, lip_rail)),
        origin=Origin(xyz=(0.0, lip_y, frame_cz + clear_h / 2.0 + lip_rail / 2.0)),
        material=galvanized,
        name="stop_lip_top",
    )
    housing.visual(
        Box((lip_w, lip_depth, lip_rail)),
        origin=Origin(xyz=(0.0, lip_y, frame_cz - clear_h / 2.0 - lip_rail / 2.0)),
        material=galvanized,
        name="stop_lip_bottom",
    )
    housing.visual(
        Box((lip_side, lip_depth, lip_h)),
        origin=Origin(xyz=(-clear_w / 2.0 - lip_side / 2.0, lip_y, frame_cz)),
        material=galvanized,
        name="stop_lip_side_0",
    )
    housing.visual(
        Box((lip_side, lip_depth, lip_h)),
        origin=Origin(xyz=(clear_w / 2.0 + lip_side / 2.0, lip_y, frame_cz)),
        material=galvanized,
        name="stop_lip_side_1",
    )

    # Dark internal slats are recessed behind the mouth, spanning into the side
    # frame so they read as supported grille vanes rather than floating strips.
    for idx, z in enumerate((0.420, 0.485, 0.550)):
        housing.visual(
            Box((0.355, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, -0.128, z)),
            material=dark_void,
            name=f"shadow_louver_{idx}",
        )

    # A fixed hinge strap along the top edge gives the flap a visible mounting
    # line while leaving a narrow clearance gap to the moving barrel.
    hinge_z = frame_cz + inner_h / 2.0 + 0.005
    hinge_y = -0.198
    housing.visual(
        Box((0.41, 0.010, 0.032)),
        origin=Origin(xyz=(0.0, -0.182, hinge_z + 0.006)),
        material=bright_edge,
        name="hinge_strap",
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.36, 0.012, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=galvanized,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.012, length=0.39),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_edge,
        name="top_roll",
    )
    flap.visual(
        Box((0.34, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.002, -0.292)),
        material=bright_edge,
        name="lower_drip_edge",
    )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("housing_to_flap")

    ctx.expect_overlap(
        flap,
        housing,
        axes="xz",
        elem_a="flap_panel",
        elem_b="stop_lip_top",
        min_overlap=0.01,
        name="closed flap overlaps the fixed top stop lip in projection",
    )
    ctx.expect_gap(
        housing,
        flap,
        axis="y",
        positive_elem="stop_lip_top",
        negative_elem="flap_panel",
        min_gap=0.001,
        max_gap=0.010,
        name="closed flap sits just proud of the stop lip",
    )

    rest_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({hinge: 1.0}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
        ctx.expect_gap(
            housing,
            flap,
            axis="y",
            positive_elem="outlet_bottom_frame",
            negative_elem="lower_drip_edge",
            min_gap=0.120,
            name="opened flap lower edge swings outward from the outlet frame",
        )

    ctx.check(
        "revolute flap moves outward/upward about the top edge",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < rest_aabb[0][1] - 0.05
        and open_aabb[0][2] > rest_aabb[0][2] + 0.04,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
