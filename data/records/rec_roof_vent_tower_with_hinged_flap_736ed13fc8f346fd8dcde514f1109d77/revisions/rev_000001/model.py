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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.59, 0.58, 1.0))
    dark_cavity = model.material("dark_cavity", rgba=(0.018, 0.020, 0.022, 1.0))
    hinge_metal = model.material("dark_hinge_metal", rgba=(0.26, 0.27, 0.26, 1.0))
    weathered_panel = model.material("weathered_flap", rgba=(0.47, 0.50, 0.49, 1.0))

    housing = model.part("housing")

    # Realistic rooftop scale: a sheet-metal tower on a broad roof flashing.
    base_x, base_y, base_t = 0.72, 0.60, 0.025
    duct_x, duct_y = 0.38, 0.34
    wall_t = 0.024
    duct_bottom, duct_top = base_t, 0.82
    duct_h = duct_top - duct_bottom
    front_x = duct_x / 2.0

    outlet_open_y = 0.290
    outlet_bottom = 0.340
    outlet_top = 0.705
    frame_bar = 0.035
    frame_depth = 0.040
    frame_outer_y = outlet_open_y + 2.0 * frame_bar
    frame_outer_h = (outlet_top - outlet_bottom) + 2.0 * frame_bar
    frame_x = front_x + frame_depth / 2.0
    hinge_x = front_x + 0.095
    hinge_z = outlet_top + frame_bar + 0.020

    housing.visual(
        Box((base_x, base_y, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=galvanized,
        name="roof_flashing",
    )
    housing.visual(
        Box((0.50, 0.42, 0.115)),
        origin=Origin(xyz=(0.0, 0.0, base_t + 0.115 / 2.0)),
        material=galvanized,
        name="raised_curb",
    )

    # Four sheet-metal walls make the upright duct visibly hollow rather than a
    # solid block.  The front face is split around the outlet opening.
    housing.visual(
        Box((wall_t, duct_y, duct_h)),
        origin=Origin(xyz=(-duct_x / 2.0 + wall_t / 2.0, 0.0, duct_bottom + duct_h / 2.0)),
        material=galvanized,
        name="rear_wall",
    )
    housing.visual(
        Box((duct_x, wall_t, duct_h)),
        origin=Origin(xyz=(0.0, duct_y / 2.0 - wall_t / 2.0, duct_bottom + duct_h / 2.0)),
        material=galvanized,
        name="side_wall_0",
    )
    housing.visual(
        Box((duct_x, wall_t, duct_h)),
        origin=Origin(xyz=(0.0, -duct_y / 2.0 + wall_t / 2.0, duct_bottom + duct_h / 2.0)),
        material=galvanized,
        name="side_wall_1",
    )
    housing.visual(
        Box((duct_x, duct_y, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, duct_top - wall_t / 2.0)),
        material=galvanized,
        name="top_cap",
    )
    housing.visual(
        Box((wall_t, duct_y, outlet_bottom - duct_bottom + 0.004)),
        origin=Origin(
            xyz=(front_x - wall_t / 2.0, 0.0, duct_bottom + (outlet_bottom - duct_bottom) / 2.0)
        ),
        material=galvanized,
        name="front_lower_wall",
    )
    housing.visual(
        Box((wall_t, duct_y, duct_top - outlet_top + 0.004)),
        origin=Origin(xyz=(front_x - wall_t / 2.0, 0.0, outlet_top + (duct_top - outlet_top) / 2.0)),
        material=galvanized,
        name="front_upper_wall",
    )
    housing.visual(
        Box((0.012, duct_y - 0.006, outlet_top - outlet_bottom - 0.020)),
        origin=Origin(xyz=(front_x - 0.115, 0.0, (outlet_top + outlet_bottom) / 2.0)),
        material=dark_cavity,
        name="recessed_shadow",
    )

    # Protruding, continuous outlet frame.
    housing.visual(
        Box((frame_depth, frame_outer_y, frame_bar)),
        origin=Origin(xyz=(frame_x, 0.0, outlet_top + frame_bar / 2.0)),
        material=galvanized,
        name="top_frame",
    )
    housing.visual(
        Box((frame_depth, frame_outer_y, frame_bar)),
        origin=Origin(xyz=(frame_x, 0.0, outlet_bottom - frame_bar / 2.0)),
        material=galvanized,
        name="bottom_frame",
    )
    housing.visual(
        Box((frame_depth, frame_bar, frame_outer_h)),
        origin=Origin(xyz=(frame_x, outlet_open_y / 2.0 + frame_bar / 2.0, (outlet_top + outlet_bottom) / 2.0)),
        material=galvanized,
        name="side_frame_0",
    )
    housing.visual(
        Box((frame_depth, frame_bar, frame_outer_h)),
        origin=Origin(xyz=(frame_x, -outlet_open_y / 2.0 - frame_bar / 2.0, (outlet_top + outlet_bottom) / 2.0)),
        material=galvanized,
        name="side_frame_1",
    )

    # Small raised screw heads and seams break up the sheet-metal surfaces.
    for idx, (yy, zz) in enumerate(
        (
            (-frame_outer_y / 2.0 + 0.035, outlet_top + frame_bar / 2.0),
            (frame_outer_y / 2.0 - 0.035, outlet_top + frame_bar / 2.0),
            (-frame_outer_y / 2.0 + 0.035, outlet_bottom - frame_bar / 2.0),
            (frame_outer_y / 2.0 - 0.035, outlet_bottom - frame_bar / 2.0),
        )
    ):
        housing.visual(
            Cylinder(radius=0.0075, length=0.006),
            origin=Origin(xyz=(front_x + frame_depth + 0.003, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"frame_screw_{idx}",
        )
    for yy in (-duct_y / 2.0 - 0.002, duct_y / 2.0 + 0.002):
        housing.visual(
            Box((0.018, 0.010, 0.60)),
            origin=Origin(xyz=(-0.015, yy, 0.43)),
            material=galvanized,
            name=f"standing_seam_{'neg' if yy < 0.0 else 'pos'}",
        )

    # Exposed top-edge hinge hardware is pulled forward from the frame so the
    # horizontal rotation axis is unambiguous.
    hinge_r = 0.015
    static_knuckle_y = (-0.155, 0.155)
    for idx, yy in enumerate(static_knuckle_y):
        housing.visual(
            Box((hinge_x - front_x + 0.010, 0.095, 0.010)),
            origin=Origin(xyz=((hinge_x + front_x) / 2.0, yy, hinge_z + 0.016)),
            material=hinge_metal,
            name=f"fixed_hinge_leaf_{idx}",
        )
        housing.visual(
            Box((0.014, 0.075, 0.050)),
            origin=Origin(xyz=(front_x + 0.010, yy, outlet_top + frame_bar + 0.013)),
            material=hinge_metal,
            name=f"hinge_standoff_{idx}",
        )
        housing.visual(
            Cylinder(radius=hinge_r, length=0.085),
            origin=Origin(xyz=(hinge_x, yy, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"fixed_knuckle_{idx}",
        )
    for idx, yy in enumerate((-0.203, 0.203)):
        housing.visual(
            Cylinder(radius=0.011, length=0.012),
            origin=Origin(xyz=(hinge_x, yy, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"pin_end_{idx}",
        )

    flap = model.part("flap")
    flap.visual(
        Box((0.020, 0.360, 0.400)),
        origin=Origin(xyz=(-0.040, 0.0, -0.205)),
        material=weathered_panel,
        name="panel",
    )
    flap.visual(
        Box((0.012, 0.330, 0.024)),
        origin=Origin(xyz=(-0.025, 0.0, -0.205)),
        material=galvanized,
        name="horizontal_stiffener",
    )
    flap.visual(
        Box((0.012, 0.030, 0.325)),
        origin=Origin(xyz=(-0.025, 0.0, -0.215)),
        material=galvanized,
        name="center_stiffener",
    )
    flap.visual(
        Box((0.075, 0.210, 0.010)),
        origin=Origin(xyz=(-0.0375, 0.0, -0.012)),
        material=hinge_metal,
        name="moving_hinge_leaf",
    )
    flap.visual(
        Cylinder(radius=hinge_r, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="moving_knuckle",
    )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("flap_hinge")

    ctx.check(
        "single top edge revolute flap",
        len(object_model.articulations) == 1
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and 1.0 <= hinge.motion_limits.upper <= 1.4,
        details=f"articulations={len(object_model.articulations)}, limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            housing,
            axis="x",
            positive_elem="panel",
            negative_elem="top_frame",
            min_gap=0.002,
            max_gap=0.020,
            name="closed flap sits just proud of outlet frame",
        )
        ctx.expect_overlap(
            flap,
            housing,
            axes="yz",
            elem_a="panel",
            elem_b="recessed_shadow",
            min_overlap=0.25,
            name="flap panel covers the framed outlet",
        )
        closed_panel_aabb = ctx.part_element_world_aabb(flap, elem="panel")

    with ctx.pose({hinge: 1.0}):
        opened_panel_aabb = ctx.part_element_world_aabb(flap, elem="panel")

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) / 2.0

    closed_x = _aabb_center_x(closed_panel_aabb)
    opened_x = _aabb_center_x(opened_panel_aabb)
    ctx.check(
        "positive hinge motion opens flap outward",
        closed_x is not None and opened_x is not None and opened_x > closed_x + 0.10,
        details=f"closed_x={closed_x}, opened_x={opened_x}",
    )

    return ctx.report()


object_model = build_object_model()
