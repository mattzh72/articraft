from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_room_air_purifier")

    white = model.material("warm_white_powder_coat", rgba=(0.92, 0.94, 0.93, 1.0))
    light_gray = model.material("soft_gray_plastic", rgba=(0.64, 0.67, 0.68, 1.0))
    dark = model.material("charcoal_grille", rgba=(0.04, 0.05, 0.055, 1.0))
    blue = model.material("blue_filter_media", rgba=(0.25, 0.46, 0.63, 1.0))
    teal = model.material("teal_hepa_label", rgba=(0.0, 0.46, 0.48, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.025, 0.025, 0.025, 1.0))

    width = 0.80
    depth = 0.45
    cabinet_bottom = 0.55
    cabinet_top = 1.50
    wall = 0.04

    housing = model.part("housing")

    # Low mobile medical stand and pedestal.
    housing.visual(
        Box((0.74, 0.54, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=light_gray,
        name="stand_base",
    )
    housing.visual(
        Box((0.84, 0.13, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=light_gray,
        name="wide_base_outrigger",
    )
    housing.visual(
        Box((0.16, 0.58, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=light_gray,
        name="deep_base_outrigger",
    )
    housing.visual(
        Box((0.18, 0.14, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=light_gray,
        name="pedestal_post",
    )
    for i, (x, y) in enumerate(((-0.30, -0.20), (-0.30, 0.20), (0.30, -0.20), (0.30, 0.20))):
        housing.visual(
            Box((0.13, 0.08, 0.025)),
            origin=Origin(xyz=(x, y, -0.0125)),
            material=rubber,
            name=f"caster_pad_{i}",
        )

    # Wide purifier cabinet.  The right side is framed open for the sliding pre-filter.
    cabinet_center_z = (cabinet_bottom + cabinet_top) / 2.0
    cabinet_height = cabinet_top - cabinet_bottom
    housing.visual(
        Box((width, depth, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_bottom + 0.04)),
        material=white,
        name="lower_plenum",
    )
    housing.visual(
        Box((wall, depth, cabinet_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, cabinet_center_z)),
        material=white,
        name="side_wall",
    )
    housing.visual(
        Box((width, wall, cabinet_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, cabinet_center_z)),
        material=white,
        name="front_face",
    )
    housing.visual(
        Box((width, wall, cabinet_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, cabinet_center_z)),
        material=white,
        name="rear_face",
    )

    top_rail_z = cabinet_top + 0.025
    housing.visual(
        Box((width, 0.055, 0.05)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.0275, top_rail_z)),
        material=white,
        name="top_front_rail",
    )
    housing.visual(
        Box((width, 0.055, 0.05)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.0275, top_rail_z)),
        material=white,
        name="top_rear_rail",
    )
    housing.visual(
        Box((0.055, depth - 0.10, 0.05)),
        origin=Origin(xyz=(-width / 2.0 + 0.0275, 0.0, top_rail_z)),
        material=white,
        name="top_side_rail",
    )
    housing.visual(
        Box((0.055, depth - 0.10, 0.05)),
        origin=Origin(xyz=(width / 2.0 - 0.0275, 0.0, top_rail_z)),
        material=white,
        name="side_slot_top_stile",
    )

    # Side opening frame and guide rails for the side-removable pre-filter.
    filter_center_z = 1.07
    filter_height = 0.70
    side_x = width / 2.0 + 0.018
    housing.visual(
        Box((0.045, 0.030, 0.78)),
        origin=Origin(xyz=(side_x, 0.174, filter_center_z)),
        material=white,
        name="slot_front_jamb",
    )
    housing.visual(
        Box((0.045, 0.030, 0.78)),
        origin=Origin(xyz=(side_x, -0.174, filter_center_z)),
        material=white,
        name="slot_rear_jamb",
    )
    housing.visual(
        Box((0.40, depth, 0.024)),
        origin=Origin(xyz=(0.275, 0.0, filter_center_z + filter_height / 2.0 + 0.018)),
        material=light_gray,
        name="side_top_guide",
    )
    housing.visual(
        Box((0.40, depth, 0.024)),
        origin=Origin(xyz=(0.275, 0.0, filter_center_z - filter_height / 2.0 - 0.018)),
        material=light_gray,
        name="side_bottom_guide",
    )

    # Finished front outlet grille.
    front_grille = VentGrilleGeometry(
        (0.58, 0.62),
        frame=0.020,
        face_thickness=0.006,
        duct_depth=0.024,
        slat_pitch=0.036,
        slat_width=0.014,
        slat_angle_deg=28.0,
        corner_radius=0.012,
        slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=2, divider_width=0.006),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.002),
    )
    housing.visual(
        mesh_from_geometry(front_grille, "front_grille"),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.010, 1.08), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="front_grille",
    )

    # Parent hinge hardware: two fixed hinge knuckles and a rear reinforcement bar.
    hinge_y = -depth / 2.0 - 0.020
    hinge_z = cabinet_top + 0.075
    housing.visual(
        Box((0.72, 0.040, 0.032)),
        origin=Origin(xyz=(0.0, hinge_y + 0.010, cabinet_top + 0.032)),
        material=light_gray,
        name="rear_hinge_backbone",
    )
    for i, x in enumerate((-0.14, 0.14)):
        housing.visual(
            Box((0.12, 0.026, 0.032)),
            origin=Origin(xyz=(x, hinge_y + 0.004, cabinet_top + 0.044)),
            material=light_gray,
            name=f"hinge_saddle_{i}",
        )
        housing.visual(
            Cylinder(radius=0.016, length=0.12),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=light_gray,
            name=f"fixed_hinge_barrel_{i}",
        )

    # Hinged top HEPA access lid.
    lid = model.part("lid")
    lid.visual(
        Box((0.78, 0.43, 0.045)),
        origin=Origin(xyz=(0.0, 0.235, -0.0025)),
        material=white,
        name="lid_panel",
    )
    lid.visual(
        Box((0.24, 0.026, 0.034)),
        origin=Origin(xyz=(0.0, 0.428, 0.037)),
        material=light_gray,
        name="front_handle",
    )
    lid.visual(
        Box((0.25, 0.12, 0.004)),
        origin=Origin(xyz=(0.0, 0.235, 0.022)),
        material=teal,
        name="hepa_label",
    )
    for i, (x, barrel_len) in enumerate(((-0.28, 0.12), (0.0, 0.12), (0.28, 0.12))):
        lid.visual(
            Cylinder(radius=0.016, length=barrel_len),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=light_gray,
            name=f"lid_hinge_barrel_{i}",
        )
        lid.visual(
            Box((barrel_len, 0.050, 0.018)),
            origin=Origin(xyz=(x, 0.035, 0.005)),
            material=light_gray,
            name=f"lid_hinge_leaf_{i}",
        )

    model.articulation(
        "housing_to_lid",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.22),
    )

    # Sliding side pre-filter cassette.  It is at the side opening in the zero pose
    # and translates outward along +X while long guide tongues remain captured.
    pre_filter = model.part("pre_filter")
    pre_filter.visual(
        Box((0.040, 0.320, filter_height)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=white,
        name="filter_cassette",
    )
    filter_media = SlotPatternPanelGeometry(
        (0.53, 0.235),
        0.004,
        slot_size=(0.040, 0.006),
        pitch=(0.060, 0.024),
        frame=0.014,
        corner_radius=0.004,
        stagger=True,
    )
    pre_filter.visual(
        mesh_from_geometry(filter_media, "prefilter_media"),
        origin=Origin(xyz=(0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="filter_media",
    )
    pre_filter.visual(
        Box((0.50, 0.035, 0.012)),
        origin=Origin(xyz=(-0.190, 0.0, 0.344)),
        material=light_gray,
        name="top_runner",
    )
    pre_filter.visual(
        Box((0.50, 0.035, 0.012)),
        origin=Origin(xyz=(-0.190, 0.0, -0.344)),
        material=light_gray,
        name="bottom_runner",
    )
    pre_filter.visual(
        Box((0.030, 0.090, 0.24)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=light_gray,
        name="pull_tab",
    )
    pre_filter.visual(
        Box((0.010, 0.055, 0.15)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=dark,
        name="finger_grip",
    )

    model.articulation(
        "housing_to_pre_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=pre_filter,
        origin=Origin(xyz=(width / 2.0 + 0.005, 0.0, filter_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lid = object_model.get_part("lid")
    pre_filter = object_model.get_part("pre_filter")
    lid_joint = object_model.get_articulation("housing_to_lid")
    filter_joint = object_model.get_articulation("housing_to_pre_filter")

    def aabb_center(aabb, index: int) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][index] + aabb[1][index]) / 2.0

    ctx.expect_gap(
        lid,
        housing,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="top_front_rail",
        min_gap=0.0,
        max_gap=0.003,
        name="closed HEPA lid sits on top rim",
    )
    ctx.expect_overlap(
        lid,
        housing,
        axes="xy",
        elem_a="lid_panel",
        elem_b="top_front_rail",
        min_overlap=0.03,
        name="top lid spans the cabinet opening",
    )
    ctx.expect_gap(
        housing,
        pre_filter,
        axis="z",
        positive_elem="side_top_guide",
        negative_elem="top_runner",
        min_gap=0.0,
        max_gap=0.010,
        name="upper guide clears the filter runner",
    )
    ctx.expect_gap(
        pre_filter,
        housing,
        axis="z",
        positive_elem="bottom_runner",
        negative_elem="side_bottom_guide",
        min_gap=0.0,
        max_gap=0.010,
        name="lower runner rides above lower guide",
    )
    ctx.expect_within(
        pre_filter,
        housing,
        axes="y",
        inner_elem="filter_cassette",
        outer_elem="side_top_guide",
        margin=0.02,
        name="pre-filter cassette is centered in the side slot",
    )
    ctx.expect_overlap(
        pre_filter,
        housing,
        axes="x",
        elem_a="top_runner",
        elem_b="side_top_guide",
        min_overlap=0.08,
        name="filter runner is retained while closed",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="front_handle")
    closed_handle_z = aabb_center(closed_lid_aabb, 2)
    with ctx.pose({lid_joint: 1.10}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="front_handle")
        opened_handle_z = aabb_center(opened_lid_aabb, 2)
    ctx.check(
        "rear-hinged lid opens upward",
        closed_handle_z is not None
        and opened_handle_z is not None
        and opened_handle_z > closed_handle_z + 0.20,
        details=f"closed_z={closed_handle_z}, opened_z={opened_handle_z}",
    )

    closed_filter_aabb = ctx.part_element_world_aabb(pre_filter, elem="filter_cassette")
    closed_filter_x = aabb_center(closed_filter_aabb, 0)
    with ctx.pose({filter_joint: 0.22}):
        ctx.expect_overlap(
            pre_filter,
            housing,
            axes="x",
            elem_a="top_runner",
            elem_b="side_top_guide",
            min_overlap=0.08,
            name="extended filter runner remains captured",
        )
        extended_filter_aabb = ctx.part_element_world_aabb(pre_filter, elem="filter_cassette")
        extended_filter_x = aabb_center(extended_filter_aabb, 0)
    ctx.check(
        "side pre-filter slides outward",
        closed_filter_x is not None
        and extended_filter_x is not None
        and extended_filter_x > closed_filter_x + 0.18,
        details=f"closed_x={closed_filter_x}, extended_x={extended_filter_x}",
    )

    return ctx.report()


object_model = build_object_model()
