from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_access_hatch")

    curb_len = 1.18
    curb_w = 0.78
    curb_h = 0.25
    wall = 0.07
    flange_w = 0.11
    flange_t = 0.018
    gasket_h = 0.012

    lid_len = 1.32
    lid_w = 0.90
    lid_t = 0.055
    hinge_x = -0.66
    hinge_z = curb_h + gasket_h + lid_t / 2.0

    curb_mat = model.material("galvanized_curb", rgba=(0.58, 0.61, 0.62, 1.0))
    lid_mat = model.material("light_galvanized_lid", rgba=(0.74, 0.77, 0.77, 1.0))
    edge_mat = model.material("darker_folded_edges", rgba=(0.47, 0.50, 0.50, 1.0))
    rubber_mat = model.material("black_rubber_gasket", rgba=(0.015, 0.014, 0.012, 1.0))
    steel_mat = model.material("stainless_hardware", rgba=(0.80, 0.81, 0.78, 1.0))
    stay_mat = model.material("zinc_side_stays", rgba=(0.64, 0.66, 0.64, 1.0))

    curb = model.part("curb_frame")
    # A hollow curb: four raised walls plus a roof flashing ring, not a solid block.
    curb.visual(
        Box((wall, curb_w, curb_h)),
        origin=Origin(xyz=(-curb_len / 2.0 + wall / 2.0, 0.0, curb_h / 2.0)),
        material=curb_mat,
        name="rear_wall",
    )
    curb.visual(
        Box((wall, curb_w, curb_h)),
        origin=Origin(xyz=(curb_len / 2.0 - wall / 2.0, 0.0, curb_h / 2.0)),
        material=curb_mat,
        name="front_wall",
    )
    curb.visual(
        Box((curb_len, wall, curb_h)),
        origin=Origin(xyz=(0.0, curb_w / 2.0 - wall / 2.0, curb_h / 2.0)),
        material=curb_mat,
        name="side_wall_0",
    )
    curb.visual(
        Box((curb_len, wall, curb_h)),
        origin=Origin(xyz=(0.0, -curb_w / 2.0 + wall / 2.0, curb_h / 2.0)),
        material=curb_mat,
        name="side_wall_1",
    )
    curb.visual(
        Box((curb_len + 2.0 * flange_w, flange_w, flange_t)),
        origin=Origin(xyz=(0.0, curb_w / 2.0 + flange_w / 2.0, flange_t / 2.0)),
        material=curb_mat,
        name="flashing_strip_0",
    )
    curb.visual(
        Box((curb_len + 2.0 * flange_w, flange_w, flange_t)),
        origin=Origin(xyz=(0.0, -curb_w / 2.0 - flange_w / 2.0, flange_t / 2.0)),
        material=curb_mat,
        name="flashing_strip_1",
    )
    curb.visual(
        Box((flange_w, curb_w, flange_t)),
        origin=Origin(xyz=(curb_len / 2.0 + flange_w / 2.0, 0.0, flange_t / 2.0)),
        material=curb_mat,
        name="front_flashing",
    )
    curb.visual(
        Box((flange_w, curb_w, flange_t)),
        origin=Origin(xyz=(-curb_len / 2.0 - flange_w / 2.0, 0.0, flange_t / 2.0)),
        material=curb_mat,
        name="rear_flashing",
    )
    gasket_z = curb_h + gasket_h / 2.0
    curb.visual(
        Box((curb_len - 0.05, 0.040, gasket_h)),
        origin=Origin(xyz=(0.0, curb_w / 2.0 - 0.020, gasket_z)),
        material=rubber_mat,
        name="gasket_0",
    )
    curb.visual(
        Box((curb_len - 0.05, 0.040, gasket_h)),
        origin=Origin(xyz=(0.0, -curb_w / 2.0 + 0.020, gasket_z)),
        material=rubber_mat,
        name="gasket_1",
    )
    curb.visual(
        Box((0.040, curb_w - 0.05, gasket_h)),
        origin=Origin(xyz=(curb_len / 2.0 - 0.020, 0.0, gasket_z)),
        material=rubber_mat,
        name="front_gasket",
    )
    curb.visual(
        Box((0.040, curb_w - 0.05, gasket_h)),
        origin=Origin(xyz=(-curb_len / 2.0 + 0.020, 0.0, gasket_z)),
        material=rubber_mat,
        name="rear_gasket",
    )

    # Exposed rear hinge barrels and support webs on the fixed curb.
    for idx, y in enumerate((-0.31, 0.31)):
        curb.visual(
            Cylinder(radius=0.016, length=0.18),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel_mat,
            name=f"curb_hinge_barrel_{idx}",
        )
        curb.visual(
            Box((0.080, 0.18, 0.022)),
            origin=Origin(xyz=(hinge_x + 0.035, y, hinge_z - 0.047)),
            material=steel_mat,
            name=f"hinge_mount_{idx}",
        )
        curb.visual(
            Box((0.024, 0.18, 0.052)),
            origin=Origin(xyz=(hinge_x, y, hinge_z - 0.030)),
            material=steel_mat,
            name=f"hinge_web_{idx}",
        )

    stay_pivot_x = -0.43
    stay_pivot_z = 0.12
    stay_y = 0.54
    for idx, (y_sign, pin_name) in enumerate(((1.0, "stay_pin_0"), (-1.0, "stay_pin_1"))):
        y = y_sign * stay_y
        bracket_y = y_sign * 0.455
        curb.visual(
            Box((0.085, 0.130, 0.060)),
            origin=Origin(xyz=(stay_pivot_x, bracket_y, stay_pivot_z)),
            material=steel_mat,
            name=f"stay_base_{idx}",
        )
        curb.visual(
            Cylinder(radius=0.012, length=0.033),
            origin=Origin(
                xyz=(stay_pivot_x, y_sign * 0.511, stay_pivot_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel_mat,
            name=pin_name,
        )

    lid = model.part("lid")
    lid.visual(
        Box((lid_len, lid_w, lid_t)),
        origin=Origin(xyz=(lid_len / 2.0 + 0.025, 0.0, 0.0)),
        material=lid_mat,
        name="lid_panel",
    )
    lid.visual(
        Box((lid_len, 0.026, 0.095)),
        origin=Origin(xyz=(lid_len / 2.0 + 0.025, lid_w / 2.0 + 0.013, -0.055)),
        material=edge_mat,
        name="side_lip_0",
    )
    lid.visual(
        Box((lid_len, 0.026, 0.095)),
        origin=Origin(xyz=(lid_len / 2.0 + 0.025, -lid_w / 2.0 - 0.013, -0.055)),
        material=edge_mat,
        name="side_lip_1",
    )
    lid.visual(
        Box((0.030, lid_w + 0.052, 0.095)),
        origin=Origin(xyz=(lid_len + 0.040, 0.0, -0.055)),
        material=edge_mat,
        name="front_lip",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.080, 0.30, 0.024)),
        origin=Origin(xyz=(0.030, 0.0, 0.010)),
        material=steel_mat,
        name="lid_hinge_leaf",
    )

    handle_pivot_x = 1.04
    handle_pivot_z = 0.065
    for y, lug_name in zip((-0.135, 0.135), ("handle_lug_0", "handle_lug_1")):
        lid.visual(
            Box((0.055, 0.024, 0.034)),
            origin=Origin(xyz=(handle_pivot_x, y, 0.0445)),
            material=steel_mat,
            name=lug_name,
        )

    stay_len = 0.64
    stay_angle = 0.27
    stay_tip_x = stay_pivot_x + stay_len * math.cos(stay_angle)
    stay_tip_z = stay_pivot_z + stay_len * math.sin(stay_angle)
    stay_lid_x = stay_tip_x - hinge_x
    stay_lid_z = stay_tip_z - hinge_z
    for idx, y_sign in enumerate((1.0, -1.0)):
        lid.visual(
            Box((0.090, 0.035, 0.050)),
            origin=Origin(xyz=(stay_lid_x, y_sign * 0.493, stay_lid_z)),
            material=steel_mat,
            name=f"stay_lid_lug_{idx}",
        )

    handle = model.part("front_handle")
    for idx, y in enumerate((-0.105, 0.105)):
        handle.visual(
            Box((0.225, 0.018, 0.018)),
            origin=Origin(xyz=(0.1125, y, 0.0)),
            material=steel_mat,
            name=f"handle_arm_{idx}",
        )
    handle.visual(
        Cylinder(radius=0.010, length=0.246),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="handle_pivot_bar",
    )
    handle.visual(
        Cylinder(radius=0.016, length=0.25),
        origin=Origin(xyz=(0.225, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="handle_grip",
    )

    stay_parts = []
    for idx, y_sign in enumerate((1.0, -1.0)):
        stay = model.part(f"side_stay_{idx}")
        stay.visual(
            Cylinder(radius=0.025, length=0.025),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stay_mat,
            name="lower_eye",
        )
        stay.visual(
            Box((stay_len, 0.018, 0.018)),
            origin=Origin(
                xyz=(stay_len * math.cos(stay_angle) / 2.0, 0.0, stay_len * math.sin(stay_angle) / 2.0),
                rpy=(0.0, -stay_angle, 0.0),
            ),
            material=stay_mat,
            name="stay_bar",
        )
        stay.visual(
            Cylinder(radius=0.024, length=0.025),
            origin=Origin(
                xyz=(stay_len * math.cos(stay_angle), 0.0, stay_len * math.sin(stay_angle)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=stay_mat,
            name="upper_eye",
        )
        stay_parts.append((stay, y_sign))

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=curb,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.20),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(handle_pivot_x, 0.0, handle_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.35),
    )
    for idx, (stay, y_sign) in enumerate(stay_parts):
        model.articulation(
            f"stay_joint_{idx}",
            ArticulationType.REVOLUTE,
            parent=curb,
            child=stay,
            origin=Origin(xyz=(stay_pivot_x, y_sign * stay_y, stay_pivot_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=0.72),
            mimic=Mimic(joint="rear_hinge", multiplier=0.55),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    curb = object_model.get_part("curb_frame")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("front_handle")
    stay_0 = object_model.get_part("side_stay_0")
    stay_1 = object_model.get_part("side_stay_1")
    rear_hinge = object_model.get_articulation("rear_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.expect_gap(
        lid,
        curb,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_gasket",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid rests on the curb gasket",
    )
    ctx.expect_overlap(
        lid,
        curb,
        axes="xy",
        elem_a="lid_panel",
        elem_b="front_gasket",
        min_overlap=0.030,
        name="flat lid overhangs the front curb gasket",
    )
    ctx.expect_contact(
        handle,
        lid,
        elem_a="handle_pivot_bar",
        elem_b="handle_lug_0",
        contact_tol=0.001,
        name="handle pivot bar is captured by a lid lug",
    )
    ctx.expect_contact(
        stay_0,
        curb,
        elem_a="lower_eye",
        elem_b="stay_pin_0",
        contact_tol=0.001,
        name="first side stay rotates on its curb pin",
    )
    ctx.expect_contact(
        stay_1,
        curb,
        elem_a="lower_eye",
        elem_b="stay_pin_1",
        contact_tol=0.001,
        name="second side stay rotates on its curb pin",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    closed_stay = ctx.part_element_world_aabb(stay_0, elem="upper_eye")
    with ctx.pose({rear_hinge: 1.0}):
        raised_front = ctx.part_element_world_aabb(lid, elem="front_lip")
        raised_stay = ctx.part_element_world_aabb(stay_0, elem="upper_eye")
    ctx.check(
        "rear hinge raises the front edge of the lid",
        closed_front is not None
        and raised_front is not None
        and raised_front[1][2] > closed_front[1][2] + 0.35,
        details=f"closed_front={closed_front}, raised_front={raised_front}",
    )
    ctx.check(
        "side stay follows the lid through its support joint",
        closed_stay is not None
        and raised_stay is not None
        and raised_stay[1][2] > closed_stay[1][2] + 0.20,
        details=f"closed_stay={closed_stay}, raised_stay={raised_stay}",
    )

    flat_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({handle_pivot: 1.0}):
        lifted_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    ctx.check(
        "pull handle pivots upward from the lid",
        flat_grip is not None
        and lifted_grip is not None
        and lifted_grip[1][2] > flat_grip[1][2] + 0.12,
        details=f"flat_grip={flat_grip}, lifted_grip={lifted_grip}",
    )

    return ctx.report()


object_model = build_object_model()
