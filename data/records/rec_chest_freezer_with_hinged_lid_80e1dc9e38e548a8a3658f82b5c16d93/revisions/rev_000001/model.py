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
    mesh_from_cadquery,
)
import cadquery as cq


BODY_D = 0.48
BODY_W = 0.90
BODY_H = 0.42
WALL = 0.040
BOTTOM = 0.070
LID_D = 0.56
LID_W = 0.98
LID_T = 0.085
HINGE_X = BODY_D / 2.0 + 0.055
HINGE_Z = BODY_H + 0.035
FRONT_X = -BODY_D / 2.0


def _hollow_body_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_D, BODY_W, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .edges("|Z")
        .fillet(0.026)
    )
    inner = (
        cq.Workplane("XY")
        .box(BODY_D - 2.0 * WALL, BODY_W - 2.0 * WALL, BODY_H + 0.08)
        .translate((0.0, 0.0, BOTTOM + (BODY_H + 0.08) / 2.0))
    )
    return outer.cut(inner)


def _lid_shell() -> cq.Workplane:
    # The child frame is the rear hinge pin axis.  The thick lid lies forward
    # of that axis with a small rear clearance for the exposed barrel hinges.
    lid_center_x = -0.020 - LID_D / 2.0
    lid_center_z = (BODY_H + LID_T / 2.0) - HINGE_Z
    return (
        cq.Workplane("XY")
        .box(LID_D, LID_W, LID_T)
        .translate((lid_center_x, 0.0, lid_center_z))
        .edges("|Z")
        .fillet(0.020)
    )


def _lid_plug() -> cq.Workplane:
    lid_center_x = -0.020 - LID_D / 2.0
    lid_center_z = (BODY_H + LID_T / 2.0) - HINGE_Z
    plug = (
        cq.Workplane("XY")
        .box(BODY_D - 2.0 * WALL - 0.045, BODY_W - 2.0 * WALL - 0.045, 0.028)
        .translate((lid_center_x, 0.0, lid_center_z - LID_T / 2.0 - 0.014))
        .edges("|Z")
        .fillet(0.010)
    )
    return plug


def _gasket_ring() -> cq.Workplane:
    lid_center_x = -0.020 - LID_D / 2.0
    z = (BODY_H - HINGE_Z) - 0.006
    outer = cq.Workplane("XY").box(BODY_D - 2.0 * WALL - 0.010, BODY_W - 2.0 * WALL - 0.010, 0.012)
    inner = cq.Workplane("XY").box(BODY_D - 2.0 * WALL - 0.072, BODY_W - 2.0 * WALL - 0.072, 0.030)
    return outer.cut(inner).translate((lid_center_x, 0.0, z))


def _body_rim_ring() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_D + 0.012, BODY_W + 0.012, 0.030)
    inner = cq.Workplane("XY").box(BODY_D - 2.0 * WALL - 0.012, BODY_W - 2.0 * WALL - 0.012, 0.050)
    return outer.cut(inner).translate((0.0, 0.0, BODY_H - 0.015))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_chest_cooler")

    white_plastic = Material("warm_white_rotomolded_plastic", color=(0.92, 0.94, 0.91, 1.0))
    shadow_blue = Material("marine_blue_shadow", color=(0.05, 0.22, 0.38, 1.0))
    black_rubber = Material("black_rubber", color=(0.01, 0.012, 0.014, 1.0))
    stainless = Material("brushed_stainless", color=(0.70, 0.72, 0.70, 1.0))
    dark_plastic = Material("dark_drain_plastic", color=(0.025, 0.03, 0.035, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_hollow_body_shell(), "hollow_cooler_body", tolerance=0.0015),
        material=white_plastic,
        name="open_insulated_tub",
    )
    # A blue band at the open rim makes the thick marine cooler mouth legible
    # without capping the real hollow cavity.
    body.visual(
        mesh_from_cadquery(_body_rim_ring(), "blue_open_top_rim", tolerance=0.0015),
        material=shadow_blue,
        name="blue_top_rim",
    )

    # Drain spigot fixed body at the low front face.
    spigot_z = 0.105
    spigot_x = FRONT_X - 0.040
    body.visual(
        Cylinder(radius=0.034, length=0.080),
        origin=Origin(xyz=(spigot_x, 0.0, spigot_z), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="drain_spigot_body",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.052),
        origin=Origin(xyz=(FRONT_X - 0.102, 0.0, spigot_z - 0.008), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="drain_nozzle",
    )
    body.visual(
        Box((0.010, 0.110, 0.070)),
        origin=Origin(xyz=(FRONT_X - 0.001, 0.0, spigot_z), rpy=(0.0, 0.0, 0.0)),
        material=black_rubber,
        name="spigot_gasket",
    )

    # Two exposed rear barrel hinge bodies: each is made from separated knuckles
    # so the moving lid knuckle can occupy the center gap without intersection.
    hinge_centers_y = (-0.285, 0.285)
    hinge_len = 0.165
    body_knuckle_len = 0.046
    for idx, y0 in enumerate(hinge_centers_y):
        for seg, dy in enumerate((-hinge_len / 2.0 + body_knuckle_len / 2.0, hinge_len / 2.0 - body_knuckle_len / 2.0)):
            body.visual(
                Cylinder(radius=0.014, length=body_knuckle_len),
                origin=Origin(xyz=(HINGE_X, y0 + dy, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=stainless,
                name=f"hinge_{idx}_body_knuckle_{seg}",
            )
            body.visual(
                Box((0.055, body_knuckle_len, 0.010)),
                origin=Origin(xyz=(BODY_D / 2.0 + 0.027, y0 + dy, BODY_H - 0.006)),
                material=stainless,
                name=f"hinge_{idx}_body_web_{seg}",
            )
            body.visual(
                Box((0.016, body_knuckle_len, 0.028)),
                origin=Origin(xyz=(HINGE_X - 0.008, y0 + dy, BODY_H + 0.008)),
                material=stainless,
                name=f"hinge_{idx}_body_riser_{seg}",
            )
        body.visual(
            Box((0.007, hinge_len, 0.085)),
            origin=Origin(xyz=(BODY_D / 2.0 + 0.0035, y0, BODY_H - 0.045)),
            material=stainless,
            name=f"hinge_{idx}_body_leaf",
        )

    # Side bracket for the lower end of the hold-open stay rod.  It is fixed to
    # the tub side but clearanced around the rod eye.
    bracket_x = HINGE_X - 0.020 - 0.180 - 0.115
    bracket_z = HINGE_Z - 0.020 - 0.235
    side_y = BODY_W / 2.0
    body.visual(
        Box((0.095, 0.040, 0.070)),
        origin=Origin(xyz=(bracket_x, side_y + 0.020, bracket_z)),
        material=stainless,
        name="stay_bracket_base",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(xyz=(bracket_x, side_y + 0.055, bracket_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="stay_bracket_pin",
    )
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "thick_insulated_lid", tolerance=0.0015),
        material=white_plastic,
        name="main_lid_slab",
    )
    lid.visual(
        mesh_from_cadquery(_lid_plug(), "underside_lid_plug", tolerance=0.0015),
        material=white_plastic,
        name="lid_plug",
    )
    lid.visual(
        mesh_from_cadquery(_gasket_ring(), "lid_gasket_ring", tolerance=0.0015),
        material=black_rubber,
        name="rubber_gasket",
    )
    # Moving hinge leaves and center knuckles, aligned with the body knuckles.
    lid_leaf_center_x = -0.040
    lid_leaf_center_z = (BODY_H + LID_T + 0.004) - HINGE_Z
    center_knuckle_len = 0.058
    for idx, y0 in enumerate(hinge_centers_y):
        lid.visual(
            Cylinder(radius=0.0135, length=center_knuckle_len),
            origin=Origin(xyz=(0.0, y0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"hinge_{idx}_lid_knuckle",
        )
        lid.visual(
            Box((0.076, center_knuckle_len, 0.007)),
            origin=Origin(xyz=(lid_leaf_center_x, y0, lid_leaf_center_z - 0.0025)),
            material=stainless,
            name=f"hinge_{idx}_lid_leaf",
        )
        lid.visual(
            Box((0.020, center_knuckle_len, 0.025)),
            origin=Origin(xyz=(-0.010, y0, -0.006)),
            material=stainless,
            name=f"hinge_{idx}_lid_web",
        )

    # A side pin plate carries the hold-open stay rod.
    stay_pin_local = (-0.200, LID_W / 2.0 + 0.006, -0.020)
    lid.visual(
        Box((0.065, 0.008, 0.052)),
        origin=Origin(xyz=(stay_pin_local[0], stay_pin_local[1] - 0.010, stay_pin_local[2])),
        material=stainless,
        name="stay_pin_plate",
    )
    lid.visual(
        Cylinder(radius=0.016, length=0.028),
        origin=Origin(xyz=(stay_pin_local[0], stay_pin_local[1] + 0.006, stay_pin_local[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="stay_pin_head",
    )

    stay_rod = model.part("stay_rod")
    rod_vec = (-0.115, 0.0, -0.235)
    rod_len = math.sqrt(rod_vec[0] ** 2 + rod_vec[2] ** 2)
    rod_angle = math.atan2(-rod_vec[2], rod_vec[0])
    rod_unit = (rod_vec[0] / rod_len, 0.0, rod_vec[2] / rod_len)
    bar_start = (rod_unit[0] * 0.022, 0.0, rod_unit[2] * 0.022)
    bar_end = (rod_vec[0] - rod_unit[0] * 0.022, 0.0, rod_vec[2] - rod_unit[2] * 0.022)
    bar_center = (
        (bar_start[0] + bar_end[0]) / 2.0,
        0.0,
        (bar_start[2] + bar_end[2]) / 2.0,
    )
    stay_rod.visual(
        Box((rod_len - 0.044, 0.010, 0.010)),
        origin=Origin(xyz=bar_center, rpy=(0.0, rod_angle, 0.0)),
        material=stainless,
        name="flat_stay_bar",
    )
    stay_rod.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="upper_eye",
    )
    stay_rod.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=rod_vec, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="lower_eye",
    )

    spigot_lever = model.part("spigot_lever")
    lever_pivot_x = FRONT_X - 0.135
    lever_pivot_z = spigot_z
    spigot_lever.visual(
        Cylinder(radius=0.010, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="lever_pin",
    )
    spigot_lever.visual(
        Box((0.018, 0.088, 0.070)),
        origin=Origin(xyz=(-0.003, 0.0, -0.040)),
        material=shadow_blue,
        name="turn_lever",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.4, lower=0.0, upper=1.35),
    )
    model.articulation(
        "lid_to_stay_rod",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=stay_rod,
        origin=Origin(xyz=stay_pin_local),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.35, upper=1.15),
    )
    model.articulation(
        "body_to_spigot_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spigot_lever,
        origin=Origin(xyz=(lever_pivot_x, 0.0, lever_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5, lower=-0.85, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    stay_rod = object_model.get_part("stay_rod")
    spigot_lever = object_model.get_part("spigot_lever")
    lid_joint = object_model.get_articulation("body_to_lid")
    stay_joint = object_model.get_articulation("lid_to_stay_rod")
    lever_joint = object_model.get_articulation("body_to_spigot_lever")

    ctx.allow_overlap(
        body,
        stay_rod,
        elem_a="stay_bracket_pin",
        elem_b="lower_eye",
        reason="The lower eye of the hold-open stay is represented captured on the side bracket pin at the closed pose.",
    )
    ctx.allow_overlap(
        lid,
        stay_rod,
        elem_a="stay_pin_head",
        elem_b="upper_eye",
        reason="The lid-side stay pin intentionally passes through the rod eye to form the revolute mounting.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="open_insulated_tub",
        elem_b="rubber_gasket",
        reason="The black rubber gasket is intentionally shown slightly compressed against the cooler mouth for a sealed marine cooler lid.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="blue_top_rim",
        elem_b="rubber_gasket",
        reason="The gasket is intentionally shown compressed against the raised blue rim band that forms the cooler mouth seal.",
    )

    with ctx.pose({lid_joint: 0.0, stay_joint: 0.0, lever_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="main_lid_slab",
            negative_elem="open_insulated_tub",
            max_gap=0.004,
            max_penetration=0.0005,
            name="closed insulated lid sits on the cooler rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="main_lid_slab",
            elem_b="open_insulated_tub",
            min_overlap=0.30,
            name="wide lid covers the rectangular tub footprint",
        )
        ctx.expect_contact(
            spigot_lever,
            body,
            elem_a="lever_pin",
            elem_b="drain_nozzle",
            contact_tol=0.012,
            name="spigot lever pin is seated at the drain body",
        )
        ctx.expect_overlap(
            stay_rod,
            lid,
            axes="xz",
            elem_a="upper_eye",
            elem_b="stay_pin_head",
            min_overlap=0.010,
            name="stay rod upper eye is captured on the lid-side pin",
        )
        ctx.expect_overlap(
            stay_rod,
            body,
            axes="xz",
            elem_a="lower_eye",
            elem_b="stay_bracket_pin",
            min_overlap=0.010,
            name="stay rod lower eye aligns with the side bracket",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="rubber_gasket",
            negative_elem="open_insulated_tub",
            max_penetration=0.013,
            max_gap=0.002,
            name="rubber gasket is lightly compressed on the cooler mouth",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="rubber_gasket",
            negative_elem="blue_top_rim",
            max_penetration=0.013,
            max_gap=0.002,
            name="rubber gasket compresses against the raised rim band",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.20, stay_joint: 0.55}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid revolute hinge raises the heavy lid",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.20,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    lever_rest = ctx.part_world_aabb(spigot_lever)
    with ctx.pose({lever_joint: 0.70}):
        lever_turned = ctx.part_world_aabb(spigot_lever)
        ctx.check(
            "drain spigot lever turns on a revolute pin",
            lever_rest is not None
            and lever_turned is not None
            and abs(lever_turned[0][2] - lever_rest[0][2]) > 0.010,
            details=f"rest={lever_rest}, turned={lever_turned}",
        )

    return ctx.report()


object_model = build_object_model()
