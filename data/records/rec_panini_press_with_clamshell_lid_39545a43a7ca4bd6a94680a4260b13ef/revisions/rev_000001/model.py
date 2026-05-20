from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    mesh_from_cadquery,
)


BASE_W = 0.315
BASE_D = 0.258
BASE_H = 0.072
BASE_FLOOR = 0.017
BASE_WALL = 0.008
BASE_TOP_Z = BASE_H

LID_W = 0.306
LID_D = 0.250
LID_H = 0.050
LID_WALL = 0.006
SEAM_GAP = 0.0014

HINGE_Z = BASE_TOP_Z + SEAM_GAP
HINGE_Y = BASE_D / 2.0 - 0.010

STRAP_HINGE_Y = -BASE_D / 2.0 - 0.003
STRAP_HINGE_Z = BASE_TOP_Z - 0.011
STRAP_BARREL_R = 0.004

KNOB_Y = -BASE_D / 2.0
TIMER_PANEL_Y = -BASE_D / 2.0 + 0.016
KNOB_Z = 0.038


def _base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_W, BASE_D, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.024)
    )

    cavity = (
        cq.Workplane("XY")
        .box(
            BASE_W - 2.0 * BASE_WALL,
            BASE_D - 2.0 * BASE_WALL - 0.016,
            BASE_H - BASE_FLOOR + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.004, BASE_FLOOR))
        .edges("|Z")
        .fillet(0.014)
    )

    front_recess = (
        cq.Workplane("XY")
        .box(0.086, 0.020, 0.044, centered=(True, True, False))
        .translate((0.0, -BASE_D / 2.0 + 0.009, 0.016))
        .edges("|Z")
        .fillet(0.008)
    )

    front_panel = (
        cq.Workplane("XY")
        .box(0.102, 0.010, 0.034, centered=(True, True, False))
        .translate((0.0, -BASE_D / 2.0 + 0.005, 0.019))
        .edges("|Z")
        .fillet(0.004)
    )

    rear_hinge_pad = (
        cq.Workplane("XY")
        .box(0.238, 0.018, 0.004, centered=(True, True, False))
        .translate((0.0, BASE_D / 2.0 - 0.018, BASE_TOP_Z - 0.004))
        .edges("|Z")
        .fillet(0.003)
    )

    feet = []
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            feet.append(
                cq.Workplane("XY")
                .cylinder(
                    0.005,
                    0.014,
                    centered=(True, True, False),
                )
                .translate(
                    (
                        x_sign * (BASE_W / 2.0 - 0.044),
                        y_sign * (BASE_D / 2.0 - 0.044),
                        0.0,
                    )
                )
            )

    result = body.cut(cavity).cut(front_recess).union(rear_hinge_pad)
    for foot in feet:
        result = result.union(foot)
    return result


def _latch_mount_shape() -> cq.Workplane:
    main_block = (
        cq.Workplane("XY")
        .box(0.050, 0.016, 0.018, centered=(True, True, False))
        .translate((0.0, -BASE_D / 2.0 + 0.014, STRAP_HINGE_Z - STRAP_BARREL_R - 0.018))
        .edges("|Z")
        .fillet(0.004)
    )
    front_web = (
        cq.Workplane("XY")
        .box(0.034, 0.008, 0.016, centered=(True, True, False))
        .translate((0.0, -BASE_D / 2.0 + 0.008, STRAP_HINGE_Z - STRAP_BARREL_R - 0.016))
        .edges("|Z")
        .fillet(0.003)
    )
    return main_block.union(front_web)


def _timer_panel_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.062, 0.008, 0.028, centered=(True, True, False))
        .translate((0.0, TIMER_PANEL_Y, KNOB_Z - 0.014))
        .edges("|Z")
        .fillet(0.0015)
    )


def _lid_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H, centered=(True, True, False))
        .translate((0.0, -LID_D / 2.0, 0.0))
        .edges("|Z")
        .fillet(0.022)
    )

    crown = (
        cq.Workplane("XY")
        .box(LID_W - 0.042, LID_D - 0.054, 0.016, centered=(True, True, False))
        .translate((0.0, -LID_D / 2.0 + 0.003, LID_H - 0.004))
        .edges("|Z")
        .fillet(0.006)
    )

    cavity = (
        cq.Workplane("XY")
        .box(
            LID_W - 2.0 * LID_WALL,
            LID_D - 0.030,
            LID_H - LID_WALL + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, -LID_D / 2.0 + 0.004, 0.0))
        .edges("|Z")
        .fillet(0.016)
    )

    rear_skirt = (
        cq.Workplane("XY")
        .box(LID_W - 0.080, 0.018, 0.010, centered=(True, True, False))
        .translate((0.0, -0.012, 0.004))
        .edges("|Z")
        .fillet(0.003)
    )

    return outer.union(crown).union(rear_skirt).cut(cavity)


def _lid_hinge_bridge_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.228, 0.014, 0.008, centered=(True, True, False))
        .translate((0.0, -0.007, -SEAM_GAP))
        .edges("|Z")
        .fillet(0.002)
    )


def _lid_catch_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.052, 0.008, 0.008, centered=(True, True, False))
        .translate((0.0, -LID_D + 0.001, 0.018))
        .edges("|Z")
        .fillet(0.002)
    )


def _strap_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("XY")
        .cylinder(0.040, STRAP_BARREL_R, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )
    stem = (
        cq.Workplane("XY")
        .box(0.038, 0.004, 0.026, centered=(True, True, False))
        .translate((0.0, -0.004, 0.003))
        .edges("|Z")
        .fillet(0.0015)
    )
    hook = (
        cq.Workplane("XY")
        .box(0.038, 0.008, 0.008, centered=(True, True, False))
        .translate((0.0, -0.008, 0.028))
        .edges("|Z")
        .fillet(0.002)
    )
    return barrel.union(stem).union(hook)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamshell_sandwich_press")

    base_finish = model.material("base_finish", rgba=(0.35, 0.37, 0.39, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.89, 0.90, 0.88, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    strap_finish = model.material("strap_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base"),
        material=base_finish,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(_latch_mount_shape(), "latch_mount"),
        material=base_finish,
        name="latch_mount",
    )
    base.visual(
        mesh_from_cadquery(_timer_panel_shape(), "timer_panel"),
        material=panel_finish,
        name="timer_panel",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "lid"),
        material=lid_finish,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_lid_hinge_bridge_shape(), "lid_hinge_bridge"),
        material=lid_finish,
        name="lid_hinge_bridge",
    )
    lid.visual(
        mesh_from_cadquery(_lid_catch_shape(), "lid_catch"),
        material=lid_finish,
        name="lid_catch",
    )

    strap = model.part("strap")
    strap.visual(
        mesh_from_cadquery(_strap_shape(), "strap"),
        material=strap_finish,
        name="strap_body",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=panel_finish,
        name="knob_shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.020,
                body_style="skirted",
                top_diameter=0.026,
                skirt=KnobSkirt(0.040, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "timer_knob",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="knob_shell",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=18.0, velocity=1.5),
    )
    model.articulation(
        "base_to_strap",
        ArticulationType.REVOLUTE,
        parent=base,
        child=strap,
        origin=Origin(xyz=(0.0, STRAP_HINGE_Y, STRAP_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=4.0, velocity=2.2),
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.0, KNOB_Y, KNOB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    strap = object_model.get_part("strap")
    knob = object_model.get_part("knob")
    lid_hinge = object_model.get_articulation("base_to_lid")
    strap_hinge = object_model.get_articulation("base_to_strap")
    knob_joint = object_model.get_articulation("base_to_knob")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.0006,
            max_gap=0.0045,
            positive_elem="lid_shell",
            negative_elem="base_shell",
            name="lid_seam_gap_reads_as_casting_split",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.220,
            elem_a="lid_shell",
            elem_b="base_shell",
            name="lid_covers_the_base_footprint",
        )
        ctx.expect_gap(
            strap,
            base,
            axis="z",
            max_penetration=1e-5,
            max_gap=0.0006,
            positive_elem="strap_body",
            negative_elem="latch_mount",
            name="strap_is_anchored_at_the_base_lip",
        )
        ctx.expect_gap(
            base,
            knob,
            axis="y",
            max_penetration=1e-5,
            max_gap=0.0025,
            positive_elem="timer_panel",
            negative_elem="knob_shaft",
            name="timer_knob_sits_in_a_real_front_recess",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    open_aabb = None
    with ctx.pose({lid_hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(lid)

    lid_lifts = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.080
        and open_aabb[0][1] > closed_aabb[0][1] + 0.030
    )
    ctx.check(
        "lid_opens_upward_from_the_rear_hinge",
        lid_lifts,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    strap_box = ctx.part_element_world_aabb(strap, elem="strap_body")
    catch_box = ctx.part_element_world_aabb(lid, elem="lid_catch")
    strap_bridges_lid = False
    if strap_box is not None and catch_box is not None:
        strap_min, strap_max = strap_box
        catch_min, catch_max = catch_box
        overlap_x = min(strap_max[0], catch_max[0]) - max(strap_min[0], catch_min[0])
        front_band_delta = abs(((strap_min[1] + strap_max[1]) * 0.5) - ((catch_min[1] + catch_max[1]) * 0.5))
        z_clearance = abs(strap_max[2] - catch_min[2])
        strap_bridges_lid = overlap_x >= 0.034 and front_band_delta <= 0.010 and z_clearance <= 0.010
    ctx.check(
        "strap_reaches_the_lid_front",
        strap_bridges_lid,
        details=f"strap_box={strap_box}, catch_box={catch_box}",
    )

    closed_strap_aabb = ctx.part_world_aabb(strap)
    open_strap_aabb = None
    with ctx.pose({strap_hinge: 1.00}):
        open_strap_aabb = ctx.part_world_aabb(strap)
    strap_swings_clear = (
        closed_strap_aabb is not None
        and open_strap_aabb is not None
        and open_strap_aabb[0][1] < closed_strap_aabb[0][1] - 0.010
        and open_strap_aabb[1][2] < closed_strap_aabb[1][2] - 0.008
    )
    ctx.check(
        "strap_swings_forward_to_unlatch",
        strap_swings_clear,
        details=f"closed_aabb={closed_strap_aabb}, open_aabb={open_strap_aabb}",
    )

    knob_rest = ctx.part_world_position(knob)
    knob_turned = None
    with ctx.pose({knob_joint: 1.60}):
        knob_turned = ctx.part_world_position(knob)
    knob_stays_on_axis = (
        knob_rest is not None
        and knob_turned is not None
        and max(abs(knob_rest[i] - knob_turned[i]) for i in range(3)) < 1e-9
    )
    ctx.check(
        "timer_knob_rotates_on_a_fixed_front_axis",
        knob_stays_on_axis,
        details=f"rest={knob_rest}, turned={knob_turned}",
    )

    ctx.check(
        "expected_joint_types_present",
        getattr(lid_hinge, "articulation_type", None) == ArticulationType.REVOLUTE
        and getattr(strap_hinge, "articulation_type", None) == ArticulationType.REVOLUTE
        and getattr(knob_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=(
            f"lid={getattr(lid_hinge, 'articulation_type', None)}, "
            f"strap={getattr(strap_hinge, 'articulation_type', None)}, "
            f"knob={getattr(knob_joint, 'articulation_type', None)}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
