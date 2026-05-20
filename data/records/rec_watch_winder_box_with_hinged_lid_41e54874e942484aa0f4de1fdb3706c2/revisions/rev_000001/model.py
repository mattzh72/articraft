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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    # Overall envelope: a lacquered desktop presentation case large enough for a
    # single automatic-watch winder module.
    body_w = 0.360
    body_d = 0.260
    body_h = 0.120
    wall = 0.016
    panel_w = 0.312
    panel_h = 0.094
    panel_t = 0.010
    panel_y = -0.028
    panel_z = 0.066
    opening_x = 0.055  # deliberately off center: wider fixed panel mass on the left
    opening_z_local = 0.004
    opening_r = 0.049

    walnut = model.material("dark_walnut", rgba=(0.23, 0.10, 0.035, 1.0))
    black_velvet = model.material("black_velvet", rgba=(0.006, 0.005, 0.004, 1.0))
    suede = model.material("warm_suede", rgba=(0.43, 0.28, 0.16, 1.0))
    brass = model.material("brushed_brass", rgba=(0.86, 0.62, 0.25, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.30, 0.42, 0.48, 0.38))
    satin_black = model.material("satin_black", rgba=(0.015, 0.015, 0.014, 1.0))

    body_shell_cq = (
        cq.Workplane("XY")
        .box(body_w, body_d, body_h)
        .translate((0.0, 0.0, body_h / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )
    inner_cut = (
        cq.Workplane("XY")
        .box(body_w - 2.0 * wall, body_d - 2.0 * wall, body_h)
        .translate((0.0, 0.0, body_h / 2.0 + wall))
    )
    body_shell_cq = body_shell_cq.cut(inner_cut)

    panel_cq = cq.Workplane("XZ").box(panel_w, panel_h, panel_t)
    panel_cut = (
        cq.Workplane("XZ")
        .center(opening_x, opening_z_local)
        .circle(opening_r)
        .extrude(panel_t * 3.0, both=True)
    )
    panel_cq = panel_cq.cut(panel_cut).edges("|Y").fillet(0.003)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_shell_cq, "body_shell", tolerance=0.0008),
        material=walnut,
        name="body_shell",
    )
    body.visual(
        Box((body_w - 2.0 * wall - 0.004, body_d - 2.0 * wall - 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, wall + 0.0015)),
        material=black_velvet,
        name="velvet_floor",
    )
    body.visual(
        mesh_from_cadquery(panel_cq, "offset_panel", tolerance=0.0008),
        origin=Origin(xyz=(0.0, panel_y, panel_z)),
        material=black_velvet,
        name="offset_panel",
    )
    # Two wood side keys slightly bite into the offset panel so it reads as a
    # fixed installed bulkhead rather than a floating plate.
    for x, name in ((-0.166, "panel_key_0"), (0.166, "panel_key_1")):
        body.visual(
            Box((0.020, 0.022, panel_h + 0.010)),
            origin=Origin(xyz=(x, panel_y, panel_z)),
            material=walnut,
            name=name,
        )
    body.visual(
        Box((0.128, 0.034, 0.012)),
        origin=Origin(xyz=(-0.082, -0.076, 0.0255)),
        material=suede,
        name="left_accessory_tray",
    )
    # The spindle is a short fixed socket on the front of the offset opening; it
    # carries the rotating cradle without spanning the clear aperture.
    body.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(
            xyz=(opening_x, panel_y - panel_t / 2.0 - 0.0085, panel_z + opening_z_local),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="spindle_socket",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(
            xyz=(opening_x, panel_y - panel_t / 2.0 - 0.0015, panel_z + opening_z_local),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="spindle_flange",
    )
    body.visual(
        Box((2.0 * (opening_r + 0.006), 0.004, 0.004)),
        origin=Origin(xyz=(opening_x, panel_y - panel_t / 2.0 - 0.001, panel_z + opening_z_local)),
        material=brass,
        name="spindle_spoke_h",
    )
    body.visual(
        Box((0.004, 0.004, 2.0 * (opening_r + 0.006))),
        origin=Origin(xyz=(opening_x, panel_y - panel_t / 2.0 - 0.001, panel_z + opening_z_local)),
        material=brass,
        name="spindle_spoke_v",
    )
    for x, y, name in (
        (-0.168, -0.104, "lid_bumper_0"),
        (0.168, -0.104, "lid_bumper_1"),
        (-0.168, 0.104, "lid_bumper_2"),
        (0.168, 0.104, "lid_bumper_3"),
    ):
        body.visual(
            Box((0.016, 0.016, 0.0055)),
            origin=Origin(xyz=(x, y, body_h + 0.00225)),
            material=satin_black,
            name=name,
        )
    for x, name in ((-0.090, "hinge_barrel_0"), (0.090, "hinge_barrel_1")):
        body.visual(
            Cylinder(radius=0.006, length=0.070),
            origin=Origin(
                xyz=(x, body_d / 2.0 + 0.006, body_h + 0.004),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=name,
        )
        body.visual(
            Box((0.076, 0.003, 0.040)),
            origin=Origin(xyz=(x, body_d / 2.0 + 0.0015, body_h - 0.016)),
            material=brass,
            name=f"hinge_leaf_{name[-1]}",
        )

    lid_w = 0.372
    lid_d = 0.272
    lid_t = 0.018
    lid_open_w = 0.284
    lid_open_d = 0.184
    lid_cy = -lid_d / 2.0
    lid_frame_cq = cq.Workplane("XY").box(lid_w, lid_d, lid_t).translate((0.0, lid_cy, lid_t / 2.0))
    lid_cut = (
        cq.Workplane("XY")
        .rect(lid_open_w, lid_open_d)
        .extrude(lid_t * 3.0, both=True)
        .translate((0.0, lid_cy, lid_t / 2.0))
    )
    lid_frame_cq = lid_frame_cq.cut(lid_cut).edges("|Z").fillet(0.008)

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(lid_frame_cq, "lid_frame", tolerance=0.0008),
        material=walnut,
        name="lid_frame",
    )
    lid.visual(
        Box((lid_open_w + 0.014, lid_open_d + 0.014, 0.004)),
        origin=Origin(xyz=(0.0, lid_cy, lid_t * 0.48)),
        material=smoked_glass,
        name="glass_pane",
    )
    lid.visual(
        Box((0.070, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -lid_d + 0.010, lid_t + 0.004)),
        material=brass,
        name="front_latch",
    )
    for x, name in ((-0.090, "lid_hinge_leaf_0"), (0.090, "lid_hinge_leaf_1")):
        lid.visual(
            Box((0.076, 0.032, 0.003)),
            origin=Origin(xyz=(x, -0.016, 0.002)),
            material=brass,
            name=name,
        )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.044, length=0.015),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="cradle_cup",
    )
    cradle.visual(
        Box((0.070, 0.028, 0.042)),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=suede,
        name="watch_pillow",
    )
    cradle.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, -0.038, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hub_cap",
    )
    cradle.visual(
        Box((0.086, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.040, 0.022)),
        material=brass,
        name="watch_band_top",
    )
    cradle.visual(
        Box((0.086, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.040, -0.022)),
        material=brass,
        name="watch_band_bottom",
    )
    cradle.visual(
        Box((0.012, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.008, 0.046)),
        material=brass,
        name="index_mark",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, body_d / 2.0, body_h + 0.005)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(opening_x, panel_y - panel_t / 2.0 - 0.017, panel_z + opening_z_local)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_joint = object_model.get_articulation("body_to_lid")
    cradle_joint = object_model.get_articulation("body_to_cradle")

    ctx.check(
        "lid uses a rear horizontal revolute hinge",
        lid_joint.articulation_type == ArticulationType.REVOLUTE
        and lid_joint.axis == (-1.0, 0.0, 0.0)
        and lid_joint.motion_limits is not None
        and lid_joint.motion_limits.lower == 0.0
        and lid_joint.motion_limits.upper is not None
        and lid_joint.motion_limits.upper > 1.4,
        details=f"type={lid_joint.articulation_type}, axis={lid_joint.axis}, limits={lid_joint.motion_limits}",
    )
    ctx.check(
        "cradle uses a continuous spindle joint",
        cradle_joint.articulation_type == ArticulationType.CONTINUOUS
        and cradle_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={cradle_joint.articulation_type}, axis={cradle_joint.axis}",
    )

    opening_x = 0.055
    opening_r = 0.049
    panel_w = 0.312
    left_land = opening_x - opening_r + panel_w / 2.0
    right_land = panel_w / 2.0 - (opening_x + opening_r)
    ctx.check(
        "panel opening is deliberately offset",
        left_land > 2.5 * right_land,
        details=f"left fixed land={left_land:.3f}m, right fixed land={right_land:.3f}m",
    )

    with ctx.pose({lid_joint: 0.0, cradle_joint: 0.0}):
        ctx.expect_contact(
            body,
            lid,
            elem_a="lid_bumper_0",
            elem_b="lid_frame",
            contact_tol=0.001,
            name="closed lid rests on body bumpers",
        )
        ctx.expect_contact(
            body,
            cradle,
            elem_a="spindle_socket",
            elem_b="cradle_cup",
            contact_tol=0.001,
            name="cradle is carried by the fixed spindle socket",
        )
        resting_lid_aabb = ctx.part_world_aabb(lid)
        resting_mark_aabb = ctx.part_element_world_aabb(cradle, elem="index_mark")

    with ctx.pose({lid_joint: 1.25}):
        opened_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward from the rear hinge",
        resting_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > resting_lid_aabb[1][2] + 0.12,
        details=f"closed={resting_lid_aabb}, opened={opened_lid_aabb}",
    )

    with ctx.pose({cradle_joint: math.pi / 2.0}):
        rotated_mark_aabb = ctx.part_element_world_aabb(cradle, elem="index_mark")

    def _aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3)) if aabb else None

    resting_mark_center = _aabb_center(resting_mark_aabb)
    rotated_mark_center = _aabb_center(rotated_mark_aabb)
    ctx.check(
        "cradle visibly rotates about the spindle axis",
        resting_mark_center is not None
        and rotated_mark_center is not None
        and rotated_mark_center[0] > resting_mark_center[0] + 0.030
        and rotated_mark_center[2] < resting_mark_center[2] - 0.030,
        details=f"rest marker={resting_mark_center}, rotated marker={rotated_mark_center}",
    )

    return ctx.report()


object_model = build_object_model()
