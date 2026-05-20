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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stick_vacuum_folding_wand")

    charcoal = model.material("charcoal_plastic", rgba=(0.055, 0.060, 0.065, 1.0))
    satin_black = model.material("satin_black", rgba=(0.010, 0.011, 0.012, 1.0))
    grey_plastic = model.material("warm_grey_plastic", rgba=(0.46, 0.48, 0.50, 1.0))
    tube_metal = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.77, 1.0))
    accent = model.material("blue_release_accent", rgba=(0.05, 0.32, 0.85, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    brush = model.material("red_brush_strip", rgba=(0.75, 0.06, 0.04, 1.0))

    body = model.part("body")
    # A compact motor canister, battery/grip, handle loop, and the fixed half of
    # the fold hinge.  The root frame is the upright vacuum frame in meters.
    body.visual(
        Cylinder(radius=0.115, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 1.255), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="motor_canister",
    )
    body.visual(
        Cylinder(radius=0.070, length=0.150),
        origin=Origin(xyz=(0.0, 0.000, 1.255), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grey_plastic,
        name="filter_cap",
    )
    body.visual(
        Box((0.105, 0.105, 0.155)),
        origin=Origin(xyz=(0.0, -0.105, 1.325)),
        material=charcoal,
        name="battery_pack",
    )
    body.visual(
        Box((0.070, 0.060, 0.210)),
        origin=Origin(xyz=(0.0, -0.112, 1.185)),
        material=satin_black,
        name="rear_grip",
    )
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, -0.115, 1.225),
                    (0.0, -0.150, 1.405),
                    (0.0, -0.055, 1.520),
                    (0.0, 0.075, 1.430),
                    (0.0, 0.070, 1.285),
                ],
                radius=0.018,
                samples_per_segment=16,
                radial_segments=18,
            ),
            "stick_vacuum_handle_loop",
        ),
        material=satin_black,
        name="handle_loop",
    )
    body.visual(
        Box((0.145, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, 0.000, 1.155)),
        material=charcoal,
        name="fold_support_bridge",
    )
    body.visual(
        Box((0.035, 0.050, 0.140)),
        origin=Origin(xyz=(-0.062, 0.000, 1.095)),
        material=charcoal,
        name="fold_support_cheek_0",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.040),
        origin=Origin(xyz=(-0.062, 0.000, 1.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_plastic,
        name="fixed_hinge_barrel_0",
    )
    body.visual(
        Box((0.035, 0.050, 0.140)),
        origin=Origin(xyz=(0.062, 0.000, 1.095)),
        material=charcoal,
        name="fold_support_cheek_1",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.040),
        origin=Origin(xyz=(0.062, 0.000, 1.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_plastic,
        name="fixed_hinge_barrel_1",
    )
    body.visual(
        Box((0.052, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.152, 1.240)),
        material=accent,
        name="release_button",
    )

    wand = model.part("wand")
    # Child frame is on the fold pin.  The actual straight wand centerline is
    # deliberately shifted to the rear side of the fixed support by 65 mm.
    wand.visual(
        Cylinder(radius=0.029, length=0.084),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_plastic,
        name="fold_barrel",
    )
    wand.visual(
        Box((0.042, 0.080, 0.050)),
        origin=Origin(xyz=(0.0, -0.040, -0.042)),
        material=grey_plastic,
        name="offset_link",
    )
    wand.visual(
        Cylinder(radius=0.018, length=0.740),
        origin=Origin(xyz=(0.0, -0.065, -0.437)),
        material=tube_metal,
        name="wand_tube",
    )
    wand.visual(
        Cylinder(radius=0.027, length=0.105),
        origin=Origin(xyz=(0.0, -0.065, -0.070)),
        material=grey_plastic,
        name="upper_collar",
    )
    wand.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.0, -0.065, -0.819)),
        material=grey_plastic,
        name="lower_socket",
    )
    wand.visual(
        Box((0.160, 0.045, 0.025)),
        origin=Origin(xyz=(0.0, -0.065, -0.827)),
        material=grey_plastic,
        name="head_fork_bridge",
    )
    wand.visual(
        Box((0.030, 0.055, 0.080)),
        origin=Origin(xyz=(-0.062, -0.065, -0.875)),
        material=grey_plastic,
        name="head_fork_cheek_0",
    )
    wand.visual(
        Box((0.030, 0.055, 0.080)),
        origin=Origin(xyz=(0.062, -0.065, -0.875)),
        material=grey_plastic,
        name="head_fork_cheek_1",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.026, length=0.094),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey_plastic,
        name="head_hinge_barrel",
    )
    floor_head.visual(
        Box((0.085, 0.095, 0.050)),
        origin=Origin(xyz=(0.0, 0.045, -0.045)),
        material=grey_plastic,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.355, 0.225, 0.060)),
        origin=Origin(xyz=(0.0, 0.130, -0.105)),
        material=charcoal,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.320, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, 0.022, -0.072)),
        material=accent,
        name="front_light_strip",
    )
    floor_head.visual(
        Cylinder(radius=0.026, length=0.330),
        origin=Origin(xyz=(0.0, 0.228, -0.110), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brush,
        name="brush_roller",
    )
    for index, x in enumerate((-0.140, 0.140)):
        floor_head.visual(
            Cylinder(radius=0.028, length=0.018),
            origin=Origin(xyz=(x, 0.020, -0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"rear_wheel_{index}",
        )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.0, 0.0, 1.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.25),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, -0.065, -0.875)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("fold_joint")
    head_pitch = object_model.get_articulation("head_pitch")

    def _coord(vec, index: int) -> float:
        try:
            return float(vec[index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[index])

    def _aabb_center_axis(aabb, index: int) -> float | None:
        if aabb is None:
            return None
        return (_coord(aabb[0], index) + _coord(aabb[1], index)) * 0.5

    ctx.check(
        "two requested revolute joints",
        len(object_model.articulations) == 2
        and fold.articulation_type == ArticulationType.REVOLUTE
        and head_pitch.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "fold and head axes are horizontal",
        abs(fold.axis[2]) < 1e-6
        and abs(head_pitch.axis[2]) < 1e-6
        and (fold.axis[0] ** 2 + fold.axis[1] ** 2) > 0.99
        and (head_pitch.axis[0] ** 2 + head_pitch.axis[1] ** 2) > 0.99,
        details=f"fold_axis={fold.axis}, head_axis={head_pitch.axis}",
    )

    support_aabb = ctx.part_element_world_aabb(body, elem="fold_support_bridge")
    wand_aabb = ctx.part_element_world_aabb(wand, elem="wand_tube")
    support_y = _aabb_center_axis(support_aabb, 1)
    wand_y = _aabb_center_axis(wand_aabb, 1)
    ctx.check(
        "wand chain is side-offset from fixed support",
        support_y is not None and wand_y is not None and wand_y < support_y - 0.045,
        details=f"support_y={support_y}, wand_y={wand_y}",
    )

    ctx.expect_contact(
        body,
        wand,
        elem_a="fixed_hinge_barrel_0",
        elem_b="fold_barrel",
        contact_tol=0.002,
        name="fold hinge barrels meet at one side",
    )
    ctx.expect_contact(
        wand,
        floor_head,
        elem_a="head_fork_cheek_0",
        elem_b="head_hinge_barrel",
        contact_tol=0.002,
        name="floor head hinge is captured in fork",
    )

    rest_head_pos = ctx.part_world_position(floor_head)
    with ctx.pose({fold: 1.25, head_pitch: 0.0}):
        folded_head_pos = ctx.part_world_position(floor_head)
    ctx.check(
        "fold joint raises the wand and head",
        rest_head_pos is not None
        and folded_head_pos is not None
        and _coord(folded_head_pos, 2) > _coord(rest_head_pos, 2) + 0.45,
        details=f"rest={rest_head_pos}, folded={folded_head_pos}",
    )

    with ctx.pose({head_pitch: -0.55}):
        pitched_down = ctx.part_element_world_aabb(floor_head, elem="head_shell")
        down_z = _aabb_center_axis(pitched_down, 2)
    with ctx.pose({head_pitch: 0.55}):
        pitched_up = ctx.part_element_world_aabb(floor_head, elem="head_shell")
        up_z = _aabb_center_axis(pitched_up, 2)
    ctx.check(
        "floor head pitch visibly changes head angle",
        down_z is not None and up_z is not None and up_z > down_z + 0.08,
        details=f"down_center_z={down_z}, up_center_z={up_z}",
    )

    return ctx.report()


object_model = build_object_model()
