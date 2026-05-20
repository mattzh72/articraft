from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.62
BODY_D = 0.38
BODY_BASE_Z = 0.018
BODY_H = 0.317
BODY_TOP_Z = BODY_BASE_Z + BODY_H
BODY_WALL = 0.008
BODY_FLOOR = 0.012

TOWER_W = 0.34
TOWER_D = 0.09
TOWER_Y = 0.165
TOWER_H = 0.512
TOWER_TOP_Z = BODY_BASE_Z + TOWER_H

LID_W = 0.586
LID_D = 0.292
LID_H = 0.092
LID_Y = -0.044
LID_BASE_Z = BODY_TOP_Z + 0.004
LID_TOP_Z = LID_BASE_Z + LID_H

ORGANIZER_W = 0.246
ORGANIZER_D = 0.164
ORGANIZER_Y = -0.012
ORGANIZER_REAR_Y = ORGANIZER_Y + ORGANIZER_D / 2.0
ORGANIZER_XS = (-0.147, 0.147)

HANDLE_RAIL_X = 0.12
HANDLE_GUIDE_Y = 0.155
HANDLE_GUIDE_TOP_Z = 0.500
HANDLE_TRAVEL = 0.360

WHEEL_RADIUS = 0.09
WHEEL_WIDTH = 0.045
WHEEL_Y = 0.125
WHEEL_Z = 0.092
WHEEL_XS = (-0.3325, 0.3325)


def _add_cq_visual(part, shape, mesh_name: str, material: str, *, name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=name,
    )


def _body_shape() -> cq.Workplane:
    main_shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
        .translate((0.0, 0.0, BODY_BASE_Z))
    )

    rear_tower = (
        cq.Workplane("XY")
        .box(TOWER_W, TOWER_D, TOWER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.020)
        .translate((0.0, TOWER_Y, BODY_BASE_Z))
    )

    wheel_pod_left = (
        cq.Workplane("XY")
        .box(0.040, 0.090, 0.155, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
        .translate((-0.280, 0.118, BODY_BASE_Z))
    )
    wheel_pod_right = (
        cq.Workplane("XY")
        .box(0.040, 0.090, 0.155, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
        .translate((0.280, 0.118, BODY_BASE_Z))
    )

    shell = main_shell.union(rear_tower).union(wheel_pod_left).union(wheel_pod_right)

    lower_compartment = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * BODY_WALL,
            0.272,
            BODY_H + 0.030,
            centered=(True, True, False),
        )
        .translate((0.0, -0.048, BODY_BASE_Z + BODY_FLOOR))
    )
    shell = shell.cut(lower_compartment)

    grip_pocket = (
        cq.Workplane("XY")
        .box(0.300, 0.095, 0.078, centered=(True, True, False))
        .translate((0.0, 0.162, 0.460))
    )
    shell = shell.cut(grip_pocket)

    for x_pos in (-HANDLE_RAIL_X, HANDLE_RAIL_X):
        guide_slot = (
            cq.Workplane("XY")
            .box(0.032, 0.078, HANDLE_GUIDE_TOP_Z + 0.020, centered=(True, True, False))
            .translate((x_pos, HANDLE_GUIDE_Y, 0.010))
        )
        shell = shell.cut(guide_slot)

    front_latch_clearance = (
        cq.Workplane("XY")
        .box(0.095, 0.022, 0.028, centered=(True, True, False))
        .translate((0.0, -0.182, BODY_TOP_Z - 0.010))
    )
    shell = shell.cut(front_latch_clearance)

    return shell.combine()


def _lid_shape() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, LID_Y, LID_BASE_Z))
    )

    nose = (
        cq.Workplane("XY")
        .box(0.110, 0.034, 0.040, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, -0.191, LID_BASE_Z + 0.024))
    )
    lid = lid.union(nose)

    for x_pos in ORGANIZER_XS:
        organizer_pocket = (
            cq.Workplane("XY")
            .box(ORGANIZER_W, ORGANIZER_D, 0.050, centered=(True, True, False))
            .translate((x_pos, ORGANIZER_Y, LID_TOP_Z - 0.050))
        )
        lid = lid.cut(organizer_pocket)

    underside_relief = (
        cq.Workplane("XY")
        .box(LID_W - 0.024, LID_D - 0.028, 0.020, centered=(True, True, False))
        .translate((0.0, LID_Y, LID_BASE_Z))
    )
    lid = lid.cut(underside_relief)

    button_slot = (
        cq.Workplane("XY")
        .box(0.082, 0.072, 0.036, centered=(True, True, False))
        .translate((0.0, -0.194, LID_BASE_Z + 0.020))
    )
    lid = lid.cut(button_slot)

    return lid


def _cover_shape(width: float, depth: float, thickness: float) -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(width, depth, thickness, centered=(True, True, False))
        .translate((0.0, -depth / 2.0, 0.0))
    )

    rim = (
        cq.Workplane("XY")
        .box(width - 0.022, depth - 0.022, thickness * 0.60, centered=(True, True, False))
        .translate((0.0, -depth / 2.0, thickness))
    )
    panel = panel.union(rim)

    pull_tab = (
        cq.Workplane("XY")
        .box(width * 0.34, 0.012, thickness * 1.30, centered=(True, True, False))
        .translate((0.0, -depth + 0.004, 0.0))
    )
    panel = panel.union(pull_tab)

    for x_pos in (-width * 0.28, 0.0, width * 0.28):
        barrel = (
            cq.Workplane("YZ")
            .cylinder(width * 0.16, 0.007)
            .translate((x_pos, -0.003, 0.006))
        )
        panel = panel.union(barrel)

    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")

    model.material("body_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("lid_black", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("trim_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("handle_metal", rgba=(0.62, 0.66, 0.70, 1.0))
    model.material("wheel_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("hub_gray", rgba=(0.47, 0.50, 0.54, 1.0))
    model.material("cover_clear", rgba=(0.68, 0.83, 0.92, 0.36))
    model.material("organizer_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    model.material("latch_orange", rgba=(0.95, 0.56, 0.10, 1.0))

    body = model.part("body")
    _add_cq_visual(body, _body_shape(), "toolbox_body", "body_charcoal", name="shell")
    body.visual(
        Box((0.074, 0.044, BODY_BASE_Z)),
        origin=Origin(xyz=(-0.182, -0.150, BODY_BASE_Z / 2.0)),
        material="trim_dark",
        name="foot_0",
    )
    body.visual(
        Box((0.074, 0.044, BODY_BASE_Z)),
        origin=Origin(xyz=(0.182, -0.150, BODY_BASE_Z / 2.0)),
        material="trim_dark",
        name="foot_1",
    )
    body.visual(
        Box((0.084, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.182, BODY_TOP_Z - 0.006)),
        material="hub_gray",
        name="striker",
    )

    lid = model.part("lid")
    _add_cq_visual(lid, _lid_shape(), "toolbox_lid", "lid_black", name="shell")
    for x_pos in ORGANIZER_XS:
        lid.visual(
            Box((ORGANIZER_W - 0.014, ORGANIZER_D - 0.014, 0.004)),
            origin=Origin(xyz=(x_pos, ORGANIZER_Y, LID_BASE_Z + 0.024)),
            material="organizer_dark",
            name=f"tray_floor_{0 if x_pos < 0 else 1}",
        )
        lid.visual(
            Box((0.008, ORGANIZER_D - 0.014, 0.026)),
            origin=Origin(xyz=(x_pos, ORGANIZER_Y, LID_BASE_Z + 0.035)),
            material="organizer_dark",
            name=f"tray_divider_{0 if x_pos < 0 else 1}",
        )
        lid.visual(
            Box((ORGANIZER_W, 0.010, 0.010)),
            origin=Origin(xyz=(x_pos, ORGANIZER_REAR_Y + 0.004, LID_TOP_Z - 0.004)),
            material="trim_dark",
            name=f"cover_hinge_base_{0 if x_pos < 0 else 1}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.FIXED,
        parent=body,
        child=lid,
        origin=Origin(),
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.032, 0.010, 0.470)),
        origin=Origin(xyz=(-HANDLE_RAIL_X, 0.0, -0.215)),
        material="handle_metal",
        name="rail_0",
    )
    handle.visual(
        Box((0.032, 0.010, 0.470)),
        origin=Origin(xyz=(HANDLE_RAIL_X, 0.0, -0.215)),
        material="handle_metal",
        name="rail_1",
    )
    handle.visual(
        Box((0.234, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material="handle_metal",
        name="bridge",
    )
    handle.visual(
        Cylinder(radius=0.019, length=0.292),
        origin=Origin(xyz=(0.0, 0.0, 0.019), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="trim_dark",
        name="grip",
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_GUIDE_Y, HANDLE_GUIDE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.45,
            lower=0.0,
            upper=HANDLE_TRAVEL,
        ),
    )

    for index, x_pos in enumerate(WHEEL_XS):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material="wheel_rubber",
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.054, length=WHEEL_WIDTH - 0.010),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material="hub_gray",
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.028, length=0.008),
            origin=Origin(
                xyz=((WHEEL_WIDTH - 0.008) / 2.0 - 0.001, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="trim_dark",
            name="cap",
        )
        model.articulation(
            f"body_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x_pos, WHEEL_Y, WHEEL_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=12.0),
        )

    for index, x_pos in enumerate(ORGANIZER_XS):
        cover = model.part(f"cover_{index}")
        _add_cq_visual(
            cover,
            _cover_shape(ORGANIZER_W + 0.006, ORGANIZER_D + 0.008, 0.008),
            f"organizer_cover_{index}",
            "cover_clear",
            name="cover",
        )
        model.articulation(
            f"lid_to_cover_{index}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=cover,
            origin=Origin(xyz=(x_pos, ORGANIZER_REAR_Y, LID_TOP_Z + 0.0017)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=2.6,
                lower=0.0,
                upper=1.95,
            ),
        )

    latch = model.part("latch")
    latch.visual(
        Box((0.076, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material="latch_orange",
        name="cap",
    )
    latch.visual(
        Box((0.046, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material="latch_orange",
        name="stem",
    )
    latch.visual(
        Box((0.020, 0.010, 0.012)),
        origin=Origin(xyz=(-0.031, 0.005, 0.0)),
        material="latch_orange",
        name="guide_0",
    )
    latch.visual(
        Box((0.020, 0.010, 0.012)),
        origin=Origin(xyz=(0.031, 0.005, 0.0)),
        material="latch_orange",
        name="guide_1",
    )
    model.articulation(
        "lid_to_latch",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.0, -0.209, LID_BASE_Z + 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.012,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    cover_0 = object_model.get_part("cover_0")
    cover_1 = object_model.get_part("cover_1")
    latch = object_model.get_part("latch")

    ctx.allow_overlap(
        latch,
        lid,
        elem_a="guide_0",
        elem_b="shell",
        reason="The left hidden latch guide fin is intentionally simplified as riding inside the molded latch channel of the lid nose.",
    )
    ctx.allow_overlap(
        latch,
        lid,
        elem_a="guide_1",
        elem_b="shell",
        reason="The right hidden latch guide fin is intentionally simplified as riding inside the molded latch channel of the lid nose.",
    )
    ctx.allow_overlap(
        body,
        handle,
        elem_a="shell",
        elem_b="rail_0",
        reason="The left telescoping rail is intentionally represented as sliding inside the molded rear guide sleeve proxy of the body shell.",
    )
    ctx.allow_overlap(
        body,
        handle,
        elem_a="shell",
        elem_b="rail_1",
        reason="The right telescoping rail is intentionally represented as sliding inside the molded rear guide sleeve proxy of the body shell.",
    )

    handle_slide = object_model.get_articulation("body_to_handle")
    cover_joint_0 = object_model.get_articulation("lid_to_cover_0")
    cover_joint_1 = object_model.get_articulation("lid_to_cover_1")
    latch_slide = object_model.get_articulation("lid_to_latch")
    wheel_joint_0 = object_model.get_articulation("body_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("body_to_wheel_1")

    handle_limits = handle_slide.motion_limits
    cover_limits_0 = cover_joint_0.motion_limits
    cover_limits_1 = cover_joint_1.motion_limits
    latch_limits = latch_slide.motion_limits

    with ctx.pose({"body_to_handle": 0.0, "lid_to_cover_0": 0.0, "lid_to_cover_1": 0.0, "lid_to_latch": 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="shell",
            negative_elem="striker",
            max_penetration=0.0,
            max_gap=0.030,
            name="lid nose clears the front latch striker",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.24,
            name="lid covers the body opening footprint",
        )
        ctx.expect_gap(
            cover_0,
            lid,
            axis="z",
            negative_elem="shell",
            max_gap=0.006,
            max_penetration=0.0,
            name="first organizer cover rests on the lid surface",
        )
        ctx.expect_gap(
            cover_1,
            lid,
            axis="z",
            negative_elem="shell",
            max_gap=0.006,
            max_penetration=0.0,
            name="second organizer cover rests on the lid surface",
        )
        ctx.expect_overlap(
            cover_0,
            lid,
            axes="xy",
            min_overlap=0.12,
            name="first organizer cover stays over its pocket",
        )
        ctx.expect_overlap(
            cover_1,
            lid,
            axes="xy",
            min_overlap=0.12,
            name="second organizer cover stays over its pocket",
        )

    handle_rest_aabb = ctx.part_world_aabb(handle)
    if handle_limits is not None and handle_limits.upper is not None:
        with ctx.pose({"body_to_handle": handle_limits.upper}):
            handle_open_aabb = ctx.part_world_aabb(handle)
            ctx.expect_overlap(
                handle,
                body,
                axes="x",
                min_overlap=0.18,
                name="handle stays aligned with the rear guide band",
            )
        ctx.check(
            "handle extends upward from the recessed position",
            handle_rest_aabb is not None
            and handle_open_aabb is not None
            and handle_open_aabb[1][2] > handle_rest_aabb[1][2] + 0.28,
            details=f"rest={handle_rest_aabb}, extended={handle_open_aabb}",
        )

    cover_rest_aabb = ctx.part_world_aabb(cover_0)
    if cover_limits_0 is not None and cover_limits_0.upper is not None:
        with ctx.pose({"lid_to_cover_0": cover_limits_0.upper * 0.75}):
            cover_open_aabb = ctx.part_world_aabb(cover_0)
        ctx.check(
            "first organizer cover lifts upward on its rear hinge",
            cover_rest_aabb is not None
            and cover_open_aabb is not None
            and cover_open_aabb[1][2] > cover_rest_aabb[1][2] + 0.08,
            details=f"closed={cover_rest_aabb}, open={cover_open_aabb}",
        )

    cover_rest_aabb_1 = ctx.part_world_aabb(cover_1)
    if cover_limits_1 is not None and cover_limits_1.upper is not None:
        with ctx.pose({"lid_to_cover_1": cover_limits_1.upper * 0.75}):
            cover_open_aabb_1 = ctx.part_world_aabb(cover_1)
        ctx.check(
            "second organizer cover lifts upward on its rear hinge",
            cover_rest_aabb_1 is not None
            and cover_open_aabb_1 is not None
            and cover_open_aabb_1[1][2] > cover_rest_aabb_1[1][2] + 0.08,
            details=f"closed={cover_rest_aabb_1}, open={cover_open_aabb_1}",
        )

    latch_rest = ctx.part_world_position(latch)
    if latch_limits is not None and latch_limits.upper is not None:
        with ctx.pose({"lid_to_latch": latch_limits.upper}):
            latch_pressed = ctx.part_world_position(latch)
        ctx.check(
            "latch button presses rearward into the lid nose",
            latch_rest is not None
            and latch_pressed is not None
            and latch_pressed[1] > latch_rest[1] + 0.009,
            details=f"rest={latch_rest}, pressed={latch_pressed}",
        )

    for joint, name in ((wheel_joint_0, "wheel_0"), (wheel_joint_1, "wheel_1")):
        limits = joint.motion_limits
        articulation_name = getattr(joint.articulation_type, "name", str(joint.articulation_type))
        ctx.check(
            f"{name} uses a continuous rolling joint",
            articulation_name == "CONTINUOUS"
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={articulation_name}, limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
