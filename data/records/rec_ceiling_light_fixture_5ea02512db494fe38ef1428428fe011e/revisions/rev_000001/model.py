from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


CAGE_WIDTH = 0.28
FRAME_STOCK = 0.018
RING_HEIGHT = 0.03
OPENING_HEIGHT = 0.32
EAVE_WIDTH = 0.31
ROOF_RISE = 0.084
ROOF_THICKNESS = 0.014
VENT_WIDTH = 0.10
VENT_FRAME_STOCK = 0.016
VENT_HEIGHT = 0.03
CANOPY_SIZE = 0.16
CANOPY_HEIGHT = 0.022
PANEL_CLEARANCE = 0.003
PANEL_THICKNESS = 0.010
PANEL_FRAME = 0.014
PANEL_SIDE_REVEAL = 0.010
HINGE_RADIUS = 0.004

INNER_SPAN = CAGE_WIDTH - 2.0 * FRAME_STOCK
BODY_TOP_Z = 2.0 * RING_HEIGHT + OPENING_HEIGHT
ROOF_BASE_Z = BODY_TOP_Z - 0.010
ROOF_PEAK_Z = BODY_TOP_Z + ROOF_RISE
VENT_TOP_Z = ROOF_PEAK_Z + VENT_HEIGHT

PANEL_WIDTH = INNER_SPAN - 2.0 * PANEL_SIDE_REVEAL
PANEL_HEIGHT = OPENING_HEIGHT - 2.0 * PANEL_CLEARANCE


def _add_square_ring(
    part,
    *,
    z_center: float,
    outer_size: float,
    bar: float,
    height: float,
    material: str,
    name_prefix: str,
) -> None:
    rail_offset = (outer_size - bar) / 2.0
    part.visual(
        Box((outer_size, bar, height)),
        origin=Origin(xyz=(0.0, rail_offset, z_center)),
        material=material,
        name=f"{name_prefix}_front",
    )
    part.visual(
        Box((outer_size, bar, height)),
        origin=Origin(xyz=(0.0, -rail_offset, z_center)),
        material=material,
        name=f"{name_prefix}_rear",
    )
    part.visual(
        Box((bar, outer_size, height)),
        origin=Origin(xyz=(rail_offset, 0.0, z_center)),
        material=material,
        name=f"{name_prefix}_right",
    )
    part.visual(
        Box((bar, outer_size, height)),
        origin=Origin(xyz=(-rail_offset, 0.0, z_center)),
        material=material,
        name=f"{name_prefix}_left",
    )


def _build_panel(
    part,
    *,
    span_axis: str,
    span_direction: float,
    width: float,
    height: float,
    thickness: float,
    frame_bar: float,
    frame_material: str,
    glass_material: str,
) -> None:
    half_height = height / 2.0
    half_width = width / 2.0
    direction = 1.0 if span_direction >= 0.0 else -1.0

    if span_axis == "x":
        part.visual(
            Box((width, thickness, frame_bar)),
            origin=Origin(xyz=(direction * half_width, 0.0, frame_bar / 2.0)),
            material=frame_material,
            name="bottom_rail",
        )
        part.visual(
            Box((width, thickness, frame_bar)),
            origin=Origin(xyz=(direction * half_width, 0.0, height - frame_bar / 2.0)),
            material=frame_material,
            name="top_rail",
        )
        part.visual(
            Box((frame_bar, thickness, height)),
            origin=Origin(xyz=(direction * frame_bar / 2.0, 0.0, half_height)),
            material=frame_material,
            name="hinge_stile",
        )
        part.visual(
            Box((frame_bar, thickness, height)),
            origin=Origin(xyz=(direction * (width - frame_bar / 2.0), 0.0, half_height)),
            material=frame_material,
            name="free_stile",
        )
        part.visual(
            Box((width - 2.0 * frame_bar + 0.002, thickness * 0.38, height - 2.0 * frame_bar + 0.002)),
            origin=Origin(xyz=(direction * half_width, 0.0, half_height)),
            material=glass_material,
            name="glass",
        )
        for idx, z_center in enumerate((0.055, half_height, height - 0.055)):
            part.visual(
                Cylinder(radius=HINGE_RADIUS, length=0.030),
                origin=Origin(xyz=(0.0, -0.002, z_center), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=frame_material,
                name=f"hinge_knuckle_{idx}",
            )
        part.visual(
            Box((0.010, thickness * 0.85, 0.040)),
            origin=Origin(xyz=(direction * (width - 0.007), 0.0, half_height)),
            material=frame_material,
            name="latch_tab",
        )
        return

    part.visual(
        Box((thickness, width, frame_bar)),
        origin=Origin(xyz=(0.0, direction * half_width, frame_bar / 2.0)),
        material=frame_material,
        name="bottom_rail",
    )
    part.visual(
        Box((thickness, width, frame_bar)),
        origin=Origin(xyz=(0.0, direction * half_width, height - frame_bar / 2.0)),
        material=frame_material,
        name="top_rail",
    )
    part.visual(
        Box((thickness, frame_bar, height)),
        origin=Origin(xyz=(0.0, direction * frame_bar / 2.0, half_height)),
        material=frame_material,
        name="hinge_stile",
    )
    part.visual(
        Box((thickness, frame_bar, height)),
        origin=Origin(xyz=(0.0, direction * (width - frame_bar / 2.0), half_height)),
        material=frame_material,
        name="free_stile",
    )
    part.visual(
        Box((thickness * 0.38, width - 2.0 * frame_bar + 0.002, height - 2.0 * frame_bar + 0.002)),
        origin=Origin(xyz=(0.0, direction * half_width, half_height)),
        material=glass_material,
        name="glass",
    )
    for idx, z_center in enumerate((0.055, half_height, height - 0.055)):
        part.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.030),
            origin=Origin(xyz=(0.002, 0.0, z_center), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=frame_material,
            name=f"hinge_knuckle_{idx}",
        )
    part.visual(
        Box((thickness * 0.85, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, direction * (width - 0.007), half_height)),
        material=frame_material,
        name="latch_tab",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="porch_lantern_fixture")

    frame_finish = model.material("frame_finish", rgba=(0.15, 0.16, 0.17, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.82, 0.90, 0.95, 0.35))
    brass_socket = model.material("socket_finish", rgba=(0.55, 0.45, 0.28, 1.0))
    bulb_frost = model.material("bulb_frost", rgba=(0.96, 0.95, 0.90, 0.75))

    body = model.part("body")
    _add_square_ring(
        body,
        z_center=RING_HEIGHT / 2.0,
        outer_size=CAGE_WIDTH,
        bar=FRAME_STOCK,
        height=RING_HEIGHT,
        material=frame_finish,
        name_prefix="base_ring",
    )
    _add_square_ring(
        body,
        z_center=RING_HEIGHT + OPENING_HEIGHT + RING_HEIGHT / 2.0,
        outer_size=CAGE_WIDTH,
        bar=FRAME_STOCK,
        height=RING_HEIGHT,
        material=frame_finish,
        name_prefix="top_ring",
    )
    body.visual(
        Box((INNER_SPAN + 0.004, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z - 0.007)),
        material=frame_finish,
        name="upper_cross_x",
    )
    body.visual(
        Box((0.016, INNER_SPAN + 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z - 0.007)),
        material=frame_finish,
        name="upper_cross_y",
    )

    post_offset = (CAGE_WIDTH - FRAME_STOCK) / 2.0
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Box((FRAME_STOCK, FRAME_STOCK, OPENING_HEIGHT)),
                origin=Origin(
                    xyz=(
                        x_sign * post_offset,
                        y_sign * post_offset,
                        RING_HEIGHT + OPENING_HEIGHT / 2.0,
                    )
                ),
                material=frame_finish,
                name=f"corner_post_{int((x_sign + 1.0) / 2.0)}_{int((y_sign + 1.0) / 2.0)}",
            )

    roof_run = (EAVE_WIDTH - VENT_WIDTH) / 2.0
    roof_length = math.hypot(roof_run, ROOF_RISE)
    roof_angle = math.atan2(ROOF_RISE, roof_run)
    roof_center_y = (EAVE_WIDTH / 2.0 + VENT_WIDTH / 2.0) / 2.0
    roof_center_z = (ROOF_BASE_Z + ROOF_PEAK_Z) / 2.0
    body.visual(
        Box((EAVE_WIDTH, roof_length, ROOF_THICKNESS)),
        origin=Origin(xyz=(0.0, roof_center_y, roof_center_z), rpy=(-roof_angle, 0.0, 0.0)),
        material=frame_finish,
        name="roof_front",
    )
    body.visual(
        Box((EAVE_WIDTH, roof_length, ROOF_THICKNESS)),
        origin=Origin(xyz=(0.0, -roof_center_y, roof_center_z), rpy=(roof_angle, 0.0, 0.0)),
        material=frame_finish,
        name="roof_rear",
    )
    body.visual(
        Box((roof_length, EAVE_WIDTH, ROOF_THICKNESS)),
        origin=Origin(xyz=(roof_center_y, 0.0, roof_center_z), rpy=(0.0, roof_angle, 0.0)),
        material=frame_finish,
        name="roof_right",
    )
    body.visual(
        Box((roof_length, EAVE_WIDTH, ROOF_THICKNESS)),
        origin=Origin(xyz=(-roof_center_y, 0.0, roof_center_z), rpy=(0.0, -roof_angle, 0.0)),
        material=frame_finish,
        name="roof_left",
    )

    _add_square_ring(
        body,
        z_center=ROOF_PEAK_Z + VENT_HEIGHT / 2.0,
        outer_size=VENT_WIDTH,
        bar=VENT_FRAME_STOCK,
        height=VENT_HEIGHT,
        material=frame_finish,
        name_prefix="vent_frame",
    )
    body.visual(
        Box((VENT_WIDTH, 0.052, 0.010)),
        origin=Origin(xyz=(0.0, -0.019, VENT_TOP_Z + 0.005)),
        material=frame_finish,
        name="vent_cap_rear",
    )
    body.visual(
        Box((0.022, 0.044, 0.010)),
        origin=Origin(xyz=(-0.039, 0.023, VENT_TOP_Z + 0.005)),
        material=frame_finish,
        name="vent_cap_left",
    )
    body.visual(
        Box((0.022, 0.044, 0.010)),
        origin=Origin(xyz=(0.039, 0.023, VENT_TOP_Z + 0.005)),
        material=frame_finish,
        name="vent_cap_right",
    )

    body.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, VENT_TOP_Z + 0.010)),
        material=frame_finish,
        name="stem_collar",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, VENT_TOP_Z + 0.054)),
        material=frame_finish,
        name="stem",
    )
    body.visual(
        Box((CANOPY_SIZE, CANOPY_SIZE, CANOPY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, VENT_TOP_Z + 0.103)),
        material=frame_finish,
        name="canopy",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, VENT_TOP_Z + 0.084)),
        material=frame_finish,
        name="canopy_nut",
    )

    body.visual(
        Cylinder(radius=0.006, length=0.544),
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
        material=frame_finish,
        name="center_rod",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=brass_socket,
        name="socket",
    )
    body.visual(
        Sphere(radius=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.228)),
        material=bulb_frost,
        name="bulb",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=frame_finish,
        name="bottom_finial",
    )
    body.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=frame_finish,
        name="finial_tip",
    )

    panel_0 = model.part("side_panel_0")
    _build_panel(
        panel_0,
        span_axis="x",
        span_direction=1.0,
        width=PANEL_WIDTH,
        height=PANEL_HEIGHT,
        thickness=PANEL_THICKNESS,
        frame_bar=PANEL_FRAME,
        frame_material=frame_finish,
        glass_material=glass_tint,
    )

    panel_1 = model.part("side_panel_1")
    _build_panel(
        panel_1,
        span_axis="y",
        span_direction=-1.0,
        width=PANEL_WIDTH,
        height=PANEL_HEIGHT,
        thickness=PANEL_THICKNESS,
        frame_bar=PANEL_FRAME,
        frame_material=frame_finish,
        glass_material=glass_tint,
    )

    panel_2 = model.part("side_panel_2")
    _build_panel(
        panel_2,
        span_axis="x",
        span_direction=-1.0,
        width=PANEL_WIDTH,
        height=PANEL_HEIGHT,
        thickness=PANEL_THICKNESS,
        frame_bar=PANEL_FRAME,
        frame_material=frame_finish,
        glass_material=glass_tint,
    )

    panel_3 = model.part("side_panel_3")
    _build_panel(
        panel_3,
        span_axis="y",
        span_direction=1.0,
        width=PANEL_WIDTH,
        height=PANEL_HEIGHT,
        thickness=PANEL_THICKNESS,
        frame_bar=PANEL_FRAME,
        frame_material=frame_finish,
        glass_material=glass_tint,
    )

    panel_base_z = RING_HEIGHT + PANEL_CLEARANCE
    panel_face_offset = (CAGE_WIDTH / 2.0) - FRAME_STOCK - PANEL_THICKNESS / 2.0 - 0.001
    hinge_offset = INNER_SPAN / 2.0

    model.articulation(
        "body_to_side_panel_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel_0,
        origin=Origin(xyz=(-hinge_offset, panel_face_offset, panel_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=6.0, velocity=1.5),
    )
    model.articulation(
        "body_to_side_panel_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel_1,
        origin=Origin(xyz=(panel_face_offset, hinge_offset, panel_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=6.0, velocity=1.5),
    )
    model.articulation(
        "body_to_side_panel_2",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel_2,
        origin=Origin(xyz=(hinge_offset, -panel_face_offset, panel_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=6.0, velocity=1.5),
    )
    model.articulation(
        "body_to_side_panel_3",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel_3,
        origin=Origin(xyz=(-panel_face_offset, -hinge_offset, panel_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=6.0, velocity=1.5),
    )

    vent_flap = model.part("vent_flap")
    flap_width = 0.058
    flap_depth = 0.038
    vent_flap.visual(
        Box((flap_width, flap_depth, 0.006)),
        origin=Origin(xyz=(0.0, flap_depth / 2.0, 0.003)),
        material=frame_finish,
        name="flap_plate",
    )
    vent_flap.visual(
        Cylinder(radius=0.0035, length=0.022),
        origin=Origin(xyz=(-flap_width / 4.0, 0.012, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="hinge_knuckle_0",
    )
    vent_flap.visual(
        Cylinder(radius=0.0035, length=0.022),
        origin=Origin(xyz=(flap_width / 4.0, 0.012, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="hinge_knuckle_1",
    )

    model.articulation(
        "body_to_vent_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=vent_flap,
        origin=Origin(xyz=(0.0, 0.018, VENT_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=3.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    vent_flap = object_model.get_part("vent_flap")

    panel_joints = (
        (
            object_model.get_part("side_panel_0"),
            object_model.get_articulation("body_to_side_panel_0"),
            "front panel opens outward",
            "y",
            "max",
            0.055,
        ),
        (
            object_model.get_part("side_panel_1"),
            object_model.get_articulation("body_to_side_panel_1"),
            "right panel opens outward",
            "x",
            "max",
            0.055,
        ),
        (
            object_model.get_part("side_panel_2"),
            object_model.get_articulation("body_to_side_panel_2"),
            "rear panel opens outward",
            "y",
            "min",
            0.055,
        ),
        (
            object_model.get_part("side_panel_3"),
            object_model.get_articulation("body_to_side_panel_3"),
            "left panel opens outward",
            "x",
            "min",
            0.055,
        ),
    )

    axis_index = {"x": 0, "y": 1}
    for panel, joint, name, axis, bound, delta in panel_joints:
        closed = ctx.part_world_aabb(panel)
        with ctx.pose({joint: joint.motion_limits.upper}):
            opened = ctx.part_world_aabb(panel)
        idx = axis_index[axis]
        if bound == "max":
            moved_ok = (
                closed is not None
                and opened is not None
                and float(opened[1][idx]) > float(closed[1][idx]) + delta
            )
        else:
            moved_ok = (
                closed is not None
                and opened is not None
                and float(opened[0][idx]) < float(closed[0][idx]) - delta
            )
        ctx.check(name, moved_ok, details=f"closed={closed!r}, opened={opened!r}")

    ctx.expect_gap(
        vent_flap,
        body,
        axis="z",
        positive_elem="flap_plate",
        negative_elem="vent_frame_front",
        max_gap=0.0015,
        max_penetration=1e-5,
        name="vent flap sits flush on the vent frame",
    )
    ctx.expect_overlap(
        vent_flap,
        body,
        axes="xy",
        elem_a="flap_plate",
        elem_b="vent_frame_front",
        min_overlap=0.01,
        name="vent flap spans the vent curb",
    )

    vent_hinge = object_model.get_articulation("body_to_vent_flap")
    closed_flap = ctx.part_world_aabb(vent_flap)
    with ctx.pose({vent_hinge: vent_hinge.motion_limits.upper}):
        opened_flap = ctx.part_world_aabb(vent_flap)
    ctx.check(
        "vent flap lifts upward",
        closed_flap is not None
        and opened_flap is not None
        and float(opened_flap[1][2]) > float(closed_flap[1][2]) + 0.02,
        details=f"closed={closed_flap!r}, opened={opened_flap!r}",
    )

    return ctx.report()


object_model = build_object_model()
