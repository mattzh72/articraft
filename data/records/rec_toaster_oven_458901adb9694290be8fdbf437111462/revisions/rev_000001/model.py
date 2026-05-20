from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


DEPTH = 0.34
WIDTH = 0.46
HEIGHT = 0.26
FRONT_X = DEPTH / 2.0

OPENING_Y = -0.055
OPENING_W = 0.300
OPENING_Z_MIN = 0.066
OPENING_Z_MAX = 0.232
OPENING_H = OPENING_Z_MAX - OPENING_Z_MIN

TRAY_SLOT_Z_MIN = 0.026
TRAY_SLOT_Z_MAX = 0.052
TRAY_SLOT_H = TRAY_SLOT_Z_MAX - TRAY_SLOT_Z_MIN
TRAY_Y = OPENING_Y

DOOR_HINGE_X = FRONT_X + 0.006
DOOR_HINGE_Z = OPENING_Z_MIN
DOOR_W = OPENING_W + 0.024
DOOR_H = OPENING_H + 0.014


def _oven_body_mesh() -> cq.Workplane:
    """One continuous rounded cabinet with real front cavity and tray slot."""
    body = (
        cq.Workplane("XY")
        .box(DEPTH, WIDTH, HEIGHT)
        .translate((0.0, 0.0, HEIGHT / 2.0))
        .edges("|X")
        .fillet(0.012)
        .edges("|Y")
        .fillet(0.006)
    )

    cavity_cut = (
        cq.Workplane("XY")
        .box(0.318, OPENING_W, OPENING_H)
        .translate((FRONT_X - 0.318 / 2.0 + 0.002, OPENING_Y, (OPENING_Z_MIN + OPENING_Z_MAX) / 2.0))
    )
    tray_slot_cut = (
        cq.Workplane("XY")
        .box(0.318, OPENING_W, TRAY_SLOT_H)
        .translate((FRONT_X - 0.318 / 2.0 + 0.002, TRAY_Y, (TRAY_SLOT_Z_MIN + TRAY_SLOT_Z_MAX) / 2.0))
    )

    return body.cut(cavity_cut).cut(tray_slot_cut)


def _door_frame_mesh() -> cq.Workplane:
    """Continuous front door frame with a clear glass opening."""
    frame_t = 0.018
    frame = (
        cq.Workplane("XY")
        .box(frame_t, DOOR_W, DOOR_H)
        .translate((0.006, 0.0, DOOR_H / 2.0))
        .edges("|X")
        .fillet(0.003)
    )
    glass_cut = (
        cq.Workplane("XY")
        .box(frame_t + 0.012, DOOR_W - 0.070, DOOR_H - 0.068)
        .translate((0.006, 0.0, DOOR_H / 2.0 + 0.010))
    )
    return frame.cut(glass_cut)


def _crumb_tray_mesh() -> cq.Workplane:
    """Shallow stamped tray sized to slide inside the lower slot."""
    tray_len = 0.270
    tray_w = 0.284
    sheet_t = 0.004
    side_h = 0.016
    front_h = 0.024
    x_center = -0.015

    base = cq.Workplane("XY").box(tray_len, tray_w, sheet_t).translate((x_center, 0.0, 0.0))
    left_lip = (
        cq.Workplane("XY")
        .box(tray_len, 0.006, side_h)
        .translate((x_center, -tray_w / 2.0 + 0.003, side_h / 2.0 - sheet_t / 2.0))
    )
    right_lip = (
        cq.Workplane("XY")
        .box(tray_len, 0.006, side_h)
        .translate((x_center, tray_w / 2.0 - 0.003, side_h / 2.0 - sheet_t / 2.0))
    )
    front_lip = (
        cq.Workplane("XY")
        .box(0.018, tray_w, front_h)
        .translate((x_center + tray_len / 2.0 - 0.009, 0.0, front_h / 2.0 - sheet_t / 2.0))
    )
    rear_lip = (
        cq.Workplane("XY")
        .box(0.010, tray_w, 0.010)
        .translate((x_center - tray_len / 2.0 + 0.005, 0.0, 0.005 - sheet_t / 2.0))
    )
    return base.union(left_lip).union(right_lip).union(front_lip).union(rear_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_toaster_oven")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.68, 0.62, 1.0))
    dark = model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    black_glass = model.material("smoked_glass", rgba=(0.06, 0.09, 0.10, 0.38))
    warm_rod = model.material("warm_heater_tube", rgba=(1.0, 0.30, 0.08, 1.0))
    tray_metal = model.material("dull_aluminum", rgba=(0.78, 0.77, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_oven_body_mesh(), "rounded_housing", tolerance=0.0008),
        material=stainless,
        name="rounded_housing",
    )
    body.visual(
        Box((0.005, 0.104, 0.188)),
        origin=Origin(xyz=(FRONT_X + 0.001, 0.167, 0.139)),
        material=dark,
        name="control_face",
    )
    body.visual(
        Box((0.004, 0.282, 0.136)),
        origin=Origin(xyz=(-0.146, OPENING_Y, 0.148)),
        material=dark,
        name="cavity_back",
    )
    body.visual(
        Box((0.230, 0.006, 0.006)),
        origin=Origin(xyz=(-0.006, OPENING_Y - OPENING_W / 2.0 - 0.001, 0.024)),
        material=stainless,
        name="lower_guide_0",
    )
    body.visual(
        Box((0.230, 0.006, 0.006)),
        origin=Origin(xyz=(-0.006, OPENING_Y + OPENING_W / 2.0 + 0.001, 0.024)),
        material=stainless,
        name="lower_guide_1",
    )
    for idx, z in enumerate((0.101, 0.196)):
        body.visual(
            Cylinder(radius=0.0045, length=0.306),
            origin=Origin(xyz=(0.000, OPENING_Y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm_rod,
            name=f"heater_rod_{idx}",
        )
        body.visual(
            Box((0.014, 0.010, 0.014)),
            origin=Origin(xyz=(0.000, OPENING_Y - OPENING_W / 2.0 - 0.004, z)),
            material=dark,
            name=f"heater_socket_{idx}_0",
        )
        body.visual(
            Box((0.014, 0.010, 0.014)),
            origin=Origin(xyz=(0.000, OPENING_Y + OPENING_W / 2.0 + 0.004, z)),
            material=dark,
            name=f"heater_socket_{idx}_1",
        )

    for collar_name, tick_name, z in (
        ("knob_collar_0", "knob_tick_0", 0.169),
        ("knob_collar_1", "knob_tick_1", 0.105),
    ):
        body.visual(
            Cylinder(radius=0.028, length=0.006),
            origin=Origin(xyz=(FRONT_X + 0.003, 0.167, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=collar_name,
        )
        body.visual(
            Box((0.002, 0.054, 0.003)),
            origin=Origin(xyz=(FRONT_X + 0.0045, 0.167, z + 0.042)),
            material=dark,
            name=tick_name,
        )

    for idx, y in enumerate((-0.175, 0.065, 0.188)):
        body.visual(
            Box((0.050, 0.040, 0.010)),
            origin=Origin(xyz=(-0.050, y, 0.005)),
            material=dark,
            name=f"rubber_foot_{idx}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_frame_mesh(), "door_frame", tolerance=0.0006),
        material=stainless,
        name="door_frame",
    )
    door.visual(
        Box((0.004, DOOR_W - 0.060, DOOR_H - 0.058)),
        origin=Origin(xyz=(-0.002, 0.0, DOOR_H / 2.0 + 0.008)),
        material=black_glass,
        name="glass_pane",
    )
    door.visual(
        Box((0.014, DOOR_W, 0.012)),
        origin=Origin(xyz=(0.006, 0.0, 0.006)),
        material=stainless,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.007, length=DOOR_W),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.240),
        origin=Origin(xyz=(0.042, 0.0, DOOR_H - 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="door_handle",
    )
    for idx, y in enumerate((-0.100, 0.100)):
        door.visual(
            Cylinder(radius=0.006, length=0.036),
            origin=Origin(xyz=(0.024, y, DOOR_H - 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"handle_standoff_{idx}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, OPENING_Y, DOOR_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.048,
            0.026,
            body_style="skirted",
            top_diameter=0.038,
            skirt=KnobSkirt(0.055, 0.006, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=22, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0009, angle_deg=0.0),
            center=False,
        ),
        "shared_knob_cap",
    )
    for name, z in (("upper_knob", 0.169), ("lower_knob", 0.105)):
        knob = model.part(name)
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=0.005, length=0.010),
            origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="shaft_stub",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(FRONT_X + 0.007, 0.167, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=8.0),
        )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        mesh_from_cadquery(_crumb_tray_mesh(), "crumb_tray_pan", tolerance=0.0006),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tray_metal,
        name="tray_pan",
    )
    crumb_tray.visual(
        Box((0.008, 0.150, 0.010)),
        origin=Origin(xyz=(0.143, 0.0, 0.010)),
        material=dark,
        name="tray_pull",
    )
    crumb_tray.visual(
        Box((0.026, 0.070, 0.006)),
        origin=Origin(xyz=(0.130, 0.0, 0.010)),
        material=dark,
        name="pull_bridge",
    )
    model.articulation(
        "body_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=crumb_tray,
        origin=Origin(xyz=(0.021, TRAY_Y, 0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.25, lower=0.0, upper=0.090),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    upper_knob = object_model.get_part("upper_knob")
    lower_knob = object_model.get_part("lower_knob")
    crumb_tray = object_model.get_part("crumb_tray")
    door_joint = object_model.get_articulation("body_to_door")
    tray_joint = object_model.get_articulation("body_to_crumb_tray")

    ctx.allow_overlap(
        body,
        crumb_tray,
        elem_a="rounded_housing",
        elem_b="tray_pan",
        reason="The shallow crumb tray pan is intentionally nested inside the lower sheet-metal slot and guide pocket.",
    )
    ctx.allow_overlap(
        body,
        upper_knob,
        elem_a="knob_collar_0",
        elem_b="shaft_stub",
        reason="The rotary knob shaft stub is intentionally captured inside the fixed front collar.",
    )
    ctx.allow_overlap(
        body,
        upper_knob,
        elem_a="control_face",
        elem_b="shaft_stub",
        reason="The knob shaft intentionally passes through the fixed control face into the appliance front.",
    )
    ctx.expect_within(
        upper_knob,
        body,
        axes="yz",
        inner_elem="shaft_stub",
        outer_elem="control_face",
        name="upper knob shaft passes through the control face",
    )
    ctx.expect_contact(
        body,
        upper_knob,
        elem_a="knob_collar_0",
        elem_b="knob_cap",
        contact_tol=0.004,
        name="upper knob cap seats on collar",
    )
    ctx.allow_overlap(
        body,
        lower_knob,
        elem_a="knob_collar_1",
        elem_b="shaft_stub",
        reason="The rotary knob shaft stub is intentionally captured inside the fixed front collar.",
    )
    ctx.allow_overlap(
        body,
        lower_knob,
        elem_a="control_face",
        elem_b="shaft_stub",
        reason="The knob shaft intentionally passes through the fixed control face into the appliance front.",
    )
    ctx.expect_within(
        lower_knob,
        body,
        axes="yz",
        inner_elem="shaft_stub",
        outer_elem="control_face",
        name="lower knob shaft passes through the control face",
    )
    ctx.expect_contact(
        body,
        lower_knob,
        elem_a="knob_collar_1",
        elem_b="knob_cap",
        contact_tol=0.004,
        name="lower knob cap seats on collar",
    )

    with ctx.pose({door_joint: 0.0, tray_joint: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            min_overlap=0.12,
            elem_a="door_frame",
            elem_b="rounded_housing",
            name="closed door covers the oven opening",
        )
        ctx.expect_gap(
            door,
            body,
            axis="x",
            min_gap=-0.020,
            max_gap=0.030,
            positive_elem="door_frame",
            negative_elem="rounded_housing",
            name="closed door is seated at the front face",
        )
        ctx.expect_within(
            crumb_tray,
            body,
            axes="yz",
            margin=0.006,
            inner_elem="tray_pan",
            outer_elem="rounded_housing",
            name="crumb tray fits between lower slot guides",
        )

    rest_tray_pos = ctx.part_world_position(crumb_tray)
    with ctx.pose({door_joint: 1.35, tray_joint: 0.090}):
        ctx.expect_origin_gap(
            door,
            body,
            axis="x",
            min_gap=0.04,
            name="door swings downward and outward at open pose",
        )
        ctx.expect_overlap(
            crumb_tray,
            body,
            axes="x",
            min_overlap=0.08,
            elem_a="tray_pan",
            elem_b="rounded_housing",
            name="extended crumb tray remains retained in short guides",
        )
        extended_tray_pos = ctx.part_world_position(crumb_tray)

    ctx.check(
        "crumb tray slides forward",
        rest_tray_pos is not None and extended_tray_pos is not None and extended_tray_pos[0] > rest_tray_pos[0] + 0.085,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
