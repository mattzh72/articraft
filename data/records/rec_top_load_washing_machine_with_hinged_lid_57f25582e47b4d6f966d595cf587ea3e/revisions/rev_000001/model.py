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


CABINET_W = 0.69
CABINET_D = 0.70
BODY_H = 0.89
BODY_WALL = 0.018
BACKSPLASH_DEPTH = 0.095
BACKSPLASH_H = 0.115
CONTROL_FACE_Y = CABINET_D * 0.5 - BACKSPLASH_DEPTH

OPENING_W = 0.455
OPENING_D = 0.435
OPENING_CENTER_Y = -0.055

LID_W = 0.605
LID_D = 0.540
LID_T = 0.024
LID_HINGE_Y = OPENING_CENTER_Y + OPENING_D * 0.5 + 0.030
LID_HINGE_Z = BODY_H + 0.014

TUB_DEPTH = 0.560
TUB_OUTER_R = 0.198
TUB_WALL = 0.010
TUB_RIM_R = 0.212
TUB_HUB_R = 0.056
TUB_HUB_H = 0.018
AGITATOR_POST_R = 0.022
AGITATOR_POST_H = 0.180
TUB_RIM_Z = BODY_H - BODY_WALL

DIAL_OUTER_R = 0.046
DIAL_INNER_R = 0.030
DIAL_DEPTH = 0.018
DIAL_X = -0.150
DIAL_Z = BODY_H + 0.065

BUTTON_W = 0.034
BUTTON_H = 0.034
BUTTON_DEPTH = 0.010
BUTTON_REST_OFFSET = -0.007
BUTTON_TRAVEL = 0.002
BUTTON_XS = (0.050, 0.095, 0.140, 0.185)
BUTTON_Z = BODY_H + 0.065


def _cabinet_shape() -> cq.Workplane:
    cabinet = (
        cq.Workplane("XY")
        .box(CABINET_W, CABINET_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.020)
        .faces("<Z")
        .shell(-BODY_WALL)
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .center(0.0, OPENING_CENTER_Y)
        .rect(OPENING_W, OPENING_D)
        .cutThruAll()
    )

    backsplash = (
        cq.Workplane("XY")
        .box(CABINET_W, BACKSPLASH_DEPTH, BACKSPLASH_H, centered=(True, True, False))
        .translate((0.0, CONTROL_FACE_Y + BACKSPLASH_DEPTH * 0.5, BODY_H))
        .edges(">Z")
        .fillet(0.010)
    )

    return cabinet.union(backsplash)


def _lid_shape() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T, centered=(True, False, False))
        .translate((0.0, -LID_D, -0.012))
        .edges("|Z")
        .fillet(0.010)
    )

    lid = (
        lid.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .rect(LID_W - 0.100, LID_D - 0.085)
        .cutBlind(-0.004)
    )

    return lid


def _tub_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .circle(TUB_OUTER_R)
        .extrude(TUB_DEPTH)
        .faces(">Z")
        .shell(-TUB_WALL)
        .translate((0.0, 0.0, -TUB_DEPTH))
    )

    rim = (
        cq.Workplane("XY")
        .circle(TUB_RIM_R)
        .circle(TUB_OUTER_R - 0.006)
        .extrude(0.012)
        .translate((0.0, 0.0, -0.012))
    )

    hub = (
        cq.Workplane("XY")
        .circle(TUB_HUB_R)
        .extrude(TUB_HUB_H)
        .translate((0.0, 0.0, -TUB_DEPTH + TUB_WALL))
    )

    return shell.union(rim).union(hub)


def _dial_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .circle(DIAL_OUTER_R)
        .circle(DIAL_INNER_R)
        .extrude(DIAL_DEPTH)
    )

    rear_flange = (
        cq.Workplane("XY")
        .circle(DIAL_OUTER_R - 0.002)
        .circle(DIAL_INNER_R + 0.004)
        .extrude(0.002)
        .translate((0.0, 0.0, -0.002))
    )

    pointer = (
        cq.Workplane("XY")
        .box(0.008, 0.016, 0.003, centered=(True, True, False))
        .translate((0.0, DIAL_OUTER_R - 0.010, DIAL_DEPTH))
    )

    return ring.union(rear_flange).union(pointer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.96, 0.97, 0.98, 1.0))
    warm_white = model.material("warm_white", rgba=(0.93, 0.94, 0.95, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.77, 0.80, 0.84, 1.0))
    dial_silver = model.material("dial_silver", rgba=(0.73, 0.76, 0.79, 1.0))
    tub_white = model.material("tub_white", rgba=(0.89, 0.90, 0.92, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.62, 0.65, 0.70, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shape(), "washer_cabinet_shell"),
        material=cabinet_white,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((0.570, 0.006, 0.086)),
        origin=Origin(xyz=(0.020, CONTROL_FACE_Y + 0.001, BUTTON_Z)),
        material=panel_grey,
        name="control_face",
    )
    cabinet.visual(
        Box((0.004, 0.008, 0.018)),
        origin=Origin(xyz=(DIAL_X, CONTROL_FACE_Y - 0.001, DIAL_Z + 0.055)),
        material=accent_grey,
        name="dial_index",
    )
    cabinet.visual(
        Box((0.030, 0.140, 0.002)),
        origin=Origin(xyz=(-0.282, OPENING_CENTER_Y, BODY_H + 0.001)),
        material=warm_white,
        name="lid_pad_0",
    )
    cabinet.visual(
        Box((0.030, 0.140, 0.002)),
        origin=Origin(xyz=(0.282, OPENING_CENTER_Y, BODY_H + 0.001)),
        material=warm_white,
        name="lid_pad_1",
    )
    cabinet.visual(
        Cylinder(radius=0.036, length=0.262),
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, 0.131)),
        material=accent_grey,
        name="drive_mount",
    )
    cabinet.visual(
        Box((0.060, 0.300, 0.032)),
        origin=Origin(xyz=(0.0, -0.195, 0.032)),
        material=accent_grey,
        name="drive_brace",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "washer_lid_panel"),
        material=warm_white,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.100),
        origin=Origin(xyz=(-0.190, 0.006, 0.000), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=panel_grey,
        name="hinge_barrel_0",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.100),
        origin=Origin(xyz=(0.190, 0.006, 0.000), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=panel_grey,
        name="hinge_barrel_1",
    )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_tub_shape(), "washer_tub"),
        material=tub_white,
        name="tub_shell",
    )
    tub.visual(
        Cylinder(radius=AGITATOR_POST_R, length=AGITATOR_POST_H),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -TUB_DEPTH + TUB_WALL + TUB_HUB_H + AGITATOR_POST_H * 0.5,
            )
        ),
        material=accent_grey,
        name="agitator_post",
    )
    tub.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -TUB_DEPTH - 0.025)),
        material=accent_grey,
        name="drive_shaft",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_shape(), "washer_ring_dial"),
        origin=Origin(xyz=(0.0, -0.0040, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dial_silver,
        name="dial_ring",
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_W, BUTTON_DEPTH, BUTTON_H)),
            origin=Origin(xyz=(0.0, BUTTON_REST_OFFSET, 0.0)),
            material=panel_grey,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(button_x, CONTROL_FACE_Y, BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.060,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    agitator_cap = model.part("agitator_cap")
    agitator_cap.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=accent_grey,
        name="cap_stem",
    )
    agitator_cap.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=accent_grey,
        name="agitator_cap",
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=2.5,
            lower=0.0,
            upper=1.42,
        ),
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, TUB_RIM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=9.0,
        ),
    )
    model.articulation(
        "cabinet_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(DIAL_X, CONTROL_FACE_Y, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=5.0,
        ),
    )
    model.articulation(
        "tub_to_agitator_cap",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=agitator_cap,
        origin=Origin(xyz=(0.0, 0.0, -TUB_DEPTH + TUB_WALL + TUB_HUB_H + AGITATOR_POST_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    agitator_cap = object_model.get_part("agitator_cap")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    tub_spin = object_model.get_articulation("cabinet_to_tub")
    dial_spin = object_model.get_articulation("cabinet_to_dial")
    cap_spin = object_model.get_articulation("tub_to_agitator_cap")

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    ctx.check("cabinet_aabb_present", cabinet_aabb is not None, "Expected cabinet bounds.")
    if cabinet_aabb is not None:
        cabinet_min, cabinet_max = cabinet_aabb
        cabinet_size = tuple(float(cabinet_max[i] - cabinet_min[i]) for i in range(3))
        ctx.check(
            "washer_domestic_scale",
            0.64 <= cabinet_size[0] <= 0.74
            and 0.66 <= cabinet_size[1] <= 0.74
            and 0.98 <= cabinet_size[2] <= 1.05,
            details=f"cabinet_size={cabinet_size!r}",
        )

    tub_aabb = ctx.part_world_aabb(tub)
    ctx.check("tub_aabb_present", tub_aabb is not None, "Expected tub bounds.")
    if tub_aabb is not None:
        tub_min, tub_max = tub_aabb
        tub_size = tuple(float(tub_max[i] - tub_min[i]) for i in range(3))
        ctx.check("deep_laundry_cavity", tub_size[2] >= 0.54, details=f"tub_size={tub_size!r}")

    closed_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check("lid_aabb_present", closed_lid_aabb is not None, "Expected lid bounds.")
    if closed_lid_aabb is not None and tub_aabb is not None:
        lid_min, lid_max = closed_lid_aabb
        _tub_min, tub_max = tub_aabb
        ctx.check(
            "lid_closes_above_tub",
            lid_min[2] >= tub_max[2] + 0.015,
            details=f"lid_min_z={lid_min[2]}, tub_max_z={tub_max[2]}",
        )

    ctx.expect_overlap(
        lid,
        tub,
        axes="xy",
        min_overlap=0.30,
        name="lid covers the tub opening",
    )
    ctx.expect_within(
        agitator_cap,
        tub,
        axes="xy",
        margin=0.0,
        name="agitator cap stays centered inside tub",
    )

    upper_lid = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if upper_lid is not None and closed_lid_aabb is not None:
        with ctx.pose({lid_hinge: upper_lid}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check("open_lid_aabb_present", open_lid_aabb is not None, "Expected open lid bounds.")
        if open_lid_aabb is not None:
            closed_max_z = closed_lid_aabb[1][2]
            open_max_z = open_lid_aabb[1][2]
            ctx.check(
                "lid_opens_upward",
                open_max_z >= closed_max_z + 0.25,
                details=f"closed_max_z={closed_max_z}, open_max_z={open_max_z}",
            )

    for joint_name in ("cabinet_to_tub", "cabinet_to_dial", "tub_to_agitator_cap"):
        articulation = object_model.get_articulation(joint_name)
        limits = articulation.motion_limits
        ctx.check(
            f"{joint_name}_is_continuous",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"limits={limits!r}",
        )

    ctx.check(
        "continuous_axes_match_prompt",
        tub_spin.axis == (0.0, 0.0, 1.0)
        and dial_spin.axis == (0.0, 1.0, 0.0)
        and cap_spin.axis == (0.0, 0.0, 1.0),
        details=f"tub_axis={tub_spin.axis}, dial_axis={dial_spin.axis}, cap_axis={cap_spin.axis}",
    )

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"cabinet_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
        pressed_pos = None
        if upper is not None:
            with ctx.pose({button_joint: upper}):
                pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index}_presses_inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] >= rest_pos[1] + BUTTON_TRAVEL * 0.75,
            details=f"rest_pos={rest_pos}, pressed_pos={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
