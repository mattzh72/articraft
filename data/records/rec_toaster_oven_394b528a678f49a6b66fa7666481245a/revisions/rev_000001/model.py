from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
)


BODY_W = 0.48
BODY_D = 0.36
BODY_H = 0.29
SHELL_T = 0.012
FRONT_T = 0.014
BACK_T = 0.010

OPENING_CENTER_X = -0.05
OPENING_W = 0.32
DOOR_HINGE_Z = 0.062
DOOR_W = 0.318
DOOR_H = 0.182
DOOR_T = 0.016

TRAY_W = 0.302
TRAY_D = 0.22
TRAY_H = 0.018
TRAY_TRAVEL = 0.12


def add_knob(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    x: float,
    z: float,
    knob_material,
) -> None:
    knob = model.part(name)
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.028,
                body_style="skirted",
                top_diameter=0.031,
                skirt=KnobSkirt(0.044, 0.006, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(
                    style="line",
                    mode="engraved",
                    depth=0.0007,
                    angle_deg=0.0,
                ),
                center=False,
            ),
            f"{name}_shell",
        ),
        material=knob_material,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        name="knob_shell",
    )
    model.articulation(
        f"body_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(x, -BODY_D / 2.0, z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toaster_oven")

    shell_metal = model.material("shell_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    trim_metal = model.material("trim_metal", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.12, 0.14, 0.17, 0.95))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.74, 0.75, 0.77, 1.0))
    latch_black = model.material("latch_black", rgba=(0.14, 0.14, 0.15, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        Box((SHELL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + SHELL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=shell_metal,
        name="left_shell",
    )
    body.visual(
        Box((SHELL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - SHELL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=shell_metal,
        name="right_shell",
    )
    body.visual(
        Box((BODY_W, BODY_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - SHELL_T / 2.0)),
        material=shell_metal,
        name="top_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, BACK_T, BODY_H - SHELL_T)),
        origin=Origin(
            xyz=(0.0, BODY_D / 2.0 - BACK_T / 2.0, (BODY_H - SHELL_T) / 2.0)
        ),
        material=shell_metal,
        name="back_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, BODY_D - BACK_T, 0.022)),
        origin=Origin(xyz=(0.0, -BACK_T / 2.0, 0.011)),
        material=shell_metal,
        name="lower_base",
    )
    body.visual(
        Box((0.03, FRONT_T, BODY_H - SHELL_T)),
        origin=Origin(
            xyz=(-BODY_W / 2.0 + 0.015, -BODY_D / 2.0 + FRONT_T / 2.0, 0.139)
        ),
        material=trim_metal,
        name="left_jamb",
    )
    body.visual(
        Box((0.13, FRONT_T, BODY_H - SHELL_T)),
        origin=Origin(
            xyz=(BODY_W / 2.0 - 0.065, -BODY_D / 2.0 + FRONT_T / 2.0, 0.139)
        ),
        material=trim_metal,
        name="control_panel",
    )
    body.visual(
        Box((0.012, BODY_D - BACK_T, BODY_H - SHELL_T)),
        origin=Origin(xyz=(0.116, -BACK_T / 2.0, 0.139)),
        material=shell_metal,
        name="control_wall",
    )
    body.visual(
        Box((OPENING_W, FRONT_T, BODY_H - (DOOR_HINGE_Z + DOOR_H))),
        origin=Origin(
            xyz=(
                OPENING_CENTER_X,
                -BODY_D / 2.0 + FRONT_T / 2.0,
                DOOR_HINGE_Z + DOOR_H + (BODY_H - (DOOR_HINGE_Z + DOOR_H)) / 2.0,
            )
        ),
        material=trim_metal,
        name="front_header",
    )
    body.visual(
        Box((OPENING_W, FRONT_T, 0.010)),
        origin=Origin(
            xyz=(OPENING_CENTER_X, -BODY_D / 2.0 + FRONT_T / 2.0, DOOR_HINGE_Z - 0.005)
        ),
        material=trim_metal,
        name="front_sill",
    )
    body.visual(
        Box((0.33, 0.32, 0.008)),
        origin=Origin(xyz=(-0.045, -0.015, 0.071)),
        material=shell_metal,
        name="chamber_floor",
    )
    body.visual(
        Box((0.020, 0.16, 0.008)),
        origin=Origin(xyz=(-0.219, -0.03, 0.049)),
        material=trim_metal,
        name="guide_0",
    )
    body.visual(
        Box((0.016, 0.16, 0.008)),
        origin=Origin(xyz=(0.116, -0.03, 0.049)),
        material=trim_metal,
        name="guide_1",
    )
    body.visual(
        Box((0.020, 0.19, 0.004)),
        origin=Origin(xyz=(-0.219, 0.01, 0.145)),
        material=trim_metal,
        name="rack_0",
    )
    body.visual(
        Box((0.016, 0.19, 0.004)),
        origin=Origin(xyz=(0.116, 0.01, 0.145)),
        material=trim_metal,
        name="rack_1",
    )

    for idx, (x, y) in enumerate(((-0.18, -0.12), (0.18, -0.12), (-0.18, 0.12), (0.18, 0.12))):
        body.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(x, y, -0.006)),
            material=rubber,
            name=f"foot_{idx}",
        )

    door = model.part("door")
    frame_border = 0.03
    stile_h = DOOR_H - 2.0 * frame_border
    door.visual(
        Box((DOOR_W, DOOR_T, frame_border)),
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0, frame_border / 2.0)),
        material=trim_metal,
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_W, DOOR_T, frame_border)),
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0, DOOR_H - frame_border / 2.0)),
        material=trim_metal,
        name="top_rail",
    )
    door.visual(
        Box((frame_border, DOOR_T, stile_h)),
        origin=Origin(
            xyz=(-DOOR_W / 2.0 + frame_border / 2.0, -DOOR_T / 2.0, DOOR_H / 2.0)
        ),
        material=trim_metal,
        name="left_stile",
    )
    door.visual(
        Box((frame_border, DOOR_T, stile_h)),
        origin=Origin(
            xyz=(DOOR_W / 2.0 - frame_border / 2.0, -DOOR_T / 2.0, DOOR_H / 2.0)
        ),
        material=trim_metal,
        name="right_stile",
    )
    door.visual(
        Box((DOOR_W - 0.088, 0.004, DOOR_H - 0.070)),
        origin=Origin(xyz=(0.0, -0.010, DOOR_H / 2.0 + 0.004)),
        material=dark_glass,
        name="window",
    )
    door.visual(
        Cylinder(radius=0.0075, length=0.18),
        origin=Origin(xyz=(0.0, -0.032, DOOR_H - 0.032), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_metal,
        name="handle_bar",
    )
    for idx, x in enumerate((-0.065, 0.065)):
        door.visual(
            Cylinder(radius=0.004, length=0.018),
            origin=Origin(xyz=(x, -0.021, DOOR_H - 0.032), rpy=(pi / 2.0, 0.0, 0.0)),
            material=trim_metal,
            name=f"handle_post_{idx}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(OPENING_CENTER_X, -BODY_D / 2.0, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.75),
    )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_W, TRAY_D, 0.002)),
        origin=Origin(xyz=(0.0, TRAY_D / 2.0, -TRAY_H / 2.0 + 0.001)),
        material=tray_metal,
        name="tray_bottom",
    )
    tray.visual(
        Box((0.008, TRAY_D, TRAY_H)),
        origin=Origin(xyz=(-TRAY_W / 2.0 + 0.004, TRAY_D / 2.0, 0.0)),
        material=tray_metal,
        name="tray_wall_0",
    )
    tray.visual(
        Box((0.008, TRAY_D, TRAY_H)),
        origin=Origin(xyz=(TRAY_W / 2.0 - 0.004, TRAY_D / 2.0, 0.0)),
        material=tray_metal,
        name="tray_wall_1",
    )
    tray.visual(
        Box((TRAY_W, 0.008, TRAY_H)),
        origin=Origin(xyz=(0.0, TRAY_D - 0.004, 0.0)),
        material=tray_metal,
        name="tray_back",
    )
    tray.visual(
        Box((TRAY_W + 0.008, 0.010, TRAY_H + 0.010)),
        origin=Origin(xyz=(0.0, -0.005, 0.005)),
        material=tray_metal,
        name="front_lip",
    )
    tray.visual(
        Box((0.12, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.015, 0.008)),
        material=trim_metal,
        name="tray_handle",
    )
    tray.visual(
        Box((0.010, 0.018, 0.010)),
        origin=Origin(xyz=(TRAY_W / 2.0 + 0.005, 0.002, 0.010)),
        material=tray_metal,
        name="tray_tab",
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(OPENING_CENTER_X, -BODY_D / 2.0 + 0.002, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.15,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    latch = model.part("side_latch")
    latch.visual(
        Cylinder(radius=0.005, length=0.014),
        material=latch_black,
        name="pivot",
    )
    latch.visual(
        Box((0.028, 0.006, 0.008)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0)),
        material=latch_black,
        name="latch_arm",
    )
    latch.visual(
        Box((0.010, 0.010, 0.006)),
        origin=Origin(xyz=(-0.028, -0.004, -0.001)),
        material=latch_black,
        name="hook_tip",
    )

    model.articulation(
        "body_to_side_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(0.120, -0.181, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.10),
    )

    add_knob(model, body, name="mode_knob", x=0.175, z=0.214, knob_material=knob_black)
    add_knob(model, body, name="temp_knob", x=0.175, z=0.145, knob_material=knob_black)
    add_knob(model, body, name="timer_knob", x=0.175, z=0.076, knob_material=knob_black)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    tray = object_model.get_part("tray")
    latch = object_model.get_part("side_latch")

    door_joint = object_model.get_articulation("body_to_door")
    tray_joint = object_model.get_articulation("body_to_tray")
    latch_joint = object_model.get_articulation("body_to_side_latch")

    knob_joints = (
        object_model.get_articulation("body_to_mode_knob"),
        object_model.get_articulation("body_to_temp_knob"),
        object_model.get_articulation("body_to_timer_knob"),
    )

    ctx.expect_gap(
        body,
        door,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="door seats against the front frame",
    )
    ctx.expect_overlap(
        latch,
        tray,
        axes="xy",
        elem_a="latch_arm",
        elem_b="tray_tab",
        min_overlap=0.004,
        name="closed latch covers the tray tab",
    )
    ctx.expect_gap(
        latch,
        tray,
        axis="z",
        positive_elem="latch_arm",
        negative_elem="tray_tab",
        min_gap=0.0008,
        max_gap=0.006,
        name="closed latch hovers just above the tray tab",
    )

    def elem_center(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    closed_handle = elem_center(door, "handle_bar")
    with ctx.pose({door_joint: 1.55}):
        open_handle = elem_center(door, "handle_bar")
    ctx.check(
        "door opens downward",
        closed_handle is not None
        and open_handle is not None
        and open_handle[2] < closed_handle[2] - 0.10
        and open_handle[1] < closed_handle[1] - 0.08,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: TRAY_TRAVEL}):
        tray_extended = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.09,
            name="extended tray remains retained inside the oven guides",
        )
    ctx.check(
        "tray pulls forward",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] < tray_rest[1] - 0.10,
        details=f"tray_rest={tray_rest}, tray_extended={tray_extended}",
    )

    closed_latch = elem_center(latch, "latch_arm")
    with ctx.pose({latch_joint: 1.0}):
        open_latch = elem_center(latch, "latch_arm")
    ctx.check(
        "latch swings clear of the tray tab",
        closed_latch is not None
        and open_latch is not None
        and open_latch[0] > closed_latch[0] + 0.004
        and open_latch[1] < closed_latch[1] - 0.008,
        details=f"closed_latch={closed_latch}, open_latch={open_latch}",
    )

    ctx.check(
        "control knobs are continuous",
        all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in knob_joints),
        details=f"knob_types={[joint.articulation_type for joint in knob_joints]}",
    )

    return ctx.report()


object_model = build_object_model()
