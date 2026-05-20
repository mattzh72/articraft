from __future__ import annotations

import math

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
    wire_from_points,
)


BODY_WIDTH = 0.52
BODY_DEPTH = 0.40
BODY_HEIGHT = 0.34
SHELL_THICKNESS = 0.015
FRONT_TRIM_DEPTH = 0.014

LEFT_BEZEL_WIDTH = 0.020
CONTROL_COLUMN_WIDTH = 0.118

BOTTOM_APRON_HEIGHT = 0.028
TRAY_SLOT_HEIGHT = 0.024
DOOR_BOTTOM_Z = 0.082
DOOR_HEIGHT = 0.205

DOOR_OPENING_WIDTH = BODY_WIDTH - LEFT_BEZEL_WIDTH - CONTROL_COLUMN_WIDTH
DOOR_CENTER_X = (-BODY_WIDTH * 0.5 + LEFT_BEZEL_WIDTH + BODY_WIDTH * 0.5 - CONTROL_COLUMN_WIDTH) * 0.5
DOOR_WIDTH = DOOR_OPENING_WIDTH - 0.008

CAVITY_WIDTH = 0.352
CAVITY_DEPTH = 0.292
CAVITY_HEIGHT = 0.214
CAVITY_CENTER_Y = FRONT_TRIM_DEPTH + CAVITY_DEPTH * 0.5
CAVITY_CENTER_Z = DOOR_BOTTOM_Z + CAVITY_HEIGHT * 0.5

TRAY_WIDTH = 0.338
TRAY_DEPTH = 0.272
TRAY_TRAVEL = 0.160

RACK_WIDTH = 0.326
RACK_DEPTH = 0.266
RACK_TRAVEL = 0.130


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.14, 0.16, 0.18, 0.45))
    cavity_dark = model.material("cavity_dark", rgba=(0.20, 0.20, 0.21, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.18, 0.19, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5, SHELL_THICKNESS * 0.5)),
        material=stainless,
        name="base",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5, BODY_HEIGHT - SHELL_THICKNESS * 0.5)),
        material=stainless,
        name="top",
    )
    body.visual(
        Box((SHELL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(-BODY_WIDTH * 0.5 + SHELL_THICKNESS * 0.5, BODY_DEPTH * 0.5, BODY_HEIGHT * 0.5)
        ),
        material=stainless,
        name="left_shell",
    )
    body.visual(
        Box((SHELL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(BODY_WIDTH * 0.5 - SHELL_THICKNESS * 0.5, BODY_DEPTH * 0.5, BODY_HEIGHT * 0.5)
        ),
        material=stainless,
        name="right_shell",
    )
    body.visual(
        Box((BODY_WIDTH, SHELL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, BODY_DEPTH - SHELL_THICKNESS * 0.5, BODY_HEIGHT * 0.5)),
        material=cavity_dark,
        name="back_panel",
    )
    body.visual(
        Box((BODY_WIDTH, FRONT_TRIM_DEPTH, BOTTOM_APRON_HEIGHT)),
        origin=Origin(xyz=(0.0, FRONT_TRIM_DEPTH * 0.5, BOTTOM_APRON_HEIGHT * 0.5)),
        material=stainless,
        name="front_apron",
    )
    body.visual(
        Box((LEFT_BEZEL_WIDTH, FRONT_TRIM_DEPTH, BODY_HEIGHT - BOTTOM_APRON_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH * 0.5 + LEFT_BEZEL_WIDTH * 0.5,
                FRONT_TRIM_DEPTH * 0.5,
                BOTTOM_APRON_HEIGHT + (BODY_HEIGHT - BOTTOM_APRON_HEIGHT) * 0.5,
            )
        ),
        material=stainless,
        name="left_bezel",
    )
    body.visual(
        Box((CONTROL_COLUMN_WIDTH, FRONT_TRIM_DEPTH, BODY_HEIGHT - BOTTOM_APRON_HEIGHT)),
        origin=Origin(
            xyz=(
                BODY_WIDTH * 0.5 - CONTROL_COLUMN_WIDTH * 0.5,
                FRONT_TRIM_DEPTH * 0.5,
                BOTTOM_APRON_HEIGHT + (BODY_HEIGHT - BOTTOM_APRON_HEIGHT) * 0.5,
            )
        ),
        material=stainless,
        name="control_panel",
    )
    body.visual(
        Box((DOOR_OPENING_WIDTH, FRONT_TRIM_DEPTH, BODY_HEIGHT - (DOOR_BOTTOM_Z + DOOR_HEIGHT))),
        origin=Origin(
            xyz=(
                DOOR_CENTER_X,
                FRONT_TRIM_DEPTH * 0.5,
                DOOR_BOTTOM_Z + DOOR_HEIGHT + (BODY_HEIGHT - (DOOR_BOTTOM_Z + DOOR_HEIGHT)) * 0.5,
            )
        ),
        material=stainless,
        name="top_bezel",
    )
    body.visual(
        Box((DOOR_OPENING_WIDTH, FRONT_TRIM_DEPTH, DOOR_BOTTOM_Z - (BOTTOM_APRON_HEIGHT + TRAY_SLOT_HEIGHT))),
        origin=Origin(
            xyz=(
                DOOR_CENTER_X,
                FRONT_TRIM_DEPTH * 0.5,
                BOTTOM_APRON_HEIGHT
                + TRAY_SLOT_HEIGHT
                + (DOOR_BOTTOM_Z - (BOTTOM_APRON_HEIGHT + TRAY_SLOT_HEIGHT)) * 0.5,
            )
        ),
        material=stainless,
        name="tray_header",
    )
    body.visual(
        Box((CAVITY_WIDTH, CAVITY_DEPTH, 0.010)),
        origin=Origin(xyz=(DOOR_CENTER_X, CAVITY_CENTER_Y, DOOR_BOTTOM_Z - 0.005)),
        material=cavity_dark,
        name="cavity_floor",
    )
    body.visual(
        Box((0.010, CAVITY_DEPTH, CAVITY_HEIGHT)),
        origin=Origin(
            xyz=(DOOR_CENTER_X - CAVITY_WIDTH * 0.5 - 0.005, CAVITY_CENTER_Y, CAVITY_CENTER_Z)
        ),
        material=cavity_dark,
        name="cavity_wall_left",
    )
    body.visual(
        Box((0.010, CAVITY_DEPTH, CAVITY_HEIGHT)),
        origin=Origin(
            xyz=(DOOR_CENTER_X + CAVITY_WIDTH * 0.5 + 0.005, CAVITY_CENTER_Y, CAVITY_CENTER_Z)
        ),
        material=cavity_dark,
        name="cavity_wall_right",
    )
    body.visual(
        Box((0.020, 0.268, 0.008)),
        origin=Origin(
            xyz=(DOOR_CENTER_X - TRAY_WIDTH * 0.5 - 0.010, 0.140, 0.036),
        ),
        material=cavity_dark,
        name="crumb_guide_left",
    )
    body.visual(
        Box((0.020, 0.268, 0.008)),
        origin=Origin(
            xyz=(DOOR_CENTER_X + TRAY_WIDTH * 0.5 + 0.010, 0.140, 0.036),
        ),
        material=cavity_dark,
        name="crumb_guide_right",
    )
    body.visual(
        Box((0.010, 0.268, 0.020)),
        origin=Origin(
            xyz=(DOOR_CENTER_X - TRAY_WIDTH * 0.5 - 0.015, 0.140, 0.025),
        ),
        material=cavity_dark,
        name="crumb_support_left",
    )
    body.visual(
        Box((0.010, 0.268, 0.020)),
        origin=Origin(
            xyz=(DOOR_CENTER_X + TRAY_WIDTH * 0.5 + 0.015, 0.140, 0.025),
        ),
        material=cavity_dark,
        name="crumb_support_right",
    )
    body.visual(
        Box((0.018, 0.286, 0.006)),
        origin=Origin(
            xyz=(DOOR_CENTER_X - RACK_WIDTH * 0.5 - 0.007, 0.171, 0.134),
        ),
        material=cavity_dark,
        name="rack_runner_left",
    )
    body.visual(
        Box((0.018, 0.286, 0.006)),
        origin=Origin(
            xyz=(DOOR_CENTER_X + RACK_WIDTH * 0.5 + 0.007, 0.171, 0.134),
        ),
        material=cavity_dark,
        name="rack_runner_right",
    )

    chrome_knob = mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.026,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.050, 0.005, flare=0.05),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            center=False,
        ),
        "toaster_oven_chrome_knob",
    )

    chrome = model.material("chrome", rgba=(0.84, 0.85, 0.87, 1.0))

    for index, knob_z in enumerate((0.232, 0.164)):
        knob = model.part(f"knob_{index}")
        knob.visual(
            chrome_knob,
            origin=Origin(
                xyz=(0.0, 0.0, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=chrome,
            name="knob",
        )
        model.articulation(
            f"knob_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(BODY_WIDTH * 0.5 - CONTROL_COLUMN_WIDTH * 0.5, 0.0, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.25,
                velocity=6.0,
            ),
        )

    door = model.part("door")
    door_thickness = 0.020
    frame_width = 0.028
    door.visual(
        Box((DOOR_WIDTH, door_thickness, frame_width)),
        origin=Origin(xyz=(0.0, -door_thickness * 0.5, frame_width * 0.5)),
        material=stainless,
        name="door_bottom",
    )
    door.visual(
        Box((DOOR_WIDTH, door_thickness, frame_width)),
        origin=Origin(xyz=(0.0, -door_thickness * 0.5, DOOR_HEIGHT - frame_width * 0.5)),
        material=stainless,
        name="door_top",
    )
    door.visual(
        Box((frame_width, door_thickness, DOOR_HEIGHT)),
        origin=Origin(xyz=(-DOOR_WIDTH * 0.5 + frame_width * 0.5, -door_thickness * 0.5, DOOR_HEIGHT * 0.5)),
        material=stainless,
        name="door_left",
    )
    door.visual(
        Box((frame_width, door_thickness, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH * 0.5 - frame_width * 0.5, -door_thickness * 0.5, DOOR_HEIGHT * 0.5)),
        material=stainless,
        name="door_right",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.048, 0.008, DOOR_HEIGHT - 0.050)),
        origin=Origin(xyz=(0.0, -0.010, DOOR_HEIGHT * 0.5)),
        material=dark_glass,
        name="door_glass",
    )
    handle_span = DOOR_WIDTH - 0.120
    handle_z = DOOR_HEIGHT - 0.042
    handle_y = -0.036
    door.visual(
        Cylinder(radius=0.008, length=handle_span),
        origin=Origin(xyz=(0.0, handle_y, handle_z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=handle_dark,
        name="door_handle",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        door.visual(
            Box((0.012, 0.022, 0.020)),
            origin=Origin(
                xyz=(sign * (handle_span * 0.5 - 0.018), -0.018, handle_z),
            ),
            material=handle_dark,
            name=f"handle_mount_{side}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_CENTER_X, 0.0, DOOR_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        Box((TRAY_WIDTH, TRAY_DEPTH, 0.004)),
        origin=Origin(xyz=(0.0, TRAY_DEPTH * 0.5, 0.002)),
        material=cavity_dark,
        name="tray_pan",
    )
    crumb_tray.visual(
        Box((0.004, TRAY_DEPTH, 0.012)),
        origin=Origin(
            xyz=(-TRAY_WIDTH * 0.5 + 0.002, TRAY_DEPTH * 0.5, 0.006),
        ),
        material=stainless,
        name="tray_side_left",
    )
    crumb_tray.visual(
        Box((0.004, TRAY_DEPTH, 0.012)),
        origin=Origin(
            xyz=(TRAY_WIDTH * 0.5 - 0.002, TRAY_DEPTH * 0.5, 0.006),
        ),
        material=stainless,
        name="tray_side_right",
    )
    crumb_tray.visual(
        Box((TRAY_WIDTH, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, TRAY_DEPTH - 0.002, 0.006)),
        material=stainless,
        name="tray_back",
    )
    crumb_tray.visual(
        Box((TRAY_WIDTH - 0.036, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.005, 0.006)),
        material=stainless,
        name="tray_lip",
    )
    model.articulation(
        "crumb_tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=crumb_tray,
        origin=Origin(xyz=(DOOR_CENTER_X, 0.006, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.20,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    rack_ring = mesh_from_geometry(
        wire_from_points(
            [
                (-RACK_WIDTH * 0.5, 0.0, 0.010),
                (RACK_WIDTH * 0.5, 0.0, 0.010),
                (RACK_WIDTH * 0.5, RACK_DEPTH, 0.010),
                (-RACK_WIDTH * 0.5, RACK_DEPTH, 0.010),
            ],
            radius=0.003,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=0.008,
            radial_segments=16,
        ),
        "toaster_oven_rack_ring",
    )
    rack = model.part("rack")
    rack.visual(
        rack_ring,
        material=chrome,
        name="rack_ring",
    )
    for index, y_pos in enumerate((0.050, 0.096, 0.142, 0.188, 0.234)):
        rack.visual(
            Cylinder(radius=0.0024, length=RACK_WIDTH + 0.004),
            origin=Origin(
                xyz=(0.0, y_pos, 0.010),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=chrome,
            name=f"rack_cross_{index}",
        )
    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(DOOR_CENTER_X, 0.028, 0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=0.0,
            upper=RACK_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    crumb_tray = object_model.get_part("crumb_tray")
    rack = object_model.get_part("rack")
    door_hinge = object_model.get_articulation("door_hinge")
    crumb_tray_slide = object_model.get_articulation("crumb_tray_slide")
    rack_slide = object_model.get_articulation("rack_slide")
    knob_0_spin = object_model.get_articulation("knob_0_spin")
    knob_1_spin = object_model.get_articulation("knob_1_spin")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="door closes against the front opening plane",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="xz",
            min_overlap=0.10,
            name="door covers the front opening",
        )
        ctx.expect_within(
            crumb_tray,
            body,
            axes="x",
            inner_elem="tray_pan",
            outer_elem="cavity_floor",
            margin=0.002,
            name="crumb tray stays centered below the cavity",
        )
        ctx.expect_within(
            rack,
            body,
            axes="x",
            inner_elem="rack_ring",
            outer_elem="cavity_floor",
            margin=0.003,
            name="wire rack stays centered between the side walls",
        )
        ctx.expect_overlap(
            crumb_tray,
            body,
            axes="y",
            elem_a="tray_pan",
            elem_b="crumb_guide_left",
            min_overlap=0.20,
            name="crumb tray remains deeply inserted at rest",
        )
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            elem_a="rack_ring",
            elem_b="rack_runner_left",
            min_overlap=0.23,
            name="wire rack remains deeply inserted at rest",
        )

    closed_aabb = ctx.part_world_aabb(door)
    tray_rest = ctx.part_world_position(crumb_tray)
    rack_rest = ctx.part_world_position(rack)
    with ctx.pose({door_hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(door)
    with ctx.pose({crumb_tray_slide: TRAY_TRAVEL}):
        tray_extended = ctx.part_world_position(crumb_tray)
        ctx.expect_overlap(
            crumb_tray,
            body,
            axes="y",
            elem_a="tray_pan",
            elem_b="crumb_guide_left",
            min_overlap=0.10,
            name="crumb tray remains retained on the guides when extended",
        )
    with ctx.pose({rack_slide: RACK_TRAVEL}):
        rack_extended = ctx.part_world_position(rack)
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            elem_a="rack_ring",
            elem_b="rack_runner_left",
            min_overlap=0.12,
            name="wire rack remains retained on the runners when extended",
        )

    ctx.check(
        "door opens downward from the lower hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] < closed_aabb[1][2] - 0.12
        and open_aabb[0][1] < closed_aabb[0][1] - 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    ctx.check(
        "crumb tray pulls forward from the lower slot",
        tray_rest is not None and tray_extended is not None and tray_extended[1] < tray_rest[1] - 0.12,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )
    ctx.check(
        "wire rack slides forward from the cavity",
        rack_rest is not None and rack_extended is not None and rack_extended[1] < rack_rest[1] - 0.10,
        details=f"rest={rack_rest}, extended={rack_extended}",
    )
    ctx.check(
        "both front knobs are continuous rotary controls",
        knob_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_1_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_0_spin.motion_limits is not None
        and knob_1_spin.motion_limits is not None
        and knob_0_spin.motion_limits.lower is None
        and knob_0_spin.motion_limits.upper is None
        and knob_1_spin.motion_limits.lower is None
        and knob_1_spin.motion_limits.upper is None,
        details=(
            f"knob_0={knob_0_spin.articulation_type}/{knob_0_spin.motion_limits}, "
            f"knob_1={knob_1_spin.articulation_type}/{knob_1_spin.motion_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
