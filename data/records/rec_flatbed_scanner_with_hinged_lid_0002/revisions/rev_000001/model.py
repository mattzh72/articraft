from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

BODY_W = 0.44
BODY_D = 0.30
BOTTOM_T = 0.004
WALL_T = 0.018
WALL_H = 0.040
FRAME_H = 0.008
BODY_H = BOTTOM_T + WALL_H + FRAME_H

PLATEN_W = 0.296
PLATEN_D = 0.201
PLATEN_T = 0.004
PLATEN_Z = BODY_H - (PLATEN_T * 0.5)

SIDE_FRAME_W = (BODY_W - PLATEN_W) * 0.5
END_FRAME_D = (BODY_D - PLATEN_D) * 0.5
FRAME_Z = BOTTOM_T + WALL_H + (FRAME_H * 0.5)
LEFT_FRAME_X = -((BODY_W * 0.5) - (SIDE_FRAME_W * 0.5))
RIGHT_FRAME_X = -LEFT_FRAME_X
FRONT_FRAME_Y = -((BODY_D * 0.5) - (END_FRAME_D * 0.5))
REAR_FRAME_Y = -FRONT_FRAME_Y

HINGE_AXIS_Y = BODY_D * 0.5
HINGE_AXIS_Z = BODY_H + 0.008
HINGE_R = 0.007
HINGE_SEGMENT_L = 0.012

LEFT_HINGE_X = -0.158
RIGHT_HINGE_X = 0.158
HINGE_STEP_X = 0.012
LEFT_BODY_OUTER_X = LEFT_HINGE_X - HINGE_STEP_X
LEFT_BODY_INNER_X = LEFT_HINGE_X + HINGE_STEP_X
RIGHT_BODY_INNER_X = RIGHT_HINGE_X - HINGE_STEP_X
RIGHT_BODY_OUTER_X = RIGHT_HINGE_X + HINGE_STEP_X

LID_W = 0.438
LID_D = 0.268
LID_T = 0.012
LID_CENTER = (0.0, -0.148, -0.001)
LID_BACKING_W = 0.362
LID_BACKING_D = 0.236
LID_BACKING_T = 0.003
LID_BACKING_CENTER = (0.0, -0.148, -0.0085)
LID_FRONT_LIP_CENTER = (0.0, -0.274, -0.002)


def _hinge_cylinder_origin(x_pos: float) -> Origin:
    return Origin(
        xyz=(x_pos, HINGE_AXIS_Y, HINGE_AXIS_Z),
        rpy=(0.0, math.pi * 0.5, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flatbed_document_scanner")

    scanner_body = model.material("scanner_body", rgba=(0.82, 0.84, 0.85, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.91, 0.92, 0.93, 1.0))
    liner_white = model.material("liner_white", rgba=(0.96, 0.96, 0.95, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.18, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.86, 0.92, 0.35))
    hinge_metal = model.material("hinge_metal", rgba=(0.57, 0.59, 0.62, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T * 0.5)),
        material=scanner_body,
        name="bottom_plate",
    )
    body.visual(
        Box((WALL_T, BODY_D, WALL_H)),
        origin=Origin(xyz=(-(BODY_W * 0.5) + (WALL_T * 0.5), 0.0, BOTTOM_T + (WALL_H * 0.5))),
        material=scanner_body,
        name="left_wall",
    )
    body.visual(
        Box((WALL_T, BODY_D, WALL_H)),
        origin=Origin(xyz=((BODY_W * 0.5) - (WALL_T * 0.5), 0.0, BOTTOM_T + (WALL_H * 0.5))),
        material=scanner_body,
        name="right_wall",
    )
    body.visual(
        Box((BODY_W - (2.0 * WALL_T), WALL_T, WALL_H)),
        origin=Origin(
            xyz=(0.0, -(BODY_D * 0.5) + (WALL_T * 0.5), BOTTOM_T + (WALL_H * 0.5))
        ),
        material=scanner_body,
        name="front_wall",
    )
    body.visual(
        Box((BODY_W - (2.0 * WALL_T), WALL_T, WALL_H)),
        origin=Origin(
            xyz=(0.0, (BODY_D * 0.5) - (WALL_T * 0.5), BOTTOM_T + (WALL_H * 0.5))
        ),
        material=scanner_body,
        name="rear_wall",
    )
    body.visual(
        Box((SIDE_FRAME_W, PLATEN_D, FRAME_H)),
        origin=Origin(xyz=(LEFT_FRAME_X, 0.0, FRAME_Z)),
        material=dark_trim,
        name="left_frame",
    )
    body.visual(
        Box((SIDE_FRAME_W, PLATEN_D, FRAME_H)),
        origin=Origin(xyz=(RIGHT_FRAME_X, 0.0, FRAME_Z)),
        material=dark_trim,
        name="right_frame",
    )
    body.visual(
        Box((PLATEN_W, END_FRAME_D, FRAME_H)),
        origin=Origin(xyz=(0.0, FRONT_FRAME_Y, FRAME_Z)),
        material=dark_trim,
        name="front_frame",
    )
    body.visual(
        Box((PLATEN_W, END_FRAME_D, FRAME_H)),
        origin=Origin(xyz=(0.0, REAR_FRAME_Y, FRAME_Z)),
        material=dark_trim,
        name="rear_frame",
    )
    body.visual(
        Box((PLATEN_W, PLATEN_D, PLATEN_T)),
        origin=Origin(xyz=(0.0, 0.0, PLATEN_Z)),
        material=glass,
        name="glass_platen",
    )
    body.visual(
        Box((0.140, 0.024, 0.002)),
        origin=Origin(xyz=(-0.085, FRONT_FRAME_Y, BODY_H - 0.001)),
        material=dark_trim,
        name="control_strip",
    )
    body.visual(
        Box((0.042, 0.014, 0.014)),
        origin=Origin(xyz=(LEFT_HINGE_X, 0.152, 0.046)),
        material=hinge_metal,
        name="left_hinge_mount",
    )
    body.visual(
        Box((0.042, 0.014, 0.014)),
        origin=Origin(xyz=(RIGHT_HINGE_X, 0.152, 0.046)),
        material=hinge_metal,
        name="right_hinge_mount",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=0.012),
        origin=_hinge_cylinder_origin(LEFT_BODY_OUTER_X),
        material=hinge_metal,
        name="left_body_outer_knuckle",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=0.012),
        origin=_hinge_cylinder_origin(LEFT_BODY_INNER_X),
        material=hinge_metal,
        name="left_body_inner_knuckle",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=0.012),
        origin=_hinge_cylinder_origin(RIGHT_BODY_INNER_X),
        material=hinge_metal,
        name="right_body_inner_knuckle",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=0.012),
        origin=_hinge_cylinder_origin(RIGHT_BODY_OUTER_X),
        material=hinge_metal,
        name="right_body_outer_knuckle",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_T)),
        origin=Origin(xyz=LID_CENTER),
        material=lid_finish,
        name="lid_panel",
    )
    lid.visual(
        Box((LID_BACKING_W, LID_BACKING_D, LID_BACKING_T)),
        origin=Origin(xyz=LID_BACKING_CENTER),
        material=liner_white,
        name="lid_backing",
    )
    lid.visual(
        Box((LID_W - 0.050, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.016, -0.004)),
        material=lid_finish,
        name="lid_rear_rail",
    )
    lid.visual(
        Box((0.026, 0.019, 0.010)),
        origin=Origin(xyz=(LEFT_HINGE_X, -0.016, -0.004)),
        material=hinge_metal,
        name="left_lid_leaf",
    )
    lid.visual(
        Box((0.026, 0.019, 0.010)),
        origin=Origin(xyz=(RIGHT_HINGE_X, -0.016, -0.004)),
        material=hinge_metal,
        name="right_lid_leaf",
    )
    lid.visual(
        Box((0.152, 0.012, 0.010)),
        origin=Origin(xyz=LID_FRONT_LIP_CENTER),
        material=dark_trim,
        name="lid_front_lip",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=HINGE_SEGMENT_L),
        origin=Origin(xyz=(LEFT_HINGE_X, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="left_lid_knuckle",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=HINGE_SEGMENT_L),
        origin=Origin(xyz=(RIGHT_HINGE_X, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="right_lid_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, 0.018)),
        mass=1.2,
        origin=Origin(xyz=LID_CENTER),
    )

    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    rear_lid_hinge = object_model.get_articulation("rear_lid_hinge")

    bottom_plate = body.get_visual("bottom_plate")
    glass_platen = body.get_visual("glass_platen")
    left_body_outer_knuckle = body.get_visual("left_body_outer_knuckle")
    left_body_inner_knuckle = body.get_visual("left_body_inner_knuckle")
    right_body_inner_knuckle = body.get_visual("right_body_inner_knuckle")
    right_body_outer_knuckle = body.get_visual("right_body_outer_knuckle")

    lid_panel = lid.get_visual("lid_panel")
    lid_backing = lid.get_visual("lid_backing")
    lid_front_lip = lid.get_visual("lid_front_lip")
    left_lid_knuckle = lid.get_visual("left_lid_knuckle")
    right_lid_knuckle = lid.get_visual("right_lid_knuckle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48, name="rear_lid_hinge_clearance")

    hinge_axis_ok = all(
        math.isclose(actual, expected, abs_tol=1e-9)
        for actual, expected in zip(rear_lid_hinge.axis, (-1.0, 0.0, 0.0))
    )
    ctx.check("rear_lid_hinge_axis", hinge_axis_ok, f"axis was {rear_lid_hinge.axis!r}")

    limits = rear_lid_hinge.motion_limits
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and math.isclose(limits.lower, 0.0, abs_tol=1e-9)
        and 1.20 <= limits.upper <= 1.40
    )
    ctx.check("rear_lid_hinge_limits", limits_ok, f"limits were {limits!r}")

    ctx.expect_within(
        body,
        body,
        axes="xy",
        inner_elem=glass_platen,
        outer_elem=bottom_plate,
        name="glass_within_body_plan",
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        positive_elem=glass_platen,
        negative_elem=bottom_plate,
        min_gap=0.043,
        max_gap=0.045,
        name="glass_sits_on_top_face",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({rear_lid_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_lid_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_lid_hinge_lower_no_floating")
            ctx.expect_overlap(
                lid,
                body,
                axes="xy",
                min_overlap=0.19,
                elem_a=lid_panel,
                elem_b=glass_platen,
                name="lid_covers_glass_when_closed",
            )
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem=lid_panel,
                negative_elem=glass_platen,
                min_gap=0.0008,
                max_gap=0.0015,
                name="closed_lid_panel_clearance",
            )
            ctx.expect_contact(
                lid,
                body,
                elem_a=lid_backing,
                elem_b=glass_platen,
                contact_tol=0.0001,
                name="lid_backing_meets_platen",
            )
            ctx.expect_contact(
                lid,
                body,
                elem_a=left_lid_knuckle,
                elem_b=left_body_outer_knuckle,
                contact_tol=0.0001,
                name="left_hinge_outer_knuckle_contact",
            )
            ctx.expect_contact(
                lid,
                body,
                elem_a=left_lid_knuckle,
                elem_b=left_body_inner_knuckle,
                contact_tol=0.0001,
                name="left_hinge_inner_knuckle_contact",
            )
            ctx.expect_contact(
                lid,
                body,
                elem_a=right_lid_knuckle,
                elem_b=right_body_inner_knuckle,
                contact_tol=0.0001,
                name="right_hinge_inner_knuckle_contact",
            )
            ctx.expect_contact(
                lid,
                body,
                elem_a=right_lid_knuckle,
                elem_b=right_body_outer_knuckle,
                contact_tol=0.0001,
                name="right_hinge_outer_knuckle_contact",
            )

        with ctx.pose({rear_lid_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_lid_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_lid_hinge_upper_no_floating")
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem=lid_front_lip,
                negative_elem=glass_platen,
                min_gap=0.19,
                name="front_edge_lifts_clear_of_platen",
            )
            ctx.expect_contact(
                lid,
                body,
                elem_a=left_lid_knuckle,
                elem_b=left_body_outer_knuckle,
                contact_tol=0.0001,
                name="left_hinge_outer_contact_open",
            )
            ctx.expect_contact(
                lid,
                body,
                elem_a=right_lid_knuckle,
                elem_b=right_body_outer_knuckle,
                contact_tol=0.0001,
                name="right_hinge_outer_contact_open",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
