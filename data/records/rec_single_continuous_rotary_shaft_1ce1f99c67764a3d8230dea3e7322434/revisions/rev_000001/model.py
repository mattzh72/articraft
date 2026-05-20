from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_LENGTH = 0.52
BASE_WIDTH = 0.20
BASE_THICKNESS = 0.022
AXIS_HEIGHT = 0.165
BASE_CENTER_Z = -AXIS_HEIGHT + (BASE_THICKNESS / 2.0)

SUPPORT_CENTER_OFFSET = 0.14
SUPPORT_LENGTH = 0.065
SUPPORT_FOOT_WIDTH = 0.16
SUPPORT_FOOT_THICKNESS = 0.022
SUPPORT_FOOT_CENTER_Z = -0.136
SUPPORT_COLUMN_WIDTH = 0.10
SUPPORT_COLUMN_HEIGHT = 0.118
SUPPORT_COLUMN_CENTER_Z = -0.077
SUPPORT_HOUSING_RADIUS = 0.056

SHAFT_RADIUS = 0.022
SHAFT_LENGTH = 0.51
SUPPORT_BORE_RADIUS = 0.0235
RETAINER_RADIUS = 0.034
RETAINER_THICKNESS = 0.012
LEFT_SUPPORT_OUTER_X = -SUPPORT_CENTER_OFFSET - (SUPPORT_LENGTH / 2.0)
RIGHT_SUPPORT_OUTER_X = SUPPORT_CENTER_OFFSET + (SUPPORT_LENGTH / 2.0)
LEFT_RETAINER_CENTER_X = LEFT_SUPPORT_OUTER_X - (RETAINER_THICKNESS / 2.0)
RIGHT_RETAINER_CENTER_X = RIGHT_SUPPORT_OUTER_X + (RETAINER_THICKNESS / 2.0)
FLANGE_RADIUS = 0.055
FLANGE_THICKNESS = 0.018
FLANGE_CENTER_X = (SHAFT_LENGTH / 2.0) - (FLANGE_THICKNESS / 2.0)


def _build_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
    base = (
        base.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.205, -0.070),
                (-0.205, 0.070),
                (0.205, -0.070),
                (0.205, 0.070),
            ]
        )
        .hole(0.014)
    )
    return base.edges("|Z").fillet(0.006)


def _build_support_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("YZ")
        .center(0.0, SUPPORT_FOOT_CENTER_Z)
        .rect(SUPPORT_FOOT_WIDTH, SUPPORT_FOOT_THICKNESS)
        .extrude(SUPPORT_LENGTH / 2.0, both=True)
    )
    column = (
        cq.Workplane("YZ")
        .center(0.0, SUPPORT_COLUMN_CENTER_Z)
        .rect(SUPPORT_COLUMN_WIDTH, SUPPORT_COLUMN_HEIGHT)
        .extrude(SUPPORT_LENGTH / 2.0, both=True)
    )
    housing = (
        cq.Workplane("YZ")
        .circle(SUPPORT_HOUSING_RADIUS)
        .extrude(SUPPORT_LENGTH / 2.0, both=True)
    )
    bore = (
        cq.Workplane("YZ")
        .circle(SUPPORT_BORE_RADIUS)
        .extrude((SUPPORT_LENGTH / 2.0) + 0.010, both=True)
    )
    return foot.union(column).union(housing).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_spindle_shaft_module")

    model.material("frame_paint", rgba=(0.24, 0.28, 0.30, 1.0))
    model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    frame = model.part("pedestal_frame")
    frame.visual(
        mesh_from_cadquery(_build_base_shape(), "frame_base"),
        origin=Origin(xyz=(0.0, 0.0, BASE_CENTER_Z)),
        material="frame_paint",
        name="frame_base",
    )
    frame.visual(
        mesh_from_cadquery(_build_support_shape(), "left_support"),
        origin=Origin(xyz=(-SUPPORT_CENTER_OFFSET, 0.0, 0.0)),
        material="frame_paint",
        name="left_support",
    )
    frame.visual(
        mesh_from_cadquery(_build_support_shape(), "right_support"),
        origin=Origin(xyz=(SUPPORT_CENTER_OFFSET, 0.0, 0.0)),
        material="frame_paint",
        name="right_support",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.19)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="shaft_body",
    )
    spindle.visual(
        Cylinder(radius=RETAINER_RADIUS, length=RETAINER_THICKNESS),
        origin=Origin(
            xyz=(LEFT_RETAINER_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="machined_steel",
        name="left_retainer",
    )
    spindle.visual(
        Cylinder(radius=RETAINER_RADIUS, length=RETAINER_THICKNESS),
        origin=Origin(
            xyz=(RIGHT_RETAINER_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="machined_steel",
        name="right_retainer",
    )
    spindle.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_THICKNESS),
        origin=Origin(xyz=(FLANGE_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="end_flange",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        mass=8.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("pedestal_frame")
    spindle = object_model.get_part("spindle")
    shaft_body = spindle.get_visual("shaft_body")
    left_retainer = spindle.get_visual("left_retainer")
    right_retainer = spindle.get_visual("right_retainer")
    end_flange = spindle.get_visual("end_flange")
    left_support = frame.get_visual("left_support")
    right_support = frame.get_visual("right_support")
    spin_joint = object_model.get_articulation("frame_to_spindle")

    ctx.check(
        "spindle joint is continuous about the shaft axis",
        spin_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin_joint.axis) == (1.0, 0.0, 0.0)
        and spin_joint.motion_limits is not None
        and spin_joint.motion_limits.lower is None
        and spin_joint.motion_limits.upper is None,
        details=(
            f"type={spin_joint.articulation_type}, axis={spin_joint.axis}, "
            f"limits={spin_joint.motion_limits}"
        ),
    )

    ctx.expect_origin_distance(
        frame,
        spindle,
        axes="yz",
        min_dist=0.0,
        max_dist=1e-6,
        name="spindle axis stays centered in the pedestal frame",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a=shaft_body,
        elem_b=left_support,
        min_overlap=SUPPORT_LENGTH - 0.005,
        name="shaft spans the left support bearing block",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a=shaft_body,
        elem_b=right_support,
        min_overlap=SUPPORT_LENGTH - 0.005,
        name="shaft spans the right support bearing block",
    )
    ctx.expect_contact(
        spindle,
        frame,
        elem_a=left_retainer,
        elem_b=left_support,
        contact_tol=5e-4,
        name="left spindle retainer seats against the left support",
    )
    ctx.expect_contact(
        spindle,
        frame,
        elem_a=right_retainer,
        elem_b=right_support,
        contact_tol=5e-4,
        name="right spindle retainer seats against the right support",
    )
    ctx.expect_gap(
        spindle,
        frame,
        axis="x",
        positive_elem=end_flange,
        negative_elem=right_support,
        min_gap=0.045,
        name="plain end flange projects beyond the outer support",
    )

    with ctx.pose({spin_joint: 1.7}):
        ctx.expect_gap(
            spindle,
            frame,
            axis="x",
            positive_elem=end_flange,
            negative_elem=right_support,
            min_gap=0.045,
            name="flange clearance is preserved while the spindle spins",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
