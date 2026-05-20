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
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.44
BODY_DEPTH = 0.18
BODY_HEIGHT = 0.22
BODY_CENTER_Z = 0.12
BODY_FRONT_Y = -BODY_DEPTH / 2.0
PIVOT_Z = 0.235
PIVOT_DISK_X = BODY_WIDTH / 2.0 + 0.022
CONTROL_FACE_Y = BODY_FRONT_Y - 0.030
CONTROL_Z = 0.212


def _rounded_box(width: float, depth: float, height: float, radius: float) -> object:
    """CadQuery rounded box centered on the local origin."""
    return (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges()
        .fillet(radius)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="battery_outdoor_speaker")

    charcoal = model.material("charcoal_rubber", rgba=(0.015, 0.017, 0.018, 1.0))
    dark_teal = model.material("dark_teal_shell", rgba=(0.035, 0.115, 0.120, 1.0))
    grille_black = model.material("black_perforated_metal", rgba=(0.005, 0.006, 0.007, 1.0))
    warm_gray = model.material("warm_gray_plastic", rgba=(0.18, 0.18, 0.17, 1.0))
    pale_mark = model.material("white_control_mark", rgba=(0.90, 0.88, 0.78, 1.0))
    amber = model.material("amber_led", rgba=(1.0, 0.56, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(
            _rounded_box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, 0.030),
            "rounded_speaker_body",
            tolerance=0.001,
            angular_tolerance=0.12,
        ),
        origin=Origin(xyz=(0.0, 0.0, BODY_CENTER_Z)),
        material=dark_teal,
        name="shell",
    )

    body.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.365, 0.135),
                0.006,
                hole_diameter=0.0065,
                pitch=(0.013, 0.012),
                frame=0.010,
                corner_radius=0.012,
                stagger=True,
            ),
            "front_perforated_grille",
        ),
        origin=Origin(
            xyz=(0.0, BODY_FRONT_Y - 0.003, 0.120),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_black,
        name="front_grille",
    )

    body.visual(
        mesh_from_cadquery(
            _rounded_box(0.215, 0.030, 0.052, 0.008),
            "upper_control_pod",
            tolerance=0.0008,
            angular_tolerance=0.12,
        ),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.015, CONTROL_Z)),
        material=charcoal,
        name="control_pod",
    )

    # Small fixed indicator lights on the control pod reinforce the scale and
    # battery-powered speaker identity without adding secondary articulation.
    for i, x in enumerate((-0.086, -0.073, -0.060, 0.086)):
        body.visual(
            Cylinder(radius=0.0032, length=0.0025),
            origin=Origin(
                xyz=(x, CONTROL_FACE_Y - 0.0011, CONTROL_Z - 0.018),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=amber if i == 0 else pale_mark,
            name=f"status_light_{i}",
        )

    # Rugged rubber feet and side passive-radiator pads are fixed to the body.
    for i, x in enumerate((-0.135, 0.135)):
        body.visual(
            Box((0.105, 0.038, 0.016)),
            origin=Origin(xyz=(x, 0.018, 0.009)),
            material=charcoal,
            name=f"foot_{i}",
        )

    for i, x in enumerate((-BODY_WIDTH / 2.0 - 0.002, BODY_WIDTH / 2.0 + 0.002)):
        body.visual(
            Cylinder(radius=0.058, length=0.009),
            origin=Origin(
                xyz=(x, 0.0, BODY_CENTER_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=charcoal,
            name=f"side_radiator_{i}",
        )

    for x, pad_name in (
        (-BODY_WIDTH / 2.0 - 0.004, "pivot_pad_0"),
        (BODY_WIDTH / 2.0 + 0.004, "pivot_pad_1"),
    ):
        body.visual(
            Cylinder(radius=0.036, length=0.018),
            origin=Origin(
                xyz=(x, 0.0, PIVOT_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=warm_gray,
            name=pad_name,
        )

    top_bar = model.part("top_bar")
    for x, disk_name, arm_name in (
        (-PIVOT_DISK_X, "pivot_disk_0", "side_arm_0"),
        (PIVOT_DISK_X, "pivot_disk_1", "side_arm_1"),
    ):
        top_bar.visual(
            Cylinder(radius=0.030, length=0.018),
            origin=Origin(
                xyz=(x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=charcoal,
            name=disk_name,
        )
        top_bar.visual(
            Box((0.024, 0.044, 0.150)),
            origin=Origin(xyz=(math.copysign(PIVOT_DISK_X + 0.013, x), 0.0, 0.072)),
            material=charcoal,
            name=arm_name,
        )

    top_bar.visual(
        Box((0.535, 0.056, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=charcoal,
        name="crossbar",
    )
    top_bar.visual(
        Box((0.455, 0.038, 0.009)),
        origin=Origin(xyz=(0.0, -0.001, 0.169)),
        material=warm_gray,
        name="grip_pad",
    )

    model.articulation(
        "body_to_top_bar",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_bar,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.040,
            0.022,
            body_style="faceted",
            base_diameter=0.042,
            top_diameter=0.034,
            edge_radius=0.0008,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0008, width=0.0013),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "rotary_control_knob",
    )

    for i, x in enumerate((-0.068, 0.0, 0.068)):
        knob = model.part(f"knob_{i}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm_gray,
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=0.023, length=0.006),
            origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="knob_collar",
        )
        knob.visual(
            Box((0.004, 0.002, 0.018)),
            origin=Origin(xyz=(0.0, -0.0215, 0.006)),
            material=pale_mark,
            name="pointer_mark",
        )
        model.articulation(
            f"body_to_knob_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, CONTROL_FACE_Y, CONTROL_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0, lower=-2.35, upper=2.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    top_bar = object_model.get_part("top_bar")
    bar_joint = object_model.get_articulation("body_to_top_bar")

    ctx.check(
        "folding bar plus three rotary controls are articulated",
        len(object_model.articulations) == 4,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )

    ctx.expect_contact(
        body,
        top_bar,
        elem_a="pivot_pad_1",
        elem_b="pivot_disk_1",
        contact_tol=0.0015,
        name="top bar is carried on a side pivot",
    )

    rest_aabb = ctx.part_world_aabb(top_bar)
    with ctx.pose({bar_joint: 1.20}):
        folded_aabb = ctx.part_world_aabb(top_bar)
    ctx.check(
        "top bar folds forward and downward",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] < rest_aabb[1][2] - 0.035
        and folded_aabb[0][1] < rest_aabb[0][1] - 0.055,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    for i in range(3):
        knob = object_model.get_part(f"knob_{i}")
        joint = object_model.get_articulation(f"body_to_knob_{i}")
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            positive_elem="control_pod",
            negative_elem="knob_collar",
            max_gap=0.0015,
            max_penetration=0.0,
            name=f"knob_{i} collar seats on the pod face",
        )
        ctx.expect_overlap(
            knob,
            body,
            axes="xz",
            elem_a="knob_cap",
            elem_b="control_pod",
            min_overlap=0.030,
            name=f"knob_{i} shaft is centered on the control pod",
        )
        ctx.check(
            f"knob_{i} has realistic rotary travel",
            joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower < -2.0
            and joint.motion_limits.upper > 2.0,
            details=str(joint.motion_limits),
        )

    return ctx.report()


object_model = build_object_model()
