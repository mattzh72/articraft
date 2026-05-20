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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_face,
)


BODY_W = 0.68
BODY_D = 0.44
BODY_H = 0.39
BEZEL_D = 0.055

OUTLET_W = 0.42
OUTLET_H = 0.105
OUTLET_X = -0.09
OUTLET_Z = 0.11
OUTLET_FACE_Y = BODY_D / 2.0 + BEZEL_D - 0.0025

INTAKE_W = 0.46
INTAKE_H = 0.20
INTAKE_X = -0.08
INTAKE_Z = -0.09
INTAKE_FACE_Y = BODY_D / 2.0 + BEZEL_D - 0.002

POD_W = 0.14
POD_D = 0.055
POD_H = 0.19
POD_X = 0.24
POD_Z = 0.03

SHAFT_RADIUS = 0.0045
SHAFT_LEN = 0.006
SHAFT_GAP = 0.002

FLAP_W = 0.396
FLAP_H = 0.080
FLAP_THICKNESS = 0.008
FLAP_BARREL_RADIUS = 0.006
FLAP_HINGE_Y = BODY_D / 2.0 + BEZEL_D + 0.007
FLAP_HINGE_Z = OUTLET_Z - OUTLET_H / 2.0 + 0.010


def _make_knob_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.024,
            body_style="skirted",
            top_diameter=0.037,
            skirt=KnobSkirt(0.052, 0.0055, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_air_conditioner")

    body_paint = model.material("body_paint", rgba=(0.88, 0.88, 0.84, 1.0))
    fascia = model.material("fascia", rgba=(0.82, 0.83, 0.80, 1.0))
    dark_grille = model.material("dark_grille", rgba=(0.27, 0.30, 0.33, 1.0))
    control_finish = model.material("control_finish", rgba=(0.19, 0.20, 0.21, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.14, 0.14, 0.15, 1.0))
    shaft_finish = model.material("shaft_finish", rgba=(0.52, 0.53, 0.55, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BODY_H)),
        material=body_paint,
        name="cabinet",
    )
    body.visual(
        Box((BODY_W + 0.010, BEZEL_D, BODY_H + 0.010)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 + BEZEL_D / 2.0, 0.0)),
        material=fascia,
        name="front_bezel",
    )
    body.visual(
        Box((OUTLET_W + 0.030, 0.008, OUTLET_H + 0.030)),
        origin=Origin(xyz=(OUTLET_X, BODY_D / 2.0 + BEZEL_D - 0.004, OUTLET_Z)),
        material=fascia,
        name="outlet_surround",
    )
    body.visual(
        Box((OUTLET_W + 0.020, 0.006, 0.014)),
        origin=Origin(xyz=(OUTLET_X, BODY_D / 2.0 + BEZEL_D - 0.002, FLAP_HINGE_Z)),
        material=fascia,
        name="flap_support",
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (OUTLET_W, OUTLET_H),
                0.005,
                slot_size=(0.078, 0.019),
                pitch=(0.031, 0.094),
                frame=0.010,
                corner_radius=0.004,
                slot_angle_deg=89.0,
            ),
            "outlet_grille",
        ),
        origin=Origin(xyz=(OUTLET_X, OUTLET_FACE_Y, OUTLET_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grille,
        name="outlet_grille",
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (INTAKE_W, INTAKE_H),
                0.004,
                slot_size=(0.040, 0.006),
                pitch=(0.050, 0.016),
                frame=0.014,
                corner_radius=0.004,
            ),
            "intake_grille",
        ),
        origin=Origin(xyz=(INTAKE_X, INTAKE_FACE_Y, INTAKE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grille,
        name="intake_grille",
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((POD_W, POD_D, POD_H)),
        material=control_finish,
        name="pod_shell",
    )
    model.articulation(
        "body_to_control_pod",
        ArticulationType.FIXED,
        parent=body,
        child=control_pod,
        origin=Origin(xyz=(POD_X, BODY_D / 2.0 + BEZEL_D + POD_D / 2.0, POD_Z)),
    )

    knob_origins = (
        place_on_face(control_pod, "+y", face_pos=(0.0, 0.045), proud=SHAFT_LEN),
        place_on_face(control_pod, "+y", face_pos=(0.0, -0.045), proud=SHAFT_LEN),
    )

    control_pod.visual(
        Box((POD_W * 0.84, 0.003, POD_H * 0.88)),
        origin=Origin(xyz=(0.0, POD_D / 2.0 + 0.0015, 0.0)),
        material=fascia,
        name="pod_face",
    )

    for index, z_offset in enumerate((0.045, -0.045)):
        control_pod.visual(
            Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LEN),
            origin=Origin(
                xyz=(0.0, POD_D / 2.0 + SHAFT_LEN / 2.0, z_offset),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=shaft_finish,
            name=f"shaft_{index}",
        )

    knob_mesh = _make_knob_mesh("control_knob")
    for index, origin in enumerate(knob_origins):
        knob = model.part(f"knob_{index}")
        knob.visual(
            knob_mesh,
            material=knob_finish,
            name="knob_shell",
        )
        model.articulation(
            f"control_pod_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=control_pod,
            child=knob,
            origin=origin,
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.20, velocity=8.0),
        )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=FLAP_BARREL_RADIUS, length=FLAP_W),
        origin=Origin(xyz=(0.0, 0.0, FLAP_BARREL_RADIUS), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fascia,
        name="flap_barrel",
    )
    flap.visual(
        Box((FLAP_W, FLAP_THICKNESS, FLAP_H)),
        origin=Origin(xyz=(0.0, FLAP_THICKNESS / 2.0, FLAP_H / 2.0)),
        material=fascia,
        name="flap_blade",
    )
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(OUTLET_X, FLAP_HINGE_Y, FLAP_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    control_pod = object_model.get_part("control_pod")
    flap = object_model.get_part("flap")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    flap_hinge = object_model.get_articulation("body_to_flap")
    knob_joint_0 = object_model.get_articulation("control_pod_to_knob_0")
    knob_joint_1 = object_model.get_articulation("control_pod_to_knob_1")

    ctx.expect_gap(
        control_pod,
        body,
        axis="y",
        positive_elem="pod_shell",
        negative_elem="front_bezel",
        min_gap=0.0,
        max_gap=0.001,
        name="control pod seats flush on the front bezel",
    )
    ctx.expect_overlap(
        control_pod,
        body,
        axes="xz",
        elem_a="pod_shell",
        elem_b="front_bezel",
        min_overlap=0.12,
        name="control pod overlaps the right side of the bezel footprint",
    )

    for index, knob in enumerate((knob_0, knob_1)):
        ctx.expect_gap(
            knob,
            control_pod,
            axis="y",
            positive_elem="knob_shell",
            negative_elem="pod_shell",
            min_gap=0.0055,
            max_gap=0.0065,
            name=f"knob_{index} stands off from the pod face",
        )
        ctx.expect_gap(
            knob,
            control_pod,
            axis="y",
            positive_elem="knob_shell",
            negative_elem=f"shaft_{index}",
            min_gap=0.0,
            max_gap=0.0005,
            name=f"knob_{index} seats on its exposed shaft tip",
        )
        ctx.expect_overlap(
            knob,
            control_pod,
            axes="xz",
            elem_a="knob_shell",
            elem_b="pod_shell",
            min_overlap=0.035,
            name=f"knob_{index} stays centered on the control pod face",
        )

    ctx.expect_gap(
        flap,
        body,
        axis="y",
        positive_elem="flap_blade",
        negative_elem="outlet_grille",
        min_gap=0.006,
        max_gap=0.014,
        name="flap rests slightly in front of the outlet grille",
    )
    ctx.expect_overlap(
        flap,
        body,
        axes="x",
        elem_a="flap_blade",
        elem_b="outlet_grille",
        min_overlap=0.36,
        name="flap spans almost the full outlet width",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_blade")
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_blade")
    flap_opens_forward = (
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and float(open_flap_aabb[1][1]) > float(closed_flap_aabb[1][1]) + 0.05
    )
    ctx.check(
        "flap opens forward from the lower hinge",
        flap_opens_forward,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    for joint in (knob_joint_0, knob_joint_1):
        limits = joint.motion_limits
        is_continuous = (
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
        )
        ctx.check(
            f"{joint.name} is continuous",
            is_continuous,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
