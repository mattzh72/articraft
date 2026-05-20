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
    mesh_from_geometry,
    tube_from_spline_points,
)


CROWN_Z = 0.718
LEG_HINGE_RADIUS = 0.064
PAN_Z = 1.460
YOKE_ARM_INNER_Y = 0.116
HEAD_TILT_Z = 0.160


def _lamp_head_shape() -> cq.Workplane:
    shell = cq.Workplane("YZ").workplane(offset=-0.050).circle(0.102).extrude(0.210)
    cavity = cq.Workplane("YZ").workplane(offset=-0.005).circle(0.089).extrude(0.165)
    bezel = cq.Workplane("YZ").workplane(offset=0.135).circle(0.113).extrude(0.035)
    rear_cap = cq.Workplane("YZ").workplane(offset=-0.088).circle(0.072).extrude(0.048)
    return shell.cut(cavity).union(bezel).union(rear_cap).clean().val()


def _element_center(ctx: TestContext, part_name: str, elem_name: str) -> tuple[float, float, float] | None:
    bounds = ctx.part_element_world_aabb(part_name, elem=elem_name)
    if bounds is None:
        return None
    lower, upper = bounds
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_tripod")

    powder_black = model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.24, 0.24, 0.26, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.08, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.52, 0.58, 0.66, 0.92))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.052, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, CROWN_Z)),
        material=powder_black,
        name="crown_hub",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.760)),
        material=powder_black,
        name="crown_collar",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.675),
        origin=Origin(xyz=(0.0, 0.0, 1.0925)),
        material=powder_black,
        name="mast",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 1.435)),
        material=powder_black,
        name="top_socket",
    )
    stand.visual(
        Cylinder(radius=0.046, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.450)),
        material=satin_graphite,
        name="top_plate",
    )
    stand.visual(
        Cylinder(radius=0.0045, length=0.026),
        origin=Origin(xyz=(0.039, 0.0, 0.760), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_graphite,
        name="clamp_stem",
    )
    stand.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.055, 0.0, 0.760), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="clamp_knob",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        tangent = (-s, c, 0.0)
        hinge_center = (LEG_HINGE_RADIUS * c, LEG_HINGE_RADIUS * s, CROWN_Z)
        for ear_suffix, sign in (("a", -1.0), ("b", 1.0)):
            stand.visual(
                Box((0.022, 0.010, 0.024)),
                origin=Origin(
                    xyz=(
                        hinge_center[0] - 0.0045 * c + sign * 0.016 * tangent[0],
                        hinge_center[1] - 0.0045 * s + sign * 0.016 * tangent[1],
                        hinge_center[2],
                    ),
                    rpy=(0.0, 0.0, angle),
                ),
                material=powder_black,
                name=f"leg_{index}_ear_{ear_suffix}",
            )

    leg_points = [
        (0.006, 0.0, -0.004),
        (0.100, 0.0, -0.068),
        (0.235, 0.0, -0.235),
        (0.362, 0.0, -0.470),
        (0.438, 0.0, -0.698),
    ]

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{index}")
        leg.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    leg_points,
                    radius=0.011,
                    samples_per_segment=20,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"tripod_leg_{index}",
            ),
            material=powder_black,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.018, length=0.024),
            origin=Origin(
                xyz=(0.438, 0.0, -0.698),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rubber_black,
            name="foot_tip",
        )

        model.articulation(
            f"stand_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=leg,
            origin=Origin(xyz=(LEG_HINGE_RADIUS * math.cos(angle), LEG_HINGE_RADIUS * math.sin(angle), CROWN_Z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=1.8,
                lower=0.0,
                upper=2.35,
            ),
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.034, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_graphite,
        name="pan_collar",
    )
    yoke.visual(
        Box((0.064, 0.300, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=powder_black,
        name="yoke_base",
    )
    yoke.visual(
        Box((0.036, 0.032, 0.238)),
        origin=Origin(xyz=(0.0, -0.132, 0.129)),
        material=powder_black,
        name="arm_0",
    )
    yoke.visual(
        Box((0.036, 0.032, 0.238)),
        origin=Origin(xyz=(0.0, 0.132, 0.129)),
        material=powder_black,
        name="arm_1",
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        mesh_from_cadquery(_lamp_head_shape(), "spotlight_head"),
        material=satin_graphite,
        name="head_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.010, length=2.0 * YOKE_ARM_INNER_Y),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="trunnion_shaft",
    )
    lamp_head.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(-0.005, -0.098, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="trunnion_boss_0",
    )
    lamp_head.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(-0.005, 0.098, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="trunnion_boss_1",
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, HEAD_TILT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.45,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    lamp_head = object_model.get_part("lamp_head")
    leg_0 = object_model.get_part("leg_0")

    pan = object_model.get_articulation("stand_to_yoke")
    tilt = object_model.get_articulation("yoke_to_head")
    leg_fold = object_model.get_articulation("stand_to_leg_0")

    ctx.allow_overlap(
        "leg_1",
        "stand",
        elem_a="leg_tube",
        elem_b="leg_1_ear_b",
        reason="The deployed tripod leg is simplified to nest slightly into the crown clevis at the hinge.",
    )
    ctx.allow_overlap(
        "leg_2",
        "stand",
        elem_a="leg_tube",
        elem_b="leg_2_ear_a",
        reason="The deployed tripod leg is simplified to nest slightly into the crown clevis at the hinge.",
    )

    ctx.expect_contact(
        yoke,
        stand,
        elem_a="yoke_base",
        elem_b="top_plate",
        contact_tol=1e-4,
        name="yoke base seats on mast top",
    )
    ctx.expect_contact(
        lamp_head,
        yoke,
        elem_a="trunnion_shaft",
        elem_b="arm_0",
        contact_tol=1e-4,
        name="lamp head bears on first yoke arm",
    )
    ctx.expect_contact(
        lamp_head,
        yoke,
        elem_a="trunnion_shaft",
        elem_b="arm_1",
        contact_tol=1e-4,
        name="lamp head bears on second yoke arm",
    )

    rest_bezel = _element_center(ctx, "lamp_head", "head_shell")
    with ctx.pose({pan: 0.75}):
        panned_bezel = _element_center(ctx, "lamp_head", "head_shell")
    ctx.check(
        "yoke pan sweeps the lamp head around the mast",
        rest_bezel is not None
        and panned_bezel is not None
        and abs(panned_bezel[1]) > abs(rest_bezel[1]) + 0.02,
        details=f"rest_bezel={rest_bezel}, panned_bezel={panned_bezel}",
    )

    with ctx.pose({tilt: 1.0}):
        tilted_bezel = _element_center(ctx, "lamp_head", "head_shell")
    ctx.check(
        "lamp head tilt raises the lens",
        rest_bezel is not None
        and tilted_bezel is not None
        and tilted_bezel[2] > rest_bezel[2] + 0.03,
        details=f"rest_bezel={rest_bezel}, tilted_bezel={tilted_bezel}",
    )

    rest_foot = _element_center(ctx, "leg_0", "foot_tip")
    with ctx.pose({leg_fold: 2.1}):
        folded_foot = _element_center(ctx, "leg_0", "foot_tip")
    ctx.check(
        "tripod leg folds upward toward transport",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[2] > rest_foot[2] + 0.45,
        details=f"rest_foot={rest_foot}, folded_foot={folded_foot}",
    )

    return ctx.report()


object_model = build_object_model()
