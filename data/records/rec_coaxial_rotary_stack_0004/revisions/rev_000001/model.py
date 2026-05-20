from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

PLINTH_RADIUS = 0.115
PLINTH_HEIGHT = 0.020

LOWER_SEAT_RADIUS = 0.058
LOWER_SEAT_HEIGHT = 0.008
LOWER_SEAT_TOP = PLINTH_HEIGHT + LOWER_SEAT_HEIGHT

MIDDLE_SEAT_RADIUS = 0.045
MIDDLE_SEAT_HEIGHT = 0.006
MIDDLE_SEAT_BOTTOM = 0.050
MIDDLE_SEAT_TOP = MIDDLE_SEAT_BOTTOM + MIDDLE_SEAT_HEIGHT

TOP_SEAT_RADIUS = 0.030
TOP_SEAT_HEIGHT = 0.006
TOP_SEAT_BOTTOM = 0.078
TOP_SEAT_TOP = TOP_SEAT_BOTTOM + TOP_SEAT_HEIGHT

SHAFT_RADIUS = 0.012
SHAFT_BOTTOM = PLINTH_HEIGHT
SHAFT_TOP = 0.108
SHAFT_HEIGHT = SHAFT_TOP - SHAFT_BOTTOM

BORE_RADIUS = 0.0135

BASE_STAGE_OUTER_RADIUS = 0.092
BASE_STAGE_HEIGHT = 0.012

MIDDLE_RING_OUTER_RADIUS = 0.067
MIDDLE_RING_HEIGHT = 0.010

TOP_FLANGE_OUTER_RADIUS = 0.047
TOP_FLANGE_HEIGHT = 0.008

LIMIT_120_DEG = math.radians(120.0)


def _make_base_stage() -> cq.Workplane:
    main_annulus = (
        cq.Workplane("XY")
        .circle(BASE_STAGE_OUTER_RADIUS)
        .circle(BORE_RADIUS)
        .extrude(0.008)
    )
    outer_rim = (
        cq.Workplane("XY")
        .circle(BASE_STAGE_OUTER_RADIUS)
        .circle(0.074)
        .extrude(BASE_STAGE_HEIGHT)
    )
    center_land = (
        cq.Workplane("XY")
        .circle(0.042)
        .circle(BORE_RADIUS)
        .extrude(0.010)
    )
    return main_annulus.union(outer_rim).union(center_land)


def _make_middle_ring() -> cq.Workplane:
    main_ring = (
        cq.Workplane("XY")
        .circle(MIDDLE_RING_OUTER_RADIUS)
        .circle(BORE_RADIUS)
        .extrude(0.006)
    )
    crown = (
        cq.Workplane("XY")
        .circle(0.058)
        .circle(0.022)
        .extrude(MIDDLE_RING_HEIGHT)
    )
    return main_ring.union(crown)


def _make_top_flange() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .circle(TOP_FLANGE_OUTER_RADIUS)
        .circle(BORE_RADIUS)
        .extrude(0.006)
    )
    raised_hub = (
        cq.Workplane("XY")
        .circle(0.026)
        .circle(BORE_RADIUS)
        .extrude(TOP_FLANGE_HEIGHT)
    )
    flange = flange.union(raised_hub)
    return (
        flange.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .polarArray(0.030, 0.0, 360.0, 3)
        .hole(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_rotary_stack", assets=ASSETS)

    dark_base = model.material("dark_base", rgba=(0.20, 0.21, 0.23, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    lower_stage_finish = model.material("lower_stage_finish", rgba=(0.54, 0.56, 0.60, 1.0))
    middle_stage_finish = model.material("middle_stage_finish", rgba=(0.30, 0.33, 0.37, 1.0))
    top_stage_finish = model.material("top_stage_finish", rgba=(0.73, 0.75, 0.78, 1.0))

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=PLINTH_RADIUS, length=PLINTH_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
        material=dark_base,
        name="plinth",
    )
    frame.visual(
        Cylinder(radius=LOWER_SEAT_RADIUS, length=LOWER_SEAT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + LOWER_SEAT_HEIGHT * 0.5)),
        material=dark_base,
        name="lower_seat",
    )
    frame.visual(
        Cylinder(radius=MIDDLE_SEAT_RADIUS, length=MIDDLE_SEAT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_SEAT_BOTTOM + MIDDLE_SEAT_HEIGHT * 0.5)),
        material=dark_base,
        name="middle_seat",
    )
    frame.visual(
        Cylinder(radius=TOP_SEAT_RADIUS, length=TOP_SEAT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, TOP_SEAT_BOTTOM + TOP_SEAT_HEIGHT * 0.5)),
        material=dark_base,
        name="top_seat",
    )
    frame.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_BOTTOM + SHAFT_HEIGHT * 0.5)),
        material=shaft_steel,
        name="shaft",
    )
    frame.inertial = Inertial.from_geometry(
        Box((2.0 * PLINTH_RADIUS, 2.0 * PLINTH_RADIUS, SHAFT_TOP)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_TOP * 0.5)),
    )

    base_stage = model.part("base_stage")
    base_stage.visual(
        mesh_from_cadquery(_make_base_stage(), "base_stage.obj", assets=ASSETS),
        material=lower_stage_finish,
        name="shell",
    )
    base_stage.inertial = Inertial.from_geometry(
        Box((2.0 * BASE_STAGE_OUTER_RADIUS, 2.0 * BASE_STAGE_OUTER_RADIUS, BASE_STAGE_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, BASE_STAGE_HEIGHT * 0.5)),
    )

    middle_ring = model.part("middle_ring")
    middle_ring.visual(
        mesh_from_cadquery(_make_middle_ring(), "middle_ring.obj", assets=ASSETS),
        material=middle_stage_finish,
        name="shell",
    )
    middle_ring.inertial = Inertial.from_geometry(
        Box((2.0 * MIDDLE_RING_OUTER_RADIUS, 2.0 * MIDDLE_RING_OUTER_RADIUS, MIDDLE_RING_HEIGHT)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_RING_HEIGHT * 0.5)),
    )

    top_flange = model.part("top_flange")
    top_flange.visual(
        mesh_from_cadquery(_make_top_flange(), "top_flange.obj", assets=ASSETS),
        material=top_stage_finish,
        name="shell",
    )
    top_flange.inertial = Inertial.from_geometry(
        Box((2.0 * TOP_FLANGE_OUTER_RADIUS, 2.0 * TOP_FLANGE_OUTER_RADIUS, TOP_FLANGE_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, TOP_FLANGE_HEIGHT * 0.5)),
    )

    model.articulation(
        "frame_to_base_stage",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=base_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_SEAT_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
    )
    model.articulation(
        "frame_to_middle_ring",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=middle_ring,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_SEAT_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-LIMIT_120_DEG,
            upper=LIMIT_120_DEG,
        ),
    )
    model.articulation(
        "frame_to_top_flange",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=top_flange,
        origin=Origin(xyz=(0.0, 0.0, TOP_SEAT_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=9.0,
            velocity=2.0,
            lower=-LIMIT_120_DEG,
            upper=LIMIT_120_DEG,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    base_stage = object_model.get_part("base_stage")
    middle_ring = object_model.get_part("middle_ring")
    top_flange = object_model.get_part("top_flange")

    base_joint = object_model.get_articulation("frame_to_base_stage")
    middle_joint = object_model.get_articulation("frame_to_middle_ring")
    top_joint = object_model.get_articulation("frame_to_top_flange")

    frame_lower_seat = frame.get_visual("lower_seat")
    frame_middle_seat = frame.get_visual("middle_seat")
    frame_top_seat = frame.get_visual("top_seat")
    base_shell = base_stage.get_visual("shell")
    middle_shell = middle_ring.get_visual("shell")
    top_shell = top_flange.get_visual("shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all expected parts present",
        all(part is not None for part in (frame, base_stage, middle_ring, top_flange)),
        "missing one or more named parts in the coaxial rotary stack",
    )
    ctx.check(
        "all rotary joints use vertical axis",
        base_joint.axis == (0.0, 0.0, 1.0)
        and middle_joint.axis == (0.0, 0.0, 1.0)
        and top_joint.axis == (0.0, 0.0, 1.0),
        f"joint axes were {base_joint.axis}, {middle_joint.axis}, {top_joint.axis}",
    )
    ctx.check(
        "base stage is continuous",
        base_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"expected CONTINUOUS, got {base_joint.articulation_type}",
    )
    ctx.check(
        "upper joints are limited to about one hundred twenty degrees each way",
        middle_joint.articulation_type == ArticulationType.REVOLUTE
        and top_joint.articulation_type == ArticulationType.REVOLUTE
        and math.isclose(middle_joint.motion_limits.lower, -LIMIT_120_DEG, abs_tol=1e-6)
        and math.isclose(middle_joint.motion_limits.upper, LIMIT_120_DEG, abs_tol=1e-6)
        and math.isclose(top_joint.motion_limits.lower, -LIMIT_120_DEG, abs_tol=1e-6)
        and math.isclose(top_joint.motion_limits.upper, LIMIT_120_DEG, abs_tol=1e-6),
        "middle or top joint limits do not match the requested ±120° sweep",
    )

    ctx.expect_origin_distance(base_stage, frame, axes="xy", max_dist=1e-6)
    ctx.expect_origin_distance(middle_ring, frame, axes="xy", max_dist=1e-6)
    ctx.expect_origin_distance(top_flange, frame, axes="xy", max_dist=1e-6)

    ctx.expect_gap(
        base_stage,
        frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=base_shell,
        negative_elem=frame_lower_seat,
        name="base stage seats on lower bearing face",
    )
    ctx.expect_overlap(
        base_stage,
        frame,
        axes="xy",
        min_overlap=0.040,
        elem_a=base_shell,
        elem_b=frame_lower_seat,
        name="base stage footprint overlaps lower seat",
    )
    ctx.expect_contact(
        base_stage,
        frame,
        elem_a=base_shell,
        elem_b=frame_lower_seat,
        name="base stage contacts lower seat",
    )

    ctx.expect_gap(
        middle_ring,
        frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=1e-6,
        positive_elem=middle_shell,
        negative_elem=frame_middle_seat,
        name="middle ring seats on middle bearing face",
    )
    ctx.expect_overlap(
        middle_ring,
        frame,
        axes="xy",
        min_overlap=0.028,
        elem_a=middle_shell,
        elem_b=frame_middle_seat,
        name="middle ring footprint overlaps middle seat",
    )
    ctx.expect_contact(
        middle_ring,
        frame,
        elem_a=middle_shell,
        elem_b=frame_middle_seat,
        name="middle ring contacts middle seat",
    )

    ctx.expect_gap(
        top_flange,
        frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=top_shell,
        negative_elem=frame_top_seat,
        name="top flange seats on upper bearing face",
    )
    ctx.expect_overlap(
        top_flange,
        frame,
        axes="xy",
        min_overlap=0.015,
        elem_a=top_shell,
        elem_b=frame_top_seat,
        name="top flange footprint overlaps top seat",
    )
    ctx.expect_contact(
        top_flange,
        frame,
        elem_a=top_shell,
        elem_b=frame_top_seat,
        name="top flange contacts top seat",
    )

    ctx.expect_gap(
        middle_ring,
        base_stage,
        axis="z",
        min_gap=0.010,
        positive_elem=middle_shell,
        negative_elem=base_shell,
        name="middle ring stays above base stage",
    )
    ctx.expect_gap(
        top_flange,
        middle_ring,
        axis="z",
        min_gap=0.010,
        positive_elem=top_shell,
        negative_elem=middle_shell,
        name="top flange stays above middle ring",
    )

    with ctx.pose({base_joint: 1.75, middle_joint: LIMIT_120_DEG, top_joint: -LIMIT_120_DEG}):
        ctx.expect_contact(
            base_stage,
            frame,
            elem_a=base_shell,
            elem_b=frame_lower_seat,
            name="base stage remains mounted at spun pose",
        )
        ctx.expect_contact(
            middle_ring,
            frame,
            elem_a=middle_shell,
            elem_b=frame_middle_seat,
            name="middle ring remains mounted at positive limit",
        )
        ctx.expect_contact(
            top_flange,
            frame,
            elem_a=top_shell,
            elem_b=frame_top_seat,
            name="top flange remains mounted at negative limit",
        )
        ctx.expect_gap(
            middle_ring,
            base_stage,
            axis="z",
            min_gap=0.010,
            positive_elem=middle_shell,
            negative_elem=base_shell,
            name="middle ring clears base stage at posed limits",
        )
        ctx.expect_gap(
            top_flange,
            middle_ring,
            axis="z",
            min_gap=0.010,
            positive_elem=top_shell,
            negative_elem=middle_shell,
            name="top flange clears middle ring at posed limits",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
