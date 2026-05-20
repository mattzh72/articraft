from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_PLINTH_RADIUS = 0.140
BASE_PLINTH_HEIGHT = 0.040

LOWER_SUPPORT_RADIUS = 0.085
LOWER_SUPPORT_HEIGHT = 0.015
LOWER_STAGE_Z = BASE_PLINTH_HEIGHT + LOWER_SUPPORT_HEIGHT

LOWER_COLUMN_RADIUS = 0.055
LOWER_COLUMN_HEIGHT = 0.050

MIDDLE_SUPPORT_RADIUS = 0.058
MIDDLE_SUPPORT_HEIGHT = 0.012
MIDDLE_STAGE_Z = LOWER_STAGE_Z + LOWER_COLUMN_HEIGHT
MIDDLE_SUPPORT_BOTTOM = MIDDLE_STAGE_Z - MIDDLE_SUPPORT_HEIGHT

UPPER_COLUMN_RADIUS = 0.034
UPPER_COLUMN_HEIGHT = 0.050

TOP_SUPPORT_RADIUS = 0.040
TOP_SUPPORT_HEIGHT = 0.012
TOP_STAGE_Z = MIDDLE_STAGE_Z + UPPER_COLUMN_HEIGHT
TOP_SUPPORT_BOTTOM = TOP_STAGE_Z - TOP_SUPPORT_HEIGHT

SHAFT_RADIUS = 0.017
SHAFT_BOTTOM = BASE_PLINTH_HEIGHT
SHAFT_TOP = 0.195
SHAFT_HEIGHT = SHAFT_TOP - SHAFT_BOTTOM

BASE_STAGE_INNER_RADIUS = 0.060
MIDDLE_STAGE_INNER_RADIUS = 0.040
TOP_STAGE_BORE_RADIUS = 0.022


def _annulus(outer_radius: float, inner_radius: float, height: float, z0: float = 0.0):
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _polar_points(count: int, radius: float) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / count),
            radius * math.sin((2.0 * math.pi * index) / count),
        )
        for index in range(count)
    ]


def _cut_holes(
    shape,
    *,
    points: list[tuple[float, float]],
    hole_radius: float,
    height: float,
    z0: float = -0.001,
):
    result = shape
    for x_pos, y_pos in points:
        cutter = (
            cq.Workplane("XY", origin=(x_pos, y_pos, z0))
            .circle(hole_radius)
            .extrude(height + 0.002)
        )
        result = result.cut(cutter)
    return result


def _make_base_turntable():
    plate_thickness = 0.018
    hub_height = 0.014

    shape = _annulus(0.120, BASE_STAGE_INNER_RADIUS, plate_thickness)
    hub = _annulus(0.085, BASE_STAGE_INNER_RADIUS, hub_height, z0=plate_thickness)
    shape = shape.union(hub)
    shape = _cut_holes(
        shape,
        points=_polar_points(6, 0.090),
        hole_radius=0.0055,
        height=plate_thickness + hub_height,
    )
    return shape


def _make_middle_ring():
    body_thickness = 0.014
    collar_height = 0.013

    outer_ring = _annulus(0.095, 0.068, body_thickness)
    inner_ring = _annulus(0.060, MIDDLE_STAGE_INNER_RADIUS, body_thickness)

    spokes = None
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        spoke = (
            cq.Workplane("XY")
            .center(0.064, 0.0)
            .rect(0.018, 0.018)
            .extrude(body_thickness)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        spokes = spoke if spokes is None else spokes.union(spoke)

    collar = _annulus(0.060, MIDDLE_STAGE_INNER_RADIUS, collar_height, z0=body_thickness)
    shape = outer_ring.union(inner_ring).union(spokes).union(collar)
    shape = _cut_holes(
        shape,
        points=_polar_points(4, 0.082),
        hole_radius=0.0040,
        height=body_thickness + collar_height,
    )
    return shape


def _make_top_flange():
    flange_thickness = 0.012
    pilot_height = 0.012

    shape = cq.Workplane("XY").circle(0.062).extrude(flange_thickness)
    pilot = _annulus(0.038, TOP_STAGE_BORE_RADIUS, pilot_height, z0=flange_thickness)
    shape = shape.union(pilot)
    shape = _cut_holes(
        shape,
        points=_polar_points(6, 0.041),
        hole_radius=0.0045,
        height=flange_thickness + pilot_height,
    )
    bore = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .circle(TOP_STAGE_BORE_RADIUS)
        .extrude(flange_thickness + pilot_height + 0.002)
    )
    return shape.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_indexing_stack")

    base_dark = model.material("base_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.60, 0.64, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    machined = model.material("machined", rgba=(0.82, 0.84, 0.86, 1.0))

    fixed_base = model.part("fixed_base")
    fixed_base.visual(
        Cylinder(radius=BASE_PLINTH_RADIUS, length=BASE_PLINTH_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLINTH_HEIGHT / 2.0)),
        material=base_dark,
        name="base_plinth",
    )
    fixed_base.visual(
        Cylinder(radius=LOWER_SUPPORT_RADIUS, length=LOWER_SUPPORT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLINTH_HEIGHT + (LOWER_SUPPORT_HEIGHT / 2.0))),
        material=steel,
        name="lower_support",
    )
    fixed_base.visual(
        Cylinder(radius=LOWER_COLUMN_RADIUS, length=LOWER_COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_Z + (LOWER_COLUMN_HEIGHT / 2.0))),
        material=base_dark,
        name="lower_column",
    )
    fixed_base.visual(
        Cylinder(radius=MIDDLE_SUPPORT_RADIUS, length=MIDDLE_SUPPORT_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, MIDDLE_SUPPORT_BOTTOM + (MIDDLE_SUPPORT_HEIGHT / 2.0))
        ),
        material=steel,
        name="middle_support",
    )
    fixed_base.visual(
        Cylinder(radius=UPPER_COLUMN_RADIUS, length=UPPER_COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_STAGE_Z + (UPPER_COLUMN_HEIGHT / 2.0))),
        material=base_dark,
        name="upper_column",
    )
    fixed_base.visual(
        Cylinder(radius=TOP_SUPPORT_RADIUS, length=TOP_SUPPORT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, TOP_SUPPORT_BOTTOM + (TOP_SUPPORT_HEIGHT / 2.0))),
        material=steel,
        name="top_support",
    )
    fixed_base.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_BOTTOM + (SHAFT_HEIGHT / 2.0))),
        material=machined,
        name="shaft",
    )

    base_stage = model.part("base_stage")
    base_stage.visual(
        mesh_from_cadquery(_make_base_turntable(), "base_turntable_mesh"),
        material=aluminum,
        name="base_turntable",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(_make_middle_ring(), "middle_ring_mesh"),
        material=steel,
        name="middle_ring",
    )

    top_stage = model.part("top_stage")
    top_stage.visual(
        mesh_from_cadquery(_make_top_flange(), "top_flange_mesh"),
        material=machined,
        name="top_flange",
    )

    one_turn_limits = MotionLimits(
        effort=25.0,
        velocity=2.5,
        lower=-math.pi,
        upper=math.pi,
    )

    model.articulation(
        "fixed_to_base_stage",
        ArticulationType.REVOLUTE,
        parent=fixed_base,
        child=base_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=one_turn_limits,
    )
    model.articulation(
        "fixed_to_middle_stage",
        ArticulationType.REVOLUTE,
        parent=fixed_base,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=one_turn_limits,
    )
    model.articulation(
        "fixed_to_top_stage",
        ArticulationType.REVOLUTE,
        parent=fixed_base,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, TOP_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=one_turn_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_base = object_model.get_part("fixed_base")
    base_stage = object_model.get_part("base_stage")
    middle_stage = object_model.get_part("middle_stage")
    top_stage = object_model.get_part("top_stage")

    lower_support = fixed_base.get_visual("lower_support")
    middle_support = fixed_base.get_visual("middle_support")
    top_support = fixed_base.get_visual("top_support")
    base_turntable = base_stage.get_visual("base_turntable")
    middle_ring = middle_stage.get_visual("middle_ring")
    top_flange = top_stage.get_visual("top_flange")

    base_joint = object_model.get_articulation("fixed_to_base_stage")
    middle_joint = object_model.get_articulation("fixed_to_middle_stage")
    top_joint = object_model.get_articulation("fixed_to_top_stage")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
        "all_stage_parts_present",
        all(
            part is not None
            for part in (fixed_base, base_stage, middle_stage, top_stage)
        ),
        details="Expected fixed base and all three rotary stages.",
    )
    ctx.check(
        "vertical_axes_shared",
        base_joint.axis == (0.0, 0.0, 1.0)
        and middle_joint.axis == (0.0, 0.0, 1.0)
        and top_joint.axis == (0.0, 0.0, 1.0),
        details="All three stage joints should rotate about the common +Z shaft axis.",
    )

    ctx.expect_origin_distance(base_stage, fixed_base, axes="xy", max_dist=1e-6)
    ctx.expect_origin_distance(middle_stage, fixed_base, axes="xy", max_dist=1e-6)
    ctx.expect_origin_distance(top_stage, fixed_base, axes="xy", max_dist=1e-6)
    ctx.expect_origin_gap(
        middle_stage,
        base_stage,
        axis="z",
        min_gap=0.049,
        max_gap=0.051,
    )
    ctx.expect_origin_gap(
        top_stage,
        middle_stage,
        axis="z",
        min_gap=0.049,
        max_gap=0.051,
    )

    with ctx.pose({base_joint: 0.0, middle_joint: 0.0, top_joint: 0.0}):
        ctx.expect_gap(
            base_stage,
            fixed_base,
            axis="z",
            max_gap=0.0005,
            max_penetration=1e-6,
            positive_elem=base_turntable,
            negative_elem=lower_support,
            name="base_stage_seated_on_lower_support",
        )
        ctx.expect_overlap(
            base_stage,
            fixed_base,
            axes="xy",
            min_overlap=0.10,
            elem_a=base_turntable,
            elem_b=lower_support,
            name="base_stage_support_footprint",
        )
        ctx.expect_contact(
            base_stage,
            fixed_base,
            elem_a=base_turntable,
            elem_b=lower_support,
            name="base_stage_bearing_contact",
        )

        ctx.expect_gap(
            middle_stage,
            fixed_base,
            axis="z",
            max_gap=0.0005,
            max_penetration=1e-6,
            positive_elem=middle_ring,
            negative_elem=middle_support,
            name="middle_stage_seated_on_middle_support",
        )
        ctx.expect_overlap(
            middle_stage,
            fixed_base,
            axes="xy",
            min_overlap=0.08,
            elem_a=middle_ring,
            elem_b=middle_support,
            name="middle_stage_support_footprint",
        )
        ctx.expect_contact(
            middle_stage,
            fixed_base,
            elem_a=middle_ring,
            elem_b=middle_support,
            name="middle_stage_bearing_contact",
        )

        ctx.expect_gap(
            top_stage,
            fixed_base,
            axis="z",
            max_gap=0.0005,
            max_penetration=1e-6,
            positive_elem=top_flange,
            negative_elem=top_support,
            name="top_stage_seated_on_top_support",
        )
        ctx.expect_overlap(
            top_stage,
            fixed_base,
            axes="xy",
            min_overlap=0.06,
            elem_a=top_flange,
            elem_b=top_support,
            name="top_stage_support_footprint",
        )
        ctx.expect_contact(
            top_stage,
            fixed_base,
            elem_a=top_flange,
            elem_b=top_support,
            name="top_stage_bearing_contact",
        )

        ctx.expect_gap(
            middle_stage,
            base_stage,
            axis="z",
            min_gap=0.015,
            name="middle_stage_clears_base_stage",
        )
        ctx.expect_gap(
            top_stage,
            middle_stage,
            axis="z",
            min_gap=0.020,
            name="top_stage_clears_middle_stage",
        )

    for joint in (base_joint, middle_joint, top_joint):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{joint.name}_lower_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{joint.name}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    with ctx.pose(
        {
            base_joint: 1.1,
            middle_joint: -0.8,
            top_joint: 0.65,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
