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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_WIDTH = 0.072
PLATE_THICKNESS = 0.0018
BARREL_RADIUS = 0.0042
PIN_RADIUS = 0.00235
PIN_CLEARANCE = 0.00015
BRACKET_HEIGHT = 0.026
COVER_LENGTH = 0.040
FLANGE_HEIGHT = 0.012
KNUCKLE_GAP = 0.001
END_KNUCKLE_LENGTH = 0.018
MID_KNUCKLE_LENGTH = HINGE_WIDTH - 2.0 * END_KNUCKLE_LENGTH - 2.0 * KNUCKLE_GAP
KNUCKLE_CENTER_Y = 0.5 * MID_KNUCKLE_LENGTH + KNUCKLE_GAP + 0.5 * END_KNUCKLE_LENGTH
FLANGE_SPAN = MID_KNUCKLE_LENGTH + 0.006
PIN_OVERHANG = 0.0012
MERGE_OVERLAP = 0.00035
PLATE_TO_BARREL_RELIEF = 0.0003
BRIDGE_HEIGHT = 0.0055
BRIDGE_LENGTH_Y = END_KNUCKLE_LENGTH + 0.0006


def _y_cylinder(radius: float, length: float, center_y: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, center_y, 0.0))
    )


def _make_bracket_body() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_THICKNESS, HINGE_WIDTH, BRACKET_HEIGHT).translate(
        (-BARREL_RADIUS - PLATE_TO_BARREL_RELIEF - PLATE_THICKNESS / 2.0, 0.0, -BRACKET_HEIGHT / 2.0)
    )
    lower_knuckle = _y_cylinder(BARREL_RADIUS, END_KNUCKLE_LENGTH, -KNUCKLE_CENTER_Y)
    upper_knuckle = _y_cylinder(BARREL_RADIUS, END_KNUCKLE_LENGTH, KNUCKLE_CENTER_Y)
    lower_bridge = cq.Workplane("XY").box(
        BARREL_RADIUS + PLATE_TO_BARREL_RELIEF + PLATE_THICKNESS,
        BRIDGE_LENGTH_Y,
        BRIDGE_HEIGHT,
    ).translate(
        (
            -(BARREL_RADIUS + PLATE_TO_BARREL_RELIEF + PLATE_THICKNESS) / 2.0,
            -KNUCKLE_CENTER_Y,
            -BARREL_RADIUS,
        )
    )
    upper_bridge = cq.Workplane("XY").box(
        BARREL_RADIUS + PLATE_TO_BARREL_RELIEF + PLATE_THICKNESS,
        BRIDGE_LENGTH_Y,
        BRIDGE_HEIGHT,
    ).translate(
        (
            -(BARREL_RADIUS + PLATE_TO_BARREL_RELIEF + PLATE_THICKNESS) / 2.0,
            KNUCKLE_CENTER_Y,
            -BARREL_RADIUS,
        )
    )
    return plate.union(lower_knuckle).union(upper_knuckle).union(lower_bridge).union(upper_bridge)


def _make_cover_knuckle() -> cq.Workplane:
    outer = _y_cylinder(BARREL_RADIUS, MID_KNUCKLE_LENGTH)
    inner = _y_cylinder(PIN_RADIUS + PIN_CLEARANCE, MID_KNUCKLE_LENGTH + 0.002)
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_cover_hinge")

    bracket_finish = model.material("bracket_finish", rgba=(0.48, 0.50, 0.53, 1.0))
    cover_finish = model.material("cover_finish", rgba=(0.72, 0.74, 0.77, 1.0))
    pin_finish = model.material("pin_finish", rgba=(0.62, 0.64, 0.67, 1.0))

    bracket = model.part("bracket")
    bracket.visual(
        mesh_from_cadquery(_make_bracket_body(), "bracket_body", tolerance=0.00025, angular_tolerance=0.05),
        material=bracket_finish,
        name="bracket_body",
    )
    bracket.visual(
        Cylinder(radius=PIN_RADIUS, length=HINGE_WIDTH + 2.0 * PIN_OVERHANG),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_finish,
        name="pin",
    )

    cover = model.part("cover")
    cover.visual(
        Box((COVER_LENGTH, HINGE_WIDTH, PLATE_THICKNESS)),
        origin=Origin(
            xyz=(
                BARREL_RADIUS + COVER_LENGTH / 2.0,
                0.0,
                FLANGE_HEIGHT + PLATE_THICKNESS / 2.0 - MERGE_OVERLAP / 2.0,
            )
        ),
        material=cover_finish,
        name="cover_panel",
    )
    cover.visual(
        Box((PLATE_THICKNESS, FLANGE_SPAN, FLANGE_HEIGHT)),
        origin=Origin(xyz=(BARREL_RADIUS + PLATE_THICKNESS / 2.0 - MERGE_OVERLAP, 0.0, FLANGE_HEIGHT / 2.0)),
        material=cover_finish,
        name="cover_flange",
    )
    cover.visual(
        mesh_from_cadquery(_make_cover_knuckle(), "cover_knuckle", tolerance=0.0002, angular_tolerance=0.04),
        material=cover_finish,
        name="cover_knuckle",
    )

    model.articulation(
        "bracket_to_cover",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("bracket")
    cover = object_model.get_part("cover")
    hinge = object_model.get_articulation("bracket_to_cover")

    bracket_body = bracket.get_visual("bracket_body")
    pin = bracket.get_visual("pin")
    cover_panel = cover.get_visual("cover_panel")
    cover_flange = cover.get_visual("cover_flange")
    cover_knuckle = cover.get_visual("cover_knuckle")

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

    limits = hinge.motion_limits
    ctx.check(
        "hinge axis and range are box-cover like",
        hinge.axis == (0.0, -1.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 1.6,
        details="Expected a single outward-opening revolute axis with a little over 90 degrees of travel.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            cover,
            cover,
            elem_a=cover_flange,
            elem_b=cover_knuckle,
            contact_tol=0.0004,
            name="cover flange is connected into its knuckle barrel",
        )
        ctx.expect_gap(
            cover,
            bracket,
            axis="z",
            positive_elem=cover_panel,
            negative_elem=bracket_body,
            min_gap=0.006,
            max_gap=0.011,
            name="closed cover panel clears the fixed bracket leaf",
        )
        ctx.expect_overlap(
            cover,
            bracket,
            axes="xz",
            min_overlap=0.004,
            elem_a=cover_knuckle,
            elem_b=pin,
            name="cover knuckle stays concentric with the pin axis",
        )

    with ctx.pose({hinge: 1.2}):
        ctx.fail_if_parts_overlap_in_current_pose(name="opened hinge stays clear")

    with ctx.pose({hinge: 0.0}):
        closed_panel_aabb = ctx.part_element_world_aabb(cover, elem=cover_panel.name)
    with ctx.pose({hinge: 1.2}):
        open_panel_aabb = ctx.part_element_world_aabb(cover, elem=cover_panel.name)

    opens_upward = (
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.02
    )
    ctx.check(
        "cover opens upward",
        opens_upward,
        details="The free edge of the cover panel should rise noticeably when the hinge opens.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
