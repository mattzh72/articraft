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


BASE_RADIUS = 0.090
BASE_LOWER_HEIGHT = 0.012
BASE_DECK_RADIUS = 0.065
BASE_DECK_HEIGHT = 0.010
BASE_HEIGHT = BASE_LOWER_HEIGHT + BASE_DECK_HEIGHT

BEARING_OUTER_RADIUS = 0.043
LOWER_BEARING_INNER_RADIUS = 0.0105
UPPER_BEARING_INNER_RADIUS = 0.0095
BEARING_HEIGHT = 0.006

SPINDLE_RADIUS = 0.008
SPINDLE_HEIGHT = 0.006

UPPER_PLATE_RADIUS = 0.078
UPPER_PLATE_HEIGHT = 0.010
HUB_POCKET_RADIUS = 0.012
HUB_POCKET_DEPTH = 0.006

BRACKET_WIDTH = 0.036
BRACKET_DEPTH = 0.022
BRACKET_HEIGHT = 0.038
BRACKET_SLOT_WIDTH = 0.022

JOINT_Z = BASE_HEIGHT + BEARING_HEIGHT


def _base_shell_shape() -> cq.Workplane:
    lower = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_LOWER_HEIGHT)
    deck = (
        cq.Workplane("XY")
        .circle(BASE_DECK_RADIUS)
        .extrude(BASE_DECK_HEIGHT)
        .translate((0.0, 0.0, BASE_LOWER_HEIGHT))
    )
    return lower.union(deck)


def _annular_ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    ring = cq.Workplane("XY").circle(outer_radius).extrude(height)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(height)
    return ring.cut(bore)


def _upper_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").circle(UPPER_PLATE_RADIUS).extrude(UPPER_PLATE_HEIGHT)
    pocket = cq.Workplane("XY").circle(HUB_POCKET_RADIUS).extrude(HUB_POCKET_DEPTH)
    return plate.cut(pocket)


def _payload_bracket_shape() -> cq.Workplane:
    block = cq.Workplane("XY").rect(BRACKET_WIDTH, BRACKET_DEPTH).extrude(BRACKET_HEIGHT)
    slot_depth = BRACKET_HEIGHT - UPPER_PLATE_HEIGHT / 2.0
    return (
        block.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(BRACKET_SLOT_WIDTH, BRACKET_DEPTH + 0.002)
        .cutBlind(-slot_depth)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_turntable")

    base_finish = model.material("base_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    aluminum = model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    hub_finish = model.material("hub_finish", rgba=(0.46, 0.48, 0.50, 1.0))
    bracket_finish = model.material("bracket_finish", rgba=(0.42, 0.45, 0.48, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shell_shape(), "base_shell"),
        origin=Origin(),
        material=base_finish,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(
            _annular_ring(
                BEARING_OUTER_RADIUS,
                LOWER_BEARING_INNER_RADIUS,
                BEARING_HEIGHT,
            ),
            "lower_bearing_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        material=aluminum,
        name="lower_bearing_ring",
    )
    base.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=SPINDLE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + SPINDLE_HEIGHT / 2.0)),
        material=hub_finish,
        name="center_spindle",
    )

    upper = model.part("upper_plate")
    upper.visual(
        mesh_from_cadquery(
            _annular_ring(
                BEARING_OUTER_RADIUS,
                UPPER_BEARING_INNER_RADIUS,
                BEARING_HEIGHT,
            ),
            "upper_bearing_ring",
        ),
        origin=Origin(),
        material=aluminum,
        name="upper_bearing_ring",
    )
    upper.visual(
        mesh_from_cadquery(_upper_plate_shape(), "upper_plate_disk"),
        origin=Origin(xyz=(0.0, 0.0, BEARING_HEIGHT)),
        material=aluminum,
        name="upper_plate_disk",
    )
    upper.visual(
        mesh_from_cadquery(_payload_bracket_shape(), "payload_bracket"),
        origin=Origin(xyz=(0.0, 0.0, BEARING_HEIGHT + UPPER_PLATE_HEIGHT)),
        material=bracket_finish,
        name="payload_bracket",
    )

    model.articulation(
        "base_to_upper_plate",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-math.radians(150.0),
            upper=math.radians(150.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper = object_model.get_part("upper_plate")
    yaw = object_model.get_articulation("base_to_upper_plate")

    base_shell = base.get_visual("base_shell")
    lower_bearing = base.get_visual("lower_bearing_ring")
    spindle = base.get_visual("center_spindle")
    upper_bearing = upper.get_visual("upper_bearing_ring")
    upper_plate = upper.get_visual("upper_plate_disk")
    bracket = upper.get_visual("payload_bracket")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_part_contains_disconnected_geometry_islands(name="no_disconnected_visual_islands")
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=16, name="yaw_pose_clearance")
    ctx.fail_if_articulation_origin_far_from_geometry(
        tol=0.010,
        name="yaw_axis_close_to_bearing_stack",
    )

    ctx.check(
        "key_visuals_present",
        all(
            visual is not None
            for visual in (
                base_shell,
                lower_bearing,
                spindle,
                upper_bearing,
                upper_plate,
                bracket,
            )
        ),
        "One or more named visuals could not be resolved.",
    )
    ctx.check(
        "yaw_joint_axis_vertical",
        tuple(round(value, 6) for value in yaw.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical axis, got {yaw.axis!r}.",
    )

    limits = yaw.motion_limits
    ctx.check(
        "yaw_joint_limits_realistic",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and math.isclose(limits.lower, -math.radians(150.0), abs_tol=1e-6)
        and math.isclose(limits.upper, math.radians(150.0), abs_tol=1e-6),
        f"Unexpected limits: {limits!r}.",
    )

    with ctx.pose({yaw: 0.0}):
        ctx.expect_contact(
            upper,
            base,
            elem_a=upper_bearing,
            elem_b=lower_bearing,
            contact_tol=1e-6,
            name="bearing_stack_contact_at_rest",
        )
        ctx.expect_gap(
            upper,
            base,
            axis="z",
            positive_elem=upper_plate,
            negative_elem=base_shell,
            min_gap=0.011,
            max_gap=0.013,
            name="upper_plate_stands_above_base",
        )
        ctx.expect_within(
            upper,
            base,
            axes="xy",
            inner_elem=upper_plate,
            outer_elem=base_shell,
            margin=0.0,
            name="upper_plate_kept_inside_base_footprint",
        )
        ctx.expect_origin_distance(
            upper,
            base,
            axes="xy",
            min_dist=0.0,
            max_dist=1e-6,
            name="turntable_axes_are_centered",
        )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({yaw: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="yaw_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="yaw_lower_no_floating")
            ctx.expect_contact(
                upper,
                base,
                elem_a=upper_bearing,
                elem_b=lower_bearing,
                contact_tol=1e-6,
                name="bearing_stack_contact_at_lower_limit",
            )
        with ctx.pose({yaw: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="yaw_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="yaw_upper_no_floating")
            ctx.expect_contact(
                upper,
                base,
                elem_a=upper_bearing,
                elem_b=lower_bearing,
                contact_tol=1e-6,
                name="bearing_stack_contact_at_upper_limit",
            )

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
