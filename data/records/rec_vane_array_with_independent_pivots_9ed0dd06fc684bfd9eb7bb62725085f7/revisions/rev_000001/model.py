from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_WIDTH = 1.00
OUTER_HEIGHT = 1.25
FRAME_DEPTH = 0.12
STILE_WIDTH = 0.07
RAIL_HEIGHT = 0.08

SLAT_COUNT = 6
SLAT_CHORD = 0.105
SLAT_THICKNESS = 0.018
PIVOT_HUB_RADIUS = 0.015
PIVOT_HUB_LENGTH = 0.012
REST_TILT_RAD = math.radians(-22.0)


def _x_cylinder(radius: float, length: float, x_start: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((x_start, 0.0, 0.0))
    )


def _frame_inner_width() -> float:
    return OUTER_WIDTH - 2.0 * STILE_WIDTH


def _slat_centers_z() -> list[float]:
    opening_bottom = RAIL_HEIGHT
    opening_top = OUTER_HEIGHT - RAIL_HEIGHT
    clear_height = opening_top - opening_bottom
    pitch = clear_height / (SLAT_COUNT + 1)
    return [opening_bottom + pitch * (index + 1) for index in range(SLAT_COUNT)]


def _build_slat_solid() -> cq.Workplane:
    inner_width = _frame_inner_width()
    body_length = inner_width - 2.0 * PIVOT_HUB_LENGTH

    body = (
        cq.Workplane("YZ")
        .ellipse(SLAT_CHORD / 2.0, SLAT_THICKNESS / 2.0)
        .extrude(body_length)
        .translate((PIVOT_HUB_LENGTH, 0.0, 0.0))
    )

    left_hub = _x_cylinder(PIVOT_HUB_RADIUS, PIVOT_HUB_LENGTH, 0.0)
    right_hub = _x_cylinder(
        PIVOT_HUB_RADIUS,
        PIVOT_HUB_LENGTH,
        inner_width - PIVOT_HUB_LENGTH,
    )

    return body.union(left_hub).union(right_hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louver_bank")

    frame_color = model.material("frame_powdercoat", rgba=(0.20, 0.22, 0.24, 1.0))
    slat_color = model.material("slat_finish", rgba=(0.80, 0.82, 0.84, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(
            xyz=(
                -(OUTER_WIDTH / 2.0 - STILE_WIDTH / 2.0),
                0.0,
                OUTER_HEIGHT / 2.0,
            )
        ),
        material=frame_color,
        name="left_stile",
    )
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_WIDTH / 2.0 - STILE_WIDTH / 2.0,
                0.0,
                OUTER_HEIGHT / 2.0,
            )
        ),
        material=frame_color,
        name="right_stile",
    )
    frame.visual(
        Box((OUTER_WIDTH - 2.0 * STILE_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT / 2.0)),
        material=frame_color,
        name="bottom_rail",
    )
    frame.visual(
        Box((OUTER_WIDTH - 2.0 * STILE_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT - RAIL_HEIGHT / 2.0)),
        material=frame_color,
        name="top_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((OUTER_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT / 2.0)),
    )

    slat_mesh = mesh_from_cadquery(_build_slat_solid(), "louver_slat")
    inner_width = _frame_inner_width()
    slat_box_length = inner_width
    slat_box_center_x = inner_width / 2.0

    for index, z in enumerate(_slat_centers_z(), start=1):
        slat = model.part(f"slat_{index}")
        slat.visual(
            slat_mesh,
            origin=Origin(rpy=(REST_TILT_RAD, 0.0, 0.0)),
            material=slat_color,
            name="slat_shell",
        )
        slat.inertial = Inertial.from_geometry(
            Box((slat_box_length, SLAT_CHORD, SLAT_THICKNESS)),
            mass=1.0,
            origin=Origin(xyz=(slat_box_center_x, 0.0, 0.0)),
        )

        model.articulation(
            f"frame_to_slat_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(
                xyz=(-inner_width / 2.0, 0.0, z),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=1.6,
                lower=math.radians(-35.0),
                upper=math.radians(55.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    slats = [object_model.get_part(f"slat_{index}") for index in range(1, SLAT_COUNT + 1)]
    joints = [
        object_model.get_articulation(f"frame_to_slat_{index}")
        for index in range(1, SLAT_COUNT + 1)
    ]

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
        "independent_slat_joint_count",
        len(joints) == SLAT_COUNT,
        f"expected {SLAT_COUNT} revolute slat joints, found {len(joints)}",
    )
    ctx.check(
        "slat_joint_axes_aligned_with_span",
        all(joint.axis == (1.0, 0.0, 0.0) for joint in joints),
        "every vane should rotate about its long x-axis pivot",
    )

    for index, slat in enumerate(slats, start=1):
        ctx.expect_contact(
            slat,
            frame,
            contact_tol=1e-4,
            name=f"slat_{index}_mounted_to_frame",
        )
        ctx.expect_within(
            slat,
            frame,
            axes="yz",
            margin=0.002,
            name=f"slat_{index}_contained_in_frame_span",
        )

    open_pose = {joint: math.radians(42.0) for joint in joints}
    with ctx.pose(open_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_slats_open_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
