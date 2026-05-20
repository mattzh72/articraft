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
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 0.18
BASE_W = 0.10
BASE_T = 0.012

SUPPORT_CENTER_X = 0.055
SUPPORT_LEN = 0.030
SUPPORT_W = 0.072
SHAFT_AXIS_Z = 0.060
LOWER_LAND_Z = SHAFT_AXIS_Z - SHAFT_RADIUS if "SHAFT_RADIUS" in globals() else 0.045
UPPER_LAND_Z = SHAFT_AXIS_Z + SHAFT_RADIUS if "SHAFT_RADIUS" in globals() else 0.075
SUPPORT_H = LOWER_LAND_Z - BASE_T

WEB_W = 0.012
WEB_H = 0.024
WEB_Y = SUPPORT_W / 2.0 - WEB_W / 2.0

SHAFT_RADIUS = 0.015
COLLAR_RADIUS = 0.024
COLLAR_W = 0.010
SHAFT_LEN = 0.210

KEEPER_H = 0.028
BRIDGE_W = 0.042
BRIDGE_H = 0.016
BRIDGE_LEN = 2.0 * SUPPORT_CENTER_X - SUPPORT_LEN
LEG_W = 0.012
LEG_Y = SUPPORT_W / 2.0 - LEG_W / 2.0


def _box_on_base(
    sx: float,
    sy: float,
    sz: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    z0: float = 0.0,
):
    return (
        cq.Workplane("XY")
        .center(center_x, center_y)
        .box(sx, sy, sz, centered=(True, True, False))
        .translate((0.0, 0.0, z0))
    )


def _make_lower_frame():
    frame = _box_on_base(BASE_LEN, BASE_W, BASE_T)

    for sx in (-SUPPORT_CENTER_X, SUPPORT_CENTER_X):
        frame = frame.union(
            _box_on_base(
                SUPPORT_LEN,
                SUPPORT_W,
                SUPPORT_H,
                center_x=sx,
                z0=BASE_T,
            )
        )

    for sy in (-WEB_Y, WEB_Y):
        frame = frame.union(
            _box_on_base(
                2.0 * SUPPORT_CENTER_X + SUPPORT_LEN,
                WEB_W,
                WEB_H,
                center_y=sy,
                z0=BASE_T,
            )
        )

    return frame


def _make_top_keeper():
    keeper = cq.Workplane("XY")

    for sx in (-SUPPORT_CENTER_X, SUPPORT_CENTER_X):
        keeper = keeper.union(
            _box_on_base(
                SUPPORT_LEN,
                SUPPORT_W,
                KEEPER_H,
                center_x=sx,
                z0=UPPER_LAND_Z,
            )
        )

    for sx in (-SUPPORT_CENTER_X, SUPPORT_CENTER_X):
        for sy in (-LEG_Y, LEG_Y):
            keeper = keeper.union(
                _box_on_base(
                    SUPPORT_LEN,
                    LEG_W,
                    UPPER_LAND_Z - LOWER_LAND_Z,
                    center_x=sx,
                    center_y=sy,
                    z0=LOWER_LAND_Z,
                )
            )

    keeper = keeper.union(
        _box_on_base(
            BRIDGE_LEN,
            BRIDGE_W,
            BRIDGE_H,
            z0=UPPER_LAND_Z + KEEPER_H - BRIDGE_H,
        )
    )

    return keeper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_rotary_shaft_fixture")

    model.material("painted_frame", rgba=(0.24, 0.34, 0.56, 1.0))
    model.material("painted_keeper", rgba=(0.21, 0.30, 0.50, 1.0))
    model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        mesh_from_cadquery(_make_lower_frame(), "lower_frame"),
        material="painted_frame",
        name="frame_body",
    )

    top_keeper = model.part("top_keeper")
    top_keeper.visual(
        mesh_from_cadquery(_make_top_keeper(), "top_keeper"),
        material="painted_keeper",
        name="keeper_body",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LEN),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel",
        name="shaft_body",
    )

    collar_center_x = BASE_LEN / 2.0 + COLLAR_W / 2.0 + 0.004
    left_collar_x = -collar_center_x
    right_collar_x = collar_center_x
    collar_pose = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_W),
        origin=Origin(
            xyz=(left_collar_x, 0.0, 0.0),
            rpy=collar_pose.rpy,
        ),
        material="steel",
        name="left_collar",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_W),
        origin=Origin(
            xyz=(right_collar_x, 0.0, 0.0),
            rpy=collar_pose.rpy,
        ),
        material="steel",
        name="right_collar",
    )

    model.articulation(
        "frame_to_keeper",
        ArticulationType.FIXED,
        parent=lower_frame,
        child=top_keeper,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=lower_frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=15.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_frame = object_model.get_part("lower_frame")
    top_keeper = object_model.get_part("top_keeper")
    shaft = object_model.get_part("shaft")
    shaft_joint = object_model.get_articulation("frame_to_shaft")

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
        "shaft_joint_is_continuous",
        shaft_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected CONTINUOUS, got {shaft_joint.articulation_type}",
    )
    ctx.check(
        "shaft_joint_axis_runs_along_x",
        tuple(round(v, 6) for v in shaft_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis was {shaft_joint.axis}",
    )
    ctx.check(
        "continuous_joint_has_no_position_limits",
        shaft_joint.motion_limits is not None
        and shaft_joint.motion_limits.lower is None
        and shaft_joint.motion_limits.upper is None,
        details="continuous shaft should not have lower/upper position limits",
    )

    ctx.expect_contact(
        top_keeper,
        lower_frame,
        contact_tol=1e-4,
        name="keeper_seats_on_lower_frame",
    )
    ctx.expect_contact(
        shaft,
        lower_frame,
        contact_tol=1e-4,
        name="shaft_is_supported_by_lower_frame_lands",
    )
    ctx.expect_contact(
        shaft,
        top_keeper,
        contact_tol=1e-4,
        name="shaft_is_captured_by_top_keeper",
    )
    ctx.expect_origin_gap(
        shaft,
        lower_frame,
        axis="z",
        min_gap=SHAFT_AXIS_Z - 1e-6,
        max_gap=SHAFT_AXIS_Z + 1e-6,
        name="shaft_origin_matches_bearing_axis_height",
    )
    ctx.expect_overlap(
        shaft,
        top_keeper,
        axes="yz",
        min_overlap=0.028,
        name="keeper_stays_centered_over_shaft",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
