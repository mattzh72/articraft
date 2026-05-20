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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOP_PLATE_LENGTH = 0.240
TOP_PLATE_DEPTH = 0.090
TOP_PLATE_THICKNESS = 0.012

AXIS_DROP = 0.105
AXIS_Z_IN_SUPPORT = -AXIS_DROP

HANGER_X = 0.074
HANGER_THICKNESS = 0.016
HANGER_DEPTH = 0.048
HANGER_HEIGHT = 0.077
HANGER_CENTER_Z = -0.043
BRIDGE_LENGTH = 0.182
BRIDGE_DEPTH = 0.032
BRIDGE_THICKNESS = 0.006
BRIDGE_CENTER_Z = -((TOP_PLATE_THICKNESS / 2.0) + (BRIDGE_THICKNESS / 2.0) - 0.0015)
BEARING_PAD_LENGTH = 0.026
BEARING_PAD_DEPTH = 0.022
BEARING_PAD_THICKNESS = 0.008
BEARING_PAD_CENTER_Z = -0.085

SHAFT_RADIUS = 0.014
JOURNAL_RADIUS = 0.0160
SHAFT_START_X = -0.128
SHAFT_END_X = 0.128
ARBOR_RADIUS = 0.032
ARBOR_START_X = -0.048
ARBOR_END_X = 0.048
JOURNAL_LENGTH = 0.036

FLANGE_X = -0.107
FLANGE_RADIUS = 0.080
FLANGE_THICKNESS = 0.012
FLANGE_HUB_RADIUS = 0.027
FLANGE_HUB_THICKNESS = 0.018

NOSE_START_X = 0.092
NOSE_LENGTH = 0.035
NOSE_BASE_RADIUS = 0.022
NOSE_TIP_RADIUS = 0.005

HANDLE_RADIUS = 0.008
HANDLE_LENGTH = 0.018
HANDLE_X = -0.093
HANDLE_Y = 0.060
HANDLE_Z = 0.0


def _x_cylinder(start_x: float, length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ", origin=(start_x, 0.0, 0.0)).circle(radius).extrude(length)


def _build_support_frame() -> cq.Workplane:
    frame = (
        cq.Workplane("XY", origin=(0.0, 0.0, BRIDGE_CENTER_Z))
        .box(BRIDGE_LENGTH, BRIDGE_DEPTH, BRIDGE_THICKNESS)
    )

    for x_center in (-HANGER_X, HANGER_X):
        hanger = (
            cq.Workplane("XY", origin=(x_center, 0.0, HANGER_CENTER_Z))
            .box(HANGER_THICKNESS, HANGER_DEPTH, HANGER_HEIGHT)
        )
        lightening_slot = (
            cq.Workplane("YZ", origin=(x_center, 0.0, -0.043))
            .slot2D(0.040, 0.014, angle=90.0)
            .extrude(HANGER_THICKNESS + 0.006, both=True)
        )
        bearing_pad = (
            cq.Workplane("XY", origin=(x_center, 0.0, BEARING_PAD_CENTER_Z))
            .box(BEARING_PAD_LENGTH, BEARING_PAD_DEPTH, BEARING_PAD_THICKNESS)
        )
        frame = frame.union(hanger).union(bearing_pad).cut(lightening_slot)

    return frame


def _build_spindle_body() -> cq.Workplane:
    body = _x_cylinder(SHAFT_START_X, SHAFT_END_X - SHAFT_START_X, SHAFT_RADIUS)

    for center_x in (-HANGER_X, HANGER_X):
        body = body.union(
            _x_cylinder(center_x - (JOURNAL_LENGTH / 2.0), JOURNAL_LENGTH, JOURNAL_RADIUS)
        )

    body = body.union(_x_cylinder(ARBOR_START_X, ARBOR_END_X - ARBOR_START_X, ARBOR_RADIUS))
    body = body.union(
        cq.Workplane("YZ", origin=(NOSE_START_X, 0.0, 0.0))
        .circle(NOSE_BASE_RADIUS)
        .workplane(offset=NOSE_LENGTH)
        .circle(NOSE_TIP_RADIUS)
        .loft(combine=True, ruled=False)
    )

    return body


def _build_flange_wheel() -> cq.Workplane:
    hole_points: list[tuple[float, float]] = []
    for angle_deg in (25.0, 145.0, 255.0):
        angle = math.radians(angle_deg)
        hole_points.append((0.042 * math.cos(angle), 0.042 * math.sin(angle)))

    wheel = (
        cq.Workplane("YZ", origin=(FLANGE_X - (FLANGE_THICKNESS / 2.0), 0.0, 0.0))
        .circle(FLANGE_RADIUS)
        .pushPoints(hole_points)
        .circle(0.013)
        .extrude(FLANGE_THICKNESS)
    )
    hub = _x_cylinder(
        FLANGE_X - (FLANGE_HUB_THICKNESS / 2.0),
        FLANGE_HUB_THICKNESS,
        FLANGE_HUB_RADIUS,
    )
    return wheel.union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_roll_fixture")

    bracket_finish = model.material("bracket_finish", rgba=(0.25, 0.28, 0.31, 1.0))
    spindle_finish = model.material("spindle_finish", rgba=(0.71, 0.73, 0.76, 1.0))
    flange_finish = model.material("flange_finish", rgba=(0.52, 0.54, 0.57, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.14, 0.15, 0.16, 1.0))

    support_bracket = model.part("support_bracket")
    support_bracket.visual(
        Box((TOP_PLATE_LENGTH, TOP_PLATE_DEPTH, TOP_PLATE_THICKNESS)),
        material=bracket_finish,
        name="top_plate",
    )
    support_bracket.visual(
        mesh_from_cadquery(_build_support_frame(), "support_frame"),
        material=bracket_finish,
        name="hanger_frame",
    )
    support_bracket.inertial = Inertial.from_geometry(
        Box((TOP_PLATE_LENGTH, TOP_PLATE_DEPTH, AXIS_DROP + 0.030)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, -(AXIS_DROP / 2.0))),
    )

    spindle_package = model.part("spindle_package")
    spindle_package.visual(
        mesh_from_cadquery(_build_spindle_body(), "spindle_body"),
        material=spindle_finish,
        name="spindle_body",
    )
    spindle_package.visual(
        mesh_from_cadquery(_build_flange_wheel(), "spindle_flange"),
        material=flange_finish,
        name="flange_wheel",
    )
    spindle_package.visual(
        Cylinder(radius=HANDLE_RADIUS, length=HANDLE_LENGTH),
        origin=Origin(
            xyz=(HANDLE_X, HANDLE_Y, HANDLE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_finish,
        name="drive_handle",
    )
    spindle_package.inertial = Inertial.from_geometry(
        Box((0.270, 0.160, 0.160)),
        mass=3.1,
        origin=Origin(),
    )

    model.articulation(
        "bracket_to_spindle",
        ArticulationType.REVOLUTE,
        parent=support_bracket,
        child=spindle_package,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z_IN_SUPPORT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-(2.0 * math.pi),
            upper=2.0 * math.pi,
            effort=25.0,
            velocity=12.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    support_bracket = object_model.get_part("support_bracket")
    spindle_package = object_model.get_part("spindle_package")
    spin_joint = object_model.get_articulation("bracket_to_spindle")
    top_plate = support_bracket.get_visual("top_plate")
    spindle_body = spindle_package.get_visual("spindle_body")
    flange_wheel = spindle_package.get_visual("flange_wheel")

    ctx.allow_isolated_part(
        spindle_package,
        reason=(
            "The under-slung spindle is carried by hidden bearing-pad contact under the hanger frame; "
            "the compiled floating check sees only a 5e-06 m tessellation gap at that mount."
        ),
    )

    ctx.check("support bracket present", support_bracket is not None)
    ctx.check("spindle package present", spindle_package is not None)
    ctx.check(
        "spin joint uses longitudinal x axis",
        tuple(spin_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin_joint.axis}",
    )
    ctx.expect_origin_gap(
        support_bracket,
        spindle_package,
        axis="z",
        min_gap=0.095,
        max_gap=0.115,
        name="spindle package hangs below the top support bracket",
    )
    ctx.expect_gap(
        support_bracket,
        spindle_package,
        axis="z",
        positive_elem=top_plate,
        negative_elem=flange_wheel,
        min_gap=0.015,
        name="support bracket remains visually distinct above the flange",
    )
    ctx.expect_gap(
        support_bracket,
        spindle_package,
        axis="z",
        positive_elem=top_plate,
        negative_elem=spindle_body,
        min_gap=0.055,
        name="top plate clears the carried spindle body",
    )
    ctx.expect_contact(
        support_bracket,
        spindle_package,
        elem_a="hanger_frame",
        elem_b=spindle_body,
        contact_tol=1e-4,
        name="bearing pads support the spindle journals",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(spindle_package, elem="drive_handle")
    with ctx.pose({spin_joint: math.pi / 2.0}):
        turned_handle_aabb = ctx.part_element_world_aabb(spindle_package, elem="drive_handle")
        ctx.expect_gap(
            support_bracket,
            spindle_package,
            axis="z",
            positive_elem=top_plate,
            negative_elem="drive_handle",
            min_gap=0.025,
            name="rotated handle still clears the support bracket",
        )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((low + high) / 2.0 for low, high in zip(aabb[0], aabb[1]))

    rest_center = _aabb_center(rest_handle_aabb)
    turned_center = _aabb_center(turned_handle_aabb)
    ctx.check(
        "positive spin lifts the off-axis handle around the shaft",
        rest_center is not None
        and turned_center is not None
        and abs(turned_center[0] - rest_center[0]) < 0.005
        and turned_center[2] > rest_center[2] + 0.045,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
