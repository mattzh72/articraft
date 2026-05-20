from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

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


BASE_LENGTH = 0.30
BASE_WIDTH = 0.11
BASE_THICKNESS = 0.012

AXIS_ABOVE_PLATE = 0.045
SHAFT_AXIS_Z = BASE_THICKNESS / 2.0 + AXIS_ABOVE_PLATE

BEARING_CENTER_X = 0.075
BLOCK_LENGTH = 0.054
BLOCK_WIDTH = 0.060
FOOT_THICKNESS = 0.010
PEDESTAL_HEIGHT = 0.020
HOUSING_RADIUS = 0.022
BOLT_SPACING_X = 0.017
BOLT_DIAMETER = 0.008

SHAFT_RADIUS = 0.009
SHAFT_LENGTH = 0.245
LEFT_COLLAR_RADIUS = 0.013
LEFT_COLLAR_LENGTH = 0.008
HUB_RADIUS = 0.014
HUB_LENGTH = 0.014
FLANGE_RADIUS = 0.0165
FLANGE_LENGTH = 0.004
RECESS_RADIUS = 0.0026
RECESS_DEPTH = 0.004
V_NOTCH_TOP_Z = 0.010
V_NOTCH_APEX_Z = -SHAFT_RADIUS * sqrt(2.0)
V_NOTCH_LIP_Y = V_NOTCH_TOP_Z - V_NOTCH_APEX_Z


def _mesh(shape: object, name: str):
    return mesh_from_cadquery(
        shape,
        name,
        tolerance=0.0003,
        angular_tolerance=0.05,
        unit_scale=1.0,
    )


def _make_bearing_block() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(BLOCK_LENGTH, BLOCK_WIDTH, FOOT_THICKNESS)
        .translate((0.0, 0.0, -AXIS_ABOVE_PLATE + FOOT_THICKNESS / 2.0))
    )
    pedestal = (
        cq.Workplane("XY")
        .box(BLOCK_LENGTH * 0.64, BLOCK_WIDTH * 0.64, PEDESTAL_HEIGHT)
        .translate(
            (
                0.0,
                0.0,
                -AXIS_ABOVE_PLATE + FOOT_THICKNESS + PEDESTAL_HEIGHT / 2.0,
            )
        )
    )
    housing = cq.Workplane("YZ").circle(HOUSING_RADIUS).extrude(BLOCK_LENGTH / 2.0, both=True)
    body = foot.union(pedestal).union(housing)

    notch = cq.Workplane("YZ").polyline(
        [
            (-0.040, HOUSING_RADIUS + 0.012),
            (0.040, HOUSING_RADIUS + 0.012),
            (V_NOTCH_LIP_Y, V_NOTCH_TOP_Z),
            (0.0, V_NOTCH_APEX_Z),
            (-V_NOTCH_LIP_Y, V_NOTCH_TOP_Z),
        ]
    ).close().extrude(
        BLOCK_LENGTH,
        both=True,
    )
    body = body.cut(notch)

    body = (
        body.faces("<Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-BOLT_SPACING_X, 0.0), (BOLT_SPACING_X, 0.0)])
        .hole(BOLT_DIAMETER, depth=FOOT_THICKNESS + 0.004)
    )
    return body


def _make_end_module() -> cq.Workplane:
    hub_center_x = 0.111
    flange_center_x = 0.1205

    hub = (
        cq.Workplane("YZ")
        .circle(HUB_RADIUS)
        .extrude(HUB_LENGTH / 2.0, both=True)
        .translate((hub_center_x, 0.0, 0.0))
    )
    flange = (
        cq.Workplane("YZ")
        .circle(FLANGE_RADIUS)
        .extrude(FLANGE_LENGTH / 2.0, both=True)
        .translate((flange_center_x, 0.0, 0.0))
    )
    recess = (
        cq.Workplane("XZ")
        .circle(RECESS_RADIUS)
        .extrude(RECESS_DEPTH / 2.0, both=True)
        .translate((hub_center_x, HUB_RADIUS - RECESS_DEPTH / 2.0, 0.0))
    )
    return hub.union(flange).cut(recess)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supported_rotary_shaft_module")

    plate_mat = model.material("plate_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    block_mat = model.material("bearing_green", rgba=(0.22, 0.46, 0.24, 1.0))
    shaft_mat = model.material("shaft_steel", rgba=(0.74, 0.76, 0.78, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        material=plate_mat,
        name="plate",
    )

    left_block = model.part("left_bearing_block")
    left_block.visual(
        _mesh(_make_bearing_block(), "left_bearing_block"),
        material=block_mat,
        name="block_housing",
    )

    right_block = model.part("right_bearing_block")
    right_block.visual(
        _mesh(_make_bearing_block(), "right_bearing_block"),
        material=block_mat,
        name="block_housing",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        material=shaft_mat,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        name="shaft_body",
    )
    shaft.visual(
        Cylinder(radius=LEFT_COLLAR_RADIUS, length=LEFT_COLLAR_LENGTH),
        material=shaft_mat,
        origin=Origin(xyz=(-0.111, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        name="left_collar",
    )
    shaft.visual(
        _mesh(_make_end_module(), "shaft_end_module"),
        material=shaft_mat,
        name="end_module",
    )

    model.articulation(
        "base_to_left_bearing_block",
        ArticulationType.FIXED,
        parent=base_plate,
        child=left_block,
        origin=Origin(xyz=(-BEARING_CENTER_X, 0.0, SHAFT_AXIS_Z)),
    )
    model.articulation(
        "base_to_right_bearing_block",
        ArticulationType.FIXED,
        parent=base_plate,
        child=right_block,
        origin=Origin(xyz=(BEARING_CENTER_X, 0.0, SHAFT_AXIS_Z)),
    )
    model.articulation(
        "base_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    left_block = object_model.get_part("left_bearing_block")
    right_block = object_model.get_part("right_bearing_block")
    shaft = object_model.get_part("shaft")
    shaft_spin = object_model.get_articulation("base_to_shaft")
    shaft_body = shaft.get_visual("shaft_body")
    end_module = shaft.get_visual("end_module")

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
        shaft_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"Expected continuous shaft joint, got {shaft_spin.articulation_type!r}",
    )
    ctx.check(
        "shaft_joint_axis_is_x",
        tuple(round(v, 6) for v in shaft_spin.axis) == (1.0, 0.0, 0.0),
        details=f"Expected shaft axis (1, 0, 0), got {shaft_spin.axis!r}",
    )

    ctx.expect_contact(
        left_block,
        base_plate,
        contact_tol=5e-4,
        name="left_block_contacts_plate",
    )
    ctx.expect_contact(
        right_block,
        base_plate,
        contact_tol=5e-4,
        name="right_block_contacts_plate",
    )
    ctx.expect_overlap(
        left_block,
        base_plate,
        axes="xy",
        min_overlap=0.03,
        name="left_block_has_plate_footprint",
    )
    ctx.expect_overlap(
        right_block,
        base_plate,
        axes="xy",
        min_overlap=0.03,
        name="right_block_has_plate_footprint",
    )
    ctx.expect_origin_distance(
        left_block,
        right_block,
        axes="x",
        min_dist=0.149,
        max_dist=0.151,
        name="bearing_blocks_have_realistic_spacing",
    )
    ctx.expect_overlap(
        shaft,
        left_block,
        axes="yz",
        min_overlap=0.016,
        name="shaft_runs_through_left_block",
    )
    ctx.expect_overlap(
        shaft,
        right_block,
        axes="yz",
        min_overlap=0.016,
        name="shaft_runs_through_right_block",
    )
    ctx.expect_contact(
        shaft,
        left_block,
        elem_a=shaft_body,
        contact_tol=5e-5,
        name="shaft_body_is_supported_in_left_block",
    )
    ctx.expect_contact(
        shaft,
        right_block,
        elem_a=shaft_body,
        contact_tol=5e-5,
        name="shaft_body_is_supported_in_right_block",
    )
    ctx.expect_gap(
        shaft,
        base_plate,
        axis="z",
        positive_elem=shaft_body,
        min_gap=0.035,
        max_gap=0.037,
        name="shaft_body_clears_plate",
    )
    ctx.expect_gap(
        shaft,
        right_block,
        axis="x",
        positive_elem=end_module,
        min_gap=0.001,
        max_gap=0.006,
        name="hub_starts_outboard_of_right_block",
    )

    for angle, tag in (
        (0.0, "rest"),
        (pi / 2.0, "quarter_turn"),
        (-pi / 2.0, "downward_feature"),
        (pi, "half_turn"),
    ):
        with ctx.pose({shaft_spin: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"shaft_{tag}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"shaft_{tag}_no_floating")

    with ctx.pose({shaft_spin: -pi / 2.0}):
        ctx.expect_gap(
            shaft,
            base_plate,
            axis="z",
            positive_elem=end_module,
            min_gap=0.028,
            max_gap=0.029,
            name="end_module_clears_plate_when_rotated_down",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
