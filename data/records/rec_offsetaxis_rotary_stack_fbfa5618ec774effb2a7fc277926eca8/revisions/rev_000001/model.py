from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 1.08
BASE_WIDTH = 0.68
BASE_HEIGHT = 0.16
PEDESTAL_OUTER_RADIUS = 0.17
PEDESTAL_INNER_RADIUS = 0.09
PEDESTAL_HEIGHT = 0.045

TURNTABLE_RADIUS = 0.27
TURNTABLE_THICKNESS = 0.05
TURNTABLE_BOLT_RADIUS = 0.19
CENTER_SHAFT_RADIUS = 0.075
CENTER_SHAFT_LENGTH = 0.004

OFFSET_AXIS_X = 0.40
BRIDGE_WIDTH = 0.16
BRIDGE_START_X = 0.09
BRIDGE_END_X = 0.31
BRIDGE_SIDE_THICKNESS = 0.025
BRIDGE_SIDE_HEIGHT = 0.10
BRIDGE_TOP_THICKNESS = 0.03
BRIDGE_LOWER_THICKNESS = 0.035

FACEPLATE_RADIUS = 0.135
FACEPLATE_THICKNESS = 0.035
FACEPLATE_RIM_HEIGHT = 0.012
FACEPLATE_BOLT_RADIUS = 0.082
HOUSING_OUTER_RADIUS = 0.12
HOUSING_BORE_RADIUS = 0.058
HOUSING_HEIGHT = 0.13


def _make_base_shape() -> cq.Workplane:
    plinth = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.028)
    )

    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_OUTER_RADIUS)
        .circle(PEDESTAL_INNER_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )

    side_skids = (
        cq.Workplane("XY")
        .box(0.82, 0.16, 0.03)
        .translate((0.0, 0.23, 0.015))
        .union(
            cq.Workplane("XY")
            .box(0.82, 0.16, 0.03)
            .translate((0.0, -0.23, 0.015))
        )
    )

    return plinth.union(pedestal).union(side_skids)


def _make_bridge_carrier_shape() -> cq.Workplane:
    bridge_length = BRIDGE_END_X - BRIDGE_START_X
    bridge_center_x = 0.5 * (BRIDGE_START_X + BRIDGE_END_X)
    lower_center_z = TURNTABLE_THICKNESS + BRIDGE_LOWER_THICKNESS / 2.0
    side_center_z = TURNTABLE_THICKNESS + BRIDGE_SIDE_HEIGHT / 2.0
    top_center_z = TURNTABLE_THICKNESS + BRIDGE_SIDE_HEIGHT + BRIDGE_TOP_THICKNESS / 2.0

    turntable = (
        cq.Workplane("XY")
        .circle(TURNTABLE_RADIUS)
        .extrude(TURNTABLE_THICKNESS)
        .faces(">Z")
        .workplane()
        .polarArray(TURNTABLE_BOLT_RADIUS, 0.0, 360.0, 8)
        .hole(0.018)
    )

    center_shaft = (
        cq.Workplane("XY")
        .circle(CENTER_SHAFT_RADIUS)
        .extrude(CENTER_SHAFT_LENGTH)
        .translate((0.0, 0.0, -CENTER_SHAFT_LENGTH))
    )

    lower_bridge = (
        cq.Workplane("XY")
        .box(bridge_length, BRIDGE_WIDTH, BRIDGE_LOWER_THICKNESS)
        .translate((bridge_center_x, 0.0, lower_center_z))
    )

    side_web_y = (BRIDGE_WIDTH - BRIDGE_SIDE_THICKNESS) / 2.0
    side_web_pos = (
        cq.Workplane("XY")
        .box(bridge_length, BRIDGE_SIDE_THICKNESS, BRIDGE_SIDE_HEIGHT)
        .translate((bridge_center_x, side_web_y, side_center_z))
    )
    side_web_neg = (
        cq.Workplane("XY")
        .box(bridge_length, BRIDGE_SIDE_THICKNESS, BRIDGE_SIDE_HEIGHT)
        .translate((bridge_center_x, -side_web_y, side_center_z))
    )

    top_bridge = (
        cq.Workplane("XY")
        .box(bridge_length - 0.03, BRIDGE_WIDTH, BRIDGE_TOP_THICKNESS)
        .translate((bridge_center_x + 0.015, 0.0, top_center_z))
    )

    root_riser = (
        cq.Workplane("XY")
        .box(0.12, 0.18, 0.055)
        .translate((0.08, 0.0, TURNTABLE_THICKNESS + 0.0275))
    )

    end_housing = (
        cq.Workplane("XY")
        .circle(HOUSING_OUTER_RADIUS)
        .circle(HOUSING_BORE_RADIUS)
        .extrude(HOUSING_HEIGHT)
        .translate((OFFSET_AXIS_X, 0.0, TURNTABLE_THICKNESS))
    )

    rear_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.02, TURNTABLE_THICKNESS),
                (0.15, TURNTABLE_THICKNESS),
                (0.23, TURNTABLE_THICKNESS + 0.095),
                (0.09, TURNTABLE_THICKNESS + 0.095),
            ]
        )
        .close()
        .extrude(0.02)
        .translate((0.0, 0.055, 0.0))
    )
    front_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.02, TURNTABLE_THICKNESS),
                (0.15, TURNTABLE_THICKNESS),
                (0.23, TURNTABLE_THICKNESS + 0.095),
                (0.09, TURNTABLE_THICKNESS + 0.095),
            ]
        )
        .close()
        .extrude(-0.02)
        .translate((0.0, -0.055, 0.0))
    )

    return (
        turntable.union(center_shaft)
        .union(lower_bridge)
        .union(side_web_pos)
        .union(side_web_neg)
        .union(top_bridge)
        .union(root_riser)
        .union(end_housing)
        .union(rear_gusset)
        .union(front_gusset)
    )


def _make_faceplate_shape() -> cq.Workplane:
    plate_blank = (
        cq.Workplane("XY")
        .circle(FACEPLATE_RADIUS)
        .extrude(FACEPLATE_THICKNESS)
    )

    back_relief = cq.Workplane("XY").circle(0.088).extrude(0.008)

    plate = (
        plate_blank.cut(back_relief)
        .faces(">Z")
        .workplane()
        .polarArray(FACEPLATE_BOLT_RADIUS, 0.0, 360.0, 6)
        .hole(0.014)
    )

    rim = (
        cq.Workplane("XY")
        .circle(FACEPLATE_RADIUS)
        .circle(FACEPLATE_RADIUS - 0.022)
        .extrude(FACEPLATE_RIM_HEIGHT)
        .translate((0.0, 0.0, FACEPLATE_THICKNESS))
    )

    center_cap = (
        cq.Workplane("XY")
        .circle(0.045)
        .extrude(0.016)
        .translate((0.0, 0.0, FACEPLATE_THICKNESS))
    )

    nose_ring = (
        cq.Workplane("XY")
        .circle(0.07)
        .circle(0.052)
        .extrude(0.018)
        .translate((0.0, 0.0, FACEPLATE_THICKNESS))
    )

    return plate.union(rim).union(center_cap).union(nose_ring)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_offset_rotary_fixture")

    base_material = model.material("base_paint", rgba=(0.19, 0.20, 0.22, 1.0))
    bridge_material = model.material("bridge_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    faceplate_material = model.material("faceplate_steel", rgba=(0.68, 0.70, 0.73, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_frame"),
        material=base_material,
        name="base_shell",
    )

    bridge = model.part("bridge_carrier")
    bridge.visual(
        mesh_from_cadquery(_make_bridge_carrier_shape(), "bridge_carrier"),
        material=bridge_material,
        name="bridge_shell",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        mesh_from_cadquery(_make_faceplate_shape(), "faceplate"),
        material=faceplate_material,
        name="faceplate_shell",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.5,
            lower=-pi,
            upper=pi,
        ),
    )

    model.articulation(
        "bridge_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=faceplate,
        origin=Origin(xyz=(OFFSET_AXIS_X, 0.0, TURNTABLE_THICKNESS + HOUSING_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=2.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    bridge = object_model.get_part("bridge_carrier")
    faceplate = object_model.get_part("faceplate")
    base_joint = object_model.get_articulation("base_to_bridge")
    faceplate_joint = object_model.get_articulation("bridge_to_faceplate")

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

    joint_offset_xy = (faceplate_joint.origin.xyz[0] ** 2 + faceplate_joint.origin.xyz[1] ** 2) ** 0.5
    ctx.check(
        "parallel_vertical_revolute_axes",
        base_joint.articulation_type == ArticulationType.REVOLUTE
        and faceplate_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(base_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(faceplate_joint.axis) == (0.0, 0.0, 1.0),
        details=(
            f"base axis={base_joint.axis}, faceplate axis={faceplate_joint.axis}, "
            f"types=({base_joint.articulation_type}, {faceplate_joint.articulation_type})"
        ),
    )
    ctx.check(
        "laterally_separated_axes",
        0.36 <= joint_offset_xy <= 0.44,
        details=f"expected faceplate axis offset near 0.40 m, got {joint_offset_xy:.3f} m",
    )

    ctx.expect_contact(
        bridge,
        base,
        contact_tol=0.002,
        name="turntable_is_supported_by_base_pedestal",
    )
    ctx.expect_contact(
        faceplate,
        bridge,
        contact_tol=0.002,
        name="faceplate_is_supported_by_bridge_housing",
    )
    ctx.expect_origin_distance(
        faceplate,
        bridge,
        axes="xy",
        min_dist=0.36,
        max_dist=0.44,
        name="faceplate_axis_is_offset_from_turntable_axis",
    )
    ctx.expect_gap(
        faceplate,
        base,
        axis="z",
        min_gap=0.10,
        name="offset_faceplate_clears_the_fixed_base",
    )

    with ctx.pose({base_joint: 1.0, faceplate_joint: -1.2}):
        ctx.expect_contact(
            faceplate,
            bridge,
            contact_tol=0.002,
            name="faceplate_remains_supported_when_both_axes_rotate",
        )
        ctx.expect_origin_distance(
            faceplate,
            bridge,
            axes="xy",
            min_dist=0.36,
            max_dist=0.44,
            name="offset_is_preserved_in_rotated_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
