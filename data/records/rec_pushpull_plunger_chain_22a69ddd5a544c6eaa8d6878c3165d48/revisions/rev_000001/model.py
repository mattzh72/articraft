from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


PLATE_W = 0.140
PLATE_H = 0.220
PLATE_T = 0.008
PLATE_CORNER_R = 0.010
MOUNT_HOLE_R = 0.005

TUBE_Z = 0.020
TUBE_OUTER_R = 0.014
TUBE_INNER_R = 0.010
TUBE_LEN = 0.018
TUBE_FRONT_Y = PLATE_T + TUBE_LEN
TUBE_BOSS_R = 0.020
TUBE_BOSS_T = 0.008

CLEVIS_GAP = 0.012
EAR_T = 0.008
CLEVIS_W = CLEVIS_GAP + (2.0 * EAR_T)
CLEVIS_D = 0.024
PIVOT_Y = 0.082
CLEVIS_REAR_Y = PIVOT_Y - (CLEVIS_D / 2.0)
CLEVIS_FRONT_Y = CLEVIS_REAR_Y + CLEVIS_D
CLEVIS_H = 0.040
CLEVIS_BRIDGE_T = 0.006
PIVOT_Z = 0.070
PIVOT_PIN_R = 0.0035

NECK_W = 0.022
NECK_T = 0.016
NECK_H = 0.050
NECK_Y = CLEVIS_REAR_Y - (NECK_T / 2.0)
NECK_Z = 0.045

LEVER_T = 0.008
LEVER_D = 0.010
LEVER_LEN = 0.088
FOLLOWER_R = 0.007
FOLLOWER_Y = 0.0
FOLLOWER_Z = TUBE_Z - PIVOT_Z

ROD_SHAFT_R = 0.0085
ROD_LEN = PIVOT_Y - PLATE_T
ROD_COLLAR_T = 0.004
ROD_COLLAR_R = 0.0155
ROD_COLLAR_Y = ROD_COLLAR_T / 2.0

OUTPUT_PAD_Y = 0.001
OUTPUT_PAD_Z = -0.080
OUTPUT_PAD_D = 0.014
OUTPUT_PAD_H = 0.016
INPUT_PAD_D = 0.010
INPUT_PAD_H = 0.014
INPUT_PAD_Y = 0.005
INPUT_PAD_Z = FOLLOWER_Z
LEVER_BODY_Y = 0.010


def _y_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    )


def _y_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    outer = _y_cylinder(outer_radius, length)
    inner = _y_cylinder(inner_radius, length + 0.002)
    return outer.cut(inner)


def _x_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )


def _mount_hole_points() -> list[tuple[float, float]]:
    return [
        (-0.048, -0.078),
        (0.048, -0.078),
        (-0.048, 0.078),
        (0.048, 0.078),
    ]


def _make_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_W, PLATE_T, PLATE_H)
        .translate((0.0, PLATE_T / 2.0, 0.0))
        .edges("|Y")
        .fillet(PLATE_CORNER_R)
    )
    mount_holes = (
        cq.Workplane("XZ")
        .pushPoints(_mount_hole_points())
        .circle(MOUNT_HOLE_R)
        .extrude(PLATE_T + 0.002)
        .translate((0.0, -0.001, 0.0))
    )
    through_bore = (
        cq.Workplane("XZ")
        .circle(TUBE_INNER_R)
        .extrude(PLATE_T + 0.002)
        .translate((0.0, -0.001, TUBE_Z))
    )
    return plate.cut(mount_holes).cut(through_bore)


def _make_guide_shape() -> cq.Workplane:
    boss = _y_tube(TUBE_BOSS_R, TUBE_INNER_R, TUBE_BOSS_T).translate(
        (0.0, TUBE_BOSS_T / 2.0, 0.0)
    )
    tube = _y_tube(TUBE_OUTER_R, TUBE_INNER_R, TUBE_LEN).translate(
        (0.0, TUBE_LEN / 2.0, 0.0)
    )
    return boss.union(tube)


def _make_pushrod_shape() -> cq.Workplane:
    shaft = _y_cylinder(ROD_SHAFT_R, ROD_LEN).translate((0.0, ROD_LEN / 2.0, 0.0))
    collar = _y_cylinder(ROD_COLLAR_R, ROD_COLLAR_T).translate(
        (0.0, TUBE_LEN + (ROD_COLLAR_T / 2.0), 0.0)
    )
    return shaft.union(collar)


def _make_clevis_shape() -> cq.Workplane:
    ear_offset_x = (CLEVIS_GAP / 2.0) + (EAR_T / 2.0)
    left_ear = (
        cq.Workplane("XY")
        .box(EAR_T, CLEVIS_D, CLEVIS_H)
        .translate((-ear_offset_x, 0.0, 0.0))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(EAR_T, CLEVIS_D, CLEVIS_H)
        .translate((ear_offset_x, 0.0, 0.0))
    )
    top_tie = cq.Workplane("XY").box(CLEVIS_W, 0.008, 0.010).translate(
        (0.0, -0.002, 0.015)
    )
    arm_len = PIVOT_Y - PLATE_T - (CLEVIS_D / 2.0)
    arm_center_y = -((CLEVIS_D / 2.0) + arm_len / 2.0)
    top_strap = cq.Workplane("XY").box(NECK_W, arm_len, 0.012).translate(
        (0.0, arm_center_y, 0.018)
    )
    mount_pad = cq.Workplane("XY").box(NECK_W + 0.014, 0.006, 0.028).translate(
        (0.0, -(PIVOT_Y - PLATE_T) + 0.003, 0.010)
    )
    pivot_bore = _x_cylinder(PIVOT_PIN_R, CLEVIS_W + 0.004)
    return left_ear.union(right_ear).union(top_tie).union(top_strap).union(mount_pad).cut(
        pivot_bore
    )


def _make_lever_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(LEVER_T, LEVER_D, LEVER_LEN)
        .translate((0.0, LEVER_BODY_Y, -(LEVER_LEN / 2.0)))
        .edges("|Z")
        .fillet(0.002)
    )
    hub = cq.Workplane("XY").box(0.010, 0.012, 0.016).translate((0.0, LEVER_BODY_Y, -0.006))
    return body.union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_pushrod_lever")

    model.material("painted_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("zinc_plate", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("bright_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("oxide_black", rgba=(0.12, 0.13, 0.14, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_make_plate_shape(), "backplate_plate"),
        material="painted_steel",
        name="plate",
    )
    backplate.inertial = Inertial.from_geometry(
        Box((PLATE_W, PLATE_T, PLATE_H)),
        mass=2.0,
        origin=Origin(xyz=(0.0, PLATE_T / 2.0, 0.0)),
    )

    guide = model.part("guide")
    guide.visual(
        mesh_from_cadquery(_make_guide_shape(), "guide_tube"),
        material="painted_steel",
        name="tube",
    )

    clevis = model.part("clevis")
    clevis.visual(
        mesh_from_cadquery(_make_clevis_shape(), "front_clevis"),
        material="painted_steel",
        name="clevis_body",
    )

    pushrod = model.part("pushrod")
    pushrod.visual(
        mesh_from_cadquery(_make_pushrod_shape(), "pushrod_body"),
        material="bright_steel",
        name="rod_body",
    )
    pushrod.inertial = Inertial.from_geometry(
        Cylinder(radius=ROD_COLLAR_R, length=ROD_LEN),
        mass=0.18,
        origin=Origin(xyz=(0.0, ROD_LEN / 2.0, 0.0), rpy=(-1.57079632679, 0.0, 0.0)),
    )

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_make_lever_body_shape(), "lever_body"),
        material="zinc_plate",
        name="lever_body",
    )
    lever.visual(
        Box((LEVER_T, INPUT_PAD_D, INPUT_PAD_H)),
        origin=Origin(xyz=(0.0, INPUT_PAD_Y, INPUT_PAD_Z)),
        material="oxide_black",
        name="input_pad",
    )
    lever.visual(
        Box((LEVER_T, OUTPUT_PAD_D, OUTPUT_PAD_H)),
        origin=Origin(xyz=(0.0, OUTPUT_PAD_Y, OUTPUT_PAD_Z)),
        material="zinc_plate",
        name="output_pad",
    )
    lever.inertial = Inertial.from_geometry(
        Box((CLEVIS_W, OUTPUT_PAD_D, LEVER_LEN)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -(LEVER_LEN / 2.0))),
    )

    model.articulation(
        "backplate_to_guide",
        ArticulationType.FIXED,
        parent=backplate,
        child=guide,
        origin=Origin(xyz=(0.0, PLATE_T, TUBE_Z)),
    )
    model.articulation(
        "backplate_to_clevis",
        ArticulationType.FIXED,
        parent=backplate,
        child=clevis,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
    )
    model.articulation(
        "pushrod_slide",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=pushrod,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.008,
            effort=80.0,
            velocity=0.040,
        ),
    )
    model.articulation(
        "lever_hinge",
        ArticulationType.REVOLUTE,
        parent=clevis,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.95,
            effort=15.0,
            velocity=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    guide = object_model.get_part("guide")
    clevis = object_model.get_part("clevis")
    pushrod = object_model.get_part("pushrod")
    lever = object_model.get_part("lever")
    pushrod_slide = object_model.get_articulation("pushrod_slide")
    lever_hinge = object_model.get_articulation("lever_hinge")

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
    ctx.allow_overlap(
        pushrod,
        guide,
        reason="The pushrod runs concentrically inside the hollow guide tube; nested coaxial travel is intentional.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "mechanism_parts_present",
        all(part is not None for part in (backplate, guide, clevis, pushrod, lever)),
        "Expected backplate, guide, clevis, pushrod, and lever parts.",
    )
    ctx.check(
        "joint_axes_match_mechanism",
        tuple(pushrod_slide.axis) == (0.0, 1.0, 0.0)
        and tuple(lever_hinge.axis) == (1.0, 0.0, 0.0)
        and isclose(pushrod_slide.motion_limits.upper or 0.0, 0.008, abs_tol=1e-9)
        and isclose(lever_hinge.motion_limits.upper or 0.0, 0.95, abs_tol=1e-9),
        "Expected a forward sliding pushrod and an x-axis output hinge with realistic travel.",
    )
    ctx.expect_contact(
        backplate,
        guide,
        name="guide_is_mounted_on_backplate",
    )
    ctx.expect_contact(
        backplate,
        clevis,
        name="clevis_is_mounted_on_backplate",
    )
    ctx.expect_overlap(
        lever,
        clevis,
        axes="xy",
        min_overlap=0.006,
        name="lever_sits_within_clevis_span",
    )
    ctx.expect_contact(
        pushrod,
        lever,
        elem_b="input_pad",
        name="pushrod_meets_follower_at_rest",
    )

    with ctx.pose({lever_hinge: 0.80}):
        ctx.expect_gap(
            lever,
            backplate,
            axis="y",
            positive_elem="output_pad",
            min_gap=0.012,
            name="output_pad_swings_forward_when_hinge_opens",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
