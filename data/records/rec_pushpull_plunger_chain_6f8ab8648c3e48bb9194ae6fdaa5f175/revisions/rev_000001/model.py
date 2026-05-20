from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


PLUNGER_SHAFT_RADIUS = 0.0042
GUIDE_OUTER_RADIUS = 0.010
GUIDE_HOLE_RADIUS = 0.0050
PLUNGER_COLLAR_RADIUS = 0.0078
PIN_RADIUS = 0.0030
LEVER_THICKNESS = 0.008
CLEVIS_GAP = 0.012
CLEVIS_CHEEK_THICKNESS = 0.004
CLEVIS_PIVOT_X = 0.140
PLUNGER_Z = 0.006
PIVOT_Z = 0.018
PLUNGER_REST_TIP_X = 0.1245


def _guide_ring(x_start: float, length: float = 0.010) -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(GUIDE_OUTER_RADIUS).extrude(length).translate((x_start, 0.0, PLUNGER_Z))
    hole = (
        cq.Workplane("YZ")
        .circle(GUIDE_HOLE_RADIUS)
        .extrude(length + 0.004)
        .translate((x_start - 0.002, 0.0, PLUNGER_Z))
    )
    return outer.cut(hole)


def _make_housing_shape() -> cq.Workplane:
    base_plate = cq.Workplane("XY").box(0.160, 0.050, 0.006).translate((0.080, 0.0, -0.018))

    rear_support = cq.Workplane("XY").box(0.010, 0.020, 0.012).translate((0.005, 0.0, -0.012))
    front_support = cq.Workplane("XY").box(0.010, 0.020, 0.012).translate((0.065, 0.0, -0.012))
    tie_bar = cq.Workplane("XY").box(0.060, 0.020, 0.006).translate((0.035, 0.0, -0.015))

    rear_guide = _guide_ring(0.000).union(rear_support)
    front_guide = _guide_ring(0.060).union(front_support)

    left_rib = cq.Workplane("XY").box(0.070, 0.004, 0.020).translate((0.105, -0.008, -0.006))
    right_rib = cq.Workplane("XY").box(0.070, 0.004, 0.020).translate((0.105, 0.008, -0.006))

    left_cheek = cq.Workplane("XY").box(0.016, CLEVIS_CHEEK_THICKNESS, 0.016).translate(
        (CLEVIS_PIVOT_X, -(CLEVIS_GAP / 2.0 + CLEVIS_CHEEK_THICKNESS / 2.0), PIVOT_Z)
    )
    right_cheek = cq.Workplane("XY").box(0.016, CLEVIS_CHEEK_THICKNESS, 0.016).translate(
        (CLEVIS_PIVOT_X, CLEVIS_GAP / 2.0 + CLEVIS_CHEEK_THICKNESS / 2.0, PIVOT_Z)
    )

    pin_shaft = cq.Workplane("XZ").circle(PIN_RADIUS).extrude(CLEVIS_GAP + 2 * CLEVIS_CHEEK_THICKNESS).translate(
        (CLEVIS_PIVOT_X, -(CLEVIS_GAP / 2.0 + CLEVIS_CHEEK_THICKNESS), PIVOT_Z)
    )

    return (
        base_plate.union(rear_guide)
        .union(front_guide)
        .union(tie_bar)
        .union(left_rib)
        .union(right_rib)
        .union(left_cheek)
        .union(right_cheek)
        .union(pin_shaft)
    )


def _make_plunger_shape() -> cq.Workplane:
    shaft = cq.Workplane("YZ").circle(PLUNGER_SHAFT_RADIUS).extrude(PLUNGER_REST_TIP_X)
    collar = cq.Workplane("YZ").circle(PLUNGER_COLLAR_RADIUS).extrude(0.006).translate((-0.006, 0.0, 0.0))
    knob = cq.Workplane("YZ").circle(0.0105).extrude(0.012).translate((-0.018, 0.0, 0.0))
    return shaft.union(collar).union(knob)


def _make_lever_shape() -> cq.Workplane:
    profile = (
        cq.Workplane("XZ")
        .moveTo(-0.0155, -0.004)
        .lineTo(-0.006, -0.009)
        .lineTo(0.004, -0.004)
        .lineTo(0.012, 0.010)
        .lineTo(0.028, 0.021)
        .lineTo(0.038, 0.021)
        .lineTo(0.038, 0.029)
        .lineTo(0.024, 0.031)
        .lineTo(0.010, 0.024)
        .lineTo(-0.002, 0.014)
        .lineTo(-0.0155, 0.004)
        .close()
        .extrude(LEVER_THICKNESS)
        .translate((0.0, -LEVER_THICKNESS / 2.0, 0.0))
    )
    hub = cq.Workplane("XZ").circle(0.007).extrude(LEVER_THICKNESS).translate((0.0, -LEVER_THICKNESS / 2.0, 0.0))
    hole = cq.Workplane("XZ").circle(PIN_RADIUS + 0.0004).extrude(LEVER_THICKNESS + 0.002).translate(
        (0.0, -(LEVER_THICKNESS + 0.002) / 2.0, 0.0)
    )
    return profile.union(hub).cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_pull_plunger_chain")

    housing_mat = model.material("housing_mat", rgba=(0.40, 0.43, 0.46, 1.0))
    plunger_mat = model.material("plunger_mat", rgba=(0.78, 0.80, 0.82, 1.0))
    lever_mat = model.material("lever_mat", rgba=(0.14, 0.15, 0.16, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.160, 0.050, 0.006)),
        origin=Origin(xyz=(0.080, 0.0, -0.018)),
        material=housing_mat,
        name="base_plate",
    )
    housing.visual(
        Box((0.120, 0.004, 0.030)),
        origin=Origin(xyz=(0.060, -0.010, -0.003)),
        material=housing_mat,
        name="left_rail",
    )
    housing.visual(
        Box((0.120, 0.004, 0.030)),
        origin=Origin(xyz=(0.060, 0.010, -0.003)),
        material=housing_mat,
        name="right_rail",
    )
    housing.visual(
        Box((0.004, 0.004, 0.014)),
        origin=Origin(xyz=(0.002, -0.006, PLUNGER_Z)),
        material=housing_mat,
        name="rear_stop_left",
    )
    housing.visual(
        Box((0.004, 0.004, 0.014)),
        origin=Origin(xyz=(0.002, 0.006, PLUNGER_Z)),
        material=housing_mat,
        name="rear_stop_right",
    )
    housing.visual(
        Box((0.032, 0.004, 0.038)),
        origin=Origin(xyz=(0.126, -0.012, -0.001)),
        material=housing_mat,
        name="left_post",
    )
    housing.visual(
        Box((0.032, 0.004, 0.038)),
        origin=Origin(xyz=(0.126, 0.012, -0.001)),
        material=housing_mat,
        name="right_post",
    )
    housing.visual(
        Box((0.016, 0.004, 0.016)),
        origin=Origin(xyz=(CLEVIS_PIVOT_X, -(CLEVIS_GAP / 2.0 + CLEVIS_CHEEK_THICKNESS / 2.0), PIVOT_Z)),
        material=housing_mat,
        name="left_cheek",
    )
    housing.visual(
        Box((0.016, 0.004, 0.016)),
        origin=Origin(xyz=(CLEVIS_PIVOT_X, CLEVIS_GAP / 2.0 + CLEVIS_CHEEK_THICKNESS / 2.0, PIVOT_Z)),
        material=housing_mat,
        name="right_cheek",
    )
    housing.visual(
        Cylinder(radius=PIN_RADIUS, length=CLEVIS_GAP + 2 * CLEVIS_CHEEK_THICKNESS),
        origin=Origin(xyz=(CLEVIS_PIVOT_X, 0.0, PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=housing_mat,
        name="pivot_pin",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=PLUNGER_SHAFT_RADIUS, length=PLUNGER_REST_TIP_X),
        origin=Origin(xyz=(PLUNGER_REST_TIP_X / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=plunger_mat,
        name="shaft",
    )
    plunger.visual(
        Cylinder(radius=PLUNGER_COLLAR_RADIUS, length=0.006),
        origin=Origin(xyz=(-0.003, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=plunger_mat,
        name="collar",
    )
    plunger.visual(
        Cylinder(radius=0.0105, length=0.012),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=plunger_mat,
        name="knob",
    )

    lever = model.part("front_lever")
    lever.visual(
        Cylinder(radius=0.007, length=LEVER_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=lever_mat,
        name="hub",
    )
    lever.visual(
        Box((0.015, LEVER_THICKNESS, 0.020)),
        origin=Origin(xyz=(-0.008, 0.0, -0.010)),
        material=lever_mat,
        name="tail",
    )
    lever.visual(
        Box((0.028, LEVER_THICKNESS, 0.010)),
        origin=Origin(xyz=(0.014, 0.0, 0.010)),
        material=lever_mat,
        name="arm",
    )
    lever.visual(
        Box((0.010, 0.010, 0.014)),
        origin=Origin(xyz=(0.033, 0.0, 0.018)),
        material=lever_mat,
        name="pad",
    )

    model.articulation(
        "housing_to_plunger",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, PLUNGER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.08,
            lower=-0.010,
            upper=0.0,
        ),
    )

    model.articulation(
        "housing_to_front_lever",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lever,
        origin=Origin(xyz=(CLEVIS_PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    plunger = object_model.get_part("plunger")
    lever = object_model.get_part("front_lever")
    plunger_slide = object_model.get_articulation("housing_to_plunger")
    lever_pivot = object_model.get_articulation("housing_to_front_lever")

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
        housing,
        lever,
        elem_a="pivot_pin",
        elem_b="hub",
        reason="The lever rotates on the integral transverse pivot pin captured by the housing.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "parts_present",
        {part.name for part in object_model.parts} == {"housing", "plunger", "front_lever"},
        details="Expected housing, plunger, and front_lever parts.",
    )
    ctx.check(
        "plunger_is_prismatic_on_housing_axis",
        plunger_slide.joint_type == ArticulationType.PRISMATIC and plunger_slide.axis == (1.0, 0.0, 0.0),
        details=f"Expected prismatic +X slide, got type={plunger_slide.joint_type} axis={plunger_slide.axis}.",
    )
    ctx.check(
        "lever_is_revolute_on_transverse_pin_axis",
        lever_pivot.joint_type == ArticulationType.REVOLUTE
        and lever_pivot.axis == (0.0, 1.0, 0.0)
        and lever_pivot.origin.xyz == (CLEVIS_PIVOT_X, 0.0, PIVOT_Z),
        details=(
            f"Expected revolute +Y pin axis at {(CLEVIS_PIVOT_X, 0.0, PIVOT_Z)}, "
            f"got type={lever_pivot.joint_type} origin={lever_pivot.origin.xyz} axis={lever_pivot.axis}."
        ),
    )

    ctx.expect_contact(
        plunger,
        housing,
        contact_tol=5e-4,
        name="plunger_collar_seats_against_housing_stop",
    )
    ctx.expect_contact(
        plunger,
        lever,
        contact_tol=5e-4,
        name="plunger_tip_drives_lever_tail_in_rest_pose",
    )

    with ctx.pose({plunger_slide: -0.008}):
        ctx.expect_gap(
            lever,
            plunger,
            axis="x",
            min_gap=0.0075,
            max_gap=0.0085,
            name="retracted_plunger_opens_a_small_drive_gap",
        )

    with ctx.pose({plunger_slide: -0.008, lever_pivot: 0.45}):
        ctx.fail_if_parts_overlap_in_current_pose(name="retracted_open_pose_remains_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
