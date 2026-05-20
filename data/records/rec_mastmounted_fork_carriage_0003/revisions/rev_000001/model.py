from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

MAST_WIDTH = 0.30
MAST_THICKNESS = 0.016
MAST_HEIGHT = 0.74

RAIL_WIDTH = 0.040
RAIL_DEPTH = 0.030
RAIL_HEIGHT = 0.62
RAIL_CENTER_X = 0.10
RAIL_CENTER_Y = (MAST_THICKNESS / 2.0) + (RAIL_DEPTH / 2.0)
RAIL_BASE_Z = 0.05
RAIL_CENTER_Z = RAIL_BASE_Z + (RAIL_HEIGHT / 2.0)

CARRIAGE_WIDTH = 0.30
CARRIAGE_HEIGHT = 0.20
CARRIAGE_REST_Z = 0.18
CARRIAGE_TRAVEL = 0.30
CARRIAGE_FRONT_FACE_Y = 0.068

TINE_SPACING = 0.19
TINE_WIDTH = 0.042
TINE_LENGTH = 0.312
TINE_THICKNESS = 0.020

def _make_carriage_shape() -> cq.Workplane:
    left_upper_shoe = (
        cq.Workplane("XY")
        .box(0.052, 0.014, 0.052)
        .translate((-RAIL_CENTER_X, 0.007, 0.060))
    )
    left_lower_shoe = (
        cq.Workplane("XY")
        .box(0.052, 0.014, 0.052)
        .translate((-RAIL_CENTER_X, 0.007, -0.060))
    )
    right_upper_shoe = (
        cq.Workplane("XY")
        .box(0.052, 0.014, 0.052)
        .translate((RAIL_CENTER_X, 0.007, 0.060))
    )
    right_lower_shoe = (
        cq.Workplane("XY")
        .box(0.052, 0.014, 0.052)
        .translate((RAIL_CENTER_X, 0.007, -0.060))
    )

    left_web = (
        cq.Workplane("XY")
        .box(0.040, 0.050, 0.178)
        .translate((-0.112, 0.025, 0.0))
    )
    right_web = (
        cq.Workplane("XY")
        .box(0.040, 0.050, 0.178)
        .translate((0.112, 0.025, 0.0))
    )
    top_beam = (
        cq.Workplane("XY")
        .box(CARRIAGE_WIDTH, 0.026, 0.032)
        .translate((0.0, 0.055, 0.084))
    )
    center_plate = (
        cq.Workplane("XY")
        .box(0.180, 0.020, 0.110)
        .translate((0.0, 0.048, -0.004))
    )
    lower_beam = (
        cq.Workplane("XY")
        .box(CARRIAGE_WIDTH, 0.026, 0.040)
        .translate((0.0, 0.055, -0.080))
    )
    tine_bar = (
        cq.Workplane("XY")
        .box(0.240, 0.020, 0.026)
        .translate((0.0, 0.058, -0.097))
    )

    return (
        left_upper_shoe.union(left_lower_shoe)
        .union(right_upper_shoe)
        .union(right_lower_shoe)
        .union(left_web)
        .union(right_web)
        .union(top_beam)
        .union(center_plate)
        .union(lower_beam)
        .union(tine_bar)
    )


def _make_tine_shape() -> cq.Workplane:
    heel = (
        cq.Workplane("XY")
        .box(TINE_WIDTH, 0.022, 0.070)
        .translate((0.0, 0.011, 0.020))
    )
    blade = (
        cq.Workplane("XY")
        .box(TINE_WIDTH - 0.002, TINE_LENGTH - 0.022, TINE_THICKNESS)
        .translate((0.0, 0.167, -0.010))
    )
    tip = (
        cq.Workplane("XY")
        .box(TINE_WIDTH - 0.008, 0.040, 0.012)
        .translate((0.0, TINE_LENGTH - 0.020, -0.006))
    )
    return heel.union(blade).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_fork_carriage", assets=ASSETS)

    mast_gray = model.material("mast_gray", rgba=(0.30, 0.33, 0.36, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    tine_steel = model.material("tine_steel", rgba=(0.40, 0.42, 0.45, 1.0))

    carriage_mesh = mesh_from_cadquery(
        _make_carriage_shape(),
        "fork_carriage.obj",
        assets=ASSETS,
    )
    tine_mesh = mesh_from_cadquery(
        _make_tine_shape(),
        "fork_tine.obj",
        assets=ASSETS,
    )

    mast = model.part("mast")
    mast.visual(
        Box((MAST_WIDTH, MAST_THICKNESS, MAST_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT / 2.0)),
        material=mast_gray,
        name="mast_backplate",
    )
    mast.visual(
        Box((RAIL_WIDTH, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(-RAIL_CENTER_X, RAIL_CENTER_Y, RAIL_CENTER_Z)),
        material=mast_gray,
        name="left_rail",
    )
    mast.visual(
        Box((RAIL_WIDTH, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(RAIL_CENTER_X, RAIL_CENTER_Y, RAIL_CENTER_Z)),
        material=mast_gray,
        name="right_rail",
    )
    mast.inertial = Inertial.from_geometry(
        Box((MAST_WIDTH, 0.060, MAST_HEIGHT)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.014, MAST_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(carriage_mesh, material=carriage_gray, name="carriage_frame")
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_WIDTH, 0.068, CARRIAGE_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.034, 0.0)),
    )

    left_tine = model.part("left_tine")
    left_tine.visual(tine_mesh, material=tine_steel, name="left_tine_body")
    left_tine.inertial = Inertial.from_geometry(
        Box((TINE_WIDTH, TINE_LENGTH, 0.070)),
        mass=6.5,
        origin=Origin(xyz=(0.0, TINE_LENGTH / 2.0, 0.010)),
    )

    right_tine = model.part("right_tine")
    right_tine.visual(tine_mesh, material=tine_steel, name="right_tine_body")
    right_tine.inertial = Inertial.from_geometry(
        Box((TINE_WIDTH, TINE_LENGTH, 0.070)),
        mass=6.5,
        origin=Origin(xyz=(0.0, TINE_LENGTH / 2.0, 0.010)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, RAIL_CENTER_Y + (RAIL_DEPTH / 2.0), CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.40,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_left_tine",
        ArticulationType.FIXED,
        parent=carriage,
        child=left_tine,
        origin=Origin(xyz=(-TINE_SPACING / 2.0, CARRIAGE_FRONT_FACE_Y, -0.165)),
    )
    model.articulation(
        "carriage_to_right_tine",
        ArticulationType.FIXED,
        parent=carriage,
        child=right_tine,
        origin=Origin(xyz=(TINE_SPACING / 2.0, CARRIAGE_FRONT_FACE_Y, -0.165)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    left_tine = object_model.get_part("left_tine")
    right_tine = object_model.get_part("right_tine")
    mast_to_carriage = object_model.get_articulation("mast_to_carriage")
    carriage_frame = carriage.get_visual("carriage_frame")
    left_rail = mast.get_visual("left_rail")
    right_rail = mast.get_visual("right_rail")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        carriage,
        mast,
        elem_a=carriage_frame,
        elem_b=left_rail,
        reason="Sliding guide shoe runs in zero-clearance contact on the left mast rail face.",
    )
    ctx.allow_overlap(
        carriage,
        mast,
        elem_a=carriage_frame,
        elem_b=right_rail,
        reason="Sliding guide shoe runs in zero-clearance contact on the right mast rail face.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "required_parts_present",
        all(part is not None for part in (mast, carriage, left_tine, right_tine)),
        "mast, carriage, and both tines must exist",
    )
    ctx.check(
        "mast_prismatic_joint_present",
        mast_to_carriage is not None,
        "carriage must slide on a prismatic mast joint",
    )

    ctx.expect_contact(
        carriage,
        mast,
        contact_tol=5e-4,
        elem_a=carriage_frame,
        elem_b=left_rail,
        name="carriage_guides_contact_rails_at_rest",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="xz",
        min_overlap=0.16,
        name="carriage_spans_the_mast_rail_zone",
    )
    ctx.expect_contact(
        left_tine,
        carriage,
        contact_tol=5e-4,
        name="left_tine_is_seated_on_carriage",
    )
    ctx.expect_contact(
        right_tine,
        carriage,
        contact_tol=5e-4,
        name="right_tine_is_seated_on_carriage",
    )
    ctx.expect_gap(
        right_tine,
        left_tine,
        axis="x",
        min_gap=0.10,
        name="tines_have_clear_spacing_between_them",
    )
    ctx.expect_origin_distance(
        left_tine,
        right_tine,
        axes="x",
        min_dist=0.18,
        max_dist=0.20,
        name="tine_centers_are_realistically_spaced",
    )

    with ctx.pose({mast_to_carriage: 0.0}):
        ctx.expect_origin_gap(
            carriage,
            mast,
            axis="z",
            min_gap=0.17,
            max_gap=0.19,
            name="carriage_starts_low_on_the_mast",
        )
        ctx.expect_contact(
            carriage,
            mast,
            contact_tol=5e-4,
            name="carriage_remains_guided_at_rest",
        )

    with ctx.pose({mast_to_carriage: CARRIAGE_TRAVEL}):
        ctx.expect_origin_gap(
            carriage,
            mast,
            axis="z",
            min_gap=0.47,
            max_gap=0.49,
            name="carriage_raises_by_about_300mm",
        )
        ctx.expect_contact(
            carriage,
            mast,
            contact_tol=5e-4,
            name="carriage_remains_guided_when_raised",
        )
        ctx.expect_contact(
            left_tine,
            carriage,
            contact_tol=5e-4,
            name="left_tine_stays_rigid_in_raised_pose",
        )
        ctx.expect_contact(
            right_tine,
            carriage,
            contact_tol=5e-4,
            name="right_tine_stays_rigid_in_raised_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
