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

HOUSING_X = 0.080
HOUSING_Y = 0.042
HOUSING_Z = 0.052

RAIL_X = 0.150
RAIL_Y = 0.012
RAIL_Z = 0.008
RAIL_FRONT_CENTER_Y = 0.014
UPPER_RAIL_Z = 0.036
LOWER_RAIL_Z = 0.016

OPEN_OFFSET_X = 0.043
TRAVEL = 0.035

CARRIAGE_X = 0.022
CARRIAGE_Y = 0.016
CARRIAGE_Z = 0.028
PAD_Y = 0.004
PAD_Z = 0.010
PAD_CENTER_Y = 0.022

FINGER_X = 0.004
FINGER_Y = 0.012
FINGER_Z = 0.046
FINGER_CENTER_Y = 0.034
FINGER_CENTER_Z = 0.053
FINGER_OFFSET_X = 0.006


def _housing_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(HOUSING_X, HOUSING_Y, HOUSING_Z, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0025)
    )
    front_window = (
        cq.Workplane("XY")
        .box(0.058, 0.020, 0.022, centered=(True, True, False))
        .translate((0.0, HOUSING_Y / 2 - 0.010, 0.014))
    )
    throat_relief = (
        cq.Workplane("XY")
        .box(0.026, 0.028, 0.026, centered=(True, True, False))
        .translate((0.0, HOUSING_Y / 2 - 0.014, 0.026))
    )
    top_pocket = (
        cq.Workplane("XY")
        .box(0.036, 0.024, 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, HOUSING_Z - 0.006))
    )
    return body.cut(front_window).cut(throat_relief).cut(top_pocket)


def _carriage_shape() -> cq.Workplane:
    carriage = (
        cq.Workplane("XY")
        .box(CARRIAGE_X, CARRIAGE_Y, CARRIAGE_Z, centered=(True, True, False))
        .translate((0.0, 0.032, 0.010))
    )
    clamp_cap = (
        cq.Workplane("XY")
        .box(CARRIAGE_X, 0.010, 0.018, centered=(True, True, False))
        .translate((0.0, 0.039, 0.030))
    )
    return (carriage.union(clamp_cap)).edges("|Z").fillet(0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_slide_gripper", assets=ASSETS)

    anodized_black = model.material("anodized_black", rgba=(0.16, 0.17, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.38, 0.40, 0.43, 1.0))
    jaw_gray = model.material("jaw_gray", rgba=(0.47, 0.49, 0.52, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing_body.obj", assets=ASSETS),
        name="housing_body",
        material=anodized_black,
    )
    housing.visual(
        Box((RAIL_X, RAIL_Y, RAIL_Z)),
        origin=Origin(xyz=(0.0, RAIL_FRONT_CENTER_Y, UPPER_RAIL_Z)),
        name="upper_rail",
        material=steel,
    )
    housing.visual(
        Box((RAIL_X, RAIL_Y, RAIL_Z)),
        origin=Origin(xyz=(0.0, RAIL_FRONT_CENTER_Y, LOWER_RAIL_Z)),
        name="lower_rail",
        material=steel,
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_X, HOUSING_Y, HOUSING_Z)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_Z / 2)),
    )

    left_carriage = model.part("left_carriage")
    left_carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "left_carriage.obj", assets=ASSETS),
        name="body",
        material=dark_steel,
    )
    left_carriage.visual(
        Box((CARRIAGE_X, PAD_Y, PAD_Z)),
        origin=Origin(xyz=(0.0, PAD_CENTER_Y, UPPER_RAIL_Z)),
        name="upper_pad",
        material=jaw_gray,
    )
    left_carriage.visual(
        Box((CARRIAGE_X, PAD_Y, PAD_Z)),
        origin=Origin(xyz=(0.0, PAD_CENTER_Y, LOWER_RAIL_Z)),
        name="lower_pad",
        material=jaw_gray,
    )
    left_carriage.visual(
        Box((FINGER_X, FINGER_Y, FINGER_Z)),
        origin=Origin(xyz=(FINGER_OFFSET_X, FINGER_CENTER_Y, FINGER_CENTER_Z)),
        name="finger",
        material=jaw_gray,
    )
    left_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_X, 0.028, 0.072)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.030, 0.040)),
    )

    right_carriage = model.part("right_carriage")
    right_carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "right_carriage.obj", assets=ASSETS),
        name="body",
        material=dark_steel,
    )
    right_carriage.visual(
        Box((CARRIAGE_X, PAD_Y, PAD_Z)),
        origin=Origin(xyz=(0.0, PAD_CENTER_Y, UPPER_RAIL_Z)),
        name="upper_pad",
        material=jaw_gray,
    )
    right_carriage.visual(
        Box((CARRIAGE_X, PAD_Y, PAD_Z)),
        origin=Origin(xyz=(0.0, PAD_CENTER_Y, LOWER_RAIL_Z)),
        name="lower_pad",
        material=jaw_gray,
    )
    right_carriage.visual(
        Box((FINGER_X, FINGER_Y, FINGER_Z)),
        origin=Origin(xyz=(-FINGER_OFFSET_X, FINGER_CENTER_Y, FINGER_CENTER_Z)),
        name="finger",
        material=jaw_gray,
    )
    right_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_X, 0.028, 0.072)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.030, 0.040)),
    )

    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_carriage,
        origin=Origin(xyz=(-OPEN_OFFSET_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=TRAVEL),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_carriage,
        origin=Origin(xyz=(OPEN_OFFSET_X, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    left_carriage = object_model.get_part("left_carriage")
    right_carriage = object_model.get_part("right_carriage")
    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")

    upper_rail = housing.get_visual("upper_rail")
    lower_rail = housing.get_visual("lower_rail")
    left_upper_pad = left_carriage.get_visual("upper_pad")
    left_lower_pad = left_carriage.get_visual("lower_pad")
    right_upper_pad = right_carriage.get_visual("upper_pad")
    right_lower_pad = right_carriage.get_visual("lower_pad")
    left_finger = left_carriage.get_visual("finger")
    right_finger = right_carriage.get_visual("finger")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        left_carriage,
        housing,
        elem_a=left_upper_pad,
        elem_b=upper_rail,
        name="left upper pad rides on upper rail",
    )
    ctx.expect_contact(
        left_carriage,
        housing,
        elem_a=left_lower_pad,
        elem_b=lower_rail,
        name="left lower pad rides on lower rail",
    )
    ctx.expect_contact(
        right_carriage,
        housing,
        elem_a=right_upper_pad,
        elem_b=upper_rail,
        name="right upper pad rides on upper rail",
    )
    ctx.expect_contact(
        right_carriage,
        housing,
        elem_a=right_lower_pad,
        elem_b=lower_rail,
        name="right lower pad rides on lower rail",
    )
    ctx.expect_overlap(
        left_carriage,
        housing,
        axes="xz",
        min_overlap=0.0075,
        elem_a=left_upper_pad,
        elem_b=upper_rail,
        name="left upper pad overlaps rail footprint",
    )
    ctx.expect_overlap(
        right_carriage,
        housing,
        axes="xz",
        min_overlap=0.0075,
        elem_a=right_upper_pad,
        elem_b=upper_rail,
        name="right upper pad overlaps rail footprint",
    )
    ctx.expect_gap(
        right_carriage,
        left_carriage,
        axis="x",
        min_gap=0.068,
        positive_elem=right_finger,
        negative_elem=left_finger,
        name="open jaws leave a central pickup window",
    )

    left_rest_x = ctx.part_world_position(left_carriage)[0]
    right_rest_x = ctx.part_world_position(right_carriage)[0]
    with ctx.pose({left_slide: TRAVEL, right_slide: TRAVEL}):
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            max_gap=0.0005,
            max_penetration=1e-6,
            positive_elem=right_finger,
            negative_elem=left_finger,
            name="closed jaws meet at shared pickup zone",
        )
        ctx.expect_overlap(
            left_carriage,
            right_carriage,
            axes="yz",
            min_overlap=0.010,
            elem_a=left_finger,
            elem_b=right_finger,
            name="jaw fingers stay aligned through closure",
        )
        left_closed_x = ctx.part_world_position(left_carriage)[0]
        right_closed_x = ctx.part_world_position(right_carriage)[0]

    ctx.check(
        "left carriage travel is 35 mm inward",
        abs((left_closed_x - left_rest_x) - TRAVEL) <= 1e-6,
        details=f"expected {TRAVEL:.6f} m, got {left_closed_x - left_rest_x:.6f} m",
    )
    ctx.check(
        "right carriage travel is 35 mm inward",
        abs((right_rest_x - right_closed_x) - TRAVEL) <= 1e-6,
        details=f"expected {TRAVEL:.6f} m, got {right_rest_x - right_closed_x:.6f} m",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
