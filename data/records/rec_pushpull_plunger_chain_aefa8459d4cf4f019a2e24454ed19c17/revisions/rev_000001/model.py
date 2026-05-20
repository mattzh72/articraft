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


HOUSING_LENGTH = 0.180
HOUSING_WIDTH = 0.070
HOUSING_HEIGHT = 0.052
BASE_FLANGE_LENGTH = 0.200
BASE_FLANGE_WIDTH = 0.084
BASE_FLANGE_HEIGHT = 0.008

GUIDE_CENTER_Z = 0.028

PLUNGER_JOINT_X = -0.040
PLUNGER_BODY_WIDTH = 0.014
PLUNGER_BODY_HEIGHT = 0.012
PLUNGER_NOSE_LENGTH = 0.142

TAB_HINGE_X = 0.102
TAB_HINGE_Z = 0.040
TAB_PANEL_WIDTH = 0.028
TAB_PANEL_HEIGHT = 0.020
TAB_PANEL_THICKNESS = 0.004
TAB_KNUCKLE_RADIUS = 0.0025
TAB_KNUCKLE_LENGTH = 0.020


def _housing_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").box(
        BASE_FLANGE_LENGTH,
        BASE_FLANGE_WIDTH,
        BASE_FLANGE_HEIGHT,
        centered=(True, True, False),
    )
    left_rail = (
        cq.Workplane("XY")
        .box(
            0.145,
            0.016,
            0.028,
            centered=(True, True, False),
        )
        .translate((0.002, 0.018, BASE_FLANGE_HEIGHT))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(0.145, 0.016, 0.028, centered=(True, True, False))
        .translate((0.002, -0.018, BASE_FLANGE_HEIGHT))
    )
    top_bridge = (
        cq.Workplane("XY")
        .box(
            0.170,
            0.052,
            0.010,
            centered=(True, True, False),
        )
        .translate((0.013, 0.0, 0.036))
    )
    front_plate = (
        cq.Workplane("XY")
        .box(0.012, 0.052, 0.038, centered=(True, True, False))
        .translate((0.092, 0.0, BASE_FLANGE_HEIGHT))
    )
    lower_lip = (
        cq.Workplane("XY")
        .box(0.016, 0.040, 0.010, centered=(True, True, False))
        .translate((0.086, 0.0, BASE_FLANGE_HEIGHT))
    )
    housing = (
        cq.Workplane("XY")
        .add(flange.val())
        .add(left_rail.val())
        .add(right_rail.val())
        .add(top_bridge.val())
        .add(front_plate.val())
        .add(lower_lip.val())
        .combine()
    )

    nose_window = (
        cq.Workplane("XY")
        .box(0.022, 0.020, 0.018, centered=(True, True, True))
        .translate((0.092, 0.0, GUIDE_CENTER_Z))
    )
    housing = housing.cut(nose_window)

    return housing


def _plunger_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, -PLUNGER_BODY_HEIGHT / 2.0),
                (0.0, PLUNGER_BODY_HEIGHT / 2.0),
                (0.112, PLUNGER_BODY_HEIGHT / 2.0),
                (0.132, 0.0035),
                (PLUNGER_NOSE_LENGTH, 0.0040),
                (PLUNGER_NOSE_LENGTH, -PLUNGER_BODY_HEIGHT / 2.0),
            ]
        )
        .close()
        .extrude(PLUNGER_BODY_WIDTH, both=True)
    )
    rear_stem = (
        cq.Workplane("XY")
        .box(
            0.020,
            0.012,
            0.010,
            centered=(True, True, True),
        )
        .translate((-0.010, 0.0, 0.0))
    )
    rear_button = (
        cq.Workplane("XY")
        .box(0.038, 0.030, 0.020, centered=(True, True, True))
        .translate((-0.039, 0.0, 0.0))
    )
    return body.union(rear_stem).union(rear_button)


def _tab_shape() -> cq.Workplane:
    knuckle = (
        cq.Workplane("XZ")
        .circle(TAB_KNUCKLE_RADIUS)
        .extrude(TAB_KNUCKLE_LENGTH, both=True)
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.004, 0.022, 0.004, centered=(True, True, False))
        .translate((0.002, 0.0, -0.004))
    )
    panel = (
        cq.Workplane("XY")
        .box(
            TAB_PANEL_THICKNESS,
            TAB_PANEL_WIDTH,
            TAB_PANEL_HEIGHT,
            centered=(True, True, False),
        )
        .translate((TAB_PANEL_THICKNESS / 2.0, 0.0, -TAB_PANEL_HEIGHT))
    )
    return knuckle.union(bridge).union(panel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plunger_tab_mechanism")

    model.material("housing_gray", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("plunger_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("tab_black", rgba=(0.14, 0.15, 0.16, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.210, 0.080, 0.008)),
        material="housing_gray",
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        name="base",
    )
    housing.visual(
        Box((0.164, 0.012, 0.028)),
        material="housing_gray",
        origin=Origin(xyz=(0.0, 0.016, 0.022)),
        name="left_rail",
    )
    housing.visual(
        Box((0.164, 0.012, 0.028)),
        material="housing_gray",
        origin=Origin(xyz=(0.0, -0.016, 0.022)),
        name="right_rail",
    )
    housing.visual(
        Box((0.164, 0.044, 0.008)),
        material="housing_gray",
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        name="roof",
    )
    housing.visual(
        Box((0.012, 0.044, 0.012)),
        material="housing_gray",
        origin=Origin(xyz=(0.088, 0.0, 0.014)),
        name="front_bottom",
    )
    housing.visual(
        Box((0.012, 0.044, 0.016)),
        material="housing_gray",
        origin=Origin(xyz=(0.0885, 0.0, 0.044)),
        name="front_top",
    )
    housing.visual(
        Box((0.012, 0.012, 0.016)),
        material="housing_gray",
        origin=Origin(xyz=(0.088, 0.016, 0.028)),
        name="front_left_cheek",
    )
    housing.visual(
        Box((0.012, 0.012, 0.016)),
        material="housing_gray",
        origin=Origin(xyz=(0.088, -0.016, 0.028)),
        name="front_right_cheek",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Box((0.154, 0.014, 0.012)),
        material="plunger_steel",
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        name="shaft",
    )
    plunger.visual(
        Box((0.025, 0.010, 0.008)),
        material="plunger_steel",
        origin=Origin(xyz=(0.1245, 0.0, 0.0)),
        name="nose",
    )
    plunger.visual(
        Box((0.020, 0.018, 0.012)),
        material="plunger_steel",
        origin=Origin(xyz=(-0.052, 0.0, 0.0)),
        name="neck",
    )
    plunger.visual(
        Box((0.040, 0.030, 0.020)),
        material="plunger_steel",
        origin=Origin(xyz=(-0.082, 0.0, 0.0)),
        name="button",
    )

    tab = model.part("tab")
    tab.visual(
        Cylinder(radius=0.0025, length=0.020),
        material="tab_black",
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        name="barrel",
    )
    tab.visual(
        Box((0.004, 0.020, 0.006)),
        material="tab_black",
        origin=Origin(xyz=(0.002, 0.0, -0.003)),
        name="stem",
    )
    tab.visual(
        Box((0.004, 0.028, 0.020)),
        material="tab_black",
        origin=Origin(xyz=(0.002, 0.0, -0.016)),
        name="panel",
    )

    model.articulation(
        "housing_to_plunger",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=plunger,
        origin=Origin(xyz=(-0.040, 0.0, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.15,
            lower=-0.018,
            upper=0.004,
        ),
    )
    model.articulation(
        "housing_to_tab",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=tab,
        origin=Origin(xyz=(0.097, 0.0, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.5,
            lower=0.0,
            upper=0.90,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    plunger = object_model.get_part("plunger")
    tab = object_model.get_part("tab")
    plunger_joint = object_model.get_articulation("housing_to_plunger")
    tab_joint = object_model.get_articulation("housing_to_tab")
    base = housing.get_visual("base")
    left_rail = housing.get_visual("left_rail")
    right_rail = housing.get_visual("right_rail")
    roof = housing.get_visual("roof")
    front_top = housing.get_visual("front_top")
    shaft = plunger.get_visual("shaft")
    nose = plunger.get_visual("nose")
    barrel = tab.get_visual("barrel")
    panel = tab.get_visual("panel")

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

    ctx.expect_gap(
        housing,
        plunger,
        axis="y",
        positive_elem=left_rail,
        negative_elem=shaft,
        min_gap=0.002,
        max_gap=0.0045,
        name="plunger_clear_of_left_rail",
    )
    ctx.expect_gap(
        plunger,
        housing,
        axis="y",
        positive_elem=shaft,
        negative_elem=right_rail,
        min_gap=0.002,
        max_gap=0.0045,
        name="plunger_clear_of_right_rail",
    )
    ctx.expect_gap(
        housing,
        plunger,
        axis="z",
        positive_elem=roof,
        negative_elem=shaft,
        min_gap=0.001,
        max_gap=0.004,
        name="plunger_clear_under_roof",
    )
    ctx.expect_gap(
        plunger,
        housing,
        axis="z",
        positive_elem=shaft,
        negative_elem=base,
        min_gap=0.012,
        max_gap=0.016,
        name="plunger_above_base",
    )
    ctx.expect_contact(
        plunger,
        tab,
        elem_a=nose,
        elem_b=panel,
        contact_tol=0.0005,
        name="plunger_nose_meets_tab",
    )
    with ctx.pose({tab_joint: 0.0}):
        ctx.expect_contact(
            tab,
            housing,
            elem_a=barrel,
            elem_b=front_top,
            contact_tol=1e-6,
            name="tab_barrel_supported_by_front_header",
        )
        ctx.expect_overlap(
            tab,
            housing,
            axes="yz",
            elem_a=barrel,
            elem_b=front_top,
            min_overlap=0.002,
            name="tab_hinge_aligned_with_front_header",
        )

    with ctx.pose({plunger_joint: -0.018}):
        retracted_aabb = ctx.part_element_world_aabb(plunger, elem="shaft")
    with ctx.pose({plunger_joint: 0.0}):
        nominal_aabb = ctx.part_element_world_aabb(plunger, elem="shaft")

    plunger_motion_ok = (
        retracted_aabb is not None
        and nominal_aabb is not None
        and nominal_aabb[1][0] > retracted_aabb[1][0] + 0.015
    )
    ctx.check(
        "plunger_extends_forward_on_positive_slide",
        plunger_motion_ok,
        details=(
            f"retracted={retracted_aabb}, nominal={nominal_aabb}"
            if not plunger_motion_ok
            else ""
        ),
    )

    with ctx.pose({tab_joint: 0.0}):
        closed_tab_aabb = ctx.part_element_world_aabb(tab, elem="panel")
    with ctx.pose({tab_joint: 0.70}):
        open_tab_aabb = ctx.part_element_world_aabb(tab, elem="panel")

    tab_motion_ok = (
        closed_tab_aabb is not None
        and open_tab_aabb is not None
        and open_tab_aabb[1][0] > closed_tab_aabb[1][0] + 0.010
    )
    ctx.check(
        "tab_swings_outward_from_front_face",
        tab_motion_ok,
        details=(
            f"closed={closed_tab_aabb}, open={open_tab_aabb}"
            if not tab_motion_ok
            else ""
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
