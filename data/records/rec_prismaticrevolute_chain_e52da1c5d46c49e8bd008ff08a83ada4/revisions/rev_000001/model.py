from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


SLIDE_LENGTH = 0.36
SLIDE_WIDTH = 0.12
SLIDE_END_HEIGHT = 0.006
SLIDE_TOP_HEIGHT = 0.016
SLIDE_RAMP_LENGTH = 0.03

CARRIAGE_LENGTH = 0.11
CARRIAGE_WIDTH = 0.096
CARRIAGE_BODY_HEIGHT = 0.022
CARRIAGE_PAD_HEIGHT = 0.004
CARRIAGE_FRONT_PAD_LENGTH = 0.074
CARRIAGE_REAR_PAD_LENGTH = 0.036

HINGE_X = -0.039
HINGE_Z = 0.042
LUG_WIDTH = 0.014
LUG_GAP = 0.040
LUG_STEM_LENGTH = 0.018
LUG_STEM_HEIGHT = 0.020
LUG_HEAD_RADIUS = 0.006

TAB_BARREL_RADIUS = 0.006
TAB_BARREL_LENGTH = LUG_GAP
TAB_PLATE_LENGTH = 0.054
TAB_PLATE_WIDTH = 0.046
TAB_PLATE_THICKNESS = 0.007
TAB_WEB_LENGTH = 0.028
TAB_WEB_WIDTH = 0.012
TAB_WEB_HEIGHT = 0.014

PRISMATIC_TRAVEL = 0.16
TAB_OPEN_ANGLE = 1.35


def _cylinder_along_y(radius: float, length: float, x: float, y_center: float, z: float):
    return cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(x, y_center - length / 2.0, z),
        cq.Vector(0.0, 1.0, 0.0),
    )


def make_slide_shape():
    side_profile = [
        (-SLIDE_LENGTH / 2.0, 0.0),
        (-SLIDE_LENGTH / 2.0, SLIDE_END_HEIGHT),
        (-SLIDE_LENGTH / 2.0 + SLIDE_RAMP_LENGTH, SLIDE_TOP_HEIGHT),
        (SLIDE_LENGTH / 2.0 - SLIDE_RAMP_LENGTH, SLIDE_TOP_HEIGHT),
        (SLIDE_LENGTH / 2.0, SLIDE_END_HEIGHT),
        (SLIDE_LENGTH / 2.0, 0.0),
    ]

    slide = (
        cq.Workplane("XZ")
        .polyline(side_profile)
        .close()
        .extrude(SLIDE_WIDTH / 2.0, both=True)
        .edges("|Y")
        .fillet(0.0025)
    )
    return slide


def make_carriage_shape():
    body = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_HEIGHT)
        .translate((0.0, 0.0, CARRIAGE_BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.003)
    )

    front_pad = (
        cq.Workplane("XY")
        .box(CARRIAGE_FRONT_PAD_LENGTH, 0.062, CARRIAGE_PAD_HEIGHT)
        .translate(
            (
                0.012,
                0.0,
                CARRIAGE_BODY_HEIGHT + CARRIAGE_PAD_HEIGHT / 2.0,
            )
        )
    )
    rear_pad = (
        cq.Workplane("XY")
        .box(CARRIAGE_REAR_PAD_LENGTH, CARRIAGE_WIDTH, CARRIAGE_PAD_HEIGHT)
        .translate(
            (
                -0.028,
                0.0,
                CARRIAGE_BODY_HEIGHT + CARRIAGE_PAD_HEIGHT / 2.0,
            )
        )
    )

    lug_y = LUG_GAP / 2.0 + LUG_WIDTH / 2.0
    left_stem = (
        cq.Workplane("XY")
        .box(LUG_STEM_LENGTH, LUG_WIDTH, LUG_STEM_HEIGHT)
        .translate((HINGE_X, -lug_y, 0.036))
    )
    right_stem = (
        cq.Workplane("XY")
        .box(LUG_STEM_LENGTH, LUG_WIDTH, LUG_STEM_HEIGHT)
        .translate((HINGE_X, lug_y, 0.036))
    )
    left_head = cq.Workplane(
        obj=_cylinder_along_y(LUG_HEAD_RADIUS, LUG_WIDTH, HINGE_X, -lug_y, HINGE_Z)
    )
    right_head = cq.Workplane(
        obj=_cylinder_along_y(LUG_HEAD_RADIUS, LUG_WIDTH, HINGE_X, lug_y, HINGE_Z)
    )

    carriage = (
        cq.Workplane("XY")
        .add(body.val())
        .add(front_pad.val())
        .add(rear_pad.val())
        .add(left_stem.val())
        .add(right_stem.val())
        .add(left_head.val())
        .add(right_head.val())
        .combine()
    )
    return carriage


def make_support_tab_shape():
    barrel = _cylinder_along_y(TAB_BARREL_RADIUS, TAB_BARREL_LENGTH, 0.0, 0.0, 0.0)
    web = (
        cq.Workplane("XY")
        .box(TAB_WEB_LENGTH, TAB_WEB_WIDTH, TAB_WEB_HEIGHT)
        .translate((0.012, 0.0, TAB_WEB_HEIGHT / 2.0))
        .val()
    )
    plate = (
        cq.Workplane("XY")
        .box(TAB_PLATE_LENGTH, TAB_PLATE_WIDTH, TAB_PLATE_THICKNESS)
        .translate((0.038, 0.0, 0.011))
        .val()
    )

    tab = barrel.fuse(web).fuse(plate)
    return cq.Workplane(obj=tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_carriage")

    slide_mat = model.material("slide_finish", rgba=(0.22, 0.24, 0.27, 1.0))
    carriage_mat = model.material("carriage_finish", rgba=(0.70, 0.72, 0.74, 1.0))
    tab_mat = model.material("tab_finish", rgba=(0.17, 0.18, 0.20, 1.0))

    slide = model.part("slide")
    slide.visual(
        mesh_from_cadquery(make_slide_shape(), "slide_body"),
        material=slide_mat,
        name="slide_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_BODY_HEIGHT / 2.0)),
        material=carriage_mat,
        name="carriage_body",
    )
    carriage.visual(
        Box((CARRIAGE_FRONT_PAD_LENGTH, 0.062, CARRIAGE_PAD_HEIGHT)),
        origin=Origin(
            xyz=(0.012, 0.0, CARRIAGE_BODY_HEIGHT + CARRIAGE_PAD_HEIGHT / 2.0)
        ),
        material=carriage_mat,
        name="carriage_front_pad",
    )
    carriage.visual(
        Box((CARRIAGE_REAR_PAD_LENGTH, CARRIAGE_WIDTH, CARRIAGE_PAD_HEIGHT)),
        origin=Origin(
            xyz=(-0.028, 0.0, CARRIAGE_BODY_HEIGHT + CARRIAGE_PAD_HEIGHT / 2.0)
        ),
        material=carriage_mat,
        name="carriage_rear_pad",
    )

    lug_y = LUG_GAP / 2.0 + LUG_WIDTH / 2.0
    lug_size = (LUG_STEM_LENGTH, LUG_WIDTH, 0.028)
    lug_center_z = 0.036
    carriage.visual(
        Box(lug_size),
        origin=Origin(xyz=(HINGE_X, -lug_y, lug_center_z)),
        material=carriage_mat,
        name="carriage_left_lug",
    )
    carriage.visual(
        Box(lug_size),
        origin=Origin(xyz=(HINGE_X, lug_y, lug_center_z)),
        material=carriage_mat,
        name="carriage_right_lug",
    )

    support_tab = model.part("support_tab")
    support_tab.visual(
        Cylinder(radius=0.0055, length=LUG_GAP),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=tab_mat,
        name="tab_barrel",
    )
    support_tab.visual(
        Box((0.022, 0.012, 0.018)),
        origin=Origin(xyz=(0.011, 0.0, 0.011)),
        material=tab_mat,
        name="tab_web",
    )
    support_tab.visual(
        Box((0.056, 0.044, 0.007)),
        origin=Origin(xyz=(0.039, 0.0, 0.0215)),
        material=tab_mat,
        name="tab_plate",
    )

    model.articulation(
        "slide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=slide,
        child=carriage,
        origin=Origin(xyz=(-0.08, 0.0, SLIDE_TOP_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=PRISMATIC_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=support_tab,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=TAB_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slide = object_model.get_part("slide")
    carriage = object_model.get_part("carriage")
    support_tab = object_model.get_part("support_tab")
    slider = object_model.get_articulation("slide_to_carriage")
    tab_hinge = object_model.get_articulation("carriage_to_tab")

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

    ctx.expect_contact(
        carriage,
        slide,
        contact_tol=0.0005,
        name="carriage_is_seated_on_slide",
    )
    ctx.expect_within(
        carriage,
        slide,
        axes="xy",
        margin=0.0,
        name="carriage_stays_within_slide_footprint_at_home",
    )
    ctx.expect_contact(
        support_tab,
        carriage,
        contact_tol=0.0005,
        name="support_tab_is_supported_by_carriage_closed",
    )

    with ctx.pose({slider: slider.motion_limits.upper}):
        ctx.expect_contact(
            carriage,
            slide,
            contact_tol=0.0005,
            name="carriage_remains_supported_at_full_extension",
        )
        ctx.expect_within(
            carriage,
            slide,
            axes="xy",
            margin=0.0,
            name="carriage_stays_within_slide_footprint_extended",
        )

    with ctx.pose({tab_hinge: tab_hinge.motion_limits.upper}):
        ctx.expect_contact(
            support_tab,
            carriage,
            contact_tol=0.0005,
            name="support_tab_hinge_remains_supported_open",
        )

    with ctx.pose({slider: slider.motion_limits.lower}):
        carriage_home = ctx.part_world_position(carriage)
    with ctx.pose({slider: slider.motion_limits.upper}):
        carriage_extended = ctx.part_world_position(carriage)
    ctx.check(
        "carriage_translates_along_slide",
        carriage_home is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_home[0] + 0.12,
        details=(
            f"expected forward prismatic travel; got home={carriage_home}, "
            f"extended={carriage_extended}"
        ),
    )

    with ctx.pose({tab_hinge: 0.0}):
        closed_tab_aabb = ctx.part_world_aabb(support_tab)
    with ctx.pose({tab_hinge: tab_hinge.motion_limits.upper}):
        open_tab_aabb = ctx.part_world_aabb(support_tab)
    ctx.check(
        "support_tab_rotates_upward",
        closed_tab_aabb is not None
        and open_tab_aabb is not None
        and open_tab_aabb[1][2] > closed_tab_aabb[1][2] + 0.03,
        details=(
            f"expected open tab to raise above closed pose; got "
            f"closed={closed_tab_aabb}, open={open_tab_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
