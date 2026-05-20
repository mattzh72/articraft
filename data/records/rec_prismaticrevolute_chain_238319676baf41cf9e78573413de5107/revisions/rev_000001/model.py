from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)


BODY_LENGTH = 0.220
BODY_WIDTH = 0.140
BASE_THICKNESS = 0.012
GUIDE_LENGTH = 0.170
GUIDE_THICKNESS = 0.014
GUIDE_HEIGHT = 0.024
GUIDE_INNER_WIDTH = 0.094
GUIDE_CENTER_Y = GUIDE_INNER_WIDTH / 2.0 + GUIDE_THICKNESS / 2.0
BACKSTOP_THICKNESS = 0.012
BACKSTOP_HEIGHT = 0.022
BACKSTOP_CENTER_X = -0.091

CARRIAGE_LENGTH = 0.074
CARRIAGE_WIDTH = 0.088
CARRIAGE_BASE_HEIGHT = 0.010
CARRIAGE_RETRACTED_X = 0.022
SLIDE_TRAVEL = 0.050

SADDLE_LENGTH = 0.052
SADDLE_WIDTH = 0.056
SADDLE_HEIGHT = 0.006
SADDLE_CENTER_X = 0.007

HINGE_X = -0.019
HINGE_Z = 0.020
LUG_CENTER_X = -0.024
LUG_CENTER_Y = 0.030
LUG_THICKNESS = 0.008
LUG_WIDTH = 0.016
LUG_HEIGHT = 0.014

TAB_LENGTH = 0.050
TAB_WIDTH = 0.040
TAB_THICKNESS = 0.004
TAB_BARREL_RADIUS = 0.004
TAB_BARREL_LENGTH = 0.044
TAB_OPEN_ANGLE = 1.15


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_slide_hinge_module")

    body_finish = model.material("body_finish", rgba=(0.18, 0.20, 0.22, 1.0))
    carriage_finish = model.material("carriage_finish", rgba=(0.62, 0.65, 0.69, 1.0))
    tab_finish = model.material("tab_finish", rgba=(0.86, 0.61, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=body_finish,
        name="body_base",
    )
    body.visual(
        Box((GUIDE_LENGTH, GUIDE_THICKNESS, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, GUIDE_CENTER_Y, BASE_THICKNESS + GUIDE_HEIGHT / 2.0)
        ),
        material=body_finish,
        name="left_guide",
    )
    body.visual(
        Box((GUIDE_LENGTH, GUIDE_THICKNESS, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -GUIDE_CENTER_Y, BASE_THICKNESS + GUIDE_HEIGHT / 2.0)
        ),
        material=body_finish,
        name="right_guide",
    )
    body.visual(
        Box((BACKSTOP_THICKNESS, GUIDE_INNER_WIDTH + 2.0 * GUIDE_THICKNESS, BACKSTOP_HEIGHT)),
        origin=Origin(
            xyz=(
                BACKSTOP_CENTER_X,
                0.0,
                BASE_THICKNESS + BACKSTOP_HEIGHT / 2.0,
            )
        ),
        material=body_finish,
        name="backstop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_BASE_HEIGHT / 2.0)),
        material=carriage_finish,
        name="carriage_base",
    )
    carriage.visual(
        Box((SADDLE_LENGTH, SADDLE_WIDTH, SADDLE_HEIGHT)),
        origin=Origin(
            xyz=(
                SADDLE_CENTER_X,
                0.0,
                CARRIAGE_BASE_HEIGHT + SADDLE_HEIGHT / 2.0,
            )
        ),
        material=carriage_finish,
        name="carriage_saddle",
    )
    carriage.visual(
        Box((LUG_THICKNESS, LUG_WIDTH, LUG_HEIGHT)),
        origin=Origin(
            xyz=(
                LUG_CENTER_X,
                LUG_CENTER_Y,
                CARRIAGE_BASE_HEIGHT + LUG_HEIGHT / 2.0,
            )
        ),
        material=carriage_finish,
        name="left_hinge_lug",
    )
    carriage.visual(
        Box((LUG_THICKNESS, LUG_WIDTH, LUG_HEIGHT)),
        origin=Origin(
            xyz=(
                LUG_CENTER_X,
                -LUG_CENTER_Y,
                CARRIAGE_BASE_HEIGHT + LUG_HEIGHT / 2.0,
            )
        ),
        material=carriage_finish,
        name="right_hinge_lug",
    )

    tab = model.part("tab")
    tab.visual(
        Cylinder(radius=TAB_BARREL_RADIUS, length=TAB_BARREL_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=tab_finish,
        name="tab_barrel",
    )
    tab.visual(
        Box((TAB_LENGTH, TAB_WIDTH, TAB_THICKNESS)),
        origin=Origin(
            xyz=(TAB_LENGTH / 2.0, 0.0, -TAB_THICKNESS / 2.0)
        ),
        material=tab_finish,
        name="tab_plate",
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_RETRACTED_X, 0.0, BASE_THICKNESS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.20,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tab,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=TAB_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    tab = object_model.get_part("tab")
    slide = object_model.get_articulation("body_to_carriage")
    hinge = object_model.get_articulation("carriage_to_tab")

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
        "slide_joint_axis",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"expected prismatic slide axis +X, got {slide.axis}",
    )
    ctx.check(
        "tab_hinge_axis",
        tuple(hinge.axis) == (0.0, -1.0, 0.0),
        f"expected carried hinge axis -Y, got {hinge.axis}",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_contact(
            carriage,
            body,
            contact_tol=1e-6,
            name="carriage_supported_by_body",
        )
        ctx.expect_contact(
            tab,
            carriage,
            contact_tol=1e-6,
            name="tab_supported_by_carriage_closed",
        )
        ctx.expect_within(
            carriage,
            body,
            axes="y",
            margin=0.0,
            name="carriage_stays_between_guides",
        )
        ctx.expect_overlap(
            carriage,
            body,
            axes="xy",
            min_overlap=0.070,
            name="carriage_overlaps_body_footprint",
        )

    with ctx.pose({slide: 0.0}):
        retracted_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage_slides_forward",
        retracted_pos is not None
        and extended_pos is not None
        and extended_pos[0] > retracted_pos[0] + 0.045,
        f"expected carriage to move +X by about {SLIDE_TRAVEL:.3f} m, got {retracted_pos} -> {extended_pos}",
    )

    with ctx.pose({slide: SLIDE_TRAVEL, hinge: 0.0}):
        ctx.expect_contact(
            carriage,
            body,
            contact_tol=1e-6,
            name="extended_carriage_remains_supported",
        )
        closed_tab_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")

    with ctx.pose({slide: SLIDE_TRAVEL, hinge: TAB_OPEN_ANGLE}):
        ctx.expect_contact(
            tab,
            carriage,
            contact_tol=1e-6,
            name="opened_tab_remains_carried",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="extended_open_pose_no_overlap")
        open_tab_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")

    ctx.check(
        "tab_opens_upward",
        closed_tab_aabb is not None
        and open_tab_aabb is not None
        and open_tab_aabb[1][2] > closed_tab_aabb[1][2] + 0.030,
        f"expected tab free edge to lift in open pose, got {closed_tab_aabb} -> {open_tab_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
