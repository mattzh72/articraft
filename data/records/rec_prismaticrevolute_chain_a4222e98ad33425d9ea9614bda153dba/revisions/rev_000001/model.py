from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BODY_LENGTH = 0.40
BODY_WIDTH = 0.11
BASE_THICKNESS = 0.012
RAIL_WIDTH = 0.018
RAIL_HEIGHT = 0.030
STOP_LENGTH = 0.028
STOP_HEIGHT = 0.018

CARRIAGE_LENGTH = 0.10
CARRIAGE_WIDTH = 0.052
CARRIAGE_HEIGHT = 0.018

SLIDE_RETRACTED_X = -0.08
SLIDE_TRAVEL = 0.16

HINGE_X_ON_CARRIAGE = 0.058
HINGE_Z_ON_CARRIAGE = 0.026


def _build_slide_body() -> cq.Workplane:
    rail_y = BODY_WIDTH / 2.0 - RAIL_WIDTH / 2.0
    stop_x = BODY_LENGTH / 2.0 - STOP_LENGTH / 2.0

    left_rail = cq.Workplane("XY").box(BODY_LENGTH, RAIL_WIDTH, RAIL_HEIGHT).translate(
        (0.0, rail_y, RAIL_HEIGHT / 2.0)
    )
    right_rail = cq.Workplane("XY").box(BODY_LENGTH, RAIL_WIDTH, RAIL_HEIGHT).translate(
        (0.0, -rail_y, RAIL_HEIGHT / 2.0)
    )
    left_base_foot = cq.Workplane("XY").box(BODY_LENGTH, 0.012, BASE_THICKNESS).translate(
        (0.0, rail_y, BASE_THICKNESS / 2.0)
    )
    right_base_foot = cq.Workplane("XY").box(BODY_LENGTH, 0.012, BASE_THICKNESS).translate(
        (0.0, -rail_y, BASE_THICKNESS / 2.0)
    )
    left_stop = cq.Workplane("XY").box(STOP_LENGTH, 0.036, STOP_HEIGHT).translate(
        (-stop_x, 0.0, BASE_THICKNESS + STOP_HEIGHT / 2.0)
    )
    right_stop = cq.Workplane("XY").box(STOP_LENGTH, 0.036, STOP_HEIGHT).translate(
        (stop_x, 0.0, BASE_THICKNESS + STOP_HEIGHT / 2.0)
    )
    left_bridge = cq.Workplane("XY").box(STOP_LENGTH, BODY_WIDTH, BASE_THICKNESS).translate(
        (-stop_x, 0.0, BASE_THICKNESS / 2.0)
    )
    right_bridge = cq.Workplane("XY").box(STOP_LENGTH, BODY_WIDTH, BASE_THICKNESS).translate(
        (stop_x, 0.0, BASE_THICKNESS / 2.0)
    )

    return (
        left_rail.union(right_rail)
        .union(left_base_foot)
        .union(right_base_foot)
        .union(left_bridge)
        .union(right_bridge)
        .union(left_stop)
        .union(right_stop)
    )


def _build_carriage() -> cq.Workplane:
    main = cq.Workplane("XY").box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT).translate(
        (0.0, 0.0, 0.014)
    )
    pocket = cq.Workplane("XY").box(0.058, 0.024, 0.016).translate((-0.008, 0.0, 0.021))
    main = main.cut(pocket)

    runner_width = 0.014
    runner_y = BODY_WIDTH / 2.0 - runner_width / 2.0 - 0.002
    left_runner = cq.Workplane("XY").box(0.084, runner_width, 0.004).translate((0.0, runner_y, 0.002))
    right_runner = cq.Workplane("XY").box(0.084, runner_width, 0.004).translate((0.0, -runner_y, 0.002))
    left_web = cq.Workplane("XY").box(0.090, 0.018, 0.012).translate((0.0, 0.032, 0.009))
    right_web = cq.Workplane("XY").box(0.090, 0.018, 0.012).translate((0.0, -0.032, 0.009))

    cheek_length = 0.016
    cheek_width = 0.010
    cheek_height = 0.020
    cheek_center_x = 0.042
    cheek_center_y = 0.014

    left_cheek = cq.Workplane("XY").box(cheek_length, cheek_width, cheek_height).translate(
        (cheek_center_x, cheek_center_y, 0.024)
    )
    right_cheek = cq.Workplane("XY").box(cheek_length, cheek_width, cheek_height).translate(
        (cheek_center_x, -cheek_center_y, 0.024)
    )
    hinge_buttress = cq.Workplane("XY").box(0.020, 0.036, 0.010).translate((0.026, 0.0, 0.021))

    return (
        main.union(left_runner)
        .union(right_runner)
        .union(left_web)
        .union(right_web)
        .union(left_cheek)
        .union(right_cheek)
        .union(hinge_buttress)
    )


def _build_hinged_arm() -> cq.Workplane:
    barrel = cq.Workplane("XZ").circle(0.008).extrude(0.018, both=True)
    arm_leaf = cq.Workplane("XY").box(0.110, 0.018, 0.010).translate((0.058, 0.0, -0.002))
    output_tab = cq.Workplane("XY").box(0.034, 0.028, 0.006).translate((0.126, 0.0, -0.004))
    return barrel.union(arm_leaf).union(output_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shuttle_hinge_chain")

    body_material = model.material("body_aluminum", rgba=(0.72, 0.75, 0.79, 1.0))
    carriage_material = model.material("carriage_steel", rgba=(0.25, 0.29, 0.33, 1.0))
    arm_material = model.material("arm_painted", rgba=(0.80, 0.20, 0.14, 1.0))

    slide_body = model.part("slide_body")
    slide_body.visual(
        mesh_from_cadquery(_build_slide_body(), "slide_body"),
        material=body_material,
        name="body_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage(), "carriage"),
        material=carriage_material,
        name="carriage_shell",
    )

    hinged_arm = model.part("hinged_arm")
    hinged_arm.visual(
        mesh_from_cadquery(_build_hinged_arm(), "hinged_arm"),
        material=arm_material,
        name="arm_shell",
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=slide_body,
        child=carriage,
        origin=Origin(xyz=(SLIDE_RETRACTED_X, 0.0, RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=hinged_arm,
        origin=Origin(xyz=(HINGE_X_ON_CARRIAGE, 0.0, HINGE_Z_ON_CARRIAGE)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slide_body = object_model.get_part("slide_body")
    carriage = object_model.get_part("carriage")
    hinged_arm = object_model.get_part("hinged_arm")
    slide = object_model.get_articulation("body_to_carriage")
    hinge = object_model.get_articulation("carriage_to_arm")

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
        "slide_joint_is_prismatic_x",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"expected prismatic +X slide, got type={slide.articulation_type!r}, axis={slide.axis!r}",
    )
    ctx.check(
        "hinge_joint_is_revolute_pitch",
        hinge.articulation_type == ArticulationType.REVOLUTE and tuple(hinge.axis) == (0.0, -1.0, 0.0),
        f"expected carried revolute about -Y, got type={hinge.articulation_type!r}, axis={hinge.axis!r}",
    )

    ctx.expect_contact(
        carriage,
        slide_body,
        contact_tol=0.001,
        name="carriage_bears_on_slide_body",
    )
    ctx.expect_within(
        carriage,
        slide_body,
        axes="xy",
        margin=0.001,
        name="carriage_stays_inside_body_plan",
    )
    ctx.expect_contact(
        hinged_arm,
        carriage,
        contact_tol=0.001,
        name="arm_barrel_is_supported_by_carriage",
    )

    retracted_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            slide_body,
            contact_tol=0.001,
            name="extended_carriage_still_bears_on_body",
        )
        ctx.expect_contact(
            hinged_arm,
            carriage,
            contact_tol=0.001,
            name="hinge_support_remains_at_full_slide",
        )
    ctx.check(
        "carriage_translates_forward_along_body",
        retracted_pos is not None
        and extended_pos is not None
        and (extended_pos[0] - retracted_pos[0]) > 0.15,
        f"expected about {SLIDE_TRAVEL:.3f} m forward travel, got "
        f"{None if retracted_pos is None or extended_pos is None else extended_pos[0] - retracted_pos[0]:.6f}",
    )

    closed_arm_aabb = ctx.part_world_aabb(hinged_arm)
    with ctx.pose({hinge: 1.0}):
        open_arm_aabb = ctx.part_world_aabb(hinged_arm)
        ctx.expect_contact(
            hinged_arm,
            carriage,
            contact_tol=0.001,
            name="hinge_support_remains_when_arm_opens",
        )
    ctx.check(
        "arm_opens_upward",
        closed_arm_aabb is not None
        and open_arm_aabb is not None
        and open_arm_aabb[1][2] > closed_arm_aabb[1][2] + 0.06,
        "arm should raise its tip significantly in +Z when the carried hinge opens",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
