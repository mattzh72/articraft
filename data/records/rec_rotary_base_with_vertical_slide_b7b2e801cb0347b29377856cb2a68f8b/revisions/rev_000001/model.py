from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.34
BASE_WIDTH = 0.26
BASE_HEIGHT = 0.085

ROTARY_RADIUS = 0.092
ROTARY_DECK_HEIGHT = 0.020
ROTARY_PEDESTAL_RADIUS = 0.050
ROTARY_PEDESTAL_HEIGHT = 0.026

GUIDE_OFFSET_Y = -0.018
GUIDE_OUTER_X = 0.086
GUIDE_OUTER_Y = 0.062
GUIDE_OUTER_Z = 0.170
GUIDE_INNER_X = 0.066
GUIDE_INNER_Y = 0.042

MAST_X = 0.058
MAST_Y = 0.034
MAST_Z = 0.220
HEAD_X = 0.126
HEAD_Y = 0.096
HEAD_Z = 0.034
PLATE_X = 0.088
PLATE_Y = 0.018
PLATE_Z = 0.074

LIFT_TRAVEL = 0.110
PAN_LIMIT = 2.6


def _base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.014)
        .edges(">Z")
        .fillet(0.006)
    )

    service_hatch = (
        cq.Workplane("XY")
        .box(BASE_LENGTH * 0.58, BASE_WIDTH * 0.44, 0.006)
        .translate((0.0, 0.012, BASE_HEIGHT - 0.003))
    )
    front_panel = (
        cq.Workplane("XY")
        .box(BASE_LENGTH * 0.62, 0.018, BASE_HEIGHT * 0.34)
        .translate((0.0, (BASE_WIDTH / 2.0) - 0.009, BASE_HEIGHT * 0.36))
    )
    return body.cut(service_hatch).cut(front_panel)


def _rotary_deck_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(ROTARY_RADIUS).extrude(ROTARY_DECK_HEIGHT)


def _guide_housing_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(GUIDE_OUTER_X, GUIDE_OUTER_Y, GUIDE_OUTER_Z)
        .translate((0.0, GUIDE_OFFSET_Y, ROTARY_DECK_HEIGHT + (GUIDE_OUTER_Z / 2.0)))
    )
    inner = (
        cq.Workplane("XY")
        .box(GUIDE_INNER_X, GUIDE_INNER_Y, GUIDE_OUTER_Z + 0.004)
        .translate((0.0, GUIDE_OFFSET_Y, ROTARY_DECK_HEIGHT + (GUIDE_OUTER_Z / 2.0)))
    )
    return outer.cut(inner).edges("|Z").fillet(0.004)


def _mast_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(MAST_X, MAST_Y, MAST_Z)
        .translate((0.0, 0.0, MAST_Z / 2.0))
        .edges("|Z")
        .fillet(0.003)
    )


def _carriage_head_shape() -> cq.Workplane:
    head = (
        cq.Workplane("XY")
        .box(HEAD_X, HEAD_Y, HEAD_Z)
        .translate((0.0, 0.004, MAST_Z - (HEAD_Z / 2.0)))
        .edges("|Z")
        .fillet(0.004)
    )
    return head


def _service_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PLATE_X, PLATE_Y, PLATE_Z)
        .translate((0.0, (HEAD_Y / 2.0) + (PLATE_Y / 2.0) - 0.002, MAST_Z - 0.062))
        .edges("|Z")
        .fillet(0.003)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_pan_lift_unit")

    model.material("base_paint", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("machined_steel", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("service_orange", rgba=(0.91, 0.49, 0.14, 1.0))

    base = model.part("ground_body")
    base.visual(
        mesh_from_cadquery(_base_shape(), "ground_body"),
        material="base_paint",
        name="body_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        mesh_from_cadquery(_rotary_deck_shape(), "pan_stage_rotary_deck"),
        material="machined_steel",
        name="rotary_deck",
    )
    pan_stage.visual(
        mesh_from_cadquery(_guide_housing_shape(), "pan_stage_guide_housing"),
        material="machined_steel",
        name="guide_housing",
    )
    pan_stage.inertial = Inertial.from_geometry(
        Box((0.184, 0.184, GUIDE_OUTER_Z + ROTARY_DECK_HEIGHT)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, (GUIDE_OUTER_Z + ROTARY_DECK_HEIGHT) / 2.0)),
    )

    lift_carriage = model.part("lift_carriage")
    lift_carriage.visual(
        mesh_from_cadquery(_mast_shape(), "lift_carriage_inner_mast"),
        material="service_orange",
        name="inner_mast",
    )
    lift_carriage.visual(
        mesh_from_cadquery(_carriage_head_shape(), "lift_carriage_head"),
        material="service_orange",
        name="carriage_head",
    )
    lift_carriage.visual(
        mesh_from_cadquery(_service_plate_shape(), "lift_carriage_service_plate"),
        material="service_orange",
        name="service_plate",
    )
    lift_carriage.inertial = Inertial.from_geometry(
        Box((HEAD_X, HEAD_Y + PLATE_Y, MAST_Z)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.02, MAST_Z / 2.0)),
    )

    model.articulation(
        "body_to_pan_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-PAN_LIMIT,
            upper=PAN_LIMIT,
            effort=35.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "pan_stage_to_lift_carriage",
        ArticulationType.PRISMATIC,
        parent=pan_stage,
        child=lift_carriage,
        origin=Origin(xyz=(0.0, GUIDE_OFFSET_Y, ROTARY_DECK_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=180.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("ground_body")
    pan_stage = object_model.get_part("pan_stage")
    lift_carriage = object_model.get_part("lift_carriage")
    pan_joint = object_model.get_articulation("body_to_pan_stage")
    lift_joint = object_model.get_articulation("pan_stage_to_lift_carriage")
    lift_upper = lift_joint.motion_limits.upper if lift_joint.motion_limits is not None else 0.0

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

    with ctx.pose({pan_joint: 0.0, lift_joint: 0.0}):
        ctx.expect_gap(
            pan_stage,
            base,
            axis="z",
            max_gap=0.0015,
            max_penetration=0.0,
            name="pan stage sits on the grounded body",
        )
        ctx.expect_overlap(
            pan_stage,
            base,
            axes="xy",
            min_overlap=0.12,
            name="ground body broadly supports the pan stage",
        )
        ctx.expect_within(
            lift_carriage,
            pan_stage,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="guide_housing",
            margin=0.001,
            name="lift mast stays centered inside the guide housing",
        )
        ctx.expect_overlap(
            lift_carriage,
            pan_stage,
            axes="z",
            elem_a="inner_mast",
            elem_b="guide_housing",
            min_overlap=0.16,
            name="collapsed lift carriage remains deeply inserted in the guide housing",
        )

    rest_pos = ctx.part_world_position(lift_carriage)
    with ctx.pose({pan_joint: 1.2, lift_joint: 0.0}):
        panned_pos = ctx.part_world_position(lift_carriage)
    ctx.check(
        "positive pan rotates the carriage around the vertical axis",
        rest_pos is not None
        and panned_pos is not None
        and panned_pos[0] > rest_pos[0] + 0.012
        and abs(panned_pos[2] - rest_pos[2]) < 0.002,
        details=f"rest={rest_pos}, panned={panned_pos}",
    )

    with ctx.pose({pan_joint: 0.0, lift_joint: lift_upper}):
        lifted_pos = ctx.part_world_position(lift_carriage)
        ctx.expect_within(
            lift_carriage,
            pan_stage,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="guide_housing",
            margin=0.001,
            name="extended lift mast stays centered inside the guide housing",
        )
        ctx.expect_overlap(
            lift_carriage,
            pan_stage,
            axes="z",
            elem_a="inner_mast",
            elem_b="guide_housing",
            min_overlap=0.05,
            name="extended lift carriage still retains insertion in the guide housing",
        )
    ctx.check(
        "positive lift raises the carriage upward",
        rest_pos is not None and lifted_pos is not None and lifted_pos[2] > rest_pos[2] + 0.09,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
