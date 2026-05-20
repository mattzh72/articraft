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
)


BODY_LENGTH = 0.096
BODY_HALF_LENGTH = BODY_LENGTH / 2.0
PLUNGER_STROKE = 0.040

SHAFT_RADIUS = 0.006
SHAFT_START_X = -0.102
SHAFT_END_X = 0.098

GUIDE_RAIL_RADIUS = 0.0055
GUIDE_RAIL_OFFSET = SHAFT_RADIUS + GUIDE_RAIL_RADIUS

REAR_CONNECTOR_LENGTH = 0.008
FRONT_CONNECTOR_LENGTH = 0.006
CORNER_BLOCK_SIZE = 0.010
CORNER_BLOCK_OFFSET = 0.0105

FRONT_STOP_LENGTH = 0.006
FRONT_STOP_RADIUS = 0.010
FRONT_STOP_REAR_X = BODY_HALF_LENGTH

KNOB_THICKNESS = 0.014
KNOB_RADIUS = 0.017
KNOB_FRONT_X = -BODY_HALF_LENGTH - PLUNGER_STROKE

CLEVIS_REAR_X = 0.086
CLEVIS_LENGTH = 0.034
CLEVIS_TINE_WIDTH = 0.008
CLEVIS_GAP_WIDTH = 0.008
CLEVIS_HEIGHT = 0.018
CLEVIS_BRIDGE_LENGTH = 0.010
CLEVIS_OUTER_WIDTH = (2.0 * CLEVIS_TINE_WIDTH) + CLEVIS_GAP_WIDTH


def x_axis_cylinder_origin(
    center_x: float,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> Origin:
    return Origin(xyz=(center_x, center_y, center_z), rpy=(0.0, pi / 2.0, 0.0))


def add_x_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    center_x: float,
    center_y: float = 0.0,
    center_z: float = 0.0,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=x_axis_cylinder_origin(center_x, center_y, center_z),
        material=material,
        name=name,
    )


def add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_pull_plunger")

    body_metal = model.material("body_metal", rgba=(0.33, 0.35, 0.38, 1.0))
    plunger_metal = model.material("plunger_metal", rgba=(0.72, 0.73, 0.76, 1.0))
    knob_polymer = model.material("knob_polymer", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("guide_body")
    add_x_cylinder(
        body,
        name="top_rail",
        radius=GUIDE_RAIL_RADIUS,
        length=BODY_LENGTH,
        center_x=0.0,
        center_z=GUIDE_RAIL_OFFSET,
        material=body_metal,
    )
    add_x_cylinder(
        body,
        name="bottom_rail",
        radius=GUIDE_RAIL_RADIUS,
        length=BODY_LENGTH,
        center_x=0.0,
        center_z=-GUIDE_RAIL_OFFSET,
        material=body_metal,
    )
    add_x_cylinder(
        body,
        name="left_rail",
        radius=GUIDE_RAIL_RADIUS,
        length=BODY_LENGTH,
        center_x=0.0,
        center_y=-GUIDE_RAIL_OFFSET,
        material=body_metal,
    )
    add_x_cylinder(
        body,
        name="right_rail",
        radius=GUIDE_RAIL_RADIUS,
        length=BODY_LENGTH,
        center_x=0.0,
        center_y=GUIDE_RAIL_OFFSET,
        material=body_metal,
    )

    rear_center_x = -BODY_HALF_LENGTH + (REAR_CONNECTOR_LENGTH / 2.0)
    front_center_x = BODY_HALF_LENGTH - (FRONT_CONNECTOR_LENGTH / 2.0)
    for prefix, center_x, length in (
        ("rear", rear_center_x, REAR_CONNECTOR_LENGTH),
        ("front", front_center_x, FRONT_CONNECTOR_LENGTH),
    ):
        for suffix, center_y, center_z in (
            ("tr", CORNER_BLOCK_OFFSET, CORNER_BLOCK_OFFSET),
            ("tl", -CORNER_BLOCK_OFFSET, CORNER_BLOCK_OFFSET),
            ("br", CORNER_BLOCK_OFFSET, -CORNER_BLOCK_OFFSET),
            ("bl", -CORNER_BLOCK_OFFSET, -CORNER_BLOCK_OFFSET),
        ):
            add_box(
                body,
                name=f"{prefix}_{suffix}",
                size=(length, CORNER_BLOCK_SIZE, CORNER_BLOCK_SIZE),
                center=(center_x, center_y, center_z),
                material=body_metal,
            )

    plunger = model.part("plunger")
    add_x_cylinder(
        plunger,
        name="shaft",
        radius=SHAFT_RADIUS,
        length=SHAFT_END_X - SHAFT_START_X,
        center_x=(SHAFT_START_X + SHAFT_END_X) / 2.0,
        material=plunger_metal,
    )
    add_x_cylinder(
        plunger,
        name="front_stop",
        radius=FRONT_STOP_RADIUS,
        length=FRONT_STOP_LENGTH,
        center_x=FRONT_STOP_REAR_X + (FRONT_STOP_LENGTH / 2.0),
        material=plunger_metal,
    )
    add_x_cylinder(
        plunger,
        name="rear_knob",
        radius=KNOB_RADIUS,
        length=KNOB_THICKNESS,
        center_x=KNOB_FRONT_X - (KNOB_THICKNESS / 2.0),
        material=knob_polymer,
    )
    add_box(
        plunger,
        name="clevis_bridge",
        size=(CLEVIS_BRIDGE_LENGTH, CLEVIS_OUTER_WIDTH, CLEVIS_HEIGHT),
        center=(CLEVIS_REAR_X + (CLEVIS_BRIDGE_LENGTH / 2.0), 0.0, 0.0),
        material=plunger_metal,
    )
    add_box(
        plunger,
        name="clevis_upper_tine",
        size=(CLEVIS_LENGTH, CLEVIS_TINE_WIDTH, CLEVIS_HEIGHT / 2.0),
        center=(
            CLEVIS_REAR_X + (CLEVIS_LENGTH / 2.0),
            (CLEVIS_GAP_WIDTH / 2.0) + (CLEVIS_TINE_WIDTH / 2.0),
            0.0,
        ),
        material=plunger_metal,
    )
    add_box(
        plunger,
        name="clevis_lower_tine",
        size=(CLEVIS_LENGTH, CLEVIS_TINE_WIDTH, CLEVIS_HEIGHT / 2.0),
        center=(
            CLEVIS_REAR_X + (CLEVIS_LENGTH / 2.0),
            -((CLEVIS_GAP_WIDTH / 2.0) + (CLEVIS_TINE_WIDTH / 2.0)),
            0.0,
        ),
        material=plunger_metal,
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=PLUNGER_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("guide_body")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("body_to_plunger")

    top_rail = body.get_visual("top_rail")
    front_tr = body.get_visual("front_tr")
    rear_tr = body.get_visual("rear_tr")
    shaft = plunger.get_visual("shaft")
    stop = plunger.get_visual("front_stop")
    clevis_upper = plunger.get_visual("clevis_upper_tine")
    clevis_lower = plunger.get_visual("clevis_lower_tine")
    knob = plunger.get_visual("rear_knob")

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

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    limits = slide.motion_limits
    ctx.check(
        "plunger_joint_is_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type was {slide.articulation_type!r}",
    )
    ctx.check(
        "plunger_joint_axis_along_x",
        tuple(round(value, 6) for value in slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis was {slide.axis!r}",
    )
    ctx.check(
        "plunger_joint_limits_match_stroke",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-9
        and abs(limits.upper - PLUNGER_STROKE) < 1e-9,
        details=f"limits were {limits!r}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            plunger,
            body,
            elem_a=shaft,
            elem_b=top_rail,
            contact_tol=1e-6,
            name="shaft_guided_by_top_rail_lower",
        )
        ctx.expect_gap(
            plunger,
            body,
            axis="x",
            positive_elem=stop,
            negative_elem=front_tr,
            max_gap=0.0005,
            max_penetration=1e-6,
            name="front_stop_seats_on_body_lower",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="yz",
            elem_a=stop,
            elem_b=front_tr,
            min_overlap=0.004,
            name="front_stop_face_overlap_lower",
        )
        ctx.expect_gap(
            body,
            plunger,
            axis="x",
            positive_elem=rear_tr,
            negative_elem=knob,
            min_gap=PLUNGER_STROKE - 0.0005,
            max_gap=PLUNGER_STROKE + 0.0005,
            name="rear_knob_clear_of_body_lower",
        )
        ctx.expect_gap(
            plunger,
            body,
            axis="x",
            positive_elem=clevis_upper,
            negative_elem=front_tr,
            min_gap=0.0375,
            name="clevis_ahead_of_body_lower",
        )

    with ctx.pose({slide: PLUNGER_STROKE * 0.5}):
        ctx.fail_if_parts_overlap_in_current_pose(name="plunger_mid_no_overlap")
        ctx.fail_if_isolated_parts(name="plunger_mid_no_floating")
        ctx.expect_contact(
            plunger,
            body,
            elem_a=shaft,
            elem_b=top_rail,
            contact_tol=1e-6,
            name="shaft_guided_by_top_rail_mid",
        )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({slide: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="plunger_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="plunger_upper_no_floating")
            ctx.expect_contact(
                plunger,
                body,
                elem_a=shaft,
                elem_b=top_rail,
                contact_tol=1e-6,
                name="shaft_guided_by_top_rail_upper",
            )
            ctx.expect_gap(
                body,
                plunger,
                axis="x",
                positive_elem=rear_tr,
                negative_elem=knob,
                max_gap=0.0005,
                max_penetration=1e-6,
                name="rear_knob_seats_on_body_upper",
            )
            ctx.expect_overlap(
                plunger,
                body,
                axes="yz",
                elem_a=knob,
                elem_b=rear_tr,
                min_overlap=0.004,
                name="rear_knob_face_overlap_upper",
            )
            ctx.expect_gap(
                plunger,
                body,
                axis="x",
                positive_elem=stop,
                negative_elem=front_tr,
                min_gap=PLUNGER_STROKE - 0.0005,
                max_gap=PLUNGER_STROKE + 0.0005,
                name="front_stop_forward_clearance_upper",
            )
            ctx.expect_gap(
                plunger,
                body,
                axis="x",
                positive_elem=clevis_lower,
                negative_elem=front_tr,
                min_gap=0.0775,
                name="clevis_ahead_of_body_upper",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
