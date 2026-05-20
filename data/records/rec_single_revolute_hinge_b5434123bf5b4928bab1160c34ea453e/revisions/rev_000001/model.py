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


LEAF_WIDTH = 0.090
PLATE_THICKNESS = 0.012
PLATE_LENGTH = 0.235
BARREL_OUTER_RADIUS = 0.018
PIN_RADIUS = 0.008
PIN_HEAD_RADIUS = 0.012
PIN_HEAD_THICKNESS = 0.004
KNUCKLE_GAP = 0.002
KNUCKLE_LENGTH = (LEAF_WIDTH - 4.0 * KNUCKLE_GAP) / 5.0
KNUCKLE_STEP = KNUCKLE_LENGTH + KNUCKLE_GAP
PLATE_HOLE_DIAMETER = 0.015
PLATE_EDGE_FILLET = 0.004
WEB_THICKNESS_X = BARREL_OUTER_RADIUS


def _plate_shape(x_sign: int) -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_LENGTH, LEAF_WIDTH, PLATE_THICKNESS, centered=(True, True, True))
        .edges("|Z")
        .fillet(PLATE_EDGE_FILLET)
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.072, 0.0),
                (0.0, 0.0),
                (0.072, 0.0),
            ]
        )
        .hole(PLATE_HOLE_DIAMETER)
    )
    x_center = x_sign * (BARREL_OUTER_RADIUS + PLATE_LENGTH / 2.0)
    return plate.translate((x_center, 0.0, 0.0))


def _y_cylinder(radius: float, length: float, y_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((0.0, y_center, 0.0))
    )


def _y_tube(outer_radius: float, inner_radius: float, length: float, y_center: float) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    inner_length = length + 0.004
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(inner_length)
        .translate((0.0, 0.0, -inner_length / 2.0))
    )
    return (
        outer.cut(inner)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((0.0, y_center, 0.0))
    )


def _knuckle_shape(y_center: float) -> cq.Workplane:
    return _y_tube(BARREL_OUTER_RADIUS, PIN_RADIUS, KNUCKLE_LENGTH, y_center)


def _web_shape(x_sign: int, y_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            WEB_THICKNESS_X,
            KNUCKLE_LENGTH,
            PLATE_THICKNESS,
            centered=(True, True, True),
        )
        .translate(
            (x_sign * (BARREL_OUTER_RADIUS + WEB_THICKNESS_X / 2.0), y_center, 0.0)
        )
    )


def _pin_shape() -> cq.Workplane:
    return (
        _y_cylinder(PIN_RADIUS, LEAF_WIDTH, 0.0)
        .union(
            _y_cylinder(
                PIN_HEAD_RADIUS,
                PIN_HEAD_THICKNESS,
                LEAF_WIDTH / 2.0 + PIN_HEAD_THICKNESS / 2.0,
            )
        )
        .union(
            _y_cylinder(
                PIN_HEAD_RADIUS,
                PIN_HEAD_THICKNESS,
                -LEAF_WIDTH / 2.0 - PIN_HEAD_THICKNESS / 2.0,
            )
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_strap_hinge")

    leaf_finish = model.material("leaf_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    pin_finish = model.material("pin_finish", rgba=(0.55, 0.57, 0.60, 1.0))

    left_leaf = model.part("left_leaf")
    right_leaf = model.part("right_leaf")
    hinge_pin = model.part("hinge_pin")

    left_knuckle_centers = (-2.0 * KNUCKLE_STEP, 0.0, 2.0 * KNUCKLE_STEP)
    right_knuckle_centers = (-1.0 * KNUCKLE_STEP, 1.0 * KNUCKLE_STEP)

    left_leaf.visual(
        mesh_from_cadquery(_plate_shape(-1), "left_leaf_plate"),
        material=leaf_finish,
        name="plate",
    )
    right_leaf.visual(
        mesh_from_cadquery(_plate_shape(1), "right_leaf_plate"),
        material=leaf_finish,
        name="plate",
    )

    for name, y_center in (
        ("web_bottom", -2.0 * KNUCKLE_STEP),
        ("web_center", 0.0),
        ("web_top", 2.0 * KNUCKLE_STEP),
    ):
        left_leaf.visual(
            mesh_from_cadquery(_web_shape(-1, y_center), f"left_leaf_{name}"),
            material=leaf_finish,
            name=name,
        )
    for name, y_center in (
        ("web_lower", -1.0 * KNUCKLE_STEP),
        ("web_upper", 1.0 * KNUCKLE_STEP),
    ):
        right_leaf.visual(
            mesh_from_cadquery(_web_shape(1, y_center), f"right_leaf_{name}"),
            material=leaf_finish,
            name=name,
        )

    for name, y_center in (
        ("knuckle_bottom", -2.0 * KNUCKLE_STEP),
        ("knuckle_center", 0.0),
        ("knuckle_top", 2.0 * KNUCKLE_STEP),
    ):
        left_leaf.visual(
            mesh_from_cadquery(_knuckle_shape(y_center), f"left_leaf_{name}"),
            material=leaf_finish,
            name=name,
        )
    for name, y_center in (
        ("knuckle_lower", -1.0 * KNUCKLE_STEP),
        ("knuckle_upper", 1.0 * KNUCKLE_STEP),
    ):
        right_leaf.visual(
            mesh_from_cadquery(_knuckle_shape(y_center), f"right_leaf_{name}"),
            material=leaf_finish,
            name=name,
        )

    hinge_pin.visual(
        mesh_from_cadquery(_pin_shape(), "hinge_pin"),
        material=pin_finish,
        name="pin",
    )

    model.articulation(
        "left_leaf_to_right_leaf",
        ArticulationType.REVOLUTE,
        parent=left_leaf,
        child=right_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-3.0,
            upper=0.0,
        ),
    )
    model.articulation(
        "left_leaf_to_pin",
        ArticulationType.FIXED,
        parent=left_leaf,
        child=hinge_pin,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    hinge_pin = object_model.get_part("hinge_pin")
    hinge = object_model.get_articulation("left_leaf_to_right_leaf")

    left_plate = left_leaf.get_visual("plate")
    right_plate = right_leaf.get_visual("plate")
    pin_visual = hinge_pin.get_visual("pin")
    left_knuckle_names = ("knuckle_bottom", "knuckle_center", "knuckle_top")
    right_knuckle_names = ("knuckle_lower", "knuckle_upper")

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
    for knuckle_name in left_knuckle_names:
        ctx.allow_overlap(
            hinge_pin,
            left_leaf,
            elem_a=pin_visual,
            elem_b=left_leaf.get_visual(knuckle_name),
            reason="The steel hinge pin runs concentrically through the left leaf barrel knuckles.",
        )
    for knuckle_name in right_knuckle_names:
        ctx.allow_overlap(
            hinge_pin,
            right_leaf,
            elem_a=pin_visual,
            elem_b=right_leaf.get_visual(knuckle_name),
            reason="The steel hinge pin runs concentrically through the right leaf barrel knuckles.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "heavy_strap_hinge_parts_present",
        len(object_model.parts) == 3,
        f"Expected 3 parts, found {len(object_model.parts)}.",
    )
    ctx.check(
        "hinge_axis_follows_barrel",
        tuple(round(value, 6) for value in hinge.axis) == (0.0, 1.0, 0.0),
        f"Expected barrel axis (0, 1, 0), found {hinge.axis}.",
    )
    limits = hinge.motion_limits
    ctx.check(
        "hinge_has_realistic_swing_limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -2.8
        and -0.05 <= limits.upper <= 0.05,
        f"Unexpected hinge limits: {limits}.",
    )

    ctx.expect_contact(
        hinge_pin,
        left_leaf,
        elem_a=pin_visual,
        elem_b=left_leaf.get_visual("knuckle_center"),
        contact_tol=0.0005,
        name="pin_is_centered_in_left_leaf_barrel_stack",
    )
    ctx.expect_origin_distance(
        hinge_pin,
        right_leaf,
        axes="xz",
        min_dist=0.0,
        max_dist=0.001,
        name="right_leaf_rotates_about_the_pin_axis",
    )
    ctx.expect_gap(
        right_leaf,
        left_leaf,
        axis="x",
        min_gap=0.035,
        max_gap=0.038,
        positive_elem=right_plate,
        negative_elem=left_plate,
        name="strap_plates_clear_each_other_at_rest",
    )

    with ctx.pose({hinge: -1.57}):
        ctx.expect_origin_distance(
            hinge_pin,
            right_leaf,
            axes="xz",
            min_dist=0.0,
            max_dist=0.001,
            name="right_leaf_stays_coaxial_with_pin_when_opened",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_right_angle")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
