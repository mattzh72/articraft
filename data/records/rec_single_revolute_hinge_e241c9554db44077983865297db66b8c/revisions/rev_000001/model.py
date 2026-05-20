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


LEAF_THICKNESS = 0.004
BARREL_RADIUS = 0.006
PIN_RADIUS = 0.0026

BACK_LEAF_REACH = 0.055
STRAP_LEAF_REACH = 0.185
HINGE_WIDTH = 0.100
TIP_WIDTH = 0.042
SHOULDER_X = 0.070
TIP_CENTER_X = 0.164
STRAP_FLARE_X = 0.024
BACK_PLATE_MAX_X = -0.003
STRAP_PLATE_MIN_X = 0.003
STRAP_NECK_WIDTH = 0.036
STRAP_FLARE_WIDTH = 0.072

BACK_KNUCKLE_LENGTH = 0.029
KNUCKLE_CLEARANCE = 0.003
STRAP_KNUCKLE_LENGTH = (
    HINGE_WIDTH - 2.0 * BACK_KNUCKLE_LENGTH - 2.0 * KNUCKLE_CLEARANCE
)
BACK_KNUCKLE_CENTER = (
    STRAP_KNUCKLE_LENGTH / 2.0
    + KNUCKLE_CLEARANCE
    + BACK_KNUCKLE_LENGTH / 2.0
)

BACK_HOLE_RADIUS = 0.0028
STRAP_HOLE_RADIUS = 0.0032
PIN_HEAD_RADIUS = 0.0054
PIN_HEAD_THICKNESS = 0.0018
PIN_PEEN_RADIUS = 0.0044
PIN_PEEN_THICKNESS = 0.0014

MESH_TOLERANCE = 0.0002
MESH_ANGULAR_TOLERANCE = 0.05


def _y_cylinder(radius: float, length: float, y_center: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .workplane(offset=y_center)
        .circle(radius)
        .extrude(length / 2.0, both=True)
    )


def _knuckle_shell(length: float, hole_radius: float, y_center: float = 0.0) -> cq.Workplane:
    return _y_cylinder(BARREL_RADIUS, length, y_center).cut(
        _y_cylinder(hole_radius, length + 0.003, y_center)
    )


def _back_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center((-BACK_LEAF_REACH + BACK_PLATE_MAX_X) / 2.0, 0.0)
        .box(
            BACK_LEAF_REACH + BACK_PLATE_MAX_X,
            HINGE_WIDTH,
            LEAF_THICKNESS,
        )
        .edges("|Z")
        .fillet(0.0035)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.029, -0.024),
                (-0.029, 0.024),
            ]
        )
        .hole(2.0 * BACK_HOLE_RADIUS)
    )


def _strap_plate_shape() -> cq.Workplane:
    strap_neck_half_width = STRAP_NECK_WIDTH / 2.0
    strap_flare_half_width = STRAP_FLARE_WIDTH / 2.0
    strap_outline = (
        cq.Workplane("XY")
        .moveTo(STRAP_PLATE_MIN_X, -strap_neck_half_width)
        .lineTo(STRAP_FLARE_X, -strap_flare_half_width)
        .lineTo(SHOULDER_X, -HINGE_WIDTH / 2.0)
        .lineTo(TIP_CENTER_X, -TIP_WIDTH / 2.0)
        .threePointArc(
            (TIP_CENTER_X + TIP_WIDTH / 2.0, 0.0),
            (TIP_CENTER_X, TIP_WIDTH / 2.0),
        )
        .lineTo(SHOULDER_X, HINGE_WIDTH / 2.0)
        .lineTo(STRAP_FLARE_X, strap_flare_half_width)
        .lineTo(STRAP_PLATE_MIN_X, strap_neck_half_width)
        .close()
    )

    return (
        strap_outline.extrude(LEAF_THICKNESS / 2.0, both=True)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (0.047, 0.0),
                (0.095, 0.0),
                (0.143, 0.0),
            ]
        )
        .hole(2.0 * STRAP_HOLE_RADIUS)
    )

def _pin_shape() -> cq.Workplane:
    shank = _y_cylinder(PIN_RADIUS, HINGE_WIDTH)
    head = _y_cylinder(
        PIN_HEAD_RADIUS,
        PIN_HEAD_THICKNESS,
        HINGE_WIDTH / 2.0 + PIN_HEAD_THICKNESS / 2.0,
    )
    peen = _y_cylinder(
        PIN_PEEN_RADIUS,
        PIN_PEEN_THICKNESS,
        -HINGE_WIDTH / 2.0 - PIN_PEEN_THICKNESS / 2.0,
    )
    return shank.union(head).union(peen)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="strap_service_hinge")

    leaf_finish = model.material("leaf_finish", rgba=(0.28, 0.30, 0.33, 1.0))
    pin_finish = model.material("pin_finish", rgba=(0.69, 0.71, 0.75, 1.0))

    back_leaf = model.part("back_leaf")
    back_leaf.visual(
        mesh_from_cadquery(
            _back_plate_shape(),
            "back_plate_shell",
            tolerance=MESH_TOLERANCE,
            angular_tolerance=MESH_ANGULAR_TOLERANCE,
        ),
        material=leaf_finish,
        name="back_plate_shell",
    )
    back_leaf.visual(
        mesh_from_cadquery(
            _knuckle_shell(BACK_KNUCKLE_LENGTH, BACK_HOLE_RADIUS, BACK_KNUCKLE_CENTER),
            "back_knuckle_upper_shell",
            tolerance=MESH_TOLERANCE,
            angular_tolerance=MESH_ANGULAR_TOLERANCE,
        ),
        material=leaf_finish,
        name="back_knuckle_upper_shell",
    )
    back_leaf.visual(
        mesh_from_cadquery(
            _knuckle_shell(BACK_KNUCKLE_LENGTH, BACK_HOLE_RADIUS, -BACK_KNUCKLE_CENTER),
            "back_knuckle_lower_shell",
            tolerance=MESH_TOLERANCE,
            angular_tolerance=MESH_ANGULAR_TOLERANCE,
        ),
        material=leaf_finish,
        name="back_knuckle_lower_shell",
    )
    back_leaf.visual(
        mesh_from_cadquery(
            _pin_shape(),
            "hinge_pin_shell",
            tolerance=MESH_TOLERANCE,
            angular_tolerance=MESH_ANGULAR_TOLERANCE,
        ),
        material=pin_finish,
        name="hinge_pin_shell",
    )

    strap_leaf = model.part("strap_leaf")
    strap_leaf.visual(
        mesh_from_cadquery(
            _strap_plate_shape(),
            "strap_plate_shell",
            tolerance=MESH_TOLERANCE,
            angular_tolerance=MESH_ANGULAR_TOLERANCE,
        ),
        material=leaf_finish,
        name="strap_plate_shell",
    )
    strap_leaf.visual(
        mesh_from_cadquery(
            _knuckle_shell(STRAP_KNUCKLE_LENGTH, STRAP_HOLE_RADIUS),
            "strap_knuckle_shell",
            tolerance=MESH_TOLERANCE,
            angular_tolerance=MESH_ANGULAR_TOLERANCE,
        ),
        material=leaf_finish,
        name="strap_knuckle_shell",
    )

    model.articulation(
        "back_leaf_to_strap",
        ArticulationType.REVOLUTE,
        parent=back_leaf,
        child=strap_leaf,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_leaf = object_model.get_part("back_leaf")
    strap_leaf = object_model.get_part("strap_leaf")
    hinge = object_model.get_articulation("back_leaf_to_strap")

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
        strap_leaf,
        back_leaf,
        axis="x",
        min_gap=0.0055,
        max_gap=0.0075,
        positive_elem="strap_plate_shell",
        negative_elem="back_plate_shell",
        name="closed_leaf_plates_clear_at_hinge_line",
    )
    ctx.expect_overlap(
        strap_leaf,
        back_leaf,
        axes="y",
        min_overlap=0.09,
        elem_a="strap_plate_shell",
        elem_b="back_plate_shell",
        name="leaf_plates_share_hinge_width_alignment",
    )

    ctx.expect_origin_distance(
        strap_leaf,
        back_leaf,
        axes="yz",
        min_dist=0.0,
        max_dist=0.0001,
        name="strap_origin_stays_on_hinge_axis",
    )

    with ctx.pose({hinge: 1.05}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_service_open_angle")

    with ctx.pose({hinge: 0.0}):
        closed_box = ctx.part_world_aabb(strap_leaf)
    with ctx.pose({hinge: 1.05}):
        open_box = ctx.part_world_aabb(strap_leaf)

    opens_upward = False
    details = "strap leaf AABB unavailable"
    if closed_box is not None and open_box is not None:
        closed_max_x = closed_box[1][0]
        closed_max_z = closed_box[1][2]
        open_max_x = open_box[1][0]
        open_max_z = open_box[1][2]
        opens_upward = open_max_z > closed_max_z + 0.06 and open_max_x < closed_max_x - 0.02
        details = (
            f"closed max (x,z)=({closed_max_x:.4f}, {closed_max_z:.4f}), "
            f"open max (x,z)=({open_max_x:.4f}, {open_max_z:.4f})"
        )
    ctx.check("strap_opens_upward_about_pin_axis", opens_upward, details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
