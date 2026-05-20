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


HINGE_HEIGHT = 0.090
BARREL_RADIUS = 0.0068
PIN_RADIUS = 0.0020
PIN_CLEAR_RADIUS = PIN_RADIUS
PIN_CAP_RADIUS = 0.0048
PIN_CAP_HEIGHT = 0.0022
PIN_STUB_HEIGHT = 0.0012
LEAF_THICKNESS = 0.0030

FIXED_LEAF_HEIGHT = 0.050
FIXED_LEAF_OUTER_X = -0.034
FIXED_LEAF_INNER_X = -0.018

FIXED_STRAP_OUTER_X = -0.018
FIXED_STRAP_INNER_X = -0.006

MOVING_LEAF_HEIGHT = 0.084
MOVING_LEAF_OFFSET_Y = 0.0130
MOVING_LEAF_OUTER_X = -0.046
MOVING_LEAF_INNER_X = -0.018
MOVING_WEB_INNER_X = -0.028
MOVING_LUG_INNER_X = -0.003
MOVING_LUG_Y_CENTER = 0.0045
MOVING_LUG_THICKNESS = 0.0045
MOVING_BRIDGE_Y_CENTER = 0.0090
MOVING_BRIDGE_THICKNESS = 0.0055
MOVING_KNUCKLE_RADIUS = 0.0064
MOVING_KNUCKLE_BORE_RADIUS = 0.00235
MOVING_KNUCKLE_SUPPORT_THICKNESS = 0.0042
MOVING_KNUCKLE_SUPPORT_Y = 0.0092

SPACER_STOP_OUTER_X = -0.020
SPACER_STOP_INNER_X = -0.012
SPACER_STOP_Y_THICKNESS = 0.0048
SPACER_STOP_Y_CENTER = 0.0039
SPACER_STOP_HEIGHT = 0.018

HOLE_DIAMETER = 0.0045
CSK_DIAMETER = 0.0090

FIXED_SEGMENTS = (
    (-0.031, 0.017),
    (0.000, 0.017),
    (0.031, 0.017),
)
MOVING_SEGMENTS = (
    (-0.0155, 0.011),
    (0.0155, 0.011),
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_segment(radius: float, length: float, z_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, z_center - length / 2.0))
    )


def _fixed_plate_shape() -> cq.Workplane:
    leaf = _box(
        (
            FIXED_LEAF_INNER_X - FIXED_LEAF_OUTER_X,
            LEAF_THICKNESS,
            FIXED_LEAF_HEIGHT,
        ),
        (
            (FIXED_LEAF_OUTER_X + FIXED_LEAF_INNER_X) / 2.0,
            0.0,
            0.0,
        ),
    )
    leaf = leaf.edges("|Z").fillet(0.0012)
    leaf = (
        leaf.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, -0.014), (0.0, 0.014)])
        .cskHole(HOLE_DIAMETER, CSK_DIAMETER, 90)
    )
    return leaf


def _fixed_barrel_shape() -> cq.Workplane:
    barrel = None
    for z_center, length in FIXED_SEGMENTS:
        lug = _box(
            (
                FIXED_STRAP_INNER_X - FIXED_STRAP_OUTER_X,
                LEAF_THICKNESS,
                length,
            ),
            (
                (FIXED_STRAP_OUTER_X + FIXED_STRAP_INNER_X) / 2.0,
                0.0,
                z_center,
            ),
        )
        segment = _cylinder_segment(BARREL_RADIUS, length, z_center).union(lug)
        barrel = segment if barrel is None else barrel.union(segment)

    pin = _cylinder_segment(PIN_RADIUS, HINGE_HEIGHT, 0.0)
    top_cap = _cylinder_segment(
        PIN_CAP_RADIUS,
        PIN_CAP_HEIGHT,
        HINGE_HEIGHT / 2.0 + PIN_CAP_HEIGHT / 2.0,
    )
    bottom_cap = _cylinder_segment(
        PIN_CAP_RADIUS,
        PIN_CAP_HEIGHT,
        -HINGE_HEIGHT / 2.0 - PIN_CAP_HEIGHT / 2.0,
    )

    return barrel.union(pin).union(top_cap).union(bottom_cap)


def _fixed_spacer_stop_shape() -> cq.Workplane:
    stop = _box(
        (
            SPACER_STOP_INNER_X - SPACER_STOP_OUTER_X,
            SPACER_STOP_Y_THICKNESS,
            SPACER_STOP_HEIGHT,
        ),
        (
            (SPACER_STOP_OUTER_X + SPACER_STOP_INNER_X) / 2.0,
            SPACER_STOP_Y_CENTER,
            0.0,
        ),
    )
    return stop.edges("|Z").fillet(0.0008)


def _moving_plate_shape() -> cq.Workplane:
    leaf = _box(
        (
            MOVING_LEAF_INNER_X - MOVING_LEAF_OUTER_X,
            LEAF_THICKNESS,
            MOVING_LEAF_HEIGHT,
        ),
        (
            (MOVING_LEAF_OUTER_X + MOVING_LEAF_INNER_X) / 2.0,
            MOVING_LEAF_OFFSET_Y,
            0.0,
        ),
    )
    leaf = leaf.edges("|Z").fillet(0.0011)
    leaf = (
        leaf.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, -0.026), (0.0, 0.0), (0.0, 0.026)])
        .cskHole(HOLE_DIAMETER, CSK_DIAMETER, 90)
    )
    return leaf


def _moving_bridge_shape() -> cq.Workplane:
    bridge = _box(
        (
            MOVING_LEAF_INNER_X - MOVING_WEB_INNER_X,
            MOVING_BRIDGE_THICKNESS,
            MOVING_LEAF_HEIGHT,
        ),
        (
            (MOVING_WEB_INNER_X + MOVING_LEAF_INNER_X) / 2.0,
            MOVING_BRIDGE_Y_CENTER,
            0.0,
        ),
    )
    return bridge.edges("|Z").fillet(0.0008)


def _moving_knuckles_shape() -> cq.Workplane:
    knuckles = None
    support_outer_x = -MOVING_KNUCKLE_RADIUS
    support_center_x = (MOVING_LEAF_INNER_X + support_outer_x) / 2.0
    support_length = support_outer_x - MOVING_LEAF_INNER_X

    for z_center, length in MOVING_SEGMENTS:
        ring = _cylinder_segment(MOVING_KNUCKLE_RADIUS, length, z_center).cut(
            _cylinder_segment(MOVING_KNUCKLE_BORE_RADIUS, length + 0.0015, z_center)
        )
        segment = ring.union(
            _box(
                (support_length, MOVING_KNUCKLE_SUPPORT_THICKNESS, length - 0.0006),
                (support_center_x, MOVING_KNUCKLE_SUPPORT_Y, z_center),
            )
        )
        knuckles = segment if knuckles is None else knuckles.union(segment)

    return knuckles


def _fixed_inertial() -> Inertial:
    return Inertial.from_geometry(
        Box((0.034, 0.014, HINGE_HEIGHT)),
        mass=0.22,
        origin=Origin(xyz=(-0.017, 0.0, 0.0)),
    )


def _moving_inertial() -> Inertial:
    return Inertial.from_geometry(
        Box((0.046, 0.017, MOVING_LEAF_HEIGHT)),
        mass=0.24,
        origin=Origin(xyz=(-0.023, 0.0065, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_inspection_hatch_hinge")

    fixed_finish = model.material("fixed_finish", rgba=(0.37, 0.39, 0.42, 1.0))
    moving_finish = model.material("moving_finish", rgba=(0.64, 0.66, 0.69, 1.0))
    pin_finish = model.material("pin_finish", rgba=(0.18, 0.19, 0.20, 1.0))

    fixed_leaf = model.part("fixed_leaf", inertial=_fixed_inertial())
    fixed_leaf.visual(
        mesh_from_cadquery(_fixed_plate_shape(), "fixed_leaf_plate"),
        material=fixed_finish,
        name="fixed_plate",
    )
    fixed_leaf.visual(
        mesh_from_cadquery(_fixed_barrel_shape(), "fixed_leaf_barrel_v3"),
        material=pin_finish,
        name="fixed_barrel",
    )
    fixed_leaf.visual(
        mesh_from_cadquery(_fixed_spacer_stop_shape(), "fixed_leaf_spacer_stop_v1"),
        material=fixed_finish,
        name="fixed_spacer_stop",
    )

    moving_leaf = model.part("moving_leaf", inertial=_moving_inertial())
    moving_leaf.visual(
        mesh_from_cadquery(_moving_plate_shape(), "moving_leaf_plate"),
        material=moving_finish,
        name="moving_plate",
    )
    moving_leaf.visual(
        mesh_from_cadquery(_moving_bridge_shape(), "moving_leaf_bridge"),
        material=moving_finish,
        name="moving_bridge",
    )
    moving_leaf.visual(
        mesh_from_cadquery(_moving_knuckles_shape(), "moving_leaf_knuckles_v6"),
        material=moving_finish,
        name="moving_knuckles",
    )

    model.articulation(
        "hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=2.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_leaf = object_model.get_part("fixed_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    fixed_plate = fixed_leaf.get_visual("fixed_plate")
    fixed_barrel = fixed_leaf.get_visual("fixed_barrel")
    fixed_spacer_stop = fixed_leaf.get_visual("fixed_spacer_stop")
    moving_plate = moving_leaf.get_visual("moving_plate")
    moving_bridge = moving_leaf.get_visual("moving_bridge")
    moving_knuckles = moving_leaf.get_visual("moving_knuckles")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=16)

    ctx.expect_overlap(
        moving_leaf,
        fixed_leaf,
        axes="z",
        elem_a=moving_knuckles,
        elem_b=fixed_barrel,
        min_overlap=0.040,
        name="knuckle_segments_track_barrel_height",
    )
    ctx.expect_contact(
        moving_leaf,
        fixed_leaf,
        elem_a=moving_bridge,
        elem_b=fixed_spacer_stop,
        name="closed_leaf_is_grounded_by_spacer_stop",
    )
    ctx.expect_gap(
        moving_leaf,
        fixed_leaf,
        axis="y",
        positive_elem=moving_plate,
        negative_elem=fixed_plate,
        min_gap=0.009,
        max_gap=0.012,
        name="closed_leaf_faces_stay_barrel_spaced",
    )
    ctx.expect_overlap(
        moving_leaf,
        fixed_leaf,
        axes="xz",
        elem_a=moving_plate,
        elem_b=fixed_plate,
        min_overlap=0.015,
        name="closed_leaf_footprints_read_as_offset_hinge",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
