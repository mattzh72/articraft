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


PLATE_WIDTH = 0.220
PLATE_DEPTH = 0.140
PLATE_THICKNESS = 0.012

BODY_WIDTH = 0.094
BODY_DEPTH = 0.076
BODY_HEIGHT = 0.030
BODY_CENTER_Y = -0.006

CHEEK_INNER_SPAN = 0.120
CHEEK_THICKNESS = 0.018
CHEEK_BOX_DEPTH = 0.064
CHEEK_BOX_HEIGHT = 0.044
CHEEK_CENTER_Y = 0.006

PIVOT_Y = 0.014
PIVOT_Z = 0.060
BEARING_HOLE_RADIUS = 0.011
SHAFT_RADIUS = 0.0106
CHEEK_BOSS_RADIUS = 0.022

GUSSET_THICKNESS = 0.014
GUSSET_TOP_Z = 0.040
FRONT_GUSSET_Y = 0.018
REAR_GUSSET_Y = -0.020

TRUNNION_BARREL_LENGTH = CHEEK_INNER_SPAN + 2.0 * CHEEK_THICKNESS - 0.0008
HEAD_WEB_WIDTH = 0.030
HEAD_WEB_DEPTH = 0.022
HEAD_WEB_HEIGHT = 0.026
HEAD_WEB_CENTER_Y = 0.016
HEAD_WEB_CENTER_Z = 0.012

HEAD_NECK_WIDTH = 0.072
HEAD_NECK_DEPTH = 0.024
HEAD_NECK_HEIGHT = 0.020
HEAD_NECK_CENTER_Y = 0.030
HEAD_NECK_CENTER_Z = 0.023

HEAD_FACE_WIDTH = 0.094
HEAD_FACE_DEPTH = 0.016
HEAD_FACE_HEIGHT = 0.026
HEAD_FACE_CENTER_Y = 0.046
HEAD_FACE_CENTER_Z = 0.034


def _trunnion_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def _gusset(sign: float, y_center: float) -> cq.Workplane:
    body_edge_x = sign * (BODY_WIDTH / 2.0)
    cheek_inner_x = sign * (CHEEK_INNER_SPAN / 2.0)
    profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (body_edge_x, PLATE_THICKNESS),
                (cheek_inner_x, PLATE_THICKNESS),
                (cheek_inner_x, GUSSET_TOP_Z),
            ]
        )
        .close()
        .extrude(GUSSET_THICKNESS, both=True)
        .translate((0.0, y_center, 0.0))
    )
    return profile


def _make_housing_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_WIDTH, PLATE_DEPTH, PLATE_THICKNESS)
        .translate((0.0, 0.0, PLATE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.080, -0.045),
                (-0.080, 0.045),
                (0.080, -0.045),
                (0.080, 0.045),
            ]
        )
        .hole(0.010)
    )

    body = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .translate((0.0, BODY_CENTER_Y, PLATE_THICKNESS + BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )

    housing = plate.union(body)

    cheek_center_x = CHEEK_INNER_SPAN / 2.0 + CHEEK_THICKNESS / 2.0
    for sign in (-1.0, 1.0):
        cheek_x = sign * cheek_center_x
        cheek_box = cq.Workplane("XY").box(
            CHEEK_THICKNESS,
            CHEEK_BOX_DEPTH,
            CHEEK_BOX_HEIGHT,
        ).translate((cheek_x, CHEEK_CENTER_Y, PLATE_THICKNESS + CHEEK_BOX_HEIGHT / 2.0))
        cheek_boss = _trunnion_cylinder(CHEEK_BOSS_RADIUS, CHEEK_THICKNESS).translate(
            (cheek_x, PIVOT_Y, PIVOT_Z)
        )
        housing = housing.union(cheek_box).union(cheek_boss)
        housing = housing.union(_gusset(sign, FRONT_GUSSET_Y))
        housing = housing.union(_gusset(sign, REAR_GUSSET_Y))

        bearing_hole = _trunnion_cylinder(
            BEARING_HOLE_RADIUS,
            CHEEK_THICKNESS + 0.004,
        ).translate((cheek_x, PIVOT_Y, PIVOT_Z))
        housing = housing.cut(bearing_hole)

    relief = cq.Workplane("XY").box(
        CHEEK_INNER_SPAN - 0.022,
        0.036,
        0.018,
    ).translate((0.0, 0.002, PLATE_THICKNESS + 0.009))
    housing = housing.cut(relief)

    return housing


def _make_head_shape() -> cq.Workplane:
    barrel = _trunnion_cylinder(SHAFT_RADIUS, TRUNNION_BARREL_LENGTH)
    web = cq.Workplane("XY").box(
        HEAD_WEB_WIDTH,
        HEAD_WEB_DEPTH,
        HEAD_WEB_HEIGHT,
    ).translate((0.0, HEAD_WEB_CENTER_Y, HEAD_WEB_CENTER_Z))
    neck = cq.Workplane("XY").box(
        HEAD_NECK_WIDTH,
        HEAD_NECK_DEPTH,
        HEAD_NECK_HEIGHT,
    ).translate((0.0, HEAD_NECK_CENTER_Y, HEAD_NECK_CENTER_Z))
    face = cq.Workplane("XY").box(
        HEAD_FACE_WIDTH,
        HEAD_FACE_DEPTH,
        HEAD_FACE_HEIGHT,
    ).translate((0.0, HEAD_FACE_CENTER_Y, HEAD_FACE_CENTER_Z))
    return barrel.union(web).union(neck).union(face)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_axis_trunnion_module")

    base_finish = model.material("base_finish", rgba=(0.27, 0.29, 0.31, 1.0))
    head_finish = model.material("head_finish", rgba=(0.72, 0.75, 0.78, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_shape(), "housing"),
        material=base_finish,
        name="housing_shell",
    )
    housing.inertial = Inertial.from_geometry(
        Box((PLATE_WIDTH, PLATE_DEPTH, 0.082)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_make_head_shape(), "head"),
        material=head_finish,
        name="head_shell",
    )
    head.inertial = Inertial.from_geometry(
        Box((HEAD_FACE_WIDTH, 0.076, 0.086)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.032, 0.026)),
    )

    model.articulation(
        "pitch_tilt",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=head,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.6,
            lower=-0.42,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    head = object_model.get_part("head")
    pitch_tilt = object_model.get_articulation("pitch_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0005)
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
        "pitch_axis_is_horizontal_x",
        tuple(round(v, 6) for v in pitch_tilt.axis) == (1.0, 0.0, 0.0),
        f"axis={pitch_tilt.axis}",
    )

    with ctx.pose({pitch_tilt: 0.0}):
        ctx.expect_contact(
            head,
            housing,
            contact_tol=0.0005,
            name="closed_trunnion_is_supported",
        )
        closed_aabb = ctx.part_world_aabb(head)

    with ctx.pose({pitch_tilt: pitch_tilt.motion_limits.upper}):
        ctx.expect_contact(
            head,
            housing,
            contact_tol=0.0005,
            name="open_trunnion_remains_supported",
        )
        open_aabb = ctx.part_world_aabb(head)

    if closed_aabb is not None and open_aabb is not None:
        closed_center_y = 0.5 * (closed_aabb[0][1] + closed_aabb[1][1])
        closed_center_z = 0.5 * (closed_aabb[0][2] + closed_aabb[1][2])
        open_center_y = 0.5 * (open_aabb[0][1] + open_aabb[1][1])
        open_center_z = 0.5 * (open_aabb[0][2] + open_aabb[1][2])
        ctx.check(
            "positive_pitch_lifts_head",
            open_center_z > closed_center_z + 0.005,
            (
                f"closed_center_z={closed_center_z:.4f}, "
                f"open_center_z={open_center_z:.4f}"
            ),
        )
        ctx.check(
            "positive_pitch_swings_head_back",
            open_center_y < closed_center_y - 0.008,
            (
                f"closed_center_y={closed_center_y:.4f}, "
                f"open_center_y={open_center_y:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
