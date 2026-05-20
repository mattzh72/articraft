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


BED_X = 1.22
BED_Y = 1.68
BED_Z = 0.22
TABLE_X = 0.84
TABLE_Y = 1.30
TABLE_Z = 0.05
TABLE_RECESS = 0.035

SIDE_RAIL_X = 0.18
SIDE_RAIL_Y = 1.46
SIDE_RAIL_Z = 0.30
SIDE_RAIL_CENTER_X = 0.47
GUIDE_CAP_X = 0.14
GUIDE_CAP_Y = 1.40
GUIDE_CAP_Z = 0.02
GUIDE_TOP_Z = BED_Z + SIDE_RAIL_Z + GUIDE_CAP_Z

PORTAL_SPAN_X = 1.06
PORTAL_BEAM_Y = 0.18
PORTAL_BEAM_Z = 0.16
PORTAL_LEG_X = 0.14
PORTAL_LEG_Y = 0.22
PORTAL_LEG_Z = 0.56
PORTAL_TRUCK_X = 0.22
PORTAL_TRUCK_Y = 0.24
PORTAL_TRUCK_Z = 0.14
PORTAL_PAD_Z = 0.02
PORTAL_LEG_CENTER_X = SIDE_RAIL_CENTER_X
PORTAL_BOTTOM_LOCAL_Z = -0.76
PORTAL_FRAME_Z = GUIDE_TOP_Z - PORTAL_BOTTOM_LOCAL_Z
PORTAL_TRAVEL = 0.42

CARRIAGE_REAR_X = 0.24
CARRIAGE_REAR_Y = 0.12
CARRIAGE_REAR_Z = 0.08
CARRIAGE_BODY_X = 0.28
CARRIAGE_BODY_Y = 0.08
CARRIAGE_BODY_Z = 0.22
CARRIAGE_MAST_X = 0.20
CARRIAGE_MAST_Y = 0.06
CARRIAGE_MAST_Z = 0.38
CARRIAGE_RAIL_X = 0.022
CARRIAGE_RAIL_Y = 0.018
CARRIAGE_RAIL_Z = 0.34
CARRIAGE_FRONT_FACE_Y = 0.07
CARRIAGE_TRAVEL = 0.24

HEAD_SLIDE_X = 0.16
HEAD_SLIDE_Y = 0.06
HEAD_SLIDE_Z = 0.58
HEAD_PLATE_X = 0.22
HEAD_PLATE_Y = 0.02
HEAD_PLATE_Z = 0.22
HEAD_BODY_X = 0.18
HEAD_BODY_Y = 0.05
HEAD_BODY_Z = 0.12
HEAD_RETAINED_INSERTION = 0.08
HEAD_TRAVEL = 0.22


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_machine_bed")

    model.material("machine_gray", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("portal_gray", rgba=(0.64, 0.66, 0.69, 1.0))
    model.material("carriage_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("head_light", rgba=(0.84, 0.85, 0.87, 1.0))

    bed = model.part("bed")
    bed.visual(
        mesh_from_cadquery(_make_bed_shape(), "bed_frame"),
        material="machine_gray",
        name="bed_frame",
    )
    bed.inertial = Inertial.from_geometry(
        Box((BED_X, BED_Y, GUIDE_TOP_Z)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z / 2.0)),
    )

    portal = model.part("portal")
    portal.visual(
        mesh_from_cadquery(_make_portal_shape(), "portal_frame"),
        material="portal_gray",
        name="portal_frame",
    )
    portal.inertial = Inertial.from_geometry(
        Box((PORTAL_SPAN_X + 0.02, PORTAL_TRUCK_Y, PORTAL_FRAME_Z - GUIDE_TOP_Z + 0.10)),
        mass=185.0,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "beam_carriage"),
        material="carriage_dark",
        name="beam_carriage",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_BODY_X, 0.14, 0.44)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.07, -0.22)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_make_head_shape(), "vertical_head"),
        material="head_light",
        name="vertical_head",
    )
    head.inertial = Inertial.from_geometry(
        Box((HEAD_PLATE_X, 0.08, 0.60)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.04, -0.21)),
    )

    model.articulation(
        "bed_to_portal",
        ArticulationType.PRISMATIC,
        parent=bed,
        child=portal,
        origin=Origin(xyz=(0.0, 0.0, PORTAL_FRAME_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-PORTAL_TRAVEL,
            upper=PORTAL_TRAVEL,
            effort=4000.0,
            velocity=0.8,
        ),
    )
    model.articulation(
        "portal_to_carriage",
        ArticulationType.PRISMATIC,
        parent=portal,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.04, -0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
            effort=1200.0,
            velocity=0.9,
        ),
    )
    model.articulation(
        "carriage_to_head",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(0.0, 0.096, -0.26)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=HEAD_TRAVEL,
            effort=900.0,
            velocity=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed")
    portal = object_model.get_part("portal")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("head")

    bed_to_portal = object_model.get_articulation("bed_to_portal")
    portal_to_carriage = object_model.get_articulation("portal_to_carriage")
    carriage_to_head = object_model.get_articulation("carriage_to_head")

    ctx.check("bed exists", bed is not None)
    ctx.check("portal exists", portal is not None)
    ctx.check("carriage exists", carriage is not None)
    ctx.check("head exists", head is not None)

    ctx.expect_contact(portal, bed, name="portal rests on the side rails")
    ctx.expect_contact(carriage, portal, name="carriage stays mounted to the beam")
    ctx.expect_contact(head, carriage, name="vertical head stays mounted to the carriage")

    portal_rest = ctx.part_world_position(portal)
    carriage_rest = ctx.part_world_position(carriage)
    head_rest = ctx.part_world_position(head)

    with ctx.pose({bed_to_portal: PORTAL_TRAVEL}):
        ctx.expect_contact(portal, bed, name="portal remains supported at beam travel limit")
        portal_far = ctx.part_world_position(portal)

    with ctx.pose({portal_to_carriage: CARRIAGE_TRAVEL}):
        ctx.expect_contact(carriage, portal, name="carriage remains supported at x travel limit")
        carriage_far = ctx.part_world_position(carriage)

    with ctx.pose({carriage_to_head: HEAD_TRAVEL}):
        ctx.expect_contact(head, carriage, name="head remains supported at z travel limit")
        head_far = ctx.part_world_position(head)

    ctx.check(
        "portal travels along bed length",
        portal_rest is not None
        and portal_far is not None
        and portal_far[1] > portal_rest[1] + 0.20,
        details=f"rest={portal_rest}, far={portal_far}",
    )
    ctx.check(
        "carriage travels across the beam",
        carriage_rest is not None
        and carriage_far is not None
        and carriage_far[0] > carriage_rest[0] + 0.20,
        details=f"rest={carriage_rest}, far={carriage_far}",
    )
    ctx.check(
        "head moves downward",
        head_rest is not None and head_far is not None and head_far[2] < head_rest[2] - 0.10,
        details=f"rest={head_rest}, far={head_far}",
    )

    return ctx.report()


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _make_bed_shape() -> cq.Workplane:
    bed = _box(BED_X, BED_Y, BED_Z, (0.0, 0.0, BED_Z / 2.0))
    pocket = _box(TABLE_X, TABLE_Y, TABLE_RECESS, (0.0, 0.0, BED_Z - TABLE_RECESS / 2.0))
    table = _box(
        TABLE_X,
        TABLE_Y,
        TABLE_Z,
        (0.0, 0.0, BED_Z - TABLE_RECESS + TABLE_Z / 2.0),
    )
    left_rail = _box(
        SIDE_RAIL_X,
        SIDE_RAIL_Y,
        SIDE_RAIL_Z,
        (SIDE_RAIL_CENTER_X, 0.0, BED_Z + SIDE_RAIL_Z / 2.0),
    )
    right_rail = _box(
        SIDE_RAIL_X,
        SIDE_RAIL_Y,
        SIDE_RAIL_Z,
        (-SIDE_RAIL_CENTER_X, 0.0, BED_Z + SIDE_RAIL_Z / 2.0),
    )
    left_cap = _box(
        GUIDE_CAP_X,
        GUIDE_CAP_Y,
        GUIDE_CAP_Z,
        (SIDE_RAIL_CENTER_X, 0.0, BED_Z + SIDE_RAIL_Z + GUIDE_CAP_Z / 2.0),
    )
    right_cap = _box(
        GUIDE_CAP_X,
        GUIDE_CAP_Y,
        GUIDE_CAP_Z,
        (-SIDE_RAIL_CENTER_X, 0.0, BED_Z + SIDE_RAIL_Z + GUIDE_CAP_Z / 2.0),
    )

    return (
        bed.cut(pocket)
        .union(table)
        .union(left_rail)
        .union(right_rail)
        .union(left_cap)
        .union(right_cap)
    )


def _make_portal_shape() -> cq.Workplane:
    top_beam = _box(1.04, PORTAL_BEAM_Y, PORTAL_BEAM_Z, (0.0, 0.0, -0.02))
    left_leg = _box(PORTAL_LEG_X, PORTAL_LEG_Y, PORTAL_LEG_Z, (PORTAL_LEG_CENTER_X, 0.0, -0.38))
    right_leg = _box(PORTAL_LEG_X, PORTAL_LEG_Y, PORTAL_LEG_Z, (-PORTAL_LEG_CENTER_X, 0.0, -0.38))
    lower_stiffener = _box(0.78, 0.06, 0.06, (0.0, -0.03, -0.12))

    portal = top_beam.union(left_leg).union(right_leg).union(lower_stiffener)
    for x_sign in (-1.0, 1.0):
        x = x_sign * PORTAL_LEG_CENTER_X
        truck = _box(PORTAL_TRUCK_X, PORTAL_TRUCK_Y, PORTAL_TRUCK_Z, (x, 0.0, -0.67))
        pad = _box(0.18, 0.20, PORTAL_PAD_Z, (x, 0.0, -0.75))
        cheek = _box(0.12, 0.12, 0.18, (x, 0.0, -0.19))
        portal = portal.union(truck).union(pad).union(cheek)

    return portal


def _make_carriage_shape() -> cq.Workplane:
    saddle = _box(0.28, 0.07, 0.06, (0.0, 0.035, -0.03))
    hanger = _box(0.22, 0.05, 0.18, (0.0, 0.025, -0.15))
    bridge = _box(0.16, 0.04, 0.10, (0.0, 0.05, -0.11))
    front_plate = _box(0.18, 0.032, 0.20, (0.0, 0.072, -0.16))
    lower_block = _box(0.14, 0.03, 0.10, (0.0, 0.07, -0.31))
    left_rail = _box(0.022, 0.012, 0.30, (0.07, 0.09, -0.27))
    right_rail = _box(0.022, 0.012, 0.30, (-0.07, 0.09, -0.27))

    return (
        saddle.union(hanger)
        .union(bridge)
        .union(front_plate)
        .union(lower_block)
        .union(left_rail)
        .union(right_rail)
    )


def _make_head_shape() -> cq.Workplane:
    slide = _box(
        HEAD_SLIDE_X,
        HEAD_SLIDE_Y,
        HEAD_SLIDE_Z,
        (0.0, 0.03, -HEAD_SLIDE_Z / 2.0 + HEAD_RETAINED_INSERTION),
    )
    body = _box(HEAD_BODY_X, HEAD_BODY_Y, HEAD_BODY_Z, (0.0, 0.035, -0.18 + HEAD_RETAINED_INSERTION))
    faceplate = _box(HEAD_PLATE_X, HEAD_PLATE_Y, HEAD_PLATE_Z, (0.0, 0.07, -0.47 + HEAD_RETAINED_INSERTION))
    return slide.union(body).union(faceplate)


# >>> USER_CODE_END

object_model = build_object_model()
