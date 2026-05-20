from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.014
PLATE_W = 0.220
PLATE_H = 0.300
PLATE_CORNER_R = 0.012
MOUNT_HOLE_D = 0.012
MOUNT_HOLE_Y = 0.070
MOUNT_HOLE_Z = 0.085

RAIL_L = 0.290
RAIL_W = 0.032
RAIL_H = 0.032
RAIL_CENTER_Z = 0.090
RAIL_EDGE_R = 0.002

SHOE_L = 0.060
SHOE_W = 0.078
SHOE_H = 0.060
SHOE_CENTER_X = PLATE_T + (SHOE_L / 2.0)
SHOE_CENTER_Z = SHOE_H / 2.0

CHEEK_L = 0.044
CHEEK_T = 0.008
CHEEK_H = 0.086
CHEEK_CENTER_X = PLATE_T + (CHEEK_L / 2.0)
CHEEK_CENTER_Z = CHEEK_H / 2.0
CHEEK_CENTER_Y = (SHOE_W / 2.0) - (CHEEK_T / 2.0)

CARRIAGE_L = 0.088
CARRIAGE_W = 0.080
CARRIAGE_H = 0.068
ARM_L = 0.040
ARM_W = 0.048
ARM_H = 0.084
ARM_CENTER_X = 0.018
ARM_CENTER_Z = (CARRIAGE_H / 2.0) + (ARM_H / 2.0) - 0.004
FACE_T = 0.012
FACE_SIZE = 0.100
FACE_CENTER_X = 0.044
FACE_CENTER_Z = 0.072
FACE_RECESS = 0.004

HOME_X = 0.118
TRAVEL = 0.145


def _box(length: float, width: float, height: float, center: tuple[float, float, float]):
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _build_mounting_support_shape():
    plate = _box(
        PLATE_T,
        PLATE_W,
        PLATE_H,
        (PLATE_T / 2.0, 0.0, PLATE_H / 2.0),
    ).edges("|X").fillet(PLATE_CORNER_R)

    hole_cutter = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-MOUNT_HOLE_Y, -MOUNT_HOLE_Z),
                (MOUNT_HOLE_Y, -MOUNT_HOLE_Z),
                (-MOUNT_HOLE_Y, MOUNT_HOLE_Z),
                (MOUNT_HOLE_Y, MOUNT_HOLE_Z),
            ]
        )
        .circle(MOUNT_HOLE_D / 2.0)
        .extrude(PLATE_T + 0.004)
        .translate((-0.002, 0.0, PLATE_H / 2.0))
    )
    plate = plate.cut(hole_cutter)

    shoe = _box(SHOE_L, SHOE_W, SHOE_H, (SHOE_CENTER_X, 0.0, SHOE_CENTER_Z))
    left_cheek = _box(
        CHEEK_L,
        CHEEK_T,
        CHEEK_H,
        (CHEEK_CENTER_X, CHEEK_CENTER_Y, CHEEK_CENTER_Z),
    )
    right_cheek = _box(
        CHEEK_L,
        CHEEK_T,
        CHEEK_H,
        (CHEEK_CENTER_X, -CHEEK_CENTER_Y, CHEEK_CENTER_Z),
    )

    return (
        plate.union(shoe)
        .union(left_cheek)
        .union(right_cheek)
        .edges("|Y")
        .fillet(0.002)
    )


def _build_rail_shape():
    rail = _box(
        RAIL_L,
        RAIL_W,
        RAIL_H,
        (PLATE_T + (RAIL_L / 2.0), 0.0, RAIL_CENTER_Z),
    )
    return rail.edges("|X").fillet(RAIL_EDGE_R)


def _build_carriage_shape():
    sleeve = _box(CARRIAGE_L, CARRIAGE_W, CARRIAGE_H, (0.0, 0.0, 0.0))
    rail_tunnel = _box(CARRIAGE_L + 0.004, RAIL_W, RAIL_H, (0.0, 0.0, 0.0))
    sleeve = sleeve.cut(rail_tunnel)

    upright = _box(ARM_L, ARM_W, ARM_H, (ARM_CENTER_X, 0.0, ARM_CENTER_Z))
    face = (
        _box(FACE_T, FACE_SIZE, FACE_SIZE, (FACE_CENTER_X, 0.0, FACE_CENTER_Z))
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(FACE_SIZE * 0.68, FACE_SIZE * 0.68)
        .cutBlind(-FACE_RECESS)
    )

    carriage = sleeve.union(upright).union(face)
    return carriage.edges("|X").fillet(0.002)


def _build_front_face_shape():
    outer = _box(0.012, FACE_SIZE, FACE_SIZE, (0.094, 0.0, 0.0))
    opening = _box(0.016, 0.052, 0.052, (0.094, 0.0, 0.0))
    return outer.cut(opening)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_shuttle_module")

    model.material("support_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("rail_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("carriage_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("face_gray", rgba=(0.55, 0.57, 0.60, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((PLATE_T, PLATE_W, PLATE_H)),
        origin=Origin(xyz=(PLATE_T / 2.0, 0.0, PLATE_H / 2.0)),
        material="support_gray",
        name="mounting_support",
    )
    side_plate.visual(
        Box((0.028, 0.060, 0.024)),
        origin=Origin(xyz=(PLATE_T + 0.014, 0.0, RAIL_CENTER_Z + 0.028)),
        material="support_gray",
        name="upper_rail_brace",
    )
    side_plate.visual(
        Box((0.028, 0.060, 0.024)),
        origin=Origin(xyz=(PLATE_T + 0.014, 0.0, RAIL_CENTER_Z - 0.028)),
        material="support_gray",
        name="lower_rail_brace",
    )
    side_plate.visual(
        Box((RAIL_L, RAIL_W, RAIL_H)),
        origin=Origin(xyz=(PLATE_T + (RAIL_L / 2.0), 0.0, RAIL_CENTER_Z)),
        material="rail_steel",
        name="guide_rail",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.100, 0.060, 0.016)),
        origin=Origin(xyz=(0.038, 0.0, 0.024)),
        material="carriage_black",
        name="upper_runner",
    )
    carriage.visual(
        Box((0.100, 0.060, 0.016)),
        origin=Origin(xyz=(0.038, 0.0, -0.024)),
        material="carriage_black",
        name="lower_runner",
    )
    carriage.visual(
        Box((0.100, 0.012, 0.064)),
        origin=Origin(xyz=(0.038, 0.030, 0.0)),
        material="carriage_black",
        name="left_web",
    )
    carriage.visual(
        Box((0.100, 0.012, 0.064)),
        origin=Origin(xyz=(0.038, -0.030, 0.0)),
        material="carriage_black",
        name="carriage_body",
    )
    carriage.visual(
        mesh_from_cadquery(_build_front_face_shape(), "front_face"),
        material="face_gray",
        name="front_face",
    )

    model.articulation(
        "side_plate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=carriage,
        origin=Origin(xyz=(HOME_X, 0.0, RAIL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.40,
            lower=0.0,
            upper=TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("side_plate_to_carriage")
    rail = side_plate.get_visual("guide_rail")

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
        "parts_exist",
        side_plate is not None and carriage is not None and slide is not None,
        "Expected side_plate, carriage, and side_plate_to_carriage articulation.",
    )
    ctx.check(
        "prismatic_axis_points_forward",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"Expected +X prismatic axis, got {slide.axis!r}.",
    )
    ctx.check(
        "prismatic_limits_are_forward_extension",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 0.14,
        f"Unexpected prismatic limits: {slide.motion_limits!r}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            carriage,
            side_plate,
            elem_b=rail,
            name="carriage_contacts_rail_retracted",
        )
        ctx.expect_origin_gap(
            carriage,
            side_plate,
            axis="x",
            min_gap=HOME_X - 0.003,
            max_gap=HOME_X + 0.003,
            name="carriage_starts_retracted_near_plate",
        )
        ctx.expect_within(
            carriage,
            side_plate,
            axes="y",
            margin=0.0,
            name="carriage_stays_within_module_width_retracted",
        )

    with ctx.pose({slide: TRAVEL}):
        ctx.expect_contact(
            carriage,
            side_plate,
            elem_b=rail,
            name="carriage_contacts_rail_extended",
        )
        ctx.expect_origin_gap(
            carriage,
            side_plate,
            axis="x",
            min_gap=(HOME_X + TRAVEL) - 0.003,
            max_gap=(HOME_X + TRAVEL) + 0.003,
            name="carriage_extends_forward_along_rail",
        )
        ctx.expect_within(
            carriage,
            side_plate,
            axes="y",
            margin=0.0,
            name="carriage_stays_within_module_width_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
