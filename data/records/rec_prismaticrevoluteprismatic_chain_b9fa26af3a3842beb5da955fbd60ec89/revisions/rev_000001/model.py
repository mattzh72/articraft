from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PLATE_W = 0.12
PLATE_T = 0.01
PLATE_H = 0.34

RAIL_W = 0.05
RAIL_D = 0.014
RAIL_H = 0.24
RAIL_Z0 = 0.05
RAIL_CENTER_Y = (PLATE_T / 2.0) + (RAIL_D / 2.0)
RAIL_FRONT_Y = (PLATE_T / 2.0) + RAIL_D

CARR_W = 0.07
CARR_D = 0.032
CARR_H = 0.09
CARR_BACK = 0.06
SLIDE_STROKE = 0.12

HINGE_X = 0.065
BARREL_R = 0.016
BARREL_LEN = BODY_W = 0.04
WRIST_LOWER = -0.25
WRIST_UPPER = 1.05

BODY_MOUNT_L = 0.02
BODY_LEN = 0.15
BODY_H = 0.05
BODY_W = 0.04
BODY_OFFSET_Y = RAIL_FRONT_Y + (BARREL_LEN / 2.0)

EXT_ORIGIN_X = BODY_MOUNT_L
EXT_W = 0.022
EXT_H = 0.03
EXT_LEN = 0.12
EXT_STROKE = 0.06


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_slide_wristed_extension")

    model.material("plate_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("carriage_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("body_blue", rgba=(0.23, 0.33, 0.46, 1.0))
    model.material("extension_aluminum", rgba=(0.82, 0.83, 0.84, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((PLATE_W, PLATE_T, PLATE_H)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_H / 2.0)),
        material="plate_gray",
        name="plate_panel",
    )
    side_plate.visual(
        Box((RAIL_W, RAIL_D, RAIL_H)),
        origin=Origin(
            xyz=(
                HINGE_X - ((CARR_BACK + BARREL_R) / 2.0),
                RAIL_CENTER_Y,
                RAIL_Z0 + (RAIL_H / 2.0),
            )
        ),
        material="carriage_silver",
        name="slide_rail",
    )
    side_plate.inertial = Inertial.from_geometry(
        Box((PLATE_W, 0.03, PLATE_H)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, PLATE_H / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARR_BACK - BARREL_R, CARR_D, CARR_H)),
        origin=Origin(
            xyz=(
                -((CARR_BACK + BARREL_R) / 2.0),
                RAIL_FRONT_Y + (CARR_D / 2.0),
                0.0,
            )
        ),
        material="carriage_silver",
        name="carriage_block",
    )
    carriage.visual(
        Box((0.018, CARR_D, 0.038)),
        origin=Origin(
            xyz=(-0.026, RAIL_FRONT_Y + (CARR_D / 2.0), (CARR_H / 2.0) - 0.019)
        ),
        material="carriage_silver",
        name="carriage_cap",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARR_BACK, CARR_D, CARR_H)),
        mass=1.2,
        origin=Origin(
            xyz=(-CARR_BACK / 2.0, RAIL_FRONT_Y + (CARR_D / 2.0), 0.0)
        ),
    )

    body = model.part("hinged_body")
    body.visual(
        Cylinder(radius=BARREL_R, length=BARREL_LEN),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material="body_blue",
        name="hinge_barrel",
    )
    body.visual(
        Box((BODY_MOUNT_L + 0.002, BODY_W, BODY_H)),
        origin=Origin(xyz=((BODY_MOUNT_L / 2.0) - 0.001, 0.0, 0.0)),
        material="body_blue",
        name="mount_block",
    )
    body.visual(
        Box((BODY_LEN, BODY_W, (BODY_H - EXT_H) / 2.0)),
        origin=Origin(
            xyz=(
                BODY_MOUNT_L + (BODY_LEN / 2.0),
                0.0,
                (EXT_H / 2.0) + ((BODY_H - EXT_H) / 4.0),
            )
        ),
        material="body_blue",
        name="top_guide",
    )
    body.visual(
        Box((BODY_LEN, BODY_W, (BODY_H - EXT_H) / 2.0)),
        origin=Origin(
            xyz=(
                BODY_MOUNT_L + (BODY_LEN / 2.0),
                0.0,
                -((EXT_H / 2.0) + ((BODY_H - EXT_H) / 4.0)),
            )
        ),
        material="body_blue",
        name="bottom_guide",
    )
    body.visual(
        Box((BODY_LEN, (BODY_W - EXT_W) / 2.0, EXT_H)),
        origin=Origin(
            xyz=(
                BODY_MOUNT_L + (BODY_LEN / 2.0),
                (EXT_W / 2.0) + ((BODY_W - EXT_W) / 4.0),
                0.0,
            )
        ),
        material="body_blue",
        name="left_guide",
    )
    body.visual(
        Box((BODY_LEN, (BODY_W - EXT_W) / 2.0, EXT_H)),
        origin=Origin(
            xyz=(
                BODY_MOUNT_L + (BODY_LEN / 2.0),
                -((EXT_W / 2.0) + ((BODY_W - EXT_W) / 4.0)),
                0.0,
            )
        ),
        material="body_blue",
        name="right_guide",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_MOUNT_L + BODY_LEN, BODY_W, BODY_H)),
        mass=0.9,
        origin=Origin(xyz=((BODY_MOUNT_L + BODY_LEN) / 2.0, 0.0, 0.0)),
    )

    extension = model.part("extension")
    extension.visual(
        Box((EXT_LEN, EXT_W, EXT_H)),
        origin=Origin(xyz=(EXT_LEN / 2.0, 0.0, 0.0)),
        material="extension_aluminum",
        name="extension_beam",
    )
    extension.inertial = Inertial.from_geometry(
        Box((EXT_LEN, EXT_W, EXT_H)),
        mass=0.35,
        origin=Origin(xyz=(EXT_LEN / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "side_plate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=carriage,
        origin=Origin(xyz=(HINGE_X, 0.0, RAIL_Z0 + (CARR_H / 2.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.22,
            lower=0.0,
            upper=SLIDE_STROKE,
        ),
    )
    model.articulation(
        "carriage_to_hinged_body",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=body,
        origin=Origin(xyz=(0.0, BODY_OFFSET_Y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=WRIST_LOWER,
            upper=WRIST_UPPER,
        ),
    )
    model.articulation(
        "hinged_body_to_extension",
        ArticulationType.PRISMATIC,
        parent=body,
        child=extension,
        origin=Origin(xyz=(EXT_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.20,
            lower=0.0,
            upper=EXT_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    carriage = object_model.get_part("carriage")
    body = object_model.get_part("hinged_body")
    extension = object_model.get_part("extension")
    slide = object_model.get_articulation("side_plate_to_carriage")
    wrist = object_model.get_articulation("carriage_to_hinged_body")
    telescope = object_model.get_articulation("hinged_body_to_extension")

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
        "articulation_topology",
        (
            slide.articulation_type == ArticulationType.PRISMATIC
            and wrist.articulation_type == ArticulationType.REVOLUTE
            and telescope.articulation_type == ArticulationType.PRISMATIC
            and tuple(slide.axis) == (0.0, 0.0, 1.0)
            and tuple(wrist.axis) == (0.0, -1.0, 0.0)
            and tuple(telescope.axis) == (1.0, 0.0, 0.0)
        ),
        details="Expected prismatic → revolute → prismatic with vertical slide, wrist pitch, and outward extension.",
    )

    with ctx.pose({slide: 0.0, wrist: 0.0, telescope: 0.0}):
        ctx.expect_contact(
            carriage,
            side_plate,
            name="carriage_seats_on_side_plate_rail",
        )
        ctx.expect_contact(
            body,
            carriage,
            name="hinged_body_is_supported_by_carriage_bearing",
        )
        ctx.expect_contact(
            extension,
            body,
            name="extension_is_guided_inside_body",
        )
        ctx.expect_within(
            extension,
            body,
            axes="yz",
            margin=0.002,
            name="extension_cross_section_stays_within_body_guides",
        )

    with ctx.pose({slide: SLIDE_STROKE, wrist: 0.0, telescope: 0.0}):
        ctx.expect_contact(
            carriage,
            side_plate,
            name="carriage_remains_supported_at_upper_slide_pose",
        )

    with ctx.pose({slide: 0.0, wrist: WRIST_UPPER, telescope: 0.0}):
        ctx.expect_contact(
            body,
            carriage,
            name="wrist_bearing_stays_supported_when_raised",
        )

    with ctx.pose({slide: 0.0, wrist: 0.0, telescope: EXT_STROKE}):
        ctx.expect_contact(
            extension,
            body,
            name="extension_remains_supported_when_extended",
        )

    with ctx.pose({slide: 0.0, wrist: 0.0, telescope: 0.0}):
        carriage_low = ctx.part_world_position(carriage)
        body_closed_aabb = ctx.part_world_aabb(body)
        extension_closed = ctx.part_world_position(extension)

    with ctx.pose({slide: SLIDE_STROKE, wrist: 0.0, telescope: 0.0}):
        carriage_high = ctx.part_world_position(carriage)

    with ctx.pose({slide: 0.0, wrist: WRIST_UPPER, telescope: 0.0}):
        body_open_aabb = ctx.part_world_aabb(body)

    with ctx.pose({slide: 0.0, wrist: 0.0, telescope: EXT_STROKE}):
        extension_open = ctx.part_world_position(extension)

    slide_rise = (
        None
        if carriage_low is None or carriage_high is None
        else carriage_high[2] - carriage_low[2]
    )
    body_lift = (
        None
        if body_closed_aabb is None or body_open_aabb is None
        else body_open_aabb[1][2] - body_closed_aabb[1][2]
    )
    extension_travel = (
        None
        if extension_closed is None or extension_open is None
        else extension_open[0] - extension_closed[0]
    )

    ctx.check(
        "slide_motion_moves_carriage_upward",
        slide_rise is not None and slide_rise > 0.10,
        details=f"Expected >0.10 m vertical rise, got {slide_rise!r}.",
    )
    ctx.check(
        "wrist_motion_lifts_body",
        body_lift is not None and body_lift > 0.05,
        details=f"Expected >0.05 m body lift, got {body_lift!r}.",
    )
    ctx.check(
        "extension_motion_moves_outward",
        extension_travel is not None and extension_travel > 0.05,
        details=f"Expected >0.05 m outward extension, got {extension_travel!r}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
