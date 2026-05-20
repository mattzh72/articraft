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


GUIDE_LENGTH = 0.42
GUIDE_WIDTH = 0.12
BASE_THICK = 0.012
RAIL_LENGTH = 0.35
RAIL_WIDTH = 0.024
RAIL_HEIGHT = 0.022
RAIL_Y = 0.036

CARRIAGE_LENGTH = 0.10
CARRIAGE_WIDTH = 0.084
CARRIAGE_PAD_LENGTH = 0.090
CARRIAGE_PAD_WIDTH = 0.018
CARRIAGE_PAD_HEIGHT = 0.008
CARRIAGE_WEB_HEIGHT = 0.022
CARRIAGE_BRIDGE_HEIGHT = 0.022
CARRIAGE_BRIDGE_Z = CARRIAGE_PAD_HEIGHT + CARRIAGE_WEB_HEIGHT
CARRIAGE_FRONT_PLATE_THICK = 0.012
CARRIAGE_FRONT_PLATE_WIDTH = 0.056
CARRIAGE_FRONT_PLATE_HEIGHT = 0.044
CARRIAGE_FRONT_PLATE_Z = 0.036
CARRIAGE_REAR_CAP_LENGTH = 0.026
CARRIAGE_REAR_CAP_WIDTH = 0.060
CARRIAGE_REAR_CAP_HEIGHT = 0.012
CARRIAGE_REAR_CAP_Z = CARRIAGE_BRIDGE_Z + CARRIAGE_BRIDGE_HEIGHT

NOSE_COLLAR_LENGTH = 0.010
NOSE_COLLAR_RADIUS = 0.022
NOSE_BODY_LENGTH = 0.024
NOSE_BODY_RADIUS = 0.017
NOSE_FLANGE_LENGTH = 0.006
NOSE_FLANGE_RADIUS = 0.028

SLIDE_START_X = -0.11
SLIDE_TRAVEL = 0.22
NOSE_AXIS_Z = 0.058
NOSE_MOUNT_X = 0.061
NOSE_SWING_LIMIT = 2.7


def _build_guide_body_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        GUIDE_LENGTH,
        GUIDE_WIDTH,
        BASE_THICK,
        centered=(True, True, False),
    )

    left_rail = (
        cq.Workplane("XY")
        .box(
            RAIL_LENGTH,
            RAIL_WIDTH,
            RAIL_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, RAIL_Y, BASE_THICK))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(
            RAIL_LENGTH,
            RAIL_WIDTH,
            RAIL_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, -RAIL_Y, BASE_THICK))
    )
    left_end_cheek = (
        cq.Workplane("XY")
        .box(
            0.034,
            0.034,
            0.018,
            centered=(True, True, False),
        )
        .translate((GUIDE_LENGTH * 0.5 - 0.045, RAIL_Y, BASE_THICK))
    )
    right_end_cheek = (
        cq.Workplane("XY")
        .box(0.034, 0.034, 0.018, centered=(True, True, False))
        .translate((GUIDE_LENGTH * 0.5 - 0.045, -RAIL_Y, BASE_THICK))
    )
    left_rear_cheek = (
        cq.Workplane("XY")
        .box(0.034, 0.034, 0.018, centered=(True, True, False))
        .translate((-GUIDE_LENGTH * 0.5 + 0.045, RAIL_Y, BASE_THICK))
    )
    right_rear_cheek = (
        cq.Workplane("XY")
        .box(0.034, 0.034, 0.018, centered=(True, True, False))
        .translate((-GUIDE_LENGTH * 0.5 + 0.045, -RAIL_Y, BASE_THICK))
    )

    body = base.union(left_rail).union(right_rail)
    body = body.union(left_end_cheek).union(right_end_cheek).union(left_rear_cheek).union(right_rear_cheek)
    return body


def _build_carriage_shape() -> cq.Workplane:
    left_pad = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_PAD_LENGTH,
            CARRIAGE_PAD_WIDTH,
            CARRIAGE_PAD_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, RAIL_Y, 0.0))
    )
    right_pad = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_PAD_LENGTH,
            CARRIAGE_PAD_WIDTH,
            CARRIAGE_PAD_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, -RAIL_Y, 0.0))
    )
    left_web = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_PAD_LENGTH,
            CARRIAGE_PAD_WIDTH,
            CARRIAGE_WEB_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, RAIL_Y, CARRIAGE_PAD_HEIGHT))
    )
    right_web = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_PAD_LENGTH,
            CARRIAGE_PAD_WIDTH,
            CARRIAGE_WEB_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, -RAIL_Y, CARRIAGE_PAD_HEIGHT))
    )
    bridge = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LENGTH,
            CARRIAGE_WIDTH,
            CARRIAGE_BRIDGE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CARRIAGE_BRIDGE_Z))
    )
    front_mount = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_FRONT_PLATE_THICK,
            CARRIAGE_FRONT_PLATE_WIDTH,
            CARRIAGE_FRONT_PLATE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((NOSE_MOUNT_X - (CARRIAGE_FRONT_PLATE_THICK * 0.5), 0.0, CARRIAGE_FRONT_PLATE_Z))
    )
    rear_cap = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_REAR_CAP_LENGTH,
            CARRIAGE_REAR_CAP_WIDTH,
            CARRIAGE_REAR_CAP_HEIGHT,
            centered=(True, True, False),
        )
        .translate((-0.026, 0.0, CARRIAGE_REAR_CAP_Z))
    )

    body = left_pad.union(right_pad).union(left_web).union(right_web)
    body = body.union(bridge).union(front_mount).union(rear_cap)
    return body


def _build_nose_shape() -> cq.Workplane:
    nose = cq.Workplane("YZ").circle(NOSE_COLLAR_RADIUS).extrude(NOSE_COLLAR_LENGTH)
    nose = nose.faces(">X").workplane().circle(NOSE_BODY_RADIUS).extrude(NOSE_BODY_LENGTH)
    return nose.faces(">X").workplane().circle(NOSE_FLANGE_RADIUS).extrude(NOSE_FLANGE_LENGTH)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_body_slide")

    model.material("guide_gray", rgba=(0.50, 0.53, 0.57, 1.0))
    model.material("carriage_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("nose_steel", rgba=(0.74, 0.76, 0.79, 1.0))

    guide_body = model.part("guide_body")
    guide_body.visual(
        mesh_from_cadquery(_build_guide_body_shape(), "guide_body"),
        material="guide_gray",
        name="guide_shell",
    )
    guide_body.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, BASE_THICK + RAIL_HEIGHT)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICK + RAIL_HEIGHT) * 0.5)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "carriage"),
        material="carriage_black",
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.112, CARRIAGE_WIDTH, 0.080)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    nose_cartridge = model.part("nose_cartridge")
    nose_cartridge.visual(
        mesh_from_cadquery(_build_nose_shape(), "nose_cartridge"),
        material="nose_steel",
        name="nose_shell",
    )
    nose_cartridge.inertial = Inertial.from_geometry(
        Box((NOSE_COLLAR_LENGTH + NOSE_BODY_LENGTH + NOSE_FLANGE_LENGTH, 0.056, 0.056)),
        mass=0.65,
        origin=Origin(
            xyz=(
                (NOSE_COLLAR_LENGTH + NOSE_BODY_LENGTH + NOSE_FLANGE_LENGTH) * 0.5,
                0.0,
                0.0,
            )
        ),
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=carriage,
        origin=Origin(xyz=(SLIDE_START_X, 0.0, BASE_THICK + RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.30,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_nose",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=nose_cartridge,
        origin=Origin(
            xyz=(
                NOSE_MOUNT_X,
                0.0,
                NOSE_AXIS_Z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-NOSE_SWING_LIMIT,
            upper=NOSE_SWING_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_body = object_model.get_part("guide_body")
    carriage = object_model.get_part("carriage")
    nose_cartridge = object_model.get_part("nose_cartridge")
    slide = object_model.get_articulation("guide_to_carriage")
    nose_joint = object_model.get_articulation("carriage_to_nose")

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
        "part_tree_present",
        guide_body.name == "guide_body"
        and carriage.name == "carriage"
        and nose_cartridge.name == "nose_cartridge",
        "Expected guide_body, carriage, and nose_cartridge parts.",
    )
    ctx.check(
        "articulation_topology",
        slide.parent == "guide_body"
        and slide.child == "carriage"
        and nose_joint.parent == "carriage"
        and nose_joint.child == "nose_cartridge",
        "Expected guide_body -> carriage slide and carriage -> nose_cartridge rotary joint.",
    )
    ctx.check(
        "joint_axes_are_axial",
        tuple(slide.axis) == (1.0, 0.0, 0.0) and tuple(nose_joint.axis) == (1.0, 0.0, 0.0),
        "Both primary mechanisms should run on the X axis.",
    )

    with ctx.pose({slide: 0.0, nose_joint: 0.0}):
        ctx.expect_contact(
            carriage,
            guide_body,
            name="carriage_supported_retracted",
        )
        ctx.expect_within(
            carriage,
            guide_body,
            axes="y",
            margin=0.0,
            name="carriage_centered_in_split_body",
        )
        ctx.expect_contact(
            nose_cartridge,
            carriage,
            name="nose_cartridge_seated_on_carriage",
        )
        ctx.expect_overlap(
            nose_cartridge,
            carriage,
            axes="yz",
            min_overlap=0.040,
            name="nose_bearing_face_aligned",
        )

    with ctx.pose({slide: SLIDE_TRAVEL, nose_joint: NOSE_SWING_LIMIT}):
        ctx.expect_contact(
            carriage,
            guide_body,
            name="carriage_supported_extended",
        )
        ctx.expect_contact(
            nose_cartridge,
            carriage,
            name="nose_keeps_bearing_contact_when_rotated",
        )

    with ctx.pose({slide: 0.0}):
        retracted_x = ctx.part_world_position(carriage)[0]
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_x = ctx.part_world_position(carriage)[0]
    ctx.check(
        "slide_travel_matches_joint",
        abs((extended_x - retracted_x) - SLIDE_TRAVEL) <= 1e-6,
        f"Expected carriage translation of {SLIDE_TRAVEL:.3f} m, got {(extended_x - retracted_x):.6f} m.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
