from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BODY_X = 0.100
BODY_Y = 0.070
BODY_Z = 0.050

MOUNT_X = 0.090
MOUNT_Y = 0.010
MOUNT_Z = 0.068
MOUNT_HOLE_D = 0.006
MOUNT_HOLE_X = 0.028
MOUNT_HOLE_Z = 0.022

RAIL_X = 0.160
RAIL_Y = 0.008
RAIL_Z = 0.008
RAIL_CENTER_Y = BODY_Y / 2.0 + RAIL_Y / 2.0
RAIL_OFFSET_Z = 0.012

JAW_TRAVEL = 0.040
LEFT_JAW_OPEN_X = -0.067
RIGHT_JAW_OPEN_X = 0.067
JAW_ORIGIN_Y = RAIL_CENTER_Y

JAW_CARRIAGE_X = 0.028
JAW_CARRIAGE_Y = 0.028
JAW_CARRIAGE_Z = 0.046
JAW_FINGER_X = 0.010
JAW_FINGER_Y = 0.024
JAW_FINGER_Z = 0.028
JAW_FINGER_OFFSET_X = JAW_CARRIAGE_X / 2.0 + JAW_FINGER_X / 2.0
JAW_FINGER_OFFSET_Y = 0.018


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_X, BODY_Y, BODY_Z)
    front_pocket = (
        cq.Workplane("XY")
        .box(0.050, 0.012, 0.032)
        .translate((0.0, BODY_Y / 2.0 - 0.006, 0.0))
    )
    return body.cut(front_pocket)


def _mount_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(MOUNT_X, MOUNT_Y, MOUNT_Z)
    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-MOUNT_HOLE_X, -MOUNT_HOLE_Z),
                (-MOUNT_HOLE_X, MOUNT_HOLE_Z),
                (MOUNT_HOLE_X, -MOUNT_HOLE_Z),
                (MOUNT_HOLE_X, MOUNT_HOLE_Z),
            ]
        )
        .hole(MOUNT_HOLE_D)
    )
    return plate


def _jaw_shape(side: str) -> cq.Workplane:
    inward_sign = 1.0 if side == "left" else -1.0

    carriage = cq.Workplane("XY").box(JAW_CARRIAGE_X, JAW_CARRIAGE_Y, JAW_CARRIAGE_Z)
    finger = cq.Workplane("XY").box(JAW_FINGER_X, JAW_FINGER_Y, JAW_FINGER_Z).translate(
        (
            inward_sign * JAW_FINGER_OFFSET_X,
            JAW_FINGER_OFFSET_Y,
            0.0,
        )
    )
    jaw = carriage.union(finger)

    rail_slot = cq.Workplane("XY").box(JAW_CARRIAGE_X + 0.010, RAIL_Y + 0.0005, RAIL_Z + 0.0005)
    jaw = jaw.cut(rail_slot.translate((0.0, 0.0, RAIL_OFFSET_Z)))
    jaw = jaw.cut(rail_slot.translate((0.0, 0.0, -RAIL_OFFSET_Z)))
    return jaw


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_slide_gripper", assets=ASSETS)

    body_mat = model.material("body_anodized", rgba=(0.23, 0.25, 0.28, 1.0))
    rail_mat = model.material("rail_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    jaw_mat = model.material("jaw_black", rgba=(0.16, 0.17, 0.19, 1.0))
    plate_mat = model.material("mount_plate", rgba=(0.52, 0.54, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell.obj", assets=ASSETS),
        name="body_shell",
        material=body_mat,
    )
    body.inertial = Inertial.from_geometry(Box((BODY_X, BODY_Y, BODY_Z)), mass=1.6)

    mount_plate = model.part("mount_plate")
    mount_plate.visual(
        mesh_from_cadquery(_mount_plate_shape(), "mount_plate.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="plate_shell",
        material=plate_mat,
    )
    mount_plate.inertial = Inertial.from_geometry(Box((MOUNT_X, MOUNT_Y, MOUNT_Z)), mass=0.35)

    upper_rail = model.part("upper_rail")
    upper_rail.visual(
        Box((RAIL_X, RAIL_Y, RAIL_Z)),
        name="rail_shell",
        material=rail_mat,
    )
    upper_rail.inertial = Inertial.from_geometry(Box((RAIL_X, RAIL_Y, RAIL_Z)), mass=0.18)

    lower_rail = model.part("lower_rail")
    lower_rail.visual(
        Box((RAIL_X, RAIL_Y, RAIL_Z)),
        name="rail_shell",
        material=rail_mat,
    )
    lower_rail.inertial = Inertial.from_geometry(Box((RAIL_X, RAIL_Y, RAIL_Z)), mass=0.18)

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        mesh_from_cadquery(_jaw_shape("left"), "left_jaw.obj", assets=ASSETS),
        name="jaw_shell",
        material=jaw_mat,
    )
    left_jaw.inertial = Inertial.from_geometry(
        Box((JAW_CARRIAGE_X + JAW_FINGER_X, JAW_CARRIAGE_Y + 0.010, JAW_CARRIAGE_Z)),
        mass=0.22,
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_cadquery(_jaw_shape("right"), "right_jaw.obj", assets=ASSETS),
        name="jaw_shell",
        material=jaw_mat,
    )
    right_jaw.inertial = Inertial.from_geometry(
        Box((JAW_CARRIAGE_X + JAW_FINGER_X, JAW_CARRIAGE_Y + 0.010, JAW_CARRIAGE_Z)),
        mass=0.22,
    )

    model.articulation(
        "body_to_mount_plate",
        ArticulationType.FIXED,
        parent=body,
        child=mount_plate,
        origin=Origin(xyz=(0.0, -(BODY_Y + MOUNT_Y) / 2.0, 0.0)),
    )
    model.articulation(
        "body_to_upper_rail",
        ArticulationType.FIXED,
        parent=body,
        child=upper_rail,
        origin=Origin(xyz=(0.0, RAIL_CENTER_Y, RAIL_OFFSET_Z)),
    )
    model.articulation(
        "body_to_lower_rail",
        ArticulationType.FIXED,
        parent=body,
        child=lower_rail,
        origin=Origin(xyz=(0.0, RAIL_CENTER_Y, -RAIL_OFFSET_Z)),
    )
    model.articulation(
        "body_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_jaw,
        origin=Origin(xyz=(LEFT_JAW_OPEN_X, JAW_ORIGIN_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.15,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_jaw,
        origin=Origin(xyz=(RIGHT_JAW_OPEN_X, JAW_ORIGIN_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.15,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    mount_plate = object_model.get_part("mount_plate")
    upper_rail = object_model.get_part("upper_rail")
    lower_rail = object_model.get_part("lower_rail")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")

    body_shell = body.get_visual("body_shell")
    plate_shell = mount_plate.get_visual("plate_shell")
    upper_rail_shell = upper_rail.get_visual("rail_shell")
    lower_rail_shell = lower_rail.get_visual("rail_shell")
    left_jaw_shell = left_jaw.get_visual("jaw_shell")
    right_jaw_shell = right_jaw.get_visual("jaw_shell")

    left_slide = object_model.get_articulation("body_to_left_jaw")
    right_slide = object_model.get_articulation("body_to_right_jaw")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_isolated_part(
        left_jaw,
        reason="Left jaw runs on close-clearance guide rails; mount is proven by rail containment and overlap checks instead of zero-gap broad contact.",
    )
    ctx.allow_isolated_part(
        right_jaw,
        reason="Right jaw runs on close-clearance guide rails; mount is proven by rail containment and overlap checks instead of zero-gap broad contact.",
    )

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "part_presence",
        all(part is not None for part in (body, mount_plate, upper_rail, lower_rail, left_jaw, right_jaw)),
        "Expected body, mount plate, rails, and both jaws to exist.",
    )

    ctx.check(
        "left_slide_axis_and_travel",
        tuple(left_slide.axis) == (1.0, 0.0, 0.0)
        and abs(left_slide.motion_limits.upper - left_slide.motion_limits.lower - JAW_TRAVEL) < 1e-9,
        "Left jaw should translate +X with 40 mm travel.",
    )
    ctx.check(
        "right_slide_axis_and_travel",
        tuple(right_slide.axis) == (-1.0, 0.0, 0.0)
        and abs(right_slide.motion_limits.upper - right_slide.motion_limits.lower - JAW_TRAVEL) < 1e-9,
        "Right jaw should translate -X with 40 mm travel.",
    )

    ctx.expect_gap(
        body,
        mount_plate,
        axis="y",
        max_gap=1e-6,
        max_penetration=1e-6,
        positive_elem=body_shell,
        negative_elem=plate_shell,
        name="mount_plate_seated_to_rear_face",
    )
    ctx.expect_overlap(
        mount_plate,
        body,
        axes="xz",
        min_overlap=0.045,
        elem_a=plate_shell,
        elem_b=body_shell,
        name="mount_plate_covers_rear_face",
    )

    ctx.expect_gap(
        upper_rail,
        body,
        axis="y",
        max_gap=1e-6,
        max_penetration=1e-6,
        positive_elem=upper_rail_shell,
        negative_elem=body_shell,
        name="upper_rail_seated_on_front_face",
    )
    ctx.expect_gap(
        lower_rail,
        body,
        axis="y",
        max_gap=1e-6,
        max_penetration=1e-6,
        positive_elem=lower_rail_shell,
        negative_elem=body_shell,
        name="lower_rail_seated_on_front_face",
    )

    ctx.expect_overlap(
        left_jaw,
        upper_rail,
        axes="x",
        min_overlap=0.030,
        elem_a=left_jaw_shell,
        elem_b=upper_rail_shell,
        name="left_jaw_overlaps_upper_rail_along_slide",
    )
    ctx.expect_overlap(
        left_jaw,
        lower_rail,
        axes="x",
        min_overlap=0.030,
        elem_a=left_jaw_shell,
        elem_b=lower_rail_shell,
        name="left_jaw_overlaps_lower_rail_along_slide",
    )
    ctx.expect_overlap(
        right_jaw,
        upper_rail,
        axes="x",
        min_overlap=0.030,
        elem_a=right_jaw_shell,
        elem_b=upper_rail_shell,
        name="right_jaw_overlaps_upper_rail_along_slide",
    )
    ctx.expect_overlap(
        right_jaw,
        lower_rail,
        axes="x",
        min_overlap=0.030,
        elem_a=right_jaw_shell,
        elem_b=lower_rail_shell,
        name="right_jaw_overlaps_lower_rail_along_slide",
    )

    ctx.expect_within(
        upper_rail,
        left_jaw,
        axes="yz",
        inner_elem=upper_rail_shell,
        outer_elem=left_jaw_shell,
        name="left_jaw_wraps_upper_rail",
    )
    ctx.expect_within(
        lower_rail,
        left_jaw,
        axes="yz",
        inner_elem=lower_rail_shell,
        outer_elem=left_jaw_shell,
        name="left_jaw_wraps_lower_rail",
    )
    ctx.expect_within(
        upper_rail,
        right_jaw,
        axes="yz",
        inner_elem=upper_rail_shell,
        outer_elem=right_jaw_shell,
        name="right_jaw_wraps_upper_rail",
    )
    ctx.expect_within(
        lower_rail,
        right_jaw,
        axes="yz",
        inner_elem=lower_rail_shell,
        outer_elem=right_jaw_shell,
        name="right_jaw_wraps_lower_rail",
    )

    ctx.expect_gap(
        right_jaw,
        left_jaw,
        axis="x",
        min_gap=0.084,
        max_gap=0.088,
        positive_elem=right_jaw_shell,
        negative_elem=left_jaw_shell,
        name="open_jaw_gap",
    )

    left_open_x = ctx.part_world_position(left_jaw)[0]
    right_open_x = ctx.part_world_position(right_jaw)[0]
    with ctx.pose({left_slide: JAW_TRAVEL, right_slide: JAW_TRAVEL}):
        left_closed_x = ctx.part_world_position(left_jaw)[0]
        right_closed_x = ctx.part_world_position(right_jaw)[0]
        ctx.check(
            "jaws_translate_toward_center",
            left_closed_x > left_open_x + 0.039 and right_closed_x < right_open_x - 0.039,
            "Each jaw should move roughly 40 mm toward the centerline.",
        )
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.004,
            max_gap=0.008,
            positive_elem=right_jaw_shell,
            negative_elem=left_jaw_shell,
            name="closed_jaw_gap",
        )
        ctx.expect_overlap(
            left_jaw,
            upper_rail,
            axes="x",
            min_overlap=0.030,
            elem_a=left_jaw_shell,
            elem_b=upper_rail_shell,
            name="left_jaw_stays_on_upper_rail_closed",
        )
        ctx.expect_overlap(
            right_jaw,
            lower_rail,
            axes="x",
            min_overlap=0.030,
            elem_a=right_jaw_shell,
            elem_b=lower_rail_shell,
            name="right_jaw_stays_on_lower_rail_closed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
