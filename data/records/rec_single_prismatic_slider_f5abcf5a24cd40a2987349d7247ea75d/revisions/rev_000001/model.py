from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


FRAME_WIDTH = 0.180
FRAME_HEIGHT = 0.160
BRIDGE_THICKNESS = 0.018
UPRIGHT_WIDTH = 0.028
TOP_BRIDGE_HEIGHT = 0.022
BOTTOM_BRIDGE_HEIGHT = 0.026

SUPPORT_LENGTH = 0.026
SUPPORT_WIDTH = 0.038
SUPPORT_HEIGHT = 0.034

FOOT_LENGTH = 0.040
FOOT_WIDTH = 0.050
FOOT_HEIGHT = 0.012

ROD_RADIUS = 0.008
ROD_LENGTH = 0.300
ROD_CENTER_X = 0.160
ROD_Y_OFFSET = 0.048

REAR_STOP_LENGTH = 0.014
REAR_STOP_WIDTH = 0.034
REAR_STOP_HEIGHT = 0.034
REAR_STOP_X = 0.036

FRONT_STOP_LENGTH = 0.014
FRONT_STOP_RADIUS = 0.014
FRONT_STOP_X = 0.311

CARRIAGE_BODY_LENGTH = 0.060
CARRIAGE_BODY_WIDTH = 0.084
CARRIAGE_BODY_HEIGHT = 0.072

MOUNT_FACE_LENGTH = 0.012
MOUNT_FACE_WIDTH = 0.076
MOUNT_FACE_HEIGHT = 0.120
MOUNT_FACE_CENTER_X = 0.054

SHOE_LENGTH = 0.056
SHOE_WIDTH = 0.022
SHOE_HEIGHT = 0.016
SHOE_UPPER_Z = ROD_RADIUS + SHOE_HEIGHT / 2.0
SHOE_LOWER_Z = -SHOE_UPPER_Z

OUTER_WALL_WIDTH = 0.012
OUTER_WALL_HEIGHT = 0.048
OUTER_WALL_Y = ROD_Y_OFFSET + SHOE_WIDTH / 2.0 + OUTER_WALL_WIDTH / 2.0

SPINE_LENGTH = 0.028
SPINE_WIDTH = 0.040
SPINE_HEIGHT = 0.082
SPINE_CENTER_X = 0.028

MOUNT_STEM_LENGTH = 0.012
MOUNT_STEM_WIDTH = 0.028
MOUNT_STEM_HEIGHT = 0.074
MOUNT_STEM_CENTER_X = 0.045

CENTER_BLOCK_LENGTH = 0.050
CENTER_BLOCK_WIDTH = 0.072
CENTER_BLOCK_HEIGHT = 0.052
CENTER_BLOCK_X = 0.002

CARRIAGE_REAR_X = SHOE_LENGTH / 2.0
CARRIAGE_FRONT_X = MOUNT_FACE_CENTER_X + MOUNT_FACE_LENGTH / 2.0

SLIDE_HOME_X = REAR_STOP_X + (REAR_STOP_LENGTH / 2.0) + CARRIAGE_REAR_X
SLIDE_TRAVEL = (
    (FRONT_STOP_X - (FRONT_STOP_LENGTH / 2.0))
    - SLIDE_HOME_X
    - CARRIAGE_REAR_X
)


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_x_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_slide_unit")

    model.material("frame_charcoal", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("rod_steel", rgba=(0.71, 0.74, 0.78, 1.0))
    model.material("carriage_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("mount_face_silver", rgba=(0.86, 0.88, 0.90, 1.0))
    model.material("stop_black", rgba=(0.09, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    _add_box(
        frame,
        size=(BRIDGE_THICKNESS, UPRIGHT_WIDTH, FRAME_HEIGHT),
        xyz=(BRIDGE_THICKNESS / 2.0, -(FRAME_WIDTH / 2.0 - UPRIGHT_WIDTH / 2.0), 0.0),
        material="frame_charcoal",
        name="left_upright",
    )
    _add_box(
        frame,
        size=(BRIDGE_THICKNESS, UPRIGHT_WIDTH, FRAME_HEIGHT),
        xyz=(BRIDGE_THICKNESS / 2.0, FRAME_WIDTH / 2.0 - UPRIGHT_WIDTH / 2.0, 0.0),
        material="frame_charcoal",
        name="right_upright",
    )
    _add_box(
        frame,
        size=(BRIDGE_THICKNESS, FRAME_WIDTH - UPRIGHT_WIDTH * 2.0, TOP_BRIDGE_HEIGHT),
        xyz=(BRIDGE_THICKNESS / 2.0, 0.0, FRAME_HEIGHT / 2.0 - TOP_BRIDGE_HEIGHT / 2.0),
        material="frame_charcoal",
        name="top_bridge",
    )
    _add_box(
        frame,
        size=(BRIDGE_THICKNESS + 0.004, FRAME_WIDTH - UPRIGHT_WIDTH * 2.0, BOTTOM_BRIDGE_HEIGHT),
        xyz=(
            (BRIDGE_THICKNESS + 0.004) / 2.0,
            0.0,
            -(FRAME_HEIGHT / 2.0 - BOTTOM_BRIDGE_HEIGHT / 2.0),
        ),
        material="frame_charcoal",
        name="bottom_bridge",
    )
    _add_box(
        frame,
        size=(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT),
        xyz=(FOOT_LENGTH / 2.0, -0.060, -(FRAME_HEIGHT / 2.0 + FOOT_HEIGHT / 2.0 - 0.008)),
        material="frame_charcoal",
        name="left_foot",
    )
    _add_box(
        frame,
        size=(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT),
        xyz=(FOOT_LENGTH / 2.0, 0.060, -(FRAME_HEIGHT / 2.0 + FOOT_HEIGHT / 2.0 - 0.008)),
        material="frame_charcoal",
        name="right_foot",
    )
    _add_box(
        frame,
        size=(SUPPORT_LENGTH, SUPPORT_WIDTH, SUPPORT_HEIGHT),
        xyz=(SUPPORT_LENGTH / 2.0, -ROD_Y_OFFSET, 0.0),
        material="frame_charcoal",
        name="left_support",
    )
    _add_box(
        frame,
        size=(SUPPORT_LENGTH, SUPPORT_WIDTH, SUPPORT_HEIGHT),
        xyz=(SUPPORT_LENGTH / 2.0, ROD_Y_OFFSET, 0.0),
        material="frame_charcoal",
        name="right_support",
    )
    _add_x_cylinder(
        frame,
        radius=ROD_RADIUS,
        length=ROD_LENGTH,
        xyz=(ROD_CENTER_X, -ROD_Y_OFFSET, 0.0),
        material="rod_steel",
        name="left_rod",
    )
    _add_x_cylinder(
        frame,
        radius=ROD_RADIUS,
        length=ROD_LENGTH,
        xyz=(ROD_CENTER_X, ROD_Y_OFFSET, 0.0),
        material="rod_steel",
        name="right_rod",
    )
    _add_box(
        frame,
        size=(REAR_STOP_LENGTH, REAR_STOP_WIDTH, REAR_STOP_HEIGHT),
        xyz=(REAR_STOP_X, -ROD_Y_OFFSET, 0.0),
        material="stop_black",
        name="rear_stop_left",
    )
    _add_box(
        frame,
        size=(REAR_STOP_LENGTH, REAR_STOP_WIDTH, REAR_STOP_HEIGHT),
        xyz=(REAR_STOP_X, ROD_Y_OFFSET, 0.0),
        material="stop_black",
        name="rear_stop_right",
    )
    _add_x_cylinder(
        frame,
        radius=FRONT_STOP_RADIUS,
        length=FRONT_STOP_LENGTH,
        xyz=(FRONT_STOP_X, -ROD_Y_OFFSET, 0.0),
        material="stop_black",
        name="front_stop_left",
    )
    _add_x_cylinder(
        frame,
        radius=FRONT_STOP_RADIUS,
        length=FRONT_STOP_LENGTH,
        xyz=(FRONT_STOP_X, ROD_Y_OFFSET, 0.0),
        material="stop_black",
        name="front_stop_right",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.320, FRAME_WIDTH, FRAME_HEIGHT + FOOT_HEIGHT)),
        mass=3.6,
        origin=Origin(xyz=(0.160, 0.0, -0.006)),
    )

    carriage = model.part("carriage")
    _add_box(
        carriage,
        size=(SHOE_LENGTH, SHOE_WIDTH, SHOE_HEIGHT),
        xyz=(0.0, -ROD_Y_OFFSET, SHOE_UPPER_Z),
        material="carriage_gray",
        name="left_upper_shoe",
    )
    _add_box(
        carriage,
        size=(SHOE_LENGTH, SHOE_WIDTH, SHOE_HEIGHT),
        xyz=(0.0, -ROD_Y_OFFSET, SHOE_LOWER_Z),
        material="carriage_gray",
        name="left_lower_shoe",
    )
    _add_box(
        carriage,
        size=(SHOE_LENGTH, OUTER_WALL_WIDTH, OUTER_WALL_HEIGHT),
        xyz=(0.0, -OUTER_WALL_Y, 0.0),
        material="carriage_gray",
        name="left_outer_wall",
    )
    _add_box(
        carriage,
        size=(SHOE_LENGTH, SHOE_WIDTH, SHOE_HEIGHT),
        xyz=(0.0, ROD_Y_OFFSET, SHOE_UPPER_Z),
        material="carriage_gray",
        name="right_upper_shoe",
    )
    _add_box(
        carriage,
        size=(SHOE_LENGTH, SHOE_WIDTH, SHOE_HEIGHT),
        xyz=(0.0, ROD_Y_OFFSET, SHOE_LOWER_Z),
        material="carriage_gray",
        name="right_lower_shoe",
    )
    _add_box(
        carriage,
        size=(SHOE_LENGTH, OUTER_WALL_WIDTH, OUTER_WALL_HEIGHT),
        xyz=(0.0, OUTER_WALL_Y, 0.0),
        material="carriage_gray",
        name="right_outer_wall",
    )
    _add_box(
        carriage,
        size=(CARRIAGE_BODY_LENGTH, CARRIAGE_BODY_WIDTH, CARRIAGE_BODY_HEIGHT),
        xyz=(CENTER_BLOCK_X, 0.0, 0.0),
        material="carriage_gray",
        name="carriage_body",
    )
    _add_box(
        carriage,
        size=(SPINE_LENGTH, SPINE_WIDTH, SPINE_HEIGHT),
        xyz=(SPINE_CENTER_X, 0.0, 0.0),
        material="carriage_gray",
        name="front_spine",
    )
    _add_box(
        carriage,
        size=(MOUNT_STEM_LENGTH, MOUNT_STEM_WIDTH, MOUNT_STEM_HEIGHT),
        xyz=(MOUNT_STEM_CENTER_X, 0.0, 0.0),
        material="carriage_gray",
        name="mount_stem",
    )
    _add_box(
        carriage,
        size=(MOUNT_FACE_LENGTH, MOUNT_FACE_WIDTH, MOUNT_FACE_HEIGHT),
        xyz=(MOUNT_FACE_CENTER_X, 0.0, 0.0),
        material="mount_face_silver",
        name="mounting_face",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, MOUNT_FACE_HEIGHT)),
        mass=1.15,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=450.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("frame_to_carriage")

    frame.get_visual("rear_stop_left")
    frame.get_visual("front_stop_left")
    carriage.get_visual("carriage_body")
    carriage.get_visual("mounting_face")

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

    limits = slide.motion_limits
    ctx.check(
        "slide_joint_is_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"expected PRISMATIC, got {slide.articulation_type}",
    )
    ctx.check(
        "slide_axis_points_along_guide_path",
        tuple(round(v, 6) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected axis (1, 0, 0), got {slide.axis}",
    )
    ctx.check(
        "slide_limits_match_visible_stops",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and abs(limits.upper - SLIDE_TRAVEL) < 1e-9,
        details=f"unexpected limits: {limits}",
    )
    ctx.expect_contact(
        carriage,
        frame,
        name="carriage_is_physically_supported_by_frame",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="yz",
        min_overlap=0.10,
        name="carriage_tracks_in_frame_envelope",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="x",
        positive_elem="mounting_face",
        negative_elem="top_bridge",
        min_gap=0.08,
        name="plain_mounting_face_runs_in_front_of_back_bridge",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="x",
        positive_elem="left_upper_shoe",
        negative_elem="rear_stop_left",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear_stop_catches_carriage_home_position",
    )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_gap(
            frame,
            carriage,
            axis="x",
            positive_elem="front_stop_left",
            negative_elem="left_upper_shoe",
            max_gap=0.001,
            max_penetration=0.0,
            name="front_stop_limits_full_extension",
        )
        ctx.expect_contact(
            carriage,
            frame,
            name="carriage_remains_supported_at_full_extension",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
