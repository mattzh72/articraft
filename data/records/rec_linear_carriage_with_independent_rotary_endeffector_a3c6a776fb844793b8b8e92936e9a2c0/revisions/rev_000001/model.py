from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SUPPORT_LENGTH = 0.42
SUPPORT_TOP_WIDTH = 0.11
SUPPORT_TOP_THICKNESS = 0.018
GUIDE_BAR_LENGTH = 0.404
GUIDE_BAR_WIDTH = 0.052
GUIDE_BAR_HEIGHT = 0.020
GUIDE_BAR_BOTTOM_Z = -GUIDE_BAR_HEIGHT

CARRIAGE_LENGTH = 0.14
CARRIAGE_CHEEK_OUTER_Y = 0.035
CARRIAGE_CHEEK_WIDTH = 0.018
CARRIAGE_CHEEK_HEIGHT = 0.050
CARRIAGE_BRIDGE_LENGTH = 0.096
CARRIAGE_BRIDGE_WIDTH = 0.068
CARRIAGE_BRIDGE_HEIGHT = 0.030
CARRIAGE_BRIDGE_BOTTOM_Z = -0.068
CARRIAGE_NOSE_LENGTH = 0.062
CARRIAGE_NOSE_WIDTH = 0.020
CARRIAGE_NOSE_HEIGHT = 0.052
CARRIAGE_NOSE_BOTTOM_Z = -0.104
NOSE_CENTER_Y = 0.029
BEARING_AXIS_Y = 0.048
BEARING_AXIS_Z = -0.074
SPINDLE_COLLAR_RADIUS = 0.015
SPINDLE_COLLAR_LENGTH = 0.006
SPINDLE_HUB_RADIUS = 0.012
SPINDLE_HUB_LENGTH = 0.024
SPINDLE_FACEPLATE_RADIUS = 0.030
SPINDLE_FACEPLATE_LENGTH = 0.006
SPINDLE_BOSS_RADIUS = 0.008
SPINDLE_BOSS_LENGTH = 0.004


def _y_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(-math.pi * 0.5, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_slide_rotary_module")

    model.material("support_dark", color=(0.22, 0.24, 0.27))
    model.material("carriage_gray", color=(0.56, 0.58, 0.60))
    model.material("spindle_metal", color=(0.77, 0.79, 0.82))
    model.material("index_black", color=(0.14, 0.14, 0.15))

    support = model.part("support")
    support.visual(
        Box((SUPPORT_LENGTH, SUPPORT_TOP_WIDTH, SUPPORT_TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_TOP_THICKNESS * 0.5)),
        name="top_plate",
        material="support_dark",
    )
    support.visual(
        Box((GUIDE_BAR_LENGTH, GUIDE_BAR_WIDTH, GUIDE_BAR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_BAR_BOTTOM_Z + GUIDE_BAR_HEIGHT * 0.5)),
        name="guide_bar",
        material="support_dark",
    )
    support.visual(
        Box((0.020, GUIDE_BAR_WIDTH, GUIDE_BAR_HEIGHT)),
        origin=Origin(
            xyz=(
                -(SUPPORT_LENGTH * 0.5 - 0.010),
                0.0,
                GUIDE_BAR_BOTTOM_Z + GUIDE_BAR_HEIGHT * 0.5,
            )
        ),
        name="left_stop",
        material="support_dark",
    )
    support.visual(
        Box((0.020, GUIDE_BAR_WIDTH, GUIDE_BAR_HEIGHT)),
        origin=Origin(
            xyz=(
                SUPPORT_LENGTH * 0.5 - 0.010,
                0.0,
                GUIDE_BAR_BOTTOM_Z + GUIDE_BAR_HEIGHT * 0.5,
            )
        ),
        name="right_stop",
        material="support_dark",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_CHEEK_WIDTH, CARRIAGE_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, CARRIAGE_CHEEK_OUTER_Y, -CARRIAGE_CHEEK_HEIGHT * 0.5)),
        name="left_cheek",
        material="carriage_gray",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_CHEEK_WIDTH, CARRIAGE_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, -CARRIAGE_CHEEK_OUTER_Y, -CARRIAGE_CHEEK_HEIGHT * 0.5)),
        name="right_cheek",
        material="carriage_gray",
    )
    carriage.visual(
        Box((CARRIAGE_BRIDGE_LENGTH, CARRIAGE_BRIDGE_WIDTH, CARRIAGE_BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_BRIDGE_BOTTOM_Z + CARRIAGE_BRIDGE_HEIGHT * 0.5)
        ),
        name="bridge",
        material="carriage_gray",
    )
    carriage.visual(
        Box((CARRIAGE_NOSE_LENGTH, CARRIAGE_NOSE_WIDTH, CARRIAGE_NOSE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                NOSE_CENTER_Y,
                CARRIAGE_NOSE_BOTTOM_Z + CARRIAGE_NOSE_HEIGHT * 0.5,
            )
        ),
        name="bearing_nose",
        material="carriage_gray",
    )
    carriage.visual(
        Box((0.074, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, 0.042, BEARING_AXIS_Z)),
        name="thrust_plate",
        material="carriage_gray",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SPINDLE_COLLAR_RADIUS, length=SPINDLE_COLLAR_LENGTH),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        name="rear_collar",
        material="spindle_metal",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_HUB_RADIUS, length=SPINDLE_HUB_LENGTH),
        origin=_y_cylinder_origin(
            (0.0, SPINDLE_COLLAR_LENGTH * 0.5 + SPINDLE_HUB_LENGTH * 0.5 - 0.001, 0.0)
        ),
        name="hub",
        material="spindle_metal",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_FACEPLATE_RADIUS, length=SPINDLE_FACEPLATE_LENGTH),
        origin=_y_cylinder_origin(
            (
                0.0,
                SPINDLE_COLLAR_LENGTH + SPINDLE_HUB_LENGTH - SPINDLE_FACEPLATE_LENGTH * 0.5,
                0.0,
            )
        ),
        name="faceplate",
        material="spindle_metal",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_BOSS_RADIUS, length=SPINDLE_BOSS_LENGTH),
        origin=_y_cylinder_origin(
            (
                0.0,
                SPINDLE_COLLAR_LENGTH
                + SPINDLE_HUB_LENGTH
                - SPINDLE_FACEPLATE_LENGTH
                + SPINDLE_BOSS_LENGTH * 0.5,
                0.0,
            )
        ),
        name="center_boss",
        material="spindle_metal",
    )
    spindle.visual(
        Cylinder(radius=0.0035, length=0.006),
        origin=_y_cylinder_origin(
            (
                0.018,
                SPINDLE_COLLAR_LENGTH + SPINDLE_HUB_LENGTH - SPINDLE_FACEPLATE_LENGTH * 0.5,
                0.0,
            )
        ),
        name="index_pin",
        material="index_black",
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=support,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_BAR_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.45,
            lower=-0.10,
            upper=0.10,
        ),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, BEARING_AXIS_Y, BEARING_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=6.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("support_to_carriage")
    rotary = object_model.get_articulation("carriage_to_spindle")

    def _center_of_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((a + b) * 0.5 for a, b in zip(lo, hi))

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

    ctx.expect_contact(
        support,
        carriage,
        contact_tol=0.0015,
        name="support_carries_hanging_carriage",
    )
    ctx.expect_contact(
        carriage,
        spindle,
        contact_tol=0.0015,
        name="carriage_bearing_support_contacts_spindle",
    )
    ctx.expect_overlap(
        support,
        carriage,
        axes="xy",
        min_overlap=0.050,
        name="carriage_stays_under_support_in_projection",
    )

    with ctx.pose({slide: 0.0, rotary: 0.0}):
        carriage_rest = ctx.part_world_position(carriage)
        spindle_rest = ctx.part_world_position(spindle)
        pin_rest = _center_of_aabb(ctx.part_element_world_aabb(spindle, elem="index_pin"))

    with ctx.pose({slide: slide.motion_limits.upper, rotary: 0.0}):
        carriage_slid = ctx.part_world_position(carriage)
        spindle_slid = ctx.part_world_position(spindle)

    slide_ok = (
        carriage_rest is not None
        and carriage_slid is not None
        and spindle_rest is not None
        and spindle_slid is not None
        and carriage_slid[0] - carriage_rest[0] > 0.08
        and abs(carriage_slid[1] - carriage_rest[1]) < 1e-6
        and abs(carriage_slid[2] - carriage_rest[2]) < 1e-6
        and abs((spindle_slid[0] - spindle_rest[0]) - (carriage_slid[0] - carriage_rest[0])) < 1e-6
    )
    ctx.check(
        "carriage_moves_prismatically_along_support",
        slide_ok,
        details=(
            f"carriage_rest={carriage_rest}, carriage_slid={carriage_slid}, "
            f"spindle_rest={spindle_rest}, spindle_slid={spindle_slid}"
        ),
    )

    with ctx.pose({slide: 0.0, rotary: 1.2}):
        spindle_rot = ctx.part_world_position(spindle)
        pin_rot = _center_of_aabb(ctx.part_element_world_aabb(spindle, elem="index_pin"))

    spin_ok = False
    if spindle_rest is not None and spindle_rot is not None and pin_rest is not None and pin_rot is not None:
        rest_radius = math.hypot(pin_rest[0] - spindle_rest[0], pin_rest[2] - spindle_rest[2])
        rot_radius = math.hypot(pin_rot[0] - spindle_rot[0], pin_rot[2] - spindle_rot[2])
        spin_ok = (
            abs(spindle_rot[0] - spindle_rest[0]) < 1e-6
            and abs(spindle_rot[1] - spindle_rest[1]) < 1e-6
            and abs(spindle_rot[2] - spindle_rest[2]) < 1e-6
            and abs(rest_radius - rot_radius) < 0.0015
            and abs(pin_rot[0] - pin_rest[0]) > 0.004
            and abs(pin_rot[2] - pin_rest[2]) > 0.004
        )
    ctx.check(
        "spindle_rotates_independently_about_local_bearing_axis",
        spin_ok,
        details=(
            f"spindle_rest={spindle_rest}, spindle_rot={spindle_rot}, "
            f"pin_rest={pin_rest}, pin_rot={pin_rot}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
