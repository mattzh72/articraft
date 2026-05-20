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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_RADIUS = 0.0105
HINGE_RADIUS = 0.0024
HINGE_OFFSET_X = BODY_RADIUS + HINGE_RADIUS
LOWER_LENGTH = 0.118
UPPER_LENGTH = 0.096
SCROLL_Z = 0.058
WHEEL_RADIUS = 0.0072
WHEEL_THICKNESS = 0.0044


def _lower_shell_mesh():
    profile = [
        (0.0, 0.0),
        (BODY_RADIUS * 0.68, 0.004),
        (BODY_RADIUS * 0.96, 0.011),
        (BODY_RADIUS, 0.018),
        (BODY_RADIUS, LOWER_LENGTH),
        (0.0, LOWER_LENGTH),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=72), "pen_phone_lower_shell")


def _upper_shell_mesh():
    profile = [
        (0.0, 0.0),
        (BODY_RADIUS, 0.0),
        (BODY_RADIUS, UPPER_LENGTH - 0.014),
        (BODY_RADIUS * 0.88, UPPER_LENGTH - 0.005),
        (0.0, UPPER_LENGTH),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=72), "pen_phone_upper_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pen_form_flip_phone")

    body_silver = model.material("body_silver", rgba=(0.74, 0.76, 0.80, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    display_black = model.material("display_black", rgba=(0.08, 0.11, 0.14, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.12, 0.16, 0.18, 0.88))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.34, 0.36, 0.39, 1.0))

    lower_body = model.part("lower_body")
    lower_body.visual(_lower_shell_mesh(), material=body_silver, name="lower_shell")
    lower_body.visual(
        Cylinder(radius=BODY_RADIUS + 0.0004, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, LOWER_LENGTH - 0.002)),
        material=graphite,
        name="lower_seam_ring",
    )
    lower_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.020),
        origin=Origin(xyz=(HINGE_OFFSET_X, 0.0, LOWER_LENGTH - 0.010)),
        material=hinge_dark,
        name="lower_hinge_barrel",
    )
    lower_body.visual(
        Box((0.0040, 0.0060, 0.020)),
        origin=Origin(xyz=(BODY_RADIUS + 0.0015, 0.0, LOWER_LENGTH - 0.010)),
        material=hinge_dark,
        name="lower_hinge_leaf",
    )
    lower_body.visual(
        Box((0.012, 0.0022, 0.030)),
        origin=Origin(xyz=(0.0, BODY_RADIUS + 0.0011, SCROLL_Z)),
        material=graphite,
        name="wheel_bezel",
    )
    lower_body.visual(
        Box((0.0018, 0.010, 0.020)),
        origin=Origin(
            xyz=(WHEEL_THICKNESS * 0.5 + 0.0013, BODY_RADIUS + 0.005, SCROLL_Z)
        ),
        material=graphite,
        name="wheel_left_cheek",
    )
    lower_body.visual(
        Box((0.0018, 0.010, 0.020)),
        origin=Origin(
            xyz=(-(WHEEL_THICKNESS * 0.5 + 0.0013), BODY_RADIUS + 0.005, SCROLL_Z)
        ),
        material=graphite,
        name="wheel_right_cheek",
    )
    lower_body.visual(
        Box((0.0055, 0.0010, 0.014)),
        origin=Origin(xyz=(0.0, BODY_RADIUS + 0.0005, 0.020)),
        material=graphite,
        name="microphone_slit",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((0.028, 0.028, LOWER_LENGTH)),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, LOWER_LENGTH * 0.5)),
    )

    upper_body = model.part("upper_display_body")
    upper_body.visual(
        _upper_shell_mesh(),
        origin=Origin(xyz=(-HINGE_OFFSET_X, 0.0, 0.0)),
        material=body_silver,
        name="upper_shell",
    )
    upper_body.visual(
        Cylinder(radius=BODY_RADIUS + 0.0004, length=0.004),
        origin=Origin(xyz=(-HINGE_OFFSET_X, 0.0, 0.002)),
        material=graphite,
        name="upper_seam_ring",
    )
    upper_body.visual(
        Box((0.0125, 0.0016, 0.056)),
        origin=Origin(
            xyz=(-HINGE_OFFSET_X, BODY_RADIUS + 0.0008, UPPER_LENGTH * 0.50)
        ),
        material=smoked_glass,
        name="display_window",
    )
    upper_body.visual(
        Box((0.0060, 0.0012, 0.013)),
        origin=Origin(
            xyz=(-HINGE_OFFSET_X, BODY_RADIUS + 0.0007, UPPER_LENGTH * 0.79)
        ),
        material=display_black,
        name="earpiece_slit",
    )
    upper_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=hinge_dark,
        name="upper_hinge_barrel",
    )
    upper_body.visual(
        Box((0.0040, 0.0060, 0.020)),
        origin=Origin(xyz=(-0.0036, 0.0, 0.010)),
        material=hinge_dark,
        name="upper_hinge_leaf",
    )
    upper_body.inertial = Inertial.from_geometry(
        Box((0.032, 0.024, UPPER_LENGTH)),
        mass=0.09,
        origin=Origin(xyz=(-HINGE_OFFSET_X * 0.72, 0.0, UPPER_LENGTH * 0.5)),
    )

    scroll_wheel = model.part("scroll_wheel")
    scroll_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_THICKNESS),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_rubber,
        name="wheel_tire",
    )
    scroll_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.70, length=WHEEL_THICKNESS + 0.0006),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_core,
        name="wheel_hub",
    )
    scroll_wheel.visual(
        Box((WHEEL_THICKNESS * 0.90, 0.0012, 0.0032)),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_RADIUS + 0.0008)),
        material=wheel_core,
        name="thumb_tab",
    )
    scroll_wheel.inertial = Inertial.from_geometry(
        Box((0.006, 0.016, 0.018)),
        mass=0.012,
    )

    model.articulation(
        "lower_to_upper_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(xyz=(HINGE_OFFSET_X, 0.0, LOWER_LENGTH)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.6,
            velocity=2.4,
            lower=0.0,
            upper=3.0,
        ),
    )
    model.articulation(
        "lower_to_scroll_wheel",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=scroll_wheel,
        origin=Origin(xyz=(0.0, BODY_RADIUS + WHEEL_RADIUS, SCROLL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=8.0,
            lower=-1.5,
            upper=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    upper_body = object_model.get_part("upper_display_body")
    scroll_wheel = object_model.get_part("scroll_wheel")
    cap_hinge = object_model.get_articulation("lower_to_upper_hinge")
    wheel_joint = object_model.get_articulation("lower_to_scroll_wheel")

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
        upper_body,
        lower_body,
        name="upper_body_is_physically_seated_on_lower_body",
    )
    ctx.expect_contact(
        scroll_wheel,
        lower_body,
        name="scroll_wheel_is_physically_mounted_to_lower_body",
    )
    ctx.expect_overlap(
        upper_body,
        lower_body,
        axes="xy",
        min_overlap=0.018,
        name="closed_phone_halves_share_the_same_barrel_footprint",
    )
    ctx.expect_gap(
        upper_body,
        lower_body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper_body_sits_flush_on_the_lower_seam",
    )
    ctx.check(
        "cap_hinge_axis_runs_along_the_pen_body",
        cap_hinge.axis == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {cap_hinge.axis}",
    )
    ctx.check(
        "scroll_wheel_rotates_on_a_crosswise_axle",
        wheel_joint.axis == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {wheel_joint.axis}",
    )

    upper_closed_aabb = ctx.part_world_aabb(upper_body)
    assert upper_closed_aabb is not None
    upper_closed_center_x = (upper_closed_aabb[0][0] + upper_closed_aabb[1][0]) * 0.5
    with ctx.pose({cap_hinge: 2.85}):
        upper_open_aabb = ctx.part_world_aabb(upper_body)
        assert upper_open_aabb is not None
        upper_open_center_x = (upper_open_aabb[0][0] + upper_open_aabb[1][0]) * 0.5
        ctx.check(
            "cap_swings_sideways_when_opened",
            upper_open_center_x > upper_closed_center_x + 0.020,
            details=(
                f"expected opened cap center x to move sideways by > 0.020 m; "
                f"closed={upper_closed_center_x:.4f}, open={upper_open_center_x:.4f}"
            ),
        )
        ctx.expect_contact(
            upper_body,
            lower_body,
            name="hinge_keeps_upper_body_connected_when_open",
        )

    tab_rest_aabb = ctx.part_element_world_aabb(scroll_wheel, elem="thumb_tab")
    assert tab_rest_aabb is not None
    tab_rest_center = (
        (tab_rest_aabb[0][0] + tab_rest_aabb[1][0]) * 0.5,
        (tab_rest_aabb[0][1] + tab_rest_aabb[1][1]) * 0.5,
        (tab_rest_aabb[0][2] + tab_rest_aabb[1][2]) * 0.5,
    )
    with ctx.pose({wheel_joint: 1.1}):
        tab_turn_aabb = ctx.part_element_world_aabb(scroll_wheel, elem="thumb_tab")
        assert tab_turn_aabb is not None
        tab_turn_center = (
            (tab_turn_aabb[0][0] + tab_turn_aabb[1][0]) * 0.5,
            (tab_turn_aabb[0][1] + tab_turn_aabb[1][1]) * 0.5,
            (tab_turn_aabb[0][2] + tab_turn_aabb[1][2]) * 0.5,
        )
        yz_shift = math.hypot(
            tab_turn_center[1] - tab_rest_center[1],
            tab_turn_center[2] - tab_rest_center[2],
        )
        ctx.check(
            "scroll_wheel_pose_moves_the_thumb_tab",
            yz_shift > 0.004,
            details=f"expected thumb tab yz shift > 0.004 m, got {yz_shift:.4f} m",
        )
        ctx.expect_contact(
            scroll_wheel,
            lower_body,
            name="scroll_wheel_remains_in_contact_through_rotation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
