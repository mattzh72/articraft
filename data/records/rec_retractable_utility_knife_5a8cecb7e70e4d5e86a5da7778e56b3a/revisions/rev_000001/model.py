from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


HANDLE_LENGTH = 0.170
HANDLE_WIDTH = 0.034
HANDLE_HEIGHT = 0.022
CAVITY_WIDTH = 0.026
CAVITY_HEIGHT = 0.014


def _rounded_bar_mesh(
    *,
    length: float,
    width: float,
    height: float,
    radius: float,
    name: str,
):
    profile = rounded_rect_profile(
        height,
        width,
        min(radius, height * 0.49, width * 0.49),
    )
    geometry = ExtrudeGeometry(profile, length, cap=True, center=True)
    geometry.rotate_y(math.pi / 2.0).translate(0.0, 0.0, height * 0.5)
    return mesh_from_geometry(geometry, name)


def _blade_mesh():
    blade_profile = [
        (0.000, 0.000),
        (0.036, 0.000),
        (0.048, 0.0024),
        (0.043, 0.0058),
        (0.000, 0.0058),
    ]
    geometry = ExtrudeGeometry.from_z0(blade_profile, 0.0008)
    geometry.rotate_x(math.pi / 2.0).translate(0.0, 0.0004, 0.0)
    return mesh_from_geometry(geometry, "knife_blade")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="auto_retract_safety_knife")

    shell_orange = model.material("shell_orange", rgba=(0.95, 0.55, 0.10, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.82, 1.0))

    handle_shell = model.part("handle_shell")
    handle_shell.visual(
        Box((HANDLE_LENGTH, HANDLE_WIDTH, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=shell_orange,
        name="bottom_shell",
    )
    handle_shell.visual(
        Box((HANDLE_LENGTH, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.015, 0.011)),
        material=shell_orange,
        name="left_shell_wall",
    )
    handle_shell.visual(
        Box((HANDLE_LENGTH, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.015, 0.011)),
        material=shell_orange,
        name="right_shell_wall",
    )
    handle_shell.visual(
        Box((0.014, CAVITY_WIDTH, 0.014)),
        origin=Origin(xyz=(-0.078, 0.0, 0.011)),
        material=charcoal,
        name="rear_cap",
    )
    handle_shell.visual(
        Box((0.110, 0.022, 0.0015)),
        origin=Origin(xyz=(0.006, 0.0, 0.01075)),
        material=charcoal,
        name="blade_channel_floor",
    )
    handle_shell.visual(
        Box((0.104, 0.010, 0.003)),
        origin=Origin(xyz=(-0.008, -0.008, 0.0205)),
        material=shell_orange,
        name="top_left_rail",
    )
    handle_shell.visual(
        Box((0.104, 0.010, 0.003)),
        origin=Origin(xyz=(-0.008, 0.008, 0.0205)),
        material=shell_orange,
        name="top_right_rail",
    )
    handle_shell.visual(
        Box((0.034, CAVITY_WIDTH, 0.0025)),
        origin=Origin(xyz=(0.068, 0.0, 0.00925)),
        material=charcoal,
        name="front_guide_shelf",
    )
    handle_shell.visual(
        Box((0.006, HANDLE_WIDTH, 0.003)),
        origin=Origin(xyz=(0.082, 0.0, 0.020)),
        material=shell_orange,
        name="front_bridge",
    )
    handle_shell.visual(
        Box((0.006, 0.006, 0.015)),
        origin=Origin(xyz=(0.082, -0.014, 0.0115)),
        material=charcoal,
        name="front_left_cheek",
    )
    handle_shell.visual(
        Box((0.006, 0.006, 0.015)),
        origin=Origin(xyz=(0.082, 0.014, 0.0115)),
        material=charcoal,
        name="front_right_cheek",
    )
    handle_shell.visual(
        Box((0.078, 0.0016, 0.012)),
        origin=Origin(xyz=(0.000, -0.0178, 0.011)),
        material=dark_trim,
        name="left_grip",
    )
    handle_shell.visual(
        Box((0.078, 0.0016, 0.012)),
        origin=Origin(xyz=(0.000, 0.0178, 0.011)),
        material=dark_trim,
        name="right_grip",
    )
    handle_shell.inertial = Inertial.from_geometry(
        Box((HANDLE_LENGTH, HANDLE_WIDTH, HANDLE_HEIGHT)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_HEIGHT * 0.5)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.040, 0.022, 0.0050)),
        origin=Origin(xyz=(0.020, 0.0, 0.0140)),
        material=charcoal,
        name="carrier_sled",
    )
    blade_carrier.visual(
        Box((0.006, 0.004, 0.0055)),
        origin=Origin(xyz=(0.018, 0.0, 0.0190)),
        material=charcoal,
        name="slider_stem",
    )
    blade_carrier.visual(
        Box((0.018, 0.010, 0.0028)),
        origin=Origin(xyz=(0.018, 0.0, 0.0231)),
        material=dark_trim,
        name="thumb_slider",
    )
    blade_carrier.visual(
        _blade_mesh(),
        origin=Origin(xyz=(0.018, 0.0, 0.0118)),
        material=steel,
        name="blade_edge",
    )
    blade_carrier.visual(
        Box((0.012, 0.0008, 0.0046)),
        origin=Origin(xyz=(0.065, 0.0, 0.0141)),
        material=steel,
        name="blade_tip",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.072, 0.022, 0.012)),
        mass=0.05,
        origin=Origin(xyz=(0.032, 0.0, 0.014)),
    )

    nose_guard = model.part("nose_guard")
    nose_guard.visual(
        Box((0.022, 0.018, 0.0045)),
        origin=Origin(xyz=(0.011, 0.0, 0.00525)),
        material=charcoal,
        name="guide_tongue",
    )
    nose_guard.visual(
        Box((0.012, CAVITY_WIDTH, 0.0045)),
        origin=Origin(xyz=(0.010, 0.0, 0.00525)),
        material=charcoal,
        name="capture_clip",
    )
    nose_guard.visual(
        _rounded_bar_mesh(
            length=0.012,
            width=0.020,
            height=0.0045,
            radius=0.0018,
            name="nose_guard_bumper",
        ),
        origin=Origin(xyz=(0.028, 0.0, 0.003)),
        material=dark_trim,
        name="nose_bumper",
    )
    nose_guard.inertial = Inertial.from_geometry(
        Box((0.036, 0.026, 0.006)),
        mass=0.02,
        origin=Origin(xyz=(0.018, 0.0, 0.006)),
    )

    model.articulation(
        "handle_to_blade_carrier",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=blade_carrier,
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=0.0,
            upper=0.050,
        ),
    )
    model.articulation(
        "handle_to_nose_guard",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=nose_guard,
        origin=Origin(xyz=(0.053, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.014,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle_shell = object_model.get_part("handle_shell")
    blade_carrier = object_model.get_part("blade_carrier")
    nose_guard = object_model.get_part("nose_guard")
    blade_slide = object_model.get_articulation("handle_to_blade_carrier")
    guard_slide = object_model.get_articulation("handle_to_nose_guard")

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
        "blade_carrier_axis_aligned_with_handle",
        tuple(blade_slide.axis) == (1.0, 0.0, 0.0),
        f"axis={blade_slide.axis}",
    )
    ctx.check(
        "nose_guard_axis_aligned_with_tip_guide",
        tuple(guard_slide.axis) == (-1.0, 0.0, 0.0),
        f"axis={guard_slide.axis}",
    )

    ctx.expect_contact(
        blade_carrier,
        handle_shell,
        name="blade_carrier_captured_in_handle",
    )
    ctx.expect_contact(
        nose_guard,
        handle_shell,
        name="nose_guard_captured_in_front_guide",
    )
    ctx.expect_within(
        blade_carrier,
        handle_shell,
        axes="yz",
        inner_elem="carrier_sled",
        margin=0.0005,
        name="blade_carrier_stays_in_main_channel",
    )
    ctx.expect_within(
        nose_guard,
        handle_shell,
        axes="yz",
        inner_elem="guide_tongue",
        margin=0.0005,
        name="nose_guard_stays_in_tip_guide",
    )
    ctx.expect_gap(
        blade_carrier,
        nose_guard,
        axis="z",
        positive_elem="carrier_sled",
        negative_elem="guide_tongue",
        min_gap=0.0025,
        max_gap=0.010,
        name="blade_carrier_and_nose_guard_use_separate_channels",
    )
    ctx.expect_gap(
        handle_shell,
        nose_guard,
        axis="x",
        positive_elem="front_left_cheek",
        negative_elem="capture_clip",
        min_gap=0.008,
        name="nose_guard_clip_stays_behind_front_guide_lip",
    )

    carrier_rest = ctx.part_world_position(blade_carrier)
    guard_rest = ctx.part_world_position(nose_guard)
    assert carrier_rest is not None
    assert guard_rest is not None

    with ctx.pose({blade_slide: 0.050}):
        carrier_extended = ctx.part_world_position(blade_carrier)
        assert carrier_extended is not None
        ctx.check(
            "blade_carrier_moves_forward",
            carrier_extended[0] > carrier_rest[0] + 0.045,
            f"rest={carrier_rest}, extended={carrier_extended}",
        )
        ctx.expect_within(
            blade_carrier,
            handle_shell,
            axes="yz",
            inner_elem="carrier_sled",
            margin=0.0005,
            name="blade_carrier_stays_guided_when_extended",
        )
        ctx.expect_gap(
            blade_carrier,
            handle_shell,
            axis="x",
            positive_elem="blade_tip",
            min_gap=0.010,
            max_gap=0.030,
            name="blade_tip_projects_beyond_handle_when_slider_is_forward",
        )

    with ctx.pose({guard_slide: 0.014}):
        guard_retracted = ctx.part_world_position(nose_guard)
        assert guard_retracted is not None
        ctx.check(
            "nose_guard_retracts_back_on_its_own_short_axis",
            guard_retracted[0] < guard_rest[0] - 0.010,
            f"rest={guard_rest}, retracted={guard_retracted}",
        )
        ctx.expect_within(
            nose_guard,
            handle_shell,
            axes="yz",
            inner_elem="guide_tongue",
            margin=0.0005,
            name="nose_guard_remains_captured_when_retracted",
        )
        ctx.expect_gap(
            handle_shell,
            nose_guard,
            axis="x",
            positive_elem="front_left_cheek",
            negative_elem="capture_clip",
            min_gap=0.020,
            name="capture_clip_stays_behind_front_lip_through_travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
