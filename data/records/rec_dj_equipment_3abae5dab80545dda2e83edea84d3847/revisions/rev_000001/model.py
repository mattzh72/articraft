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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cdj_digital_turntable")

    body_dark = model.material("body_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    panel_black = model.material("panel_black", rgba=(0.09, 0.10, 0.11, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.12, 0.12, 0.13, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    display_black = model.material("display_black", rgba=(0.11, 0.11, 0.12, 1.0))
    display_glass = model.material("display_glass", rgba=(0.18, 0.42, 0.50, 0.55))
    control_green = model.material("control_green", rgba=(0.18, 0.62, 0.32, 1.0))
    control_orange = model.material("control_orange", rgba=(0.88, 0.48, 0.16, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.37, 0.39, 0.42, 1.0))

    housing = model.part("housing")

    housing_shell = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.410, 0.320, 0.024, corner_segments=8),
            0.055,
            cap=True,
            center=True,
            closed=True,
        ),
        "cdj_housing_shell",
    )
    housing.visual(
        housing_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=body_dark,
        name="housing_shell",
    )
    housing.visual(
        Box((0.392, 0.298, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0568)),
        material=panel_black,
        name="top_panel",
    )
    housing.visual(
        Box((0.392, 0.088, 0.026)),
        origin=Origin(xyz=(0.0, 0.104, 0.0676)),
        material=body_dark,
        name="display_riser",
    )
    housing.visual(
        Box((0.150, 0.050, 0.004)),
        origin=Origin(xyz=(0.0, 0.071, 0.0830)),
        material=panel_black,
        name="display_shelf",
    )
    housing.visual(
        Box((0.012, 0.028, 0.032)),
        origin=Origin(xyz=(-0.054, 0.104, 0.0710)),
        material=trim_gray,
        name="left_hinge_cheek",
    )
    housing.visual(
        Box((0.012, 0.028, 0.032)),
        origin=Origin(xyz=(0.054, 0.104, 0.0710)),
        material=trim_gray,
        name="right_hinge_cheek",
    )
    housing.visual(
        Box((0.112, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, 0.118, 0.0780)),
        material=trim_gray,
        name="hinge_backstop",
    )
    housing.visual(
        Box((0.014, 0.120, 0.004)),
        origin=Origin(xyz=(0.166, 0.018, 0.0589)),
        material=trim_gray,
        name="pitch_slider_slot",
    )
    housing.visual(
        Box((0.024, 0.024, 0.004)),
        origin=Origin(xyz=(-0.148, -0.109, 0.0589)),
        material=control_green,
        name="play_button",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(-0.186, -0.109, 0.0589)),
        material=control_orange,
        name="cue_button",
    )
    housing.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.142, -0.100, 0.0628)),
        material=trim_gray,
        name="browse_encoder",
    )
    housing.visual(
        Box((0.036, 0.010, 0.003)),
        origin=Origin(xyz=(-0.154, 0.066, 0.0584)),
        material=trim_gray,
        name="loop_button_left",
    )
    housing.visual(
        Box((0.036, 0.010, 0.003)),
        origin=Origin(xyz=(-0.112, 0.066, 0.0584)),
        material=trim_gray,
        name="loop_button_right",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.410, 0.320, 0.102)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
    )

    jog_wheel = model.part("jog_wheel")
    jog_profile = [
        (0.0, 0.000),
        (0.032, 0.000),
        (0.050, 0.0015),
        (0.080, 0.0020),
        (0.090, 0.0055),
        (0.095, 0.0110),
        (0.092, 0.0175),
        (0.084, 0.0210),
        (0.058, 0.0220),
        (0.0, 0.0220),
    ]
    jog_mesh = mesh_from_geometry(LatheGeometry(jog_profile, segments=72), "cdj_jog_wheel")
    jog_wheel.visual(
        jog_mesh,
        material=wheel_black,
        name="platter_shell",
    )
    jog_wheel.visual(
        Cylinder(radius=0.070, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.0198)),
        material=wheel_silver,
        name="touch_disc",
    )
    jog_wheel.visual(
        Cylinder(radius=0.020, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.0223)),
        material=trim_gray,
        name="center_cap",
    )
    jog_wheel.visual(
        Box((0.011, 0.004, 0.002)),
        origin=Origin(xyz=(0.061, 0.0, 0.0226)),
        material=wheel_silver,
        name="rim_marker",
    )
    jog_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.022),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    model.articulation(
        "housing_to_jog_wheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=jog_wheel,
        origin=Origin(xyz=(-0.030, -0.040, 0.0588)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=20.0,
        ),
    )

    display_bezel = model.part("display_bezel")
    display_bezel.visual(
        Cylinder(radius=0.0045, length=0.084),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=display_black,
        name="hinge_barrel",
    )
    display_bezel.visual(
        Box((0.136, 0.084, 0.011)),
        origin=Origin(xyz=(0.0, -0.042, 0.0060)),
        material=display_black,
        name="bezel_shell",
    )
    display_bezel.visual(
        Box((0.112, 0.060, 0.002)),
        origin=Origin(xyz=(0.0, -0.042, 0.0108)),
        material=display_glass,
        name="screen_glass",
    )
    display_bezel.visual(
        Box((0.090, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.004, 0.0035)),
        material=trim_gray,
        name="lower_frame_rib",
    )
    display_bezel.inertial = Inertial.from_geometry(
        Box((0.136, 0.084, 0.014)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.042, 0.007)),
    )

    model.articulation(
        "housing_to_display_bezel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=display_bezel,
        origin=Origin(xyz=(0.0, 0.104, 0.086)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    housing = object_model.get_part("housing")
    jog_wheel = object_model.get_part("jog_wheel")
    display_bezel = object_model.get_part("display_bezel")
    jog_spin = object_model.get_articulation("housing_to_jog_wheel")
    display_tilt = object_model.get_articulation("housing_to_display_bezel")

    ctx.check(
        "jog wheel uses a continuous vertical articulation",
        jog_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(jog_spin.axis) == (0.0, 0.0, 1.0)
        and jog_spin.motion_limits is not None
        and jog_spin.motion_limits.lower is None
        and jog_spin.motion_limits.upper is None,
        details=(
            f"type={jog_spin.articulation_type}, axis={jog_spin.axis}, "
            f"limits={jog_spin.motion_limits}"
        ),
    )
    ctx.check(
        "display tilts on a bottom-edge hinge",
        display_tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(display_tilt.axis) == (-1.0, 0.0, 0.0)
        and display_tilt.motion_limits is not None
        and display_tilt.motion_limits.lower == 0.0
        and display_tilt.motion_limits.upper is not None
        and display_tilt.motion_limits.upper > 1.0,
        details=(
            f"type={display_tilt.articulation_type}, axis={display_tilt.axis}, "
            f"limits={display_tilt.motion_limits}"
        ),
    )

    with ctx.pose({display_tilt: 0.0}):
        ctx.expect_contact(
            jog_wheel,
            housing,
            contact_tol=0.001,
            name="jog wheel platter sits on the deck surface",
        )
        ctx.expect_overlap(
            jog_wheel,
            housing,
            axes="xy",
            min_overlap=0.18,
            name="jog wheel stays centered over the housing footprint",
        )
        ctx.expect_gap(
            display_bezel,
            housing,
            axis="z",
            positive_elem="bezel_shell",
            negative_elem="display_shelf",
            min_gap=0.0005,
            max_gap=0.0030,
            name="stowed display bezel clears the support shelf",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))

    marker_rest = ctx.part_element_world_aabb(jog_wheel, elem="rim_marker")
    with ctx.pose({jog_spin: math.pi / 2.0}):
        marker_quarter = ctx.part_element_world_aabb(jog_wheel, elem="rim_marker")
    marker_rest_center = _aabb_center(marker_rest)
    marker_quarter_center = _aabb_center(marker_quarter)
    marker_rotates = (
        marker_rest_center is not None
        and marker_quarter_center is not None
        and abs(marker_rest_center[0] - marker_quarter_center[0]) > 0.045
        and abs(marker_rest_center[1] - marker_quarter_center[1]) > 0.045
    )
    ctx.check(
        "jog wheel marker visibly rotates with the platter",
        marker_rotates,
        details=f"rest={marker_rest_center}, quarter_turn={marker_quarter_center}",
    )

    glass_rest = ctx.part_element_world_aabb(display_bezel, elem="screen_glass")
    upper_tilt = display_tilt.motion_limits.upper if display_tilt.motion_limits is not None else None
    with ctx.pose({display_tilt: upper_tilt or 0.0}):
        glass_open = ctx.part_element_world_aabb(display_bezel, elem="screen_glass")
    display_raises = (
        glass_rest is not None
        and glass_open is not None
        and glass_open[1][2] > glass_rest[1][2] + 0.045
    )
    ctx.check(
        "display screen rises when tilted open",
        display_raises,
        details=f"rest={glass_rest}, open={glass_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
