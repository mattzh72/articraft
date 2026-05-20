from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    mesh_from_geometry,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_box_fan")

    frame_white = model.material("frame_white", rgba=(0.94, 0.95, 0.97, 1.0))
    housing_white = model.material("housing_white", rgba=(0.90, 0.91, 0.92, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.67, 0.70, 0.73, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.74, 0.77, 0.80, 1.0))
    slider_dark = model.material("slider_dark", rgba=(0.25, 0.27, 0.29, 1.0))

    frame_outer_w = 0.78
    frame_outer_h = 0.40
    frame_depth = 0.18
    jamb_t = 0.08
    head_t = 0.03

    housing_w = 0.59
    housing_h = 0.31
    housing_d = 0.10
    wall_t = 0.012

    frame = model.part("window_frame")
    frame.visual(
        Box((frame_outer_w, frame_depth, head_t)),
        origin=Origin(xyz=(0.0, 0.0, -(frame_outer_h / 2.0 - head_t / 2.0))),
        material=frame_white,
        name="sill",
    )
    frame.visual(
        Box((frame_outer_w, frame_depth, head_t)),
        origin=Origin(xyz=(0.0, 0.0, frame_outer_h / 2.0 - head_t / 2.0)),
        material=frame_white,
        name="head",
    )
    frame.visual(
        Box((jamb_t, frame_depth, frame_outer_h - 2.0 * head_t)),
        origin=Origin(xyz=(-(frame_outer_w / 2.0 - jamb_t / 2.0), 0.0, 0.0)),
        material=frame_white,
        name="left_jamb",
    )
    frame.visual(
        Box((jamb_t, frame_depth, frame_outer_h - 2.0 * head_t)),
        origin=Origin(xyz=(frame_outer_w / 2.0 - jamb_t / 2.0, 0.0, 0.0)),
        material=frame_white,
        name="right_jamb",
    )
    frame.visual(
        Box((0.012, 0.17, 0.30)),
        origin=Origin(xyz=(-0.304, 0.0, 0.0)),
        material=metal_gray,
        name="left_guide",
    )
    frame.visual(
        Box((0.012, 0.17, 0.30)),
        origin=Origin(xyz=(0.304, 0.0, 0.0)),
        material=metal_gray,
        name="right_guide",
    )
    frame.inertial = Inertial.from_geometry(
        Box((frame_outer_w, frame_depth, frame_outer_h)),
        mass=10.0,
    )

    housing = model.part("fan_housing")
    housing.visual(
        Box((housing_w, housing_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, housing_h / 2.0 - wall_t / 2.0)),
        material=housing_white,
        name="top_shell",
    )
    housing.visual(
        Box((housing_w, housing_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, -(housing_h / 2.0 - wall_t / 2.0))),
        material=housing_white,
        name="bottom_shell",
    )
    housing.visual(
        Box((wall_t, housing_d, housing_h - 2.0 * wall_t)),
        origin=Origin(xyz=(-(housing_w / 2.0 - wall_t / 2.0), 0.0, 0.0)),
        material=housing_white,
        name="left_shell",
    )
    housing.visual(
        Box((wall_t, 0.018, 0.22)),
        origin=Origin(xyz=(housing_w / 2.0 - wall_t / 2.0, -0.041, 0.0)),
        material=housing_white,
        name="right_shell_front",
    )
    housing.visual(
        Box((wall_t, 0.018, 0.22)),
        origin=Origin(xyz=(housing_w / 2.0 - wall_t / 2.0, 0.041, 0.0)),
        material=housing_white,
        name="right_shell_rear",
    )
    housing.visual(
        Box((wall_t, 0.064, 0.083)),
        origin=Origin(xyz=(housing_w / 2.0 - wall_t / 2.0, 0.0, 0.1015)),
        material=housing_white,
        name="right_shell_top",
    )
    housing.visual(
        Box((wall_t, 0.064, 0.083)),
        origin=Origin(xyz=(housing_w / 2.0 - wall_t / 2.0, 0.0, -0.1015)),
        material=housing_white,
        name="right_shell_bottom",
    )
    for z_pos in (-0.03, 0.0, 0.03):
        housing.visual(
            Box((0.004, 0.064, 0.006)),
            origin=Origin(xyz=(0.293, 0.0, z_pos)),
            material=metal_gray,
            name=f"vent_slat_{z_pos:+.2f}".replace(".", "_").replace("+", "p").replace("-", "m"),
        )
    housing.visual(
        Box((0.004, 0.004, 0.20)),
        origin=Origin(xyz=(0.297, -0.032, 0.0)),
        material=metal_gray,
        name="vent_track_front",
    )
    housing.visual(
        Box((0.004, 0.004, 0.20)),
        origin=Origin(xyz=(0.297, 0.032, 0.0)),
        material=metal_gray,
        name="vent_track_rear",
    )
    housing.visual(
        Box((0.002, 0.07, 0.28)),
        origin=Origin(xyz=(-0.296, 0.0, 0.0)),
        material=metal_gray,
        name="left_runner",
    )
    housing.visual(
        Box((0.002, 0.07, 0.28)),
        origin=Origin(xyz=(0.296, 0.0, 0.0)),
        material=metal_gray,
        name="right_runner",
    )

    front_y = -(housing_d / 2.0 - 0.002)
    rear_y = housing_d / 2.0 - 0.002
    for y_pos, prefix in ((front_y, "front"), (rear_y, "rear")):
        for x_pos in (-0.18, -0.09, 0.0, 0.09, 0.18):
            housing.visual(
                Box((0.006, 0.004, housing_h - 2.0 * wall_t)),
                origin=Origin(xyz=(x_pos, y_pos, 0.0)),
                material=metal_gray,
                name=f"{prefix}_vert_{x_pos:+.2f}".replace(".", "_").replace("+", "p").replace("-", "m"),
            )
        for z_pos in (-0.10, 0.0, 0.10):
            housing.visual(
                Box((housing_w - 2.0 * wall_t, 0.004, 0.006)),
                origin=Origin(xyz=(0.0, y_pos, z_pos)),
                material=metal_gray,
                name=f"{prefix}_horiz_{z_pos:+.2f}".replace(".", "_").replace("+", "p").replace("-", "m"),
            )

    housing.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.0, 0.042, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="motor_mount",
    )
    housing.visual(
        Box((0.242, 0.008, 0.02)),
        origin=Origin(xyz=(-0.162, 0.044, 0.0)),
        material=metal_gray,
        name="left_motor_strut",
    )
    housing.visual(
        Box((0.242, 0.008, 0.02)),
        origin=Origin(xyz=(0.162, 0.044, 0.0)),
        material=metal_gray,
        name="right_motor_strut",
    )
    housing.visual(
        Box((0.02, 0.008, 0.102)),
        origin=Origin(xyz=(0.0, 0.044, 0.092)),
        material=metal_gray,
        name="top_motor_strut",
    )
    housing.visual(
        Box((0.02, 0.008, 0.102)),
        origin=Origin(xyz=(0.0, 0.044, -0.092)),
        material=metal_gray,
        name="bottom_motor_strut",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_w, housing_d, housing_h)),
        mass=6.0,
    )

    blade = model.part("fan_blade")
    blade_mesh = mesh_from_geometry(
        section_loft(
            [
                [
                    (0.000, -0.003, 0.010),
                    (0.012, -0.003, 0.020),
                    (0.085, -0.003, 0.012),
                    (0.102, -0.003, 0.002),
                    (0.070, -0.003, -0.018),
                    (0.006, -0.003, -0.012),
                ],
                [
                    (0.000, 0.003, 0.006),
                    (0.014, 0.003, 0.016),
                    (0.088, 0.003, 0.006),
                    (0.102, 0.003, -0.004),
                    (0.066, 0.003, -0.022),
                    (0.004, 0.003, -0.016),
                ],
            ]
        ),
        "fan_blade_panel",
    )
    blade.visual(
        Cylinder(radius=0.034, length=0.07),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=blade_gray,
        name="hub",
    )
    blade.visual(
        Cylinder(radius=0.048, length=0.02),
        origin=Origin(xyz=(0.0, -0.01, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blade_gray,
        name="nose_cap",
    )
    blade_radius = 0.032
    for index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0), start=1):
        blade.visual(
            blade_mesh,
            origin=Origin(
                xyz=(blade_radius * cos(angle), 0.0, blade_radius * sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=blade_gray,
            name=f"blade_{index}",
        )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.02),
        mass=0.7,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    vent_slider = model.part("vent_slider")
    vent_slider.visual(
        Box((0.004, 0.058, 0.14)),
        material=slider_dark,
        name="panel",
    )
    vent_slider.visual(
        Box((0.004, 0.02, 0.018)),
        origin=Origin(xyz=(0.0, 0.016, 0.045)),
        material=slider_dark,
        name="thumb_tab",
    )
    vent_slider.inertial = Inertial.from_geometry(
        Box((0.012, 0.058, 0.14)),
        mass=0.2,
    )

    model.articulation(
        "frame_to_housing",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=housing,
        origin=Origin(xyz=(0.0, -0.04, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.15, lower=0.0, upper=0.075),
    )
    model.articulation(
        "housing_to_fan_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=15.0),
    )
    model.articulation(
        "housing_to_vent_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=vent_slider,
        origin=Origin(xyz=(0.281, 0.0, -0.005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.05, lower=0.0, upper=0.07),
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

    frame = object_model.get_part("window_frame")
    housing = object_model.get_part("fan_housing")
    blade = object_model.get_part("fan_blade")
    vent_slider = object_model.get_part("vent_slider")

    housing_slide = object_model.get_articulation("frame_to_housing")
    blade_spin = object_model.get_articulation("housing_to_fan_blade")
    vent_slide = object_model.get_articulation("housing_to_vent_slider")

    ctx.check("frame part exists", frame is not None)
    ctx.check("housing part exists", housing is not None)
    ctx.check("blade part exists", blade is not None)
    ctx.check("vent slider part exists", vent_slider is not None)

    ctx.check(
        "housing slide is depth-axis prismatic",
        housing_slide.articulation_type == ArticulationType.PRISMATIC and housing_slide.axis == (0.0, 1.0, 0.0),
        details=f"type={housing_slide.articulation_type}, axis={housing_slide.axis}",
    )
    ctx.check(
        "blade spin is continuous about the fan axle",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and blade_spin.axis == (0.0, 1.0, 0.0)
        and blade_spin.motion_limits is not None
        and blade_spin.motion_limits.lower is None
        and blade_spin.motion_limits.upper is None,
        details=(
            f"type={blade_spin.articulation_type}, axis={blade_spin.axis}, "
            f"limits={blade_spin.motion_limits}"
        ),
    )
    ctx.check(
        "vent slider is vertical prismatic",
        vent_slide.articulation_type == ArticulationType.PRISMATIC and vent_slide.axis == (0.0, 0.0, 1.0),
        details=f"type={vent_slide.articulation_type}, axis={vent_slide.axis}",
    )

    housing_upper = housing_slide.motion_limits.upper if housing_slide.motion_limits is not None else 0.0
    vent_upper = vent_slide.motion_limits.upper if vent_slide.motion_limits is not None else 0.0

    with ctx.pose({housing_slide: 0.0}):
        ctx.expect_gap(
            housing,
            frame,
            axis="x",
            positive_elem="left_runner",
            negative_elem="left_guide",
            min_gap=0.0005,
            max_gap=0.003,
            name="left runner clears left guide at rest",
        )
        ctx.expect_gap(
            frame,
            housing,
            axis="x",
            positive_elem="right_guide",
            negative_elem="right_runner",
            min_gap=0.0005,
            max_gap=0.003,
            name="right runner clears right guide at rest",
        )
        ctx.expect_within(
            housing,
            frame,
            axes="xz",
            margin=0.0,
            name="housing stays within frame footprint at rest",
        )
        ctx.expect_overlap(
            blade,
            housing,
            axes="xz",
            min_overlap=0.12,
            name="blade assembly stays centered in the housing",
        )
        ctx.expect_overlap(
            vent_slider,
            housing,
            axes="yz",
            min_overlap=0.05,
            name="vent slider stays mounted on the housing side",
        )
        housing_rest_pos = ctx.part_world_position(housing)

    with ctx.pose({housing_slide: housing_upper}):
        ctx.expect_gap(
            housing,
            frame,
            axis="x",
            positive_elem="left_runner",
            negative_elem="left_guide",
            min_gap=0.0005,
            max_gap=0.003,
            name="left runner clears left guide when inserted",
        )
        ctx.expect_gap(
            frame,
            housing,
            axis="x",
            positive_elem="right_guide",
            negative_elem="right_runner",
            min_gap=0.0005,
            max_gap=0.003,
            name="right runner clears right guide when inserted",
        )
        ctx.expect_within(
            housing,
            frame,
            axes="xz",
            margin=0.0,
            name="housing stays within frame footprint when inserted",
        )
        housing_inserted_pos = ctx.part_world_position(housing)

    ctx.check(
        "housing slides into the window channel",
        housing_rest_pos is not None
        and housing_inserted_pos is not None
        and housing_inserted_pos[1] > housing_rest_pos[1] + 0.05,
        details=f"rest={housing_rest_pos}, inserted={housing_inserted_pos}",
    )

    with ctx.pose({vent_slide: 0.0}):
        vent_rest_pos = ctx.part_world_position(vent_slider)

    with ctx.pose({vent_slide: vent_upper}):
        ctx.expect_overlap(
            vent_slider,
            housing,
            axes="yz",
            min_overlap=0.04,
            name="vent slider remains captured by the side tracks when opened",
        )
        vent_open_pos = ctx.part_world_position(vent_slider)

    ctx.check(
        "vent slider opens upward",
        vent_rest_pos is not None and vent_open_pos is not None and vent_open_pos[2] > vent_rest_pos[2] + 0.05,
        details=f"rest={vent_rest_pos}, open={vent_open_pos}",
    )

    with ctx.pose({blade_spin: pi / 2.0}):
        ctx.expect_overlap(
            blade,
            housing,
            axes="xz",
            min_overlap=0.12,
            name="blade remains inside the housing aperture while spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
