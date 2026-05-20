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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barn_ventilation_fan")

    galvanized = model.material("galvanized_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    weathered_panel = model.material("weathered_panel", rgba=(0.79, 0.81, 0.83, 1.0))
    fan_dark = model.material("fan_dark", rgba=(0.22, 0.23, 0.24, 1.0))
    motor_dark = model.material("motor_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    warning_red = model.material("warning_red", rgba=(0.72, 0.18, 0.12, 1.0))

    housing = model.part("housing")

    outer_size = 1.20
    housing_depth = 0.34
    opening_size = 0.92
    frame_band = (outer_size - opening_size) * 0.5

    housing.visual(
        Box((frame_band, housing_depth, outer_size)),
        origin=Origin(xyz=(-(opening_size * 0.5 + frame_band * 0.5), 0.0, 0.0)),
        material=galvanized,
        name="left_wall",
    )
    housing.visual(
        Box((frame_band, housing_depth, outer_size)),
        origin=Origin(xyz=((opening_size * 0.5 + frame_band * 0.5), 0.0, 0.0)),
        material=galvanized,
        name="right_wall",
    )
    housing.visual(
        Box((opening_size, housing_depth, frame_band)),
        origin=Origin(xyz=(0.0, 0.0, (opening_size * 0.5 + frame_band * 0.5))),
        material=galvanized,
        name="top_wall",
    )
    housing.visual(
        Box((opening_size, housing_depth, frame_band)),
        origin=Origin(xyz=(0.0, 0.0, -(opening_size * 0.5 + frame_band * 0.5))),
        material=galvanized,
        name="bottom_wall",
    )

    housing.visual(
        Box((outer_size, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, housing_depth * 0.5 - 0.01, outer_size * 0.5 - 0.03)),
        material=weathered_panel,
        name="front_top_stop",
    )
    housing.visual(
        Box((outer_size, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, housing_depth * 0.5 - 0.01, -(outer_size * 0.5 - 0.03))),
        material=weathered_panel,
        name="front_bottom_stop",
    )
    housing.visual(
        Box((0.06, 0.02, outer_size - 0.12)),
        origin=Origin(xyz=(outer_size * 0.5 - 0.03, housing_depth * 0.5 - 0.01, 0.0)),
        material=weathered_panel,
        name="front_right_stop",
    )
    housing.visual(
        Box((0.06, 0.02, outer_size - 0.12)),
        origin=Origin(xyz=(-(outer_size * 0.5 - 0.03), housing_depth * 0.5 - 0.01, 0.0)),
        material=weathered_panel,
        name="front_left_stop",
    )

    housing.visual(
        Box((opening_size, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.06, 0.0)),
        material=galvanized,
        name="rear_crossbar_horizontal",
    )
    housing.visual(
        Box((0.06, 0.06, opening_size)),
        origin=Origin(xyz=(0.0, -0.06, 0.0)),
        material=galvanized,
        name="rear_crossbar_vertical",
    )
    housing.visual(
        Cylinder(radius=0.12, length=0.16),
        origin=Origin(xyz=(0.0, -0.10, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=motor_dark,
        name="motor_can",
    )
    housing.visual(
        Box((0.05, 0.16, 0.18)),
        origin=Origin(xyz=(outer_size * 0.5 + 0.025, -0.03, -0.30)),
        material=weathered_panel,
        name="control_box",
    )
    housing.visual(
        Box((0.012, 0.10, 0.10)),
        origin=Origin(xyz=(outer_size * 0.5 + 0.031, -0.03, -0.30)),
        material=warning_red,
        name="dial_backplate",
    )
    housing.inertial = Inertial.from_geometry(
        Box((outer_size + 0.05, housing_depth, outer_size)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        Cylinder(radius=0.10, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fan_dark,
        name="hub_shell",
    )
    fan_rotor.visual(
        Cylinder(radius=0.035, length=0.07),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=motor_dark,
        name="shaft_stub",
    )
    fan_rotor.visual(
        Box((0.18, 0.018, 0.11)),
        origin=Origin(xyz=(0.09, 0.016, 0.0), rpy=(0.28, 0.0, 0.0)),
        material=fan_dark,
        name="blade_pos_x_root",
    )
    fan_rotor.visual(
        Box((0.17, 0.014, 0.07)),
        origin=Origin(xyz=(0.225, 0.028, 0.0), rpy=(0.40, 0.0, 0.0)),
        material=fan_dark,
        name="blade_pos_x_tip",
    )
    fan_rotor.visual(
        Box((0.18, 0.018, 0.11)),
        origin=Origin(xyz=(-0.09, 0.016, 0.0), rpy=(-0.28, 0.0, 0.0)),
        material=fan_dark,
        name="blade_neg_x_root",
    )
    fan_rotor.visual(
        Box((0.17, 0.014, 0.07)),
        origin=Origin(xyz=(-0.225, 0.028, 0.0), rpy=(-0.40, 0.0, 0.0)),
        material=fan_dark,
        name="blade_neg_x_tip",
    )
    fan_rotor.visual(
        Box((0.11, 0.018, 0.18)),
        origin=Origin(xyz=(0.0, 0.016, 0.09), rpy=(0.0, 0.0, -0.28)),
        material=fan_dark,
        name="blade_pos_z_root",
    )
    fan_rotor.visual(
        Box((0.07, 0.014, 0.17)),
        origin=Origin(xyz=(0.0, 0.028, 0.225), rpy=(0.0, 0.0, -0.40)),
        material=fan_dark,
        name="blade_pos_z_tip",
    )
    fan_rotor.visual(
        Box((0.11, 0.018, 0.18)),
        origin=Origin(xyz=(0.0, 0.016, -0.09), rpy=(0.0, 0.0, 0.28)),
        material=fan_dark,
        name="blade_neg_z_root",
    )
    fan_rotor.visual(
        Box((0.07, 0.014, 0.17)),
        origin=Origin(xyz=(0.0, 0.028, -0.225), rpy=(0.0, 0.0, 0.40)),
        material=fan_dark,
        name="blade_neg_z_tip",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Box((0.76, 0.12, 0.76)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "housing_to_fan_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan_rotor,
        origin=Origin(xyz=(0.0, 0.03, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )

    panel_thickness = 0.016
    hinge_radius = 0.012
    hinge_y = housing_depth * 0.5 + hinge_radius
    panel_y = -0.004

    top_panel = model.part("top_shutter")
    top_panel_height = 0.235
    top_panel.visual(
        Box((opening_size - 0.01, panel_thickness, top_panel_height)),
        origin=Origin(xyz=(0.0, panel_y, -top_panel_height * 0.5)),
        material=weathered_panel,
        name="panel_skin",
    )
    top_panel.visual(
        Box((opening_size - 0.06, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -0.001, -top_panel_height + 0.020)),
        material=galvanized,
        name="stiffener",
    )
    top_panel.visual(
        Cylinder(radius=hinge_radius, length=opening_size - 0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    top_panel.inertial = Inertial.from_geometry(
        Box((opening_size - 0.01, 0.03, top_panel_height)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -0.002, -top_panel_height * 0.5)),
    )
    model.articulation(
        "housing_to_top_shutter",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=top_panel,
        origin=Origin(xyz=(0.0, hinge_y, opening_size * 0.5)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=1.32,
        ),
    )

    bottom_panel = model.part("bottom_shutter")
    bottom_panel_height = 0.235
    bottom_panel.visual(
        Box((opening_size - 0.01, panel_thickness, bottom_panel_height)),
        origin=Origin(xyz=(0.0, panel_y, bottom_panel_height * 0.5)),
        material=weathered_panel,
        name="panel_skin",
    )
    bottom_panel.visual(
        Box((opening_size - 0.06, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -0.001, bottom_panel_height - 0.020)),
        material=galvanized,
        name="stiffener",
    )
    bottom_panel.visual(
        Cylinder(radius=hinge_radius, length=opening_size - 0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    bottom_panel.inertial = Inertial.from_geometry(
        Box((opening_size - 0.01, 0.03, bottom_panel_height)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -0.002, bottom_panel_height * 0.5)),
    )
    model.articulation(
        "housing_to_bottom_shutter",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=bottom_panel,
        origin=Origin(xyz=(0.0, hinge_y, -(opening_size * 0.5))),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=1.32,
        ),
    )

    side_panel_height = 0.44
    side_panel_width = 0.455

    left_panel = model.part("left_shutter")
    left_panel.visual(
        Box((side_panel_width, panel_thickness, side_panel_height)),
        origin=Origin(xyz=(side_panel_width * 0.5, panel_y, 0.0)),
        material=weathered_panel,
        name="panel_skin",
    )
    left_panel.visual(
        Box((0.035, 0.020, side_panel_height - 0.04)),
        origin=Origin(xyz=(side_panel_width - 0.0175, -0.001, 0.0)),
        material=galvanized,
        name="stiffener",
    )
    left_panel.visual(
        Cylinder(radius=hinge_radius, length=side_panel_height),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    left_panel.inertial = Inertial.from_geometry(
        Box((side_panel_width, 0.03, side_panel_height)),
        mass=1.8,
        origin=Origin(xyz=(side_panel_width * 0.5, -0.002, 0.0)),
    )
    model.articulation(
        "housing_to_left_shutter",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=left_panel,
        origin=Origin(xyz=(-(opening_size * 0.5), hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=1.32,
        ),
    )

    right_panel = model.part("right_shutter")
    right_panel.visual(
        Box((side_panel_width, panel_thickness, side_panel_height)),
        origin=Origin(xyz=(-(side_panel_width * 0.5), panel_y, 0.0)),
        material=weathered_panel,
        name="panel_skin",
    )
    right_panel.visual(
        Box((0.035, 0.020, side_panel_height - 0.04)),
        origin=Origin(xyz=(-(side_panel_width - 0.0175), -0.001, 0.0)),
        material=galvanized,
        name="stiffener",
    )
    right_panel.visual(
        Cylinder(radius=hinge_radius, length=side_panel_height),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    right_panel.inertial = Inertial.from_geometry(
        Box((side_panel_width, 0.03, side_panel_height)),
        mass=1.8,
        origin=Origin(xyz=(-(side_panel_width * 0.5), -0.002, 0.0)),
    )
    model.articulation(
        "housing_to_right_shutter",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=right_panel,
        origin=Origin(xyz=((opening_size * 0.5), hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=1.32,
        ),
    )

    speed_knob = model.part("speed_dial")
    speed_knob.visual(
        Cylinder(radius=0.036, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_dark,
        name="knob_body",
    )
    speed_knob.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_dark,
        name="knob_stem",
    )
    speed_knob.visual(
        Box((0.010, 0.004, 0.020)),
        origin=Origin(xyz=(0.040, 0.0, 0.024)),
        material=warning_red,
        name="indicator_tab",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Box((0.055, 0.080, 0.080)),
        mass=0.24,
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
    )
    model.articulation(
        "housing_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(outer_size * 0.5 + 0.05, -0.03, -0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
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
    fan_rotor = object_model.get_part("fan_rotor")
    top_shutter = object_model.get_part("top_shutter")
    bottom_shutter = object_model.get_part("bottom_shutter")
    left_shutter = object_model.get_part("left_shutter")
    right_shutter = object_model.get_part("right_shutter")
    speed_dial = object_model.get_part("speed_dial")

    rotor_joint = object_model.get_articulation("housing_to_fan_rotor")
    top_joint = object_model.get_articulation("housing_to_top_shutter")
    bottom_joint = object_model.get_articulation("housing_to_bottom_shutter")
    left_joint = object_model.get_articulation("housing_to_left_shutter")
    right_joint = object_model.get_articulation("housing_to_right_shutter")
    knob_joint = object_model.get_articulation("housing_to_speed_dial")

    ctx.check(
        "fan rotor uses continuous articulation",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS
        and rotor_joint.motion_limits is not None
        and rotor_joint.motion_limits.lower is None
        and rotor_joint.motion_limits.upper is None
        and rotor_joint.axis == (0.0, 1.0, 0.0),
        details=(
            f"type={rotor_joint.articulation_type}, axis={rotor_joint.axis}, "
            f"limits={rotor_joint.motion_limits}"
        ),
    )
    ctx.check(
        "speed dial uses continuous articulation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None
        and knob_joint.axis == (1.0, 0.0, 0.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}, limits={knob_joint.motion_limits}",
    )

    ctx.check(
        "shutter hinge axes open outward",
        top_joint.axis == (1.0, 0.0, 0.0)
        and bottom_joint.axis == (-1.0, 0.0, 0.0)
        and left_joint.axis == (0.0, 0.0, 1.0)
        and right_joint.axis == (0.0, 0.0, -1.0),
        details=(
            f"top={top_joint.axis}, bottom={bottom_joint.axis}, "
            f"left={left_joint.axis}, right={right_joint.axis}"
        ),
    )

    ctx.expect_within(
        fan_rotor,
        housing,
        axes="xyz",
        margin=0.02,
        name="fan rotor stays inside housing envelope",
    )
    ctx.expect_gap(
        speed_dial,
        housing,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem="knob_stem",
        negative_elem="control_box",
        name="speed dial mounts flush to the control box",
    )

    def _center_y(part, *, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    rest_centers = {
        "top": _center_y(top_shutter, elem="panel_skin"),
        "bottom": _center_y(bottom_shutter, elem="panel_skin"),
        "left": _center_y(left_shutter, elem="panel_skin"),
        "right": _center_y(right_shutter, elem="panel_skin"),
    }
    with ctx.pose(
        {
            top_joint: 1.15,
            bottom_joint: 1.15,
            left_joint: 1.15,
            right_joint: 1.15,
        }
    ):
        open_centers = {
            "top": _center_y(top_shutter, elem="panel_skin"),
            "bottom": _center_y(bottom_shutter, elem="panel_skin"),
            "left": _center_y(left_shutter, elem="panel_skin"),
            "right": _center_y(right_shutter, elem="panel_skin"),
        }
        ctx.expect_gap(
            top_shutter,
            housing,
            axis="y",
            min_gap=0.006,
            positive_elem="panel_skin",
            name="top shutter clears the housing when opened",
        )
        ctx.expect_gap(
            bottom_shutter,
            housing,
            axis="y",
            min_gap=0.006,
            positive_elem="panel_skin",
            name="bottom shutter clears the housing when opened",
        )
        ctx.expect_gap(
            left_shutter,
            housing,
            axis="y",
            min_gap=0.006,
            positive_elem="panel_skin",
            name="left shutter clears the housing when opened",
        )
        ctx.expect_gap(
            right_shutter,
            housing,
            axis="y",
            min_gap=0.006,
            positive_elem="panel_skin",
            name="right shutter clears the housing when opened",
        )

    for name in ("top", "bottom", "left", "right"):
        rest_y = rest_centers[name]
        open_y = open_centers[name]
        ctx.check(
            f"{name} shutter swings outward",
            rest_y is not None and open_y is not None and open_y > rest_y + 0.05,
            details=f"rest_center_y={rest_y}, open_center_y={open_y}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
