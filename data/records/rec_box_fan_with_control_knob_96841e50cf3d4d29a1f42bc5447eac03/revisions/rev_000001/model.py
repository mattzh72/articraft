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
    mesh_from_geometry,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_oscillating_box_fan")

    bracket_steel = model.material("bracket_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    housing_white = model.material("housing_white", rgba=(0.90, 0.91, 0.89, 1.0))
    grille_silver = model.material("grille_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    fan_blade_white = model.material("fan_blade_white", rgba=(0.94, 0.95, 0.95, 1.0))
    fan_hub_gray = model.material("fan_hub_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_indicator = model.material("knob_indicator", rgba=(0.78, 0.15, 0.10, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.02, 0.18, 0.24)),
        origin=Origin(xyz=(-0.01, 0.0, 0.0)),
        material=bracket_steel,
        name="wall_plate",
    )
    wall_bracket.visual(
        Box((0.10, 0.05, 0.09)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        material=bracket_steel,
        name="support_arm",
    )
    wall_bracket.visual(
        Box((0.05, 0.10, 0.12)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=bracket_steel,
        name="pivot_block",
    )
    wall_bracket.visual(
        Cylinder(radius=0.04, length=0.014),
        origin=Origin(xyz=(0.107, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_steel,
        name="pivot_collar",
    )
    wall_bracket.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(-0.003, 0.0, 0.07), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_silver,
        name="upper_wall_fastener",
    )
    wall_bracket.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(-0.003, 0.0, -0.07), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_silver,
        name="lower_wall_fastener",
    )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.13, 0.18, 0.24)),
        mass=2.5,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
    )

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_steel,
        name="pivot_disk",
    )
    housing.visual(
        Cylinder(radius=0.03, length=0.028),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_steel,
        name="pivot_neck",
    )
    housing.visual(
        Cylinder(radius=0.072, length=0.048),
        origin=Origin(xyz=(0.069, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_steel,
        name="motor_pod",
    )
    housing.visual(
        Box((0.135, 0.03, 0.44)),
        origin=Origin(xyz=(0.0975, 0.205, 0.0)),
        material=housing_white,
        name="right_side_wall",
    )
    housing.visual(
        Box((0.135, 0.03, 0.44)),
        origin=Origin(xyz=(0.0975, -0.205, 0.0)),
        material=housing_white,
        name="left_side_wall",
    )
    housing.visual(
        Box((0.135, 0.38, 0.03)),
        origin=Origin(xyz=(0.0975, 0.0, 0.205)),
        material=housing_white,
        name="top_wall",
    )
    housing.visual(
        Box((0.135, 0.38, 0.03)),
        origin=Origin(xyz=(0.0975, 0.0, -0.205)),
        material=housing_white,
        name="bottom_wall",
    )
    housing.visual(
        Box((0.05, 0.23, 0.02)),
        origin=Origin(xyz=(0.068, 0.115, 0.0)),
        material=bracket_steel,
        name="right_motor_strut",
    )
    housing.visual(
        Box((0.05, 0.23, 0.02)),
        origin=Origin(xyz=(0.068, -0.115, 0.0)),
        material=bracket_steel,
        name="left_motor_strut",
    )
    housing.visual(
        Box((0.05, 0.02, 0.23)),
        origin=Origin(xyz=(0.068, 0.0, 0.115)),
        material=bracket_steel,
        name="top_motor_strut",
    )
    housing.visual(
        Box((0.05, 0.02, 0.23)),
        origin=Origin(xyz=(0.068, 0.0, -0.115)),
        material=bracket_steel,
        name="bottom_motor_strut",
    )
    housing.visual(
        Box((0.012, 0.07, 0.07)),
        origin=Origin(xyz=(0.159, -0.182, -0.182)),
        material=housing_white,
        name="control_pad",
    )

    for side_name, side_y in (("left", -0.186), ("right", 0.186)):
        housing.visual(
            Cylinder(radius=0.004, length=0.372),
            origin=Origin(xyz=(0.158, side_y, 0.0)),
            material=grille_silver,
            name=f"grille_frame_{side_name}",
        )
    for side_name, side_z in (("bottom", -0.186), ("top", 0.186)):
        housing.visual(
            Cylinder(radius=0.004, length=0.372),
            origin=Origin(
                xyz=(0.158, 0.0, side_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=grille_silver,
            name=f"grille_frame_{side_name}",
        )

    vertical_rod_positions = (-0.124, -0.062, 0.0, 0.062, 0.124)
    for index, rod_y in enumerate(vertical_rod_positions):
        housing.visual(
            Cylinder(radius=0.0028, length=0.372),
            origin=Origin(xyz=(0.158, rod_y, 0.0)),
            material=grille_silver,
            name="grille_v_center" if index == 2 else f"grille_v_{index}",
        )

    horizontal_rod_positions = (-0.124, -0.062, 0.0, 0.062, 0.124)
    for index, rod_z in enumerate(horizontal_rod_positions):
        housing.visual(
            Cylinder(radius=0.0028, length=0.372),
            origin=Origin(
                xyz=(0.158, 0.0, rod_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=grille_silver,
            name=f"grille_h_{index}",
        )

    housing.inertial = Inertial.from_geometry(
        Box((0.17, 0.44, 0.44)),
        mass=3.4,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
    )

    def blade_section(
        y_pos: float,
        leading_x: float,
        trailing_x: float,
        half_chord: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (leading_x, y_pos, -half_chord),
            (trailing_x, y_pos, -half_chord * 0.62),
            (trailing_x, y_pos, half_chord * 0.62),
            (leading_x, y_pos, half_chord),
        ]

    fan_blade_mesh = mesh_from_geometry(
        section_loft(
            [
                blade_section(-0.010, -0.006, 0.002, 0.036),
                blade_section(0.082, -0.001, 0.007, 0.026),
                blade_section(0.168, 0.004, 0.011, 0.016),
            ]
        ),
        "fan_blade",
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.046, length=0.042),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fan_hub_gray,
        name="hub_shell",
    )
    propeller.visual(
        Cylinder(radius=0.054, length=0.016),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fan_hub_gray,
        name="hub_cap",
    )
    propeller.visual(
        fan_blade_mesh,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=fan_blade_white,
        name="blade_y_pos",
    )
    propeller.visual(
        fan_blade_mesh,
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(math.pi, 0.0, 0.0)),
        material=fan_blade_white,
        name="blade_y_neg",
    )
    propeller.visual(
        fan_blade_mesh,
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fan_blade_white,
        name="blade_z_pos",
    )
    propeller.visual(
        fan_blade_mesh,
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=fan_blade_white,
        name="blade_z_neg",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.06, 0.34, 0.34)),
        mass=0.45,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="selector_base",
    )
    selector_knob.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="selector_dial",
    )
    selector_knob.visual(
        Box((0.004, 0.003, 0.012)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=knob_indicator,
        name="selector_indicator",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Box((0.03, 0.04, 0.04)),
        mass=0.05,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "bracket_to_housing_pan",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=housing,
        origin=Origin(xyz=(0.114, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.9,
            lower=-0.8,
            upper=0.8,
        ),
    )
    model.articulation(
        "housing_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=propeller,
        origin=Origin(xyz=(0.092, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=35.0),
    )
    model.articulation(
        "housing_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=selector_knob,
        origin=Origin(xyz=(0.165, -0.182, -0.182)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=0.0,
            upper=2.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    housing = object_model.get_part("housing")
    propeller = object_model.get_part("propeller")
    selector_knob = object_model.get_part("selector_knob")

    pan_joint = object_model.get_articulation("bracket_to_housing_pan")
    propeller_joint = object_model.get_articulation("housing_to_propeller")
    selector_joint = object_model.get_articulation("housing_to_selector_knob")

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
        "primary articulations use the requested axes",
        pan_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(pan_joint.axis) == (0.0, 0.0, 1.0)
        and propeller_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(propeller_joint.axis) == (1.0, 0.0, 0.0)
        and selector_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(selector_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"pan={pan_joint.articulation_type}/{pan_joint.axis}, "
            f"propeller={propeller_joint.articulation_type}/{propeller_joint.axis}, "
            f"selector={selector_joint.articulation_type}/{selector_joint.axis}"
        ),
    )
    ctx.check(
        "propeller joint remains continuous",
        propeller_joint.motion_limits is not None
        and propeller_joint.motion_limits.lower is None
        and propeller_joint.motion_limits.upper is None,
        details=f"limits={propeller_joint.motion_limits}",
    )

    ctx.expect_contact(
        housing,
        wall_bracket,
        elem_a="pivot_disk",
        elem_b="pivot_collar",
        name="housing bears on the wall bracket pivot collar",
    )
    ctx.expect_contact(
        propeller,
        housing,
        elem_a="hub_shell",
        elem_b="motor_pod",
        name="propeller hub seats against the motor pod",
    )
    ctx.expect_contact(
        selector_knob,
        housing,
        elem_a="selector_base",
        elem_b="control_pad",
        name="selector knob mounts to the front control pad",
    )
    ctx.expect_gap(
        housing,
        propeller,
        axis="x",
        positive_elem="grille_v_center",
        negative_elem="hub_cap",
        min_gap=0.003,
        max_gap=0.018,
        name="spinning hub clears the front grille",
    )
    ctx.expect_overlap(
        propeller,
        housing,
        axes="yz",
        min_overlap=0.30,
        name="propeller disk stays within the square housing footprint",
    )

    knob_rest = ctx.part_world_position(selector_knob)
    with ctx.pose({pan_joint: 0.6}):
        knob_swung = ctx.part_world_position(selector_knob)
    ctx.check(
        "pan joint swings the housing sideways from the bracket",
        knob_rest is not None
        and knob_swung is not None
        and knob_swung[1] > knob_rest[1] + 0.10,
        details=f"rest={knob_rest}, swung={knob_swung}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
