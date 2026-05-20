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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
    rounded_rect_profile,
)


def _rounded_rect_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    height: float,
    radius: float,
):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, depth, radius, corner_segments=10),
            height,
            cap=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    body = model.material("body", rgba=(0.82, 0.84, 0.86, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    black = model.material("black", rgba=(0.08, 0.09, 0.10, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.31, 0.33, 0.35, 1.0))
    steel = model.material("steel", rgba=(0.61, 0.65, 0.69, 1.0))

    base_width = 0.34
    base_depth = 0.24
    base_height = 0.045

    housing_width = 0.13
    housing_depth = 0.11
    wall_thickness = 0.004
    collar_radius = 0.07
    collar_height = 0.018
    column_bottom = 0.014
    column_height = 0.908
    column_top = column_bottom + column_height
    top_cap_height = 0.014

    front_opening_width = 0.064
    front_opening_bottom = 0.118
    front_opening_height = 0.722
    front_opening_top = front_opening_bottom + front_opening_height

    rear_opening_width = 0.084
    rear_opening_bottom = 0.148
    rear_opening_height = 0.582
    rear_opening_top = rear_opening_bottom + rear_opening_height

    impeller_radius = 0.041
    impeller_height = 0.742
    impeller_center_y = 0.008
    impeller_center_z = 0.462

    base = model.part("pedestal_base")
    base.visual(
        _rounded_rect_mesh(
            "tower_fan_base_shell",
            width=base_width,
            depth=base_depth,
            height=base_height,
            radius=0.060,
        ),
        material=charcoal,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.074, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, base_height - 0.004)),
        material=dark_grey,
        name="bearing_pad",
    )

    housing = model.part("tower_housing")
    housing.visual(
        Cylinder(radius=collar_radius, length=collar_height),
        origin=Origin(xyz=(0.0, 0.0, collar_height * 0.5)),
        material=body,
        name="turntable_collar",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_grey,
        name="drive_hub",
    )
    side_panel_height = column_height
    side_panel_z = column_bottom + (side_panel_height * 0.5)
    side_x = (housing_width - wall_thickness) * 0.5
    housing.visual(
        Box((wall_thickness, housing_depth, side_panel_height)),
        origin=Origin(xyz=(-side_x, 0.0, side_panel_z)),
        material=body,
        name="left_side_panel",
    )
    housing.visual(
        Box((wall_thickness, housing_depth, side_panel_height)),
        origin=Origin(xyz=(side_x, 0.0, side_panel_z)),
        material=body,
        name="right_side_panel",
    )

    front_face_y = (housing_depth - wall_thickness) * 0.5
    front_upper_height = column_top - front_opening_top
    front_lower_height = front_opening_bottom - column_bottom
    front_stile_width = 0.019
    front_stile_x = (front_opening_width * 0.5) + (front_stile_width * 0.5) - 0.001
    housing.visual(
        Box((housing_width - (2.0 * wall_thickness), wall_thickness, front_lower_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y,
                column_bottom + (front_lower_height * 0.5),
            )
        ),
        material=body,
        name="front_lower_panel",
    )
    housing.visual(
        Box((housing_width - (2.0 * wall_thickness), wall_thickness, front_upper_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y,
                front_opening_top + (front_upper_height * 0.5),
            )
        ),
        material=body,
        name="front_upper_panel",
    )
    housing.visual(
        Box((front_stile_width, wall_thickness, front_opening_height + 0.004)),
        origin=Origin(
            xyz=(
                -front_stile_x,
                front_face_y,
                front_opening_bottom + (front_opening_height * 0.5),
            )
        ),
        material=body,
        name="front_left_stile",
    )
    housing.visual(
        Box((front_stile_width, wall_thickness, front_opening_height + 0.004)),
        origin=Origin(
            xyz=(
                front_stile_x,
                front_face_y,
                front_opening_bottom + (front_opening_height * 0.5),
            )
        ),
        material=body,
        name="front_right_stile",
    )
    for index in range(11):
        z = front_opening_bottom + 0.040 + (index * 0.060)
        housing.visual(
            Box((front_opening_width + 0.004, wall_thickness * 0.8, 0.005)),
            origin=Origin(xyz=(0.0, front_face_y - 0.0008, z)),
            material=black,
            name=f"front_grille_slat_{index:02d}",
        )

    rear_face_y = -front_face_y
    rear_lower_height = rear_opening_bottom - column_bottom
    rear_upper_height = column_top - rear_opening_top
    rear_stile_width = 0.016
    rear_stile_x = (rear_opening_width * 0.5) + (rear_stile_width * 0.5) - 0.001
    housing.visual(
        Box((housing_width - (2.0 * wall_thickness), wall_thickness, rear_lower_height)),
        origin=Origin(
            xyz=(
                0.0,
                rear_face_y,
                column_bottom + (rear_lower_height * 0.5),
            )
        ),
        material=body,
        name="rear_lower_panel",
    )
    housing.visual(
        Box((housing_width - (2.0 * wall_thickness), wall_thickness, rear_upper_height)),
        origin=Origin(
            xyz=(
                0.0,
                rear_face_y,
                rear_opening_top + (rear_upper_height * 0.5),
            )
        ),
        material=body,
        name="rear_top_panel",
    )
    housing.visual(
        Box((rear_stile_width, wall_thickness, rear_opening_height + 0.004)),
        origin=Origin(
            xyz=(
                -rear_stile_x,
                rear_face_y,
                rear_opening_bottom + (rear_opening_height * 0.5),
            )
        ),
        material=body,
        name="rear_left_stile",
    )
    housing.visual(
        Box((rear_stile_width, wall_thickness, rear_opening_height + 0.004)),
        origin=Origin(
            xyz=(
                rear_stile_x,
                rear_face_y,
                rear_opening_bottom + (rear_opening_height * 0.5),
            )
        ),
        material=body,
        name="rear_right_stile",
    )
    rear_slat_span = rear_opening_width - 0.010
    for index in range(12):
        if index == 5:
            x = -0.004
        elif index == 6:
            x = 0.004
        else:
            x = -rear_slat_span * 0.5 + (rear_slat_span * index / 11.0)
        housing.visual(
            Box((0.003, wall_thickness, rear_opening_height + 0.004)),
            origin=Origin(
                xyz=(
                    x,
                    rear_face_y,
                    rear_opening_bottom + (rear_opening_height * 0.5),
                )
            ),
            material=black,
            name=f"rear_intake_slat_{index:02d}",
        )

    housing.visual(
        Box((0.040, housing_depth - (2.0 * wall_thickness), 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=dark_grey,
        name="lower_motor_cradle",
    )
    housing.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(
            xyz=(
                0.0,
                impeller_center_y,
                impeller_center_z - (impeller_height * 0.5) - 0.005,
            )
        ),
        material=dark_grey,
        name="lower_bearing_boss",
    )
    housing.visual(
        Box((housing_width - (2.0 * wall_thickness), 0.020, 0.012)),
        origin=Origin(xyz=(0.0, impeller_center_y, 0.849)),
        material=dark_grey,
        name="upper_bearing_bridge",
    )
    housing.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(
            xyz=(
                0.0,
                impeller_center_y,
                impeller_center_z + (impeller_height * 0.5) + 0.005,
            )
        ),
        material=dark_grey,
        name="upper_bearing_boss",
    )

    housing.visual(
        _rounded_rect_mesh(
            "tower_fan_top_cap",
            width=housing_width - 0.006,
            depth=housing_depth - 0.006,
            height=top_cap_height,
            radius=0.028,
        ),
        origin=Origin(xyz=(0.0, 0.0, column_top - top_cap_height)),
        material=body,
        name="top_cap",
    )
    housing.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, rear_face_y - 0.001, column_top - 0.040)),
        material=dark_grey,
        name="rear_switch_panel",
    )

    impeller = model.part("impeller_drum")
    impeller.visual(
        Cylinder(radius=0.008, length=impeller_height),
        material=dark_grey,
        name="axle",
    )
    impeller.visual(
        Cylinder(radius=impeller_radius, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -(impeller_height * 0.5) + 0.007)),
        material=steel,
        name="lower_end_disc",
    )
    impeller.visual(
        Cylinder(radius=impeller_radius, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, (impeller_height * 0.5) - 0.007)),
        material=steel,
        name="upper_end_disc",
    )
    blade_radius = 0.031
    for index in range(16):
        angle = (2.0 * math.pi * index) / 16.0
        impeller.visual(
            Box((0.004, 0.016, impeller_height + 0.004)),
            origin=Origin(
                xyz=(
                    blade_radius * math.cos(angle),
                    blade_radius * math.sin(angle),
                    0.0,
                ),
                rpy=(0.0, 0.0, angle + 0.32),
            ),
            material=steel,
            name=f"blade_{index:02d}",
        )

    knob = model.part("speed_selector_knob")
    knob.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="knob_body",
    )
    knob.visual(
        Box((0.003, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.020, 0.012)),
        material=steel,
        name="indicator_ridge",
    )

    model.articulation(
        "base_oscillation",
        ArticulationType.REVOLUTE,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, base_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=-1.05,
            upper=1.05,
        ),
    )
    model.articulation(
        "housing_to_impeller",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=impeller,
        origin=Origin(xyz=(0.0, impeller_center_y, impeller_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=18.0,
        ),
    )
    model.articulation(
        "housing_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=knob,
        origin=place_on_surface(
            knob,
            housing.get_visual("rear_top_panel"),
            point_hint=(0.0, -housing_depth * 0.5, 0.782),
            child_axis="-y",
            clearance=0.0,
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=2.5,
            lower=0.0,
            upper=4.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("pedestal_base")
    housing = object_model.get_part("tower_housing")
    impeller = object_model.get_part("impeller_drum")
    knob = object_model.get_part("speed_selector_knob")
    oscillation = object_model.get_articulation("base_oscillation")
    impeller_joint = object_model.get_articulation("housing_to_impeller")
    knob_joint = object_model.get_articulation("housing_to_speed_knob")

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
        "fan_parts_present",
        all(part is not None for part in (base, housing, impeller, knob)),
        "Expected pedestal base, tower housing, impeller drum, and speed selector knob.",
    )
    ctx.check(
        "oscillation_joint_vertical",
        tuple(oscillation.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical oscillation axis, got {oscillation.axis}.",
    )
    ctx.check(
        "impeller_joint_is_continuous",
        impeller_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"Expected continuous impeller joint, got {impeller_joint.articulation_type}.",
    )
    ctx.check(
        "knob_joint_rear_spindle_axis",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected rear knob revolute axis along y, got {knob_joint.articulation_type} {knob_joint.axis}.",
    )

    ctx.expect_gap(
        housing,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="housing_seated_on_base",
    )
    ctx.expect_overlap(
        housing,
        base,
        axes="xy",
        min_overlap=0.10,
        name="housing_centered_over_base",
    )
    ctx.expect_within(
        impeller,
        housing,
        axes=("x", "y", "z"),
        margin=0.0,
        name="impeller_within_housing",
    )
    ctx.expect_contact(
        impeller,
        housing,
        contact_tol=0.001,
        name="impeller_supported_by_bearings",
    )
    ctx.expect_contact(
        knob,
        housing,
        contact_tol=0.001,
        name="knob_contacts_rear_panel",
    )
    ctx.expect_within(
        knob,
        housing,
        axes=("x", "z"),
        margin=0.0,
        name="knob_within_rear_panel_span",
    )

    with ctx.pose({oscillation: 0.8}):
        ctx.expect_gap(
            housing,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="housing_seated_when_oscillated",
        )
        ctx.expect_overlap(
            housing,
            base,
            axes="xy",
            min_overlap=0.10,
            name="housing_supported_when_oscillated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
