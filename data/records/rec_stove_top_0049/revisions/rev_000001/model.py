from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_gas_stove", assets=ASSETS)

    enamel_white = model.material("enamel_white", rgba=(0.93, 0.94, 0.95, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.08, 0.08, 0.09, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    glass = model.material("oven_glass", rgba=(0.18, 0.24, 0.28, 0.45))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    burner_black = model.material("burner_black", rgba=(0.07, 0.07, 0.08, 1.0))
    burner_gray = model.material("burner_gray", rgba=(0.27, 0.28, 0.30, 1.0))

    body_width = 0.760
    body_depth = 0.620
    cabinet_height = 0.825
    cooktop_width = 0.800
    cooktop_depth = 0.660
    cooktop_thickness = 0.035
    cooktop_top_z = cabinet_height + cooktop_thickness
    cooktop_center_y = -0.010
    front_y = -body_depth * 0.5
    back_y = body_depth * 0.5
    panel_t = 0.018
    back_t = 0.015

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    center_burner_base_mesh = save_mesh(
        "center_burner_base.obj",
        ExtrudeGeometry(
            rounded_rect_profile(0.172, 0.088, 0.034, corner_segments=10),
            0.008,
            cap=True,
            center=True,
        ),
    )
    center_burner_cap_mesh = save_mesh(
        "center_burner_cap.obj",
        ExtrudeGeometry(
            rounded_rect_profile(0.150, 0.070, 0.030, corner_segments=10),
            0.012,
            cap=True,
            center=True,
        ),
    )
    knob_body_mesh = save_mesh(
        "control_knob_body.obj",
        LatheGeometry(
            [
                (0.0, -0.016),
                (0.024, -0.016),
                (0.027, -0.013),
                (0.028, -0.009),
                (0.028, -0.005),
                (0.026, -0.001),
                (0.024, 0.008),
                (0.020, 0.013),
                (0.013, 0.016),
                (0.0, 0.016),
            ],
            segments=48,
        ),
    )

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((cooktop_width, cooktop_depth, 0.910)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
    )

    chassis.visual(
        Box((panel_t, body_depth, cabinet_height)),
        origin=Origin(xyz=(-body_width * 0.5 + panel_t * 0.5, 0.0, cabinet_height * 0.5)),
        material=enamel_white,
        name="left_side_panel",
    )
    chassis.visual(
        Box((panel_t, body_depth, cabinet_height)),
        origin=Origin(xyz=(body_width * 0.5 - panel_t * 0.5, 0.0, cabinet_height * 0.5)),
        material=enamel_white,
        name="right_side_panel",
    )
    chassis.visual(
        Box((body_width - 2.0 * panel_t, back_t, cabinet_height)),
        origin=Origin(xyz=(0.0, back_y - back_t * 0.5, cabinet_height * 0.5)),
        material=enamel_white,
        name="rear_panel",
    )
    chassis.visual(
        Box((body_width - 2.0 * panel_t, body_depth - back_t, 0.030)),
        origin=Origin(xyz=(0.0, -0.0075, 0.015)),
        material=graphite,
        name="bottom_panel",
    )
    chassis.visual(
        Box((0.644, 0.020, 0.090)),
        origin=Origin(xyz=(0.0, -0.300, 0.045)),
        material=enamel_white,
        name="toe_kick",
    )
    chassis.visual(
        Box((0.050, 0.020, 0.655)),
        origin=Origin(xyz=(-0.331, -0.300, 0.4025)),
        material=enamel_white,
        name="left_front_stile",
    )
    chassis.visual(
        Box((0.050, 0.020, 0.655)),
        origin=Origin(xyz=(0.331, -0.300, 0.4025)),
        material=enamel_white,
        name="right_front_stile",
    )
    chassis.visual(
        Box((0.662, 0.020, 0.070)),
        origin=Origin(xyz=(0.0, -0.300, 0.710)),
        material=stainless,
        name="oven_vent_panel",
    )
    chassis.visual(
        Box((0.700, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, -0.300, 0.785)),
        material=stainless,
        name="control_rail",
    )

    cavity_width = 0.600
    cavity_depth = 0.520
    cavity_height = 0.580
    cavity_y = -0.030
    cavity_z = 0.400
    chassis.visual(
        Box((0.012, cavity_depth, cavity_height)),
        origin=Origin(xyz=(-0.300, cavity_y, cavity_z)),
        material=graphite,
        name="oven_liner_left",
    )
    chassis.visual(
        Box((0.012, cavity_depth, cavity_height)),
        origin=Origin(xyz=(0.300, cavity_y, cavity_z)),
        material=graphite,
        name="oven_liner_right",
    )
    chassis.visual(
        Box((cavity_width, 0.012, cavity_height)),
        origin=Origin(xyz=(0.0, 0.236, cavity_z)),
        material=graphite,
        name="oven_liner_back",
    )
    chassis.visual(
        Box((cavity_width, cavity_depth, 0.015)),
        origin=Origin(xyz=(0.0, cavity_y, 0.1025)),
        material=graphite,
        name="oven_floor",
    )
    chassis.visual(
        Box((cavity_width, cavity_depth, 0.015)),
        origin=Origin(xyz=(0.0, cavity_y, 0.6975)),
        material=graphite,
        name="oven_roof",
    )

    chassis.visual(
        Box((cooktop_width, cooktop_depth, cooktop_thickness)),
        origin=Origin(xyz=(0.0, cooktop_center_y, cabinet_height + cooktop_thickness * 0.5)),
        material=stainless,
        name="cooktop_slab",
    )
    chassis.visual(
        Box((0.720, 0.580, 0.006)),
        origin=Origin(xyz=(0.0, cooktop_center_y, cooktop_top_z + 0.003)),
        material=graphite,
        name="cooktop_surface",
    )

    round_burners = [
        ("burner_front_left", -0.225, -0.135),
        ("burner_rear_left", -0.225, 0.135),
        ("burner_front_right", 0.225, -0.135),
        ("burner_rear_right", 0.225, 0.135),
    ]
    for burner_name, burner_x, burner_y in round_burners:
        chassis.visual(
            Cylinder(radius=0.046, length=0.006),
            origin=Origin(xyz=(burner_x, burner_y, cooktop_top_z + 0.009)),
            material=burner_gray,
            name=f"{burner_name}_base",
        )
        chassis.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(burner_x, burner_y, cooktop_top_z + 0.017)),
            material=burner_black,
            name=f"{burner_name}_cap",
        )

    chassis.visual(
        center_burner_base_mesh,
        origin=Origin(xyz=(0.0, 0.0, cooktop_top_z + 0.010)),
        material=burner_gray,
        name="center_burner_base",
    )
    chassis.visual(
        center_burner_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, cooktop_top_z + 0.020)),
        material=burner_black,
        name="center_burner_cap",
    )

    grate_z = cooktop_top_z + 0.033
    grate_h = 0.010
    chassis.visual(
        Box((0.700, 0.012, grate_h)),
        origin=Origin(xyz=(0.0, -0.250, grate_z)),
        material=cast_iron,
        name="grate_front_bar",
    )
    chassis.visual(
        Box((0.700, 0.012, grate_h)),
        origin=Origin(xyz=(0.0, 0.230, grate_z)),
        material=cast_iron,
        name="grate_rear_bar",
    )
    chassis.visual(
        Box((0.012, 0.492, grate_h)),
        origin=Origin(xyz=(-0.344, -0.010, grate_z)),
        material=cast_iron,
        name="grate_left_bar",
    )
    chassis.visual(
        Box((0.012, 0.492, grate_h)),
        origin=Origin(xyz=(0.344, -0.010, grate_z)),
        material=cast_iron,
        name="grate_right_bar",
    )
    chassis.visual(
        Box((0.012, 0.492, grate_h)),
        origin=Origin(xyz=(-0.180, -0.010, grate_z)),
        material=cast_iron,
        name="grate_left_longitudinal",
    )
    chassis.visual(
        Box((0.012, 0.492, grate_h)),
        origin=Origin(xyz=(0.180, -0.010, grate_z)),
        material=cast_iron,
        name="grate_right_longitudinal",
    )
    chassis.visual(
        Box((0.560, 0.012, grate_h)),
        origin=Origin(xyz=(0.0, -0.135, grate_z)),
        material=cast_iron,
        name="grate_front_cross",
    )
    chassis.visual(
        Box((0.560, 0.012, grate_h)),
        origin=Origin(xyz=(0.0, 0.135, grate_z)),
        material=cast_iron,
        name="grate_rear_cross",
    )
    for foot_name, foot_x, foot_y in [
        ("grate_foot_fl", -0.344, -0.250),
        ("grate_foot_fr", 0.344, -0.250),
        ("grate_foot_rl", -0.344, 0.230),
        ("grate_foot_rr", 0.344, 0.230),
        ("grate_foot_cl", -0.180, -0.010),
        ("grate_foot_cr", 0.180, -0.010),
    ]:
        chassis.visual(
            Box((0.014, 0.014, 0.022)),
            origin=Origin(xyz=(foot_x, foot_y, cooktop_top_z + 0.017)),
            material=cast_iron,
            name=foot_name,
        )

    door_hinge_y = front_y - 0.001
    door_hinge_z = 0.110
    chassis.visual(
        Box((0.040, 0.024, 0.024)),
        origin=Origin(xyz=(-0.315, door_hinge_y + 0.012, door_hinge_z - 0.012)),
        material=burner_gray,
        name="door_hinge_support_left",
    )
    chassis.visual(
        Box((0.040, 0.024, 0.024)),
        origin=Origin(xyz=(0.315, door_hinge_y + 0.012, door_hinge_z - 0.012)),
        material=burner_gray,
        name="door_hinge_support_right",
    )

    knob_positions = [
        (-0.248, 0.771),
        (-0.124, 0.778),
        (0.000, 0.782),
        (0.124, 0.778),
        (0.248, 0.771),
    ]
    knob_parts = []
    for index, (knob_x, knob_z) in enumerate(knob_positions, start=1):
        knob = model.part(f"knob_{index}")
        knob.visual(
            knob_body_mesh,
            origin=Origin(
                xyz=(0.0, -0.016, 0.0),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.005, 0.014, 0.024)),
            origin=Origin(xyz=(0.0, -0.012, 0.012)),
            material=stainless,
            name="knob_indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.056, 0.032, 0.056)),
            mass=0.050,
            origin=Origin(xyz=(0.0, -0.012, 0.0)),
        )
        model.articulation(
            f"control_rail_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=knob,
            origin=Origin(xyz=(knob_x, front_y, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=8.0),
        )
        knob_parts.append(knob)

    oven_door = model.part("oven_door")
    door_width = 0.662
    door_height = 0.620
    door_t = 0.028
    oven_door.visual(
        Box((door_width, door_t, door_height)),
        origin=Origin(xyz=(0.0, -door_t * 0.5, door_height * 0.5)),
        material=enamel_white,
        name="door_panel",
    )
    oven_door.visual(
        Box((0.420, 0.004, 0.250)),
        origin=Origin(xyz=(0.0, -0.024, 0.395)),
        material=glass,
        name="door_glass",
    )
    oven_door.visual(
        Cylinder(radius=0.012, length=0.420),
        origin=Origin(
            xyz=(0.0, -0.048, 0.508),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=stainless,
        name="door_handle_bar",
    )
    oven_door.visual(
        Box((0.020, 0.028, 0.060)),
        origin=Origin(xyz=(-0.150, -0.038, 0.508)),
        material=stainless,
        name="door_handle_left_post",
    )
    oven_door.visual(
        Box((0.020, 0.028, 0.060)),
        origin=Origin(xyz=(0.150, -0.038, 0.508)),
        material=stainless,
        name="door_handle_right_post",
    )
    oven_door.visual(
        Cylinder(radius=0.012, length=0.100),
        origin=Origin(
            xyz=(-0.245, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=burner_gray,
        name="door_hinge_left",
    )
    oven_door.visual(
        Cylinder(radius=0.012, length=0.100),
        origin=Origin(
            xyz=(0.245, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=burner_gray,
        name="door_hinge_right",
    )
    oven_door.visual(
        Box((0.024, 0.018, 0.040)),
        origin=Origin(xyz=(-0.245, -0.009, 0.020)),
        material=burner_gray,
        name="door_hinge_left_bracket",
    )
    oven_door.visual(
        Box((0.024, 0.018, 0.040)),
        origin=Origin(xyz=(0.245, -0.009, 0.020)),
        material=burner_gray,
        name="door_hinge_right_bracket",
    )
    oven_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.080, door_height)),
        mass=15.0,
        origin=Origin(xyz=(0.0, -0.020, door_height * 0.5)),
    )
    model.articulation(
        "oven_door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=oven_door,
        origin=Origin(xyz=(0.0, door_hinge_y, door_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    ctx.fail_if_articulation_overlaps(max_pose_samples=40, name="articulation_pose_clearance")

    chassis = object_model.get_part("chassis")
    oven_door = object_model.get_part("oven_door")
    knobs = [object_model.get_part(f"knob_{index}") for index in range(1, 6)]

    door_joint = object_model.get_articulation("oven_door_hinge")
    knob_joints = [object_model.get_articulation(f"control_rail_to_knob_{index}") for index in range(1, 6)]

    def axis_matches(actual, expected) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(actual, expected))

    chassis_aabb = ctx.part_world_aabb(chassis)
    assert chassis_aabb is not None
    chassis_size = tuple(
        chassis_aabb[1][axis] - chassis_aabb[0][axis] for axis in range(3)
    )
    ctx.check(
        "stove_width_realistic",
        0.79 <= chassis_size[0] <= 0.81,
        f"width={chassis_size[0]:.3f} m",
    )
    ctx.check(
        "stove_depth_realistic",
        0.65 <= chassis_size[1] <= 0.67,
        f"depth={chassis_size[1]:.3f} m",
    )
    ctx.check(
        "stove_height_realistic",
        0.89 <= chassis_size[2] <= 0.91,
        f"height={chassis_size[2]:.3f} m",
    )

    center_burner_aabb = ctx.part_element_world_aabb(chassis, elem="center_burner_cap")
    assert center_burner_aabb is not None
    center_burner_dx = center_burner_aabb[1][0] - center_burner_aabb[0][0]
    center_burner_dy = center_burner_aabb[1][1] - center_burner_aabb[0][1]
    ctx.check(
        "center_burner_is_oval",
        center_burner_dx > 0.14 and center_burner_dx > center_burner_dy * 1.7,
        f"center burner footprint={center_burner_dx:.3f} x {center_burner_dy:.3f} m",
    )

    round_burner_names = (
        "burner_front_left_cap",
        "burner_rear_left_cap",
        "burner_front_right_cap",
        "burner_rear_right_cap",
    )
    round_burner_centers = []
    for burner_name in round_burner_names:
        burner_aabb = ctx.part_element_world_aabb(chassis, elem=burner_name)
        assert burner_aabb is not None
        round_burner_centers.append(
            tuple((burner_aabb[0][axis] + burner_aabb[1][axis]) * 0.5 for axis in range(3))
        )
    ctx.check(
        "five_burner_layout_present",
        (
            round_burner_centers[0][0] < -0.15
            and round_burner_centers[1][0] < -0.15
            and round_burner_centers[2][0] > 0.15
            and round_burner_centers[3][0] > 0.15
            and round_burner_centers[0][1] < -0.05
            and round_burner_centers[1][1] > 0.05
            and round_burner_centers[2][1] < -0.05
            and round_burner_centers[3][1] > 0.05
        ),
        f"round burner centers={round_burner_centers}",
    )

    ctx.check(
        "door_hinge_axis_is_horizontal_x",
        axis_matches(door_joint.axis, (1.0, 0.0, 0.0)),
        f"axis={door_joint.axis}",
    )
    ctx.check(
        "door_hinge_limit_is_realistic",
        (
            door_joint.motion_limits is not None
            and door_joint.motion_limits.lower == 0.0
            and door_joint.motion_limits.upper is not None
            and 1.10 <= door_joint.motion_limits.upper <= 1.35
        ),
        f"limits={door_joint.motion_limits}",
    )

    knob_positions = []
    for index, (knob, joint) in enumerate(zip(knobs, knob_joints), start=1):
        ctx.check(
            f"knob_{index}_continuous_joint",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"type={joint.articulation_type}",
        )
        ctx.check(
            f"knob_{index}_front_to_back_axis",
            axis_matches(joint.axis, (0.0, 1.0, 0.0)),
            f"axis={joint.axis}",
        )
        ctx.expect_contact(knob, chassis, name=f"knob_{index}_contacts_rail")
        ctx.expect_gap(
            chassis,
            knob,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="control_rail",
            negative_elem="knob_body",
            name=f"knob_{index}_front_clearance",
        )
        position = ctx.part_world_position(knob)
        assert position is not None
        knob_positions.append(position)

    knob_xs = [position[0] for position in knob_positions]
    knob_zs = [position[2] for position in knob_positions]
    knob_spacings = [knob_xs[index + 1] - knob_xs[index] for index in range(4)]
    ctx.check(
        "knobs_evenly_spaced",
        max(knob_spacings) - min(knob_spacings) < 0.003 and 0.11 <= knob_spacings[0] <= 0.14,
        f"spacings={knob_spacings}",
    )
    ctx.check(
        "knobs_form_shallow_arc",
        (
            abs(knob_zs[0] - knob_zs[4]) < 0.002
            and abs(knob_zs[1] - knob_zs[3]) < 0.002
            and knob_zs[2] > knob_zs[1] > knob_zs[0]
        ),
        f"z positions={knob_zs}",
    )

    with ctx.pose(
        {
            knob_joints[0]: 1.1,
            knob_joints[1]: -0.7,
            knob_joints[2]: 2.2,
            knob_joints[3]: -1.6,
            knob_joints[4]: 0.5,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_pose_no_floating")
        for index, knob in enumerate(knobs, start=1):
            ctx.expect_contact(knob, chassis, name=f"knob_{index}_pose_contact")

    with ctx.pose({door_joint: 0.0}):
        closed_aabb = ctx.part_world_aabb(oven_door)
        assert closed_aabb is not None
        ctx.expect_contact(oven_door, chassis, name="door_closed_hinge_contact")
        ctx.expect_gap(
            chassis,
            oven_door,
            axis="y",
            max_gap=0.003,
            max_penetration=0.0,
            positive_elem="left_front_stile",
            negative_elem="door_panel",
            name="door_closed_front_gap",
        )
        ctx.expect_overlap(
            oven_door,
            chassis,
            axes="x",
            min_overlap=0.64,
            name="door_closed_width_overlap",
        )
        ctx.expect_overlap(
            oven_door,
            chassis,
            axes="z",
            min_overlap=0.60,
            name="door_closed_height_overlap",
        )

    limits = door_joint.motion_limits
    assert limits is not None and limits.lower is not None and limits.upper is not None
    with ctx.pose({door_joint: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_hinge_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="oven_door_hinge_lower_no_floating")

    with ctx.pose({door_joint: limits.upper}):
        open_aabb = ctx.part_world_aabb(oven_door)
        assert open_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_hinge_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="oven_door_hinge_upper_no_floating")
        ctx.check(
            "door_opens_downward",
            open_aabb[1][2] < closed_aabb[1][2] - 0.20 and open_aabb[0][1] < closed_aabb[0][1] - 0.15,
            f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
