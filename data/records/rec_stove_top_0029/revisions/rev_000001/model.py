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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_cooktop_base_cabinet", assets=ASSETS)

    cabinet_paint = model.material("cabinet_paint", rgba=(0.94, 0.94, 0.92, 1.0))
    worktop_stone = model.material("worktop_stone", rgba=(0.58, 0.60, 0.62, 1.0))
    glass_black = model.material("glass_black", rgba=(0.08, 0.08, 0.09, 1.0))
    appliance_metal = model.material("appliance_metal", rgba=(0.40, 0.42, 0.44, 1.0))
    radiant_red = model.material("radiant_red", rgba=(0.55, 0.16, 0.14, 0.42))
    button_grey = model.material("button_grey", rgba=(0.20, 0.21, 0.22, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    cabinet_width = 0.90
    cabinet_depth = 0.58
    cabinet_height = 0.87
    side_thickness = 0.018
    back_thickness = 0.006
    bottom_thickness = 0.018
    top_rail_height = 0.040
    toe_kick_height = 0.100
    front_y = cabinet_depth * 0.5

    worktop_width = 0.92
    worktop_depth = 0.62
    worktop_thickness = 0.030
    cooktop_width = 0.620
    cooktop_depth = 0.520
    cooktop_plate_thickness = 0.006

    hinge_pin_radius = 0.006
    hinge_barrel_outer_radius = 0.010
    door_height = 0.724
    door_bottom = toe_kick_height + 0.003
    door_thickness = 0.020
    center_door_gap = 0.004
    hinge_axis_offset = hinge_barrel_outer_radius
    door_width = (cabinet_width - center_door_gap) / 2.0

    button_size = 0.034
    button_travel = 0.002

    cabinet_frame = model.part("cabinet_frame")
    cabinet_frame.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                -(cabinet_width * 0.5 - side_thickness * 0.5),
                0.0,
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_paint,
        name="left_side",
    )
    cabinet_frame.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                cabinet_width * 0.5 - side_thickness * 0.5,
                0.0,
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_paint,
        name="right_side",
    )
    cabinet_frame.visual(
        Box((cabinet_width - 2.0 * side_thickness, back_thickness, cabinet_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(cabinet_depth * 0.5 - back_thickness * 0.5),
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_paint,
        name="back_panel",
    )
    cabinet_frame.visual(
        Box((cabinet_width - 2.0 * side_thickness, cabinet_depth - back_thickness, bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -back_thickness * 0.5,
                toe_kick_height + bottom_thickness * 0.5,
            )
        ),
        material=cabinet_paint,
        name="bottom_panel",
    )
    cabinet_frame.visual(
        Box((cabinet_width - 2.0 * side_thickness, 0.018, toe_kick_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - 0.040,
                toe_kick_height * 0.5,
            )
        ),
        material=cabinet_paint,
        name="toe_kick",
    )
    cabinet_frame.visual(
        Box((cabinet_width - 2.0 * side_thickness, 0.060, top_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - 0.030,
                cabinet_height - top_rail_height * 0.5,
            )
        ),
        material=cabinet_paint,
        name="top_front_rail",
    )
    cabinet_frame.visual(
        Box((cabinet_width - 2.0 * side_thickness, 0.060, top_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(front_y - 0.030),
                cabinet_height - top_rail_height * 0.5,
            )
        ),
        material=cabinet_paint,
        name="top_rear_rail",
    )
    cabinet_frame.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    def add_hinge_mount(name: str, hinge_x: float) -> None:
        hinge_mount = model.part(name)
        hinge_mount.visual(
            Cylinder(radius=hinge_pin_radius, length=door_height),
            material=appliance_metal,
            name="hinge_pin",
        )
        hinge_mount.visual(
            Box((0.020, 0.024, door_height)),
            origin=Origin(xyz=(0.0, 0.012, door_height * 0.5)),
            material=appliance_metal,
            name="hinge_leaf",
        )
        hinge_mount.inertial = Inertial.from_geometry(
            Box((0.024, 0.028, door_height)),
            mass=0.8,
            origin=Origin(xyz=(0.0, 0.012, door_height * 0.5)),
        )
        model.articulation(
            f"cabinet_to_{name}",
            ArticulationType.FIXED,
            parent=cabinet_frame,
            child=hinge_mount,
            origin=Origin(xyz=(hinge_x, front_y, door_bottom)),
        )

    add_hinge_mount("left_hinge_mount", -(cabinet_width * 0.5 + hinge_axis_offset))
    add_hinge_mount("right_hinge_mount", cabinet_width * 0.5 + hinge_axis_offset)

    worktop = model.part("worktop")
    worktop.visual(
        Box(((worktop_width - cooktop_width) * 0.5, worktop_depth, worktop_thickness)),
        origin=Origin(
            xyz=(
                -(cooktop_width * 0.5 + (worktop_width - cooktop_width) * 0.25),
                0.0,
                worktop_thickness * 0.5,
            )
        ),
        material=worktop_stone,
        name="left_worktop_strip",
    )
    worktop.visual(
        Box(((worktop_width - cooktop_width) * 0.5, worktop_depth, worktop_thickness)),
        origin=Origin(
            xyz=(
                cooktop_width * 0.5 + (worktop_width - cooktop_width) * 0.25,
                0.0,
                worktop_thickness * 0.5,
            )
        ),
        material=worktop_stone,
        name="right_worktop_strip",
    )
    worktop.visual(
        Box((cooktop_width, (worktop_depth - cooktop_depth) * 0.5, worktop_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                cooktop_depth * 0.5 + (worktop_depth - cooktop_depth) * 0.25,
                worktop_thickness * 0.5,
            )
        ),
        material=worktop_stone,
        name="front_worktop_strip",
    )
    worktop.visual(
        Box((cooktop_width, (worktop_depth - cooktop_depth) * 0.5, worktop_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -(cooktop_depth * 0.5 + (worktop_depth - cooktop_depth) * 0.25),
                worktop_thickness * 0.5,
            )
        ),
        material=worktop_stone,
        name="rear_worktop_strip",
    )
    worktop.inertial = Inertial.from_geometry(
        Box((worktop_width, worktop_depth, worktop_thickness)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, worktop_thickness * 0.5)),
    )
    model.articulation(
        "cabinet_to_worktop",
        ArticulationType.FIXED,
        parent=cabinet_frame,
        child=worktop,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height)),
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((0.400, cooktop_depth, cooktop_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cooktop_plate_thickness * 0.5)),
        material=glass_black,
        name="glass_center",
    )

    def add_control_lane(prefix: str, x_center: float) -> None:
        for name, xyz, size in (
            (
                f"{prefix}_outer_strip",
                (x_center - 0.036, 0.0, cooktop_plate_thickness * 0.5),
                (0.038, cooktop_depth, cooktop_plate_thickness),
            ),
            (
                f"{prefix}_inner_strip",
                (x_center + 0.036, 0.0, cooktop_plate_thickness * 0.5),
                (0.038, cooktop_depth, cooktop_plate_thickness),
            ),
            (
                f"{prefix}_top_strip",
                (x_center, 0.181, cooktop_plate_thickness * 0.5),
                (button_size, 0.158, cooktop_plate_thickness),
            ),
            (
                f"{prefix}_middle_strip",
                (x_center, 0.0, cooktop_plate_thickness * 0.5),
                (button_size, 0.136, cooktop_plate_thickness),
            ),
            (
                f"{prefix}_bottom_strip",
                (x_center, -0.181, cooktop_plate_thickness * 0.5),
                (button_size, 0.158, cooktop_plate_thickness),
            ),
        ):
            cooktop.visual(
                Box(size),
                origin=Origin(xyz=xyz),
                material=glass_black,
                name=name,
            )

    add_control_lane("left_lane", -0.255)
    add_control_lane("right_lane", 0.255)

    for heater_name, heater_xyz, heater_size in (
        ("heater_box_front_left", (-0.115, -0.105, -0.030), (0.170, 0.170, 0.060)),
        ("heater_box_rear_left", (-0.115, 0.105, -0.030), (0.170, 0.170, 0.060)),
        ("heater_box_front_right", (0.115, -0.105, -0.030), (0.170, 0.170, 0.060)),
        ("heater_box_rear_right", (0.115, 0.105, -0.030), (0.170, 0.170, 0.060)),
    ):
        cooktop.visual(
            Box(heater_size),
            origin=Origin(xyz=heater_xyz),
            material=appliance_metal,
            name=heater_name,
        )
    cooktop.visual(
        Box((0.160, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.250, -0.003)),
        material=appliance_metal,
        name="front_mount_lip",
    )
    cooktop.visual(
        Box((0.160, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.250, -0.003)),
        material=appliance_metal,
        name="rear_mount_lip",
    )
    cooktop.visual(
        Box((0.024, 0.120, 0.006)),
        origin=Origin(xyz=(-0.198, 0.0, -0.003)),
        material=appliance_metal,
        name="left_support_runner",
    )
    cooktop.visual(
        Box((0.024, 0.120, 0.006)),
        origin=Origin(xyz=(0.198, 0.0, -0.003)),
        material=appliance_metal,
        name="right_support_runner",
    )
    for zone_name, zone_xyz in (
        ("zone_front_left", (-0.115, -0.105, 0.0055)),
        ("zone_rear_left", (-0.115, 0.105, 0.0055)),
        ("zone_front_right", (0.115, -0.105, 0.0055)),
        ("zone_rear_right", (0.115, 0.105, 0.0055)),
    ):
        cooktop.visual(
            Cylinder(radius=0.085, length=0.001),
            origin=Origin(xyz=zone_xyz),
            material=radiant_red,
            name=zone_name,
        )
    cooktop.inertial = Inertial.from_geometry(
        Box((cooktop_width, cooktop_depth, 0.128)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
    )
    model.articulation(
        "worktop_to_cooktop",
        ArticulationType.FIXED,
        parent=worktop,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, worktop_thickness - cooktop_plate_thickness)),
    )

    def add_button(
        name: str,
        x: float,
        y: float,
    ) -> None:
        button = model.part(name)
        button.visual(
            Box((button_size, button_size, cooktop_plate_thickness)),
            origin=Origin(xyz=(0.0, 0.0, cooktop_plate_thickness * 0.5)),
            material=button_grey,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((button_size, button_size, cooktop_plate_thickness)),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, cooktop_plate_thickness * 0.5)),
        )
        model.articulation(
            f"cooktop_to_{name}",
            ArticulationType.PRISMATIC,
            parent=cooktop,
            child=button,
            origin=Origin(xyz=(x, y, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.04,
                lower=0.0,
                upper=button_travel,
            ),
        )

    add_button("button_left_lower", -0.255, -0.085)
    add_button("button_left_upper", -0.255, 0.085)
    add_button("button_right_lower", 0.255, -0.085)
    add_button("button_right_upper", 0.255, 0.085)

    def add_door(name: str, hinge_x: float, x_sign: float, lower: float, upper: float) -> None:
        door = model.part(name)
        door.visual(
            Cylinder(radius=hinge_barrel_outer_radius, length=door_height),
            material=appliance_metal,
            name="hinge_barrel",
        )
        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(
                xyz=(
                    x_sign * (hinge_barrel_outer_radius + door_width * 0.5),
                    door_thickness * 0.5,
                    door_height * 0.5,
                )
            ),
            material=cabinet_paint,
            name="door_slab",
        )
        door.visual(
            Box((0.018, 0.012, 0.180)),
            origin=Origin(
                xyz=(
                    x_sign * (hinge_barrel_outer_radius + door_width - 0.040),
                    door_thickness + 0.006,
                    door_height * 0.5,
                )
            ),
            material=handle_steel,
            name="handle",
        )
        door.inertial = Inertial.from_geometry(
            Box((door_width + 0.020, 0.040, door_height)),
            mass=4.0,
            origin=Origin(
                xyz=(
                    x_sign * ((hinge_barrel_outer_radius + door_width) * 0.5),
                    0.020,
                    door_height * 0.5,
                )
            ),
        )
        model.articulation(
            f"cabinet_to_{name}",
            ArticulationType.REVOLUTE,
            parent=f"{name.replace('_door', '')}_hinge_mount",
            child=door,
            origin=Origin(),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=16.0,
                velocity=1.0,
                lower=lower,
                upper=upper,
            ),
        )

    add_door(
        "left_door",
        -(cabinet_width * 0.5 + hinge_axis_offset),
        1.0,
        0.0,
        math.radians(110.0),
    )
    add_door(
        "right_door",
        cabinet_width * 0.5 + hinge_axis_offset,
        -1.0,
        -math.radians(110.0),
        0.0,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet_frame = object_model.get_part("cabinet_frame")
    worktop = object_model.get_part("worktop")
    cooktop = object_model.get_part("cooktop")
    left_hinge_mount = object_model.get_part("left_hinge_mount")
    right_hinge_mount = object_model.get_part("right_hinge_mount")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    button_left_lower = object_model.get_part("button_left_lower")
    button_left_upper = object_model.get_part("button_left_upper")
    button_right_lower = object_model.get_part("button_right_lower")
    button_right_upper = object_model.get_part("button_right_upper")

    left_door_joint = object_model.get_articulation("cabinet_to_left_door")
    right_door_joint = object_model.get_articulation("cabinet_to_right_door")
    button_joints = [
        object_model.get_articulation("cooktop_to_button_left_lower"),
        object_model.get_articulation("cooktop_to_button_left_upper"),
        object_model.get_articulation("cooktop_to_button_right_lower"),
        object_model.get_articulation("cooktop_to_button_right_upper"),
    ]

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
    ctx.allow_overlap(
        left_hinge_mount,
        left_door,
        reason="Left hinge pin passes through the left door hinge barrel.",
    )
    ctx.allow_overlap(
        right_hinge_mount,
        right_door,
        reason="Right hinge pin passes through the right door hinge barrel.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(worktop, cabinet_frame, name="worktop_supported_by_cabinet")
    ctx.expect_contact(cooktop, worktop, name="cooktop_flush_mounted")
    ctx.expect_contact(left_hinge_mount, cabinet_frame, name="left_hinge_mount_attached")
    ctx.expect_contact(right_hinge_mount, cabinet_frame, name="right_hinge_mount_attached")
    ctx.expect_contact(left_door, left_hinge_mount, name="left_door_hinge_contact")
    ctx.expect_contact(right_door, right_hinge_mount, name="right_door_hinge_contact")
    ctx.expect_contact(button_left_lower, cooktop, name="button_left_lower_guided")
    ctx.expect_contact(button_left_upper, cooktop, name="button_left_upper_guided")
    ctx.expect_contact(button_right_lower, cooktop, name="button_right_lower_guided")
    ctx.expect_contact(button_right_upper, cooktop, name="button_right_upper_guided")

    ctx.expect_gap(worktop, left_door, axis="z", min_gap=0.030, name="left_door_below_worktop")
    ctx.expect_gap(worktop, right_door, axis="z", min_gap=0.030, name="right_door_below_worktop")
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.002,
        max_gap=0.030,
        name="door_center_gap",
    )

    articulated_joints = [left_door_joint, right_door_joint, *button_joints]
    for articulated_joint in articulated_joints:
        limits = articulated_joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulated_joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{articulated_joint.name}_lower_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"{articulated_joint.name}_lower_no_floating")
        with ctx.pose({articulated_joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{articulated_joint.name}_upper_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"{articulated_joint.name}_upper_no_floating")

    button_motion_pairs = [
        ("button_left_lower", button_left_lower, button_joints[0]),
        ("button_left_upper", button_left_upper, button_joints[1]),
        ("button_right_lower", button_right_lower, button_joints[2]),
        ("button_right_upper", button_right_upper, button_joints[3]),
    ]
    for button_name, button_part, button_joint in button_motion_pairs:
        rest_aabb = ctx.part_world_aabb(button_part)
        if rest_aabb is None:
            ctx.fail(f"{button_name}_rest_aabb", "missing rest-pose AABB")
            continue
        limits = button_joint.motion_limits
        assert limits is not None and limits.upper is not None
        with ctx.pose({button_joint: limits.upper}):
            pressed_aabb = ctx.part_world_aabb(button_part)
            if pressed_aabb is None:
                ctx.fail(f"{button_name}_pressed_aabb", "missing pressed-pose AABB")
                continue
            ctx.check(
                f"{button_name}_moves_inward",
                pressed_aabb[1][2] < rest_aabb[1][2] - 0.0015,
                details=(
                    f"expected {button_name} to move down when pressed; "
                    f"rest max z={rest_aabb[1][2]:.4f}, pressed max z={pressed_aabb[1][2]:.4f}"
                ),
            )
            ctx.expect_contact(
                button_part,
                cooktop,
                name=f"{button_name}_pressed_guided",
            )

    left_closed_aabb = ctx.part_world_aabb(left_door)
    right_closed_aabb = ctx.part_world_aabb(right_door)
    if left_closed_aabb is None or right_closed_aabb is None:
        ctx.fail("door_closed_aabbs", "missing closed-door AABB data")
    else:
        left_limits = left_door_joint.motion_limits
        right_limits = right_door_joint.motion_limits
        assert left_limits is not None and left_limits.upper is not None
        assert right_limits is not None and right_limits.lower is not None
        with ctx.pose({left_door_joint: left_limits.upper}):
            left_open_aabb = ctx.part_world_aabb(left_door)
            if left_open_aabb is None:
                ctx.fail("left_door_open_aabb", "missing open AABB")
            else:
                ctx.check(
                    "left_door_swings_outward",
                    left_open_aabb[1][1] > left_closed_aabb[1][1] + 0.12,
                    details=(
                        "left door did not swing outward enough: "
                        f"closed max y={left_closed_aabb[1][1]:.4f}, "
                        f"open max y={left_open_aabb[1][1]:.4f}"
                    ),
                )
                ctx.expect_contact(left_door, left_hinge_mount, name="left_door_open_hinge_contact")
        with ctx.pose({right_door_joint: right_limits.lower}):
            right_open_aabb = ctx.part_world_aabb(right_door)
            if right_open_aabb is None:
                ctx.fail("right_door_open_aabb", "missing open AABB")
            else:
                ctx.check(
                    "right_door_swings_outward",
                    right_open_aabb[1][1] > right_closed_aabb[1][1] + 0.12,
                    details=(
                        "right door did not swing outward enough: "
                        f"closed max y={right_closed_aabb[1][1]:.4f}, "
                        f"open max y={right_open_aabb[1][1]:.4f}"
                    ),
                )
                ctx.expect_contact(
                    right_door,
                    right_hinge_mount,
                    name="right_door_open_hinge_contact",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
