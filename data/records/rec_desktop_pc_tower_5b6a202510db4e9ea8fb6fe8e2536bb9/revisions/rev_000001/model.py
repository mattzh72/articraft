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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _perforated_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    hole_w: float,
    hole_h: float,
    pitch_x: float,
    pitch_y: float,
    margin_x: float,
    margin_y: float,
    mesh_name: str,
):
    usable_w = width - (2.0 * margin_x)
    usable_h = height - (2.0 * margin_y)
    cols = max(1, int(usable_w / pitch_x))
    rows = max(1, int(usable_h / pitch_y))
    start_x = -((cols - 1) * pitch_x) * 0.5
    start_y = -((rows - 1) * pitch_y) * 0.5

    holes: list[list[tuple[float, float]]] = []
    for row in range(rows):
        for col in range(cols):
            holes.append(
                [
                    (start_x + (col * pitch_x) - hole_w * 0.5, start_y + (row * pitch_y) - hole_h * 0.5),
                    (start_x + (col * pitch_x) + hole_w * 0.5, start_y + (row * pitch_y) - hole_h * 0.5),
                    (start_x + (col * pitch_x) + hole_w * 0.5, start_y + (row * pitch_y) + hole_h * 0.5),
                    (start_x + (col * pitch_x) - hole_w * 0.5, start_y + (row * pitch_y) + hole_h * 0.5),
                ]
            )

    panel = ExtrudeWithHolesGeometry(
        _rect_profile(width, height),
        holes,
        height=thickness,
        center=True,
    )
    return mesh_from_geometry(panel, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mid_tower_atx_case")

    width = 0.230
    depth = 0.470
    height = 0.470
    steel_t = 0.002

    front_frame_w = 0.018
    front_door_t = 0.014
    glass_t = 0.004

    tray_w = 0.158
    tray_d = 0.180
    tray_runner_w = 0.014
    tray_runner_h = 0.016
    tray_closed_y = 0.110
    tray_z = 0.024
    tray_travel = 0.140

    steel = model.material("steel", rgba=(0.16, 0.17, 0.18, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    mesh_black = model.material("mesh_black", rgba=(0.12, 0.13, 0.14, 0.95))
    smoked_glass = model.material("smoked_glass", rgba=(0.34, 0.40, 0.46, 0.28))
    rail_steel = model.material("rail_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    psu_black = model.material("psu_black", rgba=(0.08, 0.08, 0.09, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    # Main steel body and frame.
    chassis.visual(
        Box((width, depth, steel_t)),
        origin=Origin(xyz=(0.0, 0.0, steel_t * 0.5)),
        material=steel,
        name="floor_pan",
    )
    chassis.visual(
        Box((width, depth, steel_t)),
        origin=Origin(xyz=(0.0, 0.0, height - steel_t * 0.5)),
        material=steel,
        name="roof_pan",
    )
    chassis.visual(
        Box((steel_t, depth, height - (2.0 * steel_t))),
        origin=Origin(xyz=(width * 0.5 - steel_t * 0.5, 0.0, height * 0.5)),
        material=steel,
        name="right_side_panel",
    )
    chassis.visual(
        Box((steel_t, depth - 0.070, height - 0.090)),
        origin=Origin(xyz=(width * 0.5 - (1.5 * steel_t), 0.005, 0.222)),
        material=steel_dark,
        name="motherboard_tray",
    )

    # Front frame around the mesh door opening.
    chassis.visual(
        Box((front_frame_w, steel_t, height - (2.0 * steel_t))),
        origin=Origin(
            xyz=(-width * 0.5 + front_frame_w * 0.5, -depth * 0.5 + steel_t * 0.5, height * 0.5)
        ),
        material=steel,
        name="front_left_post",
    )
    chassis.visual(
        Box((front_frame_w, steel_t, height - (2.0 * steel_t))),
        origin=Origin(
            xyz=(width * 0.5 - front_frame_w * 0.5, -depth * 0.5 + steel_t * 0.5, height * 0.5)
        ),
        material=steel,
        name="front_right_post",
    )
    chassis.visual(
        Box((width - (2.0 * front_frame_w), steel_t, 0.060)),
        origin=Origin(xyz=(0.0, -depth * 0.5 + steel_t * 0.5, height - 0.030)),
        material=steel,
        name="front_top_crossmember",
    )
    chassis.visual(
        Box((width - (2.0 * front_frame_w), steel_t, 0.050)),
        origin=Origin(xyz=(0.0, -depth * 0.5 + steel_t * 0.5, 0.025)),
        material=steel,
        name="front_bottom_crossmember",
    )

    # Rear structure with an open lower center for the PSU tray to slide out.
    chassis.visual(
        Box((0.028, steel_t, height - (2.0 * steel_t))),
        origin=Origin(xyz=(-width * 0.5 + 0.014, depth * 0.5 - steel_t * 0.5, height * 0.5)),
        material=steel,
        name="rear_left_post",
    )
    chassis.visual(
        Box((0.044, steel_t, height - (2.0 * steel_t))),
        origin=Origin(xyz=(width * 0.5 - 0.022, depth * 0.5 - steel_t * 0.5, height * 0.5)),
        material=steel,
        name="rear_right_post",
    )
    chassis.visual(
        Box((width - 0.072, steel_t, 0.080)),
        origin=Origin(xyz=(0.010, depth * 0.5 - steel_t * 0.5, height - 0.040)),
        material=steel,
        name="rear_top_band",
    )
    chassis.visual(
        Box((0.070, steel_t, 0.240)),
        origin=Origin(xyz=(0.034, depth * 0.5 - steel_t * 0.5, 0.250)),
        material=steel_dark,
        name="rear_slot_band",
    )
    chassis.visual(
        Box((0.008, steel_t, 0.240)),
        origin=Origin(xyz=(0.073, depth * 0.5 - steel_t * 0.5, 0.250)),
        material=steel_dark,
        name="rear_slot_bridge",
    )

    # Bottom internal guide rails for the sliding PSU tray.
    guide_center_x = (tray_w * 0.5) - (tray_runner_w * 0.5)
    chassis.visual(
        Box((tray_runner_w, 0.260, tray_runner_h)),
        origin=Origin(xyz=(-guide_center_x, 0.105, tray_runner_h * 0.5)),
        material=rail_steel,
        name="left_psu_guide",
    )
    chassis.visual(
        Box((tray_runner_w, 0.260, tray_runner_h)),
        origin=Origin(xyz=(guide_center_x, 0.105, tray_runner_h * 0.5)),
        material=rail_steel,
        name="right_psu_guide",
    )

    # Feet to read as a floor-standing tower.
    for foot_name, foot_x, foot_y in (
        ("front_left_foot", -0.084, -0.180),
        ("front_right_foot", 0.084, -0.180),
        ("rear_left_foot", -0.084, 0.180),
        ("rear_right_foot", 0.084, 0.180),
    ):
        chassis.visual(
            Box((0.022, 0.032, 0.010)),
            origin=Origin(xyz=(foot_x, foot_y, -0.005)),
            material=rubber,
            name=foot_name,
        )

    # Front door parent-side hinge knuckles.
    door_hinge_r = 0.008
    front_hinge_x = -width * 0.5 - door_hinge_r
    front_hinge_y = -depth * 0.5 - door_hinge_r
    chassis.visual(
        Cylinder(radius=door_hinge_r, length=0.110),
        origin=Origin(xyz=(front_hinge_x, front_hinge_y, 0.382)),
        material=steel_dark,
        name="front_door_hinge_top",
    )
    chassis.visual(
        Cylinder(radius=door_hinge_r, length=0.110),
        origin=Origin(xyz=(front_hinge_x, front_hinge_y, 0.088)),
        material=steel_dark,
        name="front_door_hinge_bottom",
    )
    chassis.visual(
        Box((front_door_t + 0.010, steel_t, 0.045)),
        origin=Origin(xyz=(-width * 0.5 - 0.009, -depth * 0.5 + steel_t * 0.5, 0.382)),
        material=steel,
        name="front_hinge_ear_top",
    )
    chassis.visual(
        Box((front_door_t + 0.010, steel_t, 0.045)),
        origin=Origin(xyz=(-width * 0.5 - 0.009, -depth * 0.5 + steel_t * 0.5, 0.088)),
        material=steel,
        name="front_hinge_ear_bottom",
    )

    # Side-panel parent-side hinge knuckles.
    glass_hinge_r = 0.006
    glass_hinge_x = -width * 0.5 - glass_hinge_r
    glass_hinge_y = depth * 0.5 + glass_t * 0.5
    chassis.visual(
        Cylinder(radius=glass_hinge_r, length=0.110),
        origin=Origin(xyz=(glass_hinge_x, glass_hinge_y, 0.392)),
        material=steel_dark,
        name="glass_hinge_top",
    )
    chassis.visual(
        Cylinder(radius=glass_hinge_r, length=0.110),
        origin=Origin(xyz=(glass_hinge_x, glass_hinge_y, 0.078)),
        material=steel_dark,
        name="glass_hinge_bottom",
    )
    chassis.visual(
        Box((steel_t, glass_t + 0.010, 0.040)),
        origin=Origin(xyz=(-width * 0.5 + steel_t * 0.5, depth * 0.5 + 0.005, 0.392)),
        material=steel,
        name="glass_hinge_mount_top",
    )
    chassis.visual(
        Box((steel_t, glass_t + 0.010, 0.040)),
        origin=Origin(xyz=(-width * 0.5 + steel_t * 0.5, depth * 0.5 + 0.005, 0.078)),
        material=steel,
        name="glass_hinge_mount_bottom",
    )

    # Front mesh door.
    front_door = model.part("front_mesh_door")
    door_w = width + front_door_t
    door_h = height - 0.006
    mesh_w = door_w - (2.0 * front_frame_w)
    mesh_h = door_h - (2.0 * front_frame_w)
    front_mesh = _perforated_panel_mesh(
        width=mesh_w,
        height=mesh_h,
        thickness=0.0022,
        hole_w=0.008,
        hole_h=0.012,
        pitch_x=0.016,
        pitch_y=0.020,
        margin_x=0.010,
        margin_y=0.012,
        mesh_name="front_door_mesh_panel",
    )
    front_door.visual(
        Box((front_frame_w, front_door_t, door_h)),
        origin=Origin(xyz=(door_hinge_r + front_frame_w * 0.5, door_hinge_r - front_door_t * 0.5, 0.0)),
        material=steel,
        name="hinge_stile",
    )
    front_door.visual(
        Box((front_frame_w, front_door_t, door_h)),
        origin=Origin(
            xyz=(door_hinge_r + door_w - front_frame_w * 0.5, door_hinge_r - front_door_t * 0.5, 0.0)
        ),
        material=steel,
        name="latch_stile",
    )
    front_door.visual(
        Box((door_w - (2.0 * front_frame_w), front_door_t, front_frame_w)),
        origin=Origin(
            xyz=(
                door_hinge_r + door_w * 0.5,
                door_hinge_r - front_door_t * 0.5,
                door_h * 0.5 - front_frame_w * 0.5,
            )
        ),
        material=steel,
        name="top_rail",
    )
    front_door.visual(
        Box((door_w - (2.0 * front_frame_w), front_door_t, front_frame_w)),
        origin=Origin(
            xyz=(
                door_hinge_r + door_w * 0.5,
                door_hinge_r - front_door_t * 0.5,
                -door_h * 0.5 + front_frame_w * 0.5,
            )
        ),
        material=steel,
        name="bottom_rail",
    )
    front_door.visual(
        front_mesh,
        origin=Origin(
            xyz=(door_hinge_r + front_frame_w + mesh_w * 0.5, door_hinge_r - front_door_t * 0.5, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=mesh_black,
        name="mesh_panel",
    )
    front_door.visual(
        Box((0.008, 0.010, 0.120)),
        origin=Origin(xyz=(door_hinge_r + door_w - 0.016, -0.005, 0.0)),
        material=steel_dark,
        name="door_handle",
    )
    front_door.visual(
        Cylinder(radius=door_hinge_r, length=0.184),
        origin=Origin(),
        material=steel_dark,
        name="door_center_knuckle",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((door_w, front_door_t + 0.010, door_h)),
        mass=1.5,
        origin=Origin(xyz=(door_hinge_r + door_w * 0.5, door_hinge_r - front_door_t * 0.5, 0.0)),
    )

    # Left tempered-glass side panel.
    glass_panel = model.part("side_glass_panel")
    glass_panel_depth = depth
    glass_panel_height = height - 0.008
    glass_panel.visual(
        Box((glass_t, glass_panel_depth, glass_panel_height)),
        origin=Origin(xyz=(glass_hinge_r + glass_t * 0.5, -glass_panel_depth * 0.5, 0.0)),
        material=smoked_glass,
        name="glass_slab",
    )
    glass_panel.visual(
        Box((0.012, 0.020, glass_panel_height)),
        origin=Origin(xyz=(glass_hinge_r + 0.006, -glass_panel_depth * 0.5, 0.0)),
        material=steel_dark,
        name="rear_mount_strip",
    )
    glass_panel.visual(
        Box((0.018, 0.012, 0.100)),
        origin=Origin(xyz=(glass_hinge_r + glass_t * 0.5, -glass_panel_depth + 0.030, 0.120)),
        material=steel_dark,
        name="glass_pull_tab",
    )
    glass_panel.visual(
        Cylinder(radius=glass_hinge_r, length=0.200),
        origin=Origin(),
        material=steel_dark,
        name="glass_center_knuckle",
    )
    glass_panel.inertial = Inertial.from_geometry(
        Box((0.016, glass_panel_depth, glass_panel_height)),
        mass=1.9,
        origin=Origin(xyz=(glass_hinge_r + 0.004, -glass_panel_depth * 0.5, 0.0)),
    )

    # Sliding PSU tray at the base.
    psu_tray = model.part("power_supply_tray")
    psu_tray.visual(
        Box((tray_w, tray_d, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="tray_floor",
    )
    psu_tray.visual(
        Box((0.002, tray_d, 0.060)),
        origin=Origin(xyz=(-tray_w * 0.5 + 0.001, 0.0, 0.039)),
        material=steel,
        name="tray_left_wall",
    )
    psu_tray.visual(
        Box((0.002, tray_d, 0.060)),
        origin=Origin(xyz=(tray_w * 0.5 - 0.001, 0.0, 0.039)),
        material=steel,
        name="tray_right_wall",
    )
    psu_tray.visual(
        Box((tray_w, 0.002, 0.060)),
        origin=Origin(xyz=(0.0, tray_d * 0.5 - 0.001, 0.039)),
        material=steel,
        name="tray_back_wall",
    )
    psu_tray.visual(
        Box((tray_runner_w, tray_d, tray_runner_h)),
        origin=Origin(xyz=(-guide_center_x, 0.0, 0.0)),
        material=rail_steel,
        name="left_runner",
    )
    psu_tray.visual(
        Box((tray_runner_w, tray_d, tray_runner_h)),
        origin=Origin(xyz=(guide_center_x, 0.0, 0.0)),
        material=rail_steel,
        name="right_runner",
    )
    psu_tray.visual(
        Box((0.150, 0.140, 0.086)),
        origin=Origin(xyz=(0.0, -0.008, 0.052)),
        material=psu_black,
        name="psu_block",
    )
    psu_tray.visual(
        Box((0.050, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, tray_d * 0.5 + 0.002, 0.040)),
        material=steel_dark,
        name="rear_handle",
    )
    psu_tray.inertial = Inertial.from_geometry(
        Box((tray_w, tray_d, 0.102)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
    )

    model.articulation(
        "front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(xyz=(front_hinge_x, front_hinge_y, height * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "side_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=glass_panel,
        origin=Origin(xyz=(glass_hinge_x, glass_hinge_y, height * 0.5)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "psu_tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=psu_tray,
        origin=Origin(xyz=(0.0, tray_closed_y, tray_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.20,
            lower=0.0,
            upper=tray_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    chassis = object_model.get_part("chassis")
    front_door = object_model.get_part("front_mesh_door")
    glass_panel = object_model.get_part("side_glass_panel")
    psu_tray = object_model.get_part("power_supply_tray")

    front_hinge = object_model.get_articulation("front_door_hinge")
    side_hinge = object_model.get_articulation("side_panel_hinge")
    tray_slide = object_model.get_articulation("psu_tray_slide")

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
        "front_door_axis_is_vertical",
        tuple(front_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected +Z axis, got {front_hinge.axis}",
    )
    ctx.check(
        "side_panel_axis_is_vertical",
        tuple(side_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"expected -Z axis, got {side_hinge.axis}",
    )
    ctx.check(
        "psu_tray_axis_is_depthwise",
        tuple(tray_slide.axis) == (0.0, 1.0, 0.0),
        details=f"expected +Y axis, got {tray_slide.axis}",
    )

    front_limits = front_hinge.motion_limits
    side_limits = side_hinge.motion_limits
    tray_limits = tray_slide.motion_limits
    assert front_limits is not None
    assert side_limits is not None
    assert tray_limits is not None

    ctx.check(
        "front_door_limit_realistic",
        front_limits.upper is not None and math.radians(95.0) <= front_limits.upper <= math.radians(120.0),
        details=f"front door upper limit was {front_limits.upper}",
    )
    ctx.check(
        "side_panel_limit_realistic",
        side_limits.upper is not None and math.radians(80.0) <= side_limits.upper <= math.radians(120.0),
        details=f"side panel upper limit was {side_limits.upper}",
    )
    ctx.check(
        "psu_tray_travel_realistic",
        tray_limits.upper is not None and 0.10 <= tray_limits.upper <= 0.18,
        details=f"psu tray upper travel was {tray_limits.upper}",
    )

    with ctx.pose({front_hinge: 0.0, side_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_contact(front_door, chassis, name="front_door_closed_hinge_contact")
        ctx.expect_contact(glass_panel, chassis, name="glass_panel_closed_hinge_contact")
        ctx.expect_contact(psu_tray, chassis, name="psu_tray_closed_guide_contact")
        closed_chassis_aabb = ctx.part_world_aabb(chassis)
        closed_door_mesh_aabb = ctx.part_element_world_aabb(front_door, elem="mesh_panel")
        closed_glass_slab_aabb = ctx.part_element_world_aabb(glass_panel, elem="glass_slab")
        assert closed_chassis_aabb is not None
        assert closed_door_mesh_aabb is not None
        assert closed_glass_slab_aabb is not None
        ctx.check(
            "front_door_closed_front_face_seating",
            closed_chassis_aabb[0][1] <= closed_door_mesh_aabb[1][1] <= closed_chassis_aabb[0][1] + 0.020,
            details=(
                f"expected door mesh to sit just ahead of case front, "
                f"chassis min y={closed_chassis_aabb[0][1]}, "
                f"door mesh max y={closed_door_mesh_aabb[1][1]}"
            ),
        )
        ctx.expect_overlap(
            front_door,
            chassis,
            axes="xz",
            min_overlap=0.18,
            name="front_door_closed_covers_front",
        )
        ctx.check(
            "glass_panel_closed_side_seating",
            closed_chassis_aabb[0][0] <= closed_glass_slab_aabb[1][0] <= closed_chassis_aabb[0][0] + 0.035,
            details=(
                f"expected glass slab to sit just outside left side, "
                f"chassis min x={closed_chassis_aabb[0][0]}, "
                f"glass slab max x={closed_glass_slab_aabb[1][0]}"
            ),
        )
        ctx.expect_overlap(
            glass_panel,
            chassis,
            axes="yz",
            min_overlap=0.38,
            name="glass_panel_closed_covers_side",
        )
        ctx.expect_within(
            psu_tray,
            chassis,
            axes="x",
            margin=0.0,
            name="psu_tray_closed_within_case_width",
        )

    closed_door_aabb = ctx.part_world_aabb(front_door)
    closed_glass_aabb = ctx.part_world_aabb(glass_panel)
    closed_tray_pos = ctx.part_world_position(psu_tray)
    assert closed_door_aabb is not None
    assert closed_glass_aabb is not None
    assert closed_tray_pos is not None

    if front_limits.lower is not None and front_limits.upper is not None:
        with ctx.pose({front_hinge: front_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="front_door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="front_door_lower_no_floating")
        with ctx.pose({front_hinge: front_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="front_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="front_door_upper_no_floating")
            ctx.expect_contact(front_door, chassis, name="front_door_open_hinge_contact")
            open_door_aabb = ctx.part_world_aabb(front_door)
            assert open_door_aabb is not None
            ctx.check(
                "front_door_swings_outward",
                open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.18,
                details=f"closed max y {closed_door_aabb[1][1]}, open max y {open_door_aabb[1][1]}",
            )

    if side_limits.lower is not None and side_limits.upper is not None:
        with ctx.pose({side_hinge: side_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="side_panel_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="side_panel_lower_no_floating")
        with ctx.pose({side_hinge: side_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="side_panel_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="side_panel_upper_no_floating")
            ctx.expect_contact(glass_panel, chassis, name="side_panel_open_hinge_contact")
            open_glass_aabb = ctx.part_world_aabb(glass_panel)
            assert open_glass_aabb is not None
            ctx.check(
                "side_panel_swings_outward",
                open_glass_aabb[0][0] < closed_glass_aabb[0][0] - 0.18,
                details=f"closed min x {closed_glass_aabb[0][0]}, open min x {open_glass_aabb[0][0]}",
            )

    if tray_limits.lower is not None and tray_limits.upper is not None:
        with ctx.pose({tray_slide: tray_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="psu_tray_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="psu_tray_lower_no_floating")
        with ctx.pose({tray_slide: tray_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="psu_tray_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="psu_tray_upper_no_floating")
            ctx.expect_contact(psu_tray, chassis, name="psu_tray_open_guide_contact")
            open_tray_pos = ctx.part_world_position(psu_tray)
            assert open_tray_pos is not None
            ctx.check(
                "psu_tray_slides_to_rear",
                open_tray_pos[1] > closed_tray_pos[1] + 0.12,
                details=f"closed y {closed_tray_pos[1]}, open y {open_tray_pos[1]}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
