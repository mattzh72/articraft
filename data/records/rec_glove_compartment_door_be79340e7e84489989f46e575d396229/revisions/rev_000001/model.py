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


def _rect_profile(
    min_x: float,
    min_z: float,
    max_x: float,
    max_z: float,
) -> list[tuple[float, float]]:
    return [
        (max_x, max_z),
        (max_x, min_z),
        (min_x, min_z),
        (min_x, max_z),
    ]


def _panel_with_holes_mesh(
    name: str,
    *,
    outer: tuple[float, float, float, float],
    holes: list[tuple[float, float, float, float]],
    thickness: float,
):
    outer_profile = _rect_profile(*outer)
    hole_profiles = [_rect_profile(*hole) for hole in holes]
    geometry = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=thickness,
        center=True,
    )
    geometry.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_glove_compartment")

    dashboard_plastic = model.material("dashboard_plastic", rgba=(0.22, 0.23, 0.25, 1.0))
    dashboard_soft = model.material("dashboard_soft", rgba=(0.16, 0.17, 0.19, 1.0))
    bin_plastic = model.material("bin_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    door_plastic = model.material("door_plastic", rgba=(0.19, 0.20, 0.22, 1.0))
    latch_finish = model.material("latch_finish", rgba=(0.70, 0.72, 0.75, 1.0))

    opening_width = 0.364
    opening_height = 0.150
    panel_width = 0.600
    frame_thickness = 0.028
    side_bezel = (panel_width - opening_width) / 2.0
    hinge_axis_y = -0.010
    door_width = 0.356
    door_height = 0.146
    barrel_radius = 0.006
    latch_slot_width = 0.014
    latch_slot_height = 0.008
    latch_slot_center_z = 0.118

    dashboard = model.part("dashboard")
    dashboard.visual(
        Box((side_bezel, frame_thickness, 0.230)),
        origin=Origin(xyz=(-(opening_width + side_bezel) / 2.0, -frame_thickness / 2.0, 0.115)),
        material=dashboard_plastic,
        name="left_frame",
    )
    dashboard.visual(
        Box((side_bezel, frame_thickness, 0.230)),
        origin=Origin(xyz=((opening_width + side_bezel) / 2.0, -frame_thickness / 2.0, 0.115)),
        material=dashboard_plastic,
        name="right_frame",
    )
    dashboard.visual(
        Box((panel_width, frame_thickness, 0.080)),
        origin=Origin(xyz=(0.0, -frame_thickness / 2.0, 0.190)),
        material=dashboard_plastic,
        name="top_frame",
    )
    dashboard.visual(
        Box((0.600, 0.110, 0.030)),
        origin=Origin(xyz=(0.0, -0.083, 0.245)),
        material=dashboard_soft,
        name="dashboard_top_pad",
    )
    dashboard.visual(
        Box((0.034, 0.012, 0.018)),
        origin=Origin(xyz=(-0.150, -0.022, -0.009)),
        material=dashboard_plastic,
        name="left_hinge_support",
    )
    dashboard.visual(
        Box((0.034, 0.012, 0.018)),
        origin=Origin(xyz=(0.150, -0.022, -0.009)),
        material=dashboard_plastic,
        name="right_hinge_support",
    )
    dashboard.visual(
        Box((0.015, 0.012, 0.018)),
        origin=Origin(xyz=(-0.1745, -0.022, -0.009)),
        material=dashboard_plastic,
        name="left_hinge_bridge",
    )
    dashboard.visual(
        Box((0.015, 0.012, 0.018)),
        origin=Origin(xyz=(0.1745, -0.022, -0.009)),
        material=dashboard_plastic,
        name="right_hinge_bridge",
    )
    dashboard.inertial = Inertial.from_geometry(
        Box((0.600, 0.140, 0.290)),
        mass=3.8,
        origin=Origin(xyz=(0.0, -0.070, 0.110)),
    )

    storage_bin = model.part("storage_bin")
    flange_thickness = 0.004
    flange_y = -frame_thickness - flange_thickness / 2.0
    storage_bin.visual(
        Box((0.030, flange_thickness, opening_height + 0.030)),
        origin=Origin(xyz=(-(opening_width + 0.030) / 2.0, flange_y, 0.075)),
        material=bin_plastic,
        name="left_flange",
    )
    storage_bin.visual(
        Box((0.030, flange_thickness, opening_height + 0.030)),
        origin=Origin(xyz=((opening_width + 0.030) / 2.0, flange_y, 0.075)),
        material=bin_plastic,
        name="right_flange",
    )
    storage_bin.visual(
        Box((opening_width + 0.030 * 2.0, flange_thickness, 0.015)),
        origin=Origin(xyz=(0.0, flange_y, opening_height + 0.0075)),
        material=bin_plastic,
        name="top_flange",
    )
    storage_bin.visual(
        Box((opening_width + 0.030 * 2.0, flange_thickness, 0.012)),
        origin=Origin(xyz=(0.0, flange_y, -0.006)),
        material=bin_plastic,
        name="bottom_flange",
    )
    storage_bin.visual(
        Box((0.015, 0.236, 0.180)),
        origin=Origin(xyz=(-0.1895, -0.146, 0.075)),
        material=bin_plastic,
        name="left_wall",
    )
    storage_bin.visual(
        Box((0.015, 0.236, 0.180)),
        origin=Origin(xyz=(0.1895, -0.146, 0.075)),
        material=bin_plastic,
        name="right_wall",
    )
    storage_bin.visual(
        Box((opening_width, 0.236, 0.015)),
        origin=Origin(xyz=(0.0, -0.146, -0.0075)),
        material=bin_plastic,
        name="floor",
    )
    storage_bin.visual(
        Box((opening_width, 0.236, 0.015)),
        origin=Origin(xyz=(0.0, -0.146, 0.1575)),
        material=bin_plastic,
        name="ceiling",
    )
    storage_bin.visual(
        Box((0.394, 0.008, 0.180)),
        origin=Origin(xyz=(0.0, -0.268, 0.075)),
        material=bin_plastic,
        name="back_wall",
    )
    storage_bin.inertial = Inertial.from_geometry(
        Box((0.394, 0.244, 0.180)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.150, 0.075)),
    )

    glovebox_door = model.part("glovebox_door")
    side_face_width = (door_width - latch_slot_width) / 2.0
    lower_face_height = latch_slot_center_z - latch_slot_height / 2.0
    upper_face_height = door_height - (latch_slot_center_z + latch_slot_height / 2.0)
    glovebox_door.visual(
        Box((side_face_width, 0.006, door_height)),
        origin=Origin(
            xyz=(
                -(latch_slot_width + side_face_width) / 2.0,
                -0.003,
                door_height / 2.0,
            )
        ),
        material=door_plastic,
        name="left_face_strip",
    )
    glovebox_door.visual(
        Box((side_face_width, 0.006, door_height)),
        origin=Origin(
            xyz=(
                (latch_slot_width + side_face_width) / 2.0,
                -0.003,
                door_height / 2.0,
            )
        ),
        material=door_plastic,
        name="right_face_strip",
    )
    glovebox_door.visual(
        Box((latch_slot_width, 0.006, lower_face_height)),
        origin=Origin(xyz=(0.0, -0.003, lower_face_height / 2.0)),
        material=door_plastic,
        name="lower_face_strip",
    )
    glovebox_door.visual(
        Box((latch_slot_width, 0.006, upper_face_height)),
        origin=Origin(
            xyz=(
                0.0,
                -0.003,
                latch_slot_center_z + latch_slot_height / 2.0 + upper_face_height / 2.0,
            )
        ),
        material=door_plastic,
        name="upper_face_strip",
    )
    glovebox_door.visual(
        Box((0.028, 0.008, door_height - 0.020)),
        origin=Origin(xyz=(-(door_width / 2.0) + 0.018, -0.010, 0.073)),
        material=door_plastic,
        name="left_rear_rib",
    )
    glovebox_door.visual(
        Box((0.028, 0.008, door_height - 0.020)),
        origin=Origin(xyz=((door_width / 2.0) - 0.018, -0.010, 0.073)),
        material=door_plastic,
        name="right_rear_rib",
    )
    glovebox_door.visual(
        Box((door_width - 0.080, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, 0.009)),
        material=door_plastic,
        name="bottom_rear_rib",
    )
    glovebox_door.visual(
        Box((door_width - 0.080, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, door_height - 0.009)),
        material=door_plastic,
        name="top_rear_rib",
    )
    glovebox_door.visual(
        Cylinder(radius=barrel_radius, length=0.300),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=door_plastic,
        name="door_hinge_barrel",
    )
    glovebox_door.inertial = Inertial.from_geometry(
        Box((0.356, 0.022, 0.150)),
        mass=1.0,
        origin=Origin(xyz=(0.0, -0.014, 0.073)),
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        Box((0.020, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=latch_finish,
        name="button_cap",
    )
    latch_button.visual(
        Box((0.008, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material=latch_finish,
        name="button_plunger",
    )
    latch_button.inertial = Inertial.from_geometry(
        Box((0.014, 0.014, 0.008)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
    )

    model.articulation(
        "dashboard_to_storage_bin",
        ArticulationType.FIXED,
        parent=dashboard,
        child=storage_bin,
        origin=Origin(),
    )
    model.articulation(
        "dashboard_to_glovebox_door",
        ArticulationType.REVOLUTE,
        parent=dashboard,
        child=glovebox_door,
        origin=Origin(xyz=(0.0, hinge_axis_y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.50,
        ),
    )
    model.articulation(
        "glovebox_door_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=glovebox_door,
        child=latch_button,
        origin=Origin(xyz=(0.0, 0.0, latch_slot_center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    opening_width = 0.364
    dashboard = object_model.get_part("dashboard")
    storage_bin = object_model.get_part("storage_bin")
    glovebox_door = object_model.get_part("glovebox_door")
    latch_button = object_model.get_part("latch_button")
    door_hinge = object_model.get_articulation("dashboard_to_glovebox_door")
    latch_slide = object_model.get_articulation("glovebox_door_to_latch_button")

    ctx.allow_isolated_part(
        latch_button,
        reason="Latch plunger uses internal clearance guidance inside the door and may not maintain visible mesh contact at every sampled pose.",
    )

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
        "door_hinge_axis",
        tuple(door_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"expected door hinge axis (-1, 0, 0), got {door_hinge.axis}",
    )
    ctx.check(
        "latch_axis",
        tuple(latch_slide.axis) == (0.0, -1.0, 0.0),
        details=f"expected latch axis (0, -1, 0), got {latch_slide.axis}",
    )

    ctx.expect_contact(
        storage_bin,
        dashboard,
        name="bin_flange_contacts_dashboard",
    )
    ctx.expect_contact(
        glovebox_door,
        dashboard,
        elem_a="door_hinge_barrel",
        elem_b="left_hinge_support",
        name="left_hinge_knuckle_contacts_door",
    )
    ctx.expect_contact(
        glovebox_door,
        dashboard,
        elem_a="door_hinge_barrel",
        elem_b="right_hinge_support",
        name="right_hinge_knuckle_contacts_door",
    )

    left_face = ctx.part_element_world_aabb(glovebox_door, elem="left_face_strip")
    right_face = ctx.part_element_world_aabb(glovebox_door, elem="right_face_strip")
    left_frame = ctx.part_element_world_aabb(dashboard, elem="left_frame")
    closed_button_cap = ctx.part_element_world_aabb(latch_button, elem="button_cap")
    closed_button_plunger = ctx.part_element_world_aabb(latch_button, elem="button_plunger")
    bin_aabb = ctx.part_world_aabb(storage_bin)

    if left_face is not None and right_face is not None and left_frame is not None:
        door_face_max_y = max(left_face[1][1], right_face[1][1])
        door_x_min = min(left_face[0][0], right_face[0][0])
        door_x_max = max(left_face[1][0], right_face[1][0])
        front_offset = door_face_max_y - left_frame[1][1]
        ctx.check(
            "door_closed_flush_with_dashboard",
            -0.012 <= front_offset <= -0.008,
            details=f"door face should sit 8-12 mm behind the dashboard frame, got {front_offset:.4f} m",
        )
        total_side_slack = opening_width - (door_x_max - door_x_min)
        ctx.check(
            "door_fills_opening_width",
            0.006 <= total_side_slack <= 0.012,
            details=f"door total side slack should be 6-12 mm, got {total_side_slack:.4f} m",
        )
    if closed_button_cap is not None and left_face is not None:
        button_proud = closed_button_cap[1][1] - left_face[1][1]
        ctx.check(
            "button_proud_when_unpressed",
            0.003 <= button_proud <= 0.006,
            details=f"button should stand proud by 3-6 mm, got {button_proud:.4f} m",
        )
    if closed_button_plunger is not None and left_face is not None:
        plunger_depth_behind_face = left_face[1][1] - closed_button_plunger[0][1]
        ctx.check(
            "button_plunger_sits_in_door_pocket",
            0.008 <= plunger_depth_behind_face <= 0.012,
            details=f"button plunger should sit just behind the door face, got depth {plunger_depth_behind_face:.4f} m",
        )
    if bin_aabb is not None:
        bin_depth = bin_aabb[1][1] - bin_aabb[0][1]
        ctx.check(
            "storage_bin_has_real_depth",
            bin_depth >= 0.23,
            details=f"storage bin depth should be at least 0.23 m, got {bin_depth:.4f} m",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="door_lower_no_floating")
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="door_upper_no_floating")
            open_door_panel = ctx.part_world_aabb(glovebox_door)
            if open_door_panel is not None:
                ctx.check(
                    "door_swings_outward",
                    open_door_panel[1][1] >= 0.13,
                    details=f"open door should project well into the cabin, max y was {open_door_panel[1][1]:.4f} m",
                )
                ctx.check(
                    "door_drops_below_opening_when_open",
                    open_door_panel[1][2] <= 0.030,
                    details=f"open door top should sit near the hinge plane, max z was {open_door_panel[1][2]:.4f} m",
                )

    latch_limits = latch_slide.motion_limits
    if latch_limits is not None and latch_limits.lower is not None and latch_limits.upper is not None:
        with ctx.pose({latch_slide: latch_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="latch_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="latch_lower_no_floating")
        with ctx.pose({latch_slide: latch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="latch_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="latch_upper_no_floating")
            pressed_button_cap = ctx.part_element_world_aabb(latch_button, elem="button_cap")
            if pressed_button_cap is not None and closed_button_cap is not None and left_face is not None:
                travel = closed_button_cap[1][1] - pressed_button_cap[1][1]
                proud_after_press = pressed_button_cap[1][1] - left_face[1][1]
                ctx.check(
                    "button_has_short_travel",
                    0.0035 <= travel <= 0.0045,
                    details=f"button travel should be about 4 mm, got {travel:.4f} m",
                )
                ctx.check(
                    "button_retracts_into_door",
                    proud_after_press <= 0.0005,
                    details=f"pressed button should be flush or recessed, proud amount was {proud_after_press:.4f} m",
                )
        with ctx.pose({door_hinge: door_hinge.motion_limits.upper, latch_slide: latch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_latch_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name="door_open_latch_pressed_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
