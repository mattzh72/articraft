from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 0.218
BODY_D = 0.560
BODY_H = 0.600
TOP_T = 0.012
BOTTOM_T = 0.012
FRAME_POST_W = 0.018
FRAME_FACE_T = 0.024
PANEL_T = 0.006
DOOR_T = 0.018

TRAY_W = 0.148
TRAY_D = 0.190
TRAY_H = 0.036
TRAY_CENTERS_Z = (0.116, 0.162, 0.208, 0.254)
TRAY_FRONT_Y = -0.245


def _add_drive_sled(model: ArticulatedObject, index: int, *, center_z: float, tray_color, latch_color):
    sled = model.part(f"drive_sled_{index}")
    sled.visual(
        Box((TRAY_W, TRAY_D, TRAY_H)),
        origin=Origin(xyz=(0.0, TRAY_D * 0.5, 0.0)),
        material=tray_color,
        name="tray_body",
    )
    sled.visual(
        Box((TRAY_W + 0.008, 0.008, TRAY_H + 0.004)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=tray_color,
        name="front_bezel",
    )
    sled.visual(
        Box((TRAY_W * 0.72, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.013, 0.0)),
        material=latch_color,
        name="release_handle",
    )
    sled.inertial = Inertial.from_geometry(
        Box((TRAY_W + 0.008, TRAY_D + 0.010, TRAY_H + 0.004)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.090, 0.0)),
    )
    model.articulation(
        f"drive_sled_{index}_rail",
        ArticulationType.PRISMATIC,
        parent="chassis",
        child=sled,
        origin=Origin(xyz=(0.0, TRAY_FRONT_Y, center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.22,
            lower=0.0,
            upper=0.180,
        ),
    )
    return sled


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="server_tower_chassis")

    chassis_paint = model.material("chassis_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    panel_paint = model.material("panel_paint", rgba=(0.23, 0.24, 0.25, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    bay_black = model.material("bay_black", rgba=(0.11, 0.12, 0.13, 1.0))
    drive_metal = model.material("drive_metal", rgba=(0.63, 0.65, 0.68, 1.0))
    latch_blue = model.material("latch_blue", rgba=(0.16, 0.43, 0.75, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((BODY_W, BODY_D, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T * 0.5)),
        material=chassis_paint,
        name="bottom_pan",
    )
    chassis.visual(
        Box((BODY_W, BODY_D, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - TOP_T * 0.5)),
        material=chassis_paint,
        name="top_cover",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        chassis.visual(
            Box((FRAME_POST_W, FRAME_FACE_T, BODY_H)),
            origin=Origin(
                xyz=(
                    sign * (BODY_W * 0.5 - FRAME_POST_W * 0.5),
                    -BODY_D * 0.5 + FRAME_FACE_T * 0.5,
                    BODY_H * 0.5,
                )
            ),
            material=chassis_paint,
            name=f"front_{side_name}_post",
        )
        chassis.visual(
            Box((FRAME_POST_W, FRAME_FACE_T, BODY_H)),
            origin=Origin(
                xyz=(
                    sign * (BODY_W * 0.5 - FRAME_POST_W * 0.5),
                    BODY_D * 0.5 - FRAME_FACE_T * 0.5,
                    BODY_H * 0.5,
                )
            ),
            material=chassis_paint,
            name=f"rear_{side_name}_post",
        )
    chassis.visual(
        Box((BODY_W - 2.0 * FRAME_POST_W, FRAME_FACE_T, 0.030)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + FRAME_FACE_T * 0.5, 0.015)),
        material=chassis_paint,
        name="front_lower_rail",
    )
    chassis.visual(
        Box((BODY_W - 2.0 * FRAME_POST_W, FRAME_FACE_T, 0.030)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + FRAME_FACE_T * 0.5, BODY_H - 0.015)),
        material=chassis_paint,
        name="front_upper_rail",
    )
    chassis.visual(
        Box((BODY_W - 2.0 * FRAME_POST_W, FRAME_FACE_T, 0.040)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - FRAME_FACE_T * 0.5, BODY_H - 0.020)),
        material=chassis_paint,
        name="rear_upper_rail",
    )
    chassis.visual(
        Box((BODY_W - 2.0 * FRAME_POST_W, FRAME_FACE_T, 0.220)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - FRAME_FACE_T * 0.5, 0.130)),
        material=chassis_paint,
        name="rear_lower_wall",
    )
    chassis.visual(
        Box((0.182, 0.020, 0.180)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - 0.010, 0.420)),
        material=trim_black,
        name="rear_fan_wall",
    )
    chassis.visual(
        Box((0.170, 0.210, 0.100)),
        origin=Origin(xyz=(0.0, -0.135, 0.050)),
        material=trim_black,
        name="bay_plinth",
    )
    chassis.visual(
        Box((0.006, 0.185, 0.225)),
        origin=Origin(xyz=(-0.083, -0.1475, 0.2075)),
        material=chassis_paint,
        name="bay_left_rail_stack",
    )
    chassis.visual(
        Box((0.006, 0.185, 0.225)),
        origin=Origin(xyz=(0.083, -0.1475, 0.2075)),
        material=chassis_paint,
        name="bay_right_rail_stack",
    )
    chassis.visual(
        Box((0.170, 0.190, 0.220)),
        origin=Origin(xyz=(0.0, -0.135, 0.410)),
        material=trim_black,
        name="upper_front_module",
    )
    chassis.visual(
        Box((0.160, 0.140, 0.090)),
        origin=Origin(xyz=(0.0, 0.180, 0.057)),
        material=trim_black,
        name="power_supply_block",
    )
    for index, tray_z in enumerate(TRAY_CENTERS_Z):
        chassis.visual(
            Box((0.160, 0.185, 0.002)),
            origin=Origin(xyz=(0.0, -0.1475, tray_z - TRAY_H * 0.5 - 0.001)),
            material=chassis_paint,
            name=f"bay_floor_{index}",
        )
    chassis.inertial = Inertial.from_geometry(
        Box((BODY_W + 0.020, BODY_D + 0.020, BODY_H)),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    left_panel = model.part("left_side_panel")
    left_panel.visual(
        Box((PANEL_T, BODY_D, BODY_H - TOP_T - BOTTOM_T)),
        origin=Origin(xyz=(-PANEL_T * 0.5, -BODY_D * 0.5, BODY_H * 0.5)),
        material=panel_paint,
        name="left_panel_sheet",
    )
    left_panel.visual(
        Box((0.012, 0.120, 0.020)),
        origin=Origin(xyz=(-0.009, -0.365, 0.300)),
        material=trim_black,
        name="left_pull",
    )
    left_panel.inertial = Inertial.from_geometry(
        Box((PANEL_T, BODY_D, BODY_H - TOP_T - BOTTOM_T)),
        mass=1.0,
        origin=Origin(xyz=(-PANEL_T * 0.5, -BODY_D * 0.5, BODY_H * 0.5)),
    )

    right_panel = model.part("right_side_panel")
    right_panel.visual(
        Box((PANEL_T, BODY_D, BODY_H - TOP_T - BOTTOM_T)),
        origin=Origin(xyz=(PANEL_T * 0.5, -BODY_D * 0.5, BODY_H * 0.5)),
        material=panel_paint,
        name="right_panel_sheet",
    )
    right_panel.visual(
        Box((0.012, 0.120, 0.020)),
        origin=Origin(xyz=(0.009, -0.365, 0.300)),
        material=trim_black,
        name="right_pull",
    )
    right_panel.inertial = Inertial.from_geometry(
        Box((PANEL_T, BODY_D, BODY_H - TOP_T - BOTTOM_T)),
        mass=1.0,
        origin=Origin(xyz=(PANEL_T * 0.5, -BODY_D * 0.5, BODY_H * 0.5)),
    )

    door = model.part("front_door")
    door.visual(
        Box((0.020, DOOR_T, BODY_H)),
        origin=Origin(xyz=(0.010, -DOOR_T * 0.5, BODY_H * 0.5)),
        material=panel_paint,
        name="left_stile",
    )
    door.visual(
        Box((0.016, DOOR_T, BODY_H)),
        origin=Origin(xyz=(BODY_W - 0.008, -DOOR_T * 0.5, BODY_H * 0.5)),
        material=panel_paint,
        name="right_stile",
    )
    door.visual(
        Box((BODY_W - 0.020, DOOR_T, 0.030)),
        origin=Origin(xyz=((BODY_W - 0.020) * 0.5 + 0.010, -DOOR_T * 0.5, 0.015)),
        material=panel_paint,
        name="bottom_rail",
    )
    door.visual(
        Box((BODY_W - 0.020, DOOR_T, 0.030)),
        origin=Origin(
            xyz=((BODY_W - 0.020) * 0.5 + 0.010, -DOOR_T * 0.5, BODY_H - 0.015)
        ),
        material=panel_paint,
        name="top_rail",
    )
    door.visual(
        Box((BODY_W - 0.028, 0.004, 0.430)),
        origin=Origin(xyz=(BODY_W * 0.5, -0.011, 0.300)),
        material=bay_black,
        name="mesh_panel",
    )
    door.visual(
        Box((0.020, 0.010, 0.120)),
        origin=Origin(xyz=(BODY_W - 0.013, -0.021, 0.270)),
        material=trim_black,
        name="latch_handle",
    )
    door.visual(
        Box((0.060, 0.004, 0.020)),
        origin=Origin(xyz=(BODY_W * 0.52, -0.011, BODY_H - 0.020)),
        material=latch_blue,
        name="badge_strip",
    )
    door.inertial = Inertial.from_geometry(
        Box((BODY_W, DOOR_T + 0.010, BODY_H)),
        mass=1.5,
        origin=Origin(xyz=(BODY_W * 0.5, -DOOR_T * 0.5, BODY_H * 0.5)),
    )

    cable_bracket = model.part("rear_cable_bracket")
    cable_bracket.visual(
        Box((0.012, 0.012, 0.430)),
        origin=Origin(xyz=(-0.006, 0.006, 0.215)),
        material=drive_metal,
        name="hinge_bar",
    )
    cable_bracket.visual(
        Box((0.012, 0.012, 0.430)),
        origin=Origin(xyz=(-0.110, 0.006, 0.215)),
        material=drive_metal,
        name="outer_bar",
    )
    cable_bracket.visual(
        Box((0.116, 0.012, 0.012)),
        origin=Origin(xyz=(-0.058, 0.006, 0.006)),
        material=drive_metal,
        name="bottom_link",
    )
    cable_bracket.visual(
        Box((0.116, 0.012, 0.012)),
        origin=Origin(xyz=(-0.058, 0.006, 0.424)),
        material=drive_metal,
        name="top_link",
    )
    cable_bracket.visual(
        Box((0.092, 0.012, 0.010)),
        origin=Origin(xyz=(-0.056, 0.006, 0.215)),
        material=drive_metal,
        name="mid_link",
    )
    cable_bracket.inertial = Inertial.from_geometry(
        Box((0.116, 0.012, 0.430)),
        mass=0.30,
        origin=Origin(xyz=(-0.058, 0.006, 0.215)),
    )

    model.articulation(
        "left_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=left_panel,
        origin=Origin(xyz=(-BODY_W * 0.5, BODY_D * 0.5, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=1.80,
        ),
    )
    model.articulation(
        "right_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=right_panel,
        origin=Origin(xyz=(BODY_W * 0.5, BODY_D * 0.5, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=1.80,
        ),
    )
    model.articulation(
        "front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(-BODY_W * 0.5, -BODY_D * 0.5, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.3,
            lower=0.0,
            upper=2.10,
        ),
    )
    model.articulation(
        "rear_bracket_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=cable_bracket,
        origin=Origin(xyz=(BODY_W * 0.5, BODY_D * 0.5, 0.080)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.0,
            lower=0.0,
            upper=1.60,
        ),
    )

    for index, tray_z in enumerate(TRAY_CENTERS_Z):
        _add_drive_sled(
            model,
            index,
            center_z=tray_z,
            tray_color=drive_metal,
            latch_color=latch_blue if index in (0, 2) else trim_black,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    chassis = object_model.get_part("chassis")
    left_panel = object_model.get_part("left_side_panel")
    right_panel = object_model.get_part("right_side_panel")
    door = object_model.get_part("front_door")
    cable_bracket = object_model.get_part("rear_cable_bracket")
    sleds = [object_model.get_part(f"drive_sled_{index}") for index in range(len(TRAY_CENTERS_Z))]

    left_panel_hinge = object_model.get_articulation("left_panel_hinge")
    right_panel_hinge = object_model.get_articulation("right_panel_hinge")
    front_door_hinge = object_model.get_articulation("front_door_hinge")
    rear_bracket_hinge = object_model.get_articulation("rear_bracket_hinge")
    sled_rails = [
        object_model.get_articulation(f"drive_sled_{index}_rail")
        for index in range(len(TRAY_CENTERS_Z))
    ]

    ctx.check("expected_parts_present", True)
    ctx.check("expected_articulations_present", True)

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

    ctx.expect_gap(
        chassis,
        left_panel,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="left_panel_sits_flush_to_left_side",
    )
    ctx.expect_gap(
        right_panel,
        chassis,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="right_panel_sits_flush_to_right_side",
    )
    ctx.expect_gap(
        chassis,
        door,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="front_door_closes_against_front_frame",
    )
    ctx.expect_gap(
        cable_bracket,
        chassis,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear_bracket_folds_flush_to_rear",
    )
    ctx.expect_overlap(
        door,
        chassis,
        axes="xz",
        min_overlap=0.18,
        name="front_door_covers_front_opening",
    )

    for index, sled in enumerate(sleds):
        ctx.expect_contact(
            sled,
            chassis,
            name=f"drive_sled_{index}_supported_by_bay_rails",
        )
        ctx.expect_within(
            sled,
            chassis,
            axes="x",
            margin=0.0,
            name=f"drive_sled_{index}_stays_within_chassis_width",
        )
        ctx.expect_overlap(
            door,
            sled,
            axes="xz",
            min_overlap=0.03,
            name=f"front_door_covers_drive_sled_{index}",
        )

    def _is_vertical_axis(axis: tuple[float, float, float]) -> bool:
        return abs(axis[0]) < 1e-9 and abs(axis[1]) < 1e-9 and abs(abs(axis[2]) - 1.0) < 1e-9

    def _is_y_axis(axis: tuple[float, float, float]) -> bool:
        return abs(axis[0]) < 1e-9 and abs(abs(axis[1]) - 1.0) < 1e-9 and abs(axis[2]) < 1e-9

    ctx.check("left_panel_hinge_axis_vertical", _is_vertical_axis(left_panel_hinge.axis))
    ctx.check("right_panel_hinge_axis_vertical", _is_vertical_axis(right_panel_hinge.axis))
    ctx.check("front_door_hinge_axis_vertical", _is_vertical_axis(front_door_hinge.axis))
    ctx.check("rear_bracket_hinge_axis_vertical", _is_vertical_axis(rear_bracket_hinge.axis))
    ctx.check(
        "drive_sled_rails_translate_fore_aft",
        all(_is_y_axis(rail.axis) for rail in sled_rails),
    )

    door_rest = ctx.part_element_world_aabb(door, elem="latch_handle")
    left_rest = ctx.part_element_world_aabb(left_panel, elem="left_pull")
    right_rest = ctx.part_element_world_aabb(right_panel, elem="right_pull")
    bracket_rest = ctx.part_element_world_aabb(cable_bracket, elem="outer_bar")
    sled_rest = ctx.part_element_world_aabb(sleds[0], elem="front_bezel")

    with ctx.pose({front_door_hinge: 1.55}):
        door_open = ctx.part_element_world_aabb(door, elem="latch_handle")
        ctx.check(
            "front_door_swings_open",
            door_rest is not None
            and door_open is not None
            and door_open[0][1] < door_rest[0][1] - 0.12,
        )

    with ctx.pose({left_panel_hinge: 1.25}):
        left_open = ctx.part_element_world_aabb(left_panel, elem="left_pull")
        ctx.check(
            "left_side_panel_swings_outward",
            left_rest is not None
            and left_open is not None
            and left_open[0][0] < left_rest[0][0] - 0.08,
        )

    with ctx.pose({right_panel_hinge: 1.25}):
        right_open = ctx.part_element_world_aabb(right_panel, elem="right_pull")
        ctx.check(
            "right_side_panel_swings_outward",
            right_rest is not None
            and right_open is not None
            and right_open[1][0] > right_rest[1][0] + 0.08,
        )

    with ctx.pose({rear_bracket_hinge: 1.10}):
        bracket_open = ctx.part_element_world_aabb(cable_bracket, elem="outer_bar")
        ctx.check(
            "rear_cable_bracket_swings_clear",
            bracket_rest is not None
            and bracket_open is not None
            and bracket_open[1][1] > bracket_rest[1][1] + 0.05,
        )

    with ctx.pose({sled_rails[0]: 0.160}):
        sled_open = ctx.part_element_world_aabb(sleds[0], elem="front_bezel")
        ctx.check(
            "drive_sled_extends_forward_on_rails",
            sled_rest is not None
            and sled_open is not None
            and sled_open[0][1] < sled_rest[0][1] - 0.12,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
