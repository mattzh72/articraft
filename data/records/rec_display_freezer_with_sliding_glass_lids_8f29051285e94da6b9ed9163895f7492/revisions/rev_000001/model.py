from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FREEZER_LENGTH = 1.36
FREEZER_WIDTH = 0.82
PLINTH_HEIGHT = 0.07
BODY_HEIGHT = 0.68
BODY_TOP_Z = PLINTH_HEIGHT + BODY_HEIGHT
WALL_THICKNESS = 0.04


def _add_lid(
    model: ArticulatedObject,
    *,
    name: str,
    frame_material: str,
    glass_material: str,
    panel_length: float,
    panel_width: float,
    handle_sign: float,
    add_guide_runners: bool = False,
):
    lid = model.part(name)
    frame_height = 0.014
    frame_width = 0.028

    lid.visual(
        Box((panel_length - 0.050, panel_width - 0.050, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=glass_material,
        name="glass",
    )
    lid.visual(
        Box((panel_length, frame_width, frame_height)),
        origin=Origin(xyz=(0.0, -(panel_width * 0.5 - frame_width * 0.5), frame_height * 0.5)),
        material=frame_material,
        name="front_frame",
    )
    lid.visual(
        Box((panel_length, frame_width, frame_height)),
        origin=Origin(xyz=(0.0, panel_width * 0.5 - frame_width * 0.5, frame_height * 0.5)),
        material=frame_material,
        name="rear_frame",
    )
    lid.visual(
        Box((frame_width, panel_width - 2.0 * frame_width, frame_height)),
        origin=Origin(xyz=(-(panel_length * 0.5 - frame_width * 0.5), 0.0, frame_height * 0.5)),
        material=frame_material,
        name="outer_frame",
    )
    lid.visual(
        Box((frame_width, panel_width - 2.0 * frame_width, frame_height)),
        origin=Origin(xyz=(panel_length * 0.5 - frame_width * 0.5, 0.0, frame_height * 0.5)),
        material=frame_material,
        name="inner_frame",
    )
    lid.visual(
        Box((0.016, panel_width * 0.54, 0.010)),
        origin=Origin(
            xyz=(handle_sign * (panel_length * 0.5 - 0.008), 0.0, 0.010),
        ),
        material=frame_material,
        name="handle",
    )
    if add_guide_runners:
        runner_y = panel_width * 0.5 + 0.022
        for runner_name, runner_sign in (("front_runner", -1.0), ("rear_runner", 1.0)):
            lid.visual(
                Box((panel_length - 0.080, 0.018, 0.004)),
                origin=Origin(xyz=(0.0, runner_sign * runner_y, -0.002)),
                material=frame_material,
                name=runner_name,
            )
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="end_cap_display_freezer")

    shell_white = model.material("shell_white", rgba=(0.93, 0.95, 0.96, 1.0))
    plinth_gray = model.material("plinth_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    liner_white = model.material("liner_white", rgba=(0.96, 0.97, 0.98, 1.0))
    rail_silver = model.material("rail_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    seal_gray = model.material("seal_gray", rgba=(0.38, 0.40, 0.42, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.70, 0.84, 0.90, 0.32))
    panel_gray = model.material("panel_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    panel_black = model.material("panel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.50, 0.52, 0.55, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((FREEZER_LENGTH - 0.08, FREEZER_WIDTH - 0.08, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
        material=plinth_gray,
        name="plinth",
    )
    cabinet.visual(
        Box((FREEZER_LENGTH, FREEZER_WIDTH, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + 0.025)),
        material=shell_white,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((FREEZER_LENGTH, WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -(FREEZER_WIDTH * 0.5 - WALL_THICKNESS * 0.5), PLINTH_HEIGHT + BODY_HEIGHT * 0.5)),
        material=shell_white,
        name="front_wall",
    )
    cabinet.visual(
        Box((FREEZER_LENGTH, WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, FREEZER_WIDTH * 0.5 - WALL_THICKNESS * 0.5, PLINTH_HEIGHT + BODY_HEIGHT * 0.5)),
        material=shell_white,
        name="rear_wall",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, FREEZER_WIDTH - 2.0 * WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(-(FREEZER_LENGTH * 0.5 - WALL_THICKNESS * 0.5), 0.0, PLINTH_HEIGHT + BODY_HEIGHT * 0.5)),
        material=shell_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, FREEZER_WIDTH - 2.0 * WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(FREEZER_LENGTH * 0.5 - WALL_THICKNESS * 0.5, 0.0, PLINTH_HEIGHT + BODY_HEIGHT * 0.5)),
        material=shell_white,
        name="right_wall",
    )

    liner_floor_top = PLINTH_HEIGHT + 0.056
    liner_height = 0.594
    liner_width = FREEZER_WIDTH - 2.0 * 0.055
    liner_length = FREEZER_LENGTH - 2.0 * 0.055
    liner_thickness = 0.006
    cabinet.visual(
        Box((liner_length, liner_width, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, liner_floor_top)),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        Box((liner_length, liner_thickness, liner_height)),
        origin=Origin(xyz=(0.0, -(liner_width * 0.5 - liner_thickness * 0.5), liner_floor_top + liner_height * 0.5)),
        material=liner_white,
        name="liner_front",
    )
    cabinet.visual(
        Box((liner_length, liner_thickness, liner_height)),
        origin=Origin(xyz=(0.0, liner_width * 0.5 - liner_thickness * 0.5, liner_floor_top + liner_height * 0.5)),
        material=liner_white,
        name="liner_rear",
    )
    cabinet.visual(
        Box((liner_thickness, liner_width - 2.0 * liner_thickness, liner_height)),
        origin=Origin(xyz=(-(liner_length * 0.5 - liner_thickness * 0.5), 0.0, liner_floor_top + liner_height * 0.5)),
        material=liner_white,
        name="liner_left",
    )
    cabinet.visual(
        Box((liner_thickness, liner_width - 2.0 * liner_thickness, liner_height)),
        origin=Origin(xyz=(liner_length * 0.5 - liner_thickness * 0.5, 0.0, liner_floor_top + liner_height * 0.5)),
        material=liner_white,
        name="liner_right",
    )

    cabinet.visual(
        Box((FREEZER_LENGTH - 0.06, 0.050, 0.034)),
        origin=Origin(xyz=(0.0, -(FREEZER_WIDTH * 0.5 - 0.025), BODY_TOP_Z + 0.017)),
        material=rail_silver,
        name="front_rail",
    )
    cabinet.visual(
        Box((FREEZER_LENGTH - 0.06, 0.050, 0.034)),
        origin=Origin(xyz=(0.0, FREEZER_WIDTH * 0.5 - 0.025, BODY_TOP_Z + 0.017)),
        material=rail_silver,
        name="rear_rail",
    )
    cabinet.visual(
        Box((FREEZER_LENGTH - 0.08, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -(FREEZER_WIDTH * 0.5 - 0.009), BODY_TOP_Z + 0.043)),
        material=seal_gray,
        name="front_guide",
    )
    cabinet.visual(
        Box((FREEZER_LENGTH - 0.08, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, FREEZER_WIDTH * 0.5 - 0.009, BODY_TOP_Z + 0.043)),
        material=seal_gray,
        name="rear_guide",
    )
    cabinet.visual(
        Box((0.050, FREEZER_WIDTH - 0.10, 0.040)),
        origin=Origin(xyz=(-(FREEZER_LENGTH * 0.5 - 0.025), 0.0, BODY_TOP_Z + 0.020)),
        material=rail_silver,
        name="left_cap",
    )
    cabinet.visual(
        Box((0.050, FREEZER_WIDTH - 0.10, 0.040)),
        origin=Origin(xyz=(FREEZER_LENGTH * 0.5 - 0.025, 0.0, BODY_TOP_Z + 0.020)),
        material=rail_silver,
        name="right_cap",
    )

    lid_0 = _add_lid(
        model,
        name="lid_0",
        frame_material="seal_gray",
        glass_material="glass_tint",
        panel_length=0.72,
        panel_width=FREEZER_WIDTH - 0.036,
        handle_sign=1.0,
    )
    lid_1 = _add_lid(
        model,
        name="lid_1",
        frame_material="seal_gray",
        glass_material="glass_tint",
        panel_length=0.72,
        panel_width=FREEZER_WIDTH - 0.036,
        handle_sign=-1.0,
    )

    model.articulation(
        "cabinet_to_lid_0",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_0,
        origin=Origin(xyz=(-0.310, 0.0, BODY_TOP_Z + 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.34),
    )
    model.articulation(
        "cabinet_to_lid_1",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_1,
        origin=Origin(xyz=(0.310, 0.0, BODY_TOP_Z + 0.052)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.34),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.010, 0.140, 0.120)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=panel_black,
        name="rear_mount",
    )
    control_panel.visual(
        Box((0.032, 0.220, 0.170)),
        origin=Origin(xyz=(0.021, 0.0, 0.0)),
        material=panel_gray,
        name="panel_body",
    )
    control_panel.visual(
        Box((0.006, 0.244, 0.194)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=panel_black,
        name="panel_bezel",
    )
    control_panel.visual(
        Box((0.004, 0.132, 0.050)),
        origin=Origin(xyz=(0.037, 0.0, -0.032)),
        material=metal_dark,
        name="dial_plate",
    )
    control_panel.visual(
        Box((0.004, 0.080, 0.018)),
        origin=Origin(xyz=(0.037, 0.0, 0.060)),
        material=metal_dark,
        name="legend_plate",
    )
    model.articulation(
        "cabinet_to_control_panel",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_panel,
        origin=Origin(xyz=(FREEZER_LENGTH * 0.5, 0.0, 0.520)),
    )

    temperature_knob = model.part("temperature_knob")
    temperature_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.028,
                body_style="skirted",
                top_diameter=0.038,
                skirt=KnobSkirt(0.060, 0.007, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "temperature_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=panel_black,
        name="knob_shell",
    )
    temperature_knob.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_dark,
        name="shaft_collar",
    )
    model.articulation(
        "control_panel_to_temperature_knob",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=temperature_knob,
        origin=Origin(xyz=(0.038, 0.0, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    lock_x = 0.500
    lock_z = 0.446
    hinge_z = 0.500
    cabinet.visual(
        Box((0.042, 0.004, 0.040)),
        origin=Origin(xyz=(lock_x, FREEZER_WIDTH * 0.5 + 0.002, lock_z)),
        material=panel_black,
        name="key_guard",
    )
    cabinet.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(lock_x, FREEZER_WIDTH * 0.5 + 0.008, lock_z, ), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_dark,
        name="key_cylinder",
    )
    cabinet.visual(
        Box((0.014, 0.008, 0.014)),
        origin=Origin(xyz=(lock_x - 0.048, FREEZER_WIDTH * 0.5 + 0.004, hinge_z)),
        material=metal_dark,
        name="hinge_ear_0",
    )
    cabinet.visual(
        Box((0.014, 0.008, 0.014)),
        origin=Origin(xyz=(lock_x + 0.048, FREEZER_WIDTH * 0.5 + 0.004, hinge_z)),
        material=metal_dark,
        name="hinge_ear_1",
    )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(
        Box((0.110, 0.004, 0.070)),
        origin=Origin(xyz=(0.0, -0.002, -0.037)),
        material=shell_white,
        name="cover_panel",
    )
    lock_flap.visual(
        Box((0.090, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.002, -0.005)),
        material=metal_dark,
        name="hinge_leaf",
    )
    lock_flap.visual(
        Cylinder(radius=0.004, length=0.074),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_dark,
        name="hinge_barrel",
    )
    lock_flap.visual(
        Box((0.044, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.001, -0.071)),
        material=metal_dark,
        name="pull_lip",
    )
    model.articulation(
        "cabinet_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lock_flap,
        origin=Origin(xyz=(lock_x, FREEZER_WIDTH * 0.5 + 0.004, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    control_panel = object_model.get_part("control_panel")
    temperature_knob = object_model.get_part("temperature_knob")
    lock_flap = object_model.get_part("lock_flap")
    lid_0_slide = object_model.get_articulation("cabinet_to_lid_0")
    lid_1_slide = object_model.get_articulation("cabinet_to_lid_1")
    knob_spin = object_model.get_articulation("control_panel_to_temperature_knob")
    flap_hinge = object_model.get_articulation("cabinet_to_lock_flap")

    ctx.expect_overlap(lid_0, cabinet, axes="y", min_overlap=0.68, name="lid_0 spans the freezer opening width")
    ctx.expect_overlap(lid_1, cabinet, axes="y", min_overlap=0.68, name="lid_1 spans the freezer opening width")
    ctx.expect_overlap(lid_0, lid_1, axes="y", min_overlap=0.68, name="glass lids share the same top track width")

    lid_0_rest = ctx.part_world_position(lid_0)
    lid_1_rest = ctx.part_world_position(lid_1)
    with ctx.pose({lid_0_slide: 0.34, lid_1_slide: 0.34}):
        lid_0_open = ctx.part_world_position(lid_0)
        lid_1_open = ctx.part_world_position(lid_1)
        ctx.expect_overlap(lid_0, cabinet, axes="y", min_overlap=0.68, name="lid_0 stays on the side rails when opened")
        ctx.expect_overlap(lid_1, cabinet, axes="y", min_overlap=0.68, name="lid_1 stays on the side rails when opened")

    ctx.check(
        "lid_0 slides toward the center",
        lid_0_rest is not None and lid_0_open is not None and lid_0_open[0] > lid_0_rest[0] + 0.20,
        details=f"rest={lid_0_rest}, open={lid_0_open}",
    )
    ctx.check(
        "lid_1 slides toward the center",
        lid_1_rest is not None and lid_1_open is not None and lid_1_open[0] < lid_1_rest[0] - 0.20,
        details=f"rest={lid_1_rest}, open={lid_1_open}",
    )

    ctx.expect_gap(
        control_panel,
        cabinet,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="control panel mounts flush to the end wall",
    )
    ctx.expect_overlap(
        control_panel,
        cabinet,
        axes="yz",
        min_overlap=0.12,
        name="control panel is centered on the freezer end wall",
    )
    ctx.expect_origin_gap(
        temperature_knob,
        control_panel,
        axis="x",
        min_gap=0.030,
        max_gap=0.050,
        name="temperature knob projects forward from the control panel",
    )
    ctx.check(
        "temperature knob uses continuous rotation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.motion_limits is not None
        and knob_spin.motion_limits.lower is None
        and knob_spin.motion_limits.upper is None,
        details=f"type={knob_spin.articulation_type}, limits={knob_spin.motion_limits}",
    )

    ctx.expect_overlap(
        lock_flap,
        cabinet,
        axes="xz",
        elem_a="cover_panel",
        elem_b="key_cylinder",
        min_overlap=0.015,
        name="closed flap covers the key cylinder",
    )
    flap_closed_aabb = ctx.part_element_world_aabb(lock_flap, elem="cover_panel")
    with ctx.pose({flap_hinge: 1.0}):
        flap_open_aabb = ctx.part_element_world_aabb(lock_flap, elem="cover_panel")
    ctx.check(
        "lock flap swings outward on its hinge",
        flap_closed_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[1][1] > flap_closed_aabb[1][1] + 0.04
        and flap_open_aabb[0][2] > flap_closed_aabb[0][2] + 0.015,
        details=f"closed={flap_closed_aabb}, open={flap_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
