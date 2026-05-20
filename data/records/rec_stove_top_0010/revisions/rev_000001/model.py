from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


CABINET_WIDTH = 0.96
CABINET_DEPTH = 0.58
CABINET_HEIGHT = 0.72
PANEL_THICKNESS = 0.018

COUNTER_WIDTH = 1.02
COUNTER_DEPTH = 0.63
COUNTER_THICKNESS = 0.03
COUNTER_CENTER_Z = CABINET_HEIGHT + (COUNTER_THICKNESS / 2.0)

CUTOUT_WIDTH = 0.892
CUTOUT_DEPTH = 0.524

COOKTOP_BODY_WIDTH = 0.84
COOKTOP_BODY_DEPTH = 0.47
COOKTOP_BODY_HEIGHT = 0.104
COOKTOP_BODY_CENTER_Z = 0.692

GLASS_WIDTH = 0.888
GLASS_DEPTH = 0.52
GLASS_THICKNESS = 0.006
GLASS_CENTER_Z = CABINET_HEIGHT + COUNTER_THICKNESS - (GLASS_THICKNESS / 2.0)

CONTROL_PANEL_THICKNESS = 0.008
CONTROL_PANEL_CENTER_Y = -(COOKTOP_BODY_DEPTH / 2.0) + (CONTROL_PANEL_THICKNESS / 2.0)
CONTROL_PANEL_BACK_FACE_Y = CONTROL_PANEL_CENTER_Y + (CONTROL_PANEL_THICKNESS / 2.0)

BUTTON_TRAVEL = 0.004

BUTTON_XS = (-0.18, -0.09, 0.09, 0.18)
CONTROL_Z = 0.0
BUTTON_OPENING = 0.03
KNOB_OPENING = 0.05
CONTROL_OPENING_HEIGHT = 0.054


def _write_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _button_cap_mesh():
    cap_profile = rounded_rect_profile(0.036, 0.036, 0.007, corner_segments=8)
    cap_geometry = ExtrudeGeometry(cap_profile, 0.006, center=True, cap=True, closed=True)
    cap_geometry.rotate_x(-pi / 2.0)
    return _write_mesh("cooktop_button_cap.obj", cap_geometry)


def _knob_shell_mesh():
    knob_profile = [
        (0.0, 0.0),
        (0.010, 0.0),
        (0.014, 0.003),
        (0.024, 0.006),
        (0.031, 0.011),
        (0.034, 0.017),
        (0.034, 0.024),
        (0.032, 0.028),
        (0.026, 0.032),
        (0.0, 0.032),
    ]
    knob_geometry = LatheGeometry(knob_profile, segments=56)
    knob_geometry.rotate_x(-pi / 2.0)
    return _write_mesh("cooktop_center_knob.obj", knob_geometry)


def _add_zone_marker(part, name: str, *, xy: tuple[float, float], radius: float, material: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=0.0006),
        origin=Origin(xyz=(xy[0], xy[1], (GLASS_THICKNESS / 2.0) - 0.0003)),
        material=material,
        name=name,
    )


def _add_button_geometry(part, material: str) -> None:
    part.visual(
        _button_cap_mesh(),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=material,
        name="cap",
    )
    part.visual(
        Box((0.018, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=material,
        name="shaft",
    )
    part.visual(
        Box((0.036, 0.004, 0.036)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=material,
        name="retainer",
    )


def _add_knob_geometry(part, knob_material: str, marker_material: str) -> None:
    part.visual(
        _knob_shell_mesh(),
        origin=Origin(xyz=(0.0, -0.039, 0.0)),
        material=knob_material,
        name="knob_body",
    )
    part.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="knob_collar",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="knob_shaft",
    )
    part.visual(
        Box((0.005, 0.0025, 0.018)),
        origin=Origin(xyz=(0.0, -0.0385, 0.017)),
        material=marker_material,
        name="pointer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_induction_cooktop", assets=ASSETS)

    cabinet_finish = model.material("cabinet_finish", color=(0.93, 0.94, 0.95))
    counter_finish = model.material("counter_finish", color=(0.78, 0.80, 0.82))
    cooktop_metal = model.material("cooktop_metal", color=(0.19, 0.20, 0.21))
    glass_finish = model.material("glass_finish", rgba=(0.05, 0.05, 0.06, 0.92))
    zone_finish = model.material("zone_finish", rgba=(0.78, 0.80, 0.84, 0.25))
    control_finish = model.material("control_finish", color=(0.14, 0.14, 0.15))
    knob_finish = model.material("knob_finish", color=(0.74, 0.76, 0.78))
    marker_finish = model.material("marker_finish", color=(0.11, 0.11, 0.12))

    cabinet = model.part("cabinet_opening")
    cabinet.visual(
        Box((PANEL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(-(CABINET_WIDTH / 2.0) + (PANEL_THICKNESS / 2.0), 0.0, CABINET_HEIGHT / 2.0)
        ),
        material=cabinet_finish,
        name="left_side",
    )
    cabinet.visual(
        Box((PANEL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=((CABINET_WIDTH / 2.0) - (PANEL_THICKNESS / 2.0), 0.0, CABINET_HEIGHT / 2.0)
        ),
        material=cabinet_finish,
        name="right_side",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - (2.0 * PANEL_THICKNESS), CABINET_DEPTH - PANEL_THICKNESS, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(PANEL_THICKNESS / 2.0),
                PANEL_THICKNESS / 2.0,
            )
        ),
        material=cabinet_finish,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - (2.0 * PANEL_THICKNESS), PANEL_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (CABINET_DEPTH / 2.0) - (PANEL_THICKNESS / 2.0),
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=cabinet_finish,
        name="back_panel",
    )

    counter = model.part("countertop")
    counter.visual(
        Box((((COUNTER_WIDTH - CUTOUT_WIDTH) / 2.0), COUNTER_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                -((CUTOUT_WIDTH / 2.0) + ((COUNTER_WIDTH - CUTOUT_WIDTH) / 4.0)),
                0.0,
                0.0,
            )
        ),
        material=counter_finish,
        name="left_strip",
    )
    counter.visual(
        Box((((COUNTER_WIDTH - CUTOUT_WIDTH) / 2.0), COUNTER_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                (CUTOUT_WIDTH / 2.0) + ((COUNTER_WIDTH - CUTOUT_WIDTH) / 4.0),
                0.0,
                0.0,
            )
        ),
        material=counter_finish,
        name="right_strip",
    )
    counter.visual(
        Box((CUTOUT_WIDTH, ((COUNTER_DEPTH - CUTOUT_DEPTH) / 2.0), COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -((CUTOUT_DEPTH / 2.0) + ((COUNTER_DEPTH - CUTOUT_DEPTH) / 4.0)),
                0.0,
            )
        ),
        material=counter_finish,
        name="front_strip",
    )
    counter.visual(
        Box((CUTOUT_WIDTH, ((COUNTER_DEPTH - CUTOUT_DEPTH) / 2.0), COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                (CUTOUT_DEPTH / 2.0) + ((COUNTER_DEPTH - CUTOUT_DEPTH) / 4.0),
                0.0,
            )
        ),
        material=counter_finish,
        name="back_strip",
    )

    body = model.part("cooktop_body")
    body.visual(
        Box((0.01, COOKTOP_BODY_DEPTH, COOKTOP_BODY_HEIGHT)),
        origin=Origin(xyz=(-(COOKTOP_BODY_WIDTH / 2.0) + 0.005, 0.0, 0.0)),
        material=cooktop_metal,
        name="left_wall",
    )
    body.visual(
        Box((0.01, COOKTOP_BODY_DEPTH, COOKTOP_BODY_HEIGHT)),
        origin=Origin(xyz=((COOKTOP_BODY_WIDTH / 2.0) - 0.005, 0.0, 0.0)),
        material=cooktop_metal,
        name="right_wall",
    )
    body.visual(
        Box((COOKTOP_BODY_WIDTH - 0.02, 0.01, COOKTOP_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, (COOKTOP_BODY_DEPTH / 2.0) - 0.005, 0.0)),
        material=cooktop_metal,
        name="back_wall",
    )
    body.visual(
        Box((COOKTOP_BODY_WIDTH - 0.02, COOKTOP_BODY_DEPTH - 0.01, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, -(COOKTOP_BODY_HEIGHT / 2.0) + 0.005)),
        material=cooktop_metal,
        name="bottom_pan",
    )
    body.visual(
        Box((0.026, CUTOUT_DEPTH, 0.008)),
        origin=Origin(xyz=(-(CUTOUT_WIDTH / 2.0) + 0.013, 0.0, 0.024)),
        material=cooktop_metal,
        name="left_mount_flange",
    )
    body.visual(
        Box((0.026, CUTOUT_DEPTH, 0.008)),
        origin=Origin(xyz=((CUTOUT_WIDTH / 2.0) - 0.013, 0.0, 0.024)),
        material=cooktop_metal,
        name="right_mount_flange",
    )
    body.visual(
        Box((0.17, 0.027, 0.008)),
        origin=Origin(xyz=(-0.295, -((CUTOUT_DEPTH / 2.0) - 0.0135), 0.024)),
        material=cooktop_metal,
        name="front_left_mount_flange",
    )
    body.visual(
        Box((0.17, 0.027, 0.008)),
        origin=Origin(xyz=(0.295, -((CUTOUT_DEPTH / 2.0) - 0.0135), 0.024)),
        material=cooktop_metal,
        name="front_right_mount_flange",
    )
    body.visual(
        Box((COOKTOP_BODY_WIDTH, 0.027, 0.008)),
        origin=Origin(xyz=(0.0, (CUTOUT_DEPTH / 2.0) - 0.0135, 0.024)),
        material=cooktop_metal,
        name="back_mount_flange",
    )
    body.visual(
        Box((COOKTOP_BODY_WIDTH - 0.02, CONTROL_PANEL_THICKNESS, 0.01)),
        origin=Origin(xyz=(0.0, CONTROL_PANEL_CENTER_Y, 0.032)),
        material=cooktop_metal,
        name="control_top_rail",
    )
    body.visual(
        Box((COOKTOP_BODY_WIDTH - 0.02, CONTROL_PANEL_THICKNESS, 0.01)),
        origin=Origin(xyz=(0.0, CONTROL_PANEL_CENTER_Y, -0.032)),
        material=cooktop_metal,
        name="control_bottom_rail",
    )

    panel_bars = (
        ("bar_left_outer", -0.3025, 0.215),
        ("bar_left_inner", -0.135, 0.06),
        ("bar_left_center", -0.05, 0.05),
        ("bar_right_center", 0.05, 0.05),
        ("bar_right_inner", 0.135, 0.06),
        ("bar_right_outer", 0.3025, 0.215),
    )
    for bar_name, x_center, width in panel_bars:
        body.visual(
            Box((width, CONTROL_PANEL_THICKNESS, CONTROL_OPENING_HEIGHT)),
            origin=Origin(xyz=(x_center, CONTROL_PANEL_CENTER_Y, 0.0)),
            material=cooktop_metal,
            name=bar_name,
        )

    glass = model.part("glass_top")
    glass.visual(
        Box((GLASS_WIDTH, GLASS_DEPTH, GLASS_THICKNESS)),
        material=glass_finish,
        name="glass_slab",
    )
    for zone_name, xy, radius in (
        ("zone_back_left", (-0.22, 0.11), 0.095),
        ("zone_back_center", (0.0, 0.13), 0.112),
        ("zone_back_right", (0.22, 0.11), 0.095),
        ("zone_front_left", (-0.14, -0.105), 0.086),
        ("zone_front_right", (0.16, -0.11), 0.086),
    ):
        _add_zone_marker(glass, zone_name, xy=xy, radius=radius, material=zone_finish)

    left_outer_button = model.part("left_outer_button")
    _add_button_geometry(left_outer_button, control_finish)
    left_inner_button = model.part("left_inner_button")
    _add_button_geometry(left_inner_button, control_finish)
    center_knob = model.part("center_knob")
    _add_knob_geometry(center_knob, knob_finish, marker_finish)
    right_inner_button = model.part("right_inner_button")
    _add_button_geometry(right_inner_button, control_finish)
    right_outer_button = model.part("right_outer_button")
    _add_button_geometry(right_outer_button, control_finish)

    model.articulation(
        "cabinet_to_counter",
        ArticulationType.FIXED,
        parent=cabinet,
        child=counter,
        origin=Origin(xyz=(0.0, 0.0, COUNTER_CENTER_Z)),
    )
    model.articulation(
        "counter_to_cooktop_body",
        ArticulationType.FIXED,
        parent=counter,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, COOKTOP_BODY_CENTER_Z - COUNTER_CENTER_Z)),
    )
    model.articulation(
        "body_to_glass",
        ArticulationType.FIXED,
        parent=body,
        child=glass,
        origin=Origin(xyz=(0.0, 0.0, GLASS_CENTER_Z - COOKTOP_BODY_CENTER_Z)),
    )

    button_joint_limits = MotionLimits(
        effort=18.0,
        velocity=0.08,
        lower=0.0,
        upper=BUTTON_TRAVEL,
    )

    model.articulation(
        "body_to_left_outer_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_outer_button,
        origin=Origin(xyz=(BUTTON_XS[0], CONTROL_PANEL_BACK_FACE_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=button_joint_limits,
    )
    model.articulation(
        "body_to_left_inner_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_inner_button,
        origin=Origin(xyz=(BUTTON_XS[1], CONTROL_PANEL_BACK_FACE_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=button_joint_limits,
    )
    model.articulation(
        "body_to_center_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=center_knob,
        origin=Origin(xyz=(0.0, CONTROL_PANEL_BACK_FACE_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=6.0),
    )
    model.articulation(
        "body_to_right_inner_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_inner_button,
        origin=Origin(xyz=(BUTTON_XS[2], CONTROL_PANEL_BACK_FACE_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=button_joint_limits,
    )
    model.articulation(
        "body_to_right_outer_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_outer_button,
        origin=Origin(xyz=(BUTTON_XS[3], CONTROL_PANEL_BACK_FACE_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=button_joint_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet_opening")
    counter = object_model.get_part("countertop")
    body = object_model.get_part("cooktop_body")
    glass = object_model.get_part("glass_top")
    left_outer_button = object_model.get_part("left_outer_button")
    left_inner_button = object_model.get_part("left_inner_button")
    center_knob = object_model.get_part("center_knob")
    right_inner_button = object_model.get_part("right_inner_button")
    right_outer_button = object_model.get_part("right_outer_button")

    left_outer_joint = object_model.get_articulation("body_to_left_outer_button")
    left_inner_joint = object_model.get_articulation("body_to_left_inner_button")
    knob_joint = object_model.get_articulation("body_to_center_knob")
    right_inner_joint = object_model.get_articulation("body_to_right_inner_button")
    right_outer_joint = object_model.get_articulation("body_to_right_outer_button")

    button_parts = (
        (left_outer_button, left_outer_joint),
        (left_inner_button, left_inner_joint),
        (right_inner_button, right_inner_joint),
        (right_outer_button, right_outer_joint),
    )

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

    ctx.expect_contact(counter, cabinet, name="counter_supported_by_cabinet")
    ctx.expect_contact(body, counter, name="cooktop_body_hung_from_counter")
    ctx.expect_contact(glass, body, name="glass_supported_by_body")

    glass_aabb = ctx.part_element_world_aabb(glass, elem="glass_slab")
    counter_aabb = ctx.part_world_aabb(counter)
    flush_ok = (
        glass_aabb is not None
        and counter_aabb is not None
        and abs(glass_aabb[1][2] - counter_aabb[1][2]) <= 0.0005
    )
    ctx.check(
        "glass_flush_with_countertop",
        flush_ok,
        details=f"glass top z={glass_aabb[1][2] if glass_aabb else None}, counter top z={counter_aabb[1][2] if counter_aabb else None}",
    )

    ctx.expect_gap(
        glass,
        counter,
        axis="x",
        min_gap=0.001,
        max_gap=0.0035,
        negative_elem="left_strip",
        name="left_glass_seam",
    )
    ctx.expect_gap(
        counter,
        glass,
        axis="x",
        min_gap=0.001,
        max_gap=0.0035,
        positive_elem="right_strip",
        name="right_glass_seam",
    )
    ctx.expect_gap(
        glass,
        counter,
        axis="y",
        min_gap=0.001,
        max_gap=0.0035,
        negative_elem="front_strip",
        name="front_glass_seam",
    )
    ctx.expect_gap(
        counter,
        glass,
        axis="y",
        min_gap=0.001,
        max_gap=0.0035,
        positive_elem="back_strip",
        name="rear_glass_seam",
    )

    control_positions = [
        ctx.part_world_position(left_outer_button),
        ctx.part_world_position(left_inner_button),
        ctx.part_world_position(center_knob),
        ctx.part_world_position(right_inner_button),
        ctx.part_world_position(right_outer_button),
    ]
    control_xs = [pos[0] if pos is not None else None for pos in control_positions]
    ordered_controls = all(
        control_xs[idx] is not None and control_xs[idx + 1] is not None and control_xs[idx] < control_xs[idx + 1]
        for idx in range(len(control_xs) - 1)
    )
    centered_knob = control_xs[2] is not None and abs(control_xs[2]) <= 0.005
    ctx.check(
        "controls_line_up_left_to_right",
        ordered_controls and centered_knob,
        details=f"control x positions={control_xs}",
    )

    zone_visuals = [visual for visual in glass.visuals if visual.name and visual.name.startswith("zone_")]
    ctx.check(
        "five_induction_zones_present",
        len(zone_visuals) == 5,
        details=f"zone visuals={[visual.name for visual in zone_visuals]}",
    )

    moving_joints = [
        articulation
        for articulation in object_model.articulations
        if articulation.articulation_type != ArticulationType.FIXED
    ]
    continuous_count = sum(
        articulation.articulation_type == ArticulationType.CONTINUOUS for articulation in moving_joints
    )
    prismatic_count = sum(
        articulation.articulation_type == ArticulationType.PRISMATIC for articulation in moving_joints
    )
    ctx.check(
        "only_knob_and_four_buttons_move",
        len(moving_joints) == 5 and continuous_count == 1 and prismatic_count == 4,
        details=f"moving joints={[(joint.name, str(joint.articulation_type)) for joint in moving_joints]}",
    )

    axis_ok = tuple(knob_joint.axis) == (0.0, 1.0, 0.0)
    knob_limits = knob_joint.motion_limits
    knob_limits_ok = (
        knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None
        and knob_limits.velocity > 0.0
        and knob_limits.effort > 0.0
    )
    ctx.check(
        "knob_is_continuous_y_axis",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS and axis_ok and knob_limits_ok,
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}, limits={knob_limits}",
    )

    for button_part, button_joint in button_parts:
        limits = button_joint.motion_limits
        button_ok = (
            button_joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(button_joint.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and abs(limits.upper - BUTTON_TRAVEL) <= 1e-9
        )
        ctx.check(
            f"{button_joint.name}_prismatic_inward",
            button_ok,
            details=f"type={button_joint.articulation_type}, axis={button_joint.axis}, limits={limits}",
        )

        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({button_joint: limits.lower}):
                ctx.expect_contact(
                    button_part,
                    body,
                    name=f"{button_joint.name}_rest_contact",
                )
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_joint.name}_rest_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{button_joint.name}_rest_no_floating")

            with ctx.pose({button_joint: limits.upper}):
                ctx.expect_contact(
                    button_part,
                    body,
                    name=f"{button_joint.name}_pressed_contact",
                )
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_joint.name}_pressed_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{button_joint.name}_pressed_no_floating")

    ctx.expect_contact(center_knob, body, name="knob_contact_at_rest")
    with ctx.pose({knob_joint: pi / 2.0}):
        ctx.expect_contact(center_knob, body, name="knob_contact_quarter_turn")
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_quarter_turn_no_floating")

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="controls_clear_through_motion")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
