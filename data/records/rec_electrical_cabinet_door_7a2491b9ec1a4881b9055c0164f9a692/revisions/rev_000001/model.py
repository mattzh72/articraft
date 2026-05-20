from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

BODY_W = 0.46
BODY_H = 0.72
BODY_D = 0.19
STEEL_T = 0.0018
FRONT_LIP_D = 0.018
SHELL_MAIN_D = BODY_D - FRONT_LIP_D - STEEL_T

DOOR_PIN_R = 0.0055
DOOR_KNUCKLE_OUTER_R = 0.0072
DOOR_KNUCKLE_LEN = 0.090
DOOR_BOTTOM_CLR = 0.020
DOOR_TOP_CLR = 0.020
DOOR_H = BODY_H - DOOR_BOTTOM_CLR - DOOR_TOP_CLR
DOOR_SKIN_T = 0.0012
DOOR_SKIN_LEFT = DOOR_KNUCKLE_OUTER_R + 0.012
DOOR_SKIN_W = 0.440
DOOR_RETURN_D = 0.016
DOOR_RETURN_T = 0.016
DOOR_LEAF_X = DOOR_SKIN_LEFT - DOOR_KNUCKLE_OUTER_R
DOOR_LEAF_D = 0.006
DOOR_AXIS_X = -(BODY_W / 2.0) - DOOR_PIN_R
DOOR_AXIS_Y = BODY_D + DOOR_PIN_R

INNER_PIN_R = 0.0040
INNER_KNUCKLE_OUTER_R = 0.0055
INNER_KNUCKLE_LEN = 0.080
INNER_AXIS_X = -(BODY_W / 2.0) + 0.011 + INNER_PIN_R
INNER_PANEL_Y = 0.122
INNER_AXIS_Y = INNER_PANEL_Y + INNER_PIN_R
INNER_BOTTOM = 0.080
INNER_H = 0.580
INNER_FRAME_LEFT = INNER_KNUCKLE_OUTER_R + 0.010
INNER_FRAME_W = 0.368
INNER_FRAME_T = 0.0015
INNER_FRAME_SIDE = 0.018
INNER_FRAME_RAIL = 0.018
INNER_LEAF_X = INNER_FRAME_LEFT - INNER_KNUCKLE_OUTER_R
INNER_LEAF_D = 0.006


def _hinge_shell(name: str, inner_radius: float, outer_radius: float, length: float):
    outer_profile = [
        (outer_radius, -length / 2.0),
        (outer_radius, length / 2.0),
    ]
    inner_profile = [
        (inner_radius, -length / 2.0),
        (inner_radius, length / 2.0),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surface_mount_distribution_board")

    shell_gray = model.material("shell_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    door_gray = model.material("door_gray", rgba=(0.84, 0.85, 0.86, 1.0))
    galvanized = model.material("galvanized", rgba=(0.66, 0.69, 0.72, 1.0))
    breaker_black = model.material("breaker_black", rgba=(0.12, 0.13, 0.14, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    label_white = model.material("label_white", rgba=(0.95, 0.95, 0.93, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.73, 0.17, 0.14, 1.0))

    door_knuckle_mesh = _hinge_shell(
        "door_hinge_knuckle",
        inner_radius=DOOR_PIN_R,
        outer_radius=DOOR_KNUCKLE_OUTER_R,
        length=DOOR_KNUCKLE_LEN,
    )
    inner_knuckle_mesh = _hinge_shell(
        "inner_hinge_knuckle",
        inner_radius=INNER_PIN_R,
        outer_radius=INNER_KNUCKLE_OUTER_R,
        length=INNER_KNUCKLE_LEN,
    )

    enclosure_body = model.part("enclosure_body")
    enclosure_body.visual(
        Box((BODY_W, STEEL_T, BODY_H)),
        origin=Origin(xyz=(0.0, STEEL_T / 2.0, BODY_H / 2.0)),
        material=shell_gray,
        name="back_plate",
    )
    enclosure_body.visual(
        Box((STEEL_T, SHELL_MAIN_D, BODY_H)),
        origin=Origin(
            xyz=(
                -(BODY_W / 2.0) + (STEEL_T / 2.0),
                STEEL_T + (SHELL_MAIN_D / 2.0),
                BODY_H / 2.0,
            )
        ),
        material=shell_gray,
        name="left_side_wall",
    )
    enclosure_body.visual(
        Box((STEEL_T, SHELL_MAIN_D, BODY_H)),
        origin=Origin(
            xyz=(
                (BODY_W / 2.0) - (STEEL_T / 2.0),
                STEEL_T + (SHELL_MAIN_D / 2.0),
                BODY_H / 2.0,
            )
        ),
        material=shell_gray,
        name="right_side_wall",
    )
    enclosure_body.visual(
        Box((BODY_W - (2.0 * STEEL_T), SHELL_MAIN_D, STEEL_T)),
        origin=Origin(
            xyz=(
                0.0,
                STEEL_T + (SHELL_MAIN_D / 2.0),
                STEEL_T / 2.0,
            )
        ),
        material=shell_gray,
        name="bottom_panel",
    )
    enclosure_body.visual(
        Box((BODY_W - (2.0 * STEEL_T), SHELL_MAIN_D, STEEL_T)),
        origin=Origin(
            xyz=(
                0.0,
                STEEL_T + (SHELL_MAIN_D / 2.0),
                BODY_H - (STEEL_T / 2.0),
            )
        ),
        material=shell_gray,
        name="top_panel",
    )
    enclosure_body.visual(
        Box((STEEL_T, FRONT_LIP_D, BODY_H - (2.0 * STEEL_T))),
        origin=Origin(
            xyz=(
                -(BODY_W / 2.0) + (STEEL_T / 2.0),
                BODY_D - (FRONT_LIP_D / 2.0),
                BODY_H / 2.0,
            )
        ),
        material=shell_gray,
        name="left_front_return",
    )
    enclosure_body.visual(
        Box((STEEL_T, FRONT_LIP_D, BODY_H - (2.0 * STEEL_T))),
        origin=Origin(
            xyz=(
                (BODY_W / 2.0) - (STEEL_T / 2.0),
                BODY_D - (FRONT_LIP_D / 2.0),
                BODY_H / 2.0,
            )
        ),
        material=shell_gray,
        name="right_front_return",
    )
    enclosure_body.visual(
        Box((BODY_W - (2.0 * STEEL_T), FRONT_LIP_D, STEEL_T)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D - (FRONT_LIP_D / 2.0),
                STEEL_T / 2.0,
            )
        ),
        material=shell_gray,
        name="bottom_front_return",
    )
    enclosure_body.visual(
        Box((BODY_W - (2.0 * STEEL_T), FRONT_LIP_D, STEEL_T)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D - (FRONT_LIP_D / 2.0),
                BODY_H - (STEEL_T / 2.0),
            )
        ),
        material=shell_gray,
        name="top_front_return",
    )
    enclosure_body.visual(
        Box((0.036, 0.020, 0.050)),
        origin=Origin(
            xyz=((BODY_W / 2.0) - STEEL_T - 0.018, BODY_D - FRONT_LIP_D - 0.008, BODY_H / 2.0)
        ),
        material=galvanized,
        name="door_strike_block",
    )
    enclosure_body.visual(
        Box((DOOR_PIN_R, 0.008, DOOR_KNUCKLE_LEN)),
        origin=Origin(
            xyz=(
                -(BODY_W / 2.0) - (DOOR_PIN_R / 2.0),
                BODY_D - 0.004,
                DOOR_BOTTOM_CLR + 0.115,
            )
        ),
        material=galvanized,
        name="lower_hinge_bracket",
    )
    enclosure_body.visual(
        Box((DOOR_PIN_R, 0.008, DOOR_KNUCKLE_LEN)),
        origin=Origin(
            xyz=(
                -(BODY_W / 2.0) - (DOOR_PIN_R / 2.0),
                BODY_D - 0.004,
                BODY_H - DOOR_TOP_CLR - 0.115,
            )
        ),
        material=galvanized,
        name="upper_hinge_bracket",
    )
    enclosure_body.visual(
        Cylinder(radius=DOOR_PIN_R, length=DOOR_KNUCKLE_LEN),
        origin=Origin(
            xyz=(DOOR_AXIS_X, DOOR_AXIS_Y, DOOR_BOTTOM_CLR + 0.115)
        ),
        material=galvanized,
        name="lower_hinge_pin",
    )
    enclosure_body.visual(
        Cylinder(radius=DOOR_PIN_R, length=DOOR_KNUCKLE_LEN),
        origin=Origin(
            xyz=(DOOR_AXIS_X, DOOR_AXIS_Y, BODY_H - DOOR_TOP_CLR - 0.115)
        ),
        material=galvanized,
        name="upper_hinge_pin",
    )
    inner_bracket_x = (INNER_AXIS_X - (-(BODY_W / 2.0) + STEEL_T))
    enclosure_body.visual(
        Box((inner_bracket_x, 0.008, INNER_KNUCKLE_LEN)),
        origin=Origin(
            xyz=(
                (-(BODY_W / 2.0) + STEEL_T) + (inner_bracket_x / 2.0),
                INNER_PANEL_Y - 0.004,
                INNER_BOTTOM + 0.110,
            )
        ),
        material=galvanized,
        name="lower_inner_hinge_bracket",
    )
    enclosure_body.visual(
        Box((inner_bracket_x, 0.008, INNER_KNUCKLE_LEN)),
        origin=Origin(
            xyz=(
                (-(BODY_W / 2.0) + STEEL_T) + (inner_bracket_x / 2.0),
                INNER_PANEL_Y - 0.004,
                INNER_BOTTOM + INNER_H - 0.110,
            )
        ),
        material=galvanized,
        name="upper_inner_hinge_bracket",
    )
    enclosure_body.visual(
        Cylinder(radius=INNER_PIN_R, length=INNER_KNUCKLE_LEN),
        origin=Origin(xyz=(INNER_AXIS_X, INNER_AXIS_Y, INNER_BOTTOM + 0.110)),
        material=galvanized,
        name="lower_inner_hinge_pin",
    )
    enclosure_body.visual(
        Cylinder(radius=INNER_PIN_R, length=INNER_KNUCKLE_LEN),
        origin=Origin(xyz=(INNER_AXIS_X, INNER_AXIS_Y, INNER_BOTTOM + INNER_H - 0.110)),
        material=galvanized,
        name="upper_inner_hinge_pin",
    )
    enclosure_body.visual(
        Box((0.014, 0.010, 0.620)),
        origin=Origin(
            xyz=(
                -(BODY_W / 2.0) + STEEL_T + 0.007,
                INNER_PANEL_Y - 0.005,
                INNER_BOTTOM + (INNER_H / 2.0),
            )
        ),
        material=galvanized,
        name="inner_mount_rail",
    )
    enclosure_body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D + 0.020, BODY_H)),
        mass=18.0,
        origin=Origin(xyz=(0.0, (BODY_D + 0.020) / 2.0, BODY_H / 2.0)),
    )

    front_door = model.part("front_door")
    front_door.visual(
        Box((DOOR_SKIN_W, DOOR_SKIN_T, DOOR_H)),
        origin=Origin(
            xyz=(
                DOOR_SKIN_LEFT + (DOOR_SKIN_W / 2.0),
                (DOOR_SKIN_T / 2.0) - DOOR_PIN_R,
                DOOR_H / 2.0,
            )
        ),
        material=door_gray,
        name="door_skin",
    )
    front_door.visual(
        Box((DOOR_SKIN_W, DOOR_RETURN_D, DOOR_RETURN_T)),
        origin=Origin(
            xyz=(
                DOOR_SKIN_LEFT + (DOOR_SKIN_W / 2.0),
                -(DOOR_PIN_R + (DOOR_RETURN_D / 2.0)),
                DOOR_RETURN_T / 2.0,
            )
        ),
        material=door_gray,
        name="bottom_return",
    )
    front_door.visual(
        Box((DOOR_SKIN_W, DOOR_RETURN_D, DOOR_RETURN_T)),
        origin=Origin(
            xyz=(
                DOOR_SKIN_LEFT + (DOOR_SKIN_W / 2.0),
                -(DOOR_PIN_R + (DOOR_RETURN_D / 2.0)),
                DOOR_H - (DOOR_RETURN_T / 2.0),
            )
        ),
        material=door_gray,
        name="top_return",
    )
    front_door.visual(
        Box((DOOR_RETURN_T, DOOR_RETURN_D, DOOR_H - (2.0 * DOOR_RETURN_T))),
        origin=Origin(
            xyz=(
                DOOR_SKIN_LEFT + DOOR_SKIN_W - (DOOR_RETURN_T / 2.0),
                -(DOOR_PIN_R + (DOOR_RETURN_D / 2.0)),
                DOOR_H / 2.0,
            )
        ),
        material=door_gray,
        name="right_return",
    )
    front_door.visual(
        door_knuckle_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=galvanized,
        name="lower_hinge_sleeve",
    )
    front_door.visual(
        door_knuckle_mesh,
        origin=Origin(xyz=(0.0, 0.0, DOOR_H - 0.115)),
        material=galvanized,
        name="upper_hinge_sleeve",
    )
    front_door.visual(
        Box((DOOR_LEAF_X, DOOR_LEAF_D, DOOR_KNUCKLE_LEN)),
        origin=Origin(
            xyz=(
                DOOR_KNUCKLE_OUTER_R + (DOOR_LEAF_X / 2.0),
                -DOOR_PIN_R + (DOOR_LEAF_D / 2.0),
                0.115,
            )
        ),
        material=galvanized,
        name="lower_leaf",
    )
    front_door.visual(
        Box((DOOR_LEAF_X, DOOR_LEAF_D, DOOR_KNUCKLE_LEN)),
        origin=Origin(
            xyz=(
                DOOR_KNUCKLE_OUTER_R + (DOOR_LEAF_X / 2.0),
                -DOOR_PIN_R + (DOOR_LEAF_D / 2.0),
                DOOR_H - 0.115,
            )
        ),
        material=galvanized,
        name="upper_leaf",
    )
    front_door.visual(
        Box((0.038, 0.016, 0.105)),
        origin=Origin(
            xyz=(
                DOOR_SKIN_LEFT + DOOR_SKIN_W - 0.050,
                -DOOR_PIN_R + DOOR_SKIN_T + 0.008,
                DOOR_H / 2.0,
            )
        ),
        material=handle_black,
        name="pull_handle",
    )
    front_door.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(
            xyz=(
                DOOR_SKIN_LEFT + DOOR_SKIN_W - 0.050,
                -DOOR_PIN_R + DOOR_SKIN_T + 0.021,
                DOOR_H / 2.0,
            ),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material=indicator_red,
        name="lock_core",
    )
    front_door.visual(
        Box((0.120, 0.0012, 0.050)),
        origin=Origin(
            xyz=(
                DOOR_SKIN_LEFT + (DOOR_SKIN_W / 2.0),
                -DOOR_PIN_R + DOOR_SKIN_T + 0.0006,
                DOOR_H - 0.090,
            )
        ),
        material=label_white,
        name="circuit_schedule_card",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((DOOR_SKIN_LEFT + DOOR_SKIN_W, 0.030, DOOR_H)),
        mass=5.0,
        origin=Origin(
            xyz=((DOOR_SKIN_LEFT + DOOR_SKIN_W) / 2.0, 0.0, DOOR_H / 2.0)
        ),
    )

    inner_subpanel = model.part("inner_subpanel")
    inner_subpanel.visual(
        inner_knuckle_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=galvanized,
        name="lower_inner_sleeve",
    )
    inner_subpanel.visual(
        inner_knuckle_mesh,
        origin=Origin(xyz=(0.0, 0.0, INNER_H - 0.110)),
        material=galvanized,
        name="upper_inner_sleeve",
    )
    inner_subpanel.visual(
        Box((INNER_LEAF_X, INNER_LEAF_D, INNER_KNUCKLE_LEN)),
        origin=Origin(
            xyz=(
                INNER_KNUCKLE_OUTER_R + (INNER_LEAF_X / 2.0),
                -INNER_PIN_R + (INNER_LEAF_D / 2.0),
                0.110,
            )
        ),
        material=galvanized,
        name="lower_inner_leaf",
    )
    inner_subpanel.visual(
        Box((INNER_LEAF_X, INNER_LEAF_D, INNER_KNUCKLE_LEN)),
        origin=Origin(
            xyz=(
                INNER_KNUCKLE_OUTER_R + (INNER_LEAF_X / 2.0),
                -INNER_PIN_R + (INNER_LEAF_D / 2.0),
                INNER_H - 0.110,
            )
        ),
        material=galvanized,
        name="upper_inner_leaf",
    )
    inner_subpanel.visual(
        Box((INNER_FRAME_SIDE, INNER_FRAME_T, INNER_H)),
        origin=Origin(
            xyz=(
                INNER_FRAME_LEFT + (INNER_FRAME_SIDE / 2.0),
                (INNER_FRAME_T / 2.0) - INNER_PIN_R,
                INNER_H / 2.0,
            )
        ),
        material=door_gray,
        name="left_upright",
    )
    inner_subpanel.visual(
        Box((INNER_FRAME_SIDE, INNER_FRAME_T, INNER_H)),
        origin=Origin(
            xyz=(
                INNER_FRAME_LEFT + INNER_FRAME_W - (INNER_FRAME_SIDE / 2.0),
                (INNER_FRAME_T / 2.0) - INNER_PIN_R,
                INNER_H / 2.0,
            )
        ),
        material=door_gray,
        name="right_upright",
    )
    inner_subpanel.visual(
        Box((INNER_FRAME_W, INNER_FRAME_T, INNER_FRAME_RAIL)),
        origin=Origin(
            xyz=(
                INNER_FRAME_LEFT + (INNER_FRAME_W / 2.0),
                (INNER_FRAME_T / 2.0) - INNER_PIN_R,
                INNER_FRAME_RAIL / 2.0,
            )
        ),
        material=door_gray,
        name="bottom_rail",
    )
    inner_subpanel.visual(
        Box((INNER_FRAME_W, INNER_FRAME_T, INNER_FRAME_RAIL)),
        origin=Origin(
            xyz=(
                INNER_FRAME_LEFT + (INNER_FRAME_W / 2.0),
                (INNER_FRAME_T / 2.0) - INNER_PIN_R,
                INNER_H - (INNER_FRAME_RAIL / 2.0),
            )
        ),
        material=door_gray,
        name="top_rail",
    )

    rail_w = INNER_FRAME_W - (2.0 * INNER_FRAME_SIDE)
    rail_x = INNER_FRAME_LEFT + INNER_FRAME_SIDE + (rail_w / 2.0)
    row_zs = (
        INNER_H - 0.120,
        INNER_H - 0.270,
        INNER_H - 0.420,
    )
    row_names = ("upper", "middle", "lower")
    for row_name, row_z in zip(row_names, row_zs):
        inner_subpanel.visual(
            Box((rail_w, 0.004, 0.016)),
            origin=Origin(
                xyz=(
                    rail_x,
                    (INNER_FRAME_T / 2.0) - INNER_PIN_R + 0.002,
                    row_z,
                )
            ),
            material=galvanized,
            name=f"din_rail_{row_name}",
        )
        inner_subpanel.visual(
            Box((rail_w - 0.010, 0.024, 0.070)),
            origin=Origin(
                xyz=(
                    rail_x,
                    (INNER_FRAME_T / 2.0) - INNER_PIN_R + 0.016,
                    row_z + 0.012,
                )
            ),
            material=breaker_black,
            name=f"breaker_bank_{row_name}",
        )
        toggle_pitch = (rail_w - 0.050) / 5.0
        toggle_left = rail_x - ((rail_w - 0.050) / 2.0)
        for toggle_index in range(6):
            inner_subpanel.visual(
                Box((0.014, 0.010, 0.026)),
                origin=Origin(
                    xyz=(
                        toggle_left + (toggle_index * toggle_pitch),
                        (INNER_FRAME_T / 2.0) - INNER_PIN_R + 0.033,
                        row_z + 0.004,
                    )
                ),
                material=label_white,
                name=f"{row_name}_toggle_{toggle_index}",
            )

    inner_subpanel.visual(
        Box((0.026, 0.014, 0.080)),
        origin=Origin(
            xyz=(
                INNER_FRAME_LEFT + INNER_FRAME_W - 0.020,
                (INNER_FRAME_T / 2.0) - INNER_PIN_R + 0.00775,
                INNER_H / 2.0,
            )
        ),
        material=handle_black,
        name="subpanel_latch",
    )
    inner_subpanel.inertial = Inertial.from_geometry(
        Box((INNER_FRAME_LEFT + INNER_FRAME_W, 0.040, INNER_H)),
        mass=3.0,
        origin=Origin(
            xyz=((INNER_FRAME_LEFT + INNER_FRAME_W) / 2.0, 0.0, INNER_H / 2.0)
        ),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure_body,
        child=front_door,
        origin=Origin(xyz=(DOOR_AXIS_X, DOOR_AXIS_Y, DOOR_BOTTOM_CLR)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=2.05,
        ),
    )
    model.articulation(
        "inner_subpanel_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure_body,
        child=inner_subpanel,
        origin=Origin(xyz=(INNER_AXIS_X, INNER_AXIS_Y, INNER_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=1.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure_body = object_model.get_part("enclosure_body")
    front_door = object_model.get_part("front_door")
    inner_subpanel = object_model.get_part("inner_subpanel")
    door_hinge = object_model.get_articulation("door_hinge")
    inner_hinge = object_model.get_articulation("inner_subpanel_hinge")

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
    ctx.allow_overlap(
        front_door,
        enclosure_body,
        elem_a="lower_hinge_sleeve",
        elem_b="lower_hinge_pin",
        reason="Door hinge sleeve rotates concentrically around the fixed hinge pin.",
    )
    ctx.allow_overlap(
        front_door,
        enclosure_body,
        elem_a="upper_hinge_sleeve",
        elem_b="upper_hinge_pin",
        reason="Door hinge sleeve rotates concentrically around the fixed hinge pin.",
    )
    ctx.allow_overlap(
        inner_subpanel,
        enclosure_body,
        elem_a="lower_inner_sleeve",
        elem_b="lower_inner_hinge_pin",
        reason="Inner subpanel hinge sleeve rotates around the fixed inner hinge pin.",
    )
    ctx.allow_overlap(
        inner_subpanel,
        enclosure_body,
        elem_a="upper_inner_sleeve",
        elem_b="upper_inner_hinge_pin",
        reason="Inner subpanel hinge sleeve rotates around the fixed inner hinge pin.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "door hinge axis is vertical",
        tuple(round(value, 4) for value in door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"door hinge axis={door_hinge.axis}",
    )
    ctx.check(
        "inner hinge axis is vertical",
        tuple(round(value, 4) for value in inner_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"inner hinge axis={inner_hinge.axis}",
    )
    ctx.check(
        "door opens with realistic travel",
        door_hinge.motion_limits is not None and door_hinge.motion_limits.upper is not None and door_hinge.motion_limits.upper > 1.8,
        details=f"door upper limit={None if door_hinge.motion_limits is None else door_hinge.motion_limits.upper}",
    )
    ctx.check(
        "subpanel opens with realistic travel",
        inner_hinge.motion_limits is not None and inner_hinge.motion_limits.upper is not None and inner_hinge.motion_limits.upper > 1.4,
        details=f"subpanel upper limit={None if inner_hinge.motion_limits is None else inner_hinge.motion_limits.upper}",
    )

    ctx.expect_contact(
        front_door,
        enclosure_body,
        elem_a="lower_hinge_sleeve",
        elem_b="lower_hinge_pin",
        contact_tol=0.0005,
        name="door lower knuckle is mounted on hinge pin",
    )
    ctx.expect_contact(
        inner_subpanel,
        enclosure_body,
        elem_a="lower_inner_sleeve",
        elem_b="lower_inner_hinge_pin",
        contact_tol=0.0005,
        name="subpanel lower knuckle is mounted on hinge pin",
    )
    ctx.expect_gap(
        front_door,
        inner_subpanel,
        axis="y",
        positive_elem="door_skin",
        min_gap=0.020,
        max_gap=0.040,
        name="door skin clears breaker subpanel",
    )
    ctx.expect_overlap(
        front_door,
        enclosure_body,
        axes="xz",
        elem_a="door_skin",
        elem_b="back_plate",
        min_overlap=0.400,
        name="door covers body opening in front view",
    )
    ctx.expect_within(
        inner_subpanel,
        enclosure_body,
        axes="xz",
        margin=0.035,
        name="inner subpanel stays inside enclosure envelope",
    )

    door_rest = ctx.part_element_world_aabb(front_door, elem="door_skin")
    subpanel_rest = ctx.part_element_world_aabb(inner_subpanel, elem="breaker_bank_upper")
    assert door_rest is not None
    assert subpanel_rest is not None

    with ctx.pose({door_hinge: 1.30}):
        door_open = ctx.part_element_world_aabb(front_door, elem="door_skin")
        assert door_open is not None
        ctx.check(
            "door swings outward when opened",
            door_open[1][1] > door_rest[1][1] + 0.12,
            details=f"rest_max_y={door_rest[1][1]:.4f}, open_max_y={door_open[1][1]:.4f}",
        )

    with ctx.pose({door_hinge: 1.15, inner_hinge: 0.95}):
        subpanel_open = ctx.part_element_world_aabb(inner_subpanel, elem="breaker_bank_upper")
        assert subpanel_open is not None
        ctx.check(
            "subpanel swings outward behind open door",
            subpanel_open[1][1] > subpanel_rest[1][1] + 0.05,
            details=f"rest_max_y={subpanel_rest[1][1]:.4f}, open_max_y={subpanel_open[1][1]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
