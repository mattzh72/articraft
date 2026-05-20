from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ExtrudeWithHolesGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_microwave")

    stainless = model.material("stainless", rgba=(0.70, 0.72, 0.74, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.11, 0.12, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.16, 0.20, 0.23, 0.58))
    display_glass = model.material("display_glass", rgba=(0.08, 0.15, 0.12, 0.90))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    body = model.part("body")

    cabinet_w = 0.52
    cabinet_d = 0.42
    cabinet_h = 0.30
    feet_h = 0.010

    side_t = 0.014
    top_t = 0.014
    bottom_t = 0.012
    liner_t = 0.006

    front_y = cabinet_d / 2.0
    body_top_z = feet_h + cabinet_h

    # Main cabinet shell.
    body.visual(
        Box((cabinet_w, cabinet_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, feet_h + bottom_t / 2.0)),
        material=stainless,
        name="cabinet_bottom",
    )
    body.visual(
        Box((cabinet_w, cabinet_d, top_t)),
        origin=Origin(xyz=(0.0, 0.0, body_top_z - top_t / 2.0)),
        material=stainless,
        name="cabinet_top",
    )
    body.visual(
        Box((side_t, cabinet_d, cabinet_h - top_t - bottom_t)),
        origin=Origin(xyz=(-cabinet_w / 2.0 + side_t / 2.0, 0.0, feet_h + bottom_t + (cabinet_h - top_t - bottom_t) / 2.0)),
        material=stainless,
        name="cabinet_left",
    )
    body.visual(
        Box((side_t, cabinet_d, cabinet_h - top_t - bottom_t)),
        origin=Origin(xyz=(cabinet_w / 2.0 - side_t / 2.0, 0.0, feet_h + bottom_t + (cabinet_h - top_t - bottom_t) / 2.0)),
        material=stainless,
        name="cabinet_right",
    )
    body.visual(
        Box((cabinet_w - 2.0 * side_t, 0.012, cabinet_h - top_t - bottom_t)),
        origin=Origin(xyz=(0.0, -cabinet_d / 2.0 + 0.006, feet_h + bottom_t + (cabinet_h - top_t - bottom_t) / 2.0)),
        material=stainless,
        name="cabinet_back",
    )

    # Interior cooking cavity, explicitly hollow rather than a solid proxy.
    cavity_w = 0.344
    cavity_d = 0.322
    cavity_x = -0.056
    cavity_y = 0.026
    cavity_z0 = feet_h + bottom_t
    cavity_z1 = body_top_z - top_t - 0.006
    cavity_h = cavity_z1 - cavity_z0

    body.visual(
        Box((cavity_w, cavity_d, liner_t)),
        origin=Origin(xyz=(cavity_x, cavity_y, cavity_z0 + liner_t / 2.0)),
        material=charcoal,
        name="cavity_floor",
    )
    body.visual(
        Box((cavity_w, cavity_d, liner_t)),
        origin=Origin(xyz=(cavity_x, cavity_y, cavity_z1 - liner_t / 2.0)),
        material=charcoal,
        name="cavity_ceiling",
    )
    body.visual(
        Box((liner_t, cavity_d, cavity_h - 2.0 * liner_t)),
        origin=Origin(
            xyz=(
                cavity_x - cavity_w / 2.0 - liner_t / 2.0,
                cavity_y,
                (cavity_z0 + cavity_z1) / 2.0,
            )
        ),
        material=charcoal,
        name="cavity_left",
    )
    body.visual(
        Box((liner_t, cavity_d, cavity_h - 2.0 * liner_t)),
        origin=Origin(
            xyz=(
                cavity_x + cavity_w / 2.0 + liner_t / 2.0,
                cavity_y,
                (cavity_z0 + cavity_z1) / 2.0,
            )
        ),
        material=charcoal,
        name="cavity_right",
    )
    body.visual(
        Box((cavity_w + 2.0 * liner_t, liner_t, cavity_h - 2.0 * liner_t)),
        origin=Origin(
            xyz=(
                cavity_x,
                cavity_y - cavity_d / 2.0 - liner_t / 2.0,
                (cavity_z0 + cavity_z1) / 2.0,
            )
        ),
        material=charcoal,
        name="cavity_back",
    )

    # Front opening trim and control column.
    door_w = 0.386
    door_h = 0.248
    door_bottom_z = 0.043
    door_center_z = door_bottom_z + door_h / 2.0
    door_hinge_x = -0.244
    door_center_x = door_hinge_x + door_w / 2.0
    door_frame_w = door_w + 0.020

    body.visual(
        Box((door_frame_w, 0.014, 0.018)),
        origin=Origin(xyz=(door_center_x, front_y - 0.007, body_top_z - 0.009)),
        material=trim_black,
        name="door_header",
    )
    body.visual(
        Box((door_frame_w, 0.018, 0.018)),
        origin=Origin(xyz=(door_center_x, front_y - 0.009, feet_h + 0.009)),
        material=trim_black,
        name="door_sill",
    )
    body.visual(
        Box((0.016, 0.018, door_h + 0.020)),
        origin=Origin(xyz=(-0.252, front_y - 0.009, door_center_z)),
        material=trim_black,
        name="door_jamb",
    )
    body.visual(
        Box((0.010, 0.008, 0.052)),
        origin=Origin(xyz=(-0.251, front_y - 0.004, door_center_z + 0.075)),
        material=trim_black,
        name="hinge_mount_upper",
    )
    body.visual(
        Box((0.010, 0.008, 0.052)),
        origin=Origin(xyz=(-0.251, front_y - 0.004, door_center_z - 0.075)),
        material=trim_black,
        name="hinge_mount_lower",
    )

    panel_w = 0.110
    panel_center_x = 0.200
    panel_depth = 0.018

    body.visual(
        Box((0.010, panel_depth, 0.260)),
        origin=Origin(xyz=(0.150, front_y - panel_depth / 2.0, 0.165)),
        material=trim_black,
        name="panel_left_rail",
    )
    body.visual(
        Box((0.010, panel_depth, 0.260)),
        origin=Origin(xyz=(0.250, front_y - panel_depth / 2.0, 0.165)),
        material=trim_black,
        name="panel_right_rail",
    )
    body.visual(
        Box((panel_w, panel_depth, 0.025)),
        origin=Origin(xyz=(panel_center_x, front_y - panel_depth / 2.0, 0.2825)),
        material=trim_black,
        name="panel_top_cap",
    )
    body.visual(
        Box((panel_w, panel_depth, 0.040)),
        origin=Origin(xyz=(panel_center_x, front_y - panel_depth / 2.0, 0.030)),
        material=trim_black,
        name="panel_bottom_cap",
    )
    body.visual(
        Box((0.090, 0.012, 0.170)),
        origin=Origin(xyz=(panel_center_x, 0.186, 0.155)),
        material=charcoal,
        name="panel_recess_backer",
    )
    body.visual(
        Box((0.090, 0.001, 0.170)),
        origin=Origin(xyz=(panel_center_x, front_y - 0.0005, 0.155)),
        material=trim_black,
        name="keypad_membrane",
    )
    body.visual(
        Box((0.090, 0.010, 0.050)),
        origin=Origin(xyz=(panel_center_x, 0.201, 0.248)),
        material=display_glass,
        name="display_window",
    )

    # Countertop feet.
    foot_size = (0.035, 0.030, feet_h)
    for idx, (fx, fy) in enumerate(((-0.18, -0.15), (0.18, -0.15), (-0.18, 0.14), (0.18, 0.14)), start=1):
        body.visual(
            Box(foot_size),
            origin=Origin(xyz=(fx, fy, feet_h / 2.0)),
            material=rubber,
            name=f"foot_{idx}",
        )

    # Side-hinged door.
    door = model.part("door")
    door_frame = ExtrudeWithHolesGeometry(
        rounded_rect_profile(door_w, door_h, 0.010, corner_segments=8),
        [rounded_rect_profile(0.320, 0.176, 0.006, corner_segments=8)],
        0.022,
        center=True,
        closed=True,
    )
    door.visual(
        mesh_from_geometry(door_frame, "microwave_door_frame"),
        origin=Origin(xyz=(door_w / 2.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="door_frame",
    )
    door.visual(
        Box((0.334, 0.006, 0.190)),
        origin=Origin(xyz=(door_w / 2.0, -0.005, 0.0)),
        material=smoked_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.016, 0.020, 0.168)),
        origin=Origin(xyz=(door_w - 0.028, 0.019, 0.0)),
        material=charcoal,
        name="door_handle",
    )
    door.visual(
        Box((0.014, 0.024, 0.180)),
        origin=Origin(xyz=(0.001, -0.004, 0.0)),
        material=trim_black,
        name="door_hinge_leaf",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_hinge_x, front_y + 0.016, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.8,
            lower=0.0,
            upper=1.85,
        ),
    )

    # Membrane button row on short plungers into the control pocket.
    button_zs = (0.196, 0.169, 0.142, 0.115, 0.088)
    for idx, button_z in enumerate(button_zs, start=1):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.066, 0.0035, 0.018)),
            origin=Origin(xyz=(0.0, 0.00175, 0.0)),
            material=charcoal,
            name="button_cap",
        )
        button.visual(
            Box((0.046, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=rubber,
            name="button_plunger",
        )
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(panel_center_x, front_y, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    ctx.check(
        "door hinge is vertical",
        tuple(round(v, 6) for v in door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "door has realistic side swing range",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and 1.5 <= door_hinge.motion_limits.upper <= 2.0,
        details=f"limits={door_hinge.motion_limits}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            max_penetration=1e-5,
            max_gap=0.03,
            name="closed door sits just proud of the cabinet",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.22,
            name="closed door covers the cooking cavity opening",
        )
        closed_aabb = ctx.part_world_aabb(door)

    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door opens outward on its side hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.12,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    button_rest_positions = []
    button_pressed_positions = []
    for idx in range(1, 6):
        button = object_model.get_part(f"button_{idx}")
        button_joint = object_model.get_articulation(f"body_to_button_{idx}")
        limits = button_joint.motion_limits

        ctx.check(
            f"button_{idx} has short inward travel",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.0015 <= limits.upper <= 0.004,
            details=f"limits={limits}",
        )

        with ctx.pose({button_joint: 0.0}):
            rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button)

        button_rest_positions.append(rest_pos)
        button_pressed_positions.append(pressed_pos)
        ctx.check(
            f"button_{idx} presses into the panel",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    stacked = all(
        button_rest_positions[i] is not None
        and button_rest_positions[i + 1] is not None
        and button_rest_positions[i][2] > button_rest_positions[i + 1][2] + 0.015
        for i in range(len(button_rest_positions) - 1)
    )
    ctx.check(
        "buttons form a vertical control row",
        stacked,
        details=f"rest_positions={button_rest_positions}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
