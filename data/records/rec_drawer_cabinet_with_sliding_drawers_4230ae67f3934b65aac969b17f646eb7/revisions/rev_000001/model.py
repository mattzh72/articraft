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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_modular_bin_cabinet")

    bracket_metal = model.material("bracket_metal", rgba=(0.19, 0.20, 0.22, 1.0))
    cabinet_steel = model.material("cabinet_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    cabinet_shadow = model.material("cabinet_shadow", rgba=(0.50, 0.52, 0.55, 1.0))
    hopper_amber = model.material("hopper_amber", rgba=(0.79, 0.56, 0.16, 0.78))
    handle_dark = model.material("handle_dark", rgba=(0.12, 0.13, 0.14, 1.0))

    cabinet_width = 1.70
    cabinet_height = 0.76
    cabinet_depth = 0.29
    back_panel_t = 0.003
    side_wall_t = 0.018
    rail_h = 0.022
    divider_t = 0.008
    rows = 2
    cols = 10

    opening_width = (cabinet_width - (2.0 * side_wall_t) - ((cols - 1) * divider_t)) / cols
    opening_height = (cabinet_height - (3.0 * rail_h)) / rows
    row_pitch = opening_height + rail_h
    col_pitch = opening_width + divider_t

    door_width = opening_width - 0.012
    door_panel_height = opening_height - 0.082
    door_lip_height = 0.040
    door_flange_depth = 0.030
    door_sheet_t = 0.0035
    closed_tilt = 0.30

    def _rotate_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
        x, y, z = point
        c = math.cos(angle)
        s = math.sin(angle)
        return (x, y * c - z * s, y * s + z * c)

    def _door_origin(local_center: tuple[float, float, float]) -> Origin:
        pitch = -closed_tilt
        return Origin(xyz=_rotate_x(local_center, pitch), rpy=(pitch, 0.0, 0.0))

    wall_bracket = model.part("wall_bracket")
    bracket_width = cabinet_width - 0.10
    bracket_height = cabinet_height - 0.08
    bracket_t = 0.008
    bracket_side_w = 0.045
    bracket_cross_h = 0.050
    bracket_center_w = 0.080
    bracket_channel_h = 0.040

    wall_bracket.visual(
        Box((bracket_side_w, bracket_height, bracket_t)),
        origin=Origin(xyz=(-(bracket_width * 0.5) + (bracket_side_w * 0.5), 0.0, bracket_t * 0.5)),
        material=bracket_metal,
        name="left_rail",
    )
    wall_bracket.visual(
        Box((bracket_side_w, bracket_height, bracket_t)),
        origin=Origin(xyz=((bracket_width * 0.5) - (bracket_side_w * 0.5), 0.0, bracket_t * 0.5)),
        material=bracket_metal,
        name="right_rail",
    )
    wall_bracket.visual(
        Box((bracket_width, bracket_cross_h, bracket_t)),
        origin=Origin(xyz=(0.0, (bracket_height * 0.5) - (bracket_cross_h * 0.5), bracket_t * 0.5)),
        material=bracket_metal,
        name="top_rail",
    )
    wall_bracket.visual(
        Box((bracket_width, bracket_cross_h, bracket_t)),
        origin=Origin(xyz=(0.0, -(bracket_height * 0.5) + (bracket_cross_h * 0.5), bracket_t * 0.5)),
        material=bracket_metal,
        name="bottom_rail",
    )
    wall_bracket.visual(
        Box((bracket_center_w, bracket_height, bracket_t)),
        origin=Origin(xyz=(0.0, 0.0, bracket_t * 0.5)),
        material=bracket_metal,
        name="center_rail",
    )
    for channel_index, y_pos in enumerate((0.22, -0.22)):
        wall_bracket.visual(
            Box((bracket_width * 0.78, bracket_channel_h, bracket_t)),
            origin=Origin(xyz=(0.0, y_pos, bracket_t * 0.5)),
            material=bracket_metal,
            name=f"mount_channel_{channel_index}",
        )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((bracket_width, bracket_height, bracket_t)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, bracket_t * 0.5)),
    )

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        Box((cabinet_width, cabinet_height, back_panel_t)),
        origin=Origin(xyz=(0.0, 0.0, back_panel_t * 0.5)),
        material=cabinet_steel,
        name="rear_panel",
    )
    cabinet_body.visual(
        Box((side_wall_t, cabinet_height, cabinet_depth)),
        origin=Origin(
            xyz=(
                -(cabinet_width * 0.5) + (side_wall_t * 0.5),
                0.0,
                cabinet_depth * 0.5,
            )
        ),
        material=cabinet_steel,
        name="left_side",
    )
    cabinet_body.visual(
        Box((side_wall_t, cabinet_height, cabinet_depth)),
        origin=Origin(
            xyz=(
                (cabinet_width * 0.5) - (side_wall_t * 0.5),
                0.0,
                cabinet_depth * 0.5,
            )
        ),
        material=cabinet_steel,
        name="right_side",
    )
    cabinet_body.visual(
        Box((cabinet_width, rail_h, cabinet_depth)),
        origin=Origin(xyz=(0.0, (cabinet_height * 0.5) - (rail_h * 0.5), cabinet_depth * 0.5)),
        material=cabinet_steel,
        name="top_panel",
    )
    cabinet_body.visual(
        Box((cabinet_width, rail_h, cabinet_depth)),
        origin=Origin(xyz=(0.0, -(cabinet_height * 0.5) + (rail_h * 0.5), cabinet_depth * 0.5)),
        material=cabinet_steel,
        name="bottom_panel",
    )
    cabinet_body.visual(
        Box((cabinet_width - (2.0 * side_wall_t), rail_h, cabinet_depth)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_depth * 0.5)),
        material=cabinet_shadow,
        name="mid_shelf",
    )
    for divider_index in range(cols - 1):
        x_pos = (
            -(cabinet_width * 0.5)
            + side_wall_t
            + opening_width
            + (divider_index * col_pitch)
            + (divider_t * 0.5)
        )
        cabinet_body.visual(
            Box((divider_t, cabinet_height - (2.0 * rail_h), cabinet_depth)),
            origin=Origin(xyz=(x_pos, 0.0, cabinet_depth * 0.5)),
            material=cabinet_shadow,
            name=f"divider_{divider_index}",
        )
    cabinet_body.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_height, cabinet_depth)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_depth * 0.5)),
    )

    model.articulation(
        "wall_bracket_to_cabinet_body",
        ArticulationType.FIXED,
        parent=wall_bracket,
        child=cabinet_body,
        origin=Origin(xyz=(0.0, 0.0, bracket_t)),
    )

    def _add_hopper_door(row_index: int, col_index: int) -> None:
        door = model.part(f"bin_door_{row_index}_{col_index}")
        door.visual(
            Cylinder(radius=0.0035, length=door_width - 0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=handle_dark,
            name="hinge_barrel",
        )
        door.visual(
            Box((door_width, door_panel_height, door_sheet_t)),
            origin=_door_origin((0.0, -(door_panel_height * 0.5), 0.0)),
            material=hopper_amber,
            name="door_panel",
        )
        door.visual(
            Box((door_sheet_t, door_panel_height, door_flange_depth)),
            origin=_door_origin(
                (
                    -((door_width * 0.5) - (door_sheet_t * 0.5)),
                    -(door_panel_height * 0.5),
                    -(door_flange_depth * 0.5),
                )
            ),
            material=hopper_amber,
            name="left_flange",
        )
        door.visual(
            Box((door_sheet_t, door_panel_height, door_flange_depth)),
            origin=_door_origin(
                (
                    (door_width * 0.5) - (door_sheet_t * 0.5),
                    -(door_panel_height * 0.5),
                    -(door_flange_depth * 0.5),
                )
            ),
            material=hopper_amber,
            name="right_flange",
        )
        door.visual(
            Box((door_width, door_lip_height, door_flange_depth + door_sheet_t)),
            origin=_door_origin(
                (
                    0.0,
                    -door_panel_height - (door_lip_height * 0.5),
                    -((door_flange_depth - door_sheet_t) * 0.5),
                )
            ),
            material=hopper_amber,
            name="hopper_lip",
        )
        door.visual(
            Box((door_width * 0.44, 0.016, 0.010)),
            origin=_door_origin((0.0, -(door_panel_height * 0.78), 0.0045)),
            material=handle_dark,
            name="pull_grip",
        )
        door.inertial = Inertial.from_geometry(
            Box((door_width, door_panel_height + door_lip_height, 0.060)),
            mass=0.85,
            origin=Origin(
                xyz=(
                    0.0,
                    -0.5 * (door_panel_height + door_lip_height),
                    0.028,
                )
            ),
        )

        hinge_x = (
            -(cabinet_width * 0.5)
            + side_wall_t
            + (opening_width * 0.5)
            + (col_index * col_pitch)
        )
        row_center_y = ((rows - 1) * 0.5 - row_index) * row_pitch
        hinge_y = row_center_y + (opening_height * 0.5)
        hinge_z = cabinet_depth + 0.0035

        model.articulation(
            f"cabinet_to_bin_door_{row_index}_{col_index}",
            ArticulationType.REVOLUTE,
            parent=cabinet_body,
            child=door,
            origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.2,
                lower=0.0,
                upper=1.10,
            ),
        )

    for row_index in range(rows):
        for col_index in range(cols):
            _add_hopper_door(row_index, col_index)

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

    wall_bracket = object_model.get_part("wall_bracket")
    cabinet_body = object_model.get_part("cabinet_body")
    sample_upper_door = object_model.get_part("bin_door_0_0")
    sample_lower_door = object_model.get_part("bin_door_1_9")
    sample_upper_hinge = object_model.get_articulation("cabinet_to_bin_door_0_0")

    missing_parts: list[str] = []
    missing_joints: list[str] = []
    for row_index in range(2):
        for col_index in range(10):
            part_name = f"bin_door_{row_index}_{col_index}"
            joint_name = f"cabinet_to_bin_door_{row_index}_{col_index}"
            try:
                object_model.get_part(part_name)
            except Exception:
                missing_parts.append(part_name)
            try:
                object_model.get_articulation(joint_name)
            except Exception:
                missing_joints.append(joint_name)

    ctx.check(
        "all twenty hopper doors are authored",
        not missing_parts and not missing_joints,
        details=f"missing_parts={missing_parts}, missing_joints={missing_joints}",
    )

    ctx.expect_contact(
        cabinet_body,
        wall_bracket,
        name="cabinet body sits flush against the wall bracket",
    )

    with ctx.pose({sample_upper_hinge: 0.0}):
        ctx.expect_overlap(
            sample_upper_door,
            cabinet_body,
            axes="xy",
            min_overlap=0.12,
            elem_a="door_panel",
            name="upper hopper door aligns over the cabinet opening footprint",
        )
        ctx.expect_gap(
            sample_upper_door,
            cabinet_body,
            axis="z",
            positive_elem="door_panel",
            max_gap=0.03,
            max_penetration=0.0,
            name="closed hopper door sits just proud of the cabinet face",
        )
        ctx.expect_gap(
            sample_lower_door,
            cabinet_body,
            axis="z",
            positive_elem="door_panel",
            max_gap=0.03,
            max_penetration=0.0,
            name="lower hopper door also stays clear of the cabinet face",
        )

    closed_lip = ctx.part_element_world_aabb(sample_upper_door, elem="hopper_lip")
    upper_limit = 1.10
    if sample_upper_hinge.motion_limits is not None and sample_upper_hinge.motion_limits.upper is not None:
        upper_limit = sample_upper_hinge.motion_limits.upper
    with ctx.pose({sample_upper_hinge: upper_limit}):
        opened_lip = ctx.part_element_world_aabb(sample_upper_door, elem="hopper_lip")

    lip_moves_outward = (
        closed_lip is not None
        and opened_lip is not None
        and opened_lip[1][2] > closed_lip[1][2] + 0.06
        and opened_lip[0][1] > closed_lip[0][1] + 0.015
    )
    ctx.check(
        "representative hopper door swings outward from its top hinge",
        lip_moves_outward,
        details=f"closed_lip={closed_lip}, opened_lip={opened_lip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
