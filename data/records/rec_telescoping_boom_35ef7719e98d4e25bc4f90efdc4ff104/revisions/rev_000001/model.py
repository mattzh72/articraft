from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_L = 0.95
BASE_W = 0.62
BASE_T = 0.03
PEDESTAL_L = 0.22
PEDESTAL_W = 0.28
PEDESTAL_H = 0.33
SADDLE_L = 0.36
SADDLE_W = 0.26
SADDLE_T = 0.025
CHEEK_L = 0.22
CHEEK_T = 0.024
CHEEK_H = 0.17

OUTER_L = 1.55
OUTER_SIDE = 0.26
OUTER_WALL = 0.012
OUTER_REAR_CAP = 0.04
OUTER_REAR_X = -0.18
BOOM_CENTER_Z = BASE_T + PEDESTAL_H + SADDLE_T + (OUTER_SIDE / 2.0)

STAGE1_LEN = 1.40
STAGE1_INSERT = 1.16
STAGE1_PROJ = STAGE1_LEN - STAGE1_INSERT
STAGE1_SIDE = 0.214
STAGE1_WALL = 0.010
STAGE1_TRAVEL = 0.62

STAGE2_LEN = 1.20
STAGE2_INSERT = 0.98
STAGE2_PROJ = STAGE2_LEN - STAGE2_INSERT
STAGE2_SIDE = 0.172
STAGE2_WALL = 0.009
STAGE2_TRAVEL = 0.50

STAGE3_LEN = 1.00
STAGE3_INSERT = 0.80
STAGE3_PROJ = STAGE3_LEN - STAGE3_INSERT
STAGE3_SIDE = 0.134
STAGE3_WALL = 0.008
STAGE3_TRAVEL = 0.42


def _add_box_visual(part, size, xyz, material: str, name: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_square_tube_visuals(
    part,
    *,
    prefix: str,
    material: str,
    length: float,
    outer_side: float,
    wall: float,
    x_start: float,
    rear_cap: float = 0.0,
    front_cap: float = 0.0,
) -> None:
    inner_side = outer_side - (2.0 * wall)
    x_mid = x_start + (length / 2.0)
    side_mid = (outer_side / 2.0) - (wall / 2.0)

    _add_box_visual(
        part,
        (length, outer_side, wall),
        (x_mid, 0.0, side_mid),
        material,
        f"{prefix}_top_wall",
    )
    _add_box_visual(
        part,
        (length, outer_side, wall),
        (x_mid, 0.0, -side_mid),
        material,
        f"{prefix}_bottom_wall",
    )
    _add_box_visual(
        part,
        (length, wall, inner_side),
        (x_mid, side_mid, 0.0),
        material,
        f"{prefix}_left_wall",
    )
    _add_box_visual(
        part,
        (length, wall, inner_side),
        (x_mid, -side_mid, 0.0),
        material,
        f"{prefix}_right_wall",
    )

    if rear_cap > 0.0:
        _add_box_visual(
            part,
            (rear_cap, inner_side, inner_side),
            (x_start + (rear_cap / 2.0), 0.0, 0.0),
            material,
            f"{prefix}_rear_cap",
        )

    if front_cap > 0.0:
        _add_box_visual(
            part,
            (front_cap, inner_side, inner_side),
            (x_start + length - (front_cap / 2.0), 0.0, 0.0),
            material,
            f"{prefix}_front_cap",
        )


def _add_bottom_pads(
    part,
    *,
    prefix: str,
    material: str,
    tube_side: float,
    pad_thickness: float,
    pad_positions: tuple[float, ...],
    pad_length: float,
    width_ratio: float = 0.62,
) -> None:
    pad_width = tube_side * width_ratio
    z_center = -((tube_side / 2.0) + (pad_thickness / 2.0))
    for index, x_start in enumerate(pad_positions, start=1):
        _add_box_visual(
            part,
            (pad_length, pad_width, pad_thickness),
            (x_start + (pad_length / 2.0), 0.0, z_center),
            material,
            f"{prefix}_bottom_pad_{index}",
        )


def _add_stage_visuals(
    part,
    *,
    prefix: str,
    material: str,
    total_length: float,
    insert_length: float,
    outer_side: float,
    wall: float,
    pad_thickness: float,
    pad_positions: tuple[float, ...],
    pad_length: float,
    front_cap: float = 0.0,
) -> None:
    x_start = -insert_length
    _add_square_tube_visuals(
        part,
        prefix=prefix,
        material=material,
        length=total_length,
        outer_side=outer_side,
        wall=wall,
        x_start=x_start,
        rear_cap=wall * 2.5,
        front_cap=front_cap,
    )
    _add_bottom_pads(
        part,
        prefix=prefix,
        material=material,
        tube_side=outer_side,
        pad_thickness=pad_thickness,
        pad_positions=pad_positions,
        pad_length=pad_length,
    )


def _root_bracket_visuals(model: ArticulatedObject, part) -> None:
    _add_box_visual(
        part,
        (BASE_L, BASE_W, BASE_T),
        (0.0, 0.0, BASE_T / 2.0),
        "bracket_steel",
        "base_plate",
    )
    _add_box_visual(
        part,
        (PEDESTAL_L, PEDESTAL_W, PEDESTAL_H),
        (0.0, 0.0, BASE_T + (PEDESTAL_H / 2.0)),
        "bracket_steel",
        "pedestal",
    )
    _add_box_visual(
        part,
        (SADDLE_L, SADDLE_W, SADDLE_T),
        (OUTER_REAR_X + (SADDLE_L / 2.0), 0.0, BASE_T + PEDESTAL_H + (SADDLE_T / 2.0)),
        "bracket_steel",
        "saddle_top",
    )
    _add_box_visual(
        part,
        (CHEEK_L, CHEEK_T, CHEEK_H),
        (
            OUTER_REAR_X + (CHEEK_L / 2.0) + 0.02,
            (SADDLE_W / 2.0) + (CHEEK_T / 2.0),
            BASE_T + PEDESTAL_H + SADDLE_T + (CHEEK_H / 2.0),
        ),
        "bracket_steel",
        "left_cheek",
    )
    _add_box_visual(
        part,
        (CHEEK_L, CHEEK_T, CHEEK_H),
        (
            OUTER_REAR_X + (CHEEK_L / 2.0) + 0.02,
            -((SADDLE_W / 2.0) + (CHEEK_T / 2.0)),
            BASE_T + PEDESTAL_H + SADDLE_T + (CHEEK_H / 2.0),
        ),
        "bracket_steel",
        "right_cheek",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_boom")

    model.material("bracket_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("boom_yellow", rgba=(0.92, 0.75, 0.16, 1.0))
    model.material("boom_amber", rgba=(0.88, 0.62, 0.16, 1.0))
    model.material("boom_orange", rgba=(0.82, 0.43, 0.12, 1.0))
    model.material("inner_dark", rgba=(0.36, 0.38, 0.42, 1.0))

    root_bracket = model.part("root_bracket")
    _root_bracket_visuals(model, root_bracket)

    outer_section = model.part("outer_section")
    _add_square_tube_visuals(
        outer_section,
        prefix="outer",
        material="boom_yellow",
        length=OUTER_L,
        outer_side=OUTER_SIDE,
        wall=OUTER_WALL,
        x_start=0.0,
        rear_cap=OUTER_REAR_CAP,
    )

    stage_1 = model.part("stage_1")
    _add_stage_visuals(
        stage_1,
        prefix="stage1",
        material="boom_amber",
        total_length=STAGE1_LEN,
        insert_length=STAGE1_INSERT,
        outer_side=STAGE1_SIDE,
        wall=STAGE1_WALL,
        pad_thickness=((OUTER_SIDE - (2.0 * OUTER_WALL)) - STAGE1_SIDE) / 2.0,
        pad_positions=(-1.02, -0.82),
        pad_length=0.16,
    )

    stage_2 = model.part("stage_2")
    _add_stage_visuals(
        stage_2,
        prefix="stage2",
        material="boom_yellow",
        total_length=STAGE2_LEN,
        insert_length=STAGE2_INSERT,
        outer_side=STAGE2_SIDE,
        wall=STAGE2_WALL,
        pad_thickness=((STAGE1_SIDE - (2.0 * STAGE1_WALL)) - STAGE2_SIDE) / 2.0,
        pad_positions=(-0.84, -0.66),
        pad_length=0.14,
    )

    stage_3 = model.part("stage_3")
    _add_stage_visuals(
        stage_3,
        prefix="stage3",
        material="inner_dark",
        total_length=STAGE3_LEN,
        insert_length=STAGE3_INSERT,
        outer_side=STAGE3_SIDE,
        wall=STAGE3_WALL,
        pad_thickness=((STAGE2_SIDE - (2.0 * STAGE2_WALL)) - STAGE3_SIDE) / 2.0,
        pad_positions=(-0.68, -0.52),
        pad_length=0.12,
        front_cap=0.02,
    )

    model.articulation(
        "bracket_to_outer",
        ArticulationType.FIXED,
        parent=root_bracket,
        child=outer_section,
        origin=Origin(xyz=(OUTER_REAR_X, 0.0, BOOM_CENTER_Z)),
    )
    model.articulation(
        "outer_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=outer_section,
        child=stage_1,
        origin=Origin(xyz=(OUTER_L, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.35,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(STAGE1_PROJ, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.35,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(STAGE2_PROJ, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.35,
            lower=0.0,
            upper=STAGE3_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    outer_section = object_model.get_part("outer_section")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")

    outer_to_stage_1 = object_model.get_articulation("outer_to_stage_1")
    stage_1_to_stage_2 = object_model.get_articulation("stage_1_to_stage_2")
    stage_2_to_stage_3 = object_model.get_articulation("stage_2_to_stage_3")

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

    ctx.expect_contact(
        root_bracket,
        outer_section,
        elem_a="saddle_top",
        elem_b="outer_bottom_wall",
        contact_tol=1e-5,
        name="outer_box_seated_on_root_saddle",
    )
    ctx.expect_contact(
        outer_section,
        stage_1,
        elem_a="outer_bottom_wall",
        elem_b="stage1_bottom_pad_1",
        contact_tol=1e-5,
        name="stage_1_supported_inside_outer_box",
    )
    ctx.expect_contact(
        stage_1,
        stage_2,
        elem_a="stage1_bottom_wall",
        elem_b="stage2_bottom_pad_1",
        contact_tol=1e-5,
        name="stage_2_supported_inside_stage_1",
    )
    ctx.expect_contact(
        stage_2,
        stage_3,
        elem_a="stage2_bottom_wall",
        elem_b="stage3_bottom_pad_1",
        contact_tol=1e-5,
        name="stage_3_supported_inside_stage_2",
    )

    ctx.expect_within(stage_1, outer_section, axes="yz", margin=0.0005, name="stage_1_cross_section_fits_outer")
    ctx.expect_within(stage_2, stage_1, axes="yz", margin=0.0005, name="stage_2_cross_section_fits_stage_1")
    ctx.expect_within(stage_3, stage_2, axes="yz", margin=0.0005, name="stage_3_cross_section_fits_stage_2")

    ctx.check(
        "all_nested_members_use_forward_prismatic_axes",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == (1.0, 0.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > 0.0
            for joint in (outer_to_stage_1, stage_1_to_stage_2, stage_2_to_stage_3)
        ),
        "Expected three forward-travel prismatic slides along +X.",
    )

    with ctx.pose(
        {
            outer_to_stage_1: STAGE1_TRAVEL,
            stage_1_to_stage_2: STAGE2_TRAVEL,
            stage_2_to_stage_3: STAGE3_TRAVEL,
        }
    ):
        outer_x = ctx.part_world_position(outer_section)[0]
        stage_1_x = ctx.part_world_position(stage_1)[0]
        stage_2_x = ctx.part_world_position(stage_2)[0]
        stage_3_x = ctx.part_world_position(stage_3)[0]
        ctx.check(
            "nested_stages_project_forward_in_sequence",
            stage_3_x > stage_2_x > stage_1_x > outer_x + OUTER_L - 0.02,
            (
                "Expected the three nested stages to telescope forward in order; "
                f"got outer={outer_x:.3f}, s1={stage_1_x:.3f}, s2={stage_2_x:.3f}, s3={stage_3_x:.3f}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
