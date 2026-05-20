from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 0.42
BASE_W = 0.10
BEAM_H = 0.032
FOOT_LEN = 0.058
FOOT_W = 0.132
FOOT_H = 0.010
RAIL_LEN = 0.34
RAIL_W = 0.022
RAIL_H = 0.008
RAIL_Y = 0.029

SADDLE_LEN = 0.16
SADDLE_W = 0.14
SADDLE_BODY_H = 0.055
SADDLE_MAST_W = 0.118
SADDLE_MAST_D = 0.018
SADDLE_MAST_H = 0.24
SADDLE_GUSSET_X = 0.042
SADDLE_GUSSET_L = 0.024
SADDLE_GUSSET_D = 0.044
SADDLE_GUSSET_H = 0.10
SADDLE_POCKET_L = 0.106
SADDLE_POCKET_W = 0.058
SADDLE_POCKET_H = 0.036
SADDLE_SHOE_LEN = 0.14
SADDLE_SHOE_W = 0.024
SADDLE_SHOE_H = 0.008
SADDLE_TRAVEL = 0.09

GUIDE_X = 0.034
GUIDE_W = 0.020
GUIDE_D = 0.010
GUIDE_Z0 = 0.020
GUIDE_H = 0.21

RAM_PLATE_W = 0.096
RAM_PLATE_D = 0.016
RAM_PLATE_H = 0.18
RAM_PAD_W = 0.122
RAM_PAD_D = 0.020
RAM_PAD_H = 0.030
RAM_HEAD_W = 0.060
RAM_HEAD_D = 0.018
RAM_HEAD_H = 0.018
RAM_WINDOW_W = 0.046
RAM_WINDOW_H = 0.068
RAM_SHOE_W = 0.024
RAM_SHOE_D = 0.014
RAM_SHOE_H = 0.08
RAM_LIFT = 0.12
RAM_ORIGIN_Z = 0.025


def _base_body_shape() -> cq.Workplane:
    beam = (
        cq.Workplane("XY")
        .box(BASE_LEN, BASE_W, BEAM_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
    )
    left_foot = (
        cq.Workplane("XY")
        .box(FOOT_LEN, FOOT_W, FOOT_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((-BASE_LEN / 2.0 + FOOT_LEN / 2.0, 0.0, 0.0))
    )
    right_foot = (
        cq.Workplane("XY")
        .box(FOOT_LEN, FOOT_W, FOOT_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((BASE_LEN / 2.0 - FOOT_LEN / 2.0, 0.0, 0.0))
    )
    return beam.union(left_foot).union(right_foot)


def _saddle_body_shape() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(SADDLE_LEN, SADDLE_W, SADDLE_BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, SADDLE_SHOE_H))
    )
    mast = (
        cq.Workplane("XY")
        .box(SADDLE_MAST_W, SADDLE_MAST_D, SADDLE_MAST_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, SADDLE_W / 2.0 - SADDLE_MAST_D / 2.0, SADDLE_SHOE_H))
    )
    left_gusset = (
        cq.Workplane("YZ")
        .polyline(
            [
                (0.0, 0.0),
                (SADDLE_GUSSET_D, 0.0),
                (SADDLE_GUSSET_D, SADDLE_GUSSET_H),
            ]
        )
        .close()
        .extrude(SADDLE_GUSSET_L)
        .translate(
            (
                -SADDLE_GUSSET_X - SADDLE_GUSSET_L / 2.0,
                SADDLE_W / 2.0 - SADDLE_GUSSET_D - 0.004,
                SADDLE_SHOE_H,
            )
        )
    )
    right_gusset = (
        cq.Workplane("YZ")
        .polyline(
            [
                (0.0, 0.0),
                (SADDLE_GUSSET_D, 0.0),
                (SADDLE_GUSSET_D, SADDLE_GUSSET_H),
            ]
        )
        .close()
        .extrude(SADDLE_GUSSET_L)
        .translate(
            (
                SADDLE_GUSSET_X - SADDLE_GUSSET_L / 2.0,
                SADDLE_W / 2.0 - SADDLE_GUSSET_D - 0.004,
                SADDLE_SHOE_H,
            )
        )
    )
    pocket = (
        cq.Workplane("XY")
        .box(SADDLE_POCKET_L, SADDLE_POCKET_W, SADDLE_POCKET_H, centered=(True, True, False))
        .translate((0.0, 0.0, SADDLE_SHOE_H))
    )
    return deck.union(mast).union(left_gusset).union(right_gusset).cut(pocket)


def _ram_body_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(RAM_PLATE_W, RAM_PLATE_D, RAM_PLATE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, RAM_SHOE_D, 0.0))
    )
    pad = (
        cq.Workplane("XY")
        .box(RAM_PAD_W, RAM_PAD_D, RAM_PAD_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, RAM_SHOE_D + 0.004, 0.0))
    )
    head = (
        cq.Workplane("XY")
        .box(RAM_HEAD_W, RAM_HEAD_D, RAM_HEAD_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.0, RAM_SHOE_D + 0.003, RAM_PLATE_H - RAM_HEAD_H))
    )
    window = (
        cq.Workplane("XY")
        .box(RAM_WINDOW_W, RAM_PLATE_D + 0.006, RAM_WINDOW_H, centered=(True, True, False))
        .translate((0.0, RAM_SHOE_D + 0.003, 0.060))
    )
    return plate.union(pad).union(head).cut(window)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_lift_transfer_stage")

    model.material("base_gray", rgba=(0.28, 0.31, 0.34, 1.0))
    model.material("saddle_gray", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("ram_silver", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("guide_steel", rgba=(0.58, 0.61, 0.66, 1.0))

    base = model.part("base_beam")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "base_beam_body"),
        material="base_gray",
        name="body",
    )
    base.visual(
        Box((RAIL_LEN, RAIL_W, RAIL_H)),
        origin=Origin(xyz=(0.0, -RAIL_Y, BEAM_H + RAIL_H / 2.0)),
        material="guide_steel",
        name="left_rail",
    )
    base.visual(
        Box((RAIL_LEN, RAIL_W, RAIL_H)),
        origin=Origin(xyz=(0.0, RAIL_Y, BEAM_H + RAIL_H / 2.0)),
        material="guide_steel",
        name="right_rail",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, FOOT_W, BEAM_H + RAIL_H)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (BEAM_H + RAIL_H) / 2.0)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_cadquery(_saddle_body_shape(), "saddle_body"),
        material="saddle_gray",
        name="body",
    )
    saddle.visual(
        Box((SADDLE_SHOE_LEN, SADDLE_SHOE_W, SADDLE_SHOE_H)),
        origin=Origin(xyz=(0.0, -RAIL_Y, SADDLE_SHOE_H / 2.0)),
        material="guide_steel",
        name="left_shoe",
    )
    saddle.visual(
        Box((SADDLE_SHOE_LEN, SADDLE_SHOE_W, SADDLE_SHOE_H)),
        origin=Origin(xyz=(0.0, RAIL_Y, SADDLE_SHOE_H / 2.0)),
        material="guide_steel",
        name="right_shoe",
    )
    saddle.visual(
        Box((GUIDE_W, GUIDE_D, GUIDE_H)),
        origin=Origin(
            xyz=(
                -GUIDE_X,
                SADDLE_W / 2.0 + GUIDE_D / 2.0,
                GUIDE_Z0 + GUIDE_H / 2.0,
            )
        ),
        material="guide_steel",
        name="left_guide",
    )
    saddle.visual(
        Box((GUIDE_W, GUIDE_D, GUIDE_H)),
        origin=Origin(
            xyz=(
                GUIDE_X,
                SADDLE_W / 2.0 + GUIDE_D / 2.0,
                GUIDE_Z0 + GUIDE_H / 2.0,
            )
        ),
        material="guide_steel",
        name="right_guide",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((SADDLE_LEN, SADDLE_W + GUIDE_D, SADDLE_MAST_H)),
        mass=3.2,
        origin=Origin(xyz=(0.0, GUIDE_D / 2.0, SADDLE_MAST_H / 2.0)),
    )

    ram = model.part("ram")
    ram.visual(
        mesh_from_cadquery(_ram_body_shape(), "ram_body"),
        material="ram_silver",
        name="body",
    )
    ram.visual(
        Box((RAM_SHOE_W, RAM_SHOE_D, RAM_SHOE_H)),
        origin=Origin(xyz=(-GUIDE_X, RAM_SHOE_D / 2.0, RAM_SHOE_H / 2.0)),
        material="guide_steel",
        name="left_shoe",
    )
    ram.visual(
        Box((RAM_SHOE_W, RAM_SHOE_D, RAM_SHOE_H)),
        origin=Origin(xyz=(GUIDE_X, RAM_SHOE_D / 2.0, RAM_SHOE_H / 2.0)),
        material="guide_steel",
        name="right_shoe",
    )
    ram.inertial = Inertial.from_geometry(
        Box((RAM_PAD_W, RAM_SHOE_D + RAM_PAD_D, RAM_PLATE_H)),
        mass=1.7,
        origin=Origin(
            xyz=(
                0.0,
                (RAM_SHOE_D + RAM_PAD_D) / 2.0,
                RAM_PLATE_H / 2.0,
            )
        ),
    )

    model.articulation(
        "base_to_saddle_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, BEAM_H + RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SADDLE_TRAVEL,
            upper=SADDLE_TRAVEL,
            effort=1200.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "saddle_to_ram_lift",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=ram,
        origin=Origin(
            xyz=(
                0.0,
                SADDLE_W / 2.0 + GUIDE_D,
                RAM_ORIGIN_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=RAM_LIFT,
            effort=900.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_beam")
    saddle = object_model.get_part("saddle")
    ram = object_model.get_part("ram")
    slide_x = object_model.get_articulation("base_to_saddle_slide")
    lift_z = object_model.get_articulation("saddle_to_ram_lift")

    base_left_rail = base.get_visual("left_rail")
    base_right_rail = base.get_visual("right_rail")
    saddle_left_shoe = saddle.get_visual("left_shoe")
    saddle_right_shoe = saddle.get_visual("right_shoe")
    saddle_left_guide = saddle.get_visual("left_guide")
    saddle_right_guide = saddle.get_visual("right_guide")
    ram_left_shoe = ram.get_visual("left_shoe")
    ram_right_shoe = ram.get_visual("right_shoe")

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
        saddle,
        base,
        elem_a=saddle_left_shoe,
        elem_b=base_left_rail,
        name="left_saddle_shoe_seats_on_left_base_rail",
    )
    ctx.expect_contact(
        saddle,
        base,
        elem_a=saddle_right_shoe,
        elem_b=base_right_rail,
        name="right_saddle_shoe_seats_on_right_base_rail",
    )
    ctx.expect_contact(
        ram,
        saddle,
        elem_a=ram_left_shoe,
        elem_b=saddle_left_guide,
        name="left_ram_shoe_contacts_left_vertical_guide",
    )
    ctx.expect_contact(
        ram,
        saddle,
        elem_a=ram_right_shoe,
        elem_b=saddle_right_guide,
        name="right_ram_shoe_contacts_right_vertical_guide",
    )

    rest_saddle_pos = ctx.part_world_position(saddle)
    with ctx.pose({slide_x: SADDLE_TRAVEL}):
        shifted_saddle_pos = ctx.part_world_position(saddle)
        ctx.expect_contact(
            saddle,
            base,
            elem_a=saddle_left_shoe,
            elem_b=base_left_rail,
            name="left_lower_slide_remains_supported_at_positive_x_travel",
        )
        ctx.expect_contact(
            saddle,
            base,
            elem_a=saddle_right_shoe,
            elem_b=base_right_rail,
            name="right_lower_slide_remains_supported_at_positive_x_travel",
        )
    ctx.check(
        "lower_stage_moves_along_positive_x",
        shifted_saddle_pos is not None
        and rest_saddle_pos is not None
        and shifted_saddle_pos[0] > rest_saddle_pos[0] + 0.08,
        details=f"rest={rest_saddle_pos}, shifted={shifted_saddle_pos}",
    )

    rest_ram_pos = ctx.part_world_position(ram)
    with ctx.pose({lift_z: RAM_LIFT}):
        lifted_ram_pos = ctx.part_world_position(ram)
        ctx.expect_contact(
            ram,
            saddle,
            elem_a=ram_left_shoe,
            elem_b=saddle_left_guide,
            name="left_vertical_slide_remains_guided_at_upper_travel",
        )
        ctx.expect_contact(
            ram,
            saddle,
            elem_a=ram_right_shoe,
            elem_b=saddle_right_guide,
            name="right_vertical_slide_remains_guided_at_upper_travel",
        )
    ctx.check(
        "upper_stage_moves_along_positive_z",
        lifted_ram_pos is not None
        and rest_ram_pos is not None
        and lifted_ram_pos[2] > rest_ram_pos[2] + 0.10,
        details=f"rest={rest_ram_pos}, lifted={lifted_ram_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
