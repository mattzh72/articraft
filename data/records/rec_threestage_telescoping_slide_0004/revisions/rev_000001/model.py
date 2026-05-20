from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

OUTER_LEN = 0.180
OUTER_W = 0.040
OUTER_H = 0.025
OUTER_WALL = 0.0025

MIDDLE_LEN = 0.190
MIDDLE_W = 0.032
MIDDLE_H = 0.019
MIDDLE_WALL = 0.0020

INNER_LEN = 0.190
INNER_W = 0.024
INNER_H = 0.014
INNER_WALL = 0.0018

STAGE_TRAVEL = 0.120

OUTER_INNER_H = OUTER_H - 2.0 * OUTER_WALL
MIDDLE_INNER_H = MIDDLE_H - 2.0 * MIDDLE_WALL
MIDDLE_PAD_T = 0.5 * (OUTER_INNER_H - MIDDLE_H)
INNER_PAD_T = 0.5 * (MIDDLE_INNER_H - INNER_H)
PAD_LEN = 0.050

END_PLATE_T = 0.004
END_PLATE_W = 0.030
END_PLATE_H = 0.022


def _rectangular_tube(length: float, width: float, height: float, wall: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(length, width, height, centered=(False, True, True))
    inner = (
        cq.Workplane("XY")
        .box(
            length + 0.004,
            width - 2.0 * wall,
            height - 2.0 * wall,
            centered=(False, True, True),
        )
        .translate((-0.002, 0.0, 0.0))
    )
    return outer.cut(inner)


def _end_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(END_PLATE_T, END_PLATE_W, END_PLATE_H, centered=(False, True, True))
        .edges("|X")
        .fillet(0.0012)
        .faces("<X")
        .workplane(centerOption="CenterOfMass")
        .hole(0.008)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide", assets=ASSETS)

    anodized = model.material("anodized_aluminum", rgba=(0.62, 0.65, 0.69, 1.0))
    steel = model.material("drawn_steel", rgba=(0.55, 0.58, 0.61, 1.0))
    pad_black = model.material("pad_black", rgba=(0.16, 0.17, 0.18, 1.0))

    outer = model.part("outer_sleeve")
    outer.visual(
        mesh_from_cadquery(
            _rectangular_tube(OUTER_LEN, OUTER_W, OUTER_H, OUTER_WALL),
            "outer_sleeve.obj",
            assets=ASSETS,
        ),
        material=anodized,
        name="sleeve_shell",
    )
    outer.visual(
        Box((0.030, 0.050, 0.003)),
        origin=Origin(xyz=(0.040, 0.0, -OUTER_H / 2.0 - 0.0015)),
        material=steel,
        name="rear_mount_foot",
    )
    outer.visual(
        Box((0.030, 0.050, 0.003)),
        origin=Origin(xyz=(0.140, 0.0, -OUTER_H / 2.0 - 0.0015)),
        material=steel,
        name="front_mount_foot",
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LEN, 0.050, OUTER_H + 0.003)),
        mass=0.85,
        origin=Origin(xyz=(OUTER_LEN / 2.0, 0.0, -0.0015)),
    )

    middle = model.part("middle_stage")
    middle.visual(
        mesh_from_cadquery(
            _rectangular_tube(MIDDLE_LEN, MIDDLE_W, MIDDLE_H, MIDDLE_WALL),
            "middle_stage.obj",
            assets=ASSETS,
        ),
        material=steel,
        name="stage_shell",
    )
    for pad_name, x_pos, z_pos in (
        ("top_pad_rear", 0.040, MIDDLE_H / 2.0 + MIDDLE_PAD_T / 2.0),
        ("bottom_pad_rear", 0.040, -MIDDLE_H / 2.0 - MIDDLE_PAD_T / 2.0),
        ("top_pad_front", 0.120, MIDDLE_H / 2.0 + MIDDLE_PAD_T / 2.0),
        ("bottom_pad_front", 0.120, -MIDDLE_H / 2.0 - MIDDLE_PAD_T / 2.0),
    ):
        middle.visual(
            Box((PAD_LEN, MIDDLE_W * 0.72, MIDDLE_PAD_T)),
            origin=Origin(xyz=(x_pos, 0.0, z_pos)),
            material=pad_black,
            name=pad_name,
        )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LEN, MIDDLE_W, MIDDLE_H + 2.0 * MIDDLE_PAD_T)),
        mass=0.55,
        origin=Origin(xyz=(MIDDLE_LEN / 2.0, 0.0, 0.0)),
    )

    inner = model.part("inner_stage")
    inner.visual(
        mesh_from_cadquery(
            _rectangular_tube(INNER_LEN, INNER_W, INNER_H, INNER_WALL),
            "inner_stage.obj",
            assets=ASSETS,
        ),
        material=anodized,
        name="stage_shell",
    )
    for pad_name, x_pos, z_pos in (
        ("top_pad_rear", 0.040, INNER_H / 2.0 + INNER_PAD_T / 2.0),
        ("bottom_pad_rear", 0.040, -INNER_H / 2.0 - INNER_PAD_T / 2.0),
        ("top_pad_front", 0.120, INNER_H / 2.0 + INNER_PAD_T / 2.0),
        ("bottom_pad_front", 0.120, -INNER_H / 2.0 - INNER_PAD_T / 2.0),
    ):
        inner.visual(
            Box((PAD_LEN, INNER_W * 0.72, INNER_PAD_T)),
            origin=Origin(xyz=(x_pos, 0.0, z_pos)),
            material=pad_black,
            name=pad_name,
        )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LEN, INNER_W, INNER_H + 2.0 * INNER_PAD_T)),
        mass=0.35,
        origin=Origin(xyz=(INNER_LEN / 2.0, 0.0, 0.0)),
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        mesh_from_cadquery(_end_plate_shape(), "end_plate.obj", assets=ASSETS),
        material=steel,
        name="plate_shell",
    )
    end_plate.inertial = Inertial.from_geometry(
        Box((END_PLATE_T, END_PLATE_W, END_PLATE_H)),
        mass=0.08,
        origin=Origin(xyz=(END_PLATE_T / 2.0, 0.0, 0.0)),
    )

    middle_slide = model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=STAGE_TRAVEL,
        ),
    )
    inner_slide = model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_end_plate",
        ArticulationType.FIXED,
        parent=inner,
        child=end_plate,
        origin=Origin(xyz=(INNER_LEN, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    outer = object_model.get_part("outer_sleeve")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    end_plate = object_model.get_part("end_plate")
    middle_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    outer_shell = outer.get_visual("sleeve_shell")
    middle_shell = middle.get_visual("stage_shell")
    inner_shell = inner.get_visual("stage_shell")
    middle_top_pad = middle.get_visual("top_pad_rear")
    inner_top_pad = inner.get_visual("top_pad_rear")
    end_plate_shell = end_plate.get_visual("plate_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=16)

    ctx.check(
        "all_slide_parts_present",
        all(part is not None for part in (outer, middle, inner, end_plate)),
        details="outer sleeve, middle stage, inner stage, and end plate must all exist",
    )

    with ctx.pose({middle_slide: 0.0, inner_slide: 0.0}):
        ctx.expect_contact(
            middle,
            outer,
            elem_a=middle_top_pad,
            elem_b=outer_shell,
            name="middle_stage_pad_bears_on_outer_sleeve",
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a=inner_top_pad,
            elem_b=middle_shell,
            name="inner_stage_pad_bears_on_middle_stage",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="yz",
            min_overlap=0.018,
            name="middle_stage_cross_section_tracks_within_outer",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="yz",
            min_overlap=0.013,
            name="inner_stage_cross_section_tracks_within_middle",
        )
        ctx.expect_gap(
            end_plate,
            inner,
            axis="x",
            max_gap=0.0,
            max_penetration=0.0,
            positive_elem=end_plate_shell,
            negative_elem=inner_shell,
            name="end_plate_seats_on_inner_stage_front",
        )
        ctx.expect_origin_gap(
            middle,
            outer,
            axis="x",
            min_gap=0.0,
            max_gap=0.0,
            name="middle_stage_starts_fully_retracted",
        )
        ctx.expect_origin_gap(
            inner,
            middle,
            axis="x",
            min_gap=0.0,
            max_gap=0.0,
            name="inner_stage_starts_fully_retracted",
        )

    with ctx.pose({middle_slide: STAGE_TRAVEL, inner_slide: STAGE_TRAVEL}):
        ctx.expect_origin_gap(
            middle,
            outer,
            axis="x",
            min_gap=STAGE_TRAVEL - 1e-6,
            max_gap=STAGE_TRAVEL + 1e-6,
            name="middle_stage_reaches_full_travel",
        )
        ctx.expect_origin_gap(
            inner,
            middle,
            axis="x",
            min_gap=STAGE_TRAVEL - 1e-6,
            max_gap=STAGE_TRAVEL + 1e-6,
            name="inner_stage_reaches_full_travel",
        )
        ctx.expect_contact(
            middle,
            outer,
            elem_a=middle_top_pad,
            elem_b=outer_shell,
            name="middle_stage_remains_guided_when_extended",
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a=inner_top_pad,
            elem_b=middle_shell,
            name="inner_stage_remains_guided_when_extended",
        )
        ctx.expect_origin_gap(
            end_plate,
            outer,
            axis="x",
            min_gap=INNER_LEN + 2.0 * STAGE_TRAVEL - 1e-6,
            max_gap=INNER_LEN + 2.0 * STAGE_TRAVEL + 1e-6,
            name="end_plate_advances_with_both_slide_stages",
        )
        ctx.expect_origin_distance(
            end_plate,
            outer,
            axes="yz",
            max_dist=1e-6,
            name="end_plate_stays_axially_aligned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
