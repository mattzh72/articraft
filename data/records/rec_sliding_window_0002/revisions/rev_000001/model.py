from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_W = 1.24
FRAME_H = 0.76
FRAME_D = 0.10

JAMB_W = 0.08
HEAD_H = 0.08
SILL_H = 0.10

TRACK_W = 1.02
TRACK_Y = -0.022
TRACK_DEPTH = 0.030
TRACK_LIP_DEPTH = 0.006
TRACK_PLATE_T = 0.012
TRACK_LIP_H = 0.028
TRACK_FLOOR_Z = -0.274
TRACK_ROOF_Z = 0.274

SASH_W = 0.56
SASH_H = 0.52
SASH_D = 0.020
SASH_CLOSED_X = 0.24
SASH_Y = -0.022
SLIDE_TRAVEL = 0.32


def _add_screw_head(
    part,
    *,
    name: str,
    xyz: tuple[float, float, float],
    radius: float,
    length: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_sliding_window", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.31, 0.35, 0.33, 1.0))
    sash_paint = model.material("sash_paint", rgba=(0.24, 0.27, 0.28, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.56, 0.66, 0.72, 0.38))
    seal_black = model.material("seal_black", rgba=(0.09, 0.09, 0.10, 1.0))
    hardware = model.material("hardware", rgba=(0.72, 0.74, 0.77, 1.0))
    roller_polymer = model.material("roller_polymer", rgba=(0.14, 0.14, 0.14, 1.0))
    handle_polymer = model.material("handle_polymer", rgba=(0.17, 0.18, 0.19, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((JAMB_W, FRAME_D, FRAME_H)),
        origin=Origin(xyz=(-0.58, 0.0, 0.0)),
        material=frame_paint,
        name="left_jamb",
    )
    frame.visual(
        Box((JAMB_W, FRAME_D, FRAME_H)),
        origin=Origin(xyz=(0.58, 0.0, 0.0)),
        material=frame_paint,
        name="right_jamb",
    )
    frame.visual(
        Box((FRAME_W, FRAME_D, HEAD_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=frame_paint,
        name="head",
    )
    frame.visual(
        Box((FRAME_W, FRAME_D, SILL_H)),
        origin=Origin(xyz=(0.0, 0.0, -0.33)),
        material=frame_paint,
        name="sill",
    )

    for name, xyz in (
        ("corner_block_tl", (-0.515, 0.022, 0.285)),
        ("corner_block_tr", (0.515, 0.022, 0.285)),
        ("corner_block_bl", (-0.515, 0.022, -0.275)),
        ("corner_block_br", (0.515, 0.022, -0.275)),
    ):
        frame.visual(
            Box((0.11, 0.034, 0.11)),
            origin=Origin(xyz=xyz),
            material=frame_paint,
            name=name,
        )

    frame.visual(
        Box((0.07, 0.042, 0.60)),
        origin=Origin(xyz=(-0.05, 0.016, 0.0)),
        material=frame_paint,
        name="meeting_post",
    )
    frame.visual(
        Box((0.034, 0.014, 0.50)),
        origin=Origin(xyz=(-0.018, -0.006, 0.0)),
        material=frame_paint,
        name="mullion_rear_land",
    )
    frame.visual(
        Box((0.010, 0.004, 0.44)),
        origin=Origin(xyz=(-0.014, -0.003, 0.0)),
        material=seal_black,
        name="meeting_seal_strip",
    )
    frame.visual(
        Box((0.060, 0.010, 0.44)),
        origin=Origin(xyz=(-0.05, 0.041, 0.0)),
        material=frame_paint,
        name="meeting_service_plate",
    )

    frame.visual(
        Box((TRACK_W, TRACK_DEPTH, TRACK_PLATE_T)),
        origin=Origin(xyz=(0.0, TRACK_Y, TRACK_FLOOR_Z)),
        material=frame_paint,
        name="rear_track_floor",
    )
    frame.visual(
        Box((TRACK_W, TRACK_LIP_DEPTH, TRACK_LIP_H)),
        origin=Origin(xyz=(0.0, -0.004, -0.254)),
        material=frame_paint,
        name="rear_track_front_lip",
    )
    frame.visual(
        Box((TRACK_W, TRACK_LIP_DEPTH, TRACK_LIP_H)),
        origin=Origin(xyz=(0.0, -0.040, -0.254)),
        material=frame_paint,
        name="rear_track_back_lip",
    )
    frame.visual(
        Box((TRACK_W, TRACK_DEPTH, TRACK_PLATE_T)),
        origin=Origin(xyz=(0.0, TRACK_Y, TRACK_ROOF_Z)),
        material=frame_paint,
        name="rear_track_roof",
    )
    frame.visual(
        Box((TRACK_W, TRACK_LIP_DEPTH, TRACK_LIP_H)),
        origin=Origin(xyz=(0.0, -0.004, 0.254)),
        material=frame_paint,
        name="rear_track_front_keeper",
    )
    frame.visual(
        Box((TRACK_W, TRACK_LIP_DEPTH, TRACK_LIP_H)),
        origin=Origin(xyz=(0.0, -0.040, 0.254)),
        material=frame_paint,
        name="rear_track_back_keeper",
    )

    frame.visual(
        Box((0.40, 0.005, 0.44)),
        origin=Origin(xyz=(-0.312, 0.018, 0.0)),
        material=glass_tint,
        name="fixed_glass",
    )
    frame.visual(
        Box((0.028, 0.006, 0.44)),
        origin=Origin(xyz=(-0.526, 0.018, 0.0)),
        material=seal_black,
        name="fixed_left_gasket",
    )
    frame.visual(
        Box((0.027, 0.006, 0.44)),
        origin=Origin(xyz=(-0.0985, 0.018, 0.0)),
        material=seal_black,
        name="fixed_right_gasket",
    )
    frame.visual(
        Box((0.40, 0.006, 0.06)),
        origin=Origin(xyz=(-0.312, 0.018, -0.25)),
        material=seal_black,
        name="fixed_bottom_gasket",
    )
    frame.visual(
        Box((0.40, 0.006, 0.08)),
        origin=Origin(xyz=(-0.312, 0.018, 0.26)),
        material=seal_black,
        name="fixed_top_gasket",
    )

    for idx, xyz in enumerate(
        (
            (-0.47, 0.052, 0.325),
            (0.47, 0.052, 0.325),
            (-0.47, 0.052, -0.29),
            (0.47, 0.052, -0.29),
            (-0.05, 0.052, 0.18),
            (-0.05, 0.052, -0.18),
            (0.545, 0.052, 0.18),
            (0.545, 0.052, -0.18),
        )
    ):
        _add_screw_head(
            frame,
            name=f"frame_screw_{idx}",
            xyz=(xyz[0], 0.048, xyz[2]),
            radius=0.0085,
            length=0.006,
            material=hardware,
        )

    frame.inertial = Inertial.from_geometry(Box((FRAME_W, FRAME_D, FRAME_H)), mass=28.0)

    sash = model.part("sash")
    sash.visual(
        Box((0.045, SASH_D, SASH_H)),
        origin=Origin(xyz=(-0.2575, 0.0, 0.0)),
        material=sash_paint,
        name="meeting_stile",
    )
    sash.visual(
        Box((0.055, SASH_D, SASH_H)),
        origin=Origin(xyz=(0.2525, 0.0, 0.0)),
        material=sash_paint,
        name="latch_stile",
    )
    sash.visual(
        Box((SASH_W, SASH_D, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.2375)),
        material=sash_paint,
        name="top_rail",
    )
    sash.visual(
        Box((SASH_W, 0.022, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.2325)),
        material=sash_paint,
        name="bottom_rail",
    )

    for name, xyz in (
        ("sash_corner_reinforce_tl", (-0.215, 0.0, 0.205)),
        ("sash_corner_reinforce_tr", (0.215, 0.0, 0.205)),
        ("sash_corner_reinforce_bl", (-0.215, 0.0, -0.205)),
        ("sash_corner_reinforce_br", (0.215, 0.0, -0.205)),
    ):
        sash.visual(
            Box((0.075, 0.018, 0.075)),
            origin=Origin(xyz=xyz),
            material=sash_paint,
            name=name,
        )

    sash.visual(
        Box((0.426, 0.005, 0.394)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=glass_tint,
        name="sash_glass",
    )
    sash.visual(
        Box((0.022, 0.006, 0.394)),
        origin=Origin(xyz=(-0.224, 0.0, 0.005)),
        material=seal_black,
        name="glass_gasket_left",
    )
    sash.visual(
        Box((0.012, 0.006, 0.394)),
        origin=Origin(xyz=(0.219, 0.0, 0.005)),
        material=seal_black,
        name="glass_gasket_right",
    )
    sash.visual(
        Box((0.426, 0.006, 0.013)),
        origin=Origin(xyz=(0.0, 0.0, 0.2085)),
        material=seal_black,
        name="glass_gasket_top",
    )
    sash.visual(
        Box((0.426, 0.006, 0.013)),
        origin=Origin(xyz=(0.0, 0.0, -0.1985)),
        material=seal_black,
        name="glass_gasket_bottom",
    )
    sash.visual(
        Box((0.016, 0.004, 0.42)),
        origin=Origin(xyz=(-0.248, 0.012, 0.0)),
        material=seal_black,
        name="sash_meeting_seal",
    )

    sash.visual(
        Box((0.080, 0.018, 0.032)),
        origin=Origin(xyz=(-0.18, 0.0, -0.242)),
        material=sash_paint,
        name="left_roller_housing",
    )
    sash.visual(
        Box((0.080, 0.018, 0.032)),
        origin=Origin(xyz=(0.18, 0.0, -0.242)),
        material=sash_paint,
        name="right_roller_housing",
    )
    sash.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(-0.18, 0.0, -0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_polymer,
        name="left_roller",
    )
    sash.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.18, 0.0, -0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_polymer,
        name="right_roller",
    )
    sash.visual(
        Box((0.040, 0.010, 0.016)),
        origin=Origin(xyz=(-0.18, 0.0, 0.260)),
        material=roller_polymer,
        name="left_top_guide",
    )
    sash.visual(
        Box((0.040, 0.010, 0.016)),
        origin=Origin(xyz=(0.18, 0.0, 0.260)),
        material=roller_polymer,
        name="right_top_guide",
    )

    sash.visual(
        Box((0.10, 0.012, 0.050)),
        origin=Origin(xyz=(0.18, 0.012, 0.0)),
        material=handle_polymer,
        name="pull_base",
    )
    sash.visual(
        Box((0.06, 0.018, 0.018)),
        origin=Origin(xyz=(0.18, 0.024, 0.0)),
        material=handle_polymer,
        name="pull_grip",
    )

    for idx, xyz in enumerate(((-0.18, 0.010, -0.226), (0.18, 0.010, -0.226))):
        _add_screw_head(
            sash,
            name=f"roller_housing_screw_{idx}",
            xyz=xyz,
            radius=0.0055,
            length=0.003,
            material=hardware,
        )

    sash.inertial = Inertial.from_geometry(Box((SASH_W, 0.030, SASH_H)), mass=10.0)

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(SASH_CLOSED_X, SASH_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.60,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("frame_to_sash")

    rear_track_floor = frame.get_visual("rear_track_floor")
    rear_track_roof = frame.get_visual("rear_track_roof")
    fixed_glass = frame.get_visual("fixed_glass")
    mullion_rear_land = frame.get_visual("mullion_rear_land")
    meeting_seal_strip = frame.get_visual("meeting_seal_strip")

    left_roller = sash.get_visual("left_roller")
    right_roller = sash.get_visual("right_roller")
    left_top_guide = sash.get_visual("left_top_guide")
    right_top_guide = sash.get_visual("right_top_guide")
    meeting_stile = sash.get_visual("meeting_stile")
    sash_meeting_seal = sash.get_visual("sash_meeting_seal")
    sash_glass = sash.get_visual("sash_glass")

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

    ctx.expect_gap(
        sash,
        frame,
        axis="z",
        positive_elem=left_roller,
        negative_elem=rear_track_floor,
        max_gap=0.0005,
        max_penetration=0.0,
        name="left roller sits on rear lower track",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="z",
        positive_elem=right_roller,
        negative_elem=rear_track_floor,
        max_gap=0.0005,
        max_penetration=0.0,
        name="right roller sits on rear lower track",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="z",
        positive_elem=rear_track_roof,
        negative_elem=left_top_guide,
        max_gap=0.0005,
        max_penetration=0.0,
        name="left top guide runs against upper keeper",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="z",
        positive_elem=rear_track_roof,
        negative_elem=right_top_guide,
        max_gap=0.0005,
        max_penetration=0.0,
        name="right top guide runs against upper keeper",
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="xz",
        elem_a=meeting_stile,
        elem_b=mullion_rear_land,
        min_overlap=0.03,
        name="closed sash overlaps reinforced meeting land",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="y",
        positive_elem=meeting_seal_strip,
        negative_elem=sash_meeting_seal,
        min_gap=0.002,
        max_gap=0.006,
        name="seal break remains visible between frame and sash seals",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="y",
        positive_elem=fixed_glass,
        negative_elem=sash_glass,
        min_gap=0.030,
        max_gap=0.045,
        name="fixed lite and sliding sash occupy separate depth channels",
    )

    closed_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        open_pos = ctx.part_world_position(sash)
        ctx.check(
            "sash moves laterally along the frame",
            closed_pos is not None
            and open_pos is not None
            and open_pos[0] < closed_pos[0] - 0.30
            and abs(open_pos[1] - closed_pos[1]) < 1e-6
            and abs(open_pos[2] - closed_pos[2]) < 1e-6,
            details=f"closed={closed_pos}, open={open_pos}",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem=left_roller,
            negative_elem=rear_track_floor,
            max_gap=0.0005,
            max_penetration=0.0,
            name="left roller stays seated when open",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem=right_roller,
            negative_elem=rear_track_floor,
            max_gap=0.0005,
            max_penetration=0.0,
            name="right roller stays seated when open",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            positive_elem=rear_track_roof,
            negative_elem=left_top_guide,
            max_gap=0.0005,
            max_penetration=0.0,
            name="left top guide stays captured when open",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            positive_elem=rear_track_roof,
            negative_elem=right_top_guide,
            max_gap=0.0005,
            max_penetration=0.0,
            name="right top guide stays captured when open",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="x",
            inner_elem=left_roller,
            outer_elem=rear_track_floor,
            margin=0.0,
            name="left roller remains within lower rail span",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="x",
            inner_elem=right_roller,
            outer_elem=rear_track_floor,
            margin=0.0,
            name="right roller remains within lower rail span",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="xz",
            elem_a=sash_glass,
            elem_b=fixed_glass,
            min_overlap=0.18,
            name="open sash parks behind the fixed lite",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
