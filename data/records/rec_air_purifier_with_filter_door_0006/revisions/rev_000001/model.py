from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)

ASSETS = AssetContext.from_script(__file__)

WIDTH = 0.56
DEPTH = 0.24
HEIGHT = 0.18
WALL = 0.018
TOP_THICKNESS = 0.020
FRONT_FRAME_DEPTH = 0.018
REAR_FRAME_DEPTH = 0.018
REAR_GRILLE_FRAME = 0.014
REAR_GRILLE_FRAME_DEPTH = 0.006
REAR_GRILLE_DEPTH = 0.004
REAR_GRILLE_CAVITY = 0.006
REAR_GRILLE_BACKER_DEPTH = 0.003
REAR_GRILLE_BOTTOM = 0.028
REAR_GRILLE_TOP = HEIGHT - 0.026
PANEL_CLEARANCE = 0.002
PANEL_THICKNESS = 0.012
SEAT_THICKNESS = 0.006
SEAT_OVERLAP = 0.004
PANEL_WIDTH = WIDTH - (2.0 * WALL) - (2.0 * PANEL_CLEARANCE)
PANEL_DEPTH = DEPTH - FRONT_FRAME_DEPTH - REAR_FRAME_DEPTH - (2.0 * PANEL_CLEARANCE)
HINGE_Y = (-DEPTH / 2.0) + REAR_FRAME_DEPTH + PANEL_CLEARANCE
REAR_OPENING_WIDTH = (WIDTH - (2.0 * WALL)) - (2.0 * REAR_GRILLE_FRAME)
REAR_OPENING_HEIGHT = REAR_GRILLE_TOP - REAR_GRILLE_BOTTOM


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mid_size_air_purifier", assets=ASSETS)

    shell = model.material("shell", rgba=(0.86, 0.87, 0.84, 1.0))
    accent = model.material("accent", rgba=(0.19, 0.21, 0.23, 1.0))
    panel_color = model.material("panel_color", rgba=(0.28, 0.30, 0.33, 1.0))

    body = model.part("body")
    body.visual(
        Box((WIDTH, DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - (TOP_THICKNESS / 2.0))),
        material=shell,
        name="top_shell",
    )
    body.visual(
        Box((WALL, DEPTH, HEIGHT - TOP_THICKNESS)),
        origin=Origin(xyz=(-(WIDTH / 2.0) + (WALL / 2.0), 0.0, (HEIGHT - TOP_THICKNESS) / 2.0)),
        material=shell,
        name="left_side",
    )
    body.visual(
        Box((WALL, DEPTH, HEIGHT - TOP_THICKNESS)),
        origin=Origin(xyz=((WIDTH / 2.0) - (WALL / 2.0), 0.0, (HEIGHT - TOP_THICKNESS) / 2.0)),
        material=shell,
        name="right_side",
    )
    body.visual(
        Box((WIDTH - (2.0 * WALL), FRONT_FRAME_DEPTH, HEIGHT - TOP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, (DEPTH / 2.0) - (FRONT_FRAME_DEPTH / 2.0), (HEIGHT - TOP_THICKNESS) / 2.0)
        ),
        material=shell,
        name="front_fascia",
    )
    body.visual(
        Box((WIDTH - (2.0 * WALL), REAR_FRAME_DEPTH, 0.028)),
        origin=Origin(xyz=(0.0, -(DEPTH / 2.0) + (REAR_FRAME_DEPTH / 2.0), 0.014)),
        material=shell,
        name="rear_bottom_rail",
    )
    body.visual(
        Box((REAR_GRILLE_FRAME, REAR_GRILLE_FRAME_DEPTH, REAR_OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                -((WIDTH - (2.0 * WALL)) / 2.0) + (REAR_GRILLE_FRAME / 2.0),
                -(DEPTH / 2.0) + (REAR_GRILLE_FRAME_DEPTH / 2.0),
                REAR_GRILLE_BOTTOM + (REAR_OPENING_HEIGHT / 2.0),
            )
        ),
        material=shell,
        name="rear_left_upright",
    )
    body.visual(
        Box((REAR_GRILLE_FRAME, REAR_GRILLE_FRAME_DEPTH, REAR_OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                ((WIDTH - (2.0 * WALL)) / 2.0) - (REAR_GRILLE_FRAME / 2.0),
                -(DEPTH / 2.0) + (REAR_GRILLE_FRAME_DEPTH / 2.0),
                REAR_GRILLE_BOTTOM + (REAR_OPENING_HEIGHT / 2.0),
            )
        ),
        material=shell,
        name="rear_right_upright",
    )
    body.visual(
        Box((REAR_OPENING_WIDTH, REAR_GRILLE_BACKER_DEPTH, REAR_OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(DEPTH / 2.0)
                + REAR_GRILLE_DEPTH
                + REAR_GRILLE_CAVITY
                + (REAR_GRILLE_BACKER_DEPTH / 2.0),
                REAR_GRILLE_BOTTOM + (REAR_OPENING_HEIGHT / 2.0),
            )
        ),
        material=accent,
        name="rear_intake_backer",
    )
    body.visual(
        Box((0.008, PANEL_DEPTH, SEAT_THICKNESS)),
        origin=Origin(xyz=(-(PANEL_WIDTH / 2.0) + 0.002, HINGE_Y + (PANEL_DEPTH / 2.0), SEAT_THICKNESS / 2.0)),
        material=shell,
        name="left_panel_seat",
    )
    body.visual(
        Box((0.008, PANEL_DEPTH, SEAT_THICKNESS)),
        origin=Origin(xyz=((PANEL_WIDTH / 2.0) - 0.002, HINGE_Y + (PANEL_DEPTH / 2.0), SEAT_THICKNESS / 2.0)),
        material=shell,
        name="right_panel_seat",
    )
    body.visual(
        Box((PANEL_WIDTH, 0.008, SEAT_THICKNESS)),
        origin=Origin(
            xyz=(0.0, HINGE_Y + PANEL_DEPTH + (0.004 - PANEL_CLEARANCE), SEAT_THICKNESS / 2.0)
        ),
        material=shell,
        name="front_panel_seat",
    )

    slat_width = REAR_OPENING_WIDTH
    slat_height = 0.008
    slat_depth = REAR_GRILLE_DEPTH
    first_slat_z = 0.038
    slat_pitch = 0.016
    for index in range(8):
        body.visual(
            Box((slat_width, slat_depth, slat_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    -(DEPTH / 2.0) + (slat_depth / 2.0),
                    first_slat_z + (index * slat_pitch),
                )
            ),
            material=accent,
            name=f"rear_grille_slat_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((WIDTH, DEPTH, HEIGHT)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, HEIGHT / 2.0)),
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((PANEL_WIDTH, PANEL_DEPTH, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, PANEL_DEPTH / 2.0, -(PANEL_THICKNESS / 2.0))),
        material=panel_color,
        name="panel_leaf",
    )
    access_panel.visual(
        Box((0.13, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, PANEL_DEPTH - 0.008, -(PANEL_THICKNESS + 0.004))),
        material=accent,
        name="pull_lip",
    )
    access_panel.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, PANEL_DEPTH, PANEL_THICKNESS)),
        mass=1.2,
        origin=Origin(xyz=(0.0, PANEL_DEPTH / 2.0, -(PANEL_THICKNESS / 2.0))),
    )

    model.articulation(
        "body_to_access_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=access_panel,
        origin=Origin(xyz=(0.0, HINGE_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.2, lower=-1.35, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    access_panel = object_model.get_part("access_panel")
    panel_hinge = object_model.get_articulation("body_to_access_panel")

    top_shell = body.get_visual("top_shell")
    left_side = body.get_visual("left_side")
    right_side = body.get_visual("right_side")
    front_fascia = body.get_visual("front_fascia")
    rear_bottom_rail = body.get_visual("rear_bottom_rail")
    rear_left_upright = body.get_visual("rear_left_upright")
    rear_right_upright = body.get_visual("rear_right_upright")
    left_panel_seat = body.get_visual("left_panel_seat")
    right_panel_seat = body.get_visual("right_panel_seat")
    front_panel_seat = body.get_visual("front_panel_seat")
    rear_intake_backer = body.get_visual("rear_intake_backer")
    bottom_slat = body.get_visual("rear_grille_slat_0")
    mid_slat = body.get_visual("rear_grille_slat_3")
    top_slat = body.get_visual("rear_grille_slat_7")

    panel_leaf = access_panel.get_visual("panel_leaf")
    pull_lip = access_panel.get_visual("pull_lip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_within(access_panel, body, axes="xy", inner_elem=panel_leaf, outer_elem=top_shell)
    ctx.expect_gap(
        body,
        access_panel,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_panel_seat,
        negative_elem=panel_leaf,
        name="left_side_of_panel_sits_flush",
    )
    ctx.expect_gap(
        body,
        access_panel,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_panel_seat,
        negative_elem=panel_leaf,
        name="right_side_of_panel_sits_flush",
    )
    ctx.expect_gap(
        body,
        access_panel,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=front_panel_seat,
        negative_elem=panel_leaf,
        name="front_of_panel_sits_flush",
    )
    ctx.expect_contact(body, access_panel, elem_a=left_panel_seat, elem_b=panel_leaf)
    ctx.expect_contact(body, access_panel, elem_a=front_panel_seat, elem_b=panel_leaf)

    ctx.expect_gap(
        body,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rear_left_upright,
        negative_elem=left_side,
        name="rear_grille_frame_meets_left_side_shell",
    )
    ctx.expect_gap(
        body,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_side,
        negative_elem=rear_right_upright,
        name="rear_grille_frame_meets_right_side_shell",
    )
    ctx.expect_gap(
        body,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=mid_slat,
        negative_elem=rear_left_upright,
        name="rear_grille_spans_between_left_frame_and_center",
    )
    ctx.expect_gap(
        body,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rear_right_upright,
        negative_elem=mid_slat,
        name="rear_grille_spans_between_center_and_right_frame",
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        max_gap=0.008,
        max_penetration=0.0,
        positive_elem=bottom_slat,
        negative_elem=rear_bottom_rail,
        name="rear_grille_runs_low_on_back_face",
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        max_gap=0.008,
        max_penetration=0.0,
        positive_elem=top_shell,
        negative_elem=top_slat,
        name="rear_grille_runs_high_on_back_face",
    )
    ctx.expect_gap(
        body,
        body,
        axis="y",
        min_gap=0.18,
        positive_elem=front_fascia,
        negative_elem=mid_slat,
        name="rear_intake_grille_is_on_the_back_face",
    )
    ctx.expect_gap(
        body,
        body,
        axis="y",
        min_gap=0.001,
        positive_elem=rear_intake_backer,
        negative_elem=mid_slat,
        name="rear_intake_backer_is_recessed_behind_grille",
    )
    ctx.expect_within(
        body,
        body,
        axes="xz",
        inner_elem=mid_slat,
        outer_elem=rear_intake_backer,
        name="rear_intake_backer_spans_the_grille_opening",
    )

    with ctx.pose({panel_hinge: -1.15}):
        ctx.expect_gap(
            body,
            access_panel,
            axis="z",
            min_gap=0.08,
            positive_elem=front_panel_seat,
            negative_elem=pull_lip,
            name="bottom_panel_swings_down_for_filter_access",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
