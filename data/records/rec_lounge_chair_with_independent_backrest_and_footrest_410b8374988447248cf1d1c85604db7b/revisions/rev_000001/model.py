from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_reclining_lounge_chair")

    wood = model.material("warm_oiled_wood", rgba=(0.55, 0.32, 0.16, 1.0))
    dark_wood = model.material("dark_end_grain", rgba=(0.26, 0.14, 0.07, 1.0))
    canvas = model.material("sand_canvas", rgba=(0.72, 0.66, 0.56, 1.0))
    shadow = model.material("shadowed_rail_gap", rgba=(0.08, 0.07, 0.06, 1.0))

    frame = model.part("frame")

    # Fixed seat deck, slim wooden side frames, and the front/rear rails that
    # tie the two side frames into one supported lounge-chair chassis.
    frame.visual(
        Box((0.70, 0.50, 0.060)),
        origin=Origin(xyz=(0.00, 0.00, 0.430)),
        material=canvas,
        name="seat_deck",
    )
    frame.visual(
        Box((0.080, 0.72, 0.060)),
        origin=Origin(xyz=(0.380, 0.00, 0.370)),
        material=wood,
        name="front_rail",
    )
    frame.visual(
        Box((0.080, 0.72, 0.060)),
        origin=Origin(xyz=(-0.380, 0.00, 0.370)),
        material=wood,
        name="rear_rail",
    )

    for i, y in enumerate((-0.34, 0.34)):
        frame.visual(
            Box((0.92, 0.055, 0.055)),
            origin=Origin(xyz=(0.00, y, 0.620)),
            material=wood,
            name=f"arm_rail_{i}",
        )
        frame.visual(
            Box((0.96, 0.055, 0.055)),
            origin=Origin(xyz=(0.00, y, 0.140)),
            material=wood,
            name=f"floor_runner_{i}",
        )
        frame.visual(
            Box((0.055, 0.055, 0.440)),
            origin=Origin(xyz=(0.420, y, 0.380)),
            material=wood,
            name=f"front_upright_{i}",
        )
        frame.visual(
            Box((0.055, 0.055, 0.440)),
            origin=Origin(xyz=(-0.420, y, 0.380)),
            material=wood,
            name=f"rear_upright_{i}",
        )
        frame.visual(
            Box((0.105, 0.030, 0.090)),
            origin=Origin(xyz=(-0.365, y * 0.93, 0.505)),
            material=dark_wood,
            name=f"rear_pivot_cheek_{i}",
        )
        frame.visual(
            Cylinder(radius=0.036, length=0.030),
            origin=Origin(xyz=(-0.360, y, 0.505), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_wood,
            name=f"pivot_button_{i}",
        )

    # C-channel guide rails below the front of the chair.  The footrest board
    # runs between the vertical webs and is trapped vertically by the lips.
    for i, y_sign in enumerate((-1.0, 1.0)):
        y_web = y_sign * 0.245
        y_lip = y_sign * 0.210
        frame.visual(
            Box((0.78, 0.035, 0.070)),
            origin=Origin(xyz=(0.040, y_web, 0.325)),
            material=shadow,
            name=f"guide_web_{i}",
        )
        frame.visual(
            Box((0.78, 0.080, 0.018)),
            origin=Origin(xyz=(0.040, y_lip, 0.351)),
            material=wood,
            name=f"guide_top_lip_{i}",
        )
        frame.visual(
            Box((0.78, 0.080, 0.018)),
            origin=Origin(xyz=(0.040, y_lip, 0.2915)),
            material=wood,
            name=f"guide_bottom_lip_{i}",
        )

    back = model.part("back")
    back_pitch = -0.30
    back_height = 0.82
    hinge_offset = 0.040
    center_along_panel = back_height / 2.0 + hinge_offset
    panel_center = (
        center_along_panel * math.sin(back_pitch),
        0.0,
        center_along_panel * math.cos(back_pitch),
    )
    back.visual(
        Cylinder(radius=0.025, length=0.608),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_wood,
        name="hinge_barrel",
    )
    back.visual(
        Box((0.080, 0.58, 0.080)),
        origin=Origin(
            xyz=(hinge_offset * math.sin(back_pitch), 0.0, hinge_offset * math.cos(back_pitch)),
            rpy=(0.0, back_pitch, 0.0),
        ),
        material=wood,
        name="lower_back_rail",
    )
    back.visual(
        Box((0.055, 0.54, back_height)),
        origin=Origin(xyz=panel_center, rpy=(0.0, back_pitch, 0.0)),
        material=canvas,
        name="back_panel",
    )
    back.visual(
        Box((0.070, 0.040, back_height)),
        origin=Origin(xyz=(panel_center[0], -0.245, panel_center[2]), rpy=(0.0, back_pitch, 0.0)),
        material=wood,
        name="back_stile_0",
    )
    back.visual(
        Box((0.070, 0.040, back_height)),
        origin=Origin(xyz=(panel_center[0], 0.245, panel_center[2]), rpy=(0.0, back_pitch, 0.0)),
        material=wood,
        name="back_stile_1",
    )
    back.visual(
        Box((0.070, 0.56, 0.055)),
        origin=Origin(xyz=(panel_center[0] * 1.90, 0.0, panel_center[2] * 1.90), rpy=(0.0, back_pitch, 0.0)),
        material=wood,
        name="top_cap",
    )

    footrest = model.part("footrest")
    footrest.visual(
        Box((0.78, 0.410, 0.035)),
        origin=Origin(xyz=(-0.280, 0.0, 0.0)),
        material=wood,
        name="calf_board",
    )
    footrest.visual(
        Box((0.045, 0.470, 0.070)),
        origin=Origin(xyz=(0.132, 0.0, 0.010)),
        material=dark_wood,
        name="front_pull_rail",
    )

    model.articulation(
        "frame_to_back",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back,
        origin=Origin(xyz=(-0.360, 0.0, 0.505)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.8, lower=0.0, upper=0.65),
    )
    model.articulation(
        "frame_to_footrest",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=footrest,
        origin=Origin(xyz=(0.360, 0.0, 0.318)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.42),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    back = object_model.get_part("back")
    footrest = object_model.get_part("footrest")
    back_joint = object_model.get_articulation("frame_to_back")
    slide_joint = object_model.get_articulation("frame_to_footrest")

    # The reclining back is carried on the rear pivot cheeks rather than
    # floating behind the seat.
    ctx.expect_contact(
        frame,
        back,
        elem_a="rear_pivot_cheek_0",
        elem_b="hinge_barrel",
        contact_tol=0.004,
        name="back hinge barrel seats in one pivot cheek",
    )
    ctx.expect_contact(
        frame,
        back,
        elem_a="rear_pivot_cheek_1",
        elem_b="hinge_barrel",
        contact_tol=0.004,
        name="back hinge barrel seats in opposite pivot cheek",
    )

    # At rest the calf board is supported by the lower guide lips, has vertical
    # clearance below the upper lips, and is centered between the guide webs.
    for suffix in ("0", "1"):
        ctx.expect_gap(
            footrest,
            frame,
            axis="z",
            positive_elem="calf_board",
            negative_elem=f"guide_bottom_lip_{suffix}",
            min_gap=0.0,
            max_gap=0.001,
            name=f"calf board rests on lower guide lip {suffix}",
        )
        ctx.expect_gap(
            frame,
            footrest,
            axis="z",
            positive_elem=f"guide_top_lip_{suffix}",
            negative_elem="calf_board",
            min_gap=0.003,
            max_gap=0.010,
            name=f"calf board clears upper retaining lip {suffix}",
        )

    ctx.expect_gap(
        footrest,
        frame,
        axis="y",
        positive_elem="calf_board",
        negative_elem="guide_web_0",
        min_gap=0.015,
        max_gap=0.030,
        name="calf board clears guide web 0",
    )
    ctx.expect_gap(
        frame,
        footrest,
        axis="y",
        positive_elem="guide_web_1",
        negative_elem="calf_board",
        min_gap=0.015,
        max_gap=0.030,
        name="calf board clears guide web 1",
    )
    ctx.expect_overlap(
        footrest,
        frame,
        axes="x",
        elem_a="calf_board",
        elem_b="guide_web_0",
        min_overlap=0.60,
        name="retracted calf board remains deep in the guides",
    )

    rest_board_aabb = ctx.part_element_world_aabb(footrest, elem="calf_board")
    rest_back_aabb = ctx.part_element_world_aabb(back, elem="back_panel")

    with ctx.pose({slide_joint: 0.42}):
        extended_board_aabb = ctx.part_element_world_aabb(footrest, elem="calf_board")
        ctx.expect_overlap(
            footrest,
            frame,
            axes="x",
            elem_a="calf_board",
            elem_b="guide_web_0",
            min_overlap=0.25,
            name="extended calf board is still captured in the guide",
        )
        ctx.expect_gap(
            footrest,
            frame,
            axis="z",
            positive_elem="calf_board",
            negative_elem="guide_bottom_lip_0",
            min_gap=0.0,
            max_gap=0.001,
            name="extended calf board remains supported by a lower lip",
        )
        ctx.expect_gap(
            frame,
            footrest,
            axis="z",
            positive_elem="guide_top_lip_0",
            negative_elem="calf_board",
            min_gap=0.003,
            max_gap=0.010,
            name="extended calf board remains under a retaining lip",
        )

    ctx.check(
        "footrest slides forward from the front rail",
        rest_board_aabb is not None
        and extended_board_aabb is not None
        and extended_board_aabb[1][0] > rest_board_aabb[1][0] + 0.35,
        details=f"rest={rest_board_aabb}, extended={extended_board_aabb}",
    )

    with ctx.pose({back_joint: 0.65}):
        reclined_back_aabb = ctx.part_element_world_aabb(back, elem="back_panel")

    ctx.check(
        "backrest reclines rearward about the rear horizontal pivot",
        rest_back_aabb is not None
        and reclined_back_aabb is not None
        and reclined_back_aabb[0][0] < rest_back_aabb[0][0] - 0.15,
        details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
    )
    return ctx.report()


object_model = build_object_model()
