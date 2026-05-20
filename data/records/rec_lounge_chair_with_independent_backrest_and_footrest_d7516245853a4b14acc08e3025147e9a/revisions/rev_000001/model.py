from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="travel_recliner")

    frame_metal = model.material("frame_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    deck_metal = model.material("deck_metal", rgba=(0.36, 0.38, 0.40, 1.0))
    seat_upholstery = model.material("seat_upholstery", rgba=(0.53, 0.45, 0.36, 1.0))
    back_upholstery = model.material("back_upholstery", rgba=(0.48, 0.41, 0.33, 1.0))
    foot_upholstery = model.material("foot_upholstery", rgba=(0.46, 0.39, 0.32, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.13, 0.14, 0.16, 1.0))

    side_frame = model.part("side_frame")
    side_frame.inertial = Inertial.from_geometry(
        Box((0.74, 0.66, 0.76)),
        mass=18.0,
        origin=Origin(xyz=(0.03, 0.0, 0.19)),
    )

    # Seat pan and its rigid support deck.
    side_frame.visual(
        Box((0.46, 0.54, 0.055)),
        origin=Origin(xyz=(0.02, 0.0, 0.3275)),
        material=seat_upholstery,
        name="seat_pan",
    )
    side_frame.visual(
        Box((0.43, 0.50, 0.025)),
        origin=Origin(xyz=(0.02, 0.0, 0.2875)),
        material=deck_metal,
        name="seat_deck",
    )

    # Left and right rigid side frame members.
    for side_y in (-0.30, 0.30):
        side_frame.visual(
            Box((0.62, 0.04, 0.055)),
            origin=Origin(xyz=(0.05, side_y, 0.0525)),
            material=frame_metal,
            name=f"lower_rail_{'left' if side_y > 0 else 'right'}",
        )
        side_frame.visual(
            Box((0.56, 0.04, 0.06)),
            origin=Origin(xyz=(0.02, side_y, 0.315)),
            material=frame_metal,
            name=f"upper_rail_{'left' if side_y > 0 else 'right'}",
        )
        side_frame.visual(
            Box((0.04, 0.04, 0.27)),
            origin=Origin(xyz=(0.28, side_y, 0.175)),
            material=frame_metal,
            name=f"front_upright_{'left' if side_y > 0 else 'right'}",
        )
        side_frame.visual(
            Box((0.04, 0.04, 0.40)),
            origin=Origin(xyz=(-0.21, side_y, 0.20), rpy=(0.0, -0.10, 0.0)),
            material=frame_metal,
            name=f"rear_upright_{'left' if side_y > 0 else 'right'}",
        )

    # Cross-members tying the two side frames together.
    side_frame.visual(
        Box((0.05, 0.54, 0.06)),
        origin=Origin(xyz=(0.22, 0.0, 0.295)),
        material=frame_metal,
        name="front_crossmember",
    )
    side_frame.visual(
        Box((0.05, 0.54, 0.06)),
        origin=Origin(xyz=(-0.17, 0.0, 0.295)),
        material=frame_metal,
        name="rear_crossmember",
    )
    side_frame.visual(
        Box((0.05, 0.06, 0.04)),
        origin=Origin(xyz=(0.235, -0.255, 0.255)),
        material=deck_metal,
        name="foot_hinge_beam_right",
    )
    side_frame.visual(
        Box((0.05, 0.06, 0.04)),
        origin=Origin(xyz=(0.235, 0.255, 0.255)),
        material=deck_metal,
        name="foot_hinge_beam_left",
    )

    # Compact side hinge brackets leave clearance for the rotating child parts.
    for side_y in (-0.285, 0.285):
        side_frame.visual(
            Box((0.03, 0.05, 0.05)),
            origin=Origin(xyz=(-0.24, side_y, 0.355)),
            material=hinge_dark,
            name=f"back_hinge_lug_{'left' if side_y > 0 else 'right'}",
        )
        side_frame.visual(
            Box((0.03, 0.05, 0.05)),
            origin=Origin(xyz=(0.245, side_y, 0.255)),
            material=hinge_dark,
            name=f"foot_hinge_lug_{'left' if side_y > 0 else 'right'}",
        )

    backrest = model.part("backrest")
    backrest.inertial = Inertial.from_geometry(
        Box((0.10, 0.56, 0.66)),
        mass=6.8,
        origin=Origin(xyz=(-0.03, 0.0, 0.31)),
    )
    backrest.visual(
        Cylinder(radius=0.014, length=0.52),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="back_hinge_tube",
    )
    backrest.visual(
        Box((0.05, 0.46, 0.035)),
        origin=Origin(xyz=(-0.025, 0.0, 0.025), rpy=(0.0, -0.12, 0.0)),
        material=deck_metal,
        name="back_lower_frame",
    )
    for side_y in (-0.235, 0.235):
        backrest.visual(
            Box((0.026, 0.03, 0.56)),
            origin=Origin(xyz=(-0.075, side_y, 0.295), rpy=(0.0, -0.16, 0.0)),
            material=frame_metal,
            name=f"back_side_stile_{'left' if side_y > 0 else 'right'}",
        )
    backrest.visual(
        Box((0.025, 0.44, 0.03)),
        origin=Origin(xyz=(-0.10, 0.0, 0.565), rpy=(0.0, -0.16, 0.0)),
        material=frame_metal,
        name="back_top_rail",
    )
    backrest.visual(
        Box((0.07, 0.50, 0.56)),
        origin=Origin(xyz=(-0.075, 0.0, 0.31), rpy=(0.0, -0.16, 0.0)),
        material=back_upholstery,
        name="back_cushion",
    )

    foot_panel = model.part("foot_panel")
    foot_panel.inertial = Inertial.from_geometry(
        Box((0.06, 0.46, 0.24)),
        mass=2.2,
        origin=Origin(xyz=(0.01, 0.0, -0.12)),
    )
    foot_panel.visual(
        Cylinder(radius=0.013, length=0.40),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="foot_hinge_tube",
    )
    foot_panel.visual(
        Box((0.05, 0.44, 0.22)),
        origin=Origin(xyz=(0.01, 0.0, -0.12)),
        material=foot_upholstery,
        name="foot_board",
    )
    foot_panel.visual(
        Box((0.03, 0.40, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=deck_metal,
        name="foot_upper_frame",
    )

    model.articulation(
        "side_frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=side_frame,
        child=backrest,
        origin=Origin(xyz=(-0.24, 0.0, 0.355)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.6,
            lower=-0.20,
            upper=0.95,
        ),
    )
    model.articulation(
        "side_frame_to_foot_panel",
        ArticulationType.REVOLUTE,
        parent=side_frame,
        child=foot_panel,
        origin=Origin(xyz=(0.245, 0.0, 0.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.30,
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

    side_frame = object_model.get_part("side_frame")
    backrest = object_model.get_part("backrest")
    foot_panel = object_model.get_part("foot_panel")
    back_hinge = object_model.get_articulation("side_frame_to_backrest")
    foot_hinge = object_model.get_articulation("side_frame_to_foot_panel")

    ctx.check("side frame present", side_frame is not None)
    ctx.check("backrest present", backrest is not None)
    ctx.check("foot panel present", foot_panel is not None)

    with ctx.pose({back_hinge: 0.0, foot_hinge: 0.0}):
        ctx.expect_overlap(
            backrest,
            side_frame,
            axes="y",
            min_overlap=0.48,
            elem_a="back_cushion",
            elem_b="seat_pan",
            name="backrest matches the seat width",
        )
        ctx.expect_gap(
            backrest,
            side_frame,
            axis="z",
            max_gap=0.03,
            max_penetration=0.008,
            positive_elem="back_cushion",
            negative_elem="seat_pan",
            name="backrest starts at the rear edge of the seat",
        )
        ctx.expect_within(
            foot_panel,
            side_frame,
            axes="y",
            margin=0.05,
            inner_elem="foot_board",
            outer_elem="seat_pan",
            name="stowed foot panel stays within the seat width",
        )
        ctx.expect_gap(
            side_frame,
            foot_panel,
            axis="z",
            min_gap=0.03,
            max_gap=0.09,
            positive_elem="seat_pan",
            negative_elem="foot_board",
            name="stowed foot panel hangs below the seat pan",
        )

        rest_back_aabb = ctx.part_element_world_aabb(backrest, elem="back_cushion")
        rest_foot_aabb = ctx.part_element_world_aabb(foot_panel, elem="foot_board")

    with ctx.pose({back_hinge: back_hinge.motion_limits.upper}):
        reclined_back_aabb = ctx.part_element_world_aabb(backrest, elem="back_cushion")

    with ctx.pose({foot_hinge: foot_hinge.motion_limits.upper}):
        deployed_foot_aabb = ctx.part_element_world_aabb(foot_panel, elem="foot_board")
        ctx.expect_within(
            foot_panel,
            side_frame,
            axes="y",
            margin=0.05,
            inner_elem="foot_board",
            outer_elem="seat_pan",
            name="deployed foot panel stays aligned with the chair width",
        )

    ctx.check(
        "backrest reclines rearward",
        rest_back_aabb is not None
        and reclined_back_aabb is not None
        and reclined_back_aabb[0][0] < rest_back_aabb[0][0] - 0.10,
        details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
    )
    ctx.check(
        "foot panel swings forward",
        rest_foot_aabb is not None
        and deployed_foot_aabb is not None
        and deployed_foot_aabb[1][0] > rest_foot_aabb[1][0] + 0.12,
        details=f"rest={rest_foot_aabb}, deployed={deployed_foot_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
