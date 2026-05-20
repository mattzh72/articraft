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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _scale_profile(
    profile: list[tuple[float, float]],
    *,
    sx: float = 1.0,
    sy: float = 1.0,
) -> list[tuple[float, float]]:
    return [(x * sx, y * sy) for x, y in profile]


def _board_profile() -> list[tuple[float, float]]:
    controls = [
        (-0.71, -0.17),
        (-0.58, -0.19),
        (-0.28, -0.19),
        (0.06, -0.17),
        (0.34, -0.13),
        (0.55, -0.08),
        (0.69, 0.00),
        (0.55, 0.08),
        (0.34, 0.13),
        (0.06, 0.17),
        (-0.28, 0.19),
        (-0.58, 0.19),
        (-0.71, 0.17),
    ]
    return sample_catmull_rom_spline_2d(
        controls,
        samples_per_segment=10,
        closed=True,
        alpha=0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    powder_white = model.material("powder_white", rgba=(0.95, 0.95, 0.94, 1.0))
    cover_blue = model.material("cover_blue", rgba=(0.54, 0.69, 0.86, 1.0))
    padding_white = model.material("padding_white", rgba=(0.94, 0.95, 0.96, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.69, 0.71, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    foot_black = model.material("foot_black", rgba=(0.10, 0.10, 0.11, 1.0))

    board_profile = _board_profile()
    steel_pan_mesh = _mesh(
        "ironing_board_pan",
        ExtrudeGeometry.from_z0(board_profile, 0.006),
    )
    pad_mesh = _mesh(
        "ironing_board_pad",
        ExtrudeGeometry.from_z0(_scale_profile(board_profile, sx=0.985, sy=0.955), 0.013),
    )

    underframe = model.part("underframe")
    underframe.inertial = Inertial.from_geometry(
        Box((1.18, 0.26, 0.12)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )
    underframe.visual(
        Box((0.24, 0.18, 0.018)),
        origin=Origin(xyz=(-0.59, 0.0, -0.009)),
        material=steel_gray,
        name="rear_mount_plate",
    )
    underframe.visual(
        Box((0.84, 0.028, 0.022)),
        origin=Origin(xyz=(-0.05, 0.095, -0.011)),
        material=steel_gray,
        name="left_rail",
    )
    underframe.visual(
        Box((0.84, 0.028, 0.022)),
        origin=Origin(xyz=(-0.05, -0.095, -0.011)),
        material=steel_gray,
        name="right_rail",
    )
    underframe.visual(
        Box((0.36, 0.18, 0.018)),
        origin=Origin(xyz=(-0.01, 0.0, -0.009)),
        material=steel_gray,
        name="center_bridge",
    )
    underframe.visual(
        Box((0.28, 0.11, 0.018)),
        origin=Origin(xyz=(0.49, 0.0, -0.009)),
        material=steel_gray,
        name="nose_bridge",
    )
    underframe.visual(
        Box((0.22, 0.07, 0.018)),
        origin=Origin(xyz=(0.26, 0.0, -0.009)),
        material=steel_gray,
        name="center_spine",
    )
    underframe.visual(
        Box((0.12, 0.18, 0.042)),
        origin=Origin(xyz=(-0.03, 0.0, -0.043)),
        material=dark_steel,
        name="hinge_saddle",
    )
    underframe.visual(
        Box((0.050, 0.040, 0.032)),
        origin=Origin(xyz=(-0.03, -0.110, -0.050)),
        material=dark_steel,
        name="left_hinge_knuckle",
    )
    underframe.visual(
        Box((0.050, 0.040, 0.032)),
        origin=Origin(xyz=(-0.03, 0.110, -0.050)),
        material=dark_steel,
        name="right_hinge_knuckle",
    )
    underframe.visual(
        Box((0.06, 0.12, 0.050)),
        origin=Origin(xyz=(0.56, 0.0, -0.034)),
        material=dark_steel,
        name="front_rest_block",
    )

    board_top = model.part("board_top")
    board_top.inertial = Inertial.from_geometry(
        Box((1.42, 0.38, 0.020)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    board_top.visual(
        steel_pan_mesh,
        material=powder_white,
        name="steel_pan",
    )
    board_top.visual(
        pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=padding_white,
        name="padding",
    )
    board_top.visual(
        _mesh(
            "ironing_board_cover",
            ExtrudeGeometry.from_z0(_scale_profile(board_profile, sx=0.975, sy=0.945), 0.0022),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=cover_blue,
        name="cover",
    )

    main_leg_frame = model.part("main_leg_frame")
    main_leg_frame.inertial = Inertial.from_geometry(
        Box((0.36, 0.34, 0.86)),
        mass=2.4,
        origin=Origin(xyz=(-0.15, 0.0, -0.43)),
    )
    main_leg_frame.visual(
        _mesh(
            "ironing_board_main_leg",
            wire_from_points(
                [
                    (0.0, -0.15, 0.0),
                    (-0.30, -0.15, -0.82),
                    (-0.30, 0.15, -0.82),
                    (0.0, 0.15, 0.0),
                ],
                radius=0.015,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.055,
                corner_segments=12,
            ),
        ),
        material=steel_gray,
        name="leg_loop",
    )
    main_leg_frame.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(0.0, -0.15, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_hinge_sleeve",
    )
    main_leg_frame.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(0.0, 0.15, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_hinge_sleeve",
    )
    main_leg_frame.visual(
        Box((0.055, 0.28, 0.020)),
        origin=Origin(xyz=(-0.30, 0.0, -0.830)),
        material=foot_black,
        name="foot_bar_sleeve",
    )

    underframe.visual(
        _mesh(
            "ironing_board_end_rest",
            wire_from_points(
                [
                    (0.0, -0.09, 0.0),
                    (0.11, -0.09, -0.79),
                    (0.11, 0.09, -0.79),
                    (0.0, 0.09, 0.0),
                ],
                radius=0.012,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.05,
                corner_segments=10,
            ),
        ),
        origin=Origin(xyz=(0.56, 0.0, -0.009)),
        material=steel_gray,
        name="rest_loop",
    )
    underframe.visual(
        Box((0.038, 0.21, 0.024)),
        origin=Origin(xyz=(0.56, 0.0, -0.021)),
        material=dark_steel,
        name="rest_head",
    )
    underframe.visual(
        Box((0.035, 0.035, 0.018)),
        origin=Origin(xyz=(0.67, -0.09, -0.808)),
        material=foot_black,
        name="left_foot",
    )
    underframe.visual(
        Box((0.035, 0.035, 0.018)),
        origin=Origin(xyz=(0.67, 0.09, -0.808)),
        material=foot_black,
        name="right_foot",
    )

    model.articulation(
        "top_mount",
        ArticulationType.FIXED,
        parent=underframe,
        child=board_top,
        origin=Origin(),
    )
    model.articulation(
        "main_leg_fold",
        ArticulationType.REVOLUTE,
        parent=underframe,
        child=main_leg_frame,
        origin=Origin(xyz=(-0.03, 0.0, -0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=0.0,
            upper=1.60,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board_top = object_model.get_part("board_top")
    main_leg_frame = object_model.get_part("main_leg_frame")
    main_leg_fold = object_model.get_articulation("main_leg_fold")
    folded_limit = main_leg_fold.motion_limits.upper or 1.80

    ctx.expect_gap(
        board_top,
        main_leg_frame,
        axis="z",
        positive_elem="steel_pan",
        negative_elem="foot_bar_sleeve",
        min_gap=0.78,
        max_gap=0.88,
        name="open stance gives the board a realistic working height",
    )

    rest_aabb = ctx.part_world_aabb(main_leg_frame)
    with ctx.pose({main_leg_fold: folded_limit}):
        ctx.expect_gap(
            board_top,
            main_leg_frame,
            axis="z",
            positive_elem="steel_pan",
            negative_elem="leg_loop",
            max_gap=0.14,
            max_penetration=0.0,
            name="folded leg tucks close beneath the board",
        )
        folded_aabb = ctx.part_world_aabb(main_leg_frame)

    ctx.check(
        "main leg swings upward toward the nose when folded",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] > rest_aabb[0][2] + 0.50
        and folded_aabb[1][0] > rest_aabb[1][0] + 0.55,
        details=f"rest_aabb={rest_aabb}, folded_aabb={folded_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
