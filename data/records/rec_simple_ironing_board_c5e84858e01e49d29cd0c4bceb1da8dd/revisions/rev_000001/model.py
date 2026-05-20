from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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


def _scale_profile(
    profile: list[tuple[float, float]],
    *,
    sx: float = 1.0,
    sy: float = 1.0,
) -> list[tuple[float, float]]:
    return [(x * sx, y * sy) for x, y in profile]


def _board_profile() -> list[tuple[float, float]]:
    control = [
        (-0.620, 0.000),
        (-0.585, 0.135),
        (-0.490, 0.192),
        (-0.220, 0.200),
        (0.060, 0.176),
        (0.300, 0.128),
        (0.500, 0.082),
        (0.625, 0.000),
        (0.500, -0.082),
        (0.300, -0.128),
        (0.060, -0.176),
        (-0.220, -0.200),
        (-0.490, -0.192),
        (-0.585, -0.135),
    ]
    return sample_catmull_rom_spline_2d(
        control,
        samples_per_segment=10,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ironing_board")

    fabric = model.material("fabric", rgba=(0.73, 0.79, 0.86, 1.0))
    shell = model.material("shell", rgba=(0.90, 0.90, 0.92, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))

    board_base = model.part("board_base")
    board_profile = _board_profile()
    board_pad = mesh_from_geometry(
        ExtrudeGeometry.from_z0(board_profile, 0.018),
        "ironing_board_pad",
    )
    board_shell = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_scale_profile(board_profile, sx=0.985, sy=0.965), 0.010),
        "ironing_board_shell",
    )

    board_base.visual(
        board_pad,
        origin=Origin(xyz=(0.0, 0.0, 0.804)),
        material=fabric,
        name="board_pad",
    )
    board_base.visual(
        board_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.794)),
        material=shell,
        name="board_shell",
    )
    board_base.visual(
        Box((0.220, 0.132, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.787)),
        material=steel,
        name="base_bracket",
    )
    board_base.visual(
        Box((0.018, 0.315, 0.070)),
        origin=Origin(xyz=(-0.020, 0.0, 0.745)),
        material=steel,
        name="left_hinge_cheek",
    )
    board_base.visual(
        Box((0.018, 0.315, 0.070)),
        origin=Origin(xyz=(0.020, 0.0, 0.745)),
        material=steel,
        name="right_hinge_cheek",
    )
    board_base.visual(
        Box((0.095, 0.050, 0.042)),
        origin=Origin(xyz=(0.430, 0.0, 0.779)),
        material=steel,
        name="nose_mount",
    )
    board_base.inertial = Inertial.from_geometry(
        Box((1.260, 0.400, 0.070)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
    )

    leg_frame = model.part("leg_frame")
    leg_loop = mesh_from_geometry(
        wire_from_points(
            [
                (0.0, -0.150, 0.0),
                (0.0, -0.220, -0.320),
                (0.0, -0.205, -0.742),
                (0.0, 0.205, -0.742),
                (0.0, 0.220, -0.320),
                (0.0, 0.150, 0.0),
            ],
            radius=0.010,
            radial_segments=18,
            closed_path=True,
            cap_ends=False,
            corner_mode="fillet",
            corner_radius=0.060,
            corner_segments=10,
        ),
        "ironing_board_leg_frame",
    )
    leg_frame.visual(
        leg_loop,
        material=steel,
        name="leg_loop",
    )
    leg_frame.visual(
        Box((0.022, 0.160, 0.022)),
        origin=Origin(),
        material=steel,
        name="hinge_sleeve",
    )
    leg_frame.inertial = Inertial.from_geometry(
        Box((0.050, 0.500, 0.760)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.370)),
    )

    nose_rest = model.part("nose_rest")
    rest_loop = mesh_from_geometry(
        wire_from_points(
            [
                (0.0, -0.082, 0.0),
                (0.0, -0.072, -0.250),
                (0.0, -0.066, -0.738),
                (0.0, 0.066, -0.738),
                (0.0, 0.072, -0.250),
                (0.0, 0.082, 0.0),
            ],
            radius=0.008,
            radial_segments=16,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=0.035,
            corner_segments=9,
        ),
        "ironing_board_nose_rest",
    )
    nose_rest.visual(
        rest_loop,
        material=steel,
        name="rest_loop",
    )
    nose_rest.visual(
        Box((0.060, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="rest_saddle",
    )
    nose_rest.inertial = Inertial.from_geometry(
        Box((0.030, 0.180, 0.750)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.369)),
    )

    model.articulation(
        "board_to_leg_frame",
        ArticulationType.REVOLUTE,
        parent=board_base,
        child=leg_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.748)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.3,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "board_to_nose_rest",
        ArticulationType.FIXED,
        parent=board_base,
        child=nose_rest,
        origin=Origin(xyz=(0.438, 0.0, 0.750)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board_base = object_model.get_part("board_base")
    leg_frame = object_model.get_part("leg_frame")
    nose_rest = object_model.get_part("nose_rest")
    leg_hinge = object_model.get_articulation("board_to_leg_frame")

    ctx.expect_contact(
        leg_frame,
        board_base,
        elem_a="hinge_sleeve",
        elem_b="left_hinge_cheek",
        name="leg hinge sleeve bears on left cheek",
    )
    ctx.expect_contact(
        leg_frame,
        board_base,
        elem_a="hinge_sleeve",
        elem_b="right_hinge_cheek",
        name="leg hinge sleeve bears on right cheek",
    )
    ctx.expect_contact(
        nose_rest,
        board_base,
        elem_a="rest_saddle",
        elem_b="nose_mount",
        name="fixed nose rest is mounted on the nose bracket",
    )

    rest_leg_aabb = ctx.part_world_aabb(leg_frame)
    shell_aabb = ctx.part_element_world_aabb(board_base, elem="board_shell")

    folded_leg_aabb = None
    with ctx.pose({leg_hinge: leg_hinge.motion_limits.upper}):
        folded_leg_aabb = ctx.part_world_aabb(leg_frame)

    ctx.check(
        "leg frame folds back under the board",
        rest_leg_aabb is not None
        and folded_leg_aabb is not None
        and folded_leg_aabb[0][0] < rest_leg_aabb[0][0] - 0.20,
        details=f"rest_aabb={rest_leg_aabb}, folded_aabb={folded_leg_aabb}",
    )
    ctx.check(
        "folded leg stays below the board shell",
        folded_leg_aabb is not None
        and shell_aabb is not None
        and folded_leg_aabb[1][2] <= shell_aabb[0][2] - 0.015,
        details=f"folded_aabb={folded_leg_aabb}, shell_aabb={shell_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
