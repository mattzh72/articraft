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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
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


def _board_outline() -> list[tuple[float, float]]:
    return [
        (-0.72, -0.190),
        (-0.61, -0.190),
        (-0.42, -0.186),
        (-0.18, -0.176),
        (0.08, -0.162),
        (0.34, -0.142),
        (0.56, -0.110),
        (0.68, -0.076),
        (0.75, -0.040),
        (0.775, 0.0),
        (0.75, 0.040),
        (0.68, 0.076),
        (0.56, 0.110),
        (0.34, 0.142),
        (0.08, 0.162),
        (-0.18, 0.176),
        (-0.42, 0.186),
        (-0.61, 0.190),
        (-0.72, 0.190),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    cover_fabric = model.material("cover_fabric", rgba=(0.86, 0.88, 0.90, 1.0))
    board_metal = model.material("board_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.83, 0.84, 0.86, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    tray_pad = model.material("tray_pad", rgba=(0.18, 0.19, 0.21, 1.0))

    board_outline = _board_outline()
    board_shell = _mesh("ironing_board_cover", ExtrudeGeometry(board_outline, 0.022, center=True))
    board_pan = _mesh(
        "ironing_board_pan",
        ExtrudeGeometry(_scale_profile(board_outline, sx=0.97, sy=0.94), 0.012, center=True),
    )

    board_top = model.part("board_top")
    board_top.visual(
        board_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cover_fabric,
        name="board_cover",
    )
    board_top.visual(
        board_pan,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=board_metal,
        name="board_pan",
    )
    board_top.visual(
        Box((0.26, 0.16, 0.010)),
        origin=Origin(xyz=(-0.53, 0.0, -0.004)),
        material=board_metal,
        name="tail_stiffener",
    )
    board_top.inertial = Inertial.from_geometry(
        Box((1.50, 0.40, 0.040)),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((0.42, 0.394, 0.008)),
        origin=Origin(xyz=(-0.02, 0.0, -0.004)),
        material=dark_steel,
        name="mount_plate",
    )
    base_bracket.visual(
        Box((0.20, 0.12, 0.040)),
        origin=Origin(xyz=(-0.16, 0.0, -0.024)),
        material=dark_steel,
        name="center_saddle",
    )
    base_bracket.visual(
        Box((0.14, 0.18, 0.038)),
        origin=Origin(xyz=(-0.24, 0.0, -0.023)),
        material=dark_steel,
        name="rear_box",
    )
    base_bracket.visual(
        Box((0.040, 0.022, 0.056)),
        origin=Origin(xyz=(-0.015, 0.1835, -0.036)),
        material=painted_steel,
        name="left_pivot_cheek",
    )
    base_bracket.visual(
        Box((0.040, 0.022, 0.056)),
        origin=Origin(xyz=(-0.015, -0.1835, -0.036)),
        material=painted_steel,
        name="right_pivot_cheek",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((0.42, 0.36, 0.070)),
        mass=1.9,
        origin=Origin(xyz=(-0.08, 0.0, -0.026)),
    )

    iron_rest = model.part("iron_rest")
    iron_rest.visual(
        Box((0.15, 0.10, 0.004)),
        origin=Origin(xyz=(-0.020, 0.0, 0.002)),
        material=dark_steel,
        name="mount_base",
    )
    iron_rest.visual(
        Box((0.080, 0.016, 0.042)),
        origin=Origin(xyz=(-0.065, 0.040, 0.022)),
        material=painted_steel,
        name="left_support_arm",
    )
    iron_rest.visual(
        Box((0.080, 0.016, 0.042)),
        origin=Origin(xyz=(-0.065, -0.040, 0.022)),
        material=painted_steel,
        name="right_support_arm",
    )
    iron_rest.visual(
        Box((0.23, 0.12, 0.008)),
        origin=Origin(xyz=(-0.120, 0.0, 0.043)),
        material=tray_pad,
        name="rest_plate",
    )
    iron_rest.visual(
        Box((0.018, 0.12, 0.026)),
        origin=Origin(xyz=(-0.005, 0.0, 0.032)),
        material=painted_steel,
        name="front_lip",
    )
    iron_rest.inertial = Inertial.from_geometry(
        Box((0.24, 0.12, 0.060)),
        mass=0.55,
        origin=Origin(xyz=(-0.095, 0.0, 0.028)),
    )

    leg_frame = model.part("main_leg_frame")
    leg_geom = wire_from_points(
        [
            (-0.180, 0.160, -0.820),
            (-0.080, 0.160, -0.820),
            (-0.080, 0.160, -0.380),
            (0.0, 0.160, 0.0),
            (0.0, -0.160, 0.0),
            (-0.080, -0.160, -0.380),
            (-0.080, -0.160, -0.820),
            (-0.180, -0.160, -0.820),
        ],
        radius=0.0125,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.080,
        corner_segments=12,
    )
    leg_frame.visual(
        _mesh("ironing_board_leg_frame", leg_geom),
        material=painted_steel,
        name="leg_tube",
    )
    leg_frame.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.0, 0.1575, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="left_hinge_barrel",
    )
    leg_frame.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.0, -0.1575, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="right_hinge_barrel",
    )
    leg_frame.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(-0.182, 0.160, -0.820), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="left_foot_cap",
    )
    leg_frame.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(-0.182, -0.160, -0.820), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="right_foot_cap",
    )
    leg_frame.inertial = Inertial.from_geometry(
        Box((0.22, 0.36, 0.84)),
        mass=1.4,
        origin=Origin(xyz=(-0.070, 0.0, -0.410)),
    )

    model.articulation(
        "board_to_bracket",
        ArticulationType.FIXED,
        parent=board_top,
        child=base_bracket,
        origin=Origin(xyz=(-0.03, 0.0, -0.016)),
    )
    model.articulation(
        "board_to_rest",
        ArticulationType.FIXED,
        parent=board_top,
        child=iron_rest,
        origin=Origin(xyz=(-0.63, 0.0, 0.011)),
    )
    model.articulation(
        "bracket_to_leg_frame",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=leg_frame,
        origin=Origin(xyz=(-0.02, 0.0, -0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-1.45,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board_top")
    base_bracket = object_model.get_part("base_bracket")
    iron_rest = object_model.get_part("iron_rest")
    leg_frame = object_model.get_part("main_leg_frame")
    fold_joint = object_model.get_articulation("bracket_to_leg_frame")

    ctx.expect_gap(
        board,
        base_bracket,
        axis="z",
        max_gap=0.0,
        max_penetration=0.0,
        name="base bracket mounts flush under the board",
    )
    ctx.expect_gap(
        iron_rest,
        board,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="iron rest sits cleanly on the tail end of the board",
    )
    ctx.expect_origin_gap(
        board,
        iron_rest,
        axis="x",
        min_gap=0.50,
        name="iron rest is positioned at one end of the board",
    )

    open_leg_aabb = ctx.part_world_aabb(leg_frame)

    ctx.check(
        "open leg trestle reaches realistic floor height",
        open_leg_aabb is not None and open_leg_aabb[0][2] < -0.78,
        details=f"leg_aabb={open_leg_aabb}",
    )

    with ctx.pose({fold_joint: fold_joint.motion_limits.lower}):
        folded_leg_aabb = ctx.part_world_aabb(leg_frame)
        board_aabb = ctx.part_world_aabb(board)
        ctx.expect_within(
            leg_frame,
            board,
            axes="xy",
            margin=0.015,
            name="folded leg fits inside board storage footprint",
        )
        ctx.expect_gap(
            board,
            leg_frame,
            axis="z",
            max_gap=0.060,
            max_penetration=0.0,
            name="folded leg stays tucked close below the board",
        )
        ctx.check(
            "folded leg rises substantially for storage",
            open_leg_aabb is not None
            and folded_leg_aabb is not None
            and folded_leg_aabb[0][2] > open_leg_aabb[0][2] + 0.45
            and board_aabb is not None
            and folded_leg_aabb[0][0] >= board_aabb[0][0] - 0.02,
            details=(
                f"open_leg_aabb={open_leg_aabb}, "
                f"folded_leg_aabb={folded_leg_aabb}, "
                f"board_aabb={board_aabb}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
