from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import sqrt

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
    wire_from_points,
)


def _board_outline() -> list[tuple[float, float]]:
    return [
        (-0.585, -0.185),
        (-0.460, -0.190),
        (-0.210, -0.188),
        (0.040, -0.175),
        (0.260, -0.150),
        (0.430, -0.110),
        (0.550, -0.060),
        (0.615, 0.000),
        (0.550, 0.060),
        (0.430, 0.110),
        (0.260, 0.150),
        (0.040, 0.175),
        (-0.210, 0.188),
        (-0.460, 0.190),
        (-0.585, 0.185),
    ]


def _aabb_distance(aabb_a, aabb_b) -> float | None:
    if aabb_a is None or aabb_b is None:
        return None
    dist_sq = 0.0
    for axis in range(3):
        a_min = aabb_a[0][axis]
        a_max = aabb_a[1][axis]
        b_min = aabb_b[0][axis]
        b_max = aabb_b[1][axis]
        if a_max < b_min:
            gap = b_min - a_max
        elif b_max < a_min:
            gap = a_min - b_max
        else:
            gap = 0.0
        dist_sq += gap * gap
    return sqrt(dist_sq)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ironing_board")

    cover = model.material("cover", rgba=(0.74, 0.84, 0.92, 1.0))
    board_shell = model.material("board_shell", rgba=(0.90, 0.92, 0.93, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    board_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_board_outline(), height=0.012, cap=True, closed=True),
        "ironing_board_shell",
    )
    cover_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_board_outline(), height=0.008, cap=True, closed=True),
        "ironing_board_cover",
    )

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((0.760, 0.240, 0.014)),
        origin=Origin(xyz=(-0.125, 0.000, 0.811)),
        material=dark_steel,
        name="mount_plate",
    )
    base_bracket.visual(
        Box((0.620, 0.060, 0.040)),
        origin=Origin(xyz=(-0.110, 0.000, 0.784)),
        material=frame_steel,
        name="center_spine",
    )
    base_bracket.visual(
        Box((0.030, 0.080, 0.038)),
        origin=Origin(xyz=(0.180, 0.000, 0.785)),
        material=frame_steel,
        name="hinge_saddle",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((0.760, 0.240, 0.080)),
        mass=3.2,
        origin=Origin(xyz=(-0.125, 0.000, 0.804)),
    )

    board_top = model.part("board_top")
    board_top.visual(
        board_mesh,
        material=board_shell,
        name="board_shell",
    )
    board_top.visual(
        cover_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=cover,
        name="board_cover",
    )
    board_top.visual(
        Box((0.320, 0.120, 0.008)),
        origin=Origin(xyz=(-0.420, 0.000, 0.004)),
        material=board_shell,
        name="tail_reinforcement",
    )
    board_top.inertial = Inertial.from_geometry(
        Box((1.240, 0.390, 0.022)),
        mass=4.5,
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
    )

    fixed_rest = model.part("fixed_rest")
    fixed_rest.visual(
        mesh_from_geometry(
            wire_from_points(
                [
                    (0.000, -0.120, -0.789),
                    (0.000, -0.120, -0.052),
                    (0.000, 0.120, -0.052),
                    (0.000, 0.120, -0.789),
                ],
                radius=0.013,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.040,
                corner_segments=10,
            ),
            "ironing_board_fixed_rest_u",
        ),
        material=frame_steel,
        name="rest_frame",
    )
    fixed_rest.visual(
        Box((0.040, 0.050, 0.104)),
        origin=Origin(xyz=(0.000, -0.095, -0.052)),
        material=dark_steel,
        name="left_mount_block",
    )
    fixed_rest.visual(
        Box((0.040, 0.050, 0.104)),
        origin=Origin(xyz=(0.000, 0.095, -0.052)),
        material=dark_steel,
        name="right_mount_block",
    )
    fixed_rest.visual(
        Box((0.050, 0.030, 0.020)),
        origin=Origin(xyz=(0.000, -0.120, -0.794)),
        material=rubber,
        name="left_foot",
    )
    fixed_rest.visual(
        Box((0.050, 0.030, 0.020)),
        origin=Origin(xyz=(0.000, 0.120, -0.794)),
        material=rubber,
        name="right_foot",
    )
    fixed_rest.inertial = Inertial.from_geometry(
        Box((0.100, 0.280, 0.804)),
        mass=1.2,
        origin=Origin(xyz=(0.000, 0.000, -0.402)),
    )

    main_leg = model.part("main_leg")
    main_leg_geom = wire_from_points(
        [
            (0.016, -0.092, -0.045),
            (-0.090, -0.092, -0.790),
            (-0.090, 0.092, -0.790),
            (0.016, 0.092, -0.045),
        ],
        radius=0.012,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.055,
        corner_segments=12,
    )
    main_leg.visual(
        mesh_from_geometry(main_leg_geom, "ironing_board_main_leg_frame"),
        material=frame_steel,
        name="leg_frame",
    )
    main_leg.visual(
        Box((0.030, 0.192, 0.024)),
        origin=Origin(xyz=(-0.052, 0.000, -0.450)),
        material=dark_steel,
        name="center_brace",
    )
    main_leg.visual(
        Box((0.040, 0.028, 0.090)),
        origin=Origin(xyz=(0.020, -0.092, -0.045)),
        material=dark_steel,
        name="left_hinge_ear",
    )
    main_leg.visual(
        Box((0.040, 0.028, 0.090)),
        origin=Origin(xyz=(0.020, 0.092, -0.045)),
        material=dark_steel,
        name="right_hinge_ear",
    )
    main_leg.visual(
        Box((0.058, 0.030, 0.020)),
        origin=Origin(xyz=(-0.078, -0.092, -0.794)),
        material=rubber,
        name="left_foot",
    )
    main_leg.visual(
        Box((0.058, 0.030, 0.020)),
        origin=Origin(xyz=(-0.078, 0.092, -0.794)),
        material=rubber,
        name="right_foot",
    )
    main_leg.inertial = Inertial.from_geometry(
        Box((0.260, 0.220, 0.820)),
        mass=1.9,
        origin=Origin(xyz=(-0.040, 0.000, -0.410)),
    )

    model.articulation(
        "bracket_to_board",
        ArticulationType.FIXED,
        parent=base_bracket,
        child=board_top,
        origin=Origin(xyz=(0.000, 0.000, 0.818)),
    )
    model.articulation(
        "bracket_to_fixed_rest",
        ArticulationType.FIXED,
        parent=base_bracket,
        child=fixed_rest,
        origin=Origin(xyz=(-0.470, 0.000, 0.804)),
    )
    model.articulation(
        "main_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=main_leg,
        origin=Origin(xyz=(0.180, 0.000, 0.804)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.10,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board_top = object_model.get_part("board_top")
    fixed_rest = object_model.get_part("fixed_rest")
    main_leg = object_model.get_part("main_leg")
    hinge = object_model.get_articulation("main_leg_hinge")

    ctx.expect_gap(
        board_top,
        fixed_rest,
        axis="z",
        max_gap=0.020,
        max_penetration=0.0,
        name="fixed rest seats just below the board",
    )
    ctx.expect_gap(
        board_top,
        main_leg,
        axis="z",
        max_gap=0.020,
        max_penetration=0.0,
        name="open leg frame seats just below the board",
    )

    open_leg_aabb = ctx.part_world_aabb(main_leg)
    open_rest_aabb = ctx.part_world_aabb(fixed_rest)
    open_distance = _aabb_distance(open_leg_aabb, open_rest_aabb)

    with ctx.pose({hinge: hinge.motion_limits.upper}):
        ctx.expect_gap(
            board_top,
            main_leg,
            axis="z",
            max_gap=0.085,
            max_penetration=0.0,
            name="folded leg stays tucked below the board",
        )
        ctx.expect_overlap(
            main_leg,
            fixed_rest,
            axes="y",
            min_overlap=0.180,
            name="folded leg remains aligned with the fixed rest",
        )
        folded_leg_aabb = ctx.part_world_aabb(main_leg)
        folded_rest_aabb = ctx.part_world_aabb(fixed_rest)
        folded_distance = _aabb_distance(folded_leg_aabb, folded_rest_aabb)

    ctx.check(
        "folded leg nests near the fixed rest",
        open_distance is not None
        and folded_distance is not None
        and folded_distance < 0.080
        and folded_distance < open_distance * 0.45,
        details=f"open_distance={open_distance}, folded_distance={folded_distance}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
