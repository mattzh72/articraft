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
    mesh_from_geometry,
    tube_from_spline_points,
)


LADDER_HALF_WIDTH = 0.205
RAIL_RADIUS = 0.024
GRIP_RADIUS = 0.019


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _tube_mesh(
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    name: str,
    samples_per_segment: int = 18,
    radial_segments: int = 20,
):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def _add_fold_grip(model: ArticulatedObject, name: str, chrome, rubber):
    grip = model.part(name)
    grip.visual(
        Cylinder(radius=GRIP_RADIUS, length=0.066),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_knuckle",
    )
    grip.visual(
        Box((0.028, 0.020, 0.026)),
        origin=Origin(xyz=(0.014, 0.0, 0.025)),
        material=chrome,
        name="grip_yoke",
    )
    grip.visual(
        _tube_mesh(
            [
                (0.016, 0.0, 0.022),
                (0.020, 0.0, 0.055),
                (0.018, 0.0, 0.130),
                (0.018, 0.0, 0.320),
                (0.012, 0.0, 0.540),
                (0.000, 0.0, 0.565),
            ],
            radius=GRIP_RADIUS,
            name=f"{name}_tube",
            samples_per_segment=16,
            radial_segments=18,
        ),
        material=chrome,
        name="grip_tube",
    )
    grip.visual(
        Cylinder(radius=0.025, length=0.180),
        origin=Origin(xyz=(0.016, 0.0, 0.390)),
        material=rubber,
        name="grip_cover",
    )
    grip.inertial = Inertial.from_geometry(
        Box((0.070, 0.055, 0.580)),
        mass=1.4,
        origin=Origin(xyz=(0.016, 0.0, 0.290)),
    )
    return grip


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pool_access_ladder")

    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    tread_grey = model.material("tread_grey", rgba=(0.40, 0.43, 0.46, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.11, 0.12, 1.0))

    frame = model.part("ladder_frame")

    left_stile_path = [
        (0.050, LADDER_HALF_WIDTH, 0.000),
        (0.050, LADDER_HALF_WIDTH, 0.180),
        (0.020, LADDER_HALF_WIDTH, 0.330),
        (-0.050, LADDER_HALF_WIDTH, 0.470),
        (-0.120, LADDER_HALF_WIDTH, 0.530),
        (-0.160, LADDER_HALF_WIDTH, 0.420),
        (-0.170, LADDER_HALF_WIDTH, 0.180),
        (-0.170, LADDER_HALF_WIDTH, -0.100),
        (-0.170, LADDER_HALF_WIDTH, -0.400),
        (-0.170, LADDER_HALF_WIDTH, -0.860),
    ]
    frame.visual(
        _tube_mesh(left_stile_path, radius=RAIL_RADIUS, name="left_stile_rail_mesh"),
        material=chrome,
        name="left_stile_rail",
    )
    frame.visual(
        _tube_mesh(
            _mirror_y(left_stile_path),
            radius=RAIL_RADIUS,
            name="right_stile_rail_mesh",
        ),
        material=chrome,
        name="right_stile_rail",
    )

    rung_zs = (-0.080, -0.320, -0.560, -0.800)
    for rung_index, rung_z in enumerate(rung_zs, start=1):
        frame.visual(
            Cylinder(radius=0.017, length=2.0 * LADDER_HALF_WIDTH),
            origin=Origin(
                xyz=(-0.170, 0.0, rung_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"rung_{rung_index}",
        )
        frame.visual(
            Cylinder(radius=0.022, length=0.220),
            origin=Origin(
                xyz=(-0.170, 0.0, rung_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=tread_grey,
            name=f"rung_{rung_index}_tread",
        )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        y = side_sign * LADDER_HALF_WIDTH

        frame.visual(
            Cylinder(radius=0.060, length=0.010),
            origin=Origin(xyz=(0.050, y, 0.005)),
            material=chrome,
            name=f"{side_name}_deck_flange",
        )
        frame.visual(
            Cylinder(radius=0.034, length=0.040),
            origin=Origin(xyz=(0.050, y, 0.020)),
            material=chrome,
            name=f"{side_name}_flange_socket",
        )

        for bolt_dx, bolt_dy in ((0.024, 0.020), (-0.024, 0.020), (0.024, -0.020), (-0.024, -0.020)):
            frame.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(0.050 + bolt_dx, y + bolt_dy, 0.011)),
                material=chrome,
            )

        frame.visual(
            Box((0.080, 0.032, 0.010)),
            origin=Origin(xyz=(0.110, y, 0.005)),
            material=chrome,
            name=f"{side_name}_deck_bridge",
        )
        frame.visual(
            Cylinder(radius=0.032, length=0.050),
            origin=Origin(xyz=(0.132, y, 0.025)),
            material=chrome,
            name=f"{side_name}_grip_pedestal",
        )
        frame.visual(
            Cylinder(radius=0.014, length=0.072),
            origin=Origin(xyz=(0.108, y, 0.046)),
            material=chrome,
            name=f"{side_name}_grip_stem",
        )
        frame.visual(
            Box((0.012, 0.092, 0.018)),
            origin=Origin(xyz=(0.108, y, 0.091)),
            material=chrome,
            name=f"{side_name}_hinge_spine",
        )
        frame.visual(
            Box((0.032, 0.014, 0.022)),
            origin=Origin(xyz=(0.124, y - side_sign * 0.046, 0.100)),
            material=chrome,
            name=f"{side_name}_hinge_cheek_a",
        )
        frame.visual(
            Box((0.032, 0.014, 0.022)),
            origin=Origin(xyz=(0.124, y + side_sign * 0.046, 0.100)),
            material=chrome,
            name=f"{side_name}_hinge_cheek_b",
        )
        frame.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(
                xyz=(0.140, y - side_sign * 0.042, 0.100),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"{side_name}_hinge_barrel_a",
        )
        frame.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(
                xyz=(0.140, y + side_sign * 0.042, 0.100),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"{side_name}_hinge_barrel_b",
        )

    frame.inertial = Inertial.from_geometry(
        Box((0.370, 0.540, 1.360)),
        mass=22.0,
        origin=Origin(xyz=(-0.015, 0.0, -0.130)),
    )

    left_grip = _add_fold_grip(model, "left_grip", chrome, grip_black)
    right_grip = _add_fold_grip(model, "right_grip", chrome, grip_black)

    model.articulation(
        "frame_to_left_grip",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_grip,
        origin=Origin(xyz=(0.140, LADDER_HALF_WIDTH, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=pi / 2.0,
        ),
    )
    model.articulation(
        "frame_to_right_grip",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_grip,
        origin=Origin(xyz=(0.140, -LADDER_HALF_WIDTH, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=pi / 2.0,
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

    frame = object_model.get_part("ladder_frame")
    left_grip = object_model.get_part("left_grip")
    right_grip = object_model.get_part("right_grip")
    left_joint = object_model.get_articulation("frame_to_left_grip")
    right_joint = object_model.get_articulation("frame_to_right_grip")

    ctx.expect_origin_distance(
        left_grip,
        right_grip,
        axes="y",
        min_dist=0.39,
        max_dist=0.43,
        name="fold grips straddle the ladder opening",
    )

    ctx.expect_contact(
        left_grip,
        frame,
        elem_a="pivot_knuckle",
        elem_b="left_hinge_barrel_a",
        name="left grip knuckle meets first hinge barrel",
    )
    ctx.expect_contact(
        left_grip,
        frame,
        elem_a="pivot_knuckle",
        elem_b="left_hinge_barrel_b",
        name="left grip knuckle meets second hinge barrel",
    )
    ctx.expect_contact(
        right_grip,
        frame,
        elem_a="pivot_knuckle",
        elem_b="right_hinge_barrel_a",
        name="right grip knuckle meets first hinge barrel",
    )
    ctx.expect_contact(
        right_grip,
        frame,
        elem_a="pivot_knuckle",
        elem_b="right_hinge_barrel_b",
        name="right grip knuckle meets second hinge barrel",
    )

    ctx.expect_gap(
        left_grip,
        frame,
        axis="z",
        min_gap=0.05,
        positive_elem="grip_tube",
        negative_elem="left_deck_flange",
        name="left grip stands above the deck flange when upright",
    )
    ctx.expect_gap(
        right_grip,
        frame,
        axis="z",
        min_gap=0.05,
        positive_elem="grip_tube",
        negative_elem="right_deck_flange",
        name="right grip stands above the deck flange when upright",
    )

    left_limits = left_joint.motion_limits
    right_limits = right_joint.motion_limits
    ctx.check(
        "grip motion limits run from upright to horizontal",
        left_limits is not None
        and right_limits is not None
        and abs((left_limits.lower or 0.0) - 0.0) < 1e-6
        and abs((right_limits.lower or 0.0) - 0.0) < 1e-6
        and left_limits.upper is not None
        and right_limits.upper is not None
        and abs(left_limits.upper - pi / 2.0) < 1e-6
        and abs(right_limits.upper - pi / 2.0) < 1e-6,
        details=f"left={left_limits}, right={right_limits}",
    )

    left_closed_aabb = ctx.part_element_world_aabb(left_grip, elem="grip_tube")
    right_closed_aabb = ctx.part_element_world_aabb(right_grip, elem="grip_tube")
    with ctx.pose({left_joint: pi / 2.0, right_joint: pi / 2.0}):
        left_open_aabb = ctx.part_element_world_aabb(left_grip, elem="grip_tube")
        right_open_aabb = ctx.part_element_world_aabb(right_grip, elem="grip_tube")

    def _aabb_ok(aabb) -> bool:
        return aabb is not None and len(aabb) == 2

    ctx.check(
        "left grip folds out over the deck",
        _aabb_ok(left_closed_aabb)
        and _aabb_ok(left_open_aabb)
        and left_open_aabb[1][0] > left_closed_aabb[1][0] + 0.32
        and left_open_aabb[1][2] < left_closed_aabb[1][2] - 0.28,
        details=f"closed={left_closed_aabb}, open={left_open_aabb}",
    )
    ctx.check(
        "right grip folds out over the deck",
        _aabb_ok(right_closed_aabb)
        and _aabb_ok(right_open_aabb)
        and right_open_aabb[1][0] > right_closed_aabb[1][0] + 0.32
        and right_open_aabb[1][2] < right_closed_aabb[1][2] - 0.28,
        details=f"closed={right_closed_aabb}, open={right_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
