from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _tube_bundle(
    paths: list[list[tuple[float, float, float]]],
    *,
    radius: float,
    name: str,
    radial_segments: int = 14,
) :
    geom = None
    for pts in paths:
        tube = tube_from_spline_points(
            pts,
            radius=radius,
            samples_per_segment=2 if len(pts) == 2 else 10,
            radial_segments=radial_segments,
            cap_ends=True,
        )
        if geom is None:
            geom = tube
        else:
            geom.merge(tube)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    steel = model.material("powder_coated_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    gray_cap = model.material("gray_cap", rgba=(0.48, 0.50, 0.53, 1.0))

    tube_r = 0.008
    top_z = 0.92
    half_depth = 0.21
    half_width = 0.39
    wing_span = 0.27

    center_frame = model.part("center_frame")
    center_frame.inertial = Inertial.from_geometry(
        Box((1.06, 0.62, 1.00)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    upper_paths = [
        [(-half_width, -half_depth, top_z), (half_width, -half_depth, top_z)],
        [(-half_width, half_depth, top_z), (half_width, half_depth, top_z)],
        [(-half_width, -half_depth, top_z), (-half_width, half_depth, top_z)],
        [(half_width, -half_depth, top_z), (half_width, half_depth, top_z)],
        [(-half_width, -0.14, top_z), (half_width, -0.14, top_z)],
        [(-half_width, -0.07, top_z), (half_width, -0.07, top_z)],
        [(-half_width, 0.00, top_z), (half_width, 0.00, top_z)],
        [(-half_width, 0.07, top_z), (half_width, 0.07, top_z)],
        [(-half_width, 0.14, top_z), (half_width, 0.14, top_z)],
        [(-half_width, -0.17, top_z), (-0.414, -0.17, top_z)],
        [(-half_width, 0.17, top_z), (-0.414, 0.17, top_z)],
        [(half_width, -0.17, top_z), (0.414, -0.17, top_z)],
        [(half_width, 0.17, top_z), (0.414, 0.17, top_z)],
    ]
    support_paths = [
        [(-0.46, -0.25, 0.03), (-0.39, -0.20, top_z)],
        [(-0.46, 0.25, 0.03), (-0.39, 0.20, top_z)],
        [(0.46, -0.25, 0.03), (0.39, -0.20, top_z)],
        [(0.46, 0.25, 0.03), (0.39, 0.20, top_z)],
        [(-0.46, -0.25, 0.03), (-0.46, 0.25, 0.03)],
        [(0.46, -0.25, 0.03), (0.46, 0.25, 0.03)],
        [(-0.425, -0.226, 0.47), (-0.425, 0.226, 0.47)],
        [(0.425, -0.226, 0.47), (0.425, 0.226, 0.47)],
        [(-0.438, -0.232, 0.355), (0.438, -0.232, 0.355)],
        [(-0.442, 0.234, 0.28), (0.442, 0.234, 0.28)],
        [(-0.30, -0.084, 0.42), (-0.438, -0.232, 0.355)],
        [(0.30, -0.084, 0.42), (0.438, -0.232, 0.355)],
    ]

    center_frame.visual(
        _tube_bundle(upper_paths, radius=tube_r, name="center_upper_frame"),
        material=steel,
        name="upper_frame",
    )
    center_frame.visual(
        _tube_bundle(support_paths, radius=tube_r, name="center_support_frame"),
        material=steel,
        name="stand_supports",
    )
    center_frame.visual(
        Box((0.06, 0.03, 0.02)),
        origin=Origin(xyz=(-0.46, 0.0, 0.02)),
        material=gray_cap,
        name="left_foot_cap",
    )
    center_frame.visual(
        Box((0.06, 0.03, 0.02)),
        origin=Origin(xyz=(0.46, 0.0, 0.02)),
        material=gray_cap,
        name="right_foot_cap",
    )
    center_frame.visual(
        Box((0.032, 0.34, 0.014)),
        origin=Origin(xyz=(-0.406, 0.0, top_z)),
        material=steel,
        name="left_hinge_mount",
    )
    center_frame.visual(
        Box((0.032, 0.34, 0.014)),
        origin=Origin(xyz=(0.406, 0.0, top_z)),
        material=steel,
        name="right_hinge_mount",
    )
    center_frame.visual(
        Box((0.014, 0.020, 0.100)),
        origin=Origin(xyz=(-0.406, -0.232, 0.305)),
        material=steel,
        name="left_support_hinge_bracket",
    )
    center_frame.visual(
        Box((0.014, 0.020, 0.100)),
        origin=Origin(xyz=(0.406, -0.232, 0.305)),
        material=steel,
        name="right_support_hinge_bracket",
    )

    left_wing = model.part("left_wing")
    left_wing.inertial = Inertial.from_geometry(
        Box((wing_span, 0.44, 0.03)),
        mass=1.1,
        origin=Origin(xyz=(-wing_span * 0.5, 0.0, 0.0)),
    )
    left_wing.visual(
        _tube_bundle(
            [
                [(0.0, -half_depth, 0.0), (0.0, half_depth, 0.0)],
                [(-wing_span, -half_depth, 0.0), (-wing_span, half_depth, 0.0)],
                [(0.0, -half_depth, 0.0), (-wing_span, -half_depth, 0.0)],
                [(0.0, half_depth, 0.0), (-wing_span, half_depth, 0.0)],
                [(0.0, -0.14, 0.0), (-wing_span, -0.14, 0.0)],
                [(0.0, -0.07, 0.0), (-wing_span, -0.07, 0.0)],
                [(0.0, 0.00, 0.0), (-wing_span, 0.00, 0.0)],
                [(0.0, 0.07, 0.0), (-wing_span, 0.07, 0.0)],
                [(0.0, 0.14, 0.0), (-wing_span, 0.14, 0.0)],
            ],
            radius=tube_r,
            name="left_wing_frame",
        ),
        material=steel,
        name="left_wing_frame",
    )

    right_wing = model.part("right_wing")
    right_wing.inertial = Inertial.from_geometry(
        Box((wing_span, 0.44, 0.03)),
        mass=1.1,
        origin=Origin(xyz=(wing_span * 0.5, 0.0, 0.0)),
    )
    right_wing.visual(
        _tube_bundle(
            [
                [(0.0, -half_depth, 0.0), (0.0, half_depth, 0.0)],
                [(wing_span, -half_depth, 0.0), (wing_span, half_depth, 0.0)],
                [(0.0, -half_depth, 0.0), (wing_span, -half_depth, 0.0)],
                [(0.0, half_depth, 0.0), (wing_span, half_depth, 0.0)],
                [(0.0, -0.14, 0.0), (wing_span, -0.14, 0.0)],
                [(0.0, -0.07, 0.0), (wing_span, -0.07, 0.0)],
                [(0.0, 0.00, 0.0), (wing_span, 0.00, 0.0)],
                [(0.0, 0.07, 0.0), (wing_span, 0.07, 0.0)],
                [(0.0, 0.14, 0.0), (wing_span, 0.14, 0.0)],
            ],
            radius=tube_r,
            name="right_wing_frame",
        ),
        material=steel,
        name="right_wing_frame",
    )

    lower_support = model.part("lower_support")
    lower_support.inertial = Inertial.from_geometry(
        Box((0.72, 0.28, 0.10)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.12, -0.02)),
    )
    lower_support.visual(
        _tube_bundle(
            [
                [(-0.34, 0.02, 0.0), (0.34, 0.02, 0.0)],
                [(-0.34, 0.24, 0.0), (0.34, 0.24, 0.0)],
                [(-0.34, 0.02, 0.0), (-0.34, 0.24, 0.0)],
                [(0.34, 0.02, 0.0), (0.34, 0.24, 0.0)],
                [(-0.34, 0.07, 0.0), (0.34, 0.07, 0.0)],
                [(-0.34, 0.12, 0.0), (0.34, 0.12, 0.0)],
                [(-0.34, 0.17, 0.0), (0.34, 0.17, 0.0)],
                [(-0.34, 0.22, 0.0), (0.34, 0.22, 0.0)],
                [(-0.388, 0.0, 0.0), (-0.34, 0.02, 0.0)],
                [(0.388, 0.0, 0.0), (0.34, 0.02, 0.0)],
                [(-0.23, 0.02, 0.0), (-0.23, 0.02, -0.055), (-0.23, 0.24, -0.055), (-0.23, 0.24, 0.0)],
                [(0.23, 0.045, 0.0), (0.23, 0.045, -0.055), (0.23, 0.24, -0.055), (0.23, 0.24, 0.0)],
            ],
            radius=tube_r,
            name="lower_support_frame",
        ),
        material=steel,
        name="lower_support_frame",
    )
    lower_support.visual(
        Box((0.016, 0.016, 0.014)),
        origin=Origin(xyz=(-0.391, 0.0, 0.0)),
        material=steel,
        name="left_support_knuckle",
    )
    lower_support.visual(
        Box((0.016, 0.016, 0.014)),
        origin=Origin(xyz=(0.391, 0.0, 0.0)),
        material=steel,
        name="right_support_knuckle",
    )

    model.articulation(
        "left_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=left_wing,
        origin=Origin(xyz=(-0.43, 0.0, top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "right_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=right_wing,
        origin=Origin(xyz=(0.43, 0.0, top_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "lower_support_hinge",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=lower_support,
        origin=Origin(xyz=(0.0, -0.232, 0.286)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.10),
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

    center_frame = object_model.get_part("center_frame")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    lower_support = object_model.get_part("lower_support")

    left_hinge = object_model.get_articulation("left_wing_hinge")
    right_hinge = object_model.get_articulation("right_wing_hinge")
    support_hinge = object_model.get_articulation("lower_support_hinge")

    ctx.expect_gap(
        center_frame,
        lower_support,
        axis="z",
        positive_elem="upper_frame",
        negative_elem="lower_support_frame",
        min_gap=0.50,
        max_gap=0.70,
        name="lower support sits below the central drying rails",
    )
    ctx.expect_overlap(
        lower_support,
        center_frame,
        axes="x",
        elem_a="lower_support_frame",
        elem_b="upper_frame",
        min_overlap=0.60,
        name="lower support stays centered under the main span",
    )

    left_rest = ctx.part_world_aabb(left_wing)
    right_rest = ctx.part_world_aabb(right_wing)
    support_rest = ctx.part_world_aabb(lower_support)

    with ctx.pose({left_hinge: 1.10, right_hinge: 1.10, support_hinge: 1.10}):
        left_folded = ctx.part_world_aabb(left_wing)
        right_folded = ctx.part_world_aabb(right_wing)
        support_folded = ctx.part_world_aabb(lower_support)

    ctx.check(
        "left wing folds upward",
        left_rest is not None
        and left_folded is not None
        and left_folded[1][2] > left_rest[1][2] + 0.16,
        details=f"rest={left_rest}, folded={left_folded}",
    )
    ctx.check(
        "right wing folds upward",
        right_rest is not None
        and right_folded is not None
        and right_folded[1][2] > right_rest[1][2] + 0.16,
        details=f"rest={right_rest}, folded={right_folded}",
    )
    ctx.check(
        "lower support folds upward",
        support_rest is not None
        and support_folded is not None
        and support_folded[1][2] > support_rest[1][2] + 0.10,
        details=f"rest={support_rest}, folded={support_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
