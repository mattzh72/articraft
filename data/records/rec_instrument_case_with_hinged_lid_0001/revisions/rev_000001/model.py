from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    for kwargs in ({"name": name, "color": rgba}, {"name": name, "rgba": rgba}):
        try:
            return Material(**kwargs)
        except TypeError:
            pass
    try:
        return Material(name, rgba)
    except TypeError:
        return Material(name=name)


def _scale_profile(
    profile: list[tuple[float, float]],
    sx: float,
    sy: float | None = None,
) -> list[tuple[float, float]]:
    sy = sx if sy is None else sy
    return [(x * sx, y * sy) for x, y in profile]


def _polygon_area(profile: list[tuple[float, float]]) -> float:
    area = 0.0
    for (x0, y0), (x1, y1) in zip(profile, profile[1:] + profile[:1]):
        area += x0 * y1 - x1 * y0
    return 0.5 * area


def _ensure_ccw(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return profile if _polygon_area(profile) > 0.0 else list(reversed(profile))


def _case_profile() -> list[tuple[float, float]]:
    upper_controls = [
        (-0.41, 0.040),
        (-0.36, 0.050),
        (-0.31, 0.084),
        (-0.24, 0.112),
        (-0.14, 0.124),
        (-0.03, 0.078),
        (0.08, 0.132),
        (0.20, 0.154),
        (0.31, 0.148),
        (0.39, 0.110),
        (0.43, 0.060),
    ]
    upper = sample_catmull_rom_spline_2d(
        upper_controls,
        samples_per_segment=10,
        closed=False,
    )
    lower = [(x, -y) for x, y in reversed(upper[1:-1])]
    return _ensure_ccw(upper + lower)


def _hinge_edge_profile(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    candidates = [(x, y) for x, y in sorted(profile) if y < -0.075 and -0.31 <= x <= 0.40]
    sampled = candidates[::3]
    if sampled[-1] != candidates[-1]:
        sampled.append(candidates[-1])
    return sampled


def _profile_bounds(profile: list[tuple[float, float]]) -> tuple[float, float, float, float]:
    xs = [x for x, _ in profile]
    ys = [y for _, y in profile]
    return min(xs), max(xs), min(ys), max(ys)


def _mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="instrument_case", assets=ASSETS)

    shell_black = _make_material("shell_black", (0.12, 0.12, 0.13, 1.0))
    hardware_steel = _make_material("hardware_steel", (0.70, 0.72, 0.75, 1.0))
    lining_burgundy = _make_material("lining_burgundy", (0.44, 0.10, 0.16, 1.0))
    handle_black = _make_material("handle_black", (0.09, 0.09, 0.08, 1.0))
    rubber_black = _make_material("rubber_black", (0.05, 0.05, 0.05, 1.0))

    outer_profile = _case_profile()
    cavity_profile = _scale_profile(outer_profile, 0.92, 0.915)
    liner_profile = _scale_profile(outer_profile, 0.875, 0.865)
    hinge_edge_profile = _hinge_edge_profile(outer_profile)
    min_x, max_x, min_y, max_y = _profile_bounds(outer_profile)

    case_length = max_x - min_x
    case_width = max_y - min_y
    base_height = 0.078
    lid_height = 0.052
    seam_gap = 0.004
    base_floor = 0.005
    lid_roof = 0.004
    lid_shift_y = -min_y
    lid_span_y = max_y - min_y

    base = model.part("base")
    base.visual(
        _mesh(
            "instrument_case_base_ring.obj",
            ExtrudeWithHolesGeometry(
                outer_profile,
                [cavity_profile],
                height=base_height,
                center=False,
            ),
        ),
        material=shell_black,
        name="base_ring",
    )
    base.visual(
        _mesh(
            "instrument_case_base_floor.obj",
            ExtrudeGeometry.from_z0(cavity_profile, base_floor),
        ),
        material=shell_black,
        name="base_floor",
    )
    base.visual(
        _mesh(
            "instrument_case_base_liner.obj",
            ExtrudeGeometry.from_z0(liner_profile, 0.053).translate(0.0, 0.0, base_floor),
        ),
        material=lining_burgundy,
        name="base_liner",
    )
    for suffix, x_pos in (("left", -0.165), ("right", 0.165)):
        base.visual(
            Box((0.030, 0.010, 0.018)),
            origin=Origin(xyz=(x_pos, max_y - 0.001, base_height - 0.010)),
            material=hardware_steel,
            name=f"latch_keeper_{suffix}",
        )
    base.visual(
        _mesh(
            "instrument_case_hinge_leaf_base.obj",
            tube_from_spline_points(
                [(x, y - 0.003, base_height + 0.0015) for x, y in hinge_edge_profile],
                radius=0.004,
                samples_per_segment=14,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=hardware_steel,
        name="hinge_leaf_base",
    )
    base.visual(
        _mesh(
            "instrument_case_hinge_barrel_base.obj",
            tube_from_spline_points(
                [(x, y - 0.008, base_height + seam_gap) for x, y in hinge_edge_profile],
                radius=0.0055,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=hardware_steel,
        name="hinge_barrel_base",
    )
    for idx, (x_pos, y_pos, size) in enumerate(
        [
            (-0.295, 0.0, (0.034, 0.020, 0.008)),
            (0.215, -0.085, (0.036, 0.020, 0.008)),
            (0.215, 0.085, (0.036, 0.020, 0.008)),
        ]
    ):
        base.visual(
            Box(size),
            origin=Origin(xyz=(x_pos, y_pos, size[2] / 2.0)),
            material=rubber_black,
            name=f"foot_{idx}",
        )
    base.inertial = Inertial.from_geometry(
        Box((case_length * 0.98, case_width * 0.98, 0.095)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
    )

    lid = model.part("lid")
    lid_origin = Origin(xyz=(0.0, lid_shift_y, 0.0))
    lid_handle_center_y = 0.0
    lid.visual(
        _mesh(
            "instrument_case_lid_ring.obj",
            ExtrudeWithHolesGeometry(
                outer_profile,
                [cavity_profile],
                height=lid_height,
                center=False,
            ),
        ),
        origin=lid_origin,
        material=shell_black,
        name="lid_ring",
    )
    lid.visual(
        _mesh(
            "instrument_case_lid_roof.obj",
            ExtrudeGeometry.from_z0(cavity_profile, lid_roof).translate(
                0.0,
                0.0,
                lid_height - lid_roof,
            ),
        ),
        origin=lid_origin,
        material=shell_black,
        name="lid_roof",
    )
    lid.visual(
        _mesh(
            "instrument_case_lid_liner.obj",
            ExtrudeGeometry.from_z0(liner_profile, 0.022).translate(
                0.0,
                0.0,
                lid_height - lid_roof - 0.022,
            ),
        ),
        origin=lid_origin,
        material=lining_burgundy,
        name="lid_liner",
    )
    lid.visual(
        _mesh(
            "instrument_case_handle.obj",
            tube_from_spline_points(
                [
                    (-0.068, lid_handle_center_y, lid_height + 0.004),
                    (-0.052, lid_handle_center_y, lid_height + 0.026),
                    (0.052, lid_handle_center_y, lid_height + 0.026),
                    (0.068, lid_handle_center_y, lid_height + 0.004),
                ],
                radius=0.006,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        origin=lid_origin,
        material=handle_black,
        name="carry_handle",
    )
    for suffix, x_pos in (("left", -0.060), ("right", 0.060)):
        lid.visual(
            Box((0.022, 0.028, 0.008)),
            origin=Origin(
                xyz=(x_pos, lid_shift_y + lid_handle_center_y, lid_height + 0.004),
            ),
            material=hardware_steel,
            name=f"handle_bracket_{suffix}",
        )
    latch_pivot_y = lid_span_y - 0.024
    lid.visual(
        _mesh(
            "instrument_case_hinge_leaf_lid.obj",
            tube_from_spline_points(
                [(x, y - 0.003, 0.0025) for x, y in hinge_edge_profile],
                radius=0.004,
                samples_per_segment=14,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        origin=lid_origin,
        material=hardware_steel,
        name="hinge_leaf_lid",
    )
    lid.visual(
        _mesh(
            "instrument_case_hinge_barrel_lid.obj",
            tube_from_spline_points(
                [(x, y - 0.008, 0.0) for x, y in hinge_edge_profile],
                radius=0.0055,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        origin=lid_origin,
        material=hardware_steel,
        name="hinge_barrel_lid",
    )
    for suffix, x_pos in (("left", -0.165), ("right", 0.165)):
        lid.visual(
            Box((0.042, 0.008, 0.004)),
            origin=Origin(xyz=(x_pos, latch_pivot_y - 0.004, 0.016)),
            material=hardware_steel,
            name=f"latch_mount_{suffix}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((case_length * 0.96, case_width * 0.96, lid_height)),
        mass=1.6,
        origin=Origin(xyz=(0.0, lid_shift_y, lid_height / 2.0)),
    )

    latch_left = model.part("latch_left")
    latch_right = model.part("latch_right")
    for latch in (latch_left, latch_right):
        latch.visual(
            Cylinder(radius=0.0035, length=0.042),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware_steel,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.040, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, 0.008, -0.002)),
            material=hardware_steel,
            name="latch_plate",
        )
        latch.visual(
            Box((0.036, 0.032, 0.007)),
            origin=Origin(xyz=(0.0, 0.022, -0.012), rpy=(-0.58, 0.0, 0.0)),
            material=hardware_steel,
            name="latch_arm",
        )
        latch.visual(
            Box((0.022, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, 0.036, -0.024), rpy=(-0.58, 0.0, 0.0)),
            material=hardware_steel,
            name="latch_hook",
        )
        latch.inertial = Inertial.from_geometry(
            Box((0.042, 0.030, 0.030)),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.018, -0.012)),
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lid",
        origin=Origin(xyz=(0.0, min_y, base_height + seam_gap)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "left_latch_joint",
        ArticulationType.REVOLUTE,
        parent="lid",
        child="latch_left",
        origin=Origin(xyz=(-0.165, latch_pivot_y, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=4.0,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "right_latch_joint",
        ArticulationType.REVOLUTE,
        parent="lid",
        child="latch_right",
        origin=Origin(xyz=(0.165, latch_pivot_y, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=4.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "base",
        "lid",
        reason="nested case lips and hinge hardware produce conservative convex-hull overlap in the closed shell",
    )
    ctx.allow_overlap(
        "latch_left",
        "lid",
        reason="the latch pivot barrel is intentionally nested against the lid-mounted hardware and convex collision hulls overestimate contact",
    )
    ctx.allow_overlap(
        "latch_right",
        "lid",
        reason="the latch pivot barrel is intentionally nested against the lid-mounted hardware and convex collision hulls overestimate contact",
    )
    ctx.allow_overlap(
        "base",
        "latch_left",
        reason="the closed latch hook is intended to seat against the keeper on the base shell",
    )
    ctx.allow_overlap(
        "base",
        "latch_right",
        reason="the closed latch hook is intended to seat against the keeper on the base shell",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_gap("lid", "base", axis="z", max_gap=0.012, max_penetration=0.02)
    ctx.expect_joint_motion_axis(
        "lid_hinge",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "left_latch_joint",
        "latch_left",
        world_axis="z",
        direction="positive",
        min_delta=0.010,
    )
    ctx.expect_joint_motion_axis(
        "right_latch_joint",
        "latch_right",
        world_axis="z",
        direction="positive",
        min_delta=0.010,
    )

    with ctx.pose(lid_hinge=1.25):
        ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.035)

    with ctx.pose(left_latch_joint=1.05, right_latch_joint=1.05):
        ctx.expect_aabb_gap("lid", "base", axis="z", max_gap=0.012, max_penetration=0.02)
        ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.12)

    with ctx.pose(lid_hinge=1.25, left_latch_joint=1.05, right_latch_joint=1.05):
        ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.03)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
