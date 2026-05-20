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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translated_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_pt, max_pt = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))


def _add_adjustment_foot(
    model: ArticulatedObject,
    *,
    name: str,
    sign: float,
    aluminum,
    dark_steel,
    rubber,
):
    foot = model.part(name)
    foot.visual(
        Cylinder(radius=0.011, length=0.33),
        origin=Origin(
            xyz=(0.0, sign * 0.028, -0.018),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="hinge_barrel",
    )
    foot.visual(
        Box((0.33, 0.014, 0.040)),
        origin=Origin(
            xyz=(0.0, sign * 0.040, -0.045),
            rpy=(sign * 0.38, 0.0, 0.0),
        ),
        material=dark_steel,
        name="hinge_bridge",
    )
    foot.visual(
        Box((0.33, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, sign * 0.015, -0.005)),
        material=dark_steel,
        name="mount_pad",
    )
    foot.visual(
        Box((0.44, 0.018, 0.108)),
        origin=Origin(
            xyz=(0.0, sign * 0.052, -0.070),
            rpy=(sign * 0.72, 0.0, 0.0),
        ),
        material=aluminum,
        name="support_blade",
    )
    foot.visual(
        Box((0.42, 0.030, 0.012)),
        origin=Origin(
            xyz=(0.0, sign * 0.082, -0.105),
            rpy=(sign * 0.72, 0.0, 0.0),
        ),
        material=rubber,
        name="contact_pad",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.44, 0.16, 0.14)),
        mass=0.55,
        origin=Origin(xyz=(0.0, sign * 0.045, -0.065)),
    )
    return foot


def _build_tray_mesh():
    outer = rounded_rect_profile(0.54, 0.31, 0.012, corner_segments=10)
    hole_profiles = [
        _translated_profile(rounded_rect_profile(0.130, 0.120, 0.008, corner_segments=8), dx=-0.090, dy=0.000),
        _translated_profile(rounded_rect_profile(0.150, 0.036, 0.008, corner_segments=8), dx=0.100, dy=-0.060),
        _translated_profile(rounded_rect_profile(0.085, 0.022, 0.006, corner_segments=8), dx=-0.155, dy=-0.100),
        _translated_profile(rounded_rect_profile(0.085, 0.022, 0.006, corner_segments=8), dx=-0.155, dy=0.100),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            hole_profiles,
            height=0.004,
            cap=True,
            center=True,
            closed=True,
        ),
        "bench_tray_plate",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_test_chassis")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    stand = model.part("stand")
    lower_z = 0.070
    upper_z = 0.186
    stand.visual(
        Box((0.56, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.159, lower_z)),
        material=dark_steel,
        name="front_lower_rail",
    )
    stand.visual(
        Box((0.56, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, -0.159, lower_z)),
        material=dark_steel,
        name="rear_lower_rail",
    )
    stand.visual(
        Box((0.022, 0.296, 0.022)),
        origin=Origin(xyz=(0.269, 0.0, lower_z)),
        material=dark_steel,
        name="right_lower_rail",
    )
    stand.visual(
        Box((0.022, 0.296, 0.022)),
        origin=Origin(xyz=(-0.269, 0.0, lower_z)),
        material=dark_steel,
        name="left_lower_rail",
    )
    stand.visual(
        Box((0.52, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.135, upper_z)),
        material=dark_steel,
        name="front_upper_rail",
    )
    stand.visual(
        Box((0.52, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.135, upper_z)),
        material=dark_steel,
        name="rear_upper_rail",
    )
    stand.visual(
        Box((0.018, 0.270, 0.018)),
        origin=Origin(xyz=(0.251, 0.0, upper_z)),
        material=dark_steel,
        name="right_upper_rail",
    )
    stand.visual(
        Box((0.018, 0.270, 0.018)),
        origin=Origin(xyz=(-0.251, 0.0, upper_z)),
        material=dark_steel,
        name="left_upper_rail",
    )

    for px in (-0.251, 0.251):
        for py in (-0.135, 0.135):
            tag = f"{'right' if px > 0 else 'left'}_{'front' if py > 0 else 'rear'}"
            stand.visual(
                Box((0.018, 0.018, 0.096)),
                origin=Origin(xyz=(px, py, 0.129)),
                material=dark_steel,
                name=f"{tag}_post",
            )

    for px in (-0.205, 0.205):
        for py in (-0.135, 0.135):
            tag = f"{'right' if px > 0 else 'left'}_{'front' if py > 0 else 'rear'}"
            stand.visual(
                Box((0.028, 0.028, 0.020)),
                origin=Origin(xyz=(px, py, 0.205)),
                material=black_oxide,
                name=f"{tag}_tray_pad",
            )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        for x_pos, span_tag in ((-0.190, "rear"), (0.190, "front")):
            stand.visual(
                Cylinder(radius=0.013, length=0.05),
                origin=Origin(
                    xyz=(x_pos, side_sign * 0.150, 0.059),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=black_oxide,
                name=f"{side_name}_{span_tag}_foot_ear",
            )

    stand.inertial = Inertial.from_geometry(
        Box((0.58, 0.35, 0.25)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )

    tray = model.part("tray")
    tray.visual(
        _build_tray_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=aluminum,
        name="tray_plate",
    )
    tray.visual(
        Box((0.500, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.149, 0.016)),
        material=aluminum,
        name="left_flange",
    )
    tray.visual(
        Box((0.500, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.149, 0.016)),
        material=aluminum,
        name="right_flange",
    )
    tray.visual(
        Box((0.014, 0.300, 0.038)),
        origin=Origin(xyz=(-0.263, 0.0, 0.023)),
        material=aluminum,
        name="rear_wall",
    )
    tray.visual(
        Box((0.120, 0.018, 0.018)),
        origin=Origin(xyz=(0.120, -0.136, 0.016)),
        material=black_oxide,
        name="gpu_slot_rail",
    )
    tray.visual(
        Box((0.020, 0.020, 0.136)),
        origin=Origin(xyz=(-0.205, -0.170, 0.072)),
        material=dark_steel,
        name="gpu_side_post",
    )
    tray.visual(
        Box((0.020, 0.010, 0.096)),
        origin=Origin(xyz=(-0.205, -0.185, 0.092)),
        material=dark_steel,
        name="gpu_post_backer",
    )
    tray.visual(
        Box((0.050, 0.012, 0.035)),
        origin=Origin(
            xyz=(-0.180, -0.158, 0.0215),
            rpy=(0.0, -0.68, 0.0),
        ),
        material=dark_steel,
        name="gpu_post_gusset",
    )
    tray.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(-0.205, -0.170, 0.145)),
        material=black_oxide,
        name="brace_lower_lug",
    )
    tray.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(-0.205, -0.170, 0.165)),
        material=black_oxide,
        name="brace_upper_lug",
    )
    tray.visual(
        Box((0.020, 0.010, 0.030)),
        origin=Origin(xyz=(-0.205, -0.185, 0.155)),
        material=black_oxide,
        name="brace_lug_backstrap",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.56, 0.32, 0.19)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    gpu_brace_arm = model.part("gpu_brace_arm")
    gpu_brace_arm.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(),
        material=black_oxide,
        name="hinge_barrel",
    )
    gpu_brace_arm.visual(
        Box((0.270, 0.014, 0.014)),
        origin=Origin(xyz=(0.145, -0.020, 0.0)),
        material=dark_steel,
        name="brace_beam",
    )
    gpu_brace_arm.visual(
        Box((0.044, 0.008, 0.012)),
        origin=Origin(xyz=(0.028, -0.004, 0.0)),
        material=dark_steel,
        name="brace_root_cheek",
    )
    gpu_brace_arm.visual(
        Box((0.080, 0.012, 0.010)),
        origin=Origin(xyz=(0.070, -0.011, 0.0)),
        material=dark_steel,
        name="brace_root_link",
    )
    gpu_brace_arm.visual(
        Box((0.018, 0.012, 0.110)),
        origin=Origin(xyz=(0.268, -0.020, -0.055)),
        material=aluminum,
        name="brace_drop",
    )
    gpu_brace_arm.visual(
        Box((0.032, 0.018, 0.010)),
        origin=Origin(xyz=(0.268, -0.020, -0.110)),
        material=rubber,
        name="brace_shoe",
    )
    gpu_brace_arm.inertial = Inertial.from_geometry(
        Box((0.30, 0.04, 0.13)),
        mass=0.45,
        origin=Origin(xyz=(0.150, 0.0, -0.045)),
    )

    left_foot = _add_adjustment_foot(
        model,
        name="left_adjustment_foot",
        sign=1.0,
        aluminum=aluminum,
        dark_steel=dark_steel,
        rubber=rubber,
    )
    right_foot = _add_adjustment_foot(
        model,
        name="right_adjustment_foot",
        sign=-1.0,
        aluminum=aluminum,
        dark_steel=dark_steel,
        rubber=rubber,
    )

    model.articulation(
        "stand_to_tray",
        ArticulationType.FIXED,
        parent=stand,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
    )
    model.articulation(
        "tray_to_gpu_brace_arm",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=gpu_brace_arm,
        origin=Origin(xyz=(-0.205, -0.170, 0.155)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "stand_to_left_foot",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=left_foot,
        origin=Origin(xyz=(0.0, 0.150, 0.059)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.25,
            upper=0.35,
        ),
    )
    model.articulation(
        "stand_to_right_foot",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=right_foot,
        origin=Origin(xyz=(0.0, -0.150, 0.059)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.25,
            upper=0.35,
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

    stand = object_model.get_part("stand")
    tray = object_model.get_part("tray")
    gpu_brace_arm = object_model.get_part("gpu_brace_arm")
    left_foot = object_model.get_part("left_adjustment_foot")
    right_foot = object_model.get_part("right_adjustment_foot")

    brace_joint = object_model.get_articulation("tray_to_gpu_brace_arm")
    left_foot_joint = object_model.get_articulation("stand_to_left_foot")
    right_foot_joint = object_model.get_articulation("stand_to_right_foot")

    ctx.expect_contact(
        tray,
        stand,
        contact_tol=0.001,
        name="tray sits on the stand structure",
    )
    ctx.expect_overlap(
        tray,
        stand,
        axes="xy",
        min_overlap=0.22,
        name="tray footprint stays inside the stand footprint",
    )
    ctx.expect_gap(
        gpu_brace_arm,
        tray,
        axis="z",
        positive_elem="brace_beam",
        negative_elem="tray_plate",
        min_gap=0.11,
        max_gap=0.18,
        name="closed GPU brace beam clears the tray plane",
    )

    rest_shoe = _aabb_center(ctx.part_element_world_aabb(gpu_brace_arm, elem="brace_shoe"))
    with ctx.pose({brace_joint: 1.20}):
        open_shoe = _aabb_center(ctx.part_element_world_aabb(gpu_brace_arm, elem="brace_shoe"))
    ctx.check(
        "GPU brace swings outboard from the tray side post",
        rest_shoe is not None
        and open_shoe is not None
        and open_shoe[1] < rest_shoe[1] - 0.12,
        details=f"rest={rest_shoe}, open={open_shoe}",
    )

    left_rest_pad = _aabb_center(ctx.part_element_world_aabb(left_foot, elem="contact_pad"))
    with ctx.pose({left_foot_joint: 0.30}):
        left_adjusted_pad = _aabb_center(ctx.part_element_world_aabb(left_foot, elem="contact_pad"))
    ctx.check(
        "left adjustment foot pivots downward",
        left_rest_pad is not None
        and left_adjusted_pad is not None
        and left_adjusted_pad[2] < left_rest_pad[2] - 0.015,
        details=f"rest={left_rest_pad}, adjusted={left_adjusted_pad}",
    )

    right_rest_pad = _aabb_center(ctx.part_element_world_aabb(right_foot, elem="contact_pad"))
    with ctx.pose({right_foot_joint: 0.30}):
        right_adjusted_pad = _aabb_center(ctx.part_element_world_aabb(right_foot, elem="contact_pad"))
    ctx.check(
        "right adjustment foot pivots downward",
        right_rest_pad is not None
        and right_adjusted_pad is not None
        and right_adjusted_pad[2] < right_rest_pad[2] - 0.015,
        details=f"rest={right_rest_pad}, adjusted={right_adjusted_pad}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
