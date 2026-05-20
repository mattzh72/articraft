from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _material(name: str, rgba: tuple[float, float, float, float]):
    attempts = (
        lambda: Material(name=name, rgba=rgba),
        lambda: Material(name=name, color=rgba),
        lambda: Material(name, rgba=rgba),
        lambda: Material(name, color=rgba),
        lambda: Material(name, rgba),
    )
    for make in attempts:
        try:
            return make()
        except Exception:
            continue
    return None


def _box_visual(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material=None,
    name: str | None = None,
) -> None:
    kwargs = {"origin": Origin(xyz=xyz, rpy=rpy)}
    if material is not None:
        kwargs["material"] = material
    if name is not None:
        kwargs["name"] = name
    part.visual(Box(size), **kwargs)


def _cylinder_visual(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material=None,
    name: str | None = None,
) -> None:
    kwargs = {"origin": Origin(xyz=xyz, rpy=rpy)}
    if material is not None:
        kwargs["material"] = material
    if name is not None:
        kwargs["name"] = name
    part.visual(Cylinder(radius=radius, length=length), **kwargs)


def _to_xyz(value) -> tuple[float, float, float]:
    if isinstance(value, (tuple, list)) and len(value) == 3:
        return (float(value[0]), float(value[1]), float(value[2]))
    if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
        return (float(value.x), float(value.y), float(value.z))
    return tuple(float(v) for v in value)


def _aabb_center(aabb) -> tuple[float, float, float]:
    if isinstance(aabb, (tuple, list)):
        if len(aabb) == 2:
            try:
                mn = _to_xyz(aabb[0])
                mx = _to_xyz(aabb[1])
                return (
                    0.5 * (mn[0] + mx[0]),
                    0.5 * (mn[1] + mx[1]),
                    0.5 * (mn[2] + mx[2]),
                )
            except Exception:
                pass
        if len(aabb) == 6:
            min_x, max_x, min_y, max_y, min_z, max_z = (float(v) for v in aabb)
            return (
                0.5 * (min_x + max_x),
                0.5 * (min_y + max_y),
                0.5 * (min_z + max_z),
            )

    center = getattr(aabb, "center", None)
    if center is not None:
        if callable(center):
            center = center()
        try:
            return _to_xyz(center)
        except Exception:
            pass

    attrs = (
        ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z"),
        ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax"),
    )
    for names in attrs:
        if all(hasattr(aabb, name) for name in names):
            min_x, max_x, min_y, max_y, min_z, max_z = (
                float(getattr(aabb, name)) for name in names
            )
            return (
                0.5 * (min_x + max_x),
                0.5 * (min_y + max_y),
                0.5 * (min_z + max_z),
            )

    min_corner = getattr(aabb, "min", None)
    max_corner = getattr(aabb, "max", None)
    if min_corner is not None and max_corner is not None:
        mn = _to_xyz(min_corner)
        mx = _to_xyz(max_corner)
        return (
            0.5 * (mn[0] + mx[0]),
            0.5 * (mn[1] + mx[1]),
            0.5 * (mn[2] + mx[2]),
        )

    raise AssertionError(f"Unsupported AABB representation: {type(aabb)!r}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artists_easel", assets=ASSETS)

    beech_wood = _material("beech_wood", (0.77, 0.65, 0.45, 1.0))
    brushed_steel = _material("brushed_steel", (0.63, 0.65, 0.67, 1.0))
    black_plastic = _material("black_plastic", (0.14, 0.14, 0.15, 1.0))
    rubber = _material("rubber", (0.08, 0.08, 0.09, 1.0))

    front_frame = model.part("front_frame")
    _box_visual(
        front_frame,
        (0.045, 0.028, 1.68),
        (-0.245, 0.0, 0.845),
        material=beech_wood,
        name="left_upright",
    )
    _box_visual(
        front_frame,
        (0.045, 0.028, 1.68),
        (0.245, 0.0, 0.845),
        material=beech_wood,
        name="right_upright",
    )
    _box_visual(
        front_frame,
        (0.090, 0.100, 0.045),
        (-0.245, 0.020, 0.0225),
        material=beech_wood,
        name="left_foot",
    )
    _box_visual(
        front_frame,
        (0.090, 0.100, 0.045),
        (0.245, 0.020, 0.0225),
        material=beech_wood,
        name="right_foot",
    )
    _box_visual(
        front_frame,
        (0.074, 0.082, 0.012),
        (-0.245, 0.022, 0.006),
        material=rubber,
        name="left_foot_pad",
    )
    _box_visual(
        front_frame,
        (0.074, 0.082, 0.012),
        (0.245, 0.022, 0.006),
        material=rubber,
        name="right_foot_pad",
    )
    _box_visual(
        front_frame,
        (0.550, 0.032, 0.055),
        (0.0, 0.0, 0.085),
        material=beech_wood,
        name="bottom_stretcher",
    )
    _box_visual(
        front_frame,
        (0.495, 0.028, 0.050),
        (0.0, 0.0, 0.950),
        material=beech_wood,
        name="mid_stretcher",
    )
    _box_visual(
        front_frame, (0.410, 0.038, 0.070), (0.0, 0.0, 1.645), material=beech_wood, name="top_cap"
    )
    _box_visual(
        front_frame,
        (0.065, 0.034, 1.48),
        (0.0, 0.0, 0.840),
        material=beech_wood,
        name="center_mast",
    )
    _box_visual(
        front_frame,
        (0.030, 0.014, 1.18),
        (0.0, 0.010, 0.860),
        material=brushed_steel,
        name="slide_track",
    )
    _box_visual(
        front_frame,
        (0.160, 0.072, 0.050),
        (0.0, 0.030, 1.405),
        material=beech_wood,
        name="top_holder_body",
    )
    _box_visual(
        front_frame,
        (0.120, 0.026, 0.032),
        (0.0, 0.066, 1.392),
        material=beech_wood,
        name="top_holder_lip",
    )
    _box_visual(
        front_frame,
        (0.092, 0.022, 0.080),
        (0.0, 0.016, 1.404),
        material=beech_wood,
        name="top_holder_backplate",
    )
    _cylinder_visual(
        front_frame,
        0.014,
        0.060,
        (0.082, 0.030, 1.405),
        rpy=(0.0, pi / 2.0, 0.0),
        material=black_plastic,
        name="top_holder_knob",
    )
    _box_visual(
        front_frame,
        (0.220, 0.020, 0.060),
        (0.0, -0.006, 1.620),
        material=brushed_steel,
        name="rear_hinge_boss",
    )
    _box_visual(
        front_frame,
        (0.080, 0.024, 0.090),
        (0.0, 0.000, 1.602),
        material=brushed_steel,
        name="hinge_reinforcement",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.600, 0.120, 1.72)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.018, 0.86)),
    )

    tray = model.part("canvas_tray")
    _box_visual(
        tray, (0.140, 0.044, 0.060), (0.0, 0.022, 0.030), material=brushed_steel, name="mast_clamp"
    )
    _box_visual(
        tray,
        (0.030, 0.036, 0.120),
        (-0.058, 0.020, 0.060),
        material=beech_wood,
        name="left_carriage_cheek",
    )
    _box_visual(
        tray,
        (0.030, 0.036, 0.120),
        (0.058, 0.020, 0.060),
        material=beech_wood,
        name="right_carriage_cheek",
    )
    _box_visual(
        tray,
        (0.128, 0.010, 0.110),
        (0.0, 0.042, 0.055),
        material=brushed_steel,
        name="rear_guide_plate",
    )
    _box_visual(
        tray,
        (0.170, 0.020, 0.090),
        (0.0, 0.035, 0.058),
        material=brushed_steel,
        name="front_bridge_plate",
    )
    _box_visual(
        tray, (0.220, 0.032, 0.040), (0.0, 0.052, 0.040), material=beech_wood, name="support_arm"
    )
    _box_visual(
        tray, (0.520, 0.068, 0.024), (0.0, 0.090, 0.018), material=beech_wood, name="tray_board"
    )
    _box_visual(
        tray, (0.500, 0.020, 0.038), (0.0, 0.118, 0.031), material=beech_wood, name="front_lip"
    )
    _box_visual(
        tray,
        (0.020, 0.054, 0.025),
        (-0.236, 0.094, 0.026),
        material=beech_wood,
        name="left_canvas_fence",
    )
    _box_visual(
        tray,
        (0.020, 0.054, 0.025),
        (0.236, 0.094, 0.026),
        material=beech_wood,
        name="right_canvas_fence",
    )
    _cylinder_visual(
        tray,
        0.014,
        0.032,
        (0.080, 0.026, 0.058),
        rpy=(0.0, pi / 2.0, 0.0),
        material=black_plastic,
        name="tray_lock_knob",
    )
    _cylinder_visual(
        tray,
        0.007,
        0.024,
        (-0.205, 0.128, 0.056),
        rpy=(0.0, 0.0, 0.0),
        material=brushed_steel,
        name="left_page_stop",
    )
    _cylinder_visual(
        tray,
        0.007,
        0.024,
        (0.205, 0.128, 0.056),
        rpy=(0.0, 0.0, 0.0),
        material=brushed_steel,
        name="right_page_stop",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.540, 0.130, 0.140)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.070, 0.055)),
    )

    rear_leg = model.part("rear_leg")
    open_angle = 0.42
    rear_leg_length = 1.76
    rear_leg_center = (
        0.0,
        -0.5 * rear_leg_length * sin(open_angle),
        -0.5 * rear_leg_length * cos(open_angle),
    )
    _box_visual(
        rear_leg,
        (0.038, 0.030, rear_leg_length),
        rear_leg_center,
        rpy=(-open_angle, 0.0, 0.0),
        material=beech_wood,
        name="rear_support",
    )
    _box_visual(
        rear_leg,
        (0.150, 0.024, 0.110),
        (0.0, -0.016, -0.028),
        material=brushed_steel,
        name="hinge_yoke",
    )
    _box_visual(
        rear_leg,
        (0.058, 0.024, 0.180),
        (0.0, -0.052, -0.102),
        material=beech_wood,
        name="upper_clamp_block",
    )
    _box_visual(
        rear_leg,
        (0.100, 0.070, 0.036),
        (0.0, -0.710, -1.590),
        material=beech_wood,
        name="rear_foot",
    )
    _box_visual(
        rear_leg,
        (0.086, 0.060, 0.008),
        (0.0, -0.710, -1.612),
        material=rubber,
        name="rear_foot_pad",
    )
    rear_leg.inertial = Inertial.from_geometry(
        Box((0.180, 0.780, 1.76)),
        mass=2.3,
        origin=Origin(xyz=(0.0, -0.390, -0.88)),
    )

    model.articulation(
        "tray_height",
        ArticulationType.PRISMATIC,
        parent="front_frame",
        child="canvas_tray",
        origin=Origin(xyz=(0.0, 0.019, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.50, lower=0.0, upper=0.760),
    )
    model.articulation(
        "rear_leg_fold",
        ArticulationType.REVOLUTE,
        parent="front_frame",
        child="rear_leg",
        origin=Origin(xyz=(0.0, -0.022, 1.620)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.20, lower=-0.08, upper=0.180),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "canvas_tray",
        "front_frame",
        reason="tray carriage intentionally wraps the center mast and guide track for stable height adjustment",
    )
    ctx.allow_overlap(
        "front_frame",
        "rear_leg",
        reason="close folding hinge hardware sits in intentional pivot clearance",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("canvas_tray", "front_frame", axes="xy", min_overlap=0.055)
    ctx.expect_origin_distance("canvas_tray", "front_frame", axes="xy", max_dist=0.14)
    ctx.expect_joint_motion_axis(
        "tray_height", "canvas_tray", world_axis="z", direction="positive", min_delta=0.20
    )
    ctx.expect_joint_motion_axis(
        "rear_leg_fold", "rear_leg", world_axis="y", direction="positive", min_delta=0.10
    )

    frame_pos = _to_xyz(ctx.part_world_position("front_frame"))
    tray_low_pos = _to_xyz(ctx.part_world_position("canvas_tray"))
    rear_open_center = _aabb_center(ctx.part_world_aabb("rear_leg", use="collision"))

    assert abs(tray_low_pos[0] - frame_pos[0]) < 0.02, "Tray should stay centered on the mast."
    assert tray_low_pos[1] > frame_pos[1] + 0.01, "Tray should sit in front of the frame."
    assert tray_low_pos[2] > 0.21, "Lowest tray position should still hold a canvas above the feet."
    assert rear_open_center[1] < frame_pos[1] - 0.18, (
        "Rear leg should stand meaningfully behind the front frame."
    )

    with ctx.pose(tray_height=0.760):
        ctx.expect_aabb_overlap("canvas_tray", "front_frame", axes="xy", min_overlap=0.055)
        tray_high_pos = _to_xyz(ctx.part_world_position("canvas_tray"))
        assert tray_high_pos[2] - tray_low_pos[2] > 0.70, (
            "Tray should offer a substantial working height range."
        )
        assert abs(tray_high_pos[0] - frame_pos[0]) < 0.02, (
            "Tray should remain centered when raised."
        )

    with ctx.pose(rear_leg_fold=0.180):
        folded_center = _aabb_center(ctx.part_world_aabb("rear_leg", use="collision"))
        assert folded_center[1] > rear_open_center[1] + 0.12, (
            "Folding the rear leg should pull it toward the front frame."
        )

    with ctx.pose(rear_leg_fold=-0.080):
        spread_center = _aabb_center(ctx.part_world_aabb("rear_leg", use="collision"))
        assert spread_center[1] < rear_open_center[1] - 0.03, (
            "Opening the rear leg should widen the stance."
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
