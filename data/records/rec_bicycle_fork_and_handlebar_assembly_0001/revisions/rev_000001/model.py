from __future__ import annotations

import math

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
HEAD_TILT = 0.30
HEAD_AXIS = (math.sin(HEAD_TILT), 0.0, math.cos(HEAD_TILT))


def _rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x + s * z, y, -s * x + c * z)


def _steer_frame(
    point: tuple[float, float, float],
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[float, float, float]:
    shifted = (point[0] - origin[0], point[1] - origin[1], point[2] - origin[2])
    return _rotate_y(shifted, -HEAD_TILT)


def _point_on_axis(distance: float) -> tuple[float, float, float]:
    return (HEAD_AXIS[0] * distance, 0.0, HEAD_AXIS[2] * distance)


def _mirror_y(point: tuple[float, float, float]) -> tuple[float, float, float]:
    return (point[0], -point[1], point[2])


def _pitch_from_xz(vector: tuple[float, float, float]) -> float:
    return math.atan2(vector[0], vector[2])


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _add_brake_geometry(part, hood_material, lever_material, hardware_material, lever_mesh) -> None:
    part.visual(
        Box((0.020, 0.030, 0.020)),
        origin=Origin(xyz=(0.004, 0.0, 0.002)),
        material=lever_material,
    )
    part.visual(
        Box((0.046, 0.030, 0.090)),
        origin=Origin(xyz=(0.012, 0.0, 0.040), rpy=(0.0, -0.20, 0.0)),
        material=hood_material,
    )
    part.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.014, 0.0, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
    )
    part.visual(lever_mesh, material=lever_material)
    part.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, 0.130)),
        mass=0.18,
        origin=Origin(xyz=(0.016, 0.0, 0.012)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bicycle_front_end", assets=ASSETS)

    carbon = model.material("carbon_black", rgba=(0.13, 0.14, 0.15, 1.0))
    anodized = model.material("anodized_black", rgba=(0.10, 0.10, 0.11, 1.0))
    alloy = model.material("satin_alloy", rgba=(0.61, 0.63, 0.66, 1.0))
    rubber = model.material("hood_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    steel = model.material("bolt_steel", rgba=(0.67, 0.69, 0.72, 1.0))

    frame_stub = model.part("frame_stub")
    frame_stub.visual(
        Cylinder(radius=0.0225, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, HEAD_TILT, 0.0)),
        material=carbon,
    )
    frame_stub.visual(
        Cylinder(radius=0.0245, length=0.012),
        origin=Origin(xyz=_point_on_axis(0.081), rpy=(0.0, HEAD_TILT, 0.0)),
        material=alloy,
    )
    frame_stub.visual(
        Cylinder(radius=0.0245, length=0.012),
        origin=Origin(xyz=_point_on_axis(-0.081), rpy=(0.0, HEAD_TILT, 0.0)),
        material=alloy,
    )
    frame_stub.visual(
        Cylinder(radius=0.0165, length=0.112),
        origin=Origin(
            xyz=(-0.049, 0.0, 0.052),
            rpy=(0.0, _pitch_from_xz((-0.960, 0.0, 0.280)), 0.0),
        ),
        material=carbon,
    )
    frame_stub.visual(
        Cylinder(radius=0.0220, length=0.150),
        origin=Origin(
            xyz=(-0.060, 0.0, -0.067),
            rpy=(0.0, _pitch_from_xz((-0.840, 0.0, -0.543)), 0.0),
        ),
        material=carbon,
    )
    frame_stub.visual(
        Box((0.034, 0.042, 0.056)),
        origin=Origin(xyz=(-0.012, 0.0, -0.006), rpy=(0.0, -0.55, 0.0)),
        material=carbon,
    )
    frame_stub.inertial = Inertial.from_geometry(
        Box((0.190, 0.060, 0.240)),
        mass=1.10,
        origin=Origin(xyz=(-0.032, 0.0, -0.010)),
    )

    fork = model.part("fork_assembly")
    fork.visual(
        Cylinder(radius=0.0140, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=alloy,
    )
    fork.visual(
        Cylinder(radius=0.0180, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.081)),
        material=anodized,
    )
    fork.visual(
        Box((0.058, 0.112, 0.033)),
        origin=Origin(xyz=_steer_frame((0.012, 0.0, -0.082))),
        material=carbon,
    )

    left_blade_points = [
        (0.006, 0.041, -0.081),
        (0.014, 0.044, -0.145),
        (0.024, 0.046, -0.205),
        (0.035, 0.047, -0.270),
        (0.054, 0.050, -0.409),
    ]
    right_blade_points = [_mirror_y(point) for point in left_blade_points]
    left_blade = _save_mesh(
        "fork_blade_left.obj",
        tube_from_spline_points(
            [_steer_frame(point) for point in left_blade_points],
            radius=0.0105,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
    )
    right_blade = _save_mesh(
        "fork_blade_right.obj",
        tube_from_spline_points(
            [_steer_frame(point) for point in right_blade_points],
            radius=0.0105,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
    )
    fork.visual(left_blade, material=carbon)
    fork.visual(right_blade, material=carbon)
    fork.visual(
        Box((0.016, 0.096, 0.020)),
        origin=Origin(xyz=_steer_frame((0.029, 0.0, -0.195))),
        material=carbon,
    )
    fork.visual(
        Box((0.018, 0.015, 0.032)),
        origin=Origin(xyz=_steer_frame((0.055, 0.050, -0.423))),
        material=anodized,
    )
    fork.visual(
        Box((0.018, 0.015, 0.032)),
        origin=Origin(xyz=_steer_frame((0.055, -0.050, -0.423))),
        material=anodized,
    )
    for spacer_z in (0.084, 0.092, 0.100):
        fork.visual(
            Cylinder(radius=0.0185, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, spacer_z)),
            material=anodized,
        )
    fork.visual(
        Cylinder(radius=0.0180, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=anodized,
    )
    fork.visual(
        Cylinder(radius=0.0040, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=steel,
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.180, 0.180, 0.580)),
        mass=1.25,
        origin=Origin(xyz=(0.085, 0.0, -0.145)),
    )

    stem = model.part("stem")
    stem_origin_on_axis = 0.106
    stem_origin_world = _point_on_axis(stem_origin_on_axis)
    bar_center_world = (0.108, 0.0, 0.128)
    bar_center_in_stem = _steer_frame(bar_center_world, stem_origin_world)
    stem_body_length = math.sqrt(sum(value * value for value in bar_center_in_stem))
    stem_body_pitch = _pitch_from_xz(bar_center_in_stem)

    stem.visual(
        Cylinder(radius=0.0185, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=anodized,
    )
    stem.visual(
        Cylinder(radius=0.0170, length=stem_body_length),
        origin=Origin(
            xyz=(bar_center_in_stem[0] * 0.5, 0.0, bar_center_in_stem[2] * 0.5),
            rpy=(0.0, stem_body_pitch, 0.0),
        ),
        material=anodized,
    )
    stem.visual(
        Box((0.034, 0.040, 0.022)),
        origin=Origin(
            xyz=(bar_center_in_stem[0] * 0.55, 0.0, bar_center_in_stem[2] * 0.55),
            rpy=(0.0, stem_body_pitch, 0.0),
        ),
        material=anodized,
    )
    stem.visual(
        Cylinder(radius=0.0205, length=0.046),
        origin=Origin(xyz=bar_center_in_stem, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
    )
    stem.visual(
        Box((0.020, 0.050, 0.036)),
        origin=Origin(
            xyz=(bar_center_in_stem[0] + 0.010, 0.0, bar_center_in_stem[2]),
        ),
        material=anodized,
    )
    for y_sign in (-0.014, 0.014):
        for z_sign in (-0.010, 0.010):
            stem.visual(
                Cylinder(radius=0.0032, length=0.012),
                origin=Origin(
                    xyz=(
                        bar_center_in_stem[0] + 0.019,
                        y_sign,
                        bar_center_in_stem[2] + z_sign,
                    ),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=steel,
            )
    stem.inertial = Inertial.from_geometry(
        Box((0.120, 0.060, 0.090)),
        mass=0.45,
        origin=Origin(xyz=(bar_center_in_stem[0] * 0.45, 0.0, bar_center_in_stem[2] * 0.45)),
    )

    handlebar = model.part("handlebar")
    left_drop_end = (0.146, 0.191, 0.022)
    left_drop_mid = (0.151, 0.188, 0.067)
    left_hood = (0.136, 0.170, 0.105)
    left_shoulder = (0.112, 0.115, 0.134)
    left_top = (0.103, 0.055, 0.130)
    left_center = (0.104, 0.025, 0.129)
    handlebar_points_world = [
        left_drop_end,
        left_drop_mid,
        left_hood,
        left_shoulder,
        left_top,
        left_center,
        bar_center_world,
        _mirror_y(left_center),
        _mirror_y(left_top),
        _mirror_y(left_shoulder),
        _mirror_y(left_hood),
        _mirror_y(left_drop_mid),
        _mirror_y(left_drop_end),
    ]
    handlebar_mesh = _save_mesh(
        "drop_handlebar.obj",
        tube_from_spline_points(
            [_steer_frame(point, bar_center_world) for point in handlebar_points_world],
            radius=0.0115,
            samples_per_segment=20,
            radial_segments=22,
            cap_ends=True,
        ),
    )
    handlebar.visual(handlebar_mesh, material=anodized)
    handlebar.visual(
        Cylinder(radius=0.0159, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
    )
    handlebar.inertial = Inertial.from_geometry(
        Box((0.220, 0.460, 0.180)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    lever_mesh = _save_mesh(
        "brake_lever.obj",
        tube_from_spline_points(
            [
                (0.014, 0.0, 0.003),
                (0.026, 0.0, -0.025),
                (0.024, 0.0, -0.060),
                (0.012, 0.0, -0.106),
            ],
            radius=0.0045,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        ),
    )

    left_brake = model.part("left_brake")
    _add_brake_geometry(left_brake, rubber, anodized, steel, lever_mesh)

    right_brake = model.part("right_brake")
    _add_brake_geometry(right_brake, rubber, anodized, steel, lever_mesh)

    model.articulation(
        "steering_axis",
        ArticulationType.REVOLUTE,
        parent="frame_stub",
        child="fork_assembly",
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, HEAD_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=4.0,
            lower=-1.05,
            upper=1.05,
        ),
    )
    model.articulation(
        "fork_to_stem",
        ArticulationType.FIXED,
        parent="fork_assembly",
        child="stem",
        origin=Origin(xyz=(0.0, 0.0, stem_origin_on_axis)),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.FIXED,
        parent="stem",
        child="handlebar",
        origin=Origin(xyz=bar_center_in_stem),
    )
    model.articulation(
        "handlebar_to_left_brake",
        ArticulationType.FIXED,
        parent="handlebar",
        child="left_brake",
        origin=Origin(xyz=_steer_frame(left_hood, bar_center_world)),
    )
    model.articulation(
        "handlebar_to_right_brake",
        ArticulationType.FIXED,
        parent="handlebar",
        child="right_brake",
        origin=Origin(xyz=_steer_frame(_mirror_y(left_hood), bar_center_world)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("frame_stub", "fork_assembly")
    ctx.expect_aabb_contact("fork_assembly", "stem")
    ctx.expect_aabb_contact("stem", "handlebar")
    ctx.expect_aabb_contact("handlebar", "left_brake")
    ctx.expect_aabb_contact("handlebar", "right_brake")
    ctx.expect_aabb_overlap("frame_stub", "fork_assembly", axes="yz", min_overlap=0.040)
    ctx.expect_aabb_overlap("stem", "handlebar", axes="yz", min_overlap=0.030)
    ctx.expect_aabb_overlap("handlebar", "left_brake", axes="xz", min_overlap=0.015)
    ctx.expect_aabb_overlap("handlebar", "right_brake", axes="xz", min_overlap=0.015)

    def require(condition: bool, message: str) -> None:
        if not condition:
            raise AssertionError(message)

    stem_pos = ctx.part_world_position("stem")
    bar_pos = ctx.part_world_position("handlebar")
    left_brake_pos = ctx.part_world_position("left_brake")
    right_brake_pos = ctx.part_world_position("right_brake")

    require(stem_pos[0] > 0.02, "Stem should project forward of the head tube center.")
    require(stem_pos[2] > 0.09, "Stem clamp should sit above the short head-tube stub.")
    require(bar_pos[2] > stem_pos[2] + 0.015, "Handlebar clamp should sit above the stem clamp.")
    require(left_brake_pos[1] > 0.14, "Left brake hood should sit outboard on the left side.")
    require(right_brake_pos[1] < -0.14, "Right brake hood should sit outboard on the right side.")
    require(
        abs(left_brake_pos[0] - right_brake_pos[0]) < 0.03,
        "Brake hoods should be roughly symmetric fore-aft at neutral steering.",
    )
    require(
        abs(left_brake_pos[2] - right_brake_pos[2]) < 0.02,
        "Brake hoods should be roughly symmetric in height.",
    )

    with ctx.pose(steering_axis=0.75):
        ctx.expect_aabb_contact("frame_stub", "fork_assembly")
        ctx.expect_aabb_contact("fork_assembly", "stem")
        ctx.expect_aabb_contact("stem", "handlebar")
        ctx.expect_aabb_contact("handlebar", "left_brake")
        ctx.expect_aabb_contact("handlebar", "right_brake")
        ctx.expect_aabb_overlap("frame_stub", "fork_assembly", axes="yz", min_overlap=0.030)
        left_brake_left_turn = ctx.part_world_position("left_brake")
        right_brake_left_turn = ctx.part_world_position("right_brake")
        require(
            left_brake_left_turn[2] > 0.04 and right_brake_left_turn[2] > 0.04,
            "Brake controls should stay supported above the fork crown while steering left.",
        )

    with ctx.pose(steering_axis=-0.75):
        ctx.expect_aabb_contact("frame_stub", "fork_assembly")
        ctx.expect_aabb_contact("fork_assembly", "stem")
        ctx.expect_aabb_contact("stem", "handlebar")
        ctx.expect_aabb_contact("handlebar", "left_brake")
        ctx.expect_aabb_contact("handlebar", "right_brake")
        ctx.expect_aabb_overlap("frame_stub", "fork_assembly", axes="yz", min_overlap=0.030)
        left_brake_right_turn = ctx.part_world_position("left_brake")
        right_brake_right_turn = ctx.part_world_position("right_brake")
        require(
            left_brake_right_turn[2] > 0.04 and right_brake_right_turn[2] > 0.04,
            "Brake controls should stay supported above the fork crown while steering right.",
        )

    require(
        abs(left_brake_left_turn[0] - left_brake_right_turn[0]) > 0.10,
        "The left hood should sweep through a clear steering arc.",
    )
    require(
        abs(right_brake_left_turn[0] - right_brake_right_turn[0]) > 0.10,
        "The right hood should sweep through a clear steering arc.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
