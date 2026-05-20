from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    Material,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _mesh_path(name: str) -> Path:
    if hasattr(ASSETS, "mesh_path"):
        return Path(ASSETS.mesh_path(name))
    mesh_dir = ASSETS.mesh_dir
    mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_dir / name


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _x_cylinder_geom(
    radius: float,
    length: float,
    center: tuple[float, float, float],
    radial_segments: int = 20,
) -> CylinderGeometry:
    return (
        CylinderGeometry(radius, length, radial_segments=radial_segments)
        .rotate_y(math.pi / 2.0)
        .translate(*center)
    )


def _frame_mesh() -> Mesh:
    rail_path = [
        (0.18, 0.000, 0.160),
        (0.18, 0.000, 0.420),
        (0.175, 0.008, 0.780),
        (0.155, 0.028, 1.100),
        (0.110, 0.050, 1.220),
    ]
    diagonal_path = [
        (0.18, 0.000, 0.185),
        (0.125, 0.012, 0.360),
        (0.070, 0.020, 0.600),
    ]
    toe_brace_path = [
        (0.10, 0.030, 0.055),
        (0.135, 0.018, 0.105),
        (0.180, 0.000, 0.160),
    ]

    frame_geom = tube_from_spline_points(
        rail_path,
        radius=0.016,
        samples_per_segment=18,
        radial_segments=20,
    )
    frame_geom.merge(
        tube_from_spline_points(
            _mirror_x(rail_path),
            radius=0.016,
            samples_per_segment=18,
            radial_segments=20,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            diagonal_path,
            radius=0.010,
            samples_per_segment=14,
            radial_segments=16,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            _mirror_x(diagonal_path),
            radius=0.010,
            samples_per_segment=14,
            radial_segments=16,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            toe_brace_path,
            radius=0.010,
            samples_per_segment=12,
            radial_segments=16,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            _mirror_x(toe_brace_path),
            radius=0.010,
            samples_per_segment=12,
            radial_segments=16,
        )
    )
    frame_geom.merge(_x_cylinder_geom(0.016, 0.220, (0.000, 0.050, 1.220)))
    frame_geom.merge(_x_cylinder_geom(0.013, 0.320, (0.000, 0.014, 0.660)))
    frame_geom.merge(_x_cylinder_geom(0.014, 0.400, (0.000, 0.000, 0.180)))
    frame_geom.merge(_x_cylinder_geom(0.012, 0.220, (0.000, 0.030, 0.055)))
    frame_geom.merge(_x_cylinder_geom(0.010, 0.034, (0.211, 0.000, 0.165)))
    frame_geom.merge(_x_cylinder_geom(0.010, 0.034, (-0.211, 0.000, 0.165)))
    frame_geom.merge(BoxGeometry((0.240, 0.006, 0.420)).translate(0.000, -0.004, 0.600))
    return mesh_from_geometry(frame_geom, _mesh_path("frame.obj"))


def _toe_plate_mesh() -> Mesh:
    support_path = [
        (0.100, 0.015, 0.005),
        (0.110, 0.055, 0.018),
        (0.125, 0.115, 0.040),
    ]

    plate_geom = _x_cylinder_geom(0.010, 0.200, (0.000, 0.015, 0.005), 18)
    plate_geom.merge(BoxGeometry((0.340, 0.220, 0.007)).translate(0.000, 0.128, -0.004))
    plate_geom.merge(BoxGeometry((0.340, 0.010, 0.020)).translate(0.000, 0.237, 0.010))
    plate_geom.merge(_x_cylinder_geom(0.009, 0.240, (0.000, 0.120, 0.010), 18))
    plate_geom.merge(
        tube_from_spline_points(
            support_path,
            radius=0.010,
            samples_per_segment=12,
            radial_segments=16,
        )
    )
    plate_geom.merge(
        tube_from_spline_points(
            _mirror_x(support_path),
            radius=0.010,
            samples_per_segment=12,
            radial_segments=16,
        )
    )
    plate_geom.merge(BoxGeometry((0.012, 0.180, 0.018)).translate(0.164, 0.125, 0.006))
    plate_geom.merge(BoxGeometry((0.012, 0.180, 0.018)).translate(-0.164, 0.125, 0.006))
    return mesh_from_geometry(plate_geom, _mesh_path("toe_plate.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_truck_dolly", assets=ASSETS)

    painted_steel = _material("painted_steel", (0.17, 0.18, 0.20, 1.0))
    brushed_steel = _material("brushed_steel", (0.55, 0.57, 0.60, 1.0))
    black_rubber = _material("black_rubber", (0.08, 0.08, 0.08, 1.0))
    dark_polymer = _material("dark_polymer", (0.14, 0.14, 0.15, 1.0))
    model.materials.extend([painted_steel, brushed_steel, black_rubber, dark_polymer])

    frame = model.part("frame")
    frame.visual(_frame_mesh(), material=painted_steel)
    frame.inertial = Inertial.from_geometry(
        Box((0.480, 0.120, 1.180)),
        mass=7.8,
        origin=Origin(xyz=(0.000, 0.018, 0.630)),
    )

    toe_plate = model.part("toe_plate")
    toe_plate.visual(_toe_plate_mesh(), material=brushed_steel)
    toe_plate.inertial = Inertial.from_geometry(
        Box((0.340, 0.250, 0.090)),
        mass=2.1,
        origin=Origin(xyz=(0.000, 0.120, 0.018)),
    )

    toe_extension = model.part("toe_extension")
    toe_extension.visual(
        Box((0.260, 0.130, 0.006)),
        origin=Origin(xyz=(0.000, 0.055, -0.003)),
        material=brushed_steel,
    )
    toe_extension.visual(
        Box((0.260, 0.006, 0.018)),
        origin=Origin(xyz=(0.000, 0.120, 0.007)),
        material=painted_steel,
    )
    toe_extension.visual(
        Cylinder(radius=0.006, length=0.200),
        origin=Origin(xyz=(0.000, 0.060, 0.003), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=painted_steel,
    )
    toe_extension.inertial = Inertial.from_geometry(
        Box((0.260, 0.140, 0.025)),
        mass=0.8,
        origin=Origin(xyz=(0.000, 0.065, 0.006)),
    )

    for name, wheel_offset in (("left_wheel", 0.026), ("right_wheel", -0.026)):
        wheel = model.part(name)
        wheel.visual(
            Cylinder(radius=0.130, length=0.050),
            origin=Origin(xyz=(wheel_offset, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
            material=black_rubber,
        )
        wheel.visual(
            Cylinder(radius=0.070, length=0.036),
            origin=Origin(xyz=(wheel_offset, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
            material=dark_polymer,
        )
        wheel.visual(
            Cylinder(radius=0.030, length=0.044),
            origin=Origin(xyz=(wheel_offset, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
            material=brushed_steel,
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.130, length=0.050),
            mass=1.9,
            origin=Origin(xyz=(wheel_offset, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        )

    model.articulation(
        "frame_to_toe_plate",
        ArticulationType.FIXED,
        parent="frame",
        child="toe_plate",
        origin=Origin(xyz=(0.000, 0.030, 0.055)),
    )
    model.articulation(
        "toe_extension_hinge",
        ArticulationType.REVOLUTE,
        parent="toe_plate",
        child="toe_extension",
        origin=Origin(xyz=(0.000, 0.235, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="left_wheel",
        origin=Origin(xyz=(0.235, 0.000, 0.165)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="right_wheel",
        origin=Origin(xyz=(-0.235, 0.000, 0.165)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "toe_plate",
        "toe_extension",
        reason="The folding extension intentionally shares the hinge envelope at the front lip.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)

    frame_pos = ctx.part_world_position("frame")
    toe_plate_pos = ctx.part_world_position("toe_plate")
    left_wheel_pos = ctx.part_world_position("left_wheel")
    right_wheel_pos = ctx.part_world_position("right_wheel")

    assert abs(frame_pos[0]) < 1e-6 and abs(frame_pos[1]) < 1e-6, (
        "Frame should define the centered root."
    )
    assert toe_plate_pos[1] > frame_pos[1] + 0.02, (
        "Toe plate should mount forward of the frame root."
    )
    assert toe_plate_pos[2] < left_wheel_pos[2], "Toe plate should sit below the wheel axle height."
    assert left_wheel_pos[0] > 0.20 and right_wheel_pos[0] < -0.20, (
        "Wheels should sit wide on either side of the frame."
    )
    assert abs(left_wheel_pos[0] + right_wheel_pos[0]) < 1e-6, (
        "Wheel placement should remain laterally symmetric."
    )
    assert abs(left_wheel_pos[2] - right_wheel_pos[2]) < 1e-6, "Wheel axles should be level."
    assert (left_wheel_pos[0] - right_wheel_pos[0]) > 0.45, (
        "The dolly needs a stable axle track width."
    )

    ctx.expect_origin_distance("left_wheel", "right_wheel", axes="xy", max_dist=0.60)
    ctx.expect_joint_motion_axis(
        "toe_extension_hinge",
        "toe_extension",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )

    with ctx.pose(toe_extension_hinge=0.0):
        ctx.expect_aabb_overlap("toe_extension", "toe_plate", axes="xy", min_overlap=0.008)
        deployed_ext_pos = ctx.part_world_position("toe_extension")
        assert deployed_ext_pos[1] > toe_plate_pos[1], (
            "Deployed extension should project in front of the main toe plate origin."
        )

    with ctx.pose(toe_extension_hinge=1.30):
        ctx.expect_aabb_overlap("toe_extension", "toe_plate", axes="xy", min_overlap=0.004)

    with ctx.pose(left_wheel_spin=1.70, right_wheel_spin=-1.10):
        spun_left = ctx.part_world_position("left_wheel")
        spun_right = ctx.part_world_position("right_wheel")
        assert max(abs(a - b) for a, b in zip(left_wheel_pos, spun_left)) < 1e-6, (
            "Wheel spin should not translate the left wheel center."
        )
        assert max(abs(a - b) for a, b in zip(right_wheel_pos, spun_right)) < 1e-6, (
            "Wheel spin should not translate the right wheel center."
        )
        ctx.expect_origin_distance("left_wheel", "right_wheel", axes="xy", max_dist=0.60)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
