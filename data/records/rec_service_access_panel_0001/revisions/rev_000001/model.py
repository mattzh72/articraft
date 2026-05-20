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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(ASSETS.asset_root)


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    for kwargs in (
        {"name": name, "color": rgba},
        {"name": name, "rgba": rgba},
    ):
        try:
            return Material(**kwargs)
        except TypeError:
            pass
    return Material(name, rgba)


def _add_fastener(
    part, *, x: float, y: float, z: float, radius: float, length: float, material: Material
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel", assets=ASSETS)

    housing_paint = _make_material("housing_paint", (0.66, 0.68, 0.70, 1.0))
    panel_paint = _make_material("panel_paint", (0.79, 0.80, 0.81, 1.0))
    brushed_steel = _make_material("brushed_steel", (0.72, 0.73, 0.74, 1.0))
    black_hardware = _make_material("black_hardware", (0.13, 0.14, 0.15, 1.0))
    dark_rubber = _make_material("dark_rubber", (0.08, 0.09, 0.10, 1.0))
    model.materials.extend([housing_paint, panel_paint, brushed_steel, black_hardware, dark_rubber])

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box(size=(0.036, 0.56, 0.78)),
        mass=9.0,
        origin=Origin(xyz=(-0.018, 0.0, 0.0)),
    )

    side_bar = Box(size=(0.036, 0.114, 0.780))
    top_bar = Box(size=(0.036, 0.340, 0.126))
    inner_trim_side = Box(size=(0.004, 0.016, 0.540))
    inner_trim_top = Box(size=(0.004, 0.340, 0.016))
    hinge_leaf = Box(size=(0.004, 0.028, 0.160))
    strike_plate = Box(size=(0.003, 0.018, 0.120))
    seal_strip = Box(size=(0.002, 0.008, 0.470))

    housing.visual(
        side_bar,
        origin=Origin(xyz=(-0.018, -0.223, 0.0)),
        material=housing_paint,
    )
    housing.visual(
        side_bar,
        origin=Origin(xyz=(-0.018, 0.223, 0.0)),
        material=housing_paint,
    )
    housing.visual(
        top_bar,
        origin=Origin(xyz=(-0.018, 0.0, 0.327)),
        material=housing_paint,
    )
    housing.visual(
        top_bar,
        origin=Origin(xyz=(-0.018, 0.0, -0.327)),
        material=housing_paint,
    )

    housing.visual(
        inner_trim_side,
        origin=Origin(xyz=(0.0, -0.172, 0.0)),
        material=brushed_steel,
    )
    housing.visual(
        inner_trim_side,
        origin=Origin(xyz=(0.0, 0.172, 0.0)),
        material=brushed_steel,
    )
    housing.visual(
        inner_trim_top,
        origin=Origin(xyz=(0.0, 0.0, 0.264)),
        material=brushed_steel,
    )
    housing.visual(
        inner_trim_top,
        origin=Origin(xyz=(0.0, 0.0, -0.264)),
        material=brushed_steel,
    )

    housing.visual(
        seal_strip,
        origin=Origin(xyz=(-0.033, -0.162, 0.0)),
        material=dark_rubber,
    )
    housing.visual(
        seal_strip,
        origin=Origin(xyz=(-0.033, 0.162, 0.0)),
        material=dark_rubber,
    )
    housing.visual(
        Box(size=(0.002, 0.320, 0.020)),
        origin=Origin(xyz=(-0.033, 0.0, 0.254)),
        material=dark_rubber,
    )
    housing.visual(
        Box(size=(0.002, 0.320, 0.020)),
        origin=Origin(xyz=(-0.033, 0.0, -0.254)),
        material=dark_rubber,
    )

    housing.visual(
        hinge_leaf,
        origin=Origin(xyz=(-0.002, -0.172, 0.165)),
        material=brushed_steel,
    )
    housing.visual(
        hinge_leaf,
        origin=Origin(xyz=(-0.002, -0.172, -0.165)),
        material=brushed_steel,
    )
    housing.visual(
        Box(size=(0.026, 0.012, 0.430)),
        origin=Origin(xyz=(0.001, -0.160, 0.0)),
        material=brushed_steel,
    )
    housing.visual(
        Cylinder(radius=0.005, length=0.120),
        origin=Origin(xyz=(0.008, -0.158, 0.165)),
        material=brushed_steel,
    )
    housing.visual(
        Cylinder(radius=0.005, length=0.120),
        origin=Origin(xyz=(0.008, -0.158, -0.165)),
        material=brushed_steel,
    )
    housing.visual(
        strike_plate,
        origin=Origin(xyz=(0.0005, 0.170, 0.010)),
        material=brushed_steel,
    )

    for y in (-0.220, 0.220):
        for z in (-0.300, 0.300):
            _add_fastener(
                housing,
                x=0.002,
                y=y,
                z=z,
                radius=0.008,
                length=0.006,
                material=black_hardware,
            )
    for z in (-0.320, 0.320):
        _add_fastener(
            housing,
            x=0.002,
            y=0.0,
            z=z,
            radius=0.007,
            length=0.006,
            material=black_hardware,
        )

    panel = model.part("panel")
    panel.inertial = Inertial.from_geometry(
        Box(size=(0.018, 0.302, 0.494)),
        mass=3.2,
        origin=Origin(xyz=(-0.016, 0.151, 0.0)),
    )

    panel.visual(
        Box(size=(0.014, 0.302, 0.494)),
        origin=Origin(xyz=(-0.014, 0.151, 0.0)),
        material=panel_paint,
    )

    for y in (0.032, 0.282):
        panel.visual(
            Box(size=(0.004, 0.020, 0.418)),
            origin=Origin(xyz=(-0.008, 0.032 if y < 0.1 else 0.270, 0.0)),
            material=brushed_steel,
        )
    for z in (-0.205, 0.205):
        panel.visual(
            Box(size=(0.004, 0.238, 0.020)),
            origin=Origin(xyz=(-0.008, 0.151, z)),
            material=brushed_steel,
        )

    panel.visual(
        Box(size=(0.015, 0.014, 0.470)),
        origin=Origin(xyz=(-0.0275, 0.009, 0.0)),
        material=panel_paint,
    )
    panel.visual(
        Box(size=(0.015, 0.014, 0.470)),
        origin=Origin(xyz=(-0.0275, 0.293, 0.0)),
        material=panel_paint,
    )
    panel.visual(
        Box(size=(0.015, 0.274, 0.014)),
        origin=Origin(xyz=(-0.0275, 0.151, 0.240)),
        material=panel_paint,
    )
    panel.visual(
        Box(size=(0.015, 0.274, 0.014)),
        origin=Origin(xyz=(-0.0275, 0.151, -0.240)),
        material=panel_paint,
    )

    panel.visual(
        Box(size=(0.011, 0.018, 0.380)),
        origin=Origin(xyz=(-0.0265, 0.085, 0.0)),
        material=brushed_steel,
    )
    panel.visual(
        Box(size=(0.011, 0.018, 0.380)),
        origin=Origin(xyz=(-0.0265, 0.218, 0.0)),
        material=brushed_steel,
    )
    panel.visual(
        Box(size=(0.011, 0.208, 0.018)),
        origin=Origin(xyz=(-0.0265, 0.151, -0.125)),
        material=brushed_steel,
    )

    panel.visual(
        Box(size=(0.003, 0.024, 0.430)),
        origin=Origin(xyz=(-0.0045, 0.010, 0.0)),
        material=brushed_steel,
    )
    panel.visual(
        Cylinder(radius=0.0035, length=0.110),
        origin=Origin(xyz=(-0.0015, 0.0, 0.0)),
        material=brushed_steel,
    )

    panel.visual(
        Box(size=(0.004, 0.060, 0.150)),
        origin=Origin(xyz=(-0.008, 0.258, 0.0)),
        material=brushed_steel,
    )
    panel.visual(
        Box(size=(0.012, 0.012, 0.018)),
        origin=Origin(xyz=(-0.005, 0.258, 0.046)),
        material=black_hardware,
    )
    panel.visual(
        Box(size=(0.012, 0.012, 0.018)),
        origin=Origin(xyz=(-0.005, 0.258, -0.046)),
        material=black_hardware,
    )
    panel.visual(
        Box(size=(0.020, 0.012, 0.095)),
        origin=Origin(xyz=(0.003, 0.258, 0.0)),
        material=black_hardware,
    )
    panel.visual(
        Box(size=(0.015, 0.032, 0.090)),
        origin=Origin(xyz=(-0.028, 0.258, 0.0)),
        material=black_hardware,
    )

    model.articulation(
        "service_panel_hinge",
        ArticulationType.REVOLUTE,
        parent="housing",
        child="panel",
        origin=Origin(xyz=(0.013, -0.158, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.55,
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
        "housing",
        "panel",
        reason="flush service-door reveal and hinge knuckle produce conservative collision overlap during swing",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.expect_aabb_overlap("panel", "housing", axes="xy", min_overlap=0.015)
    ctx.expect_joint_motion_axis(
        "service_panel_hinge",
        "panel",
        world_axis="x",
        direction="positive",
        min_delta=0.08,
    )

    def _as_xyz(value) -> tuple[float, float, float]:
        if isinstance(value, (tuple, list)):
            return (float(value[0]), float(value[1]), float(value[2]))
        if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
            return (float(value.x), float(value.y), float(value.z))
        seq = tuple(value)
        return (float(seq[0]), float(seq[1]), float(seq[2]))

    def _aabb_bounds(
        part_name: str, *, use: str = "visual"
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        aabb = ctx.part_world_aabb(part_name, use=use)
        if isinstance(aabb, (tuple, list)) and len(aabb) == 2:
            return _as_xyz(aabb[0]), _as_xyz(aabb[1])
        for low_name, high_name in (
            ("mins", "maxs"),
            ("min", "max"),
            ("minimum", "maximum"),
            ("lo", "hi"),
            ("min_xyz", "max_xyz"),
        ):
            if hasattr(aabb, low_name) and hasattr(aabb, high_name):
                return _as_xyz(getattr(aabb, low_name)), _as_xyz(getattr(aabb, high_name))
        if hasattr(aabb, "bounds"):
            bounds = getattr(aabb, "bounds")
            if isinstance(bounds, (tuple, list)) and len(bounds) == 2:
                return _as_xyz(bounds[0]), _as_xyz(bounds[1])
        raise AssertionError(f"Unsupported AABB structure for {part_name!r}: {aabb!r}")

    def _aabb_center(part_name: str) -> tuple[float, float, float]:
        mins, maxs = _aabb_bounds(part_name)
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    housing_mins, housing_maxs = _aabb_bounds("housing")
    panel_mins, panel_maxs = _aabb_bounds("panel")
    closed_center = _aabb_center("panel")

    housing_size_y = housing_maxs[1] - housing_mins[1]
    housing_size_z = housing_maxs[2] - housing_mins[2]
    panel_size_y = panel_maxs[1] - panel_mins[1]
    panel_size_z = panel_maxs[2] - panel_mins[2]

    assert housing_size_y > 0.54, "Housing should read as a built-in frame, not a loose trim strip."
    assert housing_size_z > 0.76, (
        "Housing should provide enough surrounding context for a service opening."
    )
    assert 0.29 < panel_size_y < 0.32, (
        "Panel width should leave a narrow but believable perimeter reveal."
    )
    assert 0.49 < panel_size_z < 0.52, (
        "Panel height should match a realistic maintenance door proportion."
    )
    assert abs(closed_center[1]) < 0.01, "Closed panel should sit centered within the opening."
    assert abs(closed_center[2]) < 0.01, "Closed panel should be vertically centered in the frame."
    assert panel_maxs[0] > 0.015, (
        "Latch hardware should protrude slightly proud of the face like a practical pull."
    )
    assert housing_mins[0] < -0.03, (
        "Frame should have enough depth to read as a built-in service recess."
    )

    with ctx.pose(service_panel_hinge=1.20):
        open_center = _aabb_center("panel")
    with ctx.pose(service_panel_hinge=1.55):
        fully_open_center = _aabb_center("panel")

    assert open_center[0] > closed_center[0] + 0.10, (
        "Panel should swing outward into service access space."
    )
    assert open_center[1] < closed_center[1] - 0.07, (
        "Open panel should pivot toward the hinge side, not slide in place."
    )
    assert abs(open_center[2] - closed_center[2]) < 0.01, (
        "A side-hinged panel should keep its vertical placement while opening."
    )
    assert fully_open_center[0] > open_center[0] + 0.01, (
        "Panel should continue moving outward up to its hard-open pose."
    )
    assert fully_open_center[1] < open_center[1] - 0.01, (
        "Full-open pose should continue the hinge-side sweep."
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
