from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _build_wheel_meshes() -> tuple[object, object]:
    wheel_width = 0.18
    rim_offset = 0.068
    rim_radius = 0.553
    rim_tube = 0.03
    inner_ring_radius = 0.40
    inner_ring_tube = 0.012
    hub_radius = 0.105
    hub_length = wheel_width
    hub_cheek_radius = 0.16
    hub_cheek_thickness = 0.024

    wood = MeshGeometry()
    iron = MeshGeometry()

    for y in (-rim_offset, rim_offset):
        wood.merge(
            TorusGeometry(
                radius=rim_radius,
                tube=rim_tube,
                radial_segments=18,
                tubular_segments=80,
            )
            .rotate_x(math.pi / 2.0)
            .translate(0.0, y, 0.0)
        )
        wood.merge(
            TorusGeometry(
                radius=inner_ring_radius,
                tube=inner_ring_tube,
                radial_segments=14,
                tubular_segments=64,
            )
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.6 * y, 0.0)
        )
        iron.merge(
            TorusGeometry(
                radius=rim_radius,
                tube=0.007,
                radial_segments=12,
                tubular_segments=80,
            )
            .rotate_x(math.pi / 2.0)
            .translate(0.0, y, 0.0)
        )

    wood.merge(
        CylinderGeometry(radius=hub_radius, height=hub_length, radial_segments=36).rotate_x(
            math.pi / 2.0
        )
    )
    for y in (-0.042, 0.042):
        wood.merge(
            CylinderGeometry(
                radius=hub_cheek_radius,
                height=hub_cheek_thickness,
                radial_segments=30,
            )
            .rotate_x(math.pi / 2.0)
            .translate(0.0, y, 0.0)
        )
    for y in (-0.06, 0.0, 0.06):
        iron.merge(
            CylinderGeometry(
                radius=hub_radius + 0.01,
                height=0.01,
                radial_segments=24,
            )
            .rotate_x(math.pi / 2.0)
            .translate(0.0, y, 0.0)
        )

    spoke_count = 12
    for index in range(spoke_count):
        angle = (2.0 * math.pi * index) / spoke_count
        wood.merge(BoxGeometry((0.42, 0.11, 0.032)).translate(0.31, 0.0, 0.0).rotate_y(angle))

    paddle_count = 16
    for index in range(paddle_count):
        angle = (2.0 * math.pi * index) / paddle_count
        wood.merge(
            BoxGeometry((0.045, wheel_width * 0.92, 0.17)).translate(0.54, 0.0, 0.0).rotate_y(angle)
        )

    wood_mesh = mesh_from_geometry(wood, ASSETS.mesh_path("undershot_wheel_wood.obj"))
    iron_mesh = mesh_from_geometry(iron, ASSETS.mesh_path("undershot_wheel_iron.obj"))
    return wood_mesh, iron_mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel", assets=ASSETS)

    weathered_oak = Material(name="weathered_oak", rgba=(0.44, 0.32, 0.20, 1.0))
    aged_stone = Material(name="aged_stone", rgba=(0.57, 0.56, 0.53, 1.0))
    wrought_iron = Material(name="wrought_iron", rgba=(0.20, 0.20, 0.22, 1.0))
    model.materials.extend([weathered_oak, aged_stone, wrought_iron])

    frame = model.part("frame")
    frame.visual(
        Box((1.65, 0.60, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=aged_stone,
        name="race_floor",
    )
    frame.visual(
        Box((0.16, 0.60, 0.20)),
        origin=Origin(xyz=(-0.75, 0.0, 0.04)),
        material=aged_stone,
        name="upstream_sill",
    )
    frame.visual(
        Box((0.16, 0.60, 0.08)),
        origin=Origin(xyz=(0.75, 0.0, -0.02)),
        material=aged_stone,
        name="downstream_apron",
    )
    for side in (-1.0, 1.0):
        y_wall = side * 0.25
        y_cap = side * 0.23
        frame.visual(
            Box((1.52, 0.12, 0.54)),
            origin=Origin(xyz=(0.0, y_wall, 0.21)),
            material=aged_stone,
            name=f"masonry_wall_{'left' if side < 0 else 'right'}",
        )
        frame.visual(
            Box((0.30, 0.18, 0.12)),
            origin=Origin(xyz=(0.0, y_cap, 0.54)),
            material=weathered_oak,
            name=f"timber_cap_{'left' if side < 0 else 'right'}",
        )
        frame.visual(
            Box((0.16, 0.10, 0.10)),
            origin=Origin(xyz=(0.0, y_cap, 0.64)),
            material=weathered_oak,
            name=f"bearing_block_{'left' if side < 0 else 'right'}",
        )
        frame.visual(
            Box((1.66, 0.08, 0.08)),
            origin=Origin(xyz=(0.0, y_cap, 0.69)),
            material=weathered_oak,
            name=f"wall_plate_{'left' if side < 0 else 'right'}",
        )
    frame.visual(
        Box((0.10, 0.58, 0.74)),
        origin=Origin(xyz=(-0.70, 0.0, 0.37)),
        material=weathered_oak,
        name="upstream_tie_beam",
    )
    frame.visual(
        Box((0.10, 0.58, 0.70)),
        origin=Origin(xyz=(0.70, 0.0, 0.35)),
        material=weathered_oak,
        name="downstream_tie_beam",
    )
    frame.visual(
        Box((0.28, 0.10, 0.24)),
        origin=Origin(xyz=(-0.54, -0.27, 0.12)),
        material=aged_stone,
        name="left_upstream_buttress",
    )
    frame.visual(
        Box((0.28, 0.10, 0.24)),
        origin=Origin(xyz=(-0.54, 0.27, 0.12)),
        material=aged_stone,
        name="right_upstream_buttress",
    )
    frame.visual(
        Box((0.22, 0.10, 0.18)),
        origin=Origin(xyz=(0.50, -0.27, 0.09)),
        material=aged_stone,
        name="left_downstream_buttress",
    )
    frame.visual(
        Box((0.22, 0.10, 0.18)),
        origin=Origin(xyz=(0.50, 0.27, 0.09)),
        material=aged_stone,
        name="right_downstream_buttress",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.65, 0.60, 0.74)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
    )

    axle = model.part("axle")
    axle.visual(
        Cylinder(radius=0.04, length=0.44),
        origin=Origin(xyz=(0.0, 0.19, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="main_shaft",
    )
    axle.visual(
        Cylinder(radius=0.055, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="left_gudgeon",
    )
    axle.visual(
        Cylinder(radius=0.055, length=0.05),
        origin=Origin(xyz=(0.0, 0.38, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="right_gudgeon",
    )
    axle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.05, length=0.44),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.19, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    wheel_wood_mesh, wheel_iron_mesh = _build_wheel_meshes()
    wheel = model.part("wheel")
    wheel.visual(wheel_wood_mesh, material=weathered_oak, name="wheel_woodwork")
    wheel.visual(wheel_iron_mesh, material=wrought_iron, name="wheel_ironwork")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.59, length=0.18),
        mass=185.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_axle",
        ArticulationType.FIXED,
        parent="frame",
        child="axle",
        origin=Origin(xyz=(0.0, -0.19, 0.64)),
    )
    model.articulation(
        "axle_to_wheel",
        ArticulationType.CONTINUOUS,
        parent="axle",
        child="wheel",
        origin=Origin(xyz=(0.0, 0.19, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "axle",
        "wheel",
        reason="the wheel turns around a fixed shaft sleeve at the hub",
    )
    ctx.allow_overlap(
        "axle",
        "frame",
        reason="open timber bearing saddles are conservatively approximated as solid blocks",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("wheel", "frame", axes="xy", max_dist=0.02)
    ctx.expect_aabb_overlap("wheel", "frame", axes="xy", min_overlap=0.15)
    ctx.expect_aabb_overlap("wheel", "axle", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_overlap("axle", "frame", axes="xy", min_overlap=0.08)

    for angle in (math.pi / 8.0, math.pi / 4.0, 3.0 * math.pi / 8.0, math.pi / 2.0):
        with ctx.pose(axle_to_wheel=angle):
            ctx.expect_origin_distance("wheel", "frame", axes="xy", max_dist=0.02)
            ctx.expect_aabb_overlap("wheel", "frame", axes="xy", min_overlap=0.15)
            ctx.expect_aabb_overlap("wheel", "axle", axes="xy", min_overlap=0.08)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
