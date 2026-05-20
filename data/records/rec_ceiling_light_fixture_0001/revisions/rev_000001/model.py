from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


PAINTED_WHITE = _material("painted_white_metal", (0.94, 0.95, 0.96, 1.0))
SATIN_ALUMINUM = _material("satin_aluminum", (0.78, 0.80, 0.82, 1.0))
BRUSHED_STEEL = _material("brushed_steel", (0.60, 0.62, 0.65, 1.0))
BLACK_POLYMER = _material("black_polymer", (0.14, 0.15, 0.16, 1.0))
FROSTED_ACRYLIC = _material("frosted_acrylic", (0.96, 0.97, 0.99, 0.48))


def _rounded_ring_mesh(
    filename: str,
    *,
    outer_size: tuple[float, float],
    inner_size: tuple[float, float],
    outer_radius: float,
    inner_radius: float,
    height: float,
):
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(
            outer_size[0],
            outer_size[1],
            outer_radius,
            corner_segments=12,
        ),
        [
            rounded_rect_profile(
                inner_size[0],
                inner_size[1],
                inner_radius,
                corner_segments=12,
            )
        ],
        height=height,
        cap=True,
        center=False,
        closed=True,
    )
    return mesh_from_geometry(geom, MESH_DIR / filename)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_ceiling_light_fixture", assets=ASSETS)

    housing_frame_mesh = _rounded_ring_mesh(
        "housing_frame.obj",
        outer_size=(0.402, 0.402),
        inner_size=(0.352, 0.352),
        outer_radius=0.046,
        inner_radius=0.028,
        height=0.035,
    )
    bezel_mesh = _rounded_ring_mesh(
        "bezel_ring.obj",
        outer_size=(0.420, 0.420),
        inner_size=(0.388, 0.388),
        outer_radius=0.052,
        inner_radius=0.034,
        height=0.014,
    )
    lower_lip_mesh = _rounded_ring_mesh(
        "retaining_lip.obj",
        outer_size=(0.392, 0.392),
        inner_size=(0.360, 0.360),
        outer_radius=0.036,
        inner_radius=0.024,
        height=0.006,
    )
    access_frame_mesh = _rounded_ring_mesh(
        "access_frame.obj",
        outer_size=(0.392, 0.388),
        inner_size=(0.350, 0.346),
        outer_radius=0.030,
        inner_radius=0.018,
        height=0.016,
    )

    canopy = model.part("ceiling_canopy")
    canopy.visual(
        Box((0.206, 0.206, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=BLACK_POLYMER,
        name="ceiling_gasket",
    )
    canopy.visual(
        Box((0.160, 0.160, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=PAINTED_WHITE,
        name="junction_box_cover",
    )
    canopy.visual(
        Box((0.220, 0.220, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=SATIN_ALUMINUM,
        name="canopy_trim_plate",
    )
    canopy.visual(
        Cylinder(radius=0.007, length=0.003),
        origin=Origin(xyz=(-0.048, 0.0, 0.004)),
        material=BRUSHED_STEEL,
        name="left_mount_screw",
    )
    canopy.visual(
        Cylinder(radius=0.007, length=0.003),
        origin=Origin(xyz=(0.048, 0.0, 0.004)),
        material=BRUSHED_STEEL,
        name="right_mount_screw",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((0.220, 0.220, 0.023)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.358, 0.358, 0.021)),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=PAINTED_WHITE,
        name="upper_pan",
    )
    housing.visual(
        housing_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=PAINTED_WHITE,
        name="side_frame",
    )
    housing.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=SATIN_ALUMINUM,
        name="trim_bezel",
    )
    housing.visual(
        lower_lip_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=PAINTED_WHITE,
        name="retaining_lip",
    )
    housing.visual(
        Box((0.322, 0.322, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=PAINTED_WHITE,
        name="reflector_pan",
    )
    housing.visual(
        Box((0.290, 0.290, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=SATIN_ALUMINUM,
        name="led_tray",
    )
    housing.visual(
        Box((0.090, 0.090, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=PAINTED_WHITE,
        name="driver_cover",
    )
    housing.visual(
        Box((0.330, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.196, 0.069)),
        material=BRUSHED_STEEL,
        name="hinge_rail",
    )
    housing.visual(
        Box((0.060, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.196, 0.069)),
        material=BRUSHED_STEEL,
        name="latch_strike",
    )
    housing.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(-0.118, -0.196, 0.069)),
        material=BRUSHED_STEEL,
        name="left_hinge_cap",
    )
    housing.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.118, -0.196, 0.069)),
        material=BRUSHED_STEEL,
        name="right_hinge_cap",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.420, 0.420, 0.073)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.0365)),
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((0.330, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.006, 0.006)),
        material=BRUSHED_STEEL,
        name="hinge_leaf",
    )
    access_panel.visual(
        access_frame_mesh,
        origin=Origin(xyz=(0.0, 0.194, 0.0)),
        material=SATIN_ALUMINUM,
        name="service_frame",
    )
    access_panel.visual(
        Box((0.356, 0.352, 0.006)),
        origin=Origin(xyz=(0.0, 0.194, 0.005)),
        material=FROSTED_ACRYLIC,
        name="diffuser_panel",
    )
    access_panel.visual(
        Box((0.054, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.388, 0.006)),
        material=BRUSHED_STEEL,
        name="latch_tab",
    )
    access_panel.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.391, 0.010)),
        material=BLACK_POLYMER,
        name="finger_pull",
    )
    access_panel.inertial = Inertial.from_geometry(
        Box((0.392, 0.388, 0.020)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.194, 0.010)),
    )

    model.articulation(
        "canopy_to_housing",
        ArticulationType.FIXED,
        parent="ceiling_canopy",
        child="housing",
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )
    model.articulation(
        "service_hinge",
        ArticulationType.REVOLUTE,
        parent="housing",
        child="access_panel",
        origin=Origin(xyz=(0.0, -0.194, 0.073)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("ceiling_canopy", "housing", axes="xy", min_overlap=0.12)
    ctx.expect_origin_distance("ceiling_canopy", "housing", axes="xy", max_dist=0.01)
    ctx.expect_aabb_gap("housing", "ceiling_canopy", axis="z", max_gap=0.002, max_penetration=0.004)

    ctx.expect_aabb_overlap("access_panel", "housing", axes="xy", min_overlap=0.14)
    ctx.expect_aabb_gap("access_panel", "housing", axis="z", max_gap=0.004, max_penetration=0.010)
    ctx.expect_joint_motion_axis(
        "service_hinge",
        "access_panel",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )

    with ctx.pose(service_hinge=0.85):
        ctx.expect_aabb_overlap("access_panel", "housing", axes="xy", min_overlap=0.08)

    with ctx.pose(service_hinge=1.45):
        ctx.expect_aabb_overlap("access_panel", "housing", axes="xy", min_overlap=0.03)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
