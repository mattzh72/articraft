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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

LEG_AZIMUTHS_DEG = (0.0, 120.0, 240.0)
LEG_SPREAD_DEG = 35.0
UPPER_LEG_LENGTH = 0.34
LOWER_LEG_JOINT_Z = 0.14
LOWER_LEG_MAX_EXTENSION = 0.12
CENTER_COLUMN_MAX_EXTENSION = 0.12


def _leg_direction(
    azimuth_deg: float, spread_deg: float = LEG_SPREAD_DEG
) -> tuple[float, float, float]:
    azimuth = math.radians(azimuth_deg)
    spread = math.radians(spread_deg)
    return (
        math.sin(spread) * math.cos(azimuth),
        math.sin(spread) * math.sin(azimuth),
        -math.cos(spread),
    )


def _leg_mount_origin(azimuth_deg: float) -> Origin:
    azimuth = math.radians(azimuth_deg)
    return Origin(
        xyz=(0.048 * math.cos(azimuth), 0.048 * math.sin(azimuth), -0.008),
        rpy=(0.0, math.pi - math.radians(LEG_SPREAD_DEG), azimuth),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_tripod_device", assets=ASSETS)

    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    carbon_fiber = model.material("carbon_fiber", rgba=(0.18, 0.19, 0.21, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.64, 0.66, 0.69, 1.0))
    soft_rubber = model.material("soft_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.58, 0.70, 0.84, 0.36))
    accent_red = model.material("accent_red", rgba=(0.64, 0.14, 0.12, 1.0))

    camera_body_mesh = mesh_from_geometry(
        superellipse_side_loft(
            [
                (-0.045, -0.016, 0.028, 0.060),
                (-0.020, -0.020, 0.036, 0.072),
                (0.010, -0.022, 0.046, 0.076),
                (0.040, -0.016, 0.040, 0.070),
                (0.060, -0.010, 0.030, 0.054),
            ],
            exponents=2.7,
            segments=56,
            cap=True,
            closed=True,
        ),
        ASSETS.mesh_path("camera_body_shell.obj"),
    )
    pan_handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, -0.014, 0.044),
                (0.000, -0.045, 0.038),
                (0.000, -0.110, 0.020),
                (0.000, -0.185, -0.004),
            ],
            radius=0.006,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        ASSETS.mesh_path("pan_handle.obj"),
    )

    spider = model.part("spider")
    spider.visual(Cylinder(radius=0.055, length=0.028), material=machined_aluminum)
    spider.visual(
        Cylinder(radius=0.044, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=satin_black,
    )
    spider.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=machined_aluminum,
    )
    spider.visual(
        Cylinder(radius=0.024, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=machined_aluminum,
    )
    spider.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=lens_glass,
    )
    for azimuth_deg in LEG_AZIMUTHS_DEG:
        joint_origin = _leg_mount_origin(azimuth_deg)
        direction = _leg_direction(azimuth_deg)
        spider.visual(
            Cylinder(radius=0.020, length=0.072),
            origin=Origin(
                xyz=(
                    joint_origin.xyz[0] + direction[0] * 0.036,
                    joint_origin.xyz[1] + direction[1] * 0.036,
                    joint_origin.xyz[2] + direction[2] * 0.036,
                ),
                rpy=joint_origin.rpy,
            ),
            material=satin_black,
        )
        spider.visual(
            Box((0.024, 0.040, 0.030)),
            origin=Origin(
                xyz=(
                    joint_origin.xyz[0]
                    + direction[0] * 0.020
                    + 0.010 * math.cos(math.radians(azimuth_deg)),
                    joint_origin.xyz[1]
                    + direction[1] * 0.020
                    + 0.010 * math.sin(math.radians(azimuth_deg)),
                    joint_origin.xyz[2] + direction[2] * 0.020,
                ),
                rpy=(0.0, 0.0, math.radians(azimuth_deg)),
            ),
            material=satin_black,
        )
    spider.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.150),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    for index, azimuth_deg in enumerate(LEG_AZIMUTHS_DEG, start=1):
        upper_leg = model.part(f"upper_leg_{index}")
        upper_leg.visual(
            Cylinder(radius=0.016, length=0.260),
            origin=Origin(xyz=(0.0, 0.0, 0.130)),
            material=carbon_fiber,
        )
        upper_leg.visual(
            Cylinder(radius=0.018, length=0.100),
            origin=Origin(xyz=(0.0, 0.0, 0.290)),
            material=satin_black,
        )
        upper_leg.visual(
            Cylinder(radius=0.021, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.025)),
            material=machined_aluminum,
        )
        upper_leg.visual(
            Box((0.048, 0.020, 0.042)),
            origin=Origin(xyz=(0.016, 0.0, 0.286)),
            material=satin_black,
        )
        upper_leg.visual(
            Box((0.012, 0.034, 0.010)),
            origin=Origin(xyz=(0.040, 0.0, 0.290)),
            material=machined_aluminum,
        )
        upper_leg.inertial = Inertial.from_geometry(
            Cylinder(radius=0.018, length=UPPER_LEG_LENGTH),
            mass=0.44,
            origin=Origin(xyz=(0.0, 0.0, UPPER_LEG_LENGTH / 2.0)),
        )

        lower_leg = model.part(f"lower_leg_{index}")
        lower_leg.visual(
            Cylinder(radius=0.0125, length=0.300),
            origin=Origin(xyz=(0.0, 0.0, 0.180)),
            material=carbon_fiber,
        )
        lower_leg.visual(
            Cylinder(radius=0.0158, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=machined_aluminum,
        )
        lower_leg.visual(
            Cylinder(radius=0.0168, length=0.036),
            origin=Origin(xyz=(0.0, 0.0, 0.038)),
            material=satin_black,
        )
        lower_leg.visual(
            Cylinder(radius=0.015, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.312)),
            material=machined_aluminum,
        )
        lower_leg.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.338)),
            material=soft_rubber,
        )
        lower_leg.inertial = Inertial.from_geometry(
            Cylinder(radius=0.015, length=0.300),
            mass=0.30,
            origin=Origin(xyz=(0.0, 0.0, 0.170)),
        )

        model.articulation(
            f"spider_to_upper_leg_{index}",
            ArticulationType.FIXED,
            parent="spider",
            child=f"upper_leg_{index}",
            origin=_leg_mount_origin(azimuth_deg),
        )
        model.articulation(
            f"leg_{index}_extension",
            ArticulationType.PRISMATIC,
            parent=f"upper_leg_{index}",
            child=f"lower_leg_{index}",
            origin=Origin(xyz=(0.0, 0.0, LOWER_LEG_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.25,
                lower=0.0,
                upper=LOWER_LEG_MAX_EXTENSION,
            ),
        )

    center_column = model.part("center_column")
    center_column.visual(
        Cylinder(radius=0.018, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=machined_aluminum,
    )
    center_column.visual(
        Cylinder(radius=0.028, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=satin_black,
    )
    center_column.visual(
        Box((0.016, 0.050, 0.014)),
        origin=Origin(xyz=(0.026, 0.0, -0.090)),
        material=machined_aluminum,
    )
    center_column.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
        material=satin_black,
    )
    center_column.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.199)),
        material=machined_aluminum,
    )
    center_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=0.340),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.032, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=satin_black,
    )
    pan_head.visual(
        Box((0.118, 0.034, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=machined_aluminum,
    )
    pan_head.visual(
        Box((0.090, 0.032, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=satin_black,
    )
    pan_head.visual(
        Box((0.012, 0.042, 0.100)),
        origin=Origin(xyz=(-0.053, 0.0, 0.086)),
        material=machined_aluminum,
    )
    pan_head.visual(
        Box((0.012, 0.042, 0.100)),
        origin=Origin(xyz=(0.053, 0.0, 0.086)),
        material=machined_aluminum,
    )
    pan_head.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.041, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_aluminum,
    )
    pan_head.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(-0.066, -0.004, 0.094), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
    )
    pan_head.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.066, -0.004, 0.094), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
    )
    pan_head.visual(
        Cylinder(radius=0.006, length=0.106),
        origin=Origin(xyz=(0.0, -0.004, 0.094), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_aluminum,
    )
    pan_head.visual(
        Box((0.054, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.004, 0.094)),
        material=satin_black,
    )
    pan_head.visual(pan_handle_mesh, material=soft_rubber)
    pan_head.inertial = Inertial.from_geometry(
        Box((0.120, 0.210, 0.140)),
        mass=0.82,
        origin=Origin(xyz=(0.0, -0.045, 0.070)),
    )

    camera_body = model.part("camera_body")
    camera_body.visual(camera_body_mesh, material=satin_black)
    camera_body.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(-0.049, -0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_aluminum,
    )
    camera_body.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.049, -0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_aluminum,
    )
    camera_body.visual(
        Cylinder(radius=0.020, length=0.052),
        origin=Origin(xyz=(0.0, 0.086, 0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined_aluminum,
    )
    camera_body.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.105, 0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent_red,
    )
    camera_body.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.0, 0.124, 0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
    )
    camera_body.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, 0.137, 0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
    )
    camera_body.visual(
        Box((0.036, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, 0.052)),
        material=machined_aluminum,
    )
    camera_body.visual(
        Box((0.052, 0.026, 0.038)),
        origin=Origin(xyz=(0.0, -0.040, 0.008)),
        material=satin_black,
    )
    camera_body.visual(
        Box((0.006, 0.028, 0.020)),
        origin=Origin(xyz=(-0.039, 0.004, 0.018)),
        material=lens_glass,
    )
    camera_body.inertial = Inertial.from_geometry(
        Box((0.090, 0.150, 0.080)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.020, 0.012)),
    )

    model.articulation(
        "center_column_lift",
        ArticulationType.PRISMATIC,
        parent="spider",
        child="center_column",
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.20,
            lower=0.0,
            upper=CENTER_COLUMN_MAX_EXTENSION,
        ),
    )
    model.articulation(
        "head_pan",
        ArticulationType.REVOLUTE,
        parent="center_column",
        child="pan_head",
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent="pan_head",
        child="camera_body",
        origin=Origin(xyz=(0.0, -0.004, 0.094)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-0.35,
            upper=1.05,
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
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    for index in range(1, 4):
        ctx.expect_aabb_contact("spider", f"upper_leg_{index}")
        ctx.expect_aabb_contact(f"upper_leg_{index}", f"lower_leg_{index}")
        ctx.expect_joint_motion_axis(
            f"leg_{index}_extension",
            f"lower_leg_{index}",
            world_axis="z",
            direction="negative",
            min_delta=0.05,
        )

    ctx.expect_aabb_contact("spider", "center_column")
    ctx.expect_aabb_overlap("spider", "center_column", axes="xy", min_overlap=0.035)
    ctx.expect_joint_motion_axis(
        "center_column_lift",
        "center_column",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )
    ctx.expect_aabb_contact("center_column", "pan_head")
    ctx.expect_aabb_contact("pan_head", "camera_body")
    ctx.expect_aabb_overlap("camera_body", "spider", axes="xy", min_overlap=0.050)
    ctx.expect_origin_distance("camera_body", "center_column", axes="xy", max_dist=0.012)
    ctx.expect_joint_motion_axis(
        "camera_tilt",
        "camera_body",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    with ctx.pose(
        leg_1_extension=LOWER_LEG_MAX_EXTENSION,
        leg_2_extension=LOWER_LEG_MAX_EXTENSION,
        leg_3_extension=LOWER_LEG_MAX_EXTENSION,
    ):
        for index in range(1, 4):
            ctx.expect_aabb_contact(f"upper_leg_{index}", f"lower_leg_{index}")

    with ctx.pose(center_column_lift=CENTER_COLUMN_MAX_EXTENSION):
        ctx.expect_aabb_contact("spider", "center_column")
        ctx.expect_aabb_contact("center_column", "pan_head")
        ctx.expect_origin_distance("camera_body", "center_column", axes="xy", max_dist=0.012)
        ctx.expect_aabb_overlap("camera_body", "spider", axes="xy", min_overlap=0.050)

    with ctx.pose(head_pan=1.4):
        ctx.expect_aabb_contact("pan_head", "camera_body")
        ctx.expect_origin_distance("camera_body", "center_column", axes="xy", max_dist=0.012)
        ctx.expect_aabb_overlap("camera_body", "spider", axes="xy", min_overlap=0.050)

    with ctx.pose(camera_tilt=1.0):
        ctx.expect_aabb_contact("pan_head", "camera_body")
        ctx.expect_origin_distance("camera_body", "center_column", axes="xy", max_dist=0.025)

    with ctx.pose(camera_tilt=-0.30):
        ctx.expect_aabb_contact("pan_head", "camera_body")
        ctx.expect_origin_distance("camera_body", "center_column", axes="xy", max_dist=0.025)

    with ctx.pose(
        leg_1_extension=LOWER_LEG_MAX_EXTENSION,
        leg_2_extension=LOWER_LEG_MAX_EXTENSION,
        leg_3_extension=LOWER_LEG_MAX_EXTENSION,
        center_column_lift=CENTER_COLUMN_MAX_EXTENSION,
        head_pan=0.9,
        camera_tilt=0.70,
    ):
        ctx.expect_aabb_contact("spider", "center_column")
        ctx.expect_aabb_contact("center_column", "pan_head")
        ctx.expect_aabb_contact("pan_head", "camera_body")
        ctx.expect_origin_distance("camera_body", "spider", axes="xy", max_dist=0.020)
        ctx.expect_aabb_overlap("camera_body", "spider", axes="xy", min_overlap=0.045)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
