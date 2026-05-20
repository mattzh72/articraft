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
)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_cctv_column")

    powder_coat = model.material("powder_coat", rgba=(0.27, 0.29, 0.31, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.64, 0.66, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.79, 0.80, 1.0))
    nylon = model.material("nylon", rgba=(0.14, 0.15, 0.16, 1.0))
    camera_white = model.material("camera_white", rgba=(0.88, 0.89, 0.90, 1.0))
    lens_black = model.material("lens_black", rgba=(0.06, 0.06, 0.07, 1.0))
    glass = model.material("glass", rgba=(0.20, 0.28, 0.33, 0.55))

    base_plate_size = 0.24
    plate_thickness = 0.02
    base_tube_outer = 0.12
    base_wall = 0.012
    base_tube_height = 0.95
    base_tube_inner = base_tube_outer - 2.0 * base_wall

    base_tube_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(base_tube_outer, base_tube_outer),
            [_rect_profile(base_tube_inner, base_tube_inner)],
            base_tube_height,
            center=True,
        ),
        "base_square_tube",
    )

    base_column = model.part("base_column")
    base_column.visual(
        Box((base_plate_size, base_plate_size, plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness * 0.5)),
        material=powder_coat,
        name="base_plate",
    )
    base_column.visual(
        base_tube_mesh,
        origin=Origin(xyz=(0.0, 0.0, plate_thickness + base_tube_height * 0.5)),
        material=powder_coat,
        name="base_tube_shell",
    )
    for sx in (-0.085, 0.085):
        for sy in (-0.085, 0.085):
            base_column.visual(
                Cylinder(radius=0.008, length=0.028),
                origin=Origin(xyz=(sx, sy, plate_thickness + 0.014)),
                material=galvanized,
                name=None,
            )

    guide_pad_size = (0.008, 0.044, 0.18)
    guide_pad_z = plate_thickness + 0.78 + guide_pad_size[2] * 0.5
    base_column.visual(
        Box(guide_pad_size),
        origin=Origin(
            xyz=((base_tube_inner * 0.5) - guide_pad_size[0] * 0.5, 0.0, guide_pad_z)
        ),
        material=nylon,
        name="guide_pad_pos_x",
    )
    base_column.visual(
        Box(guide_pad_size),
        origin=Origin(
            xyz=(-(base_tube_inner * 0.5) + guide_pad_size[0] * 0.5, 0.0, guide_pad_z)
        ),
        material=nylon,
        name="guide_pad_neg_x",
    )
    base_column.visual(
        Box((0.044, 0.008, 0.18)),
        origin=Origin(
            xyz=(0.0, (base_tube_inner * 0.5) - 0.004, guide_pad_z)
        ),
        material=nylon,
        name="guide_pad_pos_y",
    )
    base_column.visual(
        Box((0.044, 0.008, 0.18)),
        origin=Origin(
            xyz=(0.0, -(base_tube_inner * 0.5) + 0.004, guide_pad_z)
        ),
        material=nylon,
        name="guide_pad_neg_y",
    )
    base_column.inertial = Inertial.from_geometry(
        Box((base_plate_size, base_plate_size, plate_thickness + base_tube_height)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, (plate_thickness + base_tube_height) * 0.5)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.031, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=stainless,
        name="post_tube",
    )
    inner_post.visual(
        Cylinder(radius=0.040, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=galvanized,
        name="guide_sleeve",
    )
    inner_post.visual(
        Cylinder(radius=0.035, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 1.015)),
        material=powder_coat,
        name="post_top_cap",
    )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.08, 0.08, 1.03)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
    )

    side_arm = model.part("side_arm")
    side_arm.visual(
        Cylinder(radius=0.037, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=powder_coat,
        name="arm_stem",
    )
    side_arm.visual(
        Box((0.20, 0.05, 0.036)),
        origin=Origin(xyz=(0.10, 0.0, 0.082)),
        material=powder_coat,
        name="arm_beam",
    )
    _add_member(
        side_arm,
        (0.018, 0.0, 0.020),
        (0.164, 0.0, 0.068),
        0.012,
        powder_coat,
        name="arm_brace",
    )
    side_arm.visual(
        Cylinder(radius=0.035, length=0.010),
        origin=Origin(xyz=(0.198, 0.0, 0.105)),
        material=powder_coat,
        name="pan_mount",
    )
    side_arm.inertial = Inertial.from_geometry(
        Box((0.24, 0.08, 0.12)),
        mass=2.2,
        origin=Origin(xyz=(0.10, 0.0, 0.06)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.033, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=powder_coat,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.020, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0455)),
        material=powder_coat,
        name="pan_spindle",
    )
    pan_head.visual(
        Box((0.018, 0.050, 0.048)),
        origin=Origin(xyz=(0.012, 0.0, 0.060)),
        material=powder_coat,
        name="tilt_block",
    )
    pan_head.visual(
        Box((0.055, 0.006, 0.050)),
        origin=Origin(xyz=(0.042, 0.028, 0.058)),
        material=powder_coat,
        name="yoke_right",
    )
    pan_head.visual(
        Box((0.055, 0.006, 0.050)),
        origin=Origin(xyz=(0.042, -0.028, 0.058)),
        material=powder_coat,
        name="yoke_left",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 0.09)),
        mass=0.9,
        origin=Origin(xyz=(0.03, 0.0, 0.045)),
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=powder_coat,
        name="trunnion_right",
    )
    camera_head.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=powder_coat,
        name="trunnion_left",
    )
    camera_head.visual(
        Box((0.028, 0.032, 0.024)),
        origin=Origin(xyz=(0.008, 0.0, -0.004)),
        material=powder_coat,
        name="mount_block",
    )
    camera_head.visual(
        Cylinder(radius=0.028, length=0.120),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=camera_white,
        name="camera_body",
    )
    camera_head.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=powder_coat,
        name="rear_cap",
    )
    camera_head.visual(
        Cylinder(radius=0.033, length=0.032),
        origin=Origin(xyz=(0.123, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=camera_white,
        name="lens_hood",
    )
    camera_head.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.139, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=glass,
        name="lens_glass",
    )
    camera_head.visual(
        Box((0.105, 0.044, 0.004)),
        origin=Origin(xyz=(0.060, 0.0, 0.030)),
        material=camera_white,
        name="sunshield",
    )
    camera_head.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.132, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=lens_black,
        name="lens_barrel",
    )
    camera_head.inertial = Inertial.from_geometry(
        Box((0.17, 0.07, 0.08)),
        mass=0.85,
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
    )

    model.articulation(
        "telescoping_extension",
        ArticulationType.PRISMATIC,
        parent=base_column,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.18,
            lower=0.0,
            upper=0.55,
        ),
    )
    model.articulation(
        "post_to_side_arm",
        ArticulationType.FIXED,
        parent=inner_post,
        child=side_arm,
        origin=Origin(xyz=(0.0, 0.0, 1.03)),
    )
    model.articulation(
        "side_arm_pan",
        ArticulationType.REVOLUTE,
        parent=side_arm,
        child=pan_head,
        origin=Origin(xyz=(0.198, 0.0, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_head,
        origin=Origin(xyz=(0.045, 0.0, 0.058)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=-1.15,
            upper=0.52,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    base_column = object_model.get_part("base_column")
    inner_post = object_model.get_part("inner_post")
    side_arm = object_model.get_part("side_arm")
    pan_head = object_model.get_part("pan_head")
    camera_head = object_model.get_part("camera_head")

    extension = object_model.get_articulation("telescoping_extension")
    pan = object_model.get_articulation("side_arm_pan")
    tilt = object_model.get_articulation("camera_tilt")

    ctx.check(
        "extension_axis_is_vertical",
        tuple(extension.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical prismatic axis, got {extension.axis!r}",
    )
    ctx.check(
        "pan_axis_is_vertical",
        tuple(pan.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical pan axis, got {pan.axis!r}",
    )
    ctx.check(
        "tilt_axis_is_horizontal",
        tuple(tilt.axis) == (0.0, -1.0, 0.0),
        f"Expected horizontal tilt axis, got {tilt.axis!r}",
    )

    ctx.expect_contact(
        inner_post,
        base_column,
        contact_tol=1e-4,
        name="inner_post_guided_inside_base",
    )
    ctx.expect_contact(
        side_arm,
        inner_post,
        contact_tol=1e-5,
        name="side_arm_mounted_to_post",
    )
    ctx.expect_contact(
        pan_head,
        side_arm,
        contact_tol=1e-5,
        name="pan_head_seated_on_arm",
    )
    ctx.expect_contact(
        camera_head,
        pan_head,
        contact_tol=1e-5,
        name="camera_head_supported_by_pan_yoke",
    )
    ctx.expect_overlap(
        inner_post,
        base_column,
        axes="xy",
        min_overlap=0.07,
        name="post_remains_centered_in_base",
    )

    side_arm_rest = ctx.part_world_position(side_arm)
    ctx.check(
        "side_arm_rest_pose_available",
        side_arm_rest is not None,
        "Side arm world position could not be resolved.",
    )
    if side_arm_rest is not None:
        with ctx.pose({extension: 0.45}):
            side_arm_raised = ctx.part_world_position(side_arm)
            ctx.check(
                "telescoping_extension_lifts_side_arm",
                side_arm_raised is not None
                and side_arm_raised[2] > side_arm_rest[2] + 0.40,
                f"Expected side arm to rise by at least 0.40 m, got rest={side_arm_rest!r}, raised={side_arm_raised!r}",
            )

    camera_rest = ctx.part_world_position(camera_head)
    ctx.check(
        "camera_rest_pose_available",
        camera_rest is not None,
        "Camera head world position could not be resolved.",
    )
    if camera_rest is not None:
        with ctx.pose({pan: math.pi * 0.5}):
            camera_panned = ctx.part_world_position(camera_head)
            ctx.check(
                "pan_moves_camera_laterally",
                camera_panned is not None
                and camera_panned[1] > camera_rest[1] + 0.03
                and abs(camera_panned[2] - camera_rest[2]) < 0.01,
                f"Expected pan motion to swing camera around vertical axis, got rest={camera_rest!r}, panned={camera_panned!r}",
            )

    lens_rest = ctx.part_element_world_aabb(camera_head, elem="lens_glass")
    ctx.check(
        "lens_element_available",
        lens_rest is not None,
        "Lens glass element AABB could not be resolved.",
    )
    if lens_rest is not None:
        lens_rest_center_z = (lens_rest[0][2] + lens_rest[1][2]) * 0.5
        with ctx.pose({tilt: 0.35}):
            lens_tilted = ctx.part_element_world_aabb(camera_head, elem="lens_glass")
            lens_tilted_center_z = None
            if lens_tilted is not None:
                lens_tilted_center_z = (lens_tilted[0][2] + lens_tilted[1][2]) * 0.5
            ctx.check(
                "tilt_raises_camera_nose",
                lens_tilted_center_z is not None
                and lens_tilted_center_z > lens_rest_center_z + 0.03,
                f"Expected positive tilt to raise the lens, got rest_z={lens_rest_center_z!r}, tilted_z={lens_tilted_center_z!r}",
            )
            ctx.expect_contact(
                camera_head,
                pan_head,
                contact_tol=1e-5,
                name="camera_remains_supported_when_tilted",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
