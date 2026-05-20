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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_rounded_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for y, z in rounded_rect_profile(
            width,
            height,
            radius,
            corner_segments=8,
        )
    ]


def _aabb_center(aabb):
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cctv_mast")

    galvanized_steel = model.material("galvanized_steel", rgba=(0.63, 0.66, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    camera_white = model.material("camera_white", rgba=(0.86, 0.88, 0.89, 1.0))
    lens_black = model.material("lens_black", rgba=(0.09, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.25, 0.31, 0.42))

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((0.32, 0.32, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_steel,
        name="plate",
    )
    for sx in (-0.110, 0.110):
        for sy in (-0.110, 0.110):
            base_plate.visual(
                Cylinder(radius=0.012, length=0.028),
                origin=Origin(xyz=(sx, sy, 0.038)),
                material=galvanized_steel,
                name=f"anchor_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )
    base_plate.inertial = Inertial.from_geometry(
        Box((0.32, 0.32, 0.052)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    pole = model.part("pole")
    pole_shell = LatheGeometry.from_shell_profiles(
        [(0.055, 0.0), (0.055, 3.000)],
        [(0.049, 0.012), (0.049, 2.988)],
        segments=72,
    )
    pole.visual(
        mesh_from_geometry(pole_shell, "cctv_mast_pole_shell"),
        material=galvanized_steel,
        name="mast_tube",
    )
    pole.visual(
        Cylinder(radius=0.078, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="base_flange",
    )
    pole.visual(
        Cylinder(radius=0.073, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 3.007)),
        material=dark_steel,
        name="top_cap",
    )
    pole.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=3.014),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 1.507)),
    )

    model.articulation(
        "plate_to_pole",
        ArticulationType.FIXED,
        parent=base_plate,
        child=pole,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.082, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="turntable_drum",
    )
    pan_head.visual(
        Cylinder(radius=0.060, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_steel,
        name="azimuth_motor",
    )
    pan_head.visual(
        Box((0.088, 0.150, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=dark_steel,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.028, 0.014, 0.150)),
        origin=Origin(xyz=(0.0, 0.068, 0.173)),
        material=dark_steel,
        name="left_arm",
    )
    pan_head.visual(
        Box((0.028, 0.014, 0.150)),
        origin=Origin(xyz=(0.0, -0.068, 0.173)),
        material=dark_steel,
        name="right_arm",
    )
    pan_head.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.079, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized_steel,
        name="left_bearing_cap",
    )
    pan_head.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, -0.079, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized_steel,
        name="right_bearing_cap",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.18, 0.17, 0.26)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )

    model.articulation(
        "pole_to_pan",
        ArticulationType.REVOLUTE,
        parent=pole,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 3.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    camera_head = model.part("camera_head")
    camera_body = section_loft(
        [
            _yz_rounded_section(-0.085, width=0.110, height=0.082, radius=0.016),
            _yz_rounded_section(0.030, width=0.110, height=0.082, radius=0.016),
            _yz_rounded_section(0.125, width=0.096, height=0.074, radius=0.014),
        ]
    )
    camera_head.visual(
        mesh_from_geometry(camera_body, "cctv_camera_body"),
        material=camera_white,
        name="camera_body",
    )
    camera_head.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=Origin(xyz=(0.164, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="lens_hood",
    )
    camera_head.visual(
        Cylinder(radius=0.021, length=0.060),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="lens_barrel",
    )
    camera_head.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.185, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens_glass",
    )
    camera_head.visual(
        Box((0.118, 0.110, 0.006)),
        origin=Origin(xyz=(0.090, 0.0, 0.044)),
        material=camera_white,
        name="sunshield",
    )
    camera_head.visual(
        Box((0.062, 0.010, 0.012)),
        origin=Origin(xyz=(0.054, 0.034, 0.046)),
        material=camera_white,
        name="sunshield_left_bracket",
    )
    camera_head.visual(
        Box((0.062, 0.010, 0.012)),
        origin=Origin(xyz=(0.054, -0.034, 0.046)),
        material=camera_white,
        name="sunshield_right_bracket",
    )
    camera_head.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    camera_head.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, -0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    camera_head.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(-0.092, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_gland",
    )
    camera_head.inertial = Inertial.from_geometry(
        Box((0.295, 0.122, 0.100)),
        mass=3.2,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    model.articulation(
        "pan_to_camera_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_head,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-1.10,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    pole = object_model.get_part("pole")
    pan_head = object_model.get_part("pan_head")
    camera_head = object_model.get_part("camera_head")

    pan_joint = object_model.get_articulation("pole_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_camera_tilt")

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

    ctx.expect_contact(pole, base_plate, elem_a="base_flange", elem_b="plate")
    ctx.expect_contact(pan_head, pole, elem_a="turntable_drum", elem_b="top_cap")
    ctx.expect_contact(camera_head, pan_head, elem_a="left_trunnion", elem_b="left_arm")
    ctx.expect_contact(camera_head, pan_head, elem_a="right_trunnion", elem_b="right_arm")

    ctx.check(
        "pan axis is vertical",
        tuple(round(v, 6) for v in pan_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {pan_joint.axis}",
    )
    ctx.check(
        "tilt axis is horizontal",
        tuple(round(v, 6) for v in tilt_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {tilt_joint.axis}",
    )

    lens_rest_aabb = ctx.part_element_world_aabb(camera_head, elem="lens_barrel")
    if lens_rest_aabb is None:
        ctx.fail("lens barrel exists at rest", "camera_head:lens_barrel AABB unavailable")
        return ctx.report()
    lens_rest_center = _aabb_center(lens_rest_aabb)
    ctx.check(
        "camera faces forward at rest",
        lens_rest_center[0] > 0.12 and abs(lens_rest_center[1]) < 0.02,
        details=f"rest lens center {lens_rest_center}",
    )

    with ctx.pose({pan_joint: math.pi / 2.0}):
        lens_pan_aabb = ctx.part_element_world_aabb(camera_head, elem="lens_barrel")
        if lens_pan_aabb is None:
            ctx.fail("lens barrel available in pan pose", "camera_head:lens_barrel AABB unavailable")
        else:
            lens_pan_center = _aabb_center(lens_pan_aabb)
            ctx.check(
                "pan rotates camera around mast",
                abs(lens_pan_center[0]) < 0.035 and lens_pan_center[1] > 0.12,
                details=f"panned lens center {lens_pan_center}",
            )

    with ctx.pose({tilt_joint: -0.60}):
        lens_tilt_aabb = ctx.part_element_world_aabb(camera_head, elem="lens_barrel")
        if lens_tilt_aabb is None:
            ctx.fail("lens barrel available in tilt pose", "camera_head:lens_barrel AABB unavailable")
        else:
            lens_tilt_center = _aabb_center(lens_tilt_aabb)
            ctx.check(
                "tilt raises camera nose",
                lens_tilt_center[2] > lens_rest_center[2] + 0.045,
                details=f"rest {lens_rest_center}, tilted {lens_tilt_center}",
            )
            ctx.expect_contact(camera_head, pan_head, elem_a="left_trunnion", elem_b="left_arm")
            ctx.expect_contact(camera_head, pan_head, elem_a="right_trunnion", elem_b="right_arm")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
