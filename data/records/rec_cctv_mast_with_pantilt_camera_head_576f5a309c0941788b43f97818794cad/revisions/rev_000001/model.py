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
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="street_pole_cctv_mast")

    concrete = model.material("concrete", rgba=(0.71, 0.72, 0.71, 1.0))
    galvanized = model.material("galvanized", rgba=(0.56, 0.59, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    camera_white = model.material("camera_white", rgba=(0.83, 0.85, 0.87, 1.0))
    lens_black = model.material("lens_black", rgba=(0.06, 0.07, 0.09, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.68, 0.68, 0.52)),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=concrete,
        name="anchor_base",
    )
    mast.visual(
        Box((0.28, 0.28, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        material=dark_steel,
        name="base_plate",
    )
    for index, x_sign in enumerate((-1.0, 1.0)):
        for y_index, y_sign in enumerate((-1.0, 1.0)):
            mast.visual(
                Cylinder(radius=0.014, length=0.08),
                origin=Origin(xyz=(0.10 * x_sign, 0.10 * y_sign, 0.56)),
                material=dark_steel,
                name=f"anchor_bolt_{index}_{y_index}",
            )

    pole_profile = [
        (0.0, 0.55),
        (0.122, 0.55),
        (0.118, 1.10),
        (0.100, 2.40),
        (0.082, 4.70),
        (0.066, 5.20),
        (0.066, 5.62),
        (0.0, 5.62),
    ]
    mast.visual(
        _mesh("mast_pole", LatheGeometry(pole_profile, segments=72)),
        material=galvanized,
        name="pole_shaft",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.68, 0.68, 5.62)),
        mass=1850.0,
        origin=Origin(xyz=(0.0, 0.0, 2.81)),
    )

    top_bracket = model.part("top_bracket")
    top_bracket.visual(
        _mesh(
            "clamp_collar",
            LatheGeometry.from_shell_profiles(
                [(0.100, -0.09), (0.100, 0.09)],
                [(0.0655, -0.09), (0.0655, 0.09)],
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=dark_steel,
        name="clamp_collar",
    )
    top_bracket.visual(
        Box((0.060, 0.050, 0.18)),
        origin=Origin(xyz=(-0.125, 0.0, 0.0)),
        material=dark_steel,
        name="clamp_ear",
    )
    top_bracket.visual(
        Box((0.100, 0.120, 0.080)),
        origin=Origin(xyz=(0.180, 0.0, -0.010)),
        material=dark_steel,
        name="arm_transition",
    )
    top_bracket.visual(
        Box((0.16, 0.14, 0.12)),
        origin=Origin(xyz=(0.15, 0.0, 0.02)),
        material=dark_steel,
        name="arm_boss",
    )
    top_bracket.visual(
        _mesh(
            "side_arm_tube",
            tube_from_spline_points(
                [
                    (0.16, 0.0, 0.04),
                    (0.46, 0.0, 0.10),
                    (0.82, 0.0, 0.14),
                ],
                radius=0.036,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=galvanized,
        name="side_arm",
    )
    top_bracket.visual(
        _mesh(
            "arm_brace_tube",
            tube_from_spline_points(
                [
                    (0.10, 0.0, -0.03),
                    (0.30, 0.0, 0.03),
                    (0.58, 0.0, 0.10),
                ],
                radius=0.020,
                samples_per_segment=16,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=dark_steel,
        name="brace_strut",
    )
    top_bracket.visual(
        Cylinder(radius=0.050, length=0.12),
        origin=Origin(xyz=(0.82, 0.0, 0.08)),
        material=dark_steel,
        name="pan_mount",
    )
    top_bracket.visual(
        Cylinder(radius=0.009, length=0.12),
        origin=Origin(
            xyz=(-0.125, 0.0, -0.01),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="clamp_bolt",
    )
    top_bracket.inertial = Inertial.from_geometry(
        Box((1.00, 0.22, 0.34)),
        mass=42.0,
        origin=Origin(xyz=(0.40, 0.0, 0.05)),
    )

    model.articulation(
        "mast_to_top_bracket",
        ArticulationType.FIXED,
        parent=mast,
        child=top_bracket,
        origin=Origin(xyz=(0.0, 0.0, 5.34)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.056, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=dark_steel,
        name="swivel_cap",
    )
    pan_yoke.visual(
        Box((0.050, 0.070, 0.084)),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=dark_steel,
        name="hanger_block",
    )
    pan_yoke.visual(
        Box((0.08, 0.22, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.121)),
        material=dark_steel,
        name="yoke_crosshead",
    )
    pan_yoke.visual(
        Box((0.032, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, 0.09, -0.213)),
        material=dark_steel,
        name="left_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.032, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, -0.09, -0.213)),
        material=dark_steel,
        name="right_yoke_arm",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.10, 0.22, 0.30)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
    )

    model.articulation(
        "camera_pan",
        ArticulationType.REVOLUTE,
        parent=top_bracket,
        child=pan_yoke,
        origin=Origin(xyz=(0.82, 0.0, 0.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Cylinder(radius=0.011, length=0.162),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_axle",
    )
    camera_head.visual(
        _mesh(
            "camera_body_shell",
            LatheGeometry(
                [
                    (0.0, -0.065),
                    (0.032, -0.065),
                    (0.046, -0.040),
                    (0.054, 0.050),
                    (0.054, 0.185),
                    (0.066, 0.205),
                    (0.066, 0.245),
                    (0.044, 0.270),
                    (0.0, 0.270),
                ],
                segments=56,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="camera_shell",
    )
    camera_head.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.268, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="lens_bezel",
    )
    camera_head.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.274, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="lens_window",
    )
    camera_head.visual(
        Box((0.12, 0.09, 0.016)),
        origin=Origin(xyz=(0.205, 0.0, 0.038)),
        material=camera_white,
        name="sunshield",
    )
    camera_head.inertial = Inertial.from_geometry(
        Box((0.34, 0.17, 0.15)),
        mass=6.5,
        origin=Origin(xyz=(0.11, 0.0, -0.01)),
    )

    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=camera_head,
        origin=Origin(xyz=(0.0, 0.0, -0.213)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-0.55,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    top_bracket = object_model.get_part("top_bracket")
    pan_yoke = object_model.get_part("pan_yoke")
    camera_head = object_model.get_part("camera_head")
    pan = object_model.get_articulation("camera_pan")
    tilt = object_model.get_articulation("camera_tilt")

    def elem_center(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return tuple((low + high) * 0.5 for low, high in zip(aabb[0], aabb[1]))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        mast,
        top_bracket,
        elem_a="pole_shaft",
        elem_b="clamp_collar",
        reason="The split clamp collar is intentionally authored as a tight steel shell around the tapered pole.",
    )

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

    ctx.expect_contact(
        top_bracket,
        mast,
        elem_a="clamp_collar",
        elem_b="pole_shaft",
        name="clamp collar bears on the pole section",
    )
    ctx.expect_origin_distance(
        top_bracket,
        mast,
        axes="xy",
        max_dist=0.01,
        name="clamp collar remains concentric with the pole",
    )
    ctx.expect_contact(
        pan_yoke,
        top_bracket,
        elem_a="swivel_cap",
        elem_b="pan_mount",
        name="pan swivel seats on the arm tip pod",
    )
    ctx.expect_contact(
        camera_head,
        pan_yoke,
        elem_a="tilt_axle",
        elem_b="left_yoke_arm",
        name="tilt axle bears on the left yoke arm",
    )
    ctx.expect_contact(
        camera_head,
        pan_yoke,
        elem_a="tilt_axle",
        elem_b="right_yoke_arm",
        name="tilt axle bears on the right yoke arm",
    )

    rest_lens = elem_center(camera_head, "lens_window")
    with ctx.pose({pan: 1.0}):
        panned_lens = elem_center(camera_head, "lens_window")
    ctx.check(
        "positive pan swings the camera toward +Y",
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[1] > rest_lens[1] + 0.15,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    with ctx.pose({tilt: 0.70}):
        tilted_lens = elem_center(camera_head, "lens_window")
    ctx.check(
        "positive tilt aims the camera downward",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] < rest_lens[2] - 0.05,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    mast_aabb = ctx.part_world_aabb(mast)
    camera_aabb = ctx.part_world_aabb(camera_head)
    ctx.check(
        "mast reaches realistic street-monitoring height",
        mast_aabb is not None and mast_aabb[1][2] > 5.5,
        details=f"mast_aabb={mast_aabb}",
    )
    ctx.check(
        "camera projects well out beyond the pole centerline",
        mast_aabb is not None
        and camera_aabb is not None
        and camera_aabb[1][0] > mast_aabb[1][0] + 0.45,
        details=f"mast_aabb={mast_aabb}, camera_aabb={camera_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
