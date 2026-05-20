from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    DomeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _make_clamp_carriage_mesh():
    geom = BoxGeometry((0.095, 0.012, 0.095)).translate(0.0, -0.026, 0.0)
    geom.merge(BoxGeometry((0.095, 0.052, 0.014)).translate(0.0, -0.006, 0.021))
    geom.merge(BoxGeometry((0.095, 0.052, 0.014)).translate(0.0, -0.006, -0.021))
    geom.merge(BoxGeometry((0.030, 0.020, 0.040)).translate(0.0, 0.030, 0.0))

    screw_shaft = CylinderGeometry(radius=0.006, height=0.018).rotate_x(math.pi / 2.0)
    screw_shaft.translate(0.0, 0.033, 0.0)
    geom.merge(screw_shaft)

    screw_pad = CylinderGeometry(radius=0.010, height=0.004).rotate_x(math.pi / 2.0)
    screw_pad.translate(0.0, 0.022, 0.0)
    geom.merge(screw_pad)

    screw_knob = CylinderGeometry(radius=0.013, height=0.012).rotate_x(math.pi / 2.0)
    screw_knob.translate(0.0, 0.044, 0.0)
    geom.merge(screw_knob)

    arm_tube = tube_from_spline_points(
        [
            (0.0, 0.036, 0.0),
            (0.0, 0.054, 0.002),
            (0.0, 0.072, 0.004),
            (0.0, 0.084, 0.005),
        ],
        radius=0.009,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    geom.merge(arm_tube)

    arm_saddle = BoxGeometry((0.020, 0.028, 0.012)).translate(0.0, 0.096, 0.006)
    geom.merge(arm_saddle)

    pedestal = CylinderGeometry(radius=0.018, height=0.012).translate(0.0, 0.112, 0.006)
    geom.merge(pedestal)
    return geom


def _make_pan_yoke_mesh():
    geom = CylinderGeometry(radius=0.016, height=0.010).translate(0.0, 0.0, 0.005)
    geom.merge(CylinderGeometry(radius=0.012, height=0.012).translate(0.0, 0.0, 0.011))
    geom.merge(BoxGeometry((0.016, 0.056, 0.014)).translate(0.0, 0.028, 0.014))
    geom.merge(BoxGeometry((0.076, 0.010, 0.010)).translate(0.0, 0.060, 0.025))
    geom.merge(BoxGeometry((0.008, 0.014, 0.038)).translate(0.034, 0.060, 0.001))
    geom.merge(BoxGeometry((0.008, 0.014, 0.038)).translate(-0.034, 0.060, 0.001))
    return geom


def _make_camera_body_mesh():
    geom = CylinderGeometry(radius=0.021, height=0.018).translate(0.0, 0.036, -0.024)
    geom.merge(CylinderGeometry(radius=0.012, height=0.016).translate(0.0, 0.032, -0.010))
    geom.merge(CylinderGeometry(radius=0.026, height=0.008).translate(0.0, 0.036, -0.034))

    axle = CylinderGeometry(radius=0.004, height=0.060).rotate_y(math.pi / 2.0)
    geom.merge(axle)
    geom.merge(BoxGeometry((0.018, 0.020, 0.012)).translate(0.0, 0.010, -0.010))
    return geom


def _make_dome_cover_mesh():
    return DomeGeometry(radius=0.027, radial_segments=32, height_segments=14).rotate_y(math.pi).translate(
        0.0, 0.036, -0.018
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fence_clamp_camera_bracket")

    rail_metal = model.material("rail_metal", rgba=(0.54, 0.56, 0.58, 1.0))
    bracket_black = model.material("bracket_black", rgba=(0.10, 0.10, 0.11, 1.0))
    camera_white = model.material("camera_white", rgba=(0.88, 0.89, 0.91, 1.0))
    dome_smoke = model.material("dome_smoke", rgba=(0.28, 0.32, 0.36, 0.35))

    rail = model.part("rail")
    rail.visual(
        Box((0.60, 0.04, 0.028)),
        material=rail_metal,
        name="rail_section",
    )

    clamp_carriage = model.part("clamp_carriage")
    clamp_carriage.visual(
        mesh_from_geometry(_make_clamp_carriage_mesh(), "clamp_carriage"),
        material=bracket_black,
        name="carriage_shell",
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        mesh_from_geometry(_make_pan_yoke_mesh(), "pan_yoke"),
        material=bracket_black,
        name="pan_yoke_shell",
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        mesh_from_geometry(_make_camera_body_mesh(), "camera_body"),
        material=camera_white,
        name="camera_body_shell",
    )
    camera_head.visual(
        mesh_from_geometry(_make_dome_cover_mesh(), "camera_dome"),
        material=dome_smoke,
        name="camera_dome_cover",
    )

    model.articulation(
        "rail_to_clamp_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=clamp_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=-0.18,
            upper=0.18,
        ),
    )

    model.articulation(
        "clamp_to_pan",
        ArticulationType.REVOLUTE,
        parent=clamp_carriage,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.112, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-1.8,
            upper=1.8,
        ),
    )

    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=camera_head,
        origin=Origin(xyz=(0.0, 0.060, 0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.3,
            lower=-1.0,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    clamp_carriage = object_model.get_part("clamp_carriage")
    pan_yoke = object_model.get_part("pan_yoke")
    camera_head = object_model.get_part("camera_head")

    slide = object_model.get_articulation("rail_to_clamp_slide")
    pan = object_model.get_articulation("clamp_to_pan")
    tilt = object_model.get_articulation("pan_to_tilt")

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

    ctx.expect_contact(clamp_carriage, rail, name="clamp carriage grips the rail")
    ctx.expect_contact(pan_yoke, clamp_carriage, name="pan base seats on the arm pedestal")
    ctx.expect_contact(camera_head, pan_yoke, name="camera trunnion seats inside the tilt yoke")
    ctx.expect_gap(
        camera_head,
        rail,
        axis="y",
        min_gap=0.08,
        name="camera head sits clearly outboard of the fence rail",
    )

    ctx.check(
        "slide axis follows rail length",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "pan axis is vertical",
        tuple(pan.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan.axis}",
    )
    ctx.check(
        "tilt axis is horizontal",
        abs(tilt.axis[0]) > 0.99 and abs(tilt.axis[1]) < 1e-9 and abs(tilt.axis[2]) < 1e-9,
        details=f"axis={tilt.axis}",
    )

    rest_slide_pos = ctx.part_world_position(clamp_carriage)
    with ctx.pose({slide: slide.motion_limits.upper}):
        extended_slide_pos = ctx.part_world_position(clamp_carriage)
        ctx.expect_contact(clamp_carriage, rail, name="clamp remains supported at full slide")
        ctx.expect_overlap(
            clamp_carriage,
            rail,
            axes="x",
            min_overlap=0.09,
            name="slider carriage remains engaged on the rail",
        )
    ctx.check(
        "clamp translates along the rail",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[0] > rest_slide_pos[0] + 0.15,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_body_center = aabb_center(ctx.part_element_world_aabb(camera_head, elem="camera_body_shell"))
    with ctx.pose({pan: 1.2}):
        panned_body_center = aabb_center(ctx.part_element_world_aabb(camera_head, elem="camera_body_shell"))
    ctx.check(
        "pan joint swings the camera around the arm tip",
        rest_body_center is not None
        and panned_body_center is not None
        and abs(panned_body_center[0] - rest_body_center[0]) > 0.04
        and abs(panned_body_center[1] - rest_body_center[1]) > 0.02,
        details=f"rest={rest_body_center}, panned={panned_body_center}",
    )

    with ctx.pose({tilt: tilt.motion_limits.lower}):
        low_tilt_center = aabb_center(ctx.part_element_world_aabb(camera_head, elem="camera_body_shell"))
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        high_tilt_center = aabb_center(ctx.part_element_world_aabb(camera_head, elem="camera_body_shell"))
    ctx.check(
        "tilt joint changes the camera pitch",
        low_tilt_center is not None
        and high_tilt_center is not None
        and abs(high_tilt_center[1] - low_tilt_center[1]) > 0.02
        and abs(high_tilt_center[2] - low_tilt_center[2]) > 0.002,
        details=f"low={low_tilt_center}, high={high_tilt_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
