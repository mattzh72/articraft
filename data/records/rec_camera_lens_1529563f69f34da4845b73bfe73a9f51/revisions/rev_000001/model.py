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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telephoto_zoom_lens")

    lens_white = model.material("lens_white", rgba=(0.88, 0.89, 0.91, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.56, 0.58, 0.60, 1.0))

    def shell_mesh(name: str, outer_profile, inner_profile, *, segments: int = 72):
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=segments,
                start_cap="flat",
                end_cap="flat",
                lip_samples=8,
            ),
            name,
        )

    main_body = model.part("main_body")
    main_body.visual(
        shell_mesh(
            "main_body_shell",
            [
                (0.042, 0.008),
                (0.044, 0.030),
                (0.046, 0.070),
                (0.046, 0.118),
                (0.048, 0.150),
                (0.047, 0.176),
                (0.046, 0.186),
            ],
            [
                (0.034, 0.008),
                (0.035, 0.030),
                (0.036, 0.070),
                (0.037, 0.118),
                (0.038, 0.150),
                (0.037, 0.176),
                (0.036, 0.186),
            ],
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_white,
        name="body_shell",
    )
    main_body.visual(
        Cylinder(radius=0.046, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_metal,
        name="mount_flange",
    )
    main_body.visual(
        Cylinder(radius=0.050, length=0.036),
        origin=Origin(xyz=(0.118, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="focus_ring",
    )
    main_body.visual(
        Cylinder(radius=0.051, length=0.010),
        origin=Origin(xyz=(0.181, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_metal,
        name="front_bearing_ring",
    )
    main_body.visual(
        Box((0.028, 0.004, 0.020)),
        origin=Origin(xyz=(0.060, -0.044, 0.010)),
        material=dark_gray,
        name="switch_panel",
    )
    main_body.visual(
        Box((0.014, 0.005, 0.007)),
        origin=Origin(xyz=(0.060, -0.0455, 0.022)),
        material=satin_black,
        name="af_switch",
    )
    main_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.186),
        mass=1.25,
        origin=Origin(xyz=(0.093, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    zoom_barrel = model.part("zoom_barrel")
    zoom_barrel.visual(
        shell_mesh(
            "zoom_barrel_shell",
            [
                (0.051, 0.000),
                (0.052, 0.016),
                (0.056, 0.065),
                (0.060, 0.120),
                (0.064, 0.165),
                (0.067, 0.190),
            ],
            [
                (0.042, 0.000),
                (0.043, 0.016),
                (0.046, 0.065),
                (0.050, 0.120),
                (0.054, 0.165),
                (0.057, 0.190),
            ],
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_white,
        name="zoom_shell",
    )
    zoom_barrel.visual(
        Cylinder(radius=0.062, length=0.082),
        origin=Origin(xyz=(0.092, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="zoom_grip",
    )
    zoom_barrel.visual(
        Cylinder(radius=0.069, length=0.010),
        origin=Origin(xyz=(0.176, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="front_trim_ring",
    )
    zoom_barrel.visual(
        Cylinder(radius=0.051, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="rear_drive_collar",
    )
    zoom_barrel.visual(
        Box((0.030, 0.010, 0.004)),
        origin=Origin(xyz=(0.072, 0.0, 0.064)),
        material=dark_gray,
        name="zoom_marker_panel",
    )
    zoom_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.069, length=0.190),
        mass=1.05,
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    tripod_collar = model.part("tripod_collar")
    tripod_collar.visual(
        Box((0.032, 0.096, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=lens_white,
        name="top_bridge",
    )
    tripod_collar.visual(
        Box((0.032, 0.016, 0.100)),
        origin=Origin(xyz=(0.0, -0.054, 0.0)),
        material=lens_white,
        name="left_brace",
    )
    tripod_collar.visual(
        Box((0.032, 0.016, 0.100)),
        origin=Origin(xyz=(0.0, 0.054, 0.0)),
        material=lens_white,
        name="right_brace",
    )
    tripod_collar.visual(
        Box((0.032, 0.096, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=lens_white,
        name="lower_bridge",
    )
    tripod_collar.visual(
        Cylinder(radius=0.0025, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0485), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="top_pad",
    )
    tripod_collar.visual(
        Cylinder(radius=0.0025, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.0485), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="bottom_pad",
    )
    tripod_collar.visual(
        Box((0.028, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.084)),
        material=lens_white,
        name="foot_stem",
    )
    tripod_collar.visual(
        Box((0.072, 0.042, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
        material=dark_gray,
        name="tripod_foot",
    )
    tripod_collar.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.052, -0.058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="collar_lock_knob",
    )
    tripod_collar.inertial = Inertial.from_geometry(
        Box((0.090, 0.070, 0.125)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )

    model.articulation(
        "body_to_zoom",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=zoom_barrel,
        origin=Origin(xyz=(0.186, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "body_to_tripod_collar",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=tripod_collar,
        origin=Origin(xyz=(0.084, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    main_body = object_model.get_part("main_body")
    zoom_barrel = object_model.get_part("zoom_barrel")
    tripod_collar = object_model.get_part("tripod_collar")
    zoom_joint = object_model.get_articulation("body_to_zoom")
    collar_joint = object_model.get_articulation("body_to_tripod_collar")

    ctx.check(
        "zoom joint is optical-axis revolute",
        zoom_joint.axis == (1.0, 0.0, 0.0)
        and zoom_joint.motion_limits is not None
        and zoom_joint.motion_limits.upper is not None
        and abs(zoom_joint.motion_limits.upper - math.radians(95.0)) < 1e-6,
        details=f"axis={zoom_joint.axis}, limits={zoom_joint.motion_limits}",
    )
    ctx.check(
        "tripod collar joint is optical-axis revolute",
        collar_joint.axis == (1.0, 0.0, 0.0)
        and collar_joint.motion_limits is not None
        and collar_joint.motion_limits.lower is not None
        and collar_joint.motion_limits.upper is not None
        and abs(collar_joint.motion_limits.lower + math.pi) < 1e-6
        and abs(collar_joint.motion_limits.upper - math.pi) < 1e-6,
        details=f"axis={collar_joint.axis}, limits={collar_joint.motion_limits}",
    )

    ctx.expect_origin_distance(
        zoom_barrel,
        main_body,
        axes="yz",
        max_dist=0.001,
        name="zoom barrel stays coaxial with main body",
    )
    ctx.expect_origin_distance(
        tripod_collar,
        main_body,
        axes="yz",
        max_dist=0.001,
        name="tripod collar stays coaxial with main body",
    )
    ctx.expect_gap(
        zoom_barrel,
        main_body,
        axis="x",
        positive_elem="zoom_shell",
        negative_elem="body_shell",
        max_gap=0.004,
        max_penetration=0.0,
        name="zoom barrel seats close to the main body shoulder",
    )
    ctx.expect_overlap(
        zoom_barrel,
        main_body,
        axes="yz",
        elem_a="zoom_shell",
        elem_b="body_shell",
        min_overlap=0.084,
        name="zoom barrel remains centered on the optical axis",
    )
    ctx.expect_overlap(
        tripod_collar,
        main_body,
        axes="x",
        min_overlap=0.026,
        name="tripod collar ring wraps the supported body section",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    body_origin = ctx.part_world_position(main_body)
    foot_rest = aabb_center(ctx.part_element_world_aabb(tripod_collar, elem="tripod_foot"))
    marker_rest = aabb_center(ctx.part_element_world_aabb(zoom_barrel, elem="zoom_marker_panel"))

    with ctx.pose({collar_joint: math.pi / 2.0}):
        foot_rotated = aabb_center(
            ctx.part_element_world_aabb(tripod_collar, elem="tripod_foot")
        )

    with ctx.pose({zoom_joint: math.pi / 2.0}):
        marker_rotated = aabb_center(
            ctx.part_element_world_aabb(zoom_barrel, elem="zoom_marker_panel")
        )

    ctx.check(
        "tripod foot rotates from below the lens to the side",
        body_origin is not None
        and foot_rest is not None
        and foot_rotated is not None
        and foot_rest[2] < body_origin[2] - 0.080
        and foot_rotated[1] > body_origin[1] + 0.080
        and abs(foot_rotated[2] - body_origin[2]) < 0.020,
        details=(
            f"body_origin={body_origin}, foot_rest={foot_rest}, "
            f"foot_rotated={foot_rotated}"
        ),
    )
    ctx.check(
        "zoom marker rotates around the optical axis",
        marker_rest is not None
        and marker_rotated is not None
        and marker_rest[2] > 0.055
        and marker_rotated[1] < -0.050
        and abs(marker_rotated[2]) < 0.020,
        details=f"marker_rest={marker_rest}, marker_rotated={marker_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
