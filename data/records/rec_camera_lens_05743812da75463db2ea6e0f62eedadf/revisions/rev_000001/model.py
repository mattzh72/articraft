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
    TorusGeometry,
    mesh_from_geometry,
)


LENS_AXIS_RPY = (0.0, math.pi / 2.0, 0.0)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _x_axis_origin(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=LENS_AXIS_RPY)


def _shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 84,
):
    return _mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )


def _torus_mesh(
    name: str,
    *,
    radius: float,
    tube: float,
    radial_segments: int = 16,
    tubular_segments: int = 48,
):
    return _mesh(
        name,
        TorusGeometry(
            radius=radius,
            tube=tube,
            radial_segments=radial_segments,
            tubular_segments=tubular_segments,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telephoto_zoom_lens")

    lens_white = model.material("lens_white", rgba=(0.89, 0.89, 0.86, 1.0))
    matte_black = model.material("matte_black", rgba=(0.06, 0.06, 0.07, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.28, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    front_glass = model.material("front_glass", rgba=(0.22, 0.33, 0.39, 0.42))
    accent_red = model.material("accent_red", rgba=(0.70, 0.08, 0.08, 1.0))

    barrel_core = model.part("barrel_core")
    barrel_core.visual(
        Cylinder(radius=0.0340, length=0.3250),
        origin=_x_axis_origin(x=0.1525),
        material=matte_black,
        name="inner_tube",
    )
    barrel_core.visual(
        Cylinder(radius=0.0428, length=0.0180),
        origin=_x_axis_origin(x=0.0090),
        material=lens_white,
        name="rear_shell",
    )
    barrel_core.visual(
        Cylinder(radius=0.0450, length=0.0400),
        origin=_x_axis_origin(x=0.0980),
        material=lens_white,
        name="mid_shell",
    )
    barrel_core.visual(
        Cylinder(radius=0.0488, length=0.0520),
        origin=_x_axis_origin(x=0.1940),
        material=lens_white,
        name="front_shell",
    )
    barrel_core.visual(
        _shell_mesh(
            "objective_shell_v5",
            [
                (0.0492, 0.2700),
                (0.0510, 0.2760),
                (0.0535, 0.2950),
                (0.0560, 0.3070),
                (0.0575, 0.3150),
            ],
            [
                (0.0392, 0.2700),
                (0.0405, 0.2760),
                (0.0420, 0.2950),
                (0.0430, 0.3070),
                (0.0435, 0.3150),
            ],
            segments=84,
        ),
        origin=Origin(rpy=LENS_AXIS_RPY),
        material=lens_white,
        name="objective_shell",
    )
    barrel_core.visual(
        Cylinder(radius=0.0365, length=0.0060),
        origin=_x_axis_origin(x=-0.0090),
        material=mount_metal,
        name="rear_mount_flange",
    )
    barrel_core.visual(
        Cylinder(radius=0.0315, length=0.0040),
        origin=_x_axis_origin(x=-0.0015),
        material=matte_black,
        name="rear_gasket",
    )
    barrel_core.visual(
        Cylinder(radius=0.0400, length=0.0030),
        origin=_x_axis_origin(x=0.0165),
        material=dark_gray,
        name="rear_grip_step",
    )
    barrel_core.visual(
        Cylinder(radius=0.0420, length=0.0040),
        origin=_x_axis_origin(x=0.0810),
        material=dark_gray,
        name="mid_step",
    )
    barrel_core.visual(
        Cylinder(radius=0.0460, length=0.0050),
        origin=_x_axis_origin(x=0.1760),
        material=dark_gray,
        name="front_step",
    )
    barrel_core.visual(
        Cylinder(radius=0.0545, length=0.0030),
        origin=_x_axis_origin(x=0.2760),
        material=accent_red,
        name="accent_ring",
    )
    barrel_core.visual(
        Cylinder(radius=0.0575, length=0.0040),
        origin=_x_axis_origin(x=0.3070),
        material=matte_black,
        name="filter_rim",
    )
    barrel_core.visual(
        Cylinder(radius=0.0410, length=0.0040),
        origin=_x_axis_origin(x=0.3130),
        material=front_glass,
        name="front_glass",
    )
    barrel_core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0590, length=0.3250),
        mass=1.40,
        origin=_x_axis_origin(x=0.1525),
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        _shell_mesh(
            "zoom_ring_shell_v5",
            [
                (0.0455, -0.0300),
                (0.0464, -0.0240),
                (0.0468, 0.0000),
                (0.0464, 0.0240),
                (0.0455, 0.0300),
            ],
            [
                (0.0410, -0.0300),
                (0.0410, 0.0300),
            ],
            segments=84,
        ),
        origin=Origin(rpy=LENS_AXIS_RPY),
        material=rubber_black,
        name="zoom_sleeve",
    )
    zoom_ring.visual(
        Box((0.0120, 0.0060, 0.0030)),
        origin=Origin(xyz=(0.0, 0.0, 0.0454)),
        material=dark_gray,
        name="zoom_marker",
    )
    zoom_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0470, length=0.0600),
        mass=0.18,
        origin=Origin(rpy=LENS_AXIS_RPY),
    )

    tripod_collar = model.part("tripod_collar")
    tripod_collar.visual(
        _shell_mesh(
            "tripod_collar_band_v5",
            [
                (0.0496, -0.0250),
                (0.0502, -0.0200),
                (0.0502, 0.0200),
                (0.0496, 0.0250),
            ],
            [
                (0.0442, -0.0250),
                (0.0442, 0.0250),
            ],
            segments=84,
        ),
        origin=Origin(rpy=LENS_AXIS_RPY),
        material=dark_gray,
        name="collar_band",
    )
    tripod_collar.visual(
        Cylinder(radius=0.0060, length=0.0180),
        origin=Origin(xyz=(0.0, 0.0549, 0.0000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="clamp_knob",
    )
    tripod_collar.visual(
        Box((0.0340, 0.0180, 0.0240)),
        origin=Origin(xyz=(0.0, 0.0, -0.0615)),
        material=dark_gray,
        name="foot_neck",
    )
    tripod_collar.visual(
        Box((0.0880, 0.0360, 0.0100)),
        origin=Origin(xyz=(0.0, 0.0, -0.0785)),
        material=matte_black,
        name="foot_pad",
    )
    tripod_collar.visual(
        Box((0.0600, 0.0180, 0.0040)),
        origin=Origin(xyz=(0.0, 0.0, -0.0840)),
        material=dark_gray,
        name="foot_rubber_strip",
    )
    tripod_collar.inertial = Inertial.from_geometry(
        Box((0.0900, 0.0800, 0.1000)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.0440)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _shell_mesh(
            "focus_ring_shell_v5",
            [
                (0.0526, -0.0250),
                (0.0533, -0.0200),
                (0.0540, 0.0000),
                (0.0533, 0.0200),
                (0.0526, 0.0250),
            ],
            [
                (0.0482, -0.0250),
                (0.0482, 0.0250),
            ],
            segments=84,
        ),
        origin=Origin(rpy=LENS_AXIS_RPY),
        material=rubber_black,
        name="focus_sleeve",
    )
    focus_ring.visual(
        Box((0.0100, 0.0050, 0.0030)),
        origin=Origin(xyz=(0.0, 0.0, 0.0528)),
        material=dark_gray,
        name="focus_marker",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0540, length=0.0500),
        mass=0.14,
        origin=Origin(rpy=LENS_AXIS_RPY),
    )

    model.articulation(
        "zoom_rotation",
        ArticulationType.REVOLUTE,
        parent=barrel_core,
        child=zoom_ring,
        origin=Origin(xyz=(0.0480, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "tripod_collar_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel_core,
        child=tripod_collar,
        origin=Origin(xyz=(0.1430, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
        ),
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.REVOLUTE,
        parent=barrel_core,
        child=focus_ring,
        origin=Origin(xyz=(0.2450, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=-1.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel_core = object_model.get_part("barrel_core")
    zoom_ring = object_model.get_part("zoom_ring")
    tripod_collar = object_model.get_part("tripod_collar")
    focus_ring = object_model.get_part("focus_ring")

    zoom_rotation = object_model.get_articulation("zoom_rotation")
    tripod_collar_rotation = object_model.get_articulation("tripod_collar_rotation")
    focus_rotation = object_model.get_articulation("focus_rotation")

    front_glass = barrel_core.get_visual("front_glass")
    zoom_marker = zoom_ring.get_visual("zoom_marker")
    focus_marker = focus_ring.get_visual("focus_marker")
    foot_pad = tripod_collar.get_visual("foot_pad")

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(max_pose_samples=12)
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(zoom_ring, barrel_core, name="zoom_ring_is_mounted")
    ctx.expect_contact(focus_ring, barrel_core, name="focus_ring_is_mounted")
    ctx.expect_contact(tripod_collar, barrel_core, name="tripod_collar_is_mounted")

    ctx.expect_overlap(zoom_ring, barrel_core, axes="yz", min_overlap=0.080)
    ctx.expect_overlap(focus_ring, barrel_core, axes="yz", min_overlap=0.094)
    ctx.expect_overlap(tripod_collar, barrel_core, axes="yz", min_overlap=0.086)

    zoom_pos = ctx.part_world_position(zoom_ring)
    collar_pos = ctx.part_world_position(tripod_collar)
    focus_pos = ctx.part_world_position(focus_ring)
    barrel_aabb = ctx.part_world_aabb(barrel_core)
    front_glass_aabb = ctx.part_element_world_aabb(barrel_core, elem=front_glass)
    zoom_marker_aabb = ctx.part_element_world_aabb(zoom_ring, elem=zoom_marker)
    focus_marker_aabb = ctx.part_element_world_aabb(focus_ring, elem=focus_marker)
    foot_pad_aabb = ctx.part_element_world_aabb(tripod_collar, elem=foot_pad)

    if zoom_pos is None or collar_pos is None or focus_pos is None:
        ctx.fail("ring_world_positions_exist", "One or more articulated ring positions could not be resolved.")
        return ctx.report()
    if (
        barrel_aabb is None
        or front_glass_aabb is None
        or zoom_marker_aabb is None
        or focus_marker_aabb is None
        or foot_pad_aabb is None
    ):
        ctx.fail("visual_aabbs_exist", "Expected barrel or marker element AABBs were unavailable.")
        return ctx.report()

    barrel_length = barrel_aabb[1][0] - barrel_aabb[0][0]
    barrel_diameter = max(
        barrel_aabb[1][1] - barrel_aabb[0][1],
        barrel_aabb[1][2] - barrel_aabb[0][2],
    )
    ctx.check(
        "telephoto_proportions",
        0.30 <= barrel_length <= 0.34 and 0.10 <= barrel_diameter <= 0.12,
        f"Expected a long telephoto body around 0.32 m x 0.11 m, got {barrel_length:.4f} m x {barrel_diameter:.4f} m.",
    )
    ctx.check(
        "rings_progress_forward_along_barrel",
        0.03 <= zoom_pos[0] <= 0.07
        and 0.12 <= collar_pos[0] <= 0.17
        and 0.22 <= focus_pos[0] <= 0.27
        and zoom_pos[0] < collar_pos[0] < focus_pos[0],
        f"Unexpected ring order/positions: zoom={zoom_pos[0]:.4f}, collar={collar_pos[0]:.4f}, focus={focus_pos[0]:.4f}.",
    )
    ctx.check(
        "front_element_near_objective_end",
        front_glass_aabb[1][0] >= barrel_aabb[1][0] - 0.006,
        f"Front glass should sit near the objective rim, got glass max x={front_glass_aabb[1][0]:.4f} and barrel max x={barrel_aabb[1][0]:.4f}.",
    )

    zoom_marker_rest = _aabb_center(zoom_marker_aabb)
    focus_marker_rest = _aabb_center(focus_marker_aabb)
    foot_pad_rest = _aabb_center(foot_pad_aabb)

    ctx.check(
        "tripod_foot_starts_below_barrel",
        foot_pad_rest[2] < barrel_aabb[0][2] - 0.020,
        f"Tripod foot should hang below the barrel, got foot z={foot_pad_rest[2]:.4f} and barrel min z={barrel_aabb[0][2]:.4f}.",
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    for joint, child_part, marker, label in (
        (zoom_rotation, zoom_ring, zoom_marker, "zoom_rotation"),
        (focus_rotation, focus_ring, focus_marker, "focus_rotation"),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_lower_no_floating")
                ctx.expect_contact(child_part, barrel_core, name=f"{label}_lower_stays_mounted")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_upper_no_floating")
                ctx.expect_contact(child_part, barrel_core, name=f"{label}_upper_stays_mounted")

    with ctx.pose({zoom_rotation: zoom_rotation.motion_limits.upper}):
        zoom_marker_upper_aabb = ctx.part_element_world_aabb(zoom_ring, elem=zoom_marker)
        if zoom_marker_upper_aabb is None:
            ctx.fail("zoom_marker_upper_aabb_exists", "Zoom marker AABB unavailable at upper zoom pose.")
        else:
            zoom_marker_upper = _aabb_center(zoom_marker_upper_aabb)
            rest_radius = math.hypot(zoom_marker_rest[1], zoom_marker_rest[2])
            upper_radius = math.hypot(zoom_marker_upper[1], zoom_marker_upper[2])
            ctx.check(
                "zoom_ring_rotates_about_optical_axis",
                abs(zoom_marker_upper[0] - zoom_marker_rest[0]) <= 0.002
                and abs(rest_radius - upper_radius) <= 0.002
                and math.hypot(
                    zoom_marker_upper[1] - zoom_marker_rest[1],
                    zoom_marker_upper[2] - zoom_marker_rest[2],
                )
                >= 0.015,
                (
                    "Zoom marker should orbit around the lens axis without axial drift; "
                    f"rest={zoom_marker_rest}, upper={zoom_marker_upper}."
                ),
            )

    with ctx.pose({focus_rotation: focus_rotation.motion_limits.upper}):
        focus_marker_upper_aabb = ctx.part_element_world_aabb(focus_ring, elem=focus_marker)
        if focus_marker_upper_aabb is None:
            ctx.fail("focus_marker_upper_aabb_exists", "Focus marker AABB unavailable at upper focus pose.")
        else:
            focus_marker_upper = _aabb_center(focus_marker_upper_aabb)
            rest_radius = math.hypot(focus_marker_rest[1], focus_marker_rest[2])
            upper_radius = math.hypot(focus_marker_upper[1], focus_marker_upper[2])
            ctx.check(
                "focus_ring_rotates_about_optical_axis",
                abs(focus_marker_upper[0] - focus_marker_rest[0]) <= 0.002
                and abs(rest_radius - upper_radius) <= 0.002
                and math.hypot(
                    focus_marker_upper[1] - focus_marker_rest[1],
                    focus_marker_upper[2] - focus_marker_rest[2],
                )
                >= 0.030,
                (
                    "Focus marker should orbit around the lens axis without axial drift; "
                    f"rest={focus_marker_rest}, upper={focus_marker_upper}."
                ),
            )

    with ctx.pose({tripod_collar_rotation: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="tripod_collar_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="tripod_collar_quarter_turn_no_floating")
        ctx.expect_contact(tripod_collar, barrel_core, name="tripod_collar_quarter_turn_stays_mounted")
        foot_pad_quarter_aabb = ctx.part_element_world_aabb(tripod_collar, elem=foot_pad)
        if foot_pad_quarter_aabb is None:
            ctx.fail("tripod_foot_quarter_turn_aabb_exists", "Tripod foot AABB unavailable at quarter-turn pose.")
        else:
            foot_pad_quarter = _aabb_center(foot_pad_quarter_aabb)
            ctx.check(
                "tripod_collar_rotates_foot_sideways",
                abs(foot_pad_quarter[0] - foot_pad_rest[0]) <= 0.002
                and abs(foot_pad_quarter[1]) >= 0.070
                and abs(foot_pad_quarter[2]) <= 0.025,
                (
                    "Tripod foot should swing from below the barrel to the side on collar rotation; "
                    f"rest={foot_pad_rest}, quarter={foot_pad_quarter}."
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
