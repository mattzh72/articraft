from __future__ import annotations

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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

LEG_REST_SPREAD = math.radians(41.0)
HIP_RADIUS = 0.058
UPPER_LEG_LENGTH = 0.52
LOWER_LEG_TOTAL = 0.56
LOWER_LEG_INSERT = 0.14
LOWER_LEG_JOINT_Z = -0.49


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _tube_mesh(
    name: str,
    *,
    outer_width: float,
    outer_depth: float,
    inner_width: float,
    inner_depth: float,
    length: float,
    outer_radius: float,
    inner_radius: float,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(
            outer_width,
            outer_depth,
            min(outer_radius, outer_width * 0.49, outer_depth * 0.49),
            corner_segments=8,
        ),
        [
            rounded_rect_profile(
                inner_width,
                inner_depth,
                min(inner_radius, inner_width * 0.49, inner_depth * 0.49),
                corner_segments=8,
            )
        ],
        length,
        cap=True,
        center=True,
        closed=True,
    )
    return _save_mesh(name, geom)


def _plate_mesh(name: str, *, width: float, depth: float, thickness: float, radius: float):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius, corner_segments=10),
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    return _save_mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tripod_mounted_device", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.15, 0.16, 0.18, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.29, 0.31, 0.34, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.67, 0.69, 0.72, 1.0))
    carbon_matte = model.material("carbon_matte", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    clamp_pad = model.material("clamp_pad", rgba=(0.13, 0.14, 0.15, 1.0))

    upper_leg_mesh = _tube_mesh(
        "tripod_upper_leg_tube.obj",
        outer_width=0.032,
        outer_depth=0.022,
        inner_width=0.026,
        inner_depth=0.016,
        length=UPPER_LEG_LENGTH,
        outer_radius=0.005,
        inner_radius=0.002,
    )
    lower_leg_mesh = _tube_mesh(
        "tripod_lower_leg_tube.obj",
        outer_width=0.020,
        outer_depth=0.012,
        inner_width=0.014,
        inner_depth=0.007,
        length=LOWER_LEG_TOTAL,
        outer_radius=0.003,
        inner_radius=0.0015,
    )
    head_plate_mesh = _plate_mesh(
        "tripod_head_plate.obj",
        width=0.090,
        depth=0.058,
        thickness=0.008,
        radius=0.010,
    )

    core = model.part("tripod_core")
    core.visual(
        Cylinder(radius=0.050, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=matte_graphite,
        name="crown_body",
    )
    core.visual(
        Cylinder(radius=0.056, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=satin_graphite,
        name="crown_cap",
    )
    core.visual(
        Cylinder(radius=0.039, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=satin_graphite,
        name="lower_collar",
    )
    core.visual(
        Cylinder(radius=0.041, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=satin_aluminum,
        name="crown_seam_ring",
    )
    core.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=carbon_matte,
        name="center_mast",
    )
    core.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=satin_graphite,
        name="mast_collar",
    )
    core.visual(
        Box((0.038, 0.034, 0.034)),
        origin=Origin(xyz=(0.018, 0.0, 0.130)),
        material=matte_graphite,
        name="yoke_pedestal",
    )
    core.visual(
        Box((0.030, 0.072, 0.014)),
        origin=Origin(xyz=(0.046, 0.0, 0.150)),
        material=satin_graphite,
        name="yoke_bridge",
    )
    core.visual(
        Box((0.014, 0.018, 0.052)),
        origin=Origin(xyz=(0.054, 0.036, 0.143)),
        material=satin_graphite,
        name="left_yoke_arm",
    )
    core.visual(
        Box((0.014, 0.018, 0.052)),
        origin=Origin(xyz=(0.054, -0.036, 0.143)),
        material=satin_graphite,
        name="right_yoke_arm",
    )
    core.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.058, 0.045, 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="left_tilt_knob",
    )
    core.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.058, -0.045, 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="right_tilt_knob",
    )

    hip_yaws = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    upper_legs = []
    lower_legs = []
    for leg_index, yaw in enumerate(hip_yaws):
        radial_x = math.cos(yaw)
        radial_y = math.sin(yaw)
        tangent_x = -math.sin(yaw)
        tangent_y = math.cos(yaw)
        core.visual(
            Box((0.032, 0.024, 0.018)),
            origin=Origin(
                xyz=(0.048 * radial_x, 0.048 * radial_y, 0.002),
                rpy=(0.0, 0.0, yaw),
            ),
            material=satin_graphite,
            name=f"leg_{leg_index}_rib",
        )
        core.visual(
            Box((0.014, 0.008, 0.022)),
            origin=Origin(
                xyz=(
                    0.060 * radial_x + 0.008 * tangent_x,
                    0.060 * radial_y + 0.008 * tangent_y,
                    0.001,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=satin_graphite,
            name=f"leg_{leg_index}_ear_a",
        )
        core.visual(
            Box((0.014, 0.008, 0.022)),
            origin=Origin(
                xyz=(
                    0.060 * radial_x - 0.008 * tangent_x,
                    0.060 * radial_y - 0.008 * tangent_y,
                    0.001,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=satin_graphite,
            name=f"leg_{leg_index}_ear_b",
        )

        upper_leg = model.part(f"upper_leg_{leg_index}")
        upper_leg.visual(
            Cylinder(radius=0.0055, length=0.022),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_aluminum,
            name="hip_axle",
        )
        upper_leg.visual(
            Box((0.022, 0.018, 0.032)),
            origin=Origin(xyz=(0.0, 0.0, -0.016)),
            material=satin_graphite,
            name="hip_block",
        )
        upper_leg.visual(
            upper_leg_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.276)),
            material=carbon_matte,
            name="upper_tube",
        )
        upper_leg.visual(
            Box((0.036, 0.022, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.498)),
            material=satin_graphite,
            name="leg_lock_band",
        )
        upper_leg.visual(
            Box((0.010, 0.006, 0.050)),
            origin=Origin(xyz=(0.0, 0.011, -0.482)),
            material=satin_aluminum,
            name="leg_lock_tab",
        )
        upper_leg.inertial = Inertial.from_geometry(
            Box((0.050, 0.030, 0.560)),
            mass=0.55,
            origin=Origin(xyz=(0.0, 0.0, -0.270)),
        )

        lower_leg = model.part(f"lower_leg_{leg_index}")
        lower_leg.visual(
            Box((0.026, 0.018, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.000)),
            material=satin_aluminum,
            name="slider_cap",
        )
        lower_leg.visual(
            Box((0.020, 0.012, 0.034)),
            origin=Origin(xyz=(0.0, 0.0, -0.023)),
            material=satin_graphite,
            name="slider_neck",
        )
        lower_leg.visual(
            lower_leg_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.180)),
            material=carbon_matte,
            name="lower_tube",
        )
        lower_leg.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.456)),
            material=satin_aluminum,
            name="foot_shoe",
        )
        lower_leg.visual(
            Cylinder(radius=0.016, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, -0.478)),
            material=dark_rubber,
            name="foot_pad",
        )
        lower_leg.inertial = Inertial.from_geometry(
            Box((0.030, 0.022, 0.600)),
            mass=0.32,
            origin=Origin(xyz=(0.0, 0.0, -0.220)),
        )

        model.articulation(
            f"core_to_upper_leg_{leg_index}",
            ArticulationType.REVOLUTE,
            parent=core,
            child=upper_leg,
            origin=Origin(
                xyz=(HIP_RADIUS * math.cos(yaw), HIP_RADIUS * math.sin(yaw), 0.000),
                rpy=(0.0, -LEG_REST_SPREAD, yaw),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.2,
                lower=-math.radians(12.0),
                upper=math.radians(34.0),
            ),
        )
        model.articulation(
            f"upper_to_lower_leg_{leg_index}",
            ArticulationType.PRISMATIC,
            parent=upper_leg,
            child=lower_leg,
            origin=Origin(xyz=(0.0, 0.0, LOWER_LEG_JOINT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.18,
                lower=0.0,
                upper=0.18,
            ),
        )

        upper_legs.append(upper_leg)
        lower_legs.append(lower_leg)

    carriage = model.part("device_carriage")
    carriage.visual(
        Cylinder(radius=0.0055, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="tilt_axle",
    )
    carriage.visual(
        Box((0.020, 0.024, 0.024)),
        origin=Origin(xyz=(0.010, 0.0, 0.000)),
        material=matte_graphite,
        name="pivot_block",
    )
    carriage.visual(
        Box((0.060, 0.020, 0.016)),
        origin=Origin(xyz=(0.050, 0.0, 0.000)),
        material=satin_aluminum,
        name="tilt_arm",
    )
    carriage.visual(
        Box((0.044, 0.042, 0.010)),
        origin=Origin(xyz=(0.080, 0.0, 0.013)),
        material=satin_graphite,
        name="plate_saddle",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.140, 0.060, 0.050)),
        mass=0.32,
        origin=Origin(xyz=(0.060, 0.0, 0.010)),
    )

    head_plate = model.part("head_plate")
    head_plate.visual(
        head_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_graphite,
        name="head_plate_shell",
    )
    head_plate.visual(
        Box((0.070, 0.036, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=satin_aluminum,
        name="head_plate_inset",
    )
    head_plate.visual(
        Box((0.022, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=matte_graphite,
        name="head_plate_center_pad",
    )
    head_plate.inertial = Inertial.from_geometry(
        Box((0.090, 0.058, 0.010)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    bracket = model.part("device_bracket")
    bracket.visual(
        Box((0.020, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_graphite,
        name="riser",
    )
    bracket.visual(
        Box((0.040, 0.010, 0.140)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=matte_graphite,
        name="back_spine",
    )
    bracket.visual(
        Box((0.046, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=satin_aluminum,
        name="top_shoulder",
    )
    bracket.visual(
        Box((0.146, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=satin_graphite,
        name="guide_top",
    )
    bracket.visual(
        Box((0.146, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=satin_graphite,
        name="guide_bottom",
    )
    bracket.visual(
        Box((0.018, 0.024, 0.090)),
        origin=Origin(xyz=(-0.066, 0.0, 0.102)),
        material=matte_graphite,
        name="fixed_jaw",
    )
    bracket.visual(
        Box((0.006, 0.004, 0.078)),
        origin=Origin(xyz=(-0.057, 0.013, 0.102)),
        material=clamp_pad,
        name="fixed_pad",
    )
    bracket.visual(
        Cylinder(radius=0.0095, length=0.018),
        origin=Origin(xyz=(-0.020, -0.010, 0.140), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="clamp_tension_knob",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.170, 0.050, 0.180)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    clamp_jaw = model.part("clamp_jaw")
    clamp_jaw.visual(
        Box((0.104, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=satin_graphite,
        name="runner",
    )
    clamp_jaw.visual(
        Box((0.018, 0.024, 0.090)),
        origin=Origin(xyz=(0.061, 0.0, 0.0)),
        material=matte_graphite,
        name="moving_jaw",
    )
    clamp_jaw.visual(
        Box((0.006, 0.004, 0.078)),
        origin=Origin(xyz=(0.052, 0.013, 0.0)),
        material=clamp_pad,
        name="moving_pad",
    )
    clamp_jaw.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.030, -0.009, -0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="adjust_knob",
    )
    clamp_jaw.inertial = Inertial.from_geometry(
        Box((0.160, 0.040, 0.100)),
        mass=0.16,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    model.articulation(
        "core_to_device_carriage",
        ArticulationType.REVOLUTE,
        parent=core,
        child=carriage,
        origin=Origin(xyz=(0.054, 0.0, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-math.radians(62.0),
            upper=math.radians(48.0),
        ),
    )
    model.articulation(
        "carriage_to_head_plate",
        ArticulationType.FIXED,
        parent=carriage,
        child=head_plate,
        origin=Origin(xyz=(0.092, 0.0, 0.018)),
    )
    model.articulation(
        "head_plate_to_bracket",
        ArticulationType.FIXED,
        parent=head_plate,
        child=bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )
    model.articulation(
        "bracket_to_clamp_jaw",
        ArticulationType.PRISMATIC,
        parent=bracket,
        child=clamp_jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=0.08,
            lower=0.0,
            upper=0.055,
        ),
    )

    core.inertial = Inertial.from_geometry(
        Box((1.30, 1.30, 0.42)),
        mass=3.40,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    core = object_model.get_part("tripod_core")
    carriage = object_model.get_part("device_carriage")
    head_plate = object_model.get_part("head_plate")
    bracket = object_model.get_part("device_bracket")
    clamp_jaw = object_model.get_part("clamp_jaw")

    upper_legs = [object_model.get_part(f"upper_leg_{index}") for index in range(3)]
    lower_legs = [object_model.get_part(f"lower_leg_{index}") for index in range(3)]

    for upper_leg in upper_legs:
        ctx.allow_overlap(
            core,
            upper_leg,
            reason="Leg hip clevises wrap the crown pivots and visually interpenetrate at the hinge hardware.",
        )
    for upper_leg, lower_leg in zip(upper_legs, lower_legs):
        ctx.allow_overlap(
            upper_leg,
            lower_leg,
            reason="Telescoping lower tubes intentionally nest inside the upper leg shells.",
        )
    ctx.allow_overlap(
        core,
        carriage,
        reason="The tilt axle passes through the forward yoke cheeks as an intentional bearing fit.",
    )
    ctx.allow_overlap(
        carriage,
        head_plate,
        reason="The head plate is clamped onto the carriage saddle with a zero-clearance seated interface that evaluates as slight pose-dependent penetration.",
    )
    ctx.allow_overlap(
        head_plate,
        bracket,
        reason="The bracket foot is tightly seated on the head plate with a zero-clearance interface that can register as slight penetration under tilt.",
    )
    ctx.allow_overlap(
        bracket,
        clamp_jaw,
        reason="The clamp runner is intentionally captured inside the bracket guide channel.",
    )

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    hip_joints = [object_model.get_articulation(f"core_to_upper_leg_{index}") for index in range(3)]
    extension_joints = [object_model.get_articulation(f"upper_to_lower_leg_{index}") for index in range(3)]
    tilt_joint = object_model.get_articulation("core_to_device_carriage")
    clamp_joint = object_model.get_articulation("bracket_to_clamp_jaw")

    for leg_index, upper_leg in enumerate(upper_legs):
        ctx.expect_contact(upper_leg, core, name=f"upper_leg_{leg_index}_mounted_to_core")
    for leg_index, (upper_leg, lower_leg) in enumerate(zip(upper_legs, lower_legs)):
        ctx.expect_overlap(
            lower_leg,
            upper_leg,
            axes="xy",
            min_overlap=0.010,
            name=f"lower_leg_{leg_index}_nested_in_upper",
        )

    ctx.expect_contact(carriage, core, name="carriage_supported_in_yoke")
    ctx.expect_contact(head_plate, carriage, name="head_plate_seated_on_carriage")
    ctx.expect_contact(bracket, head_plate, name="bracket_mounted_on_head_plate")
    ctx.expect_overlap(clamp_jaw, bracket, axes="xy", min_overlap=0.012, name="clamp_jaw_guided_in_bracket")

    core_aabb = ctx.part_world_aabb(core)
    if core_aabb is None:
        ctx.fail("core_aabb_available", "Tripod core AABB could not be resolved.")
    else:
        foot_centers: list[tuple[float, float, float]] = []
        for leg_index, lower_leg in enumerate(lower_legs):
            foot_aabb = ctx.part_element_world_aabb(lower_leg, elem="foot_pad")
            if foot_aabb is None:
                ctx.fail(f"lower_leg_{leg_index}_foot_available", "Foot pad AABB could not be resolved.")
                continue
            foot_centers.append(
                (
                    (foot_aabb[0][0] + foot_aabb[1][0]) * 0.5,
                    (foot_aabb[0][1] + foot_aabb[1][1]) * 0.5,
                    (foot_aabb[0][2] + foot_aabb[1][2]) * 0.5,
                )
            )

        if len(foot_centers) == 3:
            radii = [math.hypot(x, y) for x, y, _ in foot_centers]
            pair_dists = [
                math.dist(foot_centers[0], foot_centers[1]),
                math.dist(foot_centers[1], foot_centers[2]),
                math.dist(foot_centers[2], foot_centers[0]),
            ]
            highest_foot = max(center[2] for center in foot_centers)
            ctx.check(
                "tripod_feet_spread_radius",
                min(radii) > 0.38 and max(radii) < 0.82,
                f"Foot radii were {radii}.",
            )
            ctx.check(
                "tripod_feet_triangle_width",
                min(pair_dists) > 0.58,
                f"Foot spacing was {pair_dists}.",
            )
            ctx.check(
                "tripod_feet_below_core",
                highest_foot < core_aabb[0][2] - 0.55,
                f"Highest foot z was {highest_foot:.3f} while core min z was {core_aabb[0][2]:.3f}.",
            )

    rest_bracket_pos = ctx.part_world_position(bracket)
    if rest_bracket_pos is None:
        ctx.fail("rest_bracket_position_available", "Bracket world position could not be resolved.")

    spread_pose = {}
    folded_pose = {}
    for hip_joint in hip_joints:
        limits = hip_joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            spread_pose[hip_joint] = limits.lower
            folded_pose[hip_joint] = limits.upper

    if spread_pose:
        with ctx.pose(spread_pose):
            ctx.fail_if_parts_overlap_in_current_pose(name="legs_spread_pose_clear")
            ctx.fail_if_isolated_parts(name="legs_spread_pose_supported")
            for leg_index, upper_leg in enumerate(upper_legs):
                ctx.expect_contact(upper_leg, core, name=f"upper_leg_{leg_index}_spread_pose_contact")

    if folded_pose:
        with ctx.pose(folded_pose):
            ctx.fail_if_parts_overlap_in_current_pose(name="legs_folded_pose_clear")
            ctx.fail_if_isolated_parts(name="legs_folded_pose_supported")
            for leg_index, upper_leg in enumerate(upper_legs):
                ctx.expect_contact(upper_leg, core, name=f"upper_leg_{leg_index}_folded_pose_contact")

    extension_pose = {}
    for joint in extension_joints:
        limits = joint.motion_limits
        if limits is not None and limits.upper is not None:
            extension_pose[joint] = limits.upper
    if extension_pose:
        rest_lower_positions = [ctx.part_world_position(lower_leg) for lower_leg in lower_legs]
        with ctx.pose(extension_pose):
            ctx.fail_if_parts_overlap_in_current_pose(name="legs_extended_pose_clear")
            ctx.fail_if_isolated_parts(name="legs_extended_pose_supported")
            for leg_index, (lower_leg, rest_pos) in enumerate(zip(lower_legs, rest_lower_positions)):
                moved_pos = ctx.part_world_position(lower_leg)
                if rest_pos is None or moved_pos is None:
                    ctx.fail(
                        f"lower_leg_{leg_index}_extended_position_available",
                        "Lower leg world position could not be resolved.",
                    )
                else:
                    ctx.check(
                        f"lower_leg_{leg_index}_extends_downward",
                        moved_pos[2] < rest_pos[2] - 0.10,
                        f"Lower leg moved from {rest_pos} to {moved_pos}.",
                    )

    tilt_limits = tilt_joint.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt_joint: tilt_limits.lower}):
            ctx.expect_contact(carriage, core, name="tilt_lower_pose_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="tilt_lower_pose_clear")
            ctx.fail_if_isolated_parts(name="tilt_lower_pose_supported")
            lower_tilt_pos = ctx.part_world_position(bracket)
        with ctx.pose({tilt_joint: tilt_limits.upper}):
            ctx.expect_contact(carriage, core, name="tilt_upper_pose_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="tilt_upper_pose_clear")
            ctx.fail_if_isolated_parts(name="tilt_upper_pose_supported")
            upper_tilt_pos = ctx.part_world_position(bracket)
        if lower_tilt_pos is None or upper_tilt_pos is None:
            ctx.fail("tilt_bracket_positions_available", "Bracket positions at tilt extremes could not be resolved.")
        else:
            ctx.check(
                "tilt_range_moves_bracket",
                max(
                    abs(upper_tilt_pos[1] - lower_tilt_pos[1]),
                    abs(upper_tilt_pos[2] - lower_tilt_pos[2]),
                )
                > 0.05,
                f"Tilt travel moved bracket from {lower_tilt_pos} to {upper_tilt_pos}.",
            )

    clamp_limits = clamp_joint.motion_limits
    clamp_rest_pos = ctx.part_world_position(clamp_jaw)
    if clamp_rest_pos is None:
        ctx.fail("clamp_rest_position_available", "Clamp jaw rest position could not be resolved.")
    elif clamp_limits is not None and clamp_limits.upper is not None:
        with ctx.pose({clamp_joint: clamp_limits.upper}):
            ctx.expect_overlap(clamp_jaw, bracket, axes="xy", min_overlap=0.010, name="clamp_open_pose_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="clamp_open_pose_clear")
            ctx.fail_if_isolated_parts(name="clamp_open_pose_supported")
            clamp_open_pos = ctx.part_world_position(clamp_jaw)
        if clamp_open_pos is None:
            ctx.fail("clamp_open_position_available", "Clamp jaw open position could not be resolved.")
        else:
            ctx.check(
                "clamp_jaw_opens_outboard",
                clamp_open_pos[0] > clamp_rest_pos[0] + 0.03,
                f"Clamp jaw moved from {clamp_rest_pos} to {clamp_open_pos}.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
