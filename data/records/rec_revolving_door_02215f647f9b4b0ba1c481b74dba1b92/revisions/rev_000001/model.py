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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestrian_revolving_door")

    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.25, 0.27, 0.29, 1.0))
    glass = model.material("door_glass", rgba=(0.58, 0.72, 0.80, 0.35))
    smoked_glass = model.material("sidelight_glass", rgba=(0.52, 0.66, 0.74, 0.28))
    gasket = model.material("gasket", rgba=(0.08, 0.08, 0.08, 1.0))

    drum_radius = 1.72
    side_glass_radius = 1.58
    floor_thickness = 0.06
    canopy_thickness = 0.12
    canopy_center_z = 2.48
    canopy_underside_z = canopy_center_z - canopy_thickness / 2.0
    sidewall_height = canopy_underside_z
    sidewall_center_z = sidewall_height / 2.0

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=drum_radius, length=floor_thickness),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=dark_metal,
        name="floor_ring",
    )
    drum.visual(
        Cylinder(radius=drum_radius, length=canopy_thickness),
        origin=Origin(xyz=(0.0, 0.0, canopy_center_z)),
        material=aluminum,
        name="canopy_disk",
    )

    sidelight_radius = 1.56
    sidelight_panel_angles = (-0.36, -0.18, 0.0, 0.18, 0.36)
    sidelight_names = (
        "south_2",
        "south_1",
        "",
        "north_1",
        "north_2",
    )
    for angle, suffix in zip(sidelight_panel_angles, sidelight_names):
        right_name = "right_sidelight" if not suffix else f"right_sidelight_{suffix}"
        left_name = "left_sidelight" if not suffix else f"left_sidelight_{suffix}"
        drum.visual(
            Box((0.03, 0.40, sidewall_height)),
            origin=Origin(
                xyz=(
                    sidelight_radius * math.cos(angle),
                    sidelight_radius * math.sin(angle),
                    sidewall_center_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=smoked_glass,
            name=right_name,
        )
        left_angle = math.pi + angle
        drum.visual(
            Box((0.03, 0.40, sidewall_height)),
            origin=Origin(
                xyz=(
                    sidelight_radius * math.cos(left_angle),
                    sidelight_radius * math.sin(left_angle),
                    sidewall_center_z,
                ),
                rpy=(0.0, 0.0, left_angle),
            ),
            material=smoked_glass,
            name=left_name,
        )
    jamb_angles = (
        math.radians(26.0),
        math.radians(154.0),
        math.radians(206.0),
        math.radians(334.0),
    )
    jamb_radius = side_glass_radius
    jamb_height = canopy_underside_z
    jamb_center_z = jamb_height / 2.0
    for idx, angle in enumerate(jamb_angles, start=1):
        drum.visual(
            Box((0.09, 0.08, jamb_height)),
            origin=Origin(
                xyz=(jamb_radius * math.cos(angle), jamb_radius * math.sin(angle), jamb_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=aluminum,
            name=f"portal_jamb_{idx}",
        )

    drum.visual(
        Cylinder(radius=0.42, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 2.62)),
        material=dark_metal,
        name="motor_housing",
    )
    drum.visual(
        Cylinder(radius=0.24, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 2.735)),
        material=aluminum,
        name="motor_cap",
    )
    drum.inertial = Inertial.from_geometry(
        Box((2.0 * drum_radius, 2.0 * drum_radius, 2.77)),
        mass=650.0,
        origin=Origin(xyz=(0.0, 0.0, 1.385)),
    )

    wing_assembly = model.part("wing_assembly")
    post_radius = 0.095
    wing_assembly.visual(
        Cylinder(radius=post_radius, length=2.27),
        origin=Origin(xyz=(0.0, 0.0, 1.225)),
        material=dark_metal,
        name="central_post",
    )
    wing_assembly.visual(
        Cylinder(radius=0.17, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_metal,
        name="bottom_hub",
    )
    wing_assembly.visual(
        Cylinder(radius=0.17, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 2.36)),
        material=dark_metal,
        name="top_hub",
    )
    wing_assembly.visual(
        Cylinder(radius=0.13, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=gasket,
        name="bottom_bearing_ring",
    )

    arm_length = 1.40
    arm_center_r = 0.785
    bottom_arm_z = 0.135
    top_arm_z = 2.295
    root_stile_length = 0.065
    root_stile_center_r = 0.117
    outer_stile_length = 0.045
    outer_stile_center_r = 1.4775
    wing_frame_height = 2.09
    wing_frame_center_z = 1.215
    glass_length = 1.306
    glass_center_r = 0.802
    glass_height = 2.01
    def radial_pose(radius: float, angle: float, z: float) -> Origin:
        return Origin(
            xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
            rpy=(0.0, 0.0, angle),
        )

    angle_ne = math.pi / 4.0
    wing_assembly.visual(
        Box((arm_length, 0.05, 0.07)),
        origin=radial_pose(arm_center_r, angle_ne, bottom_arm_z),
        material=aluminum,
        name="bottom_spider_ne",
    )
    wing_assembly.visual(
        Box((arm_length, 0.05, 0.07)),
        origin=radial_pose(arm_center_r, angle_ne, top_arm_z),
        material=aluminum,
        name="top_spider_ne",
    )
    wing_assembly.visual(
        Box((root_stile_length, 0.07, wing_frame_height)),
        origin=radial_pose(root_stile_center_r, angle_ne, wing_frame_center_z),
        material=aluminum,
        name="wing_ne_root_stile",
    )
    wing_assembly.visual(
        Box((glass_length, 0.028, glass_height)),
        origin=radial_pose(glass_center_r, angle_ne, wing_frame_center_z),
        material=glass,
        name="wing_ne_panel",
    )
    wing_assembly.visual(
        Box((outer_stile_length, 0.07, wing_frame_height)),
        origin=radial_pose(outer_stile_center_r, angle_ne, wing_frame_center_z),
        material=aluminum,
        name="wing_ne_outer_stile",
    )

    angle_nw = 3.0 * math.pi / 4.0
    wing_assembly.visual(
        Box((arm_length, 0.05, 0.07)),
        origin=radial_pose(arm_center_r, angle_nw, bottom_arm_z),
        material=aluminum,
        name="bottom_spider_nw",
    )
    wing_assembly.visual(
        Box((arm_length, 0.05, 0.07)),
        origin=radial_pose(arm_center_r, angle_nw, top_arm_z),
        material=aluminum,
        name="top_spider_nw",
    )
    wing_assembly.visual(
        Box((root_stile_length, 0.07, wing_frame_height)),
        origin=radial_pose(root_stile_center_r, angle_nw, wing_frame_center_z),
        material=aluminum,
        name="wing_nw_root_stile",
    )
    wing_assembly.visual(
        Box((glass_length, 0.028, glass_height)),
        origin=radial_pose(glass_center_r, angle_nw, wing_frame_center_z),
        material=glass,
        name="wing_nw_panel",
    )
    wing_assembly.visual(
        Box((outer_stile_length, 0.07, wing_frame_height)),
        origin=radial_pose(outer_stile_center_r, angle_nw, wing_frame_center_z),
        material=aluminum,
        name="wing_nw_outer_stile",
    )

    angle_sw = 5.0 * math.pi / 4.0
    wing_assembly.visual(
        Box((arm_length, 0.05, 0.07)),
        origin=radial_pose(arm_center_r, angle_sw, bottom_arm_z),
        material=aluminum,
        name="bottom_spider_sw",
    )
    wing_assembly.visual(
        Box((arm_length, 0.05, 0.07)),
        origin=radial_pose(arm_center_r, angle_sw, top_arm_z),
        material=aluminum,
        name="top_spider_sw",
    )
    wing_assembly.visual(
        Box((root_stile_length, 0.07, wing_frame_height)),
        origin=radial_pose(root_stile_center_r, angle_sw, wing_frame_center_z),
        material=aluminum,
        name="wing_sw_root_stile",
    )
    wing_assembly.visual(
        Box((glass_length, 0.028, glass_height)),
        origin=radial_pose(glass_center_r, angle_sw, wing_frame_center_z),
        material=glass,
        name="wing_sw_panel",
    )
    wing_assembly.visual(
        Box((outer_stile_length, 0.07, wing_frame_height)),
        origin=radial_pose(outer_stile_center_r, angle_sw, wing_frame_center_z),
        material=aluminum,
        name="wing_sw_outer_stile",
    )

    angle_se = 7.0 * math.pi / 4.0
    wing_assembly.visual(
        Box((arm_length, 0.05, 0.07)),
        origin=radial_pose(arm_center_r, angle_se, bottom_arm_z),
        material=aluminum,
        name="bottom_spider_se",
    )
    wing_assembly.visual(
        Box((arm_length, 0.05, 0.07)),
        origin=radial_pose(arm_center_r, angle_se, top_arm_z),
        material=aluminum,
        name="top_spider_se",
    )
    wing_assembly.visual(
        Box((root_stile_length, 0.07, wing_frame_height)),
        origin=radial_pose(root_stile_center_r, angle_se, wing_frame_center_z),
        material=aluminum,
        name="wing_se_root_stile",
    )
    wing_assembly.visual(
        Box((glass_length, 0.028, glass_height)),
        origin=radial_pose(glass_center_r, angle_se, wing_frame_center_z),
        material=glass,
        name="wing_se_panel",
    )
    wing_assembly.visual(
        Box((outer_stile_length, 0.07, wing_frame_height)),
        origin=radial_pose(outer_stile_center_r, angle_se, wing_frame_center_z),
        material=aluminum,
        name="wing_se_outer_stile",
    )

    wing_assembly.inertial = Inertial.from_geometry(
        Box((3.0, 3.0, 2.39)),
        mass=190.0,
        origin=Origin(xyz=(0.0, 0.0, 1.195)),
    )

    model.articulation(
        "drum_to_wing_assembly",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=wing_assembly,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.50),
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
    drum = object_model.get_part("drum")
    wing_assembly = object_model.get_part("wing_assembly")
    rotor = object_model.get_articulation("drum_to_wing_assembly")

    ctx.check(
        "wing assembly uses a vertical continuous rotor",
        rotor.articulation_type == ArticulationType.CONTINUOUS
        and rotor.axis == (0.0, 0.0, 1.0)
        and rotor.motion_limits is not None
        and rotor.motion_limits.lower is None
        and rotor.motion_limits.upper is None,
        details=(
            f"type={rotor.articulation_type}, axis={rotor.axis}, "
            f"limits={rotor.motion_limits}"
        ),
    )
    ctx.expect_contact(
        wing_assembly,
        drum,
        elem_a="bottom_hub",
        elem_b="floor_ring",
        contact_tol=0.002,
        name="rotor is seated on the floor bearing",
    )
    ctx.expect_gap(
        drum,
        wing_assembly,
        axis="z",
        positive_elem="canopy_disk",
        negative_elem="top_hub",
        min_gap=0.02,
        max_gap=0.06,
        name="rotor clears the canopy underside",
    )
    ctx.expect_overlap(
        wing_assembly,
        drum,
        axes="xy",
        elem_a="central_post",
        elem_b="motor_housing",
        min_overlap=0.18,
        name="motor housing is centered over the rotating post",
    )
    ctx.expect_within(
        wing_assembly,
        drum,
        axes="xy",
        inner_elem="wing_ne_outer_stile",
        outer_elem="floor_ring",
        margin=0.0,
        name="wing tip stays within the drum footprint at rest",
    )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            0.5 * (lower[0] + upper[0]),
            0.5 * (lower[1] + upper[1]),
            0.5 * (lower[2] + upper[2]),
        )

    rest_center = aabb_center(ctx.part_element_world_aabb(wing_assembly, elem="wing_ne_panel"))
    with ctx.pose({rotor: math.pi / 2.0}):
        ctx.expect_within(
            wing_assembly,
            drum,
            axes="xy",
            inner_elem="wing_ne_outer_stile",
            outer_elem="floor_ring",
            margin=0.0,
            name="wing tip stays within the drum footprint through a quarter turn",
        )
        quarter_turn_center = aabb_center(ctx.part_element_world_aabb(wing_assembly, elem="wing_ne_panel"))

    ctx.check(
        "positive rotation carries the northeast wing counterclockwise",
        rest_center is not None
        and quarter_turn_center is not None
        and rest_center[0] > 0.0
        and rest_center[1] > 0.0
        and quarter_turn_center[0] < -0.2
        and quarter_turn_center[1] > 0.2,
        details=f"rest_center={rest_center}, quarter_turn_center={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
