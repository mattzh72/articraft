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


def _arm_link_visuals(
    part,
    *,
    prefix: str,
    length: float,
    rail_spacing: float,
    rail_radius: float,
    block_height: float,
    block_width: float,
    cover_length: float,
    body_color,
    joint_color,
) -> None:
    rail_y = rail_spacing * 0.5
    rail_length = max(length - 0.12, 0.12)
    rail_center_x = 0.06 + rail_length * 0.5
    hub_radius = min(block_width * 0.36, 0.038)
    hub_length = block_height * 0.95

    part.visual(
        Box((0.095, block_width, block_height)),
        origin=Origin(xyz=(0.0475, 0.0, 0.0)),
        material=joint_color,
        name=f"{prefix}_base_block",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(xyz=(hub_radius, 0.0, 0.0)),
        material=joint_color,
        name=f"{prefix}_base_hub",
    )
    part.visual(
        Box((0.060, block_width * 0.94, block_height * 0.90)),
        origin=Origin(xyz=(length - 0.030, 0.0, 0.0)),
        material=joint_color,
        name=f"{prefix}_tip_block",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.92, length=hub_length * 0.92),
        origin=Origin(xyz=(length - hub_radius * 0.92, 0.0, 0.0)),
        material=joint_color,
        name=f"{prefix}_tip_hub",
    )
    part.visual(
        Cylinder(radius=rail_radius, length=rail_length),
        origin=Origin(
            xyz=(rail_center_x, rail_y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_color,
        name=f"{prefix}_rail_upper",
    )
    part.visual(
        Cylinder(radius=rail_radius, length=rail_length),
        origin=Origin(
            xyz=(rail_center_x, -rail_y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_color,
        name=f"{prefix}_rail_lower",
    )
    part.visual(
        Box((cover_length, rail_spacing * 0.70, rail_radius * 2.6)),
        origin=Origin(xyz=(length * 0.52, 0.0, 0.0)),
        material=body_color,
        name=f"{prefix}_cable_cover",
    )
    part.visual(
        Box((0.060, rail_spacing * 0.82, rail_radius * 2.0)),
        origin=Origin(xyz=(length * 0.32, 0.0, 0.0)),
        material=body_color,
        name=f"{prefix}_mid_brace",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_exam_light")

    enamel = model.material("enamel", rgba=(0.92, 0.94, 0.95, 1.0))
    light_gray = model.material("light_gray", rgba=(0.78, 0.81, 0.84, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.26, 0.29, 1.0))
    matte_black = model.material("matte_black", rgba=(0.09, 0.10, 0.11, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.76, 0.88, 0.95, 0.55))

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Cylinder(radius=0.120, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=enamel,
        name="ceiling_plate",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.032, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, -0.098)),
        material=light_gray,
        name="drop_stem",
    )
    ceiling_mount.visual(
        Box((0.090, 0.090, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.168)),
        material=dark_metal,
        name="bearing_block",
    )
    ceiling_mount.visual(
        Box((0.044, 0.110, 0.040)),
        origin=Origin(xyz=(-0.022, 0.0, -0.196)),
        material=dark_metal,
        name="shoulder_bearing",
    )
    ceiling_mount.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.23)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
    )

    arm_link1 = model.part("arm_link1")
    _arm_link_visuals(
        arm_link1,
        prefix="primary",
        length=0.555,
        rail_spacing=0.106,
        rail_radius=0.014,
        block_height=0.040,
        block_width=0.116,
        cover_length=0.205,
        body_color=light_gray,
        joint_color=dark_metal,
    )
    arm_link1.inertial = Inertial.from_geometry(
        Box((0.555, 0.116, 0.060)),
        mass=3.8,
        origin=Origin(xyz=(0.2775, 0.0, 0.0)),
    )

    arm_link2 = model.part("arm_link2")
    _arm_link_visuals(
        arm_link2,
        prefix="secondary",
        length=0.490,
        rail_spacing=0.090,
        rail_radius=0.012,
        block_height=0.034,
        block_width=0.100,
        cover_length=0.180,
        body_color=light_gray,
        joint_color=dark_metal,
    )
    arm_link2.inertial = Inertial.from_geometry(
        Box((0.490, 0.100, 0.054)),
        mass=3.0,
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        Box((0.060, 0.085, 0.046)),
        origin=Origin(xyz=(0.030, 0.0, -0.023)),
        material=dark_metal,
        name="swivel_housing",
    )
    head_swivel.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.030, 0.0, -0.078)),
        material=light_gray,
        name="drop_post",
    )
    head_swivel.visual(
        Box((0.060, 0.270, 0.022)),
        origin=Origin(xyz=(0.030, 0.0, -0.132)),
        material=dark_metal,
        name="yoke_crossbar",
    )
    head_swivel.visual(
        Box((0.030, 0.018, 0.095)),
        origin=Origin(xyz=(0.030, 0.126, -0.171)),
        material=dark_metal,
        name="left_yoke_arm",
    )
    head_swivel.visual(
        Box((0.030, 0.018, 0.095)),
        origin=Origin(xyz=(0.030, -0.126, -0.171)),
        material=dark_metal,
        name="right_yoke_arm",
    )
    head_swivel.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(
            xyz=(0.030, 0.118, -0.218),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="left_trunnion_boss",
    )
    head_swivel.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(
            xyz=(0.030, -0.118, -0.218),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="right_trunnion_boss",
    )
    head_swivel.inertial = Inertial.from_geometry(
        Box((0.080, 0.280, 0.240)),
        mass=1.4,
        origin=Origin(xyz=(0.030, 0.0, -0.120)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.108, length=0.046),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=enamel,
        name="head_housing",
    )
    lamp_head.visual(
        Cylinder(radius=0.120, length=0.014),
        origin=Origin(xyz=(0.030, 0.0, -0.030)),
        material=light_gray,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.094, length=0.004),
        origin=Origin(xyz=(0.030, 0.0, -0.039)),
        material=lens_glass,
        name="lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.068, length=0.040),
        origin=Origin(xyz=(0.030, 0.0, 0.033)),
        material=light_gray,
        name="rear_cap",
    )
    lamp_head.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, -0.066)),
        material=matte_black,
        name="head_handle_stem",
    )
    lamp_head.visual(
        Cylinder(radius=0.013, length=0.090),
        origin=Origin(
            xyz=(0.050, 0.0, -0.105),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=matte_black,
        name="head_handle_grip",
    )
    lamp_head.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(
            xyz=(0.030, 0.098, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="left_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(
            xyz=(0.030, -0.098, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="right_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(
            xyz=(0.090, 0.055, -0.002),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="focus_knob",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.260, 0.260, 0.170)),
        mass=2.6,
        origin=Origin(xyz=(0.030, 0.0, -0.015)),
    )

    model.articulation(
        "ceiling_to_arm_link1",
        ArticulationType.REVOLUTE,
        parent=ceiling_mount,
        child=arm_link1,
        origin=Origin(xyz=(0.0, 0.0, -0.216)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-2.7,
            upper=2.7,
        ),
    )
    model.articulation(
        "arm_link1_to_arm_link2",
        ArticulationType.REVOLUTE,
        parent=arm_link1,
        child=arm_link2,
        origin=Origin(xyz=(0.555, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-2.5,
            upper=2.5,
        ),
    )
    model.articulation(
        "arm_link2_to_head_swivel",
        ArticulationType.CONTINUOUS,
        parent=arm_link2,
        child=head_swivel,
        origin=Origin(xyz=(0.490, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.5,
        ),
    )
    model.articulation(
        "head_swivel_to_lamp_head",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=lamp_head,
        origin=Origin(xyz=(0.030, 0.0, -0.218)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-1.0,
            upper=0.75,
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

    ceiling_mount = object_model.get_part("ceiling_mount")
    arm_link1 = object_model.get_part("arm_link1")
    arm_link2 = object_model.get_part("arm_link2")
    head_swivel = object_model.get_part("head_swivel")
    lamp_head = object_model.get_part("lamp_head")

    shoulder = object_model.get_articulation("ceiling_to_arm_link1")
    elbow = object_model.get_articulation("arm_link1_to_arm_link2")
    head_rotation = object_model.get_articulation("arm_link2_to_head_swivel")
    head_tilt = object_model.get_articulation("head_swivel_to_lamp_head")

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))

    ctx.expect_origin_gap(
        ceiling_mount,
        lamp_head,
        axis="z",
        min_gap=0.25,
        name="lamp head hangs well below the ceiling mount",
    )
    ctx.expect_origin_gap(
        arm_link2,
        lamp_head,
        axis="z",
        min_gap=0.12,
        name="lamp head sits below the distal arm link",
    )

    rest_handle = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="head_handle_grip"))

    with ctx.pose({shoulder: 0.9}):
        swung_handle = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="head_handle_grip"))
    ctx.check(
        "shoulder joint swings the arm around the ceiling mount",
        rest_handle is not None
        and swung_handle is not None
        and abs(swung_handle[1] - rest_handle[1]) > 0.45,
        details=f"rest_handle={rest_handle}, swung_handle={swung_handle}",
    )

    with ctx.pose({elbow: 1.1}):
        folded_handle = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="head_handle_grip"))
    ctx.check(
        "elbow joint folds the distal link sideways",
        rest_handle is not None
        and folded_handle is not None
        and abs(folded_handle[1] - rest_handle[1]) > 0.18,
        details=f"rest_handle={rest_handle}, folded_handle={folded_handle}",
    )

    with ctx.pose({head_rotation: math.pi / 2.0}):
        rotated_handle = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="head_handle_grip"))
    ctx.check(
        "continuous head swivel rotates the lamp around the arm-tip axis",
        rest_handle is not None
        and rotated_handle is not None
        and abs(rotated_handle[1] - rest_handle[1]) > 0.03,
        details=f"rest_handle={rest_handle}, rotated_handle={rotated_handle}",
    )

    with ctx.pose({head_tilt: 0.6}):
        tilted_handle = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="head_handle_grip"))
    ctx.check(
        "head tilt pitches the lamp head forward",
        rest_handle is not None
        and tilted_handle is not None
        and tilted_handle[0] > rest_handle[0] + 0.04,
        details=f"rest_handle={rest_handle}, tilted_handle={tilted_handle}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
