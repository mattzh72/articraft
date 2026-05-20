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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


OPENING_WIDTH = 1.18
OPENING_HEIGHT = 1.30
FRAME_DEPTH = 0.08
JAMB_WIDTH = 0.06
HEAD_HEIGHT = 0.10

LEAF_WIDTH = (OPENING_WIDTH - 0.012) / 2.0
LEAF_HEIGHT = 1.28
LEAF_THICKNESS = 0.035
STILE_WIDTH = 0.055
RAIL_HEIGHT = 0.08
LEAF_APERTURE_WIDTH = LEAF_WIDTH - 2.0 * STILE_WIDTH

LOUVER_COUNT = 14
LOUVER_PITCH = 0.08
LOUVER_HEIGHT = 0.062
LOUVER_DEPTH = 0.012
LOUVER_PIN_RADIUS = 0.004
LOUVER_PIN_LENGTH = 0.012
LOUVER_BLADE_LENGTH = LEAF_APERTURE_WIDTH - 2.0 * LOUVER_PIN_LENGTH
LOUVER_CENTER_X = LEAF_WIDTH / 2.0
TOP_LOUVER_Z = 0.52
LOUVER_ZS = [TOP_LOUVER_Z - index * LOUVER_PITCH for index in range(LOUVER_COUNT)]

CLAMP_WIDTH = 0.006
CLAMP_DEPTH = 0.012
CLAMP_HEIGHT = 0.006
CLAMP_Z_OFFSET = 0.007

ROD_X = LEAF_WIDTH - STILE_WIDTH - 0.041
ROD_Y = LEAF_THICKNESS / 2.0 + 0.011
ROD_BAR_WIDTH = 0.014
ROD_BAR_DEPTH = 0.006
ROD_BAR_HEIGHT = 1.18
ROD_TAB_WIDTH = 0.022
ROD_TAB_DEPTH = 0.004
ROD_TAB_HEIGHT = 0.006
ROD_TAB_OFFSET_X = (ROD_BAR_WIDTH + ROD_TAB_WIDTH) / 2.0 - 0.003

GUIDE_WIDTH = 0.026
GUIDE_DEPTH = 0.008
GUIDE_HEIGHT = 0.06
GUIDE_Y = LEAF_THICKNESS / 2.0 + GUIDE_DEPTH / 2.0
GUIDE_Z = 0.59

ARM_WIDTH = 0.018
ARM_DEPTH = 0.021
ARM_HEIGHT = 0.016
ARM_X = LOUVER_BLADE_LENGTH / 2.0 - ARM_WIDTH / 2.0 - 0.004
ARM_Y = 0.016

LEFT_SIGN = 1.0
RIGHT_SIGN = -1.0


def louver_part_names(prefix: str) -> list[str]:
    return [f"{prefix}_louver_{index:02d}" for index in range(LOUVER_COUNT)]


def louver_joint_names(prefix: str) -> list[str]:
    return [f"{prefix}_louver_{index:02d}_pivot" for index in range(LOUVER_COUNT)]


def aperture_inner_face_x(is_inner_side: bool) -> float:
    if is_inner_side:
        return LEAF_WIDTH - STILE_WIDTH
    return STILE_WIDTH


def make_louver_blade_mesh():
    half_depth = LOUVER_DEPTH / 2.0
    half_height = LOUVER_HEIGHT / 2.0
    shoulder_y = half_depth * 0.78
    shoulder_z = half_height * 0.60

    def section_at(x_pos: float) -> list[tuple[float, float, float]]:
        return [
            (x_pos, -half_depth, 0.0),
            (x_pos, -shoulder_y, shoulder_z),
            (x_pos, 0.0, half_height),
            (x_pos, shoulder_y, shoulder_z),
            (x_pos, half_depth, 0.0),
            (x_pos, shoulder_y, -shoulder_z),
            (x_pos, 0.0, -half_height),
            (x_pos, -shoulder_y, -shoulder_z),
        ]

    blade_geom = section_loft(
        [
            section_at(-LOUVER_BLADE_LENGTH / 2.0),
            section_at(LOUVER_BLADE_LENGTH / 2.0),
        ]
    )
    return mesh_from_geometry(blade_geom, "louver_blade")


def _add_leaf_subassembly(
    model: ArticulatedObject,
    *,
    side_name: str,
    sign: float,
    hinge_x: float,
    leaf_lower: float,
    leaf_upper: float,
    leaf_material: str,
    louver_material: str,
    rod_material: str,
    hardware_material: str,
    louver_blade_mesh,
) -> None:
    leaf = model.part(f"{side_name}_leaf")

    leaf.visual(
        Box((STILE_WIDTH, LEAF_THICKNESS, LEAF_HEIGHT)),
        origin=Origin(xyz=(sign * (STILE_WIDTH / 2.0), 0.0, 0.0)),
        material=leaf_material,
        name="outer_stile",
    )
    leaf.visual(
        Box((STILE_WIDTH, LEAF_THICKNESS, LEAF_HEIGHT)),
        origin=Origin(xyz=(sign * (LEAF_WIDTH - STILE_WIDTH / 2.0), 0.0, 0.0)),
        material=leaf_material,
        name="inner_stile",
    )
    leaf.visual(
        Box((LEAF_WIDTH, LEAF_THICKNESS, RAIL_HEIGHT)),
        origin=Origin(xyz=(sign * (LEAF_WIDTH / 2.0), 0.0, LEAF_HEIGHT / 2.0 - RAIL_HEIGHT / 2.0)),
        material=leaf_material,
        name="top_rail",
    )
    leaf.visual(
        Box((LEAF_WIDTH, LEAF_THICKNESS, RAIL_HEIGHT)),
        origin=Origin(xyz=(sign * (LEAF_WIDTH / 2.0), 0.0, -LEAF_HEIGHT / 2.0 + RAIL_HEIGHT / 2.0)),
        material=leaf_material,
        name="bottom_rail",
    )

    leaf.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(sign * ROD_X, GUIDE_Y, GUIDE_Z)),
        material=hardware_material,
        name="upper_rod_guide",
    )
    leaf.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(sign * ROD_X, GUIDE_Y, -GUIDE_Z)),
        material=hardware_material,
        name="lower_rod_guide",
    )

    outer_clamp_x = STILE_WIDTH - CLAMP_WIDTH / 2.0
    inner_clamp_x = LEAF_WIDTH - STILE_WIDTH + CLAMP_WIDTH / 2.0

    for index, z_pos in enumerate(LOUVER_ZS):
        for clamp_sign in (-1.0, 1.0):
            leaf.visual(
                Box((CLAMP_WIDTH, CLAMP_DEPTH, CLAMP_HEIGHT)),
                origin=Origin(
                    xyz=(
                        sign * outer_clamp_x,
                        0.0,
                        z_pos + clamp_sign * CLAMP_Z_OFFSET,
                    )
                ),
                material=hardware_material,
                name=f"outer_clip_{index:02d}_{'upper' if clamp_sign > 0.0 else 'lower'}",
            )
            leaf.visual(
                Box((CLAMP_WIDTH, CLAMP_DEPTH, CLAMP_HEIGHT)),
                origin=Origin(
                    xyz=(
                        sign * inner_clamp_x,
                        0.0,
                        z_pos + clamp_sign * CLAMP_Z_OFFSET,
                    )
                ),
                material=hardware_material,
                name=f"inner_clip_{index:02d}_{'upper' if clamp_sign > 0.0 else 'lower'}",
            )

    model.articulation(
        f"frame_to_{side_name}_leaf",
        ArticulationType.REVOLUTE,
        parent="frame",
        child=leaf,
        origin=Origin(xyz=(hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=leaf_lower,
            upper=leaf_upper,
        ),
    )

    rod = model.part(f"{side_name}_tilt_rod")
    rod.visual(
        Box((ROD_BAR_WIDTH, ROD_BAR_DEPTH, ROD_BAR_HEIGHT)),
        material=rod_material,
        name="rod_bar",
    )
    for index, z_pos in enumerate(LOUVER_ZS):
        rod.visual(
            Box((ROD_TAB_WIDTH, ROD_TAB_DEPTH, ROD_TAB_HEIGHT)),
            origin=Origin(xyz=(sign * ROD_TAB_OFFSET_X, 0.0, z_pos)),
            material=hardware_material,
            name=f"tab_{index:02d}",
        )

    model.articulation(
        f"{side_name}_tilt_rod_slide",
        ArticulationType.PRISMATIC,
        parent=leaf,
        child=rod,
        origin=Origin(xyz=(sign * ROD_X, ROD_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=-0.04,
            upper=0.04,
        ),
    )

    for index, z_pos in enumerate(LOUVER_ZS):
        louver = model.part(f"{side_name}_louver_{index:02d}")
        louver.visual(
            louver_blade_mesh,
            material=louver_material,
            name="blade",
        )
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(
                xyz=(-(LOUVER_BLADE_LENGTH / 2.0 + LOUVER_PIN_LENGTH / 2.0), 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hardware_material,
            name="outer_pivot_pin",
        )
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(
                xyz=((LOUVER_BLADE_LENGTH / 2.0 + LOUVER_PIN_LENGTH / 2.0), 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hardware_material,
            name="inner_pivot_pin",
        )
        louver.visual(
            Box((ARM_WIDTH, ARM_DEPTH, ARM_HEIGHT)),
            origin=Origin(xyz=(sign * ARM_X, ARM_Y, 0.0)),
            material=hardware_material,
            name="tilt_arm",
        )

        model.articulation(
            f"{side_name}_louver_{index:02d}_pivot",
            ArticulationType.REVOLUTE,
            parent=leaf,
            child=louver,
            origin=Origin(xyz=(sign * LOUVER_CENTER_X, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=2.0,
                lower=-0.55,
                upper=0.55,
            ),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_panel_shutter")

    frame_paint = model.material("frame_paint", rgba=(0.94, 0.93, 0.90, 1.0))
    louver_paint = model.material("louver_paint", rgba=(0.97, 0.97, 0.95, 1.0))
    rod_finish = model.material("rod_finish", rgba=(0.77, 0.77, 0.78, 1.0))
    hardware_finish = model.material("hardware_finish", rgba=(0.55, 0.56, 0.58, 1.0))
    louver_blade_mesh = make_louver_blade_mesh()

    frame = model.part("frame")
    frame.visual(
        Box((JAMB_WIDTH, FRAME_DEPTH, OPENING_HEIGHT + 2.0 * HEAD_HEIGHT)),
        origin=Origin(xyz=(-(OPENING_WIDTH / 2.0 + JAMB_WIDTH / 2.0), 0.0, 0.0)),
        material=frame_paint,
        name="left_jamb",
    )
    frame.visual(
        Box((JAMB_WIDTH, FRAME_DEPTH, OPENING_HEIGHT + 2.0 * HEAD_HEIGHT)),
        origin=Origin(xyz=((OPENING_WIDTH / 2.0 + JAMB_WIDTH / 2.0), 0.0, 0.0)),
        material=frame_paint,
        name="right_jamb",
    )
    frame.visual(
        Box((OPENING_WIDTH + 2.0 * JAMB_WIDTH, FRAME_DEPTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT / 2.0 + HEAD_HEIGHT / 2.0)),
        material=frame_paint,
        name="head",
    )
    frame.visual(
        Box((OPENING_WIDTH + 2.0 * JAMB_WIDTH, FRAME_DEPTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -(OPENING_HEIGHT / 2.0 + HEAD_HEIGHT / 2.0))),
        material=frame_paint,
        name="sill",
    )

    _add_leaf_subassembly(
        model,
        side_name="left",
        sign=LEFT_SIGN,
        hinge_x=-(OPENING_WIDTH / 2.0),
        leaf_lower=-1.45,
        leaf_upper=0.05,
        leaf_material=frame_paint,
        louver_material=louver_paint,
        rod_material=rod_finish,
        hardware_material=hardware_finish,
        louver_blade_mesh=louver_blade_mesh,
    )
    _add_leaf_subassembly(
        model,
        side_name="right",
        sign=RIGHT_SIGN,
        hinge_x=(OPENING_WIDTH / 2.0),
        leaf_lower=-0.05,
        leaf_upper=1.45,
        leaf_material=frame_paint,
        louver_material=louver_paint,
        rod_material=rod_finish,
        hardware_material=hardware_finish,
        louver_blade_mesh=louver_blade_mesh,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    left_rod = object_model.get_part("left_tilt_rod")
    right_rod = object_model.get_part("right_tilt_rod")
    left_leaf_joint = object_model.get_articulation("frame_to_left_leaf")
    right_leaf_joint = object_model.get_articulation("frame_to_right_leaf")
    left_rod_joint = object_model.get_articulation("left_tilt_rod_slide")
    right_rod_joint = object_model.get_articulation("right_tilt_rod_slide")

    left_louver_names = louver_part_names("left")
    right_louver_names = louver_part_names("right")
    left_joint_names = louver_joint_names("left")
    right_joint_names = louver_joint_names("right")
    left_louvers = [object_model.get_part(name) for name in left_louver_names]
    right_louvers = [object_model.get_part(name) for name in right_louver_names]
    left_louver_joints = [object_model.get_articulation(name) for name in left_joint_names]
    right_louver_joints = [object_model.get_articulation(name) for name in right_joint_names]

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

    ctx.expect_contact(left_leaf, frame)
    ctx.expect_contact(right_leaf, frame)
    ctx.expect_contact(left_rod, left_leaf)
    ctx.expect_contact(right_rod, right_leaf)

    for louver in (left_louvers[0], left_louvers[LOUVER_COUNT // 2], left_louvers[-1]):
        ctx.expect_contact(louver, left_leaf)
    for louver in (right_louvers[0], right_louvers[LOUVER_COUNT // 2], right_louvers[-1]):
        ctx.expect_contact(louver, right_leaf)

    ctx.expect_gap(right_leaf, left_leaf, axis="x", min_gap=0.010, max_gap=0.014)

    ctx.expect_gap(
        left_leaf,
        left_louvers[0],
        axis="z",
        positive_elem="top_rail",
        negative_elem="blade",
        min_gap=0.006,
        max_gap=0.014,
        name="left_top_louver_clear_of_top_rail",
    )
    ctx.expect_gap(
        left_louvers[-1],
        left_leaf,
        axis="z",
        positive_elem="blade",
        negative_elem="bottom_rail",
        min_gap=0.006,
        max_gap=0.014,
        name="left_bottom_louver_clear_of_bottom_rail",
    )
    ctx.expect_gap(
        right_leaf,
        right_louvers[0],
        axis="z",
        positive_elem="top_rail",
        negative_elem="blade",
        min_gap=0.006,
        max_gap=0.014,
        name="right_top_louver_clear_of_top_rail",
    )
    ctx.expect_gap(
        right_louvers[-1],
        right_leaf,
        axis="z",
        positive_elem="blade",
        negative_elem="bottom_rail",
        min_gap=0.006,
        max_gap=0.014,
        name="right_bottom_louver_clear_of_bottom_rail",
    )

    for index in (0, LOUVER_COUNT // 2, LOUVER_COUNT - 1):
        ctx.expect_contact(
            left_rod,
            left_louvers[index],
            elem_a=f"tab_{index:02d}",
            elem_b="tilt_arm",
            name=f"left_rod_tab_contacts_louver_arm_{index:02d}",
        )
        ctx.expect_contact(
            right_rod,
            right_louvers[index],
            elem_a=f"tab_{index:02d}",
            elem_b="tilt_arm",
            name=f"right_rod_tab_contacts_louver_arm_{index:02d}",
        )

    ctx.check(
        "leaf_hinge_axes_vertical",
        left_leaf_joint.axis == (0.0, 0.0, 1.0) and right_leaf_joint.axis == (0.0, 0.0, 1.0),
        details=f"left={left_leaf_joint.axis}, right={right_leaf_joint.axis}",
    )
    ctx.check(
        "tilt_rods_slide_vertically",
        left_rod_joint.axis == (0.0, 0.0, 1.0) and right_rod_joint.axis == (0.0, 0.0, 1.0),
        details=f"left={left_rod_joint.axis}, right={right_rod_joint.axis}",
    )
    ctx.check(
        "all_louver_axes_horizontal",
        all(joint.axis == (1.0, 0.0, 0.0) for joint in left_louver_joints + right_louver_joints),
        details="One or more louver axes are not aligned with the slat length.",
    )

    left_rod_rest = ctx.part_world_position(left_rod)
    right_rod_rest = ctx.part_world_position(right_rod)
    if left_rod_rest is not None and right_rod_rest is not None:
        with ctx.pose({left_rod_joint: 0.03, right_rod_joint: -0.03}):
            left_rod_raised = ctx.part_world_position(left_rod)
            right_rod_lowered = ctx.part_world_position(right_rod)
            ctx.check(
                "tilt_rods_translate_in_z",
                left_rod_raised is not None
                and right_rod_lowered is not None
                and left_rod_raised[2] > left_rod_rest[2] + 0.02
                and right_rod_lowered[2] < right_rod_rest[2] - 0.02,
                details=(
                    f"left_rest={left_rod_rest}, left_raised={left_rod_raised}, "
                    f"right_rest={right_rod_rest}, right_lowered={right_rod_lowered}"
                ),
            )
            ctx.expect_contact(left_rod, left_leaf)
            ctx.expect_contact(right_rod, right_leaf)

    coordinated_pose = {left_leaf_joint: -1.2, right_leaf_joint: 1.2, left_rod_joint: 0.03, right_rod_joint: 0.03}
    for joint in left_louver_joints + right_louver_joints:
        coordinated_pose[joint] = 0.42

    with ctx.pose(coordinated_pose):
        left_leaf_aabb = ctx.part_world_aabb(left_leaf)
        right_leaf_aabb = ctx.part_world_aabb(right_leaf)
        leaves_open = False
        if left_leaf_aabb is not None and right_leaf_aabb is not None:
            left_center_y = (left_leaf_aabb[0][1] + left_leaf_aabb[1][1]) / 2.0
            right_center_y = (right_leaf_aabb[0][1] + right_leaf_aabb[1][1]) / 2.0
            leaves_open = left_center_y < -0.12 and right_center_y < -0.12
        ctx.check(
            "leaves_swing_on_outer_jamb_axes",
            leaves_open,
            details=f"left_aabb={left_leaf_aabb}, right_aabb={right_leaf_aabb}",
        )
        for louver in (left_louvers[0], left_louvers[LOUVER_COUNT // 2], left_louvers[-1]):
            ctx.expect_contact(louver, left_leaf)
        for louver in (right_louvers[0], right_louvers[LOUVER_COUNT // 2], right_louvers[-1]):
            ctx.expect_contact(louver, right_leaf)
        ctx.expect_contact(left_rod, left_leaf)
        ctx.expect_contact(right_rod, right_leaf)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
