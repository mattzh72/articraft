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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_THICKNESS = 0.0044
RIM_THICKNESS = 0.0032
LENS_WIDTH = 0.050
LENS_HEIGHT = 0.031
LENS_THICKNESS = 0.0018
BRIDGE_WIDTH = 0.016
BRIDGE_HEIGHT = 0.0062

LOOP_WIDTH = LENS_WIDTH + 2.0 * RIM_THICKNESS
LOOP_HEIGHT = LENS_HEIGHT + 2.0 * RIM_THICKNESS
LENS_CENTER_X = 0.5 * (LOOP_WIDTH + BRIDGE_WIDTH)
FRAME_OUTER_WIDTH = 2.0 * LOOP_WIDTH + BRIDGE_WIDTH

HINGE_RADIUS = 0.0017
FRAME_KNUCKLE_LENGTH = 0.0022
TEMPLE_KNUCKLE_LENGTH = 0.0044
HINGE_X = 0.5 * FRAME_OUTER_WIDTH + 0.0021
LEFT_HINGE_Y = 0.0072
RIGHT_HINGE_Y = 0.0102

TEMPLE_WIDTH = 0.0058
TEMPLE_HEIGHT = 0.0042
TEMPLE_LENGTH = 0.104
EAR_TIP_LENGTH = 0.048


def _add_rectangular_rim(part, *, prefix: str, center_x: float, frame_material, lens_material) -> None:
    vertical_offset = 0.5 * LENS_WIDTH + 0.5 * RIM_THICKNESS
    horizontal_offset = 0.5 * LENS_HEIGHT + 0.5 * RIM_THICKNESS
    side_sign = -1.0 if center_x < 0.0 else 1.0
    outer_x = center_x + side_sign * vertical_offset
    inner_x = center_x - side_sign * vertical_offset

    part.visual(
        Box((RIM_THICKNESS, FRAME_THICKNESS, LOOP_HEIGHT)),
        origin=Origin(xyz=(outer_x, 0.0, 0.0)),
        material=frame_material,
        name=f"{prefix}_outer_vertical",
    )
    part.visual(
        Box((RIM_THICKNESS, FRAME_THICKNESS, LOOP_HEIGHT)),
        origin=Origin(xyz=(inner_x, 0.0, 0.0)),
        material=frame_material,
        name=f"{prefix}_inner_vertical",
    )
    part.visual(
        Box((LENS_WIDTH, FRAME_THICKNESS, RIM_THICKNESS)),
        origin=Origin(xyz=(center_x, 0.0, horizontal_offset)),
        material=frame_material,
        name=f"{prefix}_top_bar",
    )
    part.visual(
        Box((LENS_WIDTH, FRAME_THICKNESS, RIM_THICKNESS)),
        origin=Origin(xyz=(center_x, 0.0, -horizontal_offset)),
        material=frame_material,
        name=f"{prefix}_bottom_bar",
    )
    part.visual(
        Box((LENS_WIDTH, LENS_THICKNESS, LENS_HEIGHT)),
        origin=Origin(xyz=(center_x, 0.0, 0.0)),
        material=lens_material,
        name=f"{prefix}_lens",
    )


def _add_frame_hinge(
    part,
    *,
    side_sign: float,
    hinge_y: float,
    frame_material,
    hinge_material,
    prefix: str,
) -> None:
    part.visual(
        Box((0.0036, hinge_y - 0.5 * FRAME_THICKNESS, 0.0098)),
        origin=Origin(xyz=(side_sign * (0.5 * FRAME_OUTER_WIDTH + 0.0018), 0.5 * (hinge_y + 0.5 * FRAME_THICKNESS), 0.0)),
        material=frame_material,
        name=f"{prefix}_hinge_lug",
    )
    part.visual(
        Cylinder(radius=HINGE_RADIUS, length=FRAME_KNUCKLE_LENGTH),
        origin=Origin(xyz=(side_sign * HINGE_X, hinge_y, 0.0033)),
        material=hinge_material,
        name=f"{prefix}_hinge_upper_barrel",
    )
    part.visual(
        Cylinder(radius=HINGE_RADIUS, length=FRAME_KNUCKLE_LENGTH),
        origin=Origin(xyz=(side_sign * HINGE_X, hinge_y, -0.0033)),
        material=hinge_material,
        name=f"{prefix}_hinge_lower_barrel",
    )


def _build_temple(part, *, side_sign: float, temple_material, hinge_material) -> None:
    def _temple_section(
        *,
        x_center: float,
        y: float,
        z_center: float,
        width: float,
        height: float,
    ) -> list[tuple[float, float, float]]:
        corner_radius = min(width, height) * 0.30
        return [
            (x + x_center, y, z + z_center)
            for x, z in rounded_rect_profile(width, height, corner_radius, corner_segments=5)
        ]

    arm_geom = section_loft(
        [
            _temple_section(x_center=side_sign * 0.0018, y=0.009, z_center=0.0, width=0.0052, height=0.0046),
            _temple_section(x_center=side_sign * 0.0025, y=0.032, z_center=0.0002, width=0.0060, height=0.0044),
            _temple_section(x_center=side_sign * 0.0027, y=0.080, z_center=-0.0008, width=0.0054, height=0.0038),
            _temple_section(x_center=side_sign * 0.0022, y=0.112, z_center=-0.0032, width=0.0047, height=0.0033),
            _temple_section(x_center=side_sign * 0.0016, y=0.143, z_center=-0.0108, width=0.0038, height=0.0028),
        ]
    )
    arm_mesh = mesh_from_geometry(
        arm_geom,
        ASSETS.mesh_path("left_temple_arm.obj" if side_sign < 0.0 else "right_temple_arm.obj"),
    )

    part.visual(
        Cylinder(radius=HINGE_RADIUS, length=TEMPLE_KNUCKLE_LENGTH),
        material=hinge_material,
        name="temple_barrel",
    )
    part.visual(
        Box((0.0048, 0.011, 0.0048)),
        origin=Origin(xyz=(side_sign * 0.0015, 0.006, 0.0)),
        material=temple_material,
        name="hinge_block",
    )
    part.visual(
        arm_mesh,
        material=temple_material,
        name="main_arm",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_eyeglasses", assets=ASSETS)

    acetate_black = model.material("acetate_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.66, 0.68, 0.70, 1.0))
    clear_lens = model.material("clear_lens", rgba=(0.88, 0.94, 0.98, 0.28))

    front_frame = model.part("front_frame")
    _add_rectangular_rim(
        front_frame,
        prefix="left",
        center_x=-LENS_CENTER_X,
        frame_material=acetate_black,
        lens_material=clear_lens,
    )
    _add_rectangular_rim(
        front_frame,
        prefix="right",
        center_x=LENS_CENTER_X,
        frame_material=acetate_black,
        lens_material=clear_lens,
    )
    front_frame.visual(
        Box((BRIDGE_WIDTH, FRAME_THICKNESS, BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=acetate_black,
        name="bridge",
    )
    _add_frame_hinge(
        front_frame,
        side_sign=-1.0,
        hinge_y=LEFT_HINGE_Y,
        frame_material=acetate_black,
        hinge_material=hinge_metal,
        prefix="left",
    )
    _add_frame_hinge(
        front_frame,
        side_sign=1.0,
        hinge_y=RIGHT_HINGE_Y,
        frame_material=acetate_black,
        hinge_material=hinge_metal,
        prefix="right",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH + 0.010, 0.022, LOOP_HEIGHT + 0.010)),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
    )

    left_temple = model.part("left_temple")
    _build_temple(left_temple, side_sign=-1.0, temple_material=acetate_black, hinge_material=hinge_metal)
    left_temple.inertial = Inertial.from_geometry(
        Box((0.012, 0.165, 0.018)),
        mass=0.010,
        origin=Origin(xyz=(-0.001, 0.083, -0.004)),
    )

    right_temple = model.part("right_temple")
    _build_temple(right_temple, side_sign=1.0, temple_material=acetate_black, hinge_material=hinge_metal)
    right_temple.inertial = Inertial.from_geometry(
        Box((0.012, 0.165, 0.018)),
        mass=0.010,
        origin=Origin(xyz=(0.001, 0.083, -0.004)),
    )

    model.articulation(
        "front_to_left_temple",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple,
        origin=Origin(xyz=(-HINGE_X, LEFT_HINGE_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "front_to_right_temple",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple,
        origin=Origin(xyz=(HINGE_X, RIGHT_HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    front_frame = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_hinge = object_model.get_articulation("front_to_left_temple")
    right_hinge = object_model.get_articulation("front_to_right_temple")

    left_outer_vertical = front_frame.get_visual("left_outer_vertical")
    right_outer_vertical = front_frame.get_visual("right_outer_vertical")
    left_lens = front_frame.get_visual("left_lens")
    right_lens = front_frame.get_visual("right_lens")
    left_upper_barrel = front_frame.get_visual("left_hinge_upper_barrel")
    left_lower_barrel = front_frame.get_visual("left_hinge_lower_barrel")
    right_upper_barrel = front_frame.get_visual("right_hinge_upper_barrel")
    right_lower_barrel = front_frame.get_visual("right_hinge_lower_barrel")
    left_temple_barrel = left_temple.get_visual("temple_barrel")
    right_temple_barrel = right_temple.get_visual("temple_barrel")
    left_hinge_block = left_temple.get_visual("hinge_block")
    right_hinge_block = right_temple.get_visual("hinge_block")
    left_hinge_lug = front_frame.get_visual("left_hinge_lug")
    right_hinge_lug = front_frame.get_visual("right_hinge_lug")
    left_main_arm = left_temple.get_visual("main_arm")
    right_main_arm = right_temple.get_visual("main_arm")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    ctx.expect_contact(
        left_temple,
        front_frame,
        elem_a=left_temple_barrel,
        elem_b=left_upper_barrel,
        name="left_hinge_upper_knuckle_contact",
    )
    ctx.expect_contact(
        left_temple,
        front_frame,
        elem_a=left_temple_barrel,
        elem_b=left_lower_barrel,
        name="left_hinge_lower_knuckle_contact",
    )
    ctx.expect_contact(
        right_temple,
        front_frame,
        elem_a=right_temple_barrel,
        elem_b=right_upper_barrel,
        name="right_hinge_upper_knuckle_contact",
    )
    ctx.expect_contact(
        right_temple,
        front_frame,
        elem_a=right_temple_barrel,
        elem_b=right_lower_barrel,
        name="right_hinge_lower_knuckle_contact",
    )

    ctx.expect_gap(
        left_temple,
        front_frame,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=left_hinge_block,
        negative_elem=left_hinge_lug,
        name="left_hinge_block_seated_behind_lug",
    )
    ctx.expect_gap(
        right_temple,
        front_frame,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=right_hinge_block,
        negative_elem=right_hinge_lug,
        name="right_hinge_block_seated_behind_lug",
    )
    ctx.expect_overlap(
        front_frame,
        left_temple,
        axes="z",
        min_overlap=0.004,
        elem_a=left_outer_vertical,
        elem_b=left_temple_barrel,
        name="left_hinge_vertical_alignment",
    )
    ctx.expect_overlap(
        front_frame,
        right_temple,
        axes="z",
        min_overlap=0.004,
        elem_a=right_outer_vertical,
        elem_b=right_temple_barrel,
        name="right_hinge_vertical_alignment",
    )

    front_aabb = ctx.part_world_aabb(front_frame)
    if front_aabb is not None:
        frame_width = front_aabb[1][0] - front_aabb[0][0]
        frame_depth = front_aabb[1][1] - front_aabb[0][1]
        frame_height = front_aabb[1][2] - front_aabb[0][2]
        ctx.check(
            "front_frame_proportions",
            0.125 <= frame_width <= 0.145 and 0.036 <= frame_height <= 0.050 and 0.009 <= frame_depth <= 0.018,
            f"unexpected frame dimensions width={frame_width:.4f} height={frame_height:.4f} depth={frame_depth:.4f}",
        )

    left_lens_aabb = ctx.part_element_world_aabb(front_frame, elem=left_lens)
    right_lens_aabb = ctx.part_element_world_aabb(front_frame, elem=right_lens)
    if left_lens_aabb is not None and right_lens_aabb is not None:
        left_center_x = 0.5 * (left_lens_aabb[0][0] + left_lens_aabb[1][0])
        right_center_x = 0.5 * (right_lens_aabb[0][0] + right_lens_aabb[1][0])
        left_height = left_lens_aabb[1][2] - left_lens_aabb[0][2]
        right_height = right_lens_aabb[1][2] - right_lens_aabb[0][2]
        ctx.check(
            "lenses_are_symmetric",
            abs(left_center_x + right_center_x) < 1e-4 and abs(left_height - right_height) < 1e-5,
            (
                "lenses should mirror across the bridge; "
                f"centers=({left_center_x:.4f},{right_center_x:.4f}) heights=({left_height:.4f},{right_height:.4f})"
            ),
        )

    left_open_aabb = ctx.part_world_aabb(left_temple)
    right_open_aabb = ctx.part_world_aabb(right_temple)
    with ctx.pose({left_hinge: math.radians(92.0), right_hinge: math.radians(92.0)}):
        left_folded_aabb = ctx.part_world_aabb(left_temple)
        right_folded_aabb = ctx.part_world_aabb(right_temple)
        if left_open_aabb is not None and left_folded_aabb is not None:
            ctx.check(
                "left_temple_folds_inward",
                left_folded_aabb[1][0] > left_open_aabb[1][0] + 0.050,
                (
                    "left temple should swing inward across the frame width; "
                    f"open_max_x={left_open_aabb[1][0]:.4f} folded_max_x={left_folded_aabb[1][0]:.4f}"
                ),
            )
        if right_open_aabb is not None and right_folded_aabb is not None:
            ctx.check(
                "right_temple_folds_inward",
                right_folded_aabb[0][0] < right_open_aabb[0][0] - 0.050,
                (
                    "right temple should swing inward across the frame width; "
                    f"open_min_x={right_open_aabb[0][0]:.4f} folded_min_x={right_folded_aabb[0][0]:.4f}"
                ),
            )
        ctx.expect_contact(
            left_temple,
            front_frame,
            elem_a=left_temple_barrel,
            elem_b=left_upper_barrel,
            name="left_hinge_contact_persists_folded",
        )
        ctx.expect_contact(
            right_temple,
            front_frame,
            elem_a=right_temple_barrel,
            elem_b=right_upper_barrel,
            name="right_hinge_contact_persists_folded",
        )
        ctx.expect_gap(
            left_temple,
            front_frame,
            axis="y",
            max_gap=0.012,
            max_penetration=0.001,
            positive_elem=left_main_arm,
            negative_elem=left_outer_vertical,
            name="left_temple_remains_behind_when_folded",
        )
        ctx.expect_gap(
            right_temple,
            front_frame,
            axis="y",
            max_gap=0.015,
            max_penetration=0.001,
            positive_elem=right_main_arm,
            negative_elem=right_outer_vertical,
            name="right_temple_remains_behind_when_folded",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
