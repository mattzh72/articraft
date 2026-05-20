from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

PLATE_THICK = 0.012
PLATE_WIDTH = 0.090
PLATE_HEIGHT = 0.220
WALL_STANDOFF = 0.020
WALL_JOINT_Z = 0.030

VERT_JOINT_RADIUS = 0.018
VERT_JOINT_SEGMENT = 0.012

PRIMARY_LENGTH = 0.225
SECONDARY_LENGTH = 0.185
LINK_WIDTH = 0.040
LINK_HEIGHT = 0.018
LINK_FAIRING_WIDTH = 0.028

HEAD_LENGTH = 0.072
HEAD_WIDTH = 0.060
HEAD_HEIGHT = 0.042

TILT_RADIUS = 0.012
TILT_BARREL_LENGTH = 0.042
FRAME_OUTER_WIDTH = 0.150
FRAME_OUTER_HEIGHT = 0.112
FRAME_BAR = 0.012
FRAME_THICK = 0.008


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_monitor_arm", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.14, 0.15, 0.17, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    light_metal = model.material("light_metal", rgba=(0.66, 0.68, 0.71, 1.0))
    fastener = model.material("fastener", rgba=(0.50, 0.52, 0.55, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((PLATE_THICK, PLATE_WIDTH, PLATE_HEIGHT)),
        origin=Origin(xyz=(PLATE_THICK / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="plate_shell",
    )
    wall_plate.visual(
        Cylinder(radius=VERT_JOINT_RADIUS, length=VERT_JOINT_SEGMENT),
        origin=Origin(
            xyz=(
                PLATE_THICK + 0.022,
                0.0,
                WALL_JOINT_Z - VERT_JOINT_SEGMENT / 2.0,
            )
        ),
        material=fastener,
        name="wall_joint_barrel",
    )
    wall_plate.visual(
        Box((0.018, 0.024, 0.012)),
        origin=Origin(xyz=(0.017, 0.0, WALL_JOINT_Z - 0.006)),
        material=dark_metal,
        name="wall_joint_bridge",
    )
    for side, y_pos in (("left", -0.027), ("right", 0.027)):
        wall_plate.visual(
            Box((0.022, 0.012, 0.044)),
            origin=Origin(
                xyz=(0.011, -0.024 if side == "left" else 0.024, WALL_JOINT_Z - 0.002),
            ),
            material=dark_metal,
            name=f"support_cheek_{side}",
        )
    for idx, z_pos in enumerate((-0.070, 0.070), start=1):
        for side, y_pos in (("left", -0.026), ("right", 0.026)):
            wall_plate.visual(
                Cylinder(radius=0.006, length=0.003),
                origin=Origin(
                    xyz=(PLATE_THICK + 0.0015, y_pos, z_pos),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=fastener,
                name=f"bolt_{idx}_{side}",
            )

    primary_link = model.part("primary_link")
    primary_link.visual(
        Cylinder(radius=VERT_JOINT_RADIUS, length=VERT_JOINT_SEGMENT),
        origin=Origin(xyz=(0.0, 0.0, VERT_JOINT_SEGMENT / 2.0)),
        material=fastener,
        name="prox_barrel",
    )
    primary_link.visual(
        Box((PRIMARY_LENGTH - 0.056, LINK_WIDTH, LINK_HEIGHT)),
        origin=Origin(xyz=(PRIMARY_LENGTH / 2.0, 0.0, 0.008)),
        material=matte_black,
        name="main_beam",
    )
    primary_link.visual(
        Box((0.050, LINK_FAIRING_WIDTH, LINK_HEIGHT + 0.004)),
        origin=Origin(xyz=(0.036, 0.0, 0.008)),
        material=matte_black,
        name="prox_fairing",
    )
    primary_link.visual(
        Box((0.034, LINK_FAIRING_WIDTH, LINK_HEIGHT + 0.004)),
        origin=Origin(xyz=(PRIMARY_LENGTH - 0.035, 0.0, 0.002)),
        material=matte_black,
        name="dist_fairing",
    )
    primary_link.visual(
        Cylinder(radius=VERT_JOINT_RADIUS, length=VERT_JOINT_SEGMENT),
        origin=Origin(
            xyz=(PRIMARY_LENGTH, 0.0, -VERT_JOINT_SEGMENT / 2.0),
        ),
        material=fastener,
        name="dist_barrel",
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        Cylinder(radius=VERT_JOINT_RADIUS, length=VERT_JOINT_SEGMENT),
        origin=Origin(xyz=(0.0, 0.0, VERT_JOINT_SEGMENT / 2.0)),
        material=fastener,
        name="prox_barrel",
    )
    secondary_link.visual(
        Box((SECONDARY_LENGTH - 0.048, LINK_WIDTH * 0.92, LINK_HEIGHT)),
        origin=Origin(xyz=(SECONDARY_LENGTH / 2.0, 0.0, 0.008)),
        material=matte_black,
        name="main_beam",
    )
    secondary_link.visual(
        Box((0.046, LINK_FAIRING_WIDTH * 0.95, LINK_HEIGHT + 0.004)),
        origin=Origin(xyz=(0.032, 0.0, 0.008)),
        material=matte_black,
        name="prox_fairing",
    )
    secondary_link.visual(
        Box((0.018, LINK_FAIRING_WIDTH * 0.95, LINK_HEIGHT)),
        origin=Origin(xyz=(SECONDARY_LENGTH - 0.033, 0.0, 0.008)),
        material=matte_black,
        name="dist_fairing",
    )
    secondary_link.visual(
        Box((0.006, 0.020, 0.006)),
        origin=Origin(xyz=(SECONDARY_LENGTH - 0.021, 0.0, -0.003)),
        material=matte_black,
        name="dist_connector",
    )
    secondary_link.visual(
        Cylinder(radius=VERT_JOINT_RADIUS, length=VERT_JOINT_SEGMENT),
        origin=Origin(
            xyz=(SECONDARY_LENGTH, 0.0, -VERT_JOINT_SEGMENT / 2.0),
        ),
        material=fastener,
        name="dist_barrel",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        Cylinder(radius=VERT_JOINT_RADIUS, length=VERT_JOINT_SEGMENT),
        origin=Origin(xyz=(0.0, 0.0, VERT_JOINT_SEGMENT / 2.0)),
        material=fastener,
        name="prox_barrel",
    )
    head_swivel.visual(
        Box((0.028, HEAD_WIDTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.044, 0.0, 0.008)),
        material=dark_metal,
        name="head_body",
    )
    head_swivel.visual(
        Box((0.012, HEAD_WIDTH * 0.55, HEAD_HEIGHT - 0.014)),
        origin=Origin(xyz=(0.024, 0.0, 0.008)),
        material=dark_metal,
        name="body_bridge",
    )
    head_swivel.visual(
        Box((0.010, HEAD_WIDTH * 0.72, HEAD_HEIGHT - 0.010)),
        origin=Origin(xyz=(HEAD_LENGTH - 0.021, 0.0, 0.008)),
        material=dark_metal,
        name="head_neck",
    )
    head_swivel.visual(
        Box((0.008, 0.048, 0.050)),
        origin=Origin(xyz=(HEAD_LENGTH - TILT_RADIUS - 0.004, 0.0, 0.008)),
        material=light_metal,
        name="front_pad",
    )

    mount_frame = model.part("mount_frame")
    mount_frame.visual(
        Cylinder(radius=TILT_RADIUS, length=TILT_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_metal,
        name="tilt_barrel",
    )
    mount_frame.visual(
        Box((0.018, 0.022, FRAME_OUTER_HEIGHT - FRAME_BAR)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=light_metal,
        name="spine_block",
    )
    mount_frame.visual(
        Box((FRAME_THICK, FRAME_OUTER_WIDTH, FRAME_BAR)),
        origin=Origin(
            xyz=(0.014, 0.0, FRAME_OUTER_HEIGHT / 2.0 - FRAME_BAR / 2.0),
        ),
        material=light_metal,
        name="top_bar",
    )
    mount_frame.visual(
        Box((FRAME_THICK, FRAME_OUTER_WIDTH, FRAME_BAR)),
        origin=Origin(
            xyz=(0.014, 0.0, -FRAME_OUTER_HEIGHT / 2.0 + FRAME_BAR / 2.0),
        ),
        material=light_metal,
        name="bottom_bar",
    )
    mount_frame.visual(
        Box((FRAME_THICK, FRAME_BAR, FRAME_OUTER_HEIGHT)),
        origin=Origin(
            xyz=(0.014, -FRAME_OUTER_WIDTH / 2.0 + FRAME_BAR / 2.0, 0.0),
        ),
        material=light_metal,
        name="left_bar",
    )
    mount_frame.visual(
        Box((FRAME_THICK, FRAME_BAR, FRAME_OUTER_HEIGHT)),
        origin=Origin(
            xyz=(0.014, FRAME_OUTER_WIDTH / 2.0 - FRAME_BAR / 2.0, 0.0),
        ),
        material=light_metal,
        name="right_bar",
    )

    model.articulation(
        "wall_to_primary",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=primary_link,
        origin=Origin(xyz=(PLATE_THICK + 0.022, 0.0, WALL_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-1.4,
            upper=1.4,
        ),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(PRIMARY_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-1.75,
            upper=1.75,
        ),
    )
    model.articulation(
        "secondary_to_head",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=head_swivel,
        origin=Origin(xyz=(SECONDARY_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-2.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "head_to_frame",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=mount_frame,
        origin=Origin(xyz=(HEAD_LENGTH, 0.0, 0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=-0.31,
            upper=0.31,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    wall_plate = object_model.get_part("wall_plate")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    head_swivel = object_model.get_part("head_swivel")
    mount_frame = object_model.get_part("mount_frame")

    wall_to_primary = object_model.get_articulation("wall_to_primary")
    primary_to_secondary = object_model.get_articulation("primary_to_secondary")
    secondary_to_head = object_model.get_articulation("secondary_to_head")
    head_to_frame = object_model.get_articulation("head_to_frame")

    wall_joint_barrel = wall_plate.get_visual("wall_joint_barrel")
    primary_prox_barrel = primary_link.get_visual("prox_barrel")
    primary_dist_barrel = primary_link.get_visual("dist_barrel")
    secondary_prox_barrel = secondary_link.get_visual("prox_barrel")
    secondary_dist_barrel = secondary_link.get_visual("dist_barrel")
    head_prox_barrel = head_swivel.get_visual("prox_barrel")
    head_front_pad = head_swivel.get_visual("front_pad")
    frame_tilt_barrel = mount_frame.get_visual("tilt_barrel")

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
    for part_name, part in (
        ("wall_plate", wall_plate),
        ("primary_link", primary_link),
        ("secondary_link", secondary_link),
        ("head_swivel", head_swivel),
        ("mount_frame", mount_frame),
    ):
        ctx.check(f"{part_name}_present", part is not None)

    ctx.expect_gap(
        primary_link,
        wall_plate,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        positive_elem=primary_prox_barrel,
        negative_elem=wall_joint_barrel,
        name="wall_joint_barrels_stacked",
    )
    ctx.expect_overlap(
        primary_link,
        wall_plate,
        axes="xy",
        min_overlap=0.030,
        elem_a=primary_prox_barrel,
        elem_b=wall_joint_barrel,
        name="wall_joint_barrels_share_axis",
    )
    ctx.expect_gap(
        secondary_link,
        primary_link,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        positive_elem=secondary_prox_barrel,
        negative_elem=primary_dist_barrel,
        name="elbow_joint_barrels_stacked",
    )
    ctx.expect_overlap(
        secondary_link,
        primary_link,
        axes="xy",
        min_overlap=0.030,
        elem_a=secondary_prox_barrel,
        elem_b=primary_dist_barrel,
        name="elbow_joint_barrels_share_axis",
    )
    ctx.expect_gap(
        head_swivel,
        secondary_link,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        positive_elem=head_prox_barrel,
        negative_elem=secondary_dist_barrel,
        name="head_swivel_barrels_stacked",
    )
    ctx.expect_overlap(
        head_swivel,
        secondary_link,
        axes="xy",
        min_overlap=0.030,
        elem_a=head_prox_barrel,
        elem_b=secondary_dist_barrel,
        name="head_swivel_barrels_share_axis",
    )
    ctx.expect_gap(
        mount_frame,
        head_swivel,
        axis="x",
        min_gap=0.0,
        max_gap=0.0005,
        positive_elem=frame_tilt_barrel,
        negative_elem=head_front_pad,
        name="tilt_head_seated_on_front_pad",
    )
    ctx.expect_overlap(
        mount_frame,
        head_swivel,
        axes="yz",
        min_overlap=0.024,
        elem_a=frame_tilt_barrel,
        elem_b=head_front_pad,
        name="tilt_axis_aligned_to_head",
    )
    ctx.expect_origin_gap(
        mount_frame,
        wall_plate,
        axis="x",
        min_gap=0.45,
        name="frame_projects_out_from_wall",
    )

    ctx.check(
        "fold_joint_axes_vertical",
        tuple(wall_to_primary.axis) == (0.0, 0.0, 1.0)
        and tuple(primary_to_secondary.axis) == (0.0, 0.0, 1.0)
        and tuple(secondary_to_head.axis) == (0.0, 0.0, 1.0),
        details="wall, elbow, and swivel joints should all rotate about vertical pin axes",
    )
    ctx.check(
        "tilt_joint_axis_horizontal",
        tuple(head_to_frame.axis) == (0.0, 1.0, 0.0),
        details="frame tilt should rotate about a horizontal left-right axis",
    )
    ctx.check(
        "tilt_range_compact",
        abs(head_to_frame.motion_limits.lower + 0.31) < 1e-9
        and abs(head_to_frame.motion_limits.upper - 0.31) < 1e-9,
        details="tilt range should be about +/-18 degrees",
    )

    with ctx.pose(
        {
            wall_to_primary: math.radians(55.0),
            primary_to_secondary: math.radians(-80.0),
        }
    ):
        ctx.expect_origin_gap(
            secondary_link,
            wall_plate,
            axis="x",
            min_gap=0.06,
            name="folded_secondary_stays_clear_of_wall",
        )

    with ctx.pose({secondary_to_head: math.radians(70.0)}):
        ctx.expect_origin_gap(
            head_swivel,
            wall_plate,
            axis="x",
            min_gap=0.20,
            name="swivel_head_remains_forward_of_wall",
        )

    for pose_name, tilt_angle in (
        ("tilt_up_limit", 0.31),
        ("tilt_down_limit", -0.31),
    ):
        with ctx.pose({head_to_frame: tilt_angle}):
            ctx.expect_gap(
                mount_frame,
                head_swivel,
                axis="x",
                min_gap=0.0,
                max_gap=0.0005,
                positive_elem=frame_tilt_barrel,
                negative_elem=head_front_pad,
                name=f"{pose_name}_barrel_stays_seated",
            )
            ctx.expect_overlap(
                mount_frame,
                head_swivel,
                axes="yz",
                min_overlap=0.024,
                elem_a=frame_tilt_barrel,
                elem_b=head_front_pad,
                name=f"{pose_name}_tilt_axis_stays_aligned",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
