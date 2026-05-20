from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

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
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="chain_link_fidget_ring",
        assets=ASSETS,
        meta={
            "object_type": "fidget_ring",
            "note": (
                "The five rectangular links are authored as a near-closed articulated "
                "loop. A strict tree articulation graph cannot encode the final closure "
                "hinge of a true kinematic cycle."
            ),
        },
    )

    satin_steel = model.material("satin_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    darker_steel = model.material("darker_steel", rgba=(0.47, 0.50, 0.54, 1.0))

    link_pitch = 0.0225
    bridge_len = 0.0042
    frame_outer_y = 0.0105
    rail = 0.0022
    frame_thickness = 0.0018
    hinge_radius = 0.00155
    center_knuckle_len = 0.0018
    outer_knuckle_len = 0.0014
    knuckle_offset_z = 0.5 * (center_knuckle_len + outer_knuckle_len)

    frame_outer_x = link_pitch - (2.0 * bridge_len)
    frame_center_x = link_pitch * 0.5
    rail_y = 0.5 * (frame_outer_y - rail)
    left_stile_x = bridge_len + (rail * 0.5)
    right_stile_x = link_pitch - bridge_len - (rail * 0.5)

    total_z = (2.0 * knuckle_offset_z) + outer_knuckle_len

    def add_frame_visuals(part) -> None:
        part.visual(
            Box((frame_outer_x, rail, frame_thickness)),
            origin=Origin(xyz=(frame_center_x, rail_y, 0.0)),
            material=satin_steel,
            name="top_rail",
        )
        part.visual(
            Box((frame_outer_x, rail, frame_thickness)),
            origin=Origin(xyz=(frame_center_x, -rail_y, 0.0)),
            material=satin_steel,
            name="bottom_rail",
        )
        part.visual(
            Box((rail, frame_outer_y, frame_thickness)),
            origin=Origin(xyz=(left_stile_x, 0.0, 0.0)),
            material=satin_steel,
            name="left_stile",
        )
        part.visual(
            Box((rail, frame_outer_y, frame_thickness)),
            origin=Origin(xyz=(right_stile_x, 0.0, 0.0)),
            material=satin_steel,
            name="right_stile",
        )
        part.visual(
            Box((bridge_len, rail, outer_knuckle_len)),
            origin=Origin(xyz=(bridge_len * 0.5, 0.0, knuckle_offset_z)),
            material=darker_steel,
            name="left_bridge_upper",
        )
        part.visual(
            Box((bridge_len, rail, outer_knuckle_len)),
            origin=Origin(xyz=(bridge_len * 0.5, 0.0, -knuckle_offset_z)),
            material=darker_steel,
            name="left_bridge_lower",
        )
        part.visual(
            Box((bridge_len, rail, center_knuckle_len)),
            origin=Origin(xyz=(link_pitch - (bridge_len * 0.5), 0.0, 0.0)),
            material=darker_steel,
            name="right_bridge",
        )
        part.visual(
            Cylinder(radius=hinge_radius, length=outer_knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, knuckle_offset_z)),
            material=darker_steel,
            name="left_knuckle_upper",
        )
        part.visual(
            Cylinder(radius=hinge_radius, length=outer_knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, -knuckle_offset_z)),
            material=darker_steel,
            name="left_knuckle_lower",
        )
        part.visual(
            Cylinder(radius=hinge_radius, length=center_knuckle_len),
            origin=Origin(xyz=(link_pitch, 0.0, 0.0)),
            material=darker_steel,
            name="right_knuckle",
        )

        part.inertial = Inertial.from_geometry(
            Box((link_pitch + (2.0 * hinge_radius), frame_outer_y, total_z)),
            mass=0.0012,
            origin=Origin(xyz=(frame_center_x, 0.0, 0.0)),
        )

    frames = [model.part(f"frame_{index}") for index in range(5)]
    for frame in frames:
        add_frame_visuals(frame)

    # Slightly under-rotate the articulated chain so the authored tree reads as a
    # compact near-closed ring without forcing the unmodeled closure hinge to
    # self-contact.
    rest_turn = 1.21
    for index in range(4):
        model.articulation(
            f"frame_{index}_to_frame_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=frames[index],
            child=frames[index + 1],
            origin=Origin(xyz=(link_pitch, 0.0, 0.0), rpy=(0.0, 0.0, rest_turn)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.15,
                velocity=8.0,
                lower=-1.05,
                upper=1.05,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, seed=0)

    frames = [object_model.get_part(f"frame_{index}") for index in range(5)]
    joints = [object_model.get_articulation(f"frame_{index}_to_frame_{index + 1}") for index in range(4)]

    top_rails = [frame.get_visual("top_rail") for frame in frames]
    bottom_rails = [frame.get_visual("bottom_rail") for frame in frames]
    left_stiles = [frame.get_visual("left_stile") for frame in frames]
    right_stiles = [frame.get_visual("right_stile") for frame in frames]
    left_knuckle_uppers = [frame.get_visual("left_knuckle_upper") for frame in frames]
    left_knuckle_lowers = [frame.get_visual("left_knuckle_lower") for frame in frames]
    right_knuckles = [frame.get_visual("right_knuckle") for frame in frames]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    # This object is a kinematic loop in reality, but authored as a tree because
    # the SDK articulation graph cannot close the final hinge cycle. Broad random
    # sampling across all four joints therefore includes impossible poses that the
    # real closed ring cannot reach. Instead, verify exact clearance at rest, at
    # representative folding poses, and at each joint limit independently.

    ctx.expect_gap(
        frames[0],
        frames[0],
        axis="y",
        min_gap=0.0055,
        positive_elem=top_rails[0],
        negative_elem=bottom_rails[0],
        name="frame_window_height_is_open",
    )
    ctx.expect_gap(
        frames[0],
        frames[0],
        axis="x",
        min_gap=0.0090,
        positive_elem=right_stiles[0],
        negative_elem=left_stiles[0],
        name="frame_window_width_is_open",
    )

    for index in range(4):
        ctx.expect_contact(
            frames[index],
            frames[index + 1],
            elem_a=right_knuckles[index],
            elem_b=left_knuckle_uppers[index + 1],
            name=f"hinge_{index}_upper_knuckle_contact",
        )
        ctx.expect_contact(
            frames[index],
            frames[index + 1],
            elem_a=right_knuckles[index],
            elem_b=left_knuckle_lowers[index + 1],
            name=f"hinge_{index}_lower_knuckle_contact",
        )
        ctx.expect_overlap(
            frames[index],
            frames[index + 1],
            axes="xy",
            min_overlap=0.0028,
            elem_a=right_knuckles[index],
            elem_b=left_knuckle_uppers[index + 1],
            name=f"hinge_{index}_upper_knuckle_axis_alignment",
        )
        ctx.expect_overlap(
            frames[index],
            frames[index + 1],
            axes="xy",
            min_overlap=0.0028,
            elem_a=right_knuckles[index],
            elem_b=left_knuckle_lowers[index + 1],
            name=f"hinge_{index}_lower_knuckle_axis_alignment",
        )

    ctx.expect_origin_distance(
        frames[4],
        frames[0],
        axes="xy",
        max_dist=0.031,
        name="five_frames_read_as_compact_ring",
    )

    folding_pose = {
        joints[0]: 0.72,
        joints[1]: -0.58,
        joints[2]: 0.61,
        joints[3]: -0.44,
    }
    with ctx.pose(folding_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="folding_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="folding_pose_no_floating")
        for index in range(4):
            ctx.expect_contact(
                frames[index],
                frames[index + 1],
                elem_a=right_knuckles[index],
                elem_b=left_knuckle_uppers[index + 1],
                name=f"folding_pose_hinge_{index}_upper_contact",
            )
            ctx.expect_contact(
                frames[index],
                frames[index + 1],
                elem_a=right_knuckles[index],
                elem_b=left_knuckle_lowers[index + 1],
                name=f"folding_pose_hinge_{index}_lower_contact",
            )
        ctx.expect_origin_distance(
            frames[4],
            frames[0],
            axes="xy",
            max_dist=0.032,
            name="folding_pose_keeps_chain_loop_like",
        )

    for joint in joints:
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_pose_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
