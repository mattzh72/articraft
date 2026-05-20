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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

LINK_PITCH = 0.125
BLOCK_LENGTH = 0.020
CENTER_BLOCK_WIDTH = 0.010
EAR_THICKNESS = 0.005
HINGE_BLOCK_HEIGHT = 0.028
EAR_OFFSET_Y = CENTER_BLOCK_WIDTH * 0.5 + EAR_THICKNESS * 0.5
SPINE_START_X = BLOCK_LENGTH * 0.5
SPINE_END_X = 0.095
SPINE_LENGTH = SPINE_END_X - SPINE_START_X
SPINE_HEIGHT = 0.008
SPINE_WIDTH = CENTER_BLOCK_WIDTH
YOKE_LENGTH = LINK_PITCH - BLOCK_LENGTH * 0.5 - SPINE_END_X
YOKE_CENTER_X = (SPINE_END_X + (LINK_PITCH - BLOCK_LENGTH * 0.5)) * 0.5
YOKE_WIDTH = 2.0 * EAR_OFFSET_Y + EAR_THICKNESS
YOKE_HEIGHT = 0.008
MOUNT_HEIGHT = 0.065
BASE_PLATE_THICKNESS = 0.014


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolute_chain", assets=ASSETS)

    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.72, 0.74, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.34, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    base_foot = model.part("base_foot")
    base_foot.visual(
        Box((0.120, 0.080, BASE_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS * 0.5)),
        material=rubber,
        name="plate",
    )
    post_height = MOUNT_HEIGHT - HINGE_BLOCK_HEIGHT * 0.5 - BASE_PLATE_THICKNESS
    post_center_z = BASE_PLATE_THICKNESS + post_height * 0.5
    base_foot.visual(
        Box((0.024, 0.018, post_height)),
        origin=Origin(xyz=(0.0, 0.0, post_center_z)),
        material=dark_steel,
        name="post",
    )
    base_foot.visual(
        Box((0.020, EAR_THICKNESS, 0.022)),
        origin=Origin(xyz=(-0.004, EAR_OFFSET_Y, MOUNT_HEIGHT - 0.004)),
        material=dark_steel,
        name="support_upper",
    )
    base_foot.visual(
        Box((0.020, EAR_THICKNESS, 0.022)),
        origin=Origin(xyz=(-0.004, -EAR_OFFSET_Y, MOUNT_HEIGHT - 0.004)),
        material=dark_steel,
        name="support_lower",
    )
    base_foot.visual(
        Box((BLOCK_LENGTH, EAR_THICKNESS, HINGE_BLOCK_HEIGHT)),
        origin=Origin(xyz=(0.0, EAR_OFFSET_Y, MOUNT_HEIGHT)),
        material=dark_steel,
        name="ear_upper",
    )
    base_foot.visual(
        Box((BLOCK_LENGTH, EAR_THICKNESS, HINGE_BLOCK_HEIGHT)),
        origin=Origin(xyz=(0.0, -EAR_OFFSET_Y, MOUNT_HEIGHT)),
        material=dark_steel,
        name="ear_lower",
    )
    base_foot.inertial = Inertial.from_geometry(
        Box((0.120, 0.080, MOUNT_HEIGHT + 0.020)),
        mass=1.40,
        origin=Origin(xyz=(0.0, 0.0, (MOUNT_HEIGHT + 0.020) * 0.5)),
    )

    links = []
    for index in range(5):
        link = model.part(f"link_{index + 1}")
        material = dark_steel if index == 0 else anodized_aluminum
        link.visual(
            Box((BLOCK_LENGTH, CENTER_BLOCK_WIDTH, HINGE_BLOCK_HEIGHT)),
            origin=Origin(),
            material=material,
            name="inboard_block",
        )
        link.visual(
            Box((SPINE_LENGTH, SPINE_WIDTH, SPINE_HEIGHT)),
            origin=Origin(xyz=((SPINE_START_X + SPINE_END_X) * 0.5, 0.0, 0.0)),
            material=material,
            name="spine",
        )
        link.visual(
            Box((YOKE_LENGTH, YOKE_WIDTH, YOKE_HEIGHT)),
            origin=Origin(xyz=(YOKE_CENTER_X, 0.0, 0.0)),
            material=material,
            name="yoke",
        )
        link.visual(
            Box((BLOCK_LENGTH, EAR_THICKNESS, HINGE_BLOCK_HEIGHT)),
            origin=Origin(xyz=(LINK_PITCH, EAR_OFFSET_Y, 0.0)),
            material=material,
            name="ear_upper",
        )
        link.visual(
            Box((BLOCK_LENGTH, EAR_THICKNESS, HINGE_BLOCK_HEIGHT)),
            origin=Origin(xyz=(LINK_PITCH, -EAR_OFFSET_Y, 0.0)),
            material=material,
            name="ear_lower",
        )
        link.inertial = Inertial.from_geometry(
            Box((LINK_PITCH + BLOCK_LENGTH, 2.0 * EAR_OFFSET_Y + EAR_THICKNESS, HINGE_BLOCK_HEIGHT)),
            mass=0.42,
            origin=Origin(xyz=(LINK_PITCH * 0.5, 0.0, 0.0)),
        )
        links.append(link)

    model.articulation(
        "foot_to_link_1",
        ArticulationType.FIXED,
        parent=base_foot,
        child=links[0],
        origin=Origin(xyz=(0.0, 0.0, MOUNT_HEIGHT)),
    )

    for index in range(4):
        model.articulation(
            f"joint_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=links[index],
            child=links[index + 1],
            origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.5,
                lower=-math.pi / 2.0,
                upper=math.pi / 2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_foot = object_model.get_part("base_foot")
    links = [object_model.get_part(f"link_{index}") for index in range(1, 6)]
    joints = [object_model.get_articulation(f"joint_{index}") for index in range(1, 5)]
    foot_ear_upper = base_foot.get_visual("ear_upper")
    foot_ear_lower = base_foot.get_visual("ear_lower")
    foot_post = base_foot.get_visual("post")
    inboard_blocks = [link.get_visual("inboard_block") for link in links]
    upper_ears = [link.get_visual("ear_upper") for link in links]
    lower_ears = [link.get_visual("ear_lower") for link in links]

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

    ctx.check(
        "part_count",
        len(object_model.parts) == 6,
        details=f"Expected 6 parts (base foot + 5 links), found {len(object_model.parts)}.",
    )
    ctx.check(
        "joint_count",
        len(object_model.articulations) == 5,
        details=f"Expected 5 articulations (1 fixed + 4 revolute), found {len(object_model.articulations)}.",
    )

    ctx.expect_contact(base_foot, links[0], elem_a=foot_ear_upper, elem_b=inboard_blocks[0], name="foot_upper_contact")
    ctx.expect_contact(base_foot, links[0], elem_a=foot_ear_lower, elem_b=inboard_blocks[0], name="foot_lower_contact")
    ctx.expect_contact(base_foot, links[0], elem_a=foot_post, elem_b=inboard_blocks[0], name="foot_mount_saddle_contact")
    for index in range(1, 5):
        ctx.expect_contact(
            links[index - 1],
            links[index],
            elem_a=upper_ears[index - 1],
            elem_b=inboard_blocks[index],
            name=f"joint_upper_contact_{index}",
        )
        ctx.expect_contact(
            links[index - 1],
            links[index],
            elem_a=lower_ears[index - 1],
            elem_b=inboard_blocks[index],
            name=f"joint_lower_contact_{index}",
        )

    ctx.expect_origin_gap(
        links[0],
        base_foot,
        axis="z",
        min_gap=MOUNT_HEIGHT - 1e-4,
        max_gap=MOUNT_HEIGHT + 1e-4,
        name="root_mount_height",
    )
    for index in range(1, 5):
        ctx.expect_origin_gap(
            links[index],
            links[index - 1],
            axis="x",
            min_gap=LINK_PITCH - 1e-4,
            max_gap=LINK_PITCH + 1e-4,
            name=f"even_pitch_{index}",
        )
        ctx.expect_origin_distance(
            links[index],
            links[index - 1],
            axes="yz",
            max_dist=1e-5,
            name=f"coplanar_axes_{index}",
        )

    for index, joint in enumerate(joints, start=1):
        limits = joint.motion_limits
        ctx.check(
            f"joint_{index}_limits",
            limits is not None
            and abs(limits.lower + math.pi / 2.0) < 1e-9
            and abs(limits.upper - math.pi / 2.0) < 1e-9,
            details=f"Joint {index} should have +/- 90 degree travel, got {limits}.",
        )
        ctx.check(
            f"joint_{index}_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"Joint {index} axis should be parallel to +Y, got {joint.axis}.",
        )

    rest_tip = ctx.part_world_position(links[-1])
    with ctx.pose({joints[0]: 0.9, joints[1]: -0.7, joints[2]: 0.6, joints[3]: -0.5}):
        posed_tip = ctx.part_world_position(links[-1])
        for index in range(1, 5):
            ctx.expect_contact(
                links[index - 1],
                links[index],
                elem_a=upper_ears[index - 1],
                elem_b=inboard_blocks[index],
                name=f"joint_upper_contact_pose_{index}",
            )
            ctx.expect_contact(
                links[index - 1],
                links[index],
                elem_a=lower_ears[index - 1],
                elem_b=inboard_blocks[index],
                name=f"joint_lower_contact_pose_{index}",
            )
        ctx.check(
            "articulation_changes_tip_position",
            rest_tip is not None
            and posed_tip is not None
            and math.dist(rest_tip, posed_tip) > LINK_PITCH * 1.2,
            details=f"Expected chained revolute motion to move the tip significantly, got rest={rest_tip}, posed={posed_tip}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
