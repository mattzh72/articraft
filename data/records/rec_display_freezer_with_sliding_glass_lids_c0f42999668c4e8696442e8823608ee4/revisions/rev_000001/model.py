from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supermarket_freezer_island")

    body_white = model.material("body_white", rgba=(0.92, 0.94, 0.95, 1.0))
    trim_charcoal = model.material("trim_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    frame_silver = model.material("frame_silver", rgba=(0.74, 0.77, 0.80, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.72, 0.86, 0.92, 0.38))
    bumper_black = model.material("bumper_black", rgba=(0.12, 0.12, 0.13, 1.0))

    length = 1.80
    width = 0.95
    height = 0.94
    plinth_height = 0.10
    wall_thickness = 0.04
    floor_thickness = 0.05

    top_rail_height = 0.03
    top_rail_width = 0.05
    end_cap_width = 0.06

    upper_track_height = 0.016
    upper_track_width = 0.015

    top_z = height
    body_mid_z = plinth_height + (height - plinth_height) / 2.0

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((length - 0.10, width - 0.10, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height / 2.0)),
        material=trim_charcoal,
        name="plinth",
    )
    cabinet.visual(
        Box((length, wall_thickness, height - plinth_height)),
        origin=Origin(
            xyz=(0.0, -width / 2.0 + wall_thickness / 2.0, body_mid_z),
        ),
        material=body_white,
        name="front_wall",
    )
    cabinet.visual(
        Box((length, wall_thickness, height - plinth_height)),
        origin=Origin(
            xyz=(0.0, width / 2.0 - wall_thickness / 2.0, body_mid_z),
        ),
        material=body_white,
        name="back_wall",
    )
    cabinet.visual(
        Box((wall_thickness, width - 2.0 * wall_thickness, height - plinth_height)),
        origin=Origin(
            xyz=(-length / 2.0 + wall_thickness / 2.0, 0.0, body_mid_z),
        ),
        material=body_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall_thickness, width - 2.0 * wall_thickness, height - plinth_height)),
        origin=Origin(
            xyz=(length / 2.0 - wall_thickness / 2.0, 0.0, body_mid_z),
        ),
        material=body_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((length - 2.0 * wall_thickness, width - 2.0 * wall_thickness, floor_thickness)),
        origin=Origin(
            xyz=(0.0, 0.0, plinth_height + floor_thickness / 2.0),
        ),
        material=trim_charcoal,
        name="inner_floor",
    )

    rail_length = length - 2.0 * end_cap_width
    cabinet.visual(
        Box((rail_length, top_rail_width, top_rail_height)),
        origin=Origin(
            xyz=(0.0, -width / 2.0 + top_rail_width / 2.0, top_z - top_rail_height / 2.0),
        ),
        material=trim_charcoal,
        name="front_rail",
    )
    cabinet.visual(
        Box((rail_length, top_rail_width, top_rail_height)),
        origin=Origin(
            xyz=(0.0, 0.0, top_z - top_rail_height / 2.0),
        ),
        material=trim_charcoal,
        name="center_rail",
    )
    cabinet.visual(
        Box((rail_length, top_rail_width, top_rail_height)),
        origin=Origin(
            xyz=(0.0, width / 2.0 - top_rail_width / 2.0, top_z - top_rail_height / 2.0),
        ),
        material=trim_charcoal,
        name="back_rail",
    )
    cabinet.visual(
        Box((end_cap_width, width, top_rail_height)),
        origin=Origin(
            xyz=(-length / 2.0 + end_cap_width / 2.0, 0.0, top_z - top_rail_height / 2.0),
        ),
        material=trim_charcoal,
        name="left_end_cap",
    )
    cabinet.visual(
        Box((end_cap_width, width, top_rail_height)),
        origin=Origin(
            xyz=(length / 2.0 - end_cap_width / 2.0, 0.0, top_z - top_rail_height / 2.0),
        ),
        material=trim_charcoal,
        name="right_end_cap",
    )

    cabinet.visual(
        Box((rail_length, upper_track_width, upper_track_height)),
        origin=Origin(
            xyz=(0.0, -0.4175, top_z + upper_track_height / 2.0),
        ),
        material=frame_silver,
        name="upper_front_runner",
    )
    cabinet.visual(
        Box((rail_length, upper_track_width, upper_track_height)),
        origin=Origin(
            xyz=(0.0, -0.0175, top_z + upper_track_height / 2.0),
        ),
        material=frame_silver,
        name="upper_center_runner",
    )

    handle_mount_z_upper = 0.78
    handle_mount_z_lower = 0.58
    corner_bracket_size = (0.026, 0.014, 0.032)
    corner_gusset_size = (0.018, 0.030, 0.018)
    cabinet.visual(
        Box(corner_bracket_size),
        origin=Origin(
            xyz=(-0.907, -0.479, handle_mount_z_upper),
        ),
        material=trim_charcoal,
        name="upper_handle_corner_bracket",
    )
    cabinet.visual(
        Box(corner_gusset_size),
        origin=Origin(
            xyz=(-0.901, -0.468, handle_mount_z_upper),
        ),
        material=trim_charcoal,
        name="upper_handle_corner_gusset",
    )
    cabinet.visual(
        Box(corner_bracket_size),
        origin=Origin(
            xyz=(-0.907, -0.479, handle_mount_z_lower),
        ),
        material=trim_charcoal,
        name="lower_handle_corner_bracket",
    )
    cabinet.visual(
        Box(corner_gusset_size),
        origin=Origin(
            xyz=(-0.901, -0.468, handle_mount_z_lower),
        ),
        material=trim_charcoal,
        name="lower_handle_corner_gusset",
    )

    lid_length = 1.62
    lid_width = 0.41
    frame_width = 0.025
    frame_height = 0.014
    glass_thickness = 0.006
    glass_length = lid_length - 2.0 * frame_width
    glass_width = lid_width - 2.0 * frame_width

    def add_lid(part_name: str) -> None:
        lid = model.part(part_name)
        lid.visual(
            Box((lid_length, frame_width, frame_height)),
            origin=Origin(
                xyz=(0.0, -lid_width / 2.0 + frame_width / 2.0, frame_height / 2.0),
            ),
            material=frame_silver,
            name="front_frame",
        )
        lid.visual(
            Box((lid_length, frame_width, frame_height)),
            origin=Origin(
                xyz=(0.0, lid_width / 2.0 - frame_width / 2.0, frame_height / 2.0),
            ),
            material=frame_silver,
            name="rear_frame",
        )
        lid.visual(
            Box((frame_width, lid_width, frame_height)),
            origin=Origin(
                xyz=(-lid_length / 2.0 + frame_width / 2.0, 0.0, frame_height / 2.0),
            ),
            material=frame_silver,
            name="left_frame",
        )
        lid.visual(
            Box((frame_width, lid_width, frame_height)),
            origin=Origin(
                xyz=(lid_length / 2.0 - frame_width / 2.0, 0.0, frame_height / 2.0),
            ),
            material=frame_silver,
            name="right_frame",
        )
        lid.visual(
            Box((glass_length, glass_width, glass_thickness)),
            origin=Origin(
                xyz=(0.0, 0.0, 0.008),
            ),
            material=glass_tint,
            name="glass",
        )

    add_lid("front_lid")
    add_lid("rear_lid")

    handle = model.part("front_corner_handle")
    handle.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=bumper_black,
        name="upper_pivot_sleeve",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=bumper_black,
        name="lower_pivot_sleeve",
    )
    handle.visual(
        Box((0.036, 0.012, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.100)),
        material=bumper_black,
        name="upper_link_arm",
    )
    handle.visual(
        Box((0.036, 0.012, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, -0.100)),
        material=bumper_black,
        name="lower_link_arm",
    )
    handle.visual(
        Box((0.012, 0.066, 0.012)),
        origin=Origin(xyz=(0.036, -0.027, 0.100)),
        material=bumper_black,
        name="upper_offset_bar",
    )
    handle.visual(
        Box((0.012, 0.066, 0.012)),
        origin=Origin(xyz=(0.036, -0.027, -0.100)),
        material=bumper_black,
        name="lower_offset_bar",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.220),
        origin=Origin(xyz=(0.042, -0.072, 0.0)),
        material=bumper_black,
        name="handle_grip",
    )

    model.articulation(
        "cabinet_to_front_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=model.get_part("front_lid"),
        origin=Origin(xyz=(0.0, -0.22, top_z + upper_track_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "cabinet_to_rear_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=model.get_part("rear_lid"),
        origin=Origin(xyz=(0.0, 0.22, top_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "cabinet_to_front_corner_handle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=handle,
        origin=Origin(xyz=(-0.918, -0.496, 0.68)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=1.0,
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
    cabinet = object_model.get_part("cabinet")
    front_lid = object_model.get_part("front_lid")
    rear_lid = object_model.get_part("rear_lid")
    handle = object_model.get_part("front_corner_handle")

    front_slide = object_model.get_articulation("cabinet_to_front_lid")
    rear_slide = object_model.get_articulation("cabinet_to_rear_lid")
    handle_hinge = object_model.get_articulation("cabinet_to_front_corner_handle")

    ctx.expect_gap(
        front_lid,
        cabinet,
        axis="z",
        positive_elem="front_frame",
        negative_elem="upper_front_runner",
        max_gap=0.001,
        max_penetration=0.0,
        name="front lid front edge sits on the raised front runner",
    )
    ctx.expect_gap(
        front_lid,
        cabinet,
        axis="z",
        positive_elem="rear_frame",
        negative_elem="upper_center_runner",
        max_gap=0.001,
        max_penetration=0.0,
        name="front lid rear edge sits on the raised center runner",
    )
    ctx.expect_gap(
        rear_lid,
        cabinet,
        axis="z",
        positive_elem="front_frame",
        negative_elem="center_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear lid front edge sits on the center rail",
    )
    ctx.expect_gap(
        rear_lid,
        cabinet,
        axis="z",
        positive_elem="rear_frame",
        negative_elem="back_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear lid rear edge sits on the back rail",
    )
    ctx.expect_contact(
        handle,
        cabinet,
        elem_a="upper_pivot_sleeve",
        elem_b="upper_handle_corner_bracket",
        contact_tol=0.001,
        name="upper handle pivot sleeve bears on the upper corner bracket",
    )
    ctx.expect_contact(
        handle,
        cabinet,
        elem_a="lower_pivot_sleeve",
        elem_b="lower_handle_corner_bracket",
        contact_tol=0.001,
        name="lower handle pivot sleeve bears on the lower corner bracket",
    )

    front_rest = ctx.part_world_position(front_lid)
    rear_rest = ctx.part_world_position(rear_lid)
    with ctx.pose({front_slide: 0.18, rear_slide: 0.18}):
        front_open = ctx.part_world_position(front_lid)
        rear_open = ctx.part_world_position(rear_lid)
        ctx.expect_gap(
            front_lid,
            rear_lid,
            axis="z",
            positive_elem="glass",
            negative_elem="glass",
            min_gap=0.004,
            name="stacked sliding lids keep vertical clearance",
        )

    ctx.check(
        "front lid slides rearward toward the center",
        front_rest is not None
        and front_open is not None
        and front_open[1] > front_rest[1] + 0.10,
        details=f"rest={front_rest}, open={front_open}",
    )
    ctx.check(
        "rear lid slides forward toward the center",
        rear_rest is not None
        and rear_open is not None
        and rear_open[1] < rear_rest[1] - 0.10,
        details=f"rest={rear_rest}, open={rear_open}",
    )

    handle_rest = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({handle_hinge: 1.0}):
        handle_open = ctx.part_element_world_aabb(handle, elem="handle_grip")
        ctx.expect_gap(
            cabinet,
            handle,
            axis="y",
            positive_elem="front_wall",
            negative_elem="handle_grip",
            min_gap=0.005,
            name="deployed handle grip stays in front of the cabinet face",
        )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            (min_x + max_x) / 2.0,
            (min_y + max_y) / 2.0,
            (min_z + max_z) / 2.0,
        )

    handle_rest_center = aabb_center(handle_rest)
    handle_open_center = aabb_center(handle_open)
    ctx.check(
        "corner handle swings around the front corner",
        handle_rest_center is not None
        and handle_open_center is not None
        and handle_open_center[1] > handle_rest_center[1] + 0.04
        and handle_open_center[0] > handle_rest_center[0] + 0.03,
        details=f"rest={handle_rest_center}, open={handle_open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
