from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _box(part, name, size, center, material) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _cylinder(part, name, radius, length, center, material, *, axis="z") -> None:
    rpy = (pi / 2.0, 0.0, 0.0) if axis == "y" else (0.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _build_door(
    model: ArticulatedObject,
    *,
    name: str,
    direction: float,
    door_width: float,
    door_height: float,
    door_thickness: float,
    frame_width: float,
    door_frame_y: float,
    hinge_barrel_radius: float,
    hinge_barrel_z: float,
    add_lock_bezel: bool = False,
    lock_pivot_x: float = 0.0,
):
    door = model.part(name)

    _box(
        door,
        "outer_stile",
        (frame_width, door_thickness, door_height),
        (direction * (frame_width / 2.0), door_frame_y, 0.0),
        "frame_finish",
    )
    _box(
        door,
        "inner_stile",
        (frame_width, door_thickness, door_height),
        (direction * (door_width - frame_width / 2.0), door_frame_y, 0.0),
        "frame_finish",
    )
    rail_width = door_width - (2.0 * frame_width)
    _box(
        door,
        "top_rail",
        (rail_width, door_thickness, frame_width),
        (direction * (door_width / 2.0), door_frame_y, (door_height / 2.0) - (frame_width / 2.0)),
        "frame_finish",
    )
    _box(
        door,
        "bottom_rail",
        (rail_width, door_thickness, frame_width),
        (direction * (door_width / 2.0), door_frame_y, -(door_height / 2.0) + (frame_width / 2.0)),
        "frame_finish",
    )
    _box(
        door,
        "glass_panel",
        (rail_width, 0.004, door_height - (2.0 * frame_width)),
        (direction * (door_width / 2.0), door_frame_y, 0.0),
        "glass_clear",
    )
    _cylinder(
        door,
        "upper_hinge_barrel",
        hinge_barrel_radius,
        0.08,
        (0.0, 0.0, hinge_barrel_z),
        "hardware_dark",
    )
    _cylinder(
        door,
        "lower_hinge_barrel",
        hinge_barrel_radius,
        0.08,
        (0.0, 0.0, -hinge_barrel_z),
        "hardware_dark",
    )

    if add_lock_bezel:
        _cylinder(
            door,
            "lock_bezel",
            0.008,
            0.004,
            (lock_pivot_x, door_frame_y - (door_thickness / 2.0) - 0.002, 0.0),
            "lock_brass",
            axis="y",
        )

    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_front_display_cabinet")

    model.material("carcass_finish", rgba=(0.52, 0.38, 0.24, 1.0))
    model.material("frame_finish", rgba=(0.23, 0.20, 0.17, 1.0))
    model.material("glass_clear", rgba=(0.78, 0.88, 0.96, 0.28))
    model.material("hardware_dark", rgba=(0.24, 0.24, 0.25, 1.0))
    model.material("lock_brass", rgba=(0.77, 0.63, 0.28, 1.0))

    cabinet_width = 0.90
    cabinet_depth = 0.38
    cabinet_height = 1.80
    panel_thickness = 0.02
    back_thickness = 0.006
    shelf_thickness = 0.018
    opening_clearance = 0.002
    meeting_gap = 0.004
    door_thickness = 0.028
    frame_width = 0.055
    hinge_barrel_radius = 0.008

    half_width = cabinet_width / 2.0
    half_depth = cabinet_depth / 2.0
    front_y = -half_depth
    hinge_axis_y = front_y - hinge_barrel_radius
    door_frame_y = hinge_barrel_radius + (door_thickness / 2.0)

    opening_width = cabinet_width - (2.0 * panel_thickness)
    opening_height = cabinet_height - (2.0 * panel_thickness)
    door_width = (opening_width - meeting_gap - (2.0 * opening_clearance)) / 2.0
    door_height = opening_height - (2.0 * opening_clearance)
    door_center_z = panel_thickness + opening_clearance + (door_height / 2.0)
    hinge_barrel_z = (door_height / 2.0) - 0.15

    left_hinge_x = -half_width + panel_thickness + opening_clearance
    right_hinge_x = half_width - panel_thickness - opening_clearance
    back_y = half_depth - (back_thickness / 2.0)
    back_inner_face_y = back_y - (back_thickness / 2.0)
    shelf_front_y = front_y + door_thickness + 0.006
    shelf_depth = back_inner_face_y - shelf_front_y
    shelf_y = (shelf_front_y + back_inner_face_y) / 2.0

    carcass = model.part("carcass")
    _box(
        carcass,
        "left_side_panel",
        (panel_thickness, cabinet_depth, cabinet_height),
        (-half_width + (panel_thickness / 2.0), 0.0, cabinet_height / 2.0),
        "carcass_finish",
    )
    _box(
        carcass,
        "right_side_panel",
        (panel_thickness, cabinet_depth, cabinet_height),
        (half_width - (panel_thickness / 2.0), 0.0, cabinet_height / 2.0),
        "carcass_finish",
    )
    _box(
        carcass,
        "top_panel",
        (opening_width, cabinet_depth, panel_thickness),
        (0.0, 0.0, cabinet_height - (panel_thickness / 2.0)),
        "carcass_finish",
    )
    _box(
        carcass,
        "bottom_panel",
        (opening_width, cabinet_depth, panel_thickness),
        (0.0, 0.0, panel_thickness / 2.0),
        "carcass_finish",
    )
    _box(
        carcass,
        "back_panel",
        (opening_width, back_thickness, cabinet_height - (2.0 * panel_thickness)),
        (0.0, back_y, (cabinet_height / 2.0)),
        "carcass_finish",
    )
    for index, shelf_z in enumerate((0.48, 0.92, 1.36), start=1):
        _box(
            carcass,
            f"shelf_{index}",
            (opening_width, shelf_depth, shelf_thickness),
            (0.0, shelf_y, shelf_z),
            "carcass_finish",
        )

    upper_cabinet_hinge_z = door_center_z + hinge_barrel_z + 0.08
    lower_cabinet_hinge_z = door_center_z - hinge_barrel_z - 0.08
    for side_name, hinge_x in (("left", left_hinge_x), ("right", right_hinge_x)):
        leaf_center_x = hinge_x + (-0.015 if side_name == "left" else 0.015)
        bridge_center_x = hinge_x + (-0.011 if side_name == "left" else 0.011)
        _cylinder(
            carcass,
            f"{side_name}_upper_hinge_barrel",
            hinge_barrel_radius,
            0.08,
            (hinge_x, hinge_axis_y, upper_cabinet_hinge_z),
            "hardware_dark",
        )
        _cylinder(
            carcass,
            f"{side_name}_lower_hinge_barrel",
            hinge_barrel_radius,
            0.08,
            (hinge_x, hinge_axis_y, lower_cabinet_hinge_z),
            "hardware_dark",
        )
        for leaf_name, leaf_z in (
            (f"{side_name}_upper_hinge_leaf", upper_cabinet_hinge_z),
            (f"{side_name}_lower_hinge_leaf", lower_cabinet_hinge_z),
        ):
            _box(
                carcass,
                leaf_name,
                (0.014, 0.012, 0.10),
                (leaf_center_x, front_y + 0.006, leaf_z),
                "hardware_dark",
            )
            _box(
                carcass,
                f"{leaf_name}_bridge",
                (0.012, 0.012, 0.028),
                (bridge_center_x, front_y - 0.002, leaf_z),
                "hardware_dark",
            )

    left_door = _build_door(
        model,
        name="left_door",
        direction=1.0,
        door_width=door_width,
        door_height=door_height,
        door_thickness=door_thickness,
        frame_width=frame_width,
        door_frame_y=door_frame_y,
        hinge_barrel_radius=hinge_barrel_radius,
        hinge_barrel_z=hinge_barrel_z,
    )
    lock_pivot_x = -(door_width - 0.012)
    right_door = _build_door(
        model,
        name="right_door",
        direction=-1.0,
        door_width=door_width,
        door_height=door_height,
        door_thickness=door_thickness,
        frame_width=frame_width,
        door_frame_y=door_frame_y,
        hinge_barrel_radius=hinge_barrel_radius,
        hinge_barrel_z=hinge_barrel_z,
        add_lock_bezel=True,
        lock_pivot_x=lock_pivot_x,
    )

    lock_cam = model.part("right_door_lock_cam")
    _cylinder(lock_cam, "hub", 0.006, 0.01, (0.0, 0.005, 0.0), "hardware_dark", axis="y")
    _box(lock_cam, "cam_arm", (0.038, 0.004, 0.012), (0.019, 0.012, 0.0), "lock_brass")

    left_hinge = model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(left_hinge_x, hinge_axis_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-1.6,
            upper=0.0,
        ),
    )
    right_hinge = model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(right_hinge_x, hinge_axis_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.6,
        ),
    )
    model.articulation(
        "right_door_cam_lock",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=lock_cam,
        origin=Origin(xyz=(lock_pivot_x, door_frame_y + (door_thickness / 2.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-1.57,
            upper=1.57,
        ),
    )

    left_hinge.meta["default_position"] = 0.0
    right_hinge.meta["default_position"] = 0.0

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    lock_cam = object_model.get_part("right_door_lock_cam")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    cam_lock = object_model.get_articulation("right_door_cam_lock")

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

    ctx.check(
        "door_hinge_axes_vertical",
        left_hinge.axis == (0.0, 0.0, 1.0) and right_hinge.axis == (0.0, 0.0, 1.0),
        f"left={left_hinge.axis}, right={right_hinge.axis}",
    )
    ctx.check(
        "cam_lock_axis_front_to_back",
        cam_lock.axis == (0.0, 1.0, 0.0),
        f"cam lock axis is {cam_lock.axis}",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, cam_lock: 0.0}):
        ctx.expect_contact(
            left_door,
            carcass,
            elem_a="upper_hinge_barrel",
            elem_b="left_upper_hinge_barrel",
            name="left_door_upper_hinge_contact",
        )
        ctx.expect_contact(
            right_door,
            carcass,
            elem_a="upper_hinge_barrel",
            elem_b="right_upper_hinge_barrel",
            name="right_door_upper_hinge_contact",
        )
        ctx.expect_contact(
            lock_cam,
            right_door,
            elem_a="hub",
            elem_b="inner_stile",
            name="cam_lock_hub_contacts_active_door",
        )

        ctx.expect_gap(
            left_door,
            carcass,
            axis="x",
            positive_elem="outer_stile",
            negative_elem="left_side_panel",
            min_gap=0.0015,
            max_gap=0.0025,
            name="left_door_side_clearance",
        )
        ctx.expect_gap(
            carcass,
            right_door,
            axis="x",
            positive_elem="right_side_panel",
            negative_elem="outer_stile",
            min_gap=0.0015,
            max_gap=0.0025,
            name="right_door_side_clearance",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            positive_elem="inner_stile",
            negative_elem="inner_stile",
            min_gap=0.0035,
            max_gap=0.0045,
            name="door_meeting_gap",
        )
        ctx.expect_gap(
            left_door,
            carcass,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="bottom_panel",
            min_gap=0.0015,
            max_gap=0.0025,
            name="left_door_bottom_clearance",
        )
        ctx.expect_gap(
            carcass,
            left_door,
            axis="z",
            positive_elem="top_panel",
            negative_elem="top_rail",
            min_gap=0.0015,
            max_gap=0.0025,
            name="left_door_top_clearance",
        )
        ctx.expect_gap(
            right_door,
            carcass,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="bottom_panel",
            min_gap=0.0015,
            max_gap=0.0025,
            name="right_door_bottom_clearance",
        )
        ctx.expect_gap(
            carcass,
            right_door,
            axis="z",
            positive_elem="top_panel",
            negative_elem="top_rail",
            min_gap=0.0015,
            max_gap=0.0025,
            name="right_door_top_clearance",
        )
        ctx.expect_gap(
            carcass,
            left_door,
            axis="y",
            positive_elem="shelf_1",
            negative_elem="outer_stile",
            min_gap=0.0055,
            max_gap=0.0065,
            name="interior_shelf_setback_from_left_door",
        )

        left_closed_glass = ctx.part_element_world_aabb(left_door, elem="glass_panel")
        right_closed_glass = ctx.part_element_world_aabb(right_door, elem="glass_panel")
        rest_cam = ctx.part_element_world_aabb(lock_cam, elem="cam_arm")

    with ctx.pose({left_hinge: -1.1, right_hinge: 1.1, cam_lock: 1.3}):
        left_open_glass = ctx.part_element_world_aabb(left_door, elem="glass_panel")
        right_open_glass = ctx.part_element_world_aabb(right_door, elem="glass_panel")
        rotated_cam = ctx.part_element_world_aabb(lock_cam, elem="cam_arm")

    if (
        left_closed_glass is not None
        and right_closed_glass is not None
        and left_open_glass is not None
        and right_open_glass is not None
    ):
        left_swings_out = left_open_glass[0][1] < (left_closed_glass[0][1] - 0.10)
        right_swings_out = right_open_glass[0][1] < (right_closed_glass[0][1] - 0.10)
        ctx.check(
            "doors_swing_outward_from_front_opening",
            left_swings_out and right_swings_out,
            (
                f"left min y closed/open={left_closed_glass[0][1]:.4f}/{left_open_glass[0][1]:.4f}, "
                f"right min y closed/open={right_closed_glass[0][1]:.4f}/{right_open_glass[0][1]:.4f}"
            ),
        )
    else:
        ctx.fail("doors_swing_outward_from_front_opening", "glass panel AABB unavailable")

    if rest_cam is not None and rotated_cam is not None:
        rest_z_span = rest_cam[1][2] - rest_cam[0][2]
        rotated_z_span = rotated_cam[1][2] - rotated_cam[0][2]
        ctx.check(
            "cam_lock_rotates_about_depth_axis",
            rotated_z_span > (rest_z_span + 0.015),
            f"rest z span={rest_z_span:.4f}, rotated z span={rotated_z_span:.4f}",
        )
    else:
        ctx.fail("cam_lock_rotates_about_depth_axis", "cam arm AABB unavailable")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
