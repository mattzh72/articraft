from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    hw = width / 2.0
    hh = height / 2.0
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


def circle_profile(
    radius: float,
    *,
    segments: int = 20,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos(2.0 * pi * i / segments),
            cy + radius * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_service_access_panel")

    outer_w = 0.62
    outer_h = 0.82
    frame_t = 0.05
    opening_w = 0.50
    opening_h = 0.70
    reveal = 0.002
    door_w = opening_w - 2.0 * reveal
    door_h = opening_h - 2.0 * reveal
    door_t = 0.028

    frame_front_z = frame_t / 2.0
    hinge_axis_x = -(opening_w / 2.0 + 0.010)
    hinge_axis_z = frame_front_z + 0.007
    door_lead_from_hinge = 0.012
    door_center_x = door_lead_from_hinge + door_w / 2.0
    handle_x = door_lead_from_hinge + door_w - 0.055
    door_front_z = frame_front_z + 0.003
    door_center_z = door_front_z - door_t / 2.0
    door_center_local_z = door_center_z - hinge_axis_z

    frame_color = model.material("frame_charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    door_color = model.material("door_graphite", rgba=(0.34, 0.35, 0.38, 1.0))
    steel = model.material("hardware_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    adjust = model.material("adjust_feature", rgba=(0.52, 0.55, 0.58, 1.0))
    mark = model.material("index_mark", rgba=(0.93, 0.62, 0.16, 1.0))

    frame = model.part("frame")
    frame_ring = ExtrudeWithHolesGeometry(
        rect_profile(outer_w, outer_h),
        [rect_profile(opening_w, opening_h)],
        frame_t,
        center=True,
    )
    frame.visual(mesh_from_geometry(frame_ring, "frame_ring"), material=frame_color, name="frame_ring")

    frame.visual(
        Box((0.006, 0.12, 0.010)),
        origin=Origin(xyz=(-opening_w / 2.0 - 0.003, 0.0, 0.0)),
        material=adjust,
        name="left_datum_pad",
    )
    frame.visual(
        Box((0.006, 0.09, 0.010)),
        origin=Origin(xyz=(opening_w / 2.0 + 0.003, 0.18, 0.0)),
        material=adjust,
        name="right_upper_datum",
    )
    frame.visual(
        Box((0.006, 0.09, 0.010)),
        origin=Origin(xyz=(opening_w / 2.0 + 0.003, -0.18, 0.0)),
        material=adjust,
        name="right_lower_datum",
    )
    frame.visual(
        Box((0.12, 0.006, 0.010)),
        origin=Origin(xyz=(0.12, opening_h / 2.0 + 0.003, 0.0)),
        material=adjust,
        name="top_datum",
    )
    frame.visual(
        Box((0.10, 0.006, 0.010)),
        origin=Origin(xyz=(0.12, -opening_h / 2.0 - 0.003, 0.0)),
        material=adjust,
        name="bottom_datum",
    )

    frame.visual(
        Box((0.050, 0.18, 0.006)),
        origin=Origin(xyz=(hinge_axis_x - 0.025, 0.24, frame_front_z + 0.003)),
        material=steel,
        name="frame_leaf_upper",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.17),
        origin=Origin(xyz=(hinge_axis_x, 0.24, hinge_axis_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="frame_knuckle_upper",
    )
    frame.visual(
        Box((0.050, 0.18, 0.006)),
        origin=Origin(xyz=(hinge_axis_x - 0.025, -0.24, frame_front_z + 0.003)),
        material=steel,
        name="frame_leaf_lower",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.17),
        origin=Origin(xyz=(hinge_axis_x, -0.24, hinge_axis_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="frame_knuckle_lower",
    )

    frame.visual(
        Box((0.018, 0.080, 0.004)),
        origin=Origin(xyz=(hinge_axis_x - 0.038, 0.24, frame_front_z + 0.008)),
        material=adjust,
        name="upper_adjust_rail",
    )
    frame.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(hinge_axis_x - 0.038, 0.210, frame_front_z + 0.0115)),
        material=steel,
        name="upper_adjust_screw_top",
    )
    frame.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(hinge_axis_x - 0.038, 0.270, frame_front_z + 0.0115)),
        material=steel,
        name="upper_adjust_screw_bottom",
    )
    frame.visual(
        Box((0.018, 0.080, 0.004)),
        origin=Origin(xyz=(hinge_axis_x - 0.038, -0.24, frame_front_z + 0.008)),
        material=adjust,
        name="lower_adjust_rail",
    )
    frame.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(hinge_axis_x - 0.038, -0.270, frame_front_z + 0.0115)),
        material=steel,
        name="lower_adjust_screw_bottom",
    )
    frame.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(hinge_axis_x - 0.038, -0.210, frame_front_z + 0.0115)),
        material=steel,
        name="lower_adjust_screw_top",
    )

    frame.visual(
        Box((0.012, 0.085, 0.006)),
        origin=Origin(xyz=(opening_w / 2.0 + 0.006, 0.0, -0.018)),
        material=steel,
        name="strike_backplate",
    )
    frame.visual(
        Box((0.022, 0.024, 0.008)),
        origin=Origin(xyz=(opening_w / 2.0 - 0.011, 0.024, -0.018)),
        material=steel,
        name="strike_upper_guide",
    )
    frame.visual(
        Box((0.022, 0.024, 0.008)),
        origin=Origin(xyz=(opening_w / 2.0 - 0.011, -0.024, -0.018)),
        material=steel,
        name="strike_lower_guide",
    )
    frame.visual(
        Box((0.016, 0.11, 0.004)),
        origin=Origin(xyz=(opening_w / 2.0 + 0.019, 0.0, frame_front_z + 0.002)),
        material=adjust,
        name="strike_adjust_plate",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(opening_w / 2.0 + 0.019, 0.035, frame_front_z + 0.0055)),
        material=steel,
        name="strike_adjust_screw_top",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(opening_w / 2.0 + 0.019, -0.035, frame_front_z + 0.0055)),
        material=steel,
        name="strike_adjust_screw_bottom",
    )

    frame.visual(
        Box((0.018, 0.004, 0.0015)),
        origin=Origin(xyz=(-0.18, opening_h / 2.0 + 0.010, frame_t / 2.0 + 0.00075)),
        material=mark,
        name="header_index_left",
    )
    frame.visual(
        Box((0.018, 0.004, 0.0015)),
        origin=Origin(xyz=(0.18, opening_h / 2.0 + 0.010, frame_t / 2.0 + 0.00075)),
        material=mark,
        name="header_index_right",
    )

    frame.inertial = Inertial.from_geometry(Box((outer_w, outer_h, frame_t)), mass=8.5)

    door = model.part("service_panel")
    latch_hole = circle_profile(0.0085, segments=24, center=(handle_x - door_center_x, 0.0))
    door_slab = ExtrudeWithHolesGeometry(
        rect_profile(door_w, door_h),
        [latch_hole],
        door_t,
        center=True,
    )
    door.visual(
        mesh_from_geometry(door_slab, "service_panel_slab"),
        origin=Origin(xyz=(door_center_x, 0.0, door_center_local_z)),
        material=door_color,
        name="door_slab",
    )

    border_z = (door_front_z + 0.002) - hinge_axis_z
    door.visual(
        Box((0.035, door_h, 0.004)),
        origin=Origin(xyz=(door_lead_from_hinge + 0.0175, 0.0, border_z)),
        material=door_color,
        name="border_left",
    )
    door.visual(
        Box((0.035, door_h, 0.004)),
        origin=Origin(xyz=(door_lead_from_hinge + door_w - 0.0175, 0.0, border_z)),
        material=door_color,
        name="border_right",
    )
    door.visual(
        Box((door_w - 0.07, 0.035, 0.004)),
        origin=Origin(xyz=(door_center_x, door_h / 2.0 - 0.0175, border_z)),
        material=door_color,
        name="border_top",
    )
    door.visual(
        Box((door_w - 0.07, 0.035, 0.004)),
        origin=Origin(xyz=(door_center_x, -door_h / 2.0 + 0.0175, border_z)),
        material=door_color,
        name="border_bottom",
    )

    door.visual(
        Box((0.026, 0.24, 0.006)),
        origin=Origin(xyz=(0.009, 0.0, door_front_z - 0.003 - hinge_axis_z)),
        material=steel,
        name="door_leaf",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="door_knuckle_mid",
    )

    wear_pad_z = door_center_local_z - 0.010
    door.visual(
        Box((0.005, 0.11, 0.008)),
        origin=Origin(xyz=(door_lead_from_hinge + 0.0025, 0.0, wear_pad_z)),
        material=adjust,
        name="hinge_wear_pad",
    )
    door.visual(
        Box((0.005, 0.08, 0.008)),
        origin=Origin(xyz=(door_lead_from_hinge + door_w - 0.0025, 0.18, wear_pad_z)),
        material=adjust,
        name="right_upper_wear",
    )
    door.visual(
        Box((0.005, 0.08, 0.008)),
        origin=Origin(xyz=(door_lead_from_hinge + door_w - 0.0025, -0.18, wear_pad_z)),
        material=adjust,
        name="right_lower_wear",
    )
    door.visual(
        Box((0.10, 0.005, 0.008)),
        origin=Origin(xyz=(door_center_x + 0.05, door_h / 2.0 - 0.0025, wear_pad_z)),
        material=adjust,
        name="top_wear_pad",
    )
    door.visual(
        Box((0.10, 0.005, 0.008)),
        origin=Origin(xyz=(door_center_x + 0.05, -door_h / 2.0 + 0.0025, wear_pad_z)),
        material=adjust,
        name="bottom_wear_pad",
    )

    face_mark_z = (door_front_z + 0.001) - hinge_axis_z
    door.visual(
        Box((0.004, 0.018, 0.002)),
        origin=Origin(xyz=(handle_x, 0.060, face_mark_z)),
        material=mark,
        name="locked_index_mark",
    )
    door.visual(
        Box((0.018, 0.004, 0.002)),
        origin=Origin(xyz=(handle_x + 0.060, 0.0, face_mark_z)),
        material=mark,
        name="service_index_mark",
    )

    door.inertial = Inertial.from_geometry(
        Box((door_w, door_h, door_t)),
        mass=3.2,
        origin=Origin(xyz=(door_center_x, 0.0, door_center_local_z)),
    )

    latch = model.part("latch_handle")
    latch.visual(
        Cylinder(radius=0.006, length=door_t + 0.014),
        material=steel,
        name="spindle",
    )
    latch.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, door_t / 2.0 + 0.002)),
        material=steel,
        name="outer_rosette",
    )
    latch.visual(
        Box((0.012, 0.090, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, door_t / 2.0 + 0.005)),
        material=steel,
        name="turn_bar",
    )
    latch.visual(
        Box((0.048, 0.010, 0.004)),
        origin=Origin(xyz=(0.024, 0.0, -(door_t / 2.0 + 0.003))),
        material=steel,
        name="cam_tongue",
    )
    latch.visual(
        Box((0.010, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.046, door_t / 2.0 + 0.009)),
        material=mark,
        name="handle_mark",
    )
    latch.inertial = Inertial.from_geometry(Cylinder(radius=0.018, length=door_t + 0.014), mass=0.35)

    service_hinge = model.articulation(
        "service_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "latch_quarter_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(handle_x, 0.0, door_center_local_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("service_panel")
    latch = object_model.get_part("latch_handle")
    hinge = object_model.get_articulation("service_hinge")
    latch_joint = object_model.get_articulation("latch_quarter_turn")

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

    ctx.check("frame part exists", frame is not None, "frame part missing")
    ctx.check("service panel exists", door is not None, "service panel part missing")
    ctx.check("latch handle exists", latch is not None, "latch handle part missing")
    ctx.check(
        "hinge axis orientation",
        tuple(hinge.axis) == (0.0, -1.0, 0.0),
        f"expected hinge axis (0, -1, 0), got {hinge.axis}",
    )
    ctx.check(
        "latch axis orientation",
        tuple(latch_joint.axis) == (0.0, 0.0, 1.0),
        f"expected latch axis (0, 0, 1), got {latch_joint.axis}",
    )

    ctx.expect_gap(
        door,
        frame,
        axis="x",
        positive_elem="hinge_wear_pad",
        negative_elem="left_datum_pad",
        max_gap=0.0025,
        max_penetration=0.0,
        name="hinge-side controlled reveal",
    )
    ctx.expect_gap(
        frame,
        door,
        axis="x",
        positive_elem="right_upper_datum",
        negative_elem="right_upper_wear",
        max_gap=0.0025,
        max_penetration=0.0,
        name="upper latch-side controlled reveal",
    )
    ctx.expect_gap(
        frame,
        door,
        axis="x",
        positive_elem="right_lower_datum",
        negative_elem="right_lower_wear",
        max_gap=0.0025,
        max_penetration=0.0,
        name="lower latch-side controlled reveal",
    )
    ctx.expect_gap(
        frame,
        door,
        axis="y",
        positive_elem="top_datum",
        negative_elem="top_wear_pad",
        max_gap=0.0025,
        max_penetration=0.0,
        name="top controlled reveal",
    )
    ctx.expect_gap(
        door,
        frame,
        axis="y",
        positive_elem="bottom_wear_pad",
        negative_elem="bottom_datum",
        max_gap=0.0025,
        max_penetration=0.0,
        name="bottom controlled reveal",
    )
    ctx.expect_contact(
        latch,
        door,
        elem_a="outer_rosette",
        elem_b="door_slab",
        name="latch rosette seats on service panel",
    )
    ctx.expect_gap(
        frame,
        latch,
        axis="x",
        positive_elem="strike_backplate",
        negative_elem="cam_tongue",
        min_gap=0.004,
        max_gap=0.018,
        name="latched cam reaches strike pocket",
    )

    with ctx.pose({latch_joint: pi / 2.0}):
        ctx.expect_gap(
            frame,
            latch,
            axis="x",
            positive_elem="strike_backplate",
            negative_elem="cam_tongue",
            min_gap=0.024,
            name="unlatched cam retracts from strike pocket",
        )

    with ctx.pose({hinge: 1.20, latch_joint: pi / 2.0}):
        ctx.expect_contact(
            latch,
            door,
            elem_a="outer_rosette",
            elem_b="door_slab",
            name="latch remains seated when door is open",
        )
        door_slab_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
        frame_aabb = ctx.part_world_aabb(frame)
        opened_far_enough = (
            door_slab_aabb is not None
            and frame_aabb is not None
            and door_slab_aabb[1][2] > frame_aabb[1][2] + 0.28
        )
        ctx.check(
            "door swings outward in service pose",
            opened_far_enough,
            f"door max z did not move clearly outward: door={door_slab_aabb}, frame={frame_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
