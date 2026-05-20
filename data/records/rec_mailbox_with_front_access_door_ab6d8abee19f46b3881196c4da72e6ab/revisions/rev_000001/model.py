from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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
    model = ArticulatedObject(name="security_mailbox_on_post")

    model.material("post_black", rgba=(0.11, 0.11, 0.11, 1.0))
    model.material("body_charcoal", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("door_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("lock_metal", rgba=(0.70, 0.72, 0.74, 1.0))

    post_w = 0.10
    post_d = 0.08
    post_h = 1.15

    body_w = 0.34
    body_d = 0.24
    body_h = 0.58
    panel_t = 0.012
    front_y = body_d / 2.0 - panel_t / 2.0

    side_rail_w = 0.04
    open_w = body_w - 2.0 * side_rail_w
    bottom_rail_h = 0.05
    door_open_h = 0.39
    mid_rail_h = 0.022
    slot_region_h = 0.078
    top_rail_h = 0.04

    door_t = 0.016
    door_hinge_r = 0.004
    door_hinge_y = front_y + panel_t / 2.0 + door_hinge_r
    door_leaf_w = 0.012
    door_panel_w = 0.240
    door_panel_h = 0.382
    door_panel_y = -(door_t / 2.0 + door_hinge_r - 0.002)
    door_hinge_x = -open_w / 2.0 - door_hinge_r
    door_center_z = (-body_h / 2.0 + bottom_rail_h) + door_open_h / 2.0

    slot_cover_t = 0.012
    slot_cover_h = 0.072
    slot_cover_w = 0.244
    slot_hinge_r = 0.005
    slot_hinge_y = front_y + panel_t / 2.0 + slot_hinge_r
    slot_top_z = body_h / 2.0 - top_rail_h
    slot_hinge_z = slot_top_z + slot_hinge_r
    slot_panel_y = -(slot_cover_t / 2.0 + slot_hinge_r)

    post = model.part("post")
    post.visual(
        Box((post_w, post_d, post_h)),
        origin=Origin(xyz=(0.0, 0.0, post_h / 2.0)),
        material="post_black",
        name="post_shaft",
    )
    post.visual(
        Box((0.14, 0.12, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, post_h - 0.006)),
        material="post_black",
        name="post_cap",
    )
    post.inertial = Inertial.from_geometry(
        Box((post_w, post_d, post_h)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, post_h / 2.0)),
    )

    body = model.part("body")
    body.visual(
        Box((body_w, panel_t, body_h)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 + panel_t / 2.0, 0.0)),
        material="body_charcoal",
        name="back_panel",
    )
    body.visual(
        Box((panel_t, body_d - panel_t, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + panel_t / 2.0, panel_t / 2.0, 0.0)),
        material="body_charcoal",
        name="left_wall",
    )
    body.visual(
        Box((panel_t, body_d - panel_t, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - panel_t / 2.0, panel_t / 2.0, 0.0)),
        material="body_charcoal",
        name="right_wall",
    )
    body.visual(
        Box((body_w - 2.0 * panel_t, body_d - panel_t, panel_t)),
        origin=Origin(xyz=(0.0, panel_t / 2.0, body_h / 2.0 - panel_t / 2.0)),
        material="body_charcoal",
        name="top_panel",
    )
    body.visual(
        Box((body_w - 2.0 * panel_t, body_d - panel_t, panel_t)),
        origin=Origin(xyz=(0.0, panel_t / 2.0, -body_h / 2.0 + panel_t / 2.0)),
        material="body_charcoal",
        name="bottom_panel",
    )
    body.visual(
        Box((body_w + 0.02, body_d + 0.02, 0.014)),
        origin=Origin(xyz=(0.0, 0.004, body_h / 2.0 + 0.007)),
        material="body_charcoal",
        name="roof_cap",
    )
    body.visual(
        Box((side_rail_w, panel_t, body_h - top_rail_h - bottom_rail_h)),
        origin=Origin(
            xyz=(
                -body_w / 2.0 + side_rail_w / 2.0,
                front_y,
                ((body_h / 2.0 - top_rail_h) + (-body_h / 2.0 + bottom_rail_h)) / 2.0,
            )
        ),
        material="body_charcoal",
        name="front_left_rail",
    )
    body.visual(
        Box((side_rail_w, panel_t, body_h - top_rail_h - bottom_rail_h)),
        origin=Origin(
            xyz=(
                body_w / 2.0 - side_rail_w / 2.0,
                front_y,
                ((body_h / 2.0 - top_rail_h) + (-body_h / 2.0 + bottom_rail_h)) / 2.0,
            )
        ),
        material="body_charcoal",
        name="front_right_rail",
    )
    body.visual(
        Box((body_w, panel_t, top_rail_h)),
        origin=Origin(xyz=(0.0, front_y, body_h / 2.0 - top_rail_h / 2.0)),
        material="body_charcoal",
        name="front_top_rail",
    )
    body.visual(
        Box((body_w, panel_t, bottom_rail_h)),
        origin=Origin(xyz=(0.0, front_y, -body_h / 2.0 + bottom_rail_h / 2.0)),
        material="body_charcoal",
        name="front_bottom_rail",
    )
    body.visual(
        Box((open_w, panel_t, mid_rail_h)),
        origin=Origin(
            xyz=(
                0.0,
                front_y,
                (-body_h / 2.0 + bottom_rail_h + door_open_h) + mid_rail_h / 2.0,
            )
        ),
        material="body_charcoal",
        name="front_mid_rail",
    )

    body.visual(
        Box((0.014, 0.008, door_panel_h + 0.04)),
        origin=Origin(xyz=(door_hinge_x - 0.003, front_y + panel_t / 6.0, door_center_z)),
        material="body_charcoal",
        name="body_hinge_jamb",
    )
    body.visual(
        Box((0.03, 0.008, 0.018)),
        origin=Origin(xyz=(-0.09, front_y + panel_t / 6.0, slot_hinge_z)),
        material="body_charcoal",
        name="slot_clip_left",
    )
    body.visual(
        Box((0.03, 0.008, 0.018)),
        origin=Origin(xyz=(0.09, front_y + panel_t / 6.0, slot_hinge_z)),
        material="body_charcoal",
        name="slot_clip_right",
    )

    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=12.0,
        origin=Origin(),
    )

    retrieval_door = model.part("retrieval_door")
    retrieval_door.visual(
        Box((door_panel_w, door_t, door_panel_h)),
        origin=Origin(xyz=(door_hinge_r + door_leaf_w + door_panel_w / 2.0, door_panel_y, 0.0)),
        material="door_charcoal",
        name="door_panel",
    )
    retrieval_door.visual(
        Box((door_leaf_w, 0.010, door_panel_h)),
        origin=Origin(xyz=(door_hinge_r + door_leaf_w / 2.0, -0.004, 0.0)),
        material="door_charcoal",
        name="door_hinge_leaf",
    )
    retrieval_door.visual(
        Cylinder(radius=door_hinge_r, length=door_panel_h - 0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="door_charcoal",
        name="door_hinge_barrel",
    )
    retrieval_door.inertial = Inertial.from_geometry(
        Box((door_panel_w + door_leaf_w + 2.0 * door_hinge_r, door_t, door_panel_h)),
        mass=3.2,
        origin=Origin(xyz=(door_hinge_r + door_leaf_w + door_panel_w / 2.0, 0.0, 0.0)),
    )

    cam_lock = model.part("cam_lock")
    cam_lock.visual(
        Cylinder(radius=0.016, length=0.02),
        origin=Origin(xyz=(0.0, 0.01, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="lock_metal",
        name="lock_escutcheon",
    )
    cam_lock.visual(
        Box((0.07, 0.007, 0.014)),
        origin=Origin(xyz=(0.0, 0.0235, 0.0)),
        material="lock_metal",
        name="lock_turn_tab",
    )
    cam_lock.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.02),
        mass=0.2,
        origin=Origin(xyz=(0.0, 0.01, 0.0)),
    )

    slot_cover = model.part("slot_cover")
    slot_cover.visual(
        Box((slot_cover_w, slot_cover_t, slot_cover_h)),
        origin=Origin(xyz=(0.0, slot_panel_y, -(slot_hinge_r + slot_cover_h / 2.0))),
        material="door_charcoal",
        name="slot_lid_panel",
    )
    slot_cover.visual(
        Box((slot_cover_w, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, -0.005)),
        material="door_charcoal",
        name="slot_hinge_bracket",
    )
    slot_cover.visual(
        Box((slot_cover_w, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.017, -(slot_hinge_r + slot_cover_h - 0.007))),
        material="door_charcoal",
        name="slot_lid_lip",
    )
    slot_cover.visual(
        Cylinder(radius=slot_hinge_r, length=slot_cover_w - 0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="door_charcoal",
        name="slot_hinge_rod",
    )
    slot_cover.inertial = Inertial.from_geometry(
        Box((slot_cover_w, 0.02, slot_cover_h)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.007, -(slot_hinge_r + slot_cover_h / 2.0))),
    )

    body_origin = Origin(
        xyz=(0.0, post_d / 2.0 + body_d / 2.0, post_h + body_h / 2.0),
    )
    model.articulation(
        "post_to_body",
        ArticulationType.FIXED,
        parent=post,
        child=body,
        origin=body_origin,
    )
    model.articulation(
        "body_to_retrieval_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=retrieval_door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_to_cam_lock",
        ArticulationType.REVOLUTE,
        parent=retrieval_door,
        child=cam_lock,
        origin=Origin(xyz=(door_hinge_r + door_leaf_w + door_panel_w / 2.0, door_panel_y + door_t / 2.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-pi / 2.0, upper=pi / 2.0),
    )
    model.articulation(
        "body_to_slot_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=slot_cover,
        origin=Origin(xyz=(0.0, slot_hinge_y, slot_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("post")
    body = object_model.get_part("body")
    retrieval_door = object_model.get_part("retrieval_door")
    cam_lock = object_model.get_part("cam_lock")
    slot_cover = object_model.get_part("slot_cover")

    door_hinge = object_model.get_articulation("body_to_retrieval_door")
    lock_joint = object_model.get_articulation("door_to_cam_lock")
    slot_hinge = object_model.get_articulation("body_to_slot_cover")

    back_panel = body.get_visual("back_panel")
    front_left_rail = body.get_visual("front_left_rail")
    front_top_rail = body.get_visual("front_top_rail")
    body_hinge_jamb = body.get_visual("body_hinge_jamb")
    slot_clip_left = body.get_visual("slot_clip_left")
    door_panel = retrieval_door.get_visual("door_panel")
    door_hinge_barrel = retrieval_door.get_visual("door_hinge_barrel")
    slot_lid_panel = slot_cover.get_visual("slot_lid_panel")
    slot_hinge_rod = slot_cover.get_visual("slot_hinge_rod")
    lock_escutcheon = cam_lock.get_visual("lock_escutcheon")
    lock_turn_tab = cam_lock.get_visual("lock_turn_tab")

    def _elem_aabb(part_obj, elem_name: str) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
        return ctx.part_element_world_aabb(part_obj, elem=elem_name)

    def _extent(aabb, axis: int) -> float:
        return aabb[1][axis] - aabb[0][axis]

    def _center(aabb, axis: int) -> float:
        return (aabb[0][axis] + aabb[1][axis]) / 2.0

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
        "articulated_parts_present",
        all(part is not None for part in (post, body, retrieval_door, cam_lock, slot_cover)),
        "Expected post, body, retrieval_door, cam_lock, and slot_cover parts.",
    )
    ctx.check(
        "door_hinge_is_vertical",
        isclose(door_hinge.axis[0], 0.0, abs_tol=1e-9)
        and isclose(door_hinge.axis[1], 0.0, abs_tol=1e-9)
        and isclose(door_hinge.axis[2], 1.0, abs_tol=1e-9),
        f"Door hinge axis was {door_hinge.axis}, expected vertical z-axis.",
    )
    ctx.check(
        "cam_lock_axis_faces_outward",
        isclose(lock_joint.axis[0], 0.0, abs_tol=1e-9)
        and isclose(lock_joint.axis[1], 1.0, abs_tol=1e-9)
        and isclose(lock_joint.axis[2], 0.0, abs_tol=1e-9),
        f"Cam lock axis was {lock_joint.axis}, expected local door-normal y-axis.",
    )
    ctx.check(
        "slot_cover_hinge_is_upper_horizontal",
        isclose(slot_hinge.axis[0], 1.0, abs_tol=1e-9)
        and isclose(slot_hinge.axis[1], 0.0, abs_tol=1e-9)
        and isclose(slot_hinge.axis[2], 0.0, abs_tol=1e-9),
        f"Slot cover axis was {slot_hinge.axis}, expected horizontal x-axis.",
    )

    ctx.expect_contact(body, post, elem_a=back_panel, elem_b="post_shaft", contact_tol=1e-6)
    ctx.expect_contact(retrieval_door, body, elem_a=door_hinge_barrel, elem_b=body_hinge_jamb, contact_tol=1e-6)
    ctx.expect_contact(slot_cover, body, elem_a=slot_hinge_rod, elem_b=slot_clip_left, contact_tol=1e-6)
    ctx.expect_contact(cam_lock, retrieval_door, elem_a=lock_escutcheon, elem_b=door_panel, contact_tol=1e-6)

    closed_door_aabb = _elem_aabb(retrieval_door, "door_panel")
    left_rail_aabb = _elem_aabb(body, "front_left_rail")
    closed_slot_aabb = _elem_aabb(slot_cover, "slot_lid_panel")
    top_rail_aabb = _elem_aabb(body, "front_top_rail")

    ctx.check(
        "door_sits_nearly_flush_when_closed",
        closed_door_aabb is not None
        and left_rail_aabb is not None
        and 0.0 <= closed_door_aabb[1][1] - left_rail_aabb[1][1] <= 0.0045,
        "Retrieval door outer face should sit almost flush with the front frame.",
    )
    ctx.check(
        "slot_cover_sits_flush_under_header",
        closed_slot_aabb is not None
        and top_rail_aabb is not None
        and abs(closed_slot_aabb[1][1] - top_rail_aabb[1][1]) <= 0.0015,
        "Slot cover should read as a small flush front lid below the top header.",
    )

    with ctx.pose({door_hinge: 1.2}):
        open_door_aabb = _elem_aabb(retrieval_door, "door_panel")
    ctx.check(
        "door_panel_swings_outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10,
        "Opening the retrieval door should move the large panel outward from the mailbox front.",
    )

    with ctx.pose({slot_hinge: 1.0}):
        open_slot_aabb = _elem_aabb(slot_cover, "slot_lid_panel")
    ctx.check(
        "slot_cover_lifts_on_upper_hinge",
        closed_slot_aabb is not None
        and open_slot_aabb is not None
        and open_slot_aabb[1][1] > closed_slot_aabb[1][1] + 0.025
        and open_slot_aabb[0][2] > closed_slot_aabb[0][2] + 0.015,
        "Opening the slot cover should lift the small lid upward and outward while staying hinged at the top.",
    )

    with ctx.pose({lock_joint: 0.0}):
        lock_tab_closed = _elem_aabb(cam_lock, "lock_turn_tab")
    with ctx.pose({lock_joint: pi / 2.0}):
        lock_tab_turned = _elem_aabb(cam_lock, "lock_turn_tab")
    ctx.check(
        "cam_lock_rotates_about_local_axis",
        lock_tab_closed is not None
        and lock_tab_turned is not None
        and _extent(lock_tab_closed, 0) > _extent(lock_tab_closed, 2) + 0.03
        and _extent(lock_tab_turned, 2) > _extent(lock_tab_turned, 0) + 0.03,
        "The lock tab should visibly rotate from horizontal to vertical about its local axis.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
