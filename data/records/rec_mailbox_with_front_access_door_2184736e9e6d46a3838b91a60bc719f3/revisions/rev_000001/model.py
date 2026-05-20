from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, depth: float, *, cx: float = 0.0, cy: float = 0.0) -> list[tuple[float, float]]:
    hw = width * 0.5
    hd = depth * 0.5
    return [
        (cx - hw, cy - hd),
        (cx + hw, cy - hd),
        (cx + hw, cy + hd),
        (cx - hw, cy + hd),
    ]


def _plate_with_rect_opening(
    *,
    name: str,
    width: float,
    depth: float,
    thickness: float,
    opening_width: float,
    opening_depth: float,
    opening_center_y: float,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(width, depth),
            [_rect_profile(opening_width, opening_depth, cy=opening_center_y)],
            thickness,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="secure_parcel_mailbox")

    body_paint = model.material("body_paint", rgba=(0.20, 0.22, 0.24, 1.0))
    door_paint = model.material("door_paint", rgba=(0.17, 0.18, 0.20, 1.0))
    plinth_paint = model.material("plinth_paint", rgba=(0.34, 0.35, 0.37, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    slot_metal = model.material("slot_metal", rgba=(0.28, 0.29, 0.31, 1.0))

    wall = 0.012

    base_w = 0.34
    base_d = 0.30
    base_h = 0.055
    pedestal_w = 0.20
    pedestal_d = 0.18
    pedestal_h = 0.69
    cap_w = 0.26
    cap_d = 0.22
    cap_h = 0.03

    body_w = 0.44
    body_d = 0.34
    body_h = 0.56
    inner_w = body_w - (2.0 * wall)

    hood_shell_d = 0.22
    hood_shell_y = -0.03
    hood_h = 0.11
    hood_top_w = 0.46
    hood_top_d = 0.18
    hood_top_y = -0.01
    hood_side_h = hood_h - wall

    slot_w = 0.28
    slot_d = 0.055
    slot_center_y = -0.02
    chute_wall = 0.008

    door_w = 0.376
    door_h = 0.42
    door_t = 0.018
    door_hinge_z = 0.032
    front_frame_t = wall
    jamb_w = 0.032

    housing_mount_z = base_h + pedestal_h + cap_h

    plinth = model.part("plinth")
    plinth.visual(
        Box((base_w, base_d, base_h)),
        origin=Origin(xyz=(0.0, 0.0, base_h * 0.5)),
        material=plinth_paint,
        name="base_slab",
    )
    plinth.visual(
        Box((pedestal_w, pedestal_d, pedestal_h)),
        origin=Origin(xyz=(0.0, 0.0, base_h + (pedestal_h * 0.5))),
        material=plinth_paint,
        name="pedestal",
    )
    plinth.visual(
        Box((cap_w, cap_d, cap_h)),
        origin=Origin(xyz=(0.0, 0.0, base_h + pedestal_h + (cap_h * 0.5))),
        material=plinth_paint,
        name="top_cap",
    )

    housing = model.part("housing")
    housing.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(-(body_w * 0.5) + (wall * 0.5), 0.0, body_h * 0.5)),
        material=body_paint,
        name="left_wall",
    )
    housing.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=((body_w * 0.5) - (wall * 0.5), 0.0, body_h * 0.5)),
        material=body_paint,
        name="right_wall",
    )
    housing.visual(
        Box((inner_w, wall, body_h)),
        origin=Origin(xyz=(0.0, (body_d * 0.5) - (wall * 0.5), body_h * 0.5)),
        material=body_paint,
        name="back_wall",
    )
    housing.visual(
        Box((inner_w, body_d - wall, wall)),
        origin=Origin(xyz=(0.0, wall * 0.5, wall * 0.5)),
        material=body_paint,
        name="floor_panel",
    )

    top_deck_mesh = _plate_with_rect_opening(
        name="mailbox_top_deck",
        width=inner_w,
        depth=body_d - wall,
        thickness=wall,
        opening_width=slot_w,
        opening_depth=slot_d,
        opening_center_y=slot_center_y,
    )
    housing.visual(
        top_deck_mesh,
        origin=Origin(xyz=(0.0, wall * 0.5, body_h - (wall * 0.5))),
        material=body_paint,
        name="top_deck",
    )

    opening_center_z = door_hinge_z + (door_h * 0.5)
    upper_fascia_h = body_h - (door_hinge_z + door_h)
    housing.visual(
        Box((inner_w, front_frame_t, door_hinge_z)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_d * 0.5) + (front_frame_t * 0.5),
                door_hinge_z * 0.5,
            )
        ),
        material=body_paint,
        name="hinge_rail",
    )
    housing.visual(
        Box((jamb_w, front_frame_t, door_h)),
        origin=Origin(
            xyz=(
                -((body_w * 0.5) - wall - (jamb_w * 0.5)),
                -(body_d * 0.5) + (front_frame_t * 0.5),
                opening_center_z,
            )
        ),
        material=body_paint,
        name="left_jamb",
    )
    housing.visual(
        Box((jamb_w, front_frame_t, door_h)),
        origin=Origin(
            xyz=(
                (body_w * 0.5) - wall - (jamb_w * 0.5),
                -(body_d * 0.5) + (front_frame_t * 0.5),
                opening_center_z,
            )
        ),
        material=body_paint,
        name="right_jamb",
    )
    housing.visual(
        Box((inner_w, front_frame_t, upper_fascia_h)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_d * 0.5) + (front_frame_t * 0.5),
                (door_hinge_z + door_h) + (upper_fascia_h * 0.5),
            )
        ),
        material=body_paint,
        name="upper_fascia",
    )

    housing.visual(
        Box((wall, hood_shell_d, hood_side_h)),
        origin=Origin(
            xyz=(
                -(body_w * 0.5) + (wall * 0.5),
                hood_shell_y,
                body_h + (hood_side_h * 0.5),
            )
        ),
        material=body_paint,
        name="hood_left_side",
    )
    housing.visual(
        Box((wall, hood_shell_d, hood_side_h)),
        origin=Origin(
            xyz=(
                (body_w * 0.5) - (wall * 0.5),
                hood_shell_y,
                body_h + (hood_side_h * 0.5),
            )
        ),
        material=body_paint,
        name="hood_right_side",
    )
    housing.visual(
        Box((inner_w, wall, hood_side_h)),
        origin=Origin(
            xyz=(
                0.0,
                hood_shell_y + (hood_shell_d * 0.5) - (wall * 0.5),
                body_h + (hood_side_h * 0.5),
            )
        ),
        material=body_paint,
        name="hood_back_wall",
    )

    hood_top_mesh = _plate_with_rect_opening(
        name="mailbox_hood_top",
        width=hood_top_w,
        depth=hood_top_d,
        thickness=wall,
        opening_width=slot_w,
        opening_depth=slot_d,
        opening_center_y=slot_center_y - hood_top_y,
    )
    housing.visual(
        hood_top_mesh,
        origin=Origin(
            xyz=(
                0.0,
                hood_top_y,
                body_h + hood_h - (wall * 0.5),
            )
        ),
        material=body_paint,
        name="hood_top_plate",
    )

    visor_length = math.sqrt((0.07 * 0.07) + (0.11 * 0.11))
    visor_angle = math.atan2(0.07, 0.11)
    housing.visual(
        Box((body_w - (2.0 * wall), wall, visor_length)),
        origin=Origin(
            xyz=(0.0, -0.135, 0.615),
            rpy=(visor_angle, 0.0, 0.0),
        ),
        material=body_paint,
        name="hood_front_visor",
    )

    chute_h = hood_h - wall
    chute_center_z = body_h + (chute_h * 0.5)
    housing.visual(
        Box((chute_wall, slot_d, chute_h)),
        origin=Origin(
            xyz=(
                -((slot_w * 0.5) - (chute_wall * 0.5)),
                slot_center_y,
                chute_center_z,
            )
        ),
        material=dark_trim,
        name="slot_left_wall",
    )
    housing.visual(
        Box((chute_wall, slot_d, chute_h)),
        origin=Origin(
            xyz=(
                (slot_w * 0.5) - (chute_wall * 0.5),
                slot_center_y,
                chute_center_z,
            )
        ),
        material=dark_trim,
        name="slot_right_wall",
    )
    housing.visual(
        Box((slot_w, chute_wall, chute_h)),
        origin=Origin(
            xyz=(
                0.0,
                slot_center_y - (slot_d * 0.5) + (chute_wall * 0.5),
                chute_center_z,
            )
        ),
        material=dark_trim,
        name="slot_front_wall",
    )
    housing.visual(
        Box((slot_w, chute_wall, chute_h)),
        origin=Origin(
            xyz=(
                0.0,
                slot_center_y + (slot_d * 0.5) - (chute_wall * 0.5),
                chute_center_z,
            )
        ),
        material=dark_trim,
        name="slot_rear_wall",
    )

    model.articulation(
        "plinth_to_housing",
        ArticulationType.FIXED,
        parent=plinth,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, housing_mount_z)),
    )

    door = model.part("retrieval_door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(0.0, -(door_t * 0.5), door_h * 0.5)),
        material=door_paint,
        name="door_panel",
    )
    door.visual(
        Box((door_w * 0.78, 0.008, door_h * 0.70)),
        origin=Origin(
            xyz=(
                0.0,
                -(door_t + 0.004),
                door_h * 0.48,
            )
        ),
        material=slot_metal,
        name="door_stiffener",
    )
    door.visual(
        Box((0.09, 0.022, 0.030)),
        origin=Origin(
            xyz=(
                0.0,
                -(door_t + 0.011),
                door_h * 0.72,
            )
        ),
        material=dark_trim,
        name="door_pull",
    )

    model.articulation(
        "front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, -(body_d * 0.5), door_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    flap = model.part("slot_flap")
    flap_w = slot_w - 0.004
    flap_l = slot_d - 0.004
    flap_t = 0.006
    flap.visual(
        Box((flap_w, flap_l, flap_t)),
        origin=Origin(xyz=(0.0, -(flap_l * 0.5), flap_t * 0.5)),
        material=slot_metal,
        name="flap_panel",
    )
    flap.visual(
        Box((flap_w * 0.52, 0.010, 0.016)),
        origin=Origin(
            xyz=(
                0.0,
                -(flap_l - 0.005),
                0.008,
            )
        ),
        material=dark_trim,
        name="flap_pull_lip",
    )

    model.articulation(
        "slot_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(
            xyz=(
                0.0,
                slot_center_y + (slot_d * 0.5) - 0.002,
                body_h + hood_h - wall,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        (lower[0] + upper[0]) * 0.5,
        (lower[1] + upper[1]) * 0.5,
        (lower[2] + upper[2]) * 0.5,
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    plinth = object_model.get_part("plinth")
    housing = object_model.get_part("housing")
    door = object_model.get_part("retrieval_door")
    flap = object_model.get_part("slot_flap")

    plinth_to_housing = object_model.get_articulation("plinth_to_housing")
    door_hinge = object_model.get_articulation("front_door_hinge")
    flap_hinge = object_model.get_articulation("slot_flap_hinge")

    ctx.check(
        "part_count",
        len(object_model.parts) == 4,
        f"Expected 4 parts, found {len(object_model.parts)}.",
    )
    ctx.check(
        "articulation_count",
        len(object_model.articulations) == 3,
        f"Expected 3 articulations, found {len(object_model.articulations)}.",
    )
    ctx.check(
        "door_hinge_axis",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0),
        f"Door hinge axis should run along +X, got {door_hinge.axis}.",
    )
    ctx.check(
        "slot_flap_hinge_axis",
        tuple(flap_hinge.axis) == (-1.0, 0.0, 0.0),
        f"Slot flap hinge axis should run along rear edge (-X rotation convention), got {flap_hinge.axis}.",
    )
    ctx.check(
        "housing_mounted_to_plinth",
        plinth_to_housing.articulation_type == ArticulationType.FIXED,
        "Housing should be rigidly mounted to the plinth.",
    )

    ctx.expect_gap(housing, plinth, axis="z", min_gap=0.0, max_gap=0.0015)
    ctx.expect_overlap(housing, plinth, axes="xy", min_overlap=0.18)

    with ctx.pose({door_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_contact(door, housing, elem_a="door_panel", elem_b="hinge_rail")
        ctx.expect_overlap(door, housing, axes="xz", min_overlap=0.24)
        ctx.expect_within(flap, housing, axes="xy", margin=0.10)
        ctx.expect_overlap(flap, housing, axes="x", min_overlap=0.24)

    closed_door_pull = _aabb_center(ctx.part_element_world_aabb(door, elem="door_pull"))
    with ctx.pose({door_hinge: 1.0}):
        open_door_pull = _aabb_center(ctx.part_element_world_aabb(door, elem="door_pull"))
        ctx.check(
            "door_opens_downward",
            closed_door_pull is not None
            and open_door_pull is not None
            and open_door_pull[2] < closed_door_pull[2] - 0.10
            and open_door_pull[1] < closed_door_pull[1] - 0.05,
            "The lower front retrieval door should swing outward and downward about its bottom edge.",
        )

    closed_flap_pull = _aabb_center(ctx.part_element_world_aabb(flap, elem="flap_pull_lip"))
    with ctx.pose({flap_hinge: 0.85}):
        open_flap_pull = _aabb_center(ctx.part_element_world_aabb(flap, elem="flap_pull_lip"))
        ctx.check(
            "slot_flap_lifts_from_front",
            closed_flap_pull is not None
            and open_flap_pull is not None
            and open_flap_pull[2] > closed_flap_pull[2] + 0.03,
            "The top slot flap should rotate upward from its rear edge hinge.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
