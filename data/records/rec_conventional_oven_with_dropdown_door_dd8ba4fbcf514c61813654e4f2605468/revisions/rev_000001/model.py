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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotisserie_oven")

    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.16, 0.18, 1.0))
    glass = model.material("glass", rgba=(0.55, 0.72, 0.82, 0.28))
    handle_metal = model.material("handle_metal", rgba=(0.82, 0.83, 0.84, 1.0))

    width = 0.66
    depth = 0.46
    height = 0.44
    side_t = 0.025
    back_t = 0.025
    top_t = 0.035
    bottom_t = 0.055

    door_width = 0.58
    door_height = 0.34
    door_thickness = 0.028
    door_rail = 0.04
    door_side = 0.04
    door_y = door_thickness / 2.0
    hinge_z = 0.05

    spit_y = -0.02
    spit_z = 0.23
    socket_x = width / 2.0 - side_t - 0.012
    spit_length = 2.0 * socket_x

    housing = model.part("housing")
    housing.visual(
        Box((width, depth, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=stainless,
        name="bottom_shell",
    )
    housing.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, height - top_t / 2.0)),
        material=stainless,
        name="top_shell",
    )
    side_height = height - top_t - bottom_t
    side_z = bottom_t + side_height / 2.0
    housing.visual(
        Box((side_t, depth, side_height)),
        origin=Origin(xyz=(-(width / 2.0) + side_t / 2.0, 0.0, side_z)),
        material=stainless,
        name="left_shell",
    )
    housing.visual(
        Box((side_t, depth, side_height)),
        origin=Origin(xyz=((width / 2.0) - side_t / 2.0, 0.0, side_z)),
        material=stainless,
        name="right_shell",
    )
    housing.visual(
        Box((width - (2.0 * side_t), back_t, side_height)),
        origin=Origin(xyz=(0.0, -(depth / 2.0) + back_t / 2.0, side_z)),
        material=stainless,
        name="back_shell",
    )

    socket_plate_size = (0.012, 0.05, 0.07)
    socket_lip_size = (0.018, 0.05, 0.008)
    lip_z_offset = 0.011
    socket_plate_x = -socket_x - 0.006
    socket_lip_x = -socket_x + 0.003
    housing.visual(
        Box(socket_plate_size),
        origin=Origin(xyz=(socket_plate_x, spit_y, spit_z)),
        material=dark_trim,
        name="left_socket_plate",
    )
    housing.visual(
        Box(socket_lip_size),
        origin=Origin(xyz=(socket_lip_x, spit_y, spit_z + lip_z_offset)),
        material=dark_trim,
        name="left_socket_top",
    )
    housing.visual(
        Box(socket_lip_size),
        origin=Origin(xyz=(socket_lip_x, spit_y, spit_z - lip_z_offset)),
        material=dark_trim,
        name="left_socket_bottom",
    )
    housing.visual(
        Box(socket_plate_size),
        origin=Origin(xyz=(-socket_plate_x, spit_y, spit_z)),
        material=dark_trim,
        name="right_socket_plate",
    )
    housing.visual(
        Box(socket_lip_size),
        origin=Origin(xyz=(-socket_lip_x, spit_y, spit_z + lip_z_offset)),
        material=dark_trim,
        name="right_socket_top",
    )
    housing.visual(
        Box(socket_lip_size),
        origin=Origin(xyz=(-socket_lip_x, spit_y, spit_z - lip_z_offset)),
        material=dark_trim,
        name="right_socket_bottom",
    )

    door = model.part("door")
    door.visual(
        Box((door_width, door_thickness, door_rail)),
        origin=Origin(xyz=(0.0, door_y, door_rail / 2.0)),
        material=dark_trim,
        name="bottom_frame",
    )
    door.visual(
        Box((door_width, door_thickness, door_rail)),
        origin=Origin(xyz=(0.0, door_y, door_height - (door_rail / 2.0))),
        material=dark_trim,
        name="top_frame",
    )
    door.visual(
        Box((door_side, door_thickness, door_height)),
        origin=Origin(
            xyz=(-(door_width / 2.0) + (door_side / 2.0), door_y, door_height / 2.0)
        ),
        material=dark_trim,
        name="left_frame",
    )
    door.visual(
        Box((door_side, door_thickness, door_height)),
        origin=Origin(
            xyz=((door_width / 2.0) - (door_side / 2.0), door_y, door_height / 2.0)
        ),
        material=dark_trim,
        name="right_frame",
    )
    door.visual(
        Box((door_width - 0.072, 0.006, door_height - 0.072)),
        origin=Origin(xyz=(0.0, 0.009, door_height / 2.0)),
        material=glass,
        name="glass_panel",
    )
    handle_post_y = door_thickness + 0.016
    handle_z = 0.24
    post_spacing = 0.24
    door.visual(
        Box((0.012, 0.032, 0.012)),
        origin=Origin(xyz=(-post_spacing / 2.0, handle_post_y, handle_z)),
        material=handle_metal,
        name="left_handle_post",
    )
    door.visual(
        Box((0.012, 0.032, 0.012)),
        origin=Origin(xyz=(post_spacing / 2.0, handle_post_y, handle_z)),
        material=handle_metal,
        name="right_handle_post",
    )
    door.visual(
        Box((0.032, 0.032, 0.028)),
        origin=Origin(xyz=(-post_spacing / 2.0, 0.028, handle_z)),
        material=handle_metal,
        name="left_handle_bracket",
    )
    door.visual(
        Box((0.032, 0.032, 0.028)),
        origin=Origin(xyz=(post_spacing / 2.0, 0.028, handle_z)),
        material=handle_metal,
        name="right_handle_bracket",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.30),
        origin=Origin(
            xyz=(0.0, door_thickness + 0.03, handle_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_metal,
        name="handle_bar",
    )

    spit = model.part("spit")
    spit.visual(
        Box((spit_length, 0.008, 0.006)),
        origin=Origin(xyz=(spit_length / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="spit_body",
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, depth / 2.0, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "housing_to_spit",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=spit,
        origin=Origin(xyz=(-socket_x, spit_y, spit_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    spit = object_model.get_part("spit")
    door_hinge = object_model.get_articulation("housing_to_door")
    spit_spin = object_model.get_articulation("housing_to_spit")

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

    ctx.expect_gap(
        door,
        housing,
        axis="y",
        min_gap=0.0,
        max_gap=0.0005,
        name="closed door sits flush against the front housing face",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        min_overlap=0.22,
        name="closed door covers the oven opening footprint",
    )

    ctx.expect_gap(
        housing,
        spit,
        axis="z",
        positive_elem="left_socket_top",
        negative_elem="spit_body",
        min_gap=0.001,
        max_gap=0.01,
        name="left upper socket clears the spit",
    )
    ctx.expect_gap(
        spit,
        housing,
        axis="z",
        positive_elem="spit_body",
        negative_elem="left_socket_bottom",
        min_gap=0.001,
        max_gap=0.01,
        name="left lower socket clears the spit",
    )
    ctx.expect_gap(
        housing,
        spit,
        axis="z",
        positive_elem="right_socket_top",
        negative_elem="spit_body",
        min_gap=0.001,
        max_gap=0.01,
        name="right upper socket clears the spit",
    )
    ctx.expect_gap(
        spit,
        housing,
        axis="z",
        positive_elem="spit_body",
        negative_elem="right_socket_bottom",
        min_gap=0.001,
        max_gap=0.01,
        name="right lower socket clears the spit",
    )

    closed_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    with ctx.pose({door_hinge: 1.2}):
        open_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    if closed_handle is not None and open_handle is not None:
        closed_handle_center = tuple(
            (closed_handle[0][i] + closed_handle[1][i]) / 2.0 for i in range(3)
        )
        open_handle_center = tuple(
            (open_handle[0][i] + open_handle[1][i]) / 2.0 for i in range(3)
        )
        ctx.check(
            "door opens downward and outward",
            open_handle_center[1] > closed_handle_center[1] + 0.12
            and open_handle_center[2] < closed_handle_center[2] - 0.10,
            details=f"closed={closed_handle_center}, open={open_handle_center}",
        )
    else:
        ctx.fail("door opens downward and outward", "handle bar AABB unavailable")

    rest_spit = ctx.part_element_world_aabb(spit, elem="spit_body")
    with ctx.pose({spit_spin: math.pi / 2.0}):
        turned_spit = ctx.part_element_world_aabb(spit, elem="spit_body")
    if rest_spit is not None and turned_spit is not None:
        rest_y = rest_spit[1][1] - rest_spit[0][1]
        rest_z = rest_spit[1][2] - rest_spit[0][2]
        turned_y = turned_spit[1][1] - turned_spit[0][1]
        turned_z = turned_spit[1][2] - turned_spit[0][2]
        ctx.check(
            "spit rotates about its horizontal axle",
            rest_y > rest_z + 0.001 and turned_z > turned_y + 0.001,
            details=(
                f"rest_y={rest_y:.4f}, rest_z={rest_z:.4f}, "
                f"turned_y={turned_y:.4f}, turned_z={turned_z:.4f}"
            ),
        )
    else:
        ctx.fail("spit rotates about its horizontal axle", "spit AABB unavailable")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
