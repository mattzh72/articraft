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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _yz_prism_mesh(
    *,
    name: str,
    x0: float,
    x1: float,
    yz_loop: list[tuple[float, float]],
):
    return mesh_from_geometry(
        section_loft(
            [
                [(x0, y, z) for y, z in yz_loop],
                [(x1, y, z) for y, z in yz_loop],
            ]
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teachers_podium_desk")

    wood = model.material("wood", rgba=(0.48, 0.34, 0.20, 1.0))
    laminate = model.material("laminate", rgba=(0.60, 0.47, 0.31, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.68, 0.70, 0.74, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.12, 0.12, 0.13, 1.0))

    width = 0.82
    depth = 0.58
    body_height = 1.08
    side_t = 0.022
    back_t = 0.018
    bottom_t = 0.022
    shelf_t = 0.020
    top_t = 0.038
    face_tilt = math.radians(8.0)
    face_angle = -face_tilt

    def front_face_y(z: float) -> float:
        return -0.275 + math.tan(face_tilt) * z

    inner_width = width - 2.0 * side_t

    podium = model.part("podium_body")

    left_side_loop = [
        (front_face_y(0.0), 0.0),
        (front_face_y(body_height), body_height),
        (depth * 0.5, body_height),
        (depth * 0.5, 0.0),
    ]
    podium.visual(
        _yz_prism_mesh(
            name="podium_left_side",
            x0=-width * 0.5,
            x1=-width * 0.5 + side_t,
            yz_loop=left_side_loop,
        ),
        material=wood,
        name="left_side_panel",
    )
    podium.visual(
        _yz_prism_mesh(
            name="podium_right_side",
            x0=width * 0.5 - side_t,
            x1=width * 0.5,
            yz_loop=left_side_loop,
        ),
        material=wood,
        name="right_side_panel",
    )

    bottom_front = front_face_y(0.0) + 0.020
    bottom_back = depth * 0.5 - back_t + 0.001
    bottom_depth = bottom_back - bottom_front
    podium.visual(
        Box((inner_width + 0.002, bottom_depth, bottom_t)),
        origin=Origin(
            xyz=(
                0.0,
                (bottom_front + bottom_back) * 0.5,
                bottom_t * 0.5,
            )
        ),
        material=wood,
        name="bottom_panel",
    )

    podium.visual(
        Box((inner_width + 0.004, back_t, body_height)),
        origin=Origin(
            xyz=(
                0.0,
                depth * 0.5 - back_t * 0.5,
                body_height * 0.5,
            )
        ),
        material=wood,
        name="back_panel",
    )

    shelf_bottom_z = 0.790
    shelf_top_z = shelf_bottom_z + shelf_t
    shelf_front = front_face_y(0.82) + 0.012
    shelf_back = depth * 0.5 - back_t + 0.001
    podium.visual(
        Box((inner_width + 0.002, shelf_back - shelf_front, shelf_t)),
        origin=Origin(
            xyz=(
                0.0,
                (shelf_front + shelf_back) * 0.5,
                shelf_bottom_z + shelf_t * 0.5,
            )
        ),
        material=wood,
        name="drawer_shelf",
    )

    top_front = front_face_y(body_height) - 0.045
    top_back = depth * 0.5 + 0.025
    podium.visual(
        Box((width + 0.040, top_back - top_front, top_t)),
        origin=Origin(
            xyz=(
                0.0,
                (top_front + top_back) * 0.5,
                body_height + top_t * 0.5,
            )
        ),
        material=laminate,
        name="top_deck",
    )

    podium.visual(
        Box((width + 0.010, 0.028, 0.035)),
        origin=Origin(
            xyz=(
                0.0,
                top_back - 0.014,
                body_height + top_t + 0.0175,
            )
        ),
        material=wood,
        name="top_back_lip",
    )

    rail_width = 0.036
    rail_depth = 0.360
    rail_height = 0.012
    rail_center_y = 0.060
    rail_z = shelf_top_z + rail_height * 0.5
    rail_x = 0.250
    podium.visual(
        Box((rail_width, rail_depth, rail_height)),
        origin=Origin(xyz=(-rail_x, rail_center_y, rail_z)),
        material=rail_metal,
        name="left_guide_rail",
    )
    podium.visual(
        Box((rail_width, rail_depth, rail_height)),
        origin=Origin(xyz=(rail_x, rail_center_y, rail_z)),
        material=rail_metal,
        name="right_guide_rail",
    )

    podium.inertial = Inertial.from_geometry(
        Box((width + 0.040, depth + 0.050, body_height + top_t + 0.050)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, (body_height + top_t) * 0.5)),
    )

    drawer = model.part("drawer")
    drawer_box_width = 0.720
    drawer_box_depth = 0.400
    drawer_box_height = 0.100
    drawer_bottom_t = 0.012
    drawer_side_t = 0.012
    drawer_front_width = inner_width - 0.020
    drawer_front_height = 0.130
    drawer_bottom_z = rail_z + rail_height * 0.5
    drawer_center_z = drawer_bottom_z + drawer_box_height * 0.5
    drawer_center_y = front_face_y(0.900) + drawer_box_depth * 0.5 + 0.010

    drawer.visual(
        Box((drawer_box_width, drawer_box_depth, drawer_bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, -drawer_box_height * 0.5 + drawer_bottom_t * 0.5)),
        material=laminate,
        name="drawer_bottom",
    )
    side_wall_height = drawer_box_height - drawer_bottom_t
    side_wall_z = -drawer_box_height * 0.5 + drawer_bottom_t + side_wall_height * 0.5
    drawer.visual(
        Box((drawer_side_t, drawer_box_depth - 0.008, side_wall_height)),
        origin=Origin(xyz=(-drawer_box_width * 0.5 + drawer_side_t * 0.5, 0.0, side_wall_z)),
        material=laminate,
        name="drawer_left_wall",
    )
    drawer.visual(
        Box((drawer_side_t, drawer_box_depth - 0.008, side_wall_height)),
        origin=Origin(xyz=(drawer_box_width * 0.5 - drawer_side_t * 0.5, 0.0, side_wall_z)),
        material=laminate,
        name="drawer_right_wall",
    )
    drawer.visual(
        Box((drawer_box_width - 2.0 * drawer_side_t, drawer_side_t, side_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                drawer_box_depth * 0.5 - drawer_side_t * 0.5,
                side_wall_z,
            )
        ),
        material=laminate,
        name="drawer_back",
    )
    drawer.visual(
        Box((drawer_front_width, 0.021, drawer_front_height)),
        origin=Origin(
            xyz=(
                0.0,
                -drawer_box_depth * 0.5 + 0.002,
                0.026,
            ),
            rpy=(face_angle, 0.0, 0.0),
        ),
        material=wood,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.180, 0.018, 0.024)),
        origin=Origin(
            xyz=(0.0, -drawer_box_depth * 0.5 - 0.016, 0.028),
            rpy=(face_angle, 0.0, 0.0),
        ),
        material=handle_dark,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_front_width, drawer_box_depth + 0.040, drawer_front_height)),
        mass=6.5,
        origin=Origin(xyz=(0.0, -0.010, 0.015)),
    )

    model.articulation(
        "podium_to_drawer",
        ArticulationType.PRISMATIC,
        parent=podium,
        child=drawer,
        origin=Origin(xyz=(0.0, drawer_center_y, drawer_center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.30,
            lower=0.0,
            upper=0.220,
        ),
    )

    front_door = model.part("front_access_door")
    door_width = inner_width - 0.004
    door_height = 0.710
    door_center_z = 0.050 + door_height * 0.5
    door_hinge_y = front_face_y(door_center_z)
    podium.visual(
        Box((0.016, 0.022, door_height + 0.020)),
        origin=Origin(
            xyz=(-inner_width * 0.5 - 0.008, door_hinge_y + 0.010, door_center_z),
            rpy=(face_angle, 0.0, 0.0),
        ),
        material=wood,
        name="left_door_stile",
    )
    podium.visual(
        Box((0.016, 0.022, door_height + 0.020)),
        origin=Origin(
            xyz=(inner_width * 0.5 + 0.008, door_hinge_y + 0.010, door_center_z),
            rpy=(face_angle, 0.0, 0.0),
        ),
        material=wood,
        name="right_door_stile",
    )
    front_door.visual(
        Box((door_width, 0.021, door_height)),
        origin=Origin(
            xyz=(door_width * 0.5, 0.0, 0.0),
            rpy=(face_angle, 0.0, 0.0),
        ),
        material=wood,
        name="door_panel",
    )
    front_door.visual(
        Box((0.032, 0.018, 0.160)),
        origin=Origin(
            xyz=(door_width - 0.055, -0.019, 0.0),
            rpy=(face_angle, 0.0, 0.0),
        ),
        material=handle_dark,
        name="door_pull",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.050, door_height)),
        mass=7.0,
        origin=Origin(xyz=(door_width * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "podium_to_front_door",
        ArticulationType.REVOLUTE,
        parent=podium,
        child=front_door,
        origin=Origin(
            xyz=(
                -inner_width * 0.5,
                door_hinge_y,
                door_center_z,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
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

    podium = object_model.get_part("podium_body")
    drawer = object_model.get_part("drawer")
    front_door = object_model.get_part("front_access_door")
    drawer_slide = object_model.get_articulation("podium_to_drawer")
    door_hinge = object_model.get_articulation("podium_to_front_door")

    ctx.expect_contact(
        drawer,
        podium,
        elem_a="drawer_bottom",
        elem_b="left_guide_rail",
        contact_tol=0.0015,
        name="drawer bottom rests on left guide rail",
    )
    ctx.expect_contact(
        drawer,
        podium,
        elem_a="drawer_bottom",
        elem_b="right_guide_rail",
        contact_tol=0.0015,
        name="drawer bottom rests on right guide rail",
    )
    ctx.expect_within(
        drawer,
        podium,
        axes="x",
        margin=0.002,
        name="drawer stays centered between podium sides at rest",
    )

    drawer_upper = 0.220
    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: drawer_upper}):
        ctx.expect_within(
            drawer,
            podium,
            axes="x",
            margin=0.002,
            name="drawer stays centered between podium sides when extended",
        )
        ctx.expect_overlap(
            drawer,
            podium,
            axes="y",
            min_overlap=0.140,
            name="drawer remains retained inside podium at full extension",
        )
        ctx.expect_contact(
            drawer,
            podium,
            elem_a="drawer_bottom",
            elem_b="left_guide_rail",
            contact_tol=0.0015,
            name="left rail still supports the open drawer",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends outward from the podium",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.18,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_panel")
    with ctx.pose({door_hinge: 1.10}):
        open_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_panel")

    ctx.check(
        "front door swings outward on its vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.12
        and open_door_aabb[1][0] < closed_door_aabb[1][0] - 0.28,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
