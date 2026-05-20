from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _extruded_yz_prism(width: float, profile: list[tuple[float, float]]) -> MeshGeometry:
    """Build a closed cabinet shell by extruding a side profile along X."""
    geom = MeshGeometry()
    left = [geom.add_vertex(-width / 2.0, y, z) for y, z in profile]
    right = [geom.add_vertex(width / 2.0, y, z) for y, z in profile]
    count = len(profile)

    for i in range(count):
        j = (i + 1) % count
        geom.add_face(left[i], right[i], right[j])
        geom.add_face(left[i], right[j], left[j])

    for i in range(1, count - 1):
        geom.add_face(left[0], left[i + 1], left[i])
        geom.add_face(right[0], right[i], right[i + 1])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_top_video_poker")

    model.material("cabinet_blue", rgba=(0.03, 0.045, 0.075, 1.0))
    model.material("matte_black", rgba=(0.005, 0.006, 0.008, 1.0))
    model.material("screen_glass", rgba=(0.03, 0.10, 0.16, 0.92))
    model.material("screen_glow", rgba=(0.12, 0.48, 0.82, 1.0))
    model.material("shelf_gray", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("button_ivory", rgba=(0.86, 0.82, 0.67, 1.0))
    model.material("button_label", rgba=(0.06, 0.05, 0.035, 1.0))
    model.material("cash_metal", rgba=(0.42, 0.42, 0.39, 1.0))
    model.material("brushed_steel", rgba=(0.65, 0.64, 0.58, 1.0))
    model.material("warning_red", rgba=(0.55, 0.04, 0.035, 1.0))

    cabinet = model.part("cabinet")

    cabinet_profile = [
        (-0.220, 0.000),
        (-0.220, 0.215),
        (-0.100, 0.235),
        (-0.060, 0.285),
        (0.020, 0.535),
        (0.160, 0.500),
        (0.180, 0.040),
        (0.140, 0.000),
    ]
    cabinet.visual(
        mesh_from_geometry(_extruded_yz_prism(0.540, cabinet_profile), "cabinet_body"),
        material="cabinet_blue",
        name="cabinet_body",
    )

    # A short, separate shelf just below the screen, kept visibly distinct from
    # the screen surround and from the lower cash-box door area.
    cabinet.visual(
        Box((0.505, 0.150, 0.030)),
        origin=Origin(xyz=(0.0, -0.160, 0.235)),
        material="shelf_gray",
        name="control_shelf",
    )
    cabinet.visual(
        Box((0.525, 0.020, 0.045)),
        origin=Origin(xyz=(0.0, -0.230, 0.232)),
        material="matte_black",
        name="shelf_front_lip",
    )

    screen_tilt = math.radians(65.7)
    screen_center = (0.0, -0.024, 0.407)
    screen_normal = (0.0, -math.sin(screen_tilt), math.cos(screen_tilt))
    screen_bezel = BezelGeometry(
        (0.330, 0.190),
        (0.420, 0.270),
        0.018,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.010,
        outer_corner_radius=0.020,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
    )
    cabinet.visual(
        mesh_from_geometry(screen_bezel, "screen_surround"),
        origin=Origin(xyz=screen_center, rpy=(screen_tilt, 0.0, 0.0)),
        material="matte_black",
        name="screen_surround",
    )
    cabinet.visual(
        Box((0.318, 0.178, 0.006)),
        origin=Origin(
            xyz=(
                screen_center[0] + screen_normal[0] * 0.007,
                screen_center[1] + screen_normal[1] * 0.007,
                screen_center[2] + screen_normal[2] * 0.007,
            ),
            rpy=(screen_tilt, 0.0, 0.0),
        ),
        material="screen_glass",
        name="screen_glass",
    )
    cabinet.visual(
        Box((0.250, 0.120, 0.007)),
        origin=Origin(
            xyz=(
                screen_center[0] + screen_normal[0] * 0.010,
                screen_center[1] + screen_normal[1] * 0.010,
                screen_center[2] + screen_normal[2] * 0.010,
            ),
            rpy=(screen_tilt, 0.0, 0.0),
        ),
        material="screen_glow",
        name="video_poker_display",
    )

    cabinet.visual(
        Box((0.385, 0.008, 0.015)),
        origin=Origin(xyz=(0.0, -0.223, 0.207)),
        material="matte_black",
        name="cash_top_jamb",
    )
    cabinet.visual(
        Box((0.385, 0.008, 0.015)),
        origin=Origin(xyz=(0.0, -0.223, 0.013)),
        material="matte_black",
        name="cash_bottom_jamb",
    )
    cabinet.visual(
        Box((0.015, 0.008, 0.205)),
        origin=Origin(xyz=(-0.185, -0.225, 0.110)),
        material="matte_black",
        name="cash_hinge_jamb",
    )
    cabinet.visual(
        Box((0.015, 0.008, 0.205)),
        origin=Origin(xyz=(0.185, -0.223, 0.110)),
        material="matte_black",
        name="cash_latch_jamb",
    )
    cash_door = model.part("cash_door")
    cash_door.visual(
        Box((0.340, 0.014, 0.175)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material="cash_metal",
        name="door_panel",
    )
    cash_door.visual(
        Box((0.300, 0.004, 0.125)),
        origin=Origin(xyz=(0.185, -0.0085, 0.0)),
        material="brushed_steel",
        name="recessed_plate",
    )
    cash_door.visual(
        Cylinder(radius=0.008, length=0.190),
        origin=Origin(xyz=(0.0, -0.001, 0.0)),
        material="brushed_steel",
        name="hinge_barrel",
    )
    cash_door.visual(
        Box((0.014, 0.008, 0.175)),
        origin=Origin(xyz=(-0.015, 0.003, 0.0)),
        material="brushed_steel",
        name="hinge_leaf",
    )
    cash_door.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.250, -0.013, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="key_lock",
    )
    model.articulation(
        "cash_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=cash_door,
        origin=Origin(xyz=(-0.170, -0.236, 0.110)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    shelf_top_z = 0.250
    button_xs = (-0.160, -0.080, 0.0, 0.080, 0.160)
    for index, x_pos in enumerate(button_xs):
        button = model.part(f"hold_button_{index}")
        button.visual(
            Box((0.064, 0.036, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material="button_ivory",
            name="cap",
        )
        button.visual(
            Box((0.044, 0.006, 0.0012)),
            origin=Origin(xyz=(0.0, -0.002, 0.0146)),
            material="button_label",
            name="label_bar",
        )
        model.articulation(
            f"hold_button_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, -0.160, shelf_top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.008),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    cash_door = object_model.get_part("cash_door")
    door_hinge = object_model.get_articulation("cash_door_hinge")

    ctx.expect_contact(
        cash_door,
        cabinet,
        elem_a="hinge_leaf",
        elem_b="cash_hinge_jamb",
        contact_tol=0.0015,
        name="cash door hinge leaf bears on the cabinet jamb",
    )
    ctx.expect_gap(
        cabinet,
        cash_door,
        axis="y",
        min_gap=0.001,
        positive_elem="cash_latch_jamb",
        negative_elem="door_panel",
        name="closed cash door sits proud of the front jamb",
    )

    closed_aabb = ctx.part_element_world_aabb(cash_door, elem="door_panel")
    with ctx.pose({door_hinge: 1.0}):
        open_aabb = ctx.part_element_world_aabb(cash_door, elem="door_panel")
    ctx.check(
        "cash door swings outward on vertical hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.08,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for index in range(5):
        button = object_model.get_part(f"hold_button_{index}")
        slide = object_model.get_articulation(f"hold_button_{index}_slide")
        ctx.expect_gap(
            button,
            cabinet,
            axis="z",
            positive_elem="cap",
            negative_elem="control_shelf",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"hold button {index} rests on the control shelf",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({slide: 0.008}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"hold button {index} pushes downward independently",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.006,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
