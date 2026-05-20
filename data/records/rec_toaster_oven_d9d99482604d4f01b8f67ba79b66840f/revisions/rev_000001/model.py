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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _front_fascia_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    opening_center_x: float,
    opening_center_z: float,
    opening_width: float,
    opening_height: float,
    knob_x: float,
    knob_zs: tuple[float, ...],
    knob_hole_diameter: float,
) -> object:
    outer_profile = rounded_rect_profile(width, height, 0.012, corner_segments=8)
    door_hole = _translate_profile(
        rounded_rect_profile(opening_width, opening_height, 0.008, corner_segments=8),
        dx=opening_center_x,
        dy=opening_center_z - height * 0.5,
    )
    hole_profiles = [door_hole]
    for knob_z in knob_zs:
        hole_profiles.append(
            _translate_profile(
                superellipse_profile(
                    knob_hole_diameter,
                    knob_hole_diameter,
                    exponent=2.0,
                    segments=24,
                ),
                dx=knob_x,
                dy=knob_z - height * 0.5,
            )
        )
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "toaster_oven_front_fascia")


def _door_frame_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    window_width: float,
    window_height: float,
) -> object:
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(width, height, 0.010, corner_segments=8),
        [rounded_rect_profile(window_width, window_height, 0.006, corner_segments=8)],
        height=thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "toaster_oven_door_frame")


def _add_control_knob(
    model: ArticulatedObject,
    body,
    *,
    knob_index: int,
    knob_x: float,
    knob_z: float,
    panel_y: float,
    knob_dark,
    trim_metal,
) -> None:
    knob = model.part(f"knob_{knob_index}")
    knob.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_metal,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_skirt",
    )
    knob.visual(
        Cylinder(radius=0.023, length=0.028),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, -0.034, 0.015)),
        material=trim_metal,
        name="knob_indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.055, 0.050, 0.055)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
    )

    model.articulation(
        f"body_to_knob_{knob_index}",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(knob_x, panel_y, knob_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-2.6,
            upper=2.6,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    stainless = model.material("stainless", rgba=(0.76, 0.77, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.12, 0.16, 0.18, 0.35))
    cavity = model.material("cavity", rgba=(0.13, 0.13, 0.14, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    foot_dark = model.material("foot_dark", rgba=(0.07, 0.07, 0.08, 1.0))

    body_width = 0.50
    body_depth = 0.36
    body_height = 0.31
    shell_thickness = 0.018
    floor_thickness = 0.022
    fascia_thickness = 0.012
    control_panel_width = 0.150
    opening_width = 0.320
    opening_height = 0.214
    opening_center_x = -0.060
    opening_center_z = 0.161
    knob_x = 0.182
    knob_zs = (0.240, 0.175, 0.110)

    front_y = -body_depth * 0.5 + fascia_thickness * 0.5
    shell_center_y = fascia_thickness * 0.5
    shell_depth = body_depth - fascia_thickness

    body = model.part("body")
    body.visual(
        _front_fascia_mesh(
            width=body_width,
            height=body_height,
            thickness=fascia_thickness,
            opening_center_x=opening_center_x,
            opening_center_z=opening_center_z,
            opening_width=opening_width,
            opening_height=opening_height,
            knob_x=knob_x,
            knob_zs=knob_zs,
            knob_hole_diameter=0.016,
        ),
        origin=Origin(xyz=(0.0, front_y, body_height * 0.5)),
        material=stainless,
        name="front_fascia",
    )
    body.visual(
        Box((body_width, shell_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, shell_center_y, body_height - shell_thickness * 0.5)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((body_width, shell_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, shell_center_y, floor_thickness * 0.5)),
        material=stainless,
        name="bottom_shell",
    )
    body.visual(
        Box((shell_thickness, shell_depth, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                -body_width * 0.5 + shell_thickness * 0.5,
                shell_center_y,
                floor_thickness + (body_height - floor_thickness) * 0.5,
            )
        ),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((shell_thickness, shell_depth, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                body_width * 0.5 - shell_thickness * 0.5,
                shell_center_y,
                floor_thickness + (body_height - floor_thickness) * 0.5,
            )
        ),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((body_width - 2.0 * shell_thickness, shell_thickness, body_height - floor_thickness - shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth * 0.5 - shell_thickness * 0.5,
                floor_thickness + (body_height - floor_thickness - shell_thickness) * 0.5,
            )
        ),
        material=stainless,
        name="rear_panel",
    )
    inner_width = body_width - 2.0 * shell_thickness
    inner_depth = body_depth - fascia_thickness - shell_thickness
    inner_height = body_height - floor_thickness - shell_thickness
    inner_center_y = shell_thickness * 0.5
    body.visual(
        Box((inner_width, inner_depth, 0.002)),
        origin=Origin(xyz=(0.0, inner_center_y, floor_thickness + 0.001)),
        material=cavity,
        name="cavity_floor_liner",
    )
    body.visual(
        Box((inner_width, inner_depth, 0.002)),
        origin=Origin(xyz=(0.0, inner_center_y, body_height - shell_thickness - 0.001)),
        material=cavity,
        name="cavity_ceiling_liner",
    )
    body.visual(
        Box((0.002, inner_depth, inner_height)),
        origin=Origin(
            xyz=(
                -body_width * 0.5 + shell_thickness + 0.001,
                inner_center_y,
                floor_thickness + inner_height * 0.5,
            )
        ),
        material=cavity,
        name="cavity_left_liner",
    )
    body.visual(
        Box((0.002, inner_depth, inner_height)),
        origin=Origin(
            xyz=(
                body_width * 0.5 - shell_thickness - 0.001,
                inner_center_y,
                floor_thickness + inner_height * 0.5,
            )
        ),
        material=cavity,
        name="cavity_right_liner",
    )
    body.visual(
        Box((inner_width, 0.002, inner_height)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth * 0.5 - shell_thickness - 0.001,
                floor_thickness + inner_height * 0.5,
            )
        ),
        material=cavity,
        name="cavity_back_liner",
    )
    for foot_index, foot_x in enumerate((-0.170, 0.170)):
        for foot_row, foot_y in enumerate((-0.115, 0.125)):
            body.visual(
                Box((0.050, 0.022, 0.010)),
                origin=Origin(xyz=(foot_x, foot_y, -0.005)),
                material=foot_dark,
                name=f"foot_{foot_index}_{foot_row}",
            )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    door_width = 0.326
    door_height = 0.228
    door_thickness = 0.016
    door_window_width = 0.248
    door_window_height = 0.146
    door_hinge_z = 0.047

    door = model.part("door")
    door.visual(
        _door_frame_mesh(
            width=door_width,
            height=door_height,
            thickness=door_thickness,
            window_width=door_window_width,
            window_height=door_window_height,
        ),
        origin=Origin(xyz=(0.0, 0.0, door_height * 0.5)),
        material=stainless,
        name="door_frame",
    )
    door.visual(
        Box((door_window_width + 0.024, 0.004, door_window_height + 0.020)),
        origin=Origin(xyz=(0.0, 0.002, door_height * 0.54)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Box((door_width - 0.018, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.004, 0.010)),
        material=dark_trim,
        name="door_inner_rail",
    )
    for post_side, post_x in enumerate((-0.105, 0.105)):
        door.visual(
            Box((0.012, 0.020, 0.018)),
            origin=Origin(xyz=(post_x, -(door_thickness * 0.5 + 0.010), door_height * 0.63)),
            material=handle_dark,
            name=f"handle_post_{post_side}",
        )
    door.visual(
        Cylinder(radius=0.008, length=0.220),
        origin=Origin(
            xyz=(0.0, -(door_thickness * 0.5 + 0.020), door_height * 0.63),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_dark,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.048, door_height)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.016, door_height * 0.5)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(
            xyz=(
                opening_center_x,
                -body_depth * 0.5 - door_thickness * 0.5,
                door_hinge_z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    for knob_index, knob_z in enumerate(knob_zs):
        _add_control_knob(
            model,
            body,
            knob_index=knob_index,
            knob_x=knob_x,
            knob_z=knob_z,
            panel_y=front_y,
            knob_dark=knob_dark,
            trim_metal=stainless,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")
    top_knob = object_model.get_part("knob_0")
    top_knob_joint = object_model.get_articulation("body_to_knob_0")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="front_fascia",
            negative_elem="door_frame",
            min_gap=0.0,
            max_gap=0.002,
            name="closed door sits flush against the front fascia",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="xz",
            elem_a="front_fascia",
            elem_b="door_frame",
            min_overlap=0.20,
            name="closed door covers the oven opening region",
        )
        for knob_index in range(3):
            ctx.expect_gap(
                body,
                object_model.get_part(f"knob_{knob_index}"),
                axis="y",
                positive_elem="front_fascia",
                negative_elem="knob_body",
                max_gap=0.003,
                max_penetration=1e-5,
                name=f"knob {knob_index + 1} seats against the control panel",
            )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    with ctx.pose({door_hinge: math.radians(85.0)}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_frame")

    door_opens_clear = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.14
        and open_aabb[1][2] < closed_aabb[1][2] - 0.12
    )
    ctx.check(
        "door opens downward and outward",
        door_opens_clear,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_indicator = ctx.part_element_world_aabb(top_knob, elem="knob_indicator")
    with ctx.pose({top_knob_joint: 1.2}):
        turned_indicator = ctx.part_element_world_aabb(top_knob, elem="knob_indicator")
    rest_x = None if rest_indicator is None else (rest_indicator[0][0] + rest_indicator[1][0]) * 0.5
    turned_x = None if turned_indicator is None else (turned_indicator[0][0] + turned_indicator[1][0]) * 0.5
    ctx.check(
        "top knob indicator rotates around its shaft",
        rest_x is not None and turned_x is not None and turned_x > rest_x + 0.010,
        details=f"rest={rest_indicator}, turned={turned_indicator}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
