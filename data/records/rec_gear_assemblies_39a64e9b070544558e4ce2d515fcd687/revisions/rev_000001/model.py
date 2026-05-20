from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GEAR_MODULE = 0.00248
GEAR_WIDTH = 0.012
SHAFT_RADIUS = 0.006
SHAFT_DIAMETER = SHAFT_RADIUS * 2.0
BEARING_BORE_RADIUS = 0.0066
BEARING_BOSS_RADIUS = 0.015
COLLAR_RADIUS = 0.010
COLLAR_THICKNESS = 0.003

INPUT_TEETH = 16
IDLER_TEETH = 28
OUTPUT_TEETH = 48

INPUT_Y = -0.095
IDLER_Y = -0.040
OUTPUT_Y = 0.055
SHAFT_Z = 0.060

INPUT_GEAR_OUTER_RADIUS = 0.0205
INPUT_GEAR_ROOT_RADIUS = 0.0178
IDLER_GEAR_OUTER_RADIUS = 0.0340
IDLER_GEAR_ROOT_RADIUS = 0.0302
OUTPUT_GEAR_OUTER_RADIUS = 0.0600
OUTPUT_GEAR_ROOT_RADIUS = 0.0540

PLATE_THICKNESS = 0.008
PLATE_CENTER_X = 0.022
PLATE_BOSS_THICKNESS = 0.003
INNER_FACE_X = PLATE_CENTER_X - (PLATE_THICKNESS / 2.0)
OUTER_FACE_X = PLATE_CENTER_X + (PLATE_THICKNESS / 2.0)

FRAME_WIDTH = 0.290
FRAME_HEIGHT = 0.205
FRAME_Z_CENTER = 0.050
WINDOW_WIDTH = 0.236
WINDOW_HEIGHT = 0.138
WINDOW_Z_CENTER = 0.060


def _box_at(size_x: float, size_y: float, size_z: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate((x, y, z))


def _x_cylinder(radius: float, length: float, *, center_x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return (
        cq.Workplane("YZ", origin=(center_x - (length / 2.0), y, z))
        .circle(radius)
        .extrude(length)
    )


def _x_annulus(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    center_x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
):
    return _x_cylinder(outer_radius, length, center_x=center_x, y=y, z=z).cut(
        _x_cylinder(inner_radius, length + 0.002, center_x=center_x, y=y, z=z)
    )


def _shaft_body(x_min: float, x_max: float, *, left_collar: bool = True, right_collar: bool = True):
    body = _x_cylinder(
        SHAFT_RADIUS,
        x_max - x_min,
        center_x=(x_min + x_max) / 2.0,
    )
    if left_collar:
        body = body.union(
            _x_cylinder(
                COLLAR_RADIUS,
                COLLAR_THICKNESS,
                center_x=-(OUTER_FACE_X + PLATE_BOSS_THICKNESS + (COLLAR_THICKNESS / 2.0)),
            )
        )
    if right_collar:
        body = body.union(
            _x_cylinder(
                COLLAR_RADIUS,
                COLLAR_THICKNESS,
                center_x=OUTER_FACE_X + PLATE_BOSS_THICKNESS + (COLLAR_THICKNESS / 2.0),
            )
        )
    return body


def _polar_point(radius: float, angle_rad: float):
    return (radius * math.cos(angle_rad), radius * math.sin(angle_rad))


def _gear_shape(
    teeth: int,
    *,
    outer_radius: float,
    root_radius: float,
    hole_count: int = 0,
    hole_radius: float = 0.0,
    hole_center_radius: float = 0.0,
    hub_radius: float = 0.0,
    hub_width: float = 0.0,
):
    tooth_pitch = (2.0 * math.pi) / teeth
    rise_angle = tooth_pitch * 0.18
    land_angle = tooth_pitch * 0.26
    fall_angle = tooth_pitch * 0.18

    points = []
    for index in range(teeth):
        start_angle = index * tooth_pitch
        points.extend(
            (
                _polar_point(root_radius, start_angle),
                _polar_point(outer_radius, start_angle + rise_angle),
                _polar_point(outer_radius, start_angle + rise_angle + land_angle),
                _polar_point(root_radius, start_angle + rise_angle + land_angle + fall_angle),
            )
        )

    gear = (
        cq.Workplane("YZ")
        .moveTo(*points[0])
        .polyline(points[1:])
        .close()
        .extrude(GEAR_WIDTH)
        .translate((-GEAR_WIDTH / 2.0, 0.0, 0.0))
    )

    if hub_radius > 0.0 and hub_width > 0.0:
        gear = gear.union(_x_cylinder(hub_radius, hub_width, center_x=0.0))

    if hole_count and hole_radius > 0.0 and hole_center_radius > 0.0:
        for index in range(hole_count):
            angle = (2.0 * math.pi * index) / hole_count
            gear = gear.cut(
                _x_cylinder(
                    hole_radius,
                    GEAR_WIDTH + max(hub_width, 0.0) + 0.006,
                    center_x=0.0,
                    y=hole_center_radius * math.cos(angle),
                    z=hole_center_radius * math.sin(angle),
                )
            )

    return gear


def _handwheel_shape():
    outer_radius = 0.024
    inner_radius = 0.017
    hub_radius = 0.014
    wheel_thickness = 0.014

    wheel = _x_annulus(outer_radius, inner_radius, wheel_thickness, center_x=-0.039)
    wheel = wheel.union(_x_cylinder(hub_radius, wheel_thickness, center_x=-0.039))

    spoke_length = inner_radius - (hub_radius * 1.2)
    spoke_width = 0.006
    spoke_center_radius = hub_radius + (spoke_length / 2.0)
    spoke = _box_at(
        wheel_thickness,
        spoke_length,
        spoke_width,
        x=-0.039,
        y=spoke_center_radius,
        z=0.0,
    )
    for angle_deg in (0.0, 120.0, 240.0):
        wheel = wheel.union(spoke.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg))

    grip = _x_cylinder(0.0042, 0.018, center_x=-0.030, y=0.0, z=outer_radius - 0.003)
    return wheel.union(grip)


def _output_coupling_shape():
    coupling = _x_cylinder(0.0115, 0.016, center_x=0.037)
    flange = _x_cylinder(0.021, 0.006, center_x=0.044)
    return coupling.union(flange)


def _side_plate_shape():
    plate = _box_at(PLATE_THICKNESS, FRAME_WIDTH, FRAME_HEIGHT, z=FRAME_Z_CENTER)
    plate = plate.edges("|X").fillet(0.012)

    window = _box_at(PLATE_THICKNESS + 0.006, WINDOW_WIDTH, WINDOW_HEIGHT, z=WINDOW_Z_CENTER)
    window = window.edges("|X").fillet(0.018)
    plate = plate.cut(window)

    for y_pos in (INPUT_Y, IDLER_Y, OUTPUT_Y):
        plate = plate.cut(
            _x_cylinder(
                BEARING_BORE_RADIUS,
                PLATE_THICKNESS + (2.0 * PLATE_BOSS_THICKNESS) + 0.006,
                center_x=0.0,
                y=y_pos,
                z=SHAFT_Z,
            )
        )
        boss = _x_annulus(
            BEARING_BOSS_RADIUS,
            BEARING_BORE_RADIUS,
            PLATE_THICKNESS + (2.0 * PLATE_BOSS_THICKNESS),
            center_x=0.0,
            y=y_pos,
            z=SHAFT_Z,
        )
        plate = plate.union(boss)

    return plate


def _frame_shape():
    plate = _side_plate_shape()
    frame = plate.translate((-PLATE_CENTER_X, 0.0, 0.0)).union(
        plate.translate((PLATE_CENTER_X, 0.0, 0.0))
    )

    for y_pos, z_pos, size_y, size_z in (
        (-0.118, -0.028, 0.030, 0.018),
        (0.118, -0.028, 0.030, 0.018),
        (-0.118, 0.128, 0.022, 0.014),
        (0.118, 0.128, 0.022, 0.014),
    ):
        frame = frame.union(
            _box_at(
                (2.0 * INNER_FACE_X) + 0.002,
                size_y,
                size_z,
                x=0.0,
                y=y_pos,
                z=z_pos,
            )
        )

    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spur_reduction_assembly")

    model.material("frame_paint", rgba=(0.20, 0.23, 0.27, 1.0))
    model.material("gear_metal", rgba=(0.76, 0.73, 0.67, 1.0))
    model.material("shaft_steel", rgba=(0.67, 0.70, 0.74, 1.0))
    model.material("wheel_black", rgba=(0.10, 0.10, 0.12, 1.0))
    model.material("hub_dark", rgba=(0.36, 0.38, 0.42, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_side_plate_shape(), "left_plate"),
        origin=Origin(xyz=(-PLATE_CENTER_X, 0.0, 0.0)),
        material="frame_paint",
        name="left_plate",
    )
    frame.visual(
        mesh_from_cadquery(_side_plate_shape(), "right_plate"),
        origin=Origin(xyz=(PLATE_CENTER_X, 0.0, 0.0)),
        material="frame_paint",
        name="right_plate",
    )
    for visual_name, y_pos, z_pos, size_y, size_z in (
        ("lower_left_tie", -0.118, -0.028, 0.030, 0.018),
        ("lower_right_tie", 0.118, -0.028, 0.030, 0.018),
        ("upper_left_tie", -0.118, 0.128, 0.022, 0.014),
        ("upper_right_tie", 0.118, 0.128, 0.022, 0.014),
    ):
        frame.visual(
            mesh_from_cadquery(
                _box_at((2.0 * INNER_FACE_X) + 0.002, size_y, size_z, y=y_pos, z=z_pos),
                visual_name,
            ),
            material="frame_paint",
            name=visual_name,
        )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        mesh_from_cadquery(_shaft_body(-0.046, 0.032), "input_shaft_body"),
        material="shaft_steel",
        name="shaft_body",
    )
    input_shaft.visual(
        mesh_from_cadquery(
            _gear_shape(
                INPUT_TEETH,
                outer_radius=INPUT_GEAR_OUTER_RADIUS,
                root_radius=INPUT_GEAR_ROOT_RADIUS,
                hub_radius=0.011,
                hub_width=0.016,
            ),
            "input_gear",
        ),
        material="gear_metal",
        name="input_gear",
    )
    input_shaft.visual(
        mesh_from_cadquery(_handwheel_shape(), "handwheel"),
        material="wheel_black",
        name="handwheel",
    )

    idler_shaft = model.part("idler_shaft")
    idler_shaft.visual(
        mesh_from_cadquery(_shaft_body(-0.032, 0.032), "idler_shaft_body"),
        material="shaft_steel",
        name="shaft_body",
    )
    idler_shaft.visual(
        mesh_from_cadquery(
            _gear_shape(
                IDLER_TEETH,
                outer_radius=IDLER_GEAR_OUTER_RADIUS,
                root_radius=IDLER_GEAR_ROOT_RADIUS,
                hole_count=3,
                hole_radius=0.006,
                hole_center_radius=0.016,
                hub_radius=0.014,
                hub_width=0.018,
            ),
            "idler_gear",
        ),
        material="gear_metal",
        name="idler_gear",
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        mesh_from_cadquery(_shaft_body(-0.032, 0.046, right_collar=False), "output_shaft_body"),
        material="shaft_steel",
        name="shaft_body",
    )
    output_shaft.visual(
        mesh_from_cadquery(
            _gear_shape(
                OUTPUT_TEETH,
                outer_radius=OUTPUT_GEAR_OUTER_RADIUS,
                root_radius=OUTPUT_GEAR_ROOT_RADIUS,
                hole_count=6,
                hole_radius=0.007,
                hole_center_radius=0.024,
                hub_radius=0.017,
                hub_width=0.018,
            ),
            "output_gear",
        ),
        material="gear_metal",
        name="output_gear",
    )
    output_shaft.visual(
        mesh_from_cadquery(_output_coupling_shape(), "output_coupling"),
        material="hub_dark",
        name="output_coupling",
    )

    common_limits = MotionLimits(
        effort=20.0,
        velocity=10.0,
        lower=-(2.0 * math.pi),
        upper=2.0 * math.pi,
    )

    model.articulation(
        "frame_to_input_shaft",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=input_shaft,
        origin=Origin(xyz=(0.0, INPUT_Y, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "frame_to_idler_shaft",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=idler_shaft,
        origin=Origin(xyz=(0.0, IDLER_Y, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "frame_to_output_shaft",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(0.0, OUTPUT_Y, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=common_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")

    input_joint = object_model.get_articulation("frame_to_input_shaft")
    idler_joint = object_model.get_articulation("frame_to_idler_shaft")
    output_joint = object_model.get_articulation("frame_to_output_shaft")

    input_body = input_shaft.get_visual("shaft_body")
    idler_body = idler_shaft.get_visual("shaft_body")
    output_body = output_shaft.get_visual("shaft_body")
    input_gear = input_shaft.get_visual("input_gear")
    idler_gear = idler_shaft.get_visual("idler_gear")
    output_gear = output_shaft.get_visual("output_gear")

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
        "joint_axes_are_parallel_shaft_axes",
        all(
            abs(axis[0] - 1.0) < 1e-9 and abs(axis[1]) < 1e-9 and abs(axis[2]) < 1e-9
            for axis in (input_joint.axis, idler_joint.axis, output_joint.axis)
        ),
        details=(
            f"axes={input_joint.axis}, {idler_joint.axis}, {output_joint.axis}"
        ),
    )

    ctx.expect_contact(
        input_shaft,
        frame,
        elem_a=input_body,
        name="input_shaft_is_supported",
    )
    ctx.expect_contact(
        idler_shaft,
        frame,
        elem_a=idler_body,
        name="idler_shaft_is_supported",
    )
    ctx.expect_contact(
        output_shaft,
        frame,
        elem_a=output_body,
        name="output_shaft_is_supported",
    )

    ctx.expect_overlap(
        input_shaft,
        idler_shaft,
        axes="xz",
        elem_a=input_gear,
        elem_b=idler_gear,
        min_overlap=0.010,
        name="input_and_idler_gears_share_a_mesh_plane",
    )
    ctx.expect_overlap(
        idler_shaft,
        output_shaft,
        axes="xz",
        elem_a=idler_gear,
        elem_b=output_gear,
        min_overlap=0.010,
        name="idler_and_output_gears_share_a_mesh_plane",
    )

    input_pos = ctx.part_world_position(input_shaft)
    idler_pos = ctx.part_world_position(idler_shaft)
    output_pos = ctx.part_world_position(output_shaft)
    shaft_row_ok = (
        input_pos is not None
        and idler_pos is not None
        and output_pos is not None
        and abs(input_pos[2] - SHAFT_Z) < 1e-9
        and abs(idler_pos[2] - SHAFT_Z) < 1e-9
        and abs(output_pos[2] - SHAFT_Z) < 1e-9
        and abs((idler_pos[1] - input_pos[1]) - (IDLER_Y - INPUT_Y)) < 1e-9
        and abs((output_pos[1] - idler_pos[1]) - (OUTPUT_Y - IDLER_Y)) < 1e-9
    )
    ctx.check(
        "shaft_centers_follow_the_three_shaft_reduction_layout",
        shaft_row_ok,
        details=f"input={input_pos}, idler={idler_pos}, output={output_pos}",
    )

    def _gear_diameter(part, visual):
        aabb = ctx.part_element_world_aabb(part, elem=visual)
        if aabb is None:
            return None
        min_pt, max_pt = aabb
        return max(max_pt[1] - min_pt[1], max_pt[2] - min_pt[2])

    input_d = _gear_diameter(input_shaft, input_gear)
    idler_d = _gear_diameter(idler_shaft, idler_gear)
    output_d = _gear_diameter(output_shaft, output_gear)
    ctx.check(
        "gear_sizes_make_the_reduction_obvious",
        input_d is not None
        and idler_d is not None
        and output_d is not None
        and input_d < idler_d < output_d
        and (output_d - input_d) > 0.070,
        details=f"diameters={(input_d, idler_d, output_d)}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
