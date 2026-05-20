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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(
    geom: MeshGeometry,
    a: int,
    b: int,
    c: int,
    d: int,
) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in vector))
    return (
        vector[0] / length,
        vector[1] / length,
        vector[2] / length,
    )


def _panel_prism(
    outer_quad: list[tuple[float, float, float]],
    inward_normal: tuple[float, float, float],
    thickness: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    inner_quad = [
        (
            point[0] + inward_normal[0] * thickness,
            point[1] + inward_normal[1] * thickness,
            point[2] + inward_normal[2] * thickness,
        )
        for point in outer_quad
    ]

    outer_ids = [geom.add_vertex(*point) for point in outer_quad]
    inner_ids = [geom.add_vertex(*point) for point in inner_quad]

    _add_quad(geom, outer_ids[0], outer_ids[1], outer_ids[2], outer_ids[3])
    _add_quad(geom, inner_ids[3], inner_ids[2], inner_ids[1], inner_ids[0])

    for index in range(4):
        next_index = (index + 1) % 4
        _add_quad(
            geom,
            outer_ids[index],
            outer_ids[next_index],
            inner_ids[next_index],
            inner_ids[index],
        )

    return geom


def _build_canopy_shell_mesh() -> MeshGeometry:
    bottom_half_x = 0.55
    bottom_half_y = 0.35
    top_half_x = 0.30
    top_half_y = 0.14
    canopy_height = 0.36
    shell_thickness = 0.018

    front_inward = _normalize((0.0, -canopy_height, -(bottom_half_y - top_half_y)))
    back_inward = _normalize((0.0, canopy_height, -(bottom_half_y - top_half_y)))
    left_inward = _normalize((canopy_height, 0.0, -(bottom_half_x - top_half_x)))
    right_inward = _normalize((-canopy_height, 0.0, -(bottom_half_x - top_half_x)))

    front_outer = [
        (-bottom_half_x, bottom_half_y, 0.0),
        (bottom_half_x, bottom_half_y, 0.0),
        (top_half_x, top_half_y, canopy_height),
        (-top_half_x, top_half_y, canopy_height),
    ]
    back_outer = [
        (bottom_half_x, -bottom_half_y, 0.0),
        (-bottom_half_x, -bottom_half_y, 0.0),
        (-top_half_x, -top_half_y, canopy_height),
        (top_half_x, -top_half_y, canopy_height),
    ]
    left_outer = [
        (-bottom_half_x, -bottom_half_y, 0.0),
        (-bottom_half_x, bottom_half_y, 0.0),
        (-top_half_x, top_half_y, canopy_height),
        (-top_half_x, -top_half_y, canopy_height),
    ]
    right_outer = [
        (bottom_half_x, bottom_half_y, 0.0),
        (bottom_half_x, -bottom_half_y, 0.0),
        (top_half_x, -top_half_y, canopy_height),
        (top_half_x, top_half_y, canopy_height),
    ]

    shell = MeshGeometry()
    shell.merge(_panel_prism(front_outer, front_inward, shell_thickness))
    shell.merge(_panel_prism(back_outer, back_inward, shell_thickness))
    shell.merge(_panel_prism(left_outer, left_inward, shell_thickness))
    shell.merge(_panel_prism(right_outer, right_inward, shell_thickness))
    return shell


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="island_range_hood")

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.80, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.69, 0.71, 0.73, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_filter = model.material("dark_filter", rgba=(0.24, 0.25, 0.25, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    indicator_white = model.material("indicator_white", rgba=(0.92, 0.93, 0.94, 1.0))

    hood_body = model.part("hood_body")
    hood_body.visual(
        mesh_from_geometry(_build_canopy_shell_mesh(), "range_hood_canopy_shell"),
        material=stainless,
        name="canopy_shell",
    )
    hood_body.visual(
        Box((0.62, 0.30, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.371)),
        material=satin_steel,
        name="canopy_top_plate",
    )
    hood_body.visual(
        Box((0.30, 0.30, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        material=satin_steel,
        name="chimney",
    )
    hood_body.visual(
        Box((0.40, 0.40, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 1.165)),
        material=satin_steel,
        name="ceiling_plate",
    )

    opening_width = 0.88
    opening_depth = 0.52
    lip_height = 0.03
    side_lip_width = (1.10 - opening_width) * 0.5
    front_lip_depth = (0.70 - opening_depth) * 0.5

    hood_body.visual(
        Box((opening_width, front_lip_depth, lip_height)),
        origin=Origin(xyz=(0.0, 0.305, lip_height * 0.5)),
        material=satin_steel,
        name="opening_front_lip",
    )
    hood_body.visual(
        Box((opening_width, front_lip_depth, lip_height)),
        origin=Origin(xyz=(0.0, -0.305, lip_height * 0.5)),
        material=satin_steel,
        name="opening_back_lip",
    )
    hood_body.visual(
        Box((side_lip_width, opening_depth, lip_height)),
        origin=Origin(xyz=(-0.495, 0.0, lip_height * 0.5)),
        material=satin_steel,
        name="opening_left_lip",
    )
    hood_body.visual(
        Box((side_lip_width, opening_depth, lip_height)),
        origin=Origin(xyz=(0.495, 0.0, lip_height * 0.5)),
        material=satin_steel,
        name="opening_right_lip",
    )

    hinge_radius = 0.014
    side_slope_angle = math.atan((0.55 - 0.30) / 0.36)
    knob_face_angle = math.pi * 0.5 - side_slope_angle
    knob_mount_radius = 0.015
    knob_mount_length = 0.008
    knob_mount_outer_offset = 0.006
    knob_mount_center_offset = knob_mount_outer_offset - knob_mount_length * 0.5
    knob_mount_z = 0.058
    knob_mount_x = 0.55 - (0.55 - 0.30) * (knob_mount_z / 0.36)
    left_knob_mount_point = (-knob_mount_x, 0.0, knob_mount_z)
    right_knob_mount_point = (knob_mount_x, 0.0, knob_mount_z)
    left_knob_outward = (-math.cos(side_slope_angle), 0.0, math.sin(side_slope_angle))
    right_knob_outward = (math.cos(side_slope_angle), 0.0, math.sin(side_slope_angle))
    left_knob_mount_center = (
        left_knob_mount_point[0] + left_knob_outward[0] * knob_mount_center_offset,
        left_knob_mount_point[1] + left_knob_outward[1] * knob_mount_center_offset,
        left_knob_mount_point[2] + left_knob_outward[2] * knob_mount_center_offset,
    )
    right_knob_mount_center = (
        right_knob_mount_point[0] + right_knob_outward[0] * knob_mount_center_offset,
        right_knob_mount_point[1] + right_knob_outward[1] * knob_mount_center_offset,
        right_knob_mount_point[2] + right_knob_outward[2] * knob_mount_center_offset,
    )
    left_knob_joint_point = (
        left_knob_mount_point[0] + left_knob_outward[0] * knob_mount_outer_offset,
        left_knob_mount_point[1] + left_knob_outward[1] * knob_mount_outer_offset,
        left_knob_mount_point[2] + left_knob_outward[2] * knob_mount_outer_offset,
    )
    right_knob_joint_point = (
        right_knob_mount_point[0] + right_knob_outward[0] * knob_mount_outer_offset,
        right_knob_mount_point[1] + right_knob_outward[1] * knob_mount_outer_offset,
        right_knob_mount_point[2] + right_knob_outward[2] * knob_mount_outer_offset,
    )
    left_knob_rpy = (0.0, -knob_face_angle, 0.0)
    right_knob_rpy = (0.0, knob_face_angle, 0.0)
    hood_body.visual(
        Cylinder(radius=knob_mount_radius, length=knob_mount_length),
        origin=Origin(xyz=left_knob_mount_center, rpy=left_knob_rpy),
        material=satin_steel,
        name="left_knob_mount",
    )
    hood_body.visual(
        Cylinder(radius=knob_mount_radius, length=knob_mount_length),
        origin=Origin(xyz=right_knob_mount_center, rpy=right_knob_rpy),
        material=satin_steel,
        name="right_knob_mount",
    )
    hood_body.visual(
        Cylinder(radius=hinge_radius, length=0.18),
        origin=Origin(
            xyz=(-opening_width * 0.5, -0.16, -hinge_radius),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=charcoal,
        name="body_hinge_front",
    )
    hood_body.visual(
        Cylinder(radius=hinge_radius, length=0.18),
        origin=Origin(
            xyz=(-opening_width * 0.5, 0.16, -hinge_radius),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=charcoal,
        name="body_hinge_rear",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((1.10, 0.70, 1.20)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
    )

    filter_frame = model.part("filter_frame")
    frame_width = 0.86
    frame_depth = 0.50
    frame_thickness = 0.022
    frame_rail = 0.026
    divider_width = 0.020
    panel_thickness = 0.010
    top_surface_z = hinge_radius
    rail_center_z = top_surface_z - frame_thickness * 0.5
    panel_center_z = top_surface_z - 0.008
    frame_start_x = hinge_radius
    clear_frame_width = frame_width - frame_start_x

    filter_frame.visual(
        Box((clear_frame_width, frame_rail, frame_thickness)),
        origin=Origin(
            xyz=(
                frame_start_x + clear_frame_width * 0.5,
                frame_depth * 0.5 - frame_rail * 0.5,
                rail_center_z,
            )
        ),
        material=satin_steel,
        name="frame_front_rail",
    )
    filter_frame.visual(
        Box((clear_frame_width, frame_rail, frame_thickness)),
        origin=Origin(
            xyz=(
                frame_start_x + clear_frame_width * 0.5,
                -frame_depth * 0.5 + frame_rail * 0.5,
                rail_center_z,
            )
        ),
        material=satin_steel,
        name="frame_back_rail",
    )
    filter_frame.visual(
        Box((frame_rail, frame_depth, frame_thickness)),
        origin=Origin(xyz=(frame_start_x + frame_rail * 0.5, 0.0, rail_center_z)),
        material=satin_steel,
        name="frame_left_rail",
    )
    filter_frame.visual(
        Box((frame_rail, frame_depth, frame_thickness)),
        origin=Origin(xyz=(frame_width - frame_rail * 0.5, 0.0, rail_center_z)),
        material=satin_steel,
        name="frame_right_rail",
    )
    filter_frame.visual(
        Box((divider_width, frame_depth - 2.0 * frame_rail, frame_thickness * 0.9)),
        origin=Origin(xyz=(frame_start_x + clear_frame_width * 0.5, 0.0, rail_center_z - 0.001)),
        material=satin_steel,
        name="frame_divider",
    )

    filter_panel_width = (clear_frame_width - 2.0 * frame_rail - divider_width + 0.012) * 0.5
    filter_panel_depth = frame_depth - 2.0 * frame_rail + 0.010
    left_panel_center_x = frame_start_x + frame_rail + filter_panel_width * 0.5 - 0.003
    right_panel_center_x = frame_width - frame_rail - filter_panel_width * 0.5 + 0.003
    filter_frame.visual(
        Box((filter_panel_width, filter_panel_depth, panel_thickness)),
        origin=Origin(xyz=(left_panel_center_x, 0.0, panel_center_z)),
        material=dark_filter,
        name="filter_panel_left",
    )
    filter_frame.visual(
        Box((filter_panel_width, filter_panel_depth, panel_thickness)),
        origin=Origin(xyz=(right_panel_center_x, 0.0, panel_center_z)),
        material=dark_filter,
        name="filter_panel_right",
    )
    filter_frame.visual(
        Box((0.060, 0.014, 0.012)),
        origin=Origin(xyz=(frame_width - 0.032, 0.0, top_surface_z - 0.020)),
        material=black_plastic,
        name="frame_handle",
    )
    filter_frame.visual(
        Cylinder(radius=hinge_radius, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="frame_hinge_knuckle",
    )
    filter_frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, frame_thickness)),
        mass=4.2,
        origin=Origin(xyz=(frame_width * 0.5, 0.0, rail_center_z)),
    )

    model.articulation(
        "hood_to_filter_frame",
        ArticulationType.REVOLUTE,
        parent=hood_body,
        child=filter_frame,
        origin=Origin(xyz=(-opening_width * 0.5, 0.0, -hinge_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.75,
        ),
    )

    def _add_side_knob(
        part_name: str,
        joint_name: str,
        joint_origin: Origin,
    ) -> None:
        knob_part = model.part(part_name)
        knob_part.visual(
            Cylinder(radius=0.007, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=charcoal,
            name="knob_shaft",
        )
        knob_part.visual(
            Cylinder(radius=0.015, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=satin_steel,
            name="knob_bezel",
        )
        knob_part.visual(
            Cylinder(radius=0.020, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.023)),
            material=black_plastic,
            name="knob_head",
        )
        knob_part.visual(
            Box((0.003, 0.013, 0.0025)),
            origin=Origin(xyz=(0.0, 0.0125, 0.0325)),
            material=indicator_white,
            name="knob_pointer",
        )
        knob_part.inertial = Inertial.from_geometry(
            Cylinder(radius=0.020, length=0.032),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0, 0.016)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob_part,
            origin=joint_origin,
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.2,
                velocity=6.0,
            ),
        )

    _add_side_knob(
        "left_light_knob",
        "hood_to_left_light_knob",
        joint_origin=Origin(
            xyz=left_knob_joint_point,
            rpy=left_knob_rpy,
        ),
    )
    _add_side_knob(
        "right_light_knob",
        "hood_to_right_light_knob",
        joint_origin=Origin(
            xyz=right_knob_joint_point,
            rpy=right_knob_rpy,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood_body = object_model.get_part("hood_body")
    filter_frame = object_model.get_part("filter_frame")
    left_knob = object_model.get_part("left_light_knob")
    right_knob = object_model.get_part("right_light_knob")

    filter_hinge = object_model.get_articulation("hood_to_filter_frame")
    left_knob_joint = object_model.get_articulation("hood_to_left_light_knob")
    right_knob_joint = object_model.get_articulation("hood_to_right_light_knob")

    ctx.expect_contact(
        filter_frame,
        hood_body,
        elem_a="frame_hinge_knuckle",
        elem_b="body_hinge_front",
        name="filter frame hinge knuckle seats against front body barrel",
    )
    ctx.expect_contact(
        filter_frame,
        hood_body,
        elem_a="frame_hinge_knuckle",
        elem_b="body_hinge_rear",
        name="filter frame hinge knuckle seats against rear body barrel",
    )
    ctx.expect_contact(
        left_knob,
        hood_body,
        contact_tol=0.0005,
        elem_a="knob_shaft",
        elem_b="left_knob_mount",
        name="left knob shaft mounts to side escutcheon",
    )
    ctx.expect_contact(
        right_knob,
        hood_body,
        contact_tol=0.0005,
        elem_a="knob_shaft",
        elem_b="right_knob_mount",
        name="right knob shaft mounts to side escutcheon",
    )

    left_knob_pos = ctx.part_world_position(left_knob)
    right_knob_pos = ctx.part_world_position(right_knob)
    ctx.check(
        "side knobs sit on opposite canopy faces",
        left_knob_pos is not None
        and right_knob_pos is not None
        and left_knob_pos[0] < -0.44
        and right_knob_pos[0] > 0.44,
        details=f"left={left_knob_pos}, right={right_knob_pos}",
    )

    rest_handle_center = _aabb_center(
        ctx.part_element_world_aabb(filter_frame, elem="frame_handle")
    )
    with ctx.pose({filter_hinge: 1.25}):
        open_handle_center = _aabb_center(
            ctx.part_element_world_aabb(filter_frame, elem="frame_handle")
        )
    ctx.check(
        "filter frame swings downward from one side",
        rest_handle_center is not None
        and open_handle_center is not None
        and open_handle_center[2] < rest_handle_center[2] - 0.18
        and open_handle_center[0] < rest_handle_center[0] - 0.10,
        details=f"rest={rest_handle_center}, open={open_handle_center}",
    )

    left_pointer_rest = _aabb_center(
        ctx.part_element_world_aabb(left_knob, elem="knob_pointer")
    )
    with ctx.pose({left_knob_joint: 1.0}):
        left_pointer_turned = _aabb_center(
            ctx.part_element_world_aabb(left_knob, elem="knob_pointer")
        )
    left_pointer_delta = None
    if left_pointer_rest is not None and left_pointer_turned is not None:
        left_pointer_delta = math.dist(left_pointer_rest, left_pointer_turned)
    ctx.check(
        "left knob pointer rotates around its shaft",
        left_pointer_delta is not None and left_pointer_delta > 0.010,
        details=f"rest={left_pointer_rest}, turned={left_pointer_turned}, delta={left_pointer_delta}",
    )

    right_pointer_rest = _aabb_center(
        ctx.part_element_world_aabb(right_knob, elem="knob_pointer")
    )
    with ctx.pose({right_knob_joint: -1.0}):
        right_pointer_turned = _aabb_center(
            ctx.part_element_world_aabb(right_knob, elem="knob_pointer")
        )
    right_pointer_delta = None
    if right_pointer_rest is not None and right_pointer_turned is not None:
        right_pointer_delta = math.dist(right_pointer_rest, right_pointer_turned)
    ctx.check(
        "right knob pointer rotates around its shaft",
        right_pointer_delta is not None and right_pointer_delta > 0.010,
        details=f"rest={right_pointer_rest}, turned={right_pointer_turned}, delta={right_pointer_delta}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
