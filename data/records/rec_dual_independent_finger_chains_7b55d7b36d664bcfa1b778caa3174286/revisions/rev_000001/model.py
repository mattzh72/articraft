from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from dataclasses import dataclass
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


@dataclass(frozen=True)
class LinkSpec:
    length: float
    plate_height: float
    root_length: float
    web_thickness: float
    tip_mount_length: float = 0.0


@dataclass(frozen=True)
class FingerSpec:
    prefix: str
    y_offset: float
    barrel_radius: float
    barrel_width: float
    side_width: float
    joint_axis_x: float
    joint_axis_z: float
    pedestal_width: float
    pedestal_height: float
    tip_style: str
    base: LinkSpec
    middle: LinkSpec
    distal: LinkSpec


LEFT_FINGER = FingerSpec(
    prefix="left",
    y_offset=-0.022,
    barrel_radius=0.0065,
    barrel_width=0.0080,
    side_width=0.0045,
    joint_axis_x=0.018,
    joint_axis_z=0.026,
    pedestal_width=0.025,
    pedestal_height=0.017,
    tip_style="rounded_pad",
    base=LinkSpec(length=0.036, plate_height=0.021, root_length=0.011, web_thickness=0.0055),
    middle=LinkSpec(length=0.029, plate_height=0.018, root_length=0.010, web_thickness=0.0050),
    distal=LinkSpec(
        length=0.018,
        plate_height=0.015,
        root_length=0.009,
        web_thickness=0.0045,
        tip_mount_length=0.008,
    ),
)

RIGHT_FINGER = FingerSpec(
    prefix="right",
    y_offset=0.022,
    barrel_radius=0.0055,
    barrel_width=0.0070,
    side_width=0.0040,
    joint_axis_x=0.018,
    joint_axis_z=0.026,
    pedestal_width=0.022,
    pedestal_height=0.016,
    tip_style="flat_nib",
    base=LinkSpec(length=0.029, plate_height=0.019, root_length=0.010, web_thickness=0.0048),
    middle=LinkSpec(length=0.024, plate_height=0.016, root_length=0.009, web_thickness=0.0043),
    distal=LinkSpec(
        length=0.015,
        plate_height=0.014,
        root_length=0.008,
        web_thickness=0.0040,
        tip_mount_length=0.007,
    ),
)

FINGERS = (LEFT_FINGER, RIGHT_FINGER)

PALM_BASE_LENGTH = 0.078
PALM_BASE_WIDTH = 0.078
PALM_BASE_THICKNESS = 0.012
PALM_BASE_CENTER_X = -0.010
PALM_REAR_BLOCK_LENGTH = 0.026
PALM_REAR_BLOCK_HEIGHT = 0.018


def _box(length: float, width: float, height: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center_xyz)


def _cylinder_y(
    radius: float,
    length: float,
    center_xyz: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center_xyz)


def _union_shapes(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _outer_width(finger: FingerSpec) -> float:
    return finger.barrel_width + 2.0 * finger.side_width


def _make_cap_screw(
    *,
    x: float,
    y: float,
    top_z: float,
    head_radius: float,
    head_height: float,
    shank_radius: float,
    embed_depth: float,
) -> cq.Workplane:
    head = cq.Workplane("XY").circle(head_radius).extrude(head_height).translate((x, y, top_z))
    shank = cq.Workplane("XY").circle(shank_radius).extrude(embed_depth).translate(
        (x, y, top_z - embed_depth)
    )
    return head.union(shank)


def _make_cap_screw_cluster(
    points: list[tuple[float, float]],
    *,
    top_z: float,
    head_radius: float,
    head_height: float,
    shank_radius: float,
    embed_depth: float,
) -> cq.Workplane:
    screws = [
        _make_cap_screw(
            x=x_pos,
            y=y_pos,
            top_z=top_z,
            head_radius=head_radius,
            head_height=head_height,
            shank_radius=shank_radius,
            embed_depth=embed_depth,
        )
        for x_pos, y_pos in points
    ]
    return _union_shapes(screws)


def _make_link_body(finger: FingerSpec, link: LinkSpec, *, has_distal_joint: bool) -> cq.Workplane:
    outer_width = _outer_width(finger)
    y_plate = finger.barrel_width / 2.0 + finger.side_width / 2.0
    root_height = link.plate_height * 0.56
    root_clearance = finger.barrel_radius + 0.0025
    side_plate_start = root_clearance
    root_block_start = root_clearance + 0.0010

    if has_distal_joint:
        side_plate_end = link.length - finger.barrel_radius * 0.42
        web_end = link.length - finger.barrel_radius * 1.35
    else:
        side_plate_end = link.length + link.tip_mount_length * 0.76
        web_end = link.length + link.tip_mount_length * 0.50

    side_plate_length = max(0.004, side_plate_end - side_plate_start)
    web_start = root_clearance + 0.0015
    web_length = max(0.004, web_end - web_start)
    root_block_length = max(0.005, link.root_length - 0.001)

    shapes: list[cq.Workplane] = [
        _cylinder_y(finger.barrel_radius, finger.barrel_width, (0.0, 0.0, 0.0)),
        _box(
            root_block_length,
            finger.barrel_width + finger.side_width * 0.75,
            root_height,
            (root_block_start + root_block_length / 2.0, 0.0, -link.plate_height * 0.04),
        ),
        _box(
            web_length,
            finger.barrel_width,
            link.web_thickness,
            (
                web_start + web_length / 2.0,
                0.0,
                -link.plate_height / 2.0 + link.web_thickness / 2.0,
            ),
        ),
    ]

    for sign in (-1.0, 1.0):
        shapes.append(
            _box(
                side_plate_length,
                finger.side_width,
                link.plate_height,
                (side_plate_start + side_plate_length / 2.0, sign * y_plate, 0.0),
            )
        )

    if has_distal_joint:
        for sign in (-1.0, 1.0):
            shapes.append(
                _cylinder_y(
                    finger.barrel_radius,
                    finger.side_width,
                    (link.length, sign * y_plate, 0.0),
                )
            )
    else:
        toe_height = max(link.plate_height * 0.56, link.web_thickness * 1.8)
        shapes.append(
            _box(
                link.tip_mount_length,
                outer_width - finger.side_width * 0.15,
                toe_height,
                (link.length + link.tip_mount_length / 2.0 - 0.001, 0.0, 0.0),
            )
        )

    return _union_shapes(shapes)


def _make_tip_insert(finger: FingerSpec) -> cq.Workplane:
    link = finger.distal
    if finger.tip_style == "rounded_pad":
        pad_width = finger.barrel_width + finger.side_width * 1.4
        pad_length = 0.012
        pad_height = 0.010
        return (
            cq.Workplane("XZ")
            .slot2D(pad_length, pad_height)
            .extrude(pad_width / 2.0, both=True)
            .translate((link.length + link.tip_mount_length + pad_length / 2.0 - 0.004, 0.0, 0.0))
        )

    nib_length = 0.011
    nib_height = 0.0045
    nib_width = finger.barrel_width + finger.side_width * 0.7
    return _box(
        nib_length,
        nib_width,
        nib_height,
        (link.length + link.tip_mount_length + nib_length / 2.0 - 0.004, 0.0, -0.001),
    )


def _make_tip_screws(finger: FingerSpec) -> cq.Workplane:
    link = finger.distal
    toe_height = max(link.plate_height * 0.56, link.web_thickness * 1.8)
    points = [
        (link.length + link.tip_mount_length * 0.36, 0.0),
        (link.length + link.tip_mount_length * 0.82, 0.0),
    ]
    return _make_cap_screw_cluster(
        points,
        top_z=toe_height / 2.0,
        head_radius=0.0015 if finger.prefix == "left" else 0.00135,
        head_height=0.0015,
        shank_radius=0.00065,
        embed_depth=toe_height * 0.75,
    )


def _make_palm_body() -> cq.Workplane:
    shapes: list[cq.Workplane] = [
        _box(
            PALM_BASE_LENGTH,
            PALM_BASE_WIDTH,
            PALM_BASE_THICKNESS,
            (PALM_BASE_CENTER_X, 0.0, PALM_BASE_THICKNESS / 2.0),
        ),
        _box(
            PALM_REAR_BLOCK_LENGTH,
            PALM_BASE_WIDTH,
            PALM_REAR_BLOCK_HEIGHT,
            (
                -0.027,
                0.0,
                PALM_BASE_THICKNESS + PALM_REAR_BLOCK_HEIGHT / 2.0,
            ),
        ),
        _box(
            0.020,
            0.028,
            0.014,
            (-0.008, 0.0, PALM_BASE_THICKNESS + 0.007),
        ),
    ]

    for finger in FINGERS:
        outer_width = _outer_width(finger)
        pedestal_length = 0.028
        cheek_length = 0.016
        cheek_height = max(0.014, finger.base.plate_height * 0.72)
        y_plate = finger.barrel_width / 2.0 + finger.side_width / 2.0

        shapes.append(
            _box(
                pedestal_length,
                finger.pedestal_width,
                finger.pedestal_height,
                (
                    0.000,
                    finger.y_offset,
                    PALM_BASE_THICKNESS + finger.pedestal_height / 2.0,
                ),
            )
        )
        shapes.append(
            _box(
                0.012,
                outer_width + 0.005,
                0.010,
                (0.004, finger.y_offset, finger.joint_axis_z - 0.005),
            )
        )

        for sign in (-1.0, 1.0):
            y_pos = finger.y_offset + sign * y_plate
            shapes.append(_box(cheek_length, finger.side_width, cheek_height, (0.010, y_pos, finger.joint_axis_z)))
            shapes.append(
                _cylinder_y(
                    finger.barrel_radius,
                    finger.side_width,
                    (finger.joint_axis_x, y_pos, finger.joint_axis_z),
                )
            )

    palm = _union_shapes(shapes)

    central_split = _box(0.036, 0.020, 0.027, (0.003, 0.0, 0.024))
    palm = palm.cut(central_split)

    for finger in FINGERS:
        top_pocket = _box(
            0.017,
            finger.pedestal_width - 0.007,
            0.0045,
            (
                -0.002,
                finger.y_offset,
                PALM_BASE_THICKNESS + finger.pedestal_height - 0.0015,
            ),
        )
        front_relief = _box(0.010, finger.pedestal_width - 0.010, 0.010, (0.006, finger.y_offset, finger.joint_axis_z))
        joint_window = _box(
            0.022,
            finger.barrel_width + 0.0025,
            0.020,
            (finger.joint_axis_x, finger.y_offset, finger.joint_axis_z),
        )
        palm = palm.cut(top_pocket).cut(front_relief).cut(joint_window)

    return palm


def _make_palm_screws() -> cq.Workplane:
    points = [
        (-0.010, LEFT_FINGER.y_offset),
        (0.006, LEFT_FINGER.y_offset),
        (-0.010, RIGHT_FINGER.y_offset),
        (0.006, RIGHT_FINGER.y_offset),
        (-0.034, -0.024),
        (-0.034, 0.024),
    ]
    return _make_cap_screw_cluster(
        points,
        top_z=PALM_BASE_THICKNESS + LEFT_FINGER.pedestal_height,
        head_radius=0.0024,
        head_height=0.0022,
        shank_radius=0.0010,
        embed_depth=0.0065,
    )


def _add_box_visual(
    part_obj,
    *,
    size: tuple[float, float, float],
    center_xyz: tuple[float, float, float],
    material: str,
    name: str | None = None,
) -> None:
    part_obj.visual(Box(size), origin=Origin(xyz=center_xyz), material=material, name=name)


def _add_cylinder_y_visual(
    part_obj,
    *,
    radius: float,
    length: float,
    center_xyz: tuple[float, float, float],
    material: str,
    name: str | None = None,
) -> None:
    part_obj.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center_xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_z_visual(
    part_obj,
    *,
    radius: float,
    length: float,
    center_xyz: tuple[float, float, float],
    material: str,
    name: str | None = None,
) -> None:
    part_obj.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center_xyz),
        material=material,
        name=name,
    )


def _add_palm_visuals(model: ArticulatedObject) -> None:
    palm = model.part("palm")
    rear_plate_length = 0.046
    rear_plate_center_x = -0.026

    _add_box_visual(
        palm,
        size=(rear_plate_length, PALM_BASE_WIDTH, PALM_BASE_THICKNESS),
        center_xyz=(rear_plate_center_x, 0.0, PALM_BASE_THICKNESS / 2.0),
        material="palm_finish",
        name="rear_plate",
    )
    _add_box_visual(
        palm,
        size=(0.022, PALM_BASE_WIDTH, PALM_REAR_BLOCK_HEIGHT),
        center_xyz=(-0.035, 0.0, PALM_BASE_THICKNESS + PALM_REAR_BLOCK_HEIGHT / 2.0),
        material="palm_finish",
        name="rear_block",
    )
    _add_box_visual(
        palm,
        size=(0.016, 0.028, 0.014),
        center_xyz=(-0.011, 0.0, PALM_BASE_THICKNESS + 0.007),
        material="palm_finish",
        name="mid_bridge",
    )

    for finger in FINGERS:
        y_plate = finger.barrel_width / 2.0 + finger.side_width / 2.0
        outer_width = _outer_width(finger)
        _add_box_visual(
            palm,
            size=(0.014, finger.pedestal_width, finger.pedestal_height),
            center_xyz=(0.002, finger.y_offset, PALM_BASE_THICKNESS + finger.pedestal_height / 2.0),
            material="palm_finish",
            name=f"{finger.prefix}_pedestal",
        )
        _add_box_visual(
            palm,
            size=(0.008, outer_width + 0.003, 0.008),
            center_xyz=(0.006, finger.y_offset, finger.joint_axis_z - 0.006),
            material="palm_finish",
            name=f"{finger.prefix}_saddle",
        )
        for sign in (-1.0, 1.0):
            y_pos = finger.y_offset + sign * y_plate
            _add_box_visual(
                palm,
                size=(0.012, finger.side_width, max(0.014, finger.base.plate_height * 0.72)),
                center_xyz=(0.010, y_pos, finger.joint_axis_z),
                material="palm_finish",
                name=f"{finger.prefix}_root_cheek_{'neg' if sign < 0 else 'pos'}",
            )
            _add_cylinder_y_visual(
                palm,
                radius=finger.barrel_radius,
                length=finger.side_width,
                center_xyz=(finger.joint_axis_x, y_pos, finger.joint_axis_z),
                material="palm_finish",
                name=f"{finger.prefix}_root_barrel_{'neg' if sign < 0 else 'pos'}",
            )

    palm.visual(
        mesh_from_cadquery(_make_palm_screws(), "palm_screws"),
        material="fastener_black",
        name="cap_screws",
    )


def _add_link_part(
    model: ArticulatedObject,
    *,
    name: str,
    finger: FingerSpec,
    link: LinkSpec,
    has_distal_joint: bool,
    root_style: str,
    distal_style: str,
) -> None:
    part = model.part(name)
    y_plate = finger.barrel_width / 2.0 + finger.side_width / 2.0
    outer_width = _outer_width(finger)
    root_height = link.plate_height * 0.58
    root_clear_x = finger.barrel_radius + 0.0020
    side_plate_start = root_clear_x
    side_plate_end = link.length + (link.tip_mount_length * 0.70 if distal_style == "none" else 0.0)
    side_plate_length = max(0.004, side_plate_end - side_plate_start)
    web_start = finger.barrel_radius * 0.45
    web_end = link.length - finger.barrel_radius * 0.65 if distal_style == "clevis" else link.length + link.tip_mount_length * 0.42
    web_length = max(0.004, web_end - web_start)

    if root_style == "center":
        _add_cylinder_y_visual(
            part,
            radius=finger.barrel_radius,
            length=finger.barrel_width,
            center_xyz=(0.0, 0.0, 0.0),
            material="link_alloy",
            name="body",
        )
        _add_box_visual(
            part,
            size=(max(link.root_length - 0.001, 0.006), finger.barrel_width * 0.56, root_height),
            center_xyz=(root_clear_x + max(link.root_length - 0.001, 0.006) / 2.0, 0.0, -link.plate_height * 0.04),
            material="link_alloy",
            name=f"{name}_root_block",
        )
    else:
        for sign in (-1.0, 1.0):
            y_pos = sign * y_plate
            _add_box_visual(
                part,
                size=(max(link.root_length - 0.001, 0.007), finger.side_width, link.plate_height),
                center_xyz=(root_clear_x + max(link.root_length - 0.001, 0.007) / 2.0, y_pos, 0.0),
                material="link_alloy",
                name="body" if sign < 0 else f"{name}_root_cheek_pos",
            )
            _add_cylinder_y_visual(
                part,
                radius=finger.barrel_radius,
                length=finger.side_width,
                center_xyz=(0.0, y_pos, 0.0),
                material="link_alloy",
                name=f"{name}_root_barrel_{'neg' if sign < 0 else 'pos'}",
            )
        _add_box_visual(
            part,
            size=(max(link.root_length * 0.70, 0.005), finger.barrel_width * 0.46, root_height),
            center_xyz=(root_clear_x + max(link.root_length * 0.70, 0.005) / 2.0, 0.0, -link.plate_height * 0.05),
            material="link_alloy",
            name=f"{name}_root_bridge",
        )

    _add_box_visual(
        part,
        size=(web_length, outer_width * 0.96, link.web_thickness),
        center_xyz=(web_start + web_length / 2.0, 0.0, -link.plate_height / 2.0 + link.web_thickness / 2.0),
        material="link_alloy",
        name=f"{name}_web",
    )

    for sign in (-1.0, 1.0):
        _add_box_visual(
            part,
            size=(side_plate_length, finger.side_width, link.plate_height),
            center_xyz=(side_plate_start + side_plate_length / 2.0, sign * y_plate, 0.0),
            material="link_alloy",
            name=f"{name}_side_plate_{'neg' if sign < 0 else 'pos'}",
        )

    if distal_style == "clevis":
        for sign in (-1.0, 1.0):
            _add_cylinder_y_visual(
                part,
                radius=finger.barrel_radius,
                length=finger.side_width,
                center_xyz=(link.length, sign * y_plate, 0.0),
                material="link_alloy",
                name=f"{name}_distal_barrel_{'neg' if sign < 0 else 'pos'}",
            )
    elif distal_style == "none":
        toe_height = max(link.plate_height * 0.56, link.web_thickness * 1.8)
        _add_box_visual(
            part,
            size=(link.tip_mount_length, outer_width - finger.side_width * 0.15, toe_height),
            center_xyz=(link.length + link.tip_mount_length / 2.0 - 0.001, 0.0, 0.0),
            material="link_alloy",
            name=f"{name}_tip_mount",
        )

    if not has_distal_joint:
        part.visual(
            mesh_from_cadquery(_make_tip_insert(finger), f"{name}_tip_insert"),
            material="pad_insert" if finger.tip_style == "rounded_pad" else "nib_insert",
            name="tip_insert",
        )
        toe_height = max(link.plate_height * 0.56, link.web_thickness * 1.8)
        screw_radius = 0.00145 if finger.prefix == "left" else 0.00130
        screw_height = 0.0020
        for index, x_pos in enumerate(
            (
                link.length + link.tip_mount_length * 0.34,
                link.length + link.tip_mount_length * 0.78,
            ),
            start=1,
        ):
            _add_cylinder_z_visual(
                part,
                radius=screw_radius,
                length=screw_height,
                center_xyz=(x_pos, 0.0, toe_height / 2.0),
                material="fastener_black",
                name="tip_screws" if index == 1 else f"{name}_tip_screw_{index}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_finger_manipulator_core")

    model.material("palm_finish", rgba=(0.24, 0.27, 0.31, 1.0))
    model.material("link_alloy", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("fastener_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("pad_insert", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("nib_insert", rgba=(0.20, 0.21, 0.23, 1.0))

    _add_palm_visuals(model)
    palm = model.get_part("palm")

    _add_link_part(
        model,
        name="left_base",
        finger=LEFT_FINGER,
        link=LEFT_FINGER.base,
        has_distal_joint=True,
        root_style="center",
        distal_style="clevis",
    )
    _add_link_part(
        model,
        name="left_middle",
        finger=LEFT_FINGER,
        link=LEFT_FINGER.middle,
        has_distal_joint=True,
        root_style="center",
        distal_style="clevis",
    )
    _add_link_part(
        model,
        name="left_distal",
        finger=LEFT_FINGER,
        link=LEFT_FINGER.distal,
        has_distal_joint=False,
        root_style="center",
        distal_style="none",
    )
    _add_link_part(
        model,
        name="right_base",
        finger=RIGHT_FINGER,
        link=RIGHT_FINGER.base,
        has_distal_joint=True,
        root_style="center",
        distal_style="clevis",
    )
    _add_link_part(
        model,
        name="right_middle",
        finger=RIGHT_FINGER,
        link=RIGHT_FINGER.middle,
        has_distal_joint=True,
        root_style="center",
        distal_style="clevis",
    )
    _add_link_part(
        model,
        name="right_distal",
        finger=RIGHT_FINGER,
        link=RIGHT_FINGER.distal,
        has_distal_joint=False,
        root_style="center",
        distal_style="none",
    )

    model.articulation(
        "palm_to_left_base",
        ArticulationType.REVOLUTE,
        parent=palm,
        child="left_base",
        origin=Origin(xyz=(LEFT_FINGER.joint_axis_x, LEFT_FINGER.y_offset, LEFT_FINGER.joint_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=18.0, velocity=2.8),
    )
    model.articulation(
        "left_base_to_left_middle",
        ArticulationType.REVOLUTE,
        parent="left_base",
        child="left_middle",
        origin=Origin(xyz=(LEFT_FINGER.base.length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.18, effort=12.0, velocity=3.2),
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent="left_middle",
        child="left_distal",
        origin=Origin(xyz=(LEFT_FINGER.middle.length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.12, effort=9.0, velocity=3.2),
    )

    model.articulation(
        "palm_to_right_base",
        ArticulationType.REVOLUTE,
        parent=palm,
        child="right_base",
        origin=Origin(xyz=(RIGHT_FINGER.joint_axis_x, RIGHT_FINGER.y_offset, RIGHT_FINGER.joint_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.00, effort=16.0, velocity=2.8),
    )
    model.articulation(
        "right_base_to_right_middle",
        ArticulationType.REVOLUTE,
        parent="right_base",
        child="right_middle",
        origin=Origin(xyz=(RIGHT_FINGER.base.length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.12, effort=10.0, velocity=3.2),
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent="right_middle",
        child="right_distal",
        origin=Origin(xyz=(RIGHT_FINGER.middle.length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.08, effort=8.0, velocity=3.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")
    left_base = object_model.get_part("left_base")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_base = object_model.get_part("right_base")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    left_pad = left_distal.get_visual("tip_insert")
    right_nib = right_distal.get_visual("tip_insert")

    left_root_joint = object_model.get_articulation("palm_to_left_base")
    left_mid_joint = object_model.get_articulation("left_base_to_left_middle")
    left_tip_joint = object_model.get_articulation("left_middle_to_left_distal")
    right_root_joint = object_model.get_articulation("palm_to_right_base")
    right_mid_joint = object_model.get_articulation("right_base_to_right_middle")
    right_tip_joint = object_model.get_articulation("right_middle_to_right_distal")

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

    for part_name in (
        "palm",
        "left_base",
        "left_middle",
        "left_distal",
        "right_base",
        "right_middle",
        "right_distal",
    ):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    for joint_obj in (
        left_root_joint,
        left_mid_joint,
        left_tip_joint,
        right_root_joint,
        right_mid_joint,
        right_tip_joint,
    ):
        ctx.check(
            f"{joint_obj.name}_axis_y",
            tuple(round(value, 6) for value in joint_obj.axis) == (0.0, 1.0, 0.0),
            details=f"expected a pure +Y bending axis, got {joint_obj.axis}",
        )

    ctx.expect_contact(left_base, palm, contact_tol=0.0006, name="left_base_grounded_to_palm")
    ctx.expect_contact(left_middle, left_base, contact_tol=0.0006, name="left_middle_knuckle_contact")
    ctx.expect_contact(left_distal, left_middle, contact_tol=0.0006, name="left_distal_knuckle_contact")
    ctx.expect_contact(right_base, palm, contact_tol=0.0006, name="right_base_grounded_to_palm")
    ctx.expect_contact(right_middle, right_base, contact_tol=0.0006, name="right_middle_knuckle_contact")
    ctx.expect_contact(right_distal, right_middle, contact_tol=0.0006, name="right_distal_knuckle_contact")

    ctx.expect_origin_distance(
        left_base,
        right_base,
        axes="y",
        min_dist=0.040,
        max_dist=0.048,
        name="independent_pedestal_spacing",
    )
    ctx.expect_gap(
        right_base,
        left_base,
        axis="y",
        min_gap=0.020,
        name="base_links_stay_side_by_side",
    )

    with ctx.pose(
        {
            left_root_joint: 0.58,
            left_mid_joint: 0.72,
            left_tip_joint: 0.62,
            right_root_joint: 0.52,
            right_mid_joint: 0.66,
            right_tip_joint: 0.58,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="curled_pose_no_overlap")
        ctx.expect_gap(
            left_distal,
            palm,
            axis="x",
            positive_elem=left_pad,
            min_gap=0.010,
            name="rounded_pad_stays_forward_of_palm_when_curled",
        )
        ctx.expect_gap(
            right_distal,
            palm,
            axis="x",
            positive_elem=right_nib,
            min_gap=0.008,
            name="flat_nib_stays_forward_of_palm_when_curled",
        )
        ctx.expect_gap(
            right_distal,
            left_distal,
            axis="y",
            min_gap=0.012,
            name="distal_links_remain_independent_in_curled_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
