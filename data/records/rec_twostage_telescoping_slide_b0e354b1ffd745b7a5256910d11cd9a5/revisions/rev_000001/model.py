from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


OUTER_LENGTH = 0.400
OUTER_WIDTH = 0.042
OUTER_HEIGHT = 0.014
OUTER_BASE_T = 0.0014
OUTER_WALL_T = 0.0018
OUTER_LIP_W = 0.0032
OUTER_LIP_T = 0.0010

MIDDLE_LENGTH = 0.330
MIDDLE_WIDTH = 0.030
MIDDLE_HEIGHT = 0.0105
MIDDLE_BASE_T = 0.0012
MIDDLE_WALL_T = 0.0016
MIDDLE_LIP_W = 0.0020
MIDDLE_LIP_T = 0.0009

INNER_LENGTH = 0.275
INNER_WIDTH = 0.0185
INNER_HEIGHT = 0.0068
INNER_CORE_HEIGHT = 0.0050
INNER_CROWN_WIDTH = 0.0090

PLATE_LENGTH = 0.055
PLATE_REAR_LENGTH = 0.020
PLATE_WIDTH = 0.0215
PLATE_REAR_WIDTH = 0.0188
PLATE_HEIGHT = 0.0042

GROUND_TO_MIDDLE_HOME = 0.028
GROUND_TO_MIDDLE_Z = 0.0018
GROUND_TO_MIDDLE_TRAVEL = 0.155

MIDDLE_TO_INNER_HOME = 0.022
MIDDLE_TO_INNER_Z = MIDDLE_BASE_T
MIDDLE_TO_INNER_TRAVEL = 0.125

INNER_TO_PLATE_Z = (INNER_HEIGHT - PLATE_HEIGHT) / 2.0
INNER_TONGUE_END = 0.350


def _box_from_min(dx: float, dy: float, dz: float, x0: float, y0: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").box(dx, dy, dz).translate((x0 + (dx / 2.0), y0 + (dy / 2.0), z0 + (dz / 2.0)))


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _top_socket_cap_screw(head_d: float, head_h: float, shaft_d: float, shaft_l: float) -> cq.Workplane:
    screw = cq.Workplane("XY").circle(head_d / 2.0).extrude(head_h)
    screw = screw.union(cq.Workplane("XY").circle(shaft_d / 2.0).extrude(-shaft_l))
    screw = (
        screw.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .polygon(6, head_d * 0.42)
        .cutBlind(-(head_h * 0.58))
    )
    return screw


def _side_socket_cap_screw(head_d: float, head_h: float, shaft_d: float, shaft_l: float) -> cq.Workplane:
    screw = cq.Workplane("XZ").circle(head_d / 2.0).extrude(head_h)
    screw = screw.union(cq.Workplane("XZ").circle(shaft_d / 2.0).extrude(-shaft_l))
    screw = (
        screw.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .polygon(6, head_d * 0.42)
        .cutBlind(-(head_h * 0.58))
    )
    return screw


def _side_slot_cut(slot_length: float, slot_height: float, depth: float, x_center: float, z_center: float, y_start: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_center, z_center)
        .slot2D(slot_length, slot_height)
        .extrude(depth)
        .translate((0.0, y_start, 0.0))
    )


def _channel_shape(
    *,
    length: float,
    width: float,
    height: float,
    base_t: float,
    wall_t: float,
    lip_w: float,
    lip_t: float,
    pocket_xs: tuple[float, ...],
    pocket_length: float,
    pocket_height: float,
    pocket_depth: float,
) -> cq.Workplane:
    shell = _union_all(
        [
            _box_from_min(length, width, base_t, 0.0, -width / 2.0, 0.0),
            _box_from_min(length, wall_t, height - base_t, 0.0, -width / 2.0, base_t),
            _box_from_min(length, wall_t, height - base_t, 0.0, (width / 2.0) - wall_t, base_t),
            _box_from_min(length, lip_w, lip_t, 0.0, -width / 2.0 + wall_t, height - lip_t),
            _box_from_min(length, lip_w, lip_t, 0.0, (width / 2.0) - wall_t - lip_w, height - lip_t),
        ]
    )

    for x_center in pocket_xs:
        shell = shell.cut(
            _side_slot_cut(
                pocket_length,
                pocket_height,
                pocket_depth,
                x_center=x_center,
                z_center=height * 0.54,
                y_start=(width / 2.0) - pocket_depth,
            )
        )
        shell = shell.cut(
            _side_slot_cut(
                pocket_length,
                pocket_height,
                pocket_depth,
                x_center=x_center,
                z_center=height * 0.54,
                y_start=-(width / 2.0),
            )
        )

    return shell


def _build_ground_channel() -> cq.Workplane:
    channel = _channel_shape(
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        base_t=OUTER_BASE_T,
        wall_t=OUTER_WALL_T,
        lip_w=OUTER_LIP_W,
        lip_t=OUTER_LIP_T,
        pocket_xs=(0.105, 0.288),
        pocket_length=0.032,
        pocket_height=0.0048,
        pocket_depth=0.00105,
    )

    channel = channel.union(_box_from_min(0.016, 0.013, 0.0009, 0.010, -0.0065, OUTER_HEIGHT - 0.0022))
    channel = channel.union(_box_from_min(0.018, 0.013, 0.0009, OUTER_LENGTH - 0.032, -0.0065, OUTER_HEIGHT - 0.0022))

    side_screw = _side_socket_cap_screw(head_d=0.0054, head_h=0.0019, shaft_d=0.0033, shaft_l=0.0014)
    for x_pos in (0.060, 0.340):
        channel = channel.union(side_screw.translate((x_pos, OUTER_WIDTH / 2.0, 0.0080)))
        channel = channel.union(side_screw.mirror("XZ").translate((x_pos, -OUTER_WIDTH / 2.0, 0.0080)))

    return channel


def _build_middle_channel() -> cq.Workplane:
    channel = _channel_shape(
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        base_t=MIDDLE_BASE_T,
        wall_t=MIDDLE_WALL_T,
        lip_w=MIDDLE_LIP_W,
        lip_t=MIDDLE_LIP_T,
        pocket_xs=(0.092, 0.238),
        pocket_length=0.024,
        pocket_height=0.0040,
        pocket_depth=0.00095,
    )

    channel = channel.union(_box_from_min(0.012, 0.010, 0.0008, 0.014, -0.005, MIDDLE_HEIGHT - 0.0019))
    channel = channel.union(_box_from_min(0.014, 0.010, 0.0008, MIDDLE_LENGTH - 0.026, -0.005, MIDDLE_HEIGHT - 0.0019))
    return channel


def _build_inner_slide() -> cq.Workplane:
    slide = _union_all(
        [
            _box_from_min(INNER_LENGTH, INNER_WIDTH, INNER_CORE_HEIGHT, 0.0, -INNER_WIDTH / 2.0, 0.0),
            _box_from_min(
                INNER_LENGTH * 0.90,
                INNER_CROWN_WIDTH,
                INNER_HEIGHT - INNER_CORE_HEIGHT,
                INNER_LENGTH * 0.05,
                -INNER_CROWN_WIDTH / 2.0,
                INNER_CORE_HEIGHT,
            ),
            _box_from_min(0.032, INNER_WIDTH * 0.92, 0.0012, INNER_LENGTH - 0.038, -(INNER_WIDTH * 0.46), INNER_CORE_HEIGHT),
        ]
    )

    for x_center in (0.080, 0.192):
        slide = slide.cut(
            _side_slot_cut(
                0.018,
                0.0032,
                0.00085,
                x_center=x_center,
                z_center=INNER_CORE_HEIGHT * 0.78,
                y_start=(INNER_WIDTH / 2.0) - 0.00085,
            )
        )
        slide = slide.cut(
            _side_slot_cut(
                0.018,
                0.0032,
                0.00085,
                x_center=x_center,
                z_center=INNER_CORE_HEIGHT * 0.78,
                y_start=-(INNER_WIDTH / 2.0),
            )
        )

    return slide


def _build_equipment_plate() -> cq.Workplane:
    plate = _union_all(
        [
            _box_from_min(PLATE_REAR_LENGTH, PLATE_REAR_WIDTH, PLATE_HEIGHT, 0.0, -PLATE_REAR_WIDTH / 2.0, 0.0),
            _box_from_min(
                PLATE_LENGTH - PLATE_REAR_LENGTH,
                PLATE_WIDTH,
                PLATE_HEIGHT,
                PLATE_REAR_LENGTH,
                -PLATE_WIDTH / 2.0,
                0.0,
            ),
        ]
    )

    slot_cutter = (
        cq.Workplane("XY")
        .center(0.038, 0.0061)
        .slot2D(0.013, 0.0032)
        .extrude(PLATE_HEIGHT + 0.0005)
    )
    plate = plate.cut(slot_cutter)
    plate = plate.cut(slot_cutter.mirror("XZ"))

    top_screw = _top_socket_cap_screw(head_d=0.0042, head_h=0.00155, shaft_d=0.0026, shaft_l=0.0019)
    plate = plate.union(top_screw.translate((0.010, 0.0053, PLATE_HEIGHT)))
    plate = plate.union(top_screw.translate((0.010, -0.0053, PLATE_HEIGHT)))

    return plate


def _add_mesh_proxy_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
    box_size: tuple[float, float, float],
    mass: float,
) -> object:
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name=f"{name}_body")
    part.inertial = Inertial.from_geometry(
        Box(box_size),
        mass=mass,
        origin=Origin(xyz=(box_size[0] / 2.0, 0.0, box_size[2] / 2.0)),
    )
    return part


def _add_box_visual(
    part: object,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder_visual(
    part: object,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _build_ground_channel_part(model: ArticulatedObject) -> object:
    part = model.part("ground_channel")

    _add_box_visual(
        part,
        name="ground_base",
        size=(OUTER_LENGTH, OUTER_WIDTH, OUTER_BASE_T),
        xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_BASE_T / 2.0),
        material="ground_steel",
    )
    wall_height = OUTER_HEIGHT - OUTER_BASE_T
    wall_z = OUTER_BASE_T + (wall_height / 2.0)
    _add_box_visual(
        part,
        name="left_wall",
        size=(OUTER_LENGTH, OUTER_WALL_T, wall_height),
        xyz=(OUTER_LENGTH / 2.0, -OUTER_WIDTH / 2.0 + OUTER_WALL_T / 2.0, wall_z),
        material="ground_steel",
    )
    _add_box_visual(
        part,
        name="right_wall",
        size=(OUTER_LENGTH, OUTER_WALL_T, wall_height),
        xyz=(OUTER_LENGTH / 2.0, OUTER_WIDTH / 2.0 - OUTER_WALL_T / 2.0, wall_z),
        material="ground_steel",
    )
    _add_box_visual(
        part,
        name="left_lip",
        size=(OUTER_LENGTH, OUTER_LIP_W, OUTER_LIP_T),
        xyz=(OUTER_LENGTH / 2.0, -OUTER_WIDTH / 2.0 + OUTER_WALL_T + OUTER_LIP_W / 2.0, OUTER_HEIGHT - OUTER_LIP_T / 2.0),
        material="ground_steel",
    )
    _add_box_visual(
        part,
        name="right_lip",
        size=(OUTER_LENGTH, OUTER_LIP_W, OUTER_LIP_T),
        xyz=(OUTER_LENGTH / 2.0, OUTER_WIDTH / 2.0 - OUTER_WALL_T - OUTER_LIP_W / 2.0, OUTER_HEIGHT - OUTER_LIP_T / 2.0),
        material="ground_steel",
    )
    _add_box_visual(
        part,
        name="rear_stop",
        size=(0.016, 0.034, 0.0020),
        xyz=(0.018, 0.0, 0.0122),
        material="ground_steel",
    )
    _add_box_visual(
        part,
        name="front_stop",
        size=(0.018, 0.034, 0.0020),
        xyz=(OUTER_LENGTH - 0.023, 0.0, 0.0122),
        material="ground_steel",
    )

    for idx, x_pos in enumerate((0.105, 0.288), start=1):
        _add_box_visual(
            part,
            name=f"pocket_cover_pos_{idx}",
            size=(0.032, 0.0008, 0.0056),
            xyz=(x_pos, OUTER_WIDTH / 2.0 + 0.0004, 0.0070),
            material="ground_steel",
        )
        _add_box_visual(
            part,
            name=f"pocket_cover_neg_{idx}",
            size=(0.032, 0.0008, 0.0056),
            xyz=(x_pos, -(OUTER_WIDTH / 2.0 + 0.0004), 0.0070),
            material="ground_steel",
        )

    for idx, x_pos in enumerate((0.060, 0.340), start=1):
        _add_cylinder_visual(
            part,
            name=f"side_screw_head_pos_{idx}",
            radius=0.0027,
            length=0.0019,
            xyz=(x_pos, OUTER_WIDTH / 2.0 + 0.00095, 0.0080),
            rpy=(pi / 2.0, 0.0, 0.0),
            material="ground_steel",
        )
        _add_cylinder_visual(
            part,
            name=f"side_screw_shank_pos_{idx}",
            radius=0.00165,
            length=0.0014,
            xyz=(x_pos, OUTER_WIDTH / 2.0 - 0.00070, 0.0080),
            rpy=(pi / 2.0, 0.0, 0.0),
            material="ground_steel",
        )
        _add_cylinder_visual(
            part,
            name=f"side_screw_head_neg_{idx}",
            radius=0.0027,
            length=0.0019,
            xyz=(x_pos, -(OUTER_WIDTH / 2.0 + 0.00095), 0.0080),
            rpy=(pi / 2.0, 0.0, 0.0),
            material="ground_steel",
        )
        _add_cylinder_visual(
            part,
            name=f"side_screw_shank_neg_{idx}",
            radius=0.00165,
            length=0.0014,
            xyz=(x_pos, -(OUTER_WIDTH / 2.0 - 0.00070), 0.0080),
            rpy=(pi / 2.0, 0.0, 0.0),
            material="ground_steel",
        )

    part.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=0.62,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_HEIGHT / 2.0)),
    )
    return part


def _build_middle_channel_part(model: ArticulatedObject) -> object:
    part = model.part("middle_channel")

    _add_box_visual(
        part,
        name="middle_base",
        size=(MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_BASE_T),
        xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_BASE_T / 2.0),
        material="middle_steel",
    )
    wall_height = MIDDLE_HEIGHT - MIDDLE_BASE_T
    wall_z = MIDDLE_BASE_T + (wall_height / 2.0)
    _add_box_visual(
        part,
        name="middle_left_wall",
        size=(MIDDLE_LENGTH, MIDDLE_WALL_T, wall_height),
        xyz=(MIDDLE_LENGTH / 2.0, -MIDDLE_WIDTH / 2.0 + MIDDLE_WALL_T / 2.0, wall_z),
        material="middle_steel",
    )
    _add_box_visual(
        part,
        name="middle_right_wall",
        size=(MIDDLE_LENGTH, MIDDLE_WALL_T, wall_height),
        xyz=(MIDDLE_LENGTH / 2.0, MIDDLE_WIDTH / 2.0 - MIDDLE_WALL_T / 2.0, wall_z),
        material="middle_steel",
    )
    _add_box_visual(
        part,
        name="middle_left_lip",
        size=(MIDDLE_LENGTH, MIDDLE_LIP_W, MIDDLE_LIP_T),
        xyz=(MIDDLE_LENGTH / 2.0, -MIDDLE_WIDTH / 2.0 + MIDDLE_WALL_T + MIDDLE_LIP_W / 2.0, MIDDLE_HEIGHT - MIDDLE_LIP_T / 2.0),
        material="middle_steel",
    )
    _add_box_visual(
        part,
        name="middle_right_lip",
        size=(MIDDLE_LENGTH, MIDDLE_LIP_W, MIDDLE_LIP_T),
        xyz=(MIDDLE_LENGTH / 2.0, MIDDLE_WIDTH / 2.0 - MIDDLE_WALL_T - MIDDLE_LIP_W / 2.0, MIDDLE_HEIGHT - MIDDLE_LIP_T / 2.0),
        material="middle_steel",
    )
    _add_box_visual(
        part,
        name="wear_strip_left",
        size=(0.260, 0.0036, GROUND_TO_MIDDLE_Z - OUTER_BASE_T),
        xyz=(MIDDLE_LENGTH / 2.0, -0.0102, -(GROUND_TO_MIDDLE_Z - OUTER_BASE_T) / 2.0),
        material="middle_steel",
    )
    _add_box_visual(
        part,
        name="wear_strip_right",
        size=(0.260, 0.0036, GROUND_TO_MIDDLE_Z - OUTER_BASE_T),
        xyz=(MIDDLE_LENGTH / 2.0, 0.0102, -(GROUND_TO_MIDDLE_Z - OUTER_BASE_T) / 2.0),
        material="middle_steel",
    )
    _add_box_visual(
        part,
        name="middle_rear_stop",
        size=(0.012, 0.0272, 0.0020),
        xyz=(0.020, 0.0, 0.0095),
        material="middle_steel",
    )
    _add_box_visual(
        part,
        name="middle_front_stop",
        size=(0.014, 0.0272, 0.0020),
        xyz=(MIDDLE_LENGTH - 0.019, 0.0, 0.0095),
        material="middle_steel",
    )

    for idx, x_pos in enumerate((0.092, 0.238), start=1):
        _add_box_visual(
            part,
            name=f"middle_pocket_pos_{idx}",
            size=(0.024, 0.0007, 0.0048),
            xyz=(x_pos, MIDDLE_WIDTH / 2.0 + 0.00035, 0.0056),
            material="middle_steel",
        )
        _add_box_visual(
            part,
            name=f"middle_pocket_neg_{idx}",
            size=(0.024, 0.0007, 0.0048),
            xyz=(x_pos, -(MIDDLE_WIDTH / 2.0 + 0.00035), 0.0056),
            material="middle_steel",
        )

    part.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=0.34,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT / 2.0)),
    )
    return part


def _build_inner_slide_part(model: ArticulatedObject) -> object:
    part = model.part("inner_slide")

    _add_box_visual(
        part,
        name="runner_left",
        size=(0.245, 0.0032, 0.0013),
        xyz=(0.1375, -0.0062, 0.00065),
        material="inner_steel",
    )
    _add_box_visual(
        part,
        name="runner_right",
        size=(0.245, 0.0032, 0.0013),
        xyz=(0.1375, 0.0062, 0.00065),
        material="inner_steel",
    )
    _add_box_visual(
        part,
        name="inner_web",
        size=(INNER_LENGTH, 0.0092, 0.0046),
        xyz=(INNER_LENGTH / 2.0, 0.0, 0.0036),
        material="inner_steel",
    )
    _add_box_visual(
        part,
        name="inner_crown",
        size=(0.220, 0.0064, 0.0019),
        xyz=(0.132, 0.0, 0.00555),
        material="inner_steel",
    )
    _add_box_visual(
        part,
        name="front_mount",
        size=(0.046, 0.0120, 0.0016),
        xyz=(0.252, 0.0, 0.00540),
        material="inner_steel",
    )

    part.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        mass=0.18,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0)),
    )
    return part


def _build_equipment_plate_part(model: ArticulatedObject) -> object:
    part = model.part("equipment_plate")

    _add_box_visual(
        part,
        name="rear_tongue",
        size=(0.020, 0.0165, 0.0020),
        xyz=(0.010, 0.0, 0.0010),
        material="equipment_plate_black",
    )
    _add_box_visual(
        part,
        name="deck",
        size=(0.041, PLATE_WIDTH, 0.0030),
        xyz=(0.0345, 0.0, 0.0035),
        material="equipment_plate_black",
    )
    _add_box_visual(
        part,
        name="mount_pad",
        size=(0.014, 0.0180, 0.0016),
        xyz=(0.013, 0.0, 0.0028),
        material="equipment_plate_black",
    )

    for idx, y_pos in enumerate((0.0053, -0.0053), start=1):
        _add_cylinder_visual(
            part,
            name=f"plate_screw_head_{idx}",
            radius=0.0021,
            length=0.0010,
            xyz=(0.020, y_pos, 0.0055),
            rpy=(0.0, 0.0, 0.0),
            material="ground_steel",
        )
        _add_cylinder_visual(
            part,
            name=f"plate_screw_shank_{idx}",
            radius=0.0013,
            length=0.0014,
            xyz=(0.020, y_pos, 0.0048),
            rpy=(0.0, 0.0, 0.0),
            material="ground_steel",
        )

    part.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, 0.0060)),
        mass=0.08,
        origin=Origin(xyz=(PLATE_LENGTH / 2.0, 0.0, 0.0030)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_drawer_slide_unit")

    model.material("ground_steel", rgba=(0.40, 0.43, 0.47, 1.0))
    model.material("middle_steel", rgba=(0.58, 0.60, 0.64, 1.0))
    model.material("inner_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    model.material("equipment_plate_black", rgba=(0.19, 0.20, 0.22, 1.0))

    ground_channel = _build_ground_channel_part(model)
    middle_channel = _build_middle_channel_part(model)
    inner_slide = _build_inner_slide_part(model)
    equipment_plate = _build_equipment_plate_part(model)

    model.articulation(
        "ground_to_middle",
        ArticulationType.PRISMATIC,
        parent=ground_channel,
        child=middle_channel,
        origin=Origin(xyz=(GROUND_TO_MIDDLE_HOME, 0.0, GROUND_TO_MIDDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.45,
            lower=0.0,
            upper=GROUND_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_channel,
        child=inner_slide,
        origin=Origin(xyz=(MIDDLE_TO_INNER_HOME, 0.0, MIDDLE_TO_INNER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=95.0,
            velocity=0.50,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_plate",
        ArticulationType.FIXED,
        parent=inner_slide,
        child=equipment_plate,
        origin=Origin(xyz=(0.275, 0.0, 0.0033)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_channel = object_model.get_part("ground_channel")
    middle_channel = object_model.get_part("middle_channel")
    inner_slide = object_model.get_part("inner_slide")
    equipment_plate = object_model.get_part("equipment_plate")
    ground_to_middle = object_model.get_articulation("ground_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "serial_prismatic_axes",
        (
            ground_to_middle.articulation_type == ArticulationType.PRISMATIC
            and middle_to_inner.articulation_type == ArticulationType.PRISMATIC
            and tuple(ground_to_middle.axis) == (1.0, 0.0, 0.0)
            and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0)
        ),
        details="Drawer slide should telescope with serial prismatic joints along +X.",
    )
    ctx.check(
        "serial_travel_limits",
        (
            ground_to_middle.motion_limits is not None
            and middle_to_inner.motion_limits is not None
            and ground_to_middle.motion_limits.lower == 0.0
            and middle_to_inner.motion_limits.lower == 0.0
            and abs((ground_to_middle.motion_limits.upper or 0.0) - GROUND_TO_MIDDLE_TRAVEL) < 1e-9
            and abs((middle_to_inner.motion_limits.upper or 0.0) - MIDDLE_TO_INNER_TRAVEL) < 1e-9
        ),
        details="Travel limits should preserve staged overlap instead of allowing full pull-out.",
    )

    with ctx.pose({ground_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_within(
            middle_channel,
            ground_channel,
            axes="yz",
            margin=0.0012,
            name="middle_channel_nests_in_ground_channel",
        )
        ctx.expect_within(
            inner_slide,
            middle_channel,
            axes="yz",
            margin=0.0010,
            name="inner_slide_nests_in_middle_channel",
        )
        ctx.expect_within(
            equipment_plate,
            middle_channel,
            axes="yz",
            margin=0.0010,
            name="equipment_plate_retracts_inside_middle_channel",
        )
        ctx.expect_overlap(
            ground_channel,
            middle_channel,
            axes="x",
            min_overlap=0.300,
            name="ground_and_middle_keep_closed_overlap",
        )
        ctx.expect_overlap(
            middle_channel,
            inner_slide,
            axes="x",
            min_overlap=0.230,
            name="middle_and_inner_keep_closed_overlap",
        )
        ctx.expect_contact(
            equipment_plate,
            inner_slide,
            name="equipment_plate_is_mounted_to_inner_slide",
        )

    with ctx.pose({ground_to_middle: GROUND_TO_MIDDLE_TRAVEL, middle_to_inner: MIDDLE_TO_INNER_TRAVEL}):
        ctx.expect_within(
            middle_channel,
            ground_channel,
            axes="yz",
            margin=0.0012,
            name="middle_channel_tracks_inside_ground_channel",
        )
        ctx.expect_within(
            inner_slide,
            middle_channel,
            axes="yz",
            margin=0.0010,
            name="inner_slide_tracks_inside_middle_channel",
        )
        ctx.expect_within(
            equipment_plate,
            middle_channel,
            axes="yz",
            margin=0.0010,
            name="equipment_plate_tracks_inside_middle_channel",
        )
        ctx.expect_overlap(
            ground_channel,
            middle_channel,
            axes="x",
            min_overlap=0.180,
            name="ground_and_middle_keep_service_overlap",
        )
        ctx.expect_overlap(
            middle_channel,
            inner_slide,
            axes="x",
            min_overlap=0.150,
            name="middle_and_inner_keep_service_overlap",
        )
        ctx.expect_gap(
            equipment_plate,
            ground_channel,
            axis="x",
            min_gap=0.020,
            name="equipment_plate_projects_beyond_ground_channel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
