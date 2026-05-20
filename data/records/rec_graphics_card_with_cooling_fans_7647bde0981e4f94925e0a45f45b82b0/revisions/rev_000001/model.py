from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    sweep_profile_along_spline,
)


CARD_LENGTH = 0.325
CARD_HEIGHT = 0.132
CARD_THICKNESS = 0.056


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _fan_blade_section(
    radius: float,
    center_y: float,
    center_z: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (radius, center_y - 0.95 * half_thickness, center_z - 0.54 * chord),
        (radius, center_y + 0.24 * half_thickness, center_z - 0.08 * chord),
        (radius, center_y + 0.95 * half_thickness, center_z + 0.48 * chord),
        (radius, center_y - 0.18 * half_thickness, center_z + 0.10 * chord),
    ]


def _radial_pattern(base_geom: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geom.copy().rotate_z(angle_offset + (index * math.tau / count)))
    return patterned


def _build_shroud_shell_mesh() -> MeshGeometry:
    face_t = 0.003
    back_t = 0.0025
    top_rail_y = 0.012
    bottom_rail_y = 0.014
    side_t = 0.007
    top_rail_z = 0.009
    slot_span = 0.122
    cheek_t = 0.010
    face_z = CARD_THICKNESS * 0.5 - face_t * 0.5
    back_z = -CARD_THICKNESS * 0.5 + back_t * 0.5

    outer_profile = rounded_rect_profile(CARD_LENGTH, CARD_HEIGHT, 0.018, corner_segments=8)
    fan_hole = superellipse_profile(0.083, 0.083, exponent=2.0, segments=40)
    front_face = ExtrudeWithHolesGeometry(
        outer_profile,
        [
            _translate_profile(fan_hole, -0.085, -0.003),
            _translate_profile(fan_hole, 0.067, -0.003),
        ],
        height=face_t,
        center=True,
    ).translate(0.0, 0.0, face_z)

    backplate = BoxGeometry((CARD_LENGTH - 0.006, CARD_HEIGHT - 0.004, back_t)).translate(0.0, -0.002, back_z)
    bottom_tray = BoxGeometry((CARD_LENGTH - 0.010, bottom_rail_y, CARD_THICKNESS - 0.010)).translate(
        0.0,
        -CARD_HEIGHT * 0.5 + bottom_rail_y * 0.5,
        0.0,
    )
    left_wall = BoxGeometry((side_t, CARD_HEIGHT - 0.020, CARD_THICKNESS - 0.010)).translate(
        -CARD_LENGTH * 0.5 + side_t * 0.5 + 0.001,
        -0.001,
        0.0,
    )
    top_front_rail = BoxGeometry((CARD_LENGTH - 0.012, top_rail_y, top_rail_z)).translate(
        0.0,
        CARD_HEIGHT * 0.5 - top_rail_y * 0.5,
        0.0200,
    )
    top_rear_rail = BoxGeometry((CARD_LENGTH - 0.012, top_rail_y, top_rail_z)).translate(
        0.0,
        CARD_HEIGHT * 0.5 - top_rail_y * 0.5,
        -0.0200,
    )
    left_cheek = BoxGeometry((cheek_t, 0.010, 0.034)).translate(
        -slot_span * 0.5,
        CARD_HEIGHT * 0.5 - 0.010 * 0.5,
        0.0,
    )
    right_cheek = BoxGeometry((cheek_t, 0.010, 0.034)).translate(
        slot_span * 0.5,
        CARD_HEIGHT * 0.5 - 0.010 * 0.5,
        0.0,
    )

    right_frame_outer = rounded_rect_profile(CARD_THICKNESS - 0.006, CARD_HEIGHT - 0.014, 0.010, corner_segments=8)
    right_frame_inner = rounded_rect_profile(CARD_THICKNESS - 0.024, CARD_HEIGHT * 0.56, 0.006, corner_segments=6)
    right_end_frame = (
        ExtrudeWithHolesGeometry(
            right_frame_outer,
            [right_frame_inner],
            height=0.010,
            center=True,
        )
        .rotate_y(math.pi / 2.0)
        .translate(CARD_LENGTH * 0.5 - 0.006, 0.0, 0.0)
    )

    return _merge_geometries(
        [
            front_face,
            backplate,
            bottom_tray,
            left_wall,
            top_front_rail,
            top_rear_rail,
            left_cheek,
            right_cheek,
            right_end_frame,
        ]
    )


def _build_fan_rotor_mesh(blade_count: int = 11) -> MeshGeometry:
    hub = CylinderGeometry(radius=0.016, height=0.010, radial_segments=32)
    cap = CylinderGeometry(radius=0.009, height=0.014, radial_segments=28).translate(0.0, 0.0, 0.0015)
    rear_collar = CylinderGeometry(radius=0.020, height=0.004, radial_segments=32).translate(0.0, 0.0, -0.004)

    blade = repair_loft(
        section_loft(
            [
                _fan_blade_section(0.018, -0.0035, -0.0060, 0.019, 0.0030),
                _fan_blade_section(0.026, -0.0020, -0.0010, 0.024, 0.0032),
                _fan_blade_section(0.034, 0.0008, 0.0032, 0.021, 0.0028),
                _fan_blade_section(0.038, 0.0026, 0.0054, 0.014, 0.0020),
            ]
        )
    )
    blades = _radial_pattern(blade, blade_count, angle_offset=math.pi / blade_count)
    return _merge_geometries([hub, cap, rear_collar, blades])


def _build_handle_mesh() -> MeshGeometry:
    span = 0.108
    profile = rounded_rect_profile(0.009, 0.0055, 0.0020, corner_segments=5)
    handle_path = [
        (-span * 0.5, 0.0010, 0.0000),
        (-span * 0.5, 0.0032, 0.0035),
        (-0.038, 0.0050, 0.0085),
        (-0.014, 0.0062, 0.0125),
        (0.014, 0.0062, 0.0125),
        (0.038, 0.0050, 0.0085),
        (span * 0.5, 0.0032, 0.0035),
        (span * 0.5, 0.0010, 0.0000),
    ]
    strap = sweep_profile_along_spline(
        handle_path,
        profile=profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    left_pivot = CylinderGeometry(radius=0.0045, height=0.010, radial_segments=20).rotate_y(math.pi / 2.0).translate(
        -span * 0.5,
        0.0005,
        0.0000,
    )
    right_pivot = CylinderGeometry(radius=0.0045, height=0.010, radial_segments=20).rotate_y(math.pi / 2.0).translate(
        span * 0.5,
        0.0005,
        0.0000,
    )
    return _merge_geometries([strap, left_pivot, right_pivot])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pass_through_gpu")

    shroud_black = model.material("shroud_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.21, 0.23, 0.25, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.67, 0.69, 0.72, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.10, 0.26, 0.16, 1.0))
    fin_aluminum = model.material("fin_aluminum", rgba=(0.55, 0.58, 0.61, 1.0))
    fan_black = model.material("fan_black", rgba=(0.06, 0.06, 0.07, 1.0))
    handle_black = model.material("handle_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("gpu_body")
    body.visual(
        _save_mesh("gpu_shroud_shell", _build_shroud_shell_mesh()),
        material=shroud_black,
        name="shroud_shell",
    )
    body.visual(
        Box((0.128, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, CARD_HEIGHT * 0.5 - 0.0185, 0.0)),
        material=satin_graphite,
        name="handle_recess_floor",
    )
    body.visual(
        Box((0.010, 0.012, 0.032)),
        origin=Origin(xyz=(-0.061, CARD_HEIGHT * 0.5 - 0.0155, 0.0)),
        material=satin_graphite,
        name="left_handle_support",
    )
    body.visual(
        Box((0.010, 0.012, 0.032)),
        origin=Origin(xyz=(0.061, CARD_HEIGHT * 0.5 - 0.0155, 0.0)),
        material=satin_graphite,
        name="right_handle_support",
    )
    body.visual(
        Box((CARD_LENGTH - 0.016, 0.012, 0.009)),
        origin=Origin(xyz=(0.0, CARD_HEIGHT * 0.5 - 0.006, 0.0200)),
        material=satin_graphite,
        name="top_front_rail",
    )
    body.visual(
        Box((CARD_LENGTH - 0.016, 0.012, 0.009)),
        origin=Origin(xyz=(0.0, CARD_HEIGHT * 0.5 - 0.006, -0.0200)),
        material=satin_graphite,
        name="top_rear_rail",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(-0.085, -0.003, -0.011)),
        material=satin_graphite,
        name="front_motor_can",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.067, -0.003, -0.011)),
        material=satin_graphite,
        name="rear_motor_can",
    )
    body.visual(
        Box((0.004, CARD_HEIGHT + 0.010, CARD_THICKNESS - 0.004)),
        origin=Origin(xyz=(-CARD_LENGTH * 0.5 - 0.002, -0.001, 0.0)),
        material=bracket_metal,
        name="io_bracket",
    )
    body.visual(
        Box((CARD_LENGTH - 0.070, 0.100, 0.002)),
        origin=Origin(xyz=(0.016, -0.010, -CARD_THICKNESS * 0.5 + 0.0045)),
        material=pcb_green,
        name="pcb_edge",
    )
    body.visual(
        Box((0.100, 0.090, 0.012)),
        origin=Origin(xyz=(0.089, -0.006, -0.020)),
        material=fin_aluminum,
        name="rear_fin_stack",
    )
    body.visual(
        Box((0.078, 0.008, 0.003)),
        origin=Origin(xyz=(-0.044, -CARD_HEIGHT * 0.5 - 0.003, -CARD_THICKNESS * 0.5 + 0.0035)),
        material=bracket_metal,
        name="pcie_fingers",
    )
    body.inertial = Inertial.from_geometry(
        Box((CARD_LENGTH, CARD_HEIGHT, CARD_THICKNESS)),
        mass=1.35,
    )

    front_fan = model.part("front_fan_rotor")
    front_fan.visual(
        _save_mesh("gpu_front_fan_rotor", _build_fan_rotor_mesh()),
        material=fan_black,
        name="fan_rotor",
    )
    front_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.016),
        mass=0.06,
    )

    rear_fan = model.part("rear_fan_rotor")
    rear_fan.visual(
        _save_mesh("gpu_rear_fan_rotor", _build_fan_rotor_mesh()),
        material=fan_black,
        name="fan_rotor",
    )
    rear_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.016),
        mass=0.06,
    )

    carry_handle = model.part("carry_handle")
    carry_handle.visual(
        _save_mesh("gpu_carry_handle", _build_handle_mesh()),
        material=handle_black,
        name="carry_grip",
    )
    carry_handle.inertial = Inertial.from_geometry(
        Box((0.120, 0.020, 0.030)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.004, 0.014)),
    )

    model.articulation(
        "front_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_fan,
        origin=Origin(xyz=(-0.085, -0.003, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=55.0),
    )
    model.articulation(
        "rear_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_fan,
        origin=Origin(xyz=(0.067, -0.003, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=55.0),
    )
    model.articulation(
        "handle_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carry_handle,
        origin=Origin(xyz=(0.0, CARD_HEIGHT * 0.5 - 0.0105, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("gpu_body")
    front_fan = object_model.get_part("front_fan_rotor")
    rear_fan = object_model.get_part("rear_fan_rotor")
    carry_handle = object_model.get_part("carry_handle")
    front_spin = object_model.get_articulation("front_fan_spin")
    rear_spin = object_model.get_articulation("rear_fan_spin")
    handle_fold = object_model.get_articulation("handle_fold")

    ctx.check("gpu body part exists", body is not None)
    ctx.check("front fan rotor part exists", front_fan is not None)
    ctx.check("rear fan rotor part exists", rear_fan is not None)
    ctx.check("carry handle part exists", carry_handle is not None)

    ctx.check(
        "fan joints use continuous spin about broad-face normal",
        front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(front_spin.axis) == (0.0, 0.0, 1.0)
        and tuple(rear_spin.axis) == (0.0, 0.0, 1.0),
        details=f"front={front_spin.articulation_type, front_spin.axis}, rear={rear_spin.articulation_type, rear_spin.axis}",
    )
    ctx.check(
        "carry handle folds on top-edge hinge",
        handle_fold.articulation_type == ArticulationType.REVOLUTE
        and tuple(handle_fold.axis) == (-1.0, 0.0, 0.0)
        and handle_fold.motion_limits is not None
        and handle_fold.motion_limits.lower == 0.0
        and handle_fold.motion_limits.upper is not None
        and handle_fold.motion_limits.upper >= 1.1,
        details=f"axis={handle_fold.axis}, limits={handle_fold.motion_limits}",
    )

    ctx.expect_origin_gap(
        rear_fan,
        front_fan,
        axis="x",
        min_gap=0.14,
        max_gap=0.17,
        name="two fan hubs are spaced across the long shroud",
    )

    with ctx.pose({handle_fold: 0.0}):
        ctx.expect_gap(
            carry_handle,
            body,
            axis="y",
            positive_elem="carry_grip",
            negative_elem="handle_recess_floor",
            min_gap=0.0003,
            max_gap=0.0100,
            name="folded handle sits just above the recess floor",
        )

    closed_handle_aabb = None
    open_handle_aabb = None
    top_front_aabb = None
    with ctx.pose({handle_fold: 0.0}):
        closed_handle_aabb = ctx.part_element_world_aabb(carry_handle, elem="carry_grip")
        top_front_aabb = ctx.part_element_world_aabb(body, elem="top_front_rail")
    with ctx.pose({handle_fold: 1.20}):
        open_handle_aabb = ctx.part_element_world_aabb(carry_handle, elem="carry_grip")

    closed_top = closed_handle_aabb[1][1] if closed_handle_aabb is not None else None
    open_top = open_handle_aabb[1][1] if open_handle_aabb is not None else None
    spine_top = top_front_aabb[1][1] if top_front_aabb is not None else None
    ctx.check(
        "opened handle lifts above the top spine",
        closed_top is not None
        and open_top is not None
        and spine_top is not None
        and open_top > spine_top + 0.008
        and open_top > closed_top + 0.006,
        details=f"closed_top={closed_top}, open_top={open_top}, spine_top={spine_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
