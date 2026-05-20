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
    section_loft,
)


def _square_section(size: float, z: float) -> list[tuple[float, float, float]]:
    half = size * 0.5
    return [
        (-half, -half, z),
        (half, -half, z),
        (half, half, z),
        (-half, half, z),
    ]


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _dial_ring_mesh(name: str, *, outer_radius: float, inner_radius: float, thickness: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius),
            [_circle_profile(inner_radius)],
            thickness,
            center=True,
        ),
        name,
    )


def _oriented_box(
    normal_axis: str,
    *,
    thickness: float,
    width: float,
    length: float,
    z_center: float,
) -> tuple[Box, Origin]:
    if normal_axis == "x":
        return Box((thickness, width, length)), Origin(xyz=(0.0, 0.0, z_center))
    return Box((width, thickness, length)), Origin(xyz=(0.0, 0.0, z_center))


def _add_hand_geometry(part, *, normal_axis: str, hand_kind: str, material) -> None:
    if hand_kind == "minute":
        hub_radius = 0.16
        hub_length = 0.10
        body_length = 1.06
        tip_length = 0.26
        tail_length = 0.24
        body_width = 0.11
        tip_width = 0.05
        tail_width = 0.09
        thickness = 0.03
    else:
        hub_radius = 0.18
        hub_length = 0.06
        body_length = 0.76
        tip_length = 0.20
        tail_length = 0.18
        body_width = 0.15
        tip_width = 0.07
        tail_width = 0.11
        thickness = 0.035

    hub_rpy = (0.0, math.pi / 2.0, 0.0) if normal_axis == "x" else (math.pi / 2.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(rpy=hub_rpy),
        material=material,
        name=f"{hand_kind}_hub",
    )

    body_geom, body_origin = _oriented_box(
        normal_axis,
        thickness=thickness,
        width=body_width,
        length=body_length,
        z_center=0.46,
    )
    part.visual(body_geom, origin=body_origin, material=material, name=f"{hand_kind}_body")

    tip_geom, tip_origin = _oriented_box(
        normal_axis,
        thickness=thickness,
        width=tip_width,
        length=tip_length,
        z_center=1.11 if hand_kind == "minute" else 0.86,
    )
    part.visual(tip_geom, origin=tip_origin, material=material, name=f"{hand_kind}_tip")

    tail_geom, tail_origin = _oriented_box(
        normal_axis,
        thickness=thickness,
        width=tail_width,
        length=tail_length,
        z_center=-0.15 if hand_kind == "minute" else -0.11,
    )
    part.visual(tail_geom, origin=tail_origin, material=material, name=f"{hand_kind}_tail")

    approx_length = body_length + tip_length + tail_length
    part.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, approx_length)),
        mass=0.14 if hand_kind == "minute" else 0.11,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
    )


def _add_bell_louver_face(
    part,
    *,
    normal_axis: str,
    sign: float,
    outer_offset: float,
    material_dark,
    material_stone,
) -> None:
    recess_thickness = 0.14
    slat_thickness = 0.08
    recess_center = sign * (outer_offset - 0.03)
    slat_center = sign * (outer_offset + 0.05)

    if normal_axis == "x":
        part.visual(
            Box((recess_thickness, 2.35, 3.00)),
            origin=Origin(xyz=(recess_center, 0.0, 2.15)),
            material=material_dark,
        )
        for slat_z in (1.25, 1.70, 2.15, 2.60, 3.05):
            part.visual(
                Box((slat_thickness, 2.10, 0.12)),
                origin=Origin(xyz=(slat_center, 0.0, slat_z)),
                material=material_stone,
            )
        for mullion_y in (-0.75, 0.75):
            part.visual(
                Box((slat_thickness, 0.10, 2.95)),
                origin=Origin(xyz=(slat_center, mullion_y, 2.15)),
                material=material_stone,
            )
    else:
        part.visual(
            Box((2.35, recess_thickness, 3.00)),
            origin=Origin(xyz=(0.0, recess_center, 2.15)),
            material=material_dark,
        )
        for slat_z in (1.25, 1.70, 2.15, 2.60, 3.05):
            part.visual(
                Box((2.10, slat_thickness, 0.12)),
                origin=Origin(xyz=(0.0, slat_center, slat_z)),
                material=material_stone,
            )
        for mullion_x in (-0.75, 0.75):
            part.visual(
                Box((0.10, slat_thickness, 2.95)),
                origin=Origin(xyz=(mullion_x, slat_center, 2.15)),
                material=material_stone,
            )


def _extents(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return (maxs[0] - mins[0], maxs[1] - mins[1], maxs[2] - mins[2])


def _add_clock_face(
    part,
    *,
    face_name: str,
    normal_axis: str,
    sign: float,
    face_offset: float,
    dial_height: float,
    dial_radius: float,
    dial_material,
    bezel_material,
    hub_material,
) -> None:
    face_disk = Cylinder(radius=dial_radius - 0.08, length=0.03)
    bezel_mesh = _dial_ring_mesh(
        f"{face_name}_dial_bezel_mesh",
        outer_radius=dial_radius,
        inner_radius=dial_radius - 0.14,
        thickness=0.06,
    )
    marker_long = 0.30
    marker_short = 0.12
    marker_offset = dial_radius - 0.37

    if normal_axis == "x":
        disk_rpy = (0.0, math.pi / 2.0, 0.0)
        disk_xyz = (sign * (face_offset + 0.015), 0.0, dial_height)
        bezel_xyz = (sign * (face_offset + 0.06), 0.0, dial_height)
        hub_xyz = (sign * (face_offset + 0.07), 0.0, dial_height)
        depth = 0.018
        part.visual(face_disk, origin=Origin(xyz=disk_xyz, rpy=disk_rpy), material=dial_material, name=f"{face_name}_dial_plate")
        part.visual(bezel_mesh, origin=Origin(xyz=bezel_xyz, rpy=disk_rpy), material=bezel_material, name=f"{face_name}_dial_bezel")
        part.visual(
            Cylinder(radius=0.16, length=0.10),
            origin=Origin(xyz=hub_xyz, rpy=disk_rpy),
            material=hub_material,
            name=f"{face_name}_dial_hub",
        )
        for y, z, sy, sz in (
            (0.0, dial_height + marker_offset, marker_short, marker_long),
            (0.0, dial_height - marker_offset, marker_short, marker_long),
            (marker_offset, dial_height, marker_long, marker_short),
            (-marker_offset, dial_height, marker_long, marker_short),
        ):
            part.visual(
                Box((depth, sy, sz)),
                origin=Origin(xyz=(sign * (face_offset + 0.035), y, z)),
                material=hub_material,
            )
    else:
        disk_rpy = (math.pi / 2.0, 0.0, 0.0)
        disk_xyz = (0.0, sign * (face_offset + 0.015), dial_height)
        bezel_xyz = (0.0, sign * (face_offset + 0.06), dial_height)
        hub_xyz = (0.0, sign * (face_offset + 0.07), dial_height)
        depth = 0.018
        part.visual(face_disk, origin=Origin(xyz=disk_xyz, rpy=disk_rpy), material=dial_material, name=f"{face_name}_dial_plate")
        part.visual(bezel_mesh, origin=Origin(xyz=bezel_xyz, rpy=disk_rpy), material=bezel_material, name=f"{face_name}_dial_bezel")
        part.visual(
            Cylinder(radius=0.16, length=0.10),
            origin=Origin(xyz=hub_xyz, rpy=disk_rpy),
            material=hub_material,
            name=f"{face_name}_dial_hub",
        )
        for x, z, sx, sz in (
            (0.0, dial_height + marker_offset, marker_long, marker_short),
            (0.0, dial_height - marker_offset, marker_long, marker_short),
            (marker_offset, dial_height, marker_short, marker_long),
            (-marker_offset, dial_height, marker_short, marker_long),
        ):
            part.visual(
                Box((sx, depth, sz)),
                origin=Origin(xyz=(x, sign * (face_offset + 0.035), z)),
                material=hub_material,
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="market_square_clock_tower")

    stone = model.material("stone", rgba=(0.68, 0.66, 0.61, 1.0))
    stone_dark = model.material("stone_dark", rgba=(0.55, 0.53, 0.49, 1.0))
    dial_cream = model.material("dial_cream", rgba=(0.90, 0.89, 0.82, 1.0))
    bell_void = model.material("bell_void", rgba=(0.12, 0.11, 0.10, 1.0))
    roof_slate = model.material("roof_slate", rgba=(0.25, 0.36, 0.34, 1.0))
    hand_black = model.material("hand_black", rgba=(0.10, 0.09, 0.09, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    shaft = model.part("shaft")
    shaft.visual(
        Box((8.20, 8.20, 1.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=stone_dark,
        name="base_plinth",
    )
    shaft.visual(
        Box((6.40, 6.40, 0.80)),
        origin=Origin(xyz=(0.0, 0.0, 1.60)),
        material=stone,
        name="lower_step",
    )
    shaft.visual(
        Box((5.20, 5.20, 13.20)),
        origin=Origin(xyz=(0.0, 0.0, 8.40)),
        material=stone,
        name="shaft_body",
    )
    shaft.visual(
        Box((5.90, 5.90, 1.10)),
        origin=Origin(xyz=(0.0, 0.0, 15.25)),
        material=stone_dark,
        name="shaft_cornice",
    )
    for sx in (-2.35, 2.35):
        for sy in (-2.35, 2.35):
            shaft.visual(
                Box((0.42, 0.42, 13.40)),
                origin=Origin(xyz=(sx, sy, 8.50)),
                material=stone_dark,
            )
    shaft.inertial = Inertial.from_geometry(
        Box((8.20, 8.20, 15.80)),
        mass=38000.0,
        origin=Origin(xyz=(0.0, 0.0, 7.90)),
    )

    clock_stage = model.part("clock_stage")
    clock_stage.visual(
        Box((6.80, 6.80, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=stone_dark,
        name="clock_base_course",
    )
    clock_stage.visual(
        Box((6.40, 6.40, 3.60)),
        origin=Origin(xyz=(0.0, 0.0, 2.00)),
        material=stone,
        name="clock_block",
    )
    clock_stage.visual(
        Box((7.00, 7.00, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 4.00)),
        material=stone_dark,
        name="clock_cornice",
    )
    for sx in (-2.95, 2.95):
        for sy in (-2.95, 2.95):
            clock_stage.visual(
                Box((0.35, 0.35, 3.95)),
                origin=Origin(xyz=(sx, sy, 2.03)),
                material=stone_dark,
            )

    dial_radius = 1.55
    dial_half = 3.20
    dial_height = 2.00

    _add_clock_face(
        clock_stage,
        face_name="east",
        normal_axis="x",
        sign=1.0,
        face_offset=dial_half,
        dial_height=dial_height,
        dial_radius=dial_radius,
        dial_material=dial_cream,
        bezel_material=stone_dark,
        hub_material=metal_dark,
    )
    _add_clock_face(
        clock_stage,
        face_name="west",
        normal_axis="x",
        sign=-1.0,
        face_offset=dial_half,
        dial_height=dial_height,
        dial_radius=dial_radius,
        dial_material=dial_cream,
        bezel_material=stone_dark,
        hub_material=metal_dark,
    )
    _add_clock_face(
        clock_stage,
        face_name="north",
        normal_axis="y",
        sign=1.0,
        face_offset=dial_half,
        dial_height=dial_height,
        dial_radius=dial_radius,
        dial_material=dial_cream,
        bezel_material=stone_dark,
        hub_material=metal_dark,
    )
    _add_clock_face(
        clock_stage,
        face_name="south",
        normal_axis="y",
        sign=-1.0,
        face_offset=dial_half,
        dial_height=dial_height,
        dial_radius=dial_radius,
        dial_material=dial_cream,
        bezel_material=stone_dark,
        hub_material=metal_dark,
    )
    clock_stage.inertial = Inertial.from_geometry(
        Box((7.00, 7.00, 4.20)),
        mass=9000.0,
        origin=Origin(xyz=(0.0, 0.0, 2.10)),
    )

    bell_stage = model.part("bell_stage")
    bell_stage.visual(
        Box((7.20, 7.20, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=stone_dark,
        name="bell_base_course",
    )
    bell_stage.visual(
        Box((6.80, 6.80, 4.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.35)),
        material=stone,
        name="bell_block",
    )
    bell_stage.visual(
        Box((7.40, 7.40, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, 4.55)),
        material=stone_dark,
        name="bell_cornice",
    )
    for sx in (-3.10, 3.10):
        for sy in (-3.10, 3.10):
            bell_stage.visual(
                Box((0.44, 0.44, 4.60)),
                origin=Origin(xyz=(sx, sy, 2.30)),
                material=stone_dark,
            )

    _add_bell_louver_face(
        bell_stage,
        normal_axis="x",
        sign=1.0,
        outer_offset=3.40,
        material_dark=bell_void,
        material_stone=stone_dark,
    )
    _add_bell_louver_face(
        bell_stage,
        normal_axis="x",
        sign=-1.0,
        outer_offset=3.40,
        material_dark=bell_void,
        material_stone=stone_dark,
    )
    _add_bell_louver_face(
        bell_stage,
        normal_axis="y",
        sign=1.0,
        outer_offset=3.40,
        material_dark=bell_void,
        material_stone=stone_dark,
    )
    _add_bell_louver_face(
        bell_stage,
        normal_axis="y",
        sign=-1.0,
        outer_offset=3.40,
        material_dark=bell_void,
        material_stone=stone_dark,
    )
    bell_stage.inertial = Inertial.from_geometry(
        Box((7.40, 7.40, 4.80)),
        mass=10000.0,
        origin=Origin(xyz=(0.0, 0.0, 2.40)),
    )

    spire = model.part("spire")
    spire_mesh = mesh_from_geometry(
        section_loft(
            [
                _square_section(7.30, 0.0),
                _square_section(6.30, 1.10),
                _square_section(3.80, 3.80),
                _square_section(0.70, 7.90),
                _square_section(0.14, 8.20),
            ]
        ),
        "clock_tower_spire",
    )
    spire.visual(spire_mesh, material=roof_slate, name="spire_shell")
    spire.visual(
        Cylinder(radius=0.08, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 8.55)),
        material=metal_dark,
        name="spire_finial",
    )
    spire.inertial = Inertial.from_geometry(
        Box((7.30, 7.30, 8.90)),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 4.45)),
    )

    model.articulation(
        "shaft_to_clock_stage",
        ArticulationType.FIXED,
        parent=shaft,
        child=clock_stage,
        origin=Origin(xyz=(0.0, 0.0, 15.80)),
    )
    model.articulation(
        "clock_to_bell_stage",
        ArticulationType.FIXED,
        parent=clock_stage,
        child=bell_stage,
        origin=Origin(xyz=(0.0, 0.0, 4.20)),
    )
    model.articulation(
        "bell_to_spire",
        ArticulationType.FIXED,
        parent=bell_stage,
        child=spire,
        origin=Origin(xyz=(0.0, 0.0, 4.80)),
    )

    hand_specs = [
        ("east", "x", 1.0, 3.35, "hour"),
        ("east", "x", 1.0, 3.43, "minute"),
        ("west", "x", -1.0, 3.35, "hour"),
        ("west", "x", -1.0, 3.43, "minute"),
        ("north", "y", 1.0, 3.35, "hour"),
        ("north", "y", 1.0, 3.43, "minute"),
        ("south", "y", -1.0, 3.35, "hour"),
        ("south", "y", -1.0, 3.43, "minute"),
    ]
    for face, axis_name, sign, offset, hand_kind in hand_specs:
        part_name = f"{face}_{hand_kind}_hand"
        hand_part = model.part(part_name)
        _add_hand_geometry(hand_part, normal_axis=axis_name, hand_kind=hand_kind, material=hand_black)

        if axis_name == "x":
            joint_xyz = (sign * offset, 0.0, dial_height)
            axis = (sign, 0.0, 0.0)
        else:
            joint_xyz = (0.0, sign * offset, dial_height)
            axis = (0.0, sign, 0.0)

        model.articulation(
            f"{face}_{hand_kind}_spin",
            ArticulationType.CONTINUOUS,
            parent=clock_stage,
            child=hand_part,
            origin=Origin(xyz=joint_xyz),
            axis=axis,
            motion_limits=MotionLimits(effort=12.0, velocity=1.5 if hand_kind == "hour" else 6.0),
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

    required_parts = [
        "shaft",
        "clock_stage",
        "bell_stage",
        "spire",
        "east_hour_hand",
        "east_minute_hand",
        "west_hour_hand",
        "west_minute_hand",
        "north_hour_hand",
        "north_minute_hand",
        "south_hour_hand",
        "south_minute_hand",
    ]
    for part_name in required_parts:
        try:
            object_model.get_part(part_name)
            ok = True
            details = ""
        except Exception as exc:  # pragma: no cover - defensive lookup guard
            ok = False
            details = str(exc)
        ctx.check(f"{part_name} exists", ok, details=details)

    shaft = object_model.get_part("shaft")
    clock_stage = object_model.get_part("clock_stage")
    bell_stage = object_model.get_part("bell_stage")
    spire = object_model.get_part("spire")

    ctx.expect_gap(
        clock_stage,
        shaft,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="clock stage seats on the stone shaft",
    )
    ctx.expect_overlap(
        clock_stage,
        shaft,
        axes="xy",
        min_overlap=5.0,
        name="clock stage stays centered over the shaft",
    )
    ctx.expect_gap(
        bell_stage,
        clock_stage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="bell stage seats on the clock stage",
    )
    ctx.expect_overlap(
        bell_stage,
        clock_stage,
        axes="xy",
        min_overlap=6.2,
        name="bell stage remains centered over the clock stage",
    )
    ctx.expect_gap(
        spire,
        bell_stage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="spire seats on the bell stage",
    )
    ctx.expect_overlap(
        spire,
        bell_stage,
        axes="xy",
        min_overlap=6.8,
        name="spire base covers the bell-stage roof footprint",
    )

    coaxial_specs = [
        ("east", "x", "yz", "east_hour_hand", "east_minute_hand"),
        ("west", "x", "yz", "west_hour_hand", "west_minute_hand"),
        ("north", "y", "xz", "north_hour_hand", "north_minute_hand"),
        ("south", "y", "xz", "south_hour_hand", "south_minute_hand"),
    ]
    for face, axis, plane_axes, hour_name, minute_name in coaxial_specs:
        hour_part = object_model.get_part(hour_name)
        minute_part = object_model.get_part(minute_name)
        ctx.expect_origin_distance(
            hour_part,
            minute_part,
            axes=plane_axes,
            max_dist=0.001,
            name=f"{face} hour and minute hands share the same clock center",
        )
        if axis == "x":
            if face == "east":
                ctx.expect_origin_gap(
                    minute_part,
                    hour_part,
                    axis="x",
                    min_gap=0.07,
                    max_gap=0.09,
                    name="east minute hand stands proud of the hour hand",
                )
            else:
                ctx.expect_origin_gap(
                    hour_part,
                    minute_part,
                    axis="x",
                    min_gap=0.07,
                    max_gap=0.09,
                    name="west minute hand stands proud of the hour hand",
                )
        else:
            if face == "north":
                ctx.expect_origin_gap(
                    minute_part,
                    hour_part,
                    axis="y",
                    min_gap=0.07,
                    max_gap=0.09,
                    name="north minute hand stands proud of the hour hand",
                )
            else:
                ctx.expect_origin_gap(
                    hour_part,
                    minute_part,
                    axis="y",
                    min_gap=0.07,
                    max_gap=0.09,
                    name="south minute hand stands proud of the hour hand",
                )

    expected_axes = {
        "east_hour_spin": (1.0, 0.0, 0.0),
        "east_minute_spin": (1.0, 0.0, 0.0),
        "west_hour_spin": (-1.0, 0.0, 0.0),
        "west_minute_spin": (-1.0, 0.0, 0.0),
        "north_hour_spin": (0.0, 1.0, 0.0),
        "north_minute_spin": (0.0, 1.0, 0.0),
        "south_hour_spin": (0.0, -1.0, 0.0),
        "south_minute_spin": (0.0, -1.0, 0.0),
    }
    for joint_name, expected_axis in expected_axes.items():
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} is a continuous rotation",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )
        ctx.check(
            f"{joint_name} axis matches its clock face",
            tuple(float(v) for v in joint.axis) == expected_axis,
            details=f"axis={joint.axis}, expected={expected_axis}",
        )

    east_minute = object_model.get_part("east_minute_hand")
    east_joint = object_model.get_articulation("east_minute_spin")
    east_rest = _extents(ctx.part_world_aabb(east_minute))
    with ctx.pose({east_joint: math.pi / 2.0}):
        east_quarter = _extents(ctx.part_world_aabb(east_minute))
    ctx.check(
        "east minute hand rotates in the YZ plane",
        east_rest is not None
        and east_quarter is not None
        and east_rest[2] > east_rest[1] + 0.6
        and east_quarter[1] > east_quarter[2] + 0.6,
        details=f"rest={east_rest}, quarter_turn={east_quarter}",
    )

    north_minute = object_model.get_part("north_minute_hand")
    north_joint = object_model.get_articulation("north_minute_spin")
    north_rest = _extents(ctx.part_world_aabb(north_minute))
    with ctx.pose({north_joint: math.pi / 2.0}):
        north_quarter = _extents(ctx.part_world_aabb(north_minute))
    ctx.check(
        "north minute hand rotates in the XZ plane",
        north_rest is not None
        and north_quarter is not None
        and north_rest[2] > north_rest[0] + 0.6
        and north_quarter[0] > north_quarter[2] + 0.6,
        details=f"rest={north_rest}, quarter_turn={north_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
