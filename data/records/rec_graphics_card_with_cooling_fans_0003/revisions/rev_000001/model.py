from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

CARD_LENGTH = 0.285
CARD_HEIGHT = 0.110
PCB_THICKNESS = 0.002
SHROUD_DEPTH = 0.012
SHROUD_CENTER_Y = 0.036
LEFT_FAN_CENTER_X = -0.043
RIGHT_FAN_CENTER_X = 0.054
FAN_CENTER_Z = 0.0
FAN_RING_OUTER_RADIUS = 0.043
FAN_RING_INNER_RADIUS = 0.037
FAN_BLADE_TIP_RADIUS = 0.034


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((index * math.tau) / segments),
            radius * math.sin((index * math.tau) / segments),
        )
        for index in range(segments)
    ]


def _offset_profile(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_geometry(*, outer_radius: float, inner_radius: float, depth: float) -> MeshGeometry:
    return ExtrudeWithHolesGeometry(
        outer_profile=_circle_profile(outer_radius, segments=64),
        hole_profiles=[_circle_profile(inner_radius, segments=64)],
        height=depth,
        center=True,
        cap=True,
        closed=True,
    ).rotate_x(-math.pi / 2.0)


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in vector))
    return tuple(component / length for component in vector)


def _blade_loop(
    *,
    radius: float,
    angle: float,
    chord: float,
    thickness: float,
    pitch: float,
) -> list[tuple[float, float, float]]:
    center = (radius * math.cos(angle), 0.0, radius * math.sin(angle))
    radial = (math.cos(angle), 0.0, math.sin(angle))
    tangent = (-math.sin(angle), 0.0, math.cos(angle))
    chord_direction = _normalize(
        (
            (tangent[0] * math.cos(pitch)) + (radial[0] * math.sin(pitch)),
            0.0,
            (tangent[2] * math.cos(pitch)) + (radial[2] * math.sin(pitch)),
        )
    )
    half_chord = chord * 0.5
    half_thickness = thickness * 0.5
    return [
        (
            center[0] + (half_chord * chord_direction[0]),
            center[1] + half_thickness,
            center[2] + (half_chord * chord_direction[2]),
        ),
        (
            center[0] - (half_chord * chord_direction[0]),
            center[1] + half_thickness,
            center[2] - (half_chord * chord_direction[2]),
        ),
        (
            center[0] - (half_chord * chord_direction[0]),
            center[1] - half_thickness,
            center[2] - (half_chord * chord_direction[2]),
        ),
        (
            center[0] + (half_chord * chord_direction[0]),
            center[1] - half_thickness,
            center[2] + (half_chord * chord_direction[2]),
        ),
    ]


def _fan_blade_geometry() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _blade_loop(radius=0.012, angle=-0.22, chord=0.022, thickness=0.0018, pitch=0.18),
                _blade_loop(radius=0.024, angle=0.02, chord=0.020, thickness=0.0015, pitch=0.30),
                _blade_loop(radius=FAN_BLADE_TIP_RADIUS, angle=0.28, chord=0.014, thickness=0.0012, pitch=0.42),
            ]
        )
    )
    patterned = MeshGeometry()
    for index in range(9):
        patterned.merge(blade.copy().rotate_y(index * math.tau / 9.0))
    return patterned


def _fan_ring_mesh():
    return _save_mesh(
        _ring_geometry(
            outer_radius=FAN_RING_OUTER_RADIUS,
            inner_radius=FAN_RING_INNER_RADIUS,
            depth=SHROUD_DEPTH,
        ),
        "gpu_fan_ring.obj",
    )


def _fan_blade_mesh():
    return _save_mesh(_fan_blade_geometry(), "gpu_fan_blades.obj")


def _front_fascia_mesh():
    fascia = ExtrudeWithHolesGeometry(
        outer_profile=[
            (-0.126, -0.052),
            (0.106, -0.052),
            (0.124, -0.040),
            (0.124, 0.040),
            (0.108, 0.052),
            (-0.126, 0.052),
        ],
        hole_profiles=[
            _offset_profile(_circle_profile(0.039, segments=56), LEFT_FAN_CENTER_X, 0.0),
            _offset_profile(_circle_profile(0.039, segments=56), RIGHT_FAN_CENTER_X, 0.0),
        ],
        height=0.004,
        center=True,
        cap=True,
        closed=True,
    ).rotate_x(-math.pi / 2.0)
    return _save_mesh(fascia, "gpu_front_fascia.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_slot_graphics_card", assets=ASSETS)

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.36, 0.19, 1.0))
    dark_shroud = model.material("dark_shroud", rgba=(0.12, 0.13, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.17, 0.18, 0.20, 1.0))
    fin_aluminum = model.material("fin_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.65, 0.67, 0.70, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.78, 0.65, 0.21, 1.0))
    connector_black = model.material("connector_black", rgba=(0.08, 0.09, 0.10, 1.0))

    fan_ring_mesh = _fan_ring_mesh()
    fan_blade_mesh = _fan_blade_mesh()
    front_fascia_mesh = _front_fascia_mesh()

    pcb = model.part("pcb")
    pcb.visual(
        Box((CARD_LENGTH, PCB_THICKNESS, CARD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pcb_green,
        name="pcb_board",
    )
    pcb.visual(
        Box((0.078, 0.0024, 0.010)),
        origin=Origin(xyz=(0.004, 0.0001, -0.058)),
        material=connector_gold,
        name="pcie_fingers",
    )
    pcb.visual(
        Box((0.020, 0.014, 0.010)),
        origin=Origin(xyz=(0.100, 0.006, 0.060)),
        material=connector_black,
        name="power_connector",
    )
    pcb.inertial = Inertial.from_geometry(
        Box((CARD_LENGTH, 0.014, 0.122)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.006, 0.001)),
    )

    heatsink = model.part("heatsink")
    heatsink.visual(
        Box((0.226, 0.004, 0.092)),
        origin=Origin(xyz=(0.010, 0.003, 0.0)),
        material=fin_aluminum,
        name="cold_plate",
    )
    for index in range(18):
        fin_x = -0.092 + (index * 0.011)
        heatsink.visual(
            Box((0.0045, 0.022, 0.084)),
            origin=Origin(xyz=(fin_x, 0.016, 0.0)),
            material=fin_aluminum,
            name=f"fin_{index:02d}",
        )
    for name, x_pos, z_pos in (
        ("left_upper_pad", -0.094, 0.044),
        ("left_lower_pad", -0.094, -0.044),
        ("right_upper_pad", 0.104, 0.044),
        ("right_lower_pad", 0.104, -0.044),
    ):
        heatsink.visual(
            Box((0.018, 0.004, 0.016)),
            origin=Origin(xyz=(x_pos, 0.026, z_pos)),
            material=fin_aluminum,
            name=name,
        )
    heatsink.inertial = Inertial.from_geometry(
        Box((0.226, 0.028, 0.092)),
        mass=0.85,
        origin=Origin(xyz=(0.010, 0.014, 0.0)),
    )

    shroud = model.part("shroud")
    for geom, origin, name in (
        (Box((0.252, SHROUD_DEPTH, 0.014)), Origin(xyz=(0.000, SHROUD_CENTER_Y, 0.046)), "top_rail"),
        (Box((0.252, SHROUD_DEPTH, 0.014)), Origin(xyz=(0.000, SHROUD_CENTER_Y, -0.046)), "bottom_rail"),
        (Box((0.034, SHROUD_DEPTH, 0.080)), Origin(xyz=(-0.111, SHROUD_CENTER_Y, 0.000)), "left_end_cap"),
        (Box((0.026, SHROUD_DEPTH, 0.080)), Origin(xyz=(0.115, SHROUD_CENTER_Y, 0.000)), "right_end_cap"),
        (Box((0.012, SHROUD_DEPTH, 0.080)), Origin(xyz=(0.007, SHROUD_CENTER_Y, 0.000)), "center_bridge"),
        (Box((0.016, 0.008, 0.014)), Origin(xyz=(-0.094, 0.032, 0.044)), "left_upper_post"),
        (Box((0.016, 0.008, 0.014)), Origin(xyz=(-0.094, 0.032, -0.044)), "left_lower_post"),
        (Box((0.016, 0.008, 0.014)), Origin(xyz=(0.104, 0.032, 0.044)), "right_upper_post"),
        (Box((0.016, 0.008, 0.014)), Origin(xyz=(0.104, 0.032, -0.044)), "right_lower_post"),
    ):
        shroud.visual(geom, origin=origin, material=dark_shroud, name=name)
    shroud.visual(
        front_fascia_mesh,
        origin=Origin(xyz=(0.000, 0.032, 0.000)),
        material=dark_shroud,
        name="front_fascia",
    )
    shroud.visual(
        fan_ring_mesh,
        origin=Origin(xyz=(LEFT_FAN_CENTER_X, SHROUD_CENTER_Y, FAN_CENTER_Z)),
        material=satin_black,
        name="left_fan_ring",
    )
    shroud.visual(
        fan_ring_mesh,
        origin=Origin(xyz=(RIGHT_FAN_CENTER_X, SHROUD_CENTER_Y, FAN_CENTER_Z)),
        material=satin_black,
        name="right_fan_ring",
    )
    shroud.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(LEFT_FAN_CENTER_X, 0.028, FAN_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="left_bearing_cap",
    )
    shroud.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(RIGHT_FAN_CENTER_X, 0.028, FAN_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="right_bearing_cap",
    )
    for fan_center_x, prefix in ((LEFT_FAN_CENTER_X, "left"), (RIGHT_FAN_CENTER_X, "right")):
        for x_offset, z_offset, size, name in (
            (0.023, 0.000, (0.030, 0.005, 0.004), f"{prefix}_spoke_pos_x"),
            (-0.023, 0.000, (0.030, 0.005, 0.004), f"{prefix}_spoke_neg_x"),
            (0.000, 0.023, (0.004, 0.005, 0.030), f"{prefix}_spoke_pos_z"),
            (0.000, -0.023, (0.004, 0.005, 0.030), f"{prefix}_spoke_neg_z"),
        ):
            shroud.visual(
                Box(size),
                origin=Origin(xyz=(fan_center_x + x_offset, 0.0285, z_offset)),
                material=satin_black,
                name=name,
            )
    shroud.inertial = Inertial.from_geometry(
        Box((0.252, 0.020, 0.106)),
        mass=0.42,
        origin=Origin(xyz=(0.000, 0.034, 0.000)),
    )

    left_fan = model.part("left_fan")
    left_fan.visual(
        fan_blade_mesh,
        material=fin_aluminum,
        name="blade_pack",
    )
    left_fan.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="hub",
    )
    left_fan.visual(
        Cylinder(radius=0.0065, length=0.003),
        origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_shroud,
        name="hub_cap",
    )
    left_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.012),
        mass=0.05,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_fan = model.part("right_fan")
    right_fan.visual(
        fan_blade_mesh,
        material=fin_aluminum,
        name="blade_pack",
    )
    right_fan.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="hub",
    )
    right_fan.visual(
        Cylinder(radius=0.0065, length=0.003),
        origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_shroud,
        name="hub_cap",
    )
    right_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.012),
        mass=0.05,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.003, 0.040, 0.118)),
        origin=Origin(xyz=(-0.144, 0.020, 0.0)),
        material=bracket_steel,
        name="main_plate",
    )
    bracket.visual(
        Box((0.014, 0.010, 0.026)),
        origin=Origin(xyz=(-0.1365, 0.005, -0.040)),
        material=bracket_steel,
        name="mounting_tab",
    )
    bracket.visual(
        Box((0.010, 0.012, 0.016)),
        origin=Origin(xyz=(-0.138, 0.012, -0.018)),
        material=connector_black,
        name="lower_port_body",
    )
    bracket.visual(
        Box((0.010, 0.012, 0.016)),
        origin=Origin(xyz=(-0.138, 0.012, 0.010)),
        material=connector_black,
        name="upper_port_body",
    )
    bracket.visual(
        Box((0.010, 0.004, 0.046)),
        origin=Origin(xyz=(-0.138, 0.014, 0.040)),
        material=bracket_steel,
        name="vent_bar",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.018, 0.040, 0.118)),
        mass=0.09,
        origin=Origin(xyz=(-0.139, 0.020, 0.0)),
    )

    model.articulation(
        "pcb_to_heatsink",
        ArticulationType.FIXED,
        parent=pcb,
        child=heatsink,
        origin=Origin(),
    )
    model.articulation(
        "heatsink_to_shroud",
        ArticulationType.FIXED,
        parent=heatsink,
        child=shroud,
        origin=Origin(),
    )
    model.articulation(
        "pcb_to_bracket",
        ArticulationType.FIXED,
        parent=pcb,
        child=bracket,
        origin=Origin(),
    )
    model.articulation(
        "shroud_to_left_fan",
        ArticulationType.CONTINUOUS,
        parent=shroud,
        child=left_fan,
        origin=Origin(xyz=(LEFT_FAN_CENTER_X, SHROUD_CENTER_Y, FAN_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=120.0),
    )
    model.articulation(
        "shroud_to_right_fan",
        ArticulationType.CONTINUOUS,
        parent=shroud,
        child=right_fan,
        origin=Origin(xyz=(RIGHT_FAN_CENTER_X, SHROUD_CENTER_Y, FAN_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=120.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pcb = object_model.get_part("pcb")
    heatsink = object_model.get_part("heatsink")
    shroud = object_model.get_part("shroud")
    left_fan = object_model.get_part("left_fan")
    right_fan = object_model.get_part("right_fan")
    bracket = object_model.get_part("bracket")
    left_spin = object_model.get_articulation("shroud_to_left_fan")
    right_spin = object_model.get_articulation("shroud_to_right_fan")

    pcb_board = pcb.get_visual("pcb_board")
    cold_plate = heatsink.get_visual("cold_plate")
    left_upper_pad = heatsink.get_visual("left_upper_pad")
    left_lower_pad = heatsink.get_visual("left_lower_pad")
    right_upper_pad = heatsink.get_visual("right_upper_pad")
    right_lower_pad = heatsink.get_visual("right_lower_pad")
    left_upper_post = shroud.get_visual("left_upper_post")
    left_lower_post = shroud.get_visual("left_lower_post")
    right_upper_post = shroud.get_visual("right_upper_post")
    right_lower_post = shroud.get_visual("right_lower_post")
    left_ring = shroud.get_visual("left_fan_ring")
    right_ring = shroud.get_visual("right_fan_ring")
    left_bearing = shroud.get_visual("left_bearing_cap")
    right_bearing = shroud.get_visual("right_bearing_cap")
    left_hub = left_fan.get_visual("hub")
    right_hub = right_fan.get_visual("hub")
    bracket_plate = bracket.get_visual("main_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        heatsink,
        pcb,
        axis="y",
        positive_elem=cold_plate,
        negative_elem=pcb_board,
        max_gap=0.0,
        max_penetration=0.0,
        name="cold plate seats directly on pcb",
    )
    ctx.expect_overlap(
        heatsink,
        pcb,
        axes="xz",
        elem_a=cold_plate,
        elem_b=pcb_board,
        min_overlap=0.09,
        name="heatsink footprint stays on pcb",
    )

    for post, pad, name in (
        (left_upper_post, left_upper_pad, "left upper shroud post seats on heatsink"),
        (left_lower_post, left_lower_pad, "left lower shroud post seats on heatsink"),
        (right_upper_post, right_upper_pad, "right upper shroud post seats on heatsink"),
        (right_lower_post, right_lower_pad, "right lower shroud post seats on heatsink"),
    ):
        ctx.expect_gap(
            shroud,
            heatsink,
            axis="y",
            positive_elem=post,
            negative_elem=pad,
            max_gap=1e-6,
            max_penetration=0.0,
            name=name,
        )

    ctx.expect_overlap(
        shroud,
        heatsink,
        axes="xz",
        min_overlap=0.09,
        name="shroud covers the heatsink face",
    )
    ctx.expect_gap(
        pcb,
        bracket,
        axis="x",
        positive_elem=pcb_board,
        negative_elem=bracket_plate,
        max_gap=0.0,
        max_penetration=0.0,
        name="case bracket meets pcb end edge",
    )
    ctx.expect_contact(
        left_fan,
        shroud,
        elem_a=left_hub,
        elem_b=left_bearing,
        name="left fan hub contacts bearing cap",
    )
    ctx.expect_contact(
        right_fan,
        shroud,
        elem_a=right_hub,
        elem_b=right_bearing,
        name="right fan hub contacts bearing cap",
    )
    ctx.expect_within(
        left_fan,
        shroud,
        axes="xz",
        margin=0.0015,
        outer_elem=left_ring,
        name="left rotor stays inside left ring projection",
    )
    ctx.expect_within(
        right_fan,
        shroud,
        axes="xz",
        margin=0.0015,
        outer_elem=right_ring,
        name="right rotor stays inside right ring projection",
    )
    ctx.expect_origin_distance(
        left_fan,
        right_fan,
        axes="x",
        min_dist=0.094,
        max_dist=0.099,
        name="fan centers are evenly spaced along the card",
    )
    ctx.expect_origin_distance(
        left_fan,
        right_fan,
        axes="z",
        max_dist=0.001,
        name="fan centers share a common height",
    )

    ctx.check(
        "left fan spins about face-normal axis",
        left_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in left_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={left_spin.articulation_type}, axis={left_spin.axis}",
    )
    ctx.check(
        "right fan spins about face-normal axis",
        right_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in right_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={right_spin.articulation_type}, axis={right_spin.axis}",
    )

    left_ring_aabb = ctx.part_element_world_aabb(shroud, elem=left_ring)
    if left_ring_aabb is None:
        ctx.fail("left fan ring has measurable diameter", "left_fan_ring produced no world-space bounds")
    else:
        ring_diameter_x = left_ring_aabb[1][0] - left_ring_aabb[0][0]
        ctx.check(
            "fan aperture reads as desktop axial fan",
            0.084 <= ring_diameter_x <= 0.088,
            details=f"left ring diameter x={ring_diameter_x:.4f} m",
        )

    part_aabbs = [ctx.part_world_aabb(part) for part in (pcb, heatsink, shroud, left_fan, right_fan, bracket)]
    if any(aabb is None for aabb in part_aabbs):
        ctx.fail("overall graphics card envelope measurable", "one or more part AABBs were unavailable")
    else:
        mins = [aabb[0] for aabb in part_aabbs if aabb is not None]
        maxs = [aabb[1] for aabb in part_aabbs if aabb is not None]
        overall_length = max(point[0] for point in maxs) - min(point[0] for point in mins)
        overall_thickness = max(point[1] for point in maxs) - min(point[1] for point in mins)
        ctx.check(
            "graphics card length stays desktop scale",
            0.285 <= overall_length <= 0.295,
            details=f"overall length={overall_length:.4f} m",
        )
        ctx.check(
            "graphics card reads as dual-slot thickness",
            0.038 <= overall_thickness <= 0.045,
            details=f"overall thickness={overall_thickness:.4f} m",
        )

    with ctx.pose({left_spin: math.pi / 2.0, right_spin: -math.pi / 3.0}):
        ctx.expect_contact(
            left_fan,
            shroud,
            elem_a=left_hub,
            elem_b=left_bearing,
            name="left fan stays seated when spun",
        )
        ctx.expect_contact(
            right_fan,
            shroud,
            elem_a=right_hub,
            elem_b=right_bearing,
            name="right fan stays seated when spun",
        )
        ctx.expect_within(
            left_fan,
            shroud,
            axes="xz",
            margin=0.0015,
            outer_elem=left_ring,
            name="left rotor remains inside ring at operating pose",
        )
        ctx.expect_within(
            right_fan,
            shroud,
            axes="xz",
            margin=0.0015,
            outer_elem=right_ring,
            name="right rotor remains inside ring at operating pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
