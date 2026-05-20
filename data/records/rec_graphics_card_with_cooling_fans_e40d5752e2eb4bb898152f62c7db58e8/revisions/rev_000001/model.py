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
    sweep_profile_along_spline,
    superellipse_profile,
)


CARD_LENGTH = 0.285
CARD_HEIGHT = 0.112
PCB_THICKNESS = 0.0016


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _fan_rotor(
    model: ArticulatedObject,
    *,
    part_name: str,
    blade_mesh,
    rotor_color,
) -> None:
    rotor = model.part(part_name)
    rotor.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=rotor_color,
        name="hub_collar",
    )
    rotor.visual(
        Cylinder(radius=0.013, length=0.0028),
        origin=Origin(xyz=(0.0, 0.0, 0.0034)),
        material=rotor_color,
        name="hub_body",
    )
    rotor.visual(
        Cylinder(radius=0.0085, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=rotor_color,
        name="hub_cap",
    )
    for blade_index in range(9):
        angle = (2.0 * math.pi * blade_index) / 9.0
        rotor.visual(
            blade_mesh,
            origin=Origin(
                rpy=(0.0, 0.0, angle),
            ),
            material=rotor_color,
            name=f"blade_{blade_index:02d}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.006),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_slot_graphics_card")

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.32, 0.17, 1.0))
    matte_black = model.material("matte_black", rgba=(0.09, 0.10, 0.11, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.23, 0.24, 0.27, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.77, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.61, 0.64, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.81, 0.67, 0.28, 1.0))

    shroud_outer = rounded_rect_profile(0.266, 0.102, 0.010, corner_segments=8)
    fan_hole_profile = superellipse_profile(0.074, 0.074, exponent=2.0, segments=40)
    shroud_face_mesh = _mesh(
        "gpu_shroud_face",
        ExtrudeWithHolesGeometry(
            shroud_outer,
            [
                _offset_profile(fan_hole_profile, dx=-0.056, dy=0.004),
                _offset_profile(fan_hole_profile, dx=0.068, dy=0.004),
            ],
            height=0.006,
            center=True,
        ),
    )

    ring_outer = superellipse_profile(0.080, 0.080, exponent=2.0, segments=40)
    ring_inner = superellipse_profile(0.074, 0.074, exponent=2.0, segments=40)
    fan_ring_mesh = _mesh(
        "gpu_fan_frame_ring",
        ExtrudeWithHolesGeometry(
            ring_outer,
            [ring_inner],
            height=0.006,
            center=True,
        ),
    )
    fan_blade_mesh = _mesh(
        "gpu_fan_blade",
        sweep_profile_along_spline(
            [
                (0.008, -0.0015, 0.0040),
                (0.020, 0.0015, 0.0046),
                (0.033, 0.0075, 0.0051),
            ],
            profile=rounded_rect_profile(0.008, 0.0018, 0.0007, corner_segments=5),
            samples_per_segment=14,
            cap_profile=True,
        ),
    )

    bracket_outer = _rect_profile(0.038, 0.118)
    bracket_holes = [
        _offset_profile(_rect_profile(0.014, 0.016), dx=-0.008, dy=0.028),
        _offset_profile(_rect_profile(0.014, 0.016), dx=0.008, dy=0.028),
        _offset_profile(_rect_profile(0.014, 0.016), dx=-0.008, dy=0.006),
        _offset_profile(_rect_profile(0.014, 0.016), dx=0.008, dy=0.006),
        _offset_profile(_rect_profile(0.028, 0.032), dx=0.0, dy=-0.028),
    ]
    bracket_mesh = _mesh(
        "gpu_io_bracket",
        ExtrudeWithHolesGeometry(
            bracket_outer,
            bracket_holes,
            height=0.002,
            center=True,
        ).rotate_y(math.pi / 2.0),
    )

    pcb = model.part("pcb")
    pcb.visual(
        Box((CARD_LENGTH, CARD_HEIGHT, PCB_THICKNESS)),
        material=pcb_green,
        name="board",
    )
    pcb.visual(
        Box((0.090, 0.010, 0.0006)),
        origin=Origin(xyz=(0.020, -(CARD_HEIGHT * 0.5) + 0.003, 0.0005)),
        material=connector_gold,
        name="pcie_fingers",
    )
    pcb.visual(
        Box((0.034, 0.016, 0.004)),
        origin=Origin(xyz=(-0.108, -0.012, 0.0028)),
        material=dark_graphite,
        name="rear_io_block",
    )
    pcb.inertial = Inertial.from_geometry(
        Box((CARD_LENGTH, CARD_HEIGHT, 0.012)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.0052)),
    )

    heatsink = model.part("heatsink")
    heatsink.visual(
        Box((0.248, 0.098, 0.004)),
        origin=Origin(xyz=(0.006, 0.004, 0.002)),
        material=gunmetal,
        name="baseplate",
    )
    for fin_index in range(26):
        x_pos = -0.112 + fin_index * 0.009
        heatsink.visual(
            Box((0.0014, 0.094, 0.024)),
            origin=Origin(xyz=(x_pos, 0.004, 0.016)),
            material=aluminum,
            name=f"fin_{fin_index:02d}",
        )
    for pipe_index, y_pos in enumerate((-0.022, 0.004, 0.030)):
        heatsink.visual(
            Cylinder(radius=0.003, length=0.226),
            origin=Origin(
                xyz=(0.006, y_pos, 0.007),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"heatpipe_{pipe_index:02d}",
        )
    heatsink.inertial = Inertial.from_geometry(
        Box((0.248, 0.098, 0.029)),
        mass=0.62,
        origin=Origin(xyz=(0.006, 0.004, 0.0145)),
    )

    shroud = model.part("shroud")
    shroud.visual(
        shroud_face_mesh,
        origin=Origin(xyz=(0.006, 0.004, 0.0052)),
        material=matte_black,
        name="front_fascia",
    )
    shroud.visual(
        Box((0.248, 0.008, 0.0082)),
        origin=Origin(xyz=(0.006, 0.051, 0.0041)),
        material=dark_graphite,
        name="top_rail",
    )
    shroud.visual(
        Box((0.248, 0.008, 0.0082)),
        origin=Origin(xyz=(0.006, -0.043, 0.0041)),
        material=dark_graphite,
        name="bottom_rail",
    )
    shroud.visual(
        Box((0.010, 0.094, 0.0082)),
        origin=Origin(xyz=(-0.117, 0.004, 0.0041)),
        material=dark_graphite,
        name="left_endcap",
    )
    shroud.visual(
        Box((0.010, 0.094, 0.0082)),
        origin=Origin(xyz=(0.129, 0.004, 0.0041)),
        material=dark_graphite,
        name="right_endcap",
    )
    for fan_name, x_pos in (("left", -0.056), ("right", 0.068)):
        shroud.visual(
            fan_ring_mesh,
            origin=Origin(xyz=(x_pos, 0.004, 0.004)),
            material=dark_graphite,
            name=f"{fan_name}_fan_frame",
        )
        for spoke_index in range(4):
            spoke_angle = spoke_index * (math.pi / 2.0)
            shroud.visual(
                Box((0.032, 0.004, 0.002)),
                origin=Origin(
                    xyz=(
                        x_pos + 0.023 * math.cos(spoke_angle),
                        0.004 + 0.023 * math.sin(spoke_angle),
                        0.002,
                    ),
                    rpy=(0.0, 0.0, spoke_angle),
                ),
                material=dark_graphite,
                name=f"{fan_name}_spoke_{spoke_index:02d}",
            )
        shroud.visual(
            Cylinder(radius=0.007, length=0.003),
            origin=Origin(xyz=(x_pos, 0.004, 0.0015)),
            material=gunmetal,
            name=f"{fan_name}_fan_bearing",
        )
    shroud.inertial = Inertial.from_geometry(
        Box((0.266, 0.102, 0.010)),
        mass=0.22,
        origin=Origin(xyz=(0.006, 0.004, 0.005)),
    )

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.274, 0.106, 0.002)),
        origin=Origin(xyz=(0.006, 0.004, -0.001)),
        material=dark_graphite,
        name="backplate_panel",
    )
    backplate.visual(
        Box((0.220, 0.010, 0.0014)),
        origin=Origin(xyz=(0.016, 0.045, -0.0003)),
        material=gunmetal,
        name="backplate_rib_top",
    )
    backplate.visual(
        Box((0.220, 0.010, 0.0014)),
        origin=Origin(xyz=(0.016, -0.037, -0.0003)),
        material=gunmetal,
        name="backplate_rib_bottom",
    )
    backplate.inertial = Inertial.from_geometry(
        Box((0.274, 0.106, 0.002)),
        mass=0.16,
        origin=Origin(xyz=(0.006, 0.004, -0.001)),
    )

    io_bracket = model.part("io_bracket")
    io_bracket.visual(
        bracket_mesh,
        origin=Origin(xyz=(-0.001, 0.004, 0.019)),
        material=steel,
        name="bracket_plate",
    )
    io_bracket.visual(
        Box((0.002, 0.022, 0.018)),
        origin=Origin(xyz=(-0.001, -0.047, 0.009)),
        material=steel,
        name="retention_tab",
    )
    io_bracket.inertial = Inertial.from_geometry(
        Box((0.002, 0.125, 0.038)),
        mass=0.08,
        origin=Origin(xyz=(-0.001, 0.002, 0.019)),
    )

    _fan_rotor(
        model,
        part_name="left_fan_rotor",
        blade_mesh=fan_blade_mesh,
        rotor_color=matte_black,
    )
    _fan_rotor(
        model,
        part_name="right_fan_rotor",
        blade_mesh=fan_blade_mesh,
        rotor_color=matte_black,
    )

    model.articulation(
        "pcb_to_heatsink",
        ArticulationType.FIXED,
        parent=pcb,
        child=heatsink,
        origin=Origin(xyz=(0.0, 0.0, PCB_THICKNESS * 0.5)),
    )
    model.articulation(
        "heatsink_to_shroud",
        ArticulationType.FIXED,
        parent=heatsink,
        child=shroud,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )
    model.articulation(
        "pcb_to_backplate",
        ArticulationType.FIXED,
        parent=pcb,
        child=backplate,
        origin=Origin(xyz=(0.0, 0.0, -(PCB_THICKNESS * 0.5))),
    )
    model.articulation(
        "pcb_to_io_bracket",
        ArticulationType.FIXED,
        parent=pcb,
        child=io_bracket,
        origin=Origin(xyz=(-(CARD_LENGTH * 0.5), 0.0, 0.0)),
    )
    model.articulation(
        "left_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=shroud,
        child="left_fan_rotor",
        origin=Origin(xyz=(-0.056, 0.004, 0.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=28.0),
    )
    model.articulation(
        "right_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=shroud,
        child="right_fan_rotor",
        origin=Origin(xyz=(0.068, 0.004, 0.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=28.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pcb = object_model.get_part("pcb")
    heatsink = object_model.get_part("heatsink")
    shroud = object_model.get_part("shroud")
    backplate = object_model.get_part("backplate")
    io_bracket = object_model.get_part("io_bracket")
    left_fan_rotor = object_model.get_part("left_fan_rotor")
    right_fan_rotor = object_model.get_part("right_fan_rotor")

    left_fan_spin = object_model.get_articulation("left_fan_spin")
    right_fan_spin = object_model.get_articulation("right_fan_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=20)

    ctx.expect_contact(heatsink, pcb, elem_a="baseplate", elem_b="board")
    ctx.expect_contact(shroud, heatsink, elem_a="bottom_rail", elem_b="fin_12")
    ctx.expect_contact(backplate, pcb, elem_a="backplate_panel", elem_b="board")
    ctx.expect_contact(io_bracket, pcb, elem_a="bracket_plate", elem_b="board")

    ctx.expect_gap(
        pcb,
        io_bracket,
        axis="x",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem="board",
        negative_elem="bracket_plate",
        name="io_bracket_flush_to_pcb_end",
    )
    ctx.expect_overlap(shroud, pcb, axes="xy", min_overlap=0.09)
    ctx.expect_within(heatsink, shroud, axes="xy", margin=0.01)

    ctx.expect_overlap(
        left_fan_rotor,
        shroud,
        axes="xy",
        min_overlap=0.06,
        elem_b="left_fan_frame",
        name="left_rotor_in_left_frame",
    )
    ctx.expect_overlap(
        right_fan_rotor,
        shroud,
        axes="xy",
        min_overlap=0.06,
        elem_b="right_fan_frame",
        name="right_rotor_in_right_frame",
    )
    ctx.expect_within(
        left_fan_rotor,
        shroud,
        axes="xy",
        margin=0.003,
        outer_elem="left_fan_frame",
        name="left_rotor_within_frame_bounds",
    )
    ctx.expect_within(
        right_fan_rotor,
        shroud,
        axes="xy",
        margin=0.003,
        outer_elem="right_fan_frame",
        name="right_rotor_within_frame_bounds",
    )
    ctx.expect_contact(
        left_fan_rotor,
        shroud,
        elem_a="hub_collar",
        elem_b="left_fan_bearing",
        name="left_rotor_supported_by_bearing",
    )
    ctx.expect_contact(
        right_fan_rotor,
        shroud,
        elem_a="hub_collar",
        elem_b="right_fan_bearing",
        name="right_rotor_supported_by_bearing",
    )

    ctx.expect_origin_distance(
        left_fan_rotor,
        right_fan_rotor,
        axes="x",
        min_dist=0.11,
        max_dist=0.14,
        name="dual_fans_have_realistic_spacing",
    )
    ctx.expect_origin_distance(
        left_fan_rotor,
        right_fan_rotor,
        axes="y",
        max_dist=0.001,
        name="dual_fans_share_same_height",
    )
    ctx.expect_origin_distance(
        left_fan_rotor,
        right_fan_rotor,
        axes="z",
        max_dist=0.001,
        name="dual_fans_share_same_face_plane",
    )

    ctx.check(
        "left_fan_joint_is_continuous",
        left_fan_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected CONTINUOUS, got {left_fan_spin.articulation_type}",
    )
    ctx.check(
        "right_fan_joint_is_continuous",
        right_fan_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected CONTINUOUS, got {right_fan_spin.articulation_type}",
    )
    ctx.check(
        "left_fan_axis_normal_to_face",
        tuple(left_fan_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={left_fan_spin.axis}",
    )
    ctx.check(
        "right_fan_axis_normal_to_face",
        tuple(right_fan_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={right_fan_spin.axis}",
    )

    backplate_aabb = ctx.part_world_aabb(backplate)
    shroud_aabb = ctx.part_world_aabb(shroud)
    bracket_aabb = ctx.part_world_aabb(io_bracket)
    pcb_aabb = ctx.part_world_aabb(pcb)
    if (
        backplate_aabb is not None
        and shroud_aabb is not None
        and bracket_aabb is not None
        and pcb_aabb is not None
    ):
        overall_thickness = shroud_aabb[1][2] - backplate_aabb[0][2]
        overall_length = pcb_aabb[1][0] - bracket_aabb[0][0]
        bracket_height = bracket_aabb[1][1] - bracket_aabb[0][1]
        ctx.check(
            "dual_slot_thickness_plausible",
            0.036 <= overall_thickness <= 0.045,
            details=f"overall_thickness={overall_thickness:.4f} m",
        )
        ctx.check(
            "graphics_card_length_plausible",
            0.280 <= overall_length <= 0.290,
            details=f"overall_length={overall_length:.4f} m",
        )
        ctx.check(
            "io_bracket_height_plausible",
            0.115 <= bracket_height <= 0.125,
            details=f"bracket_height={bracket_height:.4f} m",
        )
    else:
        ctx.fail("graphics_card_aabb_available", "expected part AABBs for proportion checks")

    with ctx.pose({left_fan_spin: math.pi / 2.0, right_fan_spin: math.pi / 3.0}):
        ctx.fail_if_isolated_parts(name="fans_spinning_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="fans_spinning_no_overlap")
        ctx.expect_contact(
            left_fan_rotor,
            shroud,
            elem_a="hub_collar",
            elem_b="left_fan_bearing",
            name="left_rotor_supported_while_spinning",
        )
        ctx.expect_contact(
            right_fan_rotor,
            shroud,
            elem_a="hub_collar",
            elem_b="right_fan_bearing",
            name="right_rotor_supported_while_spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
