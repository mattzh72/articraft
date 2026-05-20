from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _circle_profile(radius: float, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(2.0 * pi * index / segments),
            radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def _open_nozzle_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=32),
            [_circle_profile(inner_radius, segments=32)],
            height=length,
            center=True,
        ),
        name,
    )


def _rounded_rect_section(
    width: float,
    height: float,
    radius: float,
    *,
    y: float,
    x: float = 0.0,
    z: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(px + x, y, pz + z) for px, pz in rounded_rect_profile(width, height, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_sink_faucet")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.11, 0.12, 0.13, 1.0))

    nozzle_mesh = _open_nozzle_mesh(
        "faucet_nozzle",
        outer_radius=0.0125,
        inner_radius=0.0085,
        length=0.086,
    )
    body_shell_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.041, 0.0),
                (0.041, 0.004),
                (0.038, 0.007),
                (0.034, 0.012),
                (0.031, 0.021),
                (0.029, 0.034),
                (0.0278, 0.044),
                (0.0265, 0.048),
                (0.0245, 0.052),
                (0.0, 0.052),
            ],
            segments=56,
        ),
        "faucet_body_shell",
    )
    handle_blade_mesh = mesh_from_geometry(
        section_loft(
            [
                _rounded_rect_section(0.013, 0.009, 0.003, y=-0.006, z=0.000),
                _rounded_rect_section(0.0135, 0.009, 0.003, y=-0.024, z=0.001),
                _rounded_rect_section(0.015, 0.0095, 0.0035, y=-0.050, z=0.003),
                _rounded_rect_section(0.018, 0.010, 0.004, y=-0.078, z=0.0055),
                _rounded_rect_section(0.024, 0.0125, 0.005, y=-0.102, z=0.0075),
                _rounded_rect_section(0.030, 0.014, 0.0055, y=-0.116, z=0.009),
            ]
        ),
        "faucet_handle_blade",
    )
    grip_pad_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.018, 0.020, 0.0045),
            0.003,
        ),
        "faucet_handle_grip_pad",
    )

    base_body = model.part("base_body")
    base_body.visual(
        body_shell_mesh,
        origin=Origin(),
        material=chrome,
        name="body_shell",
    )
    base_body.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=brushed_steel,
        name="valve_collar",
    )
    base_body.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=chrome,
        name="spout_seat",
    )
    base_body.inertial = Inertial.from_geometry(
        Box((0.076, 0.076, 0.078)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=brushed_steel,
        name="swivel_collar",
    )
    spout.visual(
        Cylinder(radius=0.012, length=0.246),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=chrome,
        name="vertical_riser",
    )
    spout.visual(
        Cylinder(radius=0.0135, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.278)),
        material=chrome,
        name="spout_cap",
    )
    spout.visual(
        nozzle_mesh,
        origin=Origin(xyz=(0.040, 0.0, 0.252), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="outlet_nozzle",
    )
    spout.visual(
        Cylinder(radius=0.0145, length=0.022),
        origin=Origin(xyz=(0.072, 0.0, 0.252), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="aerator_ring",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.098, 0.030, 0.298)),
        mass=0.8,
        origin=Origin(xyz=(0.036, 0.0, 0.149)),
    )

    lever_handle = model.part("lever_handle")
    lever_handle.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(),
        material=brushed_steel,
        name="handle_hub",
    )
    lever_handle.visual(
        handle_blade_mesh,
        origin=Origin(),
        material=chrome,
        name="handle_blade",
    )
    lever_handle.visual(
        grip_pad_mesh,
        origin=Origin(xyz=(0.0, -0.104, 0.012)),
        material=black_polymer,
        name="grip_pad",
    )
    lever_handle.inertial = Inertial.from_geometry(
        Box((0.030, 0.118, 0.018)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.058, 0.004)),
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=base_body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-1.6,
            upper=1.6,
        ),
    )
    model.articulation(
        "mix_handle_swing",
        ArticulationType.REVOLUTE,
        parent=base_body,
        child=lever_handle,
        origin=Origin(xyz=(0.0, -0.034, 0.061)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.8,
            upper=0.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_body = object_model.get_part("base_body")
    spout = object_model.get_part("spout")
    lever_handle = object_model.get_part("lever_handle")
    spout_swivel = object_model.get_articulation("spout_swivel")
    mix_handle_swing = object_model.get_articulation("mix_handle_swing")

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
        "spout_swivel_axis_vertical",
        tuple(spout_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"unexpected spout axis: {spout_swivel.axis}",
    )
    ctx.check(
        "handle_axis_vertical",
        tuple(mix_handle_swing.axis) == (0.0, 0.0, 1.0),
        details=f"unexpected handle axis: {mix_handle_swing.axis}",
    )
    ctx.check(
        "spout_swivel_range_realistic",
        spout_swivel.motion_limits is not None
        and spout_swivel.motion_limits.lower is not None
        and spout_swivel.motion_limits.upper is not None
        and spout_swivel.motion_limits.lower <= -1.5
        and spout_swivel.motion_limits.upper >= 1.5,
        details="utility spout should swivel broadly side to side",
    )
    ctx.check(
        "handle_mix_range_realistic",
        mix_handle_swing.motion_limits is not None
        and mix_handle_swing.motion_limits.lower is not None
        and mix_handle_swing.motion_limits.upper is not None
        and mix_handle_swing.motion_limits.lower <= -0.7
        and mix_handle_swing.motion_limits.upper >= 0.7,
        details="single lever should sweep through a realistic hot-cold range",
    )

    ctx.expect_gap(
        spout,
        base_body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="swivel_collar",
        negative_elem="spout_seat",
        name="spout_collar_seated",
    )
    ctx.expect_contact(
        spout,
        base_body,
        elem_a="swivel_collar",
        elem_b="spout_seat",
        name="spout_contacts_body",
    )
    ctx.expect_contact(
        lever_handle,
        base_body,
        elem_a="handle_hub",
        elem_b="valve_collar",
        name="handle_hub_contacts_valve_collar",
    )
    ctx.expect_origin_gap(
        spout,
        base_body,
        axis="z",
        min_gap=0.075,
        max_gap=0.081,
        name="spout_origin_above_body",
    )

    base_aabb = ctx.part_world_aabb(base_body)
    spout_aabb = ctx.part_world_aabb(spout)
    handle_aabb = ctx.part_world_aabb(lever_handle)
    assert base_aabb is not None
    assert spout_aabb is not None
    assert handle_aabb is not None
    ctx.check(
        "spout_tall_enough",
        spout_aabb[1][2] > 0.35,
        details=f"spout top too low: {spout_aabb[1][2]:.4f} m",
    )
    ctx.check(
        "faucet_has_forward_reach",
        spout_aabb[1][0] > 0.08,
        details=f"forward reach too short: {spout_aabb[1][0]:.4f} m",
    )
    ctx.check(
        "lever_reads_large",
        (handle_aabb[1][1] - handle_aabb[0][1]) > 0.10,
        details=f"lever too short: {(handle_aabb[1][1] - handle_aabb[0][1]):.4f} m",
    )
    ctx.check(
        "body_is_round_compact",
        (base_aabb[1][0] - base_aabb[0][0]) < 0.09 and (base_aabb[1][1] - base_aabb[0][1]) < 0.09,
        details=f"base footprint too large: {base_aabb}",
    )

    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    spout_limits = spout_swivel.motion_limits
    assert spout_limits is not None
    assert spout_limits.lower is not None
    assert spout_limits.upper is not None
    with ctx.pose({spout_swivel: spout_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="spout_swivel_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="spout_swivel_lower_no_floating")
        left_spout_aabb = ctx.part_world_aabb(spout)
        assert left_spout_aabb is not None
        ctx.check(
            "spout_swung_left",
            left_spout_aabb[0][1] < -0.08,
            details=f"left swing too small: {left_spout_aabb}",
        )
        ctx.expect_contact(
            spout,
            base_body,
            elem_a="swivel_collar",
            elem_b="spout_seat",
            name="spout_lower_pose_contact",
        )

    with ctx.pose({spout_swivel: spout_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="spout_swivel_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="spout_swivel_upper_no_floating")
        right_spout_aabb = ctx.part_world_aabb(spout)
        assert right_spout_aabb is not None
        ctx.check(
            "spout_swung_right",
            right_spout_aabb[1][1] > 0.08,
            details=f"right swing too small: {right_spout_aabb}",
        )
        ctx.expect_contact(
            spout,
            base_body,
            elem_a="swivel_collar",
            elem_b="spout_seat",
            name="spout_upper_pose_contact",
        )

    handle_limits = mix_handle_swing.motion_limits
    assert handle_limits is not None
    assert handle_limits.lower is not None
    assert handle_limits.upper is not None
    with ctx.pose({mix_handle_swing: handle_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="handle_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="handle_lower_no_floating")
        lower_handle_aabb = ctx.part_world_aabb(lever_handle)
        assert lower_handle_aabb is not None
        ctx.check(
            "handle_swings_toward_cold_side",
            lower_handle_aabb[0][0] < -0.06,
            details=f"lower handle sweep too small: {lower_handle_aabb}",
        )
        ctx.expect_contact(
            lever_handle,
            base_body,
            elem_a="handle_hub",
            elem_b="valve_collar",
            name="handle_lower_pose_contact",
        )

    with ctx.pose({mix_handle_swing: handle_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="handle_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="handle_upper_no_floating")
        upper_handle_aabb = ctx.part_world_aabb(lever_handle)
        assert upper_handle_aabb is not None
        ctx.check(
            "handle_swings_toward_hot_side",
            upper_handle_aabb[1][0] > 0.06,
            details=f"upper handle sweep too small: {upper_handle_aabb}",
        )
        ctx.expect_contact(
            lever_handle,
            base_body,
            elem_a="handle_hub",
            elem_b="valve_collar",
            name="handle_upper_pose_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
