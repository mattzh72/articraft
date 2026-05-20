from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    wire_from_points,
)


def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _seat_side_profile() -> list[tuple[float, float]]:
    return [
        (0.082, 0.000),
        (0.118, 0.070),
        (0.098, 0.175),
        (0.058, 0.285),
        (0.028, 0.410),
        (-0.030, 0.520),
        (-0.098, 0.540),
        (-0.142, 0.450),
        (-0.150, 0.220),
        (-0.136, 0.032),
        (-0.060, 0.000),
    ]


def _side_panel_mesh(x_inner: float, x_outer: float) -> MeshGeometry:
    profile = _seat_side_profile()
    inner_loop = [(x_inner, y, z) for y, z in profile]
    outer_loop = [(x_outer, y, z) for y, z in profile]
    return section_loft([inner_loop, outer_loop])


def _build_crossbeam_mesh() -> MeshGeometry:
    beam = CylinderGeometry(radius=0.035, height=0.74, radial_segments=28).rotate_y(pi / 2.0)
    left_pad = BoxGeometry((0.050, 0.030, 0.020)).translate(0.210, 0.000, -0.045)
    right_pad = BoxGeometry((0.050, 0.030, 0.020)).translate(-0.210, 0.000, -0.045)
    end_cap_left = CylinderGeometry(radius=0.042, height=0.010, radial_segments=24).rotate_y(pi / 2.0).translate(0.365, 0.000, 0.000)
    end_cap_right = CylinderGeometry(radius=0.042, height=0.010, radial_segments=24).rotate_y(pi / 2.0).translate(-0.365, 0.000, 0.000)
    return _merge_meshes(beam, left_pad, right_pad, end_cap_left, end_cap_right)


def _build_hanger_mesh(side_sign: float) -> MeshGeometry:
    pivot_block = BoxGeometry((0.040, 0.024, 0.030)).translate(0.000, 0.000, 0.000)
    strap = BoxGeometry((0.012, 0.018, 0.530)).translate(-side_sign * 0.008, 0.000, -0.280)
    lower_tab = BoxGeometry((0.012, 0.018, 0.080)).translate(-side_sign * 0.008, 0.000, -0.640)
    upper_gusset = BoxGeometry((0.022, 0.018, 0.060)).translate(-side_sign * 0.005, 0.000, -0.065)
    seat_clamp = BoxGeometry((0.018, 0.020, 0.084)).translate(-side_sign * 0.025, -0.005, -0.655)
    return _merge_meshes(pivot_block, strap, lower_tab, upper_gusset, seat_clamp)


def _build_seat_shell_mesh() -> MeshGeometry:
    seat_pan = BoxGeometry((0.300, 0.165, 0.028)).translate(0.000, -0.020, 0.014)
    front_lip = BoxGeometry((0.150, 0.018, 0.022)).translate(0.000, 0.050, 0.016)
    crotch_post = BoxGeometry((0.068, 0.016, 0.100)).translate(0.000, 0.055, 0.050)
    back_shell = BoxGeometry((0.300, 0.024, 0.390)).rotate_x(0.23).translate(0.000, -0.080, 0.225)
    left_side = BoxGeometry((0.040, 0.170, 0.230)).translate(0.155, -0.010, 0.115)
    right_side = BoxGeometry((0.040, 0.170, 0.230)).translate(-0.155, -0.010, 0.115)
    left_head_wing = BoxGeometry((0.040, 0.070, 0.110)).translate(0.145, -0.085, 0.390)
    right_head_wing = BoxGeometry((0.040, 0.070, 0.110)).translate(-0.145, -0.085, 0.390)
    left_hip_bolster = BoxGeometry((0.030, 0.070, 0.085)).translate(0.145, -0.025, 0.105)
    right_hip_bolster = BoxGeometry((0.030, 0.070, 0.085)).translate(-0.145, -0.025, 0.105)
    rear_rib = BoxGeometry((0.210, 0.020, 0.090)).rotate_x(0.23).translate(0.000, -0.094, 0.120)
    return _merge_meshes(
        seat_pan,
        front_lip,
        crotch_post,
        back_shell,
        left_side,
        right_side,
        left_head_wing,
        right_head_wing,
        left_hip_bolster,
        right_hip_bolster,
        rear_rib,
    )


def _build_safety_bar_mesh() -> MeshGeometry:
    left_arm = wire_from_points(
        [
            (0.182, 0.030, -0.002),
            (0.164, 0.072, -0.020),
            (0.126, 0.116, -0.060),
        ],
        radius=0.010,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.024,
        corner_segments=8,
    )
    right_arm = wire_from_points(
        [
            (-0.182, 0.030, -0.002),
            (-0.164, 0.072, -0.020),
            (-0.126, 0.116, -0.060),
        ],
        radius=0.010,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.024,
        corner_segments=8,
    )
    crossbar = wire_from_points(
        [
            (-0.126, 0.116, -0.060),
            (-0.070, 0.138, -0.078),
            (0.000, 0.146, -0.084),
            (0.070, 0.138, -0.078),
            (0.126, 0.116, -0.060),
        ],
        radius=0.010,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.022,
        corner_segments=8,
    )
    return _merge_meshes(left_arm, right_arm, crossbar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adaptive_playground_swing")

    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    seat_blue = model.material("seat_blue", rgba=(0.16, 0.39, 0.71, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.94, 0.77, 0.18, 1.0))

    crossbeam = model.part("crossbeam")
    crossbeam.visual(
        mesh_from_geometry(_build_crossbeam_mesh(), "crossbeam_shell"),
        material=steel,
        name="crossbeam_shell",
    )
    crossbeam.inertial = Inertial.from_geometry(
        Box((0.74, 0.12, 0.12)),
        mass=8.0,
    )

    left_hanger = model.part("left_hanger")
    left_hanger.visual(
        mesh_from_geometry(_build_hanger_mesh(1.0), "left_hanger_link"),
        material=dark_steel,
        name="left_hanger_link",
    )
    left_hanger.inertial = Inertial.from_geometry(
        Box((0.05, 0.04, 0.62)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, -0.29)),
    )

    right_hanger = model.part("right_hanger")
    right_hanger.visual(
        mesh_from_geometry(_build_hanger_mesh(-1.0), "right_hanger_link"),
        material=dark_steel,
        name="right_hanger_link",
    )
    right_hanger.inertial = Inertial.from_geometry(
        Box((0.05, 0.04, 0.62)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, -0.29)),
    )

    seat_shell = model.part("seat_shell")
    seat_shell.visual(
        mesh_from_geometry(_build_seat_shell_mesh(), "seat_shell"),
        material=seat_blue,
        name="seat_body",
    )
    seat_shell.visual(
        Box((0.020, 0.018, 0.080)),
        origin=Origin(xyz=(0.185, -0.005, 0.235)),
        material=seat_blue,
        name="left_mount",
    )
    seat_shell.visual(
        Box((0.020, 0.018, 0.080)),
        origin=Origin(xyz=(-0.185, -0.005, 0.235)),
        material=seat_blue,
        name="right_mount",
    )
    seat_shell.visual(
        Box((0.020, 0.024, 0.024)),
        origin=Origin(xyz=(0.182, 0.058, 0.242)),
        material=seat_blue,
        name="left_hinge",
    )
    seat_shell.visual(
        Box((0.020, 0.024, 0.024)),
        origin=Origin(xyz=(-0.182, 0.058, 0.242)),
        material=seat_blue,
        name="right_hinge",
    )
    seat_shell.inertial = Inertial.from_geometry(
        Box((0.40, 0.34, 0.56)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -0.02, 0.24)),
    )

    safety_bar = model.part("safety_bar")
    safety_bar.visual(
        mesh_from_geometry(_build_safety_bar_mesh(), "safety_bar"),
        material=safety_yellow,
        name="bar_loop",
    )
    safety_bar.visual(
        Box((0.016, 0.030, 0.022)),
        origin=Origin(xyz=(0.182, 0.015, 0.000)),
        material=safety_yellow,
        name="left_tab",
    )
    safety_bar.visual(
        Box((0.016, 0.030, 0.022)),
        origin=Origin(xyz=(-0.182, 0.015, 0.000)),
        material=safety_yellow,
        name="right_tab",
    )
    safety_bar.inertial = Inertial.from_geometry(
        Box((0.38, 0.14, 0.10)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.045, -0.020)),
    )

    model.articulation(
        "left_top_swing",
        ArticulationType.REVOLUTE,
        parent=crossbeam,
        child=left_hanger,
        origin=Origin(xyz=(0.210, 0.000, -0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "right_top_swing",
        ArticulationType.REVOLUTE,
        parent=crossbeam,
        child=right_hanger,
        origin=Origin(xyz=(-0.210, 0.000, -0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "left_hanger_to_seat",
        ArticulationType.FIXED,
        parent=left_hanger,
        child=seat_shell,
        origin=Origin(xyz=(-0.210, 0.000, -0.890)),
    )
    model.articulation(
        "seat_to_safety_bar",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=safety_bar,
        origin=Origin(xyz=(0.000, 0.070, 0.248)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    crossbeam = object_model.get_part("crossbeam")
    left_hanger = object_model.get_part("left_hanger")
    right_hanger = object_model.get_part("right_hanger")
    seat_shell = object_model.get_part("seat_shell")
    safety_bar = object_model.get_part("safety_bar")

    left_top_swing = object_model.get_articulation("left_top_swing")
    right_top_swing = object_model.get_articulation("right_top_swing")
    seat_to_safety_bar = object_model.get_articulation("seat_to_safety_bar")

    ctx.check(
        "top swing axes run left-right",
        left_top_swing.axis == (1.0, 0.0, 0.0) and right_top_swing.axis == (1.0, 0.0, 0.0),
        details=f"left={left_top_swing.axis}, right={right_top_swing.axis}",
    )
    ctx.check(
        "safety bar hinge axis runs left-right",
        seat_to_safety_bar.axis == (1.0, 0.0, 0.0),
        details=f"bar axis={seat_to_safety_bar.axis}",
    )

    ctx.allow_overlap(
        left_hanger,
        seat_shell,
        elem_a="left_hanger_link",
        elem_b="left_mount",
        reason="Left hanger bracket clamps the molded seat-side mount tab.",
    )
    ctx.allow_overlap(
        right_hanger,
        seat_shell,
        elem_a="right_hanger_link",
        elem_b="right_mount",
        reason="Right hanger bracket clamps the molded seat-side mount tab.",
    )
    ctx.allow_overlap(
        seat_shell,
        safety_bar,
        elem_a="left_hinge",
        elem_b="left_tab",
        reason="Left safety bar pivot tab is clipped into the seat-side hinge socket.",
    )
    ctx.allow_overlap(
        seat_shell,
        safety_bar,
        elem_a="right_hinge",
        elem_b="right_tab",
        reason="Right safety bar pivot tab is clipped into the seat-side hinge socket.",
    )

    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(crossbeam, left_hanger, name="left hanger mounts to crossbeam")
    ctx.expect_contact(crossbeam, right_hanger, name="right hanger mounts to crossbeam")
    ctx.expect_contact(left_hanger, seat_shell, elem_b="left_mount", name="seat hangs from left hanger")
    ctx.expect_contact(right_hanger, seat_shell, elem_b="right_mount", name="seat hangs from right hanger")
    ctx.expect_contact(
        seat_shell,
        safety_bar,
        elem_a="left_hinge",
        elem_b="left_tab",
        name="left hinge keeps safety bar attached",
    )
    ctx.expect_contact(
        seat_shell,
        safety_bar,
        elem_a="right_hinge",
        elem_b="right_tab",
        name="right hinge keeps safety bar attached",
    )
    ctx.expect_gap(crossbeam, seat_shell, axis="z", min_gap=0.25, name="seat hangs below crossbeam")

    seat_rest = ctx.part_world_position(seat_shell)
    assert seat_rest is not None
    bar_rest_aabb = ctx.part_world_aabb(safety_bar)
    assert bar_rest_aabb is not None

    with ctx.pose({left_top_swing: 0.34, right_top_swing: 0.34}):
        swung_seat = ctx.part_world_position(seat_shell)
        assert swung_seat is not None
        ctx.check(
            "swing travels fore and aft",
            swung_seat[1] > seat_rest[1] + 0.15 and swung_seat[2] > seat_rest[2] + 0.03,
            details=f"rest={seat_rest}, swung={swung_seat}",
        )
        ctx.expect_contact(right_hanger, seat_shell, elem_b="right_mount", name="right hanger stays seated in swung pose")
        ctx.expect_contact(left_hanger, seat_shell, elem_b="left_mount", name="left hanger stays seated in swung pose")
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in swung pose")

    with ctx.pose({seat_to_safety_bar: 1.10}):
        bar_open_aabb = ctx.part_world_aabb(safety_bar)
        assert bar_open_aabb is not None
        ctx.check(
            "safety bar opens upward",
            bar_open_aabb[1][2] > bar_rest_aabb[1][2] + 0.06,
            details=f"rest max z={bar_rest_aabb[1][2]:.4f}, open max z={bar_open_aabb[1][2]:.4f}",
        )
        ctx.expect_contact(
            seat_shell,
            safety_bar,
            elem_a="left_hinge",
            elem_b="left_tab",
            name="left hinge stays attached when open",
        )
        ctx.expect_contact(
            seat_shell,
            safety_bar,
            elem_a="right_hinge",
            elem_b="right_tab",
            name="right hinge stays attached when open",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps with safety bar open")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
