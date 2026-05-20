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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _i_beam_profile(
    *, depth: float, flange_width: float, flange_thickness: float, web_thickness: float
) -> list[tuple[float, float]]:
    half_depth = depth * 0.5
    half_flange = flange_width * 0.5
    half_web = web_thickness * 0.5
    return [
        (half_depth, -half_flange),
        (half_depth, half_flange),
        (half_depth - flange_thickness, half_flange),
        (half_depth - flange_thickness, half_web),
        (-half_depth + flange_thickness, half_web),
        (-half_depth + flange_thickness, half_flange),
        (-half_depth, half_flange),
        (-half_depth, -half_flange),
        (-half_depth + flange_thickness, -half_flange),
        (-half_depth + flange_thickness, -half_web),
        (half_depth - flange_thickness, -half_web),
        (half_depth - flange_thickness, -half_flange),
    ]


def _build_i_beam_mesh():
    geometry = ExtrudeGeometry(
        _i_beam_profile(
            depth=0.24,
            flange_width=0.16,
            flange_thickness=0.03,
            web_thickness=0.045,
        ),
        1.34,
        center=True,
    ).rotate_y(pi / 2.0)
    return _save_mesh("steer_axle_i_beam", geometry)


def _build_hub_shell_mesh():
    outer_profile = [
        (0.082, 0.000),
        (0.128, 0.010),
        (0.180, 0.016),
        (0.180, 0.040),
        (0.124, 0.048),
        (0.112, 0.078),
        (0.108, 0.118),
        (0.076, 0.158),
    ]
    inner_profile = [
        (0.060, 0.000),
        (0.060, 0.050),
        (0.054, 0.116),
        (0.046, 0.158),
    ]
    geometry = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(pi / 2.0)
    return _save_mesh("disc_brake_hub_shell", geometry)


def _add_knuckle_visuals(part, *, side_sign: float, steel, machined) -> None:
    part.visual(
        Cylinder(radius=0.038, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=machined,
        name="upper_kingpin_lug",
    )
    part.visual(
        Cylinder(radius=0.038, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        material=machined,
        name="lower_kingpin_lug",
    )
    part.visual(
        Box((0.09, 0.12, 0.145)),
        origin=Origin(xyz=(0.03 * side_sign, 0.0, 0.0)),
        material=steel,
        name="upright_body",
    )
    part.visual(
        Box((0.036, 0.12, 0.12)),
        origin=Origin(xyz=(0.075 * side_sign, 0.0, 0.0)),
        material=steel,
        name="spindle_support",
    )
    part.visual(
        Cylinder(radius=0.092, length=0.016),
        origin=Origin(
            xyz=(0.100 * side_sign, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=machined,
        name="spindle_face",
    )
    part.visual(
        Cylinder(radius=0.050, length=0.110),
        origin=Origin(
            xyz=(0.161 * side_sign, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=machined,
        name="spindle_stub",
    )
    part.visual(
        Box((0.16, 0.04, 0.035)),
        origin=Origin(xyz=(-0.065 * side_sign, -0.070, -0.015)),
        material=steel,
        name="steering_arm",
    )
    part.visual(
        Cylinder(radius=0.017, length=0.10),
        origin=Origin(
            xyz=(-0.120 * side_sign, -0.130, -0.015),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="steering_arm_link",
    )
    part.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(-0.150 * side_sign, -0.180, -0.015)),
        material=machined,
        name="track_ball",
    )


def _add_hub_visuals(part, *, side_sign: float, shell_mesh, brake_iron, machined) -> None:
    part.visual(
        shell_mesh,
        origin=Origin(rpy=(0.0, 0.0, 0.0) if side_sign > 0.0 else (0.0, pi, 0.0)),
        material=brake_iron,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.090, length=0.020),
        origin=Origin(
            xyz=(0.128 * side_sign, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=brake_iron,
        name="mounting_flange",
    )
    for index in range(6):
        angle = (2.0 * pi * index) / 6.0
        y_pos = cos(angle) * 0.072
        z_pos = sin(angle) * 0.072
        part.visual(
            Cylinder(radius=0.007, length=0.022),
            origin=Origin(
                xyz=(0.136 * side_sign, y_pos, z_pos),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=machined,
            name=f"wheel_stud_{index}",
        )


def _add_track_rod_visuals(part, *, steel, machined) -> None:
    part.visual(
        Cylinder(radius=0.015, length=1.14),
        origin=Origin(xyz=(-0.650, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rod_tube",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(-0.090, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined,
        name="left_socket",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(-1.210, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined,
        name="right_socket",
    )
    part.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        material=machined,
        name="left_socket_ball",
    )
    part.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(-1.260, 0.0, 0.0)),
        material=machined,
        name="right_socket_ball",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steer_axle_with_kingpin_stub_axles")

    axle_paint = model.material("axle_paint", rgba=(0.26, 0.28, 0.30, 1.0))
    forged_steel = model.material("forged_steel", rgba=(0.36, 0.38, 0.40, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    brake_iron = model.material("brake_iron", rgba=(0.48, 0.50, 0.53, 1.0))

    i_beam_mesh = _build_i_beam_mesh()
    hub_shell_mesh = _build_hub_shell_mesh()

    beam = model.part("beam")
    beam.visual(i_beam_mesh, material=axle_paint, name="center_beam")
    for side_sign, prefix in ((1.0, "left"), (-1.0, "right")):
        beam.visual(
            Box((0.08, 0.08, 0.22)),
            origin=Origin(xyz=(0.70 * side_sign, 0.0, 0.0)),
            material=forged_steel,
            name=f"{prefix}_end_pad",
        )
        beam.visual(
            Box((0.14, 0.14, 0.05)),
            origin=Origin(xyz=(0.80 * side_sign, 0.0, 0.10)),
            material=forged_steel,
            name=f"{prefix}_upper_ear",
        )
        beam.visual(
            Box((0.14, 0.14, 0.05)),
            origin=Origin(xyz=(0.80 * side_sign, 0.0, -0.10)),
            material=forged_steel,
            name=f"{prefix}_lower_ear",
        )
    beam.inertial = Inertial.from_geometry(
        Box((1.74, 0.18, 0.30)),
        mass=210.0,
        origin=Origin(),
    )

    left_knuckle = model.part("left_knuckle")
    _add_knuckle_visuals(
        left_knuckle,
        side_sign=1.0,
        steel=forged_steel,
        machined=machined_steel,
    )
    left_knuckle.inertial = Inertial.from_geometry(
        Box((0.34, 0.30, 0.18)),
        mass=42.0,
        origin=Origin(xyz=(0.01, -0.06, 0.0)),
    )

    right_knuckle = model.part("right_knuckle")
    _add_knuckle_visuals(
        right_knuckle,
        side_sign=-1.0,
        steel=forged_steel,
        machined=machined_steel,
    )
    right_knuckle.inertial = Inertial.from_geometry(
        Box((0.34, 0.30, 0.18)),
        mass=42.0,
        origin=Origin(xyz=(-0.01, -0.06, 0.0)),
    )

    track_rod = model.part("track_rod")
    _add_track_rod_visuals(track_rod, steel=axle_paint, machined=machined_steel)
    track_rod.inertial = Inertial.from_geometry(
        Box((1.32, 0.08, 0.08)),
        mass=12.0,
        origin=Origin(xyz=(-0.65, 0.0, 0.0)),
    )

    left_hub = model.part("left_hub")
    _add_hub_visuals(
        left_hub,
        side_sign=1.0,
        shell_mesh=hub_shell_mesh,
        brake_iron=brake_iron,
        machined=machined_steel,
    )
    left_hub.inertial = Inertial.from_geometry(
        Box((0.18, 0.40, 0.40)),
        mass=28.0,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    right_hub = model.part("right_hub")
    _add_hub_visuals(
        right_hub,
        side_sign=-1.0,
        shell_mesh=hub_shell_mesh,
        brake_iron=brake_iron,
        machined=machined_steel,
    )
    right_hub.inertial = Inertial.from_geometry(
        Box((0.18, 0.40, 0.40)),
        mass=28.0,
        origin=Origin(xyz=(-0.08, 0.0, 0.0)),
    )

    model.articulation(
        "left_kingpin",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=left_knuckle,
        origin=Origin(xyz=(0.80, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2000.0,
            velocity=0.8,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "right_kingpin",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=right_knuckle,
        origin=Origin(xyz=(-0.80, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2000.0,
            velocity=0.8,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=left_knuckle,
        child=left_hub,
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=40.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=right_knuckle,
        child=right_hub,
        origin=Origin(xyz=(-0.108, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=40.0),
    )
    model.articulation(
        "left_track_rod_mount",
        ArticulationType.FIXED,
        parent=left_knuckle,
        child=track_rod,
        origin=Origin(xyz=(-0.150, -0.180, -0.015)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("beam")
    left_knuckle = object_model.get_part("left_knuckle")
    right_knuckle = object_model.get_part("right_knuckle")
    track_rod = object_model.get_part("track_rod")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")

    left_kingpin = object_model.get_articulation("left_kingpin")
    right_kingpin = object_model.get_articulation("right_kingpin")
    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")

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
        "left kingpin axis is vertical",
        left_kingpin.axis == (0.0, 0.0, 1.0),
        f"axis={left_kingpin.axis}",
    )
    ctx.check(
        "right kingpin axis is vertical",
        right_kingpin.axis == (0.0, 0.0, 1.0),
        f"axis={right_kingpin.axis}",
    )
    ctx.check(
        "left hub spins on axle axis",
        left_hub_spin.axis == (1.0, 0.0, 0.0),
        f"axis={left_hub_spin.axis}",
    )
    ctx.check(
        "right hub spins on axle axis",
        right_hub_spin.axis == (-1.0, 0.0, 0.0),
        f"axis={right_hub_spin.axis}",
    )

    ctx.expect_contact(left_knuckle, beam, name="left knuckle seated in beam yoke")
    ctx.expect_contact(right_knuckle, beam, name="right knuckle seated in beam yoke")
    ctx.expect_contact(
        left_hub,
        left_knuckle,
        elem_a="hub_shell",
        elem_b="spindle_face",
        name="left hub seated on left spindle face",
    )
    ctx.expect_contact(
        right_hub,
        right_knuckle,
        elem_a="hub_shell",
        elem_b="spindle_face",
        name="right hub seated on right spindle face",
    )
    ctx.expect_contact(
        track_rod,
        left_knuckle,
        elem_a="left_socket_ball",
        elem_b="track_ball",
        name="track rod touches left steering arm ball",
    )
    ctx.expect_contact(
        track_rod,
        right_knuckle,
        elem_a="right_socket_ball",
        elem_b="track_ball",
        name="track rod touches right steering arm ball",
    )

    ctx.expect_gap(
        left_hub,
        beam,
        axis="x",
        min_gap=0.03,
        max_gap=0.06,
        name="left hub sits just outboard of beam",
    )
    ctx.expect_gap(
        beam,
        right_hub,
        axis="x",
        min_gap=0.03,
        max_gap=0.06,
        name="right hub sits just outboard of beam",
    )
    ctx.expect_gap(
        beam,
        track_rod,
        axis="y",
        min_gap=0.05,
        max_gap=0.12,
        name="track rod runs behind beam",
    )
    ctx.expect_origin_distance(
        left_knuckle,
        right_knuckle,
        axes="x",
        min_dist=1.55,
        max_dist=1.65,
        name="kingpin span matches a wide steer axle",
    )
    ctx.expect_origin_distance(
        left_hub,
        right_hub,
        axes="x",
        min_dist=1.78,
        max_dist=1.86,
        name="hub to hub track width is realistic",
    )

    left_hub_rest = ctx.part_world_position(left_hub)
    right_hub_rest = ctx.part_world_position(right_hub)
    if left_hub_rest is None or right_hub_rest is None:
        ctx.fail("hub world positions resolve", "Could not resolve hub world positions.")
        return ctx.report()

    with ctx.pose({left_kingpin: 0.45}):
        left_hub_steered = ctx.part_world_position(left_hub)
        if left_hub_steered is None:
            ctx.fail("left steer pose resolves", "Left hub world position was not available.")
        else:
            ctx.check(
                "left steering sweeps hub around kingpin",
                left_hub_steered[1] > left_hub_rest[1] + 0.04
                and abs(left_hub_steered[2] - left_hub_rest[2]) < 1e-6,
                (
                    f"rest={left_hub_rest}, steered={left_hub_steered}, "
                    "expected positive y sweep with unchanged height"
                ),
            )
            ctx.expect_contact(left_knuckle, beam, name="left knuckle remains seated while steered")

    with ctx.pose({right_kingpin: 0.45}):
        right_hub_steered = ctx.part_world_position(right_hub)
        if right_hub_steered is None:
            ctx.fail("right steer pose resolves", "Right hub world position was not available.")
        else:
            ctx.check(
                "right steering sweeps hub around kingpin",
                right_hub_steered[1] < right_hub_rest[1] - 0.04
                and abs(right_hub_steered[2] - right_hub_rest[2]) < 1e-6,
                (
                    f"rest={right_hub_rest}, steered={right_hub_steered}, "
                    "expected negative y sweep with unchanged height"
                ),
            )
            ctx.expect_contact(right_knuckle, beam, name="right knuckle remains seated while steered")

    with ctx.pose({left_hub_spin: 1.3}):
        left_hub_spun = ctx.part_world_position(left_hub)
        if left_hub_spun is None:
            ctx.fail("left hub spin pose resolves", "Left hub world position was not available.")
        else:
            ctx.check(
                "left hub spin keeps spindle center fixed",
                all(abs(a - b) < 1e-6 for a, b in zip(left_hub_spun, left_hub_rest)),
                f"rest={left_hub_rest}, spun={left_hub_spun}",
            )
            ctx.expect_contact(
                left_hub,
                left_knuckle,
                elem_a="hub_shell",
                elem_b="spindle_face",
                name="left hub remains seated while spinning",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
