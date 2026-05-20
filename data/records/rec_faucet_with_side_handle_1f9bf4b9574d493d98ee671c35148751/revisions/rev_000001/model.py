from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_control_knob_visuals(
    part,
    knob_mesh,
    *,
    side: float,
    shell_material,
    grip_material,
    accent_material,
    add_safety_button: bool,
) -> None:
    knob_rotation = (0.0, pi, 0.0) if side < 0.0 else (0.0, 0.0, 0.0)
    x_center = 0.018 * side

    part.visual(
        knob_mesh,
        origin=Origin(rpy=knob_rotation),
        material=shell_material,
        name="knob_shell",
    )
    part.visual(
        Cylinder(radius=0.0235, length=0.010),
        origin=Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_material,
        name="grip_band",
    )
    part.visual(
        Box((0.006, 0.004, 0.012)),
        origin=Origin(xyz=(0.023 * side, 0.0, 0.022)),
        material=accent_material,
        name="index_tab",
    )
    if add_safety_button:
        part.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(0.030 * side, 0.014, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=grip_material,
            name="safety_button",
        )


def _axis_matches(axis: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) < 1e-6 for a, b in zip(axis, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="thermostatic_bar_faucet")

    chrome = model.material("chrome", rgba=(0.86, 0.87, 0.89, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.20, 0.22, 0.24, 1.0))
    hot_mark = model.material("hot_mark", rgba=(0.86, 0.20, 0.16, 1.0))
    neutral_mark = model.material("neutral_mark", rgba=(0.80, 0.82, 0.84, 1.0))

    body_radius = 0.020
    bar_length = 0.230
    knob_length = 0.032

    knob_profile = [
        (0.0, 0.0),
        (0.021, 0.0),
        (0.024, 0.004),
        (0.027, 0.010),
        (0.027, 0.022),
        (0.024, 0.028),
        (0.021, knob_length),
        (0.0, knob_length),
    ]
    knob_mesh = _save_mesh(
        "bar_faucet_control_knob",
        LatheGeometry(knob_profile, segments=64).rotate_y(pi / 2.0),
    )

    spout_mesh = _save_mesh(
        "bar_faucet_spout_tube",
        tube_from_spline_points(
            [
                (0.0, 0.0, -0.014),
                (0.0, 0.004, -0.042),
                (0.0, 0.028, -0.068),
                (0.0, 0.076, -0.082),
                (0.0, 0.118, -0.082),
            ],
            radius=0.0105,
            samples_per_segment=18,
            radial_segments=22,
            cap_ends=True,
        ),
    )

    body = model.part("bar_body")
    body.inertial = Inertial.from_geometry(
        Box((0.30, 0.09, 0.12)),
        mass=3.0,
        origin=Origin(xyz=(0.0, -0.020, -0.010)),
    )
    body.visual(
        Cylinder(radius=body_radius, length=bar_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="bar_shell",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="center_sleeve",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(xyz=(-0.111, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="left_knob_seat",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(xyz=(0.111, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="right_knob_seat",
    )
    for side_name, x_pos in (("left", -0.075), ("right", 0.075)):
        body.visual(
            Cylinder(radius=0.010, length=0.036),
            origin=Origin(xyz=(x_pos, -0.038, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"{side_name}_inlet_stub",
        )
        body.visual(
            Cylinder(radius=0.028, length=0.006),
            origin=Origin(xyz=(x_pos, -0.059, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"{side_name}_escutcheon",
        )
    body.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.010, -0.025)),
        material=chrome,
        name="spout_seat",
    )

    temperature_knob = model.part("temperature_knob")
    temperature_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.032),
        mass=0.25,
        origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_control_knob_visuals(
        temperature_knob,
        knob_mesh,
        side=-1.0,
        shell_material=chrome,
        grip_material=dark_trim,
        accent_material=hot_mark,
        add_safety_button=True,
    )

    volume_knob = model.part("volume_knob")
    volume_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.032),
        mass=0.25,
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_control_knob_visuals(
        volume_knob,
        knob_mesh,
        side=1.0,
        shell_material=chrome,
        grip_material=dark_trim,
        accent_material=neutral_mark,
        add_safety_button=False,
    )

    spout = model.part("swivel_spout")
    spout.inertial = Inertial.from_geometry(
        Box((0.040, 0.140, 0.090)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.065, -0.050)),
    )
    spout.visual(
        Cylinder(radius=0.021, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=chrome,
        name="spout_flange",
    )
    spout.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=chrome,
        name="spout_collar",
    )
    spout.visual(
        spout_mesh,
        material=chrome,
        name="spout_tube",
    )
    spout.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, 0.120, -0.082), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="aerator_housing",
    )
    spout.visual(
        Cylinder(radius=0.0085, length=0.003),
        origin=Origin(xyz=(0.0, 0.127, -0.082), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="aerator_face",
    )

    model.articulation(
        "temperature_knob_rotation",
        ArticulationType.REVOLUTE,
        parent=body,
        child=temperature_knob,
        origin=Origin(xyz=(-bar_length * 0.5, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.4, lower=-1.6, upper=1.6),
    )
    model.articulation(
        "volume_knob_rotation",
        ArticulationType.REVOLUTE,
        parent=body,
        child=volume_knob,
        origin=Origin(xyz=(bar_length * 0.5, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.4, lower=0.0, upper=1.8),
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.010, -0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bar_body")
    temperature_knob = object_model.get_part("temperature_knob")
    volume_knob = object_model.get_part("volume_knob")
    spout = object_model.get_part("swivel_spout")

    temperature_joint = object_model.get_articulation("temperature_knob_rotation")
    volume_joint = object_model.get_articulation("volume_knob_rotation")
    spout_joint = object_model.get_articulation("spout_swivel")

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

    ctx.expect_contact(temperature_knob, body, name="temperature knob contacts bar body")
    ctx.expect_contact(volume_knob, body, name="volume knob contacts bar body")
    ctx.expect_contact(spout, body, name="swivel spout contacts bar body")

    ctx.expect_gap(
        body,
        temperature_knob,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        name="temperature knob seats flush to left end",
    )
    ctx.expect_gap(
        volume_knob,
        body,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        name="volume knob seats flush to right end",
    )
    ctx.expect_gap(
        body,
        spout,
        axis="z",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="spout flange seats under bar body",
    )
    ctx.expect_origin_distance(
        temperature_knob,
        volume_knob,
        axes="x",
        min_dist=0.22,
        name="end controls span the full bar width",
    )

    ctx.check(
        "temperature control rotates about the bar axis",
        _axis_matches(temperature_joint.axis, (1.0, 0.0, 0.0)),
        details=f"axis={temperature_joint.axis}",
    )
    ctx.check(
        "volume control rotates about the bar axis",
        _axis_matches(volume_joint.axis, (1.0, 0.0, 0.0)),
        details=f"axis={volume_joint.axis}",
    )
    ctx.check(
        "spout swivels about a vertical axis",
        _axis_matches(spout_joint.axis, (0.0, 0.0, 1.0)),
        details=f"axis={spout_joint.axis}",
    )

    temp_limits = temperature_joint.motion_limits
    volume_limits = volume_joint.motion_limits
    spout_limits = spout_joint.motion_limits
    ctx.check(
        "temperature control has bounded mixing travel",
        temp_limits is not None
        and temp_limits.lower is not None
        and temp_limits.upper is not None
        and temp_limits.lower < 0.0 < temp_limits.upper,
        details=f"limits={temp_limits}",
    )
    ctx.check(
        "volume control has bounded shutoff-to-open travel",
        volume_limits is not None
        and volume_limits.lower == 0.0
        and volume_limits.upper is not None
        and volume_limits.upper > 1.5,
        details=f"limits={volume_limits}",
    )
    ctx.check(
        "spout joint is continuous",
        spout_limits is not None and spout_limits.lower is None and spout_limits.upper is None,
        details=f"limits={spout_limits}",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        dims = (
            body_aabb[1][0] - body_aabb[0][0],
            body_aabb[1][1] - body_aabb[0][1],
            body_aabb[1][2] - body_aabb[0][2],
        )
        ctx.check(
            "main body reads as a wide horizontal bar",
            dims[0] > 0.22 and dims[0] > dims[1] * 2.3 and dims[0] > dims[2] * 3.5,
            details=f"dims={dims}",
        )

    spout_position = ctx.part_world_position(spout)
    if spout_position is not None:
        ctx.check(
            "spout is centered beneath the bar",
            abs(spout_position[0]) < 1e-6,
            details=f"position={spout_position}",
        )

    with ctx.pose(
        {
            temperature_joint: 1.1,
            volume_joint: 1.0,
            spout_joint: pi / 2.0,
        }
    ):
        ctx.expect_contact(temperature_knob, body, name="temperature knob stays mounted when turned")
        ctx.expect_contact(volume_knob, body, name="volume knob stays mounted when turned")
        ctx.expect_contact(spout, body, name="spout stays mounted when swiveled")
        ctx.expect_gap(
            body,
            spout,
            axis="z",
            max_gap=0.0005,
            max_penetration=1e-6,
            name="spout swivel keeps seated joint clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
