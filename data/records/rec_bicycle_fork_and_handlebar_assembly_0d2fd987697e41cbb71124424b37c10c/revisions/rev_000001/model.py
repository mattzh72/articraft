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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_loop(radius: float, z: float, segments: int = 40) -> list[tuple[float, float, float]]:
    return [
        (
            radius * cos(2.0 * pi * i / segments),
            radius * sin(2.0 * pi * i / segments),
            z,
        )
        for i in range(segments)
    ]


def _circle_profile(radius: float, *, segments: int = 48, reverse: bool = False) -> list[tuple[float, float]]:
    points = [
        (
            radius * cos(2.0 * pi * i / segments),
            radius * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]
    return list(reversed(points)) if reverse else points


def _annular_tube(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 48,
):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments, reverse=True)],
        length,
        center=True,
        cap=True,
    )


def _yz_rounded_section(
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
    *,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for y, z in rounded_rect_profile(
            width_y,
            height_z,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _build_lower_leg(model, part_name: str, *, side_sign: float, lower_color, axle_color):
    part = model.part(part_name)
    part.inertial = Inertial.from_geometry(
        Box((0.075, 0.075, 0.470)),
        mass=1.7,
        origin=Origin(xyz=(0.010, 0.0, -0.235)),
    )

    shell = _annular_tube(0.0295, 0.0188, 0.395).translate(0.0, 0.0, -0.1975)
    part.visual(_mesh(f"{part_name}_shell", shell), material=lower_color, name="outer_shell")
    part.visual(
        _mesh(f"{part_name}_upper_bushing", _annular_tube(0.024, 0.0170, 0.016, segments=48).translate(0.0, 0.0, -0.008)),
        material=axle_color,
        name="upper_bushing",
    )
    part.visual(
        Box((0.048, 0.036, 0.060)),
        origin=Origin(xyz=(0.010, 0.0, -0.423)),
        material=lower_color,
        name="dropout",
    )
    for pad_name, pad_x in (("guide_pad_outer", 0.0225), ("guide_pad_inner", -0.0225)):
        part.visual(
            Box((0.014, 0.012, 0.012)),
            origin=Origin(xyz=(pad_x, 0.0, -0.006)),
            material=lower_color,
            name=pad_name,
        )

    arch = tube_from_spline_points(
        [
            (0.031, 0.000, -0.295),
            (0.044, -0.014 * side_sign, -0.350),
            (0.038, -0.030 * side_sign, -0.398),
            (0.026, -0.045 * side_sign, -0.422),
        ],
        radius=0.0125,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    part.visual(_mesh(f"{part_name}_arch", arch), material=lower_color, name="arch_half")
    part.visual(
        Cylinder(radius=0.0105, length=0.065),
        origin=Origin(
            xyz=(0.010, -0.0325 * side_sign, -0.425),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=axle_color,
        name="axle_half",
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_bike_air_fork")

    frame_black = model.material("frame_black", rgba=(0.14, 0.14, 0.15, 1.0))
    crown_graphite = model.material("crown_graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    lower_black = model.material("lower_black", rgba=(0.10, 0.10, 0.11, 1.0))
    stanchion_silver = model.material("stanchion_silver", rgba=(0.77, 0.79, 0.80, 1.0))
    alloy = model.material("alloy", rgba=(0.70, 0.71, 0.73, 1.0))
    grip_black = model.material("grip_black", rgba=(0.06, 0.06, 0.07, 1.0))

    head_tube = model.part("head_tube")
    head_tube.inertial = Inertial.from_geometry(
        Box((0.230, 0.080, 0.340)),
        mass=1.8,
        origin=Origin(xyz=(-0.070, 0.0, -0.015)),
    )
    head_shell = _annular_tube(0.0315, 0.0206, 0.164, segments=64)
    head_tube.visual(_mesh("head_tube_shell", head_shell), material=frame_black, name="head_shell")
    head_tube.visual(
        _mesh("upper_headset", _annular_tube(0.033, 0.0156, 0.010, segments=48)),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=alloy,
        name="upper_headset",
    )
    head_tube.visual(
        _mesh("lower_headset", _annular_tube(0.034, 0.0200, 0.012, segments=48)),
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        material=alloy,
        name="lower_headset",
    )
    head_tube.visual(
        _mesh(
            "top_tube_stub",
            tube_from_spline_points(
                [
                    (-0.030, 0.0, 0.028),
                    (-0.105, 0.0, 0.062),
                    (-0.180, 0.0, 0.078),
                ],
                radius=0.021,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="top_tube_stub",
    )
    head_tube.visual(
        _mesh(
            "down_tube_stub",
            tube_from_spline_points(
                [
                    (-0.055, 0.0, -0.060),
                    (-0.115, 0.0, -0.126),
                    (-0.180, 0.0, -0.185),
                ],
                radius=0.028,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="down_tube_stub",
    )
    head_tube.visual(
        Box((0.042, 0.048, 0.036)),
        origin=Origin(xyz=(-0.044, 0.0, -0.068)),
        material=frame_black,
        name="down_tube_gusset",
    )

    fork_upper = model.part("fork_upper")
    fork_upper.inertial = Inertial.from_geometry(
        Box((0.120, 0.220, 0.420)),
        mass=3.0,
        origin=Origin(xyz=(0.015, 0.0, -0.080)),
    )
    steerer_geom = section_loft(
        [
            _circle_loop(0.0190, -0.175),
            _circle_loop(0.0190, -0.055),
            _circle_loop(0.0165, 0.000),
            _circle_loop(0.0146, 0.055),
            _circle_loop(0.0146, 0.145),
        ]
    )
    fork_upper.visual(_mesh("fork_steerer", steerer_geom), material=crown_graphite, name="steerer_tube")

    crown_geom = section_loft(
        [
            _yz_rounded_section(-0.022, 0.178, 0.024, 0.006),
            _yz_rounded_section(0.012, 0.192, 0.038, 0.010),
            _yz_rounded_section(0.046, 0.162, 0.022, 0.006),
        ]
    ).translate(0.0, 0.0, -0.155)
    fork_upper.visual(_mesh("fork_crown", crown_geom), material=crown_graphite, name="crown")
    fork_upper.visual(
        Cylinder(radius=0.017, length=0.275),
        origin=Origin(xyz=(0.0, 0.065, -0.2775)),
        material=stanchion_silver,
        name="left_stanchion",
    )
    fork_upper.visual(
        Cylinder(radius=0.017, length=0.275),
        origin=Origin(xyz=(0.0, -0.065, -0.2775)),
        material=stanchion_silver,
        name="right_stanchion",
    )
    fork_upper.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.065, -0.135)),
        material=alloy,
        name="left_top_cap",
    )
    fork_upper.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, -0.065, -0.135)),
        material=alloy,
        name="right_top_cap",
    )
    fork_upper.visual(
        _mesh("fork_upper_spacer", _annular_tube(0.033, 0.0142, 0.008, segments=48)),
        origin=Origin(xyz=(0.0, 0.0, 0.0905)),
        material=alloy,
        name="upper_spacer",
    )
    fork_upper.visual(
        _mesh("fork_crown_race", _annular_tube(0.0345, 0.0195, 0.012, segments=48)),
        origin=Origin(xyz=(0.0, 0.0, -0.0935)),
        material=alloy,
        name="lower_crown_race",
    )

    stem = model.part("stem")
    stem.inertial = Inertial.from_geometry(
        Box((0.100, 0.060, 0.050)),
        mass=0.45,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )
    steerer_clamp_geom = _annular_tube(0.0310, 0.0153, 0.040, segments=64)
    stem.visual(_mesh("stem_steerer_clamp", steerer_clamp_geom), material=frame_black, name="steerer_clamp")
    for ear_name, z_offset in (("upper_clamp_ear", 0.011), ("lower_clamp_ear", -0.011)):
        stem.visual(
            Box((0.024, 0.042, 0.012)),
            origin=Origin(xyz=(-0.038, 0.0, z_offset)),
            material=frame_black,
            name=ear_name,
        )
    stem.visual(
        Box((0.050, 0.038, 0.020)),
        origin=Origin(xyz=(0.058, 0.0, -0.024)),
        material=frame_black,
        name="stem_body",
    )
    stem.visual(
        Box((0.030, 0.038, 0.016)),
        origin=Origin(xyz=(0.078, 0.0, -0.030)),
        material=frame_black,
        name="body_bridge",
    )
    stem.visual(
        Box((0.042, 0.046, 0.010)),
        origin=Origin(xyz=(0.052, 0.0, 0.020)),
        material=frame_black,
        name="upper_bar_clamp",
    )
    stem.visual(
        Box((0.042, 0.046, 0.010)),
        origin=Origin(xyz=(0.052, 0.0, -0.020)),
        material=frame_black,
        name="lower_bar_clamp",
    )
    for z_offset, bolt_name in ((0.011, "upper_clamp_bolt"), (-0.011, "lower_clamp_bolt")):
        stem.visual(
            Cylinder(radius=0.0045, length=0.060),
            origin=Origin(
                xyz=(-0.038, 0.0, z_offset),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=alloy,
            name=bolt_name,
        )

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.140, 0.820, 0.080)),
        mass=0.55,
        origin=Origin(xyz=(-0.020, 0.0, 0.012)),
    )
    bar_points = [
        (-0.055, -0.390, 0.012),
        (-0.047, -0.310, 0.020),
        (-0.028, -0.200, 0.014),
        (-0.006, -0.065, 0.000),
        (-0.006, 0.065, 0.000),
        (-0.028, 0.200, 0.014),
        (-0.047, 0.310, 0.020),
        (-0.055, 0.390, 0.012),
    ]
    handlebar.visual(
        _mesh(
            "mtb_handlebar",
            tube_from_spline_points(
                bar_points,
                radius=0.011,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=alloy,
        name="bar_tube",
    )
    handlebar.visual(
        Cylinder(radius=0.0150, length=0.070),
        origin=Origin(xyz=(-0.006, 0.0, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="center_bulge",
    )
    handlebar.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=Origin(xyz=(-0.055, 0.355, 0.012), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=Origin(xyz=(-0.055, -0.355, 0.012), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )

    left_lower = _build_lower_leg(
        model,
        "left_lower",
        side_sign=1.0,
        lower_color=lower_black,
        axle_color=alloy,
    )
    right_lower = _build_lower_leg(
        model,
        "right_lower",
        side_sign=-1.0,
        lower_color=lower_black,
        axle_color=alloy,
    )

    steer = model.articulation(
        "steer_axis",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=fork_upper,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "stem_mount",
        ArticulationType.FIXED,
        parent=fork_upper,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
    )
    model.articulation(
        "bar_mount",
        ArticulationType.FIXED,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
    )
    left_slide = model.articulation(
        "left_lower_slide",
        ArticulationType.PRISMATIC,
        parent=fork_upper,
        child=left_lower,
        origin=Origin(xyz=(0.0, 0.065, -0.230)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.60, lower=0.0, upper=0.100),
    )
    right_slide = model.articulation(
        "right_lower_slide",
        ArticulationType.PRISMATIC,
        parent=fork_upper,
        child=right_lower,
        origin=Origin(xyz=(0.0, -0.065, -0.230)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.60, lower=0.0, upper=0.100),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    fork_upper = object_model.get_part("fork_upper")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    left_lower = object_model.get_part("left_lower")
    right_lower = object_model.get_part("right_lower")

    steer = object_model.get_articulation("steer_axis")
    left_slide = object_model.get_articulation("left_lower_slide")
    right_slide = object_model.get_articulation("right_lower_slide")

    steerer = fork_upper.get_visual("steerer_tube")
    left_stanchion = fork_upper.get_visual("left_stanchion")
    right_stanchion = fork_upper.get_visual("right_stanchion")
    upper_spacer = fork_upper.get_visual("upper_spacer")
    lower_crown_race = fork_upper.get_visual("lower_crown_race")
    steerer_clamp = stem.get_visual("steerer_clamp")
    upper_bar_clamp = stem.get_visual("upper_bar_clamp")
    lower_bar_clamp = stem.get_visual("lower_bar_clamp")
    bar_center = handlebar.get_visual("center_bulge")
    upper_headset = head_tube.get_visual("upper_headset")
    lower_headset = head_tube.get_visual("lower_headset")
    left_outer = left_lower.get_visual("outer_shell")
    right_outer = right_lower.get_visual("outer_shell")
    left_bushing = left_lower.get_visual("upper_bushing")
    right_bushing = right_lower.get_visual("upper_bushing")
    left_axle = left_lower.get_visual("axle_half")
    right_axle = right_lower.get_visual("axle_half")

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
    ctx.allow_overlap(
        fork_upper,
        left_lower,
        elem_a=left_stanchion,
        elem_b=left_bushing,
        reason="The left lower leg includes a thin annular guide-bushing insert modeled around the stanchion; the nested sleeve fit is intentional.",
    )
    ctx.allow_overlap(
        fork_upper,
        right_lower,
        elem_a=right_stanchion,
        elem_b=right_bushing,
        reason="The right lower leg includes a thin annular guide-bushing insert modeled around the stanchion; the nested sleeve fit is intentional.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_within(
        fork_upper,
        stem,
        axes="xy",
        inner_elem=steerer,
        outer_elem=steerer_clamp,
        margin=0.0015,
        name="steerer stays centered in the stem clamp",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="y",
        elem_a=bar_center,
        elem_b=upper_bar_clamp,
        min_overlap=0.040,
        name="stem clamp spans the wide center section of the handlebar",
    )
    ctx.expect_gap(
        stem,
        handlebar,
        axis="z",
        positive_elem=upper_bar_clamp,
        negative_elem=bar_center,
        max_gap=0.001,
        max_penetration=0.0,
        name="upper faceplate seats on the handlebar center bulge",
    )
    ctx.expect_gap(
        handlebar,
        stem,
        axis="z",
        positive_elem=bar_center,
        negative_elem=lower_bar_clamp,
        max_gap=0.001,
        max_penetration=0.0,
        name="lower cradle supports the handlebar center bulge",
    )
    ctx.expect_contact(
        fork_upper,
        head_tube,
        elem_a=upper_spacer,
        elem_b=upper_headset,
        contact_tol=0.001,
        name="upper headset spacer contacts the head tube",
    )
    ctx.expect_contact(
        fork_upper,
        head_tube,
        elem_a=lower_crown_race,
        elem_b=lower_headset,
        contact_tol=0.001,
        name="lower crown race contacts the head tube",
    )
    ctx.expect_contact(
        left_lower,
        right_lower,
        elem_a=left_axle,
        elem_b=right_axle,
        contact_tol=0.0005,
        name="lower-leg axle halves meet at the fork centerline",
    )
    ctx.expect_contact(
        left_lower,
        fork_upper,
        elem_a=left_bushing,
        elem_b=left_stanchion,
        contact_tol=0.001,
        name="left lower leg is guided on the left stanchion bushing",
    )
    ctx.expect_contact(
        right_lower,
        fork_upper,
        elem_a=right_bushing,
        elem_b=right_stanchion,
        contact_tol=0.001,
        name="right lower leg is guided on the right stanchion bushing",
    )
    ctx.expect_within(
        fork_upper,
        left_lower,
        axes="xy",
        inner_elem=left_stanchion,
        outer_elem=left_outer,
        margin=0.002,
        name="left stanchion stays centered inside the left lower leg",
    )
    ctx.expect_within(
        fork_upper,
        right_lower,
        axes="xy",
        inner_elem=right_stanchion,
        outer_elem=right_outer,
        margin=0.002,
        name="right stanchion stays centered inside the right lower leg",
    )
    ctx.expect_overlap(
        fork_upper,
        left_lower,
        axes="z",
        elem_a=left_stanchion,
        elem_b=left_outer,
        min_overlap=0.175,
        name="left lower leg retains deep insertion at sag",
    )
    ctx.expect_overlap(
        fork_upper,
        right_lower,
        axes="z",
        elem_a=right_stanchion,
        elem_b=right_outer,
        min_overlap=0.175,
        name="right lower leg retains deep insertion at sag",
    )

    left_rest = ctx.part_world_position(left_lower)
    right_rest = ctx.part_world_position(right_lower)
    bar_rest = ctx.part_world_position(handlebar)
    left_upper = left_slide.motion_limits.upper if left_slide.motion_limits is not None else None
    right_upper = right_slide.motion_limits.upper if right_slide.motion_limits is not None else None

    with ctx.pose({left_slide: left_upper, right_slide: right_upper}):
        ctx.expect_contact(
            left_lower,
            right_lower,
            elem_a=left_axle,
            elem_b=right_axle,
            contact_tol=0.0005,
            name="lower-leg axle halves stay joined at full extension",
        )
        ctx.expect_contact(
            left_lower,
            fork_upper,
            elem_a=left_bushing,
            elem_b=left_stanchion,
            contact_tol=0.001,
            name="left bushing stays engaged at full extension",
        )
        ctx.expect_contact(
            right_lower,
            fork_upper,
            elem_a=right_bushing,
            elem_b=right_stanchion,
            contact_tol=0.001,
            name="right bushing stays engaged at full extension",
        )
        ctx.expect_within(
            fork_upper,
            left_lower,
            axes="xy",
            inner_elem=left_stanchion,
            outer_elem=left_outer,
            margin=0.002,
            name="left stanchion stays centered at full extension",
        )
        ctx.expect_within(
            fork_upper,
            right_lower,
            axes="xy",
            inner_elem=right_stanchion,
            outer_elem=right_outer,
            margin=0.002,
            name="right stanchion stays centered at full extension",
        )
        ctx.expect_overlap(
            fork_upper,
            left_lower,
            axes="z",
            elem_a=left_stanchion,
            elem_b=left_outer,
            min_overlap=0.075,
            name="left lower retains insertion at full extension",
        )
        ctx.expect_overlap(
            fork_upper,
            right_lower,
            axes="z",
            elem_a=right_stanchion,
            elem_b=right_outer,
            min_overlap=0.075,
            name="right lower retains insertion at full extension",
        )
        left_extended = ctx.part_world_position(left_lower)
        right_extended = ctx.part_world_position(right_lower)

    ctx.check(
        "both lower legs extend downward on their prismatic joints",
        left_rest is not None
        and right_rest is not None
        and left_extended is not None
        and right_extended is not None
        and left_extended[2] < left_rest[2] - 0.075
        and right_extended[2] < right_rest[2] - 0.075,
        details=(
            f"left_rest={left_rest}, left_extended={left_extended}, "
            f"right_rest={right_rest}, right_extended={right_extended}"
        ),
    )

    with ctx.pose({steer: 0.45}):
        bar_turned = ctx.part_world_position(handlebar)
    ctx.check(
        "steering rotates the stem and handlebar about the head-tube axis",
        bar_rest is not None
        and bar_turned is not None
        and bar_turned[1] > bar_rest[1] + 0.020
        and abs(bar_turned[2] - bar_rest[2]) < 0.010,
        details=f"bar_rest={bar_rest}, bar_turned={bar_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
