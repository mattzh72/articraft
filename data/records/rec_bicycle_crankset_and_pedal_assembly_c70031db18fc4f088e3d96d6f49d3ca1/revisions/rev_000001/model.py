from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _tooth_profile(root_radius: float, tip_radius: float, tooth_count: int) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for tooth_index in range(tooth_count):
        base = 2.0 * pi * tooth_index / tooth_count
        for frac, radius in ((0.06, root_radius), (0.32, tip_radius), (0.50, tip_radius), (0.76, root_radius)):
            angle = base + frac * (2.0 * pi / tooth_count)
            points.append((radius * cos(angle), radius * sin(angle)))
    return points


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = cos(angle)
    s = sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _pedal_frame_geometry(outboard_sign: float):
    outer = _translate_profile(
        rounded_rect_profile(0.084, 0.092, 0.008, corner_segments=7),
        outboard_sign * 0.056,
        0.0,
    )
    center_window = _translate_profile(
        rounded_rect_profile(0.046, 0.052, 0.006, corner_segments=6),
        outboard_sign * 0.057,
        0.0,
    )
    lightening_slot = _translate_profile(
        rounded_rect_profile(0.014, 0.080, 0.004, corner_segments=5),
        outboard_sign * 0.083,
        0.0,
    )
    return ExtrudeWithHolesGeometry(outer, [center_window, lightening_slot], height=0.010, center=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_bike_crankset")

    satin_alloy = model.material("satin_alloy", rgba=(0.76, 0.77, 0.74, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.88, 0.86, 0.80, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.50, 0.52, 0.53, 1.0))
    frame_black = model.material("frame_black", rgba=(0.03, 0.035, 0.04, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bb_tube = LatheGeometry.from_shell_profiles(
        [(0.026, -0.046), (0.026, 0.046)],
        [(0.0175, -0.046), (0.0175, 0.046)],
        segments=48,
    ).rotate_y(pi / 2.0)
    bottom_bracket.visual(_mesh("hollow_bottom_bracket_shell", bb_tube), material=frame_black, name="shell_tube")
    bottom_bracket.visual(
        _mesh("drive_bearing_cup", TorusGeometry(radius=0.023, tube=0.0035, radial_segments=12, tubular_segments=48).rotate_y(pi / 2.0)),
        origin=Origin(xyz=(-0.0475, 0.0, 0.0)),
        material=bearing_steel,
        name="drive_bearing_cup",
    )
    bottom_bracket.visual(
        _mesh("opposite_bearing_cup", TorusGeometry(radius=0.023, tube=0.0035, radial_segments=12, tubular_segments=48).rotate_y(pi / 2.0)),
        origin=Origin(xyz=(0.0475, 0.0, 0.0)),
        material=bearing_steel,
        name="opposite_bearing_cup",
    )
    bottom_bracket.visual(
        _mesh(
            "down_tube_stub",
            tube_from_spline_points(
                [(0.0, 0.000, 0.033), (0.0, 0.050, 0.086), (0.0, 0.096, 0.144)],
                radius=0.014,
                samples_per_segment=10,
                radial_segments=18,
            ),
        ),
        material=frame_black,
        name="down_tube_stub",
    )
    bottom_bracket.visual(
        _mesh(
            "chain_stay_stub_0",
            tube_from_spline_points(
                [(0.018, -0.026, -0.014), (0.044, -0.086, -0.032), (0.062, -0.150, -0.045)],
                radius=0.010,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=frame_black,
        name="chain_stay_0",
    )
    bottom_bracket.visual(
        _mesh(
            "chain_stay_stub_1",
            tube_from_spline_points(
                [(-0.018, -0.026, -0.014), (-0.044, -0.086, -0.032), (-0.062, -0.150, -0.045)],
                radius=0.010,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=frame_black,
        name="chain_stay_1",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0125, length=0.128),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=bearing_steel,
        name="bottom_bracket_spindle",
    )
    crank.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(-0.061, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_anodized,
        name="drive_crank_boss",
    )
    crank.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.061, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_anodized,
        name="opposite_crank_boss",
    )

    drive_arm = sweep_profile_along_spline(
        [(-0.066, 0.0, -0.022), (-0.067, 0.0, -0.070), (-0.069, 0.0, -0.124), (-0.070, 0.0, -0.158)],
        profile=rounded_rect_profile(0.018, 0.010, 0.0035, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    opposite_arm = sweep_profile_along_spline(
        [(0.066, 0.0, 0.022), (0.067, 0.0, 0.070), (0.069, 0.0, 0.124), (0.070, 0.0, 0.158)],
        profile=rounded_rect_profile(0.017, 0.009, 0.0032, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    crank.visual(_mesh("compact_drive_crank_arm", drive_arm), material=satin_alloy, name="drive_arm")
    crank.visual(_mesh("compact_opposite_crank_arm", opposite_arm), material=satin_alloy, name="opposite_arm")
    crank.visual(
        Cylinder(radius=0.017, length=0.022),
        origin=Origin(xyz=(-0.071, 0.0, -0.164), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_alloy,
        name="drive_pedal_eye",
    )
    crank.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(xyz=(0.071, 0.0, 0.164), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_alloy,
        name="opposite_pedal_eye",
    )

    chainring_outer = _tooth_profile(0.084, 0.090, 40)
    chainring_holes = [_circle_profile(0.030, 40)]
    window = rounded_rect_profile(0.012, 0.030, 0.0035, corner_segments=5)
    for index in range(5):
        angle = 2.0 * pi * index / 5.0 + pi / 10.0
        chainring_holes.append(
            _transform_profile(window, dx=0.050 * cos(angle), dy=0.050 * sin(angle), angle=angle + pi / 2.0)
        )
        chainring_holes.append(_circle_profile(0.0032, 16))
        chainring_holes[-1] = _translate_profile(chainring_holes[-1], 0.066 * cos(angle), 0.066 * sin(angle))
    chainring = ExtrudeWithHolesGeometry(chainring_outer, chainring_holes, height=0.0042, center=True).rotate_y(pi / 2.0)
    crank.visual(
        _mesh("compact_chainring_teeth", chainring),
        origin=Origin(xyz=(-0.064, 0.0, 0.0)),
        material=brushed_aluminum,
        name="chainring",
    )
    spider_profile = rounded_rect_profile(0.010, 0.005, 0.0016, corner_segments=4)
    for index in range(5):
        angle = 2.0 * pi * index / 5.0
        spoke = sweep_profile_along_spline(
            [
                (-0.064, 0.018 * cos(angle), 0.018 * sin(angle)),
                (-0.064, 0.046 * cos(angle), 0.046 * sin(angle)),
                (-0.064, 0.080 * cos(angle), 0.080 * sin(angle)),
            ],
            profile=spider_profile,
            samples_per_segment=8,
            cap_profile=True,
            up_hint=(1.0, 0.0, 0.0),
        )
        crank.visual(_mesh(f"chainring_spider_{index}", spoke), material=satin_alloy, name=f"spider_{index}")

    def make_pedal(part_name: str, outboard_sign: float, grip_prefix: str):
        pedal = model.part(part_name)
        pedal.visual(
            _mesh(f"{grip_prefix}_alloy_frame", _pedal_frame_geometry(outboard_sign)),
            material=satin_alloy,
            name="pedal_frame",
        )
        pedal.visual(
            Cylinder(radius=0.0065, length=0.060),
            origin=Origin(xyz=(outboard_sign * 0.004, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_anodized,
            name="fold_hinge_barrel",
        )
        pedal.visual(
            Cylinder(radius=0.0045, length=0.030),
            origin=Origin(xyz=(outboard_sign * 0.003, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=bearing_steel,
            name="spindle_pin",
        )
        for y in (-0.032, 0.032):
            pedal.visual(
                Box((0.026, 0.004, 0.006)),
                origin=Origin(xyz=(outboard_sign * 0.014, y, 0.0)),
                material=satin_alloy,
                name=f"hinge_clevis_{0 if y < 0 else 1}",
            )
        for y in (-0.034, 0.034):
            pedal.visual(
                Box((0.050, 0.004, 0.003)),
                origin=Origin(xyz=(outboard_sign * 0.060, y, 0.0065)),
                material=black_rubber,
                name=f"rubber_grip_{0 if y < 0 else 1}",
            )
        for y in (-0.045, 0.045):
            pedal.visual(
                Box((0.040, 0.003, 0.006)),
                origin=Origin(xyz=(outboard_sign * 0.074, y, 0.006)),
                material=bearing_steel,
                name=f"edge_teeth_{0 if y < 0 else 1}",
            )
        return pedal

    drive_pedal = make_pedal("drive_pedal", -1.0, "drive_pedal")
    opposite_pedal = make_pedal("opposite_pedal", 1.0, "opposite_pedal")

    model.articulation(
        "bottom_bracket_spin",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=crank,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=16.0),
    )
    model.articulation(
        "drive_pedal_fold",
        ArticulationType.REVOLUTE,
        parent=crank,
        child=drive_pedal,
        origin=Origin(xyz=(-0.071, 0.0, -0.164)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.57),
    )
    model.articulation(
        "opposite_pedal_fold",
        ArticulationType.REVOLUTE,
        parent=crank,
        child=opposite_pedal,
        origin=Origin(xyz=(0.071, 0.0, 0.164)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottom_bracket = object_model.get_part("bottom_bracket")
    crank = object_model.get_part("crank")
    drive_pedal = object_model.get_part("drive_pedal")
    opposite_pedal = object_model.get_part("opposite_pedal")
    spin = object_model.get_articulation("bottom_bracket_spin")
    drive_fold = object_model.get_articulation("drive_pedal_fold")
    opposite_fold = object_model.get_articulation("opposite_pedal_fold")

    ctx.allow_overlap(
        crank,
        drive_pedal,
        elem_a="drive_pedal_eye",
        elem_b="fold_hinge_barrel",
        reason="The folding-pedal hinge barrel is intentionally captured inside the crank-arm pedal eye.",
    )
    ctx.allow_overlap(
        crank,
        drive_pedal,
        elem_a="drive_pedal_eye",
        elem_b="spindle_pin",
        reason="The pedal spindle pin is intentionally seated through the crank-arm pedal eye.",
    )
    ctx.allow_overlap(
        crank,
        opposite_pedal,
        elem_a="opposite_pedal_eye",
        elem_b="fold_hinge_barrel",
        reason="The folding-pedal hinge barrel is intentionally captured inside the opposite crank-arm pedal eye.",
    )
    ctx.allow_overlap(
        crank,
        opposite_pedal,
        elem_a="opposite_pedal_eye",
        elem_b="spindle_pin",
        reason="The opposite pedal spindle pin is intentionally seated through the crank-arm pedal eye.",
    )

    ctx.expect_within(
        crank,
        bottom_bracket,
        axes="yz",
        inner_elem="bottom_bracket_spindle",
        outer_elem="shell_tube",
        margin=0.002,
        name="spindle runs through hollow bottom bracket",
    )
    ctx.expect_overlap(
        crank,
        bottom_bracket,
        axes="x",
        elem_a="bottom_bracket_spindle",
        elem_b="shell_tube",
        min_overlap=0.070,
        name="spindle retained across bearing shell",
    )
    ctx.expect_gap(
        drive_pedal,
        crank,
        axis="x",
        positive_elem="fold_hinge_barrel",
        negative_elem="drive_pedal_eye",
        max_penetration=0.022,
        name="drive pedal hinge seated in crank eye",
    )
    ctx.expect_gap(
        drive_pedal,
        crank,
        axis="x",
        positive_elem="spindle_pin",
        negative_elem="drive_pedal_eye",
        max_penetration=0.030,
        name="drive pedal spindle seated in crank eye",
    )
    ctx.expect_gap(
        crank,
        opposite_pedal,
        axis="x",
        positive_elem="opposite_pedal_eye",
        negative_elem="fold_hinge_barrel",
        max_penetration=0.022,
        name="opposite pedal hinge seated in crank eye",
    )
    ctx.expect_gap(
        crank,
        opposite_pedal,
        axis="x",
        positive_elem="opposite_pedal_eye",
        negative_elem="spindle_pin",
        max_penetration=0.030,
        name="opposite pedal spindle seated in crank eye",
    )

    rest_drive = ctx.part_world_aabb(drive_pedal)
    rest_opposite = ctx.part_world_aabb(opposite_pedal)
    with ctx.pose({drive_fold: 1.57, opposite_fold: 1.57}):
        folded_drive = ctx.part_world_aabb(drive_pedal)
        folded_opposite = ctx.part_world_aabb(opposite_pedal)
        ctx.expect_overlap(drive_pedal, crank, axes="yz", min_overlap=0.010, name="drive pedal folds into crank plane")
        ctx.expect_overlap(opposite_pedal, crank, axes="yz", min_overlap=0.010, name="opposite pedal folds into crank plane")

    def x_width(aabb):
        return None if aabb is None else aabb[1][0] - aabb[0][0]

    ctx.check(
        "drive pedal fold reduces lateral projection",
        rest_drive is not None
        and folded_drive is not None
        and x_width(folded_drive) < x_width(rest_drive) * 0.45,
        details=f"rest={rest_drive}, folded={folded_drive}",
    )
    ctx.check(
        "opposite pedal fold reduces lateral projection",
        rest_opposite is not None
        and folded_opposite is not None
        and x_width(folded_opposite) < x_width(rest_opposite) * 0.45,
        details=f"rest={rest_opposite}, folded={folded_opposite}",
    )
    with ctx.pose({spin: pi / 2.0}):
        ctx.expect_origin_gap(crank, bottom_bracket, axis="x", min_gap=-0.001, max_gap=0.001, name="crank rotates about bottom bracket center")

    return ctx.report()


object_model = build_object_model()
