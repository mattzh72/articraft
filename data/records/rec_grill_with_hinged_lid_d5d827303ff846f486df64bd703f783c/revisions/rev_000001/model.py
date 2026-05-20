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


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geom in geometries:
        merged.merge(geom)
    return merged


def _hood_section(y: float, *, crown_scale: float = 1.0, front_scale: float = 1.0) -> list[tuple[float, float, float]]:
    return [
        (-0.010, y, -0.010),
        (0.000, y, 0.020 * crown_scale),
        (0.080, y, 0.100 * crown_scale),
        (0.245, y, 0.182 * crown_scale),
        (0.430, y, 0.160 * crown_scale),
        (0.565 * front_scale, y, 0.070),
        (0.585 * front_scale, y, -0.030),
        (0.545 * front_scale, y, -0.005),
        (0.500 * front_scale, y, 0.045),
        (0.405, y, 0.095 * crown_scale),
        (0.230, y, 0.118 * crown_scale),
        (0.030, y, 0.020),
        (0.000, y, -0.002),
    ]


def _build_hood_shell(width: float) -> MeshGeometry:
    half_w = width * 0.5
    sections = [
        _hood_section(-half_w, crown_scale=0.96, front_scale=0.985),
        _hood_section(0.0, crown_scale=1.03, front_scale=1.0),
        _hood_section(half_w, crown_scale=0.96, front_scale=0.985),
    ]
    return section_loft(sections)


def _build_warming_rack() -> MeshGeometry:
    rod_radius = 0.0032
    rear_half = 0.31
    rod_specs = [
        [(0.000, -rear_half, 0.000), (0.000, rear_half, 0.000)],
        [(0.100, -0.275, -0.018), (0.100, 0.275, -0.018)],
        [(0.180, -0.262, -0.032), (0.180, 0.262, -0.032)],
        [(0.240, -0.255, -0.043), (0.240, 0.255, -0.043)],
        [(0.305, -0.248, -0.048), (0.305, 0.248, -0.048)],
        [
            (0.000, -rear_half, 0.000),
            (0.100, -0.275, -0.018),
            (0.180, -0.262, -0.032),
            (0.240, -0.255, -0.043),
            (0.305, -0.248, -0.048),
        ],
        [
            (0.000, rear_half, 0.000),
            (0.100, 0.275, -0.018),
            (0.180, 0.262, -0.032),
            (0.240, 0.255, -0.043),
            (0.305, 0.248, -0.048),
        ],
    ]
    rods = [
        wire_from_points(
            pts,
            radius=rod_radius,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.012,
            corner_segments=6,
        )
        for pts in rod_specs
    ]
    return _merge_geometries(rods)


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    center = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_gas_grill")

    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.48, 0.50, 0.53, 1.0))
    black_enamel = model.material("black_enamel", rgba=(0.10, 0.11, 0.12, 1.0))
    warm_steel = model.material("warm_steel", rgba=(0.58, 0.58, 0.60, 1.0))

    outer_width = 0.90
    outer_depth = 0.66
    body_height = 0.36
    wall_t = 0.02

    firebox_inner_width = 0.74
    firebox_inner_depth = 0.48
    firebox_wall_t = 0.015
    firebox_outer_width = firebox_inner_width + 2.0 * firebox_wall_t
    firebox_outer_depth = firebox_inner_depth + 2.0 * firebox_wall_t
    firebox_rear_x = 0.08
    firebox_floor_top = 0.055
    firebox_lip_z = 0.31
    deck_z = 0.320
    deck_t = 0.020

    hood_width = 0.84
    hood_hinge_x = 0.030
    hood_hinge_z = body_height + 0.010

    rack_pivot_x = 0.105
    rack_pivot_z = 0.292

    body = model.part("grill_body")
    body.visual(
        Box((outer_depth, outer_width, 0.02)),
        origin=Origin(xyz=(outer_depth * 0.5, 0.0, 0.01)),
        material=dark_stainless,
        name="bottom_pan",
    )
    body.visual(
        Box((outer_depth, wall_t, 0.30)),
        origin=Origin(xyz=(outer_depth * 0.5, outer_width * 0.5 - wall_t * 0.5, 0.17)),
        material=stainless,
        name="right_outer_wall",
    )
    body.visual(
        Box((outer_depth, wall_t, 0.34)),
        origin=Origin(xyz=(outer_depth * 0.5, -outer_width * 0.5 + wall_t * 0.5, 0.17)),
        material=stainless,
        name="left_outer_wall",
    )
    body.visual(
        Box((wall_t, outer_width, 0.34)),
        origin=Origin(xyz=(wall_t * 0.5, 0.0, 0.17)),
        material=stainless,
        name="rear_outer_wall",
    )
    body.visual(
        Box((0.035, outer_width, 0.24)),
        origin=Origin(xyz=(outer_depth - 0.0175, 0.0, 0.12)),
        material=stainless,
        name="front_fascia",
    )
    body.visual(
        Box((0.09, firebox_outer_width, deck_t)),
        origin=Origin(
            xyz=(0.060, 0.0, deck_z),
        ),
        material=stainless,
        name="rear_support_deck",
    )
    body.visual(
        Box((0.08, firebox_outer_width, deck_t)),
        origin=Origin(
            xyz=(firebox_rear_x + firebox_outer_depth + 0.025, 0.0, deck_z),
        ),
        material=stainless,
        name="front_support_deck",
    )
    body.visual(
        Box((firebox_outer_depth + 0.01, 0.06, deck_t)),
        origin=Origin(
            xyz=(firebox_rear_x + firebox_outer_depth * 0.5, firebox_outer_width * 0.5 + 0.015, deck_z),
        ),
        material=stainless,
        name="right_support_deck",
    )
    body.visual(
        Box((firebox_outer_depth + 0.01, 0.06, deck_t)),
        origin=Origin(
            xyz=(firebox_rear_x + firebox_outer_depth * 0.5, -firebox_outer_width * 0.5 - 0.015, deck_z),
        ),
        material=stainless,
        name="left_support_deck",
    )
    body.visual(
        Box((0.06, outer_width, 0.024)),
        origin=Origin(xyz=(0.03, 0.0, body_height - 0.022)),
        material=stainless,
        name="rear_top_rim",
    )
    body.visual(
        Box((0.038, outer_width, 0.024)),
        origin=Origin(xyz=(outer_depth - 0.019, 0.0, body_height - 0.022)),
        material=stainless,
        name="front_top_rim",
    )
    body.visual(
        Box((0.567, 0.028, 0.024)),
        origin=Origin(
            xyz=(0.3385, outer_width * 0.5 - 0.014, body_height - 0.022),
        ),
        material=stainless,
        name="right_top_rim",
    )
    body.visual(
        Box((0.567, 0.028, 0.024)),
        origin=Origin(
            xyz=(0.3385, -outer_width * 0.5 + 0.014, body_height - 0.022),
        ),
        material=stainless,
        name="left_top_rim",
    )

    firebox_center_x = firebox_rear_x + firebox_inner_depth * 0.5
    firebox_side_y = firebox_inner_width * 0.5 + firebox_wall_t * 0.5
    firebox_wall_h = firebox_lip_z - firebox_floor_top
    firebox_wall_center_z = firebox_floor_top + firebox_wall_h * 0.5

    body.visual(
        Box((firebox_outer_depth, firebox_inner_width, 0.015)),
        origin=Origin(xyz=(firebox_rear_x + firebox_outer_depth * 0.5, 0.0, firebox_floor_top - 0.0075)),
        material=black_enamel,
        name="firebox_floor",
    )
    body.visual(
        Box((firebox_inner_depth, firebox_wall_t, firebox_wall_h)),
        origin=Origin(xyz=(firebox_center_x, firebox_side_y, firebox_wall_center_z)),
        material=black_enamel,
        name="firebox_right_wall",
    )
    body.visual(
        Box((firebox_inner_depth, firebox_wall_t, firebox_wall_h)),
        origin=Origin(xyz=(firebox_center_x, -firebox_side_y, firebox_wall_center_z)),
        material=black_enamel,
        name="firebox_left_wall",
    )
    body.visual(
        Box((firebox_wall_t, firebox_outer_width, firebox_wall_h)),
        origin=Origin(xyz=(firebox_rear_x + firebox_wall_t * 0.5, 0.0, firebox_wall_center_z)),
        material=black_enamel,
        name="firebox_rear_wall",
    )
    body.visual(
        Box((firebox_wall_t, firebox_inner_width, 0.205)),
        origin=Origin(
            xyz=(firebox_rear_x + firebox_outer_depth - firebox_wall_t * 0.5, 0.0, 0.1575),
        ),
        material=black_enamel,
        name="firebox_front_wall",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.03),
        origin=Origin(
            xyz=(rack_pivot_x, 0.370, rack_pivot_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=warm_steel,
        name="rack_pivot_boss_right",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.03),
        origin=Origin(
            xyz=(rack_pivot_x, -0.370, rack_pivot_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=warm_steel,
        name="rack_pivot_boss_left",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_depth, outer_width, body_height)),
        mass=32.0,
        origin=Origin(xyz=(outer_depth * 0.5, 0.0, body_height * 0.5)),
    )

    hood = model.part("hood")
    hood.visual(
        mesh_from_geometry(_build_hood_shell(hood_width), "hood_shell"),
        material=stainless,
        name="hood_shell",
    )
    hood.visual(
        Cylinder(radius=0.010, length=0.058),
        origin=Origin(xyz=(0.548, -0.180, 0.004), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_stainless,
        name="left_handle_standoff",
    )
    hood.visual(
        Cylinder(radius=0.010, length=0.058),
        origin=Origin(xyz=(0.548, 0.180, 0.004), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_stainless,
        name="right_handle_standoff",
    )
    hood.visual(
        Box((0.032, 0.024, 0.055)),
        origin=Origin(xyz=(0.095, 0.392, -0.0125)),
        material=dark_stainless,
        name="right_hood_hinge_ear",
    )
    hood.visual(
        Box((0.032, 0.024, 0.055)),
        origin=Origin(xyz=(0.095, -0.392, -0.0125)),
        material=dark_stainless,
        name="left_hood_hinge_ear",
    )
    hood.visual(
        Box((0.20, 0.030, 0.10)),
        origin=Origin(xyz=(0.150, 0.395, 0.045)),
        material=dark_stainless,
        name="right_hood_endcap",
    )
    hood.visual(
        Box((0.20, 0.030, 0.10)),
        origin=Origin(xyz=(0.150, -0.395, 0.045)),
        material=dark_stainless,
        name="left_hood_endcap",
    )
    hood.visual(
        Cylinder(radius=0.009, length=0.420),
        origin=Origin(xyz=(0.586, 0.0, 0.004), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_stainless,
        name="hood_handle",
    )
    hood.inertial = Inertial.from_geometry(
        Box((0.60, hood_width, 0.22)),
        mass=12.0,
        origin=Origin(xyz=(0.29, 0.0, 0.07)),
    )

    rack = model.part("warming_rack")
    rack_rod_radius = 0.0038
    rack_segments = {
        "rack_rear_bar": ((0.000, -0.355, 0.000), (0.000, 0.355, 0.000)),
        "rack_mid_bar_1": ((0.100, -0.300, -0.018), (0.100, 0.300, -0.018)),
        "rack_mid_bar_2": ((0.180, -0.282, -0.032), (0.180, 0.282, -0.032)),
        "rack_mid_bar_3": ((0.240, -0.268, -0.043), (0.240, 0.268, -0.043)),
        "rack_front_bar": ((0.305, -0.250, -0.048), (0.305, 0.250, -0.048)),
        "rack_side_left_1": ((0.000, -0.355, 0.000), (0.100, -0.300, -0.018)),
        "rack_side_left_2": ((0.100, -0.300, -0.018), (0.180, -0.282, -0.032)),
        "rack_side_left_3": ((0.180, -0.282, -0.032), (0.240, -0.268, -0.043)),
        "rack_side_left_4": ((0.240, -0.268, -0.043), (0.305, -0.250, -0.048)),
        "rack_side_right_1": ((0.000, 0.355, 0.000), (0.100, 0.300, -0.018)),
        "rack_side_right_2": ((0.100, 0.300, -0.018), (0.180, 0.282, -0.032)),
        "rack_side_right_3": ((0.180, 0.282, -0.032), (0.240, 0.268, -0.043)),
        "rack_side_right_4": ((0.240, 0.268, -0.043), (0.305, 0.250, -0.048)),
    }
    for name, (start, end) in rack_segments.items():
        seg_origin, seg_length = _segment_origin(start, end)
        rack.visual(
            Cylinder(radius=rack_rod_radius, length=seg_length),
            origin=seg_origin,
            material=warm_steel,
            name=name,
        )
    rack.inertial = Inertial.from_geometry(
        Box((0.31, 0.72, 0.06)),
        mass=0.9,
        origin=Origin(xyz=(0.155, 0.0, -0.024)),
    )

    model.articulation(
        "body_to_hood",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(hood_hinge_x, 0.0, hood_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    model.articulation(
        "body_to_warming_rack",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rack,
        origin=Origin(xyz=(rack_pivot_x, 0.0, rack_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("grill_body")
    hood = object_model.get_part("hood")
    rack = object_model.get_part("warming_rack")
    hood_joint = object_model.get_articulation("body_to_hood")
    rack_joint = object_model.get_articulation("body_to_warming_rack")

    ctx.check("grill body exists", body is not None, details="grill_body lookup failed")
    ctx.check("hood exists", hood is not None, details="hood lookup failed")
    ctx.check("warming rack exists", rack is not None, details="warming_rack lookup failed")

    with ctx.pose({hood_joint: 0.0, rack_joint: 0.0}):
        ctx.expect_overlap(
            hood,
            body,
            axes="xy",
            elem_a="hood_shell",
            min_overlap=0.55,
            name="closed hood covers the firebox opening",
        )
        ctx.expect_gap(
            rack,
            body,
            axis="z",
            positive_elem="rack_front_bar",
            negative_elem="firebox_floor",
            min_gap=0.16,
            max_gap=0.24,
            name="warming rack sits above the firebox floor",
        )
        ctx.expect_contact(
            hood,
            body,
            elem_a="right_hood_hinge_ear",
            elem_b="right_support_deck",
            name="right hood hinge ear seats on the side deck",
        )
        ctx.expect_contact(
            hood,
            body,
            elem_a="left_hood_hinge_ear",
            elem_b="left_support_deck",
            name="left hood hinge ear seats on the side deck",
        )

    closed_handle = ctx.part_element_world_aabb(hood, elem="hood_handle")
    open_handle = None
    with ctx.pose({hood_joint: math.radians(80.0)}):
        open_handle = ctx.part_element_world_aabb(hood, elem="hood_handle")

    hood_motion_ok = (
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][2] > closed_handle[0][2] + 0.18
        and open_handle[0][0] < closed_handle[0][0] - 0.08
    )
    ctx.check(
        "hood opens upward from the rear hinge",
        hood_motion_ok,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    closed_rack = ctx.part_element_world_aabb(rack, elem="rack_front_bar")
    raised_rack = None
    with ctx.pose({rack_joint: math.radians(68.0)}):
        raised_rack = ctx.part_element_world_aabb(rack, elem="rack_front_bar")

    rack_motion_ok = (
        closed_rack is not None
        and raised_rack is not None
        and raised_rack[0][2] > closed_rack[0][2] + 0.10
        and raised_rack[0][0] < closed_rack[0][0] - 0.07
    )
    ctx.check(
        "warming rack rotates upward on side pivots",
        rack_motion_ok,
        details=f"closed_rack={closed_rack}, raised_rack={raised_rack}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
