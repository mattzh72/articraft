from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
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
)


BASE_TO_MAST_UPPER = 0.32
HEAD_TILT_UPPER = math.radians(70.0)
HEAD_BRACKET_FORWARD = 0.085


def _rect_loop(width: float, height: float, *, clockwise: bool = False) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    points = [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]
    return list(reversed(points)) if clockwise else points


def _circle_loop(
    radius: float,
    *,
    segments: int = 28,
    center: tuple[float, float] = (0.0, 0.0),
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    cx, cy = center
    points = [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]
    return list(reversed(points)) if clockwise else points


def _square_tube_mesh(outer_size: float, inner_size: float, height: float):
    return ExtrudeWithHolesGeometry(
        _rect_loop(outer_size, outer_size),
        [_rect_loop(inner_size, inner_size, clockwise=True)],
        height,
        center=True,
    )


def _plate_with_hole_mesh(
    *,
    width_x: float,
    height_z: float,
    thickness_y: float,
    hole_radius: float,
    hole_center: tuple[float, float] = (0.0, 0.0),
):
    plate = ExtrudeWithHolesGeometry(
        _rect_loop(width_x, height_z),
        [_circle_loop(hole_radius, center=hole_center, clockwise=True)],
        thickness_y,
        center=True,
    )
    plate.rotate_x(math.pi / 2.0)
    return plate


def _yz_section(width_y: float, height_z: float, radius: float, x_pos: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z) for z, y in rounded_rect_profile(height_z, width_y, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_floodlight_mast")

    steel = model.material("steel", rgba=(0.62, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.62, 0.72, 0.82, 0.35))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base_assembly")
    base.visual(
        Box((0.300, 0.300, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="base_plate",
    )
    base.visual(
        Box((0.160, 0.160, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=dark_steel,
        name="pedestal_block",
    )
    base.visual(
        mesh_from_geometry(_square_tube_mesh(0.130, 0.086, 0.060), "floodlight_mast_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=dark_steel,
        name="mast_collar",
    )
    base.visual(
        mesh_from_geometry(_square_tube_mesh(0.108, 0.088, 0.380), "floodlight_outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        material=steel,
        name="outer_sleeve",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(0.071, 0.0, 0.098), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="clamp_stem",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.093, 0.0, 0.098), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="clamp_knob",
    )
    for sx in (-0.100, 0.100):
        for sy in (-0.100, 0.100):
            base.visual(
                Cylinder(radius=0.010, length=0.014),
                origin=Origin(xyz=(sx, sy, 0.007)),
                material=steel,
                name=f"anchor_bolt_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )
    base.inertial = Inertial.from_geometry(
        Box((0.300, 0.300, 0.460)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
    )

    sliding_mast = model.part("sliding_mast")
    sliding_mast.visual(
        mesh_from_geometry(_square_tube_mesh(0.080, 0.064, 0.900), "floodlight_inner_mast"),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=steel,
        name="inner_mast_tube",
    )
    sliding_mast.visual(
        Box((0.040, 0.072, 0.018)),
        origin=Origin(xyz=(-0.022, 0.0, 0.831)),
        material=dark_steel,
        name="top_cap",
    )
    sliding_mast.visual(
        Box((0.106, 0.106, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.332)),
        material=dark_steel,
        name="stop_collar",
    )

    yoke_mesh = _plate_with_hole_mesh(
        width_x=0.095,
        height_z=0.120,
        thickness_y=0.014,
        hole_radius=0.014,
    )
    left_plate = yoke_mesh.copy().translate(0.010, 0.105, 0.0)
    right_plate = yoke_mesh.copy().translate(0.010, -0.105, 0.0)
    left_plate.merge(right_plate)
    left_plate.merge(BoxGeometry((0.026, 0.224, 0.012)).translate(-0.032, 0.0, 0.054))
    left_plate.merge(BoxGeometry((0.028, 0.224, 0.090)).translate(-0.041, 0.0, 0.0))
    sliding_mast.visual(
        mesh_from_geometry(left_plate, "floodlight_top_yoke"),
        origin=Origin(xyz=(HEAD_BRACKET_FORWARD, 0.0, 0.860)),
        material=dark_steel,
        name="top_yoke",
    )
    sliding_mast.inertial = Inertial.from_geometry(
        Box((0.224, 0.224, 1.000)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
    )

    flood_head = model.part("flood_head")
    head_body = section_loft(
        [
            _yz_section(0.150, 0.088, 0.012, 0.020),
            _yz_section(0.172, 0.116, 0.016, 0.135),
            _yz_section(0.186, 0.132, 0.018, 0.238),
        ]
    )
    flood_head.visual(
        mesh_from_geometry(head_body, "floodlight_head_body"),
        material=charcoal,
        name="head_body",
    )
    flood_head.visual(
        Box((0.085, 0.190, 0.012)),
        origin=Origin(xyz=(0.196, 0.0, 0.072)),
        material=charcoal,
        name="visor",
    )
    flood_head.visual(
        Box((0.016, 0.170, 0.016)),
        origin=Origin(xyz=(0.232, 0.0, 0.052)),
        material=dark_steel,
        name="bezel_top",
    )
    flood_head.visual(
        Box((0.016, 0.170, 0.016)),
        origin=Origin(xyz=(0.232, 0.0, -0.052)),
        material=dark_steel,
        name="bezel_bottom",
    )
    flood_head.visual(
        Box((0.016, 0.016, 0.094)),
        origin=Origin(xyz=(0.232, 0.077, 0.0)),
        material=dark_steel,
        name="bezel_left",
    )
    flood_head.visual(
        Box((0.016, 0.016, 0.094)),
        origin=Origin(xyz=(0.232, -0.077, 0.0)),
        material=dark_steel,
        name="bezel_right",
    )
    flood_head.visual(
        Box((0.008, 0.150, 0.086)),
        origin=Origin(xyz=(0.226, 0.0, 0.0)),
        material=glass,
        name="lens_glass",
    )
    for index, y_pos in enumerate((-0.048, -0.024, 0.0, 0.024, 0.048)):
        flood_head.visual(
            Box((0.016, 0.012, 0.120)),
            origin=Origin(xyz=(0.018, y_pos, 0.0)),
            material=dark_steel,
            name=f"heat_sink_fin_{index}",
        )
    flood_head.visual(
        Box((0.052, 0.018, 0.120)),
        origin=Origin(xyz=(0.016, 0.083, 0.0)),
        material=dark_steel,
        name="left_side_ear",
    )
    flood_head.visual(
        Box((0.052, 0.018, 0.120)),
        origin=Origin(xyz=(0.016, -0.083, 0.0)),
        material=dark_steel,
        name="right_side_ear",
    )
    flood_head.visual(
        Cylinder(radius=0.0115, length=0.030),
        origin=Origin(xyz=(0.012, 0.097, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    flood_head.visual(
        Cylinder(radius=0.0115, length=0.030),
        origin=Origin(xyz=(0.012, -0.097, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    flood_head.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.012, 0.116, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_pivot_knob",
    )
    flood_head.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.012, -0.116, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_pivot_knob",
    )
    flood_head.inertial = Inertial.from_geometry(
        Box((0.250, 0.250, 0.180)),
        mass=3.6,
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=sliding_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.12,
            lower=0.0,
            upper=BASE_TO_MAST_UPPER,
        ),
    )
    model.articulation(
        "mast_to_head",
        ArticulationType.REVOLUTE,
        parent=sliding_mast,
        child=flood_head,
        origin=Origin(xyz=(HEAD_BRACKET_FORWARD, 0.0, 0.860)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=math.radians(-55.0),
            upper=HEAD_TILT_UPPER,
        ),
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

    base = object_model.get_part("base_assembly")
    sliding_mast = object_model.get_part("sliding_mast")
    flood_head = object_model.get_part("flood_head")
    mast_slide = object_model.get_articulation("base_to_mast")
    head_tilt = object_model.get_articulation("mast_to_head")

    ctx.expect_within(
        sliding_mast,
        base,
        axes="xy",
        inner_elem="inner_mast_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="mast stays centered inside the square sleeve at rest",
    )
    ctx.expect_overlap(
        sliding_mast,
        base,
        axes="z",
        elem_a="inner_mast_tube",
        elem_b="outer_sleeve",
        min_overlap=0.18,
        name="retracted mast keeps deep insertion in the sleeve",
    )

    rest_mast_pos = ctx.part_world_position(sliding_mast)
    with ctx.pose({mast_slide: BASE_TO_MAST_UPPER}):
        ctx.expect_within(
            sliding_mast,
            base,
            axes="xy",
            inner_elem="inner_mast_tube",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended mast remains guided by the sleeve",
        )
        ctx.expect_overlap(
            sliding_mast,
            base,
            axes="z",
            elem_a="inner_mast_tube",
            elem_b="outer_sleeve",
            min_overlap=0.055,
            name="extended mast still retains insertion in the sleeve",
        )
        extended_mast_pos = ctx.part_world_position(sliding_mast)

    ctx.check(
        "mast extends upward from the base",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.25,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    ctx.expect_contact(
        flood_head,
        sliding_mast,
        elem_a="left_pivot_knob",
        elem_b="top_yoke",
        contact_tol=0.0005,
        name="left pivot knob stays seated against the yoke plate",
    )
    ctx.expect_contact(
        flood_head,
        sliding_mast,
        elem_a="right_pivot_knob",
        elem_b="top_yoke",
        contact_tol=0.0005,
        name="right pivot knob stays seated against the yoke plate",
    )

    lens_rest_aabb = ctx.part_element_world_aabb(flood_head, elem="lens_glass")
    with ctx.pose({head_tilt: HEAD_TILT_UPPER * 0.75}):
        lens_tilted_aabb = ctx.part_element_world_aabb(flood_head, elem="lens_glass")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            0.5 * (mins[0] + maxs[0]),
            0.5 * (mins[1] + maxs[1]),
            0.5 * (mins[2] + maxs[2]),
        )

    lens_rest_center = _aabb_center(lens_rest_aabb)
    lens_tilted_center = _aabb_center(lens_tilted_aabb)
    ctx.check(
        "positive head tilt raises the floodlight aim",
        lens_rest_center is not None
        and lens_tilted_center is not None
        and lens_tilted_center[2] > lens_rest_center[2] + 0.08,
        details=f"rest={lens_rest_center}, tilted={lens_tilted_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
