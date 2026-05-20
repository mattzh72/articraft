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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel_mesh(
    name: str,
    *,
    width: float,
    height: float,
    thickness: float,
    radius: float,
):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, height, radius, corner_segments=10),
            height=thickness,
            center=True,
        ),
        name,
    )


def _rounded_ring_mesh(
    name: str,
    *,
    outer_width: float,
    outer_height: float,
    inner_width: float,
    inner_height: float,
    thickness: float,
    outer_radius: float,
    inner_radius: float,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_width, outer_height, outer_radius, corner_segments=10),
            [rounded_rect_profile(inner_width, inner_height, inner_radius, corner_segments=10)],
            height=thickness,
            center=True,
        ),
        name,
    )


def _aabb_center_component(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tunnel_pet_door")

    frame_white = model.material("frame_white", rgba=(0.93, 0.94, 0.92, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.54, 0.56, 0.59, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.10, 0.10, 0.11, 1.0))
    smoked_flap = model.material("smoked_flap", rgba=(0.34, 0.38, 0.42, 0.72))

    opening_width = 0.205
    opening_height = 0.285
    tunnel_wall = 0.020
    tunnel_depth = 0.145
    tunnel_outer_width = opening_width + 2.0 * tunnel_wall
    tunnel_outer_height = opening_height + 2.0 * tunnel_wall

    trim_outer_width = 0.315
    trim_outer_height = 0.395
    trim_thickness = 0.014
    trim_overlap = 0.004
    half_tunnel_depth = tunnel_depth * 0.5
    inner_trim_z = -half_tunnel_depth - trim_thickness * 0.5 + trim_overlap
    outer_trim_z = half_tunnel_depth + trim_thickness * 0.5 - trim_overlap

    trim_ring_mesh = _rounded_ring_mesh(
        "tunnel_trim_ring",
        outer_width=trim_outer_width,
        outer_height=trim_outer_height,
        inner_width=opening_width,
        inner_height=opening_height,
        thickness=trim_thickness,
        outer_radius=0.032,
        inner_radius=0.020,
    )

    tunnel_frame = model.part("tunnel_frame")
    tunnel_frame.inertial = Inertial.from_geometry(
        Box((trim_outer_width, trim_outer_height, tunnel_depth + 2.0 * trim_thickness)),
        mass=1.8,
    )
    tunnel_frame.visual(
        Box((tunnel_wall, tunnel_outer_height, tunnel_depth)),
        origin=Origin(xyz=(opening_width * 0.5 + tunnel_wall * 0.5, 0.0, 0.0)),
        material=frame_white,
        name="right_tunnel_wall",
    )
    tunnel_frame.visual(
        Box((tunnel_wall, tunnel_outer_height, tunnel_depth)),
        origin=Origin(xyz=(-(opening_width * 0.5 + tunnel_wall * 0.5), 0.0, 0.0)),
        material=frame_white,
        name="left_tunnel_wall",
    )
    tunnel_frame.visual(
        Box((tunnel_outer_width, tunnel_wall, tunnel_depth)),
        origin=Origin(xyz=(0.0, opening_height * 0.5 + tunnel_wall * 0.5, 0.0)),
        material=frame_white,
        name="top_tunnel_wall",
    )
    tunnel_frame.visual(
        Box((tunnel_outer_width, tunnel_wall, tunnel_depth)),
        origin=Origin(xyz=(0.0, -(opening_height * 0.5 + tunnel_wall * 0.5), 0.0)),
        material=frame_white,
        name="bottom_tunnel_wall",
    )
    tunnel_frame.visual(
        trim_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, inner_trim_z)),
        material=frame_white,
        name="inner_trim",
    )
    tunnel_frame.visual(
        trim_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, outer_trim_z)),
        material=frame_white,
        name="outer_trim",
    )
    tunnel_frame.visual(
        Box((opening_width * 0.74, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, opening_height * 0.5 - 0.001, -half_tunnel_depth + 0.010)),
        material=trim_gray,
        name="inner_lintel",
    )
    tunnel_frame.visual(
        Box((opening_width * 0.76, 0.032, 0.024)),
        origin=Origin(xyz=(0.0, opening_height * 0.5 + 0.025, half_tunnel_depth - 0.002)),
        material=trim_gray,
        name="outer_hood_mount",
    )
    tunnel_frame.visual(
        Box((opening_width * 0.88, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -(opening_height * 0.5 - 0.006), -half_tunnel_depth + 0.010)),
        material=gasket_black,
        name="inner_sill_bumper",
    )

    flap_width = 0.192
    flap_height = 0.270
    flap_thickness = 0.006
    flap_panel_mesh = _rounded_panel_mesh(
        "inner_flap_panel",
        width=flap_width,
        height=flap_height,
        thickness=flap_thickness,
        radius=0.018,
    )

    inner_flap = model.part("inner_flap")
    inner_flap.inertial = Inertial.from_geometry(
        Box((flap_width, flap_height, 0.012)),
        mass=0.25,
        origin=Origin(xyz=(0.0, -flap_height * 0.5, 0.0)),
    )
    inner_flap.visual(
        flap_panel_mesh,
        origin=Origin(xyz=(0.0, -flap_height * 0.5, 0.0)),
        material=smoked_flap,
        name="flap_panel",
    )
    inner_flap.visual(
        Box((flap_width * 0.88, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=trim_gray,
        name="flap_top_rail",
    )
    inner_flap.visual(
        Cylinder(radius=0.004, length=flap_width * 0.72),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_gray,
        name="flap_hinge_barrel",
    )
    inner_flap.visual(
        Box((flap_width * 0.76, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -flap_height + 0.012, 0.001)),
        material=gasket_black,
        name="flap_bottom_strip",
    )

    hood_width = 0.255
    hood_depth = 0.078
    hood_drop = 0.052
    hood_roof_thickness = 0.005
    hood_side_thickness = 0.005

    weather_hood = model.part("weather_hood")
    weather_hood.inertial = Inertial.from_geometry(
        Box((hood_width, hood_drop, hood_depth)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -hood_drop * 0.32, hood_depth * 0.45)),
    )
    weather_hood.visual(
        Box((hood_width, hood_roof_thickness, hood_depth)),
        origin=Origin(xyz=(0.0, -hood_roof_thickness * 0.5, hood_depth * 0.5)),
        material=frame_white,
        name="hood_roof",
    )
    weather_hood.visual(
        Box((hood_side_thickness, hood_drop, hood_depth - 0.010)),
        origin=Origin(
            xyz=(hood_width * 0.5 - hood_side_thickness * 0.5, -hood_drop * 0.5, hood_depth * 0.5)
        ),
        material=frame_white,
        name="hood_right_cheek",
    )
    weather_hood.visual(
        Box((hood_side_thickness, hood_drop, hood_depth - 0.010)),
        origin=Origin(
            xyz=(-(hood_width * 0.5 - hood_side_thickness * 0.5), -hood_drop * 0.5, hood_depth * 0.5)
        ),
        material=frame_white,
        name="hood_left_cheek",
    )
    weather_hood.visual(
        Box((hood_width, hood_drop * 0.22, 0.006)),
        origin=Origin(xyz=(0.0, -hood_drop * 0.32, hood_depth - 0.003)),
        material=frame_white,
        name="hood_front_lip",
    )
    weather_hood.visual(
        Box((hood_width * 0.70, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, 0.005)),
        material=trim_gray,
        name="hood_rear_plate",
    )
    weather_hood.visual(
        Cylinder(radius=0.004, length=hood_width * 0.68),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_gray,
        name="hood_hinge_barrel",
    )

    model.articulation(
        "inner_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=tunnel_frame,
        child=inner_flap,
        origin=Origin(xyz=(0.0, opening_height * 0.5 - 0.008, -half_tunnel_depth + 0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "hood_hinge",
        ArticulationType.REVOLUTE,
        parent=tunnel_frame,
        child=weather_hood,
        origin=Origin(xyz=(0.0, opening_height * 0.5 + 0.040, half_tunnel_depth + 0.012)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=1.1),
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

    tunnel_frame = object_model.get_part("tunnel_frame")
    inner_flap = object_model.get_part("inner_flap")
    weather_hood = object_model.get_part("weather_hood")
    flap_hinge = object_model.get_articulation("inner_flap_hinge")
    hood_hinge = object_model.get_articulation("hood_hinge")

    ctx.check(
        "inner flap hinge uses a horizontal x-axis",
        flap_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"axis={flap_hinge.axis}",
    )
    ctx.check(
        "weather hood hinge uses a horizontal upper x-axis",
        hood_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"axis={hood_hinge.axis}",
    )
    ctx.check(
        "flap hinge is inside the tunnel and hood hinge is outside",
        flap_hinge.origin.xyz[2] < 0.0 and hood_hinge.origin.xyz[2] > 0.0,
        details=f"flap_origin={flap_hinge.origin.xyz}, hood_origin={hood_hinge.origin.xyz}",
    )

    ctx.expect_gap(
        inner_flap,
        tunnel_frame,
        axis="z",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="flap_panel",
        negative_elem="inner_trim",
        name="inner flap hangs just behind the interior trim",
    )
    ctx.expect_gap(
        weather_hood,
        tunnel_frame,
        axis="z",
        min_gap=0.001,
        max_gap=0.008,
        positive_elem="hood_roof",
        negative_elem="outer_trim",
        name="weather hood sits just outside the exterior trim",
    )

    rest_flap_bottom = ctx.part_element_world_aabb(inner_flap, elem="flap_bottom_strip")
    with ctx.pose({flap_hinge: 0.9}):
        opened_flap_bottom = ctx.part_element_world_aabb(inner_flap, elem="flap_bottom_strip")
    rest_flap_z = _aabb_center_component(rest_flap_bottom, 2)
    opened_flap_z = _aabb_center_component(opened_flap_bottom, 2)
    ctx.check(
        "inner flap swings outward through the tunnel",
        rest_flap_z is not None and opened_flap_z is not None and opened_flap_z > rest_flap_z + 0.055,
        details=f"rest_z={rest_flap_z}, opened_z={opened_flap_z}",
    )

    rest_hood_front = ctx.part_element_world_aabb(weather_hood, elem="hood_front_lip")
    with ctx.pose({hood_hinge: 0.85}):
        lifted_hood_front = ctx.part_element_world_aabb(weather_hood, elem="hood_front_lip")
    rest_hood_y = _aabb_center_component(rest_hood_front, 1)
    lifted_hood_y = _aabb_center_component(lifted_hood_front, 1)
    ctx.check(
        "weather hood lifts upward on its own hinge",
        rest_hood_y is not None and lifted_hood_y is not None and lifted_hood_y > rest_hood_y + 0.040,
        details=f"rest_y={rest_hood_y}, lifted_y={lifted_hood_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
