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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wrap_profile_onto_surface,
)


def _arc_points_xz(
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int,
    y: float = 0.0,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        points.append((radius * math.cos(angle), y, radius * math.sin(angle)))
    return points


def _wrapped_patch(
    *,
    name: str,
    radius: float,
    profile: list[tuple[float, float]],
    direction: tuple[float, float, float],
    thickness: float = 0.0012,
    relief: float = 0.0002,
    spin: float = 0.0,
):
    return mesh_from_geometry(
        wrap_profile_onto_surface(
            profile,
            Sphere(radius=radius),
            thickness=thickness,
            direction=direction,
            visible_relief=relief,
            mapping="intrinsic",
            spin=spin,
            surface_max_edge=0.006,
        ),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_globe")

    walnut = model.material("walnut", rgba=(0.39, 0.25, 0.16, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.75, 0.63, 0.34, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.58, 0.49, 0.25, 1.0))
    knob_black = model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))
    ocean_blue = model.material("ocean_blue", rgba=(0.18, 0.46, 0.73, 1.0))
    land_green = model.material("land_green", rgba=(0.42, 0.58, 0.26, 1.0))
    parchment = model.material("parchment", rgba=(0.90, 0.85, 0.70, 1.0))

    globe_radius = 0.102
    meridian_radius = 0.132
    trunnion_height = 0.252
    polar_tilt = math.radians(23.5)
    axis_x = math.sin(polar_tilt)
    axis_z = math.cos(polar_tilt)
    spindle_half = 0.112

    north_tip = (axis_x * spindle_half, 0.0, axis_z * spindle_half)
    south_tip = (-north_tip[0], 0.0, -north_tip[2])
    north_support_end = (north_tip[0] + axis_x * 0.003, 0.0, north_tip[2] + axis_z * 0.003)
    lower_hub = (south_tip[0] - axis_x * 0.018, 0.026, south_tip[2] - axis_z * 0.018)

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.085, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=walnut,
        name="base_disc",
    )
    stand.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=walnut,
        name="base_collar",
    )
    stand.visual(
        Cylinder(radius=0.017, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=aged_brass,
        name="pedestal_column",
    )
    stand.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=aged_brass,
        name="yoke_hub",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=warm_brass,
        name="hub_neck",
    )

    left_arm_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.024, -0.010, 0.136),
                (-0.070, -0.015, 0.143),
                (-0.116, -0.019, 0.186),
                (-0.153, -0.019, trunnion_height),
            ],
            radius=0.0075,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
        "left_yoke_arm",
    )
    right_arm_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.024, -0.010, 0.136),
                (0.070, -0.015, 0.143),
                (0.116, -0.019, 0.186),
                (0.153, -0.019, trunnion_height),
            ],
            radius=0.0075,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
        "right_yoke_arm",
    )
    stand.visual(left_arm_mesh, material=aged_brass, name="left_yoke_arm")
    stand.visual(right_arm_mesh, material=aged_brass, name="right_yoke_arm")
    stand.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(-0.162, 0.0, trunnion_height), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="left_bearing_sleeve",
    )
    stand.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.162, 0.0, trunnion_height), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="right_bearing_sleeve",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.34, 0.18, 0.19)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    meridian = model.part("meridian")
    meridian_arc = mesh_from_geometry(
        tube_from_spline_points(
            _arc_points_xz(meridian_radius, math.pi, 0.0, segments=24),
            radius=0.0065,
            samples_per_segment=4,
            radial_segments=18,
            cap_ends=True,
        ),
        "meridian_arc",
    )
    meridian.visual(meridian_arc, material=warm_brass, name="meridian_arc")
    meridian.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(xyz=(-meridian_radius, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="left_trunnion_boss",
    )
    meridian.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(xyz=(meridian_radius, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="right_trunnion_boss",
    )
    meridian.visual(
        Cylinder(radius=0.004, length=0.013),
        origin=Origin(xyz=(-0.1465, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="left_trunnion_pin",
    )
    meridian.visual(
        Cylinder(radius=0.004, length=0.013),
        origin=Origin(xyz=(0.1465, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="right_trunnion_pin",
    )

    top_anchor = (
        north_support_end[0] + 0.001,
        0.0,
        math.sqrt(max(meridian_radius * meridian_radius - (north_support_end[0] + 0.001) ** 2, 0.0)),
    )
    top_support = mesh_from_geometry(
        tube_from_spline_points(
            [
                top_anchor,
                (north_support_end[0], 0.0, north_support_end[2] + 0.018),
                north_support_end,
            ],
            radius=0.003,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        ),
        "north_pivot_support",
    )
    meridian.visual(top_support, material=aged_brass, name="north_pivot_support")
    meridian.visual(
        Cylinder(radius=0.0042, length=0.014),
        origin=Origin(
            xyz=(north_tip[0] + axis_x * 0.006, 0.0, north_tip[2] + axis_z * 0.006),
            rpy=(0.0, polar_tilt, 0.0),
        ),
        material=aged_brass,
        name="north_pivot",
    )

    left_lower_strut = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-meridian_radius, 0.006, 0.000),
                (-0.105, 0.022, -0.048),
                (-0.082, 0.028, -0.090),
                lower_hub,
            ],
            radius=0.0035,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        ),
        "left_lower_pivot_strut",
    )
    right_lower_strut = mesh_from_geometry(
        tube_from_spline_points(
            [
                (meridian_radius, 0.006, 0.000),
                (0.105, 0.022, -0.048),
                (0.030, 0.030, -0.110),
                lower_hub,
            ],
            radius=0.0035,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        ),
        "right_lower_pivot_strut",
    )
    south_pivot_arm = mesh_from_geometry(
        tube_from_spline_points(
            [
                lower_hub,
                (-0.050, 0.015, -0.112),
                (south_tip[0] - axis_x * 0.003, 0.0, south_tip[2] - axis_z * 0.003),
            ],
            radius=0.0032,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        ),
        "south_pivot_arm",
    )
    meridian.visual(left_lower_strut, material=aged_brass, name="left_lower_strut")
    meridian.visual(right_lower_strut, material=aged_brass, name="right_lower_strut")
    meridian.visual(
        Sphere(radius=0.0055),
        origin=Origin(xyz=lower_hub),
        material=aged_brass,
        name="south_pivot_hub",
    )
    meridian.visual(south_pivot_arm, material=aged_brass, name="south_pivot_arm")
    meridian.visual(
        Cylinder(radius=0.0042, length=0.014),
        origin=Origin(
            xyz=(south_tip[0] - axis_x * 0.006, 0.0, south_tip[2] - axis_z * 0.006),
            rpy=(0.0, polar_tilt, 0.0),
        ),
        material=aged_brass,
        name="south_pivot",
    )
    meridian.visual(
        Box((0.034, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=parchment,
        name="index_plate",
    )
    meridian.inertial = Inertial.from_geometry(
        Box((0.29, 0.08, 0.28)),
        mass=0.28,
        origin=Origin(xyz=(0.0, -0.012, 0.020)),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=globe_radius),
        material=ocean_blue,
        name="ocean_sphere",
    )
    globe.visual(
        Cylinder(radius=0.0035, length=spindle_half * 2.0),
        material=aged_brass,
        name="polar_spindle",
    )
    globe.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, globe_radius - 0.002)),
        material=aged_brass,
        name="north_cap",
    )
    globe.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -(globe_radius - 0.002))),
        material=aged_brass,
        name="south_cap",
    )

    americas_mesh = _wrapped_patch(
        name="americas_patch",
        radius=globe_radius,
        profile=[
            (-0.011, -0.030),
            (0.001, -0.032),
            (0.012, -0.020),
            (0.010, -0.007),
            (0.016, 0.006),
            (0.006, 0.032),
            (-0.008, 0.028),
            (-0.016, 0.010),
            (-0.013, -0.012),
        ],
        direction=(-0.83, -0.18, 0.14),
        spin=0.15,
    )
    eurasia_mesh = _wrapped_patch(
        name="eurasia_patch",
        radius=globe_radius,
        profile=[
            (-0.028, -0.008),
            (-0.015, -0.022),
            (0.006, -0.024),
            (0.026, -0.012),
            (0.020, 0.010),
            (0.008, 0.020),
            (-0.004, 0.028),
            (-0.024, 0.018),
            (-0.030, 0.004),
        ],
        direction=(0.82, 0.18, 0.10),
        spin=-0.10,
    )
    australia_mesh = _wrapped_patch(
        name="australia_patch",
        radius=globe_radius,
        profile=[
            (-0.013, -0.008),
            (-0.002, -0.015),
            (0.011, -0.010),
            (0.013, 0.002),
            (0.004, 0.011),
            (-0.009, 0.009),
            (-0.015, 0.000),
        ],
        direction=(0.60, -0.64, -0.22),
        spin=0.20,
    )
    legend_mesh = _wrapped_patch(
        name="legend_plate_patch",
        radius=globe_radius,
        profile=[(-0.012, -0.006), (0.012, -0.006), (0.012, 0.006), (-0.012, 0.006)],
        direction=(0.12, 0.99, -0.04),
        thickness=0.0010,
        relief=0.00015,
        spin=0.10,
    )
    globe.visual(americas_mesh, material=land_green, name="americas")
    globe.visual(eurasia_mesh, material=land_green, name="eurasia")
    globe.visual(australia_mesh, material=land_green, name="australia")
    globe.visual(legend_mesh, material=parchment, name="legend_plate")
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=globe_radius),
        mass=0.64,
    )

    left_knob = model.part("left_knob")
    left_knob.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="left_knob_shaft",
    )
    left_knob.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.029, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="left_knob_body",
    )
    left_knob.visual(
        Box((0.006, 0.003, 0.012)),
        origin=Origin(xyz=(-0.034, 0.0, 0.014)),
        material=parchment,
        name="pointer",
    )
    left_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.038),
        mass=0.03,
        origin=Origin(xyz=(-0.019, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_knob = model.part("right_knob")
    right_knob.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="right_knob_shaft",
    )
    right_knob.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="right_knob_body",
    )
    right_knob.visual(
        Box((0.006, 0.003, 0.012)),
        origin=Origin(xyz=(0.034, 0.0, 0.014)),
        material=parchment,
        name="pointer",
    )
    right_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.038),
        mass=0.03,
        origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "trunnion_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, trunnion_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(rpy=(0.0, polar_tilt, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=3.0,
        ),
    )
    model.articulation(
        "left_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=left_knob,
        origin=Origin(xyz=(-0.171, 0.0, trunnion_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )
    model.articulation(
        "right_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=right_knob,
        origin=Origin(xyz=(0.171, 0.0, trunnion_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")

    trunnion_tilt = object_model.get_articulation("trunnion_tilt")
    globe_spin = object_model.get_articulation("globe_spin")
    right_knob_spin = object_model.get_articulation("right_knob_spin")

    ctx.expect_contact(
        meridian,
        stand,
        elem_a="left_trunnion_pin",
        elem_b="left_bearing_sleeve",
        contact_tol=0.0005,
        name="left trunnion pin seats in left yoke sleeve",
    )
    ctx.expect_contact(
        meridian,
        stand,
        elem_a="right_trunnion_pin",
        elem_b="right_bearing_sleeve",
        contact_tol=0.0005,
        name="right trunnion pin seats in right yoke sleeve",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="polar_spindle",
        elem_b="north_pivot",
        contact_tol=0.0005,
        name="north polar spindle meets upper pivot",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="polar_spindle",
        elem_b="south_pivot",
        contact_tol=0.0005,
        name="south polar spindle meets lower pivot",
    )
    ctx.expect_contact(
        left_knob,
        stand,
        elem_a="left_knob_shaft",
        elem_b="left_bearing_sleeve",
        contact_tol=0.0005,
        name="left adjustment knob mounts to the left yoke side",
    )
    ctx.expect_contact(
        right_knob,
        stand,
        elem_a="right_knob_shaft",
        elem_b="right_bearing_sleeve",
        contact_tol=0.0005,
        name="right adjustment knob mounts to the right yoke side",
    )

    index_rest = _aabb_center(ctx.part_element_world_aabb(meridian, elem="index_plate"))
    with ctx.pose({trunnion_tilt: 0.55}):
        index_tilted = _aabb_center(ctx.part_element_world_aabb(meridian, elem="index_plate"))
    meridian_moves = (
        index_rest is not None
        and index_tilted is not None
        and index_tilted[1] < index_rest[1] - 0.05
        and index_tilted[2] < index_rest[2] - 0.01
    )
    ctx.check(
        "meridian rotates about the horizontal trunnion axis",
        meridian_moves,
        details=f"rest={index_rest}, tilted={index_tilted}",
    )

    legend_rest = _aabb_center(ctx.part_element_world_aabb(globe, elem="legend_plate"))
    with ctx.pose({globe_spin: math.pi / 2.0}):
        legend_spun = _aabb_center(ctx.part_element_world_aabb(globe, elem="legend_plate"))
    globe_moves = (
        legend_rest is not None
        and legend_spun is not None
        and math.dist(legend_rest, legend_spun) > 0.04
    )
    ctx.check(
        "globe spins on its tilted polar axis",
        globe_moves,
        details=f"rest={legend_rest}, spun={legend_spun}",
    )

    pointer_rest = _aabb_center(ctx.part_element_world_aabb(right_knob, elem="pointer"))
    with ctx.pose({right_knob_spin: math.pi / 2.0}):
        pointer_turned = _aabb_center(ctx.part_element_world_aabb(right_knob, elem="pointer"))
    knob_moves = (
        pointer_rest is not None
        and pointer_turned is not None
        and abs(pointer_turned[1] - pointer_rest[1]) > 0.010
        and abs(pointer_turned[2] - pointer_rest[2]) > 0.010
    )
    ctx.check(
        "side knob rotates on its own small axis",
        knob_moves,
        details=f"rest={pointer_rest}, turned={pointer_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
