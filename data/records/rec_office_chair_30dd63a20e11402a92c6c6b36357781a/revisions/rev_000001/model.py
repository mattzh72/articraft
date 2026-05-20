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
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rounded_xy_section(
    width: float,
    depth: float,
    z: float,
    *,
    radius: float,
    center_y: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    safe_radius = min(radius, width * 0.48, depth * 0.48)
    return [
        (x, y + center_y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            safe_radius,
            corner_segments=corner_segments,
        )
    ]


def _cushion_mesh(
    name: str,
    sections: list[list[tuple[float, float, float]]],
):
    return mesh_from_geometry(section_loft(sections), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="operator_chair")

    fabric = model.material("fabric_charcoal", rgba=(0.16, 0.16, 0.18, 1.0))
    frame = model.material("frame_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    steel = model.material("steel_grey", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    trim = model.material("trim_black", rgba=(0.10, 0.10, 0.11, 1.0))

    seat_cushion_mesh = _cushion_mesh(
        "operator_chair_seat_cushion",
        [
            _rounded_xy_section(0.410, 0.365, 0.028, radius=0.045, center_y=0.008),
            _rounded_xy_section(0.460, 0.430, 0.056, radius=0.058, center_y=0.012),
            _rounded_xy_section(0.445, 0.410, 0.082, radius=0.052, center_y=0.016),
        ],
    )
    backrest_panel_mesh = _cushion_mesh(
        "operator_chair_backrest_panel",
        [
            _rounded_xy_section(0.300, 0.050, 0.110, radius=0.022, center_y=-0.040),
            _rounded_xy_section(0.360, 0.058, 0.245, radius=0.030, center_y=-0.040),
            _rounded_xy_section(0.325, 0.052, 0.390, radius=0.026, center_y=-0.043),
        ],
    )

    base_frame = model.part("base_frame")
    base_frame.visual(
        Cylinder(radius=0.072, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=frame,
        name="hub_lower",
    )
    base_frame.visual(
        Cylinder(radius=0.050, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=frame,
        name="hub_upper",
    )
    caster_angles = [math.radians(angle_deg) for angle_deg in (90.0, 18.0, -54.0, -126.0, 162.0)]
    for index, angle in enumerate(caster_angles):
        leg_center_radius = 0.157
        socket_radius = 0.285
        base_frame.visual(
            Box((0.260, 0.045, 0.018)),
            origin=Origin(
                xyz=(
                    leg_center_radius * math.cos(angle),
                    leg_center_radius * math.sin(angle),
                    0.086,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=frame,
            name=f"leg_{index}",
        )
        base_frame.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(
                xyz=(
                    socket_radius * math.cos(angle),
                    socket_radius * math.sin(angle),
                    0.089,
                ),
            ),
            material=frame,
            name=f"caster_socket_{index}",
        )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.620, 0.620, 0.120)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.038, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=steel,
        name="column_collar",
    )
    column.visual(
        Cylinder(radius=0.030, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=trim,
        name="column_shroud",
    )
    column.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=steel,
        name="piston",
    )
    column.visual(
        Cylinder(radius=0.050, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.286)),
        material=frame,
        name="column_head",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.292)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base_frame,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.180, 0.140, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="swivel_plate",
    )
    seat.visual(
        Box((0.260, 0.200, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=frame,
        name="underframe",
    )
    seat.visual(
        Box((0.084, 0.034, 0.016)),
        origin=Origin(xyz=(0.170, 0.055, 0.008)),
        material=frame,
        name="lever_mount",
    )
    seat.visual(
        Box((0.220, 0.140, 0.016)),
        origin=Origin(xyz=(0.0, -0.155, 0.020)),
        material=frame,
        name="rear_support",
    )
    seat.visual(
        Box((0.210, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.210, 0.024)),
        material=frame,
        name="back_brace",
    )
    seat.visual(
        Box((0.014, 0.024, 0.050)),
        origin=Origin(xyz=(0.092, -0.220, 0.040)),
        material=frame,
        name="right_back_bracket",
    )
    seat.visual(
        Box((0.014, 0.024, 0.050)),
        origin=Origin(xyz=(-0.092, -0.220, 0.040)),
        material=frame,
        name="left_back_bracket",
    )
    seat.visual(
        seat_cushion_mesh,
        material=fabric,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.460, 0.430, 0.090)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.012, 0.048)),
    )

    model.articulation(
        "column_to_seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.0085, length=0.170),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    backrest.visual(
        Box((0.014, 0.036, 0.018)),
        origin=Origin(xyz=(0.076, -0.018, 0.010)),
        material=frame,
        name="right_hinge_link",
    )
    backrest.visual(
        Box((0.014, 0.036, 0.018)),
        origin=Origin(xyz=(-0.076, -0.018, 0.010)),
        material=frame,
        name="left_hinge_link",
    )
    backrest.visual(
        Cylinder(radius=0.010, length=0.180),
        origin=Origin(xyz=(0.082, -0.024, 0.098)),
        material=frame,
        name="right_upright",
    )
    backrest.visual(
        Cylinder(radius=0.010, length=0.180),
        origin=Origin(xyz=(-0.082, -0.024, 0.098)),
        material=frame,
        name="left_upright",
    )
    backrest.visual(
        Box((0.220, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.030, 0.102)),
        material=frame,
        name="lower_crossbar",
    )
    backrest.visual(
        backrest_panel_mesh,
        material=fabric,
        name="back_panel",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.360, 0.080, 0.420)),
        mass=3.0,
        origin=Origin(xyz=(0.0, -0.040, 0.220)),
    )

    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.220, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.2,
            lower=0.0,
            upper=0.55,
        ),
    )

    tilt_lever = model.part("tilt_lever")
    tilt_lever.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="pivot_barrel",
    )
    tilt_lever.visual(
        Cylinder(radius=0.004, length=0.090),
        origin=Origin(xyz=(0.045, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="lever_arm",
    )
    tilt_lever.visual(
        Box((0.020, 0.010, 0.030)),
        origin=Origin(xyz=(0.090, 0.0, -0.006)),
        material=trim,
        name="paddle",
    )
    tilt_lever.inertial = Inertial.from_geometry(
        Box((0.110, 0.026, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.050, 0.0, -0.010)),
    )

    model.articulation(
        "seat_to_tilt_lever",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=tilt_lever,
        origin=Origin(xyz=(0.195, 0.055, -0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=3.0,
            lower=-0.30,
            upper=0.45,
        ),
    )

    for index, angle in enumerate(caster_angles):
        caster_yoke = model.part(f"caster_yoke_{index}")
        caster_yoke.visual(
            Cylinder(radius=0.007, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=steel,
            name="stem",
        )
        caster_yoke.visual(
            Box((0.032, 0.020, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.028)),
            material=frame,
            name="fork_bridge",
        )
        caster_yoke.visual(
            Box((0.005, 0.020, 0.026)),
            origin=Origin(xyz=(0.012, 0.0, -0.042)),
            material=frame,
            name="right_cheek",
        )
        caster_yoke.visual(
            Box((0.005, 0.020, 0.026)),
            origin=Origin(xyz=(-0.012, 0.0, -0.042)),
            material=frame,
            name="left_cheek",
        )
        caster_yoke.visual(
            Box((0.010, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, 0.014, -0.020)),
            material=frame,
            name="nose",
        )
        caster_yoke.inertial = Inertial.from_geometry(
            Box((0.040, 0.024, 0.062)),
            mass=0.09,
            origin=Origin(xyz=(0.0, 0.0, -0.031)),
        )

        model.articulation(
            f"base_to_caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base_frame,
            child=caster_yoke,
            origin=Origin(
                xyz=(
                    0.285 * math.cos(angle),
                    0.285 * math.sin(angle),
                    0.081,
                )
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=6.0),
        )

        caster_wheel = model.part(f"caster_wheel_{index}")
        caster_wheel.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="tire",
        )
        caster_wheel.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="hub",
        )
        caster_wheel.visual(
            Box((0.004, 0.008, 0.006)),
            origin=Origin(xyz=(0.0, 0.012, 0.022)),
            material=steel,
            name="marker",
        )
        caster_wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.026, length=0.018),
            mass=0.06,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

        model.articulation(
            f"caster_yoke_{index}_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster_yoke,
            child=caster_wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
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

    column = object_model.get_part("column")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    tilt_lever = object_model.get_part("tilt_lever")
    caster_yoke = object_model.get_part("caster_yoke_0")
    caster_wheel = object_model.get_part("caster_wheel_0")

    seat_swivel = object_model.get_articulation("column_to_seat_swivel")
    backrest_hinge = object_model.get_articulation("seat_to_backrest")
    lever_hinge = object_model.get_articulation("seat_to_tilt_lever")
    caster_swivel = object_model.get_articulation("base_to_caster_swivel_0")
    caster_spin = object_model.get_articulation("caster_yoke_0_to_wheel")

    column_head = column.get_visual("column_head")
    swivel_plate = seat.get_visual("swivel_plate")
    back_panel = backrest.get_visual("back_panel")
    lever_paddle = tilt_lever.get_visual("paddle")
    caster_nose = caster_yoke.get_visual("nose")
    wheel_marker = caster_wheel.get_visual("marker")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[index] + high[index]) * 0.5 for index in range(3))

    ctx.expect_overlap(
        seat,
        column,
        axes="xy",
        elem_a=swivel_plate,
        elem_b=column_head,
        min_overlap=0.09,
        name="seat plate stays centered above the column head",
    )
    ctx.expect_gap(
        seat,
        column,
        axis="z",
        positive_elem=swivel_plate,
        negative_elem=column_head,
        max_gap=0.002,
        max_penetration=0.001,
        name="seat plate sits directly on the column head",
    )
    ctx.check(
        "primary chair joints use the intended axes",
        seat_swivel.axis == (0.0, 0.0, 1.0)
        and backrest_hinge.axis == (1.0, 0.0, 0.0)
        and lever_hinge.axis == (0.0, 1.0, 0.0)
        and caster_swivel.axis == (0.0, 0.0, 1.0)
        and caster_spin.axis == (1.0, 0.0, 0.0),
        details=(
            f"seat={seat_swivel.axis}, backrest={backrest_hinge.axis}, "
            f"lever={lever_hinge.axis}, caster_swivel={caster_swivel.axis}, "
            f"caster_spin={caster_spin.axis}"
        ),
    )

    backrest_rest_center = _aabb_center(ctx.part_element_world_aabb(backrest, elem=back_panel))
    with ctx.pose({backrest_hinge: 0.42}):
        backrest_reclined_center = _aabb_center(ctx.part_element_world_aabb(backrest, elem=back_panel))
    ctx.check(
        "backrest reclines rearward on its transverse hinge",
        backrest_rest_center is not None
        and backrest_reclined_center is not None
        and backrest_reclined_center[1] < backrest_rest_center[1] - 0.04,
        details=f"rest={backrest_rest_center}, reclined={backrest_reclined_center}",
    )

    lever_rest_center = _aabb_center(ctx.part_element_world_aabb(tilt_lever, elem=lever_paddle))
    with ctx.pose({lever_hinge: 0.35}):
        lever_pulled_center = _aabb_center(ctx.part_element_world_aabb(tilt_lever, elem=lever_paddle))
    ctx.check(
        "under-seat lever rotates downward when actuated",
        lever_rest_center is not None
        and lever_pulled_center is not None
        and lever_pulled_center[2] < lever_rest_center[2] - 0.01,
        details=f"rest={lever_rest_center}, pulled={lever_pulled_center}",
    )

    swivel_rest_center = _aabb_center(ctx.part_element_world_aabb(tilt_lever, elem=lever_paddle))
    with ctx.pose({seat_swivel: 0.75}):
        swivel_rotated_center = _aabb_center(ctx.part_element_world_aabb(tilt_lever, elem=lever_paddle))
    ctx.check(
        "seat swivel carries the under-seat lever around the column axis",
        swivel_rest_center is not None
        and swivel_rotated_center is not None
        and abs(swivel_rotated_center[1] - swivel_rest_center[1]) > 0.08,
        details=f"rest={swivel_rest_center}, swiveled={swivel_rotated_center}",
    )

    caster_nose_rest = _aabb_center(ctx.part_element_world_aabb(caster_yoke, elem=caster_nose))
    with ctx.pose({caster_swivel: 1.0}):
        caster_nose_turned = _aabb_center(ctx.part_element_world_aabb(caster_yoke, elem=caster_nose))
    ctx.check(
        "caster fork swivels around a vertical stem",
        caster_nose_rest is not None
        and caster_nose_turned is not None
        and (
            abs(caster_nose_turned[0] - caster_nose_rest[0]) > 0.008
            or abs(caster_nose_turned[1] - caster_nose_rest[1]) > 0.008
        ),
        details=f"rest={caster_nose_rest}, turned={caster_nose_turned}",
    )

    wheel_marker_rest = _aabb_center(ctx.part_element_world_aabb(caster_wheel, elem=wheel_marker))
    with ctx.pose({caster_spin: math.pi / 2.0}):
        wheel_marker_spun = _aabb_center(ctx.part_element_world_aabb(caster_wheel, elem=wheel_marker))
    ctx.check(
        "caster wheel spins about its horizontal axle",
        wheel_marker_rest is not None
        and wheel_marker_spun is not None
        and abs(wheel_marker_spun[2] - wheel_marker_rest[2]) > 0.008,
        details=f"rest={wheel_marker_rest}, spun={wheel_marker_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
