from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((x, y, z))


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def make_frame_body() -> cq.Workplane:
    foot_x = 0.13
    foot_y = 0.24
    foot_z = 0.018
    leg_x = 0.09
    leg_y = 0.16
    leg_z = 0.44
    leg_cx = 0.225
    beam_x = 0.42
    beam_y = 0.10
    beam_z = 0.10
    beam_zc = 0.508

    shapes: list[cq.Workplane] = [
        _box((beam_x, beam_y, beam_z), (0.0, 0.0, beam_zc)),
    ]

    for sx in (-1.0, 1.0):
        leg_xc = sx * leg_cx
        inner_rib_x = leg_xc - sx * 0.028
        outer_rib_x = leg_xc + sx * 0.028
        beam_cheek_x = sx * 0.176

        shapes.extend(
            [
                _box((foot_x, foot_y, foot_z), (leg_xc, 0.0, foot_z / 2.0)),
                _box((leg_x, leg_y, leg_z), (leg_xc, 0.0, foot_z + leg_z / 2.0)),
                _box((0.018, 0.112, 0.182), (inner_rib_x, 0.0, 0.112)),
                _box((0.018, 0.104, 0.136), (inner_rib_x, 0.0, 0.388)),
                _box((0.018, 0.108, 0.160), (outer_rib_x, 0.0, 0.108)),
                _box((0.018, 0.100, 0.128), (outer_rib_x, 0.0, 0.392)),
                _box((0.034, 0.10, 0.152), (beam_cheek_x, 0.0, 0.486)),
            ]
        )

    body = _union_all(shapes)

    for sx in (-1.0, 1.0):
        leg_xc = sx * leg_cx
        body = body.cut(_box((0.042, 0.084, 0.23), (leg_xc, 0.0, 0.264)))

        for dx in (-0.032, 0.032):
            for dy in (-0.082, 0.082):
                body = body.cut(
                    _z_cylinder(
                        radius=0.0065,
                        length=foot_z + 0.004,
                        center=(leg_xc + dx, dy, -0.002),
                    )
                )

    return body


def make_carriage_body() -> cq.Workplane:
    housing = _box((0.116, 0.076, 0.092), (0.0, 0.024, 0.0))
    housing = housing.cut(_box((0.084, 0.03, 0.058), (0.0, -0.006, 0.0)))

    body = _union_all(
        [
            housing,
            _box((0.106, 0.012, 0.084), (0.0, -0.006, 0.0)),
            _box((0.088, 0.034, 0.076), (0.0, 0.055, -0.004)),
            _box((0.062, 0.028, 0.024), (0.0, 0.010, 0.056)),
            _box((0.094, 0.020, 0.47), (0.0, 0.038, -0.210)),
            _box((0.074, 0.016, 0.36), (0.0, 0.052, -0.170)),
        ]
    )

    body = body.cut(_box((0.042, 0.03, 0.19), (0.0, 0.042, -0.255)))
    body = body.cut(_box((0.060, 0.050, 0.120), (0.0, 0.066, -0.060)))
    for x in (-0.024, 0.024):
        body = body.cut(_box((0.030, 0.072, 0.320), (x, 0.060, -0.185)))
    return body


def make_ram_body() -> cq.Workplane:
    body = _union_all(
        [
            _box((0.078, 0.014, 0.185), (0.0, 0.018, -0.030)),
            _box((0.092, 0.060, 0.20), (0.0, 0.046, -0.040)),
            _box((0.064, 0.040, 0.072), (0.0, 0.074, -0.145)),
            _box((0.060, 0.040, 0.022), (0.0, 0.038, 0.071)),
        ]
    )
    body = body.cut(_box((0.038, 0.024, 0.10), (0.0, 0.018, -0.020)))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_portal_gantry")

    machine_aluminum = model.material("machine_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_cover = model.material("dark_cover", rgba=(0.19, 0.21, 0.23, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    stop_orange = model.material("stop_orange", rgba=(0.93, 0.47, 0.12, 1.0))
    ram_gray = model.material("ram_gray", rgba=(0.58, 0.60, 0.63, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_frame_body(), "gantry_frame_body"),
        material=machine_aluminum,
        name="frame_body",
    )
    frame.visual(
        Box((0.308, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.057, 0.523)),
        material=rail_steel,
        name="x_upper_rail",
    )
    frame.visual(
        Box((0.308, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.057, 0.483)),
        material=rail_steel,
        name="x_lower_rail",
    )
    frame.visual(
        Box((0.012, 0.024, 0.052)),
        origin=Origin(xyz=(-0.154, 0.064, 0.503)),
        material=stop_orange,
        name="x_left_stop",
    )
    frame.visual(
        Box((0.012, 0.024, 0.052)),
        origin=Origin(xyz=(0.154, 0.064, 0.503)),
        material=stop_orange,
        name="x_right_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage_body(), "gantry_carriage_body"),
        material=dark_cover,
        name="carriage_body",
    )
    for side, x in (("left", -0.033), ("right", 0.033)):
        carriage.visual(
            Box((0.026, 0.028, 0.018)),
            origin=Origin(xyz=(x, -0.022, 0.020)),
            material=rail_steel,
            name=f"x_u_{side}_truck",
        )
        carriage.visual(
            Box((0.026, 0.028, 0.018)),
            origin=Origin(xyz=(x, -0.022, -0.020)),
            material=rail_steel,
            name=f"x_l_{side}_truck",
        )
    carriage.visual(
        Box((0.014, 0.010, 0.320)),
        origin=Origin(xyz=(-0.024, 0.049, -0.160)),
        material=rail_steel,
        name="z_left_rail",
    )
    carriage.visual(
        Box((0.014, 0.010, 0.320)),
        origin=Origin(xyz=(0.024, 0.049, -0.160)),
        material=rail_steel,
        name="z_right_rail",
    )
    carriage.visual(
        Box((0.078, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.048, -0.020)),
        material=stop_orange,
        name="z_top_stop",
    )
    carriage.visual(
        Box((0.072, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.048, -0.446)),
        material=stop_orange,
        name="z_bottom_stop",
    )

    ram = model.part("ram")
    ram.visual(
        mesh_from_cadquery(make_ram_body(), "gantry_ram_body"),
        material=ram_gray,
        name="ram_body",
    )
    for side, x in (("left", -0.024), ("right", 0.024)):
        ram.visual(
            Box((0.018, 0.030, 0.038)),
            origin=Origin(xyz=(x, 0.0, 0.055)),
            material=rail_steel,
            name=f"z_u_{side}_truck",
        )
        ram.visual(
            Box((0.018, 0.030, 0.038)),
            origin=Origin(xyz=(x, 0.0, -0.055)),
            material=rail_steel,
            name=f"z_l_{side}_truck",
        )
    ram.visual(
        Box((0.100, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.062, -0.180)),
        material=machine_aluminum,
        name="tool_plate",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.100, 0.503)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=-0.082, upper=0.082),
    )
    model.articulation(
        "carriage_to_ram",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=ram,
        origin=Origin(xyz=(0.0, 0.069, -0.115)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.30, lower=0.0, upper=0.135),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    ram = object_model.get_part("ram")
    x_axis = object_model.get_articulation("frame_to_carriage")
    z_axis = object_model.get_articulation("carriage_to_ram")

    x_upper_rail = frame.get_visual("x_upper_rail")
    x_lower_rail = frame.get_visual("x_lower_rail")
    x_left_stop = frame.get_visual("x_left_stop")
    x_right_stop = frame.get_visual("x_right_stop")
    carriage_body = carriage.get_visual("carriage_body")
    x_u_left_truck = carriage.get_visual("x_u_left_truck")
    x_u_right_truck = carriage.get_visual("x_u_right_truck")
    x_l_left_truck = carriage.get_visual("x_l_left_truck")
    z_left_rail = carriage.get_visual("z_left_rail")
    z_top_stop = carriage.get_visual("z_top_stop")
    z_bottom_stop = carriage.get_visual("z_bottom_stop")
    ram_body = ram.get_visual("ram_body")
    z_u_left_truck = ram.get_visual("z_u_left_truck")
    z_l_left_truck = ram.get_visual("z_l_left_truck")

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

    x_limits = x_axis.motion_limits
    z_limits = z_axis.motion_limits
    ctx.check(
        "x_axis_is_horizontal_prismatic",
        x_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_axis.axis) == (1.0, 0.0, 0.0)
        and x_limits is not None
        and x_limits.lower == -0.082
        and x_limits.upper == 0.082,
        "carriage must be a horizontal prismatic axis along the beam",
    )
    ctx.check(
        "z_axis_is_vertical_prismatic",
        z_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(z_axis.axis) == (0.0, 0.0, -1.0)
        and z_limits is not None
        and z_limits.lower == 0.0
        and z_limits.upper == 0.135,
        "ram must be a downward vertical prismatic axis on the carriage",
    )

    ctx.expect_contact(
        frame,
        carriage,
        elem_a=x_upper_rail,
        elem_b=x_u_left_truck,
        name="upper_x_truck_contacts_upper_beam_rail",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a=x_lower_rail,
        elem_b=x_l_left_truck,
        name="lower_x_truck_contacts_lower_beam_rail",
    )
    ctx.expect_contact(
        carriage,
        ram,
        elem_a=z_left_rail,
        elem_b=z_u_left_truck,
        name="upper_z_truck_contacts_left_slide_rail",
    )

    with ctx.pose({x_axis: x_limits.lower}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="x",
            positive_elem=carriage_body,
            negative_elem=x_left_stop,
            min_gap=0.007,
            name="carriage_clears_left_x_end_stop",
        )

    with ctx.pose({x_axis: x_limits.upper}):
        ctx.expect_contact(
            frame,
            carriage,
            elem_a=x_upper_rail,
            elem_b=x_u_right_truck,
            name="right_x_truck_stays_on_upper_rail_at_travel_limit",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="x",
            positive_elem=x_right_stop,
            negative_elem=carriage_body,
            min_gap=0.007,
            name="carriage_clears_right_x_end_stop",
        )

    with ctx.pose({z_axis: z_limits.lower}):
        ctx.expect_gap(
            carriage,
            ram,
            axis="z",
            positive_elem=z_top_stop,
            negative_elem=ram_body,
            min_gap=0.006,
            name="ram_clears_top_z_end_stop_when_retracted",
        )

    with ctx.pose({z_axis: z_limits.upper}):
        ctx.expect_contact(
            carriage,
            ram,
            elem_a=z_left_rail,
            elem_b=z_l_left_truck,
            name="lower_z_truck_stays_on_left_slide_rail_at_full_drop",
        )
        ctx.expect_gap(
            ram,
            carriage,
            axis="z",
            positive_elem=ram_body,
            negative_elem=z_bottom_stop,
            min_gap=0.008,
            name="ram_clears_bottom_z_end_stop_at_full_drop",
        )

    with ctx.pose({x_axis: x_limits.lower, z_axis: z_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_left_low_reach")

    with ctx.pose({x_axis: x_limits.upper, z_axis: z_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_right_low_reach")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
