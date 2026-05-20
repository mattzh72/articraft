from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


DECK_LENGTH = 0.82
DECK_WIDTH = 0.215
DECK_THICKNESS = 0.012
TRUCK_STATION_X = 0.19
WHEEL_RADIUS = 0.0275
WHEEL_WIDTH = 0.032
AXLE_RADIUS = 0.0042
AXLE_HALF_SPAN = 0.105
AXLE_CENTER_Z = -0.033


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _normalize(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2]))
    return (vec[0] / mag, vec[1] / mag, vec[2] / mag)


def _scale(vec: tuple[float, float, float], scale: float) -> tuple[float, float, float]:
    return (vec[0] * scale, vec[1] * scale, vec[2] * scale)


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _box_geom(size: tuple[float, float, float], center: tuple[float, float, float]) -> MeshGeometry:
    return BoxGeometry(size).translate(*center)


def _y_cylinder_geom(
    radius: float,
    length: float,
    center: tuple[float, float, float],
    *,
    radial_segments: int = 24,
) -> MeshGeometry:
    return CylinderGeometry(radius, length, radial_segments=radial_segments).rotate_x(pi / 2.0).translate(*center)


def _axis_cylinder_geom(
    radius: float,
    length: float,
    axis: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    radial_segments: int = 24,
) -> MeshGeometry:
    return CylinderGeometry(radius, length, radial_segments=radial_segments).rotate_y(atan2(axis[0], axis[2])).translate(*center)


def _deck_loop(x_pos: float, width: float, z_offset: float, concave: float) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    half_thickness = DECK_THICKNESS * 0.5
    yz_points = [
        (-half_width, z_offset + 0.0012),
        (-half_width * 0.44, z_offset + half_thickness * 0.58),
        (-half_width * 0.16, z_offset + half_thickness + concave * 0.72),
        (0.0, z_offset + half_thickness + concave),
        (half_width * 0.16, z_offset + half_thickness + concave * 0.72),
        (half_width * 0.44, z_offset + half_thickness * 0.58),
        (half_width, z_offset + 0.0012),
        (half_width, z_offset - 0.0012),
        (half_width * 0.34, z_offset - half_thickness * 0.78),
        (0.0, z_offset - half_thickness),
        (-half_width * 0.34, z_offset - half_thickness * 0.78),
        (-half_width, z_offset - 0.0012),
    ]
    return [(x_pos, y_pos, z_pos) for y_pos, z_pos in yz_points]


def _build_deck_mesh() -> MeshGeometry:
    sections = [
        _deck_loop(-0.405, 0.060, 0.055, 0.0010),
        _deck_loop(-0.315, 0.145, 0.018, 0.0018),
        _deck_loop(-0.200, 0.198, 0.000, 0.0026),
        _deck_loop(0.000, 0.215, 0.000, 0.0030),
        _deck_loop(0.200, 0.198, 0.000, 0.0026),
        _deck_loop(0.315, 0.145, 0.018, 0.0018),
        _deck_loop(0.405, 0.060, 0.055, 0.0010),
    ]
    return section_loft(sections)


def _build_truck_base_mesh(facing_sign: float) -> MeshGeometry:
    mesh = _box_geom((0.088, 0.058, 0.004), (0.0, 0.0, -0.002))
    mesh.merge(_box_geom((0.052, 0.040, 0.006), (-0.006 * facing_sign, 0.0, -0.007)))
    mesh.merge(_box_geom((0.024, 0.022, 0.006), (0.016 * facing_sign, 0.0, -0.011)))
    mesh.merge(_box_geom((0.016, 0.020, 0.006), (-0.018 * facing_sign, 0.0, -0.010)))
    for x_pos in (-0.019, 0.019):
        for y_pos in (-0.015, 0.015):
            mesh.merge(CylinderGeometry(0.0045, 0.005, radial_segments=18).translate(x_pos, y_pos, -0.0025))
    return mesh


def _build_hanger_mesh(facing_sign: float) -> MeshGeometry:
    axle_center_x = -0.020 * facing_sign
    inner_ring_y = AXLE_HALF_SPAN - (WHEEL_WIDTH * 0.5) - 0.006
    mesh = _box_geom((0.024, 0.026, 0.014), (-0.005 * facing_sign, 0.0, -0.018))
    mesh.merge(_box_geom((0.036, 0.030, 0.012), (-0.013 * facing_sign, 0.0, -0.024)))
    mesh.merge(_box_geom((0.020, 0.166, 0.010), (axle_center_x, 0.0, -0.031)))
    mesh.merge(_y_cylinder_geom(0.0045, 0.166, (axle_center_x, 0.0, AXLE_CENTER_Z), radial_segments=22))
    mesh.merge(_y_cylinder_geom(0.009, 0.012, (axle_center_x, inner_ring_y, AXLE_CENTER_Z), radial_segments=22))
    mesh.merge(_y_cylinder_geom(0.009, 0.012, (axle_center_x, -inner_ring_y, AXLE_CENTER_Z), radial_segments=22))
    return mesh


def _truck_axis(facing_sign: float) -> tuple[float, float, float]:
    return _normalize((-0.84 * facing_sign, 0.0, 0.54))


def _build_wheel_shell() -> MeshGeometry:
    outer_profile = [
        (0.0175, -WHEEL_WIDTH * 0.5),
        (0.0245, -WHEEL_WIDTH * 0.5),
        (0.0265, -0.0115),
        (WHEEL_RADIUS, -0.0045),
        (WHEEL_RADIUS, 0.0045),
        (0.0265, 0.0115),
        (0.0245, WHEEL_WIDTH * 0.5),
        (0.0175, WHEEL_WIDTH * 0.5),
    ]
    inner_profile = [
        (AXLE_RADIUS, -WHEEL_WIDTH * 0.5),
        (0.0068, -0.0100),
        (AXLE_RADIUS, 0.0),
        (0.0068, 0.0100),
        (AXLE_RADIUS, WHEEL_WIDTH * 0.5),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_calibration_skateboard")

    deck_black = model.material("deck_black", rgba=(0.11, 0.12, 0.13, 1.0))
    datum_silver = model.material("datum_silver", rgba=(0.78, 0.80, 0.84, 1.0))
    truck_metal = model.material("truck_metal", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    bushing_amber = model.material("bushing_amber", rgba=(0.84, 0.58, 0.18, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.93, 0.94, 0.92, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.18, 0.19, 0.20, 1.0))

    deck = model.part("deck")
    deck.visual(_mesh("deck_shell", _build_deck_mesh()), material=deck_black, name="deck_body")
    deck.visual(
        Box((0.620, 0.008, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0090)),
        material=datum_silver,
        name="centerline_datum",
    )
    for mark_name, x_pos in (("front_station_mark", TRUCK_STATION_X), ("rear_station_mark", -TRUCK_STATION_X)):
        deck.visual(
            Box((0.024, 0.070, 0.0008)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0087)),
            material=datum_silver,
            name=mark_name,
        )
        deck.visual(
            Box((0.050, 0.006, 0.0008)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0088)),
            material=datum_silver,
            name=f"{mark_name}_cross",
        )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.025)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    wheel_shell_mesh = _mesh("skate_wheel_shell", _build_wheel_shell())

    for truck_name, x_pos, facing_sign in (
        ("front", TRUCK_STATION_X, 1.0),
        ("rear", -TRUCK_STATION_X, -1.0),
    ):
        baseplate = model.part(f"{truck_name}_baseplate")
        baseplate.visual(
            _mesh(f"{truck_name}_truck_base", _build_truck_base_mesh(facing_sign)),
            material=truck_metal,
            name="baseplate_shell",
        )
        baseplate.visual(
            Box((0.028, 0.002, 0.0008)),
            origin=Origin(xyz=(-0.010 * facing_sign, 0.0, -0.0041)),
            material=datum_silver,
            name="kingpin_scale",
        )
        baseplate.visual(
            Box((0.002, 0.016, 0.0008)),
            origin=Origin(xyz=(-0.010 * facing_sign, 0.0, -0.0041)),
            material=datum_silver,
            name="kingpin_crosshair",
        )
        baseplate.inertial = Inertial.from_geometry(
            Box((0.088, 0.058, 0.028)),
            mass=0.45,
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
        )

        hanger = model.part(f"{truck_name}_hanger")
        hanger.visual(
            _mesh(f"{truck_name}_hanger_body", _build_hanger_mesh(facing_sign)),
            material=dark_steel,
            name="hanger_body",
        )

        axle_center_x = -0.020 * facing_sign
        axis = _truck_axis(facing_sign)
        axis_pitch = atan2(axis[0], axis[2])

        hanger.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=_scale(axis, -0.010), rpy=(0.0, axis_pitch, 0.0)),
            material=bushing_amber,
            name="lower_bushing",
        )
        hanger.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=_scale(axis, -0.018), rpy=(0.0, axis_pitch, 0.0)),
            material=bushing_amber,
            name="upper_bushing",
        )
        hanger.visual(
            Cylinder(radius=0.0095, length=0.003),
            origin=Origin(xyz=_scale(axis, -0.005), rpy=(0.0, axis_pitch, 0.0)),
            material=truck_metal,
            name="lower_washer",
        )
        hanger.visual(
            Cylinder(radius=0.0085, length=0.003),
            origin=Origin(xyz=_scale(axis, -0.023), rpy=(0.0, axis_pitch, 0.0)),
            material=truck_metal,
            name="upper_washer",
        )
        hanger.visual(
            Cylinder(radius=0.0075, length=0.006),
            origin=Origin(xyz=_scale(axis, -0.028), rpy=(0.0, axis_pitch, 0.0)),
            material=truck_metal,
            name="adjustment_nut",
        )
        hanger.visual(
            Cylinder(radius=0.0044, length=0.006),
            origin=Origin(xyz=_scale(axis, -0.0258), rpy=(0.0, axis_pitch, 0.0)),
            material=dark_steel,
            name="kingpin_stub",
        )
        hanger.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(axle_center_x, 0.084, AXLE_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=truck_metal,
            name="left_collar",
        )
        hanger.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(axle_center_x, -0.084, AXLE_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=truck_metal,
            name="right_collar",
        )
        hanger.visual(
            Box((0.020, 0.002, 0.0010)),
            origin=Origin(xyz=(-0.004 * facing_sign, 0.0, -0.0115)),
            material=datum_silver,
            name="center_index_mark",
        )
        hanger.inertial = Inertial.from_geometry(
            Box((0.090, 0.230, 0.050)),
            mass=0.55,
            origin=Origin(xyz=(axle_center_x, 0.0, -0.020)),
        )

        model.articulation(
            f"deck_to_{truck_name}_baseplate",
            ArticulationType.FIXED,
            parent=deck,
            child=baseplate,
            origin=Origin(xyz=(x_pos, 0.0, -DECK_THICKNESS * 0.5)),
        )
        model.articulation(
            f"{truck_name}_truck_steer",
            ArticulationType.REVOLUTE,
            parent=baseplate,
            child=hanger,
            origin=Origin(xyz=(0.018 * facing_sign, 0.0, -0.018)),
            axis=axis,
            motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=-0.24, upper=0.24),
        )

        for side_name, y_pos in (("left", AXLE_HALF_SPAN), ("right", -AXLE_HALF_SPAN)):
            wheel = model.part(f"{truck_name}_{side_name}_wheel")
            wheel.visual(wheel_shell_mesh, material=wheel_white, name="wheel_shell")
            wheel.visual(
                Cylinder(radius=0.012, length=0.012),
                origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
                material=wheel_core,
                name="hub_core",
            )
            wheel.visual(
                Box((0.0035, 0.0008, 0.012)),
                origin=Origin(xyz=(0.018, WHEEL_WIDTH * 0.5 - 0.0004, 0.0)),
                material=wheel_core,
                name="face_mark_pos",
            )
            wheel.visual(
                Box((0.0035, 0.0008, 0.012)),
                origin=Origin(xyz=(0.018, -WHEEL_WIDTH * 0.5 + 0.0004, 0.0)),
                material=wheel_core,
                name="face_mark_neg",
            )
            wheel.inertial = Inertial.from_geometry(
                Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
                mass=0.18,
                origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            )

            model.articulation(
                f"{truck_name}_{side_name}_spin",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(xyz=(axle_center_x, y_pos, AXLE_CENTER_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=4.0, velocity=40.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_baseplate = object_model.get_part("front_baseplate")
    rear_baseplate = object_model.get_part("rear_baseplate")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_steer = object_model.get_articulation("front_truck_steer")
    rear_steer = object_model.get_articulation("rear_truck_steer")

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

    ctx.expect_contact(deck, front_baseplate, name="front baseplate seated to deck")
    ctx.expect_contact(deck, rear_baseplate, name="rear baseplate seated to deck")
    ctx.expect_contact(front_baseplate, front_hanger, name="front hanger supported by baseplate")
    ctx.expect_contact(rear_baseplate, rear_hanger, name="rear hanger supported by baseplate")

    for wheel, hanger, name in (
        (front_left_wheel, front_hanger, "front left wheel carried by axle"),
        (front_right_wheel, front_hanger, "front right wheel carried by axle"),
        (rear_left_wheel, rear_hanger, "rear left wheel carried by axle"),
        (rear_right_wheel, rear_hanger, "rear right wheel carried by axle"),
    ):
        ctx.expect_contact(wheel, hanger, name=name)

    ctx.expect_gap(
        front_left_wheel,
        front_hanger,
        axis="y",
        positive_elem="wheel_shell",
        negative_elem="left_collar",
        min_gap=0.001,
        max_gap=0.006,
        name="front left service gap",
    )
    ctx.expect_gap(
        front_hanger,
        front_right_wheel,
        axis="y",
        positive_elem="right_collar",
        negative_elem="wheel_shell",
        min_gap=0.001,
        max_gap=0.006,
        name="front right service gap",
    )
    ctx.expect_gap(
        rear_left_wheel,
        rear_hanger,
        axis="y",
        positive_elem="wheel_shell",
        negative_elem="left_collar",
        min_gap=0.001,
        max_gap=0.006,
        name="rear left service gap",
    )
    ctx.expect_gap(
        rear_hanger,
        rear_right_wheel,
        axis="y",
        positive_elem="right_collar",
        negative_elem="wheel_shell",
        min_gap=0.001,
        max_gap=0.006,
        name="rear right service gap",
    )

    ctx.expect_gap(
        deck,
        front_hanger,
        axis="z",
        positive_elem="deck_body",
        negative_elem="adjustment_nut",
        min_gap=0.0005,
        max_gap=0.030,
        name="front adjustment nut service clearance below deck",
    )
    ctx.expect_gap(
        deck,
        rear_hanger,
        axis="z",
        positive_elem="deck_body",
        negative_elem="adjustment_nut",
        min_gap=0.0005,
        max_gap=0.030,
        name="rear adjustment nut service clearance below deck",
    )

    for joint_name in ("front_left_spin", "front_right_spin", "rear_left_spin", "rear_right_spin"):
        joint = object_model.get_articulation(joint_name)
        axis = joint.axis
        ctx.check(
            f"{joint_name} spins on axle axis",
            abs(axis[1]) > 0.99 and abs(axis[0]) < 1e-9 and abs(axis[2]) < 1e-9,
            details=f"axis={axis}",
        )

    for joint_name in ("front_truck_steer", "rear_truck_steer"):
        joint = object_model.get_articulation(joint_name)
        axis = joint.axis
        ctx.check(
            f"{joint_name} follows kingpin line",
            abs(axis[0]) > 0.75 and abs(axis[2]) > 0.45 and abs(axis[1]) < 1e-9,
            details=f"axis={axis}",
        )

    with ctx.pose({front_steer: 0.0}):
        front_left_closed = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_steer: 0.18}):
        front_left_open = ctx.part_world_position(front_left_wheel)
    with ctx.pose({rear_steer: 0.0}):
        rear_left_closed = ctx.part_world_position(rear_left_wheel)
    with ctx.pose({rear_steer: 0.18}):
        rear_left_open = ctx.part_world_position(rear_left_wheel)

    front_delta = (
        abs(front_left_open[0] - front_left_closed[0]) + abs(front_left_open[2] - front_left_closed[2])
        if front_left_open is not None and front_left_closed is not None
        else 0.0
    )
    rear_delta = (
        abs(rear_left_open[0] - rear_left_closed[0]) + abs(rear_left_open[2] - rear_left_closed[2])
        if rear_left_open is not None and rear_left_closed is not None
        else 0.0
    )
    ctx.check(
        "front truck steer repositions axle center",
        front_delta > 0.003,
        details=f"front axle center delta={front_delta:.6f}",
    )
    ctx.check(
        "rear truck steer repositions axle center",
        rear_delta > 0.003,
        details=f"rear axle center delta={rear_delta:.6f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
