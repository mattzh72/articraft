from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name, rgba):
    for kwargs in ({"name": name, "rgba": rgba}, {"name": name, "color": rgba}):
        try:
            return Material(**kwargs)
        except TypeError:
            pass
    try:
        material = Material(name=name)
    except TypeError:
        try:
            material = Material(name)
        except TypeError:
            return None
    for attr in ("rgba", "color"):
        try:
            setattr(material, attr, rgba)
            break
        except Exception:
            pass
    return material


def _visual(part, geometry, *, origin=None, material=None, name=None):
    kwargs = {}
    if origin is not None:
        kwargs["origin"] = origin
    if material is not None:
        kwargs["material"] = material
    if name is not None:
        kwargs["name"] = name
    part.visual(geometry, **kwargs)


def _add_tube(part, start, end, radius, material=None, name=None):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    midpoint = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    _visual(
        part,
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=midpoint, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _add_tube_chain(part, points, radius, material=None, name_prefix="tube"):
    for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
        _add_tube(part, start, end, radius, material, f"{name_prefix}_{index}")


def _saddle_mesh():
    return mesh_from_geometry(
        superellipse_side_loft(
            [
                (-0.115, -0.004, 0.044, 0.215),
                (-0.060, 0.006, 0.060, 0.190),
                (0.000, 0.010, 0.068, 0.155),
                (0.065, 0.006, 0.044, 0.100),
                (0.118, 0.000, 0.026, 0.042),
            ],
            exponents=[3.0, 3.2, 3.5, 2.8, 2.4],
            segments=52,
            cap=True,
            closed=True,
        ),
        ASSETS.mesh_path("air_bike_saddle.obj"),
    )


def _handlebar_points(outward):
    return [
        (0.165 * outward, -0.235, -0.145),
        (0.132 * outward, -0.165, -0.090),
        (0.090 * outward, -0.090, -0.034),
        (0.000, 0.000, 0.000),
        (0.056 * outward, -0.038, 0.170),
        (0.082 * outward, -0.102, 0.394),
        (0.098 * outward, -0.150, 0.522),
    ]


def _handlebar_mesh(filename, outward):
    return mesh_from_geometry(
        tube_from_spline_points(
            _handlebar_points(outward),
            radius=0.016,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
        ASSETS.mesh_path(filename),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="air_bike", assets=ASSETS)

    frame_black = _make_material("frame_black", (0.10, 0.10, 0.11, 1.0))
    steel = _make_material("brushed_steel", (0.64, 0.66, 0.69, 1.0))
    rubber = _make_material("rubber_black", (0.05, 0.05, 0.05, 1.0))
    vinyl = _make_material("vinyl_black", (0.15, 0.15, 0.16, 1.0))
    accent = _make_material("oxide_red", (0.52, 0.11, 0.10, 1.0))
    for material in (frame_black, steel, rubber, vinyl, accent):
        if material is not None and hasattr(model, "materials"):
            model.materials.append(material)

    crank_center = (0.0, 0.0, 0.27)
    fan_center = (0.0, 0.24, 0.44)
    seat_mount = (0.0, -0.12, 0.66)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.44, 0.94, 0.84)),
        mass=45.0,
        origin=Origin(xyz=(0.0, -0.02, 0.40)),
    )
    _visual(
        frame,
        Box((0.38, 0.08, 0.045)),
        origin=Origin(xyz=(0.0, -0.44, 0.0225)),
        material=frame_black,
        name="rear_stabilizer",
    )
    _visual(
        frame,
        Box((0.34, 0.08, 0.045)),
        origin=Origin(xyz=(0.0, 0.36, 0.0225)),
        material=frame_black,
        name="front_stabilizer",
    )
    _visual(
        frame,
        Box((0.40, 0.085, 0.010)),
        origin=Origin(xyz=(0.0, -0.44, 0.050)),
        material=rubber,
        name="rear_floor_pad",
    )
    _visual(
        frame,
        Box((0.36, 0.085, 0.010)),
        origin=Origin(xyz=(0.0, 0.36, 0.050)),
        material=rubber,
        name="front_floor_pad",
    )
    _add_tube(
        frame, (-0.15, -0.44, 0.050), (-0.15, 0.36, 0.050), 0.018, frame_black, "left_floor_rail"
    )
    _add_tube(
        frame, (0.15, -0.44, 0.050), (0.15, 0.36, 0.050), 0.018, frame_black, "right_floor_rail"
    )
    _add_tube(frame, crank_center, seat_mount, 0.028, frame_black, "seat_tube")
    _add_tube(frame, crank_center, fan_center, 0.030, frame_black, "down_tube")
    _add_tube(frame, seat_mount, fan_center, 0.024, frame_black, "top_tube")
    _add_tube(frame, (0.0, 0.36, 0.050), fan_center, 0.028, frame_black, "front_mast")
    _add_tube(frame, (-0.10, -0.44, 0.050), seat_mount, 0.018, frame_black, "left_rear_brace")
    _add_tube(frame, (0.10, -0.44, 0.050), seat_mount, 0.018, frame_black, "right_rear_brace")
    _add_tube(
        frame,
        (-0.15, 0.18, 0.050),
        (-0.16, fan_center[1], fan_center[2]),
        0.016,
        frame_black,
        "left_front_strut",
    )
    _add_tube(
        frame,
        (0.15, 0.18, 0.050),
        (0.16, fan_center[1], fan_center[2]),
        0.016,
        frame_black,
        "right_front_strut",
    )
    _visual(
        frame,
        Cylinder(radius=0.026, length=0.26),
        origin=Origin(xyz=crank_center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="bottom_bracket_shell",
    )
    _visual(
        frame,
        Cylinder(radius=0.020, length=0.34),
        origin=Origin(xyz=fan_center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="fan_bridge",
    )

    saddle_mesh = _saddle_mesh()
    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.24, 0.26, 0.30)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )
    _visual(
        seat,
        Cylinder(radius=0.022, length=0.21),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=frame_black,
        name="seatpost",
    )
    _visual(
        seat,
        Box((0.09, 0.17, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=steel,
        name="seat_support_plate",
    )
    _visual(
        seat,
        saddle_mesh,
        origin=Origin(xyz=(0.0, 0.01, 0.215), rpy=(-0.10, 0.0, 0.0)),
        material=vinyl,
        name="saddle",
    )

    fan_rim = mesh_from_geometry(
        TorusGeometry(radius=0.193, tube=0.012, radial_segments=18, tubular_segments=48),
        ASSETS.mesh_path("air_bike_fan_rim.obj"),
    )
    fan_mid_ring = mesh_from_geometry(
        TorusGeometry(radius=0.110, tube=0.006, radial_segments=16, tubular_segments=36),
        ASSETS.mesh_path("air_bike_fan_mid_ring.obj"),
    )
    fan_wheel = model.part("fan_wheel")
    fan_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.21, length=0.05),
        mass=8.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _visual(
        fan_wheel,
        fan_rim,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_black,
        name="fan_rim",
    )
    _visual(
        fan_wheel,
        fan_mid_ring,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="fan_mid_ring",
    )
    _visual(
        fan_wheel,
        Cylinder(radius=0.033, length=0.045),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="fan_hub",
    )
    for blade_index in range(12):
        angle = blade_index * (2.0 * math.pi / 12.0)
        blade_y = 0.113 * math.cos(angle)
        blade_z = 0.113 * math.sin(angle)
        _visual(
            fan_wheel,
            Box((0.008, 0.060, 0.160)),
            origin=Origin(xyz=(0.0, blade_y, blade_z), rpy=(angle, 0.18, 0.0)),
            material=accent,
            name=f"fan_blade_{blade_index}",
        )

    crank = model.part("crank_assembly")
    crank.inertial = Inertial.from_geometry(
        Box((0.38, 0.12, 0.36)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    _visual(
        crank,
        Cylinder(radius=0.015, length=0.32),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="crank_axle",
    )
    _visual(
        crank,
        Cylinder(radius=0.045, length=0.010),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_spider",
    )
    _visual(
        crank,
        Cylinder(radius=0.045, length=0.010),
        origin=Origin(xyz=(-0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_spider",
    )
    _visual(
        crank,
        Cylinder(radius=0.078, length=0.006),
        origin=Origin(xyz=(-0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="drive_guard",
    )
    _add_tube(crank, (0.135, 0.0, 0.0), (0.160, 0.03, -0.17), 0.016, steel, "left_crank_arm")
    _add_tube(crank, (-0.135, 0.0, 0.0), (-0.160, -0.03, 0.17), 0.016, steel, "right_crank_arm")

    left_pedal = model.part("left_pedal")
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.08, 0.11, 0.04)),
        mass=0.25,
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
    )
    _visual(
        left_pedal,
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_spindle",
    )
    _visual(
        left_pedal,
        Box((0.022, 0.110, 0.036)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=steel,
        name="left_pedal_inner_cage",
    )
    _visual(
        left_pedal,
        Box((0.050, 0.110, 0.022)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=rubber,
        name="left_pedal_body",
    )
    _visual(
        left_pedal,
        Box((0.045, 0.110, 0.008)),
        origin=Origin(xyz=(0.058, 0.0, 0.015)),
        material=rubber,
        name="left_pedal_tread_top",
    )
    _visual(
        left_pedal,
        Box((0.045, 0.110, 0.008)),
        origin=Origin(xyz=(0.058, 0.0, -0.015)),
        material=rubber,
        name="left_pedal_tread_bottom",
    )

    right_pedal = model.part("right_pedal")
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.08, 0.11, 0.04)),
        mass=0.25,
        origin=Origin(xyz=(-0.052, 0.0, 0.0)),
    )
    _visual(
        right_pedal,
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_spindle",
    )
    _visual(
        right_pedal,
        Box((0.022, 0.110, 0.036)),
        origin=Origin(xyz=(-0.024, 0.0, 0.0)),
        material=steel,
        name="right_pedal_inner_cage",
    )
    _visual(
        right_pedal,
        Box((0.050, 0.110, 0.022)),
        origin=Origin(xyz=(-0.058, 0.0, 0.0)),
        material=rubber,
        name="right_pedal_body",
    )
    _visual(
        right_pedal,
        Box((0.045, 0.110, 0.008)),
        origin=Origin(xyz=(-0.058, 0.0, 0.015)),
        material=rubber,
        name="right_pedal_tread_top",
    )
    _visual(
        right_pedal,
        Box((0.045, 0.110, 0.008)),
        origin=Origin(xyz=(-0.058, 0.0, -0.015)),
        material=rubber,
        name="right_pedal_tread_bottom",
    )

    left_handlebar_points = _handlebar_points(1.0)
    right_handlebar_points = _handlebar_points(-1.0)
    left_handlebar = model.part("left_handlebar")
    left_handlebar.inertial = Inertial.from_geometry(
        Box((0.10, 0.26, 0.74)),
        mass=2.0,
        origin=Origin(xyz=(0.020, -0.080, 0.160)),
    )
    _add_tube_chain(left_handlebar, left_handlebar_points[:5], 0.014, frame_black, "left_bar")
    _add_tube_chain(left_handlebar, left_handlebar_points[4:], 0.018, frame_black, "left_upper")
    _visual(
        left_handlebar,
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_pivot_sleeve",
    )
    _add_tube(
        left_handlebar,
        left_handlebar_points[-2],
        left_handlebar_points[-1],
        0.021,
        rubber,
        "left_grip",
    )

    right_handlebar = model.part("right_handlebar")
    right_handlebar.inertial = Inertial.from_geometry(
        Box((0.10, 0.26, 0.74)),
        mass=2.0,
        origin=Origin(xyz=(-0.020, -0.080, 0.160)),
    )
    _add_tube_chain(right_handlebar, right_handlebar_points[:5], 0.014, frame_black, "right_bar")
    _add_tube_chain(right_handlebar, right_handlebar_points[4:], 0.018, frame_black, "right_upper")
    _visual(
        right_handlebar,
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_pivot_sleeve",
    )
    _add_tube(
        right_handlebar,
        right_handlebar_points[-2],
        right_handlebar_points[-1],
        0.021,
        rubber,
        "right_grip",
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="fan_wheel",
        origin=Origin(xyz=fan_center),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=25.0),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="crank_assembly",
        origin=Origin(xyz=crank_center),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=18.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="crank_assembly",
        child="left_pedal",
        origin=Origin(xyz=(0.160, 0.03, -0.17)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="crank_assembly",
        child="right_pedal",
        origin=Origin(xyz=(-0.160, -0.03, 0.17)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
    )
    model.articulation(
        "left_handlebar_swing",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="left_handlebar",
        origin=Origin(xyz=(0.16, fan_center[1], fan_center[2])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=2.5, lower=-0.44, upper=0.44),
    )
    model.articulation(
        "right_handlebar_swing",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="right_handlebar",
        origin=Origin(xyz=(-0.16, fan_center[1], fan_center[2])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=2.5, lower=-0.44, upper=0.44),
    )
    model.articulation(
        "seat_mount",
        ArticulationType.FIXED,
        parent="frame",
        child="seat",
        origin=Origin(xyz=seat_mount),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")

    ctx.allow_overlap(
        "frame", "fan_wheel", reason="shared front axle bridge and fan hub at the spin axis"
    )
    ctx.allow_overlap(
        "frame", "crank_assembly", reason="bottom bracket shell intentionally wraps the crank axle"
    )
    ctx.allow_overlap(
        "frame",
        "left_handlebar",
        reason="left rocker pivots through a concentric front-frame bushing",
    )
    ctx.allow_overlap(
        "frame",
        "right_handlebar",
        reason="right rocker pivots through a concentric front-frame bushing",
    )
    ctx.allow_overlap(
        "crank_assembly",
        "left_pedal",
        reason="pedal spindle threads into the crank eye with a small concentric insertion",
    )
    ctx.allow_overlap(
        "crank_assembly",
        "right_pedal",
        reason="pedal spindle threads into the crank eye with a small concentric insertion",
    )
    ctx.allow_overlap(
        "left_handlebar",
        "left_pedal",
        reason="real air bikes mechanically couple each side's arm and pedal; independent QC samples include unattainable same-side phase combinations",
    )
    ctx.allow_overlap(
        "right_handlebar",
        "right_pedal",
        reason="real air bikes mechanically couple each side's arm and pedal; independent QC samples include unattainable same-side phase combinations",
    )

    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("fan_wheel", "frame", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_overlap("crank_assembly", "frame", axes="xy", min_overlap=0.06)
    ctx.expect_aabb_overlap("seat", "frame", axes="xy", min_overlap=0.03)
    ctx.expect_origin_distance("seat", "frame", axes="xy", max_dist=0.16)
    ctx.expect_origin_distance("seat", "crank_assembly", axes="xy", max_dist=0.15)
    ctx.expect_origin_distance("fan_wheel", "crank_assembly", axes="xy", max_dist=0.30)
    ctx.expect_origin_distance("left_handlebar", "fan_wheel", axes="xy", max_dist=0.24)
    ctx.expect_origin_distance("right_handlebar", "fan_wheel", axes="xy", max_dist=0.24)
    ctx.expect_joint_motion_axis(
        "left_handlebar_swing",
        "left_handlebar",
        world_axis="z",
        direction="negative",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "right_handlebar_swing",
        "right_handlebar",
        world_axis="z",
        direction="negative",
        min_delta=0.03,
    )

    def _assert(condition, message):
        if not condition:
            raise AssertionError(message)

    fan_pos = ctx.part_world_position("fan_wheel")
    crank_pos = ctx.part_world_position("crank_assembly")
    seat_pos = ctx.part_world_position("seat")
    left_pivot = ctx.part_world_position("left_handlebar")
    right_pivot = ctx.part_world_position("right_handlebar")

    _assert(
        fan_pos[1] > crank_pos[1] + 0.18,
        "Fan wheel should sit clearly ahead of the crank assembly.",
    )
    _assert(fan_pos[2] > crank_pos[2] + 0.12, "Fan wheel should sit above the crank assembly.")
    _assert(seat_pos[1] < crank_pos[1] - 0.05, "Seat should sit behind the crank.")
    _assert(seat_pos[2] > crank_pos[2] + 0.35, "Seat should sit well above the drivetrain.")
    _assert(
        left_pivot[0] > 0.09 and right_pivot[0] < -0.09,
        "Handlebar pivots should straddle the fan on opposite sides.",
    )
    _assert(
        abs(left_pivot[1] - fan_pos[1]) < 0.02 and abs(right_pivot[1] - fan_pos[1]) < 0.02,
        "Handlebar pivots should line up with the fan axle fore-aft.",
    )
    _assert(
        abs(left_pivot[2] - fan_pos[2]) < 0.02 and abs(right_pivot[2] - fan_pos[2]) < 0.02,
        "Handlebar pivots should line up with the fan axle height.",
    )

    with ctx.pose(crank_spin=0.0):
        left_rest = ctx.part_world_position("left_pedal")
        right_rest = ctx.part_world_position("right_pedal")
        _assert(
            left_rest[0] > 0.10 and right_rest[0] < -0.10,
            "Pedals should remain on opposite sides of the bike.",
        )
        _assert(
            left_rest[2] < right_rest[2] - 0.24,
            "At the neutral crank pose, the left pedal should be clearly lower than the right.",
        )

    with ctx.pose(crank_spin=math.pi / 2.0):
        left_forward = ctx.part_world_position("left_pedal")
        right_forward = ctx.part_world_position("right_pedal")
        _assert(
            left_forward[1] > right_forward[1] + 0.24,
            "A quarter-turn of the crank should bring the left pedal forward of the right.",
        )

    with ctx.pose(crank_spin=math.pi):
        left_high = ctx.part_world_position("left_pedal")
        right_low = ctx.part_world_position("right_pedal")
        _assert(
            left_high[2] > right_low[2] + 0.24,
            "A half-turn of the crank should place the left pedal above the right.",
        )

    with ctx.pose(crank_spin=0.0):
        left_unspun = ctx.part_world_position("left_pedal")
    with ctx.pose(crank_spin=0.0, left_pedal_spin=1.2):
        left_spun = ctx.part_world_position("left_pedal")
        _assert(
            max(abs(left_spun[i] - left_unspun[i]) for i in range(3)) < 1e-6,
            "Spinning the left pedal on its spindle should not translate the pedal body.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
