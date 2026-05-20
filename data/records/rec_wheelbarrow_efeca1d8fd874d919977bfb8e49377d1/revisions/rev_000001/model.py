from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rounded_rect_loop(
    length: float,
    width: float,
    radius: float,
    *,
    z: float,
    x: float = 0.0,
    y: float = 0.0,
    segments: int = 6,
) -> list[tuple[float, float, float]]:
    """Return a horizontal rounded-rectangle point loop in consistent order."""

    hx = length / 2.0
    hy = width / 2.0
    r = min(radius, hx * 0.98, hy * 0.98)
    centers = (
        (x + hx - r, y + hy - r, 0.0, pi / 2.0),
        (x - hx + r, y + hy - r, pi / 2.0, pi),
        (x - hx + r, y - hy + r, pi, 3.0 * pi / 2.0),
        (x + hx - r, y - hy + r, 3.0 * pi / 2.0, 2.0 * pi),
    )
    pts: list[tuple[float, float, float]] = []
    for cx, cy, a0, a1 in centers:
        for i in range(segments):
            a = a0 + (a1 - a0) * (i / segments)
            pts.append((cx + r * cos(a), cy + r * sin(a), z))
    return pts


def _add_loop(geom: MeshGeometry, loop: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in loop]


def _connect_loops(geom: MeshGeometry, a: list[int], b: list[int]) -> None:
    n = len(a)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(a[i], a[j], b[j])
        geom.add_face(a[i], b[j], b[i])


def _cap_loop(geom: MeshGeometry, loop: list[int], center: tuple[float, float, float]) -> None:
    c = geom.add_vertex(*center)
    n = len(loop)
    for i in range(n):
        geom.add_face(c, loop[i], loop[(i + 1) % n])


def _make_hollow_tray() -> MeshGeometry:
    """Heavy open wheelbarrow tray: flared walls, real floor thickness, and rolled lip."""

    geom = MeshGeometry()
    # Inner basin is smaller at the bottom, larger at the top; the outer skin
    # is offset down/out so the open tray reads as a hollow pressed-steel part.
    inner_top = _add_loop(
        geom,
        _rounded_rect_loop(1.12, 0.70, 0.105, z=0.705, x=0.02, segments=8),
    )
    inner_bottom = _add_loop(
        geom,
        _rounded_rect_loop(0.72, 0.36, 0.070, z=0.455, x=0.04, segments=8),
    )
    outer_top = _add_loop(
        geom,
        _rounded_rect_loop(1.20, 0.78, 0.125, z=0.690, x=0.02, segments=8),
    )
    outer_bottom = _add_loop(
        geom,
        _rounded_rect_loop(0.80, 0.44, 0.085, z=0.415, x=0.04, segments=8),
    )

    _connect_loops(geom, inner_top, inner_bottom)   # painted inside faces
    _connect_loops(geom, outer_top, inner_top)      # thick rolled rim land
    _connect_loops(geom, outer_bottom, outer_top)   # exterior flared shell
    _connect_loops(geom, inner_bottom, outer_bottom)  # visible floor thickness
    _cap_loop(geom, inner_bottom, (0.04, 0.0, 0.455))
    _cap_loop(geom, outer_bottom, (0.04, 0.0, 0.415))
    return geom


def _make_frame_tubes() -> MeshGeometry:
    """One welded utility tube running through both handles and around the fork crown."""

    frame = tube_from_spline_points(
        [
            (-1.08, -0.31, 0.610),
            (-0.78, -0.305, 0.555),
            (-0.30, -0.285, 0.405),
            (0.30, -0.255, 0.405),
            (0.50, -0.205, 0.455),
            (0.60, -0.110, 0.455),
            (0.60, 0.110, 0.455),
            (0.50, 0.205, 0.455),
            (0.30, 0.255, 0.405),
            (-0.30, 0.285, 0.405),
            (-0.78, 0.305, 0.555),
            (-1.08, 0.31, 0.610),
        ],
        radius=0.024,
        samples_per_segment=7,
        radial_segments=18,
        cap_ends=True,
    )

    # Cross ties are intentionally welded into the primary tube network.
    for name, pts, rad in (
        (
            "rear_handle_tie",
            [(-0.82, -0.315, 0.555), (-0.70, 0.0, 0.545), (-0.82, 0.315, 0.555)],
            0.020,
        ),
        (
            "under_tray_cross_tie",
            [(-0.18, -0.290, 0.405), (-0.06, 0.0, 0.393), (-0.18, 0.290, 0.405)],
            0.019,
        ),
        (
            "front_fork_crown",
            [(0.48, -0.250, 0.430), (0.52, 0.0, 0.425), (0.48, 0.250, 0.430)],
            0.021,
        ),
        (
            "left_fork_drop",
            [(0.48, -0.185, 0.425), (0.57, -0.115, 0.325), (0.642, -0.105, 0.230)],
            0.018,
        ),
        (
            "right_fork_drop",
            [(0.48, 0.185, 0.425), (0.57, 0.115, 0.325), (0.642, 0.105, 0.230)],
            0.018,
        ),
    ):
        del name
        frame.merge(
            tube_from_spline_points(
                pts,
                radius=rad,
                samples_per_segment=6,
                radial_segments=16,
                cap_ends=True,
            )
        )
    return frame


def _make_stand_tube() -> MeshGeometry:
    return tube_from_spline_points(
        [
            (0.0, -0.270, -0.044),
            (-0.075, -0.292, -0.165),
            (-0.195, -0.315, -0.370),
            (-0.205, 0.0, -0.388),
            (-0.195, 0.315, -0.370),
            (-0.075, 0.292, -0.165),
            (0.0, 0.270, -0.044),
        ],
        radius=0.020,
        samples_per_segment=8,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_wheelbarrow")

    painted_green = model.material("dark_green_painted_steel", rgba=(0.05, 0.23, 0.12, 1.0))
    frame_black = model.material("black_powdercoated_tube", rgba=(0.015, 0.017, 0.016, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    yellow_rim = model.material("yellow_painted_rim", rgba=(0.92, 0.62, 0.10, 1.0))
    galvanized = model.material("galvanized_fasteners", rgba=(0.62, 0.64, 0.60, 1.0))
    scuffed = model.material("scuffed_wear_edges", rgba=(0.16, 0.18, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_make_hollow_tray(), "open_pressed_tray"),
        material=painted_green,
        name="open_pressed_tray",
    )
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _rounded_rect_loop(1.23, 0.80, 0.13, z=0.698, x=0.02, segments=7),
                radius=0.022,
                samples_per_segment=4,
                closed_spline=True,
                radial_segments=16,
            ),
            "rolled_tray_lip",
        ),
        material=scuffed,
        name="rolled_tray_lip",
    )
    body.visual(
        mesh_from_geometry(_make_frame_tubes(), "welded_tubular_frame"),
        material=frame_black,
        name="welded_tubular_frame",
    )

    # Under-basin stiffeners and serviceable bolt-on saddle blocks.
    for i, x in enumerate((-0.24, 0.08, 0.38)):
        body.visual(
            Box((0.055, 0.54, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.398)),
            material=frame_black,
            name=f"under_tray_channel_{i}",
        )
    for i, x in enumerate((-0.36, 0.24)):
        body.visual(
            Box((0.18, 0.090, 0.050)),
            origin=Origin(xyz=(x, -0.300, 0.430)),
            material=frame_black,
            name=f"side_saddle_{i}_0",
        )
        body.visual(
            Box((0.18, 0.090, 0.050)),
            origin=Origin(xyz=(x, 0.300, 0.430)),
            material=frame_black,
            name=f"side_saddle_{i}_1",
        )

    # Rear folding-stand hinge pin and fork axle hardware are deliberately exposed.
    body.visual(
        Cylinder(radius=0.016, length=0.660),
        origin=Origin(xyz=(-0.470, 0.0, 0.370), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="stand_hinge_pin",
    )
    body.visual(
        Box((0.075, 0.050, 0.085)),
        origin=Origin(xyz=(-0.470, -0.340, 0.370)),
        material=frame_black,
        name="stand_clevis_0",
    )
    body.visual(
        Box((0.075, 0.050, 0.085)),
        origin=Origin(xyz=(-0.470, 0.340, 0.370)),
        material=frame_black,
        name="stand_clevis_1",
    )

    body.visual(
        Cylinder(radius=0.053, length=0.014),
        origin=Origin(xyz=(0.642, -0.084, 0.230), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="axle_washer_0",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.024),
        origin=Origin(xyz=(0.642, -0.111, 0.230), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="axle_nut_0",
    )
    body.visual(
        Cylinder(radius=0.053, length=0.014),
        origin=Origin(xyz=(0.642, 0.084, 0.230), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="axle_washer_1",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.024),
        origin=Origin(xyz=(0.642, 0.111, 0.230), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="axle_nut_1",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.250),
        origin=Origin(xyz=(0.642, 0.0, 0.230), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="axle_shaft",
    )

    # Rubberized grips over the ends of the tubular handles.
    body.visual(
        Cylinder(radius=0.034, length=0.230),
        origin=Origin(xyz=(-1.105, -0.310, 0.615), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="grip_0",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.230),
        origin=Origin(xyz=(-1.105, 0.310, 0.615), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="grip_1",
    )

    # Visible rivets/fasteners along the tray flanges and saddle blocks.
    bolt_positions = [
        (-0.42, -0.388, 0.665),
        (-0.10, -0.392, 0.680),
        (0.25, -0.388, 0.672),
        (0.50, -0.360, 0.625),
        (-0.42, 0.388, 0.665),
        (-0.10, 0.392, 0.680),
        (0.25, 0.388, 0.672),
        (0.50, 0.360, 0.625),
        (-0.36, -0.352, 0.435),
        (0.24, -0.352, 0.435),
        (-0.36, 0.352, 0.435),
        (0.24, 0.352, 0.435),
    ]
    for i, (x, y, z) in enumerate(bolt_positions):
        body.visual(
            Cylinder(radius=0.014, length=0.024),
            origin=Origin(xyz=(x, y, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"flange_bolt_{i}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.180,
                0.095,
                inner_radius=0.125,
                tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.54),
                grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
                sidewall=TireSidewall(style="square", bulge=0.025),
                shoulder=TireShoulder(width=0.010, radius=0.004),
            ),
            "knobby_utility_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.126,
                0.076,
                rim=WheelRim(inner_radius=0.080, flange_height=0.012, flange_thickness=0.006),
                hub=WheelHub(
                    radius=0.036,
                    width=0.086,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=4, circle_diameter=0.045, hole_diameter=0.006),
                ),
                face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
                spokes=WheelSpokes(style="split_y", count=4, thickness=0.006, window_radius=0.016),
                bore=WheelBore(style="round", diameter=0.020),
            ),
            "painted_spoked_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=yellow_rim,
        name="rim",
    )

    rear_stand = model.part("rear_stand")
    rear_stand.visual(
        mesh_from_geometry(_make_stand_tube(), "folding_stand_tube"),
        material=frame_black,
        name="stand_tube",
    )
    rear_stand.visual(
        Cylinder(radius=0.026, length=0.575),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="stand_hinge_sleeve",
    )
    rear_stand.visual(
        Box((0.070, 0.075, 0.044)),
        origin=Origin(xyz=(0.0, -0.270, -0.046)),
        material=frame_black,
        name="hinge_weld_0",
    )
    rear_stand.visual(
        Box((0.070, 0.075, 0.044)),
        origin=Origin(xyz=(0.0, 0.270, -0.046)),
        material=frame_black,
        name="hinge_weld_1",
    )
    rear_stand.visual(
        Box((0.135, 0.090, 0.024)),
        origin=Origin(xyz=(-0.200, -0.315, -0.390)),
        material=rubber,
        name="foot_pad_0",
    )
    rear_stand.visual(
        Box((0.135, 0.090, 0.024)),
        origin=Origin(xyz=(-0.200, 0.315, -0.390)),
        material=rubber,
        name="foot_pad_1",
    )

    model.articulation(
        "body_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(0.642, 0.0, 0.230)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )
    model.articulation(
        "body_to_rear_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_stand,
        origin=Origin(xyz=(-0.470, 0.0, 0.370)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("wheel")
    rear_stand = object_model.get_part("rear_stand")
    wheel_joint = object_model.get_articulation("body_to_wheel")
    stand_joint = object_model.get_articulation("body_to_rear_stand")

    ctx.allow_overlap(
        body,
        rear_stand,
        elem_a="stand_hinge_pin",
        elem_b="stand_hinge_sleeve",
        reason="The galvanized hinge pin is intentionally captured inside the folding stand sleeve.",
    )
    ctx.expect_within(
        body,
        rear_stand,
        axes="xz",
        inner_elem="stand_hinge_pin",
        outer_elem="stand_hinge_sleeve",
        margin=0.002,
        name="stand pin is centered in hinge sleeve",
    )
    ctx.expect_overlap(
        body,
        rear_stand,
        axes="y",
        elem_a="stand_hinge_pin",
        elem_b="stand_hinge_sleeve",
        min_overlap=0.520,
        name="stand hinge pin spans sleeve",
    )

    ctx.allow_overlap(
        body,
        wheel,
        elem_a="axle_shaft",
        elem_b="rim",
        reason="The visible axle shaft is intentionally captured through the wheel hub bore.",
    )
    ctx.expect_within(
        body,
        wheel,
        axes="xz",
        inner_elem="axle_shaft",
        outer_elem="rim",
        margin=0.004,
        name="axle shaft is centered in wheel hub",
    )
    ctx.expect_overlap(
        body,
        wheel,
        axes="y",
        elem_a="axle_shaft",
        elem_b="rim",
        min_overlap=0.060,
        name="axle shaft spans wheel hub",
    )

    ctx.expect_gap(
        wheel,
        body,
        axis="y",
        positive_elem="tire",
        negative_elem="axle_washer_0",
        min_gap=0.006,
        name="left washer clears tire sidewall",
    )
    ctx.expect_gap(
        body,
        wheel,
        axis="y",
        positive_elem="axle_washer_1",
        negative_elem="tire",
        min_gap=0.006,
        name="right washer clears tire sidewall",
    )

    rest_stand_aabb = ctx.part_world_aabb(rear_stand)
    with ctx.pose({stand_joint: 1.25}):
        folded_stand_aabb = ctx.part_world_aabb(rear_stand)
    ctx.check(
        "rear stand folds upward",
        rest_stand_aabb is not None
        and folded_stand_aabb is not None
        and folded_stand_aabb[0][2] > rest_stand_aabb[0][2] + 0.22,
        details=f"rest={rest_stand_aabb}, folded={folded_stand_aabb}",
    )

    wheel_position = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: pi}):
        rolled_position = ctx.part_world_position(wheel)
    ctx.check(
        "wheel rolls about fixed front axle",
        wheel_position is not None
        and rolled_position is not None
        and abs(wheel_position[0] - rolled_position[0]) < 1e-6
        and abs(wheel_position[1] - rolled_position[1]) < 1e-6
        and abs(wheel_position[2] - rolled_position[2]) < 1e-6,
        details=f"rest={wheel_position}, rolled={rolled_position}",
    )

    return ctx.report()


object_model = build_object_model()
