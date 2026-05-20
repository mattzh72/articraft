from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
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
    rounded_rect_profile,
    tube_from_spline_points,
    wire_from_points,
)


def _loop_vertices(
    geom: MeshGeometry, loop: list[tuple[float, float, float]]
) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in loop]


def _ring_faces(geom: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    count = len(a)
    for index in range(count):
        j = (index + 1) % count
        if flip:
            geom.add_face(a[index], b[j], a[j])
            geom.add_face(a[index], b[index], b[j])
        else:
            geom.add_face(a[index], a[j], b[j])
            geom.add_face(a[index], b[j], b[index])


def _fan_fill(geom: MeshGeometry, loop: list[int], center: tuple[float, float, float], *, flip: bool) -> None:
    center_index = geom.add_vertex(*center)
    count = len(loop)
    for index in range(count):
        j = (index + 1) % count
        if flip:
            geom.add_face(center_index, loop[j], loop[index])
        else:
            geom.add_face(center_index, loop[index], loop[j])


def _tray_shell_geometry() -> MeshGeometry:
    """Thin open wheelbarrow tub with a sloped, hollow interior."""

    def section(width_x: float, width_y: float, z: float, x_center: float, radius: float) -> list[tuple[float, float, float]]:
        return [
            (x + x_center, y, z)
            for x, y in rounded_rect_profile(width_x, width_y, radius, corner_segments=8)
        ]

    outer_top = section(1.08, 0.62, 0.70, 0.05, 0.08)
    inner_top = section(1.00, 0.54, 0.675, 0.05, 0.065)
    inner_bottom = section(0.58, 0.24, 0.405, 0.02, 0.040)
    outer_bottom = section(0.68, 0.34, 0.360, 0.02, 0.050)

    geom = MeshGeometry()
    outer_top_i = _loop_vertices(geom, outer_top)
    inner_top_i = _loop_vertices(geom, inner_top)
    inner_bottom_i = _loop_vertices(geom, inner_bottom)
    outer_bottom_i = _loop_vertices(geom, outer_bottom)

    _ring_faces(geom, outer_bottom_i, outer_top_i)
    _ring_faces(geom, inner_top_i, inner_bottom_i, flip=True)
    _ring_faces(geom, outer_top_i, inner_top_i)
    _ring_faces(geom, inner_bottom_i, outer_bottom_i)
    _fan_fill(geom, inner_bottom_i, (0.02, 0.0, 0.405), flip=False)
    _fan_fill(geom, outer_bottom_i, (0.02, 0.0, 0.360), flip=True)
    return geom


def _tube(
    points: list[tuple[float, float, float]],
    *,
    radius: float = 0.018,
    smooth: bool = False,
    corner_radius: float = 0.025,
) -> MeshGeometry:
    if smooth:
        return tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        )
    return wire_from_points(
        points,
        radius=radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=corner_radius,
        corner_segments=10,
    )


def _merge(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _steel_frame_geometry() -> MeshGeometry:
    tubes: list[MeshGeometry] = []

    for side in (-1.0, 1.0):
        y = side * 0.32
        tubes.append(
            _tube(
                [
                    (-1.26, y, 0.60),
                    (-0.82, y, 0.54),
                    (-0.54, side * 0.255, 0.41),
                    (-0.32, side * 0.28, 0.43),
                    (0.28, side * 0.22, 0.38),
                    (0.69, side * 0.145, 0.34),
                ],
                radius=0.017,
                smooth=True,
            )
        )
        tubes.append(
            _tube(
                [
                    (-0.54, side * 0.255, 0.41),
                    (-0.66, side * 0.305, 0.19),
                    (-0.72, side * 0.315, 0.055),
                    (-0.88, side * 0.315, 0.035),
                    (-0.60, side * 0.315, 0.035),
                ],
                radius=0.016,
                corner_radius=0.035,
            )
        )
        tubes.append(
            _tube(
                [
                    (0.70, side * 0.145, 0.34),
                    (0.66, side * 0.145, 0.27),
                    (0.60, side * 0.145, 0.18),
                ],
                radius=0.016,
                smooth=True,
            )
        )

    tubes.append(_tube([(-0.46, -0.30, 0.405), (-0.46, 0.30, 0.405)], radius=0.017))
    tubes.append(_tube([(0.34, -0.235, 0.385), (0.34, 0.235, 0.385)], radius=0.017))
    tubes.append(_tube([(-0.76, -0.315, 0.055), (-0.76, 0.315, 0.055)], radius=0.014))
    return _merge(tubes)


def _upper_support_geometry() -> MeshGeometry:
    return _merge(
        [
            _tube([(0.74, -0.235, 0.465), (0.74, 0.235, 0.465)], radius=0.018),
            _tube([(0.74, -0.19, 0.465), (0.66, -0.18, 0.525)], radius=0.014),
            _tube([(0.74, 0.19, 0.465), (0.66, 0.18, 0.525)], radius=0.014),
            _tube([(0.74, -0.18, 0.465), (0.69, -0.145, 0.34)], radius=0.014),
            _tube([(0.74, 0.18, 0.465), (0.69, 0.145, 0.34)], radius=0.014),
        ]
    )


def _rear_feet_geometry() -> MeshGeometry:
    return _merge(
        [
            _tube([(-0.88, -0.315, 0.035), (-0.60, -0.315, 0.035)], radius=0.020),
            _tube([(-0.88, 0.315, 0.035), (-0.60, 0.315, 0.035)], radius=0.020),
        ]
    )


def _fork_axle_geometry() -> MeshGeometry:
    return _tube([(0.60, -0.210, 0.18), (0.60, 0.210, 0.18)], radius=0.0235)


def _rubber_grips_geometry() -> MeshGeometry:
    return _merge(
        [
            _tube([(-1.42, -0.32, 0.62), (-1.23, -0.32, 0.60)], radius=0.030, smooth=True),
            _tube([(-1.42, 0.32, 0.62), (-1.23, 0.32, 0.60)], radius=0.030, smooth=True),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    tray_green = model.material("painted_green_tray", rgba=(0.12, 0.38, 0.16, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    yellow_rim = model.material("yellow_rim", rgba=(0.92, 0.62, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_tray_shell_geometry(), "tray_shell"),
        material=tray_green,
        name="tray_shell",
    )
    body.visual(
        mesh_from_geometry(_steel_frame_geometry(), "steel_frame"),
        material=dark_steel,
        name="steel_frame",
    )
    body.visual(
        mesh_from_geometry(_fork_axle_geometry(), "fork_axle"),
        material=dark_steel,
        name="fork_axle",
    )
    body.visual(
        mesh_from_geometry(_upper_support_geometry(), "upper_support"),
        material=dark_steel,
        name="upper_support",
    )
    body.visual(
        mesh_from_geometry(_rear_feet_geometry(), "rear_feet"),
        material=dark_steel,
        name="rear_feet",
    )
    body.visual(
        mesh_from_geometry(_rubber_grips_geometry(), "handle_grips"),
        material=black_rubber,
        name="handle_grips",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.180,
                0.105,
                inner_radius=0.122,
                carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
                tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.56),
                sidewall=TireSidewall(style="square", bulge=0.025),
                shoulder=TireShoulder(width=0.010, radius=0.004),
            ),
            "front_tire",
        ),
        material=black_rubber,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.118,
                0.088,
                rim=WheelRim(
                    inner_radius=0.074,
                    flange_height=0.008,
                    flange_thickness=0.004,
                    bead_seat_depth=0.004,
                ),
                hub=WheelHub(radius=0.034, width=0.082, cap_style="domed"),
                face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
                spokes=WheelSpokes(style="split_y", count=6, thickness=0.005, window_radius=0.010),
                bore=WheelBore(style="round", diameter=0.046),
            ),
            "front_rim",
        ),
        material=yellow_rim,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        name="rim",
    )

    model.articulation(
        "body_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(0.60, 0.0, 0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("body_to_wheel")

    ctx.allow_overlap(
        body,
        wheel,
        elem_a="fork_axle",
        elem_b="rim",
        reason="The fixed axle is intentionally seated through the wheel hub bore to show the captured bearing support.",
    )

    ctx.check(
        "front wheel uses continuous axle rotation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={wheel_joint.articulation_type}",
    )

    support_aabb = ctx.part_element_world_aabb(body, elem="upper_support")
    wheel_pos = ctx.part_world_position(wheel)
    support_center_x = None
    if support_aabb is not None:
        support_center_x = 0.5 * (support_aabb[0][0] + support_aabb[1][0])
    ctx.check(
        "upper support is forward of wheel center",
        support_center_x is not None
        and wheel_pos is not None
        and support_center_x > wheel_pos[0] + 0.04,
        details=f"support_center_x={support_center_x}, wheel_pos={wheel_pos}",
    )

    ctx.expect_within(
        wheel,
        body,
        axes="y",
        inner_elem="tire",
        outer_elem="fork_axle",
        margin=0.02,
        name="front wheel is captured between fork sides",
    )
    ctx.expect_overlap(
        body,
        wheel,
        axes="y",
        elem_a="fork_axle",
        elem_b="rim",
        min_overlap=0.07,
        name="axle passes through the wheel hub",
    )
    ctx.expect_overlap(
        body,
        wheel,
        axes="y",
        elem_a="fork_axle",
        elem_b="tire",
        min_overlap=0.09,
        name="axle and fork span the wheel width",
    )

    feet_aabb = ctx.part_element_world_aabb(body, elem="rear_feet")
    ctx.check(
        "two rear resting feet sit at ground height",
        feet_aabb is not None and feet_aabb[0][2] <= 0.02 and feet_aabb[1][2] < 0.07,
        details=f"rear_feet_aabb={feet_aabb}",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spin does not translate the axle center",
        rest_pos is not None
        and turned_pos is not None
        and max(abs(rest_pos[i] - turned_pos[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
