from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged: MeshGeometry | None = None
    for geometry in geometries:
        if merged is None:
            merged = geometry.copy()
        else:
            merged.merge(geometry)
    return merged if merged is not None else MeshGeometry()


def _box(
    size: tuple[float, float, float],
    *,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> MeshGeometry:
    geom = BoxGeometry(size)
    rx, ry, rz = rpy
    if rx:
        geom.rotate_x(rx)
    if ry:
        geom.rotate_y(ry)
    if rz:
        geom.rotate_z(rz)
    geom.translate(*xyz)
    return geom


def _cylinder(
    radius: float,
    length: float,
    *,
    xyz: tuple[float, float, float],
    axis: str = "z",
) -> MeshGeometry:
    geom = CylinderGeometry(radius, length, radial_segments=28, closed=True)
    if axis == "x":
        geom.rotate_y(math.pi / 2.0)
    elif axis == "y":
        geom.rotate_x(math.pi / 2.0)
    geom.translate(*xyz)
    return geom


def _bearing_shell(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    xyz: tuple[float, float, float],
) -> MeshGeometry:
    geom = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -length / 2.0),
            (outer_radius, length / 2.0),
        ],
        [
            (inner_radius, -length / 2.0),
            (inner_radius, length / 2.0),
        ],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(*xyz)
    return geom


def _polar_box(
    size: tuple[float, float, float],
    *,
    radius: float,
    angle: float,
    y: float,
    tangent: bool,
) -> MeshGeometry:
    return _box(
        size,
        xyz=(radius * math.cos(angle), y, radius * math.sin(angle)),
        rpy=(0.0, angle + (math.pi / 2.0 if tangent else 0.0), 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_undershot_waterwheel")

    timber = model.material("timber", rgba=(0.43, 0.31, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.54, 0.57, 0.60, 1.0))
    seal = model.material("seal", rgba=(0.12, 0.12, 0.14, 1.0))

    frame = model.part("frame")
    wheel = model.part("wheel")

    axle_height = 1.18
    frame_half_width = 0.58
    axle_radius = 0.055

    frame_structure = _merge_geometries(
        _box((2.24, 0.12, 0.12), xyz=(0.0, frame_half_width, 0.06)),
        _box((2.24, 0.12, 0.12), xyz=(0.0, -frame_half_width, 0.06)),
        _box((0.14, 1.16, 0.14), xyz=(1.02, 0.0, 0.10)),
        _box((0.14, 1.16, 0.14), xyz=(-1.02, 0.0, 0.10)),
        _box((0.12, 0.12, 1.10), xyz=(0.70, frame_half_width, 0.55)),
        _box((0.12, 0.12, 1.10), xyz=(-0.70, frame_half_width, 0.55)),
        _box((0.12, 0.12, 1.10), xyz=(0.70, -frame_half_width, 0.55)),
        _box((0.12, 0.12, 1.10), xyz=(-0.70, -frame_half_width, 0.55)),
        _box((1.48, 0.10, 0.12), xyz=(0.0, frame_half_width, 0.34)),
        _box((1.48, 0.10, 0.12), xyz=(0.0, -frame_half_width, 0.34)),
        _box((1.48, 0.10, 0.12), xyz=(0.0, frame_half_width, 1.02)),
        _box((1.48, 0.10, 0.12), xyz=(0.0, -frame_half_width, 1.02)),
        _box(
            (0.56, 0.10, 0.10),
            xyz=(0.50, frame_half_width, 0.76),
            rpy=(0.0, math.radians(60.0), 0.0),
        ),
        _box(
            (0.56, 0.10, 0.10),
            xyz=(-0.50, frame_half_width, 0.76),
            rpy=(0.0, -math.radians(60.0), 0.0),
        ),
        _box(
            (0.56, 0.10, 0.10),
            xyz=(0.50, -frame_half_width, 0.76),
            rpy=(0.0, math.radians(60.0), 0.0),
        ),
        _box(
            (0.56, 0.10, 0.10),
            xyz=(-0.50, -frame_half_width, 0.76),
            rpy=(0.0, -math.radians(60.0), 0.0),
        ),
        _box((0.56, 0.10, 0.10), xyz=(0.48, frame_half_width, 0.52)),
        _box((0.56, 0.10, 0.10), xyz=(-0.48, frame_half_width, 0.52)),
        _box((0.56, 0.10, 0.10), xyz=(0.48, -frame_half_width, 0.52)),
        _box((0.56, 0.10, 0.10), xyz=(-0.48, -frame_half_width, 0.52)),
        _box((0.34, 0.16, 0.22), xyz=(0.0, frame_half_width, axle_height - 0.165)),
        _box((0.34, 0.16, 0.22), xyz=(0.0, -frame_half_width, axle_height - 0.165)),
    )

    frame_hardware = _merge_geometries(
        _box((0.06, 0.08, 0.22), xyz=(0.12, frame_half_width + 0.12, axle_height - 0.02)),
        _box((0.06, 0.08, 0.22), xyz=(-0.12, frame_half_width + 0.12, axle_height - 0.02)),
        _box((0.06, 0.08, 0.22), xyz=(0.12, -frame_half_width - 0.12, axle_height - 0.02)),
        _box((0.06, 0.08, 0.22), xyz=(-0.12, -frame_half_width - 0.12, axle_height - 0.02)),
        _box((0.44, 0.14, 0.05), xyz=(0.0, frame_half_width + 0.17, axle_height + 0.09)),
        _box((0.44, 0.14, 0.05), xyz=(0.0, -frame_half_width - 0.17, axle_height + 0.09)),
        _box((0.44, 0.06, 0.03), xyz=(0.0, frame_half_width + 0.22, axle_height + 0.05)),
        _box((0.44, 0.06, 0.03), xyz=(0.0, -frame_half_width - 0.22, axle_height + 0.05)),
    )

    frame.visual(
        mesh_from_geometry(frame_structure, "frame_structure"),
        material=timber,
        name="frame_structure",
    )
    frame.visual(
        mesh_from_geometry(frame_hardware, "frame_hardware"),
        material=seal,
        name="frame_hardware",
    )

    rims = [
        _bearing_shell(
            inner_radius=0.70,
            outer_radius=0.82,
            length=0.14,
            xyz=(0.0, -0.33, 0.0),
        ),
        _bearing_shell(
            inner_radius=0.70,
            outer_radius=0.82,
            length=0.14,
            xyz=(0.0, 0.33, 0.0),
        ),
    ]

    spokes: list[MeshGeometry] = []
    for side_y in (-0.29, 0.29):
        for index in range(8):
            angle = (2.0 * math.pi * index) / 8.0
            spokes.append(
                _polar_box(
                    (0.72, 0.10, 0.07),
                    radius=0.36,
                    angle=angle,
                    y=side_y,
                    tangent=False,
                )
            )

    paddles: list[MeshGeometry] = []
    paddle_cleats: list[MeshGeometry] = []
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0 + (math.pi / 12.0)
        paddles.append(
            _polar_box(
                (0.30, 0.74, 0.08),
                radius=0.79,
                angle=angle,
                y=0.0,
                tangent=False,
            )
        )
        paddle_cleats.append(
            _polar_box(
                (0.24, 0.14, 0.10),
                radius=0.77,
                angle=angle,
                y=-0.33,
                tangent=False,
            )
        )
        paddle_cleats.append(
            _polar_box(
                (0.24, 0.14, 0.10),
                radius=0.77,
                angle=angle,
                y=0.33,
                tangent=False,
            )
        )

    wheel_wood = _merge_geometries(*rims, *spokes, *paddles, *paddle_cleats)
    wheel_steel = _merge_geometries(
        _cylinder(axle_radius, 1.28, xyz=(0.0, 0.0, 0.0), axis="y"),
        _cylinder(0.16, 0.64, xyz=(0.0, 0.0, 0.0), axis="y"),
    )

    wheel.visual(
        mesh_from_geometry(wheel_wood, "wheel_wood"),
        material=timber,
        name="wheel_wood",
    )
    wheel.visual(
        mesh_from_geometry(wheel_steel, "wheel_steel"),
        material=steel,
        name="wheel_steel",
    )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    axle_joint = object_model.get_articulation("frame_to_wheel")
    frame_structure = frame.get_visual("frame_structure")
    wheel_steel = wheel.get_visual("wheel_steel")

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

    ctx.check(
        "wheel_joint_axis_is_centered_axle",
        axle_joint.axis == (0.0, 1.0, 0.0),
        f"Expected centered axle axis (0, 1, 0), got {axle_joint.axis!r}",
    )
    ctx.expect_origin_distance(
        frame,
        wheel,
        axes="xy",
        max_dist=0.001,
        name="wheel_centered_between_side_frames",
    )
    ctx.expect_origin_gap(
        wheel,
        frame,
        axis="z",
        min_gap=1.15,
        max_gap=1.21,
        name="axle_height_above_base",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a=wheel_steel,
        elem_b=frame_structure,
        contact_tol=0.003,
        name="axle_supported_by_side_bearings",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xy",
        margin=0.02,
        name="wheel_stays_within_frame_footprint",
    )

    with ctx.pose({axle_joint: math.pi / 6.0}):
        ctx.expect_contact(
            wheel,
            frame,
            elem_a=wheel_steel,
            elem_b=frame_structure,
            contact_tol=0.003,
            name="axle_remains_supported_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
