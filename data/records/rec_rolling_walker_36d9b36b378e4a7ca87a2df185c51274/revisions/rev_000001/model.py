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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_caster_assembly(
    model: ArticulatedObject,
    *,
    side: str,
    stem_origin: tuple[float, float, float],
    fork_material,
    hub_material,
    tire_material,
) -> None:
    caster = model.part(f"{side}_caster")
    caster.inertial = Inertial.from_geometry(
        Box((0.05, 0.08, 0.20)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.02, -0.10)),
    )
    caster.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=hub_material,
        name="stem",
    )
    caster.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=hub_material,
        name="swivel_housing",
    )
    caster.visual(
        Box((0.040, 0.036, 0.012)),
        origin=Origin(xyz=(0.0, -0.015, -0.095)),
        material=fork_material,
        name="fork_bridge",
    )
    caster.visual(
        Box((0.006, 0.022, 0.082)),
        origin=Origin(xyz=(0.017, -0.028, -0.140)),
        material=fork_material,
        name="left_blade",
    )
    caster.visual(
        Box((0.006, 0.022, 0.082)),
        origin=Origin(xyz=(-0.017, -0.028, -0.140)),
        material=fork_material,
        name="right_blade",
    )
    caster.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.017, -0.028, -0.178), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="left_axle_cap",
    )
    caster.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.017, -0.028, -0.178), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="right_axle_cap",
    )

    wheel = model.part(f"{side}_caster_wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.026),
        mass=0.28,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    wheel.visual(
        Cylinder(radius=0.055, length=0.026),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=tire_material,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.030, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.015, length=0.032),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=fork_material,
        name="core",
    )

    model.articulation(
        f"frame_to_{side}_caster",
        ArticulationType.REVOLUTE,
        parent="frame",
        child=caster,
        origin=Origin(xyz=stem_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-3.0, upper=3.0),
    )
    model.articulation(
        f"{side}_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=caster,
        child=wheel,
        origin=Origin(xyz=(0.0, -0.028, -0.178)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    frame_silver = model.material("frame_silver", rgba=(0.82, 0.84, 0.86, 1.0))
    fork_grey = model.material("fork_grey", rgba=(0.50, 0.52, 0.55, 1.0))
    hub_grey = model.material("hub_grey", rgba=(0.67, 0.69, 0.72, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_black = model.material("grip_black", rgba=(0.12, 0.12, 0.13, 1.0))
    tip_grey = model.material("tip_grey", rgba=(0.28, 0.29, 0.30, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.60, 0.48, 0.92)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    left_side_path = [
        (0.26, 0.16, 0.30),
        (0.26, 0.14, 0.74),
        (0.255, -0.02, 0.86),
        (0.245, -0.14, 0.90),
        (0.235, -0.22, 0.04),
    ]
    left_lower_brace = [
        (0.252, 0.11, 0.43),
        (0.245, -0.03, 0.45),
        (0.232, -0.18, 0.41),
    ]

    frame.visual(
        _save_mesh(
            "left_side_frame",
            wire_from_points(
                left_side_path,
                radius=0.011,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.045,
                corner_segments=10,
            ),
        ),
        material=frame_silver,
        name="left_side_frame",
    )
    frame.visual(
        _save_mesh(
            "right_side_frame",
            wire_from_points(
                _mirror_x(left_side_path),
                radius=0.011,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.045,
                corner_segments=10,
            ),
        ),
        material=frame_silver,
        name="right_side_frame",
    )
    frame.visual(
        _save_mesh(
            "left_lower_brace",
            wire_from_points(
                left_lower_brace,
                radius=0.010,
                radial_segments=14,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.030,
                corner_segments=8,
            ),
        ),
        material=frame_silver,
        name="left_lower_brace",
    )
    frame.visual(
        _save_mesh(
            "right_lower_brace",
            wire_from_points(
                _mirror_x(left_lower_brace),
                radius=0.010,
                radial_segments=14,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.030,
                corner_segments=8,
            ),
        ),
        material=frame_silver,
        name="right_lower_brace",
    )

    frame.visual(
        _save_mesh(
            "upper_crossbar",
            wire_from_points(
                [(0.26, 0.14, 0.74), (0.0, 0.135, 0.72), (-0.26, 0.14, 0.74)],
                radius=0.011,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.020,
                corner_segments=8,
            ),
        ),
        material=frame_silver,
        name="upper_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.47),
        origin=Origin(xyz=(0.0, 0.04, 0.46), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_silver,
        name="lower_crossbar",
    )
    frame.visual(
        _save_mesh(
            "rear_crossbar",
            wire_from_points(
                [(0.242, -0.164, 0.64), (0.0, -0.16, 0.64), (-0.242, -0.164, 0.64)],
                radius=0.010,
                radial_segments=14,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.018,
                corner_segments=8,
            ),
        ),
        material=frame_silver,
        name="rear_crossbar",
    )
    for name, xyz, size in [
        ("left_upper_joint", (0.255, 0.14, 0.74), (0.035, 0.050, 0.060)),
        ("right_upper_joint", (-0.255, 0.14, 0.74), (0.035, 0.050, 0.060)),
        ("left_lower_joint", (0.242, 0.04, 0.46), (0.030, 0.045, 0.045)),
        ("right_lower_joint", (-0.242, 0.04, 0.46), (0.030, 0.045, 0.045)),
    ]:
        frame.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=frame_silver,
            name=name,
        )

    frame.visual(
        Cylinder(radius=0.014, length=0.10),
        origin=Origin(xyz=(0.245, -0.11, 0.90), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.10),
        origin=Origin(xyz=(-0.245, -0.11, 0.90), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    frame.visual(
        Cylinder(radius=0.0135, length=0.06),
        origin=Origin(xyz=(0.26, 0.16, 0.27)),
        material=frame_silver,
        name="left_front_socket",
    )
    frame.visual(
        Cylinder(radius=0.0135, length=0.06),
        origin=Origin(xyz=(-0.26, 0.16, 0.27)),
        material=frame_silver,
        name="right_front_socket",
    )

    frame.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(xyz=(0.235, -0.22, 0.022)),
        material=tip_grey,
        name="left_rear_tip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(xyz=(-0.235, -0.22, 0.022)),
        material=tip_grey,
        name="right_rear_tip",
    )

    _add_caster_assembly(
        model,
        side="left_front",
        stem_origin=(0.26, 0.16, 0.24),
        fork_material=fork_grey,
        hub_material=hub_grey,
        tire_material=tire_black,
    )
    _add_caster_assembly(
        model,
        side="right_front",
        stem_origin=(-0.26, 0.16, 0.24),
        fork_material=fork_grey,
        hub_material=hub_grey,
        tire_material=tire_black,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_caster = object_model.get_part("left_front_caster")
    right_caster = object_model.get_part("right_front_caster")
    left_wheel = object_model.get_part("left_front_caster_wheel")
    right_wheel = object_model.get_part("right_front_caster_wheel")
    left_swivel = object_model.get_articulation("frame_to_left_front_caster")
    right_swivel = object_model.get_articulation("frame_to_right_front_caster")
    left_spin = object_model.get_articulation("left_front_caster_to_wheel")

    ctx.expect_gap(
        frame,
        frame,
        axis="z",
        positive_elem="left_grip",
        negative_elem="left_rear_tip",
        min_gap=0.82,
        name="left grip stands well above rear support tip",
    )
    ctx.expect_gap(
        frame,
        frame,
        axis="z",
        positive_elem="right_grip",
        negative_elem="right_rear_tip",
        min_gap=0.82,
        name="right grip stands well above rear support tip",
    )
    ctx.expect_gap(
        left_wheel,
        frame,
        axis="y",
        positive_elem="tire",
        negative_elem="left_rear_tip",
        min_gap=0.25,
        name="left caster sits ahead of the rear support",
    )
    ctx.expect_gap(
        right_wheel,
        frame,
        axis="y",
        positive_elem="tire",
        negative_elem="right_rear_tip",
        min_gap=0.25,
        name="right caster sits ahead of the rear support",
    )
    ctx.expect_within(
        left_wheel,
        left_caster,
        axes="x",
        inner_elem="tire",
        outer_elem="fork_bridge",
        margin=0.0,
        name="left caster wheel stays centered between the fork cheeks",
    )
    ctx.expect_within(
        right_wheel,
        right_caster,
        axes="x",
        inner_elem="tire",
        outer_elem="fork_bridge",
        margin=0.0,
        name="right caster wheel stays centered between the fork cheeks",
    )
    ctx.expect_gap(
        left_caster,
        left_wheel,
        axis="z",
        positive_elem="fork_bridge",
        negative_elem="tire",
        min_gap=0.018,
        max_gap=0.030,
        name="left wheel clears the fork bridge",
    )
    ctx.expect_gap(
        right_caster,
        right_wheel,
        axis="z",
        positive_elem="fork_bridge",
        negative_elem="tire",
        min_gap=0.018,
        max_gap=0.030,
        name="right wheel clears the fork bridge",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    footprint_ok = (
        frame_aabb is not None
        and (frame_aabb[1][0] - frame_aabb[0][0]) < 0.60
        and (frame_aabb[1][1] - frame_aabb[0][1]) < 0.50
    )
    ctx.check(
        "frame keeps a narrow walker footprint",
        footprint_ok,
        details=f"frame_aabb={frame_aabb}",
    )

    rest_left = ctx.part_world_position(left_wheel)
    with ctx.pose({left_swivel: 1.2}):
        swivel_left = ctx.part_world_position(left_wheel)
    ctx.check(
        "left caster swivels around its vertical stem",
        rest_left is not None
        and swivel_left is not None
        and swivel_left[0] > rest_left[0] + 0.02
        and abs(swivel_left[2] - rest_left[2]) < 1e-6,
        details=f"rest={rest_left}, swivel={swivel_left}",
    )

    rest_right = ctx.part_world_position(right_wheel)
    with ctx.pose({right_swivel: -1.2}):
        swivel_right = ctx.part_world_position(right_wheel)
    ctx.check(
        "right caster swivels around its vertical stem",
        rest_right is not None
        and swivel_right is not None
        and swivel_right[0] < rest_right[0] - 0.02
        and abs(swivel_right[2] - rest_right[2]) < 1e-6,
        details=f"rest={rest_right}, swivel={swivel_right}",
    )

    with ctx.pose({left_spin: 1.5}):
        spun_left = ctx.part_world_position(left_wheel)
    ctx.check(
        "left caster wheel spins about its axle without translating",
        rest_left is not None
        and spun_left is not None
        and max(abs(spun_left[i] - rest_left[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_left}, spun={spun_left}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
