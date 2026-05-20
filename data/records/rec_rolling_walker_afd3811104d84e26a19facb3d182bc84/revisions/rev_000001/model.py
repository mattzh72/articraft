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
    tube_from_spline_points,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _add_caster_fork(part, *, fork_material) -> None:
    part.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=fork_material,
        name="swivel_cap",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=fork_material,
        name="swivel_stem",
    )
    part.visual(
        Box((0.052, 0.034, 0.014)),
        origin=Origin(xyz=(0.0, -0.012, -0.066)),
        material=fork_material,
        name="fork_crown",
    )
    part.visual(
        Box((0.008, 0.012, 0.090)),
        origin=Origin(xyz=(0.020, -0.018, -0.115)),
        material=fork_material,
        name="outer_blade",
    )
    part.visual(
        Box((0.008, 0.012, 0.090)),
        origin=Origin(xyz=(-0.020, -0.018, -0.115)),
        material=fork_material,
        name="inner_blade",
    )


def _add_caster_wheel(part, *, tire_material, hub_material) -> None:
    wheel_axis = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=0.060, length=0.024),
        origin=wheel_axis,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=wheel_axis,
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="outer_cap",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.013, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="inner_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_caster_walker")

    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    grip_black = model.material("grip_black", rgba=(0.12, 0.12, 0.13, 1.0))
    tip_gray = model.material("tip_gray", rgba=(0.24, 0.24, 0.25, 1.0))
    caster_dark = model.material("caster_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.62, 0.64, 0.67, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))

    frame = model.part("walker_frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.46, 0.94)),
        mass=6.8,
        origin=Origin(xyz=(0.0, -0.02, 0.47)),
    )

    tube_radius = 0.013
    side_points = [
        (0.275, 0.170, 0.235),
        (0.275, 0.150, 0.720),
        (0.270, 0.050, 0.840),
        (0.250, -0.110, 0.920),
        (0.222, -0.160, 0.920),
        (0.214, -0.180, 0.460),
        (0.206, -0.220, 0.000),
    ]

    left_side = wire_from_points(
        side_points,
        radius=tube_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.060,
        corner_segments=12,
    )
    right_side = wire_from_points(
        _mirror_x(side_points),
        radius=tube_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.060,
        corner_segments=12,
    )
    upper_crossbar = tube_from_spline_points(
        [(-0.275, 0.145, 0.630), (0.275, 0.145, 0.630)],
        radius=tube_radius,
        samples_per_segment=2,
        radial_segments=18,
    )
    lower_crossbar = tube_from_spline_points(
        [(-0.214, -0.180, 0.460), (0.214, -0.180, 0.460)],
        radius=tube_radius,
        samples_per_segment=2,
        radial_segments=18,
    )
    left_rear_brace = tube_from_spline_points(
        [(0.250, -0.110, 0.920), (0.214, -0.180, 0.460)],
        radius=tube_radius * 0.92,
        samples_per_segment=2,
        radial_segments=16,
    )
    right_rear_brace = tube_from_spline_points(
        [(-0.250, -0.110, 0.920), (-0.214, -0.180, 0.460)],
        radius=tube_radius * 0.92,
        samples_per_segment=2,
        radial_segments=16,
    )

    frame.visual(_mesh("left_side_frame", left_side), material=aluminum, name="left_side_frame")
    frame.visual(_mesh("right_side_frame", right_side), material=aluminum, name="right_side_frame")
    frame.visual(_mesh("upper_crossbar", upper_crossbar), material=aluminum, name="upper_crossbar")
    frame.visual(_mesh("lower_crossbar", lower_crossbar), material=aluminum, name="lower_crossbar")
    frame.visual(_mesh("left_rear_brace", left_rear_brace), material=aluminum, name="left_rear_brace")
    frame.visual(_mesh("right_rear_brace", right_rear_brace), material=aluminum, name="right_rear_brace")

    frame.visual(
        Cylinder(radius=0.019, length=0.125),
        origin=Origin(xyz=(0.237, -0.128, 0.920), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="left_hand_grip",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.125),
        origin=Origin(xyz=(-0.237, -0.128, 0.920), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="right_hand_grip",
    )

    frame.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.206, -0.220, 0.014)),
        material=tip_gray,
        name="left_rear_tip",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(-0.206, -0.220, 0.014)),
        material=tip_gray,
        name="right_rear_tip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.275, 0.170, 0.215)),
        material=tip_gray,
        name="left_caster_socket",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(-0.275, 0.170, 0.215)),
        material=tip_gray,
        name="right_caster_socket",
    )

    left_fork = model.part("left_caster_fork")
    left_fork.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.150)),
        mass=0.22,
        origin=Origin(xyz=(0.0, -0.012, -0.075)),
    )
    _add_caster_fork(left_fork, fork_material=caster_dark)

    right_fork = model.part("right_caster_fork")
    right_fork.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.150)),
        mass=0.22,
        origin=Origin(xyz=(0.0, -0.012, -0.075)),
    )
    _add_caster_fork(right_fork, fork_material=caster_dark)

    left_wheel = model.part("left_caster_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.024),
        mass=0.18,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(left_wheel, tire_material=rubber_black, hub_material=wheel_hub)

    right_wheel = model.part("right_caster_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.024),
        mass=0.18,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(right_wheel, tire_material=rubber_black, hub_material=wheel_hub)

    model.articulation(
        "left_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_fork,
        origin=Origin(xyz=(0.275, 0.170, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_fork,
        origin=Origin(xyz=(-0.275, 0.170, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "left_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=left_fork,
        child=left_wheel,
        origin=Origin(xyz=(0.0, -0.018, -0.135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=22.0),
    )
    model.articulation(
        "right_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=right_fork,
        child=right_wheel,
        origin=Origin(xyz=(0.0, -0.018, -0.135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=22.0),
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
    frame = object_model.get_part("walker_frame")
    left_fork = object_model.get_part("left_caster_fork")
    right_fork = object_model.get_part("right_caster_fork")
    left_wheel = object_model.get_part("left_caster_wheel")
    right_wheel = object_model.get_part("right_caster_wheel")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_spin = object_model.get_articulation("left_caster_spin")
    right_spin = object_model.get_articulation("right_caster_spin")

    left_grip = ctx.part_element_world_aabb(frame, elem="left_hand_grip")
    right_grip = ctx.part_element_world_aabb(frame, elem="right_hand_grip")
    grip_heights = []
    for grip in (left_grip, right_grip):
        if grip is not None:
            grip_heights.append((grip[0][2] + grip[1][2]) * 0.5)
    ctx.check(
        "hand grips sit at realistic walker height",
        len(grip_heights) == 2 and all(0.88 <= height <= 0.95 for height in grip_heights),
        details=f"grip_heights={grip_heights}",
    )
    ctx.check(
        "caster joints use swivel stems and spinning axles",
        left_swivel.articulation_type == ArticulationType.REVOLUTE
        and right_swivel.articulation_type == ArticulationType.REVOLUTE
        and left_swivel.axis == (0.0, 0.0, 1.0)
        and right_swivel.axis == (0.0, 0.0, 1.0)
        and left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_spin.axis == (0.0, 0.0, 1.0)
        and right_spin.axis == (0.0, 0.0, 1.0),
        details=(
            f"left_swivel={left_swivel.articulation_type}/{left_swivel.axis}, "
            f"right_swivel={right_swivel.articulation_type}/{right_swivel.axis}, "
            f"left_spin={left_spin.articulation_type}/{left_spin.axis}, "
            f"right_spin={right_spin.articulation_type}/{right_spin.axis}"
        ),
    )
    ctx.expect_gap(
        frame,
        left_wheel,
        axis="z",
        positive_elem="left_caster_socket",
        negative_elem="tire",
        min_gap=0.070,
        max_gap=0.090,
        name="left caster wheel hangs below its socket",
    )
    ctx.expect_gap(
        frame,
        right_wheel,
        axis="z",
        positive_elem="right_caster_socket",
        negative_elem="tire",
        min_gap=0.070,
        max_gap=0.090,
        name="right caster wheel hangs below its socket",
    )

    def _bottom(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        return None if aabb is None else aabb[0][2]

    rest_supports = {
        "left_rear_tip": _bottom(frame, "left_rear_tip"),
        "right_rear_tip": _bottom(frame, "right_rear_tip"),
        "left_tire": _bottom(left_wheel, "tire"),
        "right_tire": _bottom(right_wheel, "tire"),
    }
    rest_values = [value for value in rest_supports.values() if value is not None]
    ctx.check(
        "front casters and rear tips share one support plane",
        len(rest_values) == 4 and max(rest_values) - min(rest_values) <= 0.004,
        details=f"support_bottoms={rest_supports}",
    )
    ctx.expect_gap(
        left_fork,
        left_wheel,
        axis="z",
        positive_elem="fork_crown",
        negative_elem="tire",
        min_gap=0.001,
        max_gap=0.012,
        name="left fork crown clears the tire",
    )
    ctx.expect_gap(
        right_fork,
        right_wheel,
        axis="z",
        positive_elem="fork_crown",
        negative_elem="tire",
        min_gap=0.001,
        max_gap=0.012,
        name="right fork crown clears the tire",
    )

    with ctx.pose(
        {
            left_swivel: pi / 2.0,
            right_swivel: -pi / 2.0,
            left_spin: pi / 3.0,
            right_spin: -pi / 4.0,
        }
    ):
        swiveled_supports = {
            "left_rear_tip": _bottom(frame, "left_rear_tip"),
            "right_rear_tip": _bottom(frame, "right_rear_tip"),
            "left_tire": _bottom(left_wheel, "tire"),
            "right_tire": _bottom(right_wheel, "tire"),
        }
        swiveled_values = [value for value in swiveled_supports.values() if value is not None]
        ctx.check(
            "swiveled casters keep the walker level",
            len(swiveled_values) == 4 and max(swiveled_values) - min(swiveled_values) <= 0.004,
            details=f"swiveled_support_bottoms={swiveled_supports}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
