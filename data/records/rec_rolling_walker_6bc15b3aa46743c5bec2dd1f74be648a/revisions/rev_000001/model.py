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
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _side_frame_geometry(side_x: float):
    return tube_from_spline_points(
        [
            (side_x, 0.135, 0.180),
            (side_x, 0.125, 0.420),
            (side_x, 0.105, 0.760),
            (side_x, 0.060, 0.815),
            (side_x, -0.020, 0.825),
            (side_x, -0.105, 0.815),
            (side_x, -0.135, 0.740),
            (side_x, -0.145, 0.430),
            (side_x, -0.145, 0.030),
        ],
        radius=0.0135,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def _build_caster_fork(part, *, metal, trim) -> None:
    part.visual(
        Cylinder(radius=0.0075, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=metal,
        name="swivel_stem",
    )
    part.visual(
        Cylinder(radius=0.0145, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=metal,
        name="bearing_collar",
    )
    part.visual(
        Box((0.052, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, -0.010, -0.035)),
        material=metal,
        name="fork_crown",
    )
    for side_x in (-0.020, 0.020):
        part.visual(
            Box((0.008, 0.022, 0.074)),
            origin=Origin(xyz=(side_x, -0.022, -0.079)),
            material=metal,
            name=f"fork_blade_{'left' if side_x > 0 else 'right'}",
        )
        part.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(xyz=(side_x, -0.022, -0.115), rpy=(0.0, pi / 2.0, 0.0)),
            material=trim,
            name=f"axle_cap_{'left' if side_x > 0 else 'right'}",
        )


def _build_caster_wheel(part, *, tire, hub) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=0.065, length=0.024),
        origin=spin_origin,
        material=tire,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.040, length=0.028),
        origin=spin_origin,
        material=hub,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub,
        name="hub_face_outer",
    )
    part.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub,
        name="hub_face_inner",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    aluminum = model.material("aluminum", rgba=(0.78, 0.79, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.27, 0.29, 1.0))
    grip_black = model.material("grip_black", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber_gray = model.material("rubber_gray", rgba=(0.20, 0.20, 0.21, 1.0))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.58, 0.36, 0.84)),
        mass=6.5,
        origin=Origin(xyz=(0.0, -0.005, 0.42)),
    )

    frame.visual(_save_mesh("left_side_frame", _side_frame_geometry(0.235)), material=aluminum, name="left_side_frame")
    frame.visual(_save_mesh("right_side_frame", _side_frame_geometry(-0.235)), material=aluminum, name="right_side_frame")

    frame.visual(
        Cylinder(radius=0.012, length=0.460),
        origin=Origin(xyz=(0.0, 0.110, 0.680), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="upper_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.460),
        origin=Origin(xyz=(0.0, 0.122, 0.470), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="lower_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.452),
        origin=Origin(xyz=(0.0, -0.145, 0.265), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="rear_stabilizer",
    )

    for side_x in (0.235, -0.235):
        frame.visual(
            Cylinder(radius=0.018, length=0.042),
            origin=Origin(xyz=(side_x, 0.135, 0.187)),
            material=dark_steel,
            name=f"front_socket_{'left' if side_x > 0 else 'right'}",
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.035),
            origin=Origin(xyz=(side_x, -0.145, 0.018)),
            material=rubber_gray,
            name=f"rear_tip_{'left' if side_x > 0 else 'right'}",
        )
        frame.visual(
            Cylinder(radius=0.017, length=0.120),
            origin=Origin(xyz=(side_x, -0.025, 0.820), rpy=(0.0, pi / 2.0, 0.0)),
            material=grip_black,
            name=f"hand_grip_{'left' if side_x > 0 else 'right'}",
        )

    left_front_caster = model.part("left_front_caster")
    left_front_caster.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.130)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.010, -0.065)),
    )
    _build_caster_fork(left_front_caster, metal=dark_steel, trim=rubber_gray)

    right_front_caster = model.part("right_front_caster")
    right_front_caster.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.130)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.010, -0.065)),
    )
    _build_caster_fork(right_front_caster, metal=dark_steel, trim=rubber_gray)

    left_front_wheel = model.part("left_front_wheel")
    left_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.024),
        mass=0.24,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_caster_wheel(left_front_wheel, tire=tire_black, hub=rubber_gray)

    right_front_wheel = model.part("right_front_wheel")
    right_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.024),
        mass=0.24,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_caster_wheel(right_front_wheel, tire=tire_black, hub=rubber_gray)

    model.articulation(
        "left_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_front_caster,
        origin=Origin(xyz=(0.235, 0.135, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_front_caster,
        origin=Origin(xyz=(-0.235, 0.135, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_front_caster,
        child=left_front_wheel,
        origin=Origin(xyz=(0.0, -0.022, -0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=30.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_front_caster,
        child=right_front_wheel,
        origin=Origin(xyz=(0.0, -0.022, -0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=30.0),
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
    frame = object_model.get_part("frame")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    left_wheel = object_model.get_part("left_front_wheel")
    right_wheel = object_model.get_part("right_front_wheel")

    overall_min = [float("inf"), float("inf"), float("inf")]
    overall_max = [float("-inf"), float("-inf"), float("-inf")]
    for part_name in ("frame", "left_front_caster", "right_front_caster", "left_front_wheel", "right_front_wheel"):
        bounds = ctx.part_world_aabb(object_model.get_part(part_name))
        if bounds is None:
            continue
        for axis in range(3):
            overall_min[axis] = min(overall_min[axis], bounds[0][axis])
            overall_max[axis] = max(overall_max[axis], bounds[1][axis])

    if all(v != float("inf") for v in overall_min) and all(v != float("-inf") for v in overall_max):
        frame_dims = tuple(mx - mn for mn, mx in zip(overall_min, overall_max))
        ctx.check(
            "walker stays compact overall",
            frame_dims[0] < 0.62 and frame_dims[1] < 0.40 and 0.80 < frame_dims[2] < 0.92,
            details=f"dims={frame_dims}",
        )
    else:
        ctx.fail("walker stays compact overall", "overall AABB unavailable")

    ctx.expect_gap(
        frame,
        frame,
        axis="z",
        positive_elem="upper_crossbar",
        negative_elem="lower_crossbar",
        min_gap=0.16,
        name="walker has open hand-space between crossbars",
    )

    left_wheel_bounds = ctx.part_world_aabb(left_wheel)
    right_wheel_bounds = ctx.part_world_aabb(right_wheel)
    left_tip_bounds = ctx.part_element_world_aabb(frame, elem="rear_tip_left")
    right_tip_bounds = ctx.part_element_world_aabb(frame, elem="rear_tip_right")
    support_ok = (
        left_wheel_bounds is not None
        and right_wheel_bounds is not None
        and left_tip_bounds is not None
        and right_tip_bounds is not None
        and abs(left_wheel_bounds[0][2]) < 0.005
        and abs(right_wheel_bounds[0][2]) < 0.005
        and left_tip_bounds[0][2] < 0.006
        and right_tip_bounds[0][2] < 0.006
    )
    ctx.check(
        "front casters and rear tips define a near-floor four-point stance",
        support_ok,
        details=(
            f"left_wheel_min_z={None if left_wheel_bounds is None else left_wheel_bounds[0][2]}, "
            f"right_wheel_min_z={None if right_wheel_bounds is None else right_wheel_bounds[0][2]}, "
            f"left_tip_min_z={None if left_tip_bounds is None else left_tip_bounds[0][2]}, "
            f"right_tip_min_z={None if right_tip_bounds is None else right_tip_bounds[0][2]}"
        ),
    )

    rest_pos = ctx.part_world_position(left_wheel)
    with ctx.pose({left_swivel: pi / 2.0}):
        turned_pos = ctx.part_world_position(left_wheel)
    swivel_ok = (
        rest_pos is not None
        and turned_pos is not None
        and abs(turned_pos[2] - rest_pos[2]) < 1e-6
        and abs(turned_pos[0] - rest_pos[0]) > 0.015
        and abs(turned_pos[1] - rest_pos[1]) > 0.015
    )
    ctx.check(
        "left caster swivel reorients the trailing wheel around the stem",
        swivel_ok,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
