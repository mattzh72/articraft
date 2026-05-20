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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.17, 0.18, 0.19, 1.0))
    door_paint = model.material("door_paint", rgba=(0.22, 0.23, 0.24, 1.0))
    interior_liner = model.material("interior_liner", rgba=(0.55, 0.51, 0.43, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.14, 0.14, 0.15, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    satin_chrome = model.material("satin_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dial_metal = model.material("dial_metal", rgba=(0.84, 0.79, 0.70, 1.0))

    opening_w = 0.440
    opening_h = 0.600
    wall_t = 0.022
    back_t = 0.012
    body_w = opening_w + 2.0 * wall_t
    body_h = opening_h + 2.0 * wall_t
    depth = 0.380
    flange_w = 0.640
    flange_h = 0.780
    flange_t = 0.010

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((flange_w, depth + flange_t, flange_h)),
        mass=46.0,
        origin=Origin(xyz=(0.0, -(depth - flange_t) * 0.5, 0.0)),
    )

    trim_flange_geom = ExtrudeWithHolesGeometry(
        _rect_profile(flange_w, flange_h),
        [_rect_profile(opening_w, opening_h)],
        flange_t,
        center=False,
    ).rotate_x(-math.pi / 2.0)
    cabinet.visual(
        _save_mesh("safe_trim_flange", trim_flange_geom),
        material=cabinet_paint,
        name="trim_flange",
    )
    cabinet.visual(
        Box((body_w, back_t, body_h)),
        origin=Origin(xyz=(0.0, -depth + back_t * 0.5, 0.0)),
        material=cabinet_paint,
        name="back_wall",
    )
    cabinet.visual(
        Box((wall_t, depth - back_t, body_h)),
        origin=Origin(xyz=(-opening_w * 0.5 - wall_t * 0.5, -(depth - back_t) * 0.5, 0.0)),
        material=cabinet_paint,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall_t, depth - back_t, body_h)),
        origin=Origin(xyz=(opening_w * 0.5 + wall_t * 0.5, -(depth - back_t) * 0.5, 0.0)),
        material=cabinet_paint,
        name="right_wall",
    )
    cabinet.visual(
        Box((opening_w, depth - back_t, wall_t)),
        origin=Origin(xyz=(0.0, -(depth - back_t) * 0.5, opening_h * 0.5 + wall_t * 0.5)),
        material=cabinet_paint,
        name="top_wall",
    )
    cabinet.visual(
        Box((opening_w, depth - back_t, wall_t)),
        origin=Origin(xyz=(0.0, -(depth - back_t) * 0.5, -opening_h * 0.5 - wall_t * 0.5)),
        material=cabinet_paint,
        name="bottom_wall",
    )
    cabinet.visual(
        Box((opening_w, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, -0.190, 0.070)),
        material=interior_liner,
        name="interior_shelf",
    )
    cabinet.visual(
        Box((0.016, flange_t, 0.620)),
        origin=Origin(xyz=(-opening_w * 0.5 - 0.005, flange_t * 0.5, 0.0)),
        material=dark_hardware,
        name="hinge_jamb",
    )

    door = model.part("door")
    door_w = 0.434
    door_h = 0.594
    door_t = 0.080
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=28.0,
        origin=Origin(xyz=(door_w * 0.5, door_t * 0.5, 0.0)),
    )
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w * 0.5, door_t * 0.5, 0.0)),
        material=door_paint,
        name="door_body",
    )
    door.visual(
        Box((0.388, 0.014, 0.518)),
        origin=Origin(xyz=(door_w * 0.5, 0.073, 0.0)),
        material=door_paint,
        name="front_plate",
    )
    door.visual(
        Box((0.360, 0.018, 0.500)),
        origin=Origin(xyz=(door_w * 0.5, 0.020, 0.0)),
        material=interior_liner,
        name="inner_panel",
    )
    door.visual(
        Box((0.016, 0.072, 0.580)),
        origin=Origin(xyz=(0.008, 0.044, 0.0)),
        material=dark_hardware,
        name="hinge_stile",
    )
    door.visual(
        Cylinder(radius=0.060, length=0.010),
        origin=Origin(
            xyz=(door_w * 0.5, 0.085, 0.170),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_hardware,
        name="dial_boss",
    )
    door.visual(
        Cylinder(radius=0.048, length=0.014),
        origin=Origin(
            xyz=(door_w * 0.5, 0.087, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_hardware,
        name="handle_boss",
    )

    dial = model.part("dial")
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.026),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )
    dial.visual(
        Cylinder(radius=0.058, length=0.022),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_metal,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.040, length=0.008),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="dial_face",
    )
    dial.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.029, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="dial_cap",
    )
    dial.visual(
        Box((0.003, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.0285, 0.028)),
        material=dark_hardware,
        name="dial_pointer",
    )

    wheel_handle = model.part("wheel_handle")
    wheel_handle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.050),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )
    wheel_rim = TorusGeometry(
        radius=0.082,
        tube=0.008,
        radial_segments=18,
        tubular_segments=72,
    ).rotate_x(math.pi / 2.0)
    wheel_handle.visual(
        _save_mesh("safe_wheel_rim", wheel_rim),
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
        material=satin_chrome,
        name="wheel_rim",
    )
    wheel_handle.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="wheel_shaft",
    )
    wheel_handle.visual(
        Cylinder(radius=0.028, length=0.026),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="wheel_hub",
    )
    for index, angle in enumerate(
        (0.0, math.pi * 0.5, math.pi, math.pi * 1.5)
    ):
        spoke_inner = (0.024 * math.cos(angle), 0.028, 0.024 * math.sin(angle))
        spoke_outer = (0.076 * math.cos(angle), 0.028, 0.076 * math.sin(angle))
        _add_member(
            wheel_handle,
            spoke_inner,
            spoke_outer,
            radius=0.006,
            material=satin_chrome,
            name=f"spoke_{index}",
        )
    wheel_handle.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hub_cap",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-door_w * 0.5, flange_t, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.0,
            lower=0.0,
            upper=1.85,
        ),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(door_w * 0.5, 0.090, 0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "door_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=wheel_handle,
        origin=Origin(xyz=(door_w * 0.5, 0.094, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0),
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

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    wheel_handle = object_model.get_part("wheel_handle")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    wheel_spin = object_model.get_articulation("door_to_wheel")

    ctx.check(
        "door hinge axis is vertical",
        tuple(round(value, 3) for value in door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "dial spins about the door normal",
        tuple(round(value, 3) for value in dial_spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={dial_spin.axis}",
    )
    ctx.check(
        "wheel spins about the door normal",
        tuple(round(value, 3) for value in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={wheel_spin.axis}",
    )
    ctx.check(
        "dial articulation is continuous",
        dial_spin.motion_limits is not None
        and dial_spin.motion_limits.lower is None
        and dial_spin.motion_limits.upper is None,
        details=f"limits={dial_spin.motion_limits}",
    )
    ctx.check(
        "wheel articulation is continuous",
        wheel_spin.motion_limits is not None
        and wheel_spin.motion_limits.lower is None
        and wheel_spin.motion_limits.upper is None,
        details=f"limits={wheel_spin.motion_limits}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="door_body",
            negative_elem="trim_flange",
            min_gap=0.0,
            max_gap=0.001,
            name="door seats against the trim flange",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            elem_a="door_body",
            elem_b="trim_flange",
            min_overlap=0.43,
            name="door covers the safe opening",
        )
        ctx.expect_contact(
            dial,
            door,
            elem_a="dial_body",
            elem_b="dial_boss",
            name="dial mounts on the raised boss",
        )
        ctx.expect_contact(
            wheel_handle,
            door,
            elem_a="wheel_shaft",
            elem_b="handle_boss",
            name="wheel handle mounts on the center boss",
        )
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_body")

    with ctx.pose({door_hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_body")

    door_swings_outward = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.28
    )
    ctx.check(
        "door swings outward on the left hinge",
        door_swings_outward,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
