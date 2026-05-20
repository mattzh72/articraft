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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


AXLE_HEIGHT = 16.5
WHEEL_RADIUS = 12.9
INNER_RIM_RADIUS = 11.9
PIVOT_RADIUS = 12.35
WHEEL_HALF_WIDTH = 0.95
CABIN_COUNT = 10


def _circle_points_yz(radius: float, x: float, samples: int = 64) -> list[tuple[float, float, float]]:
    return [
        (
            x,
            radius * math.cos(i * math.tau / samples),
            radius * math.sin(i * math.tau / samples),
        )
        for i in range(samples)
    ]


def _add_box_beam_xz(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    thickness: tuple[float, float],
    material,
    name: str,
) -> None:
    x0, y0, z0 = start
    x1, y1, z1 = end
    if abs(y1 - y0) > 1e-9:
        raise ValueError("XZ beam endpoints must keep y fixed.")
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    angle_y = math.atan2(dx, dz)
    part.visual(
        Box((thickness[0], thickness[1], length)),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, y0, (z0 + z1) * 0.5),
            rpy=(0.0, angle_y, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_cylinder_beam_yz(
    part,
    *,
    x: float,
    radius: float,
    y0: float,
    z0: float,
    y1: float,
    z1: float,
    material,
    name: str,
) -> None:
    dy = y1 - y0
    dz = z1 - z0
    length = math.hypot(dy, dz)
    roll = math.atan2(-dy, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(x, 0.5 * (y0 + y1), 0.5 * (z0 + z1)),
            rpy=(roll, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_cylinder_beam_x(
    part,
    *,
    y: float,
    z: float,
    length: float,
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cabin_geometry(part, *, shell, frame, glass, accent) -> None:
    part.visual(
        Box((0.96, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
        material=frame,
        name="hanger_saddle",
    )
    for side, x in (("left", -0.43), ("right", 0.43)):
        part.visual(
            Box((0.10, 0.12, 0.57)),
            origin=Origin(xyz=(x, 0.0, -0.375)),
            material=frame,
            name=f"{side}_hanger_strap",
        )

    part.visual(
        Box((1.56, 1.26, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, -0.78)),
        material=shell,
        name="roof",
    )
    part.visual(
        Box((1.48, 1.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, -2.04)),
        material=shell,
        name="floor",
    )
    part.visual(
        Box((1.52, 1.22, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, -1.90)),
        material=accent,
        name="lower_skirt",
    )

    for side, x in (("left", -0.70), ("right", 0.70)):
        for fore_aft, y in (("front", -0.54), ("rear", 0.54)):
            part.visual(
                Box((0.10, 0.10, 1.36)),
                origin=Origin(xyz=(x, y, -1.39)),
                material=frame,
                name=f"{side}_{fore_aft}_post",
            )

    part.visual(
        Box((1.34, 0.05, 0.96)),
        origin=Origin(xyz=(0.0, -0.59, -1.39)),
        material=glass,
        name="front_glass",
    )
    part.visual(
        Box((1.34, 0.05, 0.96)),
        origin=Origin(xyz=(0.0, 0.59, -1.39)),
        material=glass,
        name="rear_glass",
    )
    part.visual(
        Box((0.05, 1.08, 0.96)),
        origin=Origin(xyz=(-0.74, 0.0, -1.39)),
        material=glass,
        name="left_glass",
    )
    part.visual(
        Box((0.05, 1.08, 0.96)),
        origin=Origin(xyz=(0.74, 0.0, -1.39)),
        material=glass,
        name="right_glass",
    )
    part.visual(
        Box((1.40, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, -0.56, -0.92)),
        material=accent,
        name="front_header",
    )
    part.visual(
        Box((1.40, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.56, -0.92)),
        material=accent,
        name="rear_header",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="waterfront_observation_wheel")

    steel = model.material("steel", rgba=(0.86, 0.88, 0.90, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.39, 0.45, 1.0))
    deck = model.material("deck", rgba=(0.52, 0.56, 0.60, 1.0))
    station_shell = model.material("station_shell", rgba=(0.82, 0.85, 0.88, 1.0))
    cabin_shell = model.material("cabin_shell", rgba=(0.95, 0.97, 0.98, 1.0))
    cabin_glass = model.material("cabin_glass", rgba=(0.23, 0.36, 0.44, 1.0))
    cabin_accent = model.material("cabin_accent", rgba=(0.14, 0.55, 0.63, 1.0))

    station = model.part("station")
    station.visual(
        Box((18.0, 12.0, 1.0)),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        material=deck,
        name="pier_deck",
    )
    station.visual(
        Box((8.6, 2.8, 2.4)),
        origin=Origin(xyz=(0.0, -4.2, 2.2)),
        material=station_shell,
        name="boarding_platform",
    )
    station.visual(
        Box((9.0, 3.0, 0.18)),
        origin=Origin(xyz=(0.0, -4.2, 3.49)),
        material=dark_steel,
        name="platform_roof",
    )
    station.visual(
        Box((4.8, 1.6, 0.30)),
        origin=Origin(xyz=(0.0, -1.9, 1.15)),
        material=deck,
        name="boarding_apron",
    )

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        _add_box_beam_xz(
            station,
            start=(sign * 4.85, -3.05, 0.70),
            end=(sign * 1.60, -3.05, AXLE_HEIGHT - 0.10),
            thickness=(0.78, 0.88),
            material=steel,
            name=f"{side_name}_front_leg",
        )
        _add_box_beam_xz(
            station,
            start=(sign * 4.85, 3.05, 0.70),
            end=(sign * 1.60, 3.05, AXLE_HEIGHT - 0.10),
            thickness=(0.78, 0.88),
            material=steel,
            name=f"{side_name}_rear_leg",
        )
        station.visual(
            Box((1.10, 6.80, 0.50)),
            origin=Origin(xyz=(sign * 4.55, 0.0, 1.05)),
            material=dark_steel,
            name=f"{side_name}_base_tie",
        )
        station.visual(
            Box((0.82, 6.90, 1.45)),
            origin=Origin(xyz=(sign * 1.55, 0.0, AXLE_HEIGHT - 0.45)),
            material=steel,
            name=f"{side_name}_crown",
        )
        station.visual(
            Box((0.46, 1.05, 0.82)),
            origin=Origin(xyz=(sign * 1.48, 0.0, AXLE_HEIGHT)),
            material=dark_steel,
            name=f"{side_name}_bearing_pod",
        )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.58, length=2.05),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.22, length=2.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_sleeve",
    )
    for side_name, x in (("left", -WHEEL_HALF_WIDTH), ("right", WHEEL_HALF_WIDTH)):
        outer_rim = tube_from_spline_points(
            _circle_points_yz(WHEEL_RADIUS, x),
            radius=0.16,
            samples_per_segment=4,
            closed_spline=True,
            radial_segments=14,
            cap_ends=False,
        )
        wheel.visual(
            mesh_from_geometry(outer_rim, f"{side_name}_outer_rim"),
            material=steel,
            name=f"{side_name}_outer_rim",
        )
        inner_rim = tube_from_spline_points(
            _circle_points_yz(INNER_RIM_RADIUS, x),
            radius=0.10,
            samples_per_segment=4,
            closed_spline=True,
            radial_segments=12,
            cap_ends=False,
        )
        wheel.visual(
            mesh_from_geometry(inner_rim, f"{side_name}_inner_rim"),
            material=dark_steel,
            name=f"{side_name}_inner_rim",
        )

    spoke_count = CABIN_COUNT * 2
    for i in range(spoke_count):
        angle = i * math.tau / spoke_count
        y0 = 0.58 * math.cos(angle)
        z0 = 0.58 * math.sin(angle)
        y1 = (INNER_RIM_RADIUS + 0.10) * math.cos(angle)
        z1 = (INNER_RIM_RADIUS + 0.10) * math.sin(angle)
        for side_name, x in (("left", -WHEEL_HALF_WIDTH), ("right", WHEEL_HALF_WIDTH)):
            _add_cylinder_beam_yz(
                wheel,
                x=x,
                radius=0.06,
                y0=y0,
                z0=z0,
                y1=y1,
                z1=z1,
                material=steel,
                name=f"{side_name}_spoke_{i}",
            )

    for i in range(CABIN_COUNT):
        angle = -math.pi / 2.0 + i * math.tau / CABIN_COUNT
        y0 = (INNER_RIM_RADIUS - 0.05) * math.cos(angle)
        z0 = (INNER_RIM_RADIUS - 0.05) * math.sin(angle)
        y1 = (WHEEL_RADIUS + 0.10) * math.cos(angle)
        z1 = (WHEEL_RADIUS + 0.10) * math.sin(angle)
        for side_name, x in (("left", -WHEEL_HALF_WIDTH), ("right", WHEEL_HALF_WIDTH)):
            _add_cylinder_beam_yz(
                wheel,
                x=x,
                radius=0.08,
                y0=y0,
                z0=z0,
                y1=y1,
                z1=z1,
                material=dark_steel,
                name=f"{side_name}_hanger_strut_{i}",
            )
        y_pivot = PIVOT_RADIUS * math.cos(angle)
        z_pivot = PIVOT_RADIUS * math.sin(angle)
        _add_cylinder_beam_x(
            wheel,
            y=y_pivot,
            z=z_pivot,
            length=2.28,
            radius=0.09,
            material=dark_steel,
            name=f"hanger_crossbeam_{i}",
        )

        cabin = model.part(f"cabin_{i}")
        _add_cabin_geometry(
            cabin,
            shell=cabin_shell,
            frame=dark_steel,
            glass=cabin_glass,
            accent=cabin_accent,
        )
        model.articulation(
            f"wheel_to_cabin_{i}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=cabin,
            origin=Origin(xyz=(0.0, y_pivot, z_pivot)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=1.5),
        )

    model.articulation(
        "station_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=station,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.35),
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

    station = object_model.get_part("station")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("station_to_wheel")
    cabin_0 = object_model.get_part("cabin_0")
    cabin_0_hanger = object_model.get_articulation("wheel_to_cabin_0")

    cabin_parts = [part for part in object_model.parts if part.name.startswith("cabin_")]
    cabin_joints = [joint for joint in object_model.articulations if joint.name.startswith("wheel_to_cabin_")]

    wheel_limits = wheel_spin.motion_limits
    ctx.check(
        "wheel uses a continuous axle articulation",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin.axis == (1.0, 0.0, 0.0)
        and wheel_limits is not None
        and wheel_limits.lower is None
        and wheel_limits.upper is None,
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}, limits={wheel_limits}",
    )
    cabin_limits = cabin_0_hanger.motion_limits
    ctx.check(
        "cabins use pivoting continuous hanger joints",
        len(cabin_parts) == CABIN_COUNT
        and len(cabin_joints) == CABIN_COUNT
        and cabin_0_hanger.articulation_type == ArticulationType.CONTINUOUS
        and cabin_0_hanger.axis == (1.0, 0.0, 0.0)
        and cabin_limits is not None
        and cabin_limits.lower is None
        and cabin_limits.upper is None,
        details=f"parts={len(cabin_parts)}, joints={len(cabin_joints)}, type={cabin_0_hanger.articulation_type}, axis={cabin_0_hanger.axis}, limits={cabin_limits}",
    )

    pivot_rest = ctx.part_world_position(cabin_0)
    floor_rest = ctx.part_element_world_aabb(cabin_0, elem="floor")
    ctx.check(
        "bottom cabin hangs below its rim pivot at rest",
        pivot_rest is not None
        and floor_rest is not None
        and floor_rest[1][2] < pivot_rest[2] - 0.8,
        details=f"pivot={pivot_rest}, floor_aabb={floor_rest}",
    )

    wheel_center_rest = ctx.part_world_position(wheel)
    with ctx.pose({wheel_spin: math.pi / 2.0, cabin_0_hanger: -math.pi / 2.0}):
        pivot_turned = ctx.part_world_position(cabin_0)
        floor_turned = ctx.part_element_world_aabb(cabin_0, elem="floor")
        wheel_center_turned = ctx.part_world_position(wheel)
        ctx.check(
            "wheel rotation carries a cabin around the rim",
            pivot_rest is not None
            and pivot_turned is not None
            and wheel_center_rest is not None
            and wheel_center_turned is not None
            and pivot_turned[1] > pivot_rest[1] + 10.0
            and abs(wheel_center_rest[2] - wheel_center_turned[2]) < 1e-6,
            details=f"rest={pivot_rest}, turned={pivot_turned}, wheel_rest={wheel_center_rest}, wheel_turned={wheel_center_turned}",
        )
        ctx.check(
            "cabin counter-rotation keeps the cabin body below its pivot",
            pivot_turned is not None
            and floor_turned is not None
            and floor_turned[1][2] < pivot_turned[2] - 0.8,
            details=f"pivot={pivot_turned}, floor_aabb={floor_turned}",
        )

    ctx.check(
        "station remains the single structural root",
        len(object_model.root_parts()) == 1 and object_model.root_parts()[0].name == station.name,
        details=f"roots={[part.name for part in object_model.root_parts()]}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
