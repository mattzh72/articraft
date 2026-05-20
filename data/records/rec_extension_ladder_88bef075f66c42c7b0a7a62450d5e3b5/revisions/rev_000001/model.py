from __future__ import annotations

from math import atan2, pi, sqrt

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


RAIL_LENGTH = 3.8
UPPER_RAIL_Y = 0.02
UPPER_RAIL_Z = 2.55
UPPER_RAIL_RADIUS = 0.040
TRACK_Y = 0.92
TRACK_Z = 0.045
TRACK_RADIUS = 0.020

LADDER_WIDTH = 0.62
STILE_X = 0.31
STILE_BOTTOM = (TRACK_Y, 0.255)
STILE_TOP = (0.18, 2.43)
STILE_DY = STILE_TOP[0] - STILE_BOTTOM[0]
STILE_DZ = STILE_TOP[1] - STILE_BOTTOM[1]
STILE_LENGTH = sqrt(STILE_DY * STILE_DY + STILE_DZ * STILE_DZ)
STILE_LEAN = atan2(-STILE_DY, STILE_DZ)

UPPER_WHEEL_RADIUS = 0.065
UPPER_WHEEL_WIDTH = 0.035
UPPER_WHEEL_Y = UPPER_RAIL_Y
UPPER_WHEEL_Z = UPPER_RAIL_Z + UPPER_RAIL_RADIUS + UPPER_WHEEL_RADIUS

BASE_WHEEL_RADIUS = 0.055
BASE_WHEEL_WIDTH = 0.035
BASE_WHEEL_Y = TRACK_Y
BASE_WHEEL_Z = TRACK_Z + TRACK_RADIUS + BASE_WHEEL_RADIUS


def _cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder descriptor and transform for a cylinder running along world X."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, pi / 2.0, 0.0))


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder descriptor and transform for a cylinder running along world Y."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(-pi / 2.0, 0.0, 0.0))


def _point_on_stile(t: float) -> tuple[float, float]:
    y = STILE_BOTTOM[0] + (STILE_TOP[0] - STILE_BOTTOM[0]) * t
    z = STILE_BOTTOM[1] + (STILE_TOP[1] - STILE_BOTTOM[1]) * t
    return y, z


def _add_roller_visuals(part, *, radius: float, width: float, body_material: str, hub_material: str) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=spin_origin,
        material=body_material,
        name="roller",
    )
    for side, x in enumerate((-width * 0.43, width * 0.43)):
        part.visual(
            Cylinder(radius=radius * 0.92, length=width * 0.10),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=spin_origin.rpy),
            material=hub_material,
            name=f"side_cap_{side}",
        )
    part.visual(
        Cylinder(radius=radius * 0.34, length=width * 1.05),
        origin=spin_origin,
        material=hub_material,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_library_ladder")

    model.material("warm_wood", rgba=(0.58, 0.34, 0.16, 1.0))
    model.material("dark_wood", rgba=(0.30, 0.17, 0.08, 1.0))
    model.material("brass", rgba=(0.78, 0.60, 0.25, 1.0))
    model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("black_rubber", rgba=(0.035, 0.035, 0.035, 1.0))
    model.material("wall_paint", rgba=(0.86, 0.82, 0.73, 1.0))
    model.material("floor_oak", rgba=(0.45, 0.29, 0.15, 1.0))

    rail = model.part("library_rail")
    rail.visual(
        Box((RAIL_LENGTH, 0.040, 2.72)),
        origin=Origin(xyz=(0.0, -0.070, 1.36)),
        material="wall_paint",
        name="wall_backer",
    )
    rail.visual(
        Box((RAIL_LENGTH, 1.10, 0.035)),
        origin=Origin(xyz=(0.0, 0.47, 0.0175)),
        material="floor_oak",
        name="floor_strip",
    )
    upper_cyl, upper_origin = _cyl_x(UPPER_RAIL_RADIUS, RAIL_LENGTH)
    rail.visual(
        upper_cyl,
        origin=Origin(xyz=(0.0, UPPER_RAIL_Y, UPPER_RAIL_Z), rpy=upper_origin.rpy),
        material="brass",
        name="upper_wall_rail",
    )
    track_cyl, track_origin = _cyl_x(TRACK_RADIUS, RAIL_LENGTH)
    rail.visual(
        track_cyl,
        origin=Origin(xyz=(0.0, TRACK_Y, TRACK_Z), rpy=track_origin.rpy),
        material="dark_steel",
        name="floor_track_rail",
    )
    rail.visual(
        Box((RAIL_LENGTH, 0.11, 0.018)),
        origin=Origin(xyz=(0.0, TRACK_Y, 0.026)),
        material="dark_steel",
        name="floor_track_plate",
    )
    for idx, x in enumerate((-1.45, 0.0, 1.45)):
        rail.visual(
            Box((0.055, 0.105, 0.090)),
            origin=Origin(xyz=(x, -0.020, UPPER_RAIL_Z)),
            material="brass",
            name=f"rail_bracket_{idx}",
        )
        rail.visual(
            Box((0.090, 0.018, 0.160)),
            origin=Origin(xyz=(x, -0.075, UPPER_RAIL_Z - 0.020)),
            material="brass",
            name=f"wall_flange_{idx}",
        )

    ladder = model.part("stile_frame")
    stile_center_y = (STILE_BOTTOM[0] + STILE_TOP[0]) * 0.5
    stile_center_z = (STILE_BOTTOM[1] + STILE_TOP[1]) * 0.5
    for side, x in enumerate((-STILE_X, STILE_X)):
        ladder.visual(
            Box((0.060, 0.048, STILE_LENGTH + 0.06)),
            origin=Origin(xyz=(x, stile_center_y, stile_center_z), rpy=(STILE_LEAN, 0.0, 0.0)),
            material="warm_wood",
            name=f"stile_{side}",
        )
        ladder.visual(
            Box((0.094, 0.080, 0.035)),
            origin=Origin(xyz=(x, BASE_WHEEL_Y, 0.205)),
            material="dark_steel",
            name=f"base_shoe_{side}",
        )
        for fork_side, yoff in enumerate((-0.026, 0.026)):
            ladder.visual(
                Box((0.074, 0.007, 0.135)),
                origin=Origin(xyz=(x, BASE_WHEEL_Y + yoff, BASE_WHEEL_Z + 0.006)),
                material="dark_steel",
                name=f"base_fork_{side}_{fork_side}",
            )
        ladder.visual(
            Box((0.070, 0.036, 0.285)),
            origin=Origin(xyz=(x, 0.130, 2.520), rpy=(0.10, 0.0, 0.0)),
            material="brass",
            name=f"upper_strap_{side}",
        )
        for fork_side, yoff in enumerate((-0.026, 0.026)):
            ladder.visual(
                Box((0.074, 0.007, 0.180)),
                origin=Origin(xyz=(x, UPPER_WHEEL_Y + yoff, UPPER_WHEEL_Z + 0.040)),
                material="brass",
                name=f"upper_fork_{side}_{fork_side}",
            )
        ladder.visual(
            Box((0.076, 0.160, 0.022)),
            origin=Origin(xyz=(x, 0.050, UPPER_WHEEL_Z + 0.110)),
            material="brass",
            name=f"upper_yoke_top_{side}",
        )
        ladder.visual(
            Box((0.058, 0.030, 0.150)),
            origin=Origin(xyz=(x, 0.108, UPPER_WHEEL_Z + 0.035)),
            material="brass",
            name=f"upper_yoke_web_{side}",
        )
        hook = tube_from_spline_points(
            [
                (x, 0.205, 2.390),
                (x, 0.115, 2.560),
                (x, 0.110, UPPER_WHEEL_Z + 0.080),
                (x, -0.030, UPPER_WHEEL_Z + 0.080),
                (x, -0.030, 2.600),
            ],
            radius=0.012,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        )
        ladder.visual(mesh_from_geometry(hook, f"hook_loop_{side}"), material="brass", name=f"hook_loop_{side}")

    top_cyl, top_origin = _cyl_x(0.023, LADDER_WIDTH + 0.12)
    ladder.visual(
        top_cyl,
        origin=Origin(xyz=(0.0, 0.230, 2.365), rpy=top_origin.rpy),
        material="dark_wood",
        name="top_crossbar",
    )
    lower_cyl, lower_origin = _cyl_x(0.022, LADDER_WIDTH + 0.10)
    ladder.visual(
        lower_cyl,
        origin=Origin(xyz=(0.0, 0.825, 0.420), rpy=lower_origin.rpy),
        material="dark_wood",
        name="lower_crossbar",
    )
    for idx, t in enumerate((0.17, 0.31, 0.45, 0.59, 0.73, 0.87)):
        y, z = _point_on_stile(t)
        ladder.visual(
            Box((LADDER_WIDTH + 0.10, 0.175, 0.038)),
            origin=Origin(xyz=(0.0, y + 0.020, z)),
            material="warm_wood",
            name=f"tread_{idx}",
        )
        nosing_cyl, nosing_origin = _cyl_x(0.019, LADDER_WIDTH + 0.12)
        ladder.visual(
            nosing_cyl,
            origin=Origin(xyz=(0.0, y + 0.100, z + 0.004), rpy=nosing_origin.rpy),
            material="dark_wood",
            name=f"tread_nosing_{idx}",
        )

    roll_joint = model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=ladder,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.40, lower=-1.25, upper=1.25),
    )
    roll_joint.meta["description"] = "The entire leaned ladder rolls horizontally along the wall rail and floor guide track."

    for idx, x in enumerate((-STILE_X, STILE_X)):
        upper_wheel = model.part(f"upper_wheel_{idx}")
        _add_roller_visuals(
            upper_wheel,
            radius=UPPER_WHEEL_RADIUS,
            width=UPPER_WHEEL_WIDTH,
            body_material="brass",
            hub_material="dark_steel",
        )
        model.articulation(
            f"upper_wheel_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=ladder,
            child=upper_wheel,
            origin=Origin(xyz=(x, UPPER_WHEEL_Y, UPPER_WHEEL_Z), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=18.0),
        )

        base_wheel = model.part(f"base_wheel_{idx}")
        _add_roller_visuals(
            base_wheel,
            radius=BASE_WHEEL_RADIUS,
            width=BASE_WHEEL_WIDTH,
            body_material="black_rubber",
            hub_material="dark_steel",
        )
        model.articulation(
            f"base_wheel_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=ladder,
            child=base_wheel,
            origin=Origin(xyz=(x, BASE_WHEEL_Y, BASE_WHEEL_Z), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=20.0),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    rail = object_model.get_part("library_rail")
    ladder = object_model.get_part("stile_frame")
    slide = object_model.get_articulation("rail_slide")
    upper_wheel = object_model.get_part("upper_wheel_0")
    base_wheel = object_model.get_part("base_wheel_0")

    ctx.expect_gap(
        upper_wheel,
        rail,
        axis="z",
        max_gap=0.008,
        max_penetration=0.001,
        positive_elem="roller",
        negative_elem="upper_wall_rail",
        name="upper hook wheel rides on the wall rail",
    )
    ctx.expect_gap(
        base_wheel,
        rail,
        axis="z",
        max_gap=0.008,
        max_penetration=0.001,
        positive_elem="roller",
        negative_elem="floor_track_rail",
        name="base wheel rides on the floor guide rail",
    )
    ctx.expect_overlap(
        upper_wheel,
        rail,
        axes="x",
        min_overlap=0.03,
        elem_a="roller",
        elem_b="upper_wall_rail",
        name="upper wheel is laterally over the long rail",
    )
    ctx.expect_overlap(
        base_wheel,
        rail,
        axes="x",
        min_overlap=0.03,
        elem_a="roller",
        elem_b="floor_track_rail",
        name="base wheel is laterally over the floor track",
    )

    upper_pos = ctx.part_world_position(upper_wheel)
    base_pos = ctx.part_world_position(base_wheel)
    ctx.check(
        "stile assembly leans from wall rail out to floor track",
        upper_pos is not None
        and base_pos is not None
        and upper_pos[1] < base_pos[1] - 0.65
        and upper_pos[2] > base_pos[2] + 2.40,
        details=f"upper={upper_pos}, base={base_pos}",
    )

    rest = ctx.part_world_position(ladder)
    with ctx.pose({slide: 0.75}):
        shifted = ctx.part_world_position(ladder)
    ctx.check(
        "rail slide moves the whole ladder along the rails",
        rest is not None and shifted is not None and shifted[0] > rest[0] + 0.70,
        details=f"rest={rest}, shifted={shifted}",
    )

    return ctx.report()


object_model = build_object_model()
