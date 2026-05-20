from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


AXIS_HEIGHT = 0.22


def _tube_mesh(name: str, length: float, outer_radius: float, inner_radius: float):
    """Thin-walled guide tube centered on local X."""
    shell = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -0.5 * length),
            (outer_radius, 0.5 * length),
        ],
        [
            (inner_radius, -0.5 * length),
            (inner_radius, 0.5 * length),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, name)


def _cone_mesh(name: str, radius: float, length: float):
    return mesh_from_geometry(
        ConeGeometry(radius, length, radial_segments=48, closed=True).rotate_y(math.pi / 2.0),
        name,
    )


def _x_cylinder(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _origin_x(x: float, *, y: float = 0.0, z_offset: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, AXIS_HEIGHT + z_offset), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ejector_plunger_stack")

    base_paint = model.material("base_paint", rgba=(0.16, 0.20, 0.25, 1.0))
    guide_dark = model.material("guide_dark", rgba=(0.23, 0.25, 0.27, 1.0))
    guide_mid = model.material("guide_mid", rgba=(0.48, 0.51, 0.54, 1.0))
    guide_light = model.material("guide_light", rgba=(0.68, 0.70, 0.72, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.84, 1.0))
    screw_black = model.material("black_screws", rgba=(0.05, 0.055, 0.06, 1.0))
    wear_band = model.material("bronze_bushing", rgba=(0.75, 0.56, 0.28, 1.0))

    # Root: the grounded base, support saddles, and the largest fixed guide.
    base = model.part("base")
    base.visual(
        Box((0.96, 0.24, 0.08)),
        origin=Origin(xyz=(0.22, 0.0, 0.04)),
        material=base_paint,
        name="ground_block",
    )
    base.visual(
        Box((0.84, 0.08, 0.035)),
        origin=Origin(xyz=(0.25, 0.0, 0.097)),
        material=base_paint,
        name="raised_rail",
    )
    base.visual(
        _tube_mesh("rear_guide_shell", 0.36, 0.058, 0.044),
        origin=Origin(xyz=(0.0, 0.0, AXIS_HEIGHT)),
        material=guide_dark,
        name="rear_guide_shell",
    )
    for x in (-0.12, 0.12):
        base.visual(
            Box((0.055, 0.085, 0.086)),
            origin=Origin(xyz=(x, 0.0, 0.122)),
            material=base_paint,
            name=f"saddle_web_{'rear' if x < 0 else 'front'}",
        )
        for y in (-0.060, 0.060):
            base.visual(
                Box((0.050, 0.028, 0.170)),
                origin=Origin(xyz=(x, y, 0.166)),
                material=base_paint,
                name=f"saddle_ear_{'rear' if x < 0 else 'front'}_{'neg' if y < 0 else 'pos'}",
            )
    for x in (-0.16, 0.17):
        base.visual(
            _tube_mesh(f"rear_clamp_ring_{x:+.2f}", 0.030, 0.069, 0.056),
            origin=Origin(xyz=(x, 0.0, AXIS_HEIGHT)),
            material=guide_mid,
            name=f"rear_clamp_{'inlet' if x < 0 else 'outlet'}",
        )
        for y in (-0.073, 0.073):
            base.visual(
                Box((0.032, 0.020, 0.030)),
                origin=Origin(xyz=(x, y * 0.88, AXIS_HEIGHT + 0.040)),
                material=guide_mid,
                name=f"rear_clamp_lug_{'neg' if y < 0 else 'pos'}_{'in' if x < 0 else 'out'}",
            )
            base.visual(
                Cylinder(radius=0.0065, length=0.018),
                origin=Origin(
                    xyz=(x, y * 0.88, AXIS_HEIGHT + 0.056),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=screw_black,
                name=f"rear_shoulder_screw_{'neg' if y < 0 else 'pos'}_{'in' if x < 0 else 'out'}",
            )
    base.visual(
        _tube_mesh("rear_end_bushing", 0.018, 0.049, 0.038),
        origin=Origin(xyz=(-0.188, 0.0, AXIS_HEIGHT)),
        material=wear_band,
        name="rear_end_bushing",
    )

    # First moving stage: a large rod and the next guide sleeve nested into the
    # fixed rear guide at the collapsed pose.
    rear_rod = model.part("rear_rod")
    rear_rod.visual(
        _x_cylinder(0.035, 0.330),
        origin=Origin(xyz=(0.135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="rear_rod_body",
    )
    rear_rod.visual(
        Box((0.180, 0.014, 0.010)),
        origin=Origin(xyz=(0.140, 0.0, -0.040)),
        material=wear_band,
        name="rear_wear_shoe",
    )
    rear_rod.visual(
        _tube_mesh("middle_guide_shell", 0.320, 0.040, 0.030),
        origin=Origin(xyz=(0.490, 0.0, 0.0)),
        material=guide_mid,
        name="middle_guide_shell",
    )
    rear_rod.visual(
        _tube_mesh("middle_rear_bushing", 0.020, 0.034, 0.028),
        origin=Origin(xyz=(0.334, 0.0, 0.0)),
        material=wear_band,
        name="middle_rear_bushing",
    )
    for x in (0.405, 0.635):
        rear_rod.visual(
            _tube_mesh(f"middle_clamp_ring_{x:.2f}", 0.026, 0.047, 0.039),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=guide_light,
            name=f"middle_clamp_{'rear' if x < 0.5 else 'front'}",
        )
        for y in (-0.052, 0.052):
            rear_rod.visual(
                Box((0.026, 0.016, 0.026)),
                origin=Origin(xyz=(x, y * 0.83, 0.030)),
                material=guide_light,
                name=f"middle_lug_{'neg' if y < 0 else 'pos'}_{'rear' if x < 0.5 else 'front'}",
            )
            rear_rod.visual(
                Cylinder(radius=0.005, length=0.014),
                origin=Origin(xyz=(x, y * 0.83, 0.044), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=screw_black,
                name=f"middle_screw_{'neg' if y < 0 else 'pos'}_{'rear' if x < 0.5 else 'front'}",
            )
    for y in (-0.034, 0.034):
        rear_rod.visual(
            Box((0.075, 0.010, 0.012)),
            origin=Origin(xyz=(0.300, y, 0.0)),
            material=polished,
            name=f"rear_bridge_{'neg' if y < 0 else 'pos'}",
        )

    # Second moving stage: a smaller rod and the front guide sleeve.
    middle_rod = model.part("middle_rod")
    middle_rod.visual(
        _x_cylinder(0.025, 0.270),
        origin=Origin(xyz=(0.115, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="middle_rod_body",
    )
    middle_rod.visual(
        _tube_mesh("front_guide_shell", 0.260, 0.029, 0.0205),
        origin=Origin(xyz=(0.410, 0.0, 0.0)),
        material=guide_light,
        name="front_guide_shell",
    )
    middle_rod.visual(
        _tube_mesh("front_rear_bushing", 0.018, 0.024, 0.019),
        origin=Origin(xyz=(0.286, 0.0, 0.0)),
        material=wear_band,
        name="front_rear_bushing",
    )
    for x in (0.300, 0.520):
        middle_rod.visual(
            _tube_mesh(f"front_clamp_ring_{x:.2f}", 0.022, 0.035, 0.028),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=guide_mid,
            name=f"front_clamp_{'rear' if x < 0.4 else 'front'}",
        )
        for y in (-0.040, 0.040):
            middle_rod.visual(
                Cylinder(radius=0.004, length=0.012),
                origin=Origin(xyz=(x, y * 0.70, 0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=screw_black,
                name=f"front_screw_{'neg' if y < 0 else 'pos'}_{'rear' if x < 0.4 else 'front'}",
            )
    for y in (-0.026, 0.026):
        middle_rod.visual(
            Box((0.072, 0.006, 0.010)),
            origin=Origin(xyz=(0.252, y * 0.77, 0.0)),
            material=polished,
            name=f"middle_bridge_{'neg' if y < 0 else 'pos'}",
        )

    # Final moving stage: a slim rod and blunt ejector tip.
    front_rod = model.part("front_rod")
    front_rod.visual(
        _x_cylinder(0.016, 0.345),
        origin=Origin(xyz=(0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="front_rod_body",
    )
    front_rod.visual(
        _cone_mesh("front_tip_mesh", 0.017, 0.060),
        origin=Origin(xyz=(0.347, 0.0, 0.0)),
        material=polished,
        name="front_tip",
    )
    front_rod.visual(
        _tube_mesh("front_wear_collar", 0.016, 0.019, 0.0150),
        origin=Origin(xyz=(-0.018, 0.0, 0.0)),
        material=wear_band,
        name="front_wear_collar",
    )

    model.articulation(
        "base_to_rear",
        ArticulationType.PRISMATIC,
        parent=base,
        child=rear_rod,
        origin=Origin(xyz=(-0.180, 0.0, AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.100),
    )
    model.articulation(
        "rear_to_middle",
        ArticulationType.PRISMATIC,
        parent=rear_rod,
        child=middle_rod,
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.22, lower=0.0, upper=0.080),
    )
    model.articulation(
        "middle_to_front",
        ArticulationType.PRISMATIC,
        parent=middle_rod,
        child=front_rod,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.20, lower=0.0, upper=0.070),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rear_rod = object_model.get_part("rear_rod")
    middle_rod = object_model.get_part("middle_rod")
    front_rod = object_model.get_part("front_rod")
    base_to_rear = object_model.get_articulation("base_to_rear")
    rear_to_middle = object_model.get_articulation("rear_to_middle")
    middle_to_front = object_model.get_articulation("middle_to_front")

    ctx.allow_overlap(
        base,
        rear_rod,
        elem_a="rear_guide_shell",
        elem_b="rear_wear_shoe",
        reason=(
            "The bronze wear shoe is intentionally modeled with a small local "
            "preload into the fixed guide bore so the sliding plunger stack is "
            "mechanically grounded rather than floating."
        ),
    )
    ctx.expect_within(
        rear_rod,
        base,
        axes="yz",
        inner_elem="rear_wear_shoe",
        outer_elem="rear_guide_shell",
        margin=0.001,
        name="preloaded rear wear shoe sits inside guide bore",
    )
    ctx.expect_overlap(
        rear_rod,
        base,
        axes="x",
        elem_a="rear_wear_shoe",
        elem_b="rear_guide_shell",
        min_overlap=0.150,
        name="rear wear shoe runs along the fixed guide",
    )

    ctx.expect_within(
        rear_rod,
        base,
        axes="yz",
        inner_elem="rear_rod_body",
        outer_elem="rear_guide_shell",
        margin=0.010,
        name="rear rod remains centered in rear guide",
    )
    ctx.expect_overlap(
        rear_rod,
        base,
        axes="x",
        elem_a="rear_rod_body",
        elem_b="rear_guide_shell",
        min_overlap=0.180,
        name="rear rod retained in fixed guide",
    )
    ctx.expect_within(
        middle_rod,
        rear_rod,
        axes="yz",
        inner_elem="middle_rod_body",
        outer_elem="middle_guide_shell",
        margin=0.006,
        name="middle rod remains centered in middle guide",
    )
    ctx.expect_overlap(
        middle_rod,
        rear_rod,
        axes="x",
        elem_a="middle_rod_body",
        elem_b="middle_guide_shell",
        min_overlap=0.220,
        name="middle rod retained in moving guide",
    )
    ctx.expect_within(
        front_rod,
        middle_rod,
        axes="yz",
        inner_elem="front_rod_body",
        outer_elem="front_guide_shell",
        margin=0.004,
        name="front rod remains centered in front guide",
    )
    ctx.expect_overlap(
        front_rod,
        middle_rod,
        axes="x",
        elem_a="front_rod_body",
        elem_b="front_guide_shell",
        min_overlap=0.230,
        name="front rod retained in smallest guide",
    )

    rest_front = ctx.part_world_position(front_rod)
    with ctx.pose({base_to_rear: 0.100, rear_to_middle: 0.080, middle_to_front: 0.070}):
        ctx.expect_within(
            rear_rod,
            base,
            axes="yz",
            inner_elem="rear_rod_body",
            outer_elem="rear_guide_shell",
            margin=0.010,
            name="extended rear rod stays centered",
        )
        ctx.expect_overlap(
            rear_rod,
            base,
            axes="x",
            elem_a="rear_wear_shoe",
            elem_b="rear_guide_shell",
            min_overlap=0.150,
            name="extended wear shoe stays in guide",
        )
        ctx.expect_overlap(
            rear_rod,
            base,
            axes="x",
            elem_a="rear_rod_body",
            elem_b="rear_guide_shell",
            min_overlap=0.140,
            name="extended rear stage keeps insertion",
        )
        ctx.expect_within(
            middle_rod,
            rear_rod,
            axes="yz",
            inner_elem="middle_rod_body",
            outer_elem="middle_guide_shell",
            margin=0.006,
            name="extended middle rod stays centered",
        )
        ctx.expect_overlap(
            middle_rod,
            rear_rod,
            axes="x",
            elem_a="middle_rod_body",
            elem_b="middle_guide_shell",
            min_overlap=0.190,
            name="extended middle stage keeps insertion",
        )
        ctx.expect_within(
            front_rod,
            middle_rod,
            axes="yz",
            inner_elem="front_rod_body",
            outer_elem="front_guide_shell",
            margin=0.004,
            name="extended front rod stays centered",
        )
        ctx.expect_overlap(
            front_rod,
            middle_rod,
            axes="x",
            elem_a="front_rod_body",
            elem_b="front_guide_shell",
            min_overlap=0.180,
            name="extended front stage keeps insertion",
        )
        extended_front = ctx.part_world_position(front_rod)

    ctx.check(
        "serial plunger translates toward the tip",
        rest_front is not None
        and extended_front is not None
        and extended_front[0] > rest_front[0] + 0.230,
        details=f"rest={rest_front}, extended={extended_front}",
    )

    return ctx.report()


object_model = build_object_model()
