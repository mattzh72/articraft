from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BOOM_REST_ANGLE = math.radians(18.0)
BOOM_LENGTH = 6.55
BOOM_TIP_Z = -0.08


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    """Orient a cylinder's local +Z axis along segment a -> b."""

    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
    material,
    *,
    name: str | None = None,
    overlap: float = 0.015,
) -> None:
    """Add a round structural member with a tiny endpoint overrun for welded joints."""

    length = _distance(a, b)
    part.visual(
        Cylinder(radius=radius, length=length + overlap * 2.0),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_lattice_boom(part, material, cable_material, steel_material) -> None:
    """Triangular luffing boom with welded tubular chords and diagonals."""

    panels = 8
    x0 = 0.22
    xs = [x0 + (BOOM_LENGTH - x0) * i / panels for i in range(panels + 1)]

    def top_z(x: float) -> float:
        t = (x - x0) / (BOOM_LENGTH - x0)
        return 0.62 + (0.20 - 0.62) * t

    half_widths = [0.20 + (0.12 - 0.20) * (x - x0) / (BOOM_LENGTH - x0) for x in xs]
    lower_left = [(x, -w, 0.0) for x, w in zip(xs, half_widths)]
    lower_right = [(x, w, 0.0) for x, w in zip(xs, half_widths)]
    upper = [(x, 0.0, top_z(x)) for x in xs]

    # Root hinge knuckle and short boxed transition into the truss.
    part.visual(
        Cylinder(radius=0.16, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_material,
        name="boom_hub",
    )
    _add_member(part, (0.0, -0.20, 0.0), lower_left[0], 0.055, steel_material)
    _add_member(part, (0.0, 0.20, 0.0), lower_right[0], 0.055, steel_material)
    _add_member(part, (0.04, 0.0, 0.13), upper[0], 0.045, steel_material)
    _add_member(part, lower_left[0], lower_right[0], 0.035, material)
    _add_member(part, lower_left[0], upper[0], 0.032, material)
    _add_member(part, lower_right[0], upper[0], 0.032, material)

    for i in range(panels):
        _add_member(part, lower_left[i], lower_left[i + 1], 0.035, material)
        _add_member(part, lower_right[i], lower_right[i + 1], 0.035, material)
        _add_member(part, upper[i], upper[i + 1], 0.032, material)
        _add_member(part, lower_left[i], upper[i + 1], 0.020, material)
        _add_member(part, lower_right[i], upper[i + 1], 0.020, material)
        _add_member(part, upper[i], lower_left[i + 1], 0.018, material)
        _add_member(part, upper[i], lower_right[i + 1], 0.018, material)
        _add_member(part, lower_left[i], lower_right[i], 0.018, material)

    _add_member(part, lower_left[-1], lower_right[-1], 0.026, material)
    _add_member(part, lower_left[-1], upper[-1], 0.024, material)
    _add_member(part, lower_right[-1], upper[-1], 0.024, material)
    _add_member(part, upper[0], upper[-1], 0.012, cable_material, name="boom_top_hoist_line", overlap=0.0)

    # Tip sheaves and a small guard frame where the pendant line leaves the boom.
    part.visual(
        Cylinder(radius=0.18, length=0.16),
        origin=Origin(xyz=(BOOM_LENGTH, 0.0, BOOM_TIP_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_material,
        name="tip_sheave",
    )
    part.visual(
        Cylinder(radius=0.115, length=0.18),
        origin=Origin(xyz=(BOOM_LENGTH - 0.08, 0.0, BOOM_TIP_Z + 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cable_material,
        name="reeved_line_on_sheave",
    )
    _add_member(part, upper[-1], (BOOM_LENGTH, 0.0, BOOM_TIP_Z + 0.16), 0.020, material)
    _add_member(part, lower_left[-1], (BOOM_LENGTH, 0.0, BOOM_TIP_Z - 0.02), 0.020, material)
    _add_member(part, lower_right[-1], (BOOM_LENGTH, 0.0, BOOM_TIP_Z - 0.02), 0.020, material)


def _build_hook_mesh():
    hook_tube = tube_from_spline_points(
        [
            (0.040, 0.0, -1.58),
            (0.078, 0.0, -1.67),
            (0.062, 0.0, -1.82),
            (-0.030, 0.0, -1.90),
            (-0.110, 0.0, -1.84),
            (-0.095, 0.0, -1.72),
            (-0.020, 0.0, -1.69),
        ],
        radius=0.025,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_tube, "offshore_pedestal_crane_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deck_mounted_offshore_pedestal_crane")

    offshore_yellow = model.material("offshore_yellow", rgba=(0.94, 0.68, 0.12, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.34, 0.08, 1.0))
    deck_grey = model.material("deck_grey", rgba=(0.38, 0.42, 0.43, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    bare_steel = model.material("bare_steel", rgba=(0.62, 0.64, 0.65, 1.0))
    cable_black = model.material("cable_black", rgba=(0.02, 0.02, 0.022, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.48, 0.70, 0.86, 0.45))

    hook_mesh = _build_hook_mesh()

    pedestal = model.part("pedestal")
    pedestal.visual(Box((3.30, 3.30, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=deck_grey, name="deck_plate")
    pedestal.visual(Cylinder(radius=1.24, length=0.20), origin=Origin(xyz=(0.0, 0.0, 0.22)), material=dark_steel, name="round_flange")
    pedestal.visual(Cylinder(radius=0.66, length=0.22), origin=Origin(xyz=(0.0, 0.0, 0.43)), material=offshore_yellow, name="pedestal_foot")
    pedestal.visual(Cylinder(radius=0.48, length=2.20), origin=Origin(xyz=(0.0, 0.0, 1.42)), material=offshore_yellow, name="pedestal_tube")
    pedestal.visual(Cylinder(radius=0.60, length=0.20), origin=Origin(xyz=(0.0, 0.0, 2.62)), material=dark_steel, name="slew_support_cap")

    # Welded triangular gussets around the pedestal foot.
    for i in range(8):
        angle = i * math.tau / 8.0
        pedestal.visual(
            Box((0.82, 0.075, 0.42)),
            origin=Origin(xyz=(0.46 * math.cos(angle), 0.46 * math.sin(angle), 0.52), rpy=(0.0, 0.0, angle)),
            material=offshore_yellow,
            name=f"base_gusset_{i}",
        )
    # Tall studs and nuts around the deck flange.
    for i in range(16):
        angle = i * math.tau / 16.0
        x = 0.96 * math.cos(angle)
        y = 0.96 * math.sin(angle)
        pedestal.visual(
            Cylinder(radius=0.045, length=0.11),
            origin=Origin(xyz=(x, y, 0.355)),
            material=bare_steel,
            name=f"flange_bolt_{i}",
        )
        pedestal.visual(
            Cylinder(radius=0.070, length=0.045),
            origin=Origin(xyz=(x, y, 0.415)),
            material=dark_steel,
            name=f"flange_nut_{i}",
        )

    superstructure = model.part("superstructure")
    superstructure.visual(Cylinder(radius=0.58, length=0.22), origin=Origin(xyz=(0.0, 0.0, 0.11)), material=dark_steel, name="slewing_bearing")
    superstructure.visual(Box((2.35, 1.18, 0.22)), origin=Origin(xyz=(-0.06, 0.0, 0.31)), material=painted_steel, name="machinery_deck")
    superstructure.visual(Box((0.98, 0.72, 0.68)), origin=Origin(xyz=(-0.42, 0.02, 0.76)), material=offshore_yellow, name="machinery_house")
    superstructure.visual(Box((0.58, 0.50, 0.54)), origin=Origin(xyz=(0.25, -0.43, 0.72)), material=safety_orange, name="operator_cab")
    superstructure.visual(Box((0.50, 0.018, 0.36)), origin=Origin(xyz=(0.30, -0.671, 0.77)), material=cab_glass, name="cab_front_glass")
    superstructure.visual(Box((0.018, 0.40, 0.32)), origin=Origin(xyz=(0.535, -0.49, 0.76)), material=cab_glass, name="cab_side_glass")
    superstructure.visual(Box((0.52, 0.92, 0.50)), origin=Origin(xyz=(-1.07, 0.0, 0.67)), material=dark_steel, name="counterweight")
    superstructure.visual(
        Cylinder(radius=0.22, length=0.68),
        origin=Origin(xyz=(-0.22, 0.03, 0.57), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="hoist_winch_drum",
    )
    superstructure.visual(Box((0.32, 0.10, 0.78)), origin=Origin(xyz=(0.86, -0.38, 0.81)), material=offshore_yellow, name="boom_clevis_0")
    superstructure.visual(Box((0.32, 0.10, 0.78)), origin=Origin(xyz=(0.86, 0.38, 0.81)), material=offshore_yellow, name="boom_clevis_1")
    superstructure.visual(
        Cylinder(radius=0.18, length=0.14),
        origin=Origin(xyz=(0.86, -0.38, 0.88), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="clevis_bearing_0",
    )
    superstructure.visual(
        Cylinder(radius=0.18, length=0.14),
        origin=Origin(xyz=(0.86, 0.38, 0.88), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="clevis_bearing_1",
    )
    superstructure.visual(Box((0.86, 0.16, 0.12)), origin=Origin(xyz=(0.54, 0.0, 0.47)), material=offshore_yellow, name="front_cross_member")

    boom = model.part("boom")
    _add_lattice_boom(boom, offshore_yellow, cable_black, dark_steel)

    hook_block = model.part("hook_block")
    hook_block.visual(
        Cylinder(radius=0.035, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cable_black,
        name="sheave_wrap",
    )
    hook_block.visual(Cylinder(radius=0.018, length=1.30), origin=Origin(xyz=(0.0, -0.055, -0.64)), material=cable_black, name="pendant_line_0")
    hook_block.visual(Cylinder(radius=0.018, length=1.30), origin=Origin(xyz=(0.0, 0.055, -0.64)), material=cable_black, name="pendant_line_1")
    hook_block.visual(Cylinder(radius=0.08, length=0.16), origin=Origin(xyz=(0.0, 0.0, -1.26), rpy=(math.pi / 2.0, 0.0, 0.0)), material=bare_steel, name="upper_sheave")
    hook_block.visual(Box((0.34, 0.22, 0.22)), origin=Origin(xyz=(0.0, 0.0, -1.39)), material=offshore_yellow, name="hook_block_shell")
    hook_block.visual(Cylinder(radius=0.055, length=0.18), origin=Origin(xyz=(0.0, 0.0, -1.50)), material=dark_steel, name="swivel_stem")
    hook_block.visual(Sphere(radius=0.075), origin=Origin(xyz=(0.0, 0.0, -1.56)), material=dark_steel, name="swivel_ball")
    hook_block.visual(hook_mesh, material=safety_orange, name="load_hook")

    model.articulation(
        "slewing",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=superstructure,
        origin=Origin(xyz=(0.0, 0.0, 2.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.45),
    )
    model.articulation(
        "boom_luff",
        ArticulationType.REVOLUTE,
        parent=superstructure,
        child=boom,
        origin=Origin(xyz=(0.86, 0.0, 0.88), rpy=(0.0, -BOOM_REST_ANGLE, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.18, lower=-0.25, upper=0.85),
    )
    model.articulation(
        "hook_pendant",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=hook_block,
        origin=Origin(xyz=(BOOM_LENGTH, 0.0, BOOM_TIP_Z - 0.199), rpy=(0.0, BOOM_REST_ANGLE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=-0.25, upper=0.85),
        mimic=Mimic("boom_luff", multiplier=1.0, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    superstructure = object_model.get_part("superstructure")
    boom = object_model.get_part("boom")
    hook_block = object_model.get_part("hook_block")
    slewing = object_model.get_articulation("slewing")
    boom_luff = object_model.get_articulation("boom_luff")
    hook_pendant = object_model.get_articulation("hook_pendant")

    ctx.check(
        "full-circle slewing joint is vertical",
        slewing.articulation_type == ArticulationType.CONTINUOUS and tuple(slewing.axis) == (0.0, 0.0, 1.0),
        details=f"type={slewing.articulation_type}, axis={slewing.axis}",
    )
    ctx.check(
        "boom luff joint is horizontal",
        boom_luff.articulation_type == ArticulationType.REVOLUTE and abs(boom_luff.axis[1]) > 0.99,
        details=f"type={boom_luff.articulation_type}, axis={boom_luff.axis}",
    )
    ctx.check(
        "hook pendant follows luffing boom",
        hook_pendant.mimic is not None and hook_pendant.mimic.joint == "boom_luff",
        details=f"mimic={hook_pendant.mimic}",
    )
    ctx.allow_overlap(
        boom,
        hook_block,
        elem_a="tip_sheave",
        elem_b="sheave_wrap",
        reason="The pendant line is intentionally shown wrapped over the boom-tip sheave; the local overlap represents rope seating in the sheave groove.",
    )
    ctx.expect_contact(
        boom,
        hook_block,
        elem_a="tip_sheave",
        elem_b="sheave_wrap",
        contact_tol=0.02,
        name="pendant line seats on boom-tip sheave",
    )
    ctx.expect_gap(
        superstructure,
        pedestal,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="slewing_bearing",
        negative_elem="slew_support_cap",
        name="turntable sits on pedestal cap",
    )
    ctx.expect_overlap(
        superstructure,
        pedestal,
        axes="xy",
        min_overlap=0.85,
        elem_a="slewing_bearing",
        elem_b="slew_support_cap",
        name="slewing bearing centered over pedestal",
    )
    ctx.expect_within(
        boom,
        superstructure,
        axes="y",
        margin=0.001,
        inner_elem="boom_hub",
        outer_elem="machinery_deck",
        name="boom hinge knuckle lies within the clevis width",
    )

    rest_hook = ctx.part_world_position(hook_block)
    with ctx.pose({boom_luff: 0.65}):
        raised_hook = ctx.part_world_position(hook_block)
    ctx.check(
        "luffing up raises the boom tip and pendant",
        rest_hook is not None and raised_hook is not None and raised_hook[2] > rest_hook[2] + 1.0,
        details=f"rest={rest_hook}, raised={raised_hook}",
    )

    return ctx.report()


object_model = build_object_model()
