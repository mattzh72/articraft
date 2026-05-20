from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


FRONT_RAIL_TOP = (0.28, -0.02, -0.04)
FRONT_RAIL_FOOT = (0.38, -0.50, -1.35)
REAR_RAIL_TOP = (0.29, 0.150, -0.04)
REAR_RAIL_FOOT = (0.43, 0.82, -1.35)
HINGE_ORIGIN = (0.0, 0.080, -0.018)


def _rail_point(
    z: float,
    *,
    side: float,
    top: tuple[float, float, float],
    foot: tuple[float, float, float],
) -> tuple[float, float, float]:
    t = (z - top[2]) / (foot[2] - top[2])
    x = side * (top[0] + t * (foot[0] - top[0]))
    y = top[1] + t * (foot[1] - top[1])
    return (x, y, z)


def _tube(points: list[tuple[float, float, float]], radius: float, name: str):
    return mesh_from_geometry(
        wire_from_points(
            points,
            radius=radius,
            radial_segments=20,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.035,
            corner_segments=10,
        ),
        name,
    )


def _rear_local(point: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        point[0] - HINGE_ORIGIN[0],
        point[1] - HINGE_ORIGIN[1],
        point[2] - HINGE_ORIGIN[2],
    )


def _add_front_rail(part, *, side: float, aluminum: Material, rubber: Material) -> None:
    top = _rail_point(FRONT_RAIL_TOP[2], side=side, top=FRONT_RAIL_TOP, foot=FRONT_RAIL_FOOT)
    mid = _rail_point(-0.70, side=side, top=FRONT_RAIL_TOP, foot=FRONT_RAIL_FOOT)
    foot = _rail_point(FRONT_RAIL_FOOT[2], side=side, top=FRONT_RAIL_TOP, foot=FRONT_RAIL_FOOT)
    side_name = "left" if side > 0 else "right"

    part.visual(
        _tube([top, mid, foot], 0.026, f"front_{side_name}_rail_tube"),
        material=aluminum,
        name="rail_tube",
    )
    part.visual(
        Box((0.085, 0.080, 0.070)),
        origin=Origin(xyz=(top[0], top[1], top[2] - 0.015)),
        material=aluminum,
        name="top_plug",
    )

    # Small inward ledger plates are welded to the rails and physically support
    # the separate tread modules without making the treads themselves part of
    # either rail.
    for idx, tread_z in enumerate((-1.05, -0.72, -0.39)):
        rx, ry, _ = _rail_point(tread_z, side=side, top=FRONT_RAIL_TOP, foot=FRONT_RAIL_FOOT)
        part.visual(
            Box((0.115, 0.070, 0.030)),
            origin=Origin(xyz=(rx - side * 0.047, ry, tread_z - 0.033)),
            material=aluminum,
            name=f"tread_ledger_{idx}",
        )

    part.visual(
        Box((0.16, 0.105, 0.050)),
        origin=Origin(xyz=(foot[0], foot[1] - 0.020, foot[2] - 0.030)),
        material=rubber,
        name="front_foot",
    )


def _add_rear_rail(part, *, side: float, aluminum: Material, rubber: Material) -> None:
    top_world = _rail_point(REAR_RAIL_TOP[2], side=side, top=REAR_RAIL_TOP, foot=REAR_RAIL_FOOT)
    mid_world = _rail_point(-0.72, side=side, top=REAR_RAIL_TOP, foot=REAR_RAIL_FOOT)
    foot_world = _rail_point(REAR_RAIL_FOOT[2], side=side, top=REAR_RAIL_TOP, foot=REAR_RAIL_FOOT)
    top = _rear_local(top_world)
    mid = _rear_local(mid_world)
    foot = _rear_local(foot_world)
    side_name = "left" if side > 0 else "right"
    part.visual(
        _tube([top, mid, foot], 0.024, f"rear_{side_name}_rail_tube"),
        material=aluminum,
        name="rail_tube",
    )
    part.visual(
        Box((0.080, 0.090, 0.070)),
        origin=Origin(xyz=top),
        material=aluminum,
        name="top_plug",
    )
    part.visual(
        Box((0.15, 0.11, 0.050)),
        origin=Origin(xyz=(foot[0], foot[1] + 0.025, foot[2] - 0.030)),
        material=rubber,
        name="rear_foot",
    )


def _add_tread(part, *, width: float, depth: float, aluminum: Material, dark_rubber: Material) -> None:
    part.visual(
        Box((width, depth, 0.036)),
        material=aluminum,
        name="tread_pan",
    )
    part.visual(
        Box((width * 0.88, depth * 0.12, 0.007)),
        origin=Origin(xyz=(0.0, -depth * 0.28, 0.020)),
        material=dark_rubber,
        name="front_grip",
    )
    part.visual(
        Box((width * 0.88, depth * 0.12, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_rubber,
        name="center_grip",
    )
    part.visual(
        Box((width * 0.88, depth * 0.12, 0.007)),
        origin=Origin(xyz=(0.0, depth * 0.28, 0.020)),
        material=dark_rubber,
        name="rear_grip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.74, 1.0))
    darker_aluminum = model.material("ribbed_aluminum", rgba=(0.56, 0.58, 0.58, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.035, 0.035, 0.032, 1.0))
    blue_plastic = model.material("blue_plastic", rgba=(0.08, 0.18, 0.40, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.30, 0.31, 0.32, 1.0))

    front_top_bracket = model.part("front_top_bracket")
    front_top_bracket.visual(
        Box((0.76, 0.30, 0.070)),
        origin=Origin(xyz=(0.0, -0.060, 0.052)),
        material=blue_plastic,
        name="top_cap",
    )
    front_top_bracket.visual(
        Box((0.68, 0.20, 0.018)),
        origin=Origin(xyz=(0.0, -0.070, 0.096)),
        material=dark_rubber,
        name="top_grip_pad",
    )
    front_top_bracket.visual(
        Cylinder(radius=0.026, length=0.16),
        origin=Origin(xyz=(-0.30, 0.080, -0.018), rpy=(0.0, 1.57079632679, 0.0)),
        material=hinge_steel,
        name="hinge_barrel_0",
    )
    front_top_bracket.visual(
        Cylinder(radius=0.026, length=0.16),
        origin=Origin(xyz=(0.30, 0.080, -0.018), rpy=(0.0, 1.57079632679, 0.0)),
        material=hinge_steel,
        name="hinge_barrel_1",
    )
    front_top_bracket.visual(
        Box((0.13, 0.12, 0.055)),
        origin=Origin(xyz=(-0.30, 0.045, -0.010)),
        material=blue_plastic,
        name="side_lug_0",
    )
    front_top_bracket.visual(
        Box((0.13, 0.12, 0.055)),
        origin=Origin(xyz=(0.30, 0.045, -0.010)),
        material=blue_plastic,
        name="side_lug_1",
    )

    front_left_rail = model.part("front_left_rail")
    _add_front_rail(front_left_rail, side=1.0, aluminum=aluminum, rubber=dark_rubber)

    front_right_rail = model.part("front_right_rail")
    _add_front_rail(front_right_rail, side=-1.0, aluminum=aluminum, rubber=dark_rubber)

    tread_centers = []
    for tread_z in (-1.05, -0.72, -0.39):
        _, tread_y, _ = _rail_point(tread_z, side=1.0, top=FRONT_RAIL_TOP, foot=FRONT_RAIL_FOOT)
        tread_centers.append((0.0, tread_y, tread_z))

    treads = []
    for idx, center in enumerate(tread_centers):
        tread = model.part(f"tread_{idx}")
        _add_tread(tread, width=(0.58, 0.52, 0.48)[idx], depth=0.19, aluminum=darker_aluminum, dark_rubber=dark_rubber)
        treads.append((tread, center))

    rear_top_bracket = model.part("rear_top_bracket")
    rear_top_bracket.visual(
        Cylinder(radius=0.024, length=0.44),
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=hinge_steel,
        name="center_barrel",
    )
    rear_top_bracket.visual(
        Box((0.54, 0.11, 0.050)),
        origin=Origin(xyz=_rear_local((0.0, 0.170, -0.045))),
        material=blue_plastic,
        name="rear_yoke",
    )
    rear_top_bracket.visual(
        Box((0.42, 0.055, 0.040)),
        origin=Origin(xyz=_rear_local((0.0, 0.235, -0.110))),
        material=aluminum,
        name="rear_cross_socket",
    )
    rear_top_bracket.visual(
        Box((0.18, 0.050, 0.070)),
        origin=Origin(xyz=_rear_local((0.0, 0.205, -0.075))),
        material=blue_plastic,
        name="socket_web",
    )
    rear_top_bracket.visual(
        Box((0.18, 0.050, 0.040)),
        origin=Origin(xyz=_rear_local((0.0, 0.115, -0.035))),
        material=blue_plastic,
        name="hinge_web",
    )

    rear_left_rail = model.part("rear_left_rail")
    _add_rear_rail(rear_left_rail, side=1.0, aluminum=aluminum, rubber=dark_rubber)

    rear_right_rail = model.part("rear_right_rail")
    _add_rear_rail(rear_right_rail, side=-1.0, aluminum=aluminum, rubber=dark_rubber)

    rear_crossbar = model.part("rear_crossbar")
    cross_z = -0.83
    _, cross_y, _ = _rail_point(cross_z, side=1.0, top=REAR_RAIL_TOP, foot=REAR_RAIL_FOOT)
    rear_crossbar.visual(
        Cylinder(radius=0.020, length=0.72),
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=aluminum,
        name="rear_cross_tube",
    )

    model.articulation(
        "front_left_mount",
        ArticulationType.FIXED,
        parent=front_top_bracket,
        child=front_left_rail,
        origin=Origin(),
    )
    model.articulation(
        "front_right_mount",
        ArticulationType.FIXED,
        parent=front_top_bracket,
        child=front_right_rail,
        origin=Origin(),
    )
    for idx, (tread, center) in enumerate(treads):
        model.articulation(
            f"tread_{idx}_mount",
            ArticulationType.FIXED,
            parent=front_top_bracket,
            child=tread,
            origin=Origin(xyz=center),
        )

    model.articulation(
        "rear_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=front_top_bracket,
        child=rear_top_bracket,
        origin=Origin(xyz=(0.0, 0.080, -0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=0.78),
    )
    model.articulation(
        "rear_left_mount",
        ArticulationType.FIXED,
        parent=rear_top_bracket,
        child=rear_left_rail,
        origin=Origin(),
    )
    model.articulation(
        "rear_right_mount",
        ArticulationType.FIXED,
        parent=rear_top_bracket,
        child=rear_right_rail,
        origin=Origin(),
    )
    model.articulation(
        "rear_crossbar_mount",
        ArticulationType.FIXED,
        parent=rear_top_bracket,
        child=rear_crossbar,
        origin=Origin(xyz=_rear_local((0.0, cross_y, cross_z))),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hinge = object_model.get_articulation("rear_frame_hinge")
    rear_left = object_model.get_part("rear_left_rail")
    front_left = object_model.get_part("front_left_rail")
    front_right = object_model.get_part("front_right_rail")
    front_top = object_model.get_part("front_top_bracket")
    rear_top = object_model.get_part("rear_top_bracket")
    lower_tread = object_model.get_part("tread_0")
    middle_tread = object_model.get_part("tread_1")
    upper_tread = object_model.get_part("tread_2")

    ctx.allow_overlap(
        front_top,
        front_left,
        elem_a="side_lug_1",
        elem_b="top_plug",
        reason="The front rail end is intentionally seated into the molded top-cap side lug.",
    )
    ctx.expect_overlap(
        front_top,
        front_left,
        axes="xyz",
        elem_a="side_lug_1",
        elem_b="top_plug",
        min_overlap=0.01,
        name="front left rail captured in top lug",
    )
    ctx.allow_overlap(
        front_top,
        front_right,
        elem_a="side_lug_0",
        elem_b="top_plug",
        reason="The front rail end is intentionally seated into the molded top-cap side lug.",
    )
    ctx.expect_overlap(
        front_top,
        front_right,
        axes="xyz",
        elem_a="side_lug_0",
        elem_b="top_plug",
        min_overlap=0.01,
        name="front right rail captured in top lug",
    )
    ctx.allow_overlap(
        rear_top,
        rear_left,
        elem_a="rear_yoke",
        elem_b="top_plug",
        reason="The rear rail end is intentionally nested in the hinged rear yoke socket.",
    )
    ctx.expect_overlap(
        rear_top,
        rear_left,
        axes="xyz",
        elem_a="rear_yoke",
        elem_b="top_plug",
        min_overlap=0.01,
        name="rear left rail captured in yoke",
    )
    ctx.allow_overlap(
        rear_top,
        object_model.get_part("rear_right_rail"),
        elem_a="rear_yoke",
        elem_b="top_plug",
        reason="The rear rail end is intentionally nested in the hinged rear yoke socket.",
    )
    ctx.expect_overlap(
        rear_top,
        object_model.get_part("rear_right_rail"),
        axes="xyz",
        elem_a="rear_yoke",
        elem_b="top_plug",
        min_overlap=0.01,
        name="rear right rail captured in yoke",
    )

    for side_name, rail_part in (("left", rear_left), ("right", object_model.get_part("rear_right_rail"))):
        ctx.allow_overlap(
            object_model.get_part("rear_crossbar"),
            rail_part,
            elem_a="rear_cross_tube",
            elem_b="rail_tube",
            reason=f"The rear {side_name} crossbar end is welded into the rear rail tube.",
        )
        ctx.expect_overlap(
            object_model.get_part("rear_crossbar"),
            rail_part,
            axes="xyz",
            elem_a="rear_cross_tube",
            elem_b="rail_tube",
            min_overlap=0.01,
            name=f"rear {side_name} crossbar welded into rail",
        )

    ctx.expect_overlap(
        lower_tread,
        front_left,
        axes="xy",
        elem_a="tread_pan",
        elem_b="tread_ledger_0",
        min_overlap=0.02,
        name="lower tread overlaps rail ledger footprint",
    )
    ctx.expect_gap(
        lower_tread,
        front_left,
        axis="z",
        positive_elem="tread_pan",
        negative_elem="tread_ledger_0",
        max_gap=0.002,
        max_penetration=0.0,
        name="lower tread rests on rail ledger",
    )
    ctx.expect_overlap(
        middle_tread,
        front_left,
        axes="xy",
        elem_a="tread_pan",
        elem_b="tread_ledger_1",
        min_overlap=0.02,
        name="middle tread overlaps rail ledger footprint",
    )
    ctx.expect_gap(
        middle_tread,
        front_left,
        axis="z",
        positive_elem="tread_pan",
        negative_elem="tread_ledger_1",
        max_gap=0.002,
        max_penetration=0.0,
        name="middle tread rests on rail ledger",
    )
    ctx.expect_overlap(
        upper_tread,
        front_left,
        axes="xy",
        elem_a="tread_pan",
        elem_b="tread_ledger_2",
        min_overlap=0.02,
        name="upper tread overlaps rail ledger footprint",
    )
    ctx.expect_gap(
        upper_tread,
        front_left,
        axis="z",
        positive_elem="tread_pan",
        negative_elem="tread_ledger_2",
        max_gap=0.002,
        max_penetration=0.0,
        name="upper tread rests on rail ledger",
    )

    open_aabb = ctx.part_world_aabb(rear_left)
    with ctx.pose({hinge: 0.78}):
        folded_aabb = ctx.part_world_aabb(rear_left)

    def _center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[1] + hi[1]) * 0.5

    ctx.check(
        "rear frame folds toward front frame",
        open_aabb is not None
        and folded_aabb is not None
        and _center_y(folded_aabb) is not None
        and _center_y(open_aabb) is not None
        and _center_y(folded_aabb) < _center_y(open_aabb) - 0.45,
        details=f"open={open_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
