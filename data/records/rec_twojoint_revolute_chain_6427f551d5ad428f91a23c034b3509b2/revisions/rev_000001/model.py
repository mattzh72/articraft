from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


BASE_HINGE_Z = 0.180
PRIMARY_LEN = 0.480
SECONDARY_LEN = 0.340


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    """CadQuery cylinder with its axis along the model Y direction."""
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )


def _cylinder_x(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    """CadQuery cylinder with its axis along the model X direction."""
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((x, 0.0, 0.0))
    )


def _base_body() -> cq.Workplane:
    body = _box((0.340, 0.250, 0.035), (0.0, 0.0, 0.0175))
    body = body.union(_box((0.165, 0.115, 0.070), (-0.012, 0.0, 0.070)))

    # Two grounded cheeks with rounded boss-like tops form the clevis around the
    # first hinge.  The generous central slot is what the broad primary link
    # swings through.
    for sign in (-1.0, 1.0):
        y = sign * 0.085
        cheek = _box((0.112, 0.028, 0.145), (0.0, y, 0.1075))
        cheek = cheek.union(_cylinder_y(0.056, 0.028, x=0.0, y=y, z=BASE_HINGE_Z))
        body = body.union(cheek)

        # External machined boss and vertical web ribs at each cheek.
        body = body.union(_cylinder_y(0.044, 0.014, x=0.0, y=sign * 0.107, z=BASE_HINGE_Z))
        for x in (-0.045, 0.045):
            body = body.union(_box((0.016, 0.018, 0.112), (x, sign * 0.082, 0.091)))

    # Low, out-of-path rubber stop pads on the pedestal.
    body = body.union(_box((0.030, 0.042, 0.015), (0.078, -0.030, 0.050)))
    body = body.union(_box((0.030, 0.042, 0.015), (0.078, 0.030, 0.050)))

    # Four cap-screw heads visibly ground the bench bracket.
    for x in (-0.125, 0.125):
        for y in (-0.088, 0.088):
            body = body.union(_cylinder_y(0.012, 0.006, x=x, y=y, z=0.039))
    return body


def _primary_body() -> cq.Workplane:
    plate_y = 0.042
    plate_t = 0.018
    plate_h = 0.070
    end_r = 0.055

    body = None
    for sign in (-1.0, 1.0):
        y = sign * plate_y
        side = _box((PRIMARY_LEN, plate_t, plate_h), (PRIMARY_LEN / 2.0, y, 0.0))
        side = side.union(_cylinder_y(end_r, plate_t, x=0.0, y=y, z=0.0))
        side = side.union(_cylinder_y(end_r, plate_t, x=PRIMARY_LEN, y=y, z=0.0))

        # Raised, machined side bosses at both pin centers.
        boss_y = sign * (plate_y + plate_t / 2.0 + 0.004)
        side = side.union(_cylinder_y(0.069, 0.008, x=0.0, y=boss_y, z=0.0))
        side = side.union(_cylinder_y(0.062, 0.008, x=PRIMARY_LEN, y=boss_y, z=0.0))
        body = side if body is None else body.union(side)

    # Cross-ribs turn the paired side plates into one broad fabricated link
    # without closing the base or elbow clearances.
    body = body.union(_box((0.260, 0.075, 0.012), (0.240, 0.0, 0.037)))
    body = body.union(_box((0.260, 0.075, 0.012), (0.240, 0.0, -0.037)))
    body = body.union(_box((0.090, 0.072, 0.014), (0.130, 0.0, 0.0)))
    body = body.union(_box((0.090, 0.072, 0.014), (PRIMARY_LEN - 0.145, 0.0, 0.0)))

    # The grounded base pin passes through this clearance bore.
    body = body.cut(_cylinder_y(0.024, 0.170, x=0.0, y=0.0, z=0.0))
    return body


def _primary_side_plate(sign: float) -> cq.Workplane:
    plate_y = sign * 0.032
    plate_t = 0.014
    plate_h = 0.070
    body = _box((PRIMARY_LEN, plate_t, plate_h), (PRIMARY_LEN / 2.0, plate_y, 0.0))
    body = body.union(_cylinder_y(0.055, plate_t, x=0.0, y=plate_y, z=0.0))
    body = body.union(_cylinder_y(0.055, plate_t, x=PRIMARY_LEN, y=plate_y, z=0.0))
    body = body.union(
        _cylinder_y(
            0.067,
            0.006,
            x=0.0,
            y=sign * (0.032 + plate_t / 2.0 + 0.003),
            z=0.0,
        )
    )
    body = body.union(
        _cylinder_y(
            0.061,
            0.006,
            x=PRIMARY_LEN,
            y=sign * (0.032 + plate_t / 2.0 + 0.003),
            z=0.0,
        )
    )
    body = body.cut(_cylinder_y(0.019, 0.060, x=0.0, y=0.0, z=0.0))
    body = body.cut(_cylinder_y(0.019, 0.060, x=PRIMARY_LEN, y=0.0, z=0.0))
    return body


def _base_cheek(sign: float) -> cq.Workplane:
    y = sign * 0.085
    cheek = _box((0.112, 0.028, 0.145), (0.0, y, 0.1075))
    cheek = cheek.union(_cylinder_y(0.056, 0.028, x=0.0, y=y, z=BASE_HINGE_Z))
    return cheek


def _secondary_body() -> cq.Workplane:
    body = _box((SECONDARY_LEN, 0.030, 0.055), (SECONDARY_LEN / 2.0, 0.0, 0.0))
    body = body.union(_cylinder_y(0.047, 0.032, x=0.0, y=0.0, z=0.0))
    body = body.union(_cylinder_y(0.038, 0.030, x=SECONDARY_LEN, y=0.0, z=0.0))

    # Narrow but stiff local ribs and an elbow boss.
    body = body.union(_box((0.230, 0.008, 0.069), (0.175, 0.019, 0.0)))
    body = body.union(_box((0.230, 0.008, 0.069), (0.175, -0.019, 0.0)))
    body = body.union(_cylinder_y(0.056, 0.034, x=0.0, y=0.0, z=0.0))

    # The primary-link elbow pin has real swing clearance through the lug.
    body = body.cut(_cylinder_y(0.019, 0.090, x=0.0, y=0.0, z=0.0))
    return body


def _tool_tab() -> cq.Workplane:
    tab_x = SECONDARY_LEN + 0.022
    tab = _box((0.026, 0.105, 0.105), (tab_x, 0.0, 0.0))
    tab = tab.union(_box((0.074, 0.014, 0.078), (SECONDARY_LEN - 0.020, 0.034, 0.0)))
    tab = tab.union(_box((0.074, 0.014, 0.078), (SECONDARY_LEN - 0.020, -0.034, 0.0)))
    tab = tab.cut(_cylinder_x(0.021, 0.060, x=tab_x, y=0.0, z=0.0))
    return tab


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_bench_arm")

    cast_iron = model.material("dark_cast_iron", rgba=(0.13, 0.14, 0.15, 1.0))
    blue_steel = model.material("blue_machined_steel", rgba=(0.05, 0.19, 0.44, 1.0))
    graphite = model.material("graphite_link", rgba=(0.18, 0.19, 0.20, 1.0))
    polished = model.material("polished_pin", rgba=(0.72, 0.70, 0.65, 1.0))
    rubber = model.material("rubber_stops", rgba=(0.03, 0.03, 0.03, 1.0))

    base = model.part("ground_clevis")
    base.visual(
        Box((0.340, 0.250, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.165, 0.115, 0.070)),
        origin=Origin(xyz=(-0.012, 0.0, 0.070)),
        material=cast_iron,
        name="pedestal",
    )
    for idx, sign in enumerate((-1.0, 1.0)):
        base.visual(
            mesh_from_cadquery(_base_cheek(sign), f"clevis_cheek_{idx}", tolerance=0.001),
            material=cast_iron,
            name=f"clevis_cheek_{idx}",
        )
        base.visual(
            Cylinder(radius=0.044, length=0.018),
            origin=Origin(xyz=(0.0, sign * 0.108, BASE_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name=f"base_pin_cap_{idx}",
        )
        for rib_idx, x in enumerate((-0.045, 0.045)):
            base.visual(
                Box((0.016, 0.018, 0.112)),
                origin=Origin(xyz=(x, sign * 0.082, 0.091)),
                material=cast_iron,
                name=f"clevis_rib_{idx}_{rib_idx}",
            )
    base.visual(
        Box((0.074, 0.024, 0.012)),
        origin=Origin(xyz=(0.078, 0.0, 0.061)),
        material=rubber,
        name="stop_face",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, BASE_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="base_pin",
    )
    for bolt_idx, (x, y) in enumerate(
        ((-0.125, -0.088), (-0.125, 0.088), (0.125, -0.088), (0.125, 0.088))
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.038)),
            material=polished,
            name=f"anchor_bolt_{bolt_idx}",
        )

    primary = model.part("primary_link")
    for idx, sign in enumerate((-1.0, 1.0)):
        primary.visual(
            mesh_from_cadquery(_primary_side_plate(sign), f"primary_side_{idx}", tolerance=0.001),
            material=blue_steel,
            name=f"primary_side_{idx}",
        )
    for rib_name, size, xyz in (
        ("top_rib", (0.260, 0.075, 0.012), (0.240, 0.0, 0.037)),
        ("bottom_rib", (0.260, 0.075, 0.012), (0.240, 0.0, -0.037)),
        ("root_bridge", (0.090, 0.072, 0.014), (0.130, 0.0, 0.0)),
        ("elbow_bridge", (0.090, 0.072, 0.014), (PRIMARY_LEN - 0.145, 0.0, 0.0)),
    ):
        primary.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=blue_steel,
            name=rib_name,
        )
    primary.visual(
        Cylinder(radius=0.020, length=0.132),
        origin=Origin(xyz=(PRIMARY_LEN, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="elbow_pin",
    )
    for idx, y in enumerate((-0.067, 0.067)):
        primary.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(PRIMARY_LEN, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name=f"elbow_cap_{idx}",
        )

    secondary = model.part("secondary_link")
    secondary.visual(
        mesh_from_cadquery(_secondary_body(), "secondary_link_body", tolerance=0.001),
        material=graphite,
        name="secondary_body",
    )
    secondary.visual(
        mesh_from_cadquery(_tool_tab(), "square_tool_tab", tolerance=0.001),
        material=polished,
        name="tool_tab",
    )

    shoulder = model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=primary,
        origin=Origin(xyz=(0.0, 0.0, BASE_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=0.0, upper=1.10),
    )
    elbow = model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(PRIMARY_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=0.0, upper=1.30),
    )

    shoulder.meta["description"] = "Supported base revolute axis through the grounded clevis."
    elbow.meta["description"] = "Parallel supported elbow axis carrying the narrow nose link."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("ground_clevis")
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    shoulder = object_model.get_articulation("base_hinge")
    elbow = object_model.get_articulation("elbow_hinge")

    ctx.check(
        "two serial revolute joints",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and elbow.parent == "primary_link"
        and elbow.child == "secondary_link",
        details=f"shoulder={shoulder}, elbow={elbow}",
    )
    ctx.check(
        "parallel supported hinge axes",
        tuple(shoulder.axis) == (0.0, -1.0, 0.0) and tuple(elbow.axis) == (0.0, -1.0, 0.0),
        details=f"shoulder_axis={shoulder.axis}, elbow_axis={elbow.axis}",
    )

    for side_name in ("primary_side_0", "primary_side_1"):
        ctx.allow_overlap(
            base,
            primary,
            elem_a="base_pin",
            elem_b=side_name,
            reason="The supported shoulder pin is intentionally captured in the primary-link bore with a slight bearing fit.",
        )
        ctx.expect_overlap(
            base,
            primary,
            axes="y",
            elem_a="base_pin",
            elem_b=side_name,
            min_overlap=0.010,
            name=f"base pin passes through {side_name}",
        )

    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_pin",
        elem_b="secondary_body",
        reason="The elbow pin is intentionally captured in the secondary-link bore as the load-carrying bearing.",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="y",
        elem_a="elbow_pin",
        elem_b="secondary_body",
        min_overlap=0.025,
        name="elbow pin passes through secondary lug",
    )

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "clevis is grounded on bench plane",
        base_aabb is not None and abs(base_aabb[0][2]) < 0.002,
        details=f"base_aabb={base_aabb}",
    )

    ctx.expect_gap(
        primary,
        base,
        axis="z",
        min_gap=0.035,
        negative_elem="stop_face",
        name="primary link clears fixed stop pads at rest",
    )
    ctx.expect_within(
        primary,
        base,
        axes="y",
        margin=0.0,
        name="primary link sits inside clevis span",
    )
    ctx.expect_within(
        secondary,
        primary,
        axes="y",
        margin=0.0,
        name="secondary lug sits inside primary fork span",
    )

    rest_tab = ctx.part_element_world_aabb(secondary, elem="tool_tab")
    with ctx.pose({shoulder: 0.85, elbow: 0.65}):
        raised_tab = ctx.part_element_world_aabb(secondary, elem="tool_tab")
        ctx.expect_gap(
            secondary,
            base,
            axis="z",
            min_gap=0.060,
            positive_elem="tool_tab",
            name="raised tool tab stays above base clevis",
        )

    ctx.check(
        "terminal tab sweeps upward through clear arc",
        rest_tab is not None and raised_tab is not None and raised_tab[1][2] > rest_tab[1][2] + 0.25,
        details=f"rest_tab={rest_tab}, raised_tab={raised_tab}",
    )

    return ctx.report()


object_model = build_object_model()
