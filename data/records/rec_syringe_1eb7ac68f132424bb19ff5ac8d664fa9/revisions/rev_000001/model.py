from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BARREL_LENGTH = 0.095
BARREL_OUTER_RADIUS = 0.0090
BARREL_INNER_RADIUS = 0.00755
PLUNGER_TRAVEL = 0.055


def _x_cylinder(radius: float, length: float, x_center: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(x_center - (length / 2.0), 0.0, 0.0))
        .circle(radius)
        .extrude(length)
    )


def _x_annulus(
    outer_radius: float,
    inner_radius: float,
    length: float,
    x_center: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(x_center - (length / 2.0), 0.0, 0.0))
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def _x_frustum(
    x0: float,
    x1: float,
    r0: float,
    r1: float,
    *,
    bore_radius: float | None = None,
) -> cq.Workplane:
    length = x1 - x0
    shape = (
        cq.Workplane("YZ", origin=(x0, 0.0, 0.0))
        .circle(r0)
        .workplane(offset=length)
        .circle(r1)
        .loft(combine=True, ruled=False)
    )
    if bore_radius is not None and bore_radius > 0.0:
        bore = _x_cylinder(bore_radius, length + 0.004, (x0 + x1) / 2.0)
        shape = shape.cut(bore)
    return shape


def _box(
    sx: float,
    sy: float,
    sz: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _build_barrel_shell() -> cq.Workplane:
    tube = _x_annulus(
        BARREL_OUTER_RADIUS,
        BARREL_INNER_RADIUS,
        BARREL_LENGTH,
        BARREL_LENGTH / 2.0,
    )
    front_shoulder = _x_frustum(
        BARREL_LENGTH - 0.001,
        0.111,
        BARREL_OUTER_RADIUS,
        0.0040,
        bore_radius=0.00155,
    )
    luer_sleeve = _x_annulus(0.00535, 0.00305, 0.014, 0.113)
    nozzle_taper = _x_frustum(0.117, 0.139, 0.00340, 0.00135, bore_radius=0.00055)
    rear_lip = _x_annulus(0.00955, BARREL_INNER_RADIUS, 0.0032, 0.0016)
    front_lip = _x_annulus(0.00945, 0.0052, 0.0026, 0.0948)

    return (
        tube.union(front_shoulder)
        .union(luer_sleeve)
        .union(nozzle_taper)
        .union(rear_lip)
        .union(front_lip)
    )


def _build_rear_hardware() -> cq.Workplane:
    guide_collar = _x_annulus(0.0072, 0.0049, 0.014, -0.004)
    finger_ring = _x_annulus(0.0115, 0.0065, 0.0052, -0.0045)

    wing_span = 0.019
    wing_a = _box(0.0052, wing_span, 0.0105, (-0.0045, 0.019, 0.0))
    wing_b = _box(0.0052, wing_span, 0.0105, (-0.0045, -0.019, 0.0))
    wing_a = wing_a.edges().fillet(0.0018)
    wing_b = wing_b.edges().fillet(0.0018)

    side_gusset_a = _box(0.0060, 0.010, 0.0055, (-0.0025, 0.010, 0.0))
    side_gusset_b = _box(0.0060, 0.010, 0.0055, (-0.0025, -0.010, 0.0))

    return (
        guide_collar.union(finger_ring)
        .union(wing_a)
        .union(wing_b)
        .union(side_gusset_a)
        .union(side_gusset_b)
    )


def _build_seam_bands() -> cq.Workplane:
    rear_band = _x_annulus(0.00985, 0.00885, 0.0010, 0.007)
    front_band = _x_annulus(0.00985, 0.00885, 0.0010, 0.089)
    luer_band = _x_annulus(0.00565, 0.00495, 0.0011, 0.1065)
    return rear_band.union(front_band).union(luer_band)


def _build_graduation_marks() -> cq.Workplane:
    marks = _box(0.074, 0.00045, 0.00028, (0.014 + 0.037, -0.0061, 0.00908))

    for index in range(21):
        x = 0.014 + index * 0.00355
        if index % 5 == 0:
            tick_len = 0.0122
            y_center = -0.0002
            tick_width = 0.00055
        elif index % 2 == 0:
            tick_len = 0.0086
            y_center = -0.0020
            tick_width = 0.00042
        else:
            tick_len = 0.0065
            y_center = -0.00305
            tick_width = 0.00036
        tick = _box(tick_width, tick_len, 0.00030, (x, y_center, 0.00911))
        marks = marks.union(tick)

    # Small connected dose-index blocks suggest printed numerals without
    # introducing floating letter islands.
    for x in (0.0315, 0.0493, 0.0670, 0.0848):
        label = _box(0.0014, 0.0014, 0.00030, (x, 0.0061, 0.00912))
        leader = _box(0.00042, 0.0041, 0.00030, (x, 0.0038, 0.00912))
        marks = marks.union(label).union(leader)
    return marks


def _build_plunger_rod() -> cq.Workplane:
    # A cruciform guided rod: broad enough to read mechanically, slim enough to
    # clear the rear guide and barrel bore.
    length = 0.145
    x_center = 0.0055
    vertical_web = _box(length, 0.00145, 0.0062, (x_center, 0.0, 0.0))
    horizontal_web = _box(length, 0.0062, 0.00145, (x_center, 0.0, 0.0))
    central_spine = _x_cylinder(0.00155, length, x_center)
    front_boss = _x_cylinder(0.0037, 0.0065, 0.078)
    rear_boss = _x_cylinder(0.0035, 0.0080, -0.062)
    return (
        vertical_web.union(horizontal_web)
        .union(central_spine)
        .union(front_boss)
        .union(rear_boss)
    )


def _build_stopper() -> cq.Workplane:
    core = _x_cylinder(0.00675, 0.0105, 0.084)
    rear_wiper = _x_cylinder(0.00758, 0.0018, 0.0795)
    front_wiper = _x_cylinder(0.00758, 0.0018, 0.0885)
    center_waist = _x_cylinder(0.00635, 0.0055, 0.084)
    return core.union(rear_wiper).union(front_wiper).union(center_waist)


def _build_thumb_pad() -> cq.Workplane:
    pad = _x_cylinder(0.0107, 0.0056, -0.067)
    front_rim = _x_cylinder(0.0112, 0.0012, -0.0642)
    rear_rim = _x_cylinder(0.0112, 0.0012, -0.0698)
    shallow_dish = _x_cylinder(0.0070, 0.0008, -0.0707)
    return pad.union(front_rim).union(rear_rim).union(shallow_dish)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_clinical_syringe")

    clear_polycarbonate = model.material(
        "clear_polycarbonate",
        rgba=(0.66, 0.87, 0.96, 0.42),
    )
    frosted_satin = model.material("frosted_satin", rgba=(0.83, 0.86, 0.88, 1.0))
    ink = model.material("dose_ink", rgba=(0.035, 0.045, 0.050, 1.0))
    graphite = model.material("matte_graphite", rgba=(0.10, 0.11, 0.12, 1.0))
    plunger_plastic = model.material(
        "satin_plunger_plastic",
        rgba=(0.78, 0.81, 0.84, 1.0),
    )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel_shell(), "barrel_shell", tolerance=0.00035),
        material=clear_polycarbonate,
        name="barrel_shell",
    )
    barrel.visual(
        mesh_from_cadquery(_build_rear_hardware(), "rear_hardware", tolerance=0.00035),
        material=frosted_satin,
        name="rear_hardware",
    )
    barrel.visual(
        mesh_from_cadquery(_build_seam_bands(), "seam_bands", tolerance=0.00030),
        material=frosted_satin,
        name="seam_bands",
    )
    barrel.visual(
        mesh_from_cadquery(
            _build_graduation_marks(),
            "graduation_marks",
            tolerance=0.00018,
        ),
        material=ink,
        name="graduation_marks",
    )

    plunger = model.part("plunger")
    plunger.visual(
        mesh_from_cadquery(_build_plunger_rod(), "plunger_rod", tolerance=0.00035),
        material=plunger_plastic,
        name="plunger_rod",
    )
    plunger.visual(
        mesh_from_cadquery(_build_stopper(), "stopper", tolerance=0.00025),
        material=graphite,
        name="stopper",
    )
    plunger.visual(
        mesh_from_cadquery(_build_thumb_pad(), "thumb_pad", tolerance=0.00035),
        material=plunger_plastic,
        name="thumb_pad",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=PLUNGER_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="barrel_shell",
        elem_b="stopper",
        reason=(
            "The elastomer stopper wiper is intentionally modeled with a tiny "
            "radial compression against the transparent barrel bore to show the seal."
        ),
    )

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="stopper",
        outer_elem="barrel_shell",
        margin=0.0005,
        name="stopper is guided inside the barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="stopper",
        elem_b="barrel_shell",
        min_overlap=0.006,
        name="depressed stopper remains in the graduated barrel",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: PLUNGER_TRAVEL}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="stopper",
            outer_elem="barrel_shell",
            margin=0.0005,
            name="withdrawn stopper remains coaxially guided",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="stopper",
            elem_b="barrel_shell",
            min_overlap=0.006,
            name="withdrawn stopper retains barrel insertion",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger travel moves rearward along barrel axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] < rest_pos[0] - (PLUNGER_TRAVEL * 0.95),
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
