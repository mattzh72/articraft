from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _mixing_bowl_mesh() -> MeshGeometry:
    outer = [
        (0.045, 0.000),
        (0.075, 0.020),
        (0.118, 0.082),
        (0.148, 0.150),
        (0.162, 0.205),
        (0.166, 0.222),
    ]
    inner = [
        (0.034, 0.012),
        (0.065, 0.033),
        (0.105, 0.091),
        (0.134, 0.156),
        (0.149, 0.205),
        (0.151, 0.214),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def _whisk_wire(angle: float) -> MeshGeometry:
    c = cos(angle)
    s = sin(angle)
    points = [
        (0.0, 0.0, -0.010),
        (0.042 * c, 0.042 * s, -0.060),
        (0.055 * c, 0.055 * s, -0.125),
        (0.018 * c, 0.018 * s, -0.185),
        (0.0, 0.0, -0.205),
    ]
    return tube_from_spline_points(
        points,
        radius=0.0022,
        samples_per_segment=16,
        radial_segments=10,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer")

    enamel = model.material("deep_red_enamel", rgba=(0.62, 0.035, 0.030, 1.0))
    enamel_shadow = model.material("dark_red_shadow", rgba=(0.34, 0.018, 0.016, 1.0))
    stainless = model.material("polished_stainless", rgba=(0.82, 0.83, 0.80, 1.0))
    chrome = model.material("bright_chrome", rgba=(0.72, 0.74, 0.76, 1.0))
    black = model.material("black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    cream = model.material("cream_badge", rgba=(0.92, 0.86, 0.68, 1.0))
    dark = model.material("dark_slot", rgba=(0.08, 0.08, 0.085, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.58, 0.38, 0.08)),
        origin=Origin(xyz=(0.02, 0.0, 0.04)),
        material=enamel,
        name="pedestal_base",
    )
    base.visual(Box((0.48, 0.30, 0.035)), origin=Origin(xyz=(0.075, 0.0, 0.098)), material=enamel_shadow, name="raised_deck")
    base.visual(Box((0.15, 0.24, 0.38)), origin=Origin(xyz=(-0.18, 0.0, 0.270)), material=enamel, name="rear_column")
    base.visual(Box((0.18, 0.050, 0.08)), origin=Origin(xyz=(-0.18, 0.120, 0.475)), material=enamel, name="hinge_cheek_0")
    base.visual(Box((0.18, 0.050, 0.08)), origin=Origin(xyz=(-0.18, -0.120, 0.475)), material=enamel, name="hinge_cheek_1")
    base.visual(Cylinder(radius=0.034, length=0.080), origin=Origin(xyz=(-0.18, 0.120, 0.500), rpy=(-pi / 2.0, 0.0, 0.0)), material=chrome, name="hinge_knuckle_0")
    base.visual(Cylinder(radius=0.034, length=0.080), origin=Origin(xyz=(-0.18, -0.120, 0.500), rpy=(-pi / 2.0, 0.0, 0.0)), material=chrome, name="hinge_knuckle_1")
    base.visual(Cylinder(radius=0.010, length=0.32), origin=Origin(xyz=(0.075, 0.095, 0.112), rpy=(0.0, pi / 2.0, 0.0)), material=chrome, name="slide_rail_0")
    base.visual(Cylinder(radius=0.010, length=0.32), origin=Origin(xyz=(0.075, -0.095, 0.112), rpy=(0.0, pi / 2.0, 0.0)), material=chrome, name="slide_rail_1")
    base.visual(Box((0.020, 0.24, 0.035)), origin=Origin(xyz=(-0.080, 0.0, 0.126)), material=dark, name="rear_slide_stop")
    base.visual(Box((0.020, 0.24, 0.035)), origin=Origin(xyz=(0.320, 0.0, 0.126)), material=dark, name="front_slide_stop")
    base.visual(Box((0.095, 0.006, 0.026)), origin=Origin(xyz=(-0.180, -0.121, 0.260)), material=dark, name="speed_gate")
    base.visual(Box((0.080, 0.007, 0.030)), origin=Origin(xyz=(-0.180, 0.114, 0.275)), material=dark, name="lock_recess")
    base.visual(Box((0.115, 0.010, 0.035)), origin=Origin(xyz=(0.100, -0.185, 0.070)), material=cream, name="artisan_badge")
    for x in (-0.20, 0.24):
        for y in (-0.13, 0.13):
            base.visual(Cylinder(radius=0.030, length=0.018), origin=Origin(xyz=(x, y, 0.009)), material=black, name=f"rubber_foot_{x}_{y}")

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(Box((0.30, 0.22, 0.032)), origin=Origin(xyz=(0.070, 0.0, 0.016)), material=chrome, name="slide_saddle")
    bowl_carriage.visual(Box((0.16, 0.025, 0.045)), origin=Origin(xyz=(0.125, 0.118, 0.048)), material=chrome, name="bowl_clip_0")
    bowl_carriage.visual(Box((0.16, 0.025, 0.045)), origin=Origin(xyz=(0.125, -0.118, 0.048)), material=chrome, name="bowl_clip_1")
    bowl_carriage.visual(Cylinder(radius=0.060, length=0.020), origin=Origin(xyz=(0.125, 0.0, 0.040)), material=chrome, name="bowl_foot_socket")
    bowl_carriage.visual(_mesh(_mixing_bowl_mesh(), "polished_mixing_bowl"), origin=Origin(xyz=(0.125, 0.0, 0.041)), material=stainless, name="mixing_bowl")

    head = model.part("tilt_head")
    head.visual(_mesh(CapsuleGeometry(radius=0.085, length=0.245).rotate_y(pi / 2.0), "rounded_head_shell"), origin=Origin(xyz=(0.310, 0.0, 0.030)), material=enamel, name="rounded_head_shell")
    head.visual(Cylinder(radius=0.035, length=0.160), origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)), material=chrome, name="center_hinge_barrel")
    head.visual(Box((0.150, 0.140, 0.070)), origin=Origin(xyz=(0.080, 0.0, 0.020)), material=enamel_shadow, name="rear_hinge_lug")
    head.visual(Cylinder(radius=0.052, length=0.030), origin=Origin(xyz=(0.510, 0.0, 0.030), rpy=(0.0, pi / 2.0, 0.0)), material=chrome, name="front_trim_band")
    head.visual(Cylinder(radius=0.036, length=0.060), origin=Origin(xyz=(0.335, 0.0, -0.080)), material=chrome, name="drive_hub")
    head.visual(Box((0.070, 0.012, 0.030)), origin=Origin(xyz=(0.220, -0.078, 0.075)), material=cream, name="side_name_plate")

    whisk = model.part("whisk")
    whisk.visual(Cylinder(radius=0.006, length=0.120), origin=Origin(xyz=(0.0, 0.0, -0.060)), material=stainless, name="drive_shaft")
    whisk.visual(Cylinder(radius=0.026, length=0.032), origin=Origin(xyz=(0.0, 0.0, -0.016)), material=chrome, name="upper_collar")
    whisk.visual(Cylinder(radius=0.018, length=0.026), origin=Origin(xyz=(0.0, 0.0, -0.204)), material=chrome, name="lower_collar")
    for i in range(6):
        whisk.visual(_mesh(_whisk_wire(2.0 * pi * i / 6.0), f"whisk_wire_{i}"), material=stainless, name=f"wire_loop_{i}")

    speed_lever = model.part("speed_lever")
    speed_lever.visual(Cylinder(radius=0.020, length=0.020), origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)), material=chrome, name="lever_pivot")
    speed_lever.visual(Box((0.018, 0.014, 0.026)), origin=Origin(xyz=(0.0, 0.003, 0.0)), material=chrome, name="lever_stem")
    speed_lever.visual(Box((0.020, 0.016, 0.112)), origin=Origin(xyz=(0.025, -0.018, 0.056)), material=chrome, name="lever_blade")
    speed_lever.visual(Cylinder(radius=0.017, length=0.026), origin=Origin(xyz=(0.047, -0.020, 0.112), rpy=(-pi / 2.0, 0.0, 0.0)), material=black, name="lever_knob")

    lock_button = model.part("lock_button")
    lock_button.visual(Cylinder(radius=0.019, length=0.026), origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material=chrome, name="push_button")
    lock_button.visual(Box((0.022, 0.010, 0.026)), origin=Origin(xyz=(0.0, -0.002, 0.0)), material=chrome, name="button_stem")

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.040, 0.0, 0.122)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.20, lower=0.0, upper=0.055),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.180, 0.0, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.6, lower=0.0, upper=1.05),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.335, 0.0, -0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )
    model.articulation(
        "base_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(-0.180, -0.131, 0.260)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.4, velocity=2.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "base_to_lock_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_button,
        origin=Origin(xyz=(-0.180, 0.126, 0.275)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.014),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("tilt_head")
    whisk = object_model.get_part("whisk")
    speed_lever = object_model.get_part("speed_lever")
    lock_button = object_model.get_part("lock_button")

    slide = object_model.get_articulation("base_to_bowl_carriage")
    hinge = object_model.get_articulation("base_to_head")
    spin = object_model.get_articulation("head_to_whisk")
    speed = object_model.get_articulation("base_to_speed_lever")
    lock = object_model.get_articulation("base_to_lock_button")

    ctx.check("bowl carriage is prismatic", slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tilt head is revolute", hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("whisk drive is continuous", spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("speed lever is revolute", speed.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("lock button is prismatic", lock.articulation_type == ArticulationType.PRISMATIC)

    ctx.expect_within(
        whisk,
        carriage,
        axes="xy",
        inner_elem="lower_collar",
        outer_elem="mixing_bowl",
        margin=0.010,
        name="whisk lower collar sits inside bowl footprint",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="slide_saddle",
        elem_b="slide_rail_0",
        min_overlap=0.16,
        name="bowl carriage remains carried on slide rails",
    )

    def _center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.055}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="slide_saddle",
            elem_b="slide_rail_0",
            min_overlap=0.12,
            name="extended bowl carriage is still retained on rails",
        )
    ctx.check(
        "bowl carriage slides forward",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.050,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_band = _center(ctx.part_element_world_aabb(head, elem="front_trim_band"))
    with ctx.pose({hinge: 0.85}):
        raised_band = _center(ctx.part_element_world_aabb(head, elem="front_trim_band"))
    ctx.check(
        "tilt head raises above bowl",
        rest_band is not None and raised_band is not None and raised_band[2] > rest_band[2] + 0.20,
        details=f"rest={rest_band}, raised={raised_band}",
    )

    rest_button = ctx.part_world_position(lock_button)
    with ctx.pose({lock: 0.014}):
        pressed_button = ctx.part_world_position(lock_button)
    ctx.check(
        "head lock button presses inward",
        rest_button is not None and pressed_button is not None and pressed_button[1] < rest_button[1] - 0.012,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    low_lever = _center(ctx.part_element_world_aabb(speed_lever, elem="lever_knob"))
    with ctx.pose({speed: 0.45}):
        high_lever = _center(ctx.part_element_world_aabb(speed_lever, elem="lever_knob"))
    ctx.check(
        "speed lever swings on side pivot",
        low_lever is not None
        and high_lever is not None
        and abs(high_lever[0] - low_lever[0]) + abs(high_lever[2] - low_lever[2]) > 0.030,
        details=f"rest={low_lever}, moved={high_lever}",
    )

    wire_rest = _center(ctx.part_element_world_aabb(whisk, elem="wire_loop_0"))
    with ctx.pose({spin: pi / 2.0}):
        wire_spun = _center(ctx.part_element_world_aabb(whisk, elem="wire_loop_0"))
    ctx.check(
        "whisk wire rotates about drive",
        wire_rest is not None
        and wire_spun is not None
        and abs(wire_spun[0] - wire_rest[0]) + abs(wire_spun[1] - wire_rest[1]) > 0.020,
        details=f"rest={wire_rest}, spun={wire_spun}",
    )

    return ctx.report()


object_model = build_object_model()
