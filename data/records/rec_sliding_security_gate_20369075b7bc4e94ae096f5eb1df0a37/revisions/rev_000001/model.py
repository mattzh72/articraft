from __future__ import annotations

from math import atan2, pi, sqrt

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
)


STEEL_DARK = Material("weathered_dark_green_steel", rgba=(0.09, 0.16, 0.13, 1.0))
GALV = Material("galvanized_worn_track", rgba=(0.55, 0.58, 0.55, 1.0))
BLACK = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
RUST = Material("oxidized_bolt_heads", rgba=(0.55, 0.25, 0.10, 1.0))
HATCH = Material("faded_service_hatch_paint", rgba=(0.72, 0.66, 0.48, 1.0))
WARNING = Material("aged_yellow_warning_edges", rgba=(0.90, 0.68, 0.12, 1.0))


def _box(part, name: str, size, xyz, material=STEEL_DARK, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, material=GALV, rpy=(0.0, 0.0, 0.0)):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _diagonal(part, name: str, x1: float, z1: float, x2: float, z2: float, *, y: float = 0.0):
    dx = x2 - x1
    dz = z2 - z1
    length = sqrt(dx * dx + dz * dz)
    angle = -atan2(dz, dx)
    _box(
        part,
        name,
        (length, 0.052, 0.055),
        ((x1 + x2) * 0.5, y, (z1 + z2) * 0.5),
        material=STEEL_DARK,
        rpy=(0.0, angle, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_sliding_security_gate")

    fixed = model.part("fixed_frame")

    # Continuous ground beam and old retrofit track plates: all fixed hardware is
    # tied together by this sill so the installation reads as one anchored base.
    _box(fixed, "concrete_sill", (5.55, 0.64, 0.06), (0.80, 0.0, 0.03), material=Material("aged_concrete", rgba=(0.38, 0.37, 0.34, 1.0)))
    _box(fixed, "lower_channel", (5.30, 0.16, 0.045), (0.80, 0.0, 0.0825), material=GALV)
    _box(fixed, "v_track", (5.30, 0.045, 0.075), (0.80, 0.0, 0.1425), material=GALV)
    _box(fixed, "front_track_lip", (5.30, 0.030, 0.085), (0.80, -0.095, 0.1325), material=GALV)
    _box(fixed, "rear_track_lip", (5.30, 0.030, 0.085), (0.80, 0.095, 0.1325), material=GALV)

    # Bolted adapter pads stitch the modern rail to the older base beam.
    for idx, x in enumerate([-1.45, -0.65, 0.15, 0.95, 1.75, 2.55, 3.25]):
        _box(fixed, f"track_adapter_{idx}", (0.20, 0.25, 0.018), (x, 0.0, 0.136), material=GALV)
        for y in (-0.085, 0.085):
            _cyl(fixed, f"track_bolt_{idx}_{'front' if y < 0 else 'rear'}", 0.018, 0.012, (x, y * 1.4, 0.160), material=RUST)

    # Receiving post, service/motor pedestal, and full-height guide tower.
    _box(fixed, "latch_post", (0.14, 0.18, 2.22), (-1.86, 0.0, 1.17), material=STEEL_DARK)
    _box(fixed, "guide_tower", (0.16, 0.20, 2.28), (3.35, 0.0, 1.20), material=STEEL_DARK)
    _box(fixed, "motor_pedestal", (0.68, 0.44, 0.08), (2.55, -0.34, 0.10), material=GALV)
    _box(fixed, "drive_cabinet", (0.62, 0.36, 0.72), (2.55, -0.34, 0.50), material=Material("olive_control_cabinet", rgba=(0.20, 0.25, 0.18, 1.0)))
    _box(fixed, "cabinet_cap", (0.68, 0.42, 0.045), (2.55, -0.34, 0.8825), material=GALV)
    _box(fixed, "conduit", (1.55, 0.050, 0.050), (1.82, -0.34, 0.17), material=BLACK)
    _box(fixed, "drive_sprocket_guard", (0.20, 0.050, 0.22), (2.14, -0.145, 0.33), material=GALV)

    # Upper capture channel: guide rollers constrain the leaf laterally while the
    # channel runs the full travel between the legacy post and the modern tower.
    _box(fixed, "upper_channel_spine", (5.20, 0.22, 0.055), (0.75, 0.0, 2.255), material=GALV)
    _box(fixed, "front_upper_lip", (5.20, 0.035, 0.24), (0.75, -0.110, 2.135), material=GALV)
    _box(fixed, "rear_upper_lip", (5.20, 0.035, 0.24), (0.75, 0.110, 2.135), material=GALV)
    for idx, x in enumerate([-1.25, 0.55, 2.35]):
        _box(fixed, f"upper_roller_bracket_{idx}_front", (0.12, 0.030, 0.20), (x, -0.092, 2.03), material=GALV)
        _box(fixed, f"upper_roller_bracket_{idx}_rear", (0.12, 0.030, 0.20), (x, 0.092, 2.03), material=GALV)
        _cyl(fixed, f"upper_roller_{idx}_front", 0.025, 0.17, (x, -0.066, 2.03), material=BLACK)
        _cyl(fixed, f"upper_roller_{idx}_rear", 0.025, 0.17, (x, 0.066, 2.03), material=BLACK)

    # End stops and latch receiver geometry.
    _box(fixed, "closed_end_stop", (0.070, 0.18, 0.20), (-1.755, 0.0, 0.55), material=BLACK)
    _box(fixed, "travel_end_stop", (0.090, 0.18, 0.55), (3.145, 0.0, 0.455), material=BLACK)
    _box(fixed, "latch_receiver_plate", (0.080, 0.22, 0.24), (-1.82, 0.0, 1.22), material=GALV)
    _box(fixed, "latch_receiver_top_cheek", (0.11, 0.16, 0.035), (-1.785, 0.0, 1.355), material=GALV)
    _box(fixed, "latch_receiver_bottom_cheek", (0.11, 0.16, 0.035), (-1.785, 0.0, 1.085), material=GALV)

    gate = model.part("gate_leaf")
    # Rigid sliding leaf: rectangular old-school tube frame with modern service
    # plates and reinforcements fastened onto the face.
    _box(gate, "bottom_tube", (3.30, 0.080, 0.120), (0.0, 0.0, 0.330), material=STEEL_DARK)
    _box(gate, "top_tube", (3.30, 0.080, 0.120), (0.0, 0.0, 2.050), material=STEEL_DARK)
    _box(gate, "latch_stile", (0.120, 0.080, 1.84), (-1.65, 0.0, 1.190), material=STEEL_DARK)
    _box(gate, "tail_stile", (0.120, 0.080, 1.84), (1.65, 0.0, 1.190), material=STEEL_DARK)
    _box(gate, "center_spine", (0.085, 0.075, 1.72), (0.0, 0.0, 1.190), material=STEEL_DARK)

    for idx, x in enumerate([-1.28, -0.96, -0.64, -0.32, 0.32, 0.64, 0.96, 1.28]):
        _box(gate, f"picket_{idx}", (0.040, 0.045, 1.60), (x, 0.0, 1.190), material=STEEL_DARK)

    _diagonal(gate, "diagonal_flat_0", -1.58, 0.39, -0.08, 1.98)
    _diagonal(gate, "diagonal_flat_1", 0.08, 1.98, 1.58, 0.39)

    for idx, (x, z) in enumerate([(-1.50, 0.47), (-1.50, 1.93), (1.50, 0.47), (1.50, 1.93), (0.0, 0.47), (0.0, 1.93)]):
        _box(gate, f"corner_gusset_{idx}", (0.30, 0.016, 0.22), (x, 0.045, z), material=GALV)
        for bx in (-0.085, 0.085):
            _cyl(gate, f"gusset_bolt_{idx}_{'a' if bx < 0 else 'b'}", 0.014, 0.016, (x + bx, 0.056, z), material=RUST, rpy=(pi / 2, 0.0, 0.0))

    # Service hatches on the leaf remain part of the rigid leaf but read as
    # accessible bolted covers for retrofit drives and sensors.
    for idx, x in enumerate([-0.58, 0.78]):
        _box(gate, f"leaf_service_hatch_{idx}", (0.50, 0.018, 0.34), (x, 0.0315, 1.24), material=HATCH)
        _box(gate, f"hatch_hinge_strip_{idx}", (0.035, 0.020, 0.38), (x - 0.267, 0.050, 1.24), material=GALV)
        _box(gate, f"hatch_pull_tab_{idx}", (0.060, 0.030, 0.090), (x + 0.185, 0.055, 1.24), material=BLACK)
        for bx in (-0.19, 0.19):
            for bz in (-0.125, 0.125):
                _cyl(gate, f"hatch_bolt_{idx}_{bx}_{bz}", 0.012, 0.016, (x + bx, 0.045, 1.24 + bz), material=RUST, rpy=(pi / 2, 0.0, 0.0))

    # Lower wheel trucks are welded/bolted to the leaf and sit on the fixed V rail.
    for idx, x in enumerate([-1.10, 1.10]):
        _box(gate, f"wheel_truck_{idx}", (0.34, 0.12, 0.080), (x, 0.0, 0.260), material=GALV)
        _box(gate, f"wheel_fork_{idx}_front", (0.28, 0.020, 0.17), (x, -0.060, 0.242), material=GALV)
        _box(gate, f"wheel_fork_{idx}_rear", (0.28, 0.020, 0.17), (x, 0.060, 0.242), material=GALV)
        _cyl(gate, f"roller_wheel_{idx}", 0.090, 0.090, (x, 0.0, 0.270), material=BLACK, rpy=(pi / 2, 0.0, 0.0))
        _cyl(gate, f"wheel_axle_{idx}", 0.018, 0.15, (x, 0.0, 0.270), material=GALV, rpy=(pi / 2, 0.0, 0.0))

    _box(gate, "latch_tongue", (0.140, 0.055, 0.120), (-1.705, 0.0, 1.22), material=GALV)
    _box(gate, "latch_wear_plate", (0.020, 0.11, 0.18), (-1.632, 0.0, 1.22), material=RUST)
    _box(gate, "warning_edge", (0.035, 0.090, 1.52), (-1.705, -0.002, 1.18), material=WARNING)

    service_hatch = model.part("service_hatch")
    # Child frame is the vertical hinge line on the cabinet face; panel extends
    # in +X, so positive yaw swings the free edge outward from the cabinet.
    _box(service_hatch, "hatch_panel", (0.46, 0.026, 0.46), (0.23, 0.0, 0.0), material=HATCH)
    _box(service_hatch, "hatch_rib", (0.40, 0.014, 0.035), (0.25, 0.020, -0.15), material=GALV)
    _box(service_hatch, "hatch_handle", (0.070, 0.045, 0.13), (0.38, 0.035, 0.0), material=BLACK)
    _cyl(service_hatch, "hinge_barrel", 0.018, 0.52, (0.0, 0.028, 0.0), material=GALV)
    for idx, z in enumerate([-0.17, 0.17]):
        _cyl(service_hatch, f"hatch_screw_{idx}", 0.011, 0.016, (0.39, 0.018, z), material=RUST, rpy=(pi / 2, 0.0, 0.0))

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.45, lower=0.0, upper=1.40),
    )

    model.articulation(
        "cabinet_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=service_hatch,
        origin=Origin(xyz=(2.32, -0.147, 0.52)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_frame")
    gate = object_model.get_part("gate_leaf")
    service_hatch = object_model.get_part("service_hatch")
    slide = object_model.get_articulation("gate_slide")
    hatch_hinge = object_model.get_articulation("cabinet_hatch_hinge")

    ctx.expect_contact(
        gate,
        fixed,
        elem_a="roller_wheel_0",
        elem_b="v_track",
        contact_tol=0.002,
        name="front wheel rides the fixed v track",
    )
    ctx.expect_contact(
        gate,
        fixed,
        elem_a="roller_wheel_1",
        elem_b="v_track",
        contact_tol=0.002,
        name="rear wheel rides the fixed v track",
    )
    ctx.expect_gap(
        gate,
        fixed,
        axis="x",
        positive_elem="latch_tongue",
        negative_elem="latch_receiver_plate",
        min_gap=0.0,
        max_gap=0.020,
        name="latch tongue parks at receiver",
    )
    ctx.expect_gap(
        service_hatch,
        fixed,
        axis="y",
        positive_elem="hatch_panel",
        negative_elem="drive_cabinet",
        min_gap=0.0,
        max_gap=0.003,
        name="cabinet service hatch sits on cabinet face",
    )

    rest_pos = ctx.part_world_position(gate)
    with ctx.pose({slide: 1.40}):
        ctx.expect_within(
            gate,
            fixed,
            axes="x",
            inner_elem="roller_wheel_1",
            outer_elem="v_track",
            margin=0.0,
            name="extended wheel remains over track length",
        )
        ctx.expect_overlap(
            gate,
            fixed,
            axes="x",
            elem_a="top_tube",
            elem_b="upper_channel_spine",
            min_overlap=1.0,
            name="upper channel still captures extended leaf",
        )
        extended_pos = ctx.part_world_position(gate)
    ctx.check(
        "gate slide opens along track",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 1.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    closed_aabb = ctx.part_world_aabb(service_hatch)
    with ctx.pose({hatch_hinge: 1.0}):
        open_aabb = ctx.part_world_aabb(service_hatch)
    ctx.check(
        "service hatch swings outward",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][1] > closed_aabb[1][1] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
