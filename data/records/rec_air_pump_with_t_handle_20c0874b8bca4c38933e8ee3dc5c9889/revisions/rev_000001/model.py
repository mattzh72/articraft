from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _annular_tube_z(length: float, inner_radius: float, outer_radius: float, *, segments: int = 48) -> MeshGeometry:
    """Closed annular tube centered on the local Z axis."""
    geom = MeshGeometry()
    rings = []
    for z in (-length / 2.0, length / 2.0):
        outer = []
        inner = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            ca, sa = math.cos(a), math.sin(a)
            outer.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z))
            inner.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z))
        rings.append((outer, inner))

    outer0, inner0 = rings[0]
    outer1, inner1 = rings[1]
    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        # Inner wall, reversed.
        geom.add_face(inner0[i], inner1[j], inner0[j])
        geom.add_face(inner0[i], inner1[i], inner1[j])
        # Bottom annular cap.
        geom.add_face(outer0[j], outer0[i], inner0[i])
        geom.add_face(outer0[j], inner0[i], inner0[j])
        # Top annular cap.
        geom.add_face(outer1[i], outer1[j], inner1[j])
        geom.add_face(outer1[i], inner1[j], inner1[i])
    return geom


def _annular_tube_x(length: float, inner_radius: float, outer_radius: float, *, segments: int = 48) -> MeshGeometry:
    """Closed annular tube centered on the local X axis."""
    return _annular_tube_z(length, inner_radius, outer_radius, segments=segments).rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_head_floor_pump")

    dark_metal = model.material("dark_metal", rgba=(0.03, 0.035, 0.04, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    blue = model.material("blue_barrel", rgba=(0.05, 0.20, 0.55, 1.0))
    black = model.material("black_plastic", rgba=(0.005, 0.005, 0.006, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    red = model.material("red_pointer", rgba=(0.85, 0.03, 0.02, 1.0))
    gauge_face = model.material("white_gauge_face", rgba=(0.94, 0.92, 0.84, 1.0))
    brass = model.material("brass_ports", rgba=(0.80, 0.56, 0.18, 1.0))

    pump = model.part("pump")

    # Wide workshop floor base with rubber foot pads.
    pump.visual(Box((0.58, 0.18, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), material=dark_metal, name="base_plate")
    pump.visual(Box((0.22, 0.055, 0.018)), origin=Origin(xyz=(-0.18, -0.055, 0.044)), material=rubber, name="foot_pad_0")
    pump.visual(Box((0.22, 0.055, 0.018)), origin=Origin(xyz=(0.18, -0.055, 0.044)), material=rubber, name="foot_pad_1")
    pump.visual(Cylinder(radius=0.042, length=0.055), origin=Origin(xyz=(0.0, 0.0, 0.0625)), material=dark_metal, name="bottom_socket")

    # Tall hollow barrel and its top guide collar.  The pump rod passes through
    # the clear inner bore instead of interpenetrating a solid proxy.
    barrel_shell = mesh_from_geometry(_annular_tube_z(0.86, 0.017, 0.026, segments=64), "barrel_shell")
    pump.visual(barrel_shell, origin=Origin(xyz=(0.0, 0.0, 0.51)), material=blue, name="barrel_shell")
    top_collar = mesh_from_geometry(_annular_tube_z(0.070, 0.0155, 0.036, segments=64), "top_collar")
    pump.visual(top_collar, origin=Origin(xyz=(0.0, 0.0, 0.935)), material=black, name="top_collar")
    pump.visual(Cylinder(radius=0.032, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.090)), material=black, name="lower_collar")

    # Low, forward-facing pressure gauge with a visible dial, rim, and needle.
    pump.visual(
        Cylinder(radius=0.061, length=0.018),
        origin=Origin(xyz=(0.0, -0.073, 0.225), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="gauge_rim",
    )
    pump.visual(
        Cylinder(radius=0.049, length=0.006),
        origin=Origin(xyz=(0.0, -0.086, 0.225), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_face,
        name="gauge_dial",
    )
    pump.visual(
        Box((0.004, 0.004, 0.072)),
        origin=Origin(xyz=(0.017, -0.091, 0.240), rpy=(0.0, 0.55, 0.0)),
        material=red,
        name="gauge_needle",
    )
    pump.visual(
        Box((0.042, 0.055, 0.026)),
        origin=Origin(xyz=(0.0, -0.040, 0.225)),
        material=dark_metal,
        name="gauge_bracket",
    )

    # Hose outlet, flexible hose, and a straight supported end into the chuck.
    pump.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.0, -0.054, 0.275), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hose_outlet",
    )
    hose_points = [
        (0.0, -0.089, 0.275),
        (0.035, -0.145, 0.205),
        (0.140, -0.245, 0.095),
        (0.215, -0.350, 0.073),
        (0.220, -0.370, 0.080),
    ]
    hose_mesh = mesh_from_geometry(
        tube_from_spline_points(hose_points, radius=0.0085, samples_per_segment=16, radial_segments=24, cap_ends=True),
        "flexible_hose",
    )
    pump.visual(hose_mesh, material=rubber, name="flexible_hose")

    # The moving pump handle: a long rod, central stem, metal T bar, and end caps.
    handle = model.part("handle")
    handle.visual(Cylinder(radius=0.0075, length=0.56), origin=Origin(xyz=(0.0, 0.0, -0.185)), material=brushed, name="rod")
    handle.visual(Cylinder(radius=0.016, length=0.035), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=brushed, name="top_guide")
    handle.visual(Cylinder(radius=0.013, length=0.205), origin=Origin(xyz=(0.0, 0.0, 0.172)), material=black, name="handle_stem")
    handle.visual(
        Cylinder(radius=0.0115, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 0.260), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="crossbar",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(-0.253, 0.0, 0.260), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="bar_end_0",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.253, 0.0, 0.260), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="bar_end_1",
    )

    model.articulation(
        "pump_to_handle",
        ArticulationType.PRISMATIC,
        parent=pump,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.895)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.6, lower=0.0, upper=0.32),
    )

    # A free-spinning folding grip sleeve around the T-handle crossbar.
    grip = model.part("grip")
    grip_sleeve = mesh_from_geometry(_annular_tube_x(0.220, 0.0110, 0.027, segments=64), "grip_sleeve")
    grip.visual(grip_sleeve, material=rubber, name="grip_sleeve")
    for idx, x in enumerate((-0.080, -0.040, 0.0, 0.040, 0.080)):
        ridge = mesh_from_geometry(_annular_tube_x(0.014, 0.0260, 0.030, segments=48), f"grip_ridge_{idx}")
        grip.visual(ridge, origin=Origin(xyz=(x, 0.0, 0.0)), material=black, name=f"grip_ridge_{idx}")

    model.articulation(
        "handle_to_grip",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=grip,
        origin=Origin(xyz=(0.125, 0.0, 0.260)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    # Dual-head chuck fixed to the hose end.  The hose terminates at the tail
    # barb so the valve head reads as supported rather than loose on the floor.
    valve_head = model.part("valve_head")
    valve_head.visual(Box((0.120, 0.052, 0.036)), origin=Origin(xyz=(0.010, 0.0, 0.0)), material=black, name="valve_body")
    valve_head.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(-0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hose_tail",
    )
    for idx, y in enumerate((-0.014, 0.014)):
        valve_head.visual(
            Cylinder(radius=0.009, length=0.045),
            origin=Origin(xyz=(0.088, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"port_nozzle_{idx}",
        )
        valve_head.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.1115, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"port_hole_{idx}",
        )
    valve_head.visual(
        Cylinder(radius=0.0055, length=0.070),
        origin=Origin(xyz=(-0.015, 0.0, 0.032), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="hinge_pin",
    )
    valve_head.visual(Box((0.040, 0.012, 0.025)), origin=Origin(xyz=(-0.015, -0.030, 0.025)), material=black, name="lever_mount_0")
    valve_head.visual(Box((0.040, 0.012, 0.025)), origin=Origin(xyz=(-0.015, 0.030, 0.025)), material=black, name="lever_mount_1")

    model.articulation(
        "pump_to_valve_head",
        ArticulationType.FIXED,
        parent=pump,
        child=valve_head,
        origin=Origin(xyz=(0.320, -0.370, 0.080)),
    )

    clamp_lever = model.part("clamp_lever")
    clamp_lever.visual(
        Cylinder(radius=0.0085, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="lever_knuckle",
    )
    clamp_lever.visual(Box((0.112, 0.024, 0.012)), origin=Origin(xyz=(0.055, 0.0, 0.012)), material=black, name="lever_blade")
    clamp_lever.visual(Box((0.040, 0.030, 0.018)), origin=Origin(xyz=(0.105, 0.0, 0.016)), material=red, name="thumb_pad")

    model.articulation(
        "valve_head_to_clamp_lever",
        ArticulationType.REVOLUTE,
        parent=valve_head,
        child=clamp_lever,
        origin=Origin(xyz=(-0.015, 0.0, 0.032)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pump = object_model.get_part("pump")
    handle = object_model.get_part("handle")
    grip = object_model.get_part("grip")
    valve_head = object_model.get_part("valve_head")
    clamp_lever = object_model.get_part("clamp_lever")
    pump_slide = object_model.get_articulation("pump_to_handle")
    lever_hinge = object_model.get_articulation("valve_head_to_clamp_lever")

    ctx.allow_overlap(
        pump,
        valve_head,
        elem_a="flexible_hose",
        elem_b="hose_tail",
        reason="The rubber hose end is intentionally seated over the valve-head tail barb.",
    )
    ctx.allow_overlap(
        valve_head,
        clamp_lever,
        elem_a="hinge_pin",
        elem_b="lever_knuckle",
        reason="The clamp lever knuckle is intentionally captured around the short hinge pin.",
    )
    ctx.allow_overlap(
        handle,
        pump,
        elem_a="top_guide",
        elem_b="top_collar",
        reason="The sliding top guide is intentionally nested in the pump collar bushing.",
    )
    ctx.allow_overlap(
        grip,
        handle,
        elem_a="grip_sleeve",
        elem_b="crossbar",
        reason="The rotating grip sleeve is intentionally captured around the crossbar.",
    )

    ctx.expect_overlap(handle, pump, axes="z", elem_a="rod", elem_b="barrel_shell", min_overlap=0.25, name="rod remains inserted in the barrel")
    ctx.expect_within(handle, pump, axes="xy", inner_elem="rod", outer_elem="barrel_shell", margin=0.0, name="rod is centered on pump axis")
    ctx.expect_within(handle, pump, axes="xy", inner_elem="top_guide", outer_elem="top_collar", margin=0.002, name="top guide stays in collar bushing")
    ctx.expect_overlap(handle, pump, axes="z", elem_a="top_guide", elem_b="top_collar", min_overlap=0.02, name="top guide is retained by collar")
    ctx.expect_overlap(grip, handle, axes="x", elem_a="grip_sleeve", elem_b="crossbar", min_overlap=0.19, name="grip sleeve surrounds crossbar length")
    ctx.expect_within(handle, grip, axes="yz", inner_elem="crossbar", outer_elem="grip_sleeve", margin=0.0, name="crossbar is captured in the grip sleeve")
    ctx.expect_contact(pump, valve_head, elem_a="flexible_hose", elem_b="hose_tail", contact_tol=0.004, name="hose end supports valve head tail")
    ctx.expect_contact(valve_head, clamp_lever, elem_a="hinge_pin", elem_b="lever_knuckle", contact_tol=0.004, name="clamp lever sits on hinge pin")

    rest_handle_pos = ctx.part_world_position(handle)
    rest_pad_aabb = ctx.part_element_world_aabb(clamp_lever, elem="thumb_pad")
    with ctx.pose({pump_slide: 0.32, lever_hinge: 0.95}):
        ctx.expect_overlap(handle, pump, axes="z", elem_a="rod", elem_b="barrel_shell", min_overlap=0.08, name="extended rod stays retained in barrel")
        extended_handle_pos = ctx.part_world_position(handle)
        raised_pad_aabb = ctx.part_element_world_aabb(clamp_lever, elem="thumb_pad")

    rest_pad_z = rest_pad_aabb[1][2] if rest_pad_aabb is not None else None
    raised_pad_z = raised_pad_aabb[1][2] if raised_pad_aabb is not None else None

    ctx.check(
        "handle translates upward on pump axis",
        rest_handle_pos is not None and extended_handle_pos is not None and extended_handle_pos[2] > rest_handle_pos[2] + 0.30,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )
    ctx.check(
        "clamp lever lifts its thumb pad",
        rest_pad_z is not None and raised_pad_z is not None and raised_pad_z > rest_pad_z + 0.05,
        details=f"rest_z={rest_pad_z}, raised_z={raised_pad_z}",
    )

    return ctx.report()


object_model = build_object_model()
