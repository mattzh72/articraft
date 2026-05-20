from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cylinder_between(part, name, p0, p1, radius, material):
    """Add a cylinder visual whose local Z axis spans p0 to p1."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _tube_mesh(name: str, outer_radius: float, inner_radius: float, length: float, segments: int = 48):
    half = length * 0.5
    tube = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, half), (inner_radius, -half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(tube, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garage_floor_pump")

    red = model.material("powder_coated_red", rgba=(0.82, 0.05, 0.03, 1.0))
    black = model.material("satin_black", rgba=(0.01, 0.012, 0.012, 1.0))
    rubber = model.material("molded_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    white = model.material("warm_white_dial", rgba=(0.92, 0.90, 0.82, 1.0))
    yellow = model.material("yellow_needle", rgba=(1.0, 0.78, 0.05, 1.0))

    body = model.part("body")

    # Wide garage floor base: a low stamped plate with rubber feet.
    body.visual(
        Box((0.64, 0.19, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dark_steel,
        name="foot_plate",
    )
    for i, x in enumerate((-0.24, 0.24)):
        body.visual(
            Box((0.16, 0.205, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.006)),
            material=rubber,
            name=f"rubber_pad_{i}",
        )

    # Tall barrel is an open thin-wall tube so the piston rod can remain visible
    # through the top guide instead of vanishing into a solid block.
    barrel_height = 0.66
    barrel_bottom = 0.07
    barrel_center_z = barrel_bottom + barrel_height * 0.5
    body.visual(
        _tube_mesh("barrel_shell_mesh", outer_radius=0.045, inner_radius=0.037, length=barrel_height),
        origin=Origin(xyz=(0.0, 0.0, barrel_center_z)),
        material=red,
        name="barrel_shell",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.045, tube=0.006, radial_segments=18, tubular_segments=56), "lower_barrel_ring"),
        origin=Origin(xyz=(0.0, 0.0, barrel_bottom + 0.012)),
        material=black,
        name="lower_ring",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.045, tube=0.006, radial_segments=18, tubular_segments=56), "top_barrel_ring"),
        origin=Origin(xyz=(0.0, 0.0, barrel_bottom + barrel_height)),
        material=black,
        name="top_ring",
    )
    body.visual(
        _tube_mesh("rod_guide_mesh", outer_radius=0.026, inner_radius=0.011, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, barrel_bottom + barrel_height + 0.0175)),
        material=black,
        name="rod_guide",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        z = barrel_bottom + barrel_height + 0.004
        _cylinder_between(
            body,
            f"guide_spoke_{i}",
            (0.022 * math.cos(angle), 0.022 * math.sin(angle), z),
            (0.045 * math.cos(angle), 0.045 * math.sin(angle), z),
            0.004,
            black,
        )

    # Welded base socket and two stays tie the tall tube back to the foot plate.
    body.visual(
        Cylinder(radius=0.060, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=black,
        name="base_socket",
    )
    _cylinder_between(body, "stay_0", (-0.23, 0.065, 0.032), (-0.036, 0.020, 0.42), 0.009, dark_steel)
    _cylinder_between(body, "stay_1", (0.23, 0.065, 0.032), (0.036, 0.020, 0.42), 0.009, dark_steel)

    # Gauge body, face, and side neck mounted low on the front of the pump.
    gauge_center = (0.0, -0.108, 0.225)
    _cylinder_between(body, "gauge_neck", (0.0, -0.043, 0.225), (0.0, -0.092, 0.225), 0.018, black)
    body.visual(
        Cylinder(radius=0.084, length=0.024),
        origin=Origin(xyz=(0.0, -0.096, 0.225), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="gauge_case",
    )
    body.visual(
        Cylinder(radius=0.073, length=0.004),
        origin=Origin(xyz=gauge_center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="gauge_face",
    )
    for i, angle in enumerate((-1.05, -0.52, 0.0, 0.52, 1.05)):
        body.visual(
            Box((0.004, 0.004, 0.018)),
            origin=Origin(
                xyz=(0.055 * math.sin(angle), -0.111, 0.225 + 0.055 * math.cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=black,
            name=f"gauge_tick_{i}",
        )
    body.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, -0.109, 0.225), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="pivot_boss",
    )

    # Side hose clip: a clipped black hose is visibly retained by a mounted U clip.
    body.visual(
        Box((0.018, 0.050, 0.095)),
        origin=Origin(xyz=(0.058, 0.0, 0.455)),
        material=black,
        name="clip_spine",
    )
    body.visual(
        Box((0.066, 0.018, 0.014)),
        origin=Origin(xyz=(0.091, 0.0, 0.492)),
        material=black,
        name="clip_upper_jaw",
    )
    body.visual(
        Box((0.066, 0.018, 0.014)),
        origin=Origin(xyz=(0.091, 0.0, 0.418)),
        material=black,
        name="clip_lower_jaw",
    )
    _cylinder_between(body, "clipped_hose", (0.108, 0.0, 0.285), (0.108, 0.0, 0.620), 0.010, rubber)

    # Sliding T-handle and visible piston rod.
    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.011, length=0.680),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=steel,
        name="piston_rod",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.480),
        origin=Origin(xyz=(0.0, 0.0, 0.390), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="crossbar",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=black,
        name="center_collar",
    )
    for i, x in enumerate((-0.252, 0.252)):
        handle.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(x, 0.0, 0.390)),
            material=black,
            name=f"bar_end_{i}",
        )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, barrel_bottom + barrel_height + 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.45, lower=0.0, upper=0.250),
    )

    # A folding/rotating rubber sleeve rides on the top crossbar.  The sleeve is
    # hollow and has an offset paddle so continuous rotation is visually legible.
    grip = model.part("grip")
    grip.visual(
        _tube_mesh("grip_sleeve_mesh", outer_radius=0.029, inner_radius=0.014, length=0.220),
        origin=Origin(xyz=(0.130, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="sleeve",
    )
    grip.visual(
        Box((0.185, 0.020, 0.046)),
        origin=Origin(xyz=(0.130, -0.038, 0.0)),
        material=rubber,
        name="folding_paddle",
    )
    grip.visual(
        Box((0.205, 0.010, 0.010)),
        origin=Origin(xyz=(0.130, -0.025, 0.027)),
        material=black,
        name="raised_seam",
    )

    model.articulation(
        "handle_to_grip",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=grip,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    # Gauge needle rotates about the central boss on the dial face.
    needle = model.part("needle")
    needle.visual(
        Box((0.008, 0.004, 0.066)),
        origin=Origin(xyz=(0.0, -0.004, 0.033)),
        material=yellow,
        name="pointer",
    )
    needle.visual(
        Cylinder(radius=0.008, length=0.005),
        origin=Origin(xyz=(0.0, -0.0015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=yellow,
        name="pivot_hub",
    )

    model.articulation(
        "body_to_needle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=needle,
        origin=Origin(xyz=(0.0, -0.112, 0.225)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=6.0, lower=-1.10, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    grip = object_model.get_part("grip")
    needle = object_model.get_part("needle")
    slide = object_model.get_articulation("body_to_handle")
    grip_spin = object_model.get_articulation("handle_to_grip")
    needle_sweep = object_model.get_articulation("body_to_needle")

    ctx.allow_overlap(
        body,
        handle,
        elem_a="rod_guide",
        elem_b="piston_rod",
        reason="The sliding piston rod is intentionally captured in the small top guide bushing.",
    )
    ctx.allow_overlap(
        grip,
        handle,
        elem_a="sleeve",
        elem_b="crossbar",
        reason="The rotating grip sleeve is intentionally captured around the T-handle crossbar.",
    )

    # The piston rod must remain a visible, exposed member above the open barrel
    # while retaining insertion in the barrel at the top of the stroke.
    barrel_aabb = ctx.part_element_world_aabb(body, elem="barrel_shell")
    rod_aabb = ctx.part_element_world_aabb(handle, elem="piston_rod")
    ctx.check(
        "piston rod visibly protrudes above barrel",
        barrel_aabb is not None
        and rod_aabb is not None
        and rod_aabb[1][2] > barrel_aabb[1][2] + 0.25
        and rod_aabb[0][2] < barrel_aabb[1][2],
        details=f"barrel={barrel_aabb}, rod={rod_aabb}",
    )
    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="barrel_shell",
        margin=0.001,
        name="piston rod centered inside barrel opening",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="piston_rod",
        elem_b="barrel_shell",
        min_overlap=0.020,
        name="piston rod retains insertion at rest",
    )
    ctx.expect_contact(
        handle,
        body,
        elem_a="piston_rod",
        elem_b="rod_guide",
        contact_tol=0.0015,
        name="piston rod is supported by top guide bushing",
    )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({slide: 0.250}):
        raised_handle_pos = ctx.part_world_position(handle)
        raised_rod = ctx.part_element_world_aabb(handle, elem="piston_rod")
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="piston_rod",
            elem_b="barrel_shell",
            min_overlap=0.020,
            name="piston rod remains inserted at raised stroke",
        )
        ctx.check(
            "raised handle still exposes rod",
            barrel_aabb is not None and raised_rod is not None and raised_rod[1][2] > barrel_aabb[1][2] + 0.45,
            details=f"barrel={barrel_aabb}, raised_rod={raised_rod}",
        )
    ctx.check(
        "T-handle slides upward along barrel axis",
        rest_handle_pos is not None
        and raised_handle_pos is not None
        and raised_handle_pos[2] > rest_handle_pos[2] + 0.20,
        details=f"rest={rest_handle_pos}, raised={raised_handle_pos}",
    )

    # The folding grip is a supported sleeve on the crossbar and rotates about
    # the crossbar axis.
    ctx.expect_within(
        handle,
        grip,
        axes="yz",
        inner_elem="crossbar",
        outer_elem="sleeve",
        margin=0.002,
        name="crossbar lies within hollow grip sleeve",
    )
    ctx.expect_overlap(
        grip,
        handle,
        axes="x",
        elem_a="sleeve",
        elem_b="crossbar",
        min_overlap=0.20,
        name="grip sleeve spans the top crossbar",
    )
    ctx.expect_contact(
        grip,
        handle,
        elem_a="sleeve",
        elem_b="crossbar",
        contact_tol=0.002,
        name="grip sleeve is carried by crossbar",
    )
    ctx.check(
        "grip has continuous crossbar rotation",
        grip_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(grip_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={grip_spin.articulation_type}, axis={grip_spin.axis}",
    )

    # Gauge needle is mounted by its pivot and sweeps around that central point.
    ctx.expect_contact(
        needle,
        body,
        elem_a="pivot_hub",
        elem_b="pivot_boss",
        contact_tol=0.002,
        name="needle hub sits on central gauge pivot",
    )
    with ctx.pose({needle_sweep: -1.0}):
        left_pointer = ctx.part_element_world_aabb(needle, elem="pointer")
    with ctx.pose({needle_sweep: 1.0}):
        right_pointer = ctx.part_element_world_aabb(needle, elem="pointer")
    ctx.check(
        "gauge needle sweeps across dial",
        left_pointer is not None
        and right_pointer is not None
        and abs(((right_pointer[0][0] + right_pointer[1][0]) * 0.5) - ((left_pointer[0][0] + left_pointer[1][0]) * 0.5)) > 0.045,
        details=f"left={left_pointer}, right={right_pointer}",
    )

    return ctx.report()


object_model = build_object_model()
