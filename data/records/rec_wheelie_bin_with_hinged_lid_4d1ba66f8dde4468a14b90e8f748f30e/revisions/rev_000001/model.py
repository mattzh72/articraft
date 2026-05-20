from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    MeshGeometry,
)


def _add_loop(mesh: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [mesh.add_vertex(x, y, z) for x, y, z in points]


def _connect_loops(mesh: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            mesh.add_face(a[i], b[i], a[j])
            mesh.add_face(a[j], b[i], b[j])
        else:
            mesh.add_face(a[i], a[j], b[i])
            mesh.add_face(a[j], b[j], b[i])


def _cap_loop(mesh: MeshGeometry, loop: list[int], center: tuple[float, float, float], *, flip: bool = False) -> None:
    c = mesh.add_vertex(*center)
    count = len(loop)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            mesh.add_face(c, loop[j], loop[i])
        else:
            mesh.add_face(c, loop[i], loop[j])


def _rounded_loop(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _bin_shell_mesh() -> MeshGeometry:
    """Open, tapered, thick-walled wheelie-bin tub with a raised interior floor."""
    mesh = MeshGeometry()

    wall = 0.040
    outer_specs = (
        (0.48, 0.50, 0.060, 0.140),
        (0.57, 0.61, 0.075, 0.570),
        (0.66, 0.72, 0.090, 1.050),
    )
    inner_specs = tuple(
        (w - 2.0 * wall, d - 2.0 * wall, max(0.020, r - wall * 0.55), z + (0.050 if i == 0 else -0.020))
        for i, (w, d, r, z) in enumerate(outer_specs)
    )

    outer_loops = [_add_loop(mesh, _rounded_loop(*spec)) for spec in outer_specs]
    inner_loops = [_add_loop(mesh, _rounded_loop(*spec)) for spec in inner_specs]

    for lower, upper in zip(outer_loops, outer_loops[1:]):
        _connect_loops(mesh, lower, upper)
    for lower, upper in zip(inner_loops, inner_loops[1:]):
        _connect_loops(mesh, upper, lower, flip=True)

    # Rounded top lip ring and sloped molded bottom pan.
    _connect_loops(mesh, outer_loops[-1], inner_loops[-1])
    _connect_loops(mesh, inner_loops[0], outer_loops[0], flip=True)
    _cap_loop(mesh, inner_loops[0], (0.0, 0.0, inner_specs[0][3]), flip=True)

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_wheelie_bin")

    bin_green = model.material("molded_green_hdpe", rgba=(0.03, 0.26, 0.16, 1.0))
    darker_green = model.material("darker_service_plastic", rgba=(0.02, 0.17, 0.11, 1.0))
    black_rubber = model.material("replaceable_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    yellow = model.material("yellow_service_labels", rgba=(0.95, 0.70, 0.08, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.61, 1.0))

    body = model.part("bin_body")
    body.visual(
        mesh_from_geometry(_bin_shell_mesh(), "open_tapered_bin_shell"),
        material=bin_green,
        name="tapered_shell",
    )

    # Chunky replaceable top rim: these rails sit proud of, and overlap into, the molded shell lip.
    body.visual(Box((0.75, 0.055, 0.060)), origin=Origin(xyz=(0.0, 0.372, 1.070)), material=darker_green, name="front_rim")
    body.visual(Box((0.75, 0.050, 0.055)), origin=Origin(xyz=(0.0, -0.372, 1.065)), material=darker_green, name="rear_rim")
    body.visual(Box((0.055, 0.70, 0.055)), origin=Origin(xyz=(-0.357, 0.0, 1.065)), material=darker_green, name="side_rim_0")
    body.visual(Box((0.055, 0.70, 0.055)), origin=Origin(xyz=(0.357, 0.0, 1.065)), material=darker_green, name="side_rim_1")

    # Front maintenance access hatch and service fasteners.
    body.visual(Box((0.40, 0.050, 0.34)), origin=Origin(xyz=(0.0, 0.320, 0.610)), material=darker_green, name="access_hatch")
    for ix, x in enumerate((-0.165, 0.165)):
        for iz, z in enumerate((0.475, 0.745)):
            body.visual(
                Cylinder(radius=0.018, length=0.014),
                origin=Origin(xyz=(x, 0.347, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=galvanized,
                name=f"hatch_bolt_{ix}_{iz}",
            )
    body.visual(Box((0.30, 0.012, 0.030)), origin=Origin(xyz=(0.0, 0.347, 0.610)), material=yellow, name="service_label")

    # Molded ribs, skid pads, and replaceable front wear strip.
    for i, x in enumerate((-0.285, 0.285)):
        body.visual(Box((0.070, 0.060, 0.72)), origin=Origin(xyz=(x, 0.255, 0.610)), material=bin_green, name=f"front_rib_{i}")
        body.visual(Box((0.075, 0.105, 0.080)), origin=Origin(xyz=(x, 0.235, 0.175)), material=black_rubber, name=f"front_skid_{i}")
    body.visual(Box((0.56, 0.045, 0.060)), origin=Origin(xyz=(0.0, 0.255, 0.225)), material=black_rubber, name="front_wear_strip")

    # Rear handle, hinge support weldment, and rear axle saddles.
    body.visual(
        Cylinder(radius=0.026, length=0.78),
        origin=Origin(xyz=(0.0, -0.465, 0.970), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="rear_handle",
    )
    for i, x in enumerate((-0.285, 0.285)):
        body.visual(Box((0.065, 0.105, 0.34)), origin=Origin(xyz=(x, -0.405, 0.850)), material=darker_green, name=f"handle_stanchion_{i}")
        body.visual(Box((0.095, 0.165, 0.165)), origin=Origin(xyz=(x, -0.275, 0.205)), material=darker_green, name=f"axle_saddle_{i}")
        body.visual(Box((0.070, 0.145, 0.070)), origin=Origin(xyz=(x, -0.220, 0.285)), material=darker_green, name=f"axle_bridge_{i}")

    body.visual(
        Cylinder(radius=0.021, length=0.790),
        origin=Origin(xyz=(0.0, -0.305, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="rear_axle",
    )
    # The visible shaft continues into the replaceable hub bores; collars remain inboard as service stops.
    for i, x in enumerate((-0.335, 0.335)):
        body.visual(
            Cylinder(radius=0.044, length=0.030),
            origin=Origin(xyz=(x, -0.305, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"axle_collar_{i}",
        )

    # Full-width hinge pin with visible bracket plates; the lid has captured sleeves around this pin.
    hinge_y = -0.405
    hinge_z = 1.120
    body.visual(
        Cylinder(radius=0.012, length=0.94),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hinge_pin",
    )
    for i, x in enumerate((-0.445, -0.110, 0.110, 0.445)):
        body.visual(Box((0.055, 0.050, 0.170)), origin=Origin(xyz=(x, hinge_y, 1.045)), material=galvanized, name=f"hinge_bracket_{i}")

    lid = model.part("lid")
    lid.visual(Box((0.76, 0.78, 0.046)), origin=Origin(xyz=(0.0, 0.365, 0.032)), material=bin_green, name="lid_panel")
    lid.visual(Box((0.69, 0.040, 0.070)), origin=Origin(xyz=(0.0, 0.765, 0.020)), material=darker_green, name="front_grip_lip")
    lid.visual(Box((0.032, 0.69, 0.070)), origin=Origin(xyz=(-0.386, 0.355, 0.020)), material=darker_green, name="side_lip_0")
    lid.visual(Box((0.032, 0.69, 0.070)), origin=Origin(xyz=(0.386, 0.355, 0.020)), material=darker_green, name="side_lip_1")
    for i, x in enumerate((-0.235, 0.0, 0.235)):
        lid.visual(
            Cylinder(radius=0.027, length=0.130),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=darker_green,
            name=f"lid_knuckle_{i}",
        )
    for i, x in enumerate((-0.19, 0.19)):
        lid.visual(Box((0.055, 0.56, 0.030)), origin=Origin(xyz=(x, 0.380, 0.063)), material=darker_green, name=f"lid_reinforcing_rib_{i}")
    lid.visual(Box((0.42, 0.030, 0.020)), origin=Origin(xyz=(0.0, 0.610, 0.070)), material=yellow, name="lid_service_strip")

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=0.0, upper=1.85),
    )

    wheel_mesh = WheelGeometry(
        0.112,
        0.085,
        rim=WheelRim(inner_radius=0.078, flange_height=0.010, flange_thickness=0.006, bead_seat_depth=0.004),
        hub=WheelHub(
            radius=0.041,
            width=0.074,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.050, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.008, front_inset=0.004, rear_inset=0.003),
        spokes=WheelSpokes(style="split_y", count=6, thickness=0.006, window_radius=0.017),
        bore=WheelBore(style="round", diameter=0.030),
    )
    tire_mesh = TireGeometry(
        0.150,
        0.095,
        inner_radius=0.108,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
        tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.56),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )

    for idx, x in enumerate((-0.405, 0.405)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(mesh_from_geometry(wheel_mesh, f"wheel_rim_{idx}"), material=galvanized, name="hub_rim")
        wheel.visual(mesh_from_geometry(tire_mesh, f"utility_tire_{idx}"), material=black_rubber, name="tire")
        wheel.visual(
            Cylinder(radius=0.026, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name="outer_hub_cap",
        )
        model.articulation(
            f"body_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x, -0.305, 0.160)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bin_body")
    lid = object_model.get_part("lid")
    wheels = [object_model.get_part("wheel_0"), object_model.get_part("wheel_1")]
    lid_joint = object_model.get_articulation("body_to_lid")

    # The hinge pin and wheel axle are modeled as real captured shafts inside
    # solid proxy sleeves/bores, so the small local intersections are intentional.
    for i in range(3):
        knuckle = f"lid_knuckle_{i}"
        ctx.allow_overlap(
            body,
            lid,
            elem_a="hinge_pin",
            elem_b=knuckle,
            reason="The full-width hinge pin is intentionally captured inside the lid's replaceable hinge sleeve proxy.",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a=knuckle,
            elem_b="hinge_pin",
            min_overlap=0.10,
            name=f"hinge sleeve {i} has retained pin engagement",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            elem_a=knuckle,
            elem_b="hinge_pin",
            min_overlap=0.020,
            name=f"hinge sleeve {i} is coaxial with pin",
        )

    for i, wheel in enumerate(wheels):
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="rear_axle",
            elem_b="hub_rim",
            reason="The steel rear axle is intentionally seated inside the wheel hub bore proxy to show supported rolling hardware.",
        )
        ctx.expect_overlap(
            wheel,
            body,
            axes="x",
            elem_a="hub_rim",
            elem_b="rear_axle",
            min_overlap=0.025,
            name=f"wheel {i} remains captured on axle",
        )
        ctx.expect_overlap(
            wheel,
            body,
            axes="yz",
            elem_a="hub_rim",
            elem_b="rear_axle",
            min_overlap=0.035,
            name=f"wheel {i} hub is coaxial with axle",
        )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_rim",
            min_gap=0.015,
            max_gap=0.060,
            name="closed lid clears service rim with robust tolerance",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="tapered_shell",
            min_overlap=0.50,
            name="full-width lid covers the open bin shell",
        )

    rest_aabb = ctx.part_element_world_aabb(lid, elem="front_grip_lip")
    with ctx.pose({lid_joint: 1.45}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="front_grip_lip")
    ctx.check(
        "lid swings upward on rear hinge",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.38,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
