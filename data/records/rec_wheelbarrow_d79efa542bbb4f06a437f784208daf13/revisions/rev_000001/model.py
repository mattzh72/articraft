from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cylinder_between(part, start, end, radius, material, name):
    """Add a cylinder visual whose local +Z axis runs between two points."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")

    # All authored frame tubes are either lateral X members or members in a
    # constant-X vertical plane, so keep the Euler solution simple and stable.
    if abs(dx) > 1e-9 and abs(dy) < 1e-9 and abs(dz) < 1e-9:
        rpy = (0.0, math.pi / 2.0 if dx > 0.0 else -math.pi / 2.0, 0.0)
    elif abs(dx) < 1e-9:
        roll = math.atan2(-dy, dz)
        rpy = (roll, 0.0, 0.0)
    else:
        # Fallback for rare skew braces: yaw in XY, then pitch local +Z over.
        yaw = math.atan2(dy, dx)
        horizontal = math.sqrt(dx * dx + dy * dy)
        pitch = math.atan2(horizontal, dz)
        rpy = (0.0, pitch, yaw)

    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=rpy,
        ),
        material=material,
        name=name,
    )


def _tray_mesh():
    """Closed thin-wall tapered tray with an open top and visible inner floor."""
    geom = MeshGeometry()

    def add_loop(width, depth, z):
        return [
            geom.add_vertex(-width / 2.0, -depth / 2.0, z),
            geom.add_vertex(width / 2.0, -depth / 2.0, z),
            geom.add_vertex(width / 2.0, depth / 2.0, z),
            geom.add_vertex(-width / 2.0, depth / 2.0, z),
        ]

    outer_bottom = add_loop(0.44, 0.72, 0.36)
    outer_top = add_loop(0.76, 1.10, 0.68)
    inner_top = add_loop(0.69, 1.03, 0.635)
    inner_bottom = add_loop(0.36, 0.62, 0.405)

    def quad(a, b, c, d):
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(4):
        j = (i + 1) % 4
        quad(outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        quad(outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        quad(inner_top[j], inner_top[i], inner_bottom[i], inner_bottom[j])

    # Outside bottom and inside floor close the metal pan while leaving the top open.
    quad(outer_bottom[3], outer_bottom[2], outer_bottom[1], outer_bottom[0])
    quad(inner_bottom[0], inner_bottom[1], inner_bottom[2], inner_bottom[3])
    return geom


def _fork_shape():
    """Bolt-on front fork with two drilled cheeks and a rear bridge."""
    gap = 0.135
    cheek_t = 0.035
    cheek_y = 0.39
    cheek_z = 0.25
    cheek_x = gap / 2.0 + cheek_t / 2.0

    left = cq.Workplane("XY").box(cheek_t, cheek_y, cheek_z).translate((cheek_x, 0.075, 0.015))
    right = cq.Workplane("XY").box(cheek_t, cheek_y, cheek_z).translate((-cheek_x, 0.075, 0.015))
    rear_bridge = cq.Workplane("XY").box(0.34, 0.055, 0.080).translate((0.0, 0.285, 0.155))
    upper_plate = cq.Workplane("XY").box(0.44, 0.085, 0.040).translate((0.0, 0.245, 0.180))
    fork = left.union(right).union(rear_bridge).union(upper_plate)

    axle_bore = cq.Workplane("YZ").circle(0.018).extrude(0.50, both=True)
    bolt_holes = None
    for x in (-0.14, 0.14):
        cutter = cq.Workplane("XY").center(x, 0.245).circle(0.014).extrude(0.35, both=True)
        bolt_holes = cutter if bolt_holes is None else bolt_holes.union(cutter)

    fork = fork.cut(axle_bore)
    if bolt_holes is not None:
        fork = fork.cut(bolt_holes)
    return fork.edges("|Z").fillet(0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    green = model.material("painted_green", rgba=(0.08, 0.36, 0.16, 1.0))
    dark_green = model.material("dark_green", rgba=(0.04, 0.18, 0.08, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.68, 0.70, 0.66, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    red = model.material("red_rim", rgba=(0.65, 0.05, 0.03, 1.0))

    upper = model.part("upper_body")
    upper.visual(
        mesh_from_geometry(_tray_mesh(), "hollow_tray"),
        material=green,
        name="hollow_tray",
    )

    # Two continuous handle rails cradle the tray and continue rearward as grips.
    for side, x in (("0", -0.45), ("1", 0.45)):
        _cylinder_between(
            upper,
            (x, -0.57, 0.31),
            (x, 1.04, 0.54),
            0.018,
            steel,
            f"side_rail_{side}",
        )
        _cylinder_between(
            upper,
            (x, 1.00, 0.535),
            (x, 1.30, 0.58),
            0.026,
            black,
            f"grip_{side}",
        )
        _cylinder_between(
            upper,
            (x, 0.42, 0.452),
            (x, 0.74, 0.060),
            0.019,
            steel,
            f"rear_leg_{side}",
        )
        upper.visual(
            Box((0.16, 0.10, 0.060)),
            origin=Origin(xyz=(x, 0.76, 0.030)),
            material=black,
            name=f"foot_pad_{side}",
        )

    # Crossmembers connect the rails and bite slightly into the tray floor so the
    # upper-body reads as one bolted module rather than separate floating pieces.
    for name, y, z in (("front_crossbar", -0.30, 0.345), ("rear_crossbar", 0.32, 0.355)):
        _cylinder_between(upper, (-0.48, y, z), (0.48, y, z), 0.020, steel, name)

    upper.visual(
        Box((0.42, 0.18, 0.050)),
        origin=Origin(xyz=(0.0, -0.395, 0.465)),
        material=dark_green,
        name="front_neck",
    )
    upper.visual(
        Box((0.48, 0.065, 0.035)),
        origin=Origin(xyz=(0.0, -0.475, 0.4475)),
        material=dark_green,
        name="front_mount_plate",
    )
    for i, x in enumerate((-0.15, 0.15)):
        upper.visual(
            Cylinder(radius=0.020, length=0.016),
            origin=Origin(xyz=(x, -0.475, 0.4725)),
            material=steel,
            name=f"mount_bolt_{i}",
        )

    fork = model.part("fork")
    fork.visual(
        mesh_from_cadquery(_fork_shape(), "fork_module", tolerance=0.0008),
        material=dark_green,
        name="fork_module",
    )

    axle = model.part("axle")
    axle.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_shaft",
    )
    for side, x in (("0", -0.125), ("1", 0.125)):
        axle.visual(
            Cylinder(radius=0.036, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"outer_washer_{side}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.225,
                0.090,
                inner_radius=0.152,
                carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
                tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.55),
                grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.004),),
                sidewall=TireSidewall(style="square", bulge=0.025),
                shoulder=TireShoulder(width=0.010, radius=0.004),
            ),
            "front_tire",
        ),
        material=black,
        name="front_tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.158,
                0.072,
                rim=WheelRim(
                    inner_radius=0.105,
                    flange_height=0.010,
                    flange_thickness=0.004,
                    bead_seat_depth=0.004,
                ),
                hub=WheelHub(
                    radius=0.040,
                    width=0.080,
                    cap_style="flat",
                    bolt_pattern=BoltPattern(count=4, circle_diameter=0.054, hole_diameter=0.006),
                ),
                face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
                spokes=WheelSpokes(style="straight", count=8, thickness=0.004, window_radius=0.018),
                bore=WheelBore(style="round", diameter=0.044),
            ),
            "red_spoked_wheel",
        ),
        material=red,
        name="red_spoked_wheel",
    )
    wheel.visual(
        Cylinder(radius=0.024, length=0.092),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub_bushing",
    )

    model.articulation(
        "body_to_fork",
        ArticulationType.FIXED,
        parent=upper,
        child=fork,
        origin=Origin(xyz=(0.0, -0.72, 0.23)),
    )
    model.articulation(
        "fork_to_axle",
        ArticulationType.FIXED,
        parent=fork,
        child=axle,
        origin=Origin(),
    )
    model.articulation(
        "axle_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=wheel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_body")
    fork = object_model.get_part("fork")
    axle = object_model.get_part("axle")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("axle_to_wheel")

    ctx.allow_overlap(
        axle,
        wheel,
        elem_a="axle_shaft",
        elem_b="hub_bushing",
        reason="The visible bearing sleeve is intentionally modeled as a captured fit around the axle shaft.",
    )
    ctx.allow_overlap(
        axle,
        fork,
        elem_a="axle_shaft",
        elem_b="fork_module",
        reason="The fixed axle is intentionally captured through the drilled fork cheeks.",
    )
    ctx.check(
        "front wheel uses continuous axle joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_contact(
        upper,
        fork,
        elem_a="front_mount_plate",
        elem_b="fork_module",
        contact_tol=0.008,
        name="bolt-on fork plate seats against upper body",
    )
    ctx.expect_overlap(
        axle,
        fork,
        axes="xyz",
        min_overlap=0.030,
        elem_a="axle_shaft",
        elem_b="fork_module",
        name="axle shaft passes through fork bores",
    )
    ctx.expect_overlap(
        axle,
        wheel,
        axes="xyz",
        min_overlap=0.030,
        elem_a="axle_shaft",
        elem_b="hub_bushing",
        name="wheel hub surrounds the axle line",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spins without translating",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(a - b) < 1e-9 for a, b in zip(rest_pos, turned_pos)),
        details=f"rest={rest_pos}, spun={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
