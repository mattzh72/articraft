from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_watch_winder_box")

    # Real-world tabletop winder proportions, in meters.
    width = 0.360
    depth = 0.260
    body_height = 0.120
    wall = 0.018
    bottom = 0.018
    rim_h = 0.015
    hinge_y = depth / 2.0 + 0.006
    hinge_z = body_height + rim_h + 0.004

    molded = model.material("molded_olive_polymer", rgba=(0.18, 0.22, 0.16, 1.0))
    edge_rubber = model.material("black_rubber_bumpers", rgba=(0.015, 0.016, 0.014, 1.0))
    dark_insert = model.material("matte_black_insert", rgba=(0.025, 0.026, 0.024, 1.0))
    gunmetal = model.material("dark_gunmetal_hardware", rgba=(0.26, 0.27, 0.26, 1.0))
    brushed = model.material("brushed_steel_fasteners", rgba=(0.62, 0.62, 0.58, 1.0))
    glass = model.material("smoked_polycarbonate_window", rgba=(0.30, 0.42, 0.48, 0.34))
    foam = model.material("black_watch_cushion", rgba=(0.005, 0.005, 0.006, 1.0))
    amber = model.material("amber_status_lens", rgba=(1.0, 0.55, 0.08, 0.88))

    case = model.part("case")
    # Hollow presentation box: separate bottom, walls, thick top rim and lip,
    # with all members overlapping locally as a manufactured polymer shell.
    case.visual(Box((width, depth, bottom)), origin=Origin(xyz=(0.0, 0.0, bottom / 2.0)), material=molded, name="bottom_pan")
    case.visual(Box((wall, depth, body_height)), origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, body_height / 2.0)), material=molded, name="side_wall_0")
    case.visual(Box((wall, depth, body_height)), origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, body_height / 2.0)), material=molded, name="side_wall_1")
    case.visual(Box((width, wall, body_height)), origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, body_height / 2.0)), material=molded, name="front_wall")
    case.visual(Box((width, wall, body_height)), origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_height / 2.0)), material=molded, name="rear_wall")

    # Raised upper rim with a soft service gasket under the transparent lid.
    case.visual(Box((width, 0.026, rim_h)), origin=Origin(xyz=(0.0, -depth / 2.0 + 0.013, body_height + rim_h / 2.0)), material=molded, name="front_top_rim")
    case.visual(Box((width, 0.026, rim_h)), origin=Origin(xyz=(0.0, depth / 2.0 - 0.013, body_height + rim_h / 2.0)), material=molded, name="rear_top_rim")
    case.visual(Box((0.026, depth, rim_h)), origin=Origin(xyz=(-width / 2.0 + 0.013, 0.0, body_height + rim_h / 2.0)), material=molded, name="side_top_rim_0")
    case.visual(Box((0.026, depth, rim_h)), origin=Origin(xyz=(width / 2.0 - 0.013, 0.0, body_height + rim_h / 2.0)), material=molded, name="side_top_rim_1")
    case.visual(Box((0.250, 0.010, 0.004)), origin=Origin(xyz=(0.0, -depth / 2.0 + 0.025, body_height + rim_h + 0.001)), material=edge_rubber, name="front_gasket")
    case.visual(Box((0.250, 0.010, 0.004)), origin=Origin(xyz=(0.0, depth / 2.0 - 0.025, body_height + rim_h + 0.001)), material=edge_rubber, name="rear_gasket")
    case.visual(Box((0.010, 0.170, 0.004)), origin=Origin(xyz=(-width / 2.0 + 0.025, 0.0, body_height + rim_h + 0.001)), material=edge_rubber, name="side_gasket_0")
    case.visual(Box((0.010, 0.170, 0.004)), origin=Origin(xyz=(width / 2.0 - 0.025, 0.0, body_height + rim_h + 0.001)), material=edge_rubber, name="side_gasket_1")

    # Rugged corner guards and molded external ribs.
    for i, sx in enumerate((-1.0, 1.0)):
        for j, sy in enumerate((-1.0, 1.0)):
            case.visual(
                Box((0.034, 0.034, body_height + rim_h - 0.001)),
                origin=Origin(xyz=(sx * (width / 2.0 - 0.017), sy * (depth / 2.0 - 0.017), (body_height + rim_h - 0.001) / 2.0)),
                material=edge_rubber,
                name=f"corner_guard_{i}_{j}",
            )
    for x in (-0.095, 0.095):
        case.visual(Box((0.018, 0.210, 0.010)), origin=Origin(xyz=(x, 0.0, bottom + 0.005)), material=dark_insert, name=f"floor_rail_{'n' if x < 0 else 'p'}")

    # Fixed motor pod and bearing boss mounted inside the box.
    case.visual(Box((0.115, 0.045, 0.058)), origin=Origin(xyz=(0.0, 0.025, 0.047)), material=dark_insert, name="motor_stand")
    case.visual(
        Cylinder(radius=0.039, length=0.035),
        origin=Origin(xyz=(0.0, 0.010, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="motor_pod",
    )
    case.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, -0.009, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="bearing_face",
    )

    # Exposed fixed hinge leaf and interleaved case knuckles on the rear.
    case.visual(Box((0.310, 0.006, 0.040)), origin=Origin(xyz=(0.0, depth / 2.0 + 0.003, hinge_z - 0.027)), material=gunmetal, name="case_hinge_leaf")
    for name, x, length in (("case_knuckle_0", -0.138, 0.068), ("case_knuckle_1", 0.0, 0.068), ("case_knuckle_2", 0.138, 0.068)):
        case.visual(
            Cylinder(radius=0.008, length=length),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gunmetal,
            name=name,
        )
    for x in (-0.177, 0.177):
        case.visual(Cylinder(radius=0.010, length=0.010), origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)), material=brushed, name=f"pin_head_{'n' if x < 0 else 'p'}")

    # Front latch hinge fixed lugs and a keeper boss on the case.
    latch_pivot_y = -depth / 2.0 - 0.006
    latch_pivot_z = 0.076
    for x in (-0.034, 0.034):
        case.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(x, latch_pivot_y, latch_pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gunmetal,
            name=f"latch_lug_{'n' if x < 0 else 'p'}",
        )
        case.visual(Box((0.020, 0.010, 0.022)), origin=Origin(xyz=(x, -depth / 2.0 - 0.002, latch_pivot_z - 0.004)), material=gunmetal, name=f"latch_bracket_{'n' if x < 0 else 'p'}")

    # Control-panel details on the front face.
    case.visual(Box((0.105, 0.004, 0.040)), origin=Origin(xyz=(-0.096, -depth / 2.0 - 0.002, 0.050)), material=dark_insert, name="control_plate")
    case.visual(Cylinder(radius=0.008, length=0.004), origin=Origin(xyz=(-0.050, -depth / 2.0 - 0.004, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)), material=amber, name="status_lens")
    case.visual(Cylinder(radius=0.020, length=0.006), origin=Origin(xyz=(-0.115, -depth / 2.0 - 0.0035, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)), material=gunmetal, name="knob_bushing")

    # Exposed fasteners embedded slightly in the molded panels.
    for x in (-0.145, 0.145):
        for z in (0.040, 0.105):
            case.visual(
                Cylinder(radius=0.005, length=0.003),
                origin=Origin(xyz=(x, -depth / 2.0 - 0.0008, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=brushed,
                name=f"front_screw_{x:.2f}_{z:.2f}",
            )

    lid = model.part("lid")
    lid_depth = 0.274
    lid_width = 0.382
    lid_t = 0.024
    # Child frame is the hinge pin center. Closed lid extends forward along -Y.
    lid.visual(Box((0.034, 0.244, lid_t)), origin=Origin(xyz=(-lid_width / 2.0 + 0.017, -0.143, lid_t / 2.0)), material=molded, name="side_frame_0")
    lid.visual(Box((0.034, 0.244, lid_t)), origin=Origin(xyz=(lid_width / 2.0 - 0.017, -0.143, lid_t / 2.0)), material=molded, name="side_frame_1")
    lid.visual(Box((lid_width, 0.036, lid_t)), origin=Origin(xyz=(0.0, -lid_depth + 0.018, lid_t / 2.0)), material=molded, name="front_frame")
    lid.visual(Box((lid_width, 0.024, lid_t)), origin=Origin(xyz=(0.0, -0.020, lid_t / 2.0)), material=molded, name="rear_frame")
    lid.visual(Box((0.320, 0.210, 0.006)), origin=Origin(xyz=(0.0, -0.142, 0.012)), material=glass, name="window_pane")
    # Inner protective grille bars visible through the clear panel.
    for x in (-0.090, 0.0, 0.090):
        lid.visual(Box((0.007, 0.160, 0.006)), origin=Origin(xyz=(x, -0.142, 0.0175)), material=gunmetal, name=f"window_bar_{x:.2f}")
    for y in (-0.195, -0.090):
        lid.visual(Box((0.270, 0.006, 0.006)), origin=Origin(xyz=(0.0, y, 0.0175)), material=gunmetal, name=f"cross_bar_{y:.2f}")
    # Moving hinge straps and knuckles interleaved with the case knuckles.
    for name, x, length in (("lid_knuckle_0", -0.070, 0.052), ("lid_knuckle_1", 0.070, 0.052)):
        lid.visual(
            Cylinder(radius=0.008, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gunmetal,
            name=name,
        )
        lid.visual(Box((length, 0.026, 0.005)), origin=Origin(xyz=(x, -0.011, 0.006)), material=gunmetal, name=f"{name}_strap")

    # Front catch block and lid fasteners.
    lid.visual(Box((0.074, 0.012, 0.020)), origin=Origin(xyz=(0.0, -lid_depth - 0.002, 0.010)), material=gunmetal, name="latch_keeper")
    for x in (-0.168, 0.168):
        for y in (-0.246, -0.040):
            lid.visual(Cylinder(radius=0.0045, length=0.003), origin=Origin(xyz=(x, y, lid_t + 0.0005)), material=brushed, name=f"lid_screw_{x:.2f}_{y:.2f}")

    cradle = model.part("winder_cradle")
    # Rotating winder: a visible drive plate, metal protective ring, soft watch
    # pillow, retaining bands and a mounted watch silhouette.
    cradle.visual(
        Cylinder(radius=0.046, length=0.012),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="drive_plate",
    )
    cradle.visual(
        mesh_from_geometry(TorusGeometry(radius=0.050, tube=0.005, radial_segments=24, tubular_segments=48), "protective_ring"),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="protective_ring",
    )
    for angle, name in ((0.0, "spoke_0"), (math.pi / 2.0, "spoke_1"), (math.pi, "spoke_2"), (3.0 * math.pi / 2.0, "spoke_3")):
        # Spoke boxes are defined along X then rotated in the cradle plane.
        cradle.visual(Box((0.078, 0.006, 0.006)), origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(0.0, 0.0, angle)), material=gunmetal, name=name)
    cradle.visual(
        Cylinder(radius=0.020, length=0.074),
        origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=foam,
        name="cushion",
    )
    for x in (-0.026, 0.026):
        cradle.visual(Box((0.007, 0.020, 0.047)), origin=Origin(xyz=(x, -0.046, 0.0)), material=edge_rubber, name=f"cushion_band_{'n' if x < 0 else 'p'}")
    cradle.visual(
        mesh_from_geometry(TorusGeometry(radius=0.029, tube=0.003, radial_segments=20, tubular_segments=40), "watch_bezel"),
        origin=Origin(xyz=(0.0, -0.053, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="watch_bezel",
    )
    cradle.visual(Cylinder(radius=0.026, length=0.003), origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_insert, name="watch_face")

    latch = model.part("front_latch")
    latch.visual(Cylinder(radius=0.006, length=0.030), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=gunmetal, name="latch_barrel")
    latch.visual(Box((0.055, 0.006, 0.062)), origin=Origin(xyz=(0.0, -0.008, 0.031)), material=gunmetal, name="hasp_plate")
    latch.visual(Box((0.065, 0.010, 0.012)), origin=Origin(xyz=(0.0, -0.009, 0.060)), material=brushed, name="hasp_hook")
    latch.visual(Cylinder(radius=0.004, length=0.004), origin=Origin(xyz=(-0.018, -0.013, 0.034), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed, name="hasp_rivet_0")
    latch.visual(Cylinder(radius=0.004, length=0.004), origin=Origin(xyz=(0.018, -0.013, 0.034), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed, name="hasp_rivet_1")

    knob = model.part("control_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.038,
            0.020,
            body_style="faceted",
            top_diameter=0.031,
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=16, depth=0.0012, width=0.0020),
            indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        ),
        "control_knob",
    )
    knob.visual(knob_mesh, origin=Origin(xyz=(0.0, -0.0095, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=gunmetal, name="knob_cap")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "winder_spin",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.023, 0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=0.8),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=case,
        child=latch,
        origin=Origin(xyz=(0.0, latch_pivot_y, latch_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "knob_turn",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=knob,
        origin=Origin(xyz=(-0.115, -depth / 2.0 - 0.007, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("winder_cradle")
    latch = object_model.get_part("front_latch")
    lid_hinge = object_model.get_articulation("lid_hinge")
    winder_spin = object_model.get_articulation("winder_spin")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        min_gap=0.0005,
        max_gap=0.0045,
        positive_elem="front_frame",
        negative_elem="front_gasket",
        name="closed lid rests just above the service gasket",
    )
    ctx.expect_within(cradle, case, axes="xz", margin=0.002, name="rotating winder is contained in the rugged box")
    ctx.expect_overlap(lid, case, axes="xy", min_overlap=0.150, elem_a="window_pane", elem_b="bottom_pan", name="clear lid window covers the presentation bay")

    front_closed = ctx.part_element_world_aabb(lid, elem="front_frame")
    with ctx.pose({lid_hinge: 1.40, latch_pivot: 1.0}):
        front_open = ctx.part_element_world_aabb(lid, elem="front_frame")
    ctx.check(
        "hinged lid opens upward and rearward on real rear pivots",
        front_closed is not None
        and front_open is not None
        and front_open[0][2] > front_closed[0][2] + 0.13
        and front_open[0][1] > front_closed[0][1] + 0.15,
        details=f"closed={front_closed}, open={front_open}",
    )

    cushion_rest = ctx.part_element_world_aabb(cradle, elem="cushion")
    with ctx.pose({winder_spin: math.pi / 2.0}):
        cushion_rotated = ctx.part_element_world_aabb(cradle, elem="cushion")
    ctx.check(
        "winder cradle visibly rotates the watch cushion",
        cushion_rest is not None
        and cushion_rotated is not None
        and (cushion_rotated[1][2] - cushion_rotated[0][2]) > (cushion_rest[1][2] - cushion_rest[0][2]) + 0.030,
        details=f"rest={cushion_rest}, rotated={cushion_rotated}",
    )

    latch_rest = ctx.part_element_world_aabb(latch, elem="hasp_hook")
    with ctx.pose({latch_pivot: 0.95}):
        latch_open = ctx.part_element_world_aabb(latch, elem="hasp_hook")
    ctx.check(
        "front latch flips outward for service",
        latch_rest is not None
        and latch_open is not None
        and latch_open[0][1] < latch_rest[0][1] - 0.025
        and latch_open[1][2] < latch_rest[1][2] - 0.018,
        details=f"rest={latch_rest}, open={latch_open}",
    )

    return ctx.report()


object_model = build_object_model()
