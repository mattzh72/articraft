from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
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
)


def _rounded_rect_loop(width: float, depth: float, radius: float, *, corner_segments: int = 8):
    """Counter-clockwise rounded rectangle in the XY plane."""
    r = min(radius, width * 0.49, depth * 0.49)
    corners = (
        (width * 0.5 - r, depth * 0.5 - r, 0.0, math.pi * 0.5),
        (-(width * 0.5 - r), depth * 0.5 - r, math.pi * 0.5, math.pi),
        (-(width * 0.5 - r), -(depth * 0.5 - r), math.pi, math.pi * 1.5),
        (width * 0.5 - r, -(depth * 0.5 - r), math.pi * 1.5, math.pi * 2.0),
    )
    pts: list[tuple[float, float]] = []
    for cx, cy, a0, a1 in corners:
        for i in range(corner_segments + 1):
            if pts and i == 0:
                continue
            a = a0 + (a1 - a0) * i / corner_segments
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _tapered_hollow_bin_mesh() -> MeshGeometry:
    """Open-topped tapered bin tub with a real wall thickness and bottom floor."""
    mesh = MeshGeometry()
    wall = 0.035
    # X is depth (front positive), Y is width, Z is up.
    sections = [
        (0.060, 0.455, 0.420, 0.055),
        (0.160, 0.500, 0.500, 0.065),
        (0.520, 0.590, 0.640, 0.075),
        (0.900, 0.640, 0.735, 0.085),
        (1.020, 0.655, 0.760, 0.090),
    ]
    outer_loops: list[list[int]] = []
    inner_loops: list[list[int]] = []
    for z, width_y, depth_x, radius in sections:
        outer = _rounded_rect_loop(depth_x, width_y, radius, corner_segments=8)
        inner = _rounded_rect_loop(
            max(0.02, depth_x - 2 * wall),
            max(0.02, width_y - 2 * wall),
            max(0.010, radius - wall),
            corner_segments=8,
        )
        outer_loops.append([mesh.add_vertex(x, y, z) for x, y in outer])
        inner_loops.append([mesh.add_vertex(x, y, z + (0.035 if z == sections[0][0] else 0.0)) for x, y in inner])

    count = len(outer_loops[0])
    for s in range(len(sections) - 1):
        for i in range(count):
            j = (i + 1) % count
            _add_quad(mesh, outer_loops[s][i], outer_loops[s][j], outer_loops[s + 1][j], outer_loops[s + 1][i])
            _add_quad(mesh, inner_loops[s][j], inner_loops[s][i], inner_loops[s + 1][i], inner_loops[s + 1][j])

    # Wide molded top rim as a flat annular ring around the opening.
    top_outer = outer_loops[-1]
    top_inner = inner_loops[-1]
    for i in range(count):
        j = (i + 1) % count
        _add_quad(mesh, top_outer[i], top_outer[j], top_inner[j], top_inner[i])

    # Closed bin floor at the bottom of the cavity.
    floor_center = mesh.add_vertex(0.0, 0.0, sections[0][0] + 0.035)
    for i in range(count):
        j = (i + 1) % count
        mesh.add_face(floor_center, inner_loops[0][j], inner_loops[0][i])

    # Underside panel so the molded base reads as a thick plastic tub.
    underside_center = mesh.add_vertex(0.0, 0.0, sections[0][0])
    for i in range(count):
        j = (i + 1) % count
        mesh.add_face(underside_center, outer_loops[0][i], outer_loops[0][j])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_wheelie_bin")

    green = model.material("molded_green_hdpe", rgba=(0.05, 0.34, 0.17, 1.0))
    dark_green = model.material("thickened_dark_green", rgba=(0.025, 0.22, 0.12, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    dark_plastic = model.material("dark_reinforced_plastic", rgba=(0.035, 0.040, 0.038, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    fastener = model.material("worn_fastener_heads", rgba=(0.42, 0.42, 0.38, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_tapered_hollow_bin_mesh(), "molded_tub"),
        material=green,
        name="molded_tub",
    )

    # Thick utility rim and molded wear bands.
    body.visual(Box((0.055, 0.70, 0.060)), origin=Origin(xyz=(0.390, 0.0, 1.025)), material=dark_green, name="front_rim")
    body.visual(Box((0.055, 0.70, 0.060)), origin=Origin(xyz=(-0.390, 0.0, 1.025)), material=dark_green, name="rear_rim")
    body.visual(Box((0.760, 0.050, 0.060)), origin=Origin(xyz=(0.000, 0.345, 1.025)), material=dark_green, name="side_rim_0")
    body.visual(Box((0.760, 0.050, 0.060)), origin=Origin(xyz=(0.000, -0.345, 1.025)), material=dark_green, name="side_rim_1")
    body.visual(Box((0.070, 0.052, 0.650)), origin=Origin(xyz=(0.350, 0.205, 0.600)), material=dark_green, name="front_rib_0")
    body.visual(Box((0.070, 0.052, 0.650)), origin=Origin(xyz=(0.350, -0.205, 0.600)), material=dark_green, name="front_rib_1")
    body.visual(Box((0.060, 0.560, 0.055)), origin=Origin(xyz=(0.315, 0.0, 0.720)), material=dark_green, name="front_wear_band")
    body.visual(Box((0.055, 0.065, 0.520)), origin=Origin(xyz=(-0.120, 0.326, 0.590)), material=dark_green, name="side_rib_0")
    body.visual(Box((0.055, 0.065, 0.520)), origin=Origin(xyz=(-0.120, -0.326, 0.590)), material=dark_green, name="side_rib_1")

    # Rear handle, axle cradles, and full-width hinge pin carried by molded lugs.
    body.visual(Cylinder(radius=0.024, length=0.700), origin=Origin(xyz=(-0.455, 0.0, 0.935), rpy=(-math.pi / 2, 0.0, 0.0)), material=dark_plastic, name="rear_handle_bar")
    body.visual(Box((0.160, 0.045, 0.245)), origin=Origin(xyz=(-0.395, 0.300, 0.855)), material=dark_green, name="handle_stanchion_0")
    body.visual(Box((0.160, 0.045, 0.245)), origin=Origin(xyz=(-0.395, -0.300, 0.855)), material=dark_green, name="handle_stanchion_1")
    body.visual(Box((0.070, 0.095, 0.220)), origin=Origin(xyz=(-0.285, 0.305, 0.215)), material=dark_green, name="axle_boss_0")
    body.visual(Box((0.070, 0.095, 0.220)), origin=Origin(xyz=(-0.285, -0.305, 0.215)), material=dark_green, name="axle_boss_1")
    body.visual(Cylinder(radius=0.016, length=0.830), origin=Origin(xyz=(-0.300, 0.0, 0.185), rpy=(-math.pi / 2, 0.0, 0.0)), material=steel, name="rear_axle")
    body.visual(Cylinder(radius=0.014, length=0.725), origin=Origin(xyz=(-0.465, 0.0, 1.070), rpy=(-math.pi / 2, 0.0, 0.0)), material=steel, name="hinge_pin")
    for idx, y in enumerate((-0.255, 0.0, 0.255)):
        body.visual(Cylinder(radius=0.031, length=0.125), origin=Origin(xyz=(-0.465, y, 1.070), rpy=(-math.pi / 2, 0.0, 0.0)), material=dark_green, name=f"body_hinge_knuckle_{idx}")
        body.visual(Box((0.115, 0.110, 0.026)), origin=Origin(xyz=(-0.420, y, 1.045)), material=dark_green, name=f"body_hinge_leaf_{idx}")

    # Exposed service fasteners on the rim, hinge leaves, and axle bosses.
    for idx, (x, y, z) in enumerate(
        [
            (0.360, 0.205, 0.900),
            (0.360, -0.205, 0.900),
            (0.350, 0.210, 0.745),
            (0.350, -0.210, 0.745),
            (-0.430, 0.255, 1.045),
            (-0.430, -0.255, 1.045),
            (-0.295, 0.285, 0.220),
            (-0.295, -0.285, 0.220),
        ]
    ):
        body.visual(Cylinder(radius=0.012, length=0.020), origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2, 0.0)), material=fastener, name=f"body_bolt_{idx}")

    lid = model.part("lid")
    lid.visual(Box((0.860, 0.740, 0.040)), origin=Origin(xyz=(0.460, 0.0, 0.030)), material=green, name="lid_slab")
    lid.visual(Box((0.620, 0.500, 0.030)), origin=Origin(xyz=(0.485, 0.0, 0.064)), material=green, name="raised_lid_panel")
    lid.visual(Box((0.630, 0.045, 0.038)), origin=Origin(xyz=(0.430, 0.205, 0.092)), material=dark_green, name="lid_rib_0")
    lid.visual(Box((0.630, 0.045, 0.038)), origin=Origin(xyz=(0.430, -0.205, 0.092)), material=dark_green, name="lid_rib_1")
    lid.visual(Box((0.055, 0.735, 0.075)), origin=Origin(xyz=(0.910, 0.0, 0.000)), material=dark_green, name="front_lid_lip")
    lid.visual(Box((0.800, 0.040, 0.070)), origin=Origin(xyz=(0.465, 0.390, 0.005)), material=dark_green, name="side_lid_lip_0")
    lid.visual(Box((0.800, 0.040, 0.070)), origin=Origin(xyz=(0.465, -0.390, 0.005)), material=dark_green, name="side_lid_lip_1")
    for idx, y in enumerate((-0.125, 0.125)):
        lid.visual(Cylinder(radius=0.027, length=0.108), origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)), material=dark_green, name=f"lid_hinge_knuckle_{idx}")
        lid.visual(Box((0.160, 0.100, 0.022)), origin=Origin(xyz=(0.100, y, 0.018)), material=dark_green, name=f"lid_hinge_leaf_{idx}")
    for idx, y in enumerate((-0.125, 0.125)):
        lid.visual(Cylinder(radius=0.009, length=0.007), origin=Origin(xyz=(0.145, y, 0.031), rpy=(0.0, 0.0, 0.0)), material=fastener, name=f"lid_bolt_{idx}")

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.155,
            0.078,
            inner_radius=0.107,
            tread=TireTread(style="block", depth=0.010, count=20, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.010, radius=0.003),
        ),
        "block_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.112,
            0.070,
            rim=WheelRim(inner_radius=0.070, flange_height=0.010, flange_thickness=0.005, bead_seat_depth=0.004),
            hub=WheelHub(
                radius=0.036,
                width=0.052,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.047, hole_diameter=0.005),
            ),
            face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=6, thickness=0.004, window_radius=0.012),
            bore=WheelBore(style="round", diameter=0.036),
        ),
        "service_wheel",
    )

    wheel_origin = Origin(rpy=(0.0, 0.0, math.pi / 2))
    wheel_0 = model.part("rear_wheel_0")
    wheel_0.visual(tire_mesh, origin=wheel_origin, material=black, name="tire")
    wheel_0.visual(wheel_mesh, origin=wheel_origin, material=dark_plastic, name="rim")
    wheel_1 = model.part("rear_wheel_1")
    wheel_1.visual(tire_mesh, origin=wheel_origin, material=black, name="tire")
    wheel_1.visual(wheel_mesh, origin=wheel_origin, material=dark_plastic, name="rim")

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.465, 0.0, 1.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=18.0, velocity=1.5),
        motion_properties=MotionProperties(damping=0.12, friction=0.08),
    )
    model.articulation(
        "body_to_rear_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel_0,
        origin=Origin(xyz=(-0.300, 0.3915, 0.185)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.04, friction=0.02),
    )
    model.articulation(
        "body_to_rear_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel_1,
        origin=Origin(xyz=(-0.300, -0.3915, 0.185)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.04, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("rear_wheel_0")
    wheel_1 = object_model.get_part("rear_wheel_1")
    lid_hinge = object_model.get_articulation("body_to_lid")

    for knuckle in ("lid_hinge_knuckle_0", "lid_hinge_knuckle_1"):
        ctx.allow_overlap(
            body,
            lid,
            elem_a="hinge_pin",
            elem_b=knuckle,
            reason="The galvanized hinge pin is intentionally captured through the lid hinge knuckle bore.",
        )
        ctx.expect_overlap(
            body,
            lid,
            axes="y",
            elem_a="hinge_pin",
            elem_b=knuckle,
            min_overlap=0.080,
            name=f"{knuckle} retains full-width pin engagement",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.080, positive_elem="lid_slab", negative_elem="front_rim", name="closed lid sits just above the front rim")
        ctx.expect_overlap(lid, body, axes="xy", elem_a="lid_slab", elem_b="molded_tub", min_overlap=0.55, name="closed lid covers the framed top opening")

    rest_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward about the rear hinge",
        rest_aabb is not None and open_aabb is not None and open_aabb[1][2] > rest_aabb[1][2] + 0.25,
        details=f"closed_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    ctx.expect_overlap(body, wheel_0, axes="y", elem_a="rear_axle", elem_b="rim", min_overlap=0.030, name="wheel_0 is carried on the rear axle")
    ctx.expect_overlap(body, wheel_1, axes="y", elem_a="rear_axle", elem_b="rim", min_overlap=0.030, name="wheel_1 is carried on the rear axle")

    return ctx.report()


object_model = build_object_model()
