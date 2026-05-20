from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
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
    mesh_from_geometry,
    rounded_rect_profile,
)


def _add_loop(geom: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in points]


def _bridge_loops(geom: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            geom.add_face(a[i], b[i], a[j])
            geom.add_face(a[j], b[i], b[j])
        else:
            geom.add_face(a[i], a[j], b[i])
            geom.add_face(a[j], b[j], b[i])


def _cap_loop(geom: MeshGeometry, loop: list[int], point: tuple[float, float, float], *, flip: bool = False) -> None:
    center = geom.add_vertex(*point)
    count = len(loop)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            geom.add_face(center, loop[j], loop[i])
        else:
            geom.add_face(center, loop[i], loop[j])


def _rounded_loop(
    depth: float,
    width: float,
    radius: float,
    z: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    corner_segments: int = 9,
) -> list[tuple[float, float, float]]:
    # rounded_rect_profile returns XY; use X as bin depth and Y as bin width.
    return [(x + cx, y + cy, z) for x, y in rounded_rect_profile(depth, width, radius, corner_segments=corner_segments)]


def _bin_shell_geometry() -> MeshGeometry:
    """A watertight thin-wall tapered bin with a genuinely open top."""
    geom = MeshGeometry()
    outer_specs = [
        # z, depth, width, radius, x center
        (0.075, 0.410, 0.440, 0.055, 0.050),
        (0.170, 0.465, 0.500, 0.062, 0.045),
        (0.560, 0.575, 0.585, 0.075, 0.030),
        (0.935, 0.650, 0.635, 0.085, 0.018),
    ]
    inner_specs = [
        (0.175, 0.335, 0.350, 0.046, 0.058),
        (0.540, 0.490, 0.500, 0.065, 0.040),
        (0.930, 0.560, 0.545, 0.075, 0.026),
    ]
    outer = [
        _add_loop(geom, _rounded_loop(depth, width, radius, z, cx=cx))
        for z, depth, width, radius, cx in outer_specs
    ]
    inner = [
        _add_loop(geom, _rounded_loop(depth, width, radius, z, cx=cx))
        for z, depth, width, radius, cx in inner_specs
    ]

    for lower, upper in zip(outer, outer[1:]):
        _bridge_loops(geom, lower, upper)
    for lower, upper in zip(inner, inner[1:]):
        _bridge_loops(geom, upper, lower, flip=True)

    # Top rim material band: connect outer top to the inset inner lip and leave the bin open.
    _bridge_loops(geom, outer[-1], inner[-1])
    # Interior floor and underside close the molded shell while preserving visible hollow depth.
    _cap_loop(geom, inner[0], (0.058, 0.0, 0.175), flip=True)
    _cap_loop(geom, outer[0], (0.050, 0.0, 0.075))
    return geom


def _lid_panel_geometry() -> MeshGeometry:
    """Crowned rounded rectangular lid panel authored in the lid hinge frame."""
    geom = MeshGeometry()
    lower = _add_loop(geom, _rounded_loop(0.675, 0.690, 0.065, 0.005, cx=0.388))
    upper = _add_loop(geom, _rounded_loop(0.655, 0.670, 0.060, 0.033, cx=0.390))
    _bridge_loops(geom, lower, upper)
    _cap_loop(geom, upper, (0.390, 0.0, 0.036))
    _cap_loop(geom, lower, (0.388, 0.0, 0.005), flip=True)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wheelie_bin")

    body_mat = model.material("matte_graphite_polymer", rgba=(0.075, 0.083, 0.083, 1.0))
    satin_mat = model.material("satin_charcoal_edges", rgba=(0.145, 0.155, 0.155, 1.0))
    lid_mat = model.material("satin_lid_graphite", rgba=(0.105, 0.118, 0.120, 1.0))
    rubber_mat = model.material("matte_black_rubber", rgba=(0.010, 0.010, 0.010, 1.0))
    hub_mat = model.material("warm_grey_reinforced_nylon", rgba=(0.420, 0.420, 0.390, 1.0))
    metal_mat = model.material("brushed_steel_hardware", rgba=(0.560, 0.550, 0.510, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_bin_shell_geometry(), "molded_bin_shell"),
        material=body_mat,
        name="bin_shell",
    )

    # Satin molded top interface and seam breaks.
    body.visual(Box((0.055, 0.675, 0.080)), origin=Origin(xyz=(0.365, 0.0, 0.970)), material=satin_mat, name="front_rim")
    body.visual(Box((0.050, 0.655, 0.060)), origin=Origin(xyz=(-0.315, 0.0, 0.950)), material=satin_mat, name="rear_rim")
    body.visual(Box((0.630, 0.042, 0.080)), origin=Origin(xyz=(0.035, 0.336, 0.970)), material=satin_mat, name="side_rim_0")
    body.visual(Box((0.630, 0.042, 0.080)), origin=Origin(xyz=(0.035, -0.336, 0.970)), material=satin_mat, name="side_rim_1")
    body.visual(Box((0.035, 0.505, 0.440)), origin=Origin(xyz=(0.320, 0.0, 0.525)), material=satin_mat, name="front_inset_seam")
    for y in (-0.255, 0.255):
        body.visual(Box((0.030, 0.040, 0.610)), origin=Origin(xyz=(0.315, y, 0.500)), material=satin_mat, name=f"front_corner_rib_{0 if y < 0 else 1}")
    for y in (-0.305, 0.305):
        body.visual(Box((0.430, 0.035, 0.420)), origin=Origin(xyz=(0.025, y, 0.635)), material=satin_mat, name=f"side_satin_break_{0 if y < 0 else 1}")

    # Rear handle, hinge-side structure, axle, and molded feet.
    body.visual(Cylinder(radius=0.022, length=0.640), origin=Origin(xyz=(-0.345, 0.0, 0.885), rpy=(pi / 2, 0.0, 0.0)), material=satin_mat, name="rear_grip_bar")
    for y in (-0.260, 0.260):
        body.visual(Box((0.060, 0.045, 0.145)), origin=Origin(xyz=(-0.315, y, 0.840)), material=satin_mat, name=f"handle_stanchion_{0 if y < 0 else 1}")
    body.visual(Cylinder(radius=0.014, length=0.740), origin=Origin(xyz=(-0.275, 0.0, 0.155), rpy=(pi / 2, 0.0, 0.0)), material=metal_mat, name="rear_axle")
    body.visual(Cylinder(radius=0.055, length=0.006), origin=Origin(xyz=(-0.275, -0.331, 0.155), rpy=(pi / 2, 0.0, 0.0)), material=metal_mat, name="axle_washer_0")
    body.visual(Cylinder(radius=0.055, length=0.006), origin=Origin(xyz=(-0.275, 0.331, 0.155), rpy=(pi / 2, 0.0, 0.0)), material=metal_mat, name="axle_washer_1")
    for y in (-0.280, 0.280):
        body.visual(Box((0.160, 0.080, 0.105)), origin=Origin(xyz=(-0.235, y, 0.165)), material=satin_mat, name=f"axle_boss_{0 if y < 0 else 1}")
    for y in (-0.200, 0.200):
        body.visual(Box((0.130, 0.085, 0.065)), origin=Origin(xyz=(0.240, y, 0.060)), material=satin_mat, name=f"front_foot_{0 if y < 0 else 1}")

    # Full-width alternating hinge knuckles fixed to the body.
    body_hinge_segments = [(-0.318, -0.215), (-0.092, 0.012), (0.132, 0.236)]
    for idx, (y0, y1) in enumerate(body_hinge_segments):
        yc = (y0 + y1) / 2.0
        length = y1 - y0
        body.visual(Cylinder(radius=0.017, length=length), origin=Origin(xyz=(-0.318, yc, 1.005), rpy=(pi / 2, 0.0, 0.0)), material=metal_mat, name=f"body_hinge_knuckle_{idx}")
        body.visual(Box((0.034, length, 0.024)), origin=Origin(xyz=(-0.296, yc, 0.978)), material=satin_mat, name=f"body_hinge_leaf_{idx}")

    lid = model.part("lid")
    lid.visual(mesh_from_geometry(_lid_panel_geometry(), "crowned_lid_panel"), material=lid_mat, name="lid_panel")
    lid.visual(Box((0.070, 0.580, 0.030)), origin=Origin(xyz=(0.750, 0.0, 0.000)), material=satin_mat, name="front_pull_lip")
    lid.visual(Box((0.500, 0.022, 0.020)), origin=Origin(xyz=(0.380, 0.250, 0.039)), material=satin_mat, name="raised_rib_0")
    lid.visual(Box((0.500, 0.022, 0.020)), origin=Origin(xyz=(0.380, -0.250, 0.039)), material=satin_mat, name="raised_rib_1")
    lid.visual(Box((0.030, 0.695, 0.018)), origin=Origin(xyz=(0.044, 0.0, 0.025)), material=satin_mat, name="lid_hinge_leaf")
    lid_hinge_segments = [(-0.205, -0.105), (0.026, 0.120), (0.250, 0.326)]
    for idx, (y0, y1) in enumerate(lid_hinge_segments):
        yc = (y0 + y1) / 2.0
        length = y1 - y0
        lid.visual(Cylinder(radius=0.016, length=length), origin=Origin(xyz=(0.0, yc, 0.0), rpy=(pi / 2, 0.0, 0.0)), material=metal_mat, name=f"lid_hinge_knuckle_{idx}")
        lid.visual(Box((0.028, length, 0.014)), origin=Origin(xyz=(0.020, yc, 0.013)), material=satin_mat, name=f"lid_hinge_tab_{idx}")

    tire = TireGeometry(
        0.145,
        0.075,
        inner_radius=0.102,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.045),
        tread=TireTread(style="block", depth=0.007, count=22, land_ratio=0.56),
        grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.003),),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.008, radius=0.004),
    )
    rim = WheelGeometry(
        0.105,
        0.068,
        rim=WheelRim(inner_radius=0.072, flange_height=0.008, flange_thickness=0.004, bead_seat_depth=0.003),
        hub=WheelHub(
            radius=0.036,
            width=0.050,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.046, hole_diameter=0.005),
        ),
        face=WheelFace(dish_depth=0.007, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.013),
        bore=WheelBore(style="round", diameter=0.040),
    )
    for idx, y in enumerate((-0.365, 0.365)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(mesh_from_geometry(tire, f"wheel_{idx}_tire"), material=rubber_mat, name="tire")
        wheel.visual(mesh_from_geometry(rim, f"wheel_{idx}_rim"), material=hub_mat, name="rim")
        model.articulation(
            f"body_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.275, y, 0.155), rpy=(0.0, 0.0, pi / 2)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=8.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.318, 0.0, 1.005)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.2, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_rim",
        max_gap=0.010,
        max_penetration=0.001,
        name="closed lid sits on a tight raised front seam",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="front_rim",
        min_overlap=0.040,
        name="lid overhang covers the top opening rim",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="front_pull_lip")
    with ctx.pose({hinge: 1.15}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="front_pull_lip")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_pull_lip",
            negative_elem="front_rim",
            min_gap=0.150,
            name="opened lid lifts the pull lip clear of the bin mouth",
        )
    ctx.check(
        "hinge lifts front edge upward",
        closed_aabb is not None and open_aabb is not None and open_aabb[0][2] > closed_aabb[0][2] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    ctx.expect_origin_gap(wheel_1, wheel_0, axis="y", min_gap=0.650, max_gap=0.800, name="wheels straddle the rear axle")
    for idx in (0, 1):
        joint = object_model.get_articulation(f"body_to_wheel_{idx}")
        ctx.check(
            f"wheel_{idx} rolls on the axle axis",
            tuple(round(v, 3) for v in joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

    return ctx.report()


object_model = build_object_model()
