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


def _add_box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_cylinder(
    part,
    name: str,
    *,
    radius: float,
    length: float,
    xyz,
    material: Material,
    rpy=(0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _quad_strip(mesh: MeshGeometry, loop_a: list[int], loop_b: list[int], *, flip: bool = False) -> None:
    count = len(loop_a)
    for i in range(count):
        j = (i + 1) % count
        a0, a1 = loop_a[i], loop_a[j]
        b0, b1 = loop_b[i], loop_b[j]
        if flip:
            mesh.add_face(a0, b1, a1)
            mesh.add_face(a0, b0, b1)
        else:
            mesh.add_face(a0, a1, b1)
            mesh.add_face(a0, b1, b0)


def _loop(mesh: MeshGeometry, profile: list[tuple[float, float]], z: float) -> list[int]:
    return [mesh.add_vertex(x, y, z) for x, y in profile]


def _bin_shell_mesh() -> MeshGeometry:
    """Open-topped tapered HDPE bin body with a visible thick wall and bottom pan."""
    mesh = MeshGeometry()
    outer_lower = _loop(mesh, rounded_rect_profile(0.54, 0.50, 0.060, corner_segments=8), 0.115)
    outer_upper = _loop(mesh, rounded_rect_profile(0.76, 0.66, 0.085, corner_segments=8), 1.000)
    inner_upper = _loop(mesh, rounded_rect_profile(0.675, 0.575, 0.060, corner_segments=8), 0.955)
    inner_lower = _loop(mesh, rounded_rect_profile(0.435, 0.395, 0.040, corner_segments=8), 0.175)

    _quad_strip(mesh, outer_lower, outer_upper)
    _quad_strip(mesh, outer_upper, inner_upper)
    _quad_strip(mesh, inner_upper, inner_lower, flip=True)
    _quad_strip(mesh, inner_lower, outer_lower, flip=True)
    return mesh


def _wheel_meshes():
    tire = TireGeometry(
        0.160,
        0.090,
        inner_radius=0.108,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
        tread=TireTread(style="block", depth=0.009, count=18, land_ratio=0.55),
        grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )
    rim = WheelGeometry(
        0.110,
        0.082,
        rim=WheelRim(inner_radius=0.070, flange_height=0.008, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.036,
            width=0.076,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.052, hole_diameter=0.005),
        ),
        face=WheelFace(dish_depth=0.008, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="split_y", count=6, thickness=0.0045, window_radius=0.012),
        bore=WheelBore(style="round", diameter=0.026),
    )
    return mesh_from_geometry(tire, "heavy_duty_tire"), mesh_from_geometry(rim, "steel_wheel_rim")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_wheelie_bin")

    hdpe_green = model.material("impact_hdpe_green", rgba=(0.05, 0.24, 0.16, 1.0))
    hdpe_dark = model.material("dark_hdpe", rgba=(0.02, 0.035, 0.032, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.78, 0.02, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.64, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    stop_red = model.material("lockout_red", rgba=(0.78, 0.04, 0.025, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_bin_shell_mesh(), "tapered_hollow_bin_shell"),
        material=hdpe_green,
        name="hollow_shell",
    )
    # Thick top rim and stress bands, visibly continuous with the molded shell.
    _add_box(body, "front_rim", (0.050, 0.690, 0.052), (0.395, 0.0, 1.005), hdpe_green)
    _add_box(body, "rear_rim", (0.050, 0.690, 0.040), (-0.395, 0.0, 0.980), hdpe_green)
    body.visual(
        Box((0.760, 0.050, 0.052)),
        origin=Origin(xyz=(0.0, 0.350, 1.005)),
        material=hdpe_green,
        name="side_rim_0",
    )
    _add_box(body, "side_rim_1", (0.760, 0.050, 0.052), (0.0, -0.350, 1.005), hdpe_green)
    _add_box(body, "front_wear_band", (0.055, 0.570, 0.060), (0.355, 0.0, 0.760), hdpe_dark)
    _add_box(body, "rear_wear_band", (0.055, 0.540, 0.060), (-0.345, 0.0, 0.710), hdpe_dark)
    _add_box(body, "side_rib_0", (0.045, 0.050, 0.650), (0.020, 0.315, 0.590), hdpe_dark)
    _add_box(body, "side_rib_1", (0.045, 0.050, 0.650), (0.020, -0.315, 0.590), hdpe_dark)
    _add_box(body, "front_lift_pad", (0.035, 0.250, 0.105), (0.392, 0.0, 0.930), safety_yellow)

    # Front skid feet are tied into the bottom pan instead of floating below it.
    for y, suffix in ((0.210, "0"), (-0.210, "1")):
        _add_box(body, f"front_foot_{suffix}", (0.185, 0.090, 0.040), (0.220, y, 0.025), rubber)
        _add_box(body, f"front_foot_riser_{suffix}", (0.105, 0.070, 0.090), (0.235, y, 0.082), hdpe_dark)

    # Rear axle, axle saddles, and wheel guard load paths.
    body.visual(
        Cylinder(radius=0.024, length=0.900),
        origin=Origin(xyz=(-0.430, 0.0, 0.170), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="rear_axle",
    )
    for y, suffix in ((0.245, "0"), (-0.245, "1")):
        _add_box(body, f"axle_saddle_{suffix}", (0.185, 0.070, 0.065), (-0.355, y, 0.170), galvanized)
        _add_box(body, f"axle_gusset_{suffix}", (0.185, 0.026, 0.105), (-0.355, y, 0.235), galvanized, rpy=(0.0, -0.48, 0.0))
    for y, sign, suffix in ((0.435, 1.0, "0"), (-0.435, -1.0, "1")):
        _add_box(body, f"wheel_guard_plate_{suffix}", (0.255, 0.030, 0.225), (-0.430, y + sign * 0.070, 0.235), safety_yellow)
        _add_box(body, f"wheel_guard_top_{suffix}", (0.245, 0.240, 0.028), (-0.430, y - sign * 0.025, 0.355), safety_yellow)
        _add_box(body, f"guard_bridge_{suffix}", (0.080, 0.130, 0.220), (-0.335, y - sign * 0.135, 0.250), galvanized)

    # Hinge load path: continuous rear strap, alternating knuckles, stop towers, and bolted plates.
    _add_box(body, "rear_hinge_strap", (0.050, 0.690, 0.060), (-0.407, 0.0, 0.970), galvanized)
    for y, length, suffix in ((-0.285, 0.140, "0"), (0.0, 0.150, "1"), (0.285, 0.140, "2")):
        _add_cylinder(
            body,
            f"hinge_knuckle_{suffix}",
            radius=0.026,
            length=length,
            xyz=(-0.410, y, 1.035),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
            material=galvanized,
        )
        _add_box(body, f"hinge_web_{suffix}", (0.022, length * 0.72, 0.060), (-0.410, y, 1.010), galvanized)
    for y, suffix in ((0.270, "0"), (-0.270, "1")):
        _add_box(body, f"overtravel_stop_{suffix}", (0.060, 0.080, 0.190), (-0.460, y, 1.080), safety_yellow)
        _add_box(body, f"stop_bumper_{suffix}", (0.032, 0.070, 0.055), (-0.428, y, 1.162), rubber)
        _add_box(body, f"rear_brace_{suffix}", (0.215, 0.034, 0.038), (-0.360, y, 0.965), galvanized, rpy=(0.0, -0.65, 0.0))
    for y, z, suffix in ((-0.250, 1.005, "0"), (0.0, 1.005, "1"), (0.250, 1.005, "2")):
        _add_cylinder(
            body,
            f"hinge_bolt_{suffix}",
            radius=0.014,
            length=0.010,
            xyz=(-0.434, y, 0.985),
            rpy=(0.0, math.pi / 2.0, 0.0),
            material=galvanized,
        )

    # Front lockout clevis mounted to the front rim.
    body.visual(
        Box((0.070, 0.024, 0.080)),
        origin=Origin(xyz=(0.440, 0.047, 0.955)),
        material=safety_yellow,
        name="lock_clevis_0",
    )
    _add_box(body, "lock_clevis_1", (0.070, 0.024, 0.080), (0.440, -0.047, 0.955), safety_yellow)
    _add_box(body, "lock_keeper", (0.028, 0.115, 0.040), (0.395, 0.0, 0.960), galvanized)

    tire_mesh, rim_mesh = _wheel_meshes()
    wheel_parts = []
    for y, suffix in ((0.430, "0"), (-0.430, "1")):
        wheel = model.part(f"wheel_{suffix}")
        wheel.visual(tire_mesh, origin=Origin(), material=rubber, name="tire")
        wheel.visual(rim_mesh, origin=Origin(), material=galvanized, name="rim")
        wheel_parts.append((wheel, y, suffix))
        model.articulation(
            f"body_to_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.430, y, 0.170), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=60.0, velocity=18.0),
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.810, 0.750, 0.044)),
        origin=Origin(xyz=(0.425, 0.0, 0.044)),
        material=hdpe_green,
        name="lid_panel",
    )
    _add_box(lid, "lid_front_lip", (0.050, 0.745, 0.080), (0.855, 0.0, 0.015), hdpe_dark)
    _add_box(lid, "lid_side_lip_0", (0.730, 0.050, 0.056), (0.455, 0.400, 0.018), hdpe_dark)
    _add_box(lid, "lid_side_lip_1", (0.730, 0.050, 0.056), (0.455, -0.400, 0.018), hdpe_dark)
    _add_box(lid, "lid_rear_leaf", (0.080, 0.120, 0.050), (0.040, -0.145, 0.035), galvanized)
    _add_box(lid, "lid_rear_leaf_1", (0.080, 0.120, 0.050), (0.040, 0.145, 0.035), galvanized)
    for y, length, suffix in ((-0.145, 0.120, "0"), (0.145, 0.120, "1")):
        _add_cylinder(
            lid,
            f"lid_knuckle_{suffix}",
            radius=0.022,
            length=length,
            xyz=(0.0, y, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
            material=galvanized,
        )
    for y, suffix in ((-0.220, "0"), (0.0, "1"), (0.220, "2")):
        _add_box(lid, f"lid_cross_rib_{suffix}", (0.585, 0.030, 0.040), (0.430, y, 0.078), hdpe_dark)
        _add_cylinder(
            lid,
            f"lid_bolt_{suffix}",
            radius=0.013,
            length=0.008,
            xyz=(0.180, y, 0.097),
            material=galvanized,
        )
    _add_box(lid, "lock_strike", (0.035, 0.150, 0.030), (0.790, 0.0, 0.054), galvanized)
    _add_box(lid, "warning_strip", (0.360, 0.035, 0.008), (0.535, 0.0, 0.071), safety_yellow)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.410, 0.0, 1.035)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.4, lower=0.0, upper=math.radians(112.0)),
    )

    lock_tab = model.part("lock_tab")
    lock_tab.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="pivot_barrel",
    )
    _add_box(lock_tab, "lock_plate", (0.035, 0.056, 0.145), (0.018, 0.0, -0.088), stop_red)
    _add_box(lock_tab, "pull_tab", (0.046, 0.075, 0.030), (0.020, 0.0, -0.168), safety_yellow)
    _add_cylinder(
        lock_tab,
        "lock_bolt",
        radius=0.011,
        length=0.064,
        xyz=(0.040, 0.0, -0.088),
        rpy=(-math.pi / 2.0, 0.0, 0.0),
        material=galvanized,
    )
    model.articulation(
        "body_to_lock_tab",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_tab,
        origin=Origin(xyz=(0.455, 0.0, 0.955)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=math.radians(-20.0), upper=math.radians(75.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lock_tab = object_model.get_part("lock_tab")
    lid_hinge = object_model.get_articulation("body_to_lid")
    lock_joint = object_model.get_articulation("body_to_lock_tab")

    for suffix in ("0", "1"):
        wheel = object_model.get_part(f"wheel_{suffix}")
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The steel rear axle is intentionally captured inside the wheel hub bore so the rolling wheel is visibly supported.",
        )
        ctx.expect_overlap(
            wheel,
            body,
            axes="xz",
            elem_a="rim",
            elem_b="rear_axle",
            min_overlap=0.035,
            name=f"wheel_{suffix} hub surrounds axle in radial projection",
        )
        ctx.expect_overlap(
            wheel,
            body,
            axes="y",
            elem_a="rim",
            elem_b="rear_axle",
            min_overlap=0.035,
            name=f"wheel_{suffix} hub remains on axle stub",
        )

    with ctx.pose({lid_hinge: 0.0, lock_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="side_rim_0",
            min_gap=0.010,
            max_gap=0.080,
            name="closed lid sits just above reinforced rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="side_rim_0",
            min_overlap=0.030,
            name="closed lid covers the side rim",
        )
        ctx.expect_contact(
            lock_tab,
            body,
            elem_a="pivot_barrel",
            elem_b="lock_clevis_0",
            contact_tol=0.002,
            name="lock tab pivot is carried by the clevis",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: math.radians(95.0)}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge raises the free edge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.35,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for suffix in ("0", "1"):
        wheel_joint = object_model.get_articulation(f"body_to_wheel_{suffix}")
        ctx.check(
            f"wheel_{suffix} has continuous rolling joint",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in wheel_joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
        )

    return ctx.report()


object_model = build_object_model()
