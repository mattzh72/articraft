from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
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
    tube_from_spline_points,
)


def _cylinder_origin_between(
    p0: tuple[float, float, float], p1: tuple[float, float, float]
) -> tuple[Origin, float]:
    """Return an origin and length for a Cylinder whose local +Z spans p0..p1."""

    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be separated")

    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    return (
        Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _tube_between(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
    name: str,
) -> None:
    origin, length = _cylinder_origin_between(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _wheel_meshes() -> tuple[object, object]:
    wheel = WheelGeometry(
        0.122,
        0.070,
        rim=WheelRim(
            inner_radius=0.084,
            flange_height=0.012,
            flange_thickness=0.005,
            bead_seat_depth=0.004,
        ),
        hub=WheelHub(
            radius=0.038,
            width=0.050,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.050, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.003),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.006, window_radius=0.020),
        bore=WheelBore(style="round", diameter=0.018),
    )
    tire = TireGeometry(
        0.165,
        0.078,
        inner_radius=0.118,
        carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.045),
        tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.55),
        grooves=(
            TireGroove(center_offset=-0.018, width=0.006, depth=0.004),
            TireGroove(center_offset=0.018, width=0.006, depth=0.004),
        ),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )
    return mesh_from_geometry(wheel, "dolly_wheel_rim"), mesh_from_geometry(tire, "dolly_utility_tire")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_hand_truck_dolly")

    red = model.material("red_powder_coat", rgba=(0.86, 0.07, 0.03, 1.0))
    dark_red = model.material("welded_red_shadow", rgba=(0.55, 0.03, 0.02, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.55, 0.52, 1.0))
    dark_steel = model.material("dark_axle_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    rim_material = model.material("silver_rim", rgba=(0.78, 0.78, 0.72, 1.0))

    frame = model.part("frame")

    # Main welded tubular frame: two continuous bent side rails plus cross braces.
    for y, suffix in [(-0.22, "0"), (0.22, "1")]:
        side_rail = tube_from_spline_points(
            [
                (0.000, y, 0.105),
                (0.012, y, 0.310),
                (0.050, y, 0.720),
                (0.130, y, 1.155),
                (0.086, y, 1.360),
            ],
            radius=0.018,
            samples_per_segment=14,
            radial_segments=20,
            cap_ends=True,
        )
        frame.visual(
            mesh_from_geometry(side_rail, f"side_rail_{suffix}_mesh"),
            material=red,
            name=f"side_rail_{suffix}",
        )

    handle_loop = tube_from_spline_points(
        [
            (0.105, -0.220, 1.250),
            (0.073, -0.255, 1.360),
            (0.055, -0.125, 1.440),
            (0.055, 0.000, 1.455),
            (0.055, 0.125, 1.440),
            (0.073, 0.255, 1.360),
            (0.105, 0.220, 1.250),
        ],
        radius=0.018,
        samples_per_segment=14,
        radial_segments=20,
        cap_ends=True,
    )
    frame.visual(mesh_from_geometry(handle_loop, "handle_loop_mesh"), material=red, name="handle_loop")

    # Rubber hand grip wrapped around the central top of the handle.
    frame.visual(
        Cylinder(radius=0.027, length=0.255),
        origin=Origin(xyz=(0.055, 0.0, 1.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="rubber_grip",
    )

    crossbars = [
        ("lower_crossbar", (0.010, -0.255, 0.290), (0.010, 0.255, 0.290), 0.015),
        ("middle_crossbar", (0.040, -0.252, 0.600), (0.040, 0.252, 0.600), 0.014),
        ("load_crossbar", (0.080, -0.252, 0.900), (0.080, 0.252, 0.900), 0.014),
        ("shoulder_crossbar", (0.116, -0.248, 1.130), (0.116, 0.248, 1.130), 0.014),
    ]
    for name, p0, p1, radius in crossbars:
        _tube_between(frame, p0, p1, radius=radius, material=red, name=name)

    _tube_between(frame, (0.028, 0.0, 0.235), (0.088, 0.0, 1.060), radius=0.011, material=red, name="center_spine")
    _tube_between(frame, (-0.190, -0.195, 0.040), (0.012, -0.220, 0.305), radius=0.011, material=red, name="toe_brace_0")
    _tube_between(frame, (-0.190, 0.195, 0.040), (0.012, 0.220, 0.305), radius=0.011, material=red, name="toe_brace_1")
    _tube_between(frame, (0.095, -0.205, 0.160), (0.017, -0.220, 0.405), radius=0.012, material=red, name="axle_strut_0")
    _tube_between(frame, (0.095, 0.205, 0.160), (0.017, 0.220, 0.405), radius=0.012, material=red, name="axle_strut_1")

    # Rugged steel toe plate and rear heel plate, welded into the lower frame.
    frame.visual(
        Box(size=(0.380, 0.540, 0.018)),
        origin=Origin(xyz=(-0.140, 0.0, 0.025)),
        material=steel,
        name="toe_plate",
    )
    frame.visual(
        Box(size=(0.028, 0.540, 0.120)),
        origin=Origin(xyz=(0.045, 0.0, 0.076)),
        material=steel,
        name="heel_plate",
    )
    frame.visual(
        Box(size=(0.020, 0.420, 0.060)),
        origin=Origin(xyz=(0.028, 0.0, 0.600)),
        material=dark_red,
        name="load_back_strap",
    )
    frame.visual(
        Box(size=(0.018, 0.420, 0.050)),
        origin=Origin(xyz=(0.067, 0.0, 0.900)),
        material=dark_red,
        name="upper_back_strap",
    )

    # Axle and dropout plates carry the large wheels.
    _tube_between(frame, (0.095, -0.390, 0.165), (0.095, 0.390, 0.165), radius=0.014, material=dark_steel, name="axle")
    for y, suffix in [(-0.306, "0"), (0.306, "1")]:
        frame.visual(
            Box(size=(0.105, 0.014, 0.112)),
            origin=Origin(xyz=(0.095, y, 0.165)),
            material=dark_steel,
            name=f"dropout_plate_{suffix}",
        )
        frame.visual(
            Cylinder(radius=0.044, length=0.010),
            origin=Origin(xyz=(0.095, y, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"inner_washer_{suffix}",
        )

    # Raised fasteners on the toe plate and load straps.
    for i, x in enumerate([-0.245, -0.075]):
        for j, y in enumerate([-0.185, 0.185]):
            frame.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(x, y, 0.037), rpy=(0.0, 0.0, 0.0)),
                material=dark_steel,
                name=f"toe_bolt_{i}_{j}",
            )
    for i, (z, x) in enumerate([(0.600, 0.018), (0.900, 0.058)]):
        for y in (-0.145, 0.145):
            frame.visual(
                Cylinder(radius=0.010, length=0.008),
                origin=Origin(xyz=(x - 0.004, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_steel,
                name=f"strap_bolt_{i}_{0 if y < 0 else 1}",
            )

    # Visible hinge pin for the fold-out loading extension.
    frame.visual(
        Cylinder(radius=0.012, length=0.560),
        origin=Origin(xyz=(-0.330, 0.0, 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="toe_hinge_pin",
    )
    for y, suffix in [(-0.267, "0"), (0.267, "1")]:
        frame.visual(
            Box(size=(0.026, 0.025, 0.034)),
            origin=Origin(xyz=(-0.330, y, 0.039)),
            material=dark_steel,
            name=f"hinge_lug_{suffix}",
        )

    toe_extension = model.part("toe_extension")
    toe_extension.visual(
        Box(size=(0.260, 0.500, 0.014)),
        origin=Origin(xyz=(-0.130, 0.0, -0.024)),
        material=steel,
        name="extension_plate",
    )
    toe_extension.visual(
        Box(size=(0.015, 0.500, 0.020)),
        origin=Origin(xyz=(-0.260, 0.0, -0.015)),
        material=dark_steel,
        name="front_lip",
    )
    for i, y in enumerate([-0.185, 0.000, 0.185]):
        toe_extension.visual(
            Cylinder(radius=0.017, length=0.105),
            origin=Origin(xyz=(0.000, y, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"hinge_sleeve_{i}",
        )

    wheel_mesh, tire_mesh = _wheel_meshes()
    for suffix, y in [("0", -0.370), ("1", 0.370)]:
        wheel = model.part(f"wheel_{suffix}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_material,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(0.0, 0.040 if y > 0 else -0.040, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="outer_hub_cap",
        )
        wheel.visual(
            Cylinder(radius=0.021, length=0.090),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="bearing_sleeve",
        )
        model.articulation(
            f"frame_to_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.095, y, 0.165)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=8.0),
        )

    model.articulation(
        "frame_to_toe_extension",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=toe_extension,
        origin=Origin(xyz=(-0.330, 0.0, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=0.0, upper=math.radians(105.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    extension = object_model.get_part("toe_extension")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    toe_hinge = object_model.get_articulation("frame_to_toe_extension")

    for sleeve_name in ("hinge_sleeve_0", "hinge_sleeve_1", "hinge_sleeve_2"):
        ctx.allow_overlap(
            frame,
            extension,
            elem_a="toe_hinge_pin",
            elem_b=sleeve_name,
            reason="The folding toe extension sleeve intentionally surrounds the fixed hinge pin.",
        )
        ctx.expect_overlap(
            frame,
            extension,
            axes="y",
            elem_a="toe_hinge_pin",
            elem_b=sleeve_name,
            min_overlap=0.08,
            name=f"{sleeve_name} captures the hinge pin",
        )

    ctx.expect_overlap(
        frame,
        extension,
        axes="y",
        elem_a="toe_plate",
        elem_b="extension_plate",
        min_overlap=0.45,
        name="folding extension matches toe plate width",
    )
    ctx.expect_gap(
        frame,
        extension,
        axis="x",
        positive_elem="toe_plate",
        negative_elem="extension_plate",
        max_gap=0.004,
        max_penetration=0.002,
        name="deployed toe extension meets the fixed toe plate edge",
    )
    toe_aabb = ctx.part_element_world_aabb(frame, elem="toe_plate")
    extension_aabb = ctx.part_element_world_aabb(extension, elem="extension_plate")
    ctx.check(
        "deployed toe plates share a loading plane",
        toe_aabb is not None
        and extension_aabb is not None
        and abs(toe_aabb[1][2] - extension_aabb[1][2]) < 0.006,
        details=f"toe={toe_aabb}, extension={extension_aabb}",
    )

    for wheel_name, wheel in [("wheel_0", wheel_0), ("wheel_1", wheel_1)]:
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="axle",
            elem_b="bearing_sleeve",
            reason="The wheel bearing sleeve is intentionally captured around the fixed steel axle.",
        )
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="axle",
            elem_b="rim",
            reason="The steel axle intentionally passes through the wheel hub and bore region.",
        )
        ctx.check(
            f"{wheel_name} has a rolling joint",
            object_model.get_articulation(f"frame_to_{wheel_name}").articulation_type
            == ArticulationType.CONTINUOUS,
            details="Utility wheels should spin freely on the axle.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="xz",
            elem_a="axle",
            elem_b="bearing_sleeve",
            min_overlap=0.025,
            name=f"{wheel_name} bearing is centered on the axle line",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="axle",
            elem_b="rim",
            min_overlap=0.04,
            name=f"{wheel_name} hub stays captured on the axle",
        )

    flat_aabb = ctx.part_world_aabb(extension)
    with ctx.pose({toe_hinge: math.radians(95.0)}):
        folded_aabb = ctx.part_world_aabb(extension)
    ctx.check(
        "toe extension folds upward",
        flat_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] > flat_aabb[1][2] + 0.17
        and folded_aabb[1][0] > -0.36,
        details=f"flat={flat_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
