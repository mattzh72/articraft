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
    wire_from_points,
)


def _triangular_plate_mesh(
    points_xz: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    *,
    thickness: float,
    side: float,
) -> MeshGeometry:
    """A simple extruded triangular side plate in the local XZ plane."""
    geom = MeshGeometry()
    y0 = 0.0
    y1 = side * thickness
    for x, z in points_xz:
        geom.add_vertex(x, y0, z)
    for x, z in points_xz:
        geom.add_vertex(x, y1, z)

    # Inner and outer triangular faces.
    geom.add_face(0, 1, 2)
    geom.add_face(3, 5, 4)
    # Side walls.
    for i in range(3):
        j = (i + 1) % 3
        geom.add_face(i, j, 3 + j)
        geom.add_face(i, 3 + j, 3 + i)
    return geom


def _wheel_center_positions(radius: float) -> tuple[tuple[float, float], ...]:
    # One wheel high, two low: the recognisable stair-climber "tri-star" layout.
    return tuple(
        (radius * math.cos(a), radius * math.sin(a))
        for a in (math.radians(90.0), math.radians(210.0), math.radians(330.0))
    )


def _add_cylinder_between(
    part,
    *,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
) -> None:
    """Add a cylinder between two points for straight tubes/struts."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return
    # Cylinder local +Z must rotate onto the direction vector.
    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stair_climber_hand_truck")

    painted_steel = model.material("painted_steel", rgba=(0.08, 0.11, 0.13, 1.0))
    toe_steel = model.material("toe_plate_steel", rgba=(0.34, 0.36, 0.37, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.55, 0.57, 0.58, 1.0))
    warning_red = model.material("red_cluster_plate", rgba=(0.78, 0.05, 0.03, 1.0))

    frame = model.part("frame")

    # One continuous bent tube for the main upright handle frame.
    main_frame_mesh = mesh_from_geometry(
        wire_from_points(
            (
                (0.0, -0.22, 0.075),
                (0.0, -0.22, 1.16),
                (0.0, -0.17, 1.25),
                (0.0, 0.17, 1.25),
                (0.0, 0.22, 1.16),
                (0.0, 0.22, 0.075),
            ),
            radius=0.018,
            radial_segments=20,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.055,
            corner_segments=10,
        ),
        "main_upright_tube",
    )
    frame.visual(main_frame_mesh, material=painted_steel, name="upright_tube")

    # Toe plate and welded back lip.
    frame.visual(
        Box((0.38, 0.56, 0.025)),
        origin=Origin(xyz=(0.16, 0.0, 0.0125)),
        material=toe_steel,
        name="toe_plate",
    )
    frame.visual(
        Box((0.035, 0.56, 0.080)),
        origin=Origin(xyz=(-0.005, 0.0, 0.065)),
        material=toe_steel,
        name="toe_lip",
    )

    # Crossbars and the low structural members that carry the stair-climber hubs.
    for z, radius, name in (
        (0.28, 0.014, "lower_crossbar"),
        (0.73, 0.013, "middle_crossbar"),
        (1.06, 0.013, "upper_crossbar"),
    ):
        frame.visual(
            Cylinder(radius=radius, length=0.49),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=painted_steel,
            name=name,
        )

    # Slightly proud rubber sleeves at the upper grip.
    for side, name in ((-1.0, "grip_0"), (1.0, "grip_1")):
        frame.visual(
            Cylinder(radius=0.026, length=0.135),
            origin=Origin(
                xyz=(0.0, side * 0.135, 1.25),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_rubber,
            name=name,
        )

    # Side stubs and bosses put real geometry at the rotating tri-star hubs.
    hub_x = -0.080
    hub_z = 0.160
    frame_side_y = 0.22
    hub_y = 0.285
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        _add_cylinder_between(
            frame,
            name=f"hub_strut_{suffix}",
            start=(0.0, side * frame_side_y, hub_z),
            end=(hub_x, side * frame_side_y, hub_z),
            radius=0.014,
            material=painted_steel,
        )
        frame.visual(
            Cylinder(radius=0.034, length=hub_y - frame_side_y),
            origin=Origin(
                xyz=(hub_x, side * (frame_side_y + hub_y) * 0.5, hub_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=hub_gray,
            name=f"frame_hub_{suffix}",
        )
        _add_cylinder_between(
            frame,
            name=f"toe_brace_{suffix}",
            start=(-0.005, side * 0.20, 0.080),
            end=(0.24, side * 0.20, 0.025),
            radius=0.010,
            material=painted_steel,
        )

    cluster_radius = 0.130
    wheel_radius = 0.070
    wheel_width = 0.052
    plate_thickness = 0.034
    wheel_offset = 0.075
    centers_xz = _wheel_center_positions(cluster_radius)
    # Slightly larger triangle than the axle-center triangle.
    plate_points = tuple((x * 1.05, z * 1.05) for x, z in centers_xz)

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            wheel_radius,
            wheel_width,
            inner_radius=0.049,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.005, count=18, land_ratio=0.55),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "small_stair_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.051,
            0.043,
            rim=WheelRim(
                inner_radius=0.032,
                flange_height=0.004,
                flange_thickness=0.003,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(radius=0.018, width=0.032, cap_style="domed"),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.024),
        ),
        "small_stair_rim",
    )

    for cluster_index, side in enumerate((-1.0, 1.0)):
        cluster = model.part(f"cluster_{cluster_index}")
        cluster.visual(
            mesh_from_geometry(
                _triangular_plate_mesh(
                    plate_points, thickness=plate_thickness, side=side
                ),
                f"cluster_plate_{cluster_index}",
            ),
            material=warning_red,
            name="tri_plate",
        )
        # Large central bearing on the frame hub.
        cluster.visual(
            Cylinder(radius=0.044, length=0.052),
            origin=Origin(
                xyz=(0.0, side * 0.026, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=hub_gray,
            name="main_hub",
        )
        # Three axle pins and round pads mark the wheel stations.
        for wheel_index, (wx, wz) in enumerate(centers_xz):
            _add_cylinder_between(
                cluster,
                name=f"spoke_{wheel_index}",
                start=(0.0, side * 0.018, 0.0),
                end=(wx, side * 0.018, wz),
                radius=0.011,
                material=warning_red,
            )
            cluster.visual(
                Cylinder(radius=0.026, length=0.030),
                origin=Origin(
                    xyz=(wx, side * 0.015, wz),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=hub_gray,
                name=f"axle_pad_{wheel_index}",
            )
            cluster.visual(
                Cylinder(radius=0.012, length=0.115),
                origin=Origin(
                    xyz=(wx, side * 0.0575, wz),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=hub_gray,
                name=f"axle_pin_{wheel_index}",
            )

            wheel = model.part(f"wheel_{cluster_index}_{wheel_index}")
            wheel.visual(tire_mesh, material=black_rubber, name="tire")
            wheel.visual(rim_mesh, material=hub_gray, name="rim")

            model.articulation(
                f"cluster_{cluster_index}_to_wheel_{wheel_index}",
                ArticulationType.CONTINUOUS,
                parent=cluster,
                child=wheel,
                origin=Origin(
                    xyz=(wx, side * wheel_offset, wz),
                    rpy=(0.0, 0.0, side * math.pi / 2.0),
                ),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=3.0, velocity=20.0),
            )

        model.articulation(
            f"frame_to_cluster_{cluster_index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=cluster,
            origin=Origin(xyz=(hub_x, side * hub_y, hub_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cluster_joints = [
        object_model.get_articulation("frame_to_cluster_0"),
        object_model.get_articulation("frame_to_cluster_1"),
    ]
    wheel_joints = [
        object_model.get_articulation(f"cluster_{ci}_to_wheel_{wi}")
        for ci in range(2)
        for wi in range(3)
    ]
    ctx.check(
        "paired three wheel clusters",
        len(cluster_joints) == 2 and len(wheel_joints) == 6,
        details="expected two rotating tri-star hubs and six independent wheel spin joints",
    )

    frame = object_model.get_part("frame")
    for ci in range(2):
        cluster = object_model.get_part(f"cluster_{ci}")
        ctx.expect_contact(
            frame,
            cluster,
            elem_a=f"frame_hub_{ci}",
            elem_b="main_hub",
            contact_tol=0.001,
            name=f"cluster_{ci} seated on frame hub",
        )
        for wi in range(3):
            wheel = object_model.get_part(f"wheel_{ci}_{wi}")
            ctx.allow_overlap(
                cluster,
                wheel,
                elem_a=f"axle_pin_{wi}",
                elem_b="rim",
                reason=(
                    "The wheel rim is captured on a simplified axle pin; "
                    "the local overlap represents the bearing/axle fit that lets the wheel spin."
                ),
            )
            ctx.expect_overlap(
                wheel,
                cluster,
                axes="xz",
                elem_a="rim",
                elem_b=f"axle_pin_{wi}",
                min_overlap=0.010,
                name=f"wheel_{ci}_{wi} centered on axle",
            )

    # A decisive pose check: the tri-star cluster truly orbits about the large hub.
    cluster_0 = object_model.get_part("cluster_0")
    top_wheel = object_model.get_part("wheel_0_0")
    rest = ctx.part_world_position(top_wheel)
    with ctx.pose({cluster_joints[0]: math.radians(60.0)}):
        moved = ctx.part_world_position(top_wheel)
    ctx.check(
        "cluster wheel orbits hub",
        rest is not None
        and moved is not None
        and abs(rest[0] - moved[0]) > 0.04
        and abs(rest[2] - moved[2]) > 0.04,
        details=f"rest={rest}, moved={moved}, cluster={ctx.part_world_position(cluster_0)}",
    )

    return ctx.report()


object_model = build_object_model()
