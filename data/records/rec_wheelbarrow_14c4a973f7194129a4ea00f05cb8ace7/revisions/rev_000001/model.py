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
    tube_from_spline_points,
    wire_from_points,
)


def _add_quad(mesh: MeshGeometry, a, b, c, d) -> None:
    ia = mesh.add_vertex(*a)
    ib = mesh.add_vertex(*b)
    ic = mesh.add_vertex(*c)
    id_ = mesh.add_vertex(*d)
    mesh.add_face(ia, ib, ic)
    mesh.add_face(ia, ic, id_)


def _tray_shell_geometry() -> MeshGeometry:
    """Thin, open-top tapered wheelbarrow tray with a real visible cavity."""
    mesh = MeshGeometry()

    # Rectangular frustum loops, ordered around the tray footprint.
    ob = [(-0.38, -0.20, 0.370), (0.38, -0.20, 0.370), (0.38, 0.20, 0.370), (-0.38, 0.20, 0.370)]
    ot = [(-0.56, -0.35, 0.660), (0.56, -0.35, 0.660), (0.56, 0.35, 0.660), (-0.56, 0.35, 0.660)]
    ib = [(-0.32, -0.145, 0.405), (0.32, -0.145, 0.405), (0.32, 0.145, 0.405), (-0.32, 0.145, 0.405)]
    it = [(-0.50, -0.305, 0.635), (0.50, -0.305, 0.635), (0.50, 0.305, 0.635), (-0.50, 0.305, 0.635)]

    # Outside sloped panels.
    for i in range(4):
        _add_quad(mesh, ob[i], ob[(i + 1) % 4], ot[(i + 1) % 4], ot[i])

    # Inside sloped panels and interior floor, leaving the top open.
    for i in range(4):
        _add_quad(mesh, ib[(i + 1) % 4], ib[i], it[i], it[(i + 1) % 4])
    _add_quad(mesh, ib[0], ib[1], ib[2], ib[3])

    # Top rolled-sheet lip and bottom underside.
    for i in range(4):
        _add_quad(mesh, ot[i], ot[(i + 1) % 4], it[(i + 1) % 4], it[i])
    _add_quad(mesh, ob[3], ob[2], ob[1], ob[0])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_wheelbarrow")

    tray_mat = Material("galvanized_green_tray", rgba=(0.36, 0.48, 0.38, 1.0))
    frame_mat = Material("dark_powder_coated_tube", rgba=(0.05, 0.07, 0.06, 1.0))
    rubber_mat = Material("matte_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    steel_mat = Material("brushed_steel", rgba=(0.68, 0.69, 0.66, 1.0))
    rim_mat = Material("painted_wheel_rim", rgba=(0.88, 0.86, 0.78, 1.0))

    body = model.part("body")

    body.visual(
        mesh_from_geometry(_tray_shell_geometry(), "tray_shell"),
        material=tray_mat,
        name="tray_shell",
    )
    body.visual(
        mesh_from_geometry(
            wire_from_points(
                [(-0.53, -0.325, 0.650), (0.53, -0.325, 0.650), (0.53, 0.325, 0.650), (-0.53, 0.325, 0.650)],
                radius=0.020,
                radial_segments=18,
                closed_path=True,
                cap_ends=False,
                corner_mode="fillet",
                corner_radius=0.050,
                corner_segments=10,
            ),
            "tray_rolled_rim",
        ),
        material=tray_mat,
        name="tray_rolled_rim",
    )

    # Two slim continuous side tubes form the handles, under-tray rails, and the
    # converging front fork members around the single wheel.
    for idx, side in enumerate((-1.0, 1.0)):
        body.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (-1.15, side * 0.43, 0.44),
                        (-0.82, side * 0.43, 0.39),
                        (-0.38, side * 0.395, 0.35),
                        (0.25, side * 0.365, 0.34),
                        (0.58, side * 0.155, 0.285),
                        (0.78, side * 0.075, 0.20835),
                    ],
                    radius=0.018,
                    samples_per_segment=18,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"side_rail_{idx}",
            ),
            material=frame_mat,
            name=f"side_rail_{idx}",
        )

    # Cross ties keep the open body stiff without making it a solid block.
    crossbars = {
        "rear_crossbar": [(-0.72, -0.435, 0.395), (-0.72, 0.435, 0.395)],
        "tray_crossbar": [(0.02, -0.385, 0.345), (0.02, 0.385, 0.345)],
        "front_bridge": [(0.52, -0.175, 0.290), (0.52, 0.175, 0.290)],
    }
    for name, points in crossbars.items():
        body.visual(
            mesh_from_geometry(
                wire_from_points(points, radius=0.015, radial_segments=16, cap_ends=True),
                name,
            ),
            material=frame_mat,
            name=name,
        )

    for idx, y in enumerate((-0.16, 0.16)):
        body.visual(
            Box((0.14, 0.055, 0.055)),
            origin=Origin(xyz=(0.02, y, 0.372)),
            material=steel_mat,
            name=f"tray_mount_{idx}",
        )

    # Rear resting legs, one on each side, are tubular struts with small rubber
    # feet that sit on the same ground plane as the tire.
    for idx, side in enumerate((-1.0, 1.0)):
        body.visual(
            mesh_from_geometry(
                wire_from_points(
                    [(-0.48, side * 0.385, 0.345), (-0.63, side * 0.405, 0.175), (-0.72, side * 0.415, 0.030)],
                    radius=0.016,
                    radial_segments=16,
                    cap_ends=True,
                    corner_mode="fillet",
                    corner_radius=0.025,
                    corner_segments=8,
                ),
                f"resting_leg_{idx}",
            ),
            material=frame_mat,
            name=f"resting_leg_{idx}",
        )
        body.visual(
            Box((0.16, 0.055, 0.024)),
            origin=Origin(xyz=(-0.735, side * 0.415, 0.012)),
            material=rubber_mat,
            name=f"foot_{idx}",
        )

    # Rubberized grips sleeve over the rear ends of the tubular handles.
    for idx, side in enumerate((-1.0, 1.0)):
        body.visual(
            Cylinder(radius=0.028, length=0.18),
            origin=Origin(xyz=(-1.15, side * 0.43, 0.44), rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber_mat,
            name=f"grip_{idx}",
        )

    # Visible steel axle through the fork.  Its diameter is smaller than the
    # modeled wheel bore, so the wheel can spin without a solid collision.
    body.visual(
        Cylinder(radius=0.017, length=0.245),
        origin=Origin(xyz=(0.78, 0.0, 0.20835), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="front_axle",
    )

    wheel = model.part("front_wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.200,
                0.090,
                inner_radius=0.145,
                carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.06),
                tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.55),
                grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
                sidewall=TireSidewall(style="square", bulge=0.025),
                shoulder=TireShoulder(width=0.010, radius=0.004),
            ),
            "front_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=rubber_mat,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.148,
                0.072,
                rim=WheelRim(inner_radius=0.096, flange_height=0.010, flange_thickness=0.004, bead_seat_depth=0.004),
                hub=WheelHub(
                    radius=0.044,
                    width=0.062,
                    cap_style="flat",
                    bolt_pattern=BoltPattern(count=4, circle_diameter=0.052, hole_diameter=0.006),
                ),
                face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
                spokes=WheelSpokes(style="straight", count=8, thickness=0.006, window_radius=0.012),
                bore=WheelBore(style="round", diameter=0.034),
            ),
            "front_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=rim_mat,
        name="rim",
    )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(0.78, 0.0, 0.20835)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=30.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("front_wheel")
    joint = object_model.get_articulation("wheel_axle")

    ctx.allow_overlap(
        body,
        wheel,
        elem_a="front_axle",
        elem_b="rim",
        reason="The steel axle is intentionally captured through the wheel hub/bore so the wheel is physically supported while rotating.",
    )
    ctx.expect_overlap(
        body,
        wheel,
        axes="y",
        elem_a="front_axle",
        elem_b="rim",
        min_overlap=0.060,
        name="axle passes through the wheel hub width",
    )
    ctx.expect_within(
        body,
        wheel,
        axes="xz",
        inner_elem="front_axle",
        outer_elem="rim",
        margin=0.002,
        name="axle is centered through the wheel bore",
    )

    ctx.check(
        "front wheel uses a continuous axle joint",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    tire_aabb = ctx.part_element_world_aabb(wheel, elem="tire")
    ctx.check(
        "front tire rests on the ground plane",
        tire_aabb is not None and abs(tire_aabb[0][2]) < 0.006,
        details=f"tire_aabb={tire_aabb}",
    )

    for foot_name in ("foot_0", "foot_1"):
        foot_aabb = ctx.part_element_world_aabb(body, elem=foot_name)
        ctx.check(
            f"{foot_name} rests on the ground plane",
            foot_aabb is not None and abs(foot_aabb[0][2]) < 0.004,
            details=f"{foot_name} aabb={foot_aabb}",
        )

    left_grip = ctx.part_element_world_aabb(body, elem="grip_0")
    right_grip = ctx.part_element_world_aabb(body, elem="grip_1")
    tray = ctx.part_element_world_aabb(body, elem="tray_shell")
    ctx.check(
        "tray sits between the two handle tubes",
        left_grip is not None
        and right_grip is not None
        and tray is not None
        and left_grip[1][1] < tray[0][1]
        and right_grip[0][1] > tray[1][1],
        details=f"left_grip={left_grip}, right_grip={right_grip}, tray={tray}",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({joint: 1.75}):
        turned_pos = ctx.part_world_position(wheel)
    ctx.check(
        "spinning the wheel keeps it captured on the axle",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_pos, turned_pos)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
